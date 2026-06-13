// Copyright (c) 2026 XenseRobotics Co., Ltd. — Apache-2.0

#include <taccap/follower_gripper.hpp>
#include <taccap/error.hpp>
#include <taccap/log.hpp>
#include <taccap/protocol/codec.hpp>
#include <taccap/protocol/payloads.hpp>

#include <chrono>
#include <cstring>

namespace xense::taccap {

namespace {

bus::Transport::Config make_transport_config(const FollowerGripper::Config& cfg) {
    bus::Transport::Config out;
    out.serial.device           = cfg.mcu_device;
    out.serial.baudrate         = cfg.baudrate;
    out.serial.read_timeout_ms  = 1;
    out.serial.write_timeout_ms = 1000;
    out.peer                    = protocol::Address::MCU;
    out.ack_timeout             = std::chrono::milliseconds(cfg.ack_timeout_ms);
    out.max_retries             = cfg.max_retries;
    return out;
}

Camera::Config make_wrist_config(const FollowerGripper::Config& cfg) {
    Camera::Config c = cfg.wrist_cam_extra;
    c.device = cfg.wrist_video;
    if (c.width  <= 0) c.width  = 640;
    if (c.height <= 0) c.height = 480;
    if (c.fps    <= 0) c.fps    = 30.0;
    return c;
}

}  // namespace

FollowerGripper::FollowerGripper(const Config& cfg)
    : cfg_(cfg),
      t_(make_transport_config(cfg)),
      imu_(t_),
      encoder_(t_),
      motor_(t_),
      key_(t_),
      errors_(t_),
      ota_(t_) {
    // Mirror LeaderGripper: drain leftover DATA, then probe firmware
    // version + SN once at construction time so the log shows what the
    // host is talking to.
    try {
        t_.send_cmd(protocol::Cmd::StopStream, {},
                    std::chrono::milliseconds(500));
    } catch (...) { /* fw already idle */ }

    std::string fw_version_str = "<unknown>";
    std::string fw_sn_str      = "<unknown>";
    try {
        auto ack = t_.send_cmd(protocol::Cmd::GetVersion, {},
                               std::chrono::milliseconds(500));
        if (!ack.is_nack &&
            ack.data.size() == sizeof(protocol::FirmwareVersion)) {
            auto v = protocol::decode_version(ack.data.data(),
                                              ack.data.size());
            fw_version_str = std::to_string(v.major) + "." +
                             std::to_string(v.minor) + "." +
                             std::to_string(v.patch) + "." +
                             std::to_string(v.build);
        }
    } catch (...) {}
    try {
        auto ack = t_.send_cmd(protocol::Cmd::GetSn, {},
                               std::chrono::milliseconds(500));
        if (!ack.is_nack && !ack.data.empty()) {
            fw_sn_str = protocol::decode_sn(ack.data.data(),
                                            ack.data.size());
        }
    } catch (...) {}

    logger()->info(
        "FollowerGripper opened: device={} firmware={} sn={} open_cameras={}",
        cfg_.mcu_device, fw_version_str, fw_sn_str, cfg_.open_cameras);

    // Cameras are off by default — an external camera service owns the wrist
    // UVC + OG V4L2 devices. Only open them when explicitly asked AND a
    // device path / serial is provided.
    if (cfg_.open_cameras) {
        if (!cfg_.wrist_video.empty()) {
            wrist_ = std::make_unique<Camera>(make_wrist_config(cfg_));
        }
        if (!cfg_.tactile_left_serial.empty()) {
            tac_l_ = std::make_unique<TactileSensor>(
                TactileSensor::Config{cfg_.tactile_left_serial, cfg_.rectify_tactile});
        }
        if (!cfg_.tactile_right_serial.empty()) {
            tac_r_ = std::make_unique<TactileSensor>(
                TactileSensor::Config{cfg_.tactile_right_serial, cfg_.rectify_tactile});
        }
    }
}

FollowerGripper::~FollowerGripper() {
    try { stop_streaming(); } catch (...) {}
}

std::unique_ptr<FollowerGripper> FollowerGripper::open() {
    // Discovery is MCU-only; cameras are owned externally and stay off
    // (open_cameras defaults to false). A caller that still wants this
    // gripper to drive the cameras must construct it explicitly with
    // open_cameras=true and the device paths/serials.
    auto eps = discovery::find_one();
    Config cfg{};
    cfg.mcu_device = eps.mcu_device;
    return std::make_unique<FollowerGripper>(cfg);
}

void FollowerGripper::start_streaming(unsigned imu_hz, unsigned encoder_hz,
                                      unsigned motor_hz) {
    if (streaming_) return;

    // Drain the firmware queue from any previous host process — same
    // rationale as LeaderGripper::start_streaming.
    try {
        t_.send_cmd(protocol::Cmd::StopStream, {},
                    std::chrono::milliseconds(500));
    } catch (...) { /* expected when fw is already idle */ }

    protocol::StreamConfig sc{};
    sc.source_mask  = protocol::StreamSrc::Imu | protocol::StreamSrc::Encoder;
    if (motor_hz > 0) sc.source_mask |= protocol::StreamSrc::MotorStatus;
    sc.mode         = static_cast<uint8_t>(protocol::StreamMode::Separate);
    sc.imu_rate     = static_cast<uint16_t>(imu_hz);
    sc.encoder_rate = static_cast<uint16_t>(encoder_hz);
    sc.eskin_rate   = 0;
    sc.motor_rate   = static_cast<uint16_t>(motor_hz);
    sc.output_iface = static_cast<uint8_t>(protocol::StreamInterface::Uart);

    auto wire = protocol::encode(sc);
    auto ack = t_.send_cmd(protocol::Cmd::StartStream, wire);
    if (ack.is_nack) {
        throw ProtocolError(std::string("FollowerGripper::start_streaming NACK: ") +
                            protocol::to_string(ack.error_code));
    }

    streaming_ = true;
}

void FollowerGripper::stop_streaming() {
    if (!streaming_) return;
    streaming_ = false;
    try {
        t_.send_cmd(protocol::Cmd::StopStream, {}, std::chrono::milliseconds{500});
    } catch (...) {
        // Best-effort.
    }
}

// ---- V1.7 follower config (reserved; not yet hardware-validated) -----------

protocol::GripperConfig FollowerGripper::get_gripper_config(
        std::chrono::milliseconds timeout) {
    auto ack = t_.send_cmd(protocol::Cmd::GetGripperConfig, {}, timeout);
    if (ack.is_nack) {
        throw ProtocolError(std::string("FollowerGripper::get_gripper_config NACK: ") +
                            protocol::to_string(ack.error_code));
    }
    return protocol::decode_gripper_config(ack.data.data(), ack.data.size());
}

void FollowerGripper::set_gripper_config(const protocol::GripperConfig& cfg) {
    auto ack = t_.send_cmd(protocol::Cmd::SetGripperConfig, protocol::encode(cfg));
    if (ack.is_nack) {
        throw ProtocolError(std::string("FollowerGripper::set_gripper_config NACK: ") +
                            protocol::to_string(ack.error_code));
    }
}

}  // namespace xense::taccap
