// Copyright (c) 2026 XenseRobotics Co., Ltd. — Apache-2.0

#include <taccap/follower_gripper.hpp>
#include <taccap/error.hpp>
#include <taccap/protocol/codec.hpp>
#include <taccap/protocol/payloads.hpp>

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
      wrist_(make_wrist_config(cfg)) {
    if (!cfg_.tactile_left_serial.empty()) {
        tac_l_ = std::make_unique<TactileSensor>(
            TactileSensor::Config{cfg_.tactile_left_serial, cfg_.rectify_tactile});
    }
    if (!cfg_.tactile_right_serial.empty()) {
        tac_r_ = std::make_unique<TactileSensor>(
            TactileSensor::Config{cfg_.tactile_right_serial, cfg_.rectify_tactile});
    }
}

FollowerGripper::~FollowerGripper() {
    try { stop_streaming(); } catch (...) {}
}

std::unique_ptr<FollowerGripper> FollowerGripper::open() {
    auto eps = discovery::find_one();
    Config cfg{};
    cfg.mcu_device           = eps.mcu_device;
    cfg.wrist_video          = eps.wrist_video;
    cfg.tactile_left_serial  = eps.tactile_left_serial;
    cfg.tactile_right_serial = eps.tactile_right_serial;
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

}  // namespace xense::taccap
