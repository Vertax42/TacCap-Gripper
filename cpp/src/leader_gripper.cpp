// Copyright (c) 2026 XenseRobotics Co., Ltd. — Apache-2.0

#include <taccap/leader_gripper.hpp>
#include <taccap/error.hpp>
#include <taccap/protocol/codec.hpp>
#include <taccap/protocol/payloads.hpp>

#include <cstring>

namespace xense::taccap {

namespace {

bus::Transport::Config make_transport_config(const LeaderGripper::Config& cfg) {
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

Camera::Config make_wrist_config(const LeaderGripper::Config& cfg) {
    Camera::Config c = cfg.wrist_cam_extra;
    c.device = cfg.wrist_video;
    if (c.width  <= 0) c.width  = 640;
    if (c.height <= 0) c.height = 480;
    if (c.fps    <= 0) c.fps    = 30.0;
    return c;
}

}  // namespace

LeaderGripper::LeaderGripper(const Config& cfg)
    : cfg_(cfg),
      t_(make_transport_config(cfg)),
      imu_(t_),
      encoder_(t_),
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

LeaderGripper::~LeaderGripper() {
    try { stop_streaming(); } catch (...) {}
}

std::unique_ptr<LeaderGripper> LeaderGripper::open() {
    auto eps = discovery::find_one();
    Config cfg{};
    cfg.mcu_device           = eps.mcu_device;
    cfg.wrist_video          = eps.wrist_video;
    cfg.tactile_left_serial  = eps.tactile_left_serial;
    cfg.tactile_right_serial = eps.tactile_right_serial;
    return std::make_unique<LeaderGripper>(cfg);
}

void LeaderGripper::start_streaming(unsigned imu_hz, unsigned encoder_hz) {
    if (streaming_) return;

    // Build StreamConfig (12 bytes — see protocol::StreamConfig).
    protocol::StreamConfig sc{};
    sc.source_mask  = protocol::StreamSrc::Imu | protocol::StreamSrc::Encoder;
    sc.mode         = static_cast<uint8_t>(protocol::StreamMode::Separate);
    sc.imu_rate     = static_cast<uint16_t>(imu_hz);
    sc.encoder_rate = static_cast<uint16_t>(encoder_hz);
    sc.eskin_rate   = 0;
    sc.motor_rate   = 0;
    sc.output_iface = static_cast<uint8_t>(protocol::StreamInterface::Uart);

    auto wire = protocol::encode(sc);
    auto ack = t_.send_cmd(protocol::Cmd::StartStream, wire);
    if (ack.is_nack) {
        throw ProtocolError(std::string("LeaderGripper::start_streaming NACK: ") +
                            protocol::to_string(ack.error_code));
    }

    // The MCU now begins emitting DATA frames; subscribers on imu_/encoder_
    // start receiving immediately.

    // Cameras stream independently of the MCU stream; the caller has set
    // their callbacks via tactile_left().start(cb) etc. We do NOT auto-
    // start them here because the user might want raw access patterns.
    // Mirror this in the documentation above.

    streaming_ = true;
}

void LeaderGripper::stop_streaming() {
    if (!streaming_) return;
    streaming_ = false;
    try {
        t_.send_cmd(protocol::Cmd::StopStream, {}, std::chrono::milliseconds{500});
    } catch (...) {
        // Best-effort: even if the MCU doesn't ACK we proceed; tearing down
        // the host-side resources is more important than a clean fw stop.
    }
}

}  // namespace xense::taccap
