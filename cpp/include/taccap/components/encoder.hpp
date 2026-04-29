// Copyright (c) 2026 XenseRobotics Co., Ltd. — Apache-2.0
//
// Encoder component: typed wrapper around Cmd::GetEncoder (one-shot) and
// FrameType::DATA frames carrying EncoderData (continuous stream).
//
// On the leader gripper this reads the dedicated encoder hardware. On the
// follower gripper the same telemetry is also available through MotorStatus
// (handled separately by the Motor component).

#pragma once

#include <taccap/bus/transport.hpp>
#include <taccap/protocol/payloads.hpp>

#include <chrono>
#include <cstdint>
#include <functional>

namespace xense::taccap {

struct EncoderSample {
    std::chrono::steady_clock::time_point host_time;
    uint32_t mcu_timestamp_us;
    uint16_t status;         // protocol::EncoderStatusBit::* bits (Ok/Error/Overflow)
    uint16_t seq;
    float    position_rad;
    float    velocity_rad_s;

    protocol::EncoderData raw;
};

class Encoder {
public:
    using SubId    = bus::Transport::SubscriptionId;
    using Callback = std::function<void(const EncoderSample&)>;

    explicit Encoder(bus::Transport& transport);

    EncoderSample read_once(std::chrono::milliseconds timeout = std::chrono::milliseconds{100});

    SubId on_data(Callback cb);
    void  off(SubId id);

    static EncoderSample decode(const std::uint8_t* payload, std::size_t len);

private:
    bus::Transport& t_;
};

}  // namespace xense::taccap
