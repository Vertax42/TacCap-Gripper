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

#include <atomic>
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

    // Latch the gripper's *current* encoder reading as the new zero
    // position. Firmware snapshots the raw count it sees the instant
    // it processes the command, so the gripper MUST already be held at
    // the desired zero pose (e.g. fully closed) before the call.
    // Throws ProtocolError on NACK or transport timeout.
    void set_zero(std::chrono::milliseconds timeout = std::chrono::milliseconds{500});

    SubId on_data(Callback cb);
    void  off(SubId id);

    static EncoderSample decode(const std::uint8_t* payload, std::size_t len);

private:
    // Post-process a freshly decoded sample: clamp the user-facing
    // position_rad to >= 0 (calibration drift / mechanical slop can
    // make "fully closed" report slightly negative), and emit a rate-
    // limited warning if the underlying drift exceeds a threshold.
    // `raw.position_rad` is left untouched so callers that want the
    // firmware-side value still have it.
    void normalize(EncoderSample& s) const;

    bus::Transport& t_;
    // Last warn timestamp (steady_clock ns since epoch) for the
    // "encoder reading too negative" message. Atomic so the on_data
    // worker thread and a concurrent read_once() caller don't both
    // fire warnings within the same throttle window.
    mutable std::atomic<int64_t> last_neg_warn_ns_{0};
};

}  // namespace xense::taccap
