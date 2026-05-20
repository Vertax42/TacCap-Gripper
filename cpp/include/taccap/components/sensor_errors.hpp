// Copyright (c) 2026 XenseRobotics Co., Ltd. — Apache-2.0
//
// SensorErrors component — V1.6 protocol surface.
//
// Firmware emits a DATA frame with cmd=SensorErrorReport (0x2A) whenever
// a sensor's error state changes (init failure, timeout, data invalid,
// offline, etc.). Payload = `SensorErrorReport` (8 bytes). This replaces
// the older pattern of detecting errors only by NACK on every cmd —
// errors are now pushed asynchronously the moment firmware detects them.
//
// Like Key, this is event-driven and works independently of
// start_streaming() — the report cmd is always emitted on state change.
//
// Note: the firmware's `timestamp` field uses HAL_GetTick() output,
// which is **milliseconds** since boot, NOT microseconds like the
// IMU/encoder packets. We surface it as `mcu_timestamp_ms` to keep the
// unit explicit at the API boundary.

#pragma once

#include <taccap/bus/transport.hpp>
#include <taccap/protocol/payloads.hpp>

#include <chrono>
#include <cstdint>
#include <functional>

namespace xense::taccap {

struct SensorErrorSample {
    std::chrono::steady_clock::time_point host_time;
    uint8_t  sensor_id;          // protocol::SensorErrorId (Imu/ImuMag/Encoder/Eskin1/Eskin2/Motor)
    uint8_t  error_code;         // protocol::SensorErrCode::* (None on recovery)
    uint16_t error_count;        // cumulative for this sensor since boot
    uint32_t mcu_timestamp_ms;   // HAL_GetTick() output — milliseconds

    protocol::SensorErrorReport raw;
};

class SensorErrors {
public:
    using SubId    = bus::Transport::SubscriptionId;
    using Callback = std::function<void(const SensorErrorSample&)>;

    explicit SensorErrors(bus::Transport& transport);

    SubId on_report(Callback cb);
    void  off(SubId id);

    static SensorErrorSample decode(const std::uint8_t* payload, std::size_t len);

private:
    bus::Transport& t_;
};

}  // namespace xense::taccap
