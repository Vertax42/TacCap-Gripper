// Copyright (c) 2026 XenseRobotics Co., Ltd. — Apache-2.0

#include <taccap/components/sensor_errors.hpp>
#include <taccap/protocol/codec.hpp>

namespace xense::taccap {

SensorErrors::SensorErrors(bus::Transport& transport) : t_(transport) {}

SensorErrorSample SensorErrors::decode(const std::uint8_t* payload, std::size_t len) {
    SensorErrorSample s{};
    s.host_time         = std::chrono::steady_clock::now();
    s.raw               = protocol::decode_sensor_error(payload, len);
    s.sensor_id         = s.raw.sensor_id;
    s.error_code        = s.raw.error_code;
    s.error_count       = s.raw.error_count;
    s.mcu_timestamp_ms  = s.raw.timestamp_ms;
    return s;
}

SensorErrors::SubId SensorErrors::on_report(Callback cb) {
    return t_.subscribe(
        protocol::Cmd::SensorErrorReport,
        [cb = std::move(cb)](const bus::Frame& f) {
            try {
                cb(decode(f.payload.data(), f.payload.size()));
            } catch (...) {}
        });
}

void SensorErrors::off(SubId id) { t_.unsubscribe(id); }

}  // namespace xense::taccap
