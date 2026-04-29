// Copyright (c) 2026 XenseRobotics Co., Ltd. — Apache-2.0

#include <taccap/components/encoder.hpp>
#include <taccap/error.hpp>
#include <taccap/protocol/codec.hpp>

#include <cstring>

namespace xense::taccap {

Encoder::Encoder(bus::Transport& transport) : t_(transport) {}

EncoderSample Encoder::decode(const std::uint8_t* payload, std::size_t len) {
    EncoderSample s{};
    s.host_time          = std::chrono::steady_clock::now();
    s.raw                = protocol::decode_encoder(payload, len);
    s.mcu_timestamp_us   = s.raw.timestamp_us;
    s.position_rad       = s.raw.position_rad;
    s.velocity_rad_s     = s.raw.velocity_rad_s;
    s.status             = s.raw.status;
    s.seq                = s.raw.seq;
    return s;
}

EncoderSample Encoder::read_once(std::chrono::milliseconds timeout) {
    auto ack = t_.send_cmd(protocol::Cmd::GetEncoder, {}, timeout);
    if (ack.is_nack) {
        throw ProtocolError(std::string("Encoder::read_once NACK: ") +
                            protocol::to_string(ack.error_code));
    }
    return decode(ack.data.data(), ack.data.size());
}

Encoder::SubId Encoder::on_data(Callback cb) {
    return t_.subscribe(
        protocol::Cmd::GetEncoder,
        [cb = std::move(cb)](const bus::Frame& f) {
            try {
                cb(decode(f.payload.data(), f.payload.size()));
            } catch (...) {}
        });
}

void Encoder::off(SubId id) { t_.unsubscribe(id); }

}  // namespace xense::taccap
