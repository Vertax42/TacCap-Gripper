// Copyright (c) 2026 XenseRobotics Co., Ltd. — Apache-2.0

#include <taccap/components/key.hpp>
#include <taccap/protocol/codec.hpp>

namespace xense::taccap {

Key::Key(bus::Transport& transport) : t_(transport) {}

KeySample Key::decode(const std::uint8_t* payload, std::size_t len) {
    KeySample s{};
    s.host_time = std::chrono::steady_clock::now();
    s.raw       = protocol::decode_key_status(payload, len);
    s.key_id    = s.raw.key_id;
    s.key_state = s.raw.key_state;
    return s;
}

Key::SubId Key::on_event(Callback cb) {
    return t_.subscribe(
        protocol::Cmd::KeyStatus,
        [cb = std::move(cb)](const bus::Frame& f) {
            try {
                cb(decode(f.payload.data(), f.payload.size()));
            } catch (...) {}
        });
}

void Key::off(SubId id) { t_.unsubscribe(id); }

}  // namespace xense::taccap
