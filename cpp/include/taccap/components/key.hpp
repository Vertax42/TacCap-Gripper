// Copyright (c) 2026 XenseRobotics Co., Ltd. — Apache-2.0
//
// Key (button) component — V1.4 protocol surface.
//
// The TC-GU-01 board has a physical button (K1). Firmware emits a DATA
// frame with cmd=KeyStatus (0x15) whenever its state changes, payload =
// `KeyStatusPayload` (2 bytes: key_id + key_state). Five state values:
// single-click down/up, double-click, long-press down/up — see
// `protocol::KeyState`.
//
// The button works WITHOUT calling start_streaming() — KeyStatus DATA
// is event-driven, not stream-driven; the firmware sends one frame per
// state change regardless of stream configuration.

#pragma once

#include <taccap/bus/transport.hpp>
#include <taccap/protocol/payloads.hpp>

#include <chrono>
#include <cstdint>
#include <functional>

namespace xense::taccap {

struct KeySample {
    std::chrono::steady_clock::time_point host_time;  // ingress timestamp
    uint8_t key_id;        // 0 = K1 (only key on TC-GU-01 today)
    uint8_t key_state;     // protocol::KeyState::*

    protocol::KeyStatusPayload raw;
};

class Key {
public:
    using SubId    = bus::Transport::SubscriptionId;
    using Callback = std::function<void(const KeySample&)>;

    explicit Key(bus::Transport& transport);

    // Subscribe to button events. Returns SubId for later unsubscribe.
    SubId on_event(Callback cb);
    void  off(SubId id);

    // Pure decoder, exposed for tests / debug.
    static KeySample decode(const std::uint8_t* payload, std::size_t len);

private:
    bus::Transport& t_;
};

}  // namespace xense::taccap
