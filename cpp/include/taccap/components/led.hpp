// Copyright (c) 2026 XenseRobotics Co., Ltd. — Apache-2.0
//
// Led — WS2812 LED control component (V1.9 protocol surface).
//
// The TC-GU-01 board carries a WS2812 addressable LED. Two command layers:
//   - set()    (Cmd::Ws2812Set 0x0A):    a base color + optional blink.
//   - effect() (Cmd::Ws2812Effect 0x0B): a preset or parametric effect
//     (blink / breathe / HSV cycle / two-color LERP) that composites over the
//     base color additively.
// Both are ACK'd commands (response payload = 1-byte error code).

#pragma once

#include <taccap/bus/transport.hpp>
#include <taccap/protocol/payloads.hpp>

#include <cstdint>

namespace xense::taccap {

class Led {
public:
    explicit Led(bus::Transport& transport);

    // Base color layer. `brightness` = 0 leaves the global brightness
    // unchanged; `blink_ms` = 0 disables blinking (steady).
    void set(protocol::Ws2812Mode mode,
             uint8_t r, uint8_t g, uint8_t b,
             uint8_t brightness = 0,
             uint16_t blink_ms  = 0);

    // Convenience: turn the LED off.
    void off();

    // Effect layer. `r2/g2/b2` are only used by the two-color LERP effect;
    // `param1/param2` are effect-specific (e.g. breathe min/max brightness).
    void effect(protocol::Ws2812EffectType effect,
                uint8_t r1, uint8_t g1, uint8_t b1,
                uint16_t period_ms = 1000,
                uint8_t r2 = 0, uint8_t g2 = 0, uint8_t b2 = 0,
                uint8_t param1 = 0, uint8_t param2 = 0);

    // Convenience: stop any running effect.
    void effect_off();

private:
    bus::Transport& t_;
};

}  // namespace xense::taccap
