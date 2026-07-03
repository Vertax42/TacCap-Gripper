// Copyright (c) 2026 XenseRobotics Co., Ltd. — Apache-2.0

#include <taccap/components/led.hpp>

#include <taccap/error.hpp>
#include <taccap/protocol/codec.hpp>
#include <taccap/protocol/commands.hpp>

#include <string>
#include <vector>

namespace xense::taccap {

namespace {
void send_or_throw(bus::Transport& t, protocol::Cmd cmd,
                   const std::vector<uint8_t>& payload, const char* what) {
    auto ack = t.send_cmd(cmd, payload);
    if (ack.is_nack) {
        throw ProtocolError(std::string("Led::") + what + " NACK: " +
                            protocol::to_string(ack.error_code));
    }
}
}  // namespace

Led::Led(bus::Transport& transport) : t_(transport) {}

void Led::set(protocol::Ws2812Mode mode, uint8_t r, uint8_t g, uint8_t b,
              uint8_t brightness, uint16_t blink_ms) {
    protocol::Ws2812Set s{static_cast<uint8_t>(mode), r, g, b, brightness, blink_ms};
    send_or_throw(t_, protocol::Cmd::Ws2812Set, protocol::encode(s), "set");
}

void Led::off() {
    set(protocol::Ws2812Mode::Off, 0, 0, 0);
}

void Led::effect(protocol::Ws2812EffectType effect,
                 uint8_t r1, uint8_t g1, uint8_t b1, uint16_t period_ms,
                 uint8_t r2, uint8_t g2, uint8_t b2,
                 uint8_t param1, uint8_t param2) {
    protocol::Ws2812Effect e{static_cast<uint8_t>(effect), param1, param2, 0,
                             period_ms, r1, g1, b1, r2, g2, b2};
    send_or_throw(t_, protocol::Cmd::Ws2812Effect, protocol::encode(e), "effect");
}

void Led::effect_off() {
    effect(protocol::Ws2812EffectType::None, 0, 0, 0);
}

}  // namespace xense::taccap
