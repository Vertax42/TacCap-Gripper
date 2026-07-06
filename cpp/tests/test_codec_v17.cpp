// Copyright (c) 2026 XenseRobotics Co., Ltd. — Apache-2.0
//
// V1.7 protocol codec tests: the follower/motor payload growth and the new
// gripper-config / control-stats structs. Pure codec — no hardware needed.

#include <cstring>
#include <gtest/gtest.h>
#include <taccap/error.hpp>
#include <taccap/protocol/codec.hpp>
#include <taccap/protocol/payloads.hpp>

namespace tp = xense::taccap::protocol;

namespace {
template <typename T>
std::vector<uint8_t> as_bytes(const T& v) {
    std::vector<uint8_t> b(sizeof(T));
    std::memcpy(b.data(), &v, sizeof(T));
    return b;
}
}  // namespace

// ---- Wire sizes (mirror firmware protocol_data.h) -------------------------
TEST(CodecV17, StructSizes) {
    EXPECT_EQ(sizeof(tp::MotorStatus),        31u);  // V1.9 (was 40)
    EXPECT_EQ(sizeof(tp::MotorImpedanceCtrl), 20u);
    EXPECT_EQ(sizeof(tp::GripperConfig),      32u);
    EXPECT_EQ(sizeof(tp::MotorControlStats),  48u);
    EXPECT_EQ(sizeof(tp::GripperAutoCalConfig), 32u);  // V1.9
    EXPECT_EQ(sizeof(tp::Ws2812Set),          7u);   // V1.9
    EXPECT_EQ(sizeof(tp::Ws2812Effect),       12u);  // V1.9
}

// ---- MotorImpedanceCtrl gained a feed-forward vel (16 -> 20) --------------
TEST(CodecV17, ImpedanceCtrlEncodesVel) {
    tp::MotorImpedanceCtrl c{1.0f, 2.0f, 3.0f, 4.0f, 5.0f};
    auto wire = tp::encode(c);
    ASSERT_EQ(wire.size(), 20u);
    float vel = 0.0f;
    std::memcpy(&vel, wire.data() + 16, sizeof(float));  // 5th float
    EXPECT_FLOAT_EQ(vel, 5.0f);
}

// ---- Control-frame encoders pin the wire bytes of Motor::submit() ---------
TEST(CodecV17, PosCtrlEncodeRoundtrip) {
    tp::MotorPosCtrl c{0.5f, 10.0f, 1.25f};
    auto wire = tp::encode(c);
    ASSERT_EQ(wire.size(), 12u);
    float f[3];
    std::memcpy(f, wire.data(), 12);
    EXPECT_FLOAT_EQ(f[0], 0.5f);   // target_pos
    EXPECT_FLOAT_EQ(f[1], 10.0f);  // max_vel
    EXPECT_FLOAT_EQ(f[2], 1.25f);  // max_torque
}

TEST(CodecV17, VelCtrlEncodeRoundtrip) {
    tp::MotorVelCtrl c{-2.0f, 0.75f, 5.0f};
    auto wire = tp::encode(c);
    ASSERT_EQ(wire.size(), 12u);
    float f[3];
    std::memcpy(f, wire.data(), 12);
    EXPECT_FLOAT_EQ(f[0], -2.0f);  // target_vel
    EXPECT_FLOAT_EQ(f[1], 0.75f);  // max_torque
    EXPECT_FLOAT_EQ(f[2], 5.0f);   // profile_acc
}

TEST(CodecV17, TorqueCtrlEncodeRoundtrip) {
    tp::MotorTorqueCtrl c{0.3f, 8.0f, 0.0f};
    auto wire = tp::encode(c);
    ASSERT_EQ(wire.size(), 12u);
    float f[3];
    std::memcpy(f, wire.data(), 12);
    EXPECT_FLOAT_EQ(f[0], 0.3f);   // target_torque
    EXPECT_FLOAT_EQ(f[1], 8.0f);   // max_vel
    EXPECT_FLOAT_EQ(f[2], 0.0f);   // reserved padding
}

// ---- MotorStatus: full 31-byte (V1.9) decode ------------------------------
TEST(CodecV17, MotorStatusFullDecode) {
    tp::MotorStatus s{};
    s.actual_pos = 0.5f;  s.actual_vel = -1.5f; s.actual_torque = 0.25f;
    s.motor_temp = 37.0f; s.status = 0x0001;
    s.target_pos = 0.6f;  s.target_vel = -1.0f; s.target_torque = 0.3f;
    s.control_mode = static_cast<uint8_t>(tp::MotorMode::Impedance);
    auto b = as_bytes(s);
    ASSERT_EQ(b.size(), 31u);

    auto d = tp::decode_motor_status(b.data(), b.size());
    EXPECT_FLOAT_EQ(d.actual_pos, 0.5f);
    EXPECT_FLOAT_EQ(d.actual_torque, 0.25f);
    EXPECT_FLOAT_EQ(d.target_pos, 0.6f);
    EXPECT_FLOAT_EQ(d.target_torque, 0.3f);
    EXPECT_EQ(d.control_mode, static_cast<uint8_t>(tp::MotorMode::Impedance));
}

// ---- MotorStatus: an 18-byte prefix still yields pose/torque/status --------
// (position / torque / status share offsets across all firmware versions.)
TEST(CodecV17, MotorStatusPrefixIsLenient) {
    tp::MotorStatus s{};
    s.actual_pos = 0.5f; s.actual_vel = 2.0f; s.actual_torque = 0.1f;
    s.motor_temp = 40.0f; s.status = 0x0002;
    auto full = as_bytes(s);
    full.resize(18);  // just the common prefix

    auto d = tp::decode_motor_status(full.data(), full.size());
    EXPECT_FLOAT_EQ(d.actual_pos, 0.5f);
    EXPECT_FLOAT_EQ(d.motor_temp, 40.0f);
    EXPECT_EQ(d.status, 0x0002u);
    EXPECT_FLOAT_EQ(d.target_pos, 0.0f);  // tail beyond the prefix defaults to 0
    EXPECT_EQ(d.control_mode, 0u);
}

TEST(CodecV17, MotorStatusRejectsTooShort) {
    std::vector<uint8_t> tiny(17, 0);
    EXPECT_THROW(tp::decode_motor_status(tiny.data(), tiny.size()),
                 xense::taccap::ProtocolError);
}

// ---- GripperConfig encode/decode roundtrip --------------------------------
TEST(CodecV17, GripperConfigRoundtrip) {
    tp::GripperConfig c{};
    c.magic   = tp::GRIPPER_CONFIG_MAGIC;
    c.version = tp::GRIPPER_CONFIG_VERSION;
    c.flags   = tp::GripperConfigFlag::Valid | tp::GripperConfigFlag::Reverse;
    c.max_open_rad = 1.30f;
    c.min_open_rad = 0.0f;

    auto wire = tp::encode(c);
    ASSERT_EQ(wire.size(), 32u);

    auto d = tp::decode_gripper_config(wire.data(), wire.size());
    EXPECT_EQ(d.magic, tp::GRIPPER_CONFIG_MAGIC);
    EXPECT_EQ(d.version, tp::GRIPPER_CONFIG_VERSION);
    EXPECT_EQ(d.flags, tp::GripperConfigFlag::Valid | tp::GripperConfigFlag::Reverse);
    EXPECT_FLOAT_EQ(d.max_open_rad, 1.30f);
}

// ---- MotorControlStats decode ---------------------------------------------
TEST(CodecV17, MotorControlStatsDecode) {
    tp::MotorControlStats s{};
    s.running = 1; s.mode = 1; s.target_hz = 500; s.actual_hz = 498.5f;
    s.loop_count = 123456u; s.error_count = 2u; s.last_error = -5;
    auto b = as_bytes(s);
    ASSERT_EQ(b.size(), 48u);

    auto d = tp::decode_motor_control_stats(b.data(), b.size());
    EXPECT_EQ(d.running, 1u);
    EXPECT_EQ(d.target_hz, 500u);
    EXPECT_FLOAT_EQ(d.actual_hz, 498.5f);
    EXPECT_EQ(d.loop_count, 123456u);
    EXPECT_EQ(d.last_error, -5);
}

// ---- V1.9: gripper auto-cal config roundtrip ------------------------------
TEST(CodecV17, GripperAutoCalConfigRoundtrip) {
    tp::GripperAutoCalConfig c{};
    c.magic   = tp::GRIPPER_AUTO_CAL_MAGIC;
    c.version = tp::GRIPPER_AUTO_CAL_VERSION;
    c.flags   = tp::GripperAutoCalFlag::Valid | tp::GripperAutoCalFlag::Enable;
    c.close_stall_torque_nm = 0.22f;
    c.open_stall_torque_nm  = 0.20f;
    c.close_speed_rad_s = 0.15f;
    c.open_speed_rad_s  = 0.20f;
    c.stall_hold_ms = 50; c.startup_delay_ms = 200; c.post_zero_delay_ms = 100;
    c.close_confirm_count = 3; c.open_confirm_count = 3;

    auto wire = tp::encode(c);
    ASSERT_EQ(wire.size(), 32u);
    auto d = tp::decode_gripper_auto_cal_config(wire.data(), wire.size());
    EXPECT_EQ(d.magic, tp::GRIPPER_AUTO_CAL_MAGIC);
    EXPECT_EQ(d.flags, tp::GripperAutoCalFlag::Valid | tp::GripperAutoCalFlag::Enable);
    EXPECT_FLOAT_EQ(d.close_stall_torque_nm, 0.22f);
    EXPECT_FLOAT_EQ(d.open_speed_rad_s, 0.20f);
    EXPECT_EQ(d.close_confirm_count, 3u);
}

// ---- V1.9: WS2812 LED payloads pin the wire layout ------------------------
TEST(CodecV17, Ws2812SetEncodeLayout) {
    tp::Ws2812Set s{static_cast<uint8_t>(tp::Ws2812Mode::Override), 10, 20, 30, 200, 500};
    auto w = tp::encode(s);
    ASSERT_EQ(w.size(), 7u);
    EXPECT_EQ(w[0], static_cast<uint8_t>(tp::Ws2812Mode::Override));
    EXPECT_EQ(w[1], 10u); EXPECT_EQ(w[2], 20u); EXPECT_EQ(w[3], 30u);
    EXPECT_EQ(w[4], 200u);
    uint16_t blink; std::memcpy(&blink, w.data() + 5, 2);
    EXPECT_EQ(blink, 500u);
}

TEST(CodecV17, Ws2812EffectEncodeLayout) {
    tp::Ws2812Effect e{static_cast<uint8_t>(tp::Ws2812EffectType::ColorLerp),
                       5, 250, 0, 1000, 255, 0, 0, 0, 0, 255};
    auto w = tp::encode(e);
    ASSERT_EQ(w.size(), 12u);
    EXPECT_EQ(w[0], static_cast<uint8_t>(tp::Ws2812EffectType::ColorLerp));
    uint16_t period; std::memcpy(&period, w.data() + 4, 2);
    EXPECT_EQ(period, 1000u);
    EXPECT_EQ(w[6], 255u);   // r1
    EXPECT_EQ(w[11], 255u);  // b2
}

// ---- V1.9+: private-protocol single param GET response ---------------------
TEST(CodecV17, MotorPrivateParamDecode) {
    tp::MotorPrivateParam p{};
    p.index = 0x0102; p.type = tp::MotorPrivateParamType::F32;
    p.access = tp::MotorPrivateParamAccess::Read | tp::MotorPrivateParamAccess::Write;
    float v = 3.5f; std::memcpy(&p.raw_value, &v, 4);
    auto b = as_bytes(p);
    ASSERT_EQ(b.size(), 8u);

    auto d = tp::decode_motor_private_param(b.data(), b.size());
    EXPECT_EQ(d.index, 0x0102u);
    EXPECT_EQ(d.type, tp::MotorPrivateParamType::F32);
    EXPECT_EQ(d.access, tp::MotorPrivateParamAccess::Read | tp::MotorPrivateParamAccess::Write);
    float back; std::memcpy(&back, &d.raw_value, 4);
    EXPECT_FLOAT_EQ(back, 3.5f);
}
