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
    EXPECT_EQ(sizeof(tp::MotorStatus),        40u);
    EXPECT_EQ(sizeof(tp::MotorImpedanceCtrl), 20u);
    EXPECT_EQ(sizeof(tp::GripperConfig),      32u);
    EXPECT_EQ(sizeof(tp::MotorControlStats),  48u);
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

// ---- MotorStatus: full 40-byte decode -------------------------------------
TEST(CodecV17, MotorStatusFullDecode) {
    tp::MotorStatus s{};
    s.actual_pos = 0.5f;  s.actual_vel = -1.5f; s.actual_torque = 0.25f;
    s.motor_temp = 37.0f; s.status = 0x0001;
    s.actual_current = 1.2f; s.target_pos = 0.6f; s.target_vel = -1.0f;
    s.target_torque = 0.3f;  s.target_current = 1.1f;
    s.control_mode = static_cast<uint8_t>(tp::MotorMode::Impedance);
    s.current_source = 1;
    auto b = as_bytes(s);

    auto d = tp::decode_motor_status(b.data(), b.size());
    EXPECT_FLOAT_EQ(d.actual_pos, 0.5f);
    EXPECT_FLOAT_EQ(d.actual_current, 1.2f);
    EXPECT_FLOAT_EQ(d.target_torque, 0.3f);
    EXPECT_EQ(d.control_mode, static_cast<uint8_t>(tp::MotorMode::Impedance));
    EXPECT_EQ(d.current_source, 1u);
}

// ---- MotorStatus: legacy 18-byte payload still decodes (new fields = 0) ---
TEST(CodecV17, MotorStatusLegacy18ByteIsLenient) {
    tp::MotorStatus s{};
    s.actual_pos = 0.5f; s.actual_vel = 2.0f; s.actual_torque = 0.1f;
    s.motor_temp = 40.0f; s.status = 0x0002;
    auto full = as_bytes(s);
    full.resize(18);  // truncate to the V1.6 prefix

    auto d = tp::decode_motor_status(full.data(), full.size());
    EXPECT_FLOAT_EQ(d.actual_pos, 0.5f);
    EXPECT_FLOAT_EQ(d.motor_temp, 40.0f);
    EXPECT_EQ(d.status, 0x0002u);
    EXPECT_FLOAT_EQ(d.actual_current, 0.0f);  // appended field defaults to 0
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
