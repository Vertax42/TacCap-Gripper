// Copyright (c) 2026 XenseRobotics Co., Ltd. — Apache-2.0

#include <gtest/gtest.h>

#include <taccap/force_position_controller.hpp>

#include <chrono>

namespace {

using xense::taccap::ForcePositionConfig;
using xense::taccap::ForcePositionState;
using xense::taccap::GripperPosition;
using xense::taccap::MotorStatusSample;
using xense::taccap::detail::ForcePositionPolicy;

MotorStatusSample sample(float pos, float vel = 0.0f, float torque = 0.0f,
                         uint16_t status = 0) {
    MotorStatusSample s{};
    s.actual_pos = pos;
    s.actual_vel = vel;
    s.actual_torque = torque;
    s.status = status;
    return s;
}

}  // namespace

TEST(ForcePositionPolicy, ClosingUsesVelocityDampingWithoutPositionSpring) {
    ForcePositionConfig cfg;
    cfg.close_speed_radps = 0.5f;
    cfg.grasp_torque_nm = 0.35f;
    ForcePositionPolicy p(GripperPosition::from_travel(1.0f), cfg);
    const auto t0 = std::chrono::steady_clock::now();
    p.reset(sample(0.8f), t0);
    p.grasp(t0);

    const auto c = p.step(sample(0.8f), t0);
    EXPECT_EQ(p.state(), ForcePositionState::Closing);
    EXPECT_FLOAT_EQ(c.target_pos, 0.8f);
    EXPECT_FLOAT_EQ(c.kp, 0.0f);
    EXPECT_FLOAT_EQ(c.target_torque, 0.0f);
    EXPECT_FLOAT_EQ(c.vel, -0.5f);
    EXPECT_NEAR(c.kd * std::abs(c.vel), 0.35f, 1e-6f);
    EXPECT_LE(p.commanded_torque_nm(), cfg.motion_torque_limit_nm);
}

TEST(ForcePositionPolicy, ClosingDampingIncludesActualVelocityInTorqueBound) {
    ForcePositionConfig cfg;
    cfg.close_speed_radps = 0.5f;
    cfg.grasp_torque_nm = 0.35f;
    cfg.motion_torque_limit_nm = 1.8f;
    ForcePositionPolicy p(GripperPosition::from_travel(1.0f), cfg);
    const auto now = std::chrono::steady_clock::now();
    p.reset(sample(0.8f), now);
    p.grasp(now);

    const float actual_velocity = 4.0f;  // moving opposite the close target
    const auto c = p.step(sample(0.8f, actual_velocity), now);
    const float predicted = c.kd * (c.vel - actual_velocity);
    EXPECT_LE(std::abs(predicted), cfg.motion_torque_limit_nm + 1e-6f);
    EXPECT_NEAR(std::abs(predicted), p.commanded_torque_nm(), 1e-6f);
}

TEST(ForcePositionPolicy, RuntimeTargetSelectsDirectionAndTorque) {
    ForcePositionConfig cfg;
    cfg.close_speed_radps = 0.5f;
    ForcePositionPolicy p(GripperPosition::from_travel(1.0f), cfg);
    const auto now = std::chrono::steady_clock::now();
    p.reset(sample(0.8f), now);

    p.set_target(sample(0.8f), 0.4f, 0.6f, now);
    const auto closing = p.step(sample(0.8f), now);
    EXPECT_EQ(p.state(), ForcePositionState::Closing);
    EXPECT_FLOAT_EQ(p.target_position(), 0.4f);
    EXPECT_FLOAT_EQ(p.grasp_torque_nm(), 0.6f);
    EXPECT_FLOAT_EQ(closing.vel, -0.5f);
    EXPECT_NEAR(closing.kd * std::abs(closing.vel), 0.6f, 1e-6f);

    p.set_target(sample(0.2f), 0.6f, 0.4f, now);
    const auto opening = p.step(sample(0.2f), now);
    EXPECT_EQ(p.state(), ForcePositionState::Opening);
    EXPECT_FLOAT_EQ(opening.vel, 0.5f);

    const auto arrived = p.step(sample(0.6f), now);
    EXPECT_EQ(p.state(), ForcePositionState::HoldingPosition);
    EXPECT_FLOAT_EQ(arrived.target_pos, 0.6f);
}

TEST(ForcePositionPolicy, RuntimeTargetContactUsesDynamicTorque) {
    ForcePositionConfig cfg;
    cfg.startup_guard_ms = 0;
    cfg.contact_samples = 1;
    ForcePositionPolicy p(GripperPosition::from_travel(1.0f), cfg);
    const auto now = std::chrono::steady_clock::now();
    p.reset(sample(0.8f), now);
    p.set_target(sample(0.8f), 0.2f, 0.6f, now);

    const auto holding = p.step(sample(0.5f, 0.0f, 0.3f), now);
    EXPECT_EQ(p.state(), ForcePositionState::HoldingForce);
    EXPECT_FLOAT_EQ(holding.target_torque, -0.6f);
    EXPECT_FLOAT_EQ(p.target_position(), 0.2f);
    EXPECT_FLOAT_EQ(p.hold_position(), 0.5f);
}

TEST(ForcePositionPolicy, RuntimeTargetRejectsUnsafeValues) {
    ForcePositionConfig cfg;
    ForcePositionPolicy p(GripperPosition::from_travel(1.0f), cfg);
    const auto now = std::chrono::steady_clock::now();
    p.reset(sample(0.5f), now);

    EXPECT_THROW(p.set_target(sample(0.5f), -0.1f, 0.3f, now),
                 std::invalid_argument);
    EXPECT_THROW(p.set_target(sample(0.5f), 0.3f, 1.81f, now),
                 std::invalid_argument);
}

TEST(ForcePositionPolicy, ConfirmedContactSwitchesToPureBoundedTorqueHold) {
    ForcePositionConfig cfg;
    cfg.grasp_torque_nm = 0.35f;
    cfg.startup_guard_ms = 250;
    cfg.contact_samples = 2;
    ForcePositionPolicy p(GripperPosition::from_travel(1.0f), cfg);
    const auto t0 = std::chrono::steady_clock::now();
    p.reset(sample(0.8f), t0);
    p.grasp(t0);

    const auto after_guard = t0 + std::chrono::milliseconds(300);
    p.step(sample(0.5f, 0.0f, 0.20f), after_guard);
    p.step(sample(0.5f, 0.0f, 0.20f),
           after_guard + std::chrono::milliseconds(10));
    EXPECT_EQ(p.state(), ForcePositionState::Closing);

    p.step(sample(0.5f, 0.0f, 0.30f),
           after_guard + std::chrono::milliseconds(20));
    const auto c = p.step(sample(0.5f, 0.0f, 0.30f),
                          after_guard + std::chrono::milliseconds(30));

    EXPECT_EQ(p.state(), ForcePositionState::HoldingForce);
    EXPECT_FLOAT_EQ(c.kp, 0.0f);
    EXPECT_FLOAT_EQ(c.kd, 0.0f);
    EXPECT_FLOAT_EQ(c.target_torque, -0.35f);
    EXPECT_FLOAT_EQ(c.target_pos, 0.5f);
    EXPECT_LE(std::abs(c.target_torque), cfg.hold_torque_limit_nm);
}

TEST(ForcePositionPolicy, ForceHoldUsesSoftwareCeilingNotMotionLimit) {
    ForcePositionConfig cfg;
    cfg.grasp_torque_nm = 1.8f;
    cfg.startup_guard_ms = 0;
    cfg.contact_samples = 1;
    ForcePositionPolicy p(GripperPosition::from_travel(1.0f), cfg);
    const auto now = std::chrono::steady_clock::now();
    p.reset(sample(0.8f), now);
    p.grasp(now);

    const auto holding = p.step(sample(0.5f, 0.0f, 0.7f), now);
    EXPECT_EQ(p.state(), ForcePositionState::HoldingForce);
    EXPECT_FLOAT_EQ(holding.kp, 0.0f);
    EXPECT_FLOAT_EQ(holding.kd, 0.0f);
    EXPECT_FLOAT_EQ(holding.target_torque, -1.8f);
    EXPECT_LT(std::abs(holding.target_torque), cfg.motion_torque_limit_nm);
}

TEST(ForcePositionPolicy, ReverseMapFlipsCloseTorqueAndVelocity) {
    ForcePositionConfig cfg;
    cfg.startup_guard_ms = 0;
    cfg.contact_samples = 1;
    ForcePositionPolicy p(GripperPosition::from_travel(1.0f, 0.0f, true), cfg);
    const auto t0 = std::chrono::steady_clock::now();
    p.reset(sample(-0.8f), t0);
    p.grasp(t0);

    const auto moving = p.step(sample(-0.8f), t0);
    EXPECT_GT(moving.vel, 0.0f);
    const auto holding = p.step(sample(-0.5f, 0.0f, 0.3f),
                                t0 + std::chrono::milliseconds(1));
    EXPECT_EQ(p.state(), ForcePositionState::HoldingForce);
    EXPECT_FLOAT_EQ(holding.target_torque, cfg.grasp_torque_nm);
}

TEST(ForcePositionPolicy, PositionHoldUsesMotionLimitAboveHoldLimit) {
    ForcePositionConfig cfg;
    cfg.position_kp = 20.0f;
    cfg.position_kd = 1.0f;
    ForcePositionPolicy p(GripperPosition::from_travel(1.0f), cfg);
    const auto now = std::chrono::steady_clock::now();
    p.reset(sample(0.0f), now);  // desired hold stays at raw zero

    const auto c = p.step(sample(1.0f), now);
    const float predicted = c.kp * (c.target_pos - 1.0f);
    EXPECT_NEAR(c.target_pos, 0.7f, 1e-6f);
    EXPECT_NEAR(std::abs(predicted), 6.0f, 1e-5f);
    EXPECT_LE(p.commanded_torque_nm(), cfg.motion_torque_limit_nm);
}

TEST(ForcePositionPolicy, FeedbackBetweenHoldAndMotionLimitsIsAllowed) {
    ForcePositionConfig cfg;
    cfg.startup_guard_ms = 1000;
    ForcePositionPolicy p(GripperPosition::from_travel(1.0f), cfg);
    const auto now = std::chrono::steady_clock::now();
    p.reset(sample(0.5f), now);
    p.grasp(now);

    p.step(sample(0.5f, 0.0f, 3.3f), now);
    EXPECT_EQ(p.state(), ForcePositionState::Closing);
    EXPECT_TRUE(p.fault_reason().empty());
}

TEST(ForcePositionPolicy, FeedbackOverMotionLimitTransitionsToZeroTorqueFault) {
    ForcePositionConfig cfg;
    ForcePositionPolicy p(GripperPosition::from_travel(1.0f), cfg);
    const auto now = std::chrono::steady_clock::now();
    p.reset(sample(0.5f), now);
    p.grasp(now);

    const auto c = p.step(sample(0.5f, 0.0f, 6.01f), now);
    EXPECT_EQ(p.state(), ForcePositionState::Fault);
    EXPECT_FLOAT_EQ(c.kp, 0.0f);
    EXPECT_FLOAT_EQ(c.kd, 0.0f);
    EXPECT_FLOAT_EQ(c.target_torque, 0.0f);
    EXPECT_FALSE(p.fault_reason().empty());
}

TEST(ForcePositionPolicy, RejectsGraspTorqueAboveMaximum) {
    ForcePositionConfig cfg;
    cfg.grasp_torque_nm = 2.0f;
    EXPECT_THROW(
        ForcePositionPolicy(GripperPosition::from_travel(1.0f), cfg),
        std::invalid_argument);
}

TEST(ForcePositionPolicy, RejectsHoldLimitAboveSoftwareMaximum) {
    ForcePositionConfig cfg;
    cfg.hold_torque_limit_nm = 1.81f;
    EXPECT_THROW(
        ForcePositionPolicy(GripperPosition::from_travel(1.0f), cfg),
        std::invalid_argument);
}

TEST(ForcePositionPolicy, RejectsMotionLimitAboveDeviceMaximum) {
    ForcePositionConfig cfg;
    cfg.motion_torque_limit_nm = 6.01f;
    EXPECT_THROW(
        ForcePositionPolicy(GripperPosition::from_travel(1.0f), cfg),
        std::invalid_argument);
}

TEST(ForcePositionPolicy, RejectsHoldLimitAboveMotionLimit) {
    ForcePositionConfig cfg;
    cfg.hold_torque_limit_nm = 1.0f;
    cfg.motion_torque_limit_nm = 0.8f;
    EXPECT_THROW(
        ForcePositionPolicy(GripperPosition::from_travel(1.0f), cfg),
        std::invalid_argument);
}
