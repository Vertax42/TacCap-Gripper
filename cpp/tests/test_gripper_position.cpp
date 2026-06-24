// Copyright (c) 2026 XenseRobotics Co., Ltd. — Apache-2.0
//
// GripperPosition: pure raw-rad <-> normalized [0,1] converter. No hardware.
// The numeric expectations mirror the real follower measured on 2026-06-25
// (max_open = 1.1802 rad, Reverse): full close raw 0 -> 0, full open
// raw -1.1802 -> 1.0.

#include <gtest/gtest.h>
#include <taccap/gripper_position.hpp>
#include <taccap/protocol/payloads.hpp>

namespace tx = xense::taccap;
namespace tp = xense::taccap::protocol;

namespace {
tp::GripperConfig make_cfg(float max_open, bool reverse, bool valid = true) {
    tp::GripperConfig c{};
    c.magic = tp::GRIPPER_CONFIG_MAGIC;
    c.version = tp::GRIPPER_CONFIG_VERSION;
    c.flags = (valid ? tp::GripperConfigFlag::Valid : 0) |
              (reverse ? tp::GripperConfigFlag::Reverse : 0);
    c.max_open_rad = max_open;
    c.min_open_rad = 0.0f;
    return c;
}
}  // namespace

TEST(GripperPosition, DefaultIsInvalid) {
    tx::GripperPosition gp;
    EXPECT_FALSE(gp.valid());
}

TEST(GripperPosition, InvalidWhenConfigNotValidFlag) {
    tx::GripperPosition gp(make_cfg(1.1802f, true, /*valid=*/false));
    EXPECT_FALSE(gp.valid());
}

TEST(GripperPosition, InvalidWhenSpanNonPositive) {
    EXPECT_FALSE(tx::GripperPosition(make_cfg(0.0f, true)).valid());
}

// The real follower: max_open = 1.1802, Reverse (open is the motor's -dir).
TEST(GripperPosition, ReverseMatchesMeasuredEndpoints) {
    tx::GripperPosition gp(make_cfg(1.1802f, /*reverse=*/true));
    ASSERT_TRUE(gp.valid());
    EXPECT_TRUE(gp.reverse());
    EXPECT_FLOAT_EQ(gp.max_open_rad(), 1.1802f);

    EXPECT_NEAR(gp.to_position(0.0f),     0.0f, 1e-6);   // full close
    EXPECT_NEAR(gp.to_position(-1.1802f), 1.0f, 1e-6);   // full open
    EXPECT_NEAR(gp.to_position(-0.5901f), 0.5f, 1e-4);   // mid

    EXPECT_NEAR(gp.to_rad(0.0f), 0.0f,      1e-6);
    EXPECT_NEAR(gp.to_rad(1.0f), -1.1802f,  1e-6);
    EXPECT_NEAR(gp.to_rad(0.5f), -0.5901f,  1e-4);
}

TEST(GripperPosition, NonReverseIsPositiveDirection) {
    tx::GripperPosition gp(make_cfg(1.0f, /*reverse=*/false));
    EXPECT_NEAR(gp.to_position(1.0f), 1.0f, 1e-6);
    EXPECT_NEAR(gp.to_rad(1.0f),      1.0f, 1e-6);
}

TEST(GripperPosition, ClampsOutOfRange) {
    tx::GripperPosition gp(make_cfg(1.1802f, true));
    // raw beyond the open endpoint and past the closed endpoint both clamp.
    EXPECT_FLOAT_EQ(gp.to_position(-2.0f), 1.0f);   // beyond full open
    EXPECT_FLOAT_EQ(gp.to_position(+0.5f), 0.0f);   // past full close (wrong dir)
    // position inputs clamp to [0,1] before converting.
    EXPECT_FLOAT_EQ(gp.to_rad(1.5f),  gp.to_rad(1.0f));
    EXPECT_FLOAT_EQ(gp.to_rad(-0.2f), gp.to_rad(0.0f));
}

TEST(GripperPosition, RoundTripsAcrossRange) {
    tx::GripperPosition gp(make_cfg(1.1802f, true));
    for (float p = 0.0f; p <= 1.0f; p += 0.1f) {
        EXPECT_NEAR(gp.to_position(gp.to_rad(p)), p, 1e-5);
    }
}
