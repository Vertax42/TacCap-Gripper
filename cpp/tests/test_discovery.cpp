// Copyright (c) 2026 XenseRobotics Co., Ltd. — Apache-2.0
//
// Discovery SN-parsing tests. parse_serial() / side_from_serial() are pure
// string functions, so no hardware or transport is needed here.

#include <gtest/gtest.h>
#include <taccap/discovery.hpp>

namespace d = xense::taccap::discovery;
using Side = d::Side;
using Role = d::Role;

// ---- Full TacCap grammar: grippers (TCGU…, with m/s patch) ----------------

TEST(ParseSerial, GripperLeaderLeft) {
    auto p = d::parse_serial("TCGU01A24Z0001m");
    EXPECT_TRUE(p.valid);
    EXPECT_EQ(p.product, "TCGU01");
    EXPECT_EQ(p.batch, "A24");
    EXPECT_EQ(p.line, 'Z');
    EXPECT_EQ(p.sequence, "0001");
    ASSERT_TRUE(p.side.has_value());
    EXPECT_EQ(*p.side, Side::Left);     // seq ends 1 (odd)
    EXPECT_EQ(p.role, Role::Leader);    // patch m
}

TEST(ParseSerial, GripperLeaderRightProductionLine) {
    auto p = d::parse_serial("TCGU01A24A0002m");
    EXPECT_TRUE(p.valid);
    EXPECT_EQ(p.line, 'A');             // production
    ASSERT_TRUE(p.side.has_value());
    EXPECT_EQ(*p.side, Side::Right);    // seq ends 2 (even)
    EXPECT_EQ(p.role, Role::Leader);
}

TEST(ParseSerial, GripperFollowerLeft) {
    auto p = d::parse_serial("TCGU01A24Z0003s");
    EXPECT_TRUE(p.valid);
    ASSERT_TRUE(p.side.has_value());
    EXPECT_EQ(*p.side, Side::Left);     // seq ends 3 (odd)
    EXPECT_EQ(p.role, Role::Follower);  // patch s
}

TEST(ParseSerial, GripperFollowerRight) {
    auto p = d::parse_serial("TCGU01A24A0004s");
    EXPECT_TRUE(p.valid);
    ASSERT_TRUE(p.side.has_value());
    EXPECT_EQ(*p.side, Side::Right);    // seq ends 4 (even)
    EXPECT_EQ(p.role, Role::Follower);
}

TEST(ParseSerial, SeqEndingZeroIsRight) {
    auto p = d::parse_serial("TCGU01A24Z0010m");
    EXPECT_TRUE(p.valid);
    ASSERT_TRUE(p.side.has_value());
    EXPECT_EQ(*p.side, Side::Right);    // 0 is even
}

// ---- Full grammar: visuotactile sensors (GSPS…, no patch) -----------------

TEST(ParseSerial, SensorLeftNoRole) {
    auto p = d::parse_serial("GSPS01A24Z0001");
    EXPECT_TRUE(p.valid);
    EXPECT_EQ(p.product, "GSPS01");
    ASSERT_TRUE(p.side.has_value());
    EXPECT_EQ(*p.side, Side::Left);
    EXPECT_EQ(p.role, Role::Unknown);   // sensors carry no m/s
}

TEST(ParseSerial, SensorRightNoRole) {
    auto p = d::parse_serial("GSPS01A24A0002");
    EXPECT_TRUE(p.valid);
    ASSERT_TRUE(p.side.has_value());
    EXPECT_EQ(*p.side, Side::Right);
    EXPECT_EQ(p.role, Role::Unknown);
}

// ---- Graceful degradation: legacy / empty / loose inputs ------------------

TEST(ParseSerial, LegacySnRecoversSideOnly) {
    auto p = d::parse_serial("SN000002");
    EXPECT_FALSE(p.valid);              // not the TacCap grammar
    ASSERT_TRUE(p.side.has_value());
    EXPECT_EQ(*p.side, Side::Right);    // last digit 2 (even)
    EXPECT_EQ(p.role, Role::Unknown);
}

TEST(ParseSerial, EmptyIsUnknownNoSide) {
    auto p = d::parse_serial("");
    EXPECT_FALSE(p.valid);
    EXPECT_FALSE(p.side.has_value());
    EXPECT_EQ(p.role, Role::Unknown);
}

TEST(ParseSerial, LooseTrailingPatchStillGivesRole) {
    // Not full grammar, but a trailing m/s should still surface the role,
    // and the last digit the side.
    auto p = d::parse_serial("weird-0007s");
    EXPECT_FALSE(p.valid);
    ASSERT_TRUE(p.side.has_value());
    EXPECT_EQ(*p.side, Side::Left);     // 7 odd
    EXPECT_EQ(p.role, Role::Follower);
}

// ---- side_from_serial back-compat + to_string -----------------------------

TEST(SideFromSerial, MatchesParseSideForNewScheme) {
    // The legacy last-digit helper must still agree with the new parser for
    // the new format (patch suffix is a non-digit, so it's skipped).
    EXPECT_EQ(d::side_from_serial("TCGU01A24Z0001m"), Side::Left);
    EXPECT_EQ(d::side_from_serial("TCGU01A24A0002m"), Side::Right);
    EXPECT_EQ(d::side_from_serial("GSPS01A24Z0003"),  Side::Left);
    EXPECT_FALSE(d::side_from_serial("nodigits").has_value());
}

TEST(ToString, RoleAndSide) {
    EXPECT_STREQ(d::to_string(Role::Leader), "Leader");
    EXPECT_STREQ(d::to_string(Role::Follower), "Follower");
    EXPECT_STREQ(d::to_string(Role::Unknown), "Unknown");
    EXPECT_STREQ(d::to_string(Side::Left), "Left");
    EXPECT_STREQ(d::to_string(Side::Right), "Right");
}
