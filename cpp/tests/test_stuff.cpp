// Copyright (c) 2026 XenseRobotics Co., Ltd. — Apache-2.0
//
// Byte-stuffing helpers are mirrored from firmware but currently not used
// in the wire path. Test them so that if the protocol later flips them on,
// we already know they round-trip cleanly.

#include <gtest/gtest.h>
#include <taccap/bus/frame.hpp>

#include <cstring>

namespace tb = xense::taccap::bus;

TEST(Stuff, EscapesOnlyReservedBytes) {
    const uint8_t in[] = {0x01, 0xAA, 0x02, 0x55, 0x03, 0x7D, 0x04};
    auto stuffed = tb::stuff_data(in, sizeof(in));

    // 4 plain bytes + 3 reserved-byte escapes (each becomes 2 bytes) = 10
    EXPECT_EQ(stuffed.size(), 10u);

    // Verify pattern: every reserved byte is replaced by ESCAPE + (b ^ 0x20).
    auto unstuffed = tb::unstuff_data(stuffed.data(), stuffed.size());
    EXPECT_EQ(unstuffed.size(), sizeof(in));
    EXPECT_EQ(0, std::memcmp(unstuffed.data(), in, sizeof(in)));
}

TEST(Stuff, RoundtripsAll256ByteValues) {
    std::vector<uint8_t> in(256);
    for (std::size_t i = 0; i < 256; ++i) in[i] = static_cast<uint8_t>(i);

    auto stuffed = tb::stuff_data(in.data(), in.size());
    // Three reserved bytes (0xAA, 0x55, 0x7D) get escaped → +3.
    EXPECT_EQ(stuffed.size(), 259u);

    auto out = tb::unstuff_data(stuffed.data(), stuffed.size());
    ASSERT_EQ(out.size(), in.size());
    EXPECT_EQ(0, std::memcmp(out.data(), in.data(), in.size()));
}

TEST(Stuff, RoundtripsConsecutiveReservedBytes) {
    const uint8_t in[] = {0xAA, 0xAA, 0x55, 0x55, 0x7D, 0x7D};
    auto stuffed = tb::stuff_data(in, sizeof(in));
    EXPECT_EQ(stuffed.size(), 12u);  // each reserved byte → 2 bytes

    auto out = tb::unstuff_data(stuffed.data(), stuffed.size());
    EXPECT_EQ(out.size(), sizeof(in));
    EXPECT_EQ(0, std::memcmp(out.data(), in, sizeof(in)));
}

TEST(Stuff, EmptyInputProducesEmpty) {
    auto stuffed = tb::stuff_data(nullptr, 0);
    auto unstuffed = tb::unstuff_data(nullptr, 0);
    EXPECT_TRUE(stuffed.empty());
    EXPECT_TRUE(unstuffed.empty());
}
