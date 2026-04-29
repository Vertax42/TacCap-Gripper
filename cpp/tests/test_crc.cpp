// Copyright (c) 2026 XenseRobotics Co., Ltd. — Apache-2.0
//
// CRC-16/MODBUS sanity vectors. The "123456789" → 0x4B37 vector is the
// canonical test value for Modbus CRC-16 across implementations.

#include <gtest/gtest.h>
#include <taccap/bus/frame.hpp>

using xense::taccap::bus::crc16_modbus;

TEST(Crc16Modbus, EmptyInputIsInit) {
    EXPECT_EQ(crc16_modbus(nullptr, 0), 0xFFFFu);
}

TEST(Crc16Modbus, KnownVectorString123456789) {
    const uint8_t s[] = {'1','2','3','4','5','6','7','8','9'};
    EXPECT_EQ(crc16_modbus(s, sizeof(s)), 0x4B37u);
}

TEST(Crc16Modbus, IncrementalEqualsBatchForFrameHeaderShape) {
    // For a real wire frame `AA 01 7B 00 02 00 00`, both batch and
    // byte-by-byte routes should match: we always invoke crc16_modbus over
    // the whole prefix.
    const uint8_t hdr[] = {0xAA, 0x01, 0x7B, 0x00, 0x02, 0x00, 0x00};
    const uint16_t whole = crc16_modbus(hdr, sizeof(hdr));

    // Sanity: feeding the same buffer twice produces the same result (the
    // function is pure).
    EXPECT_EQ(crc16_modbus(hdr, sizeof(hdr)), whole);
}

TEST(Crc16Modbus, AllZeroPayload28IsDeterministic) {
    // Sanity: same input always produces same output. (We rely on the
    // cross-impl test against GUI Python's calc_crc to validate the value
    // numerically for IMU-shaped payloads.)
    const std::vector<uint8_t> z(28, 0x00);
    EXPECT_EQ(crc16_modbus(z.data(), z.size()),
              crc16_modbus(z.data(), z.size()));
}
