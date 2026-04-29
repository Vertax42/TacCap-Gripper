// Copyright (c) 2026 XenseRobotics Co., Ltd. — Apache-2.0
//
// Component class tests. Most coverage is for the unit-conversion paths
// (decode static method) so we don't need a live transport. End-to-end
// flow is exercised in test_transport.cpp + the real-hardware demo.

#include <gtest/gtest.h>
#include <taccap/components/imu.hpp>
#include <taccap/components/encoder.hpp>
#include <taccap/protocol/payloads.hpp>

#include <cmath>
#include <cstring>

namespace tp = xense::taccap::protocol;

namespace {

template <typename T>
std::vector<uint8_t> as_bytes(const T& v) {
    std::vector<uint8_t> out(sizeof(T));
    std::memcpy(out.data(), &v, sizeof(T));
    return out;
}

}  // namespace

TEST(ImuDecode, ConvertsFixedPointToSiUnits) {
    tp::ImuData raw{};
    raw.timestamp_us = 12345678;
    raw.valid_flag   = tp::ImuValid::Accel | tp::ImuValid::Gyro |
                       tp::ImuValid::Mag   | tp::ImuValid::Temp;
    raw.accel_x = 1000;     // 1000 mg = 1 g = 9.80665 m/s²
    raw.accel_y =  500;     // 0.5 g
    raw.accel_z =    0;
    raw.gyro_x  = 18000;    // 180 dps = π rad/s
    raw.gyro_y  =     0;
    raw.gyro_z  =  -100;    // -1 dps
    raw.mag_x   =  5000;    // 50 µT
    raw.mag_y   = -3000;    // -30 µT
    raw.mag_z   =  2500;    // 25 µT
    raw.temperature = 3625; // 36.25 °C
    raw.seq     = 42;

    auto bytes = as_bytes(raw);
    auto s = xense::taccap::IMU::decode(bytes.data(), bytes.size());

    EXPECT_EQ(s.mcu_timestamp_us, 12345678u);
    EXPECT_EQ(s.valid_flag,       0x0Fu);
    EXPECT_EQ(s.seq,              42u);

    EXPECT_NEAR(s.accel_mps2[0], 9.80665f, 1e-4);
    EXPECT_NEAR(s.accel_mps2[1], 4.903325f, 1e-4);
    EXPECT_NEAR(s.accel_mps2[2], 0.0f,      1e-6);

    EXPECT_NEAR(s.gyro_radps[0], static_cast<float>(M_PI), 1e-4);
    EXPECT_NEAR(s.gyro_radps[1], 0.0f, 1e-6);
    EXPECT_NEAR(s.gyro_radps[2], static_cast<float>(-M_PI / 180.0f), 1e-4);

    EXPECT_NEAR(s.mag_uT[0],  50.0f, 1e-4);
    EXPECT_NEAR(s.mag_uT[1], -30.0f, 1e-4);
    EXPECT_NEAR(s.mag_uT[2],  25.0f, 1e-4);

    EXPECT_NEAR(s.temperature_c, 36.25f, 1e-4);

    // raw is preserved for callers that want the fixed-point fields.
    EXPECT_EQ(s.raw.accel_x, 1000);
    EXPECT_EQ(s.raw.gyro_x, 18000);
}

TEST(ImuDecode, RejectsWrongSize) {
    std::vector<uint8_t> short_(27, 0);
    EXPECT_THROW(xense::taccap::IMU::decode(short_.data(), short_.size()),
                 xense::taccap::ProtocolError);
}

TEST(EncoderDecode, RoundtripsPositionAndVelocity) {
    tp::EncoderData raw{};
    raw.timestamp_us  = 999;
    raw.position_rad  = 4.363409f;
    raw.velocity_rad_s = 1.5f;
    raw.status        = tp::EncoderStatusBit::Ok;
    raw.seq           = 5287;

    auto bytes = as_bytes(raw);
    auto s = xense::taccap::Encoder::decode(bytes.data(), bytes.size());

    EXPECT_EQ(s.mcu_timestamp_us, 999u);
    EXPECT_FLOAT_EQ(s.position_rad,    4.363409f);
    EXPECT_FLOAT_EQ(s.velocity_rad_s,  1.5f);
    EXPECT_EQ(s.status,                0u);
    EXPECT_EQ(s.seq,                   5287u);
}

TEST(EncoderDecode, RejectsWrongSize) {
    std::vector<uint8_t> short_(15, 0);
    EXPECT_THROW(xense::taccap::Encoder::decode(short_.data(), short_.size()),
                 xense::taccap::ProtocolError);
}

TEST(ImuDecode, FlagBitsRecognisedIndividually) {
    tp::ImuData raw{};
    raw.valid_flag = tp::ImuValid::Accel;        // accel only
    auto bytes = as_bytes(raw);
    auto s = xense::taccap::IMU::decode(bytes.data(), bytes.size());
    EXPECT_TRUE(s.valid_flag & tp::ImuValid::Accel);
    EXPECT_FALSE(s.valid_flag & tp::ImuValid::Gyro);
    EXPECT_FALSE(s.valid_flag & tp::ImuValid::Mag);
    EXPECT_FALSE(s.valid_flag & tp::ImuValid::Temp);
}
