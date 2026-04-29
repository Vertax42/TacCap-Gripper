// Copyright (c) 2026 XenseRobotics Co., Ltd. — Apache-2.0

#include <gtest/gtest.h>
#include <taccap/protocol/codec.hpp>
#include <taccap/protocol/commands.hpp>
#include <taccap/error.hpp>

#include <cstring>

namespace tp = xense::taccap::protocol;

TEST(Codec, MotorPosCtrlRoundtrip) {
    tp::MotorPosCtrl in{1.5f, 2.5f, 3.5f};
    auto bytes = tp::encode(in);
    EXPECT_EQ(bytes.size(), 12u);

    // Decode by reinterpret since codec.hpp doesn't expose decode_motor_pos_ctrl
    // (unlikely to be needed — host only sends, never receives this struct).
    tp::MotorPosCtrl out;
    std::memcpy(&out, bytes.data(), sizeof(out));
    EXPECT_FLOAT_EQ(out.target_pos, 1.5f);
    EXPECT_FLOAT_EQ(out.max_vel,    2.5f);
    EXPECT_FLOAT_EQ(out.max_torque, 3.5f);
}

TEST(Codec, ImuConfigEncodeDecodeKeeps10Bytes) {
    tp::ImuConfig in{};
    in.sample_rate   = 1000;
    in.odr           = 200;
    in.accel_range   = 2;
    in.gyro_range    = 1;
    in.mag_range     = 0;
    in.filter_enable = 1;
    in.filter_cutoff = 50;

    auto bytes = tp::encode(in);
    ASSERT_EQ(bytes.size(), 10u);

    auto out = tp::decode_imu_config(bytes.data(), bytes.size());
    EXPECT_EQ(out.sample_rate,   1000);
    EXPECT_EQ(out.odr,           200);
    EXPECT_EQ(out.accel_range,   2);
    EXPECT_EQ(out.gyro_range,    1);
    EXPECT_EQ(out.mag_range,     0);
    EXPECT_EQ(out.filter_enable, 1);
    EXPECT_EQ(out.filter_cutoff, 50);
}

TEST(Codec, StreamConfigRoundtrip) {
    tp::StreamConfig in{};
    in.source_mask  = tp::StreamSrc::Imu | tp::StreamSrc::Encoder;
    in.mode         = static_cast<uint8_t>(tp::StreamMode::Separate);
    in.imu_rate     = 200;
    in.encoder_rate = 200;
    in.eskin_rate   = 100;
    in.motor_rate   = 100;
    in.output_iface = static_cast<uint8_t>(tp::StreamInterface::Uart);

    auto bytes = tp::encode(in);
    ASSERT_EQ(bytes.size(), 12u);

    auto out = tp::decode_stream_config(bytes.data(), bytes.size());
    EXPECT_EQ(out.source_mask,  in.source_mask);
    EXPECT_EQ(out.mode,         in.mode);
    EXPECT_EQ(out.imu_rate,     in.imu_rate);
    EXPECT_EQ(out.encoder_rate, in.encoder_rate);
    EXPECT_EQ(out.eskin_rate,   in.eskin_rate);
    EXPECT_EQ(out.motor_rate,   in.motor_rate);
    EXPECT_EQ(out.output_iface, in.output_iface);
}

TEST(Codec, ImuDataDecodeRejectsWrongLength) {
    std::vector<uint8_t> too_small(27, 0);
    EXPECT_THROW(tp::decode_imu(too_small.data(), too_small.size()),
                 xense::taccap::ProtocolError);
    std::vector<uint8_t> too_big(29, 0);
    EXPECT_THROW(tp::decode_imu(too_big.data(), too_big.size()),
                 xense::taccap::ProtocolError);
}

TEST(Codec, AckPayloadRoundtrip) {
    tp::AckPayload in{};
    in.ack_seq     = 42;
    in.error_code  = static_cast<uint8_t>(tp::ErrorCode::CrcError);
    in.retry_count = 1234;

    std::vector<uint8_t> bytes(sizeof(tp::AckPayload));
    std::memcpy(bytes.data(), &in, sizeof(in));

    auto out = tp::decode_ack(bytes.data(), bytes.size());
    EXPECT_EQ(out.ack_seq,     42);
    EXPECT_EQ(out.error_code,  static_cast<uint8_t>(tp::ErrorCode::CrcError));
    EXPECT_EQ(out.retry_count, 1234);
}

TEST(Codec, EskinDecodesAdcGrid) {
    constexpr uint8_t rows = 4, cols = 4;
    constexpr std::size_t cells = rows * cols;
    constexpr std::size_t total = sizeof(tp::EskinHeader) + cells * 2;

    std::vector<uint8_t> bytes(total, 0);
    tp::EskinHeader hdr{};
    hdr.timestamp_us = 12345;
    hdr.rows = rows;
    hdr.cols = cols;
    hdr.type = static_cast<uint8_t>(tp::EskinOutputType::Adc);
    hdr.seq  = 7;
    std::memcpy(bytes.data(), &hdr, sizeof(hdr));

    // Cell values 100..115
    for (std::size_t i = 0; i < cells; ++i) {
        const uint16_t v = static_cast<uint16_t>(100 + i);
        std::memcpy(bytes.data() + sizeof(hdr) + i * 2, &v, 2);
    }

    auto frame = tp::decode_eskin(bytes.data(), bytes.size());
    EXPECT_EQ(frame.header.timestamp_us, 12345u);
    EXPECT_EQ(frame.header.rows, rows);
    EXPECT_EQ(frame.header.cols, cols);
    ASSERT_EQ(frame.values_u16.size(), cells);
    for (std::size_t i = 0; i < cells; ++i) {
        EXPECT_EQ(frame.values_u16[i], static_cast<uint16_t>(100 + i)) << "i=" << i;
    }
    EXPECT_TRUE(frame.values_f32.empty());
}

TEST(Codec, EskinDecodesForceGrid) {
    constexpr uint8_t rows = 2, cols = 3;
    constexpr std::size_t cells = rows * cols;
    constexpr std::size_t total = sizeof(tp::EskinHeader) + cells * 4;

    std::vector<uint8_t> bytes(total, 0);
    tp::EskinHeader hdr{};
    hdr.rows = rows;
    hdr.cols = cols;
    hdr.type = static_cast<uint8_t>(tp::EskinOutputType::Force);
    std::memcpy(bytes.data(), &hdr, sizeof(hdr));

    for (std::size_t i = 0; i < cells; ++i) {
        const float v = 0.5f + static_cast<float>(i);
        std::memcpy(bytes.data() + sizeof(hdr) + i * 4, &v, 4);
    }

    auto frame = tp::decode_eskin(bytes.data(), bytes.size());
    ASSERT_EQ(frame.values_f32.size(), cells);
    for (std::size_t i = 0; i < cells; ++i) {
        EXPECT_FLOAT_EQ(frame.values_f32[i], 0.5f + static_cast<float>(i));
    }
    EXPECT_TRUE(frame.values_u16.empty());
}

TEST(Codec, EskinRejectsLengthMismatch) {
    constexpr uint8_t rows = 4, cols = 4;
    std::vector<uint8_t> bytes(sizeof(tp::EskinHeader) + rows * cols * 2 - 4, 0);
    tp::EskinHeader hdr{};
    hdr.rows = rows;
    hdr.cols = cols;
    hdr.type = static_cast<uint8_t>(tp::EskinOutputType::Adc);
    std::memcpy(bytes.data(), &hdr, sizeof(hdr));
    EXPECT_THROW(tp::decode_eskin(bytes.data(), bytes.size()),
                 xense::taccap::ProtocolError);
}

TEST(Codec, EncodeSnPadsAndTruncates) {
    auto pad = tp::encode_sn("AB");
    EXPECT_EQ(pad.size(), 17u);
    EXPECT_EQ(pad[0], 'A');
    EXPECT_EQ(pad[1], 'B');
    for (std::size_t i = 2; i < 17; ++i) EXPECT_EQ(pad[i], 0);

    // Over-long input is truncated to 16 chars (NUL terminator preserved).
    auto trunc = tp::encode_sn("0123456789ABCDEF_TOO_LONG");
    EXPECT_EQ(trunc.size(), 17u);
    EXPECT_EQ(trunc[16], 0);
    for (std::size_t i = 0; i < 16; ++i) {
        EXPECT_EQ(trunc[i], "0123456789ABCDEF"[i]);
    }
}

TEST(Codec, DecodeSnTrimsAtNul) {
    std::vector<uint8_t> bytes(17, 0);
    const char raw[] = "Hello";
    std::memcpy(bytes.data(), raw, sizeof(raw) - 1);
    EXPECT_EQ(tp::decode_sn(bytes.data(), bytes.size()), "Hello");
}
