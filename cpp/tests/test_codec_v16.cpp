// Copyright (c) 2026 XenseRobotics Co., Ltd. — Apache-2.0
//
// Coverage for the V1.3–V1.6 protocol surface that was mirrored after
// the firmware repo went from V1.2 → V1.6. These tests pin the wire
// layout we mirror — if firmware ever resizes or reorders one of the
// new structs, the size assertions or the byte-position checks here
// will catch it before runtime parsing desyncs.
//
// What we cover, by version:
//   V1.3  OTA — start / write_block / status (encode + decode)
//   V1.4  KeyStatus, ImuMagCal (decode + Hard/Soft Iron round-trip)
//   V1.5  CalSetPayload, CalSetAllPayload, CalGetResponse (mask bits)
//   V1.6  SensorErrorReport (decode + timestamp_ms semantics)
//   New Cmd to_string entries
//   New ErrorCode to_string entries

#include <gtest/gtest.h>
#include <taccap/error.hpp>
#include <taccap/protocol/codec.hpp>
#include <taccap/protocol/commands.hpp>
#include <taccap/protocol/payloads.hpp>

#include <array>
#include <cstring>
#include <vector>

namespace tp = xense::taccap::protocol;

// =================== Cmd / ErrorCode to_string coverage ====================

TEST(CmdToString, CoversV13ToV16Additions) {
    using C = tp::Cmd;
    EXPECT_STREQ(tp::to_string(C::KeyStatus),         "KeyStatus");
    EXPECT_STREQ(tp::to_string(C::SetImuCal),         "SetImuCal");
    EXPECT_STREQ(tp::to_string(C::SetImuMagCal),      "SetImuMagCal");
    EXPECT_STREQ(tp::to_string(C::SetCalResult),      "SetCalResult");
    EXPECT_STREQ(tp::to_string(C::SetAllCalResult),   "SetAllCalResult");
    EXPECT_STREQ(tp::to_string(C::GetCalResult),      "GetCalResult");
    EXPECT_STREQ(tp::to_string(C::SensorErrorReport), "SensorErrorReport");
    EXPECT_STREQ(tp::to_string(C::OtaStart),          "OtaStart");
    EXPECT_STREQ(tp::to_string(C::OtaWriteBlock),     "OtaWriteBlock");
    EXPECT_STREQ(tp::to_string(C::OtaVerify),         "OtaVerify");
    EXPECT_STREQ(tp::to_string(C::OtaApply),          "OtaApply");
    EXPECT_STREQ(tp::to_string(C::OtaAbort),          "OtaAbort");
    EXPECT_STREQ(tp::to_string(C::OtaGetStatus),      "OtaGetStatus");
}

TEST(CmdToString, AllValuesProduceNonEmptyString) {
    // Every Cmd enumerator must hit a switch case (no "Cmd?" fallback).
    for (uint8_t raw : {
            0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
            0x10, 0x11, 0x12, 0x13, 0x14, 0x15,
            0x20, 0x21, 0x22, 0x23, 0x24, 0x25, 0x26, 0x27, 0x28, 0x29, 0x2A,
            0x30, 0x31, 0x32, 0x40, 0x41, 0x42, 0x43, 0x50,
            0x60, 0x61, 0x62, 0x63, 0x64, 0x65,
            0x70, 0x71, 0x72, 0x73, 0x74, 0x75 }) {
        const char* s = tp::to_string(static_cast<tp::Cmd>(raw));
        ASSERT_NE(s, nullptr);
        EXPECT_STRNE(s, "Cmd?")
            << "Unrecognised cmd byte 0x" << std::hex << +raw;
    }
}

TEST(ErrorCodeToString, CoversOtaBand) {
    using E = tp::ErrorCode;
    EXPECT_STREQ(tp::to_string(E::OtaBusy),       "OtaBusy");
    EXPECT_STREQ(tp::to_string(E::OtaNotStarted), "OtaNotStarted");
    EXPECT_STREQ(tp::to_string(E::OtaOffsetErr),  "OtaOffsetErr");
    EXPECT_STREQ(tp::to_string(E::OtaFlashErr),   "OtaFlashErr");
    EXPECT_STREQ(tp::to_string(E::OtaVerifyFail), "OtaVerifyFail");
    EXPECT_STREQ(tp::to_string(E::OtaSizeExceed), "OtaSizeExceed");
}

// ================== V1.4: KeyStatus / KeyState =============================

TEST(KeyStatusDecode, RoundtripsAllStates) {
    for (uint8_t state : {
            tp::KeyState::SingleClickDown,
            tp::KeyState::SingleClickUp,
            tp::KeyState::DoubleClick,
            tp::KeyState::LongPressDown,
            tp::KeyState::LongPressUp}) {
        std::array<uint8_t, 2> wire = { /*key_id*/ 0, state };
        auto p = tp::decode_key_status(wire.data(), wire.size());
        EXPECT_EQ(p.key_id, 0u);
        EXPECT_EQ(p.key_state, state);
    }
}

TEST(KeyStatusDecode, RejectsWrongSize) {
    std::array<uint8_t, 3> three = {0, 0, 0};
    EXPECT_THROW(tp::decode_key_status(three.data(), three.size()),
                 xense::taccap::ProtocolError);

    std::array<uint8_t, 1> one = {0};
    EXPECT_THROW(tp::decode_key_status(one.data(), one.size()),
                 xense::taccap::ProtocolError);
}

// ================== V1.4: ImuMagCal (hard/soft iron) =======================

TEST(ImuMagCalCodec, EncodeDecodeRoundtripsHardAndSoftIron) {
    tp::ImuMagCal in{};
    in.hard_iron[0] = 1.5f; in.hard_iron[1] = -2.25f; in.hard_iron[2] = 0.75f;
    for (int r = 0; r < 3; ++r)
        for (int c = 0; c < 3; ++c)
            in.soft_iron[r][c] = static_cast<float>(r * 3 + c) + 0.5f;

    auto wire = tp::encode(in);
    EXPECT_EQ(wire.size(), 48u);

    auto out = tp::decode_imu_mag_cal(wire.data(), wire.size());
    EXPECT_FLOAT_EQ(out.hard_iron[0],  1.5f);
    EXPECT_FLOAT_EQ(out.hard_iron[1], -2.25f);
    EXPECT_FLOAT_EQ(out.hard_iron[2],  0.75f);
    for (int r = 0; r < 3; ++r)
        for (int c = 0; c < 3; ++c)
            EXPECT_FLOAT_EQ(out.soft_iron[r][c],
                            static_cast<float>(r * 3 + c) + 0.5f);
}

TEST(ImuMagCalCodec, WireLayoutIsHardThenSoftRowMajor) {
    // Hard-Iron 3 floats at offset 0; Soft-Iron 9 floats at offset 12,
    // row-major. Zero soft-iron and write a unique element at each
    // soft_iron[r][c], then read the bytes at the expected offset.
    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) {
            tp::ImuMagCal in{};
            in.soft_iron[r][c] = 42.0f;
            auto wire = tp::encode(in);
            float seen = 0.0f;
            std::memcpy(&seen, &wire[12 + (r * 3 + c) * 4], 4);
            EXPECT_FLOAT_EQ(seen, 42.0f)
                << "soft_iron[" << r << "][" << c << "] at byte offset "
                << (12 + (r * 3 + c) * 4);
        }
    }
}

// ================== V1.5: CalResult mask round-trips =======================

TEST(CalCodec, CalSetPayloadRoundtrip) {
    tp::CalSetPayload in{};
    in.sensor_id = static_cast<uint8_t>(tp::CalSensorId::ImuMag);
    in.result    = 1;
    auto wire = tp::encode(in);
    EXPECT_EQ(wire.size(), 2u);
    EXPECT_EQ(wire[0], 1u);  // ImuMag = 1
    EXPECT_EQ(wire[1], 1u);
}

TEST(CalCodec, CalSetAllPayloadMaskBitsMatchEnum) {
    tp::CalSetAllPayload in{};
    in.mask = tp::CalMask::Imu | tp::CalMask::ImuMag |
              tp::CalMask::Encoder | tp::CalMask::Camera2;
    auto wire = tp::encode(in);
    EXPECT_EQ(wire.size(), 2u);
    uint16_t mask_wire = static_cast<uint16_t>(wire[0]) |
                         (static_cast<uint16_t>(wire[1]) << 8);
    EXPECT_EQ(mask_wire & 0x0007u, 0x0007u);     // bits 0,1,2 set
    EXPECT_NE(mask_wire & (1u << 7), 0u);        // Camera2 bit = 7
    EXPECT_EQ(mask_wire & (1u << 5), 0u);        // Eskin2 bit not set
}

TEST(CalCodec, CalGetResponseDecodes) {
    std::array<uint8_t, 2> wire = { 0x07, 0x01 };  // 0x0107 LE
    auto out = tp::decode_cal_get(wire.data(), wire.size());
    EXPECT_EQ(out.mask, 0x0107u);
}

TEST(CalCodec, CalGetResponseRejectsWrongSize) {
    std::array<uint8_t, 1> one = {0};
    EXPECT_THROW(tp::decode_cal_get(one.data(), one.size()),
                 xense::taccap::ProtocolError);
}

// ================== V1.6: SensorErrorReport ================================

TEST(SensorErrorDecode, Roundtrip) {
    tp::SensorErrorReport in{};
    in.sensor_id    = static_cast<uint8_t>(tp::SensorErrorId::Motor);
    in.error_code   = tp::SensorErrCode::CommTimeout;
    in.error_count  = 0xABCD;
    in.timestamp_ms = 0x12345678;
    std::array<uint8_t, 8> wire{};
    std::memcpy(wire.data(), &in, sizeof(in));

    auto out = tp::decode_sensor_error(wire.data(), wire.size());
    EXPECT_EQ(out.sensor_id,    static_cast<uint8_t>(tp::SensorErrorId::Motor));
    EXPECT_EQ(out.error_code,   tp::SensorErrCode::CommTimeout);
    EXPECT_EQ(out.error_count,  0xABCDu);
    EXPECT_EQ(out.timestamp_ms, 0x12345678u);
}

TEST(SensorErrorDecode, RejectsWrongSize) {
    std::array<uint8_t, 7> short_wire{};
    EXPECT_THROW(tp::decode_sensor_error(short_wire.data(), short_wire.size()),
                 xense::taccap::ProtocolError);
}

TEST(SensorErrorIds, MatchFirmwareValues) {
    // sensor_error_id_t in firmware: IMU=0, IMU_MAG=1, ENCODER=2,
    // ESKIN1=3, ESKIN2=4, MOTOR=5. Pin them all.
    EXPECT_EQ(static_cast<uint8_t>(tp::SensorErrorId::Imu),     0);
    EXPECT_EQ(static_cast<uint8_t>(tp::SensorErrorId::ImuMag),  1);
    EXPECT_EQ(static_cast<uint8_t>(tp::SensorErrorId::Encoder), 2);
    EXPECT_EQ(static_cast<uint8_t>(tp::SensorErrorId::Eskin1),  3);
    EXPECT_EQ(static_cast<uint8_t>(tp::SensorErrorId::Eskin2),  4);
    EXPECT_EQ(static_cast<uint8_t>(tp::SensorErrorId::Motor),   5);
}

// ================== V1.3 OTA: Start / WriteBlock / Status ==================

TEST(OtaCodec, OtaStartEncodeMatchesWireLayout) {
    tp::OtaStart in{};
    in.firmware_size  = 123456;
    in.firmware_crc32 = 0xDEADBEEFu;
    in.target_major   = 1;
    in.target_minor   = 6;
    in.target_patch   = 2;
    in.target_build   = 99;

    auto wire = tp::encode(in);
    EXPECT_EQ(wire.size(), 12u);

    uint32_t sz = 0, crc = 0;
    std::memcpy(&sz,  &wire[0], 4);
    std::memcpy(&crc, &wire[4], 4);
    EXPECT_EQ(sz, 123456u);
    EXPECT_EQ(crc, 0xDEADBEEFu);
    EXPECT_EQ(wire[8],  1u);
    EXPECT_EQ(wire[9],  6u);
    EXPECT_EQ(wire[10], 2u);
    EXPECT_EQ(wire[11], 99u);
}

TEST(OtaCodec, OtaWriteBlockHeaderPlusPayload) {
    std::array<uint8_t, 16> payload{
        0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF, 0x00, 0x11,
        0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88, 0x99,
    };
    auto wire = tp::encode_ota_write_block(
        /*offset*/ 4096, payload.data(), static_cast<uint16_t>(payload.size()));
    ASSERT_EQ(wire.size(), 6 + payload.size());

    uint32_t off = 0;
    uint16_t len = 0;
    std::memcpy(&off, &wire[0], 4);
    std::memcpy(&len, &wire[4], 2);
    EXPECT_EQ(off, 4096u);
    EXPECT_EQ(len, payload.size());
    EXPECT_EQ(std::memcmp(&wire[6], payload.data(), payload.size()), 0);
}

TEST(OtaCodec, OtaWriteBlockHandlesEmptyTail) {
    auto wire = tp::encode_ota_write_block(0, nullptr, 0);
    EXPECT_EQ(wire.size(), 6u);
    uint32_t off = 0;
    uint16_t len = 0;
    std::memcpy(&off, &wire[0], 4);
    std::memcpy(&len, &wire[4], 2);
    EXPECT_EQ(off, 0u);
    EXPECT_EQ(len, 0u);
}

TEST(OtaCodec, OtaWriteBlockRespectsMaxBlockSize) {
    std::vector<uint8_t> big(tp::OTA_BLOCK_SIZE, 0x5A);
    auto wire = tp::encode_ota_write_block(
        0x12340000, big.data(), tp::OTA_BLOCK_SIZE);
    EXPECT_EQ(wire.size(),
              static_cast<std::size_t>(6) + tp::OTA_BLOCK_SIZE);
}

TEST(OtaCodec, OtaStatusDecodeRoundtrip) {
    tp::OtaStatus in{};
    in.state         = static_cast<uint8_t>(tp::OtaState::Receiving);
    in.error_code    = 0;
    in.bytes_written = 250000;
    in.progress_ppt  = 547;   // 54.7%
    std::array<uint8_t, 8> wire{};
    std::memcpy(wire.data(), &in, sizeof(in));

    auto out = tp::decode_ota_status(wire.data(), wire.size());
    EXPECT_EQ(out.state,         static_cast<uint8_t>(tp::OtaState::Receiving));
    EXPECT_EQ(out.error_code,    0u);
    EXPECT_EQ(out.bytes_written, 250000u);
    EXPECT_EQ(out.progress_ppt,  547u);
}

TEST(OtaCodec, OtaStatusRejectsWrongSize) {
    std::array<uint8_t, 7> short_wire{};
    EXPECT_THROW(tp::decode_ota_status(short_wire.data(), short_wire.size()),
                 xense::taccap::ProtocolError);
}

TEST(OtaState, EnumeratorsMatchFirmwareIntegers) {
    EXPECT_EQ(static_cast<uint8_t>(tp::OtaState::Idle),      0);
    EXPECT_EQ(static_cast<uint8_t>(tp::OtaState::Started),   1);
    EXPECT_EQ(static_cast<uint8_t>(tp::OtaState::Receiving), 2);
    EXPECT_EQ(static_cast<uint8_t>(tp::OtaState::Verified),  3);
    EXPECT_EQ(static_cast<uint8_t>(tp::OtaState::Applying),  4);
    EXPECT_EQ(static_cast<uint8_t>(tp::OtaState::Error),     5);
}

// ================== Layout pin — size-invariant assertions ================

TEST(PayloadSizes, MatchFirmwareV16) {
    EXPECT_EQ(sizeof(tp::KeyStatusPayload),    2u);
    EXPECT_EQ(sizeof(tp::ImuMagCal),           48u);
    EXPECT_EQ(sizeof(tp::CalSetPayload),       2u);
    EXPECT_EQ(sizeof(tp::CalSetAllPayload),    2u);
    EXPECT_EQ(sizeof(tp::CalGetResponse),      2u);
    EXPECT_EQ(sizeof(tp::SensorErrorReport),   8u);
    EXPECT_EQ(sizeof(tp::OtaStart),            12u);
    EXPECT_EQ(sizeof(tp::OtaWriteBlockHeader), 6u);
    EXPECT_EQ(sizeof(tp::OtaStatus),           8u);
}

TEST(PayloadConstants, OtaBlockSizeMatchesFirmware) {
    // ota_driver.h #define OTA_BLOCK_SIZE (1024U)
    EXPECT_EQ(tp::OTA_BLOCK_SIZE, 1024u);
    EXPECT_EQ(tp::OTA_MAX_FW_SIZE, 456u * 1024u);
}
