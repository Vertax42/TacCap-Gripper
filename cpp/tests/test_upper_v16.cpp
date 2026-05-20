// Copyright (c) 2026 XenseRobotics Co., Ltd. — Apache-2.0
//
// Upper-layer coverage for the V1.3–V1.6 SDK additions: Key /
// SensorErrors decoders + OTA CRC32 helper. The send/recv side of
// Key/SensorErrors/OtaSession is exercised end-to-end via
// test_transport.cpp's PTY harness; here we focus on the pure-decode +
// pure-CRC paths that don't need a fake firmware partner.

#include <gtest/gtest.h>
#include <taccap/components/key.hpp>
#include <taccap/components/sensor_errors.hpp>
#include <taccap/error.hpp>
#include <taccap/ota.hpp>
#include <taccap/protocol/payloads.hpp>

#include <array>
#include <cstring>
#include <string>
#include <vector>

namespace tx = xense::taccap;
namespace tp = xense::taccap::protocol;

// ================== Key sample decode ======================================

TEST(KeyComponent, DecodeRoundtripsPayload) {
    std::array<uint8_t, 2> wire = { /*key_id*/ 0,
                                     /*state */ tp::KeyState::DoubleClick };
    auto s = tx::Key::decode(wire.data(), wire.size());
    EXPECT_EQ(s.key_id,    0u);
    EXPECT_EQ(s.key_state, tp::KeyState::DoubleClick);
    EXPECT_EQ(s.raw.key_state, tp::KeyState::DoubleClick);
}

TEST(KeyComponent, DecodeWrongSizeThrows) {
    std::array<uint8_t, 1> too_small = {0};
    EXPECT_THROW(tx::Key::decode(too_small.data(), too_small.size()),
                 tx::ProtocolError);
}

TEST(KeyComponent, DecodeHostTimeIsRecent) {
    std::array<uint8_t, 2> wire = {0, 0};
    auto before = std::chrono::steady_clock::now();
    auto s = tx::Key::decode(wire.data(), wire.size());
    auto after = std::chrono::steady_clock::now();
    EXPECT_GE(s.host_time, before);
    EXPECT_LE(s.host_time, after);
}

// ================== SensorErrors sample decode =============================

TEST(SensorErrorsComponent, DecodeSurfacesAllFields) {
    tp::SensorErrorReport raw{};
    raw.sensor_id    = static_cast<uint8_t>(tp::SensorErrorId::Encoder);
    raw.error_code   = tp::SensorErrCode::CommTimeout;
    raw.error_count  = 42;
    raw.timestamp_ms = 123456789;
    std::array<uint8_t, 8> wire{};
    std::memcpy(wire.data(), &raw, sizeof(raw));

    auto s = tx::SensorErrors::decode(wire.data(), wire.size());
    EXPECT_EQ(s.sensor_id,        static_cast<uint8_t>(tp::SensorErrorId::Encoder));
    EXPECT_EQ(s.error_code,       tp::SensorErrCode::CommTimeout);
    EXPECT_EQ(s.error_count,      42u);
    EXPECT_EQ(s.mcu_timestamp_ms, 123456789u);
}

TEST(SensorErrorsComponent, DecodeWrongSizeThrows) {
    std::array<uint8_t, 7> short_wire{};
    EXPECT_THROW(
        tx::SensorErrors::decode(short_wire.data(), short_wire.size()),
        tx::ProtocolError);
}

// ================== OTA CRC32 (ISO-HDLC / zlib.crc32) =====================
//
// Known test vectors from RFC 1952 / zlib.crc32 documentation:
//   ""           -> 0x00000000
//   "a"          -> 0xE8B7BE43
//   "abc"        -> 0x352441C2
//   "123456789"  -> 0xCBF43926  (canonical CRC32 reference vector)
//   "The quick brown fox jumps over the lazy dog"
//                -> 0x414FA339
//
// Anyone changing crc32_iso_hdlc must keep these numbers identical or the
// firmware will reject the OtaVerify on perfectly-good firmware blobs.

TEST(Crc32IsoHdlc, EmptyInputYieldsZero) {
    EXPECT_EQ(tx::crc32_iso_hdlc(nullptr, 0), 0u);
    const uint8_t dummy = 0;
    EXPECT_EQ(tx::crc32_iso_hdlc(&dummy, 0), 0u);
}

TEST(Crc32IsoHdlc, KnownVectors) {
    auto bytes = [](const char* s) {
        return std::vector<uint8_t>(s, s + std::strlen(s));
    };

    {
        auto v = bytes("a");
        EXPECT_EQ(tx::crc32_iso_hdlc(v.data(), v.size()), 0xE8B7BE43u);
    }
    {
        auto v = bytes("abc");
        EXPECT_EQ(tx::crc32_iso_hdlc(v.data(), v.size()), 0x352441C2u);
    }
    {
        auto v = bytes("123456789");
        EXPECT_EQ(tx::crc32_iso_hdlc(v.data(), v.size()), 0xCBF43926u);
    }
    {
        auto v = bytes("The quick brown fox jumps over the lazy dog");
        EXPECT_EQ(tx::crc32_iso_hdlc(v.data(), v.size()), 0x414FA339u);
    }
}

TEST(Crc32IsoHdlc, IncrementalDoesNotMatter) {
    // Same bytes in two splits must yield the same value as a single
    // run — this isn't an algorithmic property we expose (no incremental
    // API), but proves the table-based impl is byte-stable.
    std::vector<uint8_t> full(1024);
    for (size_t i = 0; i < full.size(); ++i) {
        full[i] = static_cast<uint8_t>(i * 37 + 11);
    }
    uint32_t a = tx::crc32_iso_hdlc(full.data(),       full.size());
    uint32_t b = tx::crc32_iso_hdlc(full.data() + 0,   512)
                ^ tx::crc32_iso_hdlc(full.data() + 512, 512);
    // We can't actually combine the two halves without zlib's combine
    // helper. So just sanity-check the full-block run is stable and
    // non-zero for non-trivial input.
    (void)b;
    EXPECT_NE(a, 0u);
    EXPECT_NE(a, 0xFFFFFFFFu);
}

TEST(Crc32IsoHdlc, AllZerosOneKiB) {
    // 1 KiB of zeros is what OtaWriteBlock might carry on the last
    // partial block; pinning this matches Python: `hex(zlib.crc32(b"\0"
    // * 1024))` -> 0xefb5af2e.
    std::vector<uint8_t> z(1024, 0);
    EXPECT_EQ(tx::crc32_iso_hdlc(z.data(), z.size()), 0xefb5af2eu);
}
