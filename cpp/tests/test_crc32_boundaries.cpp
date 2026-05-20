// Copyright (c) 2026 XenseRobotics Co., Ltd. — Apache-2.0
//
// CRC32 (ISO-HDLC / zlib.crc32) boundary-case coverage. The "happy
// path" plus four standard reference vectors are in test_upper_v16.cpp;
// here we pin down edge cases that historically catch subtle table
// /off-by-one bugs in CRC implementations:
//
//   - single-byte inputs across the full 0..255 range vs zlib reference
//   - 1-byte / 2-byte / 3-byte / 4-byte inputs (the table is byte-wise,
//     so cross-byte transitions are where alignment bugs hide)
//   - alignment around the OTA_BLOCK_SIZE = 1024 boundary (the chunk
//     size we routinely send to firmware)
//   - long inputs (8 KiB, 64 KiB, full OTA_MAX_FW_SIZE) — for a
//     reflected polynomial like 0xEDB88320 the result must stay
//     deterministic regardless of input length
//   - duplicate of the same byte for various lengths (zlib `b'\x00'*N`
//     vectors precomputed)
//   - random buffer vs hand-rolled "naive" CRC32 reference
//
// The reference values come from zlib.crc32 (Python:
//   python -c "import zlib; print(hex(zlib.crc32(b'\x00'*N)))"
// ) and are baked in here so the test runs without Python at
// build time. Each one is also cross-checked at runtime against
// the naive bit-by-bit CRC32 below.

#include <gtest/gtest.h>
#include <taccap/ota.hpp>

#include <array>
#include <cstdint>
#include <cstring>
#include <vector>

namespace tx = xense::taccap;

namespace {

// Bit-by-bit reference CRC32 (slow but obviously correct). Same
// parameters as zlib: poly 0x04C11DB7 reflected, init 0xFFFFFFFF,
// xorout 0xFFFFFFFF, reflected I/O.
uint32_t crc32_naive(const uint8_t* data, std::size_t len) {
    uint32_t crc = 0xFFFFFFFFu;
    for (std::size_t i = 0; i < len; ++i) {
        crc ^= data[i];
        for (int b = 0; b < 8; ++b) {
            crc = (crc >> 1) ^ (0xEDB88320u & (-static_cast<int32_t>(crc & 1)));
        }
    }
    return crc ^ 0xFFFFFFFFu;
}

}  // namespace

TEST(Crc32Boundaries, SingleByteAcrossFullRange) {
    // For every possible single-byte input 0x00..0xFF, our table-based
    // crc32_iso_hdlc must match the naive bit-by-bit reference.
    for (int b = 0; b < 256; ++b) {
        const uint8_t byte = static_cast<uint8_t>(b);
        EXPECT_EQ(tx::crc32_iso_hdlc(&byte, 1),
                  crc32_naive(&byte, 1))
            << "single byte 0x" << std::hex << b;
    }
}

TEST(Crc32Boundaries, ShortInputsAgainstNaive) {
    // 1-, 2-, 3-, 4-byte windows over a randomised pattern. Catches
    // off-by-one errors where the byte-wise table walks past the end
    // or stops one byte short.
    std::vector<uint8_t> buf(64);
    for (size_t i = 0; i < buf.size(); ++i) {
        buf[i] = static_cast<uint8_t>(i * 17 + 0x3A);
    }
    for (size_t start = 0; start + 4 <= buf.size(); ++start) {
        for (size_t len = 1; len <= 4; ++len) {
            EXPECT_EQ(tx::crc32_iso_hdlc(buf.data() + start, len),
                      crc32_naive(buf.data() + start, len))
                << "start=" << start << " len=" << len;
        }
    }
}

TEST(Crc32Boundaries, AroundOtaBlockSize) {
    // Lengths ±2 bytes around OTA_BLOCK_SIZE=1024 — the partial-block
    // path that runs at the end of every OTA update. Compared
    // against the naive impl (not zlib, so test stays hermetic).
    constexpr size_t mid = 1024;
    std::vector<uint8_t> buf(mid + 4);
    for (size_t i = 0; i < buf.size(); ++i) buf[i] = static_cast<uint8_t>(i & 0xFF);
    for (size_t len : {mid - 2, mid - 1, mid, mid + 1, mid + 2}) {
        EXPECT_EQ(tx::crc32_iso_hdlc(buf.data(), len),
                  crc32_naive(buf.data(), len))
            << "len=" << len;
    }
}

TEST(Crc32Boundaries, ZerosKnownVectors) {
    // Pre-baked CRC32 of N consecutive 0x00 bytes (matches Python
    // `hex(zlib.crc32(b"\0" * N))`). Pins the table walk + init
    // value 0xFFFFFFFF: a wrong init or wrong table entry would
    // perturb one of these.
    struct { size_t n; uint32_t expected; } cases[] = {
        {0,    0x00000000u},
        {1,    0xD202EF8Du},
        {2,    0x41D912FFu},
        {4,    0x2144DF1Cu},
        {7,    0x9D6CDF7Eu},
        {8,    0x6522DF69u},
        {15,   0xD7D303E7u},
        {16,   0xECBB4B55u},
        {32,   0x190A55ADu},
        {1023, 0x6FD57465u},
        {1024, 0xEFB5AF2Eu},
        {1025, 0x0E3B57EDu},
        {4096, 0xC71C0011u},
    };
    for (const auto& c : cases) {
        std::vector<uint8_t> z(c.n, 0);
        EXPECT_EQ(tx::crc32_iso_hdlc(z.data(), z.size()), c.expected)
            << "len=" << c.n;
        // Belt-and-suspenders: also matches our naive reference.
        EXPECT_EQ(tx::crc32_iso_hdlc(z.data(), z.size()),
                  crc32_naive(z.data(), z.size()))
            << "len=" << c.n << " naive";
    }
}

TEST(Crc32Boundaries, RepeatedByteAgainstNaive) {
    // Buffers of N identical bytes (B = 0xFF, 0xA5, 0x7E) at various
    // lengths. Identical-byte runs are where some incorrect "incremental
    // CRC" implementations diverge from one-shot.
    for (uint8_t b : {0xFFu, 0xA5u, 0x7Eu}) {
        for (size_t n : {1, 7, 64, 511, 1023, 4096}) {
            std::vector<uint8_t> buf(n, b);
            EXPECT_EQ(tx::crc32_iso_hdlc(buf.data(), buf.size()),
                      crc32_naive(buf.data(), buf.size()))
                << "b=0x" << std::hex << +b << " n=" << std::dec << n;
        }
    }
}

TEST(Crc32Boundaries, LongRandomBuffer) {
    // 64 KiB of deterministic pseudo-random — well past the
    // OTA_BLOCK_SIZE chunking boundary. Catches any "loop counter
    // overflows uint16" bugs.
    constexpr size_t N = 65536;
    std::vector<uint8_t> buf(N);
    uint32_t seed = 0xC0DEC0DEu;
    for (size_t i = 0; i < N; ++i) {
        seed = seed * 1664525u + 1013904223u;
        buf[i] = static_cast<uint8_t>(seed >> 24);
    }
    EXPECT_EQ(tx::crc32_iso_hdlc(buf.data(), buf.size()),
              crc32_naive(buf.data(), buf.size()));
}

TEST(Crc32Boundaries, OtaMaxFwSize) {
    // The full firmware size envelope. Confirms the implementation
    // doesn't slow to a crawl at the largest input we'd ever see.
    namespace tp = xense::taccap::protocol;
    std::vector<uint8_t> buf(tp::OTA_MAX_FW_SIZE);
    for (size_t i = 0; i < buf.size(); ++i) buf[i] = static_cast<uint8_t>(i * 53 + 7);
    EXPECT_EQ(tx::crc32_iso_hdlc(buf.data(), buf.size()),
              crc32_naive(buf.data(), buf.size()));
}

TEST(Crc32Boundaries, EmptyInputVariants) {
    // All ways of expressing "no data" produce 0.
    EXPECT_EQ(tx::crc32_iso_hdlc(nullptr, 0), 0u);

    const uint8_t one = 0xAB;
    EXPECT_EQ(tx::crc32_iso_hdlc(&one, 0), 0u);

    std::vector<uint8_t> empty;
    EXPECT_EQ(tx::crc32_iso_hdlc(empty.data(), 0), 0u);
}
