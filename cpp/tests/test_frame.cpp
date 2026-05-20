// Copyright (c) 2026 XenseRobotics Co., Ltd. — Apache-2.0

#include <cstring>
#include <gtest/gtest.h>
#include <taccap/bus/frame.hpp>
#include <taccap/error.hpp>
#include <taccap/protocol/commands.hpp>

#include <random>

namespace tb = xense::taccap::bus;
namespace tp = xense::taccap::protocol;

namespace {

std::vector<uint8_t> random_payload(std::size_t len, uint32_t seed = 1) {
    std::mt19937 rng(seed);
    std::uniform_int_distribution<int> dist(0, 255);
    std::vector<uint8_t> out(len);
    for (auto& b : out) b = static_cast<uint8_t>(dist(rng));
    return out;
}

void expect_frame_eq(const tb::Frame& got, tp::Address addr, uint8_t seq,
                     tp::FrameType type, tp::Cmd cmd,
                     const std::vector<uint8_t>& payload) {
    EXPECT_EQ(got.addr, addr);
    EXPECT_EQ(got.seq, seq);
    EXPECT_EQ(got.type, type);
    EXPECT_EQ(got.cmd, cmd);
    EXPECT_EQ(got.payload, payload);
}

}  // namespace

TEST(Frame, PackParseRoundtripEmptyPayload) {
    auto wire = tb::pack_frame(tp::Address::PC, 7,
                               tp::FrameType::CMD_NEED_ACK,
                               tp::Cmd::GetVersion);
    EXPECT_EQ(wire.size(), tb::MIN_FRAME_LEN);
    EXPECT_EQ(wire.front(), tb::FRAME_HEAD);
    EXPECT_EQ(wire.back(),  tb::FRAME_TAIL);

    auto out = tb::try_parse_frame(wire.data(), wire.size());
    ASSERT_EQ(out.status, tb::ParseStatus::Success);
    EXPECT_EQ(out.consumed, wire.size());
    expect_frame_eq(out.frame, tp::Address::PC, 7,
                    tp::FrameType::CMD_NEED_ACK, tp::Cmd::GetVersion, {});
}

TEST(Frame, PackParseRoundtripVariousSizes) {
    const std::size_t sizes[] = {0, 1, 4, 8, 28, 256, tb::MAX_PAYLOAD_LEN};
    for (std::size_t n : sizes) {
        const auto payload = random_payload(n, /*seed=*/123u + n);
        auto wire = tb::pack_frame(tp::Address::MCU, 42,
                                   tp::FrameType::DATA, tp::Cmd::GetImu,
                                   payload);
        ASSERT_EQ(wire.size(), tb::MIN_FRAME_LEN + n);

        auto out = tb::try_parse_frame(wire.data(), wire.size());
        ASSERT_EQ(out.status, tb::ParseStatus::Success) << "size=" << n;
        EXPECT_EQ(out.consumed, wire.size());
        expect_frame_eq(out.frame, tp::Address::MCU, 42,
                        tp::FrameType::DATA, tp::Cmd::GetImu, payload);
    }
}

TEST(Frame, PackRejectsOversizedPayload) {
    std::vector<uint8_t> big(tb::MAX_PAYLOAD_LEN + 1, 0);
    EXPECT_THROW(
        tb::pack_frame(tp::Address::PC, 0, tp::FrameType::DATA,
                       tp::Cmd::GetImu, big),
        xense::taccap::ProtocolError);
}

TEST(Frame, ParseSignalsNeedMoreDataOnTruncation) {
    auto wire = tb::pack_frame(tp::Address::PC, 1,
                               tp::FrameType::CMD_NEED_ACK,
                               tp::Cmd::GetVersion);
    // Feed all but the last byte: should ask for more.
    auto out = tb::try_parse_frame(wire.data(), wire.size() - 1);
    EXPECT_EQ(out.status, tb::ParseStatus::NeedMoreData);
}

TEST(Frame, ParseResyncsOnGarbagePrefix) {
    auto wire = tb::pack_frame(tp::Address::PC, 1,
                               tp::FrameType::CMD_NO_ACK,
                               tp::Cmd::GetVersion);
    std::vector<uint8_t> noisy = {0x00, 0xFF, 0x42, 0xAA};  // ends on a HEAD lure
    noisy.insert(noisy.end(), wire.begin(), wire.end());

    auto out = tb::try_parse_frame(noisy.data(), noisy.size());
    EXPECT_EQ(out.status, tb::ParseStatus::Resync);
    // Should skip past the leading non-HEAD bytes (3 of them) so the next
    // try lands on a HEAD.
    EXPECT_EQ(out.consumed, 3u);
}

TEST(Frame, ParseResyncsOnBadCrc) {
    auto wire = tb::pack_frame(tp::Address::PC, 1,
                               tp::FrameType::CMD_NEED_ACK,
                               tp::Cmd::GetVersion);
    // Flip a CRC byte.
    wire[wire.size() - 2] ^= 0x01;

    auto out = tb::try_parse_frame(wire.data(), wire.size());
    EXPECT_EQ(out.status, tb::ParseStatus::Resync);
    EXPECT_EQ(out.consumed, 1u);  // drop one byte and rescan
}

TEST(FrameParser, EmitsSingleCompleteFrame) {
    auto wire = tb::pack_frame(tp::Address::PC, 9,
                               tp::FrameType::DATA, tp::Cmd::GetEncoder,
                               random_payload(16));
    tb::FrameParser p;
    p.feed(wire);
    EXPECT_EQ(p.pending(), 1u);

    tb::Frame f;
    ASSERT_TRUE(p.try_pop(f));
    EXPECT_EQ(f.cmd, tp::Cmd::GetEncoder);
    EXPECT_EQ(f.payload.size(), 16u);
    EXPECT_FALSE(p.try_pop(f));
}

TEST(FrameParser, EmitsMultipleFramesFromOneFeed) {
    auto a = tb::pack_frame(tp::Address::PC, 1,
                            tp::FrameType::DATA, tp::Cmd::GetImu,
                            random_payload(28, 1));
    auto b = tb::pack_frame(tp::Address::MCU, 2,
                            tp::FrameType::DATA, tp::Cmd::GetEncoder,
                            random_payload(16, 2));
    auto c = tb::pack_frame(tp::Address::PC, 3,
                            tp::FrameType::ACK, tp::Cmd::GetVersion,
                            random_payload(4, 3));

    std::vector<uint8_t> stream;
    stream.insert(stream.end(), a.begin(), a.end());
    stream.insert(stream.end(), b.begin(), b.end());
    stream.insert(stream.end(), c.begin(), c.end());

    tb::FrameParser p;
    p.feed(stream);
    EXPECT_EQ(p.pending(), 3u);

    tb::Frame f;
    ASSERT_TRUE(p.try_pop(f)); EXPECT_EQ(f.cmd, tp::Cmd::GetImu);
    ASSERT_TRUE(p.try_pop(f)); EXPECT_EQ(f.cmd, tp::Cmd::GetEncoder);
    ASSERT_TRUE(p.try_pop(f)); EXPECT_EQ(f.cmd, tp::Cmd::GetVersion);
    EXPECT_FALSE(p.try_pop(f));
}

TEST(FrameParser, HandlesByteAtATimeFeed) {
    auto wire = tb::pack_frame(tp::Address::PC, 11,
                               tp::FrameType::CMD_NEED_ACK,
                               tp::Cmd::GetEncoder,
                               random_payload(8, 99));
    tb::FrameParser p;
    for (auto b : wire) {
        p.feed(&b, 1);
    }
    EXPECT_EQ(p.pending(), 1u);

    tb::Frame f;
    ASSERT_TRUE(p.try_pop(f));
    EXPECT_EQ(f.seq, 11u);
    EXPECT_EQ(f.payload.size(), 8u);
}

TEST(FrameParser, ResyncsAcrossGarbagePlusValidFrame) {
    // Garbage layout (positions relative to the false-positive HEAD at
    // offset 4): HEAD@+0, ADDR@+1, SEQ@+2, TYPE@+3, CMD@+4, LEN-LO@+5,
    // LEN-HI@+6. Bytes 5 and 6 after the false HEAD are 0xFF/0xFF →
    // parsed length 0xFFFF, well past MAX_PAYLOAD_LEN, so the parser
    // rejects the false start and resyncs one byte at a time until it
    // finds the real frame's HEAD.
    //
    // The original garbage `{0xDE,0xAD,0xBE,0xEF,0xAA,0x42,0x55}` was
    // 7 bytes — too short to fill the false header, so the parser read
    // LEN from the start of the real frame (wire[2:4] = 0x0305 = 773),
    // which was rejected under the V1.2 MAX_PAYLOAD_LEN=502 but is
    // plausible under V1.3+'s 2294 cap, leaving the parser stuck
    // waiting for never-arriving "frame body" bytes. See the comment
    // on MAX_FRAME_LEN in frame.hpp.
    std::vector<uint8_t> garbage = {
        0xDE, 0xAD, 0xBE, 0xEF,            // junk
        /* false HEAD @+0 */ 0xAA,
        /* false ADDR @+1 */ 0x42,
        /* false SEQ  @+2 */ 0x55,
        /* false TYPE @+3 */ 0x99,
        /* false CMD  @+4 */ 0x88,
        /* LEN-LO     @+5 */ 0xFF,
        /* LEN-HI     @+6 */ 0xFF,
    };
    auto wire = tb::pack_frame(tp::Address::PC, 5,
                               tp::FrameType::DATA, tp::Cmd::GetImu,
                               random_payload(28, 5));
    std::vector<uint8_t> stream;
    stream.insert(stream.end(), garbage.begin(), garbage.end());
    stream.insert(stream.end(), wire.begin(), wire.end());

    tb::FrameParser p;
    p.feed(stream);
    EXPECT_EQ(p.pending(), 1u);

    tb::Frame f;
    ASSERT_TRUE(p.try_pop(f));
    EXPECT_EQ(f.cmd, tp::Cmd::GetImu);
    EXPECT_EQ(f.payload.size(), 28u);
}

// Regression guard for the V1.3 frame-cap bump: confirm a >1 KB OTA
// write-block-sized frame still pack/parse round-trips.
TEST(FrameParser, OtaWriteBlockSizedFrameRoundtrips) {
    // 6-byte OtaWriteBlock header + 1024-byte block = 1030-byte payload,
    // well past the V1.2 cap of 502 and within the V1.3 cap of 2294.
    std::vector<uint8_t> payload(1030);
    for (size_t i = 0; i < payload.size(); ++i) {
        payload[i] = static_cast<uint8_t>(i * 13 + 7);
    }
    auto wire = tb::pack_frame(tp::Address::PC, 42,
                               tp::FrameType::CMD_NEED_ACK,
                               tp::Cmd::OtaWriteBlock,
                               payload);

    tb::FrameParser p;
    p.feed(wire);
    ASSERT_EQ(p.pending(), 1u);

    tb::Frame f;
    ASSERT_TRUE(p.try_pop(f));
    EXPECT_EQ(f.cmd, tp::Cmd::OtaWriteBlock);
    ASSERT_EQ(f.payload.size(), payload.size());
    EXPECT_EQ(std::memcmp(f.payload.data(), payload.data(), payload.size()), 0);
}

TEST(FrameParser, DropsBadCrcFrameButContinues) {
    auto bad = tb::pack_frame(tp::Address::PC, 1,
                              tp::FrameType::DATA, tp::Cmd::GetImu,
                              random_payload(8, 1));
    bad[bad.size() - 2] ^= 0xFF;  // corrupt CRC

    auto good = tb::pack_frame(tp::Address::PC, 2,
                               tp::FrameType::DATA, tp::Cmd::GetEncoder,
                               random_payload(16, 2));

    std::vector<uint8_t> stream;
    stream.insert(stream.end(), bad.begin(),  bad.end());
    stream.insert(stream.end(), good.begin(), good.end());

    tb::FrameParser p;
    p.feed(stream);

    tb::Frame f;
    ASSERT_TRUE(p.try_pop(f));
    EXPECT_EQ(f.cmd, tp::Cmd::GetEncoder);
    EXPECT_FALSE(p.try_pop(f));
}

TEST(FrameParser, BoundsResyncBuffer) {
    tb::FrameParser p(/*max_buffered=*/64);
    // Feed 1024 bytes of pure garbage; buffer must not exceed 64 (rough
    // upper bound: drain_ trims after each feed).
    std::vector<uint8_t> garbage(1024, 0xCC);
    p.feed(garbage);
    EXPECT_LE(p.buffered_bytes(), 64u);
    EXPECT_EQ(p.pending(), 0u);
}
