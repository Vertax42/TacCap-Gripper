// Copyright (c) 2026 XenseRobotics Co., Ltd. — Apache-2.0
//
// Retry semantics and error detection for the OTA command surface, driven
// against a PTY-backed fake firmware.
//
// Two firmware behaviours make plain retry-with-a-new-seq wrong for OTA:
//
//   1. ota_write_block() demands strictly sequential offsets and marks the
//      whole session failed on a repeat. A retry after a merely-slow ACK
//      therefore has to look like the SAME request, so the firmware replays
//      its cached response (protocol_resend_cached_response) instead of
//      re-running the handler. That is RetryMode::SameSeq.
//
//   2. A handler that returns non-OK goes through protocol_send_response(seq,
//      cmd, err, NULL, 0): the command byte is ECHOED and the error is the
//      whole payload. ack.is_nack cannot see that, so OTA has to go through
//      bus::ack_error_code — otherwise a failed update reports success.

#include "pty_helper.hpp"

#include <gtest/gtest.h>
#include <taccap/bus/transport.hpp>
#include <taccap/error.hpp>
#include <taccap/ota.hpp>
#include <taccap/protocol/payloads.hpp>

#include <atomic>
#include <chrono>
#include <cstring>
#include <thread>
#include <vector>

namespace tx = xense::taccap;
namespace tp = xense::taccap::protocol;
namespace tb = xense::taccap::bus;
using taccap_test::Pty;
using taccap_test::base_config;

namespace {

// The firmware error path: command byte echoed, payload is the bare code.
std::vector<uint8_t> error_response(tp::ErrorCode err) {
    return {static_cast<uint8_t>(err)};
}

}  // namespace

// ---- ack_error_code: the shared echoed-cmd decoder -------------------------

TEST(AckErrorCode, RecognisesTheEchoedCommandErrorPath) {
    tb::AckResponse ack{};
    ack.cmd     = tp::Cmd::OtaWriteBlock;   // echoed, not 0
    ack.is_nack = false;
    ack.data    = error_response(tp::ErrorCode::OtaOffsetErr);
    EXPECT_EQ(tb::ack_error_code(ack), tp::ErrorCode::OtaOffsetErr);
}

TEST(AckErrorCode, TreatsSingleZeroByteAsSuccess) {
    tb::AckResponse ack{};
    ack.cmd     = tp::Cmd::OtaWriteBlock;
    ack.is_nack = false;
    ack.data    = {0x00};                   // firmware's "no data" success
    EXPECT_EQ(tb::ack_error_code(ack), tp::ErrorCode::Ok);
}

TEST(AckErrorCode, LeavesMultiByteResponsesAlone) {
    tb::AckResponse ack{};
    ack.cmd     = tp::Cmd::OtaGetStatus;
    ack.is_nack = false;
    ack.data    = std::vector<uint8_t>(sizeof(tp::OtaStatus), 0x52);
    EXPECT_EQ(tb::ack_error_code(ack), tp::ErrorCode::Ok);
}

TEST(AckErrorCode, PassesThroughARealNack) {
    tb::AckResponse ack{};
    ack.cmd        = static_cast<tp::Cmd>(0);
    ack.is_nack    = true;
    ack.error_code = tp::ErrorCode::OtaBusy;
    EXPECT_EQ(tb::ack_error_code(ack), tp::ErrorCode::OtaBusy);
}

// ---- OTA must throw on the echoed-cmd error path ---------------------------
//
// Before this was handled, every one of these silently "succeeded" and the
// update ran to completion reporting success on a firmware that had aborted.

TEST(OtaErrorPath, WriteBlockThrowsOnEchoedOffsetError) {
    Pty pty;
    tb::Transport host(base_config(pty.slave_path()));
    tx::OtaSession ota(host);

    std::thread fw([&]() {
        auto f = pty.expect_frame();
        if (f) pty.send_response(f->seq, tp::Cmd::OtaWriteBlock,
                                 error_response(tp::ErrorCode::OtaOffsetErr));
    });

    std::vector<uint8_t> block(64, 0xAB);
    EXPECT_THROW(ota.write_block(0, block.data(),
                                 static_cast<uint16_t>(block.size())),
                 tx::ProtocolError);
    fw.join();
}

TEST(OtaErrorPath, StartThrowsOnEchoedBusyError) {
    Pty pty;
    tb::Transport host(base_config(pty.slave_path()));
    tx::OtaSession ota(host);

    std::thread fw([&]() {
        auto f = pty.expect_frame();
        if (f) pty.send_response(f->seq, tp::Cmd::OtaStart,
                                 error_response(tp::ErrorCode::OtaBusy));
    });

    EXPECT_THROW(ota.start(2048, 0xDEADBEEF, {1, 2, 0, 0},
                           std::chrono::milliseconds(300)),
                 tx::ProtocolError);
    fw.join();
}

TEST(OtaErrorPath, VerifyThrowsOnEchoedCrcFailure) {
    Pty pty;
    tb::Transport host(base_config(pty.slave_path()));
    tx::OtaSession ota(host);

    std::thread fw([&]() {
        auto f = pty.expect_frame();
        if (f) pty.send_response(f->seq, tp::Cmd::OtaVerify,
                                 error_response(tp::ErrorCode::OtaVerifyFail));
    });

    EXPECT_THROW(ota.verify(std::chrono::milliseconds(300)), tx::ProtocolError);
    fw.join();
}

TEST(OtaErrorPath, FullUpdateAbortsInsteadOfReportingSuccess) {
    Pty pty;
    tb::Transport host(base_config(pty.slave_path()));
    tx::OtaSession ota(host);

    // Firmware accepts the start, then rejects the very first block. The
    // update must surface that, not sail on to verify/apply.
    std::atomic<bool> saw_apply{false};
    std::thread fw([&]() {
        for (;;) {
            auto f = pty.expect_frame(1000);
            if (!f) return;
            if (f->cmd == tp::Cmd::OtaStart) {
                pty.send_ack_ok(f->seq, tp::Cmd::OtaStart);
            } else if (f->cmd == tp::Cmd::OtaWriteBlock) {
                pty.send_response(f->seq, tp::Cmd::OtaWriteBlock,
                                  error_response(tp::ErrorCode::OtaOffsetErr));
            } else if (f->cmd == tp::Cmd::OtaAbort) {
                pty.send_ack_ok(f->seq, tp::Cmd::OtaAbort);
                return;   // update_from_bytes aborts, then rethrows
            } else {
                if (f->cmd == tp::Cmd::OtaApply) saw_apply = true;
                pty.send_ack_ok(f->seq, f->cmd);
            }
        }
    });

    std::vector<uint8_t> fw_blob(512, 0x5A);
    EXPECT_THROW(ota.update_from_bytes(fw_blob, {1, 2, 0, 0}),
                 tx::ProtocolError);
    fw.join();
    EXPECT_FALSE(saw_apply.load())
        << "a rejected write must never reach OtaApply";
}

// ---- RetryMode ------------------------------------------------------------

// The whole point: a retry must reuse the seq so the firmware recognises the
// repeat. Fake firmware swallows the first attempt, then answers the second.
TEST(RetryMode, SameSeqReusesTheSequenceNumberAcrossAttempts) {
    Pty pty;
    auto cfg = base_config(pty.slave_path());
    cfg.max_retries = 2;
    cfg.ack_timeout = std::chrono::milliseconds(60);
    tb::Transport host(cfg);

    std::vector<uint8_t> seqs;
    std::thread fw([&]() {
        auto first = pty.expect_frame(500);
        if (!first) return;
        seqs.push_back(first->seq);          // drop it — simulate a lost ACK
        auto second = pty.expect_frame(500);
        if (!second) return;
        seqs.push_back(second->seq);
        pty.send_ack_ok(second->seq, tp::Cmd::OtaVerify);
    });

    auto ack = host.send_cmd(tp::Cmd::OtaVerify, {},
                             std::chrono::milliseconds(60),
                             tb::RetryMode::SameSeq);
    fw.join();

    EXPECT_EQ(ack.cmd, tp::Cmd::OtaVerify);
    ASSERT_EQ(seqs.size(), 2u);
    EXPECT_EQ(seqs[0], seqs[1])
        << "SameSeq retries must reuse the seq so the firmware replays its "
           "cached response instead of re-running a non-idempotent handler";
}

TEST(RetryMode, NewSeqIsTheDefaultAndStillAdvances) {
    Pty pty;
    auto cfg = base_config(pty.slave_path());
    cfg.max_retries = 2;
    cfg.ack_timeout = std::chrono::milliseconds(60);
    tb::Transport host(cfg);

    std::vector<uint8_t> seqs;
    std::thread fw([&]() {
        auto first = pty.expect_frame(500);
        if (!first) return;
        seqs.push_back(first->seq);
        auto second = pty.expect_frame(500);
        if (!second) return;
        seqs.push_back(second->seq);
        pty.send_ack_ok(second->seq, tp::Cmd::Heartbeat);
    });

    // No RetryMode argument — the default must stay NewSeq.
    auto ack = host.send_cmd(tp::Cmd::Heartbeat, {},
                             std::chrono::milliseconds(60));
    fw.join();

    EXPECT_EQ(ack.cmd, tp::Cmd::Heartbeat);
    ASSERT_EQ(seqs.size(), 2u);
    EXPECT_NE(seqs[0], seqs[1]);
}

// OtaSession must be wired to SameSeq — this is the regression guard for the
// bug itself, not just for the transport primitive.
TEST(RetryMode, OtaWriteBlockRetriesWithTheSameSeq) {
    Pty pty;
    auto cfg = base_config(pty.slave_path());
    cfg.max_retries = 1;
    tb::Transport host(cfg);
    tx::OtaSession ota(host);

    std::vector<uint8_t>  seqs;
    std::vector<uint32_t> offsets;
    std::thread fw([&]() {
        for (int i = 0; i < 2; ++i) {
            auto f = pty.expect_frame(1000);
            if (!f) return;
            seqs.push_back(f->seq);
            uint32_t off = 0;
            std::memcpy(&off, f->payload.data(), sizeof(off));
            offsets.push_back(off);
            // Swallow the first attempt; replay a cached OK for the repeat,
            // exactly as protocol_resend_cached_response would.
            if (i == 1) pty.send_ack_ok(f->seq, tp::Cmd::OtaWriteBlock);
        }
    });

    std::vector<uint8_t> block(128, 0x11);
    ota.write_block(4096, block.data(), static_cast<uint16_t>(block.size()),
                    std::chrono::milliseconds(60));
    fw.join();

    ASSERT_EQ(seqs.size(), 2u);
    EXPECT_EQ(seqs[0], seqs[1]);
    // Byte-identical repeat is what the firmware's dedup hashes on.
    ASSERT_EQ(offsets.size(), 2u);
    EXPECT_EQ(offsets[0], 4096u);
    EXPECT_EQ(offsets[1], 4096u);
}
