// Copyright (c) 2026 XenseRobotics Co., Ltd. — Apache-2.0
//
// Transport tests use openpty(): the host-side SerialBus opens the PTY's
// slave (a real /dev/pts/N tty so termios setup works), and the test acts
// as a "fake firmware" on the master end, reading host-issued frames and
// writing back ACK / DATA frames.

#include <gtest/gtest.h>
#include <taccap/bus/transport.hpp>
#include <taccap/bus/frame.hpp>
#include <taccap/protocol/payloads.hpp>
#include <taccap/error.hpp>

#include "pty_helper.hpp"

#include <chrono>
#include <cstring>
#include <thread>
#include <vector>

namespace tb = xense::taccap::bus;
namespace tp = xense::taccap::protocol;

using taccap_test::Pty;
using taccap_test::base_config;

TEST(Transport, SendCmdReceivesAck) {
    Pty pty;
    ASSERT_GE(pty.master(), 0);

    tb::Transport host(base_config(pty.slave_path()));

    // Drive the fake firmware in a side thread.
    std::thread fw([&]() {
        auto f = pty.expect_frame();
        ASSERT_TRUE(f.has_value());
        EXPECT_EQ(f->cmd,  tp::Cmd::GetVersion);
        EXPECT_EQ(f->type, tp::FrameType::CMD_NEED_ACK);
        // Firmware "success no data" path: send_response with empty payload
        // → wire ACK with cmd=GetVersion and payload=[ERR_OK].
        pty.send_ack_ok(f->seq, tp::Cmd::GetVersion);
    });

    auto ack = host.send_cmd(tp::Cmd::GetVersion);
    EXPECT_EQ(ack.error_code, tp::ErrorCode::Ok);
    EXPECT_FALSE(ack.is_nack);
    EXPECT_EQ(ack.cmd, tp::Cmd::GetVersion);
    EXPECT_EQ(ack.data.size(), 1u);  // single ERR_OK byte

    fw.join();
    EXPECT_EQ(host.stats().ack_timeouts, 0u);
    EXPECT_EQ(host.stats().retries,      0u);
}

TEST(Transport, SendCmdNackThrowsProtocolError) {
    Pty pty;
    ASSERT_GE(pty.master(), 0);
    tb::Transport host(base_config(pty.slave_path()));

    std::thread fw([&]() {
        auto f = pty.expect_frame();
        ASSERT_TRUE(f.has_value());
        pty.send_nack(f->seq, tp::ErrorCode::InvalidParam);
    });

    EXPECT_THROW(host.send_cmd(tp::Cmd::SetSn, std::vector<uint8_t>{1, 2, 3}),
                 xense::taccap::ProtocolError);

    fw.join();
}

TEST(Transport, RetryThenSucceed) {
    Pty pty;
    ASSERT_GE(pty.master(), 0);

    auto cfg = base_config(pty.slave_path());
    cfg.max_retries    = 3;
    cfg.ack_timeout    = std::chrono::milliseconds(40);
    cfg.retry_interval = std::chrono::milliseconds(5);
    tb::Transport host(cfg);

    std::thread fw([&]() {
        // Drop the first two attempts, ACK the third.
        for (int i = 0; i < 2; ++i) {
            auto f = pty.expect_frame(500);
            ASSERT_TRUE(f.has_value());
            // intentionally don't ACK
        }
        auto f = pty.expect_frame(500);
        ASSERT_TRUE(f.has_value());
        pty.send_ack_ok(f->seq, tp::Cmd::GetVersion);
    });

    auto ack = host.send_cmd(tp::Cmd::GetVersion);
    EXPECT_EQ(ack.error_code, tp::ErrorCode::Ok);

    fw.join();
    EXPECT_GE(host.stats().retries,      2u);
    EXPECT_GE(host.stats().ack_timeouts, 2u);
    EXPECT_GE(host.stats().frames_sent,  3u);
}

TEST(Transport, AckTimeoutThrowsAfterRetries) {
    Pty pty;
    ASSERT_GE(pty.master(), 0);

    auto cfg = base_config(pty.slave_path());
    cfg.max_retries    = 1;
    cfg.ack_timeout    = std::chrono::milliseconds(20);
    cfg.retry_interval = std::chrono::milliseconds(2);
    tb::Transport host(cfg);

    // Side thread just drains the master so the kernel buffer doesn't fill,
    // but never replies. (Not strictly needed for small commands.)
    std::thread drain([&]() {
        for (int i = 0; i <= cfg.max_retries; ++i) {
            (void)pty.expect_frame(200);
        }
    });

    EXPECT_THROW(host.send_cmd(tp::Cmd::Heartbeat),
                 xense::taccap::TimeoutError);

    drain.join();
    EXPECT_EQ(host.stats().ack_timeouts, cfg.max_retries + 1);
    EXPECT_EQ(host.stats().retries,      cfg.max_retries);
}

TEST(Transport, DataFrameDispatchedToSubscriber) {
    Pty pty;
    ASSERT_GE(pty.master(), 0);
    tb::Transport host(base_config(pty.slave_path()));

    std::mutex            m;
    std::condition_variable cv;
    bool                  got = false;
    std::vector<uint8_t>  seen;

    auto sub = host.subscribe(tp::Cmd::GetImu, [&](const tb::Frame& f) {
        std::lock_guard<std::mutex> lk(m);
        seen = f.payload;
        got  = true;
        cv.notify_one();
    });
    (void)sub;

    std::vector<uint8_t> body(sizeof(tp::ImuData), 0);
    body[0] = 0xDE; body[1] = 0xAD; body[2] = 0xBE; body[3] = 0xEF;
    pty.send_data(/*seq=*/7, tp::Cmd::GetImu, body);

    std::unique_lock<std::mutex> lk(m);
    ASSERT_TRUE(cv.wait_for(lk, std::chrono::seconds(1),
                            [&]() { return got; }));
    EXPECT_EQ(seen.size(), sizeof(tp::ImuData));
    EXPECT_EQ(seen[0], 0xDE);
}

TEST(Transport, UnsubscribeStopsCallback) {
    Pty pty;
    ASSERT_GE(pty.master(), 0);
    tb::Transport host(base_config(pty.slave_path()));

    std::atomic<int> count{0};
    auto sub = host.subscribe(tp::Cmd::GetEncoder,
                              [&](const tb::Frame&) { ++count; });

    // First frame should be delivered.
    pty.send_data(1, tp::Cmd::GetEncoder, std::vector<uint8_t>(16, 0));
    auto wait_until = [](auto pred, int ms) {
        const auto end = std::chrono::steady_clock::now() +
                         std::chrono::milliseconds(ms);
        while (std::chrono::steady_clock::now() < end) {
            if (pred()) return true;
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }
        return false;
    };
    EXPECT_TRUE(wait_until([&]() { return count.load() >= 1; }, 500));

    host.unsubscribe(sub);

    // Subsequent frame should NOT bump the counter.
    pty.send_data(2, tp::Cmd::GetEncoder, std::vector<uint8_t>(16, 0));
    std::this_thread::sleep_for(std::chrono::milliseconds(60));
    EXPECT_EQ(count.load(), 1);
}

TEST(Transport, SendCmdNoAckDoesNotBlock) {
    Pty pty;
    ASSERT_GE(pty.master(), 0);
    tb::Transport host(base_config(pty.slave_path()));

    const auto t0 = std::chrono::steady_clock::now();
    host.send_cmd_no_ack(tp::Cmd::Heartbeat);
    const auto dt = std::chrono::steady_clock::now() - t0;
    EXPECT_LT(std::chrono::duration_cast<std::chrono::milliseconds>(dt).count(),
              50)
        << "send_cmd_no_ack should not block";

    auto f = pty.expect_frame(500);
    ASSERT_TRUE(f.has_value());
    EXPECT_EQ(f->type, tp::FrameType::CMD_NO_ACK);
    EXPECT_EQ(f->cmd,  tp::Cmd::Heartbeat);
}

TEST(Transport, StrayAckIgnored) {
    Pty pty;
    ASSERT_GE(pty.master(), 0);
    tb::Transport host(base_config(pty.slave_path()));

    // No outstanding command — push a stray ACK with a random seq.
    pty.send_ack_ok(/*seq=*/123, tp::Cmd::GetVersion);

    // Give the reader a moment to process.
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    // The transport should still be running and stats should reflect the
    // unmatched frame as unexpected.
    EXPECT_TRUE(host.is_running());
    EXPECT_GE(host.stats().unexpected_frames, 1u);
}

TEST(Transport, StatsCountersIncrement) {
    Pty pty;
    ASSERT_GE(pty.master(), 0);
    tb::Transport host(base_config(pty.slave_path()));

    std::thread fw([&]() {
        auto f = pty.expect_frame();
        ASSERT_TRUE(f.has_value());
        pty.send_ack_ok(f->seq, tp::Cmd::GetVersion);
    });
    host.send_cmd(tp::Cmd::GetVersion);
    fw.join();

    auto s = host.stats();
    EXPECT_GE(s.frames_sent,     1u);
    EXPECT_GE(s.frames_received, 1u);
    EXPECT_GT(s.bytes_written,   0u);
    EXPECT_GT(s.bytes_read,      0u);
}

TEST(Transport, PureAckCmdZeroOkIsSuccess) {
    // Firmware quirk (TC-GU-01 v1.1): some handlers (notably StopStream) take
    // the protocol_send_ack(seq, ERR_OK) wire path, producing an ACK frame
    // with cmd=0 and payload=[0x00]. That's "success with no data", NOT a
    // NACK — the host must accept it.
    Pty pty;
    ASSERT_GE(pty.master(), 0);
    tb::Transport host(base_config(pty.slave_path()));

    std::thread fw([&]() {
        auto f = pty.expect_frame();
        ASSERT_TRUE(f.has_value());
        pty.send_nack(f->seq, tp::ErrorCode::Ok);   // cmd=0, payload=[0x00]
    });
    auto ack = host.send_cmd(tp::Cmd::StopStream);
    EXPECT_FALSE(ack.is_nack);
    EXPECT_EQ(ack.error_code, tp::ErrorCode::Ok);
    EXPECT_EQ(static_cast<uint8_t>(ack.cmd), 0u);   // cmd is 0 on this path
    fw.join();
}

TEST(Transport, PureAckCmdZeroNonOkIsNack) {
    // A real NACK still produces ProtocolError.
    Pty pty;
    ASSERT_GE(pty.master(), 0);
    tb::Transport host(base_config(pty.slave_path()));

    std::thread fw([&]() {
        auto f = pty.expect_frame();
        ASSERT_TRUE(f.has_value());
        pty.send_nack(f->seq, tp::ErrorCode::InvalidParam);
    });
    EXPECT_THROW(host.send_cmd(tp::Cmd::SetSn, std::vector<uint8_t>{1}),
                 xense::taccap::ProtocolError);
    fw.join();
}

TEST(Transport, StopIsIdempotent) {
    Pty pty;
    ASSERT_GE(pty.master(), 0);
    tb::Transport host(base_config(pty.slave_path()));

    EXPECT_TRUE(host.is_running());
    host.stop();
    EXPECT_FALSE(host.is_running());
    host.stop();   // second call must not crash / hang
    EXPECT_FALSE(host.is_running());

    EXPECT_THROW(host.send_cmd(tp::Cmd::Heartbeat),
                 xense::taccap::IoError);
}
