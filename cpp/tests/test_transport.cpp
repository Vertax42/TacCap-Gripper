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

#include <pty.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/select.h>

#include <chrono>
#include <cstring>
#include <thread>
#include <vector>

namespace tb = xense::taccap::bus;
namespace tp = xense::taccap::protocol;

namespace {

// PTY-backed bidirectional pipe. The host SerialBus opens the slave end;
// the test acts as the fake firmware on the master end.
class Pty {
public:
    Pty() {
        char slave_path[128] = {0};
        if (::openpty(&master_, &slave_, slave_path, nullptr, nullptr) != 0) {
            ADD_FAILURE() << "openpty failed: " << std::strerror(errno);
            master_ = slave_ = -1;
            return;
        }
        slave_path_ = slave_path;
        // Master is poll-able; non-blocking helps test cleanup.
        ::fcntl(master_, F_SETFL, ::fcntl(master_, F_GETFL, 0) | O_NONBLOCK);
    }
    ~Pty() {
        if (master_ >= 0) ::close(master_);
        if (slave_  >= 0) ::close(slave_);
    }
    Pty(const Pty&)            = delete;
    Pty& operator=(const Pty&) = delete;

    int master() const { return master_; }
    const std::string& slave_path() const { return slave_path_; }

    // Block until at least one full frame is available on the master end,
    // or `timeout_ms` elapses. Returns parsed frame on success.
    std::optional<tb::Frame> expect_frame(int timeout_ms = 1000) {
        const auto deadline = std::chrono::steady_clock::now() +
                              std::chrono::milliseconds(timeout_ms);
        uint8_t buf[256];
        for (;;) {
            tb::Frame out;
            if (parser_.try_pop(out)) return out;

            const auto now = std::chrono::steady_clock::now();
            if (now >= deadline) return std::nullopt;
            const auto remain = std::chrono::duration_cast<std::chrono::milliseconds>(
                                    deadline - now).count();

            fd_set rfds;
            FD_ZERO(&rfds);
            FD_SET(master_, &rfds);
            timeval tv{};
            tv.tv_sec  = remain / 1000;
            tv.tv_usec = (remain % 1000) * 1000;
            const int r = ::select(master_ + 1, &rfds, nullptr, nullptr, &tv);
            if (r <= 0) continue;   // timeout or EINTR

            const ssize_t n = ::read(master_, buf, sizeof(buf));
            if (n > 0) parser_.feed(buf, static_cast<std::size_t>(n));
        }
    }

    // Mirrors the firmware's two response paths (protocol_handler.c).
    //
    // success_with_data: protocol_send_response(seq, cmd, ERR_OK, payload, n).
    void send_response(uint8_t seq, tp::Cmd cmd,
                       std::vector<uint8_t> data) {
        // Firmware path: payload is data only (no err prefix). Empty data
        // becomes a single ERR_OK byte (mimics protocol_send_response with
        // payload_len == 0).
        if (data.empty()) data.push_back(static_cast<uint8_t>(tp::ErrorCode::Ok));
        write_frame(tp::Address::MCU, seq, tp::FrameType::ACK, cmd, data);
    }

    // failure: protocol_send_ack(seq, err) — frame.cmd = 0, payload = [err].
    void send_nack(uint8_t seq, tp::ErrorCode err) {
        std::vector<uint8_t> body{static_cast<uint8_t>(err)};
        write_frame(tp::Address::MCU, seq,
                    tp::FrameType::ACK,
                    static_cast<tp::Cmd>(0),  // NACK uses cmd=0
                    body);
    }

    // Convenience: success ACK with no extra data (just the ERR_OK byte).
    void send_ack_ok(uint8_t seq, tp::Cmd echoed_cmd) {
        send_response(seq, echoed_cmd, {});
    }

    void send_data(uint8_t seq, tp::Cmd cmd,
                   const std::vector<uint8_t>& payload) {
        write_frame(tp::Address::MCU, seq, tp::FrameType::DATA, cmd, payload);
    }

    void send_raw(const std::vector<uint8_t>& bytes) {
        const ssize_t n = ::write(master_, bytes.data(), bytes.size());
        ASSERT_EQ(static_cast<std::size_t>(n), bytes.size());
    }

private:
    void write_frame(tp::Address addr, uint8_t seq, tp::FrameType type,
                     tp::Cmd cmd, const std::vector<uint8_t>& payload) {
        auto wire = tb::pack_frame(addr, seq, type, cmd, payload);
        const ssize_t n = ::write(master_, wire.data(), wire.size());
        ASSERT_EQ(static_cast<std::size_t>(n), wire.size())
            << "PTY write short: " << std::strerror(errno);
    }

    int             master_ = -1;
    int             slave_  = -1;
    std::string     slave_path_;
    tb::FrameParser parser_;
};

// Build a Transport bound to the slave end. Use a 9600 baud (still
// works on PTY) to avoid setting B3000000 on a pty (some kernels reject it).
tb::Transport::Config base_config(const std::string& dev) {
    tb::Transport::Config c;
    c.serial.device          = dev;
    c.serial.baudrate        = 9600;     // PTY ignores baud, but B9600 is universal
    c.serial.read_timeout_ms = 1;
    c.peer                   = tp::Address::MCU;
    c.ack_timeout            = std::chrono::milliseconds(50);
    c.max_retries            = 2;
    c.retry_interval         = std::chrono::milliseconds(5);
    return c;
}

}  // namespace

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
