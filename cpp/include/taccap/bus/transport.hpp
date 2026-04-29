// Copyright (c) 2026 XenseRobotics Co., Ltd. — Apache-2.0
//
// Asynchronous transport for TC-GU-01:
//   - owns a SerialBus + FrameParser + a background reader thread
//   - matches CMD_NEED_ACK frames to incoming ACK frames by seq, with
//     host-side timeout/retry honouring the firmware's 10ms / 3-attempt spec
//   - dispatches DATA frames to per-Cmd subscriber callbacks
//
// Synchronous send_cmd() blocks the caller; subscribe() callbacks fire on
// the reader thread (callbacks must be short and non-blocking — push to a
// queue if you need real work). Component-level wrappers (IMU, Encoder,
// Motor, ...) build on top of this in step 4.

#pragma once

#include <taccap/bus/frame.hpp>
#include <taccap/bus/serial_bus.hpp>
#include <taccap/error.hpp>
#include <taccap/protocol/commands.hpp>
#include <taccap/protocol/payloads.hpp>

#include <atomic>
#include <chrono>
#include <cstdint>
#include <functional>
#include <future>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

namespace xense::taccap::bus {

// Decoded ACK / NACK from the firmware. The wire format mirrors firmware
// `protocol_handler.c` exactly:
//
//   - Failure path → `protocol_send_ack(seq, err)`:
//       frame.cmd == 0, frame.payload = [err_code]   (1 byte)
//   - Success path → `protocol_send_response(seq, cmd, ERR_OK, data, n)`:
//       frame.cmd == cmd
//       frame.payload = [ERR_OK] when n == 0 (handler had no data)
//       frame.payload = data    when n > 0  (NO err_code prefix)
//
// There is no retry_count on the wire — that field of `protocol::AckPayload`
// is firmware-internal retry book-keeping, not the ACK frame layout.
struct AckResponse {
    uint8_t              seq;         // matched against the request seq
    protocol::Cmd        cmd;         // 0 ↔ NACK; original cmd ↔ success
    bool                 is_nack;     // true iff static_cast<uint8_t>(cmd) == 0
    protocol::ErrorCode  error_code;  // Ok on success; payload[0] on NACK
    std::vector<uint8_t> data;        // wire payload as-is (response data on
                                      // success, [err_code] on NACK)
};

class Transport {
public:
    using DataCallback   = std::function<void(const Frame&)>;
    using SubscriptionId = uint64_t;

    struct Config {
        SerialBus::Config         serial;
        protocol::Address         peer            = protocol::Address::MCU;
        std::chrono::milliseconds ack_timeout     {10};
        unsigned                  max_retries     = 3;
        std::chrono::milliseconds retry_interval  {10};
        std::size_t               rx_chunk_bytes  = 4096;
        std::size_t               parser_max_buf  = 64 * 1024;
    };

    struct Stats {
        uint64_t bytes_read           = 0;
        uint64_t bytes_written        = 0;
        uint64_t frames_received      = 0;
        uint64_t frames_sent          = 0;
        uint64_t ack_timeouts         = 0;
        uint64_t retries              = 0;
        uint64_t unexpected_frames    = 0;
        uint64_t callback_exceptions  = 0;
    };

    // Opens the serial port, starts the reader thread.
    explicit Transport(const Config& cfg);

    // Stops the reader, fails any pending ACKs with IoError, joins.
    ~Transport();

    Transport(const Transport&)            = delete;
    Transport& operator=(const Transport&) = delete;

    // Send CMD_NEED_ACK and block until matching ACK arrives or all retries
    // are exhausted. `timeout` of 0 falls back to Config::ack_timeout.
    // Throws ProtocolError on NACK, TimeoutError on retry exhaustion,
    // IoError on transport failure.
    AckResponse send_cmd(protocol::Cmd cmd,
                         const std::vector<uint8_t>& payload = {},
                         std::chrono::milliseconds timeout = std::chrono::milliseconds{0});

    // Send CMD_NO_ACK fire-and-forget.
    void send_cmd_no_ack(protocol::Cmd cmd,
                         const std::vector<uint8_t>& payload = {});

    // Register a callback for DATA frames whose `cmd` byte matches.
    // Callback runs on the reader thread; keep it short.
    SubscriptionId subscribe(protocol::Cmd cmd, DataCallback cb);
    void unsubscribe(SubscriptionId id);

    bool is_running() const noexcept;
    void stop() noexcept;   // graceful, idempotent

    Stats stats() const noexcept;

    const Config& config() const noexcept { return cfg_; }

private:
    struct PendingAck {
        std::promise<AckResponse> promise;
    };
    struct Sub {
        SubscriptionId  id;
        protocol::Cmd   cmd;
        DataCallback    cb;
    };

    void reader_loop_();
    void dispatch_(const Frame& f);
    void handle_ack_(const Frame& f);
    void handle_data_(const Frame& f);

    // Fail every pending ACK with the given error message. Called on
    // reader exit and on stop().
    void fail_pending_(const std::string& reason) noexcept;

    Config         cfg_;
    SerialBus      serial_;
    FrameParser    parser_;

    std::atomic<bool>          stop_requested_{false};
    std::atomic<bool>          running_{false};
    std::thread                reader_;

    // ACK matching
    std::mutex                                 pending_mu_;
    std::unordered_map<uint8_t, PendingAck>    pending_acks_;
    std::atomic<uint8_t>                       next_seq_{0};

    // Subscriptions
    std::mutex                                 sub_mu_;
    std::vector<Sub>                           subs_;
    std::atomic<SubscriptionId>                next_sub_id_{1};

    // Stats (atomic counters; read via Stats snapshot)
    std::atomic<uint64_t> stat_bytes_read_         {0};
    std::atomic<uint64_t> stat_bytes_written_      {0};
    std::atomic<uint64_t> stat_frames_received_    {0};
    std::atomic<uint64_t> stat_frames_sent_        {0};
    std::atomic<uint64_t> stat_ack_timeouts_       {0};
    std::atomic<uint64_t> stat_retries_            {0};
    std::atomic<uint64_t> stat_unexpected_frames_  {0};
    std::atomic<uint64_t> stat_callback_exceptions_{0};
};

}  // namespace xense::taccap::bus
