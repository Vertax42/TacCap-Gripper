// Copyright (c) 2026 XenseRobotics Co., Ltd. — Apache-2.0

#include <taccap/bus/transport.hpp>
#include <taccap/protocol/codec.hpp>

#include <algorithm>
#include <cerrno>
#include <utility>

namespace xense::taccap::bus {

namespace {
using namespace std::chrono_literals;
}

// ---- ctor / dtor ----------------------------------------------------------

Transport::Transport(const Config& cfg)
    : cfg_(cfg),
      serial_(cfg.serial),
      parser_(cfg.parser_max_buf) {
    running_.store(true, std::memory_order_release);
    reader_ = std::thread(&Transport::reader_loop_, this);
}

Transport::~Transport() {
    stop();
}

// ---- public API -----------------------------------------------------------

AckResponse Transport::send_cmd(protocol::Cmd cmd,
                                const std::vector<uint8_t>& payload,
                                std::chrono::milliseconds timeout) {
    if (!running_.load(std::memory_order_acquire)) {
        throw IoError("send_cmd on stopped transport", EBADF);
    }
    const auto t = (timeout.count() == 0) ? cfg_.ack_timeout : timeout;

    for (unsigned attempt = 0; attempt <= cfg_.max_retries; ++attempt) {
        const uint8_t seq = next_seq_.fetch_add(1, std::memory_order_relaxed);
        std::future<AckResponse> fut;
        {
            std::lock_guard<std::mutex> lk(pending_mu_);
            // unique_ptr-style emplace: default-construct the PendingAck.
            auto& p = pending_acks_[seq];
            // If the slot was somehow taken (uint8 wrap collision with a
            // very-late stray ACK), reset its promise.
            p.promise = std::promise<AckResponse>{};
            fut = p.promise.get_future();
        }

        try {
            auto wire = pack_frame(cfg_.peer, seq,
                                   protocol::FrameType::CMD_NEED_ACK,
                                   cmd, payload);
            serial_.write(wire);
            stat_bytes_written_.fetch_add(wire.size(), std::memory_order_relaxed);
            stat_frames_sent_.fetch_add(1, std::memory_order_relaxed);
        } catch (...) {
            std::lock_guard<std::mutex> lk(pending_mu_);
            pending_acks_.erase(seq);
            throw;
        }

        const auto status = fut.wait_for(t);
        if (status == std::future_status::ready) {
            // Note: pending entry already erased by handle_ack_ on fulfilment.
            return fut.get();   // may rethrow IoError set by fail_pending_
        }

        // Timeout — clean up entry, retry.
        {
            std::lock_guard<std::mutex> lk(pending_mu_);
            pending_acks_.erase(seq);
        }
        stat_ack_timeouts_.fetch_add(1, std::memory_order_relaxed);
        if (attempt < cfg_.max_retries) {
            stat_retries_.fetch_add(1, std::memory_order_relaxed);
            std::this_thread::sleep_for(cfg_.retry_interval);
        }
    }

    throw TimeoutError("send_cmd " + std::string(protocol::to_string(cmd)) +
                       ": no ACK after " +
                       std::to_string(cfg_.max_retries + 1) + " attempts",
                       ETIMEDOUT);
}

void Transport::send_cmd_no_ack(protocol::Cmd cmd,
                                const std::vector<uint8_t>& payload) {
    if (!running_.load(std::memory_order_acquire)) {
        throw IoError("send_cmd_no_ack on stopped transport", EBADF);
    }
    const uint8_t seq = next_seq_.fetch_add(1, std::memory_order_relaxed);
    auto wire = pack_frame(cfg_.peer, seq,
                           protocol::FrameType::CMD_NO_ACK, cmd, payload);
    serial_.write(wire);
    stat_bytes_written_.fetch_add(wire.size(), std::memory_order_relaxed);
    stat_frames_sent_.fetch_add(1, std::memory_order_relaxed);
}

Transport::SubscriptionId
Transport::subscribe(protocol::Cmd cmd, DataCallback cb) {
    SubscriptionId id = next_sub_id_.fetch_add(1, std::memory_order_relaxed);
    std::lock_guard<std::mutex> lk(sub_mu_);
    subs_.push_back({id, cmd, std::move(cb)});
    return id;
}

void Transport::unsubscribe(SubscriptionId id) {
    std::lock_guard<std::mutex> lk(sub_mu_);
    subs_.erase(std::remove_if(subs_.begin(), subs_.end(),
                               [id](const Sub& s) { return s.id == id; }),
                subs_.end());
}

bool Transport::is_running() const noexcept {
    return running_.load(std::memory_order_acquire);
}

void Transport::stop() noexcept {
    if (!running_.exchange(false, std::memory_order_acq_rel)) {
        // Already stopped — make sure the reader (if it's still alive due to
        // an earlier failure path) is joined.
        if (reader_.joinable()) reader_.join();
        return;
    }
    stop_requested_.store(true, std::memory_order_release);
    if (reader_.joinable()) reader_.join();
    fail_pending_("transport stopped");
}

Transport::Stats Transport::stats() const noexcept {
    Stats s;
    s.bytes_read          = stat_bytes_read_.load(std::memory_order_relaxed);
    s.bytes_written       = stat_bytes_written_.load(std::memory_order_relaxed);
    s.frames_received     = stat_frames_received_.load(std::memory_order_relaxed);
    s.frames_sent         = stat_frames_sent_.load(std::memory_order_relaxed);
    s.ack_timeouts        = stat_ack_timeouts_.load(std::memory_order_relaxed);
    s.retries             = stat_retries_.load(std::memory_order_relaxed);
    s.unexpected_frames   = stat_unexpected_frames_.load(std::memory_order_relaxed);
    s.callback_exceptions = stat_callback_exceptions_.load(std::memory_order_relaxed);
    return s;
}

// ---- reader thread --------------------------------------------------------

void Transport::reader_loop_() {
    std::vector<uint8_t> buf(cfg_.rx_chunk_bytes);

    while (!stop_requested_.load(std::memory_order_acquire)) {
        std::size_t n = 0;
        try {
            n = serial_.read(buf.data(), buf.size());
        } catch (const std::exception& e) {
            running_.store(false, std::memory_order_release);
            fail_pending_(std::string("reader: ") + e.what());
            return;
        }
        if (n == 0) continue;   // VTIME wakeup, no data

        stat_bytes_read_.fetch_add(n, std::memory_order_relaxed);
        parser_.feed(buf.data(), n);

        Frame f;
        while (parser_.try_pop(f)) {
            stat_frames_received_.fetch_add(1, std::memory_order_relaxed);
            dispatch_(f);
        }
    }
}

void Transport::dispatch_(const Frame& f) {
    switch (f.type) {
        case protocol::FrameType::ACK:
            handle_ack_(f);
            break;
        case protocol::FrameType::DATA:
            handle_data_(f);
            break;
        case protocol::FrameType::CMD_NEED_ACK:
        case protocol::FrameType::CMD_NO_ACK:
        default:
            stat_unexpected_frames_.fetch_add(1, std::memory_order_relaxed);
            break;
    }
}

void Transport::handle_ack_(const Frame& f) {
    // Match by frame.seq (firmware echoes the request seq into the response
    // frame's seq field).
    std::promise<AckResponse> winning_promise;
    bool found = false;
    {
        std::lock_guard<std::mutex> lk(pending_mu_);
        auto it = pending_acks_.find(f.seq);
        if (it != pending_acks_.end()) {
            winning_promise = std::move(it->second.promise);
            pending_acks_.erase(it);
            found = true;
        }
    }
    if (!found) {
        // Stray ACK (sender already timed out). Drop quietly.
        stat_unexpected_frames_.fetch_add(1, std::memory_order_relaxed);
        return;
    }

    AckResponse r{};
    r.seq      = f.seq;
    r.cmd      = f.cmd;
    r.is_nack  = false;
    r.data     = f.payload;

    if (static_cast<uint8_t>(f.cmd) == 0) {
        // cmd == 0 means the firmware took the protocol_send_ack(seq, err)
        // wire path instead of protocol_send_response. The first payload
        // byte carries the error code:
        //   - ERR_OK : the handler succeeded but didn't echo the command
        //              (e.g. StopStream uses this path on TC-GU-01 v1.1
        //              firmware). Treat as success with no data.
        //   - !ERR_OK: a real NACK.
        const auto err = (!f.payload.empty())
            ? static_cast<protocol::ErrorCode>(f.payload[0])
            : protocol::ErrorCode::InvalidCmd;
        if (err != protocol::ErrorCode::Ok) {
            r.is_nack    = true;
            r.error_code = err;
            try {
                winning_promise.set_exception(std::make_exception_ptr(
                    ProtocolError(std::string("NACK: ") +
                                  protocol::to_string(err))));
            } catch (...) { /* promise already satisfied — ignore */ }
            return;
        }
        // else: pure-ACK success → fall through to the success path below
        r.error_code = protocol::ErrorCode::Ok;
    } else {
        // cmd != 0: standard send_response path. The wire payload is the
        // response data verbatim. A single 0x00 byte means "no data";
        // longer payloads carry typed data (firmware_version_t, sn_info_t,
        // ImuData, ...).
        r.error_code = protocol::ErrorCode::Ok;
    }

    try {
        winning_promise.set_value(std::move(r));
    } catch (...) { /* promise already satisfied — ignore */ }
}

void Transport::handle_data_(const Frame& f) {
    // Snapshot matching callbacks under lock; release before calling so
    // callbacks can safely subscribe/unsubscribe (re-entrant).
    std::vector<DataCallback> hits;
    {
        std::lock_guard<std::mutex> lk(sub_mu_);
        for (const auto& s : subs_) {
            if (s.cmd == f.cmd) hits.push_back(s.cb);
        }
    }
    for (auto& cb : hits) {
        try {
            cb(f);
        } catch (...) {
            stat_callback_exceptions_.fetch_add(1, std::memory_order_relaxed);
        }
    }
}

void Transport::fail_pending_(const std::string& reason) noexcept {
    std::lock_guard<std::mutex> lk(pending_mu_);
    for (auto& kv : pending_acks_) {
        try {
            kv.second.promise.set_exception(std::make_exception_ptr(
                IoError("transport: " + reason, EIO)));
        } catch (...) { /* already satisfied */ }
    }
    pending_acks_.clear();
}

}  // namespace xense::taccap::bus
