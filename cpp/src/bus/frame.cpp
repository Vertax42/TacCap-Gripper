// Copyright (c) 2026 XenseRobotics Co., Ltd. — Apache-2.0

#include <taccap/bus/frame.hpp>
#include <taccap/error.hpp>

#include <algorithm>
#include <cstring>

namespace xense::taccap::bus {

uint16_t crc16_modbus(const uint8_t* data, std::size_t len) noexcept {
    uint16_t crc = 0xFFFF;
    for (std::size_t i = 0; i < len; ++i) {
        crc ^= data[i];
        for (int b = 0; b < 8; ++b) {
            if (crc & 0x0001) {
                crc >>= 1;
                crc ^= 0xA001;
            } else {
                crc >>= 1;
            }
        }
    }
    return crc;
}

std::vector<uint8_t> stuff_data(const uint8_t* in, std::size_t len) {
    std::vector<uint8_t> out;
    out.reserve(len);
    for (std::size_t i = 0; i < len; ++i) {
        const uint8_t b = in[i];
        if (b == FRAME_HEAD || b == FRAME_TAIL || b == FRAME_ESCAPE) {
            out.push_back(FRAME_ESCAPE);
            out.push_back(static_cast<uint8_t>(b ^ 0x20));
        } else {
            out.push_back(b);
        }
    }
    return out;
}

std::vector<uint8_t> unstuff_data(const uint8_t* in, std::size_t len) {
    std::vector<uint8_t> out;
    out.reserve(len);
    for (std::size_t i = 0; i < len; ++i) {
        if (in[i] == FRAME_ESCAPE && i + 1 < len) {
            out.push_back(static_cast<uint8_t>(in[i + 1] ^ 0x20));
            ++i;
        } else {
            out.push_back(in[i]);
        }
    }
    return out;
}

std::vector<uint8_t> pack_frame(
        protocol::Address addr, uint8_t seq, protocol::FrameType type,
        protocol::Cmd cmd, const uint8_t* payload, std::size_t payload_len) {
    if (payload_len > MAX_PAYLOAD_LEN) {
        throw ProtocolError(
            "pack_frame: payload too large (" + std::to_string(payload_len) +
            " > " + std::to_string(MAX_PAYLOAD_LEN) + ")");
    }

    std::vector<uint8_t> out;
    out.reserve(MIN_FRAME_LEN + payload_len);

    out.push_back(FRAME_HEAD);
    out.push_back(static_cast<uint8_t>(addr));
    out.push_back(seq);
    out.push_back(static_cast<uint8_t>(type));
    out.push_back(static_cast<uint8_t>(cmd));
    out.push_back(static_cast<uint8_t>(payload_len & 0xFF));
    out.push_back(static_cast<uint8_t>((payload_len >> 8) & 0xFF));

    if (payload && payload_len) {
        out.insert(out.end(), payload, payload + payload_len);
    }

    const uint16_t crc = crc16_modbus(out.data(), out.size());
    out.push_back(static_cast<uint8_t>(crc & 0xFF));
    out.push_back(static_cast<uint8_t>((crc >> 8) & 0xFF));
    out.push_back(FRAME_TAIL);

    return out;
}

ParseOutcome try_parse_frame(const uint8_t* data, std::size_t len) {
    ParseOutcome r{};
    if (data == nullptr || len == 0) {
        r.status = ParseStatus::NeedMoreData;
        return r;
    }
    if (data[0] != FRAME_HEAD) {
        // Not a frame start; caller should drop bytes until they hit 0xAA.
        r.status = ParseStatus::Resync;
        // Find the next plausible start so the caller can skip ahead in one shot.
        std::size_t skip = 1;
        while (skip < len && data[skip] != FRAME_HEAD) ++skip;
        r.consumed = skip;
        return r;
    }
    if (len < MIN_FRAME_LEN) {
        r.status = ParseStatus::NeedMoreData;
        return r;
    }

    const std::size_t payload_len =
        static_cast<std::size_t>(data[5]) |
        (static_cast<std::size_t>(data[6]) << 8);

    if (payload_len > MAX_PAYLOAD_LEN) {
        // Length field is impossible — cannot be a valid frame starting here.
        r.status = ParseStatus::Resync;
        r.consumed = 1;
        return r;
    }

    const std::size_t frame_len = MIN_FRAME_LEN + payload_len;
    if (len < frame_len) {
        r.status = ParseStatus::NeedMoreData;
        return r;
    }

    if (data[frame_len - 1] != FRAME_TAIL) {
        r.status = ParseStatus::Resync;
        r.consumed = 1;
        return r;
    }

    const std::size_t crc_input_len = FRAME_HEADER_LEN + payload_len;
    const uint16_t crc_calc = crc16_modbus(data, crc_input_len);
    const uint16_t crc_recv =
        static_cast<uint16_t>(data[crc_input_len]) |
        (static_cast<uint16_t>(data[crc_input_len + 1]) << 8);

    if (crc_calc != crc_recv) {
        r.status = ParseStatus::Resync;
        r.consumed = 1;
        return r;
    }

    r.status         = ParseStatus::Success;
    r.consumed       = frame_len;
    r.frame.addr     = static_cast<protocol::Address>(data[1]);
    r.frame.seq      = data[2];
    r.frame.type     = static_cast<protocol::FrameType>(data[3]);
    r.frame.cmd      = static_cast<protocol::Cmd>(data[4]);
    r.frame.payload.assign(data + FRAME_HEADER_LEN,
                           data + FRAME_HEADER_LEN + payload_len);
    return r;
}

// ---- FrameParser ----------------------------------------------------------

FrameParser::FrameParser(std::size_t max_buffered) : max_buffered_(max_buffered) {}

void FrameParser::feed(const uint8_t* data, std::size_t len) {
    if (data && len) {
        rx_.insert(rx_.end(), data, data + len);
    }
    drain_();
}

void FrameParser::feed(const std::vector<uint8_t>& bytes) {
    feed(bytes.data(), bytes.size());
}

bool FrameParser::try_pop(Frame& out) {
    if (ready_.empty()) return false;
    out = std::move(ready_.front());
    ready_.pop_front();
    return true;
}

void FrameParser::reset() {
    rx_.clear();
    ready_.clear();
}

void FrameParser::drain_() {
    std::size_t cursor = 0;

    while (cursor < rx_.size()) {
        ParseOutcome r = try_parse_frame(rx_.data() + cursor,
                                         rx_.size() - cursor);
        if (r.status == ParseStatus::Success) {
            ready_.push_back(std::move(r.frame));
            cursor += r.consumed;
        } else if (r.status == ParseStatus::Resync) {
            cursor += (r.consumed > 0 ? r.consumed : 1);
        } else {  // NeedMoreData
            break;
        }
    }

    // Drop everything we consumed from the front of the buffer. We use a
    // single erase rather than per-byte pops to keep this O(n) overall.
    if (cursor > 0) {
        rx_.erase(rx_.begin(), rx_.begin() + static_cast<std::ptrdiff_t>(cursor));
    }

    // Bound the resync buffer so a stream of garbage cannot exhaust memory.
    if (rx_.size() > max_buffered_) {
        const std::size_t excess = rx_.size() - max_buffered_;
        rx_.erase(rx_.begin(), rx_.begin() + static_cast<std::ptrdiff_t>(excess));
    }
}

}  // namespace xense::taccap::bus
