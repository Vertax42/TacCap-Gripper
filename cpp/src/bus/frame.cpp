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

    // Build the *unescaped* frame first: HEAD | ADDR | SEQ | TYPE | CMD |
    // LEN(2) | PAYLOAD | CRC(2). The CRC is computed over HEAD..PAYLOAD
    // (firmware convention) BEFORE byte-stuffing — protocol V1.8.
    std::vector<uint8_t> u;
    u.reserve(FRAME_HEADER_LEN + payload_len + 2);
    u.push_back(FRAME_HEAD);
    u.push_back(static_cast<uint8_t>(addr));
    u.push_back(seq);
    u.push_back(static_cast<uint8_t>(type));
    u.push_back(static_cast<uint8_t>(cmd));
    u.push_back(static_cast<uint8_t>(payload_len & 0xFF));
    u.push_back(static_cast<uint8_t>((payload_len >> 8) & 0xFF));
    if (payload && payload_len) {
        u.insert(u.end(), payload, payload + payload_len);
    }
    const uint16_t crc = crc16_modbus(u.data(), u.size());  // over HEAD..PAYLOAD
    u.push_back(static_cast<uint8_t>(crc & 0xFF));
    u.push_back(static_cast<uint8_t>((crc >> 8) & 0xFF));

    // Wire frame (V1.8 global escaping): HEAD | stuff(ADDR..CRC) | TAIL.
    // HEAD/TAIL stay raw so they remain unambiguous frame delimiters; every
    // 0xAA/0x55/0x7D in the body is escaped to ESC,(byte^0x20).
    const std::vector<uint8_t> body = stuff_data(u.data() + 1, u.size() - 1);
    std::vector<uint8_t> out;
    out.reserve(2 + body.size());
    out.push_back(FRAME_HEAD);
    out.insert(out.end(), body.begin(), body.end());
    out.push_back(FRAME_TAIL);
    return out;
}

ParseOutcome try_parse_frame(const uint8_t* data, std::size_t len) {
    // Wire format (protocol V1.8): HEAD | stuff(ADDR..CRC) | TAIL. Byte
    // stuffing escapes every 0xAA/0x55/0x7D in the body, so a *raw* HEAD only
    // appears at a frame start and a *raw* TAIL only at a frame end — the
    // frame is delimited by the first raw TAIL after HEAD, and the body must
    // be unstuffed before LEN/CRC make sense.
    ParseOutcome r{};
    if (data == nullptr || len == 0) {
        r.status = ParseStatus::NeedMoreData;
        return r;
    }
    if (data[0] != FRAME_HEAD) {
        r.status = ParseStatus::Resync;
        std::size_t skip = 1;
        while (skip < len && data[skip] != FRAME_HEAD) ++skip;
        r.consumed = skip;
        return r;
    }

    // Locate the terminating raw TAIL.
    std::size_t tail = 1;
    while (tail < len && data[tail] != FRAME_TAIL) ++tail;
    if (tail >= len) {
        // No terminator yet. If the run already exceeds a max-size frame,
        // this HEAD can't begin a valid frame — drop it and resync.
        if (len > MAX_FRAME_LEN) {
            r.status = ParseStatus::Resync;
            r.consumed = 1;
            return r;
        }
        r.status = ParseStatus::NeedMoreData;
        return r;
    }

    // Unstuff the body between HEAD and TAIL -> ADDR | SEQ | TYPE | CMD |
    // LEN(2) | PAYLOAD | CRC(2).
    const std::vector<uint8_t> body = unstuff_data(data + 1, tail - 1);

    // Minimum unescaped body = ADDR+SEQ+TYPE+CMD+LEN(2)+CRC(2) = 8 bytes.
    constexpr std::size_t BODY_MIN = 8;
    if (body.size() < BODY_MIN) {
        r.status = ParseStatus::Resync;
        r.consumed = 1;
        return r;
    }

    const std::size_t payload_len =
        static_cast<std::size_t>(body[4]) |
        (static_cast<std::size_t>(body[5]) << 8);

    // LEN must agree with the actual unstuffed body length.
    if (payload_len > MAX_PAYLOAD_LEN || body.size() != BODY_MIN + payload_len) {
        r.status = ParseStatus::Resync;
        r.consumed = 1;
        return r;
    }

    // CRC is over HEAD..PAYLOAD on the *unescaped* bytes. Reconstruct that
    // region as HEAD followed by the unstuffed ADDR..PAYLOAD.
    const std::size_t crc_body = 6 + payload_len;   // ADDR..PAYLOAD within body
    std::vector<uint8_t> crc_region;
    crc_region.reserve(1 + crc_body);
    crc_region.push_back(FRAME_HEAD);
    crc_region.insert(crc_region.end(), body.begin(), body.begin() + crc_body);
    const uint16_t crc_calc = crc16_modbus(crc_region.data(), crc_region.size());
    const uint16_t crc_recv =
        static_cast<uint16_t>(body[crc_body]) |
        (static_cast<uint16_t>(body[crc_body + 1]) << 8);
    if (crc_calc != crc_recv) {
        r.status = ParseStatus::Resync;
        r.consumed = 1;
        return r;
    }

    r.status        = ParseStatus::Success;
    r.consumed      = tail + 1;   // HEAD + stuffed body + TAIL on the wire
    r.frame.addr    = static_cast<protocol::Address>(body[0]);
    r.frame.seq     = body[1];
    r.frame.type    = static_cast<protocol::FrameType>(body[2]);
    r.frame.cmd     = static_cast<protocol::Cmd>(body[3]);
    r.frame.payload.assign(body.begin() + 6, body.begin() + 6 + payload_len);
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
            continue;
        }
        if (r.status == ParseStatus::Resync) {
            cursor += (r.consumed > 0 ? r.consumed : 1);
            continue;
        }
        // NeedMoreData at cursor. Before giving up, see if any LATER 0xAA
        // in the buffer can be parsed as a complete frame. With V1.8 byte
        // stuffing a raw 0xAA is only ever a real HEAD, but line noise (or a
        // partial/garbage frame) can still leave a stray 0xAA at the front
        // with no terminator while a real frame's HEAD sits further in. Walk
        // past every 0xAA after `cursor` and attempt a parse; if any succeeds
        // (TAIL found, body unstuffs, CRC valid), commit it and drop the
        // bytes before its HEAD as garbage.
        bool recovered = false;
        std::size_t alt = cursor + 1;
        while (alt < rx_.size()) {
            if (rx_[alt] != FRAME_HEAD) { ++alt; continue; }
            ParseOutcome r2 = try_parse_frame(rx_.data() + alt,
                                              rx_.size() - alt);
            if (r2.status == ParseStatus::Success) {
                ready_.push_back(std::move(r2.frame));
                cursor = alt + r2.consumed;
                recovered = true;
                break;
            }
            if (r2.status == ParseStatus::Resync) {
                alt += (r2.consumed > 0 ? r2.consumed : 1);
                continue;
            }
            // r2 = NeedMoreData at alt. Try the next 0xAA.
            ++alt;
        }
        if (recovered) continue;
        // No later HEAD can be completed yet; honestly wait for more bytes.
        break;
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
