// Copyright (c) 2026 XenseRobotics Co., Ltd. — Apache-2.0
//
// TC-GU-01 framing layer. Implements the wire format defined in the
// firmware: HEAD(0xAA) | ADDR | SEQ | TYPE | CMD | LEN(2 LE) | PAYLOAD
//          | CRC(2 LE) | TAIL(0x55)
//
// Mirror of Embedded Software/tc-gu-01/App/protocol/protocol_frame.{h,c}.

#pragma once

#include <taccap/protocol/commands.hpp>

#include <cstdint>
#include <cstddef>
#include <deque>
#include <vector>

namespace xense::taccap::bus {

constexpr uint8_t FRAME_HEAD       = 0xAA;
constexpr uint8_t FRAME_TAIL       = 0x55;
constexpr uint8_t FRAME_ESCAPE     = 0x7D;

constexpr std::size_t FRAME_HEADER_LEN = 7;   // HEAD + ADDR + SEQ + TYPE + CMD + LEN(2)
constexpr std::size_t FRAME_TAIL_LEN   = 3;   // CRC(2) + TAIL(1)
constexpr std::size_t MIN_FRAME_LEN    = FRAME_HEADER_LEN + FRAME_TAIL_LEN;  // 10

// MAX_FRAME_LEN bounds the largest wire-format frame the SDK will
// pack or accept. Set to match firmware (`tc-gu-01/App/protocol/
// protocol_frame.h #define FRAME_MAX_LEN 2304`), sized for V1.3 OTA
// write blocks: payload = 6-byte OtaWriteBlockHeader + up to
// OTA_BLOCK_SIZE=1024 bytes of firmware data = 1030 payload bytes,
// plus 7-byte header + 3-byte tail = 1040 unstuffed; worst-case
// byte-stuffing roughly doubles that, so 2304 leaves headroom.
constexpr std::size_t MAX_FRAME_LEN    = 2304;
constexpr std::size_t MAX_PAYLOAD_LEN  = MAX_FRAME_LEN - MIN_FRAME_LEN;

// CRC-16/MODBUS (init 0xFFFF, poly 0xA001 with reflected input/output).
// Matches firmware protocol_calc_crc().
uint16_t crc16_modbus(const uint8_t* data, std::size_t len) noexcept;

// Optional byte stuffing (escape FRAME_HEAD / FRAME_TAIL / FRAME_ESCAPE inside
// payload as ESC, byte^0x20). Firmware ships these helpers but does NOT call
// them in pack/unpack today, so we mirror them only — pack_frame/try_parse_frame
// below intentionally bypass stuffing.
std::vector<uint8_t> stuff_data(const uint8_t* in, std::size_t len);
std::vector<uint8_t> unstuff_data(const uint8_t* in, std::size_t len);

// Decoded frame.
struct Frame {
    protocol::Address    addr;
    uint8_t              seq;
    protocol::FrameType  type;
    protocol::Cmd        cmd;
    std::vector<uint8_t> payload;
};

// Build a wire frame.
//
// Throws ProtocolError if payload_len > MAX_PAYLOAD_LEN.
std::vector<uint8_t> pack_frame(
    protocol::Address addr,
    uint8_t           seq,
    protocol::FrameType type,
    protocol::Cmd     cmd,
    const uint8_t*    payload     = nullptr,
    std::size_t       payload_len = 0);

inline std::vector<uint8_t> pack_frame(
        protocol::Address addr, uint8_t seq, protocol::FrameType type,
        protocol::Cmd cmd, const std::vector<uint8_t>& payload) {
    return pack_frame(addr, seq, type, cmd, payload.data(), payload.size());
}

// Single-shot parse outcome.
//   Success      : `frame` populated, advance buffer by `consumed`
//   NeedMoreData : `consumed` is 0; caller should buffer more bytes
//   Resync       : current first byte cannot start a valid frame; caller
//                  should drop one byte (or scan to next 0xAA) and retry.
//                  `consumed` may be >0 if we definitively know how many
//                  bytes are bad, otherwise 1.
enum class ParseStatus { Success, NeedMoreData, Resync };

struct ParseOutcome {
    ParseStatus  status;
    std::size_t  consumed;
    Frame        frame;   // populated when status == Success
};

ParseOutcome try_parse_frame(const uint8_t* data, std::size_t len);

// Streaming, allocation-friendly frame parser.
// Feed bytes incrementally; pop complete frames as they become available.
// Bounded by max_buffered: if the working buffer grows past that without
// producing a frame, the oldest unsynced bytes are dropped.
class FrameParser {
public:
    explicit FrameParser(std::size_t max_buffered = 64 * 1024);

    void feed(const uint8_t* data, std::size_t len);
    void feed(const std::vector<uint8_t>& bytes);

    // Consume the next ready frame, if any.
    bool try_pop(Frame& out);

    // Number of complete frames currently queued.
    std::size_t pending() const noexcept { return ready_.size(); }
    // Bytes currently held in the resync buffer (not yet a complete frame).
    std::size_t buffered_bytes() const noexcept { return rx_.size(); }

    void reset();

private:
    void drain_();

    std::vector<uint8_t> rx_;
    std::deque<Frame>    ready_;
    std::size_t          max_buffered_;
};

}  // namespace xense::taccap::bus
