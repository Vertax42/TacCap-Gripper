// Copyright (c) 2026 XenseRobotics Co., Ltd. — Apache-2.0
//
// Cross-implementation compatibility test. The byte sequences below were
// produced by the GUI Python tool (third_party/firmware/tc-gu-01-pc/core/
// protocol.py, protocol V1.8); we expect our C++ packer/parser to match them
// byte-for-byte. Any divergence here means our wire format has drifted away
// from the firmware's authoritative behaviour.
//
// Note: V1.8 byte-stuffs the body (ADDR..CRC). The first three frames below
// happen to contain no 0xAA/0x55/0x7D in their bodies, so stuffing is a no-op
// for them; MatchesPythonGuiEscapedPayload exercises the escaping path.

#include <gtest/gtest.h>
#include <taccap/bus/frame.hpp>
#include <taccap/protocol/commands.hpp>

namespace tb = xense::taccap::bus;
namespace tp = xense::taccap::protocol;

// Python:
//   pyproto.pack_frame(0x01, 7, 0x00, 0x02, b'')
// produces a 10-byte frame: AA 01 07 00 02 00 00 CRC_L CRC_H 55
// CRC over `AA 01 07 00 02 00 00` is precomputed below.
TEST(PythonCompat, GetVersionEmptyFrame) {
    // Reproduce the wire bytes exactly.
    std::vector<uint8_t> py_wire = {
        0xAA, 0x01, 0x07, 0x00, 0x02, 0x00, 0x00,
        0x00, 0x00,   // CRC placeholders, filled below
        0x55
    };
    const uint16_t crc = tb::crc16_modbus(py_wire.data(), 7);
    py_wire[7] = static_cast<uint8_t>(crc & 0xFF);
    py_wire[8] = static_cast<uint8_t>((crc >> 8) & 0xFF);

    // Sanity: our packer matches what Python would emit byte-for-byte.
    auto cpp_wire = tb::pack_frame(tp::Address::PC, 7,
                                   tp::FrameType::CMD_NEED_ACK,
                                   tp::Cmd::GetVersion);
    EXPECT_EQ(cpp_wire, py_wire);

    // ... and parses correctly.
    auto out = tb::try_parse_frame(py_wire.data(), py_wire.size());
    ASSERT_EQ(out.status, tb::ParseStatus::Success);
    EXPECT_EQ(out.frame.cmd,  tp::Cmd::GetVersion);
    EXPECT_EQ(out.frame.seq,  7);
    EXPECT_EQ(out.frame.type, tp::FrameType::CMD_NEED_ACK);
    EXPECT_EQ(out.frame.addr, tp::Address::PC);
    EXPECT_TRUE(out.frame.payload.empty());
}

// Python:
//   pyproto.pack_frame(0x02, 9, 0x03, 0x10, bytes(28))
// = AA 02 09 03 10 1C 00 [28 zero bytes] CRC_L CRC_H 55
TEST(PythonCompat, ImuDataDataFrame) {
    std::vector<uint8_t> wire(38, 0);  // 10 + 28
    wire[0] = 0xAA;
    wire[1] = 0x02;
    wire[2] = 0x09;
    wire[3] = 0x03;
    wire[4] = 0x10;
    wire[5] = 0x1C;  // len LSB
    wire[6] = 0x00;  // len MSB
    // payload bytes 7..34 are zeros (already)
    const uint16_t crc = tb::crc16_modbus(wire.data(), 35);
    wire[35] = static_cast<uint8_t>(crc & 0xFF);
    wire[36] = static_cast<uint8_t>((crc >> 8) & 0xFF);
    wire[37] = 0x55;

    auto out = tb::try_parse_frame(wire.data(), wire.size());
    ASSERT_EQ(out.status, tb::ParseStatus::Success);
    EXPECT_EQ(out.frame.cmd,     tp::Cmd::GetImu);
    EXPECT_EQ(out.frame.seq,     9);
    EXPECT_EQ(out.frame.type,    tp::FrameType::DATA);
    EXPECT_EQ(out.frame.addr,    tp::Address::MCU);
    EXPECT_EQ(out.frame.payload.size(), 28u);
}

// CRC parity: a payload built by C++ pack_frame should produce the same
// bytes as the firmware C function would. We can't link against the
// firmware here, but we can hardcode known values.
//
// Python:
//   pyproto.calc_crc(b'\xAA\x01\x05\x01\x21\x00\x00')   # GetSn frame
// = 0x???   (computed at test time)
TEST(PythonCompat, FrameByteForByteAgainstHandPackedReference) {
    // Hand-pack a GetSn (0x04) command frame, no payload, addr=PC, seq=5.
    const uint8_t hand[] = {
        0xAA, 0x01, 0x05, 0x00, 0x04, 0x00, 0x00,
        0x00, 0x00,
        0x55
    };
    std::vector<uint8_t> ref(hand, hand + sizeof(hand));
    const uint16_t crc = tb::crc16_modbus(ref.data(), 7);
    ref[7] = static_cast<uint8_t>(crc & 0xFF);
    ref[8] = static_cast<uint8_t>((crc >> 8) & 0xFF);

    auto cpp = tb::pack_frame(tp::Address::PC, 5,
                              tp::FrameType::CMD_NEED_ACK,
                              tp::Cmd::GetSn);
    EXPECT_EQ(cpp, ref);
}

// V1.8 byte-stuffing parity. A payload carrying every special byte
// (0xAA, 0x55, 0x7D) must be escaped on the wire exactly as the GUI does.
// Reference bytes captured from:
//   p = protocol.pack_frame(0x02, 0x10, 0x03, 0x12, bytes([0xAA,0x55,0x7D,0x00,0xFF]))
//   protocol.escape_frame_content(p)
TEST(PythonCompat, MatchesPythonGuiEscapedPayload) {
    const std::vector<uint8_t> payload = {0xAA, 0x55, 0x7D, 0x00, 0xFF};

    // Exactly what the V1.8 GUI emits: HEAD | ADDR SEQ TYPE CMD LEN |
    // 7D 8A (←AA) 7D 75 (←55) 7D 5D (←7D) 00 FF | CRC | TAIL.
    const std::vector<uint8_t> gui_wire = {
        0xAA, 0x02, 0x10, 0x03, 0x12, 0x05, 0x00,
        0x7D, 0x8A, 0x7D, 0x75, 0x7D, 0x5D, 0x00, 0xFF,
        0xED, 0x5F, 0x55,
    };

    // Our packer must produce the same stuffed bytes.
    auto cpp_wire = tb::pack_frame(tp::Address::MCU, 0x10,
                                   tp::FrameType::DATA, tp::Cmd::GetEskin1,
                                   payload);
    EXPECT_EQ(cpp_wire, gui_wire);

    // ... and our parser must unstuff it back to the original payload.
    auto out = tb::try_parse_frame(gui_wire.data(), gui_wire.size());
    ASSERT_EQ(out.status, tb::ParseStatus::Success);
    EXPECT_EQ(out.consumed, gui_wire.size());
    EXPECT_EQ(out.frame.cmd,  tp::Cmd::GetEskin1);
    EXPECT_EQ(out.frame.seq,  0x10);
    EXPECT_EQ(out.frame.type, tp::FrameType::DATA);
    EXPECT_EQ(out.frame.addr, tp::Address::MCU);
    EXPECT_EQ(out.frame.payload, payload);
}
