// Copyright (c) 2026 XenseRobotics Co., Ltd. — Apache-2.0
//
// V2.2 protocol codec tests: the follower-only startup limit torque (Cmd 0x3A /
// 0x3B), the 64-byte motor fault report (Cmd 0x52), the 72-byte extended motor
// status (Cmd 0x53) and the short partial-write forms of the auto-cal config
// (Cmd 0x68). Pure codec — no hardware needed.
//
// Byte offsets are transcribed from firmware docs/PROTOCOL.md §10 (the offset
// table for the 0x53 payload) and App/protocol/protocol_data.h, which are
// canonical. They are cross-checked against the PC tool's struct format strings
// in tc-gu-01-pc/core/protocol.py (parse_motor_status /
// parse_motor_fault_report) — three independent transcriptions of one layout.
//
// The load-bearing property of V2.2 is that it is PURELY ADDITIVE: 0x50 and the
// MotorStatus DATA stream still carry the 31-byte layout. MotorStatusExtSharesThe
// LegacyPrefix below is what fails if a future change breaks that.

#include <cstring>
#include <gtest/gtest.h>
#include <taccap/error.hpp>
#include <taccap/protocol/codec.hpp>
#include <taccap/protocol/commands.hpp>
#include <taccap/protocol/payloads.hpp>

namespace tp = xense::taccap::protocol;
using xense::taccap::ProtocolError;

namespace {

// A fully-populated 72-byte 0x53 payload with every field set to a distinct
// recognisable value, so a mis-set offset shows up as a wrong number rather
// than a coincidentally-matching zero.
std::vector<uint8_t> make_status_ext_wire() {
    std::vector<uint8_t> w(tp::MOTOR_STATUS_EXT_SIZE, 0);
    const float    actual_pos = 0.75f, actual_vel = -1.25f;
    const float    actual_torque = 0.5f, motor_temp = 42.5f;
    const uint16_t status = 0x0803;   // Enabled | Fault | EncoderUncalibrated
    const float    target_pos = 0.8f, target_vel = 1.0f, target_torque = 0.35f;

    std::memcpy(w.data() +  0, &actual_pos,    4);
    std::memcpy(w.data() +  4, &actual_vel,    4);
    std::memcpy(w.data() +  8, &actual_torque, 4);
    std::memcpy(w.data() + 12, &motor_temp,    4);
    std::memcpy(w.data() + 16, &status,        2);
    std::memcpy(w.data() + 18, &target_pos,    4);
    std::memcpy(w.data() + 22, &target_vel,    4);
    std::memcpy(w.data() + 26, &target_torque, 4);
    w[30] = 4;  // control_mode = Impedance

    // Monitor extension (offsets straight out of the PROTOCOL.md §10 table).
    w[31] = tp::MOTOR_MONITOR_VERSION;
    w[32] = tp::MotorMonitorFlag::StatusValid | tp::MotorMonitorFlag::FaultValid |
            tp::MotorMonitorFlag::FaultLatched;
    w[33] = static_cast<uint8_t>(tp::MotorStopReason::LimitStall);
    w[34] = tp::MotorMonitorDiag::RxSeen | tp::MotorMonitorDiag::CanFrameSeen;

    const uint32_t fault_code = tp::MotorFaultBit::StallOverload;
    const uint32_t latched    = tp::MotorFaultBit::StallOverload |
                                tp::MotorFaultBit::OverTemp;
    const uint32_t fault_ts = 0x11111111u, status_ts = 0x22222222u;
    const uint32_t stop_fault = tp::MotorFaultBit::StallOverload;
    const uint32_t stop_ts = 0x33333333u, can_id = 0xFDu;
    std::memcpy(w.data() + 35, &fault_code, 4);
    std::memcpy(w.data() + 39, &latched,    4);
    std::memcpy(w.data() + 43, &fault_ts,   4);
    std::memcpy(w.data() + 47, &status_ts,  4);
    std::memcpy(w.data() + 51, &stop_fault, 4);
    std::memcpy(w.data() + 55, &stop_ts,    4);
    std::memcpy(w.data() + 59, &can_id,     4);
    w[63] = 5;  // fault_can_dlc — a valid cmd-5 reply is >= 5
    // byte0 = motor CAN id, bytes 1..4 = little-endian fault word
    w[64] = 0x7F; w[65] = 0x00; w[66] = 0x40; w[67] = 0x00; w[68] = 0x00;
    return w;
}

}  // namespace

// ---- Wire constants (mirror firmware protocol_cmd.h) ----------------------

TEST(CodecV22, CommandWireValues) {
    EXPECT_EQ(static_cast<uint8_t>(tp::Cmd::MotorSetStartupLimitTorque), 0x3Au);
    EXPECT_EQ(static_cast<uint8_t>(tp::Cmd::MotorGetStartupLimitTorque), 0x3Bu);
    EXPECT_EQ(static_cast<uint8_t>(tp::Cmd::GetMotorFault),              0x52u);
    EXPECT_EQ(static_cast<uint8_t>(tp::Cmd::GetMotorStatusExt),          0x53u);
}

TEST(CodecV22, ToStringCoversNewNames) {
    EXPECT_STREQ(tp::to_string(tp::Cmd::MotorSetStartupLimitTorque),
                 "MotorSetStartupLimitTorque");
    EXPECT_STREQ(tp::to_string(tp::Cmd::MotorGetStartupLimitTorque),
                 "MotorGetStartupLimitTorque");
    EXPECT_STREQ(tp::to_string(tp::Cmd::GetMotorFault),     "GetMotorFault");
    EXPECT_STREQ(tp::to_string(tp::Cmd::GetMotorStatusExt), "GetMotorStatusExt");
}

TEST(CodecV22, NewMotorStatusBits) {
    EXPECT_EQ(tp::MotorStatusBit::DriverFault,         0x0100u);
    EXPECT_EQ(tp::MotorStatusBit::PositionInitError,   0x0200u);
    EXPECT_EQ(tp::MotorStatusBit::HardwareIdError,     0x0400u);
    EXPECT_EQ(tp::MotorStatusBit::EncoderUncalibrated, 0x0800u);
}

TEST(CodecV22, MonitorAndFaultConstants) {
    EXPECT_EQ(tp::MOTOR_MONITOR_VERSION,      0x02u);
    EXPECT_EQ(tp::MOTOR_FAULT_REPORT_VERSION, 0x01u);

    EXPECT_EQ(tp::MotorMonitorFlag::StatusValid,       0x01u);
    EXPECT_EQ(tp::MotorMonitorFlag::FaultStale,        0x80u);
    EXPECT_EQ(tp::MotorMonitorDiag::CanFrameSeen,      0x04u);
    EXPECT_EQ(tp::MotorFaultReportFlag::PrivatePartial, 0x40u);
    EXPECT_EQ(tp::MotorFaultSource::FwSystem,          0x08u);

    EXPECT_EQ(static_cast<uint8_t>(tp::MotorStopReason::ControlError), 0x05u);
    EXPECT_EQ(static_cast<uint32_t>(tp::FirmwareFaultCode::CanBusOff), 0x00000203u);
    EXPECT_EQ(static_cast<uint32_t>(tp::FirmwareFaultCode::ControlLimitStall),
              0x00000305u);

    // Bit positions in the raw motor fault word (firmware docs §9).
    EXPECT_EQ(tp::MotorFaultBit::OverTemp,            1u << 0);
    EXPECT_EQ(tp::MotorFaultBit::EncoderUncalibrated, 1u << 7);
    EXPECT_EQ(tp::MotorFaultBit::StallOverload,       1u << 14);
    EXPECT_EQ(tp::MotorFaultBit::PhaseAOverCurrent,   1u << 16);
}

// ---- Struct sizes and offsets --------------------------------------------

TEST(CodecV22, StructSizes) {
    EXPECT_EQ(tp::MOTOR_STATUS_LEGACY_SIZE, 31u);
    EXPECT_EQ(tp::MOTOR_STATUS_V2_SIZE,     59u);
    EXPECT_EQ(tp::MOTOR_STATUS_EXT_SIZE,    72u);
    EXPECT_EQ(sizeof(tp::MotorStatus),      31u);
    EXPECT_EQ(sizeof(tp::MotorStatusExt),   72u);
    EXPECT_EQ(sizeof(tp::MotorFaultReport), 64u);
    EXPECT_EQ(sizeof(tp::GripperAutoCalStallParam),   10u);
    EXPECT_EQ(sizeof(tp::GripperAutoCalStallParamEx), 16u);
}

// Offsets from firmware docs/PROTOCOL.md §10.
TEST(CodecV22, MotorStatusExtFieldOffsets) {
    EXPECT_EQ(offsetof(tp::MotorStatusExt, monitor_version),     31u);
    EXPECT_EQ(offsetof(tp::MotorStatusExt, monitor_flags),       32u);
    EXPECT_EQ(offsetof(tp::MotorStatusExt, stop_reason),         33u);
    EXPECT_EQ(offsetof(tp::MotorStatusExt, monitor_reserved),    34u);
    EXPECT_EQ(offsetof(tp::MotorStatusExt, fault_code),          35u);
    EXPECT_EQ(offsetof(tp::MotorStatusExt, latched_fault_code),  39u);
    EXPECT_EQ(offsetof(tp::MotorStatusExt, fault_timestamp_ms),  43u);
    EXPECT_EQ(offsetof(tp::MotorStatusExt, status_timestamp_ms), 47u);
    EXPECT_EQ(offsetof(tp::MotorStatusExt, stop_fault_code),     51u);
    EXPECT_EQ(offsetof(tp::MotorStatusExt, stop_timestamp_ms),   55u);
    EXPECT_EQ(offsetof(tp::MotorStatusExt, fault_can_id),        59u);
    EXPECT_EQ(offsetof(tp::MotorStatusExt, fault_can_dlc),       63u);
    EXPECT_EQ(offsetof(tp::MotorStatusExt, fault_can_data),      64u);
}

// Offsets implied by the PC tool's '<BBBBIIIIIIIiIBBBB8sIIB3s'.
TEST(CodecV22, MotorFaultReportFieldOffsets) {
    EXPECT_EQ(offsetof(tp::MotorFaultReport, version),                      0u);
    EXPECT_EQ(offsetof(tp::MotorFaultReport, report_flags),                 1u);
    EXPECT_EQ(offsetof(tp::MotorFaultReport, source_mask),                  2u);
    EXPECT_EQ(offsetof(tp::MotorFaultReport, protocol_mode),                3u);
    EXPECT_EQ(offsetof(tp::MotorFaultReport, event_seq),                    4u);
    EXPECT_EQ(offsetof(tp::MotorFaultReport, timestamp_ms),                 8u);
    EXPECT_EQ(offsetof(tp::MotorFaultReport, motor_fault_code),            12u);
    EXPECT_EQ(offsetof(tp::MotorFaultReport, motor_latched_fault_code),    16u);
    EXPECT_EQ(offsetof(tp::MotorFaultReport, stop_fault_code),             20u);
    EXPECT_EQ(offsetof(tp::MotorFaultReport, firmware_fault_code),         24u);
    EXPECT_EQ(offsetof(tp::MotorFaultReport, firmware_latched_fault_code), 28u);
    EXPECT_EQ(offsetof(tp::MotorFaultReport, firmware_detail_code),        32u);
    EXPECT_EQ(offsetof(tp::MotorFaultReport, fault_can_id),                36u);
    EXPECT_EQ(offsetof(tp::MotorFaultReport, fault_can_dlc),               40u);
    EXPECT_EQ(offsetof(tp::MotorFaultReport, monitor_flags),               41u);
    EXPECT_EQ(offsetof(tp::MotorFaultReport, monitor_reserved),            42u);
    EXPECT_EQ(offsetof(tp::MotorFaultReport, fault_can_data),              44u);
    EXPECT_EQ(offsetof(tp::MotorFaultReport, fault_timestamp_ms),          52u);
    EXPECT_EQ(offsetof(tp::MotorFaultReport, status_timestamp_ms),         56u);
    EXPECT_EQ(offsetof(tp::MotorFaultReport, stop_reason),                 60u);
}

// ---- Extended motor status (Cmd 0x53) ------------------------------------

TEST(CodecV22, DecodeStatusExtReadsEveryField) {
    auto w = make_status_ext_wire();
    auto s = tp::decode_motor_status_ext(w.data(), w.size());

    EXPECT_FLOAT_EQ(s.actual_pos,    0.75f);
    EXPECT_FLOAT_EQ(s.actual_vel,   -1.25f);
    EXPECT_FLOAT_EQ(s.actual_torque, 0.5f);
    EXPECT_FLOAT_EQ(s.motor_temp,   42.5f);
    EXPECT_EQ(s.status,            0x0803u);
    EXPECT_FLOAT_EQ(s.target_pos,    0.8f);
    EXPECT_FLOAT_EQ(s.target_vel,    1.0f);
    EXPECT_FLOAT_EQ(s.target_torque, 0.35f);
    EXPECT_EQ(s.control_mode, 4u);

    EXPECT_EQ(s.monitor_version, tp::MOTOR_MONITOR_VERSION);
    EXPECT_TRUE(s.monitor_flags & tp::MotorMonitorFlag::StatusValid);
    EXPECT_TRUE(s.monitor_flags & tp::MotorMonitorFlag::FaultLatched);
    EXPECT_FALSE(s.monitor_flags & tp::MotorMonitorFlag::CommStale);
    EXPECT_EQ(s.stop_reason,
              static_cast<uint8_t>(tp::MotorStopReason::LimitStall));
    EXPECT_TRUE(s.monitor_reserved & tp::MotorMonitorDiag::RxSeen);
    EXPECT_FALSE(s.monitor_reserved & tp::MotorMonitorDiag::TimeoutSeen);

    EXPECT_EQ(s.fault_code,          tp::MotorFaultBit::StallOverload);
    EXPECT_EQ(s.latched_fault_code,
              tp::MotorFaultBit::StallOverload | tp::MotorFaultBit::OverTemp);
    EXPECT_EQ(s.fault_timestamp_ms,  0x11111111u);
    EXPECT_EQ(s.status_timestamp_ms, 0x22222222u);
    EXPECT_EQ(s.stop_fault_code,     tp::MotorFaultBit::StallOverload);
    EXPECT_EQ(s.stop_timestamp_ms,   0x33333333u);

    EXPECT_EQ(s.fault_can_id,  0xFDu);
    EXPECT_EQ(s.fault_can_dlc, 5u);
    EXPECT_EQ(s.fault_can_data[0], 0x7Fu);   // motor CAN id
    // bytes 1..4 are the same fault word, little-endian
    uint32_t raw = 0;
    std::memcpy(&raw, s.fault_can_data + 1, 4);
    EXPECT_EQ(raw, tp::MotorFaultBit::StallOverload);
}

// The whole point of V2.2: the first 31 bytes of the ext payload ARE the
// legacy payload. If this ever fails, 0x50 and the DATA stream have diverged
// from 0x53 and every host that shares a decode path is silently wrong.
TEST(CodecV22, MotorStatusExtSharesTheLegacyPrefix) {
    auto w = make_status_ext_wire();
    auto legacy = tp::decode_motor_status(w.data(), tp::MOTOR_STATUS_LEGACY_SIZE);
    auto ext    = tp::decode_motor_status_ext(w.data(), w.size());

    EXPECT_FLOAT_EQ(legacy.actual_pos,    ext.actual_pos);
    EXPECT_FLOAT_EQ(legacy.actual_vel,    ext.actual_vel);
    EXPECT_FLOAT_EQ(legacy.actual_torque, ext.actual_torque);
    EXPECT_FLOAT_EQ(legacy.motor_temp,    ext.motor_temp);
    EXPECT_EQ(legacy.status,              ext.status);
    EXPECT_FLOAT_EQ(legacy.target_pos,    ext.target_pos);
    EXPECT_FLOAT_EQ(legacy.target_vel,    ext.target_vel);
    EXPECT_FLOAT_EQ(legacy.target_torque, ext.target_torque);
    EXPECT_EQ(legacy.control_mode,        ext.control_mode);

    // Same claim at the byte level, independent of field-by-field reads.
    EXPECT_EQ(0, std::memcmp(&legacy, &ext, tp::MOTOR_STATUS_LEGACY_SIZE));
}

TEST(CodecV22, DecodeStatusExtAcceptsShortPrefixesAndZeroFills) {
    auto w = make_status_ext_wire();

    // 31 bytes — exactly what 0x50 and the DATA stream carry.
    auto s31 = tp::decode_motor_status_ext(w.data(), tp::MOTOR_STATUS_LEGACY_SIZE);
    EXPECT_FLOAT_EQ(s31.actual_pos, 0.75f);
    EXPECT_EQ(s31.monitor_version, 0u);   // zero-filled, not garbage
    EXPECT_EQ(s31.fault_code,      0u);
    EXPECT_EQ(s31.fault_can_dlc,   0u);

    // 59 bytes — monitor extension without the raw-CAN tail.
    auto s59 = tp::decode_motor_status_ext(w.data(), tp::MOTOR_STATUS_V2_SIZE);
    EXPECT_EQ(s59.monitor_version,   tp::MOTOR_MONITOR_VERSION);
    EXPECT_EQ(s59.fault_code,        tp::MotorFaultBit::StallOverload);
    EXPECT_EQ(s59.stop_timestamp_ms, 0x33333333u);
    EXPECT_EQ(s59.fault_can_id,      0u);  // tail zero-filled
    EXPECT_EQ(s59.fault_can_dlc,     0u);
}

TEST(CodecV22, DecodeStatusExtRejectsShorterThanLegacy) {
    std::vector<uint8_t> too_short(tp::MOTOR_STATUS_LEGACY_SIZE - 1, 0);
    EXPECT_THROW(tp::decode_motor_status_ext(too_short.data(), too_short.size()),
                 ProtocolError);
    EXPECT_THROW(tp::decode_motor_status_ext(nullptr, tp::MOTOR_STATUS_EXT_SIZE),
                 ProtocolError);
}

// A longer-than-expected frame must not overrun the struct.
TEST(CodecV22, DecodeStatusExtIgnoresTrailingBytes) {
    auto w = make_status_ext_wire();
    w.resize(tp::MOTOR_STATUS_EXT_SIZE + 16, 0xAB);
    auto s = tp::decode_motor_status_ext(w.data(), w.size());
    EXPECT_EQ(s.fault_can_dlc, 5u);
    EXPECT_EQ(s.fault_can_data[7], 0u);   // still the payload's own zero
}

// ---- Motor fault report (Cmd 0x52) ---------------------------------------

TEST(CodecV22, DecodeFaultReportRoundtrip) {
    tp::MotorFaultReport in{};
    in.version       = tp::MOTOR_FAULT_REPORT_VERSION;
    in.report_flags  = tp::MotorFaultReportFlag::Valid |
                       tp::MotorFaultReportFlag::MotorValid |
                       tp::MotorFaultReportFlag::ForceRead;
    in.source_mask   = tp::MotorFaultSource::Motor | tp::MotorFaultSource::FwControl;
    in.protocol_mode = static_cast<uint8_t>(tp::MotorProtocol::Mit);
    in.event_seq     = 7;
    in.timestamp_ms  = 0x00ABCDEFu;
    in.motor_fault_code         = tp::MotorFaultBit::StallOverload;
    in.motor_latched_fault_code = tp::MotorFaultBit::StallOverload |
                                  tp::MotorFaultBit::UnderVolt;
    in.stop_fault_code          = tp::MotorFaultBit::StallOverload;
    in.firmware_fault_code =
        static_cast<uint32_t>(tp::FirmwareFaultCode::ControlLimitStall);
    in.firmware_latched_fault_code =
        static_cast<uint32_t>(tp::FirmwareFaultCode::ControlDeadline);
    in.firmware_detail_code = -22;     // signed — an errno, not a bit mask
    in.fault_can_id  = 0xFDu;
    in.fault_can_dlc = 5;
    in.monitor_flags = tp::MotorMonitorFlag::FaultValid;
    in.stop_reason   = static_cast<uint8_t>(tp::MotorStopReason::LimitStall);
    in.fault_can_data[0] = 0x7F;

    std::vector<uint8_t> wire(sizeof(in));
    std::memcpy(wire.data(), &in, sizeof(in));
    ASSERT_EQ(wire.size(), 64u);

    auto out = tp::decode_motor_fault_report(wire.data(), wire.size());
    EXPECT_EQ(out.version,      tp::MOTOR_FAULT_REPORT_VERSION);
    EXPECT_TRUE(out.report_flags & tp::MotorFaultReportFlag::ForceRead);
    EXPECT_TRUE(out.source_mask & tp::MotorFaultSource::FwControl);
    EXPECT_EQ(out.event_seq,    7u);
    EXPECT_EQ(out.motor_fault_code, tp::MotorFaultBit::StallOverload);
    EXPECT_EQ(out.firmware_fault_code,
              static_cast<uint32_t>(tp::FirmwareFaultCode::ControlLimitStall));
    EXPECT_EQ(out.firmware_detail_code, -22);
    EXPECT_EQ(out.fault_can_id,  0xFDu);
    EXPECT_EQ(out.stop_reason,
              static_cast<uint8_t>(tp::MotorStopReason::LimitStall));
    EXPECT_EQ(0, std::memcmp(&in, &out, sizeof(in)));
}

TEST(CodecV22, DecodeFaultReportRejectsWrongLength) {
    std::vector<uint8_t> short_buf(63, 0);
    EXPECT_THROW(tp::decode_motor_fault_report(short_buf.data(), short_buf.size()),
                 ProtocolError);
    // A bare 1-byte error code must not be mistaken for a report.
    std::vector<uint8_t> nack{static_cast<uint8_t>(tp::ErrorCode::SysBusy)};
    EXPECT_THROW(tp::decode_motor_fault_report(nack.data(), nack.size()),
                 ProtocolError);
}

// ---- Auto-cal partial writes (Cmd 0x68 short forms) -----------------------

TEST(CodecV22, StallParamEncodesToTenBytes) {
    tp::GripperAutoCalStallParam p{};
    p.close_stall_torque_nm = 0.22f;
    p.open_stall_torque_nm  = 0.20f;
    p.stall_hold_ms         = 120;

    auto wire = tp::encode(p);
    ASSERT_EQ(wire.size(), 10u);
    float f = 0.0f;
    std::memcpy(&f, wire.data() + 0, 4); EXPECT_FLOAT_EQ(f, 0.22f);
    std::memcpy(&f, wire.data() + 4, 4); EXPECT_FLOAT_EQ(f, 0.20f);
    uint16_t hold = 0;
    std::memcpy(&hold, wire.data() + 8, 2); EXPECT_EQ(hold, 120u);
}

TEST(CodecV22, StallParamExEncodesToSixteenBytes) {
    tp::GripperAutoCalStallParamEx p{};
    p.close_stall_torque_nm = 0.35f;
    p.open_stall_torque_nm  = 0.35f;
    p.stall_hold_ms         = 30;
    p.startup_delay_ms      = 300;
    p.post_zero_delay_ms    = 100;
    p.close_confirm_count   = 1;
    p.open_confirm_count    = 1;

    auto wire = tp::encode(p);
    ASSERT_EQ(wire.size(), 16u);
    uint16_t v = 0;
    std::memcpy(&v, wire.data() +  8, 2); EXPECT_EQ(v,  30u);
    std::memcpy(&v, wire.data() + 10, 2); EXPECT_EQ(v, 300u);
    std::memcpy(&v, wire.data() + 12, 2); EXPECT_EQ(v, 100u);
    EXPECT_EQ(wire[14], 1u);
    EXPECT_EQ(wire[15], 1u);
}

// The short forms share their leading fields with the full 32-byte config, so
// the firmware can copy a prefix straight across.
TEST(CodecV22, StallParamPrefixMatchesFullConfigLayout) {
    EXPECT_EQ(offsetof(tp::GripperAutoCalStallParamEx, close_stall_torque_nm), 0u);
    EXPECT_EQ(offsetof(tp::GripperAutoCalStallParamEx, open_stall_torque_nm),  4u);
    EXPECT_EQ(offsetof(tp::GripperAutoCalStallParamEx, stall_hold_ms),         8u);

    // …and the 10-byte form is a strict prefix of the 16-byte one.
    tp::GripperAutoCalStallParamEx ex{};
    ex.close_stall_torque_nm = 1.5f;
    ex.open_stall_torque_nm  = 2.5f;
    ex.stall_hold_ms         = 77;
    tp::GripperAutoCalStallParam p{};
    p.close_stall_torque_nm = 1.5f;
    p.open_stall_torque_nm  = 2.5f;
    p.stall_hold_ms         = 77;
    EXPECT_EQ(0, std::memcmp(&p, &ex, sizeof(p)));
}
