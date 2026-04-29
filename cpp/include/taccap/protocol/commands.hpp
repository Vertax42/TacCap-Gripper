// Copyright (c) 2026 XenseRobotics Co., Ltd. — Apache-2.0
//
// TC-GU-01 protocol enumerations. Wire values mirror the firmware's
// authoritative definitions in
//   Embedded Software/tc-gu-01/App/protocol/{protocol_cmd.h,protocol_frame.h}
//
// When in doubt, the firmware source is canonical. The PROTOCOL.md document
// and any host-side Python implementation are secondary.

#pragma once

#include <cstdint>

namespace xense::taccap::protocol {

// Wire address byte. Firmware has a single ADDR_MCU=0x02; the doc breaks it
// into "MCU left/right" = 0x02/0x03 distinguished by a runtime DEV_TYPE
// setting. We accept all three at the wire layer.
enum class Address : uint8_t {
    PC        = 0x01,
    MCU       = 0x02,  // canonical MCU address; left side when DEV_TYPE=LEFT
    MCU_RIGHT = 0x03,  // when DEV_TYPE=RIGHT
};

// Frame type byte. Matches firmware FRAME_TYPE_* macros.
enum class FrameType : uint8_t {
    CMD_NEED_ACK = 0x00,  // command, host expects ACK back
    CMD_NO_ACK   = 0x01,  // command, no response wanted
    ACK          = 0x02,  // ACK / NACK reply (carries AckPayload)
    DATA         = 0x03,  // unsolicited data push (e.g. streaming sensors)
};

// Application command byte. Mirrors firmware protocol_cmd.h exactly.
//
// Note: PROTOCOL.md V1.2 lists MOTOR_SET_ZERO=0x33 but the firmware does NOT
// implement it yet, so it is intentionally absent here. Add it when the
// firmware lands.
enum class Cmd : uint8_t {
    // System (0x00–0x0F)
    Heartbeat       = 0x01,
    GetVersion      = 0x02,
    ResetDevice     = 0x03,
    GetSn           = 0x04,
    SetSn           = 0x05,
    GetDevType      = 0x06,
    SetDevType      = 0x07,

    // Sensor reads (0x10–0x1F)
    GetImu          = 0x10,
    GetEncoder      = 0x11,
    GetEskin1       = 0x12,
    GetEskin2       = 0x13,
    GetAllSensors   = 0x14,

    // Stream control (0x20–0x2F)
    StartStream         = 0x20,
    StopStream          = 0x21,
    SetStreamRate       = 0x22,
    SetStreamMode       = 0x23,
    SetEncoderZero      = 0x24,
    // 0x25 = SetImuCalibration in earlier docs; firmware leaves room here
    // but does not currently expose the command.

    // Motor (0x30–0x4F)
    MotorEnable         = 0x30,
    MotorDisable        = 0x31,
    MotorClearFault     = 0x32,
    MotorPosCtrl        = 0x40,
    MotorVelCtrl        = 0x41,
    MotorTorqueCtrl     = 0x42,
    MotorImpedanceCtrl  = 0x43,
    GetMotorStatus      = 0x50,

    // Config (0x60–0x6F)
    SetImuConfig        = 0x60,
    GetImuConfig        = 0x61,
    SetEncoderConfig    = 0x62,
    GetEncoderConfig    = 0x63,
    SetEskinConfig      = 0x64,
    GetEskinConfig      = 0x65,
};

// Error code byte returned in ACK/NACK frames. Mirrors firmware ERR_*.
enum class ErrorCode : uint8_t {
    Ok              = 0x00,
    InvalidCmd      = 0x01,
    InvalidParam    = 0x02,
    LengthMismatch  = 0x03,
    CrcError        = 0x04,
    Timeout         = 0x05,
    MotorFault      = 0x10,
    SensorOffline   = 0x20,
    SysBusy         = 0x30,
    SeqMismatch     = 0x40,
};

// to_string helpers (returns a stable C string; never null).
const char* to_string(Address) noexcept;
const char* to_string(FrameType) noexcept;
const char* to_string(Cmd) noexcept;
const char* to_string(ErrorCode) noexcept;

}  // namespace xense::taccap::protocol
