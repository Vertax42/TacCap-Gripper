// Copyright (c) 2026 XenseRobotics Co., Ltd. — Apache-2.0
//
// TC-GU-01 protocol enumerations. Wire values mirror the firmware's
// authoritative definitions in
//   third_party/firmware/tc-gu-01/App/protocol/{protocol_cmd.h,protocol_frame.h}
//   (clone-on-demand, see README "Firmware / PC GUI reference repos").
//
// When in doubt, the firmware source is canonical. The PROTOCOL.md document
// and any host-side Python implementation are secondary.
//
// Tracked firmware protocol version: **V1.6** (2026-05-19).

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
    KeyStatus       = 0x15,    // V1.4 — device-side button, payload=key_status_payload_t

    // Stream control + calibration (0x20–0x2F)
    StartStream         = 0x20,
    StopStream          = 0x21,
    SetStreamRate       = 0x22,
    SetStreamMode       = 0x23,
    SetEncoderZero      = 0x24,
    SetImuCal           = 0x25,    // accel/gyro calibration (firmware-side params)
    SetImuMagCal        = 0x26,    // V1.4 — magnetometer hard/soft iron, imu_mag_cal_t (48B)
    SetCalResult        = 0x27,    // V1.5 — write one sensor's cal-success flag, cal_set_payload_t
    SetAllCalResult     = 0x28,    // V1.5 — bulk write cal mask, cal_set_all_payload_t
    GetCalResult        = 0x29,    // V1.5 — read cal-success mask, cal_get_response_t
    SensorErrorReport   = 0x2A,    // V1.6 — DATA-stream-only, sensor_error_report_t (8B)

    // Motor (0x30–0x4F)
    MotorEnable         = 0x30,
    MotorDisable        = 0x31,
    MotorClearFault     = 0x32,
    MotorSetZero        = 0x33,    // V1.7 — zero / mode (0..2-byte payload)
    MotorGetCanId       = 0x34,    // V1.7 — read motor CAN id (resp 1B)
    MotorSetCanId       = 0x35,    // V1.7 — set motor CAN id (req 1B)
    MotorSwitchProtocol = 0x36,    // V1.7 — switch CAN protocol, persist to flash
    MotorGetProtocol    = 0x37,    // V1.7 — query CAN protocol (resp 1B MotorProtocol)
    MotorPosCtrl        = 0x40,
    MotorVelCtrl        = 0x41,
    MotorTorqueCtrl     = 0x42,
    MotorImpedanceCtrl  = 0x43,
    GetMotorStatus      = 0x50,
    GetMotorControlStats = 0x51,   // V1.7 — follower control-loop stats (resp 48B)

    // Config (0x60–0x6F)
    SetImuConfig        = 0x60,
    GetImuConfig        = 0x61,
    SetEncoderConfig    = 0x62,
    GetEncoderConfig    = 0x63,
    SetEskinConfig      = 0x64,
    GetEskinConfig      = 0x65,
    SetGripperConfig    = 0x66,    // V1.7 — follower open/close limits (req 32B)
    GetGripperConfig    = 0x67,    // V1.7 — read follower config (resp 32B)

    // OTA upgrade (0x70–0x7F) — added V1.3
    OtaStart            = 0x70,    // ota_start_t (12B)
    OtaWriteBlock       = 0x71,    // ota_write_block_t (6 + len, len ≤ OTA_BLOCK_SIZE)
    OtaVerify           = 0x72,
    OtaApply            = 0x73,
    OtaAbort            = 0x74,
    OtaGetStatus        = 0x75,    // response: ota_status_t (8B)
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
    // OTA error band (V1.3+) — only returned by Cmd::Ota* NACKs.
    OtaBusy         = 0x50,
    OtaNotStarted   = 0x51,
    OtaOffsetErr    = 0x52,
    OtaFlashErr     = 0x53,
    OtaVerifyFail   = 0x54,
    OtaSizeExceed   = 0x55,
};

// to_string helpers (returns a stable C string; never null).
const char* to_string(Address) noexcept;
const char* to_string(FrameType) noexcept;
const char* to_string(Cmd) noexcept;
const char* to_string(ErrorCode) noexcept;

}  // namespace xense::taccap::protocol
