// Copyright (c) 2026 XenseRobotics Co., Ltd. — Apache-2.0
//
// TC-GU-01 protocol payload structures. Layouts mirror firmware
//   Embedded Software/tc-gu-01/App/protocol/protocol_data.h
// 1:1 — when the firmware changes, this file must follow.
//
// All structs are #pragma pack(1) to match the wire format exactly. Size
// invariants are enforced by static_assert; if a layout drift is introduced
// the build fails immediately.

#pragma once

#include <cstdint>

namespace xense::taccap::protocol {

#pragma pack(push, 1)

// ---- System --------------------------------------------------------------

struct FirmwareVersion {
    uint8_t major;
    uint8_t minor;
    uint8_t patch;
    uint8_t build;
};

// 16 character SN + NUL terminator on the wire.
struct SnInfo {
    char sn[17];
};

enum class DeviceType : uint8_t {
    Left    = 0,
    Right   = 1,
    Unknown = 0xFF,
};

// ---- IMU -----------------------------------------------------------------

namespace ImuValid {
    constexpr uint16_t Accel = 0x01;
    constexpr uint16_t Gyro  = 0x02;
    constexpr uint16_t Mag   = 0x04;
    constexpr uint16_t Temp  = 0x08;
}

struct ImuData {
    uint32_t timestamp_us;
    uint16_t valid_flag;          // ImuValid::* bits

    int16_t  accel_x, accel_y, accel_z;   // mg (0.001 g)
    int16_t  gyro_x,  gyro_y,  gyro_z;    // 0.01 dps
    int16_t  mag_x,   mag_y,   mag_z;     // 0.01 µT
    int16_t  temperature;                 // 0.01 °C
    uint16_t seq;
};

struct ImuConfig {
    uint16_t sample_rate;     // sensor internal sample rate (Hz); 0 = leave unchanged
    uint16_t odr;             // host output data rate (Hz, ≤ sample_rate); 0 = leave unchanged
    uint8_t  accel_range;     // 0=2g, 1=4g, 2=8g, 3=16g
    uint8_t  gyro_range;      // 0=250dps, 1=500dps, 2=1000dps, 3=2000dps
    uint8_t  mag_range;       // 0=4800µT
    uint8_t  filter_enable;
    uint16_t filter_cutoff;
};

// ---- Encoder -------------------------------------------------------------

namespace EncoderStatusBit {
    constexpr uint16_t Ok       = 0x0000;
    constexpr uint16_t Error    = 0x0001;
    constexpr uint16_t Overflow = 0x0002;
}

struct EncoderData {
    uint32_t timestamp_us;
    float    position_rad;
    float    velocity_rad_s;
    uint16_t status;
    uint16_t seq;
};

struct EncoderConfig {
    uint32_t baudrate;        // default 38400
    uint8_t  resolution;      // bits
    uint8_t  direction;       // 0 = forward, 1 = reversed
    float    offset_rad;      // zero-point offset
    float    ratio;           // gear ratio
};

// ---- Electronic skin -----------------------------------------------------

constexpr uint8_t ESKIN_MAX_ROWS = 12;
constexpr uint8_t ESKIN_MAX_COLS = 8;

enum class EskinOutputType : uint8_t {
    Adc     = 0,  // uint16_t per cell
    Voltage = 1,  // float per cell (V)
    Force   = 2,  // float per cell (N)
};

struct EskinHeader {
    uint32_t timestamp_us;
    uint8_t  rows;
    uint8_t  cols;
    uint8_t  type;        // EskinOutputType
    uint8_t  reserved;    // alignment padding
    uint16_t seq;
    uint16_t _reserved2;  // wire padding — firmware ESKIN_HEADER_SIZE=12
                          // memcpys 12 bytes, so the wire reserves 2 bytes
                          // here (contents undefined; do not rely on them).
    // Followed by rows*cols*(2 or 4) bytes of cell data.
};

struct EskinConfig {
    uint8_t  rows;
    uint8_t  cols;
    uint8_t  output_type;
    uint16_t sample_rate;
};

// ---- Combined sensor packet (combined-stream mode) -----------------------

namespace SensorValid {
    constexpr uint16_t Imu     = 0x0001;
    constexpr uint16_t Encoder = 0x0002;
    constexpr uint16_t Eskin1  = 0x0004;
    constexpr uint16_t Eskin2  = 0x0008;
}

struct CombinedSensorHeader {
    uint32_t timestamp_us;
    uint16_t valid_mask;
    uint16_t seq;
    // Followed by per-source [type(1) + len(2) + data] sub-packets.
};

// ---- Motor ---------------------------------------------------------------

enum class MotorMode : uint8_t {
    Idle      = 0,
    Position  = 1,
    Velocity  = 2,
    Torque    = 3,
    Impedance = 4,
};

namespace MotorStatusBit {
    constexpr uint16_t Enabled       = 0x0001;
    constexpr uint16_t Fault         = 0x0002;
    constexpr uint16_t Stalled       = 0x0004;
    constexpr uint16_t OverTemp      = 0x0008;
    constexpr uint16_t OverCurrent   = 0x0010;
    constexpr uint16_t OverVolt      = 0x0020;
    constexpr uint16_t UnderVolt     = 0x0040;
    constexpr uint16_t EncoderError  = 0x0080;
}

struct MotorPosCtrl {
    float target_pos;     // rad
    float max_vel;        // rad/s
    float max_torque;     // Nm
};

struct MotorVelCtrl {
    float target_vel;     // rad/s
    float max_torque;     // Nm
    float profile_acc;    // rad/s²
};

struct MotorTorqueCtrl {
    float target_torque;  // Nm
    float max_vel;        // rad/s
    float reserved;
};

struct MotorImpedanceCtrl {
    float target_pos;     // rad
    float kp;             // Nm/rad
    float kd;             // Nm·s/rad
    float target_torque;  // Nm (feed-forward)
};

struct MotorStatus {
    float    actual_pos;      // rad
    float    actual_vel;      // rad/s
    float    actual_torque;   // Nm
    float    motor_temp;      // °C
    uint16_t status;          // MotorStatusBit::*
};

// ---- Stream config -------------------------------------------------------

namespace StreamSrc {
    constexpr uint16_t Imu         = 0x0001;
    constexpr uint16_t Encoder     = 0x0002;
    constexpr uint16_t Eskin1      = 0x0004;
    constexpr uint16_t Eskin2      = 0x0008;
    constexpr uint16_t MotorStatus = 0x0010;
}

enum class StreamMode : uint8_t {
    Separate = 0,
    Combined = 1,
};

enum class StreamInterface : uint8_t {
    Usb  = 0,
    Uart = 1,  // canonical for v1.2 (single USART3 path)
};

struct StreamConfig {
    uint16_t source_mask;     // StreamSrc::* bits
    uint8_t  mode;            // StreamMode
    uint16_t imu_rate;        // Hz
    uint16_t encoder_rate;
    uint16_t eskin_rate;
    uint16_t motor_rate;
    uint8_t  output_iface;    // StreamInterface
};

// ---- ACK / NACK payload --------------------------------------------------

struct AckPayload {
    uint8_t  ack_seq;         // sequence number being ACKed
    uint8_t  error_code;      // ErrorCode::*; Ok for ACK, non-zero for NACK
    uint16_t retry_count;     // firmware-side retry counter
};

// ---- Key status (V1.4 — Cmd::KeyStatus, 0x15) ----------------------------

namespace KeyState {
    constexpr uint8_t SingleClickDown   = 0;
    constexpr uint8_t SingleClickUp     = 1;
    constexpr uint8_t DoubleClick       = 2;
    constexpr uint8_t LongPressDown     = 3;
    constexpr uint8_t LongPressUp       = 4;
}

struct KeyStatusPayload {
    uint8_t key_id;    // K1 = 0
    uint8_t key_state; // KeyState::*
};

// ---- IMU mag-iron calibration (V1.4 — Cmd::SetImuMagCal, 0x26) -----------

struct ImuMagCal {
    float hard_iron[3];     // bx, by, bz in µT (Hard-Iron bias)
    float soft_iron[3][3];  // 3×3 Soft-Iron correction matrix
};

// ---- Calibration result mask (V1.5 — Cmd 0x27/0x28/0x29) -----------------

enum class CalSensorId : uint8_t {
    Imu     = 0,
    ImuMag  = 1,
    Encoder = 2,
    Key     = 3,
    Eskin1  = 4,
    Eskin2  = 5,
    Camera1 = 6,
    Camera2 = 7,
    Camera3 = 8,
    // Max = 9
};

namespace CalMask {
    constexpr uint16_t Imu     = 1u << 0;
    constexpr uint16_t ImuMag  = 1u << 1;
    constexpr uint16_t Encoder = 1u << 2;
    constexpr uint16_t Key     = 1u << 3;
    constexpr uint16_t Eskin1  = 1u << 4;
    constexpr uint16_t Eskin2  = 1u << 5;
    constexpr uint16_t Camera1 = 1u << 6;
    constexpr uint16_t Camera2 = 1u << 7;
    constexpr uint16_t Camera3 = 1u << 8;
}

// CMD_SET_CAL_RESULT (0x27) request payload
struct CalSetPayload {
    uint8_t sensor_id;  // CalSensorId
    uint8_t result;     // 0 = fail/uncalibrated, 1 = success
};

// CMD_SET_ALL_CAL_RESULT (0x28) request payload
struct CalSetAllPayload {
    uint16_t mask;  // CalMask::* bits, 1 = success
};

// CMD_GET_CAL_RESULT (0x29) response payload
struct CalGetResponse {
    uint16_t mask;  // CalMask::* bits, 1 = calibrated
};

// ---- Sensor error report (V1.6 — Cmd::SensorErrorReport, 0x2A) -----------

enum class SensorErrorId : uint8_t {
    Imu      = 0,
    ImuMag   = 1,
    Encoder  = 2,
    Eskin1   = 3,
    Eskin2   = 4,
    Motor    = 5,
    // Max = 6
};

namespace SensorErrCode {
    constexpr uint8_t None        = 0x00;  // recovered
    constexpr uint8_t InitFail    = 0x01;
    constexpr uint8_t CommTimeout = 0x02;
    constexpr uint8_t DataInvalid = 0x03;  // CRC etc
    constexpr uint8_t Offline     = 0x04;
    constexpr uint8_t Overflow    = 0x05;
    constexpr uint8_t Range       = 0x06;
}

struct SensorErrorReport {
    uint8_t  sensor_id;       // SensorErrorId
    uint8_t  error_code;      // SensorErrCode::*
    uint16_t error_count;     // cumulative error count
    uint32_t timestamp_ms;    // firmware HAL_GetTick() — milliseconds, NOT µs
};

// ---- OTA (V1.3 — Cmd::Ota* 0x70-0x75) ------------------------------------

constexpr uint16_t OTA_BLOCK_SIZE   = 1024;  // payload size per OTA_WRITE_BLOCK
constexpr uint32_t OTA_MAX_FW_SIZE  = 456u * 1024u;  // single-bank max

enum class OtaState : uint8_t {
    Idle      = 0,
    Started   = 1,  // OTA_START accepted, awaiting blocks
    Receiving = 2,  // block stream in progress
    Verified  = 3,  // CRC32 passed
    Applying  = 4,  // bank swap in flight
    Error     = 5,  // needs Cmd::OtaAbort to clear
};

// CMD_OTA_START (0x70) request payload — 12 bytes
struct OtaStart {
    uint32_t firmware_size;   // total bytes of firmware image
    uint32_t firmware_crc32;  // ISO-HDLC / zlib.crc32 of the whole image
    uint8_t  target_major;
    uint8_t  target_minor;
    uint8_t  target_patch;
    uint8_t  target_build;
};

// CMD_OTA_WRITE_BLOCK (0x71) request header — followed by `length` bytes
// of raw block data. The full wire payload size = sizeof(OtaWriteBlockHeader)
// + length, where length ≤ OTA_BLOCK_SIZE.
struct OtaWriteBlockHeader {
    uint32_t offset;   // byte offset of this block within the firmware image
    uint16_t length;   // bytes following the header
};

// CMD_OTA_GET_STATUS (0x75) response payload — 8 bytes
struct OtaStatus {
    uint8_t  state;          // OtaState
    uint8_t  error_code;     // last ErrorCode (OtaBusy/OtaNotStarted/…)
    uint32_t bytes_written;  // cumulative bytes accepted
    uint16_t progress_ppt;   // 0–1000 (per-mille)
};

#pragma pack(pop)

// Layout assertions — must match firmware *_PACKET_SIZE / *_HEADER_SIZE
// macros in protocol_data.h. If any of these fail, a wire-format change
// has slipped in and downstream parsing will desync.
static_assert(sizeof(FirmwareVersion)    == 4);
static_assert(sizeof(SnInfo)             == 17);
static_assert(sizeof(ImuData)            == 28);  // IMU_PACKET_SIZE
static_assert(sizeof(ImuConfig)          == 10);
static_assert(sizeof(EncoderData)        == 16);  // ENCODER_PACKET_SIZE
static_assert(sizeof(EncoderConfig)      == 14);
static_assert(sizeof(EskinHeader)        == 12);  // ESKIN_HEADER_SIZE
static_assert(sizeof(EskinConfig)        == 5);
static_assert(sizeof(CombinedSensorHeader) == 8);
static_assert(sizeof(MotorPosCtrl)       == 12);
static_assert(sizeof(MotorVelCtrl)       == 12);
static_assert(sizeof(MotorTorqueCtrl)    == 12);
static_assert(sizeof(MotorImpedanceCtrl) == 16);
static_assert(sizeof(MotorStatus)        == 18);  // MOTOR_STATUS_SIZE
static_assert(sizeof(StreamConfig)       == 12);
static_assert(sizeof(AckPayload)         == 4);
// V1.4+
static_assert(sizeof(KeyStatusPayload)   == 2);
static_assert(sizeof(ImuMagCal)          == 48);  // 3 + 9 floats
// V1.5+
static_assert(sizeof(CalSetPayload)      == 2);
static_assert(sizeof(CalSetAllPayload)   == 2);
static_assert(sizeof(CalGetResponse)     == 2);
// V1.6+
static_assert(sizeof(SensorErrorReport)  == 8);   // SENSOR_ERROR_REPORT_SIZE
// V1.3 OTA
static_assert(sizeof(OtaStart)           == 12);
static_assert(sizeof(OtaWriteBlockHeader) == 6);
static_assert(sizeof(OtaStatus)          == 8);

}  // namespace xense::taccap::protocol
