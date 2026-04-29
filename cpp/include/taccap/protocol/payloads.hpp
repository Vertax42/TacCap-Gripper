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

}  // namespace xense::taccap::protocol
