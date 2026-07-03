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

// Motor CAN protocol (V1.7 — Cmd::MotorGetProtocol / MotorSwitchProtocol).
enum class MotorProtocol : uint8_t {
    Private = 0,   // Xense private CAN protocol
    Mit     = 2,   // MIT-style CAN protocol
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
    float vel;            // V1.7 — feed-forward velocity (rad/s); 0 = firmware
                          // estimates from the position delta. MIT protocol only.
};

// motor_status_t — V1.9 shrank this 40 -> 31 bytes by dropping the current
// fields (actual_current / target_current / current_source). The first 18 bytes
// (actual_pos..status) are byte-identical to the older layout, so position /
// torque / status readings stay correct across firmware versions; only the
// target_* / control_mode tail moved. The SDK targets the V1.9 (31-byte) layout.
struct MotorStatus {
    float    actual_pos;      // rad
    float    actual_vel;      // rad/s
    float    actual_torque;   // Nm
    float    motor_temp;      // °C
    uint16_t status;          // MotorStatusBit::*
    float    target_pos;      // rad   — last applied target
    float    target_vel;      // rad/s
    float    target_torque;   // Nm    — target / feed-forward / clamp
    uint8_t  control_mode;    // MotorMode of the last applied command
};

// ---- Follower (slave) gripper config (V1.7 — Cmd::*GripperConfig 0x66/0x67)
constexpr uint32_t GRIPPER_CONFIG_MAGIC   = 0x47525052u;  // "GRPR"
constexpr uint16_t GRIPPER_CONFIG_VERSION = 0x0001u;
namespace GripperConfigFlag {
    constexpr uint16_t Valid   = 0x0001;  // config is valid
    constexpr uint16_t Reverse = 0x0002;  // "open" is the motor's negative dir
}

struct GripperConfig {
    uint32_t magic;          // GRIPPER_CONFIG_MAGIC
    uint16_t version;        // GRIPPER_CONFIG_VERSION
    uint16_t flags;          // GripperConfigFlag::* bits
    float    max_open_rad;   // max open span after zeroing (rad)
    float    min_open_rad;   // reserved, default 0
    uint8_t  reserved[16];   // future expansion
};

// ---- Follower motor control loop stats (V1.7 — Cmd::GetMotorControlStats 0x51)
struct MotorControlStats {
    uint8_t  running;              // control thread running?
    uint8_t  mode;                 // MotorMode
    uint16_t target_hz;            // requested control rate (Hz)
    uint16_t period_ms;            // control period (ms)
    uint16_t sample_ms;            // stats sampling window (ms)
    float    actual_hz;            // measured control output rate
    uint32_t target_seq;           // host target update sequence
    uint32_t applied_seq;          // last applied target sequence
    uint32_t loop_count;           // cumulative control loops
    uint32_t error_count;          // cumulative control errors
    uint32_t deadline_miss_count;  // cumulative period overruns
    uint32_t timeout_count;        // cumulative comm timeouts (reserved)
    int32_t  last_error;           // last error code
    uint16_t target_age_ms;        // age of current target (ms)
    uint16_t reserved;
    float    target_update_hz;     // host target update rate
};

// ---- Follower power-on auto-calibration config (V1.9 — Cmd 0x68/0x69) -----
// When Enable is set, the firmware auto-calibrates on power-up: it closes until
// the motor stalls at close_stall_torque (that pose becomes zero / fully
// closed), then opens until it stalls at open_stall_torque (that span becomes
// max_open). This automates the manual "zero at close, capture max_open" flow.
constexpr uint32_t GRIPPER_AUTO_CAL_MAGIC   = 0x4743414Cu;  // "GCAL"
constexpr uint16_t GRIPPER_AUTO_CAL_VERSION = 0x0001u;
namespace GripperAutoCalFlag {
    constexpr uint16_t Valid  = 0x0001;  // config is valid
    constexpr uint16_t Enable = 0x0002;  // run auto-cal on power-up
}

struct GripperAutoCalConfig {
    uint32_t magic;                 // GRIPPER_AUTO_CAL_MAGIC
    uint16_t version;               // GRIPPER_AUTO_CAL_VERSION
    uint16_t flags;                 // GripperAutoCalFlag::* bits
    float    close_stall_torque_nm; // close-to-zero stall torque (Nm)
    float    open_stall_torque_nm;  // open-limit stall torque (Nm)
    float    close_speed_rad_s;     // close speed (rad/s)
    float    open_speed_rad_s;      // open speed (rad/s)
    uint16_t stall_hold_ms;         // stall confirmation time (ms)
    uint16_t startup_delay_ms;      // delay after power-on before auto-cal (ms)
    uint16_t post_zero_delay_ms;    // delay after set-zero before opening (ms)
    uint8_t  close_confirm_count;   // close stall samples before set-zero
    uint8_t  open_confirm_count;    // open stall samples before saving max
};

// ---- WS2812 LED control (V1.9 — Cmd::Ws2812Set 0x0A / Ws2812Effect 0x0B) --
enum class Ws2812Mode : uint8_t {
    Off       = 0,   // all LEDs off
    EffectSet = 1,   // set the effect-layer base color
    Override  = 2,   // direct override color
};

struct Ws2812Set {
    uint8_t  mode;        // Ws2812Mode
    uint8_t  r;           // 0-255
    uint8_t  g;           // 0-255
    uint8_t  b;           // 0-255
    uint8_t  brightness;  // 0-255 (0 = leave unchanged)
    uint16_t blink_ms;    // blink half-period (ms); 0 = no blink
};

enum class Ws2812EffectType : uint8_t {
    None         = 0,
    NormalSolid  = 1,    // preset: solid green
    NormalBlink  = 2,    // preset: blinking green
    OtaBlink     = 3,    // preset: blinking blue
    FaultBlink   = 4,    // preset: blinking red
    Demo         = 5,
    ColorBlink   = 10,   // custom color blink
    ColorBreathe = 11,   // custom color breathe
    HsvCycle     = 12,   // HSV hue cycle
    ColorLerp    = 13,   // two-color LERP fade
};

struct Ws2812Effect {
    uint8_t  effect;      // Ws2812EffectType
    uint8_t  param1;      // effect param 1 (e.g. breathe min brightness)
    uint8_t  param2;      // effect param 2 (e.g. breathe max brightness)
    uint8_t  reserved;    // = 0
    uint16_t period_ms;   // effect period (ms)
    uint8_t  r1;          // color 1 / start color R
    uint8_t  g1;
    uint8_t  b1;
    uint8_t  r2;          // color 2 / end color R (LERP only)
    uint8_t  g2;
    uint8_t  b2;
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
static_assert(sizeof(MotorImpedanceCtrl) == 20);  // V1.7 (+ feed-forward vel)
static_assert(sizeof(MotorStatus)        == 31);  // V1.9 motor_status_t (was 40)
static_assert(sizeof(GripperConfig)      == 32);  // V1.7 gripper_config_t
static_assert(sizeof(MotorControlStats)  == 48);  // V1.7 motor_control_stats
static_assert(sizeof(GripperAutoCalConfig) == 32); // V1.9 gripper_auto_cal_config_t
static_assert(sizeof(Ws2812Set)          == 7);   // V1.9 ws2812_set_t
static_assert(sizeof(Ws2812Effect)       == 12);  // V1.9 ws2812_effect_t
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
