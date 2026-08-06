// Copyright (c) 2026 XenseRobotics Co., Ltd. — Apache-2.0
//
// Typed encoders / decoders for TC-GU-01 payloads. Thin layer on top of
// payloads.hpp that hides the raw memcpy and bounds-checks the buffer.
//
// Encoders return wire bytes; decoders take a pointer + length and either
// return std::optional<T> (caller-friendly) or throw ProtocolError when the
// length is wrong. We default to throwing because most call sites already
// know exactly which payload type they're parsing — wrong length is a
// firmware/version-skew bug worth surfacing, not silently dropping.

#pragma once

#include <taccap/protocol/payloads.hpp>

#include <cstdint>
#include <cstddef>
#include <cstring>
#include <string>
#include <type_traits>
#include <vector>

namespace xense::taccap::protocol {

// ---- Generic helpers ------------------------------------------------------

template <typename T>
std::vector<uint8_t> encode_pod(const T& value) {
    static_assert(std::is_trivially_copyable_v<T>,
                  "encode_pod requires a trivially-copyable POD");
    std::vector<uint8_t> out(sizeof(T));
    std::memcpy(out.data(), &value, sizeof(T));
    return out;
}

// ---- Specific payload encoders --------------------------------------------

std::vector<uint8_t> encode(const MotorPosCtrl&);
std::vector<uint8_t> encode(const MotorVelCtrl&);
std::vector<uint8_t> encode(const MotorTorqueCtrl&);
std::vector<uint8_t> encode(const MotorImpedanceCtrl&);
std::vector<uint8_t> encode(const GripperConfig&);   // V1.7
std::vector<uint8_t> encode(const GripperAutoCalConfig&);  // V1.9
std::vector<uint8_t> encode(const Ws2812Set&);       // V1.9
std::vector<uint8_t> encode(const Ws2812Effect&);    // V1.9
std::vector<uint8_t> encode(const StreamConfig&);
std::vector<uint8_t> encode(const ImuConfig&);
std::vector<uint8_t> encode(const EncoderConfig&);
std::vector<uint8_t> encode(const EskinConfig&);
std::vector<uint8_t> encode_sn(const std::string& sn);   // 17-byte NUL-padded

// V1.4+ — calibration / key
std::vector<uint8_t> encode(const ImuMagCal&);
std::vector<uint8_t> encode(const CalSetPayload&);
std::vector<uint8_t> encode(const CalSetAllPayload&);

// V2.0/V2.1 — Cmd::CameraFisheyeCal / Cmd::EncoderMaxCal multiplex read and
// write on one command via a leading CalOp byte, so they get explicit builders
// instead of an encode() overload (the same struct maps to two wire shapes).
std::vector<uint8_t> encode_camera_fisheye_cal_read();                  // 1 byte
std::vector<uint8_t> encode_camera_fisheye_cal_write(const CameraFisheyeCal&);  // 33 bytes
std::vector<uint8_t> encode_encoder_max_cal_read();                     // 1 byte
std::vector<uint8_t> encode_encoder_max_cal_write(float max_rad);       // 5 bytes

// V1.3+ — OTA. write_block has a variable-length tail so we take the
// data pointer + length explicitly rather than wrap them in a struct.
std::vector<uint8_t> encode(const OtaStart&);
std::vector<uint8_t> encode_ota_write_block(uint32_t offset,
                                            const uint8_t* data,
                                            uint16_t length);

// ---- Specific payload decoders (throw ProtocolError on size mismatch) -----

FirmwareVersion    decode_version(const uint8_t* data, std::size_t len);

// Human-facing firmware version, "MAJOR.MINOR.PATCH" — deliberately WITHOUT
// the build byte. Firmware pins build to 0 and it carries no meaning for
// users, so showing "1.2.1.0" only invited people to type the trailing zero
// into version comparisons. The byte is still on the wire and still readable
// as FirmwareVersion::build for anyone who needs it; this is the one place
// that decides how a version is *presented*, so the format cannot drift
// between the gripper open() logs, OTA logs and the example CLIs.
std::string        version_string(const FirmwareVersion&);
std::string        decode_sn(const uint8_t* data, std::size_t len);
DeviceType         decode_dev_type(const uint8_t* data, std::size_t len);
ImuData            decode_imu(const uint8_t* data, std::size_t len);
ImuConfig          decode_imu_config(const uint8_t* data, std::size_t len);
EncoderData        decode_encoder(const uint8_t* data, std::size_t len);
EncoderConfig      decode_encoder_config(const uint8_t* data, std::size_t len);
EskinHeader        decode_eskin_header(const uint8_t* data, std::size_t len);
EskinConfig        decode_eskin_config(const uint8_t* data, std::size_t len);
MotorStatus        decode_motor_status(const uint8_t* data, std::size_t len);
MotorPrivateParam  decode_motor_private_param(const uint8_t* data, std::size_t len);  // V1.9+
// V1.7 follower (slave) gripper
GripperConfig      decode_gripper_config(const uint8_t* data, std::size_t len);
GripperAutoCalConfig decode_gripper_auto_cal_config(const uint8_t* data, std::size_t len);  // V1.9
MotorControlStats  decode_motor_control_stats(const uint8_t* data, std::size_t len);
StreamConfig       decode_stream_config(const uint8_t* data, std::size_t len);
AckPayload         decode_ack(const uint8_t* data, std::size_t len);
// V1.4+
KeyStatusPayload   decode_key_status(const uint8_t* data, std::size_t len);
ImuMagCal          decode_imu_mag_cal(const uint8_t* data, std::size_t len);
// V1.5+
CalGetResponse     decode_cal_get(const uint8_t* data, std::size_t len);
// V2.0/V2.1 — read responses lead with the CalOp byte echoed back (Read), not
// an error byte; these decoders verify that byte and strip it.
CameraFisheyeCal   decode_camera_fisheye_cal(const uint8_t* data, std::size_t len);
float              decode_encoder_max_cal(const uint8_t* data, std::size_t len);
// V1.6+
SensorErrorReport  decode_sensor_error(const uint8_t* data, std::size_t len);
// V1.3 OTA
OtaStatus          decode_ota_status(const uint8_t* data, std::size_t len);

// EskinFrame collapses the wire layout (header + variable-length body) into
// one easy-to-use object. Cell data is stored in either values_u16 (when
// type == EskinOutputType::Adc) or values_f32 (Voltage / Force).
struct EskinFrame {
    EskinHeader              header;
    std::vector<uint16_t>    values_u16;
    std::vector<float>       values_f32;
};
EskinFrame decode_eskin(const uint8_t* data, std::size_t len);

}  // namespace xense::taccap::protocol
