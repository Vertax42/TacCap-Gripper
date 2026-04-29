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
std::vector<uint8_t> encode(const StreamConfig&);
std::vector<uint8_t> encode(const ImuConfig&);
std::vector<uint8_t> encode(const EncoderConfig&);
std::vector<uint8_t> encode(const EskinConfig&);
std::vector<uint8_t> encode_sn(const std::string& sn);   // 17-byte NUL-padded

// ---- Specific payload decoders (throw ProtocolError on size mismatch) -----

FirmwareVersion    decode_version(const uint8_t* data, std::size_t len);
std::string        decode_sn(const uint8_t* data, std::size_t len);
DeviceType         decode_dev_type(const uint8_t* data, std::size_t len);
ImuData            decode_imu(const uint8_t* data, std::size_t len);
ImuConfig          decode_imu_config(const uint8_t* data, std::size_t len);
EncoderData        decode_encoder(const uint8_t* data, std::size_t len);
EncoderConfig      decode_encoder_config(const uint8_t* data, std::size_t len);
EskinHeader        decode_eskin_header(const uint8_t* data, std::size_t len);
EskinConfig        decode_eskin_config(const uint8_t* data, std::size_t len);
MotorStatus        decode_motor_status(const uint8_t* data, std::size_t len);
StreamConfig       decode_stream_config(const uint8_t* data, std::size_t len);
AckPayload         decode_ack(const uint8_t* data, std::size_t len);

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
