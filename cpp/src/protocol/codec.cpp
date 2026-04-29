// Copyright (c) 2026 XenseRobotics Co., Ltd. — Apache-2.0

#include <taccap/protocol/codec.hpp>
#include <taccap/error.hpp>

#include <cstring>

namespace xense::taccap::protocol {

namespace {

template <typename T>
T pod_from_bytes(const uint8_t* data, std::size_t len, const char* type_name) {
    if (len != sizeof(T)) {
        throw ProtocolError(
            std::string("decode ") + type_name + ": expected " +
            std::to_string(sizeof(T)) + " bytes, got " + std::to_string(len));
    }
    T out{};
    std::memcpy(&out, data, sizeof(T));
    return out;
}

}  // namespace

// ---- Encoders -------------------------------------------------------------

std::vector<uint8_t> encode(const MotorPosCtrl& v)        { return encode_pod(v); }
std::vector<uint8_t> encode(const MotorVelCtrl& v)        { return encode_pod(v); }
std::vector<uint8_t> encode(const MotorTorqueCtrl& v)     { return encode_pod(v); }
std::vector<uint8_t> encode(const MotorImpedanceCtrl& v)  { return encode_pod(v); }
std::vector<uint8_t> encode(const StreamConfig& v)        { return encode_pod(v); }
std::vector<uint8_t> encode(const ImuConfig& v)           { return encode_pod(v); }
std::vector<uint8_t> encode(const EncoderConfig& v)       { return encode_pod(v); }
std::vector<uint8_t> encode(const EskinConfig& v)         { return encode_pod(v); }

std::vector<uint8_t> encode_sn(const std::string& sn) {
    std::vector<uint8_t> out(sizeof(SnInfo), 0);
    const std::size_t n = std::min(sn.size(), static_cast<std::size_t>(16));
    std::memcpy(out.data(), sn.data(), n);
    return out;
}

// ---- Decoders -------------------------------------------------------------

FirmwareVersion decode_version(const uint8_t* data, std::size_t len) {
    return pod_from_bytes<FirmwareVersion>(data, len, "FirmwareVersion");
}

std::string decode_sn(const uint8_t* data, std::size_t len) {
    if (len == 0) return {};
    // SnInfo wire size is 17 bytes, but earlier docs show queries returning
    // an unbounded ASCII string — accept anything and trim at the first NUL.
    std::size_t end = 0;
    while (end < len && data[end] != 0) ++end;
    return std::string(reinterpret_cast<const char*>(data), end);
}

DeviceType decode_dev_type(const uint8_t* data, std::size_t len) {
    if (len < 1) {
        throw ProtocolError("decode dev_type: empty payload");
    }
    return static_cast<DeviceType>(data[0]);
}

ImuData decode_imu(const uint8_t* data, std::size_t len) {
    return pod_from_bytes<ImuData>(data, len, "ImuData");
}
ImuConfig decode_imu_config(const uint8_t* data, std::size_t len) {
    return pod_from_bytes<ImuConfig>(data, len, "ImuConfig");
}
EncoderData decode_encoder(const uint8_t* data, std::size_t len) {
    return pod_from_bytes<EncoderData>(data, len, "EncoderData");
}
EncoderConfig decode_encoder_config(const uint8_t* data, std::size_t len) {
    return pod_from_bytes<EncoderConfig>(data, len, "EncoderConfig");
}
EskinHeader decode_eskin_header(const uint8_t* data, std::size_t len) {
    return pod_from_bytes<EskinHeader>(data, len, "EskinHeader");
}
EskinConfig decode_eskin_config(const uint8_t* data, std::size_t len) {
    return pod_from_bytes<EskinConfig>(data, len, "EskinConfig");
}
MotorStatus decode_motor_status(const uint8_t* data, std::size_t len) {
    return pod_from_bytes<MotorStatus>(data, len, "MotorStatus");
}
StreamConfig decode_stream_config(const uint8_t* data, std::size_t len) {
    return pod_from_bytes<StreamConfig>(data, len, "StreamConfig");
}
AckPayload decode_ack(const uint8_t* data, std::size_t len) {
    return pod_from_bytes<AckPayload>(data, len, "AckPayload");
}

EskinFrame decode_eskin(const uint8_t* data, std::size_t len) {
    if (len < sizeof(EskinHeader)) {
        throw ProtocolError(
            "decode EskinFrame: payload too short for header (" +
            std::to_string(len) + " < " + std::to_string(sizeof(EskinHeader)) + ")");
    }

    EskinFrame out;
    std::memcpy(&out.header, data, sizeof(EskinHeader));

    const std::size_t cells = static_cast<std::size_t>(out.header.rows) *
                              static_cast<std::size_t>(out.header.cols);
    const std::size_t cell_size =
        (out.header.type == static_cast<uint8_t>(EskinOutputType::Adc)) ? 2u : 4u;
    const std::size_t expected = sizeof(EskinHeader) + cells * cell_size;
    if (len != expected) {
        throw ProtocolError(
            "decode EskinFrame: length mismatch (got " + std::to_string(len) +
            ", expected " + std::to_string(expected) + " for " +
            std::to_string(cells) + " cells of " + std::to_string(cell_size) + "B)");
    }

    const uint8_t* body = data + sizeof(EskinHeader);
    if (cell_size == 2) {
        out.values_u16.resize(cells);
        std::memcpy(out.values_u16.data(), body, cells * 2);
    } else {
        out.values_f32.resize(cells);
        std::memcpy(out.values_f32.data(), body, cells * 4);
    }
    return out;
}

}  // namespace xense::taccap::protocol
