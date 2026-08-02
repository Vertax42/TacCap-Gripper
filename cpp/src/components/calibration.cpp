// Copyright (c) 2026 XenseRobotics Co., Ltd. — Apache-2.0

#include <taccap/components/calibration.hpp>

#include <taccap/error.hpp>
#include <taccap/protocol/codec.hpp>
#include <taccap/protocol/commands.hpp>

#include <string>
#include <vector>

namespace xense::taccap {

namespace {

// The firmware signals a handler error with protocol_send_response(seq, cmd,
// err, NULL, 0): the command byte is ECHOED (non-zero) and the payload is the
// single error-code byte. bus::Transport only recognises the cmd==0 wire path
// as a NACK, so it hands that back as a "successful" 1-byte response.
//
// For these two commands the ambiguity resolves cleanly, so we settle it here
// rather than change transport semantics for every command:
//   - a successful read  is CAMERA_FISHEYE_CAL_FULL_SIZE / ENCODER_MAX_CAL_FULL_SIZE bytes
//   - a successful write is the firmware's generic empty response, [0x00]
// so any 1-byte response carrying a NON-zero value is an error code.
protocol::ErrorCode ack_error(const bus::AckResponse& ack) {
    if (ack.is_nack) return ack.error_code;
    if (ack.data.size() == 1 && ack.data[0] != 0) {
        return static_cast<protocol::ErrorCode>(ack.data[0]);
    }
    return protocol::ErrorCode::Ok;
}

[[noreturn]] void throw_nack(const char* what, protocol::ErrorCode err) {
    throw ProtocolError(std::string("Calibration::") + what + " NACK: " +
                        protocol::to_string(err));
}

}  // namespace

Calibration::Calibration(bus::Transport& transport) : t_(transport) {}

// ---- Fisheye camera --------------------------------------------------------

std::optional<protocol::CameraFisheyeCal> Calibration::read_fisheye(
        std::chrono::milliseconds timeout) {
    auto ack = t_.send_cmd(protocol::Cmd::CameraFisheyeCal,
                           protocol::encode_camera_fisheye_cal_read(), timeout);
    const auto err = ack_error(ack);
    if (err == protocol::ErrorCode::CalNotSet) return std::nullopt;
    if (err != protocol::ErrorCode::Ok) throw_nack("read_fisheye", err);
    return protocol::decode_camera_fisheye_cal(ack.data.data(), ack.data.size());
}

void Calibration::write_fisheye(const protocol::CameraFisheyeCal& cal,
                                std::chrono::milliseconds timeout) {
    auto ack = t_.send_cmd(protocol::Cmd::CameraFisheyeCal,
                           protocol::encode_camera_fisheye_cal_write(cal), timeout);
    const auto err = ack_error(ack);
    if (err != protocol::ErrorCode::Ok) throw_nack("write_fisheye", err);
}

// ---- Leader encoder max travel angle ---------------------------------------

std::optional<float> Calibration::read_encoder_max_rad(
        std::chrono::milliseconds timeout) {
    auto ack = t_.send_cmd(protocol::Cmd::EncoderMaxCal,
                           protocol::encode_encoder_max_cal_read(), timeout);
    const auto err = ack_error(ack);
    if (err == protocol::ErrorCode::CalNotSet) return std::nullopt;
    if (err != protocol::ErrorCode::Ok) throw_nack("read_encoder_max_rad", err);
    return protocol::decode_encoder_max_cal(ack.data.data(), ack.data.size());
}

void Calibration::write_encoder_max_rad(float max_rad,
                                        std::chrono::milliseconds timeout) {
    auto ack = t_.send_cmd(protocol::Cmd::EncoderMaxCal,
                           protocol::encode_encoder_max_cal_write(max_rad), timeout);
    const auto err = ack_error(ack);
    if (err != protocol::ErrorCode::Ok) throw_nack("write_encoder_max_rad", err);
}

}  // namespace xense::taccap
