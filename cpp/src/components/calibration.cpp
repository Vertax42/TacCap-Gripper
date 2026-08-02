// Copyright (c) 2026 XenseRobotics Co., Ltd. — Apache-2.0

#include <taccap/components/calibration.hpp>

#include <taccap/error.hpp>
#include <taccap/protocol/codec.hpp>
#include <taccap/protocol/commands.hpp>

#include <string>
#include <vector>

namespace xense::taccap {

namespace {

// bus::ack_error_code recovers the firmware's echoed-cmd error path, which
// ack.is_nack alone cannot see. Valid here because neither command has a
// legitimate 1-byte non-zero success payload: a successful read is
// CAMERA_FISHEYE_CAL_FULL_SIZE / ENCODER_MAX_CAL_FULL_SIZE bytes, and a
// successful write is the firmware's generic empty response, [0x00].
using bus::ack_error_code;

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
    const auto err = ack_error_code(ack);
    if (err == protocol::ErrorCode::CalNotSet) return std::nullopt;
    if (err != protocol::ErrorCode::Ok) throw_nack("read_fisheye", err);
    return protocol::decode_camera_fisheye_cal(ack.data.data(), ack.data.size());
}

void Calibration::write_fisheye(const protocol::CameraFisheyeCal& cal,
                                std::chrono::milliseconds timeout) {
    auto ack = t_.send_cmd(protocol::Cmd::CameraFisheyeCal,
                           protocol::encode_camera_fisheye_cal_write(cal), timeout);
    const auto err = ack_error_code(ack);
    if (err != protocol::ErrorCode::Ok) throw_nack("write_fisheye", err);
}

// ---- Leader encoder max travel angle ---------------------------------------

std::optional<float> Calibration::read_encoder_max_rad(
        std::chrono::milliseconds timeout) {
    auto ack = t_.send_cmd(protocol::Cmd::EncoderMaxCal,
                           protocol::encode_encoder_max_cal_read(), timeout);
    const auto err = ack_error_code(ack);
    if (err == protocol::ErrorCode::CalNotSet) return std::nullopt;
    if (err != protocol::ErrorCode::Ok) throw_nack("read_encoder_max_rad", err);
    return protocol::decode_encoder_max_cal(ack.data.data(), ack.data.size());
}

void Calibration::write_encoder_max_rad(float max_rad,
                                        std::chrono::milliseconds timeout) {
    auto ack = t_.send_cmd(protocol::Cmd::EncoderMaxCal,
                           protocol::encode_encoder_max_cal_write(max_rad), timeout);
    const auto err = ack_error_code(ack);
    if (err != protocol::ErrorCode::Ok) throw_nack("write_encoder_max_rad", err);
}

}  // namespace xense::taccap
