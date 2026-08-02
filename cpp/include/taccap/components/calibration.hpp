// Copyright (c) 2026 XenseRobotics Co., Ltd. — Apache-2.0
//
// Calibration component: host-side access to the calibration parameters the
// firmware persists in its internal flash.
//
//   - Fisheye camera intrinsics + distortion  (Cmd::CameraFisheyeCal 0x2B,
//     firmware V2.0) — supported on BOTH leader and follower.
//   - Leader encoder max travel angle          (Cmd::EncoderMaxCal 0x2C,
//     firmware V2.1) — LEADER ONLY; the follower has no MT6816 encoder and
//     NACKs with ErrorCode::InvalidCmd.
//
// The firmware is a dumb store for both records: it persists the bytes, does
// no unit conversion and no range clamping, and rejects only NaN/Inf (plus
// max_rad <= 0). Interpreting the values is the host's job — this SDK uses
// encoder max travel to build a LeaderGripper's normalized-position map (see
// taccap/gripper_position.hpp), and hands the fisheye parameters straight to
// the caller's undistortion pipeline.
//
// Reading a parameter that was never written NACKs with ErrorCode::CalNotSet
// instead of returning zeros, so "never calibrated" is distinguishable from
// "calibrated to exactly 0". The read methods surface that as an empty
// std::optional — it is an expected state, not an error. Every other NACK
// still throws ProtocolError.

#pragma once

#include <taccap/bus/transport.hpp>
#include <taccap/protocol/payloads.hpp>

#include <chrono>
#include <optional>

namespace xense::taccap {

class Calibration {
public:
    explicit Calibration(bus::Transport& transport);

    // ---- Fisheye camera (leader + follower) --------------------------------
    // Returns std::nullopt when the firmware has never been given fisheye
    // parameters (ErrorCode::CalNotSet).
    std::optional<protocol::CameraFisheyeCal> read_fisheye(
        std::chrono::milliseconds timeout = std::chrono::milliseconds{200});

    // Persists to MCU flash (survives power cycles). Throws ProtocolError if
    // any value is NaN/Inf (firmware returns InvalidParam) or the flash write
    // fails (SysBusy).
    void write_fisheye(const protocol::CameraFisheyeCal& cal,
                       std::chrono::milliseconds timeout = std::chrono::milliseconds{500});

    // ---- Leader encoder max travel angle (leader only) ---------------------
    // The shaft angle (rad) at which the gripper is fully open, measured from
    // the encoder zero (fully closed). Returns std::nullopt when never
    // calibrated. Throws ProtocolError(InvalidCmd) on a follower.
    std::optional<float> read_encoder_max_rad(
        std::chrono::milliseconds timeout = std::chrono::milliseconds{200});

    // Firmware rejects NaN/Inf and max_rad <= 0 with InvalidParam.
    void write_encoder_max_rad(float max_rad,
                               std::chrono::milliseconds timeout = std::chrono::milliseconds{500});

private:
    bus::Transport& t_;
};

}  // namespace xense::taccap
