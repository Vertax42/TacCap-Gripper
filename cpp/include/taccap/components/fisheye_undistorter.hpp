// Copyright (c) 2026 XenseRobotics Co., Ltd. — Apache-2.0
//
// Fisheye undistortion for the wrist camera, driven by the intrinsics the
// firmware persists in flash (Cmd::CameraFisheyeCal 0x2B, command set >= V2.0;
// read them with Calibration::read_fisheye()).
//
// This class is deliberately protocol-free: it takes a plain
// protocol::CameraFisheyeCal value and knows nothing about transports or
// grippers, so it is equally usable on frames that never came from this SDK
// (e.g. an external camera service that owns the wrist UVC device — which is
// the common case, since Gripper::Config::open_cameras defaults to false).
//
// The remap tables are built once in the constructor and reused for every
// frame; rebuilding them per frame would dominate the cost.

#pragma once

#include <taccap/protocol/payloads.hpp>

#include <opencv2/core.hpp>

#include <cmath>

namespace xense::taccap {

// The wrist camera is calibrated at 640x480 and the firmware record carries
// ONLY the 8 intrinsic/distortion floats — no image size. Rather than guess a
// scale factor for a differently-sized frame (which would silently produce a
// wrong rectification), the constructor rejects any other size outright. If a
// second calibration resolution is ever needed, the right fix is to record the
// size in the firmware payload, not to infer it here.
inline constexpr int FISHEYE_CALIB_WIDTH  = 640;
inline constexpr int FISHEYE_CALIB_HEIGHT = 480;

// `balance` interpolates the output focal length between these two, matching
// the PC tool's fisheye_focal_scale_for_balance()
// (tc-gu-01-pc/camera_calibration/calibration/workflow.py). balance = 0 keeps
// the calibrated focal length — a natural, reference-like view; balance = 1
// shortens it to 0.70x for the widest field of view, at the cost of more
// black border around the edges.
// A reference calibration for the TC-GU-01 wrist lens, measured on a sample
// unit. It exists so a gripper whose firmware was never calibrated can still
// deliver an approximately rectified stream instead of raw fisheye — every unit
// carries the same lens and the same 640x480 sensor, so the shared numbers are
// far closer to correct than no rectification at all.
//
// It is NOT a substitute for calibrating a unit. Lens placement varies between
// assemblies, so the principal point in particular drifts per unit; anything
// that measures in pixels off a rectified frame should be calibrated properly.
// Every path that falls back to this warns, and says how to store a real one.
inline constexpr protocol::CameraFisheyeCal FISHEYE_FALLBACK_CAL{
    /* fx */ 213.0303f, /* fy */ 212.7928f,
    /* cx */ 321.4000f, /* cy */ 239.9500f,
    /* k1 */ -0.0172f,  /* k2 */ 0.0091f,
    /* k3 */ -0.0146f,  /* k4 */ 0.0051f,
};

// A record read back from flash can be present but empty: an uncalibrated unit
// answers with all-zero params rather than a NACK, and building remap tables
// from fx = fy = 0 maps every pixel outside the source image, so the "rectified"
// frame comes out uniformly black. Observed on firmware 1.1.1. Treat such a
// record as absent.
inline bool is_usable_fisheye_cal(const protocol::CameraFisheyeCal& c) {
    return std::isfinite(c.fx) && std::isfinite(c.fy) && c.fx > 0.0f && c.fy > 0.0f;
}

inline constexpr double FISHEYE_NATURAL_FOCAL_SCALE = 1.00;
inline constexpr double FISHEYE_WIDE_FOCAL_SCALE    = 0.70;

class FisheyeUndistorter {
public:
    // Throws Error when `size` is not the calibrated 640x480. `balance` is
    // clamped to [0,1] rather than rejected — it is a taste knob, not a
    // correctness one.
    FisheyeUndistorter(const protocol::CameraFisheyeCal& cal,
                       cv::Size size,
                       float    balance = 0.0f);

    // Rectify one frame. `src` must match the size this was built for;
    // mismatches throw Error rather than silently producing garbage.
    // Any cv::Mat type cv::remap accepts works — BGR8 in the SDK's own path.
    cv::Mat apply(const cv::Mat& src) const;

    cv::Size size()        const noexcept { return size_; }
    float    balance()     const noexcept { return balance_; }
    double   focal_scale() const noexcept { return focal_scale_; }

    // The camera matrix the rectified frames are expressed in — i.e. the
    // calibrated K with fx/fy scaled by focal_scale(). Callers that do their
    // own geometry on rectified pixels (projection, hand-eye) need THIS
    // matrix, not the raw one from the firmware.
    const cv::Mat& new_camera_matrix() const noexcept { return new_k_; }

private:
    cv::Size size_;
    float    balance_     = 0.0f;
    double   focal_scale_ = FISHEYE_NATURAL_FOCAL_SCALE;
    cv::Mat  new_k_;          // CV_64F 3x3
    cv::Mat  map_x_, map_y_;  // CV_32FC1, built once
};

}  // namespace xense::taccap
