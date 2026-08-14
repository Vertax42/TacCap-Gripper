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
