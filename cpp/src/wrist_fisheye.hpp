// Copyright (c) 2026 XenseRobotics Co., Ltd. — Apache-2.0
//
// Private helper shared by leader_gripper.cpp and follower_gripper.cpp — NOT
// part of the installed public headers.
//
// Both grippers wire the wrist camera's fisheye undistortion identically:
// read the intrinsics the firmware persisted, build the remap tables, hand
// them to the Camera. The interesting part is the failure policy, and having
// one copy of it keeps leader and follower from drifting apart on what counts
// as a warning versus an error.

#pragma once

#include <taccap/components/calibration.hpp>
#include <taccap/components/camera.hpp>
#include <taccap/components/fisheye_undistorter.hpp>
#include <taccap/error.hpp>
#include <taccap/log.hpp>

#include <memory>

namespace xense::taccap::detail {

// Installs fisheye undistortion on `cam`, always — from this unit's own
// calibration when the firmware holds one, and from FISHEYE_FALLBACK_CAL with a
// warning when it does not.
//
// Falls back with a warning (every unit shares the lens, so the reference
// numbers beat raw fisheye; see FISHEYE_FALLBACK_CAL for what that costs):
//   - firmware has no fisheye record yet     -> read_fisheye() == nullopt
//   - firmware answers with an empty record  -> !is_usable_fisheye_cal()
//   - firmware predates command set V2.0     -> ProtocolError(InvalidCmd)
//
// Propagates (a config bug the caller must fix, not something to paper over):
//   - the camera is not running at the calibrated 640x480, so the intrinsics
//     do not apply and any rectification would be quietly wrong.
inline void install_wrist_undistorter(Calibration& cal,
                                      Camera&      cam,
                                      float        balance,
                                      const char*  who) {
    std::optional<protocol::CameraFisheyeCal> params;
    const char* fallback_reason = nullptr;
    try {
        params = cal.read_fisheye();
    } catch (const ProtocolError& e) {
        logger()->warn("{}: the firmware would not answer the fisheye read ({}). "
                       "Command set V2.0 or newer is required.", who, e.what());
        fallback_reason = "the firmware predates command set V2.0";
    }
    if (!fallback_reason && !params) {
        fallback_reason = "the firmware holds no calibration yet";
    }
    if (!fallback_reason && !is_usable_fisheye_cal(*params)) {
        fallback_reason = "the firmware answered with an empty calibration record";
    }

    if (fallback_reason) {
        logger()->warn("{}: wrist fisheye undistortion is using the SDK's REFERENCE "
                       "intrinsics because {}. Rectification will be approximate — "
                       "lens placement varies per assembly, so the principal point "
                       "drifts. Run python/examples/fisheye_cal.py set-fisheye to "
                       "store this unit's own calibration.", who, fallback_reason);
        params = FISHEYE_FALLBACK_CAL;
    }

    const cv::Size size{cam.config().width, cam.config().height};
    auto u = std::make_shared<const FisheyeUndistorter>(*params, size, balance);
    cam.set_undistorter(u);
    logger()->info("{}: wrist fisheye undistortion on ({}x{}, balance={:.2f}, "
                   "focal_scale={:.3f}, calibration={})", who, size.width,
                   size.height, u->balance(), u->focal_scale(),
                   fallback_reason ? "SDK reference" : "this unit");
}

}  // namespace xense::taccap::detail
