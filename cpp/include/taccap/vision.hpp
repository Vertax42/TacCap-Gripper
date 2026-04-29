// Copyright (c) 2026 XenseRobotics Co., Ltd. — Apache-2.0
//
// Namespace alias header: re-exports the libxense lite raw -> rectify subset
// under xense::taccap::, so all TacCap-Gripper public C++ API lives in a
// single namespace. Zero runtime overhead — these are using-declarations.

#pragma once

#include <xense/core/frame.hpp>
#include <xense/core/types.hpp>
#include <xense/device/context.hpp>
#include <xense/device/device.hpp>
#include <xense/device/sensor.hpp>
#include <xense/config/device_config.hpp>
#include <xense/processing/rectifier.hpp>

namespace xense::taccap {

// ---- Visuotactile camera path (raw V4L2 capture -> XU calibration -> rectify)
using xense::Context;
using xense::Device;
using xense::Sensor;
using xense::Rectifier;
using xense::Frame;
using xense::FrameFormat;
using xense::StreamType;
using xense::Size;
using xense::Point2f;
using xense::RectifyConfig;
using xense::MarkerConfig;

// New TacCap-native types (Camera for the wrist UVC, LeaderGripper,
// FollowerGripper, IMU, Encoder, Motor, Bus, ...) will be added here in
// subsequent steps.

}  // namespace xense::taccap
