// Copyright (c) 2026 XenseRobotics Co., Ltd. — Apache-2.0
//
// pybind11 entry point for the xense.taccap Python package.
//
// At step 1 this only exposes version + a hello() smoke test. Component
// classes (Camera, IMU, Encoder, Motor, LeaderGripper, FollowerGripper, ...)
// will be bound here in subsequent steps.

#include <pybind11/pybind11.h>

#include <taccap/version.hpp>
#include <taccap/vision.hpp>

#include <xense/core/version.hpp>  // defines XENSESDK_VERSION_STRING (libxense version)

#include <string>

namespace py = pybind11;

PYBIND11_MODULE(_taccap_native, m) {
    m.doc() = "TacCap-Gripper native module (lite scaffold)";

    m.attr("__version__")      = TACCAP_VERSION_STRING;
    m.attr("libxense_version") = XENSESDK_VERSION_STRING;

    m.def("hello", []() {
        return std::string("taccap-gripper lite scaffold OK; "
                           "libxense lite version: ") + XENSESDK_VERSION_STRING;
    });
}
