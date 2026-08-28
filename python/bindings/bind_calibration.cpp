// Copyright (c) 2026 XenseRobotics Co., Ltd. — Apache-2.0
//
// pybind11 bindings: CameraFisheyeCal, Calibration, GripperPosition
//
// Split out of the former single-file components.cpp. Pure move — see
// bindings_common.hpp for why the call order in bind_components() matters.

#include "bindings_common.hpp"

namespace xense::taccap::python {

void bind_calibration(py::module_& m) {
    using namespace xense::taccap;
    // ---- CameraFisheyeCal (V2.0 — Cmd::CameraFisheyeCal 0x2B) -------------
    py::class_<protocol::CameraFisheyeCal>(m, "CameraFisheyeCal")
        .def(py::init([]() { return protocol::CameraFisheyeCal{}; }))
        .def(py::init([](float fx, float fy, float cx, float cy,
                         float k1, float k2, float k3, float k4) {
                 return protocol::CameraFisheyeCal{fx, fy, cx, cy, k1, k2, k3, k4};
             }),
             py::arg("fx"), py::arg("fy"), py::arg("cx"), py::arg("cy"),
             py::arg("k1") = 0.0f, py::arg("k2") = 0.0f,
             py::arg("k3") = 0.0f, py::arg("k4") = 0.0f)
        .def_readwrite("fx", &protocol::CameraFisheyeCal::fx)
        .def_readwrite("fy", &protocol::CameraFisheyeCal::fy)
        .def_readwrite("cx", &protocol::CameraFisheyeCal::cx)
        .def_readwrite("cy", &protocol::CameraFisheyeCal::cy)
        .def_readwrite("k1", &protocol::CameraFisheyeCal::k1)
        .def_readwrite("k2", &protocol::CameraFisheyeCal::k2)
        .def_readwrite("k3", &protocol::CameraFisheyeCal::k3)
        .def_readwrite("k4", &protocol::CameraFisheyeCal::k4)
        // OpenCV-shaped views: cv2.fisheye.undistortImage(img, K, D).
        .def_property_readonly("K", [](const protocol::CameraFisheyeCal& c) {
            py::array_t<double> a(std::vector<py::ssize_t>{3, 3});
            auto* p = a.mutable_data();
            p[0] = c.fx; p[1] = 0.0;  p[2] = c.cx;
            p[3] = 0.0;  p[4] = c.fy; p[5] = c.cy;
            p[6] = 0.0;  p[7] = 0.0;  p[8] = 1.0;
            return a;
        }, "3x3 camera matrix as float64 numpy array (OpenCV K).")
        .def_property_readonly("D", [](const protocol::CameraFisheyeCal& c) {
            // Shape must be spelled as a container: array_t<double>(4) picks
            // the wrong overload on pybind11 2.9 and yields a stride-0 view.
            py::array_t<double> a(std::vector<py::ssize_t>{4});
            auto* p = a.mutable_data();
            p[0] = c.k1; p[1] = c.k2; p[2] = c.k3; p[3] = c.k4;
            return a;
        }, "4x1 fisheye distortion vector as float64 numpy array (OpenCV D).")
        .def("__repr__", [](const protocol::CameraFisheyeCal& c) {
            char buf[224];
            std::snprintf(buf, sizeof(buf),
                "CameraFisheyeCal(fx=%.3f, fy=%.3f, cx=%.3f, cy=%.3f, "
                "k=[%.5f, %.5f, %.5f, %.5f])",
                c.fx, c.fy, c.cx, c.cy, c.k1, c.k2, c.k3, c.k4);
            return std::string(buf);
        });

    // ---- Calibration (V2.0/V2.1 firmware-persisted calibration records) ----
    py::class_<Calibration>(m, "Calibration")
        .def("read_fisheye",
             [](Calibration& self, unsigned timeout_ms) -> py::object {
                 std::optional<protocol::CameraFisheyeCal> v;
                 {
                     py::gil_scoped_release nogil;
                     v = self.read_fisheye(std::chrono::milliseconds(timeout_ms));
                 }
                 if (!v) return py::none();
                 return py::cast(*v);
             },
             py::arg("timeout_ms") = 200,
             "Read the fisheye intrinsics + distortion persisted in MCU flash. "
             "Returns None when the firmware has never been calibrated "
             "(ErrorCode.CalNotSet); raises ProtocolError on any other NACK.")
        .def("resolve_fisheye",
             [](Calibration& self, unsigned timeout_ms) {
                 ResolvedFisheyeCal r;
                 {
                     py::gil_scoped_release nogil;
                     r = self.resolve_fisheye(std::chrono::milliseconds(timeout_ms));
                 }
                 return py::make_tuple(r.calibration, r.is_reference, r.reason);
             },
             py::arg("timeout_ms") = 200,
             "Intrinsics to rectify with, as (calibration, is_reference, reason).\n\n"
             "Prefer this over read_fisheye() unless you need to know what the "
             "flash actually holds. An uncalibrated unit answers a read with an "
             "all-zero record rather than a NACK, and handing that to "
             "FisheyeUndistorter remaps every frame to black; this applies the "
             "fallback policy once, so a caller owning the wrist UVC device "
             "itself makes the same decision the SDK makes internally.\n\n"
             "is_reference is True when the SDK's shared reference values stood "
             "in, and reason says why — empty otherwise. Warn when it is True: "
             "the reference intrinsics are approximate, since lens placement "
             "varies per assembly.")
        .def("write_fisheye",
             [](Calibration& self, const protocol::CameraFisheyeCal& cal,
                unsigned timeout_ms) {
                 py::gil_scoped_release nogil;
                 self.write_fisheye(cal, std::chrono::milliseconds(timeout_ms));
             },
             py::arg("cal"), py::arg("timeout_ms") = 500,
             "Persist fisheye parameters to MCU flash (survives power cycles). "
             "NaN/Inf values are rejected by the firmware.")
        .def("read_encoder_max_rad",
             [](Calibration& self, unsigned timeout_ms) -> py::object {
                 std::optional<float> v;
                 {
                     py::gil_scoped_release nogil;
                     v = self.read_encoder_max_rad(std::chrono::milliseconds(timeout_ms));
                 }
                 if (!v) return py::none();
                 return py::float_(*v);
             },
             py::arg("timeout_ms") = 200,
             "Leader only. Read the encoder shaft angle (rad) at full open, "
             "measured from the encoder zero (fully closed). Returns None when "
             "never calibrated. Raises ProtocolError(InvalidCmd) on a follower "
             "or on firmware older than V2.1.")
        .def("write_encoder_max_rad",
             [](Calibration& self, float max_rad, unsigned timeout_ms) {
                 py::gil_scoped_release nogil;
                 self.write_encoder_max_rad(max_rad, std::chrono::milliseconds(timeout_ms));
             },
             py::arg("max_rad"), py::arg("timeout_ms") = 500,
             "Leader only. Persist the full-open shaft angle (rad) to MCU "
             "flash. Firmware rejects NaN/Inf and max_rad <= 0.");

    // ---- GripperPosition: pure raw-rad <-> normalized [0,1] converter ------
    py::class_<GripperPosition>(m, "GripperPosition")
        .def(py::init<>())
        .def(py::init<const protocol::GripperConfig&>(), py::arg("config"))
        .def_static("from_travel", &GripperPosition::from_travel,
                    py::arg("max_rad"), py::arg("min_rad") = 0.0f,
                    py::arg("reverse") = false,
                    "Build from an explicit travel span instead of a "
                    "GripperConfig — this is how the leader gripper's map is "
                    "built from its EncoderMaxCal value.")
        .def_property_readonly("valid",        &GripperPosition::valid)
        .def_property_readonly("max_open_rad", &GripperPosition::max_open_rad)
        .def_property_readonly("min_open_rad", &GripperPosition::min_open_rad)
        .def_property_readonly("reverse",      &GripperPosition::reverse)
        .def("to_position", &GripperPosition::to_position, py::arg("raw_rad"))
        .def("to_rad",      &GripperPosition::to_rad,      py::arg("position"))
        .def("__repr__", [](const GripperPosition& gp) {
            char buf[96];
            std::snprintf(buf, sizeof(buf),
                "GripperPosition(valid=%d, max_open=%.4frad, reverse=%d)",
                gp.valid(), gp.max_open_rad(), gp.reverse());
            return std::string(buf);
        });
}

}  // namespace xense::taccap::python
