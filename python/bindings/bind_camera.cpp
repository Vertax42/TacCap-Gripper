// Copyright (c) 2026 XenseRobotics Co., Ltd. — Apache-2.0
//
// pybind11 bindings: Camera, FisheyeUndistorter, ColorMode
//
// Split out of the former single-file components.cpp. Pure move — see
// bindings_common.hpp for why the call order in bind_components() matters.

#include "bindings_common.hpp"

namespace xense::taccap::python {

void bind_camera(py::module_& m) {
    using namespace xense::taccap;
    // ---- Camera ---------------------------------------------------------
    // ---- Fisheye fallback (used when the firmware has no calibration) ----
    m.attr("FISHEYE_FALLBACK_CAL") = py::cast(FISHEYE_FALLBACK_CAL);
    m.def("is_usable_fisheye_cal", &is_usable_fisheye_cal, py::arg("calibration"),
          "True when a CameraFisheyeCal carries usable intrinsics.\n\n"
          "An uncalibrated unit answers a flash read with an all-zero record "
          "rather than a NACK, and remapping with fx = fy = 0 yields a black "
          "frame. Use this to tell a real calibration from an empty one; "
          "FISHEYE_FALLBACK_CAL is the reference to fall back to, with a "
          "warning, when it returns False.");

    // ---- FisheyeUndistorter (V2.0+ wrist fisheye intrinsics) -------------
    // Usable standalone on frames this SDK never captured — the common case,
    // since the wrist UVC device is normally owned by an external service.
    py::class_<FisheyeUndistorter, std::shared_ptr<FisheyeUndistorter>>(
            m, "FisheyeUndistorter")
        .def(py::init([](const protocol::CameraFisheyeCal& cal,
                         int width, int height, float balance) {
                 return std::make_shared<FisheyeUndistorter>(
                     cal, cv::Size(width, height), balance);
             }),
             py::arg("calibration"),
             // Defaults are the only calibrated resolution; anything else
             // raises, because the firmware record carries no image size.
             py::arg("width")   = FISHEYE_CALIB_WIDTH,
             py::arg("height")  = FISHEYE_CALIB_HEIGHT,
             // 0 = calibrated focal length (natural view, matches the PC tool);
             // 1 = 0.70x for the widest field of view. Clamped to [0,1].
             py::arg("balance") = 0.0f)
        .def("apply", [](const FisheyeUndistorter& self, const py::array& img) {
                 cv::Mat src = numpy_to_mat_bgr(img);
                 cv::Mat dst;
                 {
                     // remap() is pure CPU work on borrowed memory — let other
                     // threads run, but keep `img` alive via the caller's ref.
                     py::gil_scoped_release gil;
                     dst = self.apply(src);
                 }
                 return mat_to_numpy(dst);
             },
             py::arg("image"),
             "Rectify one (H, W, 3) uint8 BGR frame; returns a new array.\n\n"
             "Resampled with INTER_CUBIC: the periphery is magnified about "
             "3.3x by the fisheye-to-pinhole mapping, and bilinear visibly "
             "softens an image being enlarged that much.")
        .def_property_readonly("width",  [](const FisheyeUndistorter& s) { return s.size().width; })
        .def_property_readonly("height", [](const FisheyeUndistorter& s) { return s.size().height; })
        .def_property_readonly("balance", &FisheyeUndistorter::balance)
        .def_property_readonly("focal_scale", &FisheyeUndistorter::focal_scale)
        .def_property_readonly("new_camera_matrix",
            [](const FisheyeUndistorter& s) {
                // The K rectified pixels live in — NOT the raw firmware K.
                const cv::Mat& k = s.new_camera_matrix();
                py::array_t<double> a(std::vector<py::ssize_t>{3, 3});
                std::memcpy(a.mutable_data(), k.ptr<double>(), 9 * sizeof(double));
                return a;
            },
            "3x3 camera matrix of the rectified image (calibrated K with fx/fy "
            "scaled by focal_scale).")
        .def("__repr__", [](const FisheyeUndistorter& s) {
            char buf[128];
            std::snprintf(buf, sizeof(buf),
                "FisheyeUndistorter(%dx%d, balance=%.2f, focal_scale=%.3f)",
                s.size().width, s.size().height, s.balance(), s.focal_scale());
            return std::string(buf);
        });

    py::enum_<ColorMode>(m, "ColorMode",
        "Channel order of the frames a Camera hands out.\n\n"
        "BGR is the default and is OpenCV's own convention, so imshow/imwrite on\n"
        "a frame straight out of read() looks right. Pick RGB when the frames\n"
        "feed a machine-learning pipeline — LeRobot datasets store RGB — so the\n"
        "conversion happens once in the capture path rather than at every call\n"
        "site, or worse, gets forgotten and records swapped channels.")
        .value("BGR", ColorMode::Bgr)
        .value("RGB", ColorMode::Rgb);

    py::class_<Camera>(m, "Camera")
        .def(py::init([](const std::string& dev, int w, int h, double fps, bool mjpg,
                         ColorMode color_mode) {
            return std::make_unique<Camera>(Camera::Config{dev, w, h, fps, mjpg, color_mode});
        }),
            py::arg("device"), py::arg("width") = 640, py::arg("height") = 480,
            py::arg("fps") = 30.0, py::arg("use_mjpg") = true,
            py::arg("color_mode") = ColorMode::Bgr)
        .def("read", [](Camera& self, unsigned timeout_ms) -> py::object {
            CameraFrame f;
            bool ok;
            {
                py::gil_scoped_release gil;
                ok = self.read(f, std::chrono::milliseconds(timeout_ms));
            }
            if (!ok) return py::none();
            return py::cast(std::move(f));
        }, py::arg("timeout_ms") = 500)
        .def("start", [](Camera& self, py::function pycb) {
            auto cb = make_gil_safe_callback(std::move(pycb));
            self.start([cb](const CameraFrame& f) {
                call_into_python("xense.taccap.Camera callback",
                                 [&] { (*cb)(f); });
            });
        }, py::arg("callback"))
        .def("stop", [](Camera& self) {
            py::gil_scoped_release gil;
            self.stop();
        })
        // Pass None to go back to raw frames. Safe to call while streaming.
        .def("set_undistorter", [](Camera& self,
                                   std::shared_ptr<FisheyeUndistorter> u) {
            py::gil_scoped_release gil;
            self.set_undistorter(std::move(u));
        }, py::arg("undistorter").none(true))
        .def_property_readonly("is_streaming",   &Camera::is_streaming)
        .def_property_readonly("total_frames",   &Camera::total_frames)
        .def_property_readonly("dropped_frames", &Camera::dropped_frames)
        .def_property_readonly("actual_fps",     &Camera::actual_fps);
}

}  // namespace xense::taccap::python
