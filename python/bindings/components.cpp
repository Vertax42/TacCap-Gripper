// Copyright (c) 2026 XenseRobotics Co., Ltd. — Apache-2.0
//
// pybind11 bindings for the component classes (IMU, Encoder, Camera,
// LeaderGripper, FollowerGripper) and their POD samples.
//
// Sample structs use numpy arrays for vector fields so users get the
// expected `s.accel_mps2[0]` / `np.linalg.norm(s.accel_mps2)` ergonomics
// without an extra Python conversion step.

#include <pybind11/pybind11.h>

#include "gil_safe.hpp"
#include <pybind11/stl.h>
#include <pybind11/numpy.h>
#include <pybind11/functional.h>

#include <taccap/components/calibration.hpp>
#include <taccap/components/imu.hpp>
#include <taccap/components/encoder.hpp>
#include <taccap/components/camera.hpp>
#include <taccap/components/fisheye_undistorter.hpp>
#include <taccap/components/key.hpp>
#include <taccap/components/sensor_errors.hpp>
#include <taccap/components/motor.hpp>
#include <taccap/components/led.hpp>
#include <taccap/control_loop.hpp>
#include <taccap/follower_gripper.hpp>
#include <taccap/leader_gripper.hpp>
#include <taccap/discovery.hpp>
#include <taccap/ota.hpp>

#include <chrono>
#include <memory>
#include <optional>

namespace py = pybind11;

// GIL-safe callback ownership/invocation is shared with module.cpp.
using xense::taccap::python::gil::call_into_python;
using xense::taccap::python::gil::interpreter_gone;
using xense::taccap::python::gil::make_gil_safe_callback;

namespace {

// ---- helpers --------------------------------------------------------------

// Convert std::chrono::steady_clock::time_point to seconds-since-epoch float.
// We expose a monotonic-ish double; users that need wall-clock can compare
// against time.monotonic() in Python.
double tp_to_seconds(std::chrono::steady_clock::time_point tp) {
    using ns = std::chrono::nanoseconds;
    return std::chrono::duration_cast<ns>(tp.time_since_epoch()).count() * 1e-9;
}

// Wrap a std::array<float, 3> as a numpy float32 array of shape (3,).
//
// The shape MUST be spelled as a container. `py::array_t<float> arr(3)` picks
// a different overload on pybind11 2.9 and yields a shape-(3,) array with
// stride 0 — a broadcast view of element [0]. Every value written to p[1] and
// p[2] lands on the same address, so `accel_mps2` read back as [x, x, x].
py::array make_vec3(const std::array<float, 3>& v) {
    py::array_t<float> arr(std::vector<py::ssize_t>{3});
    auto* p = arr.mutable_data();
    p[0] = v[0]; p[1] = v[1]; p[2] = v[2];
    return arr;
}

// Wrap a cv::Mat (BGR8 expected) as a (H, W, 3) uint8 numpy array. This
// makes a copy so the array is safe across frame boundaries.
py::array mat_to_numpy(const cv::Mat& m) {
    if (m.empty()) {
        return py::array_t<uint8_t>({0, 0, 3});
    }
    if (m.type() != CV_8UC3) {
        // For now we only handle 8-bit 3-channel; advanced users can use
        // OpenCV directly. Fall back to empty.
        return py::array_t<uint8_t>({0, 0, 3});
    }
    py::array_t<uint8_t> arr({m.rows, m.cols, 3});
    auto* dst = arr.mutable_data();
    if (m.isContinuous()) {
        std::memcpy(dst, m.data, static_cast<size_t>(m.rows) * m.cols * 3);
    } else {
        for (int r = 0; r < m.rows; ++r) {
            std::memcpy(dst + r * m.cols * 3, m.ptr(r), m.cols * 3);
        }
    }
    return arr;
}

// Borrow a (H, W, 3) uint8 numpy array as a cv::Mat WITHOUT copying. The
// caller must keep `arr` alive for as long as the Mat is used — every use
// here is a synchronous read inside one binding call, so that holds.
//
// Deliberately strict rather than forcecast: silently casting a float image to
// uint8 would produce a plausible-looking but wrong result.
cv::Mat numpy_to_mat_bgr(const py::array& arr) {
    if (!py::dtype::of<uint8_t>().is(arr.dtype())) {
        throw std::invalid_argument(
            "expected a uint8 array (BGR8); got dtype " +
            py::str(arr.dtype()).cast<std::string>());
    }
    if (arr.ndim() != 3 || arr.shape(2) != 3) {
        throw std::invalid_argument(
            "expected shape (H, W, 3); got ndim=" + std::to_string(arr.ndim()));
    }
    if ((arr.flags() & py::array::c_style) == 0) {
        throw std::invalid_argument(
            "expected a C-contiguous array; pass numpy.ascontiguousarray(img)");
    }
    return cv::Mat(static_cast<int>(arr.shape(0)),
                   static_cast<int>(arr.shape(1)),
                   CV_8UC3,
                   const_cast<void*>(arr.data()));
}

}  // namespace

namespace xense::taccap::python {

void bind_components(py::module_& m) {
    using namespace xense::taccap;

    // ---- ImuSample / EncoderSample -------------------------------------
    py::class_<ImuSample>(m, "ImuSample")
        .def_property_readonly("host_time", [](const ImuSample& s) {
            return tp_to_seconds(s.host_time);
        })
        .def_readonly("mcu_timestamp_us", &ImuSample::mcu_timestamp_us)
        .def_readonly("valid_flag",       &ImuSample::valid_flag)
        .def_readonly("seq",              &ImuSample::seq)
        .def_readonly("temperature_c",    &ImuSample::temperature_c)
        .def_property_readonly("accel_mps2", [](const ImuSample& s) { return make_vec3(s.accel_mps2); })
        .def_property_readonly("gyro_radps", [](const ImuSample& s) { return make_vec3(s.gyro_radps); })
        .def_property_readonly("mag_uT",     [](const ImuSample& s) { return make_vec3(s.mag_uT); })
        .def("__repr__", [](const ImuSample& s) {
            char buf[160];
            std::snprintf(buf, sizeof(buf),
                "ImuSample(seq=%u, accel=[%.2f,%.2f,%.2f], gyro=[%.3f,%.3f,%.3f], temp=%.2fC)",
                s.seq, s.accel_mps2[0], s.accel_mps2[1], s.accel_mps2[2],
                s.gyro_radps[0], s.gyro_radps[1], s.gyro_radps[2], s.temperature_c);
            return std::string(buf);
        });

    py::class_<EncoderSample>(m, "EncoderSample")
        .def_property_readonly("host_time", [](const EncoderSample& s) {
            return tp_to_seconds(s.host_time);
        })
        .def_readonly("mcu_timestamp_us", &EncoderSample::mcu_timestamp_us)
        // position_rad is the user-facing reading after SDK-side
        // normalisation: clamped to >= 0 to absorb small post-zero
        // drift. raw_position_rad exposes the unclamped firmware value
        // so calibration / diagnostic tooling can still see the drift.
        .def_readonly("position_rad",     &EncoderSample::position_rad)
        .def_readonly("velocity_rad_s",   &EncoderSample::velocity_rad_s)
        // Normalized opening in [0, 1] (0 = fully closed, 1 = fully open).
        // float('nan') unless the Encoder has a position map installed —
        // LeaderGripper(normalize_position=True) does that automatically.
        // position_rad stays in radians either way.
        .def_readonly("position",         &EncoderSample::position)
        .def_property_readonly("raw_position_rad",
                               [](const EncoderSample& s) { return s.raw.position_rad; })
        .def_property_readonly("raw_velocity_rad_s",
                               [](const EncoderSample& s) { return s.raw.velocity_rad_s; })
        .def_readonly("status",           &EncoderSample::status)
        .def_readonly("seq",              &EncoderSample::seq)
        .def("__repr__", [](const EncoderSample& s) {
            char buf[192];
            std::snprintf(buf, sizeof(buf),
                "EncoderSample(seq=%u, pos=%.4frad (raw=%.4f), vel=%.4frad/s, "
                "position=%.4f)",
                s.seq, s.position_rad, s.raw.position_rad, s.velocity_rad_s,
                s.position);
            return std::string(buf);
        });

    // ---- CameraFrame ----------------------------------------------------
    py::class_<CameraFrame>(m, "CameraFrame")
        .def_property_readonly("host_time", [](const CameraFrame& f) {
            return tp_to_seconds(f.host_time);
        })
        .def_readonly("frame_index", &CameraFrame::frame_index)
        .def_property_readonly("image", [](const CameraFrame& f) { return mat_to_numpy(f.image); });

    // ---- IMU ------------------------------------------------------------
    py::class_<IMU>(m, "IMU")
        .def("read_once", [](IMU& self, unsigned timeout_ms) {
            py::gil_scoped_release gil;
            return self.read_once(std::chrono::milliseconds(timeout_ms));
        }, py::arg("timeout_ms") = 100)
        .def("on_data", [](IMU& self, py::function pycb) {
            auto cb = make_gil_safe_callback(std::move(pycb));
            return self.on_data([cb](const ImuSample& s) {
                call_into_python("xense.taccap.IMU callback",
                                 [&] { (*cb)(s); });
            });
        }, py::arg("callback"))
        .def("off", &IMU::off, py::arg("subscription_id"))
        .def("set_mag_calibration", [](IMU& self,
                                       std::array<float, 3> hard,
                                       std::array<float, 9> soft,
                                       unsigned timeout_ms) {
            py::gil_scoped_release gil;
            self.set_mag_calibration(hard, soft,
                                     std::chrono::milliseconds(timeout_ms));
        },
            py::arg("hard_iron"),
            py::arg("soft_iron_row_major"),
            py::arg("timeout_ms") = 500u);

    // ---- KeySample + Key (V1.4) ----------------------------------------
    py::class_<KeySample>(m, "KeySample")
        .def_property_readonly("host_time", [](const KeySample& s) {
            return tp_to_seconds(s.host_time);
        })
        .def_readonly("key_id",    &KeySample::key_id)
        .def_readonly("key_state", &KeySample::key_state)
        .def("__repr__", [](const KeySample& s) {
            char buf[64];
            std::snprintf(buf, sizeof(buf),
                "KeySample(key_id=%u, key_state=%u)", s.key_id, s.key_state);
            return std::string(buf);
        });
    py::class_<Key>(m, "Key")
        .def("on_event", [](Key& self, py::function pycb) {
            auto cb = make_gil_safe_callback(std::move(pycb));
            return self.on_event([cb](const KeySample& s) {
                call_into_python("xense.taccap.Key callback",
                                 [&] { (*cb)(s); });
            });
        }, py::arg("callback"))
        .def("off", &Key::off, py::arg("subscription_id"));
    // KeyState constants for ergonomic comparison from Python.
    py::module_ key_state_mod = m.def_submodule("KeyState",
        "TC-GU-01 button state constants (V1.4).");
    key_state_mod.attr("SingleClickDown") = py::int_(xense::taccap::protocol::KeyState::SingleClickDown);
    key_state_mod.attr("SingleClickUp")   = py::int_(xense::taccap::protocol::KeyState::SingleClickUp);
    key_state_mod.attr("DoubleClick")     = py::int_(xense::taccap::protocol::KeyState::DoubleClick);
    key_state_mod.attr("LongPressDown")   = py::int_(xense::taccap::protocol::KeyState::LongPressDown);
    key_state_mod.attr("LongPressUp")     = py::int_(xense::taccap::protocol::KeyState::LongPressUp);

    // ---- Led (WS2812, V1.9) --------------------------------------------
    py::enum_<protocol::Ws2812Mode>(m, "Ws2812Mode")
        .value("Off",       protocol::Ws2812Mode::Off)
        .value("EffectSet", protocol::Ws2812Mode::EffectSet)
        .value("Override",  protocol::Ws2812Mode::Override);
    py::enum_<protocol::Ws2812EffectType>(m, "Ws2812EffectType")
        .value("None_",       protocol::Ws2812EffectType::None)
        .value("NormalSolid", protocol::Ws2812EffectType::NormalSolid)
        .value("NormalBlink", protocol::Ws2812EffectType::NormalBlink)
        .value("OtaBlink",    protocol::Ws2812EffectType::OtaBlink)
        .value("FaultBlink",  protocol::Ws2812EffectType::FaultBlink)
        .value("Demo",        protocol::Ws2812EffectType::Demo)
        .value("ColorBlink",  protocol::Ws2812EffectType::ColorBlink)
        .value("ColorBreathe",protocol::Ws2812EffectType::ColorBreathe)
        .value("HsvCycle",    protocol::Ws2812EffectType::HsvCycle)
        .value("ColorLerp",   protocol::Ws2812EffectType::ColorLerp);
    py::class_<Led>(m, "Led")
        .def("set", [](Led& self, protocol::Ws2812Mode mode, uint8_t r, uint8_t g,
                       uint8_t b, uint8_t brightness, uint16_t blink_ms) {
            py::gil_scoped_release gil;
            self.set(mode, r, g, b, brightness, blink_ms);
        }, py::arg("mode"), py::arg("r"), py::arg("g"), py::arg("b"),
           py::arg("brightness") = 0, py::arg("blink_ms") = 0)
        .def("off", [](Led& self) { py::gil_scoped_release gil; self.off(); })
        .def("effect", [](Led& self, protocol::Ws2812EffectType eff, uint8_t r1,
                          uint8_t g1, uint8_t b1, uint16_t period_ms, uint8_t r2,
                          uint8_t g2, uint8_t b2, uint8_t param1, uint8_t param2) {
            py::gil_scoped_release gil;
            self.effect(eff, r1, g1, b1, period_ms, r2, g2, b2, param1, param2);
        }, py::arg("effect"), py::arg("r1"), py::arg("g1"), py::arg("b1"),
           py::arg("period_ms") = 1000, py::arg("r2") = 0, py::arg("g2") = 0,
           py::arg("b2") = 0, py::arg("param1") = 0, py::arg("param2") = 0)
        .def("effect_off", [](Led& self) { py::gil_scoped_release gil; self.effect_off(); });

    // ---- SensorErrorSample + SensorErrors (V1.6) -----------------------
    py::class_<SensorErrorSample>(m, "SensorErrorSample")
        .def_property_readonly("host_time", [](const SensorErrorSample& s) {
            return tp_to_seconds(s.host_time);
        })
        .def_readonly("sensor_id",        &SensorErrorSample::sensor_id)
        .def_readonly("error_code",       &SensorErrorSample::error_code)
        .def_readonly("error_count",      &SensorErrorSample::error_count)
        .def_readonly("mcu_timestamp_ms", &SensorErrorSample::mcu_timestamp_ms)
        .def("__repr__", [](const SensorErrorSample& s) {
            char buf[96];
            std::snprintf(buf, sizeof(buf),
                "SensorErrorSample(sensor=%u, code=0x%02x, count=%u, ts_ms=%u)",
                s.sensor_id, s.error_code, s.error_count, s.mcu_timestamp_ms);
            return std::string(buf);
        });
    py::class_<SensorErrors>(m, "SensorErrors")
        .def("on_report", [](SensorErrors& self, py::function pycb) {
            auto cb = make_gil_safe_callback(std::move(pycb));
            return self.on_report([cb](const SensorErrorSample& s) {
                call_into_python("xense.taccap.SensorErrors callback",
                                 [&] { (*cb)(s); });
            });
        }, py::arg("callback"))
        .def("off", &SensorErrors::off, py::arg("subscription_id"));

    // ---- OtaSession (V1.3) ---------------------------------------------
    // OtaStatus is the wire payload returned by Cmd::OtaGetStatus; bound
    // here so `OtaSession.get_status()` can hand it back to Python.
    py::class_<xense::taccap::protocol::OtaStatus>(m, "OtaStatus")
        .def_readonly("state",         &xense::taccap::protocol::OtaStatus::state)
        .def_readonly("error_code",    &xense::taccap::protocol::OtaStatus::error_code)
        .def_readonly("bytes_written", &xense::taccap::protocol::OtaStatus::bytes_written)
        .def_readonly("progress_ppt",  &xense::taccap::protocol::OtaStatus::progress_ppt)
        .def("__repr__", [](const xense::taccap::protocol::OtaStatus& s) {
            char buf[96];
            std::snprintf(buf, sizeof(buf),
                "OtaStatus(state=%u, err=0x%02X, bytes=%u, ppt=%u)",
                s.state, s.error_code, s.bytes_written, s.progress_ppt);
            return std::string(buf);
        });

    py::class_<OtaSession::TargetVersion>(m, "OtaTargetVersion")
        .def(py::init<>())
        .def(py::init<uint8_t, uint8_t, uint8_t, uint8_t>(),
             py::arg("major"), py::arg("minor"), py::arg("patch"), py::arg("build"))
        .def_readwrite("major", &OtaSession::TargetVersion::major)
        .def_readwrite("minor", &OtaSession::TargetVersion::minor)
        .def_readwrite("patch", &OtaSession::TargetVersion::patch)
        .def_readwrite("build", &OtaSession::TargetVersion::build);

    py::class_<OtaSession>(m, "OtaSession")
        .def("update_from_file", [](OtaSession& self,
                                    const std::string& path,
                                    const OtaSession::TargetVersion& v,
                                    py::object on_progress) {
            OtaSession::ProgressCallback cb;
            if (!on_progress.is_none()) {
                auto pycb = make_gil_safe_callback(py::function(on_progress));
                cb = [pycb](uint32_t wr, uint32_t tot) {
                    call_into_python("OtaSession progress",
                                     [&] { (*pycb)(wr, tot); });
                };
            }
            py::gil_scoped_release gil;
            self.update_from_file(path, v, std::move(cb));
        },
            py::arg("firmware_path"),
            py::arg("target_version"),
            py::arg("on_progress") = py::none())
        .def("update_from_bytes", [](OtaSession& self,
                                     py::bytes blob,
                                     const OtaSession::TargetVersion& v,
                                     py::object on_progress) {
            // Materialise the bytes view; copy into std::vector once.
            std::string buf = blob;
            std::vector<uint8_t> fw(buf.begin(), buf.end());
            OtaSession::ProgressCallback cb;
            if (!on_progress.is_none()) {
                auto pycb = make_gil_safe_callback(py::function(on_progress));
                cb = [pycb](uint32_t wr, uint32_t tot) {
                    call_into_python("OtaSession progress",
                                     [&] { (*pycb)(wr, tot); });
                };
            }
            py::gil_scoped_release gil;
            self.update_from_bytes(fw, v, std::move(cb));
        },
            py::arg("firmware_bytes"),
            py::arg("target_version"),
            py::arg("on_progress") = py::none())
        .def("get_status", [](OtaSession& self, unsigned timeout_ms) {
            py::gil_scoped_release gil;
            return self.get_status(std::chrono::milliseconds(timeout_ms));
        }, py::arg("timeout_ms") = 500u)
        .def("abort", [](OtaSession& self) {
            py::gil_scoped_release gil;
            self.abort();
        });

    m.def("crc32_iso_hdlc", [](py::buffer b) {
        py::buffer_info info = b.request();
        if (info.itemsize != 1) {
            throw py::value_error("crc32_iso_hdlc: needs a bytes-like buffer");
        }
        return xense::taccap::crc32_iso_hdlc(
            static_cast<const uint8_t*>(info.ptr),
            static_cast<size_t>(info.size));
    }, py::arg("data"),
       "Compute CRC32 with the same parameters as zlib.crc32 / firmware.");

    // ---- V1.7 follower (slave) types ------------------------------------
    py::enum_<protocol::MotorProtocol>(m, "MotorProtocol")
        .value("Private", protocol::MotorProtocol::Private)
        .value("Mit",     protocol::MotorProtocol::Mit);

    // V2.2 — MotorStatusExt.stop_reason / MotorFaultReport.stop_reason. The
    // fault/monitor *bit masks* stay C++-only, matching MotorStatusBit: Python
    // gets the raw integers and masks them itself.
    py::enum_<protocol::MotorStopReason>(m, "MotorStopReason")
        .value("None_",        protocol::MotorStopReason::None)
        .value("Disable",      protocol::MotorStopReason::Disable)
        .value("Emergency",    protocol::MotorStopReason::Emergency)
        .value("ClearFault",   protocol::MotorStopReason::ClearFault)
        .value("LimitStall",   protocol::MotorStopReason::LimitStall)
        .value("ControlError", protocol::MotorStopReason::ControlError);

    py::class_<protocol::GripperConfig>(m, "GripperConfig")
        .def(py::init([]() {
            protocol::GripperConfig c{};
            c.magic   = protocol::GRIPPER_CONFIG_MAGIC;
            c.version = protocol::GRIPPER_CONFIG_VERSION;
            c.flags   = protocol::GripperConfigFlag::Valid;
            return c;
        }))
        .def_readwrite("magic",        &protocol::GripperConfig::magic)
        .def_readwrite("version",      &protocol::GripperConfig::version)
        .def_readwrite("flags",        &protocol::GripperConfig::flags)
        .def_readwrite("max_open_rad", &protocol::GripperConfig::max_open_rad)
        .def_readwrite("min_open_rad", &protocol::GripperConfig::min_open_rad)
        .def("__repr__", [](const protocol::GripperConfig& c) {
            char buf[128];
            std::snprintf(buf, sizeof(buf),
                "GripperConfig(flags=0x%04x, max_open=%.4frad, min_open=%.4frad)",
                c.flags, c.max_open_rad, c.min_open_rad);
            return std::string(buf);
        });

    // ---- GripperAutoCalConfig (V1.9 power-on auto-cal) -------------------
    py::class_<protocol::GripperAutoCalConfig>(m, "GripperAutoCalConfig")
        .def(py::init([]() {
            protocol::GripperAutoCalConfig c{};
            c.magic   = protocol::GRIPPER_AUTO_CAL_MAGIC;
            c.version = protocol::GRIPPER_AUTO_CAL_VERSION;
            c.flags   = protocol::GripperAutoCalFlag::Valid;
            return c;
        }))
        .def_readwrite("magic",                &protocol::GripperAutoCalConfig::magic)
        .def_readwrite("version",              &protocol::GripperAutoCalConfig::version)
        .def_readwrite("flags",                &protocol::GripperAutoCalConfig::flags)
        .def_readwrite("close_stall_torque_nm", &protocol::GripperAutoCalConfig::close_stall_torque_nm)
        .def_readwrite("open_stall_torque_nm",  &protocol::GripperAutoCalConfig::open_stall_torque_nm)
        .def_readwrite("close_speed_rad_s",    &protocol::GripperAutoCalConfig::close_speed_rad_s)
        .def_readwrite("open_speed_rad_s",     &protocol::GripperAutoCalConfig::open_speed_rad_s)
        .def_readwrite("stall_hold_ms",        &protocol::GripperAutoCalConfig::stall_hold_ms)
        .def_readwrite("startup_delay_ms",     &protocol::GripperAutoCalConfig::startup_delay_ms)
        .def_readwrite("post_zero_delay_ms",   &protocol::GripperAutoCalConfig::post_zero_delay_ms)
        .def_readwrite("close_confirm_count",  &protocol::GripperAutoCalConfig::close_confirm_count)
        .def_readwrite("open_confirm_count",   &protocol::GripperAutoCalConfig::open_confirm_count)
        .def("__repr__", [](const protocol::GripperAutoCalConfig& c) {
            char buf[160];
            std::snprintf(buf, sizeof(buf),
                "GripperAutoCalConfig(flags=0x%04x, close_torque=%.3f, open_torque=%.3f)",
                c.flags, c.close_stall_torque_nm, c.open_stall_torque_nm);
            return std::string(buf);
        });

    // ---- GripperAutoCalStallParam / …Ex (V2.2 partial 0x68 write) ---------
    // Patch just the stall-detection fields; speeds, flags and the magic/version
    // header keep their stored values. No magic/version to fill in — the
    // firmware supplies them.
    py::class_<protocol::GripperAutoCalStallParam>(m, "GripperAutoCalStallParam")
        .def(py::init([]() { return protocol::GripperAutoCalStallParam{}; }))
        .def_readwrite("close_stall_torque_nm",
                       &protocol::GripperAutoCalStallParam::close_stall_torque_nm)
        .def_readwrite("open_stall_torque_nm",
                       &protocol::GripperAutoCalStallParam::open_stall_torque_nm)
        .def_readwrite("stall_hold_ms",
                       &protocol::GripperAutoCalStallParam::stall_hold_ms)
        .def("__repr__", [](const protocol::GripperAutoCalStallParam& p) {
            char buf[128];
            std::snprintf(buf, sizeof(buf),
                "GripperAutoCalStallParam(close=%.3fNm, open=%.3fNm, hold=%ums)",
                p.close_stall_torque_nm, p.open_stall_torque_nm, p.stall_hold_ms);
            return std::string(buf);
        });

    py::class_<protocol::GripperAutoCalStallParamEx>(m, "GripperAutoCalStallParamEx")
        .def(py::init([]() { return protocol::GripperAutoCalStallParamEx{}; }))
        .def_readwrite("close_stall_torque_nm",
                       &protocol::GripperAutoCalStallParamEx::close_stall_torque_nm)
        .def_readwrite("open_stall_torque_nm",
                       &protocol::GripperAutoCalStallParamEx::open_stall_torque_nm)
        .def_readwrite("stall_hold_ms",
                       &protocol::GripperAutoCalStallParamEx::stall_hold_ms)
        .def_readwrite("startup_delay_ms",
                       &protocol::GripperAutoCalStallParamEx::startup_delay_ms)
        .def_readwrite("post_zero_delay_ms",
                       &protocol::GripperAutoCalStallParamEx::post_zero_delay_ms)
        .def_readwrite("close_confirm_count",
                       &protocol::GripperAutoCalStallParamEx::close_confirm_count)
        .def_readwrite("open_confirm_count",
                       &protocol::GripperAutoCalStallParamEx::open_confirm_count)
        .def("__repr__", [](const protocol::GripperAutoCalStallParamEx& p) {
            char buf[160];
            std::snprintf(buf, sizeof(buf),
                "GripperAutoCalStallParamEx(close=%.3fNm, open=%.3fNm, hold=%ums, "
                "startup_delay=%ums)",
                p.close_stall_torque_nm, p.open_stall_torque_nm,
                p.stall_hold_ms, p.startup_delay_ms);
            return std::string(buf);
        });

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

    // ---- MotorPrivateParam (V1.9+ private-protocol param GET response) ----
    py::class_<protocol::MotorPrivateParam>(m, "MotorPrivateParam")
        .def_readonly("index",     &protocol::MotorPrivateParam::index)
        .def_readonly("type",      &protocol::MotorPrivateParam::type)     // 1=u8, 2=f32
        .def_readonly("access",    &protocol::MotorPrivateParam::access)   // 0x01=R, 0x02=W
        .def_readonly("raw_value", &protocol::MotorPrivateParam::raw_value)
        .def_property_readonly("as_float", [](const protocol::MotorPrivateParam& p) {
            float f; std::memcpy(&f, &p.raw_value, 4); return f;
        })
        .def("__repr__", [](const protocol::MotorPrivateParam& p) {
            char buf[96];
            std::snprintf(buf, sizeof(buf),
                "MotorPrivateParam(index=0x%04x, type=%u, access=0x%02x, raw=0x%08x)",
                p.index, p.type, p.access, p.raw_value);
            return std::string(buf);
        });

    py::class_<protocol::MotorControlStats>(m, "MotorControlStats")
        .def_readonly("running",             &protocol::MotorControlStats::running)
        .def_readonly("mode",                &protocol::MotorControlStats::mode)
        .def_readonly("target_hz",           &protocol::MotorControlStats::target_hz)
        .def_readonly("period_ms",           &protocol::MotorControlStats::period_ms)
        .def_readonly("sample_ms",           &protocol::MotorControlStats::sample_ms)
        .def_readonly("actual_hz",           &protocol::MotorControlStats::actual_hz)
        .def_readonly("target_seq",          &protocol::MotorControlStats::target_seq)
        .def_readonly("applied_seq",         &protocol::MotorControlStats::applied_seq)
        .def_readonly("loop_count",          &protocol::MotorControlStats::loop_count)
        .def_readonly("error_count",         &protocol::MotorControlStats::error_count)
        .def_readonly("deadline_miss_count", &protocol::MotorControlStats::deadline_miss_count)
        .def_readonly("timeout_count",       &protocol::MotorControlStats::timeout_count)
        .def_readonly("last_error",          &protocol::MotorControlStats::last_error)
        .def_readonly("target_age_ms",       &protocol::MotorControlStats::target_age_ms)
        .def_readonly("target_update_hz",    &protocol::MotorControlStats::target_update_hz)
        .def("__repr__", [](const protocol::MotorControlStats& s) {
            char buf[160];
            std::snprintf(buf, sizeof(buf),
                "MotorControlStats(running=%d, mode=%d, actual_hz=%.1f, loops=%u, errors=%u)",
                s.running, s.mode, s.actual_hz,
                static_cast<unsigned>(s.loop_count),
                static_cast<unsigned>(s.error_count));
            return std::string(buf);
        });

    // ---- MotorStatusExt (V2.2 — Cmd 0x53, 72 bytes) ----------------------
    // Bytes 0..30 mirror MotorStatusSample; the rest is the fault / monitor
    // extension. `monitor_flags` gates the fault fields — check it first.
    py::class_<protocol::MotorStatusExt>(m, "MotorStatusExt")
        .def_readonly("actual_pos",          &protocol::MotorStatusExt::actual_pos)
        .def_readonly("actual_vel",          &protocol::MotorStatusExt::actual_vel)
        .def_readonly("actual_torque",       &protocol::MotorStatusExt::actual_torque)
        .def_readonly("motor_temp_c",        &protocol::MotorStatusExt::motor_temp)
        .def_readonly("status",              &protocol::MotorStatusExt::status)
        .def_readonly("target_pos",          &protocol::MotorStatusExt::target_pos)
        .def_readonly("target_vel",          &protocol::MotorStatusExt::target_vel)
        .def_readonly("target_torque",       &protocol::MotorStatusExt::target_torque)
        .def_readonly("control_mode",        &protocol::MotorStatusExt::control_mode)
        .def_readonly("monitor_version",     &protocol::MotorStatusExt::monitor_version)
        .def_readonly("monitor_flags",       &protocol::MotorStatusExt::monitor_flags)
        .def_readonly("stop_reason",         &protocol::MotorStatusExt::stop_reason)
        .def_readonly("monitor_reserved",    &protocol::MotorStatusExt::monitor_reserved)
        .def_readonly("fault_code",          &protocol::MotorStatusExt::fault_code)
        .def_readonly("latched_fault_code",  &protocol::MotorStatusExt::latched_fault_code)
        .def_readonly("fault_timestamp_ms",  &protocol::MotorStatusExt::fault_timestamp_ms)
        .def_readonly("status_timestamp_ms", &protocol::MotorStatusExt::status_timestamp_ms)
        .def_readonly("stop_fault_code",     &protocol::MotorStatusExt::stop_fault_code)
        .def_readonly("stop_timestamp_ms",   &protocol::MotorStatusExt::stop_timestamp_ms)
        .def_readonly("fault_can_id",        &protocol::MotorStatusExt::fault_can_id)
        .def_readonly("fault_can_dlc",       &protocol::MotorStatusExt::fault_can_dlc)
        // Raw C array -> bytes; def_readonly on uint8_t[8] would expose an
        // opaque proxy that does not survive the owning struct.
        .def_property_readonly("fault_can_data", [](const protocol::MotorStatusExt& s) {
            return py::bytes(reinterpret_cast<const char*>(s.fault_can_data),
                             sizeof(s.fault_can_data));
        })
        .def("__repr__", [](const protocol::MotorStatusExt& s) {
            char buf[200];
            std::snprintf(buf, sizeof(buf),
                "MotorStatusExt(pos=%.4frad, torque=%.3fNm, status=0x%04x, "
                "fault=0x%08x, latched=0x%08x, monitor=0x%02x, stop_reason=%u)",
                s.actual_pos, s.actual_torque, s.status,
                static_cast<unsigned>(s.fault_code),
                static_cast<unsigned>(s.latched_fault_code),
                s.monitor_flags, s.stop_reason);
            return std::string(buf);
        });

    // ---- MotorFaultReport (V2.2 — Cmd 0x52, 64 bytes) --------------------
    py::class_<protocol::MotorFaultReport>(m, "MotorFaultReport")
        .def_readonly("version",           &protocol::MotorFaultReport::version)
        .def_readonly("report_flags",      &protocol::MotorFaultReport::report_flags)
        .def_readonly("source_mask",       &protocol::MotorFaultReport::source_mask)
        .def_readonly("protocol_mode",     &protocol::MotorFaultReport::protocol_mode)
        .def_readonly("event_seq",         &protocol::MotorFaultReport::event_seq)
        .def_readonly("timestamp_ms",      &protocol::MotorFaultReport::timestamp_ms)
        .def_readonly("motor_fault_code",  &protocol::MotorFaultReport::motor_fault_code)
        .def_readonly("motor_latched_fault_code",
                      &protocol::MotorFaultReport::motor_latched_fault_code)
        .def_readonly("stop_fault_code",   &protocol::MotorFaultReport::stop_fault_code)
        .def_readonly("firmware_fault_code",
                      &protocol::MotorFaultReport::firmware_fault_code)
        .def_readonly("firmware_latched_fault_code",
                      &protocol::MotorFaultReport::firmware_latched_fault_code)
        .def_readonly("firmware_detail_code",
                      &protocol::MotorFaultReport::firmware_detail_code)
        .def_readonly("fault_can_id",      &protocol::MotorFaultReport::fault_can_id)
        .def_readonly("fault_can_dlc",     &protocol::MotorFaultReport::fault_can_dlc)
        .def_readonly("monitor_flags",     &protocol::MotorFaultReport::monitor_flags)
        .def_readonly("monitor_reserved",  &protocol::MotorFaultReport::monitor_reserved)
        .def_readonly("fault_timestamp_ms",
                      &protocol::MotorFaultReport::fault_timestamp_ms)
        .def_readonly("status_timestamp_ms",
                      &protocol::MotorFaultReport::status_timestamp_ms)
        .def_readonly("stop_reason",       &protocol::MotorFaultReport::stop_reason)
        .def_property_readonly("fault_can_data", [](const protocol::MotorFaultReport& r) {
            return py::bytes(reinterpret_cast<const char*>(r.fault_can_data),
                             sizeof(r.fault_can_data));
        })
        .def("__repr__", [](const protocol::MotorFaultReport& r) {
            char buf[200];
            std::snprintf(buf, sizeof(buf),
                "MotorFaultReport(flags=0x%02x, sources=0x%02x, motor=0x%08x, "
                "fw=0x%08x, detail=%d, stop_reason=%u)",
                r.report_flags, r.source_mask,
                static_cast<unsigned>(r.motor_fault_code),
                static_cast<unsigned>(r.firmware_fault_code),
                static_cast<int>(r.firmware_detail_code), r.stop_reason);
            return std::string(buf);
        });

    // ---- MotorStatusSample ----------------------------------------------
    py::class_<MotorStatusSample>(m, "MotorStatusSample")
        .def_property_readonly("host_time", [](const MotorStatusSample& s) {
            return tp_to_seconds(s.host_time);
        })
        .def_readonly("actual_pos",     &MotorStatusSample::actual_pos)
        .def_readonly("actual_vel",     &MotorStatusSample::actual_vel)
        .def_readonly("actual_torque",  &MotorStatusSample::actual_torque)
        .def_readonly("motor_temp_c",   &MotorStatusSample::motor_temp_c)
        .def_readonly("status",         &MotorStatusSample::status)
        // target_* / control_mode: correct on V1.9 firmware (31-byte status).
        .def_readonly("target_pos",     &MotorStatusSample::target_pos)
        .def_readonly("target_vel",     &MotorStatusSample::target_vel)
        .def_readonly("target_torque",  &MotorStatusSample::target_torque)
        .def_readonly("control_mode",   &MotorStatusSample::control_mode)
        .def("__repr__", [](const MotorStatusSample& s) {
            char buf[160];
            std::snprintf(buf, sizeof(buf),
                "MotorStatusSample(pos=%.4frad, vel=%.4frad/s, torque=%.3fNm, temp=%.1fC, status=0x%04x)",
                s.actual_pos, s.actual_vel, s.actual_torque, s.motor_temp_c, s.status);
            return std::string(buf);
        });

    // ---- Encoder --------------------------------------------------------
    py::class_<Encoder>(m, "Encoder")
        .def("read_once", [](Encoder& self, unsigned timeout_ms) {
            py::gil_scoped_release gil;
            return self.read_once(std::chrono::milliseconds(timeout_ms));
        }, py::arg("timeout_ms") = 100)
        .def("on_data", [](Encoder& self, py::function pycb) {
            auto cb = make_gil_safe_callback(std::move(pycb));
            return self.on_data([cb](const EncoderSample& s) {
                call_into_python("xense.taccap.Encoder callback",
                                 [&] { (*cb)(s); });
            });
        }, py::arg("callback"))
        .def("off", &Encoder::off, py::arg("subscription_id"))
        .def("set_zero", [](Encoder& self, unsigned timeout_ms) {
            py::gil_scoped_release gil;
            self.set_zero(std::chrono::milliseconds(timeout_ms));
        },
            py::arg("timeout_ms") = 500u,
            "Latch the current encoder reading as the new zero position. "
            "The gripper must already be held at the desired zero pose "
            "(e.g. fully closed) before calling. Raises on NACK / timeout.")
        .def("set_position_map", &Encoder::set_position_map, py::arg("position_map"),
             "Install the raw-rad -> [0,1] converter that fills "
             "EncoderSample.position. Applies to read_once() and to every "
             "already-registered on_data() subscriber. Raises ProtocolError if "
             "the map is not valid.")
        .def("clear_position_map", &Encoder::clear_position_map,
             "Remove the converter; EncoderSample.position goes back to nan.")
        .def_property_readonly("has_position_map", &Encoder::has_position_map)
        .def_property_readonly("position_map", &Encoder::position_map,
             "Copy of the installed converter; .valid is False when none is "
             "installed.");

    // ---- Motor ----------------------------------------------------------
    py::class_<Motor>(m, "Motor")
        .def("enable",      [](Motor& self) { py::gil_scoped_release g; self.enable();      })
        .def("disable",     [](Motor& self) { py::gil_scoped_release g; self.disable();     })
        .def("clear_fault", [](Motor& self) { py::gil_scoped_release g; self.clear_fault(); })
        .def("set_position", [](Motor& self, float pos, float max_vel, float max_torque) {
                py::gil_scoped_release g;
                self.set_position(pos, max_vel, max_torque);
            },
            py::arg("target_pos_rad"),
            py::arg("max_vel_radps"),
            py::arg("max_torque_nm"))
        .def("set_velocity", [](Motor& self, float vel, float max_torque, float profile_acc) {
                py::gil_scoped_release g;
                self.set_velocity(vel, max_torque, profile_acc);
            },
            py::arg("target_vel_radps"),
            py::arg("max_torque_nm"),
            py::arg("profile_acc_radps2"))
        .def("set_torque", [](Motor& self, float torque, float max_vel) {
                py::gil_scoped_release g;
                self.set_torque(torque, max_vel);
            },
            py::arg("target_torque_nm"),
            py::arg("max_vel_radps"))
        .def("set_impedance", [](Motor& self, float pos, float kp, float kd,
                                 float ff, float ff_vel) {
                py::gil_scoped_release g;
                self.set_impedance(pos, kp, kd, ff, ff_vel);
            },
            py::arg("target_pos_rad"),
            py::arg("kp_nm_per_rad"),
            py::arg("kd_nm_s_per_rad"),
            py::arg("feedforward_torque_nm"),
            py::arg("feedforward_vel_radps") = 0.0f)  // V1.7; MIT only
        // ---- High-rate control submission (no ACK) -------------------------
        // Fire-and-forget for a realtime loop (up to the firmware's 500Hz). No
        // ACK, no NACK, no retry, no throw on a rejected target — the only
        // exception is IoError on a stopped transport. MIT is assumed. Poll
        // control_stats() (off the loop) for health: applied_seq, actual_hz,
        // error_count, last_error. The follow/teleop loop + grasp FSM live in
        // the upper layer (taccap_gripper_ros2), not here.
        .def("submit_impedance", [](Motor& self, float pos, float kp, float kd,
                                    float ff, float ff_vel) {
                py::gil_scoped_release g;
                self.submit_impedance(pos, kp, kd, ff, ff_vel);
            },
            py::arg("target_pos_rad"),
            py::arg("kp_nm_per_rad"),
            py::arg("kd_nm_s_per_rad"),
            py::arg("feedforward_torque_nm"),
            py::arg("feedforward_vel_radps") = 0.0f)
        .def("submit_position", [](Motor& self, float pos, float max_vel, float max_torque) {
                py::gil_scoped_release g;
                self.submit_position(pos, max_vel, max_torque);
            },
            py::arg("target_pos_rad"),
            py::arg("max_vel_radps"),
            py::arg("max_torque_nm"))
        .def("submit_velocity", [](Motor& self, float vel, float max_torque, float profile_acc) {
                py::gil_scoped_release g;
                self.submit_velocity(vel, max_torque, profile_acc);
            },
            py::arg("target_vel_radps"),
            py::arg("max_torque_nm"),
            py::arg("profile_acc_radps2"))
        .def("submit_torque", [](Motor& self, float torque, float max_vel) {
                py::gil_scoped_release g;
                self.submit_torque(torque, max_vel);
            },
            py::arg("target_torque_nm"),
            py::arg("max_vel_radps"))
        .def("read_status", [](Motor& self, unsigned timeout_ms) {
            py::gil_scoped_release gil;
            return self.read_status(std::chrono::milliseconds(timeout_ms));
        }, py::arg("timeout_ms") = 100)
        .def("on_status", [](Motor& self, py::function pycb) {
            // make_gil_safe_callback, not a bare make_shared: the last
            // reference can die on the transport reader thread, which holds
            // no GIL.
            auto cb = make_gil_safe_callback(std::move(pycb));
            return self.on_status([cb](const MotorStatusSample& s) {
                call_into_python("xense.taccap.Motor callback",
                                 [&] { (*cb)(s); });
            });
        }, py::arg("callback"))
        .def("off", &Motor::off, py::arg("subscription_id"))
        // ---- Follower motor admin (follower-only; validated against hw_v1.1.0)
        .def("set_zero", [](Motor& self) { py::gil_scoped_release g; self.set_zero(); })
        .def("get_can_id", [](Motor& self) { py::gil_scoped_release g; return self.get_can_id(); })
        .def("set_can_id", [](Motor& self, uint8_t id) {
            py::gil_scoped_release g; self.set_can_id(id);
        }, py::arg("can_id"))
        .def("switch_protocol", [](Motor& self, protocol::MotorProtocol p) {
            py::gil_scoped_release g; self.switch_protocol(p);
        }, py::arg("protocol"))
        .def("get_protocol", [](Motor& self) {
            py::gil_scoped_release g; return self.get_protocol();
        })
        // Private-protocol single-parameter access (Cmd 0x38/0x39). NACKs
        // InvalidParam under MIT (the whole SDK assumes MIT).
        .def("get_private_param", [](Motor& self, uint16_t index) {
            py::gil_scoped_release g; return self.get_private_param(index);
        }, py::arg("index"))
        .def("set_private_param", [](Motor& self, uint16_t index, uint32_t raw_value) {
            py::gil_scoped_release g; self.set_private_param(index, raw_value);
        }, py::arg("index"), py::arg("raw_value"))
        .def("control_stats", [](Motor& self, unsigned timeout_ms) {
            py::gil_scoped_release g;
            return self.control_stats(std::chrono::milliseconds(timeout_ms));
        }, py::arg("timeout_ms") = 100)
        // ---- V2.2 diagnostics (follower >= 1.1.2; older firmware NACKs) ----
        .def("read_status_ext", [](Motor& self, unsigned timeout_ms) {
            py::gil_scoped_release g;
            return self.read_status_ext(std::chrono::milliseconds(timeout_ms));
        }, py::arg("timeout_ms") = 100)
        // force=True costs a CAN round trip and can disturb a running control
        // loop — leave it False when polling.
        .def("fault_report", [](Motor& self, bool force, unsigned timeout_ms) {
            py::gil_scoped_release g;
            return self.fault_report(force, std::chrono::milliseconds(timeout_ms));
        }, py::arg("force") = false, py::arg("timeout_ms") = 200)
        .def("set_startup_limit_torque", [](Motor& self, float torque_nm) {
            py::gil_scoped_release g; self.set_startup_limit_torque(torque_nm);
        }, py::arg("torque_nm"))
        .def("get_startup_limit_torque", [](Motor& self) {
            py::gil_scoped_release g; return self.get_startup_limit_torque();
        });

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

    py::class_<Camera>(m, "Camera")
        .def(py::init([](const std::string& dev, int w, int h, double fps, bool mjpg) {
            return std::make_unique<Camera>(Camera::Config{dev, w, h, fps, mjpg});
        }),
            py::arg("device"), py::arg("width") = 640, py::arg("height") = 480,
            py::arg("fps") = 30.0, py::arg("use_mjpg") = true)
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

    // ---- discovery ------------------------------------------------------
    py::enum_<discovery::Side>(m, "Side")
        .value("Left",    discovery::Side::Left)
        .value("Right",   discovery::Side::Right)
        .value("Unknown", discovery::Side::Unknown);

    py::enum_<discovery::Role>(m, "Role")
        .value("Leader",   discovery::Role::Leader)
        .value("Follower", discovery::Role::Follower)
        .value("Unknown",  discovery::Role::Unknown);

    py::class_<discovery::ParsedSerial>(m, "ParsedSerial")
        .def_readonly("raw",      &discovery::ParsedSerial::raw)
        .def_readonly("product",  &discovery::ParsedSerial::product)
        .def_readonly("batch",    &discovery::ParsedSerial::batch)
        .def_property_readonly("line", [](const discovery::ParsedSerial& p) {
            return std::string(1, p.line);
        })
        .def_readonly("sequence", &discovery::ParsedSerial::sequence)
        .def_readonly("side",     &discovery::ParsedSerial::side)
        .def_readonly("role",     &discovery::ParsedSerial::role)
        .def_readonly("valid",    &discovery::ParsedSerial::valid)
        .def("__repr__", [](const discovery::ParsedSerial& p) {
            return std::string("ParsedSerial(raw=") + p.raw +
                   ", product=" + p.product +
                   ", line=" + std::string(1, p.line) +
                   ", seq=" + p.sequence +
                   ", side=" + (p.side ? discovery::to_string(*p.side) : "None") +
                   ", role=" + discovery::to_string(p.role) +
                   ", valid=" + (p.valid ? "True" : "False") + ")";
        });
    m.def("parse_serial", &discovery::parse_serial, py::arg("serial"));

    py::class_<discovery::GripperEndpoints>(m, "GripperEndpoints")
        .def_readonly("side",                 &discovery::GripperEndpoints::side)
        .def_readonly("role",                 &discovery::GripperEndpoints::role)
        .def_readonly("mcu_device",           &discovery::GripperEndpoints::mcu_device)
        .def_readonly("mcu_serial",           &discovery::GripperEndpoints::mcu_serial)
        .def_readonly("firmware_sn",          &discovery::GripperEndpoints::firmware_sn)
        .def("__repr__", [](const discovery::GripperEndpoints& e) {
            return std::string("GripperEndpoints(side=") +
                   discovery::to_string(e.side) +
                   ", role=" + discovery::to_string(e.role) +
                   ", mcu=" + e.mcu_device +
                   " ch343_sn=" + e.mcu_serial +
                   " fw_sn=" + e.firmware_sn + ")";
        });
    m.def("scan_grippers",  &discovery::scan_all);
    m.def("find_one",       &discovery::find_one);
    m.def("find_left",      &discovery::find_left);
    m.def("find_right",     &discovery::find_right);
    m.def("find_leader",    &discovery::find_leader);
    m.def("find_follower",  &discovery::find_follower);

    // ---- LeaderGripper --------------------------------------------------
    py::class_<LeaderGripper>(m, "LeaderGripper")
        .def(py::init([](const std::string& mcu, const std::string& wrist,
                         uint32_t baud, unsigned ack_ms, unsigned retries,
                         bool open_cameras, bool normalize_position,
                         float encoder_max_rad,
                         bool undistort_wrist, float fisheye_balance) {
                LeaderGripper::Config cfg;
                cfg.mcu_device           = mcu;
                cfg.wrist_video          = wrist;
                cfg.baudrate             = baud;
                cfg.ack_timeout_ms       = ack_ms;
                cfg.max_retries          = retries;
                cfg.open_cameras         = open_cameras;
                cfg.normalize_position   = normalize_position;
                cfg.encoder_max_rad      = encoder_max_rad;
                cfg.undistort_wrist      = undistort_wrist;
                cfg.fisheye_balance      = fisheye_balance;
                py::gil_scoped_release gil;
                return std::make_unique<LeaderGripper>(cfg);
             }),
             py::arg("mcu_device"),
             // The wrist camera is off by default; it only matters with open_cameras=True.
             py::arg("wrist_video")         = "",
             py::arg("baudrate")            = 3'000'000u,
             py::arg("ack_timeout_ms")      = 200u,
             py::arg("max_retries")         = 1u,
             py::arg("open_cameras")        = false,
             // Normalized encoder position: fills EncoderSample.position with
             // the opening in [0,1] (0=closed, 1=open) on read_once() and on
             // every streamed sample. position_rad keeps reporting radians.
             // Raises at construction if the firmware has no encoder-max
             // calibration and encoder_max_rad isn't supplied.
             py::arg("normalize_position")  = false,
             py::arg("encoder_max_rad")     = 0.0f,
             // Wrist fisheye undistortion (needs open_cameras=True and a V2.0+
             // firmware that holds a calibration). Degrades to raw frames with
             // a warning when it does not; raises if the camera is not at the
             // calibrated 640x480.
             py::arg("undistort_wrist")     = false,
             py::arg("fisheye_balance")     = 0.0f)
        .def_static("open", [](bool normalize_position, float encoder_max_rad) {
            py::gil_scoped_release gil;
            if (!normalize_position && encoder_max_rad <= 0.0f) {
                return LeaderGripper::open();  // returns unique_ptr<LeaderGripper>
            }
            auto eps = discovery::find_one();
            LeaderGripper::Config cfg{};
            cfg.mcu_device         = eps.mcu_device;
            cfg.normalize_position = normalize_position;
            cfg.encoder_max_rad    = encoder_max_rad;
            return std::make_unique<LeaderGripper>(cfg);
        }, py::arg("normalize_position") = false, py::arg("encoder_max_rad") = 0.0f)
        .def("start_streaming", [](LeaderGripper& self, unsigned imu_hz, unsigned enc_hz) {
            py::gil_scoped_release gil;
            self.start_streaming(imu_hz, enc_hz);
        }, py::arg("imu_hz") = 100u, py::arg("encoder_hz") = 100u)
        .def("stop_streaming", [](LeaderGripper& self) {
            py::gil_scoped_release gil;
            self.stop_streaming();
        })
        .def_property_readonly("imu",           [](LeaderGripper& g) -> IMU&            { return g.imu(); },           py::return_value_policy::reference_internal)
        .def_property_readonly("encoder",       [](LeaderGripper& g) -> Encoder&        { return g.encoder(); },       py::return_value_policy::reference_internal)
        .def_property_readonly("wrist_camera",  [](LeaderGripper& g) -> Camera&         { return g.wrist_camera(); },  py::return_value_policy::reference_internal)
        .def_property_readonly("key",           [](LeaderGripper& g) -> Key&            { return g.key(); },           py::return_value_policy::reference_internal)
        .def_property_readonly("led",           [](LeaderGripper& g) -> Led&            { return g.led(); },           py::return_value_policy::reference_internal)
        .def_property_readonly("sensor_errors", [](LeaderGripper& g) -> SensorErrors&   { return g.sensor_errors(); }, py::return_value_policy::reference_internal)
        .def_property_readonly("calibration",   [](LeaderGripper& g) -> Calibration&    { return g.calibration(); },   py::return_value_policy::reference_internal)
        .def_property_readonly("ota",           [](LeaderGripper& g) -> OtaSession&     { return g.ota(); },           py::return_value_policy::reference_internal)
        .def_property_readonly("transport",     [](LeaderGripper& g) -> bus::Transport& { return g.transport(); },     py::return_value_policy::reference_internal)
        .def_property_readonly("is_streaming",  &LeaderGripper::is_streaming)

        // ---- Normalized position (0 = closed, 1 = open) ------------------
        // Available regardless of the normalize_position flag — that flag only
        // controls whether EncoderSample.position gets filled in. All of these
        // raise ProtocolError when the gripper has no encoder-max calibration.
        .def("position", [](LeaderGripper& g, unsigned timeout_ms) {
                py::gil_scoped_release gil;
                return g.position(std::chrono::milliseconds(timeout_ms));
            }, py::arg("timeout_ms") = 100u,
            "Read the encoder and return the opening normalized to [0,1].")
        .def("pos_to_rad", [](LeaderGripper& g, float p) {
                py::gil_scoped_release gil;
                return g.pos_to_rad(p);
            }, py::arg("position"))
        .def("rad_to_pos", [](LeaderGripper& g, float r) {
                py::gil_scoped_release gil;
                return g.rad_to_pos(r);
            }, py::arg("raw_rad"))
        .def_property_readonly("position_map", [](LeaderGripper& g) {
                py::gil_scoped_release gil;
                return g.position_map();
            }, "Cached raw-rad <-> [0,1] converter (loads on first access).")
        .def("reload_position_map", [](LeaderGripper& g) {
                py::gil_scoped_release gil;
                g.reload_position_map();
            },
            "Re-read the encoder-max calibration and rebuild the converter. "
            "Call after write_encoder_max_rad() or a re-zero.")

        .def("__enter__", [](LeaderGripper& g) -> LeaderGripper& { return g; })
        .def("__exit__",  [](LeaderGripper& g, py::object, py::object, py::object) {
            py::gil_scoped_release gil;
            g.stop_streaming();
            // Also tear the link down, so callbacks and the reader thread are
            // gone by the end of the `with` block rather than lingering until
            // interpreter shutdown. Leaving them alive is what let a late DATA
            // frame call into a finalized interpreter.
            g.transport().stop();
        });

    // ---- FollowerGripper ------------------------------------------------
    // ---- FirmwareVersion -------------------------------------------------
    py::class_<protocol::FirmwareVersion>(m, "FirmwareVersion")
        .def_readonly("major", &protocol::FirmwareVersion::major)
        .def_readonly("minor", &protocol::FirmwareVersion::minor)
        .def_readonly("patch", &protocol::FirmwareVersion::patch)
        .def_readonly("build", &protocol::FirmwareVersion::build)
        .def_property_readonly("tuple",
            [](const protocol::FirmwareVersion& v) {
                return py::make_tuple(v.major, v.minor, v.patch);
            },
            "(major, minor, patch) — comparable, so a feature gate reads as "
            "`gripper.firmware_version.tuple >= (2, 0, 0)`.")
        .def("__repr__", [](const protocol::FirmwareVersion& v) {
            return "FirmwareVersion(" + std::to_string(v.major) + "." +
                   std::to_string(v.minor) + "." + std::to_string(v.patch) +
                   "." + std::to_string(v.build) + ")";
        });

    py::class_<FollowerGripper>(m, "FollowerGripper")
        .def(py::init([](const std::string& mcu, const std::string& wrist,
                         uint32_t baud, unsigned ack_ms, unsigned retries,
                         bool open_cameras,
                         bool undistort_wrist, float fisheye_balance) {
                FollowerGripper::Config cfg;
                cfg.mcu_device           = mcu;
                cfg.wrist_video          = wrist;
                cfg.baudrate             = baud;
                cfg.ack_timeout_ms       = ack_ms;
                cfg.max_retries          = retries;
                cfg.open_cameras         = open_cameras;
                cfg.undistort_wrist      = undistort_wrist;
                cfg.fisheye_balance      = fisheye_balance;
                py::gil_scoped_release gil;
                return std::make_unique<FollowerGripper>(cfg);
             }),
             py::arg("mcu_device"),
             // The wrist camera is off by default; it only matters with open_cameras=True.
             py::arg("wrist_video")         = "",
             py::arg("baudrate")            = 3'000'000u,
             py::arg("ack_timeout_ms")      = 1000u,
             py::arg("max_retries")         = 2u,
             py::arg("open_cameras")        = false,
             // See LeaderGripper above — same semantics and same failure policy.
             py::arg("undistort_wrist")     = false,
             py::arg("fisheye_balance")     = 0.0f)
        .def_property_readonly("firmware_version",
            [](const FollowerGripper& g) -> py::object {
                auto v = g.firmware_version();
                if (!v) return py::none();
                return py::cast(*v);
            },
            "The firmware version reported at open(), or None when the MCU did "
            "not answer. Gate features on it: the wrist fisheye intrinsics need "
            "command set V2.0+, and skipping the check turns 'firmware too old' "
            "into an opaque protocol error.")
        .def_static("open", []() {
            py::gil_scoped_release gil;
            return FollowerGripper::open();
        })
        .def("start_streaming", [](FollowerGripper& self,
                                   unsigned imu_hz, unsigned enc_hz, unsigned motor_hz) {
            py::gil_scoped_release gil;
            self.start_streaming(imu_hz, enc_hz, motor_hz);
        }, py::arg("imu_hz") = 100u,
           py::arg("encoder_hz") = 100u,
           py::arg("motor_hz") = 0u)
        .def("stop_streaming", [](FollowerGripper& self) {
            py::gil_scoped_release gil;
            self.stop_streaming();
        })
        .def_property_readonly("imu",           [](FollowerGripper& g) -> IMU&            { return g.imu(); },           py::return_value_policy::reference_internal)
        .def_property_readonly("encoder",       [](FollowerGripper& g) -> Encoder&        { return g.encoder(); },       py::return_value_policy::reference_internal)
        .def_property_readonly("motor",         [](FollowerGripper& g) -> Motor&          { return g.motor(); },         py::return_value_policy::reference_internal)
        .def_property_readonly("wrist_camera",  [](FollowerGripper& g) -> Camera&         { return g.wrist_camera(); },  py::return_value_policy::reference_internal)
        .def_property_readonly("key",           [](FollowerGripper& g) -> Key&            { return g.key(); },           py::return_value_policy::reference_internal)
        .def_property_readonly("led",           [](FollowerGripper& g) -> Led&            { return g.led(); },           py::return_value_policy::reference_internal)
        .def_property_readonly("sensor_errors", [](FollowerGripper& g) -> SensorErrors&   { return g.sensor_errors(); }, py::return_value_policy::reference_internal)
        .def_property_readonly("calibration",   [](FollowerGripper& g) -> Calibration&    { return g.calibration(); },   py::return_value_policy::reference_internal)
        .def_property_readonly("ota",           [](FollowerGripper& g) -> OtaSession&     { return g.ota(); },           py::return_value_policy::reference_internal)
        .def_property_readonly("transport",     [](FollowerGripper& g) -> bus::Transport& { return g.transport(); },     py::return_value_policy::reference_internal)
        .def_property_readonly("is_streaming",  &FollowerGripper::is_streaming)
        // Follower gripper open/close limit config (Cmd 0x66/0x67).
        .def("get_gripper_config", [](FollowerGripper& g, unsigned timeout_ms) {
            py::gil_scoped_release gil;
            return g.get_gripper_config(std::chrono::milliseconds(timeout_ms));
        }, py::arg("timeout_ms") = 100u)
        .def("set_gripper_config", [](FollowerGripper& g,
                                      const protocol::GripperConfig& cfg) {
            py::gil_scoped_release gil;
            g.set_gripper_config(cfg);
        }, py::arg("config"))
        // Power-on auto-calibration config (Cmd 0x68/0x69).
        .def("get_auto_cal_config", [](FollowerGripper& g, unsigned timeout_ms) {
            py::gil_scoped_release gil;
            return g.get_auto_cal_config(std::chrono::milliseconds(timeout_ms));
        }, py::arg("timeout_ms") = 100u)
        .def("set_auto_cal_config", [](FollowerGripper& g,
                                       const protocol::GripperAutoCalConfig& cfg) {
            py::gil_scoped_release gil;
            g.set_auto_cal_config(cfg);
        }, py::arg("config"))
        // V2.2 — patch only the stall-detection fields. Overloaded on the
        // param type; follower firmware older than 1.1.2 NACKs LengthMismatch.
        .def("set_auto_cal_stall_param", [](FollowerGripper& g,
                                            const protocol::GripperAutoCalStallParam& p) {
            py::gil_scoped_release gil;
            g.set_auto_cal_stall_param(p);
        }, py::arg("param"))
        .def("set_auto_cal_stall_param", [](FollowerGripper& g,
                                            const protocol::GripperAutoCalStallParamEx& p) {
            py::gil_scoped_release gil;
            g.set_auto_cal_stall_param(p);
        }, py::arg("param"))
        // ---- Normalized position (0 = closed, 1 = open) -------------------
        // NOTE: normalized [0,1] — distinct from motor.set_position() (raw rad).
        // set_position() is fire-and-forget (no ACK); poll motor.control_stats()
        // for health. Throws if the gripper isn't calibrated.
        .def("position", [](FollowerGripper& g, unsigned timeout_ms) {
            py::gil_scoped_release gil;
            return g.position(std::chrono::milliseconds(timeout_ms));
        }, py::arg("timeout_ms") = 100u)
        .def("set_position", [](FollowerGripper& g, float position,
                                float kp, float kd, float ff) {
            py::gil_scoped_release gil;
            g.set_position(position, kp, kd, ff);
        }, py::arg("position"), py::arg("kp_nm_per_rad"),
           py::arg("kd_nm_s_per_rad"), py::arg("feedforward_torque_nm") = 0.0f)
        .def("pos_to_rad", [](FollowerGripper& g, float position) {
            py::gil_scoped_release gil;
            return g.pos_to_rad(position);
        }, py::arg("position"))
        .def("rad_to_pos", [](FollowerGripper& g, float raw_rad) {
            py::gil_scoped_release gil;
            return g.rad_to_pos(raw_rad);
        }, py::arg("raw_rad"))
        .def("position_map", [](FollowerGripper& g) {
            py::gil_scoped_release gil;
            return g.position_map();          // copy of the cached converter
        })
        .def("reload_config", [](FollowerGripper& g) {
            py::gil_scoped_release gil;
            g.reload_config();
        })
        .def("__enter__", [](FollowerGripper& g) -> FollowerGripper& { return g; })
        .def("__exit__",  [](FollowerGripper& g, py::object, py::object, py::object) {
            py::gil_scoped_release gil;
            g.stop_streaming();
            // See LeaderGripper::__exit__ — drop the reader thread and its
            // callbacks here rather than at interpreter shutdown.
            g.transport().stop();
        });

    // ---- GripperObservation ---------------------------------------------
    py::class_<GripperObservation>(m, "GripperObservation")
        .def_readonly("valid",    &GripperObservation::valid)
        .def_readonly("position", &GripperObservation::position)   // [0,1]
        .def_readonly("velocity", &GripperObservation::velocity)
        .def_readonly("torque",   &GripperObservation::torque)
        .def_readonly("raw_pos",  &GripperObservation::raw_pos)
        .def_readonly("status",   &GripperObservation::status)
        .def_readonly("seq",      &GripperObservation::seq)
        .def_readonly("age_ms",   &GripperObservation::age_ms)
        .def("__repr__", [](const GripperObservation& o) {
            char buf[128];
            std::snprintf(buf, sizeof(buf),
                "GripperObservation(valid=%d, position=%.4f, vel=%.4f, "
                "torque=%.4f, age=%.1fms, seq=%llu)",
                o.valid, o.position, o.velocity, o.torque, o.age_ms,
                (unsigned long long)o.seq);
            return std::string(buf);
        });

    // ---- ControlLoop: fixed-rate send/recv for embodied control ----------
    py::class_<ControlLoop>(m, "ControlLoop")
        .def(py::init([](FollowerGripper& g, unsigned hz, float kp, float kd,
                         float feedforward_torque, unsigned motor_stream_hz) {
                ControlLoop::Config c;
                c.hz = hz; c.kp = kp; c.kd = kd;
                c.feedforward_torque = feedforward_torque;
                c.motor_stream_hz = motor_stream_hz;
                return std::make_unique<ControlLoop>(g, c);
            }),
            py::arg("gripper"), py::arg("hz") = 200u,
            py::arg("kp") = 8.0f, py::arg("kd") = 1.0f,
            py::arg("feedforward_torque") = 0.0f,
            py::arg("motor_stream_hz") = 100u,
            py::keep_alive<1, 2>())   // keep the gripper alive while the loop lives
        .def("start", [](ControlLoop& l) { py::gil_scoped_release g; l.start(); })
        .def("stop",  [](ControlLoop& l) { py::gil_scoped_release g; l.stop(); })
        .def_property_readonly("running", &ControlLoop::running)
        .def("set_target", [](ControlLoop& l, float p) {
            py::gil_scoped_release g; l.set_target(p);
        }, py::arg("position"))
        .def("set_gains", [](ControlLoop& l, float kp, float kd, float ff) {
            py::gil_scoped_release g; l.set_gains(kp, kd, ff);
        }, py::arg("kp"), py::arg("kd"), py::arg("feedforward_torque") = 0.0f)
        .def_property_readonly("target", &ControlLoop::target)
        .def("observation", [](const ControlLoop& l) {
            py::gil_scoped_release g; return l.observation();
        })
        .def_property_readonly("submit_hz",    &ControlLoop::submit_hz)
        .def_property_readonly("submit_count", &ControlLoop::submit_count)
        .def("__enter__", [](ControlLoop& l) -> ControlLoop& {
            py::gil_scoped_release g; l.start(); return l;
        })
        .def("__exit__", [](ControlLoop& l, py::object, py::object, py::object) {
            py::gil_scoped_release g; l.stop();
        });
}

}  // namespace xense::taccap::python
