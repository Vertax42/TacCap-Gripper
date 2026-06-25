// Copyright (c) 2026 XenseRobotics Co., Ltd. — Apache-2.0
//
// pybind11 bindings for the component classes (IMU, Encoder, Camera,
// LeaderGripper, FollowerGripper) and their POD samples.
//
// Sample structs use numpy arrays for vector fields so users get the
// expected `s.accel_mps2[0]` / `np.linalg.norm(s.accel_mps2)` ergonomics
// without an extra Python conversion step.

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/numpy.h>
#include <pybind11/functional.h>

#include <taccap/components/imu.hpp>
#include <taccap/components/encoder.hpp>
#include <taccap/components/camera.hpp>
#include <taccap/components/key.hpp>
#include <taccap/components/sensor_errors.hpp>
#include <taccap/components/motor.hpp>
#include <taccap/control_loop.hpp>
#include <taccap/follower_gripper.hpp>
#include <taccap/leader_gripper.hpp>
#include <taccap/discovery.hpp>
#include <taccap/ota.hpp>

#include <chrono>
#include <memory>

namespace py = pybind11;

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
py::array make_vec3(const std::array<float, 3>& v) {
    py::array_t<float> arr(3);
    auto* p = arr.mutable_data();
    p[0] = v[0]; p[1] = v[1]; p[2] = v[2];
    return arr;
}

// Wrap a py::function in a shared_ptr whose deleter acquires the GIL.
// Background: the per-component callback wrappers below capture the
// shared_ptr into the C++ callback lambda. The last shared_ptr ref dies
// on whatever thread runs the lambda's destructor — usually the worker
// capture thread when stop() joins it. py::function's destructor decrefs
// a Python object, which segfaults without the GIL. Centralising the
// custom deleter here keeps every component honest.
std::shared_ptr<py::function> make_gil_safe_callback(py::function pycb) {
    return std::shared_ptr<py::function>(
        new py::function(std::move(pycb)),
        [](py::function* p) {
            py::gil_scoped_acquire gil;
            delete p;
        });
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
        .def_property_readonly("raw_position_rad",
                               [](const EncoderSample& s) { return s.raw.position_rad; })
        .def_property_readonly("raw_velocity_rad_s",
                               [](const EncoderSample& s) { return s.raw.velocity_rad_s; })
        .def_readonly("status",           &EncoderSample::status)
        .def_readonly("seq",              &EncoderSample::seq)
        .def("__repr__", [](const EncoderSample& s) {
            char buf[160];
            std::snprintf(buf, sizeof(buf),
                "EncoderSample(seq=%u, pos=%.4frad (raw=%.4f), vel=%.4frad/s)",
                s.seq, s.position_rad, s.raw.position_rad, s.velocity_rad_s);
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
                py::gil_scoped_acquire acq;
                try { (*cb)(s); }
                catch (py::error_already_set& e) {
                    e.discard_as_unraisable("xense.taccap.IMU callback");
                } catch (...) {}
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
                py::gil_scoped_acquire acq;
                try { (*cb)(s); }
                catch (py::error_already_set& e) {
                    e.discard_as_unraisable("xense.taccap.Key callback");
                } catch (...) {}
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
                py::gil_scoped_acquire acq;
                try { (*cb)(s); }
                catch (py::error_already_set& e) {
                    e.discard_as_unraisable("xense.taccap.SensorErrors callback");
                } catch (...) {}
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
                    py::gil_scoped_acquire acq;
                    try { (*pycb)(wr, tot); }
                    catch (py::error_already_set& e) {
                        e.discard_as_unraisable("OtaSession progress");
                    } catch (...) {}
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
                    py::gil_scoped_acquire acq;
                    try { (*pycb)(wr, tot); }
                    catch (py::error_already_set& e) {
                        e.discard_as_unraisable("OtaSession progress");
                    } catch (...) {}
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

    // ---- GripperPosition: pure raw-rad <-> normalized [0,1] converter ------
    py::class_<GripperPosition>(m, "GripperPosition")
        .def(py::init<>())
        .def(py::init<const protocol::GripperConfig&>(), py::arg("config"))
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
        // V1.7 (zero when firmware sends the legacy 18-byte status):
        .def_readonly("actual_current", &MotorStatusSample::actual_current)
        .def_readonly("target_pos",     &MotorStatusSample::target_pos)
        .def_readonly("target_vel",     &MotorStatusSample::target_vel)
        .def_readonly("target_torque",  &MotorStatusSample::target_torque)
        .def_readonly("target_current", &MotorStatusSample::target_current)
        .def_readonly("control_mode",   &MotorStatusSample::control_mode)
        .def_readonly("current_source", &MotorStatusSample::current_source)
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
                py::gil_scoped_acquire acq;
                try { (*cb)(s); }
                catch (py::error_already_set& e) {
                    e.discard_as_unraisable("xense.taccap.Encoder callback");
                } catch (...) {}
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
            "(e.g. fully closed) before calling. Raises on NACK / timeout.");

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
            auto cb = std::make_shared<py::function>(std::move(pycb));
            return self.on_status([cb](const MotorStatusSample& s) {
                py::gil_scoped_acquire acq;
                try { (*cb)(s); }
                catch (py::error_already_set& e) {
                    e.discard_as_unraisable("xense.taccap.Motor callback");
                } catch (...) {}
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
        .def("control_stats", [](Motor& self, unsigned timeout_ms) {
            py::gil_scoped_release g;
            return self.control_stats(std::chrono::milliseconds(timeout_ms));
        }, py::arg("timeout_ms") = 100);

    // ---- Camera ---------------------------------------------------------
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
                py::gil_scoped_acquire acq;
                try { (*cb)(f); }
                catch (py::error_already_set& e) {
                    e.discard_as_unraisable("xense.taccap.Camera callback");
                } catch (...) {}
            });
        }, py::arg("callback"))
        .def("stop", [](Camera& self) {
            py::gil_scoped_release gil;
            self.stop();
        })
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
                         bool open_cameras) {
                LeaderGripper::Config cfg;
                cfg.mcu_device           = mcu;
                cfg.wrist_video          = wrist;
                cfg.baudrate             = baud;
                cfg.ack_timeout_ms       = ack_ms;
                cfg.max_retries          = retries;
                cfg.open_cameras         = open_cameras;
                py::gil_scoped_release gil;
                return std::make_unique<LeaderGripper>(cfg);
             }),
             py::arg("mcu_device"),
             // The wrist camera is off by default; it only matters with open_cameras=True.
             py::arg("wrist_video")         = "",
             py::arg("baudrate")            = 3'000'000u,
             py::arg("ack_timeout_ms")      = 200u,
             py::arg("max_retries")         = 1u,
             py::arg("open_cameras")        = false)
        .def_static("open", []() {
            py::gil_scoped_release gil;
            return LeaderGripper::open();   // returns unique_ptr<LeaderGripper>
        })
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
        .def_property_readonly("sensor_errors", [](LeaderGripper& g) -> SensorErrors&   { return g.sensor_errors(); }, py::return_value_policy::reference_internal)
        .def_property_readonly("ota",           [](LeaderGripper& g) -> OtaSession&     { return g.ota(); },           py::return_value_policy::reference_internal)
        .def_property_readonly("transport",     [](LeaderGripper& g) -> bus::Transport& { return g.transport(); },     py::return_value_policy::reference_internal)
        .def_property_readonly("is_streaming",  &LeaderGripper::is_streaming)
        .def("__enter__", [](LeaderGripper& g) -> LeaderGripper& { return g; })
        .def("__exit__",  [](LeaderGripper& g, py::object, py::object, py::object) {
            py::gil_scoped_release gil;
            g.stop_streaming();
        });

    // ---- FollowerGripper ------------------------------------------------
    py::class_<FollowerGripper>(m, "FollowerGripper")
        .def(py::init([](const std::string& mcu, const std::string& wrist,
                         uint32_t baud, unsigned ack_ms, unsigned retries,
                         bool open_cameras) {
                FollowerGripper::Config cfg;
                cfg.mcu_device           = mcu;
                cfg.wrist_video          = wrist;
                cfg.baudrate             = baud;
                cfg.ack_timeout_ms       = ack_ms;
                cfg.max_retries          = retries;
                cfg.open_cameras         = open_cameras;
                py::gil_scoped_release gil;
                return std::make_unique<FollowerGripper>(cfg);
             }),
             py::arg("mcu_device"),
             // The wrist camera is off by default; it only matters with open_cameras=True.
             py::arg("wrist_video")         = "",
             py::arg("baudrate")            = 3'000'000u,
             py::arg("ack_timeout_ms")      = 1000u,
             py::arg("max_retries")         = 2u,
             py::arg("open_cameras")        = false)
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
        .def_property_readonly("sensor_errors", [](FollowerGripper& g) -> SensorErrors&   { return g.sensor_errors(); }, py::return_value_policy::reference_internal)
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
