// Copyright (c) 2026 XenseRobotics Co., Ltd. — Apache-2.0
//
// pybind11 bindings: IMU / Encoder samples, Key, Led, SensorErrors
//
// Split out of the former single-file components.cpp. Pure move — see
// bindings_common.hpp for why the call order in bind_components() matters.

#include "bindings_common.hpp"

namespace xense::taccap::python {

void bind_sensors(py::module_& m) {
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
}

}  // namespace xense::taccap::python
