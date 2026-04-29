// Copyright (c) 2026 XenseRobotics Co., Ltd. — Apache-2.0
//
// pybind11 entry point for the xense.taccap Python package.
//
// Layered exports:
//   - top-level: __version__, libxense_version, hello()
//   - protocol enums:    Address, FrameType, Cmd, ErrorCode
//   - bus framing:       Frame, FrameParser, pack_frame(), crc16_modbus(),
//                        stuff_data(), unstuff_data()
//   - serial transport:  SerialBus
//
// Future steps add component classes (Camera, IMU, Encoder, Motor,
// LeaderGripper, FollowerGripper) here.

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <taccap/version.hpp>
#include <taccap/vision.hpp>
#include <taccap/error.hpp>
#include <taccap/protocol/commands.hpp>
#include <taccap/protocol/payloads.hpp>
#include <taccap/protocol/codec.hpp>
#include <taccap/bus/frame.hpp>
#include <taccap/bus/serial_bus.hpp>

#include <xense/core/version.hpp>  // defines XENSESDK_VERSION_STRING

#include <cstring>
#include <string>

namespace py = pybind11;

namespace {

// Helper: copy a py::bytes into a fresh std::vector<uint8_t>.
std::vector<uint8_t> bytes_to_vec(py::bytes b) {
    char* buf = nullptr;
    py::ssize_t len = 0;
    PYBIND11_BYTES_AS_STRING_AND_SIZE(b.ptr(), &buf, &len);
    return std::vector<uint8_t>(reinterpret_cast<uint8_t*>(buf),
                                reinterpret_cast<uint8_t*>(buf) + len);
}

py::bytes vec_to_bytes(const std::vector<uint8_t>& v) {
    return py::bytes(reinterpret_cast<const char*>(v.data()), v.size());
}

}  // namespace

PYBIND11_MODULE(_taccap_native, m) {
    m.doc() = "TacCap-Gripper native module (lite scaffold + TC-GU-01 protocol)";

    m.attr("__version__")      = TACCAP_VERSION_STRING;
    m.attr("libxense_version") = XENSESDK_VERSION_STRING;

    m.def("hello", []() {
        return std::string("taccap-gripper lite scaffold OK; "
                           "libxense lite version: ") + XENSESDK_VERSION_STRING;
    });

    using namespace xense::taccap;
    namespace tp = protocol;
    namespace tb = bus;

    // ---- Exception translation ------------------------------------------
    py::register_exception<ProtocolError>(m, "ProtocolError");
    py::register_exception<CrcError>(m, "CrcError");
    py::register_exception<IoError>(m, "IoError");
    py::register_exception<TimeoutError>(m, "TimeoutError");

    // ---- Enums -----------------------------------------------------------
    py::enum_<tp::Address>(m, "Address")
        .value("PC",        tp::Address::PC)
        .value("MCU",       tp::Address::MCU)
        .value("MCU_RIGHT", tp::Address::MCU_RIGHT);

    py::enum_<tp::FrameType>(m, "FrameType")
        .value("CMD_NEED_ACK", tp::FrameType::CMD_NEED_ACK)
        .value("CMD_NO_ACK",   tp::FrameType::CMD_NO_ACK)
        .value("ACK",          tp::FrameType::ACK)
        .value("DATA",         tp::FrameType::DATA);

    py::enum_<tp::Cmd>(m, "Cmd")
        .value("Heartbeat",          tp::Cmd::Heartbeat)
        .value("GetVersion",         tp::Cmd::GetVersion)
        .value("ResetDevice",        tp::Cmd::ResetDevice)
        .value("GetSn",              tp::Cmd::GetSn)
        .value("SetSn",              tp::Cmd::SetSn)
        .value("GetDevType",         tp::Cmd::GetDevType)
        .value("SetDevType",         tp::Cmd::SetDevType)
        .value("GetImu",             tp::Cmd::GetImu)
        .value("GetEncoder",         tp::Cmd::GetEncoder)
        .value("GetEskin1",          tp::Cmd::GetEskin1)
        .value("GetEskin2",          tp::Cmd::GetEskin2)
        .value("GetAllSensors",      tp::Cmd::GetAllSensors)
        .value("StartStream",        tp::Cmd::StartStream)
        .value("StopStream",         tp::Cmd::StopStream)
        .value("SetStreamRate",      tp::Cmd::SetStreamRate)
        .value("SetStreamMode",      tp::Cmd::SetStreamMode)
        .value("SetEncoderZero",     tp::Cmd::SetEncoderZero)
        .value("MotorEnable",        tp::Cmd::MotorEnable)
        .value("MotorDisable",       tp::Cmd::MotorDisable)
        .value("MotorClearFault",    tp::Cmd::MotorClearFault)
        .value("MotorPosCtrl",       tp::Cmd::MotorPosCtrl)
        .value("MotorVelCtrl",       tp::Cmd::MotorVelCtrl)
        .value("MotorTorqueCtrl",    tp::Cmd::MotorTorqueCtrl)
        .value("MotorImpedanceCtrl", tp::Cmd::MotorImpedanceCtrl)
        .value("GetMotorStatus",     tp::Cmd::GetMotorStatus)
        .value("SetImuConfig",       tp::Cmd::SetImuConfig)
        .value("GetImuConfig",       tp::Cmd::GetImuConfig)
        .value("SetEncoderConfig",   tp::Cmd::SetEncoderConfig)
        .value("GetEncoderConfig",   tp::Cmd::GetEncoderConfig)
        .value("SetEskinConfig",     tp::Cmd::SetEskinConfig)
        .value("GetEskinConfig",     tp::Cmd::GetEskinConfig);

    py::enum_<tp::ErrorCode>(m, "ErrorCode")
        .value("Ok",             tp::ErrorCode::Ok)
        .value("InvalidCmd",     tp::ErrorCode::InvalidCmd)
        .value("InvalidParam",   tp::ErrorCode::InvalidParam)
        .value("LengthMismatch", tp::ErrorCode::LengthMismatch)
        .value("CrcError",       tp::ErrorCode::CrcError)
        .value("Timeout",        tp::ErrorCode::Timeout)
        .value("MotorFault",     tp::ErrorCode::MotorFault)
        .value("SensorOffline",  tp::ErrorCode::SensorOffline)
        .value("SysBusy",        tp::ErrorCode::SysBusy)
        .value("SeqMismatch",    tp::ErrorCode::SeqMismatch);

    // Module-level frame constants
    m.attr("FRAME_HEAD")      = py::int_(tb::FRAME_HEAD);
    m.attr("FRAME_TAIL")      = py::int_(tb::FRAME_TAIL);
    m.attr("FRAME_ESCAPE")    = py::int_(tb::FRAME_ESCAPE);
    m.attr("MIN_FRAME_LEN")   = py::int_(tb::MIN_FRAME_LEN);
    m.attr("MAX_PAYLOAD_LEN") = py::int_(tb::MAX_PAYLOAD_LEN);
    m.attr("MAX_FRAME_LEN")   = py::int_(tb::MAX_FRAME_LEN);

    // ---- Frame -----------------------------------------------------------
    py::class_<tb::Frame>(m, "Frame")
        .def(py::init<>())
        .def_readwrite("addr", &tb::Frame::addr)
        .def_readwrite("seq",  &tb::Frame::seq)
        .def_readwrite("type", &tb::Frame::type)
        .def_readwrite("cmd",  &tb::Frame::cmd)
        .def_property(
            "payload",
            [](const tb::Frame& f) { return vec_to_bytes(f.payload); },
            [](tb::Frame& f, py::bytes b) { f.payload = bytes_to_vec(b); })
        .def("__repr__", [](const tb::Frame& f) {
            return std::string("Frame(addr=") + tp::to_string(f.addr) +
                   ", seq=" + std::to_string(f.seq) +
                   ", type=" + tp::to_string(f.type) +
                   ", cmd="  + tp::to_string(f.cmd) +
                   ", payload=" + std::to_string(f.payload.size()) + "B)";
        });

    // ---- Pure functions --------------------------------------------------
    m.def("crc16_modbus", [](py::bytes b) {
        char* buf = nullptr;
        py::ssize_t len = 0;
        PYBIND11_BYTES_AS_STRING_AND_SIZE(b.ptr(), &buf, &len);
        return tb::crc16_modbus(reinterpret_cast<const uint8_t*>(buf), len);
    }, py::arg("data"));

    m.def("pack_frame",
          [](tp::Address addr, uint8_t seq, tp::FrameType type, tp::Cmd cmd,
             py::bytes payload) {
              auto v = bytes_to_vec(payload);
              auto wire = tb::pack_frame(addr, seq, type, cmd, v);
              return vec_to_bytes(wire);
          },
          py::arg("addr"), py::arg("seq"), py::arg("type"), py::arg("cmd"),
          py::arg("payload") = py::bytes(""));

    m.def("stuff_data", [](py::bytes b) {
        auto v = bytes_to_vec(b);
        return vec_to_bytes(tb::stuff_data(v.data(), v.size()));
    }, py::arg("data"));

    m.def("unstuff_data", [](py::bytes b) {
        auto v = bytes_to_vec(b);
        return vec_to_bytes(tb::unstuff_data(v.data(), v.size()));
    }, py::arg("data"));

    // ---- FrameParser -----------------------------------------------------
    py::class_<tb::FrameParser>(m, "FrameParser")
        .def(py::init<std::size_t>(), py::arg("max_buffered") = 64u * 1024u)
        .def("feed", [](tb::FrameParser& p, py::bytes b) {
            auto v = bytes_to_vec(b);
            p.feed(v.data(), v.size());
        }, py::arg("data"))
        .def("pop", [](tb::FrameParser& p) -> py::object {
            tb::Frame f;
            if (!p.try_pop(f)) return py::none();
            return py::cast(std::move(f));
        })
        .def("pending",        &tb::FrameParser::pending)
        .def("buffered_bytes", &tb::FrameParser::buffered_bytes)
        .def("reset",          &tb::FrameParser::reset);

    // ---- SerialBus -------------------------------------------------------
    py::class_<tb::SerialBus>(m, "SerialBus")
        .def(py::init([](const std::string& dev, uint32_t br,
                         unsigned rt_ms, unsigned wt_ms) {
                return std::make_unique<tb::SerialBus>(
                    tb::SerialBus::Config{dev, br, rt_ms, wt_ms});
             }),
             py::arg("device"),
             py::arg("baudrate")         = 3'000'000u,
             py::arg("read_timeout_ms")  = 1u,
             py::arg("write_timeout_ms") = 1000u)
        .def("read", [](tb::SerialBus& s, std::size_t max) {
            auto v = s.read(max);
            return vec_to_bytes(v);
        }, py::arg("max"))
        .def("write", [](tb::SerialBus& s, py::bytes b) {
            auto v = bytes_to_vec(b);
            s.write(v);
        }, py::arg("data"))
        .def("flush_input",  &tb::SerialBus::flush_input)
        .def("flush_output", &tb::SerialBus::flush_output)
        .def_property_readonly("is_open", &tb::SerialBus::is_open)
        .def_property_readonly("device", [](const tb::SerialBus& s) {
            return s.config().device;
        })
        .def_property_readonly("baudrate", [](const tb::SerialBus& s) {
            return s.config().baudrate;
        })
        .def_static("list_ports", &tb::SerialBus::list_ports);
}
