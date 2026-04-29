// Copyright (c) 2026 XenseRobotics Co., Ltd. — Apache-2.0

#include <taccap/protocol/commands.hpp>

namespace xense::taccap::protocol {

const char* to_string(Address a) noexcept {
    switch (a) {
        case Address::PC:        return "PC";
        case Address::MCU:       return "MCU";
        case Address::MCU_RIGHT: return "MCU_RIGHT";
    }
    return "Address?";
}

const char* to_string(FrameType t) noexcept {
    switch (t) {
        case FrameType::CMD_NEED_ACK: return "CMD_NEED_ACK";
        case FrameType::CMD_NO_ACK:   return "CMD_NO_ACK";
        case FrameType::ACK:          return "ACK";
        case FrameType::DATA:         return "DATA";
    }
    return "FrameType?";
}

const char* to_string(Cmd c) noexcept {
    switch (c) {
        case Cmd::Heartbeat:           return "Heartbeat";
        case Cmd::GetVersion:          return "GetVersion";
        case Cmd::ResetDevice:         return "ResetDevice";
        case Cmd::GetSn:               return "GetSn";
        case Cmd::SetSn:               return "SetSn";
        case Cmd::GetDevType:          return "GetDevType";
        case Cmd::SetDevType:          return "SetDevType";

        case Cmd::GetImu:              return "GetImu";
        case Cmd::GetEncoder:          return "GetEncoder";
        case Cmd::GetEskin1:           return "GetEskin1";
        case Cmd::GetEskin2:           return "GetEskin2";
        case Cmd::GetAllSensors:       return "GetAllSensors";

        case Cmd::StartStream:         return "StartStream";
        case Cmd::StopStream:          return "StopStream";
        case Cmd::SetStreamRate:       return "SetStreamRate";
        case Cmd::SetStreamMode:       return "SetStreamMode";
        case Cmd::SetEncoderZero:      return "SetEncoderZero";

        case Cmd::MotorEnable:         return "MotorEnable";
        case Cmd::MotorDisable:        return "MotorDisable";
        case Cmd::MotorClearFault:     return "MotorClearFault";
        case Cmd::MotorPosCtrl:        return "MotorPosCtrl";
        case Cmd::MotorVelCtrl:        return "MotorVelCtrl";
        case Cmd::MotorTorqueCtrl:     return "MotorTorqueCtrl";
        case Cmd::MotorImpedanceCtrl:  return "MotorImpedanceCtrl";
        case Cmd::GetMotorStatus:      return "GetMotorStatus";

        case Cmd::SetImuConfig:        return "SetImuConfig";
        case Cmd::GetImuConfig:        return "GetImuConfig";
        case Cmd::SetEncoderConfig:    return "SetEncoderConfig";
        case Cmd::GetEncoderConfig:    return "GetEncoderConfig";
        case Cmd::SetEskinConfig:      return "SetEskinConfig";
        case Cmd::GetEskinConfig:      return "GetEskinConfig";
    }
    return "Cmd?";
}

const char* to_string(ErrorCode e) noexcept {
    switch (e) {
        case ErrorCode::Ok:             return "Ok";
        case ErrorCode::InvalidCmd:     return "InvalidCmd";
        case ErrorCode::InvalidParam:   return "InvalidParam";
        case ErrorCode::LengthMismatch: return "LengthMismatch";
        case ErrorCode::CrcError:       return "CrcError";
        case ErrorCode::Timeout:        return "Timeout";
        case ErrorCode::MotorFault:     return "MotorFault";
        case ErrorCode::SensorOffline:  return "SensorOffline";
        case ErrorCode::SysBusy:        return "SysBusy";
        case ErrorCode::SeqMismatch:    return "SeqMismatch";
    }
    return "ErrorCode?";
}

}  // namespace xense::taccap::protocol
