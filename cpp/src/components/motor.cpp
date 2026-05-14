// Copyright (c) 2026 XenseRobotics Co., Ltd. — Apache-2.0

#include <taccap/components/motor.hpp>
#include <taccap/error.hpp>
#include <taccap/protocol/codec.hpp>

#include <cstring>

namespace xense::taccap {

namespace {

void send_or_throw(bus::Transport& t, protocol::Cmd cmd,
                   const std::vector<uint8_t>& payload, const char* what) {
    auto ack = t.send_cmd(cmd, payload);
    if (ack.is_nack) {
        throw ProtocolError(std::string("Motor::") + what + " NACK: " +
                            protocol::to_string(ack.error_code));
    }
}

}  // namespace

Motor::Motor(bus::Transport& transport) : t_(transport) {}

MotorStatusSample Motor::decode(const std::uint8_t* payload, std::size_t len) {
    MotorStatusSample s{};
    s.host_time     = std::chrono::steady_clock::now();
    s.raw           = protocol::decode_motor_status(payload, len);
    s.actual_pos    = s.raw.actual_pos;
    s.actual_vel    = s.raw.actual_vel;
    s.actual_torque = s.raw.actual_torque;
    s.motor_temp_c  = s.raw.motor_temp;
    s.status        = s.raw.status;
    return s;
}

void Motor::enable()       { send_or_throw(t_, protocol::Cmd::MotorEnable,     {}, "enable"); }
void Motor::disable()      { send_or_throw(t_, protocol::Cmd::MotorDisable,    {}, "disable"); }
void Motor::clear_fault()  { send_or_throw(t_, protocol::Cmd::MotorClearFault, {}, "clear_fault"); }

void Motor::set_position(float pos, float max_vel, float max_torque) {
    protocol::MotorPosCtrl c{pos, max_vel, max_torque};
    send_or_throw(t_, protocol::Cmd::MotorPosCtrl, protocol::encode(c), "set_position");
}

void Motor::set_velocity(float vel, float max_torque, float profile_acc) {
    protocol::MotorVelCtrl c{vel, max_torque, profile_acc};
    send_or_throw(t_, protocol::Cmd::MotorVelCtrl, protocol::encode(c), "set_velocity");
}

void Motor::set_torque(float torque, float max_vel) {
    protocol::MotorTorqueCtrl c{torque, max_vel, 0.0f};
    send_or_throw(t_, protocol::Cmd::MotorTorqueCtrl, protocol::encode(c), "set_torque");
}

void Motor::set_impedance(float pos, float kp, float kd, float ff_torque) {
    protocol::MotorImpedanceCtrl c{pos, kp, kd, ff_torque};
    send_or_throw(t_, protocol::Cmd::MotorImpedanceCtrl, protocol::encode(c), "set_impedance");
}

MotorStatusSample Motor::read_status(std::chrono::milliseconds timeout) {
    auto ack = t_.send_cmd(protocol::Cmd::GetMotorStatus, {}, timeout);
    if (ack.is_nack) {
        throw ProtocolError(std::string("Motor::read_status NACK: ") +
                            protocol::to_string(ack.error_code));
    }
    return decode(ack.data.data(), ack.data.size());
}

Motor::SubId Motor::on_status(Callback cb) {
    return t_.subscribe(
        protocol::Cmd::GetMotorStatus,
        [cb = std::move(cb)](const bus::Frame& f) {
            try {
                cb(decode(f.payload.data(), f.payload.size()));
            } catch (...) {}
        });
}

void Motor::off(SubId id) { t_.unsubscribe(id); }

}  // namespace xense::taccap
