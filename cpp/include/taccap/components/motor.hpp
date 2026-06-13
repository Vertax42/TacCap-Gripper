// Copyright (c) 2026 XenseRobotics Co., Ltd. — Apache-2.0
//
// Motor component: typed wrapper around the motor command set (enable/
// disable/clear_fault, four control modes) and the GetMotorStatus telemetry
// path (one-shot read + continuous DATA stream).
//
// Only relevant to the follower gripper — on the leader these commands
// return NACK with ErrorCode::SensorOffline (firmware reports the motor as
// absent), which the component surfaces as ProtocolError just like any
// other NACK.

#pragma once

#include <taccap/bus/transport.hpp>
#include <taccap/protocol/payloads.hpp>

#include <chrono>
#include <cstdint>
#include <functional>

namespace xense::taccap {

struct MotorStatusSample {
    std::chrono::steady_clock::time_point host_time;
    float    actual_pos;        // rad
    float    actual_vel;        // rad/s
    float    actual_torque;     // Nm
    float    motor_temp_c;      // °C
    uint16_t status;            // protocol::MotorStatusBit::* bits
    // V1.7 fields (zero on firmware that still sends the 18-byte status):
    float    actual_current;    // A
    float    target_pos;        // rad   — last applied target
    float    target_vel;        // rad/s
    float    target_torque;     // Nm
    float    target_current;    // A
    uint8_t  control_mode;      // protocol::MotorMode
    uint8_t  current_source;    // 0 = torque-estimated, 1 = low-level iq

    protocol::MotorStatus raw;
};

class Motor {
public:
    using SubId    = bus::Transport::SubscriptionId;
    using Callback = std::function<void(const MotorStatusSample&)>;

    explicit Motor(bus::Transport& transport);

    // ---- Lifecycle / fault management --------------------------------------
    void enable();
    void disable();
    void clear_fault();

    // ---- Control modes -----------------------------------------------------
    // Each call sends one Cmd::Motor*Ctrl frame and waits for ACK. Caller is
    // responsible for the control loop cadence — there is no host-side
    // interpolation or trajectory smoothing here.
    void set_position(float target_pos_rad,
                      float max_vel_radps,
                      float max_torque_nm);
    void set_velocity(float target_vel_radps,
                      float max_torque_nm,
                      float profile_acc_radps2);
    void set_torque(float target_torque_nm,
                    float max_vel_radps);
    void set_impedance(float target_pos_rad,
                       float kp_nm_per_rad,
                       float kd_nm_s_per_rad,
                       float feedforward_torque_nm,
                       float feedforward_vel_radps = 0.0f);  // V1.7; MIT only

    // ---- Telemetry ---------------------------------------------------------
    MotorStatusSample read_status(
        std::chrono::milliseconds timeout = std::chrono::milliseconds{100});

    // Subscribe to streamed MotorStatus DATA frames (StreamSrc::MotorStatus
    // must be enabled in start_streaming for these to arrive).
    SubId on_status(Callback cb);
    void  off(SubId id);

    // ---- V1.7 additions (follower-only) ------------------------------------
    // NOTE: reserved interfaces — the follower gripper hardware is not yet
    // available, so these are implemented against the firmware protocol but
    // have NOT been validated on a real motor. On leader hardware they NACK
    // (SensorOffline) -> ProtocolError, like the other motor commands.
    void              set_zero();                              // Cmd 0x33 (zero)
    uint8_t           get_can_id();                            // Cmd 0x34
    void              set_can_id(uint8_t can_id);              // Cmd 0x35
    void              switch_protocol(protocol::MotorProtocol);// Cmd 0x36 (persists)
    protocol::MotorProtocol get_protocol();                    // Cmd 0x37
    protocol::MotorControlStats control_stats(                 // Cmd 0x51
        std::chrono::milliseconds timeout = std::chrono::milliseconds{100});

    static MotorStatusSample decode(const std::uint8_t* payload, std::size_t len);

private:
    bus::Transport& t_;
};

}  // namespace xense::taccap
