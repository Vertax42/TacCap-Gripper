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

    // ---- High-rate control submission (no ACK) -----------------------------
    // Fire-and-forget MIT control frames for a host-driven realtime loop (e.g.
    // a follow / teleop loop in the upper layer running up to the firmware's
    // 500Hz slave control rate). These send a CMD_NO_ACK frame and return
    // immediately: there is NO ACK, NO NACK, NO retry, NO timeout, and NO throw
    // on a target the firmware rejects. The firmware's slave control task
    // consumes the *latest* submitted target; the host's only job is to submit
    // at rate. Unlike set_*(), which block on an ACK and throw ProtocolError on
    // NACK, submit() never blocks.
    //
    // Health/error feedback is OUT-OF-BAND — poll these off the realtime thread,
    // never inside the submit loop:
    //   - control_stats(): target_seq vs applied_seq, actual_hz, error_count,
    //     last_error, target_age_ms (host-submit vs firmware-applied cadence).
    //   - on_status(): MotorStatusBit::Fault/Stalled/OverTemp/... + target/actual mirror.
    //   - SensorErrors stream: async SensorErrorId::Motor reports.
    //
    // MIT protocol is assumed (the impedance `vel` feed-forward is MIT-only);
    // there is no per-call protocol check in the hot path. Preconditions are
    // the caller's: enable() and a cleared fault. The only exception is
    // IoError if the transport has been stopped.
    void submit(const protocol::MotorImpedanceCtrl& c);  // primary (MIT hybrid)
    void submit(const protocol::MotorPosCtrl& c);
    void submit(const protocol::MotorVelCtrl& c);
    void submit(const protocol::MotorTorqueCtrl& c);

    // Float-arg convenience forms mirroring set_*; used by the Python bindings.
    void submit_impedance(float target_pos_rad,
                          float kp_nm_per_rad,
                          float kd_nm_s_per_rad,
                          float feedforward_torque_nm,
                          float feedforward_vel_radps = 0.0f);
    void submit_position(float target_pos_rad,
                         float max_vel_radps,
                         float max_torque_nm);
    void submit_velocity(float target_vel_radps,
                         float max_torque_nm,
                         float profile_acc_radps2);
    void submit_torque(float target_torque_nm,
                       float max_vel_radps);

    // ---- Telemetry ---------------------------------------------------------
    MotorStatusSample read_status(
        std::chrono::milliseconds timeout = std::chrono::milliseconds{100});

    // Subscribe to streamed MotorStatus DATA frames (StreamSrc::MotorStatus
    // must be enabled in start_streaming for these to arrive).
    SubId on_status(Callback cb);
    void  off(SubId id);

    // ---- Follower motor admin (zero / CAN id / protocol / stats) -----------
    // Follower-only, validated against firmware hw_v1.1.0. On leader hardware
    // these NACK (SensorOffline) -> ProtocolError, like the other motor
    // commands — that is the correct way to surface a leader/follower mismatch,
    // not a stub.
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
