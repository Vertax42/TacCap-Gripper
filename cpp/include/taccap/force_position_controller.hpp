// Copyright (c) 2026 XenseRobotics Co., Ltd. — Apache-2.0
//
// ForcePositionController — contact-aware force/position hybrid grasping.
//
// A plain impedance position command keeps increasing torque while an object
// prevents the jaw from reaching its target. This controller instead mirrors
// the proven tc-gu-01-pc hybrid sequence:
//
//   velocity-damped close -> contact detection -> pure feed-forward torque hold
//
// In the force-hold state kp=kd=0, so the commanded holding torque cannot grow
// with position error. Position holds are dynamically error-clamped as a second
// line of defence. Motion transients and force holding intentionally use two
// limits: the motor's active 0x700B may allow up to 6 Nm during motion, while
// pure force holding is capped in software at 1.8 Nm. start() verifies the V2.2
// value persisted for the next boot against the motion limit. After changing
// that stored value, physically power-cycle before calling start().

#pragma once

#include <taccap/control_loop.hpp>
#include <taccap/follower_gripper.hpp>

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

namespace xense::taccap {

constexpr float FORCE_POSITION_MAX_HOLD_TORQUE_NM   = 1.8f;
constexpr float FORCE_POSITION_MAX_MOTION_TORQUE_NM = 6.0f;

enum class ForcePositionState : uint8_t {
    Idle,
    HoldingPosition,
    Closing,
    HoldingForce,
    Opening,
    Fault,
};

struct ForcePositionConfig {
    float close_position       = 0.0f;   // normalized [0,1], 0 = fully closed
    float close_speed_radps    = 0.5f;   // raw motor speed magnitude
    float grasp_torque_nm      = 0.35f;  // pure torque used after contact
    float hold_torque_limit_nm = FORCE_POSITION_MAX_HOLD_TORQUE_NM;
    float motion_torque_limit_nm = FORCE_POSITION_MAX_MOTION_TORQUE_NM;
    float contact_torque_nm    = 0.0f;   // 0 = clamp(35% of grasp, 0.25, 1.20)
    float position_kp          = 8.0f;   // safe current-position/endpoint hold
    float position_kd          = 1.0f;
    float brake_distance_rad   = 0.10f;  // switch close velocity -> clamped PD
    unsigned contact_samples   = 2;
    unsigned startup_guard_ms  = 250;    // ignore acceleration torque at close start
    unsigned status_timeout_ms = 350;    // stale stream -> zero command + Fault
    unsigned motor_stream_hz   = 100;
};

struct ForcePositionSnapshot {
    bool                running             = false;
    ForcePositionState  state               = ForcePositionState::Idle;
    GripperObservation  observation{};
    float               target_position     = 0.0f;  // normalized [0,1]
    float               hold_position       = 0.0f;  // normalized [0,1]
    float               grasp_torque_nm     = 0.0f;  // active contact/hold target
    float               commanded_torque_nm = 0.0f;  // magnitude predicted/requested
    float               hold_torque_limit_nm = 0.0f;
    float               motion_torque_limit_nm = 0.0f;
    float               device_limit_nm     = 0.0f;  // persisted 0x700B boot value
    unsigned            contact_count       = 0;
    std::string         fault_reason;
};

namespace detail {

// Pure state machine used by ForcePositionController. Kept separate from the
// transport owner so its safety transitions and command bounds are testable
// without a connected gripper.
class ForcePositionPolicy {
public:
    ForcePositionPolicy(GripperPosition map, ForcePositionConfig cfg);

    void reset(const MotorStatusSample& sample,
               std::chrono::steady_clock::time_point now);
    void grasp(std::chrono::steady_clock::time_point now);
    void release(std::chrono::steady_clock::time_point now);
    void set_target(const MotorStatusSample& sample,
                    float target_position,
                    float grasp_torque_nm,
                    std::chrono::steady_clock::time_point now);
    void hold_position(const MotorStatusSample& sample);
    void fail(std::string reason);

    protocol::MotorImpedanceCtrl step(
        const MotorStatusSample& sample,
        std::chrono::steady_clock::time_point now);

    ForcePositionState state() const noexcept { return state_; }
    float target_position() const noexcept { return target_position_; }
    float hold_position() const noexcept { return map_.to_position(hold_raw_); }
    float grasp_torque_nm() const noexcept { return grasp_torque_nm_; }
    float commanded_torque_nm() const noexcept { return commanded_torque_nm_; }
    unsigned contact_count() const noexcept { return contact_count_; }
    const std::string& fault_reason() const noexcept { return fault_reason_; }

private:
    protocol::MotorImpedanceCtrl zero_(const MotorStatusSample& sample);
    protocol::MotorImpedanceCtrl force_hold_();
    protocol::MotorImpedanceCtrl position_hold_(const MotorStatusSample& sample,
                                                 float desired_raw);
    float contact_threshold_() const noexcept;
    float direction_open_() const noexcept;

    GripperPosition map_;
    ForcePositionConfig cfg_;
    ForcePositionState state_ = ForcePositionState::Idle;
    std::chrono::steady_clock::time_point state_started_{};
    float target_position_ = 0.0f;
    float hold_raw_ = 0.0f;
    float grasp_torque_nm_ = 0.0f;
    float commanded_torque_nm_ = 0.0f;
    unsigned contact_count_ = 0;
    std::string fault_reason_;
};

}  // namespace detail

class ForcePositionController {
public:
    explicit ForcePositionController(FollowerGripper& gripper);
    ForcePositionController(FollowerGripper& gripper, ForcePositionConfig cfg);
    ~ForcePositionController();

    ForcePositionController(const ForcePositionController&) = delete;
    ForcePositionController& operator=(const ForcePositionController&) = delete;

    // Owns the follower motor-status/control path until stop(). The caller still
    // owns motor enable/disable. start() seeds a safe current-position hold.
    void start();
    void stop();
    bool running() const noexcept { return running_.load(std::memory_order_acquire); }

    void grasp();          // close toward Config::close_position, then force-hold
    void release();        // open toward 1.0 with bounded velocity damping
    // Immediately move toward a normalized target. A lower target uses the
    // contact-aware grasp path; a higher target uses bounded opening motion.
    void set_target(float position);
    void set_target(float position, float grasp_torque_nm);
    void hold_position();  // cancel motion/force and hold the latest position
    void reset();          // leave Fault after the caller has cleared the motor fault

    ForcePositionState state() const;
    ForcePositionSnapshot snapshot() const;
    const ForcePositionConfig& config() const noexcept { return cfg_; }

private:
    void validate_config_() const;
    void start_motor_stream_();
    void stop_motor_stream_();
    void on_status_(const MotorStatusSample& sample);
    void request_step_();
    void run_();

    FollowerGripper& g_;
    ForcePositionConfig cfg_;
    GripperPosition map_;

    mutable std::mutex mu_;
    std::condition_variable cv_;
    bool step_requested_ = false;
    bool stop_requested_ = false;
    bool have_sample_ = false;
    MotorStatusSample latest_{};
    std::chrono::steady_clock::time_point latest_time_{};
    GripperObservation observation_{};
    std::unique_ptr<detail::ForcePositionPolicy> policy_;
    float device_limit_nm_ = 0.0f;

    std::thread thread_;
    std::atomic<bool> running_{false};
    Motor::SubId sub_ = 0;
    bool sub_active_ = false;
    bool stream_ours_ = false;
};

const char* to_string(ForcePositionState state) noexcept;

}  // namespace xense::taccap
