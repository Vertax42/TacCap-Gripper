// Copyright (c) 2026 XenseRobotics Co., Ltd. — Apache-2.0
//
// ControlLoop — a fixed-rate send/receive loop for embodied gripper control.
//
// A background thread submits the latest normalized position target as a MIT
// impedance frame at `hz` (fire-and-forget, no ACK), while the firmware's
// motor-status stream keeps the latest observation fresh. The policy thread
// only touches two thread-safe, non-blocking calls:
//
//     loop.set_target(p)        // p in [0,1] (0 = closed, 1 = open)
//     auto obs = loop.observation()
//
// This keeps the firmware's 500Hz control target fresh without the caller
// fighting timing or the GIL, and reads observations from the push stream
// instead of polling GetMotorStatus (polling > ~100Hz can stall the firmware's
// status refresh — the stream avoids that entirely).
//
// Usage:
//   FollowerGripper g = ...;
//   g.motor().enable();
//   ControlLoop loop(g, {.hz = 200, .kp = 8.0f, .kd = 1.0f});
//   loop.start();                       // seeds target = current position
//   for (;;) {                          // your policy, at its own rate
//       auto obs = loop.observation();  // latest open amount, non-blocking
//       loop.set_target(policy(obs));   // 0..1
//   }
//   loop.stop();
//   g.motor().disable();
//
// While the loop runs it OWNS the control + telemetry path for this gripper:
// don't issue other motor commands or start/stop streaming on the same gripper
// concurrently (two writers on one serial link corrupt frames).

#pragma once

#include <taccap/components/motor.hpp>
#include <taccap/follower_gripper.hpp>
#include <taccap/gripper_position.hpp>

#include <atomic>
#include <chrono>
#include <cstdint>
#include <mutex>
#include <thread>

namespace xense::taccap {

// Latest gripper observation, refreshed from the motor-status stream.
struct GripperObservation {
    bool     valid    = false;   // false until the first status frame arrives
    float    position = 0.0f;    // [0,1] normalized open amount (0=closed,1=open)
    float    velocity = 0.0f;    // rad/s (raw motor frame)
    float    torque   = 0.0f;    // Nm
    float    raw_pos  = 0.0f;    // raw shaft angle (rad)
    uint16_t status   = 0;       // protocol::MotorStatusBit::*
    uint64_t seq      = 0;       // count of stream updates received so far
    double   age_ms   = 0.0;     // age of this sample when observation() returned
};

class ControlLoop {
public:
    struct Config {
        unsigned hz                  = 200;    // target submission rate (Hz)
        float    kp                  = 8.0f;   // impedance stiffness (Nm/rad)
        float    kd                  = 1.0f;   // impedance damping (Nm·s/rad)
        float    feedforward_torque  = 0.0f;   // Nm
        unsigned motor_stream_hz     = 100;    // motor-status stream rate (Hz)
    };

    explicit ControlLoop(FollowerGripper& gripper);   // default Config
    ControlLoop(FollowerGripper& gripper, Config cfg);
    ~ControlLoop();

    ControlLoop(const ControlLoop&)            = delete;
    ControlLoop& operator=(const ControlLoop&) = delete;

    // Start the motor-status stream + submit thread. Seeds the target with the
    // current position so that (with the motor already enabled) starting does
    // not produce a jump. Throws ProtocolError if the gripper isn't calibrated.
    void start();
    void stop();
    bool running() const noexcept { return running_.load(std::memory_order_acquire); }

    // Thread-safe, non-blocking. set_target clamps to [0,1].
    void  set_target(float position01);
    void  set_gains(float kp, float kd, float feedforward_torque);
    float target() const;

    // Thread-safe snapshot of the latest observation (non-blocking).
    GripperObservation observation() const;

    // Loop diagnostics.
    float    submit_hz()    const noexcept { return submit_hz_.load(std::memory_order_relaxed); }
    uint64_t submit_count() const noexcept { return submit_count_.load(std::memory_order_relaxed); }

    const Config& config() const noexcept { return cfg_; }

private:
    void run_();
    void on_status_(const MotorStatusSample& s);
    void start_motor_stream_();
    void stop_motor_stream_();

    FollowerGripper& g_;
    Config           cfg_;
    GripperPosition  pos_map_;

    std::thread        thread_;
    std::atomic<bool>  running_{false};
    std::atomic<bool>  stop_flag_{false};

    mutable std::mutex mu_;
    float              target_ = 0.0f;   // normalized [0,1]
    float              kp_     = 0.0f;
    float              kd_     = 0.0f;
    float              ff_     = 0.0f;
    GripperObservation obs_;
    std::chrono::steady_clock::time_point obs_time_{};

    Motor::SubId       sub_           = 0;
    bool               sub_active_    = false;
    bool               stream_ours_   = false;   // did we StartStream ourselves?

    std::atomic<uint64_t> submit_count_{0};
    std::atomic<float>    submit_hz_{0.0f};
};

}  // namespace xense::taccap
