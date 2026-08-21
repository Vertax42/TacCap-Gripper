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
//   ControlLoop loop(g, {.hz = 100, .kp = 8.0f, .kd = 1.0f});
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
// concurrently.
//
// The reason is NOT frame corruption on the host. Transport serialises writers
// internally, and a blocking tty write() is whole-call atomic anyway (measured:
// 40 x 100 kB writes against a deliberately starved drain produced zero short
// writes, so the partial-write loop in SerialBus never even runs). The reason
// is the firmware: host->MCU traffic that overlaps the MCU's own transmission
// makes it drop bytes out of the middle of the frame it is sending. Measured on
// hw_v1.1.0 with a 100Hz motor-status stream -- the damaged frame arrives with
// its payload short by a couple of bytes, fails the LEN check (so crc_errors
// stays 0, only resync_bytes moves) and is discarded whole. The rate that
// survives is not a fixed ceiling: 250Hz submits lost frames on one run and not
// on the next, because what matters is whether a command happens to land inside
// the ~137us the MCU spends transmitting each 41-byte status frame.

#pragma once

#include <taccap/components/motor.hpp>
#include <taccap/follower_gripper.hpp>
#include <taccap/gripper_position.hpp>

#include <atomic>
#include <chrono>
#include <condition_variable>
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
    // When the loop puts its MIT frame on the wire.
    //
    // The MCU drops bytes out of the middle of a status frame it is
    // transmitting if host->MCU traffic overlaps that transmission (see the
    // note above). A 41-byte status frame at 3 Mbps occupies ~137us of each
    // 10ms period, so a free-running submitter collides only occasionally --
    // which is exactly what makes the resulting rate drop look sporadic and
    // unreproducible.
    //
    // StreamLocked removes the collision instead of making it rarer: it
    // submits once per received status frame, so every write lands in the
    // ~9.8ms the MCU is known to be idle. Config::hz is then ignored -- the
    // submit rate becomes motor_stream_hz. This is the same shape as ARX5's
    // send_recv_once: one owner, one exchange per cycle, send phase-locked to
    // receive. Measured: 8 runs of 60s, free-running lost 5..154 status frames
    // per run, stream-locked lost none while putting MORE traffic on the link.
    //
    // It also holds under a full production load. With every camera on both
    // grippers streaming (4 tactile at 640x480 MJPG 120fps + 2 wrist at 30fps,
    // all on the same USB tree), four 60s runs came back at exactly 6000
    // submits : 6000 frames : 0 missing. Not "close to zero" -- every submit
    // matched by a frame. Free-running at 100Hz on the same bench lost 156..308
    // frames per run, with or without the cameras; the camera load barely moved
    // it, because the variable that matters is *when* a write lands, not how
    // busy the bus is.
    //
    // A warning for whoever re-measures this: an early 600s free-running run at
    // 100Hz came back completely clean, and that was wrong to generalise from.
    // Only one gripper assembly was plugged in at the time (4 USB devices
    // instead of 8). With both assemblies attached, the same 100Hz free-running
    // configuration loses 4% of its frames. Bench population changes the
    // answer, so measure on a bus populated like the one you ship on.
    //
    // What it does NOT protect, because the honest scope matters more than the
    // headline:
    //
    //   - ACK responses. The loop knows when the MCU emits *telemetry*; it has
    //     no idea when the MCU is answering somebody's command. Measured with
    //     no stream running and a 100Hz GetMotorStatusExt poll, adding 250Hz of
    //     concurrent no-ACK traffic corrupts responses that the control runs
    //     never lose -- on firmware 1.1.2.0 that was 5-6 per 6000 commands, and
    //     on 1.1.4.0 (logging off) 1-2. The count dropped; the exposure did
    //     not. Control runs stayed at zero on both, test runs non-zero on both.
    //     The saving grace is that commands RETRY -- every one of those was
    //     recovered, costing ~31ms each and surfacing to the caller as latency,
    //     never as failure. Stream frames have no retry, which is why the same
    //     defect reads as a rate drop on telemetry and as nothing at all on
    //     commands.
    //
    //     (Firmware 1.1.4.0 also roughly halved command latency outright --
    //     877us to 489us mean on the quiet control arm -- by no longer blocking
    //     tasks on its debug logging. That is a real gain, but it is a gain in
    //     latency, not in collision immunity.)
    //   - Streams with a high transmit duty cycle. One 41-byte status frame at
    //     3 Mbps fills ~137us of each 10ms period -- 1.4%, so the idle window
    //     we aim at is enormous and the USB delivery jitter in our estimate of
    //     "the MCU just finished sending" does not matter. Turn on IMU and
    //     encoder as well and that margin shrinks; this has not been measured.
    //   - Callers that must write asynchronously by construction. If your
    //     architecture sends when an external event says to, no phase this loop
    //     chooses can help you.
    //
    // So this is avoidance, not a fix. The defect is in the firmware's UART
    // path (tc-gu-01 issue #1) and stays worth fixing there.
    enum class SubmitPhase : uint8_t { FreeRunning, StreamLocked };

    struct Config {
        // Target submission rate (Hz). Only consulted by
        // SubmitPhase::FreeRunning -- StreamLocked takes its rate from the
        // status stream. The default matches what StreamLocked actually
        // produces, so the two phases agree out of the box and switching to
        // FreeRunning does not silently change the rate as well as the phase.
        unsigned hz                  = 100;
        float    kp                  = 8.0f;   // impedance stiffness (Nm/rad)
        float    kd                  = 1.0f;   // impedance damping (Nm·s/rad)
        float    feedforward_torque  = 0.0f;   // Nm
        unsigned motor_stream_hz     = 100;    // motor-status stream rate (Hz)
        SubmitPhase phase            = SubmitPhase::StreamLocked;
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
    void run_free_();
    void run_stream_locked_();
    // Read the target under mu_ and put one MIT frame on the wire. False means
    // the write failed and the loop has already set stop_flag_.
    bool submit_once_();
    void note_rate_(uint64_t& window_count,
                    std::chrono::steady_clock::time_point& window_start);
    void on_status_(const MotorStatusSample& s);
    void start_motor_stream_();
    void stop_motor_stream_();

    FollowerGripper& g_;
    Config           cfg_;
    GripperPosition  pos_map_;

    std::thread        thread_;
    std::atomic<bool>  running_{false};
    std::atomic<bool>  stop_flag_{false};

    // Status-frame doorbell for SubmitPhase::StreamLocked. The submit stays on
    // the loop's own thread rather than running inside on_status_: that
    // callback is the dispatcher thread, shared with every other subscriber,
    // and a write must not be able to stall them.
    std::mutex              tick_mu_;
    std::condition_variable tick_cv_;
    bool                    tick_ = false;

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
