// Copyright (c) 2026 XenseRobotics Co., Ltd. — Apache-2.0

#include <taccap/control_loop.hpp>

#include <taccap/error.hpp>
#include <taccap/log.hpp>
#include <taccap/protocol/codec.hpp>

#include <algorithm>

namespace xense::taccap {

ControlLoop::ControlLoop(FollowerGripper& gripper)
    : ControlLoop(gripper, Config{}) {}

ControlLoop::ControlLoop(FollowerGripper& gripper, Config cfg)
    : g_(gripper), cfg_(cfg) {
    if (cfg_.hz == 0) cfg_.hz = 1;
    kp_ = cfg_.kp;
    kd_ = cfg_.kd;
    ff_ = cfg_.feedforward_torque;
}

ControlLoop::~ControlLoop() {
    stop();
}

void ControlLoop::start_motor_stream_() {
    if (g_.is_streaming()) {
        // Caller already set up streaming — assume it includes motor status and
        // just ride it. We don't reconfigure (that would drop their sources).
        stream_ours_ = false;
        return;
    }
    // Motor-status-only stream so observations arrive without polling 0x50.
    protocol::StreamConfig sc{};
    sc.source_mask  = protocol::StreamSrc::MotorStatus;
    sc.mode         = static_cast<uint8_t>(protocol::StreamMode::Separate);
    sc.motor_rate   = static_cast<uint16_t>(cfg_.motor_stream_hz);
    sc.output_iface = static_cast<uint8_t>(protocol::StreamInterface::Uart);

    // Drain any leftover stream from a previous host, same as the gripper does.
    try {
        g_.transport().send_cmd(protocol::Cmd::StopStream, {},
                                std::chrono::milliseconds(500));
    } catch (...) { /* expected when fw is already idle */ }

    auto ack = g_.transport().send_cmd(protocol::Cmd::StartStream,
                                       protocol::encode(sc));
    if (ack.is_nack) {
        throw ProtocolError(std::string("ControlLoop: StartStream NACK: ") +
                            protocol::to_string(ack.error_code));
    }
    stream_ours_ = true;
}

void ControlLoop::stop_motor_stream_() {
    if (!stream_ours_) return;
    try {
        g_.transport().send_cmd(protocol::Cmd::StopStream, {},
                                std::chrono::milliseconds(500));
    } catch (...) { /* best-effort */ }
    stream_ours_ = false;
}

void ControlLoop::start() {
    if (running_.load(std::memory_order_acquire)) return;

    // Loads + validates the calibration (throws if not calibrated) and gives us
    // the raw<->position converter. Copied so the threads read it lock-free.
    pos_map_ = g_.position_map();

    // Seed the target with the current position so enabling + starting the loop
    // holds in place rather than jumping to some stale target.
    const float here = g_.position();
    {
        std::lock_guard<std::mutex> lk(mu_);
        target_ = here;
        obs_ = GripperObservation{};   // reset; first stream frame marks it valid
    }

    start_motor_stream_();
    sub_ = g_.motor().on_status(
        [this](const MotorStatusSample& s) { on_status_(s); });
    sub_active_ = true;

    submit_count_.store(0, std::memory_order_relaxed);
    submit_hz_.store(0.0f, std::memory_order_relaxed);
    stop_flag_.store(false, std::memory_order_release);
    running_.store(true, std::memory_order_release);
    thread_ = std::thread([this] { run_(); });

    logger()->info("ControlLoop started: hz={} kp={:.2f} kd={:.2f} stream_hz={}",
                   cfg_.hz, cfg_.kp, cfg_.kd, cfg_.motor_stream_hz);
}

void ControlLoop::stop() {
    stop_flag_.store(true, std::memory_order_release);
    if (thread_.joinable() &&
        thread_.get_id() != std::this_thread::get_id()) {
        thread_.join();
    }
    running_.store(false, std::memory_order_release);

    if (sub_active_) {
        g_.motor().off(sub_);
        sub_active_ = false;
    }
    stop_motor_stream_();
}

void ControlLoop::set_target(float position01) {
    const float p = std::clamp(position01, 0.0f, 1.0f);
    std::lock_guard<std::mutex> lk(mu_);
    target_ = p;
}

void ControlLoop::set_gains(float kp, float kd, float feedforward_torque) {
    std::lock_guard<std::mutex> lk(mu_);
    kp_ = kp;
    kd_ = kd;
    ff_ = feedforward_torque;
}

float ControlLoop::target() const {
    std::lock_guard<std::mutex> lk(mu_);
    return target_;
}

GripperObservation ControlLoop::observation() const {
    std::lock_guard<std::mutex> lk(mu_);
    GripperObservation o = obs_;
    if (o.valid) {
        o.age_ms = std::chrono::duration<double, std::milli>(
                       std::chrono::steady_clock::now() - obs_time_).count();
    }
    return o;
}

void ControlLoop::on_status_(const MotorStatusSample& s) {
    std::lock_guard<std::mutex> lk(mu_);
    obs_.position = pos_map_.to_position(s.actual_pos);
    obs_.velocity = s.actual_vel;
    obs_.torque   = s.actual_torque;
    obs_.raw_pos  = s.actual_pos;
    obs_.status   = s.status;
    obs_.seq     += 1;
    obs_.valid    = true;
    obs_time_     = std::chrono::steady_clock::now();
}

void ControlLoop::run_() {
    using clock = std::chrono::steady_clock;
    const auto period = std::chrono::duration_cast<clock::duration>(
        std::chrono::duration<double>(1.0 / static_cast<double>(cfg_.hz)));
    auto deadline = clock::now();

    uint64_t window_count = 0;
    auto window_start = clock::now();

    while (!stop_flag_.load(std::memory_order_acquire)) {
        deadline += period;
        std::this_thread::sleep_until(deadline);
        if (stop_flag_.load(std::memory_order_acquire)) break;

        float p, kp, kd, ff;
        {
            std::lock_guard<std::mutex> lk(mu_);
            p = target_; kp = kp_; kd = kd_; ff = ff_;
        }

        try {
            g_.motor().submit_impedance(pos_map_.to_rad(p), kp, kd, ff);
        } catch (const std::exception& e) {
            logger()->error("ControlLoop submit failed, stopping: {}", e.what());
            stop_flag_.store(true, std::memory_order_release);
            break;
        }

        submit_count_.fetch_add(1, std::memory_order_relaxed);
        ++window_count;

        const auto now = clock::now();
        const double win_s = std::chrono::duration<double>(now - window_start).count();
        if (win_s >= 0.5) {
            submit_hz_.store(static_cast<float>(window_count / win_s),
                             std::memory_order_relaxed);
            window_count = 0;
            window_start = now;
        }

        // If we fell badly behind (host stalled), don't burst to catch up —
        // resync the deadline to now.
        if (clock::now() > deadline + period) deadline = clock::now();
    }
    running_.store(false, std::memory_order_release);
}

}  // namespace xense::taccap
