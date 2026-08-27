// Copyright (c) 2026 XenseRobotics Co., Ltd. — Apache-2.0

#include <taccap/force_position_controller.hpp>

#include <taccap/log.hpp>
#include <taccap/protocol/payloads.hpp>

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace xense::taccap {

namespace {

constexpr float kEpsilon = 1e-5f;

void validate_config(const ForcePositionConfig& cfg) {
    auto finite = [](float v) { return std::isfinite(v); };
    if (!finite(cfg.close_position) || cfg.close_position < 0.0f ||
        cfg.close_position > 1.0f) {
        throw std::invalid_argument("ForcePositionConfig.close_position must be in [0,1]");
    }
    if (!finite(cfg.close_speed_radps) || cfg.close_speed_radps <= 0.0f) {
        throw std::invalid_argument("ForcePositionConfig.close_speed_radps must be > 0");
    }
    if (!finite(cfg.grasp_torque_nm) || cfg.grasp_torque_nm <= 0.0f) {
        throw std::invalid_argument("ForcePositionConfig.grasp_torque_nm must be > 0");
    }
    if (!finite(cfg.hold_torque_limit_nm) || cfg.hold_torque_limit_nm <= 0.0f ||
        cfg.hold_torque_limit_nm > FORCE_POSITION_MAX_HOLD_TORQUE_NM) {
        throw std::invalid_argument(
            "ForcePositionConfig.hold_torque_limit_nm must be in (0, 1.8]");
    }
    if (!finite(cfg.motion_torque_limit_nm) || cfg.motion_torque_limit_nm <= 0.0f ||
        cfg.motion_torque_limit_nm > FORCE_POSITION_MAX_MOTION_TORQUE_NM) {
        throw std::invalid_argument(
            "ForcePositionConfig.motion_torque_limit_nm must be in (0, 6.0]");
    }
    if (cfg.hold_torque_limit_nm > cfg.motion_torque_limit_nm) {
        throw std::invalid_argument(
            "ForcePositionConfig hold torque limit must not exceed motion torque limit");
    }
    if (cfg.grasp_torque_nm > cfg.hold_torque_limit_nm) {
        throw std::invalid_argument(
            "ForcePositionConfig.grasp_torque_nm must not exceed hold_torque_limit_nm");
    }
    if (!finite(cfg.contact_torque_nm) || cfg.contact_torque_nm < 0.0f ||
        cfg.contact_torque_nm > cfg.motion_torque_limit_nm) {
        throw std::invalid_argument(
            "ForcePositionConfig.contact_torque_nm must be 0 or in "
            "(0, motion_torque_limit_nm]");
    }
    if (!finite(cfg.position_kp) || cfg.position_kp <= 0.0f ||
        !finite(cfg.position_kd) || cfg.position_kd < 0.0f) {
        throw std::invalid_argument(
            "ForcePositionConfig position gains require kp > 0 and kd >= 0");
    }
    if (!finite(cfg.brake_distance_rad) || cfg.brake_distance_rad < 0.0f) {
        throw std::invalid_argument("ForcePositionConfig.brake_distance_rad must be >= 0");
    }
    if (cfg.contact_samples == 0 || cfg.status_timeout_ms == 0 ||
        cfg.motor_stream_hz == 0 || cfg.motor_stream_hz > 100) {
        throw std::invalid_argument(
            "ForcePositionConfig requires contact_samples/status_timeout_ms > 0 "
            "and motor_stream_hz in [1,100]");
    }
}

void validate_target(const ForcePositionConfig& cfg,
                     float position,
                     float grasp_torque_nm) {
    if (!std::isfinite(position) || position < 0.0f || position > 1.0f) {
        throw std::invalid_argument("target position must be in [0,1]");
    }
    if (!std::isfinite(grasp_torque_nm) || grasp_torque_nm <= 0.0f ||
        grasp_torque_nm > cfg.hold_torque_limit_nm) {
        throw std::invalid_argument(
            "target grasp torque must be in (0, hold_torque_limit_nm]");
    }
}

bool has_serious_fault(uint16_t status) noexcept {
    constexpr uint16_t mask =
        protocol::MotorStatusBit::Fault |
        protocol::MotorStatusBit::OverTemp |
        protocol::MotorStatusBit::OverCurrent |
        protocol::MotorStatusBit::OverVolt |
        protocol::MotorStatusBit::UnderVolt |
        protocol::MotorStatusBit::EncoderError |
        protocol::MotorStatusBit::DriverFault |
        protocol::MotorStatusBit::PositionInitError |
        protocol::MotorStatusBit::HardwareIdError |
        protocol::MotorStatusBit::EncoderUncalibrated;
    return (status & mask) != 0;
}

}  // namespace

const char* to_string(ForcePositionState state) noexcept {
    switch (state) {
        case ForcePositionState::Idle:            return "idle";
        case ForcePositionState::HoldingPosition: return "holding_position";
        case ForcePositionState::Closing:         return "closing";
        case ForcePositionState::HoldingForce:    return "holding_force";
        case ForcePositionState::Opening:         return "opening";
        case ForcePositionState::Fault:           return "fault";
    }
    return "unknown";
}

namespace detail {

ForcePositionPolicy::ForcePositionPolicy(GripperPosition map,
                                         ForcePositionConfig cfg)
    : map_(std::move(map)), cfg_(cfg) {
    if (!map_.valid()) {
        throw std::invalid_argument("ForcePositionPolicy requires a valid position map");
    }
    validate_config(cfg_);
    grasp_torque_nm_ = cfg_.grasp_torque_nm;
}

void ForcePositionPolicy::reset(const MotorStatusSample& sample,
                                std::chrono::steady_clock::time_point now) {
    state_ = ForcePositionState::HoldingPosition;
    state_started_ = now;
    target_position_ = map_.to_position(sample.actual_pos);
    hold_raw_ = sample.actual_pos;
    grasp_torque_nm_ = cfg_.grasp_torque_nm;
    commanded_torque_nm_ = 0.0f;
    contact_count_ = 0;
    fault_reason_.clear();
}

void ForcePositionPolicy::grasp(std::chrono::steady_clock::time_point now) {
    if (state_ == ForcePositionState::Fault || state_ == ForcePositionState::Idle) return;
    state_ = ForcePositionState::Closing;
    state_started_ = now;
    target_position_ = cfg_.close_position;
    grasp_torque_nm_ = cfg_.grasp_torque_nm;
    commanded_torque_nm_ = 0.0f;
    contact_count_ = 0;
}

void ForcePositionPolicy::release(std::chrono::steady_clock::time_point now) {
    if (state_ == ForcePositionState::Fault || state_ == ForcePositionState::Idle) return;
    state_ = ForcePositionState::Opening;
    state_started_ = now;
    target_position_ = 1.0f;
    grasp_torque_nm_ = cfg_.grasp_torque_nm;
    commanded_torque_nm_ = 0.0f;
    contact_count_ = 0;
}

void ForcePositionPolicy::set_target(
        const MotorStatusSample& sample,
        float target_position,
        float grasp_torque_nm,
        std::chrono::steady_clock::time_point now) {
    validate_target(cfg_, target_position, grasp_torque_nm);
    if (state_ == ForcePositionState::Fault || state_ == ForcePositionState::Idle) return;

    constexpr float kTargetTolerance = 1e-4f;
    const float current_position = map_.to_position(sample.actual_pos);
    target_position_ = target_position;
    grasp_torque_nm_ = grasp_torque_nm;
    state_started_ = now;
    commanded_torque_nm_ = 0.0f;
    contact_count_ = 0;

    if (current_position > target_position + kTargetTolerance) {
        state_ = ForcePositionState::Closing;
    } else if (current_position < target_position - kTargetTolerance) {
        state_ = ForcePositionState::Opening;
    } else {
        state_ = ForcePositionState::HoldingPosition;
        hold_raw_ = map_.to_rad(target_position);
    }
}

void ForcePositionPolicy::hold_position(const MotorStatusSample& sample) {
    if (state_ == ForcePositionState::Fault || state_ == ForcePositionState::Idle) return;
    state_ = ForcePositionState::HoldingPosition;
    target_position_ = map_.to_position(sample.actual_pos);
    hold_raw_ = sample.actual_pos;
    commanded_torque_nm_ = 0.0f;
    contact_count_ = 0;
}

void ForcePositionPolicy::fail(std::string reason) {
    state_ = ForcePositionState::Fault;
    commanded_torque_nm_ = 0.0f;
    contact_count_ = 0;
    fault_reason_ = std::move(reason);
}

float ForcePositionPolicy::direction_open_() const noexcept {
    return map_.reverse() ? -1.0f : 1.0f;
}

float ForcePositionPolicy::contact_threshold_() const noexcept {
    if (cfg_.contact_torque_nm > 0.0f) return cfg_.contact_torque_nm;
    return std::min(cfg_.motion_torque_limit_nm,
                    std::clamp(grasp_torque_nm_ * 0.35f, 0.25f, 1.20f));
}

protocol::MotorImpedanceCtrl ForcePositionPolicy::zero_(
        const MotorStatusSample& sample) {
    commanded_torque_nm_ = 0.0f;
    return {sample.actual_pos, 0.0f, 0.0f, 0.0f, 0.0f};
}

protocol::MotorImpedanceCtrl ForcePositionPolicy::force_hold_() {
    const float hold_torque = std::min(grasp_torque_nm_, cfg_.hold_torque_limit_nm);
    const float signed_torque = -direction_open_() * hold_torque;
    commanded_torque_nm_ = std::abs(signed_torque);
    // kp=kd=0 is the essential safety property: position error cannot add to
    // the requested holding torque after contact.
    return {hold_raw_, 0.0f, 0.0f, signed_torque, 0.0f};
}

protocol::MotorImpedanceCtrl ForcePositionPolicy::position_hold_(
        const MotorStatusSample& sample, float desired_raw) {
    // Bound the instantaneous PD request before it reaches the motor. The
    // motion limit bounds PD transients; motor 0x700B remains the independent
    // hardware backstop. Force holding uses the separate, lower hold limit.
    const float speed = std::abs(sample.actual_vel);
    float kd = cfg_.position_kd;
    if (speed > kEpsilon) {
        kd = std::min(kd, cfg_.motion_torque_limit_nm / speed);
    }
    const float damping = -kd * sample.actual_vel;
    const float position_budget = std::max(0.0f,
        cfg_.motion_torque_limit_nm - std::abs(damping));
    const float error_limit = position_budget / cfg_.position_kp;
    const float error = std::clamp(desired_raw - sample.actual_pos,
                                   -error_limit, error_limit);
    const float target = sample.actual_pos + error;
    const float predicted = cfg_.position_kp * error + damping;
    commanded_torque_nm_ = std::min(cfg_.motion_torque_limit_nm,
                                    std::abs(predicted));
    return {target, cfg_.position_kp, kd, 0.0f, 0.0f};
}

protocol::MotorImpedanceCtrl ForcePositionPolicy::step(
        const MotorStatusSample& sample,
        std::chrono::steady_clock::time_point now) {
    if (!std::isfinite(sample.actual_pos) || !std::isfinite(sample.actual_vel) ||
        !std::isfinite(sample.actual_torque)) {
        fail("non-finite motor status");
    } else if (has_serious_fault(sample.status)) {
        fail("motor status reports a fault");
    } else if (std::abs(sample.actual_torque) >
               cfg_.motion_torque_limit_nm + 1e-4f) {
        fail("measured torque exceeded motion_torque_limit_nm");
    }

    if (state_ == ForcePositionState::Fault || state_ == ForcePositionState::Idle) {
        return zero_(sample);
    }

    const float open_position = map_.to_position(sample.actual_pos);

    if (state_ == ForcePositionState::Closing) {
        const auto guard = std::chrono::milliseconds(cfg_.startup_guard_ms);
        const bool guard_done = now - state_started_ >= guard;
        const bool contact = std::abs(sample.actual_torque) >= contact_threshold_();
        if (guard_done && contact) ++contact_count_;
        else                       contact_count_ = 0;

        if (contact_count_ >= cfg_.contact_samples) {
            state_ = ForcePositionState::HoldingForce;
            hold_raw_ = sample.actual_pos;
            return force_hold_();
        }

        if (open_position <= target_position_ + 1e-4f) {
            state_ = ForcePositionState::HoldingPosition;
            hold_raw_ = map_.to_rad(target_position_);
            return position_hold_(sample, hold_raw_);
        }

        const float close_raw = map_.to_rad(target_position_);
        if (std::abs(sample.actual_pos - close_raw) <= cfg_.brake_distance_rad) {
            return position_hold_(sample, close_raw);
        }

        // Kp=0 prevents target-position error from generating torque. At zero
        // actual velocity, base_kd*close_speed equals grasp_torque_nm. Kd is
        // reduced further when the instantaneous velocity error would exceed
        // motion_torque_limit_nm; motor 0x700B is an independent backstop.
        const float close_velocity = -direction_open_() * cfg_.close_speed_radps;
        const float velocity_error = close_velocity - sample.actual_vel;
        const float base_kd = std::min(5.0f,
            grasp_torque_nm_ / cfg_.close_speed_radps);
        const float kd = std::min(base_kd, cfg_.motion_torque_limit_nm /
            std::max(kEpsilon, std::abs(velocity_error)));
        commanded_torque_nm_ = std::min(cfg_.motion_torque_limit_nm,
                                        std::abs(kd * velocity_error));
        return {sample.actual_pos, 0.0f, kd, 0.0f, close_velocity};
    }

    if (state_ == ForcePositionState::HoldingForce) {
        return force_hold_();
    }

    if (state_ == ForcePositionState::Opening) {
        if (open_position >= target_position_ - 1e-4f) {
            state_ = ForcePositionState::HoldingPosition;
            hold_raw_ = map_.to_rad(target_position_);
            return position_hold_(sample, hold_raw_);
        }
        const float open_velocity = direction_open_() * cfg_.close_speed_radps;
        const float velocity_error = open_velocity - sample.actual_vel;
        const float base_kd = std::min(5.0f,
            grasp_torque_nm_ / cfg_.close_speed_radps);
        const float kd = std::min(base_kd, cfg_.motion_torque_limit_nm /
            std::max(kEpsilon, std::abs(velocity_error)));
        commanded_torque_nm_ = std::min(cfg_.motion_torque_limit_nm,
                                        std::abs(kd * velocity_error));
        return {sample.actual_pos, 0.0f, kd, 0.0f, open_velocity};
    }

    return position_hold_(sample, hold_raw_);
}

}  // namespace detail

ForcePositionController::ForcePositionController(FollowerGripper& gripper)
    : ForcePositionController(gripper, ForcePositionConfig{}) {}

ForcePositionController::ForcePositionController(FollowerGripper& gripper,
                                                 ForcePositionConfig cfg)
    : g_(gripper), cfg_(cfg) {
    validate_config_();
}

ForcePositionController::~ForcePositionController() {
    stop();
}

void ForcePositionController::validate_config_() const {
    validate_config(cfg_);
}

void ForcePositionController::start_motor_stream_() {
    if (g_.is_streaming()) {
        stream_ours_ = false;
        return;
    }
    g_.start_streaming(cfg_.motor_stream_hz);
    stream_ours_ = true;
}

void ForcePositionController::stop_motor_stream_() {
    if (!stream_ours_) return;
    g_.stop_streaming();
    stream_ours_ = false;
}

void ForcePositionController::start() {
    if (running()) return;
    validate_config_();
    map_ = g_.position_map();

    const float device_limit = g_.motor().get_startup_limit_torque();
    if (!std::isfinite(device_limit) || device_limit <= 0.0f ||
        device_limit > cfg_.motion_torque_limit_nm + 1e-4f) {
        throw std::runtime_error(
            "ForcePositionController: stored motor startup torque limit is " +
            std::to_string(device_limit) + " Nm, but motion_torque_limit_nm is " +
            std::to_string(cfg_.motion_torque_limit_nm) +
            " Nm; call set_startup_limit_torque(), power-cycle the gripper, "
            "and verify the value before enabling motion");
    }
    if (device_limit + 1e-4f < cfg_.grasp_torque_nm) {
        logger()->warn(
            "ForcePositionController: device torque limit {:.3f} Nm is below "
            "requested grasp torque {:.3f} Nm; the motor limit wins",
            device_limit, cfg_.grasp_torque_nm);
    } else if (device_limit + 1e-4f < cfg_.motion_torque_limit_nm) {
        logger()->warn(
            "ForcePositionController: device torque limit {:.3f} Nm is below "
            "motion_torque_limit_nm {:.3f} Nm; the device limit wins",
            device_limit, cfg_.motion_torque_limit_nm);
    }

    const MotorStatusSample initial = g_.motor().read_status();
    const auto now = std::chrono::steady_clock::now();
    {
        std::lock_guard<std::mutex> lk(mu_);
        policy_ = std::make_unique<detail::ForcePositionPolicy>(map_, cfg_);
        policy_->reset(initial, now);
        latest_ = initial;
        latest_time_ = now;
        have_sample_ = true;
        observation_ = GripperObservation{};
        observation_.valid = true;
        observation_.position = map_.to_position(initial.actual_pos);
        observation_.velocity = initial.actual_vel;
        observation_.torque = initial.actual_torque;
        observation_.raw_pos = initial.actual_pos;
        observation_.status = initial.status;
        observation_.seq = 1;
        stop_requested_ = false;
        step_requested_ = true;
        device_limit_nm_ = device_limit;
    }

    sub_ = g_.motor().on_status(
        [this](const MotorStatusSample& sample) { on_status_(sample); });
    sub_active_ = true;
    try {
        start_motor_stream_();
    } catch (...) {
        g_.motor().off(sub_);
        sub_active_ = false;
        throw;
    }

    running_.store(true, std::memory_order_release);
    thread_ = std::thread([this] { run_(); });
    cv_.notify_one();
    logger()->info(
        "ForcePositionController started: close={:.3f} speed={:.3f}rad/s "
        "grasp={:.3f}Nm hold_limit={:.3f}Nm motion_limit={:.3f}Nm "
        "device_limit={:.3f}Nm",
        cfg_.close_position, cfg_.close_speed_radps, cfg_.grasp_torque_nm,
        cfg_.hold_torque_limit_nm, cfg_.motion_torque_limit_nm, device_limit_nm_);
}

void ForcePositionController::stop() {
    {
        std::lock_guard<std::mutex> lk(mu_);
        stop_requested_ = true;
    }
    cv_.notify_all();
    if (thread_.joinable() && thread_.get_id() != std::this_thread::get_id()) {
        thread_.join();
    }
    running_.store(false, std::memory_order_release);

    MotorStatusSample last{};
    bool have = false;
    {
        std::lock_guard<std::mutex> lk(mu_);
        last = latest_;
        have = have_sample_;
    }
    if (have) {
        try { g_.motor().submit_impedance(last.actual_pos, 0.0f, 0.0f, 0.0f); }
        catch (...) {}
    }
    if (sub_active_) {
        g_.motor().off(sub_);
        sub_active_ = false;
    }
    stop_motor_stream_();
}

void ForcePositionController::on_status_(const MotorStatusSample& sample) {
    const auto now = std::chrono::steady_clock::now();
    {
        std::lock_guard<std::mutex> lk(mu_);
        latest_ = sample;
        latest_time_ = now;
        have_sample_ = true;
        observation_.valid = true;
        observation_.position = map_.to_position(sample.actual_pos);
        observation_.velocity = sample.actual_vel;
        observation_.torque = sample.actual_torque;
        observation_.raw_pos = sample.actual_pos;
        observation_.status = sample.status;
        ++observation_.seq;
        step_requested_ = true;
    }
    cv_.notify_one();
}

void ForcePositionController::request_step_() {
    step_requested_ = true;
    cv_.notify_one();
}

void ForcePositionController::grasp() {
    std::lock_guard<std::mutex> lk(mu_);
    if (!running() || !policy_ || !have_sample_) {
        throw std::logic_error("ForcePositionController::grasp called before start");
    }
    policy_->set_target(latest_, cfg_.close_position, cfg_.grasp_torque_nm,
                        std::chrono::steady_clock::now());
    request_step_();
}

void ForcePositionController::set_target(float position) {
    set_target(position, cfg_.grasp_torque_nm);
}

void ForcePositionController::set_target(float position, float grasp_torque_nm) {
    std::lock_guard<std::mutex> lk(mu_);
    if (!running() || !policy_ || !have_sample_) {
        throw std::logic_error("ForcePositionController::set_target called before start");
    }
    policy_->set_target(latest_, position, grasp_torque_nm,
                        std::chrono::steady_clock::now());
    request_step_();
}

void ForcePositionController::release() {
    std::lock_guard<std::mutex> lk(mu_);
    if (!running() || !policy_ || !have_sample_) {
        throw std::logic_error("ForcePositionController::release called before start");
    }
    policy_->release(std::chrono::steady_clock::now());
    request_step_();
}

void ForcePositionController::hold_position() {
    std::lock_guard<std::mutex> lk(mu_);
    if (!running() || !policy_ || !have_sample_) {
        throw std::logic_error("ForcePositionController::hold_position called before start");
    }
    policy_->hold_position(latest_);
    request_step_();
}

void ForcePositionController::reset() {
    std::lock_guard<std::mutex> lk(mu_);
    if (!running() || !policy_ || !have_sample_) {
        throw std::logic_error("ForcePositionController::reset called before start");
    }
    policy_->reset(latest_, std::chrono::steady_clock::now());
    // The cached sample may still carry the fault bit that was true immediately
    // before Motor::clear_fault() ACKed. Wait for the next streamed status rather
    // than re-faulting the freshly reset policy on stale evidence.
    step_requested_ = false;
}

ForcePositionState ForcePositionController::state() const {
    std::lock_guard<std::mutex> lk(mu_);
    return policy_ ? policy_->state() : ForcePositionState::Idle;
}

ForcePositionSnapshot ForcePositionController::snapshot() const {
    const auto now = std::chrono::steady_clock::now();
    std::lock_guard<std::mutex> lk(mu_);
    ForcePositionSnapshot out;
    out.running = running();
    out.observation = observation_;
    if (out.observation.valid) {
        out.observation.age_ms = std::chrono::duration<double, std::milli>(
            now - latest_time_).count();
    }
    out.device_limit_nm = device_limit_nm_;
    if (policy_) {
        out.state = policy_->state();
        out.target_position = policy_->target_position();
        out.hold_position = policy_->hold_position();
        out.grasp_torque_nm = policy_->grasp_torque_nm();
        out.commanded_torque_nm = policy_->commanded_torque_nm();
        out.hold_torque_limit_nm = cfg_.hold_torque_limit_nm;
        out.motion_torque_limit_nm = cfg_.motion_torque_limit_nm;
        out.contact_count = policy_->contact_count();
        out.fault_reason = policy_->fault_reason();
    }
    return out;
}

void ForcePositionController::run_() {
    const auto timeout = std::chrono::milliseconds(cfg_.status_timeout_ms);
    for (;;) {
        protocol::MotorImpedanceCtrl command{};
        bool send = false;
        {
            std::unique_lock<std::mutex> lk(mu_);
            cv_.wait_for(lk, std::chrono::milliseconds(100), [this] {
                return stop_requested_ || step_requested_;
            });
            if (stop_requested_) break;
            const auto now = std::chrono::steady_clock::now();
            if (!policy_ || !have_sample_) continue;

            if (!step_requested_) {
                if (now - latest_time_ < timeout ||
                    policy_->state() == ForcePositionState::Fault) {
                    continue;
                }
                policy_->fail("motor status stream stale");
            }
            step_requested_ = false;
            command = policy_->step(latest_, now);
            send = true;
        }

        if (!send) continue;
        try {
            g_.motor().submit(command);
        } catch (const std::exception& e) {
            std::lock_guard<std::mutex> lk(mu_);
            if (policy_) policy_->fail(std::string("submit failed: ") + e.what());
        }
    }
}

}  // namespace xense::taccap
