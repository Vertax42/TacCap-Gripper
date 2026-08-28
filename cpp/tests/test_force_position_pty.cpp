// Copyright (c) 2026 XenseRobotics Co., Ltd. — Apache-2.0
//
// PTY-driven tests for ForcePositionController itself, as opposed to the pure
// ForcePositionPolicy state machine pinned in
// test_force_position_controller.cpp. Everything interesting about the
// controller lives in the parts the policy cannot see: the status-stream
// subscription, the submit thread, and the safety interlocks between them.
// Those are exactly the paths a hardware bring-up would exercise last and
// trust most, so they are driven here against a fake follower firmware.

#include "pty_helper.hpp"

#include <taccap/force_position_controller.hpp>
#include <taccap/follower_gripper.hpp>
#include <taccap/error.hpp>
#include <taccap/protocol/codec.hpp>

#include <atomic>
#include <cstring>
#include <mutex>
#include <optional>
#include <thread>
#include <vector>

namespace tx = xense::taccap;
namespace tp = xense::taccap::protocol;

using taccap_test::Pty;

namespace {

std::vector<uint8_t> pod_bytes(const void* p, std::size_t n) {
    std::vector<uint8_t> out(n);
    std::memcpy(out.data(), p, n);
    return out;
}

// Minimal follower firmware: answers the handful of commands
// FollowerGripper's constructor and ForcePositionController::start() issue,
// pushes motor-status DATA frames while "streaming", and records the MIT
// impedance frames the controller submits.
class FakeFollower {
public:
    FakeFollower(Pty& pty, uint8_t major = 1, uint8_t minor = 1, uint8_t patch = 6)
        : pty_(pty), fw_major_(major), fw_minor_(minor), fw_patch_(patch) {
        status_.actual_pos = 0.60f;
        status_.control_mode = 0;
        thread_ = std::thread([this] { run_(); });
    }
    ~FakeFollower() {
        stop_.store(true);
        if (thread_.joinable()) thread_.join();
    }

    // Stop answering the status stream without stopping the responder: this is
    // what a wedged firmware looks like to the host.
    void freeze_stream(bool frozen) { frozen_.store(frozen); }

    void set_status(float pos, float vel, float torque) {
        std::lock_guard<std::mutex> lk(mu_);
        status_.actual_pos = pos;
        status_.actual_vel = vel;
        status_.actual_torque = torque;
    }

    std::optional<tp::MotorImpedanceCtrl> last_submit() const {
        std::lock_guard<std::mutex> lk(mu_);
        return last_submit_;
    }
    unsigned submit_count() const { return submits_.load(); }

private:
    void run_() {
        auto next_status = std::chrono::steady_clock::now();
        while (!stop_.load()) {
            if (auto f = pty_.expect_frame(5)) handle_(*f);

            const auto now = std::chrono::steady_clock::now();
            if (streaming_.load() && !frozen_.load() && now >= next_status) {
                next_status = now + std::chrono::milliseconds(10);
                std::lock_guard<std::mutex> lk(mu_);
                pty_.send_data(0, tp::Cmd::GetMotorStatus,
                               pod_bytes(&status_, sizeof(status_)));
            }
        }
    }

    void handle_(const xense::taccap::bus::Frame& f) {
        switch (f.cmd) {
            case tp::Cmd::GetVersion: {
                const tp::FirmwareVersion v{fw_major_, fw_minor_, fw_patch_, 0};
                pty_.send_response(f.seq, f.cmd, pod_bytes(&v, sizeof(v)));
                return;
            }
            case tp::Cmd::GetSn: {
                const std::string sn = "TCGU01A28Z0001s";
                pty_.send_response(f.seq, f.cmd,
                                   std::vector<uint8_t>(sn.begin(), sn.end()));
                return;
            }
            case tp::Cmd::GetGripperConfig: {
                tp::GripperConfig c{};
                c.magic = 0x47435047UL;
                c.version = 1;
                c.flags = tp::GripperConfigFlag::Valid;
                c.max_open_rad = 1.30f;
                c.min_open_rad = 0.0f;
                pty_.send_response(f.seq, f.cmd, pod_bytes(&c, sizeof(c)));
                return;
            }
            case tp::Cmd::GetGripperAutoCalConfig: {
                tp::GripperAutoCalConfig c{};
                c.magic = 0x4743414CUL;
                c.version = 1;
                c.flags = tp::GripperAutoCalFlag::Valid;
                // The firmware's own tuned numbers, so the controller's
                // advisory cross-check sees a realistic device.
                c.close_stall_torque_nm = 0.35f;
                c.open_stall_torque_nm = 0.35f;
                c.close_speed_rad_s = 0.25f;
                c.open_speed_rad_s = 0.35f;
                c.stall_hold_ms = 30;
                pty_.send_response(f.seq, f.cmd, pod_bytes(&c, sizeof(c)));
                return;
            }
            case tp::Cmd::MotorGetStartupLimitTorque: {
                const float limit = 6.0f;
                pty_.send_response(f.seq, f.cmd, pod_bytes(&limit, sizeof(limit)));
                return;
            }
            case tp::Cmd::GetMotorStatus: {
                std::lock_guard<std::mutex> lk(mu_);
                pty_.send_response(f.seq, f.cmd,
                                   pod_bytes(&status_, sizeof(status_)));
                return;
            }
            case tp::Cmd::StartStream:
                streaming_.store(true);
                pty_.send_ack_ok(f.seq, f.cmd);
                return;
            case tp::Cmd::StopStream:
                streaming_.store(false);
                pty_.send_ack_ok(f.seq, f.cmd);
                return;
            case tp::Cmd::MotorImpedanceCtrl: {
                // Fire-and-forget: no ACK, exactly like the real firmware.
                if (f.payload.size() >= sizeof(tp::MotorImpedanceCtrl)) {
                    std::lock_guard<std::mutex> lk(mu_);
                    tp::MotorImpedanceCtrl c{};
                    std::memcpy(&c, f.payload.data(), sizeof(c));
                    last_submit_ = c;
                }
                submits_.fetch_add(1);
                return;
            }
            default:
                pty_.send_nack(f.seq, tp::ErrorCode::InvalidCmd);
                return;
        }
    }

    Pty& pty_;
    uint8_t fw_major_, fw_minor_, fw_patch_;
    std::thread thread_;
    std::atomic<bool> stop_{false};
    std::atomic<bool> streaming_{false};
    std::atomic<bool> frozen_{false};
    std::atomic<unsigned> submits_{0};
    mutable std::mutex mu_;
    tp::MotorStatus status_{};
    std::optional<tp::MotorImpedanceCtrl> last_submit_;
};

std::unique_ptr<tx::FollowerGripper> open_follower(const Pty& pty) {
    tx::FollowerGripper::Config cfg;
    cfg.mcu_device = pty.slave_path();
    cfg.ack_timeout_ms = 300;
    cfg.max_retries = 1;
    return std::make_unique<tx::FollowerGripper>(cfg);
}

// Spin until `pred` or the deadline. Returns whether it held.
template <typename Pred>
bool wait_for(Pred pred, std::chrono::milliseconds budget) {
    const auto deadline = std::chrono::steady_clock::now() + budget;
    while (std::chrono::steady_clock::now() < deadline) {
        if (pred()) return true;
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    return pred();
}

}  // namespace

TEST(ForcePositionControllerPty, StartSeedsPositionHoldAndSubmitsPerStatusFrame) {
    Pty pty;
    ASSERT_GE(pty.master(), 0);
    FakeFollower fw(pty);
    auto g = open_follower(pty);

    tx::ForcePositionConfig cfg;
    tx::ForcePositionController c(*g, cfg);
    c.start();
    ASSERT_TRUE(c.running());

    EXPECT_TRUE(wait_for([&] { return fw.submit_count() >= 3; },
                         std::chrono::milliseconds(1000)))
        << "controller submitted " << fw.submit_count() << " MIT frames";
    EXPECT_EQ(c.state(), tx::ForcePositionState::HoldingPosition);

    const auto snap = c.snapshot();
    EXPECT_TRUE(snap.observation.valid);
    EXPECT_FLOAT_EQ(snap.device_limit_nm, 6.0f);
    EXPECT_LE(snap.commanded_torque_nm, cfg.motion_torque_limit_nm);
    c.stop();
}

TEST(ForcePositionControllerPty, StaleStatusStreamFaultsWithAZeroTorqueCommand) {
    Pty pty;
    ASSERT_GE(pty.master(), 0);
    FakeFollower fw(pty);
    auto g = open_follower(pty);

    tx::ForcePositionConfig cfg;
    cfg.status_timeout_ms = 120;
    tx::ForcePositionController c(*g, cfg);
    c.start();
    ASSERT_TRUE(wait_for([&] { return fw.submit_count() >= 2; },
                         std::chrono::milliseconds(1000)));

    const unsigned before = fw.submit_count();
    fw.freeze_stream(true);

    // Wait for the zero-torque frame to actually reach the wire, not merely for
    // the state to flip. policy_->fail() sets Fault while holding the lock, but
    // the submit happens outside it -- so Fault is observable before the frame
    // exists, and asserting on state alone races with the write. Waiting on the
    // submit counter is what the test is really about anyway.
    ASSERT_TRUE(wait_for([&] {
                             return c.state() == tx::ForcePositionState::Fault &&
                                    fw.submit_count() > before;
                         },
                         std::chrono::milliseconds(2000)))
        << "state " << tx::to_string(c.state()) << ", submits "
        << fw.submit_count() << " (was " << before << ")";

    const auto last = fw.last_submit();
    ASSERT_TRUE(last.has_value());
    EXPECT_FLOAT_EQ(last->kp, 0.0f);
    EXPECT_FLOAT_EQ(last->kd, 0.0f);
    EXPECT_FLOAT_EQ(last->target_torque, 0.0f);
    EXPECT_FALSE(c.snapshot().fault_reason.empty());
    c.stop();
}

// Regression: staleness used to be checked only when the submit thread woke on
// its own timeout, so a caller command (which sets step_requested_) walked
// straight past it and was computed from a stale sample -- real motion ordered
// from data of unknown age. A dead stream must win over the caller.
TEST(ForcePositionControllerPty, SetTargetOnAStaleStreamCommandsZeroTorque) {
    Pty pty;
    ASSERT_GE(pty.master(), 0);
    FakeFollower fw(pty);
    auto g = open_follower(pty);

    tx::ForcePositionConfig cfg;
    cfg.status_timeout_ms = 120;
    tx::ForcePositionController c(*g, cfg);
    c.start();
    ASSERT_TRUE(wait_for([&] { return fw.submit_count() >= 2; },
                         std::chrono::milliseconds(1000)));

    fw.freeze_stream(true);
    std::this_thread::sleep_for(std::chrono::milliseconds(200));  // now stale

    const unsigned before = fw.submit_count();
    c.set_target(0.0f, 0.35f);   // ask for a full close on stale data

    ASSERT_TRUE(wait_for([&] { return fw.submit_count() > before; },
                         std::chrono::milliseconds(1000)));
    const auto last = fw.last_submit();
    ASSERT_TRUE(last.has_value());
    EXPECT_EQ(c.state(), tx::ForcePositionState::Fault);
    EXPECT_FLOAT_EQ(last->kp, 0.0f);
    EXPECT_FLOAT_EQ(last->kd, 0.0f);
    EXPECT_FLOAT_EQ(last->target_torque, 0.0f);
    EXPECT_FLOAT_EQ(last->vel, 0.0f);
    c.stop();
}

TEST(ForcePositionControllerPty, StartRefusesADeviceLimitAboveTheMotionLimit) {
    Pty pty;
    ASSERT_GE(pty.master(), 0);
    FakeFollower fw(pty);
    auto g = open_follower(pty);

    tx::ForcePositionConfig cfg;
    cfg.motion_torque_limit_nm = 1.8f;  // device answers 6.0
    cfg.hold_torque_limit_nm = 1.8f;
    tx::ForcePositionController c(*g, cfg);
    EXPECT_THROW(c.start(), std::runtime_error);
    EXPECT_FALSE(c.running());
}

// The firmware gate. 1.1.6 is where the motion safety envelope landed, and
// everything older has no stall protection at all on the MIT command path --
// a blocked jaw is bounded only by the motor's own 0x700B ceiling, which on
// 24 V browns out the board. Opening such a device must fail loudly rather
// than quietly running one blocked grasp away from that.
TEST(FollowerFirmwareGate, RefusesFirmwareOlderThanTheMinimum) {
    Pty pty;
    ASSERT_GE(pty.master(), 0);
    FakeFollower fw(pty, 1, 1, 5);          // one patch short
    EXPECT_THROW(open_follower(pty), xense::taccap::ProtocolError);
}

TEST(FollowerFirmwareGate, AcceptsTheMinimumAndNewer) {
    {
        Pty pty;
        ASSERT_GE(pty.master(), 0);
        FakeFollower fw(pty, 1, 1, 6);
        EXPECT_NO_THROW({ auto g = open_follower(pty); });
    }
    {
        Pty pty;
        ASSERT_GE(pty.master(), 0);
        FakeFollower fw(pty, 1, 2, 0);
        EXPECT_NO_THROW({ auto g = open_follower(pty); });
    }
}

// The escape hatch has to work, or a device that needs upgrading could not be
// inspected first. (OTA itself goes through LeaderGripper and is unaffected.)
TEST(FollowerFirmwareGate, AllowOutdatedFirmwareOpensAnyway) {
    Pty pty;
    ASSERT_GE(pty.master(), 0);
    FakeFollower fw(pty, 1, 0, 2);
    tx::FollowerGripper::Config cfg;
    cfg.mcu_device = pty.slave_path();
    cfg.ack_timeout_ms = 300;
    cfg.max_retries = 1;
    cfg.allow_outdated_firmware = true;
    EXPECT_NO_THROW({ tx::FollowerGripper g(cfg); });
}
