// Copyright (c) 2026 XenseRobotics Co., Ltd. — Apache-2.0
//
// FollowerGripper — aggregate object representing one TacCap-Gripper follower
// (executor / robot end). Same sensor surface as LeaderGripper plus a Motor
// component that drives the FDCAN-attached actuator on the MCU.
//
// Owns:
//   - bus::Transport                       MCU control + sensor stream link
//   - IMU            via .imu()            Cmd::GetImu / DATA stream
//   - Encoder        via .encoder()        Cmd::GetEncoder / DATA stream
//   - Motor          via .motor()          enable/control + GetMotorStatus
//
// The wrist UVC camera is NOT opened by default (an external camera service
// owns that V4L2 device). It is reachable via .wrist_camera() only when
// constructed with `open_cameras=true` and the matching path; otherwise the
// accessor throws. (The OG visuotactile sensors are not handled here — they
// are read at the Python level via the xensesdk wheel.)
//
// Streaming lifecycle (extends LeaderGripper with optional motor telemetry):
//   start_streaming(imu_hz, encoder_hz, motor_hz=0)
//     - motor_hz=0 → motor status not streamed (read on demand via motor().read_status())
//     - motor_hz>0 → StreamSrc::MotorStatus added to source_mask
//   stop_streaming()
//
// Discovery is shared with LeaderGripper: hardware enumeration cannot
// distinguish leader-build from follower-build PCBs (firmware role is a
// runtime distinction). The caller picks which class to instantiate. A
// FollowerGripper.open() on leader hardware will succeed at construction but
// Motor commands will return ProtocolError(SensorOffline) at runtime — which
// is the right time to surface the mismatch.

#pragma once

#include <taccap/bus/transport.hpp>
#include <taccap/components/camera.hpp>
#include <taccap/components/encoder.hpp>
#include <taccap/components/imu.hpp>
#include <taccap/components/key.hpp>
#include <taccap/components/motor.hpp>
#include <taccap/components/sensor_errors.hpp>
#include <taccap/discovery.hpp>
#include <taccap/error.hpp>
#include <taccap/gripper_position.hpp>
#include <taccap/ota.hpp>

#include <cerrno>
#include <chrono>
#include <memory>
#include <string>

namespace xense::taccap {

class FollowerGripper {
public:
    struct Config {
        std::string mcu_device;             // /dev/serial/by-id/... -if02
        std::string wrist_video;            // /dev/v4l/by-id/... -video-index0
        uint32_t    baudrate            = 3'000'000;
        // Match LeaderGripper defaults: the firmware can be slow to ACK
        // StartStream / StopStream while a previous stream is still
        // flushing — generous window + a couple of retries cover that.
        unsigned    ack_timeout_ms      = 1000;
        unsigned    max_retries         = 2;
        // Wrist camera off by default: the wrist UVC device is owned by an
        // external camera service. Set open_cameras=true (with wrist_video
        // populated) to have this gripper open it.
        bool        open_cameras        = false;
        Camera::Config wrist_cam_extra{};   // width/height/fps overrides
    };

    explicit FollowerGripper(const Config& cfg);
    ~FollowerGripper();

    FollowerGripper(const FollowerGripper&)            = delete;
    FollowerGripper& operator=(const FollowerGripper&) = delete;

    // Auto-discover. Same enumeration path as LeaderGripper::open() — the
    // caller is responsible for knowing they have follower hardware.
    static std::unique_ptr<FollowerGripper> open();

    // Component accessors.
    IMU&            imu()            noexcept { return imu_; }
    Encoder&        encoder()        noexcept { return encoder_; }
    // Wrist camera accessor throws IoError(ENODEV) unless the gripper was
    // constructed with open_cameras=true and the matching device path.
    Camera&         wrist_camera()   { return deref_(wrist_, "wrist camera",  "wrist_video"); }
    Motor&          motor()          noexcept { return motor_; }

    // ---- Follower gripper open/close limit config (Cmd 0x66/0x67) ----------
    // Read / write the follower's open/close limit config. NACKs as
    // SensorOffline on a leader (no follower config there).
    protocol::GripperConfig get_gripper_config(
        std::chrono::milliseconds timeout = std::chrono::milliseconds{100});
    void set_gripper_config(const protocol::GripperConfig& cfg);

    // ---- Normalized gripper position (0 = closed, 1 = open) -----------------
    // Convenience layer over the motor + GripperConfig so callers work in a
    // normalized [0,1] position instead of raw shaft radians. NOTE: this is
    // distinct from motor().set_position(), which takes RAW radians — these
    // FollowerGripper methods are normalized [0,1].
    //
    // The mapping (closed = motor zero, travel = max_open_rad, direction from
    // the Reverse flag) is read once from the firmware via Cmd::GetGripperConfig
    // and cached; call reload_config() after re-calibrating. All methods throw
    // ProtocolError if the gripper isn't calibrated (config not Valid).
    //
    // The motor must be enabled before set_position() moves anything; like
    // Motor::submit_*, set_position() is fire-and-forget (no ACK) for a host
    // realtime loop — poll motor().control_stats() for health.
    float position(                                       // read: raw -> [0,1]
        std::chrono::milliseconds timeout = std::chrono::milliseconds{100});
    void  set_position(float position,                    // command in [0,1], no-ACK
                       float kp_nm_per_rad,
                       float kd_nm_s_per_rad,
                       float feedforward_torque_nm = 0.0f);
    float pos_to_rad(float position);                     // [0,1] -> raw rad
    float rad_to_pos(float raw_rad);                      // raw rad -> [0,1]
    const GripperPosition& position_map();                // cached converter (loads if needed)
    void  reload_config();                                // re-read + rebuild converter

    Key&            key()            noexcept { return key_; }            // V1.4
    SensorErrors&   sensor_errors()  noexcept { return errors_; }         // V1.6
    OtaSession&     ota()            noexcept { return ota_; }            // V1.3
    bus::Transport& transport()      noexcept { return t_; }

    // Streaming lifecycle. motor_hz=0 means "don't stream motor status",
    // matching the leader streaming surface; non-zero adds MotorStatus to
    // the source mask so on_status() subscribers receive at that cadence.
    void start_streaming(unsigned imu_hz     = 100,
                         unsigned encoder_hz = 100,
                         unsigned motor_hz   = 0);
    void stop_streaming();
    bool is_streaming() const noexcept { return streaming_; }

    const Config& config() const noexcept { return cfg_; }

private:
    // Dereference an optional component, or throw a clear IoError naming the
    // Config field the caller must set (with open_cameras=true) to enable it.
    template <typename T>
    static T& deref_(const std::unique_ptr<T>& p, const char* what,
                     const char* cfg_field) {
        if (!p) {
            throw IoError(std::string(what) + " not opened (construct with "
                          "open_cameras=true and " + cfg_field + " set)",
                          ENODEV);
        }
        return *p;
    }

    // Load + cache the GripperPosition converter from the firmware GripperConfig
    // on first use. Throws ProtocolError if the config isn't Valid (uncalibrated).
    void ensure_position_map_();

    Config                          cfg_;
    bus::Transport                  t_;
    IMU                             imu_;
    Encoder                         encoder_;
    Motor                           motor_;
    Key                             key_;       // V1.4
    SensorErrors                    errors_;    // V1.6
    OtaSession                      ota_;       // V1.3
    std::unique_ptr<Camera>         wrist_;
    bool                            streaming_ = false;
    GripperPosition                 pos_map_;             // raw<->position, cached
    bool                            pos_map_loaded_ = false;
};

}  // namespace xense::taccap
