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
//   - TactileSensor  via .tactile_left()   OG sensor #1 (libxense lite + rectify)
//   - TactileSensor  via .tactile_right()  OG sensor #2
//   - Camera         via .wrist_camera()   wrist UVC camera
//   - Motor          via .motor()          enable/control + GetMotorStatus
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
#include <taccap/components/motor.hpp>
#include <taccap/components/tactile_sensor.hpp>
#include <taccap/discovery.hpp>

#include <memory>
#include <string>

namespace xense::taccap {

class FollowerGripper {
public:
    struct Config {
        std::string mcu_device;             // /dev/serial/by-id/... -if02
        std::string wrist_video;            // /dev/v4l/by-id/... -video-index0
        std::string tactile_left_serial;    // OG..."
        std::string tactile_right_serial;   // OG..."
        uint32_t    baudrate            = 3'000'000;
        // Match LeaderGripper defaults: the firmware can be slow to ACK
        // StartStream / StopStream while a previous stream is still
        // flushing — generous window + a couple of retries cover that.
        unsigned    ack_timeout_ms      = 1000;
        unsigned    max_retries         = 2;
        bool        rectify_tactile     = true;
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
    IMU&            imu()           noexcept { return imu_; }
    Encoder&        encoder()       noexcept { return encoder_; }
    Camera&         wrist_camera()  noexcept { return wrist_; }
    TactileSensor&  tactile_left()  noexcept { return *tac_l_; }
    TactileSensor&  tactile_right() noexcept { return *tac_r_; }
    Motor&          motor()         noexcept { return motor_; }
    bus::Transport& transport()     noexcept { return t_; }

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
    Config                          cfg_;
    bus::Transport                  t_;
    IMU                             imu_;
    Encoder                         encoder_;
    Motor                           motor_;
    Camera                          wrist_;
    std::unique_ptr<TactileSensor>  tac_l_;
    std::unique_ptr<TactileSensor>  tac_r_;
    bool                            streaming_ = false;
};

}  // namespace xense::taccap
