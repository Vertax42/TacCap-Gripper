// Copyright (c) 2026 XenseRobotics Co., Ltd. — Apache-2.0
//
// LeaderGripper — aggregate object representing one TacCap-Gripper leader
// (master / teleop end). Owns:
//
//   - bus::Transport                       MCU control + sensor stream link
//   - IMU            via .imu()            Cmd::GetImu / DATA stream
//   - Encoder        via .encoder()        Cmd::GetEncoder / DATA stream
//   - TactileSensor  via .tactile_left()   OG sensor #1 (libxense lite + rectify)
//   - TactileSensor  via .tactile_right()  OG sensor #2
//   - Camera         via .wrist_camera()   wrist UVC camera
//
// Leader gripper has no motor (TacCap-G1 design). Follower gripper adds
// `Motor` and is implemented in follower_gripper.hpp (later).
//
// Streaming lifecycle:
//   start_streaming(imu_hz, encoder_hz)
//     1. send Cmd::StartStream with StreamConfig (IMU + Encoder)
//     2. start all UVC cameras' background capture
//   stop_streaming()
//     1. stop all UVC cameras
//     2. send Cmd::StopStream
//
// Subscribers (.imu().on_data(...) / .tactile_left().start(...)) can be
// installed before or after start_streaming(); they accumulate frames as
// soon as data starts flowing.

#pragma once

#include <taccap/bus/transport.hpp>
#include <taccap/components/camera.hpp>
#include <taccap/components/encoder.hpp>
#include <taccap/components/imu.hpp>
#include <taccap/components/tactile_sensor.hpp>
#include <taccap/discovery.hpp>

#include <memory>
#include <string>

namespace xense::taccap {

class LeaderGripper {
public:
    struct Config {
        std::string mcu_device;             // /dev/ttyACM1 / /dev/taccap-leader / by-id
        std::string wrist_video;            // /dev/video3 / by-id
        std::string tactile_left_serial;    // OG..."
        std::string tactile_right_serial;   // OG..."
        uint32_t    baudrate            = 3'000'000;
        // The firmware can be slow to ACK StartStream / StopStream while
        // a previous stream is still flushing — give it a generous window
        // so a back-to-back run doesn't time out before the queued DATA
        // frames drain through the kernel rx buffer.
        unsigned    ack_timeout_ms      = 1000;
        unsigned    max_retries         = 2;
        bool        rectify_tactile     = true;
        Camera::Config wrist_cam_extra{};   // width/height/fps overrides
    };

    // Construct from explicit config (for tests, custom topologies).
    explicit LeaderGripper(const Config& cfg);
    ~LeaderGripper();

    LeaderGripper(const LeaderGripper&)            = delete;
    LeaderGripper& operator=(const LeaderGripper&) = delete;

    // Auto-discover everything. Throws IoError if hardware not found.
    // Returns a unique_ptr because LeaderGripper itself isn't copyable or
    // movable (its members hold mutexes / threads / atomics).
    static std::unique_ptr<LeaderGripper> open();

    // Component accessors.
    IMU&            imu()           noexcept { return imu_; }
    Encoder&        encoder()       noexcept { return encoder_; }
    Camera&         wrist_camera()  noexcept { return wrist_; }
    TactileSensor&  tactile_left()  noexcept { return *tac_l_; }
    TactileSensor&  tactile_right() noexcept { return *tac_r_; }
    bus::Transport& transport()     noexcept { return t_; }

    // Streaming lifecycle.
    void start_streaming(unsigned imu_hz = 100, unsigned encoder_hz = 100);
    void stop_streaming();
    bool is_streaming() const noexcept { return streaming_; }

    const Config& config() const noexcept { return cfg_; }

private:
    Config                          cfg_;
    bus::Transport                  t_;
    IMU                             imu_;
    Encoder                         encoder_;
    Camera                          wrist_;
    std::unique_ptr<TactileSensor>  tac_l_;
    std::unique_ptr<TactileSensor>  tac_r_;
    bool                            streaming_ = false;
};

}  // namespace xense::taccap
