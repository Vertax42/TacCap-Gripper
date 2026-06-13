// Copyright (c) 2026 XenseRobotics Co., Ltd. — Apache-2.0
//
// LeaderGripper — aggregate object representing one TacCap-Gripper leader
// (master / teleop end). Owns:
//
//   - bus::Transport                       MCU control + sensor stream link
//   - IMU            via .imu()            Cmd::GetImu / DATA stream
//   - Encoder        via .encoder()        Cmd::GetEncoder / DATA stream
//
// The wrist UVC camera and the two OG visuotactile sensors are NOT opened
// by default: an external camera service owns those V4L2 devices now. They
// are still reachable via .wrist_camera() / .tactile_left() /
// .tactile_right() IF the gripper was constructed with `open_cameras=true`
// and the matching device paths/serials; otherwise those accessors throw.
//
// Leader gripper has no motor (TacCap-G1 design). Follower gripper adds
// `Motor` and is implemented in follower_gripper.hpp.
//
// Streaming lifecycle (MCU control link only — cameras stream independently):
//   start_streaming(imu_hz, encoder_hz)  -> Cmd::StartStream (IMU + Encoder)
//   stop_streaming()                     -> Cmd::StopStream
//
// Subscribers (.imu().on_data(...) etc.) can be installed before or after
// start_streaming(); they accumulate frames as soon as data starts flowing.

#pragma once

#include <taccap/bus/transport.hpp>
#include <taccap/components/camera.hpp>
#include <taccap/components/encoder.hpp>
#include <taccap/components/imu.hpp>
#include <taccap/components/key.hpp>
#include <taccap/components/sensor_errors.hpp>
#include <taccap/components/tactile_sensor.hpp>
#include <taccap/discovery.hpp>
#include <taccap/error.hpp>
#include <taccap/ota.hpp>

#include <cerrno>
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
        // Cameras off by default: the wrist UVC + OG tactile sensors are
        // owned by an external camera service. Set open_cameras=true (with
        // wrist_video / tactile_*_serial populated) to have this gripper
        // open them.
        bool        open_cameras        = false;
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
    IMU&            imu()            noexcept { return imu_; }
    Encoder&        encoder()        noexcept { return encoder_; }
    // Camera / tactile accessors throw IoError(ENODEV) unless the gripper was
    // constructed with open_cameras=true and the matching device path/serial.
    Camera&         wrist_camera()   { return deref_(wrist_, "wrist camera",  "wrist_video"); }
    TactileSensor&  tactile_left()   { return deref_(tac_l_, "tactile_left",  "tactile_left_serial"); }
    TactileSensor&  tactile_right()  { return deref_(tac_r_, "tactile_right", "tactile_right_serial"); }
    Key&            key()            noexcept { return key_; }            // V1.4
    SensorErrors&   sensor_errors()  noexcept { return errors_; }         // V1.6
    OtaSession&     ota()            noexcept { return ota_; }            // V1.3
    bus::Transport& transport()      noexcept { return t_; }

    // Streaming lifecycle.
    void start_streaming(unsigned imu_hz = 100, unsigned encoder_hz = 100);
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

    Config                          cfg_;
    bus::Transport                  t_;
    IMU                             imu_;
    Encoder                         encoder_;
    Key                             key_;       // V1.4
    SensorErrors                    errors_;    // V1.6
    OtaSession                      ota_;       // V1.3
    std::unique_ptr<Camera>         wrist_;
    std::unique_ptr<TactileSensor>  tac_l_;
    std::unique_ptr<TactileSensor>  tac_r_;
    bool                            streaming_ = false;
};

}  // namespace xense::taccap
