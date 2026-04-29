// Copyright (c) 2026 XenseRobotics Co., Ltd. — Apache-2.0
//
// TactileSensor component: visuotactile sensor (Xense OG series, planar)
// access path through libxense lite. We don't own the hardware abstraction
// — libxense provides Context/Device/Sensor/Rectifier already — but we
// expose a flatter API tuned for TacCap-Gripper use cases:
//
//   - open by serial string (e.g. "OG000477")
//   - start() with a callback receiving rectified frames as cv::Mat
//   - underlying Frame / Sensor / Rectifier still accessible for advanced use
//
// Calibration is read from the on-sensor flash via V4L2 XU (libxense's
// existing path); no host-side calibration files needed.

#pragma once

#include <taccap/vision.hpp>   // re-exports xense::Sensor, Rectifier, Frame, etc.

#include <opencv2/core.hpp>

#include <atomic>
#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <string>

namespace xense::taccap {

struct TactileFrame {
    std::chrono::steady_clock::time_point host_time;
    uint64_t                              frame_index;
    cv::Mat                               raw;        // original sensor frame (BGR8)
    cv::Mat                               rectified;  // rectifier output (BGR8); empty if rectify disabled
};

class TactileSensor {
public:
    using Callback = std::function<void(const TactileFrame&)>;

    struct Config {
        std::string serial;        // e.g. "OG000477"
        bool        rectify = true;
    };

    explicit TactileSensor(const Config& cfg);
    ~TactileSensor();
    TactileSensor(const TactileSensor&)            = delete;
    TactileSensor& operator=(const TactileSensor&) = delete;

    // Streaming control. `start` registers a callback invoked from libxense's
    // sensor capture thread (don't block in the callback).
    void start(Callback cb);
    void stop();
    bool is_streaming() const noexcept { return streaming_.load(); }

    // Underlying primitives for power users.
    Sensor&    sensor()           { return *sensor_; }
    Rectifier& rectifier()        { return *rectifier_; }
    const std::string& serial() const noexcept { return cfg_.serial; }

    // Stats (mirror libxense's Sensor counters).
    uint64_t total_frames()   const noexcept;
    uint64_t dropped_frames() const noexcept;
    double   actual_fps()     const noexcept;

private:
    Config                       cfg_;
    std::shared_ptr<Context>     ctx_;
    std::shared_ptr<Device>      device_;
    std::shared_ptr<Sensor>      sensor_;
    std::unique_ptr<Rectifier>   rectifier_;
    std::atomic<bool>            streaming_{false};
    std::atomic<uint64_t>        emitted_{0};
};

}  // namespace xense::taccap
