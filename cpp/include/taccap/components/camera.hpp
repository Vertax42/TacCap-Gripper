// Copyright (c) 2026 XenseRobotics Co., Ltd. — Apache-2.0
//
// Camera component: thin wrapper around OpenCV's V4L2 capture for the
// wrist-mounted UVC camera (Sunplus XC...). Uses cv::VideoCapture under
// the hood — exposed as raw cv::Mat frames. For the visuotactile (OG)
// cameras use TactileSensor instead, which goes through libxense lite to
// pick up the on-sensor calibration / rectification path.
//
// We intentionally don't pull pybind11/numpy here — Python bindings live
// in python/bindings/components.cpp.

#pragma once

#include <opencv2/core.hpp>

#include <atomic>
#include <chrono>
#include <cstdint>
#include <functional>
#include <mutex>
#include <string>
#include <thread>

namespace xense::taccap {

struct CameraFrame {
    std::chrono::steady_clock::time_point host_time;
    uint64_t                              frame_index;   // monotonic per Camera instance
    cv::Mat                               image;         // BGR8, owned
};

class Camera {
public:
    using Callback = std::function<void(const CameraFrame&)>;

    struct Config {
        std::string device   = "";       // e.g. /dev/video3 or /dev/serial/by-id/...
        int         width    = 640;
        int         height   = 480;
        double      fps      = 30.0;
        bool        use_mjpg = true;     // request MJPEG fourcc
    };

    explicit Camera(const Config& cfg);
    ~Camera();
    Camera(const Camera&)            = delete;
    Camera& operator=(const Camera&) = delete;

    // Synchronous one-shot read. Returns false on read failure.
    bool read(CameraFrame& out, std::chrono::milliseconds timeout =
              std::chrono::milliseconds{500});

    // Async streaming: spawn a background thread that calls `cb` on each
    // frame. Stop with stop() or destructor. Re-entrant safe (callback
    // invocation is serialised on the capture thread).
    void start(Callback cb);
    void stop();
    bool is_streaming() const noexcept { return running_.load(); }

    // Stats
    uint64_t total_frames() const noexcept { return total_; }
    uint64_t dropped_frames() const noexcept { return dropped_; }
    double   actual_fps() const noexcept { return last_fps_.load(); }

    const Config& config() const noexcept { return cfg_; }

private:
    void capture_loop_(Callback cb);

    Config            cfg_;
    void*             impl_ = nullptr;   // opaque cv::VideoCapture* (kept void* so
                                         // the public header doesn't drag in OpenCV
                                         // includes for users that don't need them)
    std::thread       worker_;
    std::atomic<bool> running_{false};
    std::atomic<uint64_t> total_{0};
    std::atomic<uint64_t> dropped_{0};
    std::atomic<double>   last_fps_{0.0};
    std::mutex            cap_mu_;       // guards reads when start() inactive
};

}  // namespace xense::taccap
