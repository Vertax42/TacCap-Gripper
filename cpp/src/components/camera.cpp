// Copyright (c) 2026 XenseRobotics Co., Ltd. — Apache-2.0

#include <taccap/components/camera.hpp>
#include <taccap/error.hpp>

#include <opencv2/videoio.hpp>

#include <cerrno>
#include <utility>

namespace xense::taccap {

namespace {
inline cv::VideoCapture* as_cap(void* p) { return static_cast<cv::VideoCapture*>(p); }
}

Camera::Camera(const Config& cfg) : cfg_(cfg) {
    if (cfg_.device.empty()) {
        throw IoError("Camera: empty device path", EINVAL);
    }

    auto* cap = new cv::VideoCapture();
    impl_ = cap;
    try {
        if (!cap->open(cfg_.device, cv::CAP_V4L2)) {
            throw IoError("Camera: cv::VideoCapture::open(" + cfg_.device + ")", EIO);
        }
        if (cfg_.use_mjpg) {
            cap->set(cv::CAP_PROP_FOURCC,
                     cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
        }
        cap->set(cv::CAP_PROP_FRAME_WIDTH,  cfg_.width);
        cap->set(cv::CAP_PROP_FRAME_HEIGHT, cfg_.height);
        cap->set(cv::CAP_PROP_FPS,          cfg_.fps);
    } catch (...) {
        delete cap;
        impl_ = nullptr;
        throw;
    }
}

Camera::~Camera() {
    stop();
    if (impl_) {
        as_cap(impl_)->release();
        delete as_cap(impl_);
        impl_ = nullptr;
    }
}

bool Camera::read(CameraFrame& out, std::chrono::milliseconds /*timeout*/) {
    // VideoCapture::read is blocking up to its internal V4L2 timeout; the
    // `timeout` arg is currently informational. (Add poll/select wrapping
    // later if the underlying timeout proves insufficient.)
    if (!impl_) return false;
    if (running_.load()) {
        // While streaming, the worker thread owns the device — disallow
        // concurrent direct reads to keep the contract simple.
        return false;
    }
    std::lock_guard<std::mutex> lk(cap_mu_);
    cv::Mat frame;
    if (!as_cap(impl_)->read(frame) || frame.empty()) {
        ++dropped_;
        return false;
    }
    out.host_time   = std::chrono::steady_clock::now();
    out.frame_index = ++total_;
    out.image       = std::move(frame);
    return true;
}

void Camera::start(Callback cb) {
    if (running_.exchange(true)) return;   // idempotent
    worker_ = std::thread([this, cb = std::move(cb)]() { capture_loop_(cb); });
}

void Camera::stop() {
    if (!running_.exchange(false)) return;
    if (worker_.joinable()) worker_.join();
}

void Camera::capture_loop_(Callback cb) {
    using clock = std::chrono::steady_clock;
    auto* cap = as_cap(impl_);

    auto fps_window_start = clock::now();
    uint64_t fps_window_frames = 0;

    while (running_.load(std::memory_order_acquire)) {
        cv::Mat frame;
        if (!cap->read(frame) || frame.empty()) {
            ++dropped_;
            continue;
        }
        const auto now = clock::now();
        CameraFrame cf{};
        cf.host_time   = now;
        cf.frame_index = ++total_;
        cf.image       = std::move(frame);

        try {
            cb(cf);
        } catch (...) { /* swallow — callbacks must not throw */ }

        ++fps_window_frames;
        const auto elapsed = std::chrono::duration<double>(now - fps_window_start).count();
        if (elapsed >= 1.0) {
            last_fps_.store(static_cast<double>(fps_window_frames) / elapsed,
                            std::memory_order_relaxed);
            fps_window_start  = now;
            fps_window_frames = 0;
        }
    }
}

}  // namespace xense::taccap
