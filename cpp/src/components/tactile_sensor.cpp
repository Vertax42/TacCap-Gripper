// Copyright (c) 2026 XenseRobotics Co., Ltd. — Apache-2.0

#include <taccap/components/tactile_sensor.hpp>
#include <taccap/error.hpp>

#include <cstring>

namespace xense::taccap {

namespace {

// Convert a libxense Frame (BGR8 after MJPG decode) into an owned cv::Mat.
// Clones the data so the buffer outlives the libxense callback scope.
cv::Mat frame_to_mat(const Frame& f) {
    cv::Mat view(f.height(), f.width(), CV_8UC3,
                 const_cast<void*>(f.data()));
    return view.clone();
}

}  // namespace

TactileSensor::TactileSensor(const Config& cfg) : cfg_(cfg) {
    if (cfg_.serial.empty()) {
        throw IoError("TactileSensor: empty serial", EINVAL);
    }

    ctx_ = Context::create();
    auto infos = ctx_->enumerate_devices();
    for (const auto& info : infos) {
        if (info.serial == cfg_.serial) {
            device_ = ctx_->create_device(info);
            break;
        }
    }
    if (!device_) {
        throw IoError("TactileSensor: serial " + cfg_.serial +
                      " not enumerated; check USB connection", ENODEV);
    }

    sensor_ = device_->get_sensor();
    if (cfg_.rectify) {
        auto rect_cfg = device_->get_rectify_config();
        rectifier_ = std::make_unique<Rectifier>(rect_cfg);
    }
}

TactileSensor::~TactileSensor() { stop(); }

void TactileSensor::start(Callback cb) {
    if (streaming_.exchange(true)) return;
    sensor_->start([this, cb = std::move(cb)](Frame raw) {
        TactileFrame tf{};
        tf.host_time   = std::chrono::steady_clock::now();
        tf.frame_index = ++emitted_;
        tf.raw         = frame_to_mat(raw);
        if (rectifier_) {
            Frame rectified = rectifier_->process(raw);
            tf.rectified   = frame_to_mat(rectified);
        }
        try { cb(tf); }
        catch (...) { /* swallow */ }
    });
}

void TactileSensor::stop() {
    if (!streaming_.exchange(false)) return;
    if (sensor_) sensor_->stop();
}

uint64_t TactileSensor::total_frames() const noexcept {
    return sensor_ ? sensor_->total_frames() : 0;
}
uint64_t TactileSensor::dropped_frames() const noexcept {
    return sensor_ ? sensor_->dropped_frames() : 0;
}
double TactileSensor::actual_fps() const noexcept {
    return sensor_ ? sensor_->actual_fps() : 0.0;
}

}  // namespace xense::taccap
