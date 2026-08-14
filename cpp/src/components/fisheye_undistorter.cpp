// Copyright (c) 2026 XenseRobotics Co., Ltd. — Apache-2.0

#include <taccap/components/fisheye_undistorter.hpp>
#include <taccap/error.hpp>

#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <string>

namespace xense::taccap {

FisheyeUndistorter::FisheyeUndistorter(const protocol::CameraFisheyeCal& cal,
                                       cv::Size size,
                                       float    balance)
    : size_(size) {
    if (size.width != FISHEYE_CALIB_WIDTH || size.height != FISHEYE_CALIB_HEIGHT) {
        throw Error(
            "FisheyeUndistorter: the wrist fisheye intrinsics are calibrated at " +
            std::to_string(FISHEYE_CALIB_WIDTH) + "x" +
            std::to_string(FISHEYE_CALIB_HEIGHT) + ", got " +
            std::to_string(size.width) + "x" + std::to_string(size.height) +
            ". The firmware record carries no image size, so rescaling would be "
            "a guess — configure the camera at the calibrated resolution.");
    }

    balance_ = std::clamp(balance, 0.0f, 1.0f);
    focal_scale_ = FISHEYE_NATURAL_FOCAL_SCALE +
                   static_cast<double>(balance_) *
                       (FISHEYE_WIDE_FOCAL_SCALE - FISHEYE_NATURAL_FOCAL_SCALE);

    cv::Mat k = (cv::Mat_<double>(3, 3) <<
                 cal.fx, 0.0,    cal.cx,
                 0.0,    cal.fy, cal.cy,
                 0.0,    0.0,    1.0);
    cv::Mat d = (cv::Mat_<double>(4, 1) << cal.k1, cal.k2, cal.k3, cal.k4);

    // Only fx/fy move; the principal point stays put so the image centre does
    // not drift as `balance` changes.
    new_k_ = k.clone();
    new_k_.at<double>(0, 0) *= focal_scale_;
    new_k_.at<double>(1, 1) *= focal_scale_;

    cv::fisheye::initUndistortRectifyMap(
        k, d, cv::Mat::eye(3, 3, CV_64F), new_k_, size_, CV_32FC1,
        map_x_, map_y_);
}

cv::Mat FisheyeUndistorter::apply(const cv::Mat& src) const {
    if (src.empty()) {
        throw Error("FisheyeUndistorter::apply: empty frame");
    }
    if (src.cols != size_.width || src.rows != size_.height) {
        throw Error(
            "FisheyeUndistorter::apply: frame is " + std::to_string(src.cols) +
            "x" + std::to_string(src.rows) + " but the remap tables were built "
            "for " + std::to_string(size_.width) + "x" +
            std::to_string(size_.height));
    }
    cv::Mat dst;
    // Argument order is (src, dst, map1, map2) — swapping dst and map1 still
    // compiles and fails at runtime with a -215 assertion inside remap.
    cv::remap(src, dst, map_x_, map_y_, cv::INTER_LINEAR, cv::BORDER_CONSTANT);
    return dst;
}

}  // namespace xense::taccap
