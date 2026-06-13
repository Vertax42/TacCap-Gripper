// Copyright (c) 2026 XenseRobotics Co., Ltd. — Apache-2.0
//
// 5-second MCU-stream smoke test on a real leader gripper.
// Discover, stream IMU + Encoder, count frames, print per-stream rate.
//
// The wrist camera and OG tactile sensors are owned by an external camera
// service and are not opened here. To exercise them, construct a
// LeaderGripper explicitly with open_cameras=true plus the device paths.

#include <taccap/leader_gripper.hpp>

#include <atomic>
#include <chrono>
#include <iostream>
#include <thread>

int main() {
    using namespace std::chrono_literals;

    std::cout << "Discovering leader gripper...\n";
    auto g = xense::taccap::LeaderGripper::open();
    const auto& c = g->config();
    std::cout << "  MCU:           " << c.mcu_device << "\n";

    std::atomic<uint64_t> imu_n{0}, enc_n{0};

    g->imu().on_data([&](const auto&) { ++imu_n; });
    g->encoder().on_data([&](const auto&) { ++enc_n; });

    g->start_streaming(/*imu_hz=*/100, /*encoder_hz=*/100);

    std::cout << "Streaming for 5s...\n";
    const auto t0 = std::chrono::steady_clock::now();
    std::this_thread::sleep_for(5s);
    const auto dt = std::chrono::duration<double>(
                        std::chrono::steady_clock::now() - t0).count();

    g->stop_streaming();

    auto fps = [&](uint64_t n) { return static_cast<double>(n) / dt; };
    std::cout << "\n=== Stream rates over " << dt << "s ===\n";
    std::cout << "  IMU         : " << imu_n.load() << " frames | " << fps(imu_n.load()) << " fps\n";
    std::cout << "  Encoder     : " << enc_n.load() << " frames | " << fps(enc_n.load()) << " fps\n";

    return 0;
}
