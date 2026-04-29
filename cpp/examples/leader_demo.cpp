// Copyright (c) 2026 XenseRobotics Co., Ltd. — Apache-2.0
//
// 5-second multi-stream smoke test on a real leader gripper.
// Open everything, count frames, print per-stream effective rate.

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
    std::cout << "  MCU:           " << c.mcu_device << "\n"
              << "  Wrist cam:     " << c.wrist_video << "\n"
              << "  Tactile L:     " << c.tactile_left_serial << "\n"
              << "  Tactile R:     " << c.tactile_right_serial << "\n";

    std::atomic<uint64_t> imu_n{0}, enc_n{0}, tac_l_n{0}, tac_r_n{0}, wrist_n{0};

    g->imu().on_data([&](const auto&) { ++imu_n; });
    g->encoder().on_data([&](const auto&) { ++enc_n; });
    g->tactile_left().start([&](const auto&)  { ++tac_l_n; });
    g->tactile_right().start([&](const auto&) { ++tac_r_n; });
    g->wrist_camera().start([&](const auto&)  { ++wrist_n; });

    g->start_streaming(/*imu_hz=*/100, /*encoder_hz=*/100);

    std::cout << "Streaming for 5s...\n";
    const auto t0 = std::chrono::steady_clock::now();
    std::this_thread::sleep_for(5s);
    const auto dt = std::chrono::duration<double>(
                        std::chrono::steady_clock::now() - t0).count();

    g->stop_streaming();
    g->tactile_left().stop();
    g->tactile_right().stop();
    g->wrist_camera().stop();

    auto fps = [&](uint64_t n) { return static_cast<double>(n) / dt; };
    std::cout << "\n=== Stream rates over " << dt << "s ===\n";
    std::cout << "  IMU         : " << imu_n.load()   << " frames | " << fps(imu_n.load())   << " fps\n";
    std::cout << "  Encoder     : " << enc_n.load()   << " frames | " << fps(enc_n.load())   << " fps\n";
    std::cout << "  Tactile L   : " << tac_l_n.load() << " frames | " << fps(tac_l_n.load()) << " fps\n";
    std::cout << "  Tactile R   : " << tac_r_n.load() << " frames | " << fps(tac_r_n.load()) << " fps\n";
    std::cout << "  Wrist cam   : " << wrist_n.load() << " frames | " << fps(wrist_n.load()) << " fps\n";

    return 0;
}
