// Copyright (c) 2026 XenseRobotics Co., Ltd. — Apache-2.0
//
// End-to-end tests for the V2.0/V2.1 calibration link and the leader's
// normalized encoder position, driven against a PTY-backed fake firmware.
//
// The two calibration commands are worth an end-to-end pass rather than codec
// tests alone, because the interesting behaviour lives at the seam:
//
//   - The firmware reports handler errors via protocol_send_response(seq, cmd,
//     err, NULL, 0) — the command byte is ECHOED and the payload is the single
//     error byte, which bus::Transport surfaces as a "successful" 1-byte
//     response rather than a NACK. Calibration resolves that locally, and
//     ErrorCode::CalNotSet becoming an empty optional depends on it.
//   - Encoder normalization has to apply to both read_once() and to streamed
//     DATA frames, including subscribers registered before the map exists.

#include "pty_helper.hpp"

#include <gtest/gtest.h>
#include <taccap/components/calibration.hpp>
#include <taccap/components/encoder.hpp>
#include <taccap/error.hpp>
#include <taccap/gripper_position.hpp>
#include <taccap/protocol/codec.hpp>
#include <taccap/protocol/payloads.hpp>

#include <cmath>
#include <condition_variable>
#include <cstring>
#include <limits>
#include <mutex>
#include <thread>
#include <vector>

namespace tx = xense::taccap;
namespace tp = xense::taccap::protocol;
namespace tb = xense::taccap::bus;
using taccap_test::Pty;
using taccap_test::base_config;

namespace {

// The firmware read response: op byte echoed back, then the record body.
template <typename T>
std::vector<uint8_t> read_response(const T& body) {
    std::vector<uint8_t> out(1 + sizeof(T));
    out[0] = static_cast<uint8_t>(tp::CalOp::Read);
    std::memcpy(out.data() + 1, &body, sizeof(T));
    return out;
}

// The firmware error path: protocol_send_response(seq, cmd, err, NULL, 0)
// packs [err] as the payload with the command byte still echoed.
std::vector<uint8_t> error_response(tp::ErrorCode err) {
    return {static_cast<uint8_t>(err)};
}

std::vector<uint8_t> encoder_data_bytes(float position_rad) {
    tp::EncoderData raw{};
    raw.timestamp_us   = 123456;
    raw.position_rad   = position_rad;
    raw.velocity_rad_s = 0.0f;
    raw.status         = tp::EncoderStatusBit::Ok;
    raw.seq            = 7;
    std::vector<uint8_t> b(sizeof(raw));
    std::memcpy(b.data(), &raw, sizeof(raw));
    return b;
}

}  // namespace

// ============================================================
// Fisheye camera calibration (Cmd 0x2B)
// ============================================================

TEST(FisheyeCalEnd2End, ReadRequestIsOneOpByteAndDecodesTheResponse) {
    Pty pty;
    tb::Transport host(base_config(pty.slave_path()));
    tx::Calibration cal(host);

    const tp::CameraFisheyeCal stored{320.5f, 321.0f, 319.5f, 240.25f,
                                      -0.03f, 0.007f, -0.001f, 0.0002f};
    std::optional<tb::Frame> received;
    std::thread fw([&]() {
        received = pty.expect_frame();
        if (received) {
            pty.send_response(received->seq, tp::Cmd::CameraFisheyeCal,
                              read_response(stored));
        }
    });

    auto got = cal.read_fisheye();
    fw.join();

    ASSERT_TRUE(received.has_value());
    EXPECT_EQ(received->cmd, tp::Cmd::CameraFisheyeCal);
    ASSERT_EQ(received->payload.size(), 1u);
    EXPECT_EQ(received->payload[0], static_cast<uint8_t>(tp::CalOp::Read));

    ASSERT_TRUE(got.has_value());
    EXPECT_FLOAT_EQ(got->fx, 320.5f);
    EXPECT_FLOAT_EQ(got->cy, 240.25f);
    EXPECT_FLOAT_EQ(got->k4, 0.0002f);
}

TEST(FisheyeCalEnd2End, CalNotSetBecomesEmptyOptional) {
    Pty pty;
    tb::Transport host(base_config(pty.slave_path()));
    tx::Calibration cal(host);

    std::thread fw([&]() {
        auto f = pty.expect_frame();
        // Firmware echoes the cmd byte and returns the bare error code.
        if (f) pty.send_response(f->seq, tp::Cmd::CameraFisheyeCal,
                                 error_response(tp::ErrorCode::CalNotSet));
    });

    auto got = cal.read_fisheye();
    fw.join();
    EXPECT_FALSE(got.has_value());
}

TEST(FisheyeCalEnd2End, OtherErrorsStillThrow) {
    Pty pty;
    tb::Transport host(base_config(pty.slave_path()));
    tx::Calibration cal(host);

    std::thread fw([&]() {
        auto f = pty.expect_frame();
        if (f) pty.send_response(f->seq, tp::Cmd::CameraFisheyeCal,
                                 error_response(tp::ErrorCode::SysBusy));
    });

    EXPECT_THROW((void)cal.read_fisheye(), tx::ProtocolError);
    fw.join();
}

TEST(FisheyeCalEnd2End, WriteSendsOpPlusEightFloats) {
    Pty pty;
    tb::Transport host(base_config(pty.slave_path()));
    tx::Calibration cal(host);

    std::optional<tb::Frame> received;
    std::thread fw([&]() {
        received = pty.expect_frame();
        // Firmware write success: resp_len == 0 -> payload is [ERR_OK].
        if (received) pty.send_ack_ok(received->seq, tp::Cmd::CameraFisheyeCal);
    });

    const tp::CameraFisheyeCal cfg{600.0f, 601.0f, 320.0f, 240.0f,
                                   0.1f, -0.2f, 0.3f, -0.4f};
    cal.write_fisheye(cfg);
    fw.join();

    ASSERT_TRUE(received.has_value());
    EXPECT_EQ(received->cmd, tp::Cmd::CameraFisheyeCal);
    ASSERT_EQ(received->payload.size(), tp::CAMERA_FISHEYE_CAL_FULL_SIZE);
    EXPECT_EQ(received->payload[0], static_cast<uint8_t>(tp::CalOp::Write));

    tp::CameraFisheyeCal echoed{};
    std::memcpy(&echoed, received->payload.data() + 1, sizeof(echoed));
    EXPECT_EQ(0, std::memcmp(&cfg, &echoed, sizeof(cfg)));
}

TEST(FisheyeCalEnd2End, WriteRejectionThrows) {
    Pty pty;
    tb::Transport host(base_config(pty.slave_path()));
    tx::Calibration cal(host);

    std::thread fw([&]() {
        auto f = pty.expect_frame();
        if (f) pty.send_response(f->seq, tp::Cmd::CameraFisheyeCal,
                                 error_response(tp::ErrorCode::InvalidParam));
    });

    tp::CameraFisheyeCal bad{};
    bad.fx = std::numeric_limits<float>::quiet_NaN();
    EXPECT_THROW(cal.write_fisheye(bad), tx::ProtocolError);
    fw.join();
}

// ============================================================
// Leader encoder max travel angle (Cmd 0x2C)
// ============================================================

TEST(EncoderMaxCalEnd2End, ReadDecodesTheTravelAngle) {
    Pty pty;
    tb::Transport host(base_config(pty.slave_path()));
    tx::Calibration cal(host);

    std::optional<tb::Frame> received;
    std::thread fw([&]() {
        received = pty.expect_frame();
        if (received) pty.send_response(received->seq, tp::Cmd::EncoderMaxCal,
                                        read_response(1.3f));
    });

    auto got = cal.read_encoder_max_rad();
    fw.join();

    ASSERT_TRUE(received.has_value());
    EXPECT_EQ(received->cmd, tp::Cmd::EncoderMaxCal);
    ASSERT_EQ(received->payload.size(), 1u);
    ASSERT_TRUE(got.has_value());
    EXPECT_FLOAT_EQ(*got, 1.3f);
}

TEST(EncoderMaxCalEnd2End, CalNotSetBecomesEmptyOptional) {
    Pty pty;
    tb::Transport host(base_config(pty.slave_path()));
    tx::Calibration cal(host);

    std::thread fw([&]() {
        auto f = pty.expect_frame();
        if (f) pty.send_response(f->seq, tp::Cmd::EncoderMaxCal,
                                 error_response(tp::ErrorCode::CalNotSet));
    });

    EXPECT_FALSE(cal.read_encoder_max_rad().has_value());
    fw.join();
}

TEST(EncoderMaxCalEnd2End, FollowerNackThrows) {
    Pty pty;
    tb::Transport host(base_config(pty.slave_path()));
    tx::Calibration cal(host);

    // A follower has no MT6816, so 0x2C isn't in its command table.
    std::thread fw([&]() {
        auto f = pty.expect_frame();
        if (f) pty.send_response(f->seq, tp::Cmd::EncoderMaxCal,
                                 error_response(tp::ErrorCode::InvalidCmd));
    });

    EXPECT_THROW((void)cal.read_encoder_max_rad(), tx::ProtocolError);
    fw.join();
}

TEST(EncoderMaxCalEnd2End, WriteSendsOpPlusFloat) {
    Pty pty;
    tb::Transport host(base_config(pty.slave_path()));
    tx::Calibration cal(host);

    std::optional<tb::Frame> received;
    std::thread fw([&]() {
        received = pty.expect_frame();
        if (received) pty.send_ack_ok(received->seq, tp::Cmd::EncoderMaxCal);
    });

    cal.write_encoder_max_rad(1.25f);
    fw.join();

    ASSERT_TRUE(received.has_value());
    ASSERT_EQ(received->payload.size(), tp::ENCODER_MAX_CAL_FULL_SIZE);
    EXPECT_EQ(received->payload[0], static_cast<uint8_t>(tp::CalOp::Write));
    float v = 0.0f;
    std::memcpy(&v, received->payload.data() + 1, sizeof(v));
    EXPECT_FLOAT_EQ(v, 1.25f);
}

// ============================================================
// Encoder normalization (EncoderSample::position)
// ============================================================

TEST(EncoderNormalizationEnd2End, PositionIsNanWithoutAMap) {
    Pty pty;
    tb::Transport host(base_config(pty.slave_path()));
    tx::Encoder enc(host);
    EXPECT_FALSE(enc.has_position_map());

    std::thread fw([&]() {
        auto f = pty.expect_frame();
        if (f) pty.send_response(f->seq, tp::Cmd::GetEncoder,
                                 encoder_data_bytes(0.65f));
    });

    auto s = enc.read_once();
    fw.join();

    EXPECT_FLOAT_EQ(s.position_rad, 0.65f);
    EXPECT_TRUE(std::isnan(s.position));
}

TEST(EncoderNormalizationEnd2End, ReadOnceFillsPositionOnceMapped) {
    Pty pty;
    tb::Transport host(base_config(pty.slave_path()));
    tx::Encoder enc(host);
    enc.set_position_map(tx::GripperPosition::from_travel(1.3f));
    ASSERT_TRUE(enc.has_position_map());
    EXPECT_FLOAT_EQ(enc.position_map().max_open_rad(), 1.3f);

    std::thread fw([&]() {
        auto f = pty.expect_frame();
        if (f) pty.send_response(f->seq, tp::Cmd::GetEncoder,
                                 encoder_data_bytes(0.65f));
    });

    auto s = enc.read_once();
    fw.join();

    EXPECT_FLOAT_EQ(s.position_rad, 0.65f);   // still radians
    EXPECT_FLOAT_EQ(s.position,     0.5f);    // normalized
}

TEST(EncoderNormalizationEnd2End, MapIsRejectedWhenNotCalibrated) {
    Pty pty;
    tb::Transport host(base_config(pty.slave_path()));
    tx::Encoder enc(host);
    EXPECT_THROW(enc.set_position_map(tx::GripperPosition{}), tx::ProtocolError);
    EXPECT_THROW(enc.set_position_map(tx::GripperPosition::from_travel(0.0f)),
                 tx::ProtocolError);
    EXPECT_FALSE(enc.has_position_map());
}

TEST(EncoderNormalizationEnd2End, StreamedSamplesAreNormalizedToo) {
    Pty pty;
    tb::Transport host(base_config(pty.slave_path()));
    tx::Encoder enc(host);

    std::mutex mu;
    std::condition_variable cv;
    std::vector<tx::EncoderSample> got;
    // Subscribe BEFORE installing the map — a running stream must pick the
    // map up without re-subscribing.
    enc.on_data([&](const tx::EncoderSample& s) {
        std::lock_guard<std::mutex> g(mu);
        got.push_back(s);
        cv.notify_all();
    });
    enc.set_position_map(tx::GripperPosition::from_travel(2.0f));

    pty.send_data(0, tp::Cmd::GetEncoder, encoder_data_bytes(0.5f));
    pty.send_data(1, tp::Cmd::GetEncoder, encoder_data_bytes(2.0f));
    {
        std::unique_lock<std::mutex> lk(mu);
        ASSERT_TRUE(cv.wait_for(lk, std::chrono::seconds(1),
                                [&]{ return got.size() >= 2; }));
    }
    EXPECT_FLOAT_EQ(got[0].position, 0.25f);
    EXPECT_FLOAT_EQ(got[1].position, 1.0f);
}

TEST(EncoderNormalizationEnd2End, ClampedNegativeReadsNormalizeToZero) {
    Pty pty;
    tb::Transport host(base_config(pty.slave_path()));
    tx::Encoder enc(host);
    enc.set_position_map(tx::GripperPosition::from_travel(1.3f));

    // Small post-zero drift: position_rad is clamped to 0, and the normalized
    // value must be derived from the clamped reading, not the raw one.
    std::thread fw([&]() {
        auto f = pty.expect_frame();
        if (f) pty.send_response(f->seq, tp::Cmd::GetEncoder,
                                 encoder_data_bytes(-0.02f));
    });

    auto s = enc.read_once();
    fw.join();

    EXPECT_FLOAT_EQ(s.position_rad,     0.0f);
    EXPECT_FLOAT_EQ(s.raw.position_rad, -0.02f);
    EXPECT_FLOAT_EQ(s.position,         0.0f);
}

TEST(EncoderNormalizationEnd2End, ClearingTheMapRestoresNan) {
    Pty pty;
    tb::Transport host(base_config(pty.slave_path()));
    tx::Encoder enc(host);
    enc.set_position_map(tx::GripperPosition::from_travel(1.3f));
    enc.clear_position_map();
    EXPECT_FALSE(enc.has_position_map());
    EXPECT_FALSE(enc.position_map().valid());

    std::thread fw([&]() {
        auto f = pty.expect_frame();
        if (f) pty.send_response(f->seq, tp::Cmd::GetEncoder,
                                 encoder_data_bytes(0.65f));
    });

    auto s = enc.read_once();
    fw.join();
    EXPECT_TRUE(std::isnan(s.position));
}
