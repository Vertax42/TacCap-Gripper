// Copyright (c) 2026 XenseRobotics Co., Ltd. — Apache-2.0

#include <taccap/discovery.hpp>
#include <taccap/error.hpp>
#include <taccap/vision.hpp>

#include <algorithm>
#include <cctype>
#include <cerrno>
#include <filesystem>
#include <regex>
#include <system_error>

namespace xense::taccap::discovery {

namespace fs = std::filesystem;

namespace {

bool is_digit_(char c) noexcept { return c >= '0' && c <= '9'; }

// Last digit (0-9 only, ignoring trailing letters) → Side.
}  // namespace

std::optional<Side> side_from_serial(const std::string& s) noexcept {
    for (auto it = s.rbegin(); it != s.rend(); ++it) {
        if (is_digit_(*it)) {
            return ((*it - '0') % 2 == 1) ? Side::Left : Side::Right;
        }
    }
    return std::nullopt;
}

namespace {

// Extract SN from "/dev/serial/by-id/usb-1a86_USB_Dual_Serial_5C2C247728-if02".
// Returns the SN portion or empty string on miss.
std::string parse_ch343_serial(const std::string& path) {
    static const std::regex re(
        R"(usb-1a86_USB_Dual_Serial_([^-]+)-if02$)");
    std::smatch m;
    if (std::regex_search(path, m, re)) return m[1].str();
    return {};
}

// Extract SN from "/dev/v4l/by-id/usb-Xense_..._XC000008_..._-video-index0".
std::string parse_xc_serial(const std::string& path) {
    static const std::regex re(R"(_(XC\d+)_)");
    std::smatch m;
    if (std::regex_search(path, m, re)) return m[1].str();
    return {};
}

}  // namespace

// ---- scanners -------------------------------------------------------------

std::vector<McuEndpoint> scan_mcus() {
    std::vector<McuEndpoint> out;
    std::error_code ec;
    fs::path by_id = "/dev/serial/by-id";
    if (!fs::is_directory(by_id, ec)) return out;
    for (auto& e : fs::directory_iterator(by_id, ec)) {
        if (ec) break;
        const std::string p = e.path().string();
        // Only the if02 interface of the CH343 dual-serial is the MCU control link.
        const auto sn = parse_ch343_serial(p);
        if (sn.empty()) continue;
        auto s = side_from_serial(sn);
        if (!s) continue;
        out.push_back({p, sn, *s});
    }
    return out;
}

std::vector<OgEndpoint> scan_og_sensors() {
    std::vector<OgEndpoint> out;
    auto ctx = Context::create();
    auto infos = ctx->enumerate_devices();
    for (const auto& info : infos) {
        if (info.serial.rfind("OG", 0) != 0) continue;   // not an OG-series device
        auto s = side_from_serial(info.serial);
        if (!s) continue;
        out.push_back({info.serial, info.video_path, *s});
    }
    return out;
}

std::vector<WristCameraEndpoint> scan_wrist_cameras() {
    std::vector<WristCameraEndpoint> out;
    std::error_code ec;
    fs::path by_id = "/dev/v4l/by-id";
    if (!fs::is_directory(by_id, ec)) return out;
    for (auto& e : fs::directory_iterator(by_id, ec)) {
        if (ec) break;
        const std::string p = e.path().string();
        // Pick the main video plane only; ignore metadata indices.
        if (p.find("-video-index0") == std::string::npos &&
            p.find("-index0")       == std::string::npos) continue;
        // Identify XC* (Xense wrist camera) by name.
        const auto sn = parse_xc_serial(p);
        if (sn.empty()) {
            // Also accept generic Sunplus path for cases where the firmware
            // hasn't been programmed with an XC SN yet.
            if (p.find("usb-Sunplus") == std::string::npos &&
                p.find("usb-1bcf")    == std::string::npos) continue;
        }
        WristCameraEndpoint w{};
        w.device = p;
        w.serial = sn;
        w.side   = sn.empty() ? std::nullopt : side_from_serial(sn);
        out.push_back(std::move(w));
    }
    return out;
}

std::vector<GripperEndpoints> scan_all() {
    std::vector<GripperEndpoints> out;

    auto mcus  = scan_mcus();
    if (mcus.empty()) return out;

    auto ogs    = scan_og_sensors();
    auto wrists = scan_wrist_cameras();

    // Single-gripper case: pair the lone MCU with the OG sensors and wrist
    // camera that are visible. Multi-gripper systems will need USB-topology
    // matching (TODO: walk /sys/bus/usb to attribute each device to its
    // upstream hub and group accordingly).
    if (mcus.size() == 1) {
        GripperEndpoints g{};
        g.side       = mcus[0].side;
        g.mcu_device = mcus[0].device;
        g.mcu_serial = mcus[0].serial_number;
        for (const auto& og : ogs) {
            (og.side == Side::Left ? g.tactile_left_serial
                                   : g.tactile_right_serial) = og.serial;
        }
        if (!wrists.empty()) g.wrist_video = wrists.front().device;
        out.push_back(std::move(g));
        return out;
    }

    // Multi-gripper: split MCUs by side. We don't yet know which OG / wrist
    // belongs to which MCU, so each gripper gets its same-side OG (if any)
    // and the wrist of the same parity (if available).
    for (const auto& mcu : mcus) {
        GripperEndpoints g{};
        g.side       = mcu.side;
        g.mcu_device = mcu.device;
        g.mcu_serial = mcu.serial_number;
        for (const auto& og : ogs) {
            // For now: bilateral grippers each report two OGs locally; without
            // hub matching we just put both OGs of matching parity rule into
            // the single gripper's slot. This works for one-gripper-at-a-time
            // captures and overwrites benignly when we have a real matcher.
            (og.side == Side::Left ? g.tactile_left_serial
                                   : g.tactile_right_serial) = og.serial;
        }
        for (const auto& w : wrists) {
            if (w.side && *w.side == mcu.side) {
                g.wrist_video = w.device;
                break;
            }
        }
        if (g.wrist_video.empty() && !wrists.empty()) {
            g.wrist_video = wrists.front().device;
        }
        out.push_back(std::move(g));
    }
    return out;
}

GripperEndpoints find_one() {
    auto v = scan_all();
    if (v.empty()) {
        throw IoError("discovery::find_one: no gripper detected", ENODEV);
    }
    if (v.size() > 1) {
        throw IoError("discovery::find_one: multiple grippers detected (" +
                      std::to_string(v.size()) +
                      "); use find_left() or find_right()", EINVAL);
    }
    return v.front();
}

GripperEndpoints find_left() {
    for (const auto& g : scan_all()) {
        if (g.side == Side::Left) return g;
    }
    throw IoError("discovery::find_left: no left-side gripper detected "
                  "(MCU board SN must end in an odd digit)", ENODEV);
}

GripperEndpoints find_right() {
    for (const auto& g : scan_all()) {
        if (g.side == Side::Right) return g;
    }
    throw IoError("discovery::find_right: no right-side gripper detected "
                  "(MCU board SN must end in an even digit)", ENODEV);
}

}  // namespace xense::taccap::discovery
