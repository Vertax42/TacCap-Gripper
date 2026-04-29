// Copyright (c) 2026 XenseRobotics Co., Ltd. — Apache-2.0

#include <taccap/discovery.hpp>
#include <taccap/error.hpp>
#include <taccap/vision.hpp>

#include <algorithm>
#include <cctype>
#include <cerrno>
#include <filesystem>
#include <map>
#include <optional>
#include <regex>
#include <system_error>

namespace xense::taccap::discovery {

namespace fs = std::filesystem;

namespace {

bool is_digit_(char c) noexcept { return c >= '0' && c <= '9'; }

// Resolve `dev_path` (a /dev/* node, possibly a symlink under
// /dev/serial/by-id or /dev/v4l/by-id) to the USB hub key it lives on.
//
// We mirror xense-flare/hub_scanner.py exactly: pick the leftmost
// `<bus>-<ports>` segment from the canonical sysfs path. All four devices
// of one TacCap-Gripper unit (MCU + 2 OG + wrist) sit under the same
// external host port (via internal Corechips hubs), so this prefix is the
// stable per-gripper grouping key.
//
// Returns empty string when the path can't be resolved (device gone, or
// not a USB-attached tty / video node).
std::string find_hub_path(const std::string& dev_path) {
    std::error_code ec;
    fs::path canon = fs::canonical(dev_path, ec);
    if (ec) return {};

    std::string base = canon.filename().string();   // e.g. "ttyACM1" / "video5"

    fs::path sys_class;
    if (base.rfind("tty", 0) == 0) {
        sys_class = fs::path("/sys/class/tty") / base;
    } else if (base.rfind("video", 0) == 0) {
        sys_class = fs::path("/sys/class/video4linux") / base;
    } else {
        return {};
    }

    fs::path sys_dev = fs::canonical(sys_class, ec);
    if (ec) return {};
    const std::string s = sys_dev.string();

    // xense-flare's regex literal — matches "/<N>-<P[.P...]>/" or
    // "/<N>-<P[.P...]>:" (the colon delimits a USB interface, dot delimits
    // sub-ports). re.search picks the leftmost occurrence which is the
    // root port: even devices on different sub-hubs of the same gripper
    // share this prefix.
    static const std::regex re(R"(/(\d+-[\d.]+)(?:/|:))");
    std::smatch m;
    if (std::regex_search(s, m, re)) return m[1].str();
    return {};
}

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
    // Per-hub accumulator. One TacCap-Gripper unit = one external host USB
    // port (the "hub_path" key here, e.g. "1-3"). All MCU + OG + wrist
    // devices belonging to that gripper share the same key.
    struct Group {
        std::string                      hub_path;
        std::optional<McuEndpoint>       mcu;
        std::vector<OgEndpoint>          ogs;
        std::optional<WristCameraEndpoint> wrist;
    };
    std::map<std::string, Group> groups;

    auto gat = [&groups](const std::string& hub) -> Group& {
        auto& g = groups[hub];
        if (g.hub_path.empty()) g.hub_path = hub;
        return g;
    };

    // ---- 1) MCUs (CH343 if02 entries) ----
    for (auto& mcu : scan_mcus()) {
        const std::string hub = find_hub_path(mcu.device);
        if (hub.empty()) continue;
        gat(hub).mcu = std::move(mcu);
    }

    // ---- 2) OG visuotactile sensors (libxense lite enumerate) ----
    for (auto& og : scan_og_sensors()) {
        const std::string hub = find_hub_path(og.video_path);
        if (hub.empty()) continue;
        gat(hub).ogs.push_back(std::move(og));
    }

    // ---- 3) Wrist UVC cameras (XC* / Sunplus) ----
    for (auto& w : scan_wrist_cameras()) {
        const std::string hub = find_hub_path(w.device);
        if (hub.empty()) continue;
        gat(hub).wrist = std::move(w);
    }

    // ---- 4) Build GripperEndpoints, one per hub that has at least an MCU.
    // The MCU's chip SN parity decides the gripper's side. OG sensors are
    // distributed to .tactile_left_serial / .tactile_right_serial by their
    // own SN parity (xense-flare convention, see is_left_device()).
    std::vector<GripperEndpoints> out;
    for (auto& [hub, g] : groups) {
        if (!g.mcu) continue;   // hub without an MCU isn't a complete gripper
        GripperEndpoints e{};
        e.side       = g.mcu->side;
        e.mcu_device = g.mcu->device;
        e.mcu_serial = g.mcu->serial_number;
        if (g.wrist) e.wrist_video = g.wrist->device;
        for (const auto& og : g.ogs) {
            (og.side == Side::Left ? e.tactile_left_serial
                                   : e.tactile_right_serial) = og.serial;
        }
        out.push_back(std::move(e));
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
