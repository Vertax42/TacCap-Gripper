// Copyright (c) 2026 XenseRobotics Co., Ltd. — Apache-2.0

#include <taccap/discovery.hpp>
#include <taccap/bus/transport.hpp>
#include <taccap/error.hpp>
#include <taccap/protocol/codec.hpp>
#include <taccap/protocol/commands.hpp>

#include <algorithm>
#include <cctype>
#include <cerrno>
#include <chrono>
#include <filesystem>
#include <optional>
#include <regex>
#include <system_error>

namespace xense::taccap::discovery {

namespace fs = std::filesystem;

namespace {

bool is_digit_(char c) noexcept { return c >= '0' && c <= '9'; }

}  // namespace

std::optional<Side> side_from_serial(const std::string& s) noexcept {
    for (auto it = s.rbegin(); it != s.rend(); ++it) {
        if (is_digit_(*it)) {
            return ((*it - '0') % 2 == 1) ? Side::Left : Side::Right;
        }
    }
    return std::nullopt;
}

ParsedSerial parse_serial(const std::string& s) noexcept {
    ParsedSerial out;
    out.raw = s;

    // Full TacCap grammar:
    //   product (4 alpha + 2 digit) | batch (1 alpha + 2 digit) |
    //   line (1 alpha: Z=R&D, A=production) | sequence (4 digit) |
    //   patch (optional m|s).  e.g.  TCGU01 A24 Z 0001 m   /   GSPS01 A24 Z 0001
    static const std::regex re(
        R"(^([A-Za-z]{4}\d{2})([A-Za-z]\d{2})([A-Za-z])(\d{4})([mMsS]?)$)");
    std::smatch m;
    if (std::regex_match(s, m, re)) {
        out.product  = m[1].str();
        out.batch    = m[2].str();
        out.line     = m[3].str()[0];
        out.sequence = m[4].str();
        const char last = out.sequence.back();
        out.side = ((last - '0') % 2 == 1) ? Side::Left : Side::Right;
        const std::string patch = m[5].str();
        if (!patch.empty()) {
            const char p = static_cast<char>(
                std::tolower(static_cast<unsigned char>(patch[0])));
            out.role = (p == 'm') ? Role::Leader : Role::Follower;
        }
        out.valid = true;
        return out;
    }

    // Not a full TacCap SN (legacy "SN0000002", empty, vendor string, ...).
    // Recover what we can so callers still get a usable side/role: the last
    // digit decides side, a trailing m|s decides role.
    out.side = side_from_serial(s);
    if (!s.empty()) {
        const char last = static_cast<char>(
            std::tolower(static_cast<unsigned char>(s.back())));
        if (last == 'm') out.role = Role::Leader;
        else if (last == 's') out.role = Role::Follower;
    }
    return out;
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

std::vector<GripperEndpoints> scan_all() {
    // One MCU board (CH343 if02 entry) = one gripper unit. The wrist camera
    // and OG visuotactile sensors are owned by an external camera service
    // now, so discovery is MCU-only — no USB-hub grouping needed.
    //
    // The firmware-side SN (Cmd::GetSn) — NOT the CH343 chip SN — decides
    // the gripper's side. The CH343 USB-chip SN is hardware-burned in the
    // WCH chip and bears no relationship to the board build; the firmware
    // SN is what's burned intentionally per-board, so it's the
    // authoritative side identifier.
    std::vector<GripperEndpoints> out;
    for (auto& mcu : scan_mcus()) {
        // Open a transient transport, read firmware SN, decide side.
        // This is a one-shot startup cost — closed before LeaderGripper /
        // FollowerGripper open their own Transport on the same device.
        std::string fw_sn;
        try {
            bus::Transport::Config cfg{};
            cfg.serial.device           = mcu.device;
            cfg.serial.baudrate         = 3'000'000;
            cfg.serial.read_timeout_ms  = 1;
            cfg.serial.write_timeout_ms = 1000;
            cfg.peer                    = protocol::Address::MCU;
            cfg.ack_timeout             = std::chrono::milliseconds(1000);
            cfg.max_retries             = 2;
            bus::Transport t(cfg);
            // If the firmware is still streaming from a previous host
            // process (Ctrl+C / abort, or a sibling publisher we just
            // killed), the rx pipe is full of DATA frames and the GetSn
            // ACK gets buried until they drain. Send a best-effort
            // StopStream first to quiesce the firmware before the real
            // request — same pattern as LeaderGripper::start_streaming.
            try {
                t.send_cmd(protocol::Cmd::StopStream, {},
                           std::chrono::milliseconds(500));
            } catch (...) { /* expected when fw is already idle */ }

            auto ack = t.send_cmd(protocol::Cmd::GetSn, {},
                                  std::chrono::milliseconds(1000));
            if (!ack.is_nack && !ack.data.empty()) {
                fw_sn = protocol::decode_sn(ack.data.data(), ack.data.size());
            }
        } catch (...) {
            // Transport open failed (port already in use by another
            // process, baud rate unsupported, etc.) — fall back to the
            // CH343 chip SN parity so discovery still produces a result,
            // even if the side is unreliable. Caller can still match by
            // mcu_serial / firmware_sn after we report what we have.
        }

        // The firmware SN now carries both side and leader/follower role.
        // parse_serial degrades gracefully (legacy / empty SN → side still
        // recovered from the last digit, role Unknown).
        const auto parsed = parse_serial(fw_sn);

        GripperEndpoints e{};
        e.side        = parsed.side.value_or(mcu.side);
        e.mcu_device  = mcu.device;
        e.mcu_serial  = mcu.serial_number;
        e.firmware_sn = std::move(fw_sn);
        e.role        = parsed.role;
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
                  "(firmware SN must end in an odd digit; CH343 chip SN "
                  "parity is no longer the authoritative source)", ENODEV);
}

GripperEndpoints find_right() {
    for (const auto& g : scan_all()) {
        if (g.side == Side::Right) return g;
    }
    throw IoError("discovery::find_right: no right-side gripper detected "
                  "(firmware SN must end in an even digit; CH343 chip SN "
                  "parity is no longer the authoritative source)", ENODEV);
}

GripperEndpoints find_leader() {
    for (const auto& g : scan_all()) {
        if (g.role == Role::Leader) return g;
    }
    throw IoError("discovery::find_leader: no leader gripper detected "
                  "(firmware SN must end with patch suffix 'm'; legacy or "
                  "unburned SNs report role Unknown)", ENODEV);
}

GripperEndpoints find_follower() {
    for (const auto& g : scan_all()) {
        if (g.role == Role::Follower) return g;
    }
    throw IoError("discovery::find_follower: no follower gripper detected "
                  "(firmware SN must end with patch suffix 's'; legacy or "
                  "unburned SNs report role Unknown)", ENODEV);
}

}  // namespace xense::taccap::discovery
