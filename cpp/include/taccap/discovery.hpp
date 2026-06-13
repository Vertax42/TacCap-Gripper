// Copyright (c) 2026 XenseRobotics Co., Ltd. — Apache-2.0
//
// Zero-config gripper discovery.
//
// Side (Left/Right) comes from the firmware-burned SN's sequence number:
// **last digit odd → left, even → right**. No udev rules, no system setup.
//
//   MCU board (CH343 dual-serial, VID:PID 1a86:55d2):
//       /dev/serial/by-id/usb-1a86_USB_Dual_Serial_<SN>-if02
//
// TacCap SN grammar (firmware-burned, read via Cmd::GetSn):
//
//     TCGU01 A24 Z 0001 m      gripper, e.g. TCGU01A24Z0001m
//     └─┬──┘ └┬┘ │ └┬─┘ │
//    product batch│  seq patch        product : TCGU01 gripper / GSPS01 sensor
//                 line                line    : Z = R&D/test, A = production
//                                     seq     : last digit odd→Left, even→Right
//                                     patch   : m = Master/leader, s = Slave/
//                                               follower (grippers only; sensors
//                                               such as GSPS01A24Z0001 have none)
//
// So the SN now encodes both the side AND the leader/follower role — see
// parse_serial(). Discovery is MCU-only: the wrist UVC camera and the
// visuotactile sensors are owned by an external camera service and are not
// enumerated here. One MCU board = one gripper unit; use find_left() /
// find_right() (by side) or find_leader() / find_follower() (by role) when
// more than one is plugged in.

#pragma once

#include <optional>
#include <string>
#include <vector>

namespace xense::taccap::discovery {

enum class Side : char { Left = 'L', Right = 'R' };
constexpr const char* to_string(Side s) noexcept {
    return s == Side::Left ? "Left" : "Right";
}

// Leader/follower role, taken from the SN patch suffix (m = Master/leader,
// s = Slave/follower). Unknown for sensors (no suffix) or SNs that don't
// parse — the SDK can't tell leader from follower hardware in that case.
enum class Role : char { Leader = 'm', Follower = 's', Unknown = '?' };
constexpr const char* to_string(Role r) noexcept {
    return r == Role::Leader   ? "Leader"
         : r == Role::Follower ? "Follower"
                               : "Unknown";
}

// Structured view of a TacCap SN (e.g. "TCGU01A24Z0001m" / "GSPS01A24Z0001").
// `valid` is true only when the full grammar matched; even when it didn't,
// `side` and `role` are filled best-effort (last digit / trailing m|s) so
// callers still get something usable from a partially-conforming SN.
struct ParsedSerial {
    std::string         raw;                 // the input string, verbatim
    std::string         product;             // "TCGU01" (gripper) / "GSPS01" (sensor)
    std::string         batch;               // "A24"
    char                line = '?';          // 'Z' = R&D/test, 'A' = production
    std::string         sequence;            // "0001"
    std::optional<Side> side;                // sequence last digit: odd→Left, even→Right
    Role                role = Role::Unknown;// m→Leader, s→Follower; Unknown otherwise
    bool                valid = false;       // matched the full TacCap SN grammar
};

// Parse a TacCap SN. Never throws; falls back to best-effort side/role when
// the string doesn't match the full grammar (e.g. legacy or empty SNs).
ParsedSerial parse_serial(const std::string& s) noexcept;

// One MCU board entry.
struct McuEndpoint {
    std::string device;          // e.g. /dev/serial/by-id/...-if02
    std::string serial_number;   // e.g. "5C2C247728"
    Side        side;            // from last digit of serial_number
};

// One complete gripper unit, identified by its MCU board.
// `side` of the gripper is taken from the firmware-side SN (read at
// discovery time via Cmd::GetSn). The CH343 USB-chip SN that lives in
// `mcu_serial` is preserved for identification / debugging but does NOT
// drive the Left/Right decision — the firmware SN is what xense-flare
// uses too, and the CH343 chip SN is hardware-burned independently of
// the board so they don't have to share parity.
struct GripperEndpoints {
    Side        side;
    std::string mcu_device;
    std::string mcu_serial;               // CH343 chip SN, e.g. "5C2C247728"
    std::string firmware_sn;              // STM32 flash SN read via Cmd::GetSn,
                                          // e.g. "TCGU01A24Z0002m" — drives `side`
    Role        role = Role::Unknown;     // leader/follower from the SN patch
                                          // suffix (m/s); Unknown if SN is legacy
                                          // / empty / unparsable
};

// Lower-level scan helper (testable / inspectable).
std::vector<McuEndpoint> scan_mcus();

// Enumerate per-gripper bundles. One MCU board = one gripper unit; the
// firmware SN read over the control link decides the side.
std::vector<GripperEndpoints> scan_all();

// Convenience accessors. Throw IoError if the requested gripper isn't found.
GripperEndpoints find_one();       // exactly one gripper plugged in (any side)
GripperEndpoints find_left();      // gripper whose firmware SN ends in an odd digit
GripperEndpoints find_right();     // gripper whose firmware SN ends in an even digit
GripperEndpoints find_leader();    // gripper whose SN patch suffix is 'm' (Master)
GripperEndpoints find_follower();  // gripper whose SN patch suffix is 's' (Slave)

// Parse the last numeric digit out of a serial-like string. Returns Left
// for odd / Right for even. If the string contains no digits, returns
// std::nullopt so callers can skip / error. (parse_serial() supersedes this
// for full TacCap SNs; kept for legacy/loose inputs.)
std::optional<Side> side_from_serial(const std::string& s) noexcept;

}  // namespace xense::taccap::discovery
