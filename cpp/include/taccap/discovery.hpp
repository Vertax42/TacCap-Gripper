// Copyright (c) 2026 XenseRobotics Co., Ltd. — Apache-2.0
//
// Zero-config gripper discovery.
//
// We follow the xense-flare convention: parse the board / sensor serial
// number's last numeric digit; **odd → left**, **even → right**. No udev
// rules, no system-level setup.
//
//   MCU board (CH343 dual-serial, VID:PID 1a86:55d2):
//       /dev/serial/by-id/usb-1a86_USB_Dual_Serial_<SN>-if02
//                                                  ^^^^^ last digit
//
// Discovery is MCU-only: the wrist UVC camera and the OG visuotactile
// sensors are no longer started by this SDK (an external camera service
// owns them now), so we do not enumerate them here. One MCU board = one
// gripper unit. Today's prototype has a single gripper plugged in at a
// time; the API is designed so that adding a second one (e.g. left +
// right) just requires the user to call `find_left()` / `find_right()`
// instead of `find_one()`.

#pragma once

#include <optional>
#include <string>
#include <vector>

namespace xense::taccap::discovery {

enum class Side : char { Left = 'L', Right = 'R' };
constexpr const char* to_string(Side s) noexcept {
    return s == Side::Left ? "Left" : "Right";
}

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
                                          // e.g. "SN0000002" — drives `side`
};

// Lower-level scan helper (testable / inspectable).
std::vector<McuEndpoint> scan_mcus();

// Enumerate per-gripper bundles. One MCU board = one gripper unit; the
// firmware SN read over the control link decides the side.
std::vector<GripperEndpoints> scan_all();

// Convenience accessors. Throw IoError if the requested gripper isn't found.
GripperEndpoints find_one();    // exactly one gripper plugged in (any side)
GripperEndpoints find_left();   // gripper whose firmware SN ends in an odd digit
GripperEndpoints find_right();  // gripper whose firmware SN ends in an even digit

// Parse the last numeric digit out of a serial-like string. Returns Left
// for odd / Right for even. If the string contains no digits, returns
// std::nullopt so callers can skip / error.
std::optional<Side> side_from_serial(const std::string& s) noexcept;

}  // namespace xense::taccap::discovery
