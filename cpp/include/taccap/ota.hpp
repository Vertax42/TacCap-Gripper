// Copyright (c) 2026 XenseRobotics Co., Ltd. — Apache-2.0
//
// OTA (Over-The-Air) firmware update — V1.3 protocol surface.
//
// The TC-GU-01 MCU's firmware can be replaced over the same wire we
// already use for sensor I/O, no SWD probe required. The protocol
// (`Cmd::Ota*`, 0x70-0x75) is a state machine:
//
//   Idle → OtaStart → Started → (OtaWriteBlock×N) → Receiving
//        → OtaVerify → Verified → OtaApply → Applying → MCU REBOOT
//
//   At any error point: → Error (must OtaAbort to clear)
//
// Wire-format note: OtaWriteBlock carries a 6-byte header (offset,
// length) followed by `length` bytes of firmware data, up to
// `protocol::OTA_BLOCK_SIZE` per frame. CRC32 of the whole image must
// match firmware's expectation (ISO-HDLC / zlib.crc32 polynomial, init
// 0xFFFFFFFF, xorout 0xFFFFFFFF).
//
// Two API levels:
//
//   - High-level `update_from_file()` / `update_from_bytes()`: hand a
//     firmware blob + target version, get a single blocking call that
//     runs the whole sequence with an optional progress callback. Most
//     consumers should use this.
//
//   - Low-level individual `start()` / `write_block()` / `verify()` /
//     `apply()` / `abort()` / `get_status()`: for tests, alternative
//     workflows, or surgical recovery (e.g. probing OTA state with
//     `get_status()` to see why a previous attempt left the MCU stuck).

#pragma once

#include <taccap/bus/transport.hpp>
#include <taccap/protocol/payloads.hpp>

#include <chrono>
#include <cstdint>
#include <functional>
#include <string>
#include <vector>

namespace xense::taccap {

// Compute CRC32 with the ISO-HDLC parameters firmware uses (matches
// Python `zlib.crc32` and the firmware's `crc32` driver). Constexpr-
// initialised table, no allocations.
uint32_t crc32_iso_hdlc(const uint8_t* data, std::size_t len) noexcept;

class OtaSession {
public:
    // Per-block progress callback. Called after each successful
    // OtaWriteBlock ACK. `bytes_written` is the cumulative byte count
    // (host-side accounting); `total_bytes` is the firmware blob size.
    using ProgressCallback =
        std::function<void(uint32_t bytes_written, uint32_t total_bytes)>;

    // Target firmware version embedded in OtaStart. Firmware uses
    // this both for the upgrade record and to refuse mismatched bank
    // metadata at apply time.
    struct TargetVersion {
        uint8_t major;
        uint8_t minor;
        uint8_t patch;
        uint8_t build;
    };

    explicit OtaSession(bus::Transport& transport);

    // ---- High-level one-shot updates --------------------------------

    // Read `path` from disk, compute CRC32, then run the full
    // start→write→verify→apply sequence. Blocks until apply ACK is
    // received (after which the firmware reboots and any subsequent
    // commands on this Transport will time out — that's expected).
    void update_from_file(const std::string& path,
                          const TargetVersion& target,
                          ProgressCallback on_progress = {});

    // Same, but firmware blob already in memory. Convenient for tests
    // and for firmware-in-resource-file scenarios.
    void update_from_bytes(const std::vector<uint8_t>& firmware,
                           const TargetVersion& target,
                           ProgressCallback on_progress = {});

    // ---- Low-level individual commands ------------------------------
    //
    // Each method sends one Cmd::Ota* and waits for ACK. Throws
    // ProtocolError on NACK, IoError/TimeoutError on transport
    // failure. Designed for unit tests and recovery flows; high-level
    // `update_*()` is what 95% of callers want.

    void start(uint32_t firmware_size, uint32_t firmware_crc32,
               const TargetVersion& target,
               std::chrono::milliseconds timeout = std::chrono::milliseconds{1000});

    void write_block(uint32_t offset, const uint8_t* data, uint16_t length,
                     std::chrono::milliseconds timeout = std::chrono::milliseconds{500});

    void verify(std::chrono::milliseconds timeout = std::chrono::milliseconds{2000});

    // apply() sends OtaApply and waits for ACK; firmware will reboot
    // shortly after. The Transport this OtaSession was built on will
    // start timing out on subsequent commands — caller is expected to
    // tear it down and re-open after a brief wait.
    void apply(std::chrono::milliseconds timeout = std::chrono::milliseconds{2000});

    void abort(std::chrono::milliseconds timeout = std::chrono::milliseconds{500}) noexcept;

    protocol::OtaStatus get_status(std::chrono::milliseconds timeout = std::chrono::milliseconds{500});

private:
    bus::Transport& t_;
};

}  // namespace xense::taccap
