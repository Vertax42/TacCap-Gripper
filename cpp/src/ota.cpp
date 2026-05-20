// Copyright (c) 2026 XenseRobotics Co., Ltd. — Apache-2.0

#include <taccap/ota.hpp>
#include <taccap/error.hpp>
#include <taccap/log.hpp>
#include <taccap/protocol/codec.hpp>

#include <array>
#include <cerrno>
#include <cstdio>
#include <cstring>
#include <fstream>

namespace xense::taccap {

// ---- CRC32 (ISO-HDLC / zlib.crc32) ----------------------------------------

namespace {

// Build the 256-entry CRC table at constexpr-static init time. Poly
// 0xEDB88320 (= bit-reversed 0x04C11DB7), reflected.
struct Crc32Table {
    uint32_t v[256];
    constexpr Crc32Table() : v{} {
        for (uint32_t i = 0; i < 256; ++i) {
            uint32_t c = i;
            for (int k = 0; k < 8; ++k) {
                c = (c >> 1) ^ (0xEDB88320u & (-static_cast<int32_t>(c & 1)));
            }
            v[i] = c;
        }
    }
};

inline const Crc32Table& crc_table() {
    static constexpr Crc32Table t{};
    return t;
}

}  // namespace

uint32_t crc32_iso_hdlc(const uint8_t* data, std::size_t len) noexcept {
    const auto& T = crc_table();
    uint32_t c = 0xFFFFFFFFu;
    for (std::size_t i = 0; i < len; ++i) {
        c = T.v[(c ^ data[i]) & 0xFFu] ^ (c >> 8);
    }
    return c ^ 0xFFFFFFFFu;
}

// ---- OtaSession -----------------------------------------------------------

OtaSession::OtaSession(bus::Transport& transport) : t_(transport) {}

void OtaSession::start(uint32_t firmware_size, uint32_t firmware_crc32,
                       const TargetVersion& target,
                       std::chrono::milliseconds timeout) {
    protocol::OtaStart pl{};
    pl.firmware_size  = firmware_size;
    pl.firmware_crc32 = firmware_crc32;
    pl.target_major   = target.major;
    pl.target_minor   = target.minor;
    pl.target_patch   = target.patch;
    pl.target_build   = target.build;
    auto ack = t_.send_cmd(protocol::Cmd::OtaStart,
                           protocol::encode(pl), timeout);
    if (ack.is_nack) {
        throw ProtocolError(std::string("OtaSession::start NACK: ") +
                            protocol::to_string(ack.error_code));
    }
}

void OtaSession::write_block(uint32_t offset, const uint8_t* data,
                             uint16_t length,
                             std::chrono::milliseconds timeout) {
    auto wire = protocol::encode_ota_write_block(offset, data, length);
    auto ack = t_.send_cmd(protocol::Cmd::OtaWriteBlock, wire, timeout);
    if (ack.is_nack) {
        throw ProtocolError(std::string("OtaSession::write_block NACK: ") +
                            protocol::to_string(ack.error_code) +
                            " (offset=" + std::to_string(offset) +
                            ", length=" + std::to_string(length) + ")");
    }
}

void OtaSession::verify(std::chrono::milliseconds timeout) {
    auto ack = t_.send_cmd(protocol::Cmd::OtaVerify, {}, timeout);
    if (ack.is_nack) {
        throw ProtocolError(std::string("OtaSession::verify NACK: ") +
                            protocol::to_string(ack.error_code));
    }
}

void OtaSession::apply(std::chrono::milliseconds timeout) {
    auto ack = t_.send_cmd(protocol::Cmd::OtaApply, {}, timeout);
    if (ack.is_nack) {
        throw ProtocolError(std::string("OtaSession::apply NACK: ") +
                            protocol::to_string(ack.error_code));
    }
}

void OtaSession::abort(std::chrono::milliseconds timeout) noexcept {
    // Best-effort: any failure here is recoverable by a fresh
    // start() or by hard-resetting the MCU, so don't propagate.
    try {
        t_.send_cmd(protocol::Cmd::OtaAbort, {}, timeout);
    } catch (...) {}
}

protocol::OtaStatus OtaSession::get_status(std::chrono::milliseconds timeout) {
    auto ack = t_.send_cmd(protocol::Cmd::OtaGetStatus, {}, timeout);
    if (ack.is_nack) {
        throw ProtocolError(std::string("OtaSession::get_status NACK: ") +
                            protocol::to_string(ack.error_code));
    }
    return protocol::decode_ota_status(ack.data.data(), ack.data.size());
}

// ---- High-level update flow -----------------------------------------------

namespace {

std::vector<uint8_t> read_file_bytes(const std::string& path) {
    std::ifstream f(path, std::ios::binary);
    if (!f) {
        throw IoError("OtaSession: cannot open firmware file: " + path,
                      errno);
    }
    f.seekg(0, std::ios::end);
    const auto sz = f.tellg();
    if (sz < 0) {
        throw IoError("OtaSession: tellg failed: " + path, errno);
    }
    f.seekg(0, std::ios::beg);
    std::vector<uint8_t> out(static_cast<std::size_t>(sz));
    if (sz > 0) {
        f.read(reinterpret_cast<char*>(out.data()), sz);
        if (!f) {
            throw IoError("OtaSession: read failed: " + path, errno);
        }
    }
    return out;
}

}  // namespace

void OtaSession::update_from_file(const std::string& path,
                                  const TargetVersion& target,
                                  ProgressCallback on_progress) {
    update_from_bytes(read_file_bytes(path), target, std::move(on_progress));
}

void OtaSession::update_from_bytes(const std::vector<uint8_t>& firmware,
                                   const TargetVersion& target,
                                   ProgressCallback on_progress) {
    const uint32_t total = static_cast<uint32_t>(firmware.size());
    if (total == 0) {
        throw ProtocolError("OtaSession::update_from_bytes: empty firmware");
    }
    if (total > protocol::OTA_MAX_FW_SIZE) {
        throw ProtocolError(
            "OtaSession::update_from_bytes: firmware exceeds single-bank "
            "max (" + std::to_string(total) + " > " +
            std::to_string(protocol::OTA_MAX_FW_SIZE) + ")");
    }

    const uint32_t crc = crc32_iso_hdlc(firmware.data(), firmware.size());
    logger()->info(
        "OTA update: size={}B crc32=0x{:08X} target={}.{}.{}.{}",
        total, crc, target.major, target.minor, target.patch, target.build);

    // 1. OtaStart — abort on failure to leave firmware idle for next try
    try {
        start(total, crc, target);
    } catch (...) {
        abort();
        throw;
    }

    // 2. OtaWriteBlock × ceil(total / OTA_BLOCK_SIZE)
    uint32_t offset = 0;
    while (offset < total) {
        const uint32_t remaining = total - offset;
        const uint16_t this_block = static_cast<uint16_t>(
            std::min<uint32_t>(remaining, protocol::OTA_BLOCK_SIZE));
        try {
            write_block(offset, firmware.data() + offset, this_block);
        } catch (...) {
            abort();
            throw;
        }
        offset += this_block;
        if (on_progress) on_progress(offset, total);
    }

    // 3. Verify (CRC check on firmware side)
    try {
        verify();
    } catch (...) {
        abort();
        throw;
    }

    // 4. Apply (bank swap + reboot)
    logger()->info("OTA update: applying — firmware will reboot");
    apply();
    // No abort() here — firmware is rebooting and the next command
    // on this transport will time out, which is the expected end state.
}

}  // namespace xense::taccap
