#pragma once

#include <chrono>
#include <cstdint>
#include <span>
#include <stop_token>
#include <vector>

#include "tether/ethercat/Slave.hpp"

namespace EtherCAT {
namespace Drives {
namespace ESC211 {

/// Expected CRC value for a config data type, used by CrcVerifier.
struct CrcExpectation {
    uint16_t crcIndex;       // SDO index of the CRC register (e.g. 0xF202)
    uint32_t expectedCrc;    // Expected CRC32 value
    const char* name;        // Data type name for logging (e.g. "FNI")
};

/// Polls device CRC registers until they match expected values or a timeout
/// is reached.  The device updates CRC registers asynchronously after an
/// activate or flash-load command, so polling is more robust than a fixed
/// delay + single read.
///
/// Thin ESC211 layer over EtherCAT::waitForSdoValues (Tether) — the
/// generic poll-until-match engine; this class contributes the
/// CrcExpectation model and CRC naming.
class CrcVerifier {
public:
    CrcVerifier(EtherCAT::Slave& slave,
                std::chrono::milliseconds poll_interval = std::chrono::milliseconds(200),
                const char* tag = "ESC211CrcVerifier");

    /// Set a stop token for cooperative cancellation.  When stop is requested,
    /// waitUntilMatch() returns false immediately (after the current poll).
    void setStopToken(std::stop_token st) { stop_token_ = st; }

    /// Poll all expectations until every CRC matches or the timeout expires.
    /// Returns true if all matched, false on timeout, read error, or cancellation.
    bool waitUntilMatch(std::span<const CrcExpectation> expectations,
                        std::chrono::milliseconds timeout);

    /// Read a single CRC register (0x####:0x00).
    /// Returns 0xFFFFFFFF on read error.
    uint32_t readCrc(uint16_t crcIndex);

private:
    EtherCAT::Slave& slave_;
    std::chrono::milliseconds poll_interval_;
    const char* tag_;
    std::stop_token stop_token_;
};

} // namespace ESC211
} // namespace Drives
} // namespace EtherCAT
