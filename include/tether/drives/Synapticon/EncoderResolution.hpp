#pragma once

/**
 * @file EncoderResolution.hpp
 * @brief Read a Synapticon drive's encoder counts-per-revolution via SDO.
 *
 * The CiA 402 standard object 0x608F:1 ("Position encoder resolution —
 * encoder increments") is the canonical source, but it is absent from
 * the SOMANET object dictionary on several firmware lines.  The
 * fallback chain therefore tries, in order:
 *
 *   1. 0x608F:1 — CiA 402 encoder increments
 *   2. 0x2110:3 — Synapticon "Encoder 1 configuration → Resolution"
 *   3. 0x2112:3 — Synapticon "Encoder 2 configuration → Resolution"
 *
 * Returns the first non-zero value; callers decide their own default
 * (`EtherCAT::Drives::Synapticon::kDefaultEncoderResolution` = 524288 is
 * the SOMANET-typical value) when every read fails.
 *
 * These are plain SDO uploads — call while no PDO exchange or only
 * serialized SDO traffic is running on the slave's mailbox.
 */

#include <cstdint>
#include <expected>

#include "tether/drives/Synapticon.hpp"
#include "tether/ethercat/CoEManager.hpp"
#include "tether/ethercat/CoETypes.hpp"

namespace EtherCAT::Drives::Synapticon {

/**
 * @brief Result of an encoder-resolution read: the winning object and
 *        its value.
 */
struct EncoderResolution {
    uint32_t counts_per_rev = 0;  ///< encoder counts per shaft revolution
    uint16_t index = 0;           ///< CoE index that produced the value
    uint8_t  subindex = 0;        ///< CoE subindex that produced the value
};

/**
 * @brief Probe the encoder-resolution fallback chain on a drive.
 *
 * @param coe   CoE manager bound to the drive's slave.
 * @param opts  SDO transaction options (timeout etc.).
 * @return The first non-zero resolution with its source object, or the
 *         last read's `CoEError` when every candidate failed/returned 0.
 */
inline std::expected<EncoderResolution, CoE::CoEError>
readEncoderResolution(CoE::CoEManager& coe,
                      CoE::CoETransactionOptions opts = {}) {
    static constexpr struct { uint16_t idx; uint8_t sub; } kChain[] = {
        {0x608F, 1},  // CiA 402 encoder increments
        {0x2110, 3},  // Synapticon encoder 1 resolution
        {0x2112, 3},  // Synapticon encoder 2 resolution
    };
    CoE::CoEError last{CoE::CoEErrorCode::InternalError};
    for (const auto& c : kChain) {
        auto res = coe.readU32(c.idx, c.sub, opts);
        if (res && *res > 0) {
            return EncoderResolution{*res, c.idx, c.sub};
        }
        last = res ? CoE::CoEError{CoE::CoEErrorCode::InternalError}
                   : res.error();
    }
    return std::unexpected(last);
}

} // namespace EtherCAT::Drives::Synapticon
