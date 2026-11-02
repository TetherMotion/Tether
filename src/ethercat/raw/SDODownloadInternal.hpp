/**
 * @file SDODownloadInternal.hpp
 * @brief Shared helpers for the SDODownload translation units.
 * @internal Internal header — not installed, not part of the public API.
 */

#pragma once

#include "tether/ethercat/SDODiagnostics.hpp"
#include "tether/ethercat/SDOUpload.hpp"
#include "tether/ethercat/Master.hpp"

#include <cstddef>
#include <cstdint>

namespace EtherCAT {
namespace Raw {

/**
 * @brief Extract the slave's position index from an ADP auto-increment address.
 * @param adp 16-bit auto-increment address from a request datagram.
 * @return Slave position counter value (0 = first slave).
 */
inline uint16_t slaveIndexFromADP(uint16_t adp) {
    return Master::slaveAddressFromADP(adp).slavePosition();
}

/**
 * @brief Build a diagnostic upload functor bound to a specific SDOUpload
 *        instance.
 * @param upload SDO upload engine to forward diagnostic requests to; may be
 *        null.
 * @return UploadFn forwarding every call to `upload->execute`, or a no-op
 *         functor returning false when `upload` is null — callers can pass
 *         the result to SDODiagnostics unconditionally.
 */
inline SDODiagnostics::UploadFn makeUploadDiagFn(SDOUpload* upload) {
    return [upload](Master& m, uint16_t a, uint8_t* c,
                    uint16_t wwa, uint16_t wwl, uint16_t rwa, uint16_t rwl,
                    uint16_t idx, uint8_t s,
                    uint8_t* out, size_t out_cap, size_t* out_len,
                    bool de, unsigned int pim, unsigned int ttm) -> bool {
        if (!upload) return false;
        return upload->execute(m, a, c, wwa, wwl, rwa, rwl, idx, s,
                               out, out_cap, out_len, de, pim, ttm);
    };
}

}
} // namespace EtherCAT
