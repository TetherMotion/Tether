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

inline uint16_t slaveIndexFromADP(uint16_t adp) {
    return Master::slaveAddressFromADP(adp).slavePosition();
}

// Build a diagnostic upload functor bound to a specific SDOUpload instance.
// Returns a no-op functor (returns false) when @p upload is null, so callers
// can unconditionally pass the result to SDODiagnostics without null checks.
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
