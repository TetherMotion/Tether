#pragma once

/**
 * @file SmDeliveryProbe.hpp
 * @brief Prove (or disprove) that bytes written into an RxPDO app
 *        buffer actually reach the slave's process-data memory.
 *
 * When forwarded data seems ignored by a slave, the failure splits into
 * two very different classes: the master never delivered the bytes
 * (FMMU/logical mapping problem, wrong PDO offset, rejected mapping)
 * versus the slave received them but its firmware dropped them (e.g.
 * an FSoE stack rejecting a frame).  This probe distinguishes them by
 * reading the slave's Sync Manager region back via `FPRD`
 * (`Master::readRegister`) and comparing it against what the
 * application believes it wrote.
 *
 * Usage:
 * @code
 *   SmDeliveryProbe probe(master);
 *   auto r = probe.compareSm2(slave_index, app_buffer, app_size, 11);
 *   if (r.read_ok && !r.prefix_match) { ... delivery broken ... }
 * @endcode
 *
 * The comparison reports both the full-region match and a "prefix"
 * match so a caller can check the bytes it owns (e.g. the FSoE frame at
 * the head of a combined PDO) separately from trailing bytes owned by
 * other mappings.
 *
 * The probe performs a real EtherCAT register read — call it from a
 * diagnostic loop, serialized with other register/SDO traffic (the
 * caller supplies/owns any needed locking).
 */

#include <cstdint>
#include <cstring>
#include <vector>

#include "tether/ethercat/Master.hpp"
#include "tether/ethercat/PDOManager.hpp"
#include "tether/ethercat/PDOTypes.hpp"

namespace EtherCAT {

class SmDeliveryProbe {
public:
    /// @param master  EtherCAT master (provides readRegister + the
    ///                slave's PDO/SM configuration).
    explicit SmDeliveryProbe(Master& master) : master_(master) {}

    /**
     * @brief Result of one region comparison.
     */
    struct Result {
        bool configured = false;   ///< slave has a valid SM config
        bool read_ok    = false;   ///< FPRD of the SM region succeeded
        bool full_match  = false;  ///< memcmp over min(sm_len, app_len)
        bool prefix_match = false; ///< memcmp over prefix_len bytes
        uint16_t phys_addr = 0;    ///< SM physical start address
        uint16_t length    = 0;    ///< SM region length in bytes
        std::vector<uint8_t> sm_bytes;  ///< raw SM region as read back
    };

    /**
     * @brief Read a slave's Sync Manager `sm_index` region and compare
     *        it against an expected app buffer.
     *
     * @param slave_index EtherCAT slave position.
     * @param sm_index    Sync manager index (2 = process-data outputs/
     *                    RxPDO on typical slaves).
     * @param app_buf     Expected content (the bytes the app wrote to
     *                    the RxPDO app buffer); may be nullptr.
     * @param app_size    Size of `app_buf`.
     * @param prefix_len  Length of the leading region that must match
     *                    (0 = skip prefix check).
     */
    Result compare(uint16_t slave_index, uint8_t sm_index,
                   const uint8_t* app_buf, size_t app_size,
                   size_t prefix_len) {
        Result r;
        auto& mgr = master_.pdoForSlave(slave_index);
        const auto* cfgs = mgr.slaveConfigs();
        if (!cfgs || slave_index >= PDO::kMaxPDOSlaves) return r;
        const auto& sm = cfgs[slave_index].sm[sm_index];
        r.phys_addr = sm.phys_start_addr;
        r.length = sm.length;
        if (sm.phys_start_addr == 0 || sm.length == 0) return r;
        r.configured = true;

        r.sm_bytes.assign(sm.length, 0xEE);
        r.read_ok = master_.readRegister(
            SlaveAddress(slave_index), sm.phys_start_addr,
            r.sm_bytes.data(), sm.length, 200);
        if (!r.read_ok || !app_buf) return r;

        const size_t full_len =
            std::min<size_t>(sm.length, app_size);
        r.full_match =
            std::memcmp(app_buf, r.sm_bytes.data(), full_len) == 0;
        r.prefix_match =
            prefix_len > 0 && prefix_len <= sm.length &&
            prefix_len <= app_size &&
            std::memcmp(app_buf, r.sm_bytes.data(), prefix_len) == 0;
        return r;
    }

    /// Convenience: compare against Sync Manager 2 (process-data
    /// outputs / RxPDO) — the usual "did my frame reach the slave" check.
    Result compareSm2(uint16_t slave_index,
                      const uint8_t* app_buf, size_t app_size,
                      size_t prefix_len) {
        return compare(slave_index, 2, app_buf, app_size, prefix_len);
    }

private:
    Master& master_;
};

} // namespace EtherCAT
