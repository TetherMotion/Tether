#pragma once

/**
 * @file DriveSafetyPoll.hpp
 * @brief Lightweight live snapshot of a Synapticon drive's safety and
 *        CiA 402 status while FSoE process data is flowing.
 *
 * Reads the four objects that discriminate "FSoE released but the drive
 * is still safety-gated" (e.g. hardware safe inputs holding STO) from a
 * CiA 402 inhibit or fault:
 *
 *   - 0x6621:1/2 — Safety statusword (STO, SBC, SS1, SS2, SOS, SLS bits)
 *   - 0x2611:1/2 — Safety module input diagnostics (physical safe-input
 *                  levels — the drive's *local* hardware safety demand,
 *                  independent of anything the FSoE master sends)
 *   - 0x6041     — CiA 402 statusword (power-stage state machine)
 *   - 0x603F     — CiA 402 error code
 *
 * Unlike the heavier `runSafetyDiagnostics()` scan (intended for init /
 * recovery), this is a cheap ~6-SDO poll suitable for a periodic
 * monitor loop.  The caller decides when and under which mailbox lock
 * to call `read()`; `key()` packs the values into a single integer for
 * change detection.
 */

#include <cstdint>

#include "tether/drives/Synapticon/SafetyDiagnostics.hpp"
#include "tether/ethercat/Slave.hpp"

namespace EtherCAT::Drives::Synapticon {

/**
 * @brief One drive-safety poll sample.
 */
struct DriveSafetySnapshot {
    uint8_t  safety_sw1 = 0;   ///< 0x6621:1 — safety statusword byte 1
    uint8_t  safety_sw2 = 0;   ///< 0x6621:2 — safety statusword byte 2
    uint8_t  safe_in1   = 0;   ///< 0x2611:1 — safe input 1 level
    uint8_t  safe_in2   = 0;   ///< 0x2611:2 — safe input 2 level
    uint16_t statusword = 0;   ///< 0x6041   — CiA 402 statusword
    uint16_t error_code = 0;   ///< 0x603F   — CiA 402 error code

    /// Pack all fields into one integer — `a.key() != b.key()` iff any
    /// sampled value changed (for last-seen change detection).
    uint64_t key() const {
        return (static_cast<uint64_t>(safety_sw1) << 40) |
               (static_cast<uint64_t>(safety_sw2) << 32) |
               (static_cast<uint64_t>(safe_in1)   << 24) |
               (static_cast<uint64_t>(safe_in2)   << 16) |
               (static_cast<uint64_t>(statusword) << 0)  |
               (static_cast<uint64_t>(error_code) << 48);
    }

    /// True when a hardware safe input is low — the drive's local SMM
    /// demands a stop function regardless of the FSoE master's commands.
    bool safeInputsLow() const { return safe_in1 == 0 || safe_in2 == 0; }
};

/**
 * @brief Read one `DriveSafetySnapshot` from a drive via SDO.
 *
 * Plain `sdoReadU8/U16` calls — the caller serializes mailbox access
 * (the caller's mutex must already be held when SDO traffic is shared
 * with other threads).
 *
 * @param slave  The drive's slave handle.
 * @param out    Snapshot to fill; fields keep their previous values for
 *               objects whose read fails, so partial data is still
 *               usable (treat per-call as best effort).
 * @return true when all six reads succeeded.
 */
inline bool readDriveSafety(EtherCAT::Slave& slave,
                            DriveSafetySnapshot& out) {
    bool ok = true;
    ok &= slave.sdoReadU8(kSafetyStatuswordIndex, 1, out.safety_sw1) ==
          SlaveError::Ok;
    ok &= slave.sdoReadU8(kSafetyStatuswordIndex, 2, out.safety_sw2) ==
          SlaveError::Ok;
    ok &= slave.sdoReadU8(kSafetyModuleDiagnosticsIndex, 1, out.safe_in1) ==
          SlaveError::Ok;
    ok &= slave.sdoReadU8(kSafetyModuleDiagnosticsIndex, 2, out.safe_in2) ==
          SlaveError::Ok;
    ok &= slave.sdoReadU16(0x6041, 0, out.statusword) == SlaveError::Ok;
    ok &= slave.sdoReadU16(0x603F, 0, out.error_code) == SlaveError::Ok;
    return ok;
}

} // namespace EtherCAT::Drives::Synapticon
