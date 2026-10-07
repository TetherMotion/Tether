/**
 * @file AS715N.hpp
 * @brief ANCTL AS715N (A6-EC series) EtherCAT drive helpers
 *
 * Manual notes:
 * - 0x203F is a UInt32: high 16 bits = internal code, low 16 bits = external code.
 * - External code uses digit-nibble encoding, e.g. Er74.1 => 0x0741.
 * - "Control in Progress" is at 0x2031 (F31) with subindices for reset/init.
 */

#pragma once

#include <cstddef>
#include <cstdint>
#include "tether/drives/AS715NRegisters.hpp"
#include "tether/drives/AS715NErrors.hpp"
#include "tether/drives/AS715N/AS715NPDO.hpp"

namespace EtherCAT { namespace CoE { class CoEManager; } class CiA402Drive; }

namespace EtherCAT {
namespace Drives {

// Register and error types moved to dedicated headers:
//  - `AS715NRegisters.hpp` (register layout / raw register helpers)
//  - `AS715NErrors.hpp`    (error parsing / human-readable error info)


class AS715NFaultHandler {
public:
    static bool checkFault(EtherCAT::CoE::CoEManager& sdo, uint16_t slave_idx, uint16_t* mfr_error, uint16_t* cia402_error);

    // Fault reset via 0x2031:01 (F31.00).  Per the A6-EC manual the
    // drive ignores the pulse while S-ON (Controlword bit 0) is
    // asserted, so the sequence first clears it via SDO.  When `drive`
    // is given its cached controlword is kept in sync.  Returns true if
    // the fault appears cleared (StatusWord bit 3, else 0x203F).
    static bool resetFault(EtherCAT::CoE::CoEManager& sdo, uint16_t slave_idx,
                           CiA402Drive* drive = nullptr);

    // One-shot startup/idle fault recovery: checkFault(), route DC sync
    // errors (ErC1.x family) to handleNoSyncError(), reset everything
    // else via the F31.00 sequence — including non-recoverable codes,
    // which are attempted anyway and verified.  True when no fault was
    // pending or the reset cleared it.
    static bool resetAllFaults(EtherCAT::CoE::CoEManager& sdo, uint16_t slave_idx);

    static bool handleNoSyncError(EtherCAT::CoE::CoEManager& sdo, uint16_t slave_idx, uint8_t max_attempts = 3);

    // Returns external manufacturer fault code (0x203F low-16).
    static uint16_t readManufacturerFault(EtherCAT::CoE::CoEManager& sdo, uint16_t slave_idx);

    // Returns both internal and external parts of 0x203F.
    static AS715NManufacturerFault203F readManufacturerFaultExtended(EtherCAT::CoE::CoEManager& sdo, uint16_t slave_idx);

    static uint16_t readCiA402Error(EtherCAT::CoE::CoEManager& sdo, uint16_t slave_idx);
};

}  // namespace Drives
}  // namespace EtherCAT
