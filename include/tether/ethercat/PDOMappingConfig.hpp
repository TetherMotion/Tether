// SPDX-License-Identifier: MIT
/**
 * @file PDOMappingConfig.hpp
 * @brief Multi-PDO mapping configuration types for sync managers
 *
 * @details
 * This header provides types for assigning multiple PDO mappings to a single
 * sync manager.  In standard EtherCAT, each sync manager can have multiple
 * PDO mapping objects assigned to it via the CiA 301 PDO Assignment objects
 * (0x1C10–0x1C2F).  The PDOs are laid out contiguously in the SM's physical
 * address range.
 *
 * ## Usage
 *
 * @code
 *   using namespace EtherCAT::PDO;
 *
 *   // SM2 with two RxPDOs: 0x1600 (8 bytes) + 0x1601 (4 bytes) = 12 bytes total
 *   auto sm2 = MultiPDOSyncManagerConfig::process_output(0x1800, {
 *       {0x1600, 8},
 *       {0x1601, 4},
 *   });
 *
 *   // SM3 with one TxPDO: 0x1A00 (16 bytes)
 *   auto sm3 = MultiPDOSyncManagerConfig::process_input(0x1C00, {
 *       {0x1A00, 16},
 *   });
 * @endcode
 */

#pragma once

#include <cstdint>
#include <initializer_list>
#include <vector>
#include <bit>

#include "tether/ethercat/SMRegisters.hpp"
#include "tether/ethercat/PDOManager.hpp"  // for SyncManagerType, SM_CTRL_* constants

namespace EtherCAT {
namespace PDO {

// ---------------------------------------------------------------------------
// Constants
// ---------------------------------------------------------------------------

/// Maximum number of PDO mappings per sync manager.
/// 16 is well above any real-world slave (typically 1-4 PDOs per SM).
constexpr size_t kMaxPDOsPerSM = 16;

// ---------------------------------------------------------------------------
// PDOMappingRegion
// ---------------------------------------------------------------------------

/**
 * @brief A single PDO mapping assigned to a sync manager.
 *
 * Each region describes one PDO mapping object (e.g. 0x1600, 0x1A01) and
 * its total byte size.  The PDOs are laid out contiguously within the SM's
 * physical address range, in the order they are added.
 */
struct PDOMappingRegion {
    uint16_t pdo_index{0};      ///< OD index (0x1600-0x17FF RxPDO, 0x1A00-0x1BFF TxPDO)
    uint16_t size_bytes{0};     ///< Total size of this PDO mapping in bytes

    PDOMappingRegion() = default;
    PDOMappingRegion(uint16_t index, uint16_t size)
        : pdo_index(index), size_bytes(size) {}
};

// ---------------------------------------------------------------------------
// MultiPDOSyncManagerConfig
// ---------------------------------------------------------------------------

/**
 * @brief Sync manager configuration with multiple PDO mappings.
 *
 * Supports SM0-SM7; defaults to SM2 (ProcessOutput) and SM3 (ProcessInput).
 * Using SM0/SM1 (mailbox SMs) requires explicit sm_index setting and will
 * log a warning at runtime, as those are typically reserved for mailbox
 * communication.
 *
 * The SM's total length is the sum of all PDO mapping sizes.  The physical
 * start address is the base of the SM's address range in ESC memory; PDOs
 * are laid out contiguously starting from this address.
 */
struct MultiPDOSyncManagerConfig {
    uint8_t  sm_index{2};             ///< SM index (0-7); default SM2
    uint16_t phys_start_addr{0};      ///< Physical start address in ESC memory
    EtherCAT::SyncManager::SMControlReg control{};
    bool     enable{true};
    SyncManagerType type{SyncManagerType::ProcessOutput};
    std::vector<PDOMappingRegion> pdo_mappings;

    /// Sum of all PDO mapping sizes — the total SM length.
    uint16_t totalLength() const {
        uint16_t total = 0;
        for (const auto& p : pdo_mappings) {
            total += p.size_bytes;
        }
        return total;
    }

    /// Add a PDO mapping to this SM.
    /// @param pdo_index  OD index of the PDO mapping object
    /// @param size_bytes Total size of this PDO in bytes
    void addPDOMapping(uint16_t pdo_index, uint16_t size_bytes) {
        pdo_mappings.push_back(PDOMappingRegion(pdo_index, size_bytes));
    }

    /// Create a ProcessOutput (RxPDO) SM config with multiple PDO mappings.
    /// Defaults to SM2 with standard buffered-write control register.
    static MultiPDOSyncManagerConfig process_output(
        uint16_t addr,
        std::initializer_list<PDOMappingRegion> pdos) {
        MultiPDOSyncManagerConfig cfg;
        cfg.sm_index = 2;
        cfg.phys_start_addr = addr;
        cfg.type = SyncManagerType::ProcessOutput;
        cfg.control = std::bit_cast<EtherCAT::SyncManager::SMControlReg>(
            static_cast<uint8_t>(SM_CTRL_MODE_BUFFERED | SM_CTRL_DIR_WRITE | SM_CTRL_REPEAT_REQ));
        cfg.pdo_mappings = pdos;
        return cfg;
    }

    /// Create a ProcessInput (TxPDO) SM config with multiple PDO mappings.
    /// Defaults to SM3 with standard buffered-read control register.
    static MultiPDOSyncManagerConfig process_input(
        uint16_t addr,
        std::initializer_list<PDOMappingRegion> pdos) {
        MultiPDOSyncManagerConfig cfg;
        cfg.sm_index = 3;
        cfg.phys_start_addr = addr;
        cfg.type = SyncManagerType::ProcessInput;
        cfg.control = std::bit_cast<EtherCAT::SyncManager::SMControlReg>(
            static_cast<uint8_t>(SM_CTRL_MODE_BUFFERED | SM_CTRL_DIR_READ | SM_CTRL_REPEAT_REQ));
        cfg.pdo_mappings = pdos;
        return cfg;
    }

    /// Convert to a legacy SyncManagerConfig (single PDO, total length).
    SyncManagerConfig toLegacyConfig() const {
        SyncManagerConfig leg;
        leg.phys_start_addr = phys_start_addr;
        leg.length = totalLength();
        leg.control = control;
        leg.enable = enable;
        leg.type = type;
        return leg;
    }
};

} // namespace PDO
} // namespace EtherCAT
