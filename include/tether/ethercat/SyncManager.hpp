// SPDX-License-Identifier: MIT
#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "tether/ethercat/EtherCATPDO.hpp"
#include "tether/ethercat/EtherCATTypes.hpp"

/**
 * @file SyncManager.hpp
 * @brief EtherCAT Sync Manager constants, object dictionary definitions, and the
 *        SyncManagerAccessor class for per-SM configuration, validation, and diagnostics.
 *
 * ## Namespace layout
 *
 * - `EtherCAT::SyncManager`   – primary home for all SM constants and helper functions
 * - `EtherCAT::SyncManagerAccessor` – lightweight accessor returned by `slave.sm(n)`
 * - `CiA301`                  – backward-compatible shims (aliases into EtherCAT::SyncManager)
 *
 * ## Object dictionary indices (CiA 301)
 *
 * | Index range   | Description                         |
 * |---------------|-------------------------------------|
 * | 0x1C00        | SM Communication Types array        |
 * | 0x1C10–0x1C2F | SM PDO Assignment arrays (SM0–SM31) |
 * | 0x1C32–0x1C33 | SM Synchronization parameters       |
 *
 * ## ESC physical register layout
 *
 * Each SM occupies 8 bytes in the ESC register space:
 *
 * | Byte offset | Field                |
 * |-------------|----------------------|
 * | 0–1         | Physical start addr  |
 * | 2–3         | Length               |
 * | 4           | Control register     |
 * | 5           | Status register      |
 * | 6           | Activate register    |
 * | 7           | PDI control register |
 *
 * SM0 starts at 0x0800, SM1 at 0x0808, SM2 at 0x0810, SM3 at 0x0818, …
 */

// ============================================================================
// Forward declarations
// ============================================================================

namespace EtherCAT {
    class Slave;
    class Master;
    enum class SlaveError : uint8_t;
}

// ============================================================================
// EtherCAT::SyncManager namespace — primary constant definitions
// ============================================================================

namespace EtherCAT { namespace SyncManager {

// ---------------------------------------------------------------------------
// ESC physical register constants
// ---------------------------------------------------------------------------

/// Base address of SM0 in the ESC register space.
constexpr uint16_t kRegisterBase   = 0x0800;
/// Number of bytes each SM occupies in the ESC register space.
constexpr uint8_t  kRegisterStride = 8;
/// Maximum number of Sync Managers defined by the EtherCAT standard.
constexpr uint8_t  kMaxCount       = 32;

/// Byte offset within an SM register block: physical start address (16-bit LE).
constexpr uint8_t kRegOffsetStartAddr = 0;
/// Byte offset within an SM register block: length (16-bit LE).
constexpr uint8_t kRegOffsetLength    = 2;
/// Byte offset within an SM register block: control register.
constexpr uint8_t kRegOffsetControl   = 4;
/// Byte offset within an SM register block: status register.
constexpr uint8_t kRegOffsetStatus    = 5;
/// Byte offset within an SM register block: activate register.
constexpr uint8_t kRegOffsetActivate  = 6;
/// Byte offset within an SM register block: PDI control register.
constexpr uint8_t kRegOffsetPDICtrl   = 7;

/// SM Watchdog status register address (process-data WD expired flag).
constexpr uint16_t kWatchdogStatusReg = 0x0440;

/// Return the physical ESC register base address for SM @p smIndex.
constexpr uint16_t registerBase(uint8_t smIndex) {
    return static_cast<uint16_t>(kRegisterBase + smIndex * kRegisterStride);
}

// ---------------------------------------------------------------------------
// CiA 301 object dictionary indices
// ---------------------------------------------------------------------------

/// 0x1C00 – SM Communication Types array (subindex n = SM(n-1) type).
constexpr uint16_t kCommTypeIndex   = 0x1C00;
/// 0x1C10 – base of SM PDO Assignment array indices (0x1C10 = SM0, …, 0x1C2F = SM31).
constexpr uint16_t kPDOAssignBase   = 0x1C10;
/// 0x1C32 – SM2 Synchronization parameter object.
constexpr uint16_t kSyncParamSM2    = 0x1C32;
/// 0x1C33 – SM3 Synchronization parameter object.
constexpr uint16_t kSyncParamSM3    = 0x1C33;

/// Return the PDO Assignment OD index for SM @p smIndex (0x1C10 + smIndex).
constexpr uint16_t pdoAssignIndex(uint8_t smIndex) {
    return static_cast<uint16_t>(kPDOAssignBase + smIndex);
}

/// Return the SM Synchronization OD index for SM @p smIndex (valid for SM2/SM3 → 0x1C32/0x1C33).
constexpr uint16_t syncParamIndex(uint8_t smIndex) {
    return static_cast<uint16_t>(0x1C30 + smIndex);
}

// ---------------------------------------------------------------------------
// SM Communication Type values (0x1C00 : n)
// ---------------------------------------------------------------------------

/**
 * @brief SM Communication Type byte values read from/written to 0x1C00 : n.
 */
namespace CommType {
    constexpr uint8_t NotUsed           = 0x00; ///< Not used / disabled
    constexpr uint8_t MailboxReceive    = 0x01; ///< Mailbox receive (Master → Slave)
    constexpr uint8_t MailboxSend       = 0x02; ///< Mailbox send (Slave → Master)
    constexpr uint8_t ProcessDataOutput = 0x03; ///< Process data output / RxPDO (Master → Slave)
    constexpr uint8_t ProcessDataInput  = 0x04; ///< Process data input / TxPDO (Slave → Master)
    // Shorter aliases
    constexpr uint8_t ProcessOutput = ProcessDataOutput;
    constexpr uint8_t ProcessInput  = ProcessDataInput;
} // namespace CommType

// ---------------------------------------------------------------------------
// SM Synchronization subindices (0x1C32, 0x1C33)
// ---------------------------------------------------------------------------

/**
 * @brief Subindex definitions for SM Synchronization objects (0x1C32 / 0x1C33).
 */
namespace SyncSub {
    constexpr uint8_t SyncMode             = 0x01; ///< Sync mode (0=free-run, 1=SM, 2=DC SYNC0, 3=DC SYNC1)
    constexpr uint8_t CycleTime           = 0x02; ///< Cycle time in ns
    constexpr uint8_t ShiftTime           = 0x03; ///< Output / input shift time
    constexpr uint8_t SupportedSyncTypes  = 0x04; ///< Supported sync types bitmask
    constexpr uint8_t MinimumCycleTime    = 0x05; ///< Minimum cycle time
    constexpr uint8_t CalcAndCopyTime     = 0x06; ///< Calculation and copy time
    constexpr uint8_t Reserved7           = 0x07;
    constexpr uint8_t CommandRegister     = 0x08; ///< Command register
    constexpr uint8_t DelayTime           = 0x09; ///< Delay time
    constexpr uint8_t Sync0CycleTime      = 0x0A; ///< Sync0 cycle time
    constexpr uint8_t SMEventMissedCounter = 0x0B; ///< SM event missed counter
    constexpr uint8_t CycleTimeTooSmall   = 0x0C; ///< Cycle time too small flag
    constexpr uint8_t ShiftTimeTooShort   = 0x0D; ///< Shift time too short flag
    constexpr uint8_t SyncError           = 0x20; ///< Sync error flag
} // namespace SyncSub

// ---------------------------------------------------------------------------
// PDO mapping entry helpers
// ---------------------------------------------------------------------------

/**
 * @brief Pack a PDO mapping entry (as stored in 0x1600..0x1BFF / 0x1A00..0x1FFF).
 * @param index    Object dictionary index of the mapped object
 * @param subindex Subindex of the mapped object
 * @param bits     Bit length of the mapped object
 */
constexpr uint32_t makeMappingEntry(uint16_t index, uint8_t subindex, uint8_t bits) {
    return (static_cast<uint32_t>(index) << 16) |
           (static_cast<uint32_t>(subindex) << 8) |
            static_cast<uint32_t>(bits);
}

/// Extract the object index from a PDO mapping entry.
constexpr uint16_t mappingIndex(uint32_t entry) {
    return static_cast<uint16_t>((entry >> 16) & 0xFFFFU);
}

/// Extract the subindex from a PDO mapping entry.
constexpr uint8_t mappingSubindex(uint32_t entry) {
    return static_cast<uint8_t>((entry >> 8) & 0xFFU);
}

/// Extract the bit length from a PDO mapping entry.
constexpr uint8_t mappingBits(uint32_t entry) {
    return static_cast<uint8_t>(entry & 0xFFU);
}

}} // namespace EtherCAT::SyncManager

// ============================================================================
// CiA301 backward-compatible shims
// ============================================================================

/**
 * @brief Backward-compatible aliases for EtherCAT::SyncManager constants.
 *
 * All new code should use the `EtherCAT::SyncManager` namespace directly.
 * These aliases exist only to avoid breaking existing callers.
 */
namespace CiA301 {

// OD indices
constexpr uint16_t SyncManagerCommType   = EtherCAT::SyncManager::kCommTypeIndex;
constexpr uint16_t SyncManager0PDOAssign = EtherCAT::SyncManager::pdoAssignIndex(0);
constexpr uint16_t SyncManager1PDOAssign = EtherCAT::SyncManager::pdoAssignIndex(1);
constexpr uint16_t SyncManager2PDOAssign = EtherCAT::SyncManager::pdoAssignIndex(2);
constexpr uint16_t SyncManager3PDOAssign = EtherCAT::SyncManager::pdoAssignIndex(3);
constexpr uint16_t SyncManager4PDOAssign = EtherCAT::SyncManager::pdoAssignIndex(4);
constexpr uint16_t SyncManager5PDOAssign = EtherCAT::SyncManager::pdoAssignIndex(5);
constexpr uint16_t SyncManager6PDOAssign = EtherCAT::SyncManager::pdoAssignIndex(6);
constexpr uint16_t SyncManager7PDOAssign = EtherCAT::SyncManager::pdoAssignIndex(7);
constexpr uint16_t SM2Synchronization    = EtherCAT::SyncManager::kSyncParamSM2;
constexpr uint16_t SM3Synchronization    = EtherCAT::SyncManager::kSyncParamSM3;

// SM Communication Type values (namespace alias)
namespace SyncManagerType = EtherCAT::SyncManager::CommType;

// SM Synchronization subindex aliases (namespace alias)
namespace SMSyncSub = EtherCAT::SyncManager::SyncSub;

// PDO mapping helpers (backward-compatible function names)
constexpr uint32_t PDO_MAPPING_ENTRY(uint16_t index, uint8_t subindex, uint8_t bits) {
    return EtherCAT::SyncManager::makeMappingEntry(index, subindex, bits);
}
constexpr uint16_t PDO_MAPPING_INDEX(uint32_t entry) {
    return EtherCAT::SyncManager::mappingIndex(entry);
}
constexpr uint8_t PDO_MAPPING_SUBINDEX(uint32_t entry) {
    return EtherCAT::SyncManager::mappingSubindex(entry);
}
constexpr uint8_t PDO_MAPPING_BITS(uint32_t entry) {
    return EtherCAT::SyncManager::mappingBits(entry);
}

} // namespace CiA301

// ============================================================================
// EtherCAT::SyncManagerAccessor — per-SM configuration and diagnostics
// ============================================================================

namespace EtherCAT {

/**
 * @brief Lightweight accessor for one EtherCAT Sync Manager on a slave.
 *
 * Returned by `Slave::sm(smIndex)`.  Holds a reference to the slave
 * so it can perform SDO reads and raw APRD register reads.
 *
 * ## Typical usage
 * @code
 *   // Dump hardware register state of SM2
 *   master.slave(0).sm(2).dump("MySM");
 *
 *   // Validate that SM2 hardware matches expected config
 *   auto result = master.slave(0).sm(2).validate(expectedConfig);
 *   if (!result.valid) {
 *       printf("SM2 mismatch: %s\n", result.message.c_str());
 *   }
 *
 *   // Read comm type via SDO
 *   uint8_t type;
 *   master.slave(0).sm(2).readCommType(type);
 * @endcode
 *
 * @note The accessor is a lightweight value type (reference + byte).
 *       There is no mutable SM state stored here; all state is on the slave.
 */
class SyncManagerAccessor {
public:
    // -----------------------------------------------------------------------
    // Nested types
    // -----------------------------------------------------------------------

    /**
     * @brief Raw 8-byte hardware register block read from the ESC.
     *
     * Byte layout follows the EtherCAT specification:
     * - Bytes 0–1: Physical start address (little-endian)
     * - Bytes 2–3: Length (little-endian)
     * - Byte  4:   Control register
     * - Byte  5:   Status register
     * - Byte  6:   Activate register
     * - Byte  7:   PDI control register
     */
    struct RawHWConfig {
        uint16_t start_addr = 0; ///< Physical start address in the slave memory
        uint16_t length     = 0; ///< Region length in bytes
        uint8_t  control    = 0; ///< Control register (mode, direction, IRQ, watchdog flags)
        uint8_t  status     = 0; ///< Status register (read-only from master perspective)
        uint8_t  activate   = 0; ///< Activate register (bit 0 = SM enabled)
        uint8_t  pdi_ctrl   = 0; ///< PDI control register
        bool     read_ok    = false; ///< true if the APRD read succeeded

        /// Return true if the SM is enabled (activate bit 0 set) and the read succeeded.
        bool isEnabled()       const { return read_ok && ((activate & 0x01U) != 0); }
        /// Return true if the SM is in mailbox mode (mode bits = 0x02).
        bool isMailboxMode()   const {
            return read_ok &&
                   ((control & PDO::SM_CTRL_MODE_MASK) == PDO::SM_CTRL_MODE_MAILBOX);
        }
        /// Return true if the SM is in buffered (process data) mode (mode bits = 0x00).
        bool isBufferedMode()  const {
            return read_ok &&
                   ((control & PDO::SM_CTRL_MODE_MASK) == PDO::SM_CTRL_MODE_BUFFERED);
        }
        /// Return true if data flows from master to slave (DIR_WRITE set in control).
        bool isMasterToSlave() const {
            return read_ok && ((control & PDO::SM_CTRL_DIR_WRITE) != 0);
        }
        /// Return true if data flows from slave to master.
        bool isSlaveToMaster() const {
            return read_ok && ((control & PDO::SM_CTRL_DIR_WRITE) == 0);
        }
    };

    /**
     * @brief Result of a SM validation check.
     */
    struct ValidationResult {
        bool        valid   = false; ///< true if the SM matches expectations
        std::string message;         ///< Human-readable description of the mismatch (empty if valid)
    };

    // -----------------------------------------------------------------------
    // Construction
    // -----------------------------------------------------------------------

    /**
     * @brief Construct an accessor for SM @p smIndex of @p slave.
     * @param slave    The slave whose SM is being accessed
     * @param smIndex  Zero-based SM index (0 = SM0, 1 = SM1, …)
     */
    SyncManagerAccessor(Slave& slave, uint8_t smIndex);

    // -----------------------------------------------------------------------
    // Identity
    // -----------------------------------------------------------------------

    /** @brief Zero-based SM index. */
    uint8_t  index()            const { return index_; }
    /** @brief Physical ESC register base address (0x0800 + index * 8). */
    uint16_t physRegisterBase() const {
        return EtherCAT::SyncManager::registerBase(index_);
    }

    // -----------------------------------------------------------------------
    // Hardware register access
    // -----------------------------------------------------------------------

    /**
     * @brief Read the 8-byte SM configuration from the slave's ESC registers.
     *
     * Uses APRD to read the SM register block directly from the hardware.
     * The result is available even before any SDO communication.
     *
     * @param timeout_ms  Timeout for the APRD operation (default 200 ms)
     * @return Parsed SM configuration; `read_ok` is false on failure.
     */
    RawHWConfig readHardwareConfig(unsigned int timeout_ms = 200) const;

    // -----------------------------------------------------------------------
    // Object dictionary access (via SDO)
    // -----------------------------------------------------------------------

    /**
     * @brief Read the SM Communication Type via SDO (0x1C00 : smIndex+1).
     *
     * The communication type is stored in the CiA 301 object dictionary.
     * Requires the slave to be in at least PRE_OP state.
     *
     * @param[out] type  Populated with the SM type byte from `EtherCAT::SyncManager::CommType`
     * @return SlaveError::Ok on success
     */
    SlaveError readCommType(uint8_t& type) const;

    /**
     * @brief Read the number of PDOs assigned to this SM via SDO (0x1C10+n : 0x00).
     *
     * @param[out] count  Number of assigned PDO mapping objects
     * @return SlaveError::Ok on success
     */
    SlaveError readPDOAssignCount(uint8_t& count) const;

    /**
     * @brief Read a specific PDO assignment entry via SDO (0x1C10+n : subIndex).
     *
     * @param subIndex  One-based subindex (1 = first PDO, 2 = second PDO, …)
     * @param[out] pdoIndex  OD index of the assigned PDO mapping object
     * @return SlaveError::Ok on success
     */
    SlaveError readPDOAssignment(uint8_t subIndex, uint16_t& pdoIndex) const;

    // -----------------------------------------------------------------------
    // Validation
    // -----------------------------------------------------------------------

    /**
     * @brief Validate that the slave's SM hardware matches an expected configuration.
     *
     * Compares the hardware register readback with @p expected.  Reports the
     * first field that mismatches in the result message.
     *
     * @param expected  Expected SM configuration (from the PDO setup)
     * @return Validation result; `valid` is true only if all fields match.
     */
    ValidationResult validate(const PDO::SyncManagerConfig& expected) const;

    /**
     * @brief Validate that the SM Communication Type (0x1C00) matches @p expectedType.
     *
     * @param expectedType  Expected type byte (from `EtherCAT::SyncManager::CommType`)
     * @return Validation result
     */
    ValidationResult validateCommType(uint8_t expectedType) const;

    // -----------------------------------------------------------------------
    // Diagnostics / debugging
    // -----------------------------------------------------------------------

    /**
     * @brief Format the given raw hardware config as a human-readable string.
     *
     * Decodes mode, direction, flag bits, and enable state.  Detects
     * common conservative default mailbox configurations and marks them.
     *
     * @param cfg  Configuration to format (from `readHardwareConfig()`)
     * @return Human-readable description string
     */
    std::string formatConfig(const RawHWConfig& cfg) const;

    /**
     * @brief Read SM hardware registers and log a human-readable summary.
     *
     * Calls `readHardwareConfig()`, formats the result via `formatConfig()`,
     * and logs at INFO level under @p tag.
     *
     * @param tag  Logger tag (max 31 characters)
     */
    void dump(const char* tag = "SM") const;

    /**
     * @brief Log the SM status and activate register bytes (mailbox diagnostics).
     *
     * Reads the status (byte 5) and activate (byte 6) bytes of this SM's
     * register block and logs them.  Most useful for SM0 and SM1 (mailbox SMs)
     * to check whether the SM is active and whether the slave has set an error.
     *
     * Also reads and logs the SM watchdog status register (0x0440).
     *
     * @param tag  Logger tag
     */
    void dumpMailboxStatus(const char* tag = "SM") const;

    /**
     * @brief Read all assigned PDOs and log them (requires PRE_OP or higher).
     *
     * @param tag  Logger tag
     */
    void dumpPDOAssignments(const char* tag = "SM") const;

private:
    Slave& slave_; ///< Owning slave
    uint8_t        index_; ///< SM index (0-based)
};

/**
 * @brief Debug function to dump mailbox hardware register configuration.
 *
 * Reads and displays the actual hardware register configuration of SM0 and SM1
 * (mailbox sync managers), showing start addresses, lengths, control register
 * bits, and activation status.
 *
 * @param master        EtherCAT master instance
 * @param slave_index   Slave index (0-based)
 * @param tag           Logger tag
 */
void debugMailboxConfiguration(Master& master, uint16_t slave_index, const char* tag);

} // namespace EtherCAT
