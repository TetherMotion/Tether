/**
 * @file CiA402Drive.hpp
 * @brief CiA 402 Drive Controller with Fixed PDO Mapping
 *
 * @details
 * This module provides a CiA 402 drive controller that handles:
 * - Fixed PDO mapping configuration (drive-specific, struct-based)
 * - State machine management (INIT -> PRE_OP -> SAFE_OP -> OP)
 * - Homing procedures with current position as home
 * - Template-based typed access to RxPDO / TxPDO buffers
 *
 * ## PDO Access Model
 *
 * The drive allocates internal PDO buffers whose sizes are set by
 * assignFixedPDOs().  The user accesses these buffers through typed
 * pointers obtained via the template helpers rxPDO<T>() / txPDO<T>().
 * The actual bytes are exchanged by the DC real-time task through
 * registered entries in the PDO transport layer.
 *
 * ## Usage Example
 *
 * @code
 * #include "CiA402Drive.hpp"
 * #include "drives/AS715N/AS715NPDO.hpp"
 *
 * using namespace EtherCAT::Drives::AS715N_pdo;
 *
 * CiA402Drive drive(master, 0);
 * drive.assignFixedPDOs(0x1705, 0x1B04, RxPDO_1705.size, TxPDO_1B04.size);
 * drive.registerPDOBuffers();
 *
 * // In cyclic callback:
 * auto* tx = drive.txPDO<AS715N_TxPDO_1B04>();
 * uint16_t sw = tx->statusword;
 *
 * auto* rx = drive.rxPDO<AS715N_RxPDO_1705>();
 * rx->controlword = 0x000F;
 * rx->target_position = 1234;
 * rx->modes_of_operation = 8;  // CSP
 * @endcode
 */

#pragma once

#include <cstdint>
#include <cstddef>
#include <cstring>
#include <functional>
#include <array>
#include <memory>
#include "profiles/cia301/CiA301Defs.hpp"
#include "profiles/cia301/CiA402Defs.hpp"
#include "tether/drives/DynaDrive/Registers/Statusword.hpp"
#include "tether/drives/DynaDrive/Registers/Controlword.hpp"
#include "tether/ethercat/TetherConfig.hpp"
#include "tether/ethercat/Slave.hpp"  // for Slave::MultiPDOAssignment
#include "tether/profiles/cia402/CiA402StateUtils.hpp"  // ECState, DriveState, getECStateName, etc.

// Forward declarations
namespace EtherCAT {
class Master;
} // namespace EtherCAT

namespace EtherCAT {

enum class ControlWord : uint16_t {
    DISABLE_VOLTAGE    = 0x0000,
    SHUTDOWN           = 0x0006,
    SWITCH_ON          = 0x0007,
    ENABLE_OPERATION   = 0x000F,
    FAULT_RESET        = 0x0080,
};

// ============================================================================
// CiA 402 Drive Controller
// ============================================================================

/**
 * @brief CiA 402 Drive Controller
 *
 * Manages a single CiA 402 compatible EtherCAT servo drive.  The class
 * owns the PDO buffers and provides typed template access for zero-copy
 * read/write of process data.  The physical PDO exchange is handled by
 * the DC real-time task; this class only manages the buffer memory and
 * the SDO-based configuration / state machine.
 */
class CiA402Drive {
public:
    CiA402Drive(Master& master, uint16_t slave_index);
    ~CiA402Drive() = default;

    Master* master() const { return m_master; }

    // Non-copyable
    CiA402Drive(const CiA402Drive&) = delete;
    CiA402Drive& operator=(const CiA402Drive&) = delete;

    // ========================================================================
    // Configuration
    // ========================================================================

    uint16_t slaveIndex() const { return m_slave_index; }

    /**
     * @brief Assign fixed (slave-defined) PDOs to SM2 / SM3
     *
     * Writes the SM2 (RxPDO) and SM3 (TxPDO) assignment registers
     * (0x1C12 / 0x1C13) via SDO.  The PDO entry tables inside the
     * slave are NOT rewritten.
     *
     * @param rxpdo_index  RxPDO object index (e.g. 0x1705)
     * @param txpdo_index  TxPDO object index (e.g. 0x1B04)
     * @param rxpdo_size   RxPDO total size in bytes
     * @param txpdo_size   TxPDO total size in bytes
     * @return true (sizes are stored regardless of SDO outcome)
     */
    bool assignFixedPDOs(uint16_t rxpdo_index, uint16_t txpdo_index,
                         uint16_t rxpdo_size, uint16_t txpdo_size);

    /// Set PDO buffer sizes and indices without writing PDO assignment SDOs.
    ///
    /// This is used after configureMultiPDOs() has already written the PDO
    /// assignments (0x1C12/0x1C13) via the multi-PDO path.  Calling
    /// assignFixedPDOs() in that case would overwrite the multi-PDO
    /// assignment with a single-PDO assignment, destroying the FSoE PDO
    /// mapping.  This method only sets the internal buffer sizes and
    /// indices so that registerPDOBuffers() can register the correct
    /// buffer lengths.
    void setPDOBufferSizes(uint16_t rxpdo_index, uint16_t txpdo_index,
                           uint16_t rxpdo_size, uint16_t txpdo_size);

    bool registerPDOBuffers();
    bool isPDORegistered() const { return m_pdo_registered; }

    /**
     * @brief Reset PDO registration state so buffers can be re-registered.
     *
     * After a forced INIT (e.g. during slave recovery), the PDO mapping
     * entries in the PDOManager are stale.  Call this to clear the
     * registration flag so that a subsequent registerPDOBuffers() call
     * re-adds the entries.  The old entries should be removed from the
     * PDOManager mapping first (e.g. via remove_entries_for_slave()).
     */
    void resetPDORegistration();
    void setSDOTimeout(uint32_t timeout_ms) { m_sdo_timeout_ms = timeout_ms; }

    // ========================================================================
    // Typed PDO Buffer Access
    // ========================================================================

    static constexpr size_t kMaxPDOBufferSize = ECAT_CIA402_MAX_PDO_BUFFER_SIZE;

    /**
     * @brief Get typed pointer to the RxPDO buffer (master -> slave)
     * @tparam T  Packed struct matching the PDO layout
     */
    template<typename T>
    T* rxPDO() {
        static_assert(sizeof(T) <= kMaxPDOBufferSize, "RxPDO struct too large");
        return reinterpret_cast<T*>(m_rxpdo_buffer);
    }

    /**
     * @brief Get typed pointer to the TxPDO buffer (slave -> master)
     * @tparam T  Packed struct matching the PDO layout
     */
    template<typename T>
    const T* txPDO() const {
        static_assert(sizeof(T) <= kMaxPDOBufferSize, "TxPDO struct too large");
        return reinterpret_cast<const T*>(m_txpdo_buffer);
    }

    void*       getRxPDOBuffer() { return m_rxpdo_buffer; }
    const void* getTxPDOBuffer() const { return m_txpdo_buffer; }
    uint16_t    getRxPDOSize() const { return m_rxpdo_size; }
    uint16_t    getTxPDOSize() const { return m_txpdo_size; }
    uint16_t    getRxPDOIndex() const { return m_rxpdo_index; }
    uint16_t    getTxPDOIndex() const { return m_txpdo_index; }

    // ========================================================================
    // EtherCAT State Machine
    // ========================================================================

    ECState getECState();
    bool gotoInit();
    bool gotoPreOp();
    bool gotoSafeOp();
    bool gotoOp();

    /**
     * @brief Full state sequence: INIT -> PRE_OP -> SAFE_OP -> OP
     * @param configure_sm  If true, configure SM2/SM3 from SII and
     *                      update lengths to match assigned PDO sizes.
     */
    bool transitionToOp(bool configure_sm = true);

    /**
     * @brief Full state sequence with multi-PDO-per-SM configuration.
     *
     * Calls Slave::configureMultiPDOs() to configure the sync managers and
     * FMMUs for multiple PDO mappings per SM, then transitions through
     * PRE_OP → SAFE_OP → OP.  Unlike the single-PDO variant, this does NOT
     * re-configure SMs from SII (configureMultiPDOs handles all SM/FMMU
     * setup).  Internal PDO buffer sizes are set from the assignment's total
     * Rx/Tx sizes, and PDO buffers are registered automatically.
     *
     * Must be called in PRE_OP (or the method will transition there first).
     *
     * @param assignment  Multi-PDO assignment (SM configs + PDO mappings)
     * @return true on success, false on any transition or config failure
     */
    bool transitionToOp(const Slave::MultiPDOAssignment& assignment);

    // ========================================================================
    // CiA 402 State Machine
    // ========================================================================

    DriveState getDriveState();
    uint16_t getStatusword();
    uint16_t getControlword() const { return m_controlword; }
    void setControlword(uint16_t cw) { m_controlword = cw; }
    bool sendControlwordSDO(uint16_t controlword);
    bool enable(uint32_t timeout_ms = 5000);
    bool disable();
    bool quickStop();
    bool resetFault();
    bool isEnabled();
    bool isFaulted();
    bool isTargetReached();

    // ========================================================================
    // DynaDrive Custom FSM (rsl_drive_sdk / ANYdrive)
    // ========================================================================

    using DynaDriveState = EtherCAT::Drives::Registers::DynaDrive::Status::StateOptions;

    static DynaDriveState decodeDynaDriveState(uint32_t statusword);
    static const char* getDynaDriveStateName(DynaDriveState state);

    bool readDynaDriveStatusword(uint32_t& statusword);
    bool sendDynaDriveControlword(EtherCAT::Drives::Registers::DynaDrive::Controlword::Options controlword);
    bool enableDynaDrive(uint32_t timeout_ms = 10000);
    bool disableDynaDrive();
    bool isDynaDriveControlOp();

    // ========================================================================
    // Operating Mode (SDO-based)
    // ========================================================================

    bool setOperatingMode(int8_t mode);
    int8_t getOperatingMode();
    bool setModeCSP() { return setOperatingMode(CiA402::OperatingMode::CyclicSyncPosition); }
    bool setModeCSV() { return setOperatingMode(CiA402::OperatingMode::CyclicSyncVelocity); }
    bool setModeCST() { return setOperatingMode(CiA402::OperatingMode::CyclicSyncTorque); }
    bool setModePP()  { return setOperatingMode(CiA402::OperatingMode::ProfilePosition); }
    bool setModeHM()  { return setOperatingMode(CiA402::OperatingMode::Homing); }

    // ========================================================================
    // Homing (SDO-based)
    // ========================================================================

    bool setHomingMethod(int8_t method);
    bool homeToCurrentPosition(int32_t home_offset = 0);
    bool executeHoming(uint32_t timeout_ms = 30000);
    bool isHomingComplete();
    bool hasHomingError();

private:
    bool writeControlword(uint16_t controlword);
    bool readStatusword(uint16_t& statusword);
    bool waitForDriveState(DriveState target, uint32_t timeout_ms);

    /// Common SAFE_OP → OP tail: DC reconfig, PDO enable, diagnostics, OP request.
    /// Called by both transitionToOp() variants after SM/FMMU configuration.
    bool transitionSafeOpToOp();

    uint16_t       m_slave_index;
    Master* m_master{nullptr};
    uint32_t       m_sdo_timeout_ms{1000};

    // PDO configuration
    uint16_t m_rxpdo_index{0};
    uint16_t m_txpdo_index{0};
    uint16_t m_rxpdo_size{0};
    uint16_t m_txpdo_size{0};
    bool     m_pdo_configured{false};

    // PDO buffers
    uint8_t m_rxpdo_buffer[kMaxPDOBufferSize];
    uint8_t m_txpdo_buffer[kMaxPDOBufferSize];

    // PDO registration tracking
    int  m_rxpdo_entry_index{-1};
    int  m_txpdo_entry_index{-1};
    bool m_pdo_registered{false};

    // SDO-based state (used by enable / disable helpers)
    uint16_t m_controlword{0};
    uint16_t m_statusword{0};
};

// ============================================================================
// Drive Manager
// ============================================================================

constexpr size_t kMaxManagedDrives = ECAT_CIA402_MAX_MANAGED_DRIVES;

class DriveManager {
public:
    DriveManager() = default;
    // Destructor is no longer needed — unique_ptr handles cleanup.
    // Kept as defaulted here for ABI clarity; the .cpp no longer defines it.

    size_t initializeDrives(Master& master, size_t slave_count);
    CiA402Drive* getDrive(size_t index);
    CiA402Drive* getDriveBySlaveIndex(uint16_t slave_index);
    size_t getDriveCount() const { return m_drive_count; }
    bool transitionAllToOp();
    bool enableAll();
    bool disableAll();

private:
    std::array<std::unique_ptr<CiA402Drive>, kMaxManagedDrives> m_drives{};
    size_t m_drive_count{0};
};

} // namespace EtherCAT
