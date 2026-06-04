/**
 * @file EtherCATSlave.hpp
 * @brief Per-slave state machine for EtherCAT slave lifecycle management
 *
 * @details
 * EtherCATSlave enforces correct EtherCAT state transitions on a per-slave
 * basis.  The master owns a vector of slaves (one per discovered device) and
 * exposes them through `master.slave(index)`.
 *
 * ## State-machine guards
 *
 * | Transition          | Guard                                          |
 * |---------------------|-------------------------------------------------|
 * | INIT  → PRE_OP      | Mailbox (SM0/SM1) must be configured first      |
 * | PRE_OP → SAFE_OP    | PDO sync-managers (SM2/SM3) must be configured  |
 * | SAFE_OP → OP        | (no additional guard — standard EtherCAT rule)  |
 *
 * `assumeMailboxAlreadyConfigured()` bypasses the mailbox guard when the
 * firmware is known to be pre-configured.
 *
 * ## Detailed error reporting
 *
 * Every method returns `SlaveError` — a scoped enum — instead of a bare
 * `bool`.  Failures are also logged with human-readable messages.
 *
 * ## NonExistingSlave
 *
 * When the caller requests a slave index that does not exist,
 * `EtherCATMaster::slave()` returns a reference to a global
 * `NonExistingSlave` instance whose methods all log a CRITICAL error and
 * return `SlaveError::SlaveNotFound`.
 */
#pragma once

#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <vector>
#include <optional>

#include "tether/ethercat/EtherCATTypes.hpp"
#include "tether/ethercat/CachedSIIReader.hpp"
#include "tether/ethercat/SyncManager.hpp"
#include "tether/platform/Platform.hpp"

namespace EtherCAT {

// Forward declarations
class EtherCATMaster;

namespace SDO {
    class SDOManager;
}

class PDOManager;
class DCManager;

// ============================================================================
// Global debug control
// ============================================================================

/**
 * @brief Enable or disable detailed EtherCAT state machine debug logging
 *
 * When enabled, state transition methods will log detailed information about:
 * - Why a state change is being attempted
 * - Current state before transition
 * - Requirements being checked (mailbox configured, PDO configured, etc.)
 * - Whether requirements are fulfilled
 * - Result of the transition attempt
 *
 * @param enable true to enable debug logging, false to disable
 */
void enableStateMachineDebug(bool enable);

// ============================================================================
// SlaveError — detailed error enum
// ============================================================================

/**
 * @brief Detailed error codes returned by EtherCATSlave methods.
 */
enum class SlaveError : uint8_t {
    Ok = 0,                         ///< Operation completed successfully

    // -- Configuration guards --
    MailboxNotConfigured,           ///< Mailbox (SM0/SM1) must be configured before PRE_OP
    PDONotConfigured,               ///< PDO sync-managers must be configured before SAFE_OP
    InvalidStateTransition,         ///< Requested transition violates the ESM state machine
    SlaveNotFound,                  ///< Slave index out of range — returned by NonExistingSlave

    // -- Communication errors --
    TransportError,                 ///< Low-level send/receive failure
    Timeout,                        ///< Slave did not respond in time
    ALStatusError,                  ///< Slave reported an AL Status error code
    WorkingCounterMismatch,         ///< Unexpected WKC value

    // -- Mailbox / SDO errors --
    MailboxConfigFailed,            ///< Failed to write SM0/SM1 configuration registers
    SDOError,                       ///< SDO read/write failed
    SIIReadError,                   ///< SII EEPROM read failure

    // -- PDO errors --
    PDOConfigFailed,                ///< Failed to configure process-data sync managers
    PDOMappingFailed,               ///< Mapping finalization failed

    // -- Generic --
    NotInitialized,                 ///< Master not started
    InternalError,                  ///< Unexpected internal error
};

/**
 * @brief Convert a SlaveError to a human-readable string.
 */
inline const char* slaveErrorToString(SlaveError e) {
    switch (e) {
        case SlaveError::Ok:                     return "Ok";
        case SlaveError::MailboxNotConfigured:    return "Mailbox (SM0/SM1) not configured — call configureMailbox() or assumeMailboxAlreadyConfigured() first";
        case SlaveError::PDONotConfigured:        return "PDO sync-managers not configured — call configurePDOSyncManagers() first";
        case SlaveError::InvalidStateTransition:  return "Invalid EtherCAT state transition";
        case SlaveError::SlaveNotFound:           return "Slave index does not exist — check getDiscoveredSlaveCount()";
        case SlaveError::TransportError:          return "Transport send/receive failure";
        case SlaveError::Timeout:                 return "Slave did not respond in time";
        case SlaveError::ALStatusError:           return "Slave reported AL Status error";
        case SlaveError::WorkingCounterMismatch:  return "Working counter mismatch";
        case SlaveError::MailboxConfigFailed:     return "Failed to write mailbox SM registers";
        case SlaveError::SDOError:                return "SDO operation failed";
        case SlaveError::SIIReadError:            return "SII EEPROM read failed";
        case SlaveError::PDOConfigFailed:         return "PDO sync-manager configuration failed";
        case SlaveError::PDOMappingFailed:        return "PDO mapping finalization failed";
        case SlaveError::NotInitialized:          return "Master not initialized";
        case SlaveError::InternalError:           return "Internal error";
        default:                                  return "Unknown error";
    }
}

// ============================================================================
// EtherCATSlave — per-slave state machine
// ============================================================================

/**
 * @brief Per-slave state machine that enforces safe EtherCAT ESM transitions.
 *
 * Instances are owned by EtherCATMaster.  The slave holds a back-reference
 * to the master so it can use transport, SDO, PDO, and SII facilities.
 *
 * The class is **not** thread-safe by itself; the master serialises calls.
 *
 * ## Typical usage
 * @code
 *   auto& s = master.slave(0);
 *   s.configureMailbox();                // or s.assumeMailboxAlreadyConfigured()
 *   s.transitionToPreOp();
 *   s.configurePDOSyncManagers();
 *   s.transitionToSafeOp();
 *   s.transitionToOp();
 * @endcode
 */
class EtherCATSlave {
public:
    /**
     * @brief Construct a slave bound to a master at a given position.
     * @param master  Owning EtherCATMaster instance
     * @param index   Slave position on the bus (0-based)
     */
    EtherCATSlave(EtherCATMaster& master, uint16_t index);

    virtual ~EtherCATSlave() = default;

    // -- Identification -----------------------------------------------------

    /** @brief Slave bus index (0-based). */
    uint16_t index() const { return index_; }

    /** @brief Auto-increment ADP for this slave. */
    uint16_t adp() const;

    // -- SII cache ----------------------------------------------------------

    /**
     * @brief Access the cached SII reader for this slave.
     *
     * The CachedSIIReader deduplicates EEPROM reads across multiple callers.
     */
    SII::CachedSIIReader& siiCache() { return sii_cache_; }

    // -- Mailbox configuration -----------------------------------------------

    /**
     * @brief Configure the mailbox (SM0/SM1) from SII EEPROM automatically.
     *
     * This reads the standard mailbox offsets from the slave's EEPROM, writes
     * SM0/SM1 configuration registers, and sets up the SDO subsystem.
     *
     * @param log_level  Verbosity for diagnostic output
     * @return SlaveError::Ok on success
     */
    virtual SlaveError configureMailbox(
        Tether::Platform::LogLevel log_level = Tether::Platform::LogLevel::Info);

    /**
     * @brief Configure the mailbox with explicit parameters.
     *
     * Use this when SII is unavailable or you want to override EEPROM values.
     *
     * @return SlaveError::Ok on success
     */
    virtual SlaveError configureMailbox(
        uint16_t wr_addr, uint16_t wr_len,
        uint16_t rd_addr, uint16_t rd_len,
        uint16_t protocols);

    /**
     * @brief Inform the slave that mailbox configuration is already done.
     *
     * Call this when the slave firmware pre-configures SM0/SM1, so the
     * PRE_OP guard is satisfied without writing registers.
     */
    virtual void assumeMailboxAlreadyConfigured();

    /** @brief True if mailbox has been configured (or assumed configured). */
    bool isMailboxConfigured() const { return mailbox_configured_; }

    // -- PDO sync-manager configuration -------------------------------------

    /**
     * @brief Configure SM2/SM3 (process data SMs) from the slave's SII.
     *
     * Must be called in PRE_OP, after PDO mapping SDO writes, and before
     * the transition to SAFE_OP.
     *
     * @return SlaveError::Ok on success
     */
    virtual SlaveError configurePDOSyncManagers();

    /**
     * @brief Configure SM2/SM3 with explicit parameters.
     */
    virtual SlaveError configurePDOSyncManagers(
        uint16_t sm2_addr, uint16_t sm2_len, uint8_t sm2_ctrl,
        uint16_t sm3_addr, uint16_t sm3_len, uint8_t sm3_ctrl);

    /**
     * @brief Mark PDO sync-managers as already configured.
     */
    virtual void assumePDOAlreadyConfigured();

    /** @brief True if PDO sync-managers have been configured. */
    bool isPDOConfigured() const { return pdo_configured_; }

    // -- State transitions ---------------------------------------------------

    /**
     * @brief Request a specific ESM state, enforcing configuration guards.
     *
     * @param target  Target SlaveState
     * @return SlaveError::Ok on success
     */
    virtual SlaveError transitionTo(SlaveState target);

    /** @brief Convenience: transition to INIT. */
    virtual SlaveError transitionToInit();

    /** @brief Convenience: transition to PRE_OP (requires mailbox). */
    virtual SlaveError transitionToPreOp();

    /** @brief Convenience: transition to SAFE_OP (requires PDO). */
    virtual SlaveError transitionToSafeOp();

    /** @brief Convenience: transition to OP. */
    virtual SlaveError transitionToOp();

    /** @brief Convenience: transition to BOOT. */
    virtual SlaveError transitionToBoot();

    // -- State query ---------------------------------------------------------

    /**
     * @brief Read the current AL Status register from the slave.
     *
     * @param[out] state  Populated with the slave's current state
     * @return SlaveError::Ok on success
     */
    virtual SlaveError readState(SlaveState& state);

    /**
     * @brief Read the current AL Status Code.
     *
     * @param[out] code  AL Status Code (see `alcode` namespace)
     * @return SlaveError::Ok on success
     */
    virtual SlaveError readALStatusCode(uint16_t& code);

    /**
     * @brief Convenience accessor that reads and returns the current AL state.
     *
     * Performs a network read and returns `std::nullopt` on error.
     */
    std::optional<SlaveState> ALState();

    /**
     * @brief Convenience accessor that reads and returns the current AL status code.
     *
     * Performs a network read and returns `std::nullopt` on error.
     */
    std::optional<uint16_t> ALCode();

    // -- Watchdog ------------------------------------------------------------

    /**
     * @brief Configure PDI and process data watchdog timers.
     *
     * @param pdi_timeout_100us   PDI watchdog in 100µs units (0 = disable)
     * @param pdata_timeout_100us Process data watchdog in 100µs units
     * @return SlaveError::Ok on success
     */
    virtual SlaveError configureWatchdogs(uint16_t pdi_timeout_100us,
                                          uint16_t pdata_timeout_100us);

    /** @brief Disable all watchdogs for this slave. */
    virtual SlaveError disableWatchdogs();

    /**
     * @brief Read the watchdog status registers.
     *
     * @param[out] wd_status   Watchdog status byte
     * @param[out] pdi_cnt     PDI watchdog counter
     * @param[out] pdata_cnt   Process-data watchdog counter
     * @return SlaveError::Ok on success
     */
    virtual SlaveError readWatchdogStatus(uint8_t& wd_status,
                                          uint8_t& pdi_cnt,
                                          uint8_t& pdata_cnt);

    // -- SDO convenience -----------------------------------------------------

    /**
     * @brief Synchronous SDO read.
     *
     * @param index     Object dictionary index
     * @param subindex  Subindex
     * @param[out] data Output buffer
     * @param[in,out] size  On input: buffer capacity.  On output: bytes read.
     * @return SlaveError::Ok on success
     */
    virtual SlaveError sdoRead(uint16_t index, uint8_t subindex,
                               void* data, size_t& size);

    /**
     * @brief Synchronous SDO write.
     *
     * @param index     Object dictionary index
     * @param subindex  Subindex
     * @param data      Data to write
     * @param size      Data length
     * @return SlaveError::Ok on success
     */
    virtual SlaveError sdoWrite(uint16_t index, uint8_t subindex,
                                const void* data, size_t size);

    /** @brief Read a uint8_t via SDO. */
    virtual SlaveError sdoReadU8(uint16_t index, uint8_t sub, uint8_t& out);
    /** @brief Read a uint16_t via SDO. */
    virtual SlaveError sdoReadU16(uint16_t index, uint8_t sub, uint16_t& out);
    /** @brief Read a uint32_t via SDO. */
    virtual SlaveError sdoReadU32(uint16_t index, uint8_t sub, uint32_t& out);

    /** @brief Write a uint8_t via SDO. */
    virtual SlaveError sdoWriteU8(uint16_t index, uint8_t sub, uint8_t val);
    /** @brief Write a uint16_t via SDO. */
    virtual SlaveError sdoWriteU16(uint16_t index, uint8_t sub, uint16_t val);
    /** @brief Write a uint32_t via SDO. */
    virtual SlaveError sdoWriteU32(uint16_t index, uint8_t sub, uint32_t val);

    // -- SII convenience -----------------------------------------------------

    /**
     * @brief Read the full SII data (cached).
     *
     * @param[out] data  Output SII data structure
     * @return SlaveError::Ok on success
     */
    virtual SlaveError readSII(SII::SIIData& data);

    /** @brief Log a one-line SII summary for this slave. */
    virtual void logSIISummary(const char* tag = "EtherCAT");

    // -- Sync Manager access ------------------------------------------------------

    /**
     * @brief Access a Sync Manager on this slave by index.
     *
     * Returns a lightweight `SyncManagerAccessor` that wraps this slave and the
     * given SM index. Use it to read hardware registers, query the object
     * dictionary, validate configuration, and dump diagnostics.
     *
     * @param smIndex  Zero-based SM index (0 = SM0, 1 = SM1, …)
     * @return SyncManagerAccessor for SM @p smIndex
     *
     * @code
     *   // Dump SM2 hardware register state
     *   master.slave(0).sm(2).dump("TAG");
     *
     *   // Validate SM2 matches expected configuration
     *   auto result = master.slave(0).sm(2).validate(expectedCfg);
     * @endcode
     */
    virtual SyncManagerAccessor sm(uint8_t smIndex);

    // -- Link to master ------------------------------------------------------

    /** @brief Access the owning master. */
    EtherCATMaster& master() { return master_; }
    const EtherCATMaster& master() const { return master_; }

protected:
    EtherCATMaster& master_;
    uint16_t index_;

    bool mailbox_configured_ = false;
    bool pdo_configured_ = false;

    SII::CachedSIIReader sii_cache_;
};

// ============================================================================
// NonExistingSlave — error sentinel returned for invalid indices
// ============================================================================

/**
 * @brief Sentinel slave returned when `master.slave(n)` is out of range.
 *
 * Every method logs a CRITICAL error and returns `SlaveError::SlaveNotFound`.
 * The log message includes guidance on how to fix the problem.
 */
class NonExistingSlave final : public EtherCATSlave {
public:
    /**
     * @brief Construct a NonExistingSlave.
     * @param master   Reference back to the master
     * @param index    The invalid index that was requested
     */
    NonExistingSlave(EtherCATMaster& master, uint16_t index);

    SlaveError configureMailbox(Tether::Platform::LogLevel) override;
    SlaveError configureMailbox(uint16_t, uint16_t, uint16_t, uint16_t, uint16_t) override;
    void assumeMailboxAlreadyConfigured() override;

    SlaveError configurePDOSyncManagers() override;
    SlaveError configurePDOSyncManagers(uint16_t, uint16_t, uint8_t,
                                         uint16_t, uint16_t, uint8_t) override;
    void assumePDOAlreadyConfigured() override;

    SlaveError transitionTo(SlaveState) override;
    SlaveError transitionToInit() override;
    SlaveError transitionToPreOp() override;
    SlaveError transitionToSafeOp() override;
    SlaveError transitionToOp() override;
    SlaveError transitionToBoot() override;

    SlaveError readState(SlaveState&) override;
    SlaveError readALStatusCode(uint16_t&) override;

    SlaveError configureWatchdogs(uint16_t, uint16_t) override;
    SlaveError disableWatchdogs() override;
    SlaveError readWatchdogStatus(uint8_t&, uint8_t&, uint8_t&) override;

    SlaveError sdoRead(uint16_t, uint8_t, void*, size_t&) override;
    SlaveError sdoWrite(uint16_t, uint8_t, const void*, size_t) override;
    SlaveError sdoReadU8(uint16_t, uint8_t, uint8_t&) override;
    SlaveError sdoReadU16(uint16_t, uint8_t, uint16_t&) override;
    SlaveError sdoReadU32(uint16_t, uint8_t, uint32_t&) override;
    SlaveError sdoWriteU8(uint16_t, uint8_t, uint8_t) override;
    SlaveError sdoWriteU16(uint16_t, uint8_t, uint16_t) override;
    SlaveError sdoWriteU32(uint16_t, uint8_t, uint32_t) override;

    SlaveError readSII(SII::SIIData&) override;
    void logSIISummary(const char*) override;

    SyncManagerAccessor sm(uint8_t smIndex) override;

private:
    void logCritical(const char* method) const;
};

} // namespace EtherCAT
