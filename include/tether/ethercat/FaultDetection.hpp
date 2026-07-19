/**
 * @file FaultDetection.hpp
 * @brief EtherCAT Slave Fault Detection and Reporting
 *
 * @details
 * This module provides comprehensive fault detection, decoding, and
 * reporting for EtherCAT slaves. It handles:
 *
 * - AL Status Code interpretation (standard EtherCAT errors)
 * - CiA 402 drive fault codes (emergency messages, error codes)
 * - Manufacturer-specific fault codes
 * - DC synchronization faults
 * - Automatic fault polling and notification
 *
 * ## Error Code Format
 *
 * EtherCAT and CiA 402 use several error code formats:
 *
 * 1. **AL Status Code** (register 0x0134):
 *    - 16-bit code from ESC
 *    - Examples: 0x001A = Synchronization error, 0x002D = No sync error
 *
 * 2. **CiA 402 Error Code** (object 0x603F):
 *    - 16-bit emergency code
 *    - Format: 0xXYZZ where X=class, Y=subclass, ZZ=specific
 *
 * 3. **Manufacturer Fault Codes**:
 *    - Format varies by vendor
 *    - Example: "Err74.1" = Fault class 74, subcode 1 (No Sync)
 *
 * ## Architecture
 *
 * The FaultDetector class owns all per-slave fault state internally.
 * Network I/O is abstracted via the IFaultTransport interface, allowing
 * unit testing with a mock transport and supporting multiple independent
 * instances (no global state, no singletons).
 */

#pragma once

#include <cstdint>
#include <cstddef>
#include <functional>
#include <array>
#include <algorithm>
#include "tether/ethercat/TetherConfig.hpp"

namespace EtherCAT {

// ============================================================================
// Fault Code Definitions
// ============================================================================

/**
 * @brief AL Status Codes (EtherCAT standard, register 0x0134)
 */
enum class ALStatusCode : uint16_t {
    NoError                     = 0x0000,
    UnspecifiedError            = 0x0001,
    NoMemory                    = 0x0002,
    InvalidDeviceSetup          = 0x0003,
    ReservedCompatibility       = 0x0005,
    InvalidRequestedStateChange = 0x0011,
    UnknownRequestedState       = 0x0012,
    BootstrapNotSupported       = 0x0013,
    NoValidFirmware             = 0x0014,
    InvalidMailboxConfig        = 0x0015,
    InvalidMailboxConfigPreOp   = 0x0016,
    InvalidSyncManagerConfig    = 0x0017,
    NoValidInputs               = 0x0018,
    NoValidOutputs              = 0x0019,
    SynchronizationError        = 0x001A,
    SyncManagerWatchdog         = 0x001B,
    InvalidSyncManagerTypes     = 0x001C,
    InvalidOutputConfig         = 0x001D,
    InvalidInputConfig          = 0x001E,
    InvalidWatchdogConfig       = 0x001F,
    SlaveNeedsColdStart         = 0x0020,
    SlaveNeedsInit              = 0x0021,
    SlaveNeedsPreOp             = 0x0022,
    SlaveNeedsSafeOp            = 0x0023,
    InvalidInputMapping         = 0x0024,
    InvalidOutputMapping        = 0x0025,
    InconsistentSettings        = 0x0026,
    FreeRunNotSupported         = 0x0027,
    SyncModeNotSupported        = 0x0028,
    FreeRunNeeds3BufferMode     = 0x0029,
    BackgroundWatchdog          = 0x002A,
    NoValidInputsAndOutputs     = 0x002B,
    FatalSyncError              = 0x002C,
    NoSyncError                 = 0x002D,
    InvalidDCConfig             = 0x0030,
    InvalidDCSyncUnit           = 0x0031,
    InvalidDCCycleTime          = 0x0032,
    InvalidDCLatchConfig        = 0x0033,
    PLLError                    = 0x0034,
    DCSync1CycleTime            = 0x0035,
    MBoxEoE                     = 0x0041,
    MBoxCoE                     = 0x0042,
    MBoxFoE                     = 0x0043,
    MBoxSoE                     = 0x0044,
    MBoxVoE                     = 0x004F,
    EEPROMNoAccess              = 0x0050,
    EEPROMError                 = 0x0051,
    ExternalHardwareNotReady    = 0x0052,
    SlaveRestartedLocally       = 0x0060,
    DeviceIdUpdateError         = 0x0061,
    ApplicationControllerAvail  = 0x00F0,
    Unknown                     = 0xFFFF
};

/**
 * @brief Get human-readable name for AL Status Code
 */
const char* getALStatusCodeName(ALStatusCode code);
const char* getALStatusCodeName(uint16_t code);

/**
 * @brief CiA 402 Emergency Error Codes (object 0x603F)
 */
enum class CiA402ErrorCode : uint16_t {
    NoError                    = 0x0000,
    GenericError               = 0x1000,
    OverCurrent                = 0x2310,
    OverCurrentInternal        = 0x2311,
    OverCurrentOutputA         = 0x2312,
    OverCurrentOutputB         = 0x2313,
    OverVoltage                = 0x3110,
    OverVoltageSupply          = 0x3120,
    UnderVoltage               = 0x3210,
    UnderVoltageSupply         = 0x3220,
    OverTemperature            = 0x4210,
    OverTemperatureMotor       = 0x4310,
    OverTemperatureDrive       = 0x4311,
    SupplyVoltageFailure       = 0x5113,
    InternalSupplyFailed       = 0x5114,
    OutputStageProtection      = 0x5280,
    PositionLimitExceeded      = 0x5441,
    PositionSensorError        = 0x5442,
    EncoderError               = 0x5443,
    FollowingError             = 0x5A00,
    VelocityTooHigh            = 0x5B00,
    ExternalError              = 0x6100,
    SoftwareError              = 0x6300,
    SoftwareReset              = 0x6301,
    ObjectDictionaryError      = 0x6320,
    ObjectDictionaryMissing    = 0x6321,
    CANopenError               = 0x7000,
    PDOLengthError             = 0x8210,
    EmergencyBufferFull        = 0xFF00,
    CommWatchdogError          = 0xFF01,
    CommError                  = 0xFF02
};

/**
 * @brief Get human-readable name for CiA 402 Error Code
 */
const char* getCiA402ErrorCodeName(CiA402ErrorCode code);
const char* getCiA402ErrorCodeName(uint16_t code);

/**
 * @brief Manufacturer-specific fault code structure
 *
 * Many drives use proprietary fault code formats like "Err74.1"
 * This structure decodes such formats.
 */
struct ManufacturerFault {
    uint16_t raw_code;         ///< Raw fault code
    uint8_t class_code;        ///< Fault class (e.g., 74 in "Err74.1")
    uint8_t sub_code;          ///< Fault subcode (e.g., 1 in "Err74.1")
    const char* description;   ///< Human-readable description

    /**
     * @brief Parse manufacturer fault code
     *
     * @param raw_code Raw fault value from drive
     * @param vendor_id Vendor ID for manufacturer-specific parsing
     * @param product_code Product code for device-specific parsing
     * @return Parsed fault structure
     */
    static ManufacturerFault parse(uint16_t raw_code,
                                    uint32_t vendor_id = 0,
                                    uint32_t product_code = 0);

    /**
     * @brief Format fault code as string (e.g., "Err74.1")
     *
     * @param buffer Output buffer
     * @param buffer_size Buffer size
     * @return Number of characters written
     */
    size_t format(char* buffer, size_t buffer_size) const;
};

// ============================================================================
// Fault Detection Context
// ============================================================================

/**
 * @brief Per-slave fault state
 */
struct SlaveFaultState {
    // Current fault status
    bool has_fault;
    uint16_t al_status;            ///< Last AL_STATUS value
    ALStatusCode al_status_code;   ///< Last AL_STATUS_CODE value
    uint16_t error_code_603f;      ///< Last value of 0x603F
    ManufacturerFault mfr_fault;   ///< Decoded manufacturer fault

    // Timestamps
    uint64_t fault_detected_time;  ///< Timestamp when fault was first detected
    uint64_t last_poll_time;       ///< Last time fault registers were polled

    // Statistics
    uint32_t fault_count;          ///< Number of faults since startup
    uint32_t sync_error_count;     ///< Number of sync errors specifically
    uint32_t watchdog_count;       ///< Number of watchdog timeouts

    // Clear fault state
    void clear() {
        has_fault = false;
        al_status = 0;
        al_status_code = ALStatusCode::NoError;
        error_code_603f = 0;
        mfr_fault = {};
    }
};

/**
 * @brief Callback type for fault notifications
 */
using FaultCallback = std::function<void(uint16_t slave_index, const SlaveFaultState& state)>;

// ============================================================================
// Transport Abstraction
// ============================================================================

/**
 * @brief Abstract transport interface for fault detection I/O
 *
 * Implementations provide the actual register read/write operations.
 * A concrete implementation backed by Raw EtherCAT frames would wrap
 * ec_aprd / ec_apwr.  A mock implementation is used for unit tests.
 */
class IFaultTransport {
public:
    virtual ~IFaultTransport() = default;

    /**
     * @brief Read an ESC register from a slave
     *
     * @param slave_index Zero-based slave index
     * @param reg_addr    Register address (e.g. 0x0130 for AL_STATUS)
     * @param data        Destination buffer (host byte order on return)
     * @param size        Number of bytes to read
     * @return true on success
     */
    virtual bool readRegister(uint16_t slave_index, uint16_t reg_addr,
                              void* data, uint16_t size) = 0;

    /**
     * @brief Write an ESC register on a slave
     *
     * @param slave_index Zero-based slave index
     * @param reg_addr    Register address (e.g. 0x0120 for AL_CONTROL)
     * @param data        Source buffer (host byte order)
     * @param size        Number of bytes to write
     * @return true on success
     */
    virtual bool writeRegister(uint16_t slave_index, uint16_t reg_addr,
                               const void* data, uint16_t size) = 0;

    /**
     * @brief Get current monotonic timestamp in milliseconds
     */
    virtual uint64_t getTimestampMs() = 0;

    /**
     * @brief Blocking delay
     *
     * @param ms Milliseconds to wait
     */
    virtual void delayMs(uint32_t ms) = 0;
};

// ============================================================================
// FaultDetector — owns all per-slave fault state (no globals)
// ============================================================================

/**
 * @brief Instance-based fault detector.
 *
 * Each FaultDetector owns its own array of SlaveFaultState and its own
 * callback.  Multiple independent instances can co-exist.  Network I/O
 * is performed through the injected IFaultTransport.
 */
class FaultDetector {
public:
    static constexpr size_t kMaxSlaves = ECAT_FAULT_DETECTION_MAX_SLAVES;

    /**
     * @brief Construct a FaultDetector with the given transport.
     *
     * The transport reference must outlive the FaultDetector.
     */
    explicit FaultDetector(IFaultTransport& transport);
    ~FaultDetector() = default;

    // Non-copyable
    FaultDetector(const FaultDetector&) = delete;
    FaultDetector& operator=(const FaultDetector&) = delete;

    // Movable
    FaultDetector(FaultDetector&&) = default;
    FaultDetector& operator=(FaultDetector&&) = default;

    // ----- Lifecycle -----

    /**
     * @brief Initialize fault detection for the given number of slaves.
     *
     * Clears all fault state.  Slave count is clamped to kMaxSlaves.
     * Subsequent calls while already initialized return true (idempotent).
     *
     * @param slave_count Number of slaves to monitor
     * @return true on success
     */
    bool init(uint16_t slave_count);

    /**
     * @brief Shut down fault detection, clearing all state.
     */
    void shutdown();

    /** @brief Check whether init() has been called. */
    bool isInitialized() const { return initialized_; }

    /** @brief Number of slaves currently being monitored. */
    uint16_t slaveCount() const { return slave_count_; }

    // ----- Polling -----

    /**
     * @brief Poll fault status from a single slave via the transport.
     *
     * Reads AL_STATUS (0x0130) and AL_STATUS_CODE (0x0134).
     * Updates internal state, increments counters, and fires the
     * callback on a new fault transition.
     *
     * @param slave_index Slave to poll
     * @return Updated fault state (also stored internally)
     */
    SlaveFaultState poll(uint16_t slave_index);

    /**
     * @brief Poll all configured slaves for faults.
     *
     * @return Number of slaves with active faults
     */
    uint16_t pollAll();

    // ----- Query -----

    /**
     * @brief Get the stored fault state for a slave.
     *
     * @param slave_index Slave index
     * @return Pointer to internal state, or nullptr if invalid/uninitialized
     */
    const SlaveFaultState* getState(uint16_t slave_index) const;

    /**
     * @brief Check if any slave has an active fault.
     */
    bool anyActive() const;

    // ----- Clear -----

    /**
     * @brief Attempt to clear a fault on a slave.
     *
     * Writes ACK to AL_CONTROL, waits, then re-polls.
     *
     * @param slave_index Slave to clear
     * @return true if fault was successfully cleared
     */
    bool clear(uint16_t slave_index);

    // ----- Callback -----

    /**
     * @brief Register a callback for fault notifications.
     *
     * Called whenever a previously-unfaulted slave transitions to faulted.
     */
    void setCallback(FaultCallback callback);

    // ----- Diagnostics -----

    /**
     * @brief Log comprehensive fault diagnostics for a slave.
     */
    void diagnose(uint16_t slave_index) const;

    /**
     * @brief Log detailed "No Sync" (Err74.1) diagnosis.
     */
    void diagnoseNoSync(uint16_t slave_index) const;

private:
    IFaultTransport& transport_;
    std::array<SlaveFaultState, kMaxSlaves> slave_faults_{};
    uint16_t slave_count_ = 0;
    bool initialized_ = false;
    FaultCallback fault_callback_;
};

// ============================================================================
// Free-function API (delegates to a FaultDetector instance)
// ============================================================================

/** @brief Initialize fault detection on the given detector. */
inline bool fault_init(FaultDetector& fd, uint16_t slave_count) {
    return fd.init(slave_count);
}

/** @brief Shut down fault detection on the given detector. */
inline void fault_shutdown(FaultDetector& fd) {
    fd.shutdown();
}

/** @brief Poll a single slave via the given detector. */
inline SlaveFaultState fault_poll(FaultDetector& fd, uint16_t slave_index) {
    return fd.poll(slave_index);
}

/** @brief Poll all slaves via the given detector. */
inline uint16_t fault_poll_all(FaultDetector& fd) {
    return fd.pollAll();
}

/** @brief Get stored state from the given detector. */
inline const SlaveFaultState* fault_get_state(FaultDetector& fd, uint16_t slave_index) {
    return fd.getState(slave_index);
}

/** @brief Check if any slave has an active fault in the given detector. */
inline bool fault_any_active(FaultDetector& fd) {
    return fd.anyActive();
}

/** @brief Clear a slave fault via the given detector. */
inline bool fault_clear(FaultDetector& fd, uint16_t slave_index) {
    return fd.clear(slave_index);
}

/** @brief Register a fault callback on the given detector. */
inline void fault_set_callback(FaultDetector& fd, FaultCallback callback) {
    fd.setCallback(std::move(callback));
}

/** @brief Log diagnostics via the given detector. */
inline void fault_diagnose(FaultDetector& fd, uint16_t slave_index) {
    fd.diagnose(slave_index);
}

/** @brief Log no-sync diagnostics via the given detector. */
inline void fault_diagnose_no_sync(FaultDetector& fd, uint16_t slave_index) {
    fd.diagnoseNoSync(slave_index);
}

// ============================================================================
// Utility Functions (stateless — no detector needed)
// ============================================================================

/**
 * @brief Check if AL_STATUS indicates an error
 *
 * @param al_status AL_STATUS register value
 * @return true if error flag (bit 4) is set
 */
inline bool al_status_has_error(uint16_t al_status) {
    return (al_status & 0x0010) != 0;
}

/**
 * @brief Extract state from AL_STATUS
 *
 * @param al_status AL_STATUS register value
 * @return State code (1=INIT, 2=PRE_OP, 4=SAFE_OP, 8=OP)
 */
inline uint8_t al_status_get_state(uint16_t al_status) {
    return static_cast<uint8_t>(al_status & 0x000F);
}

/**
 * @brief Get state name from AL_STATUS
 *
 * @param al_status AL_STATUS register value
 * @return Human-readable state name
 */
const char* al_status_get_state_name(uint16_t al_status);

} // namespace EtherCAT
