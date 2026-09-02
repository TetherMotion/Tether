/**
 * @file Reset.hpp
 * @brief Comprehensive EtherCAT Slave Reset Mechanisms
 * 
 * @details
 * This module provides a complete implementation of slave reset functionality
 * covering all reset levels and protocol-specific methods defined in:
 * - EtherCAT Technology Group specifications (ETG.1000)
 * - CiA 301 CANopen Application Layer (NMT reset)
 * - CiA 402 Drives and Motion Control (fault reset)
 * - CiA 406 Encoders (position reset)
 * - Device-specific vendor resets
 * 
 * ## Reset Level Hierarchy
 * 
 * ```
 * ┌────────────────────────────────────────────────────────────────────────┐
 * │                        RESET LEVEL HIERARCHY                          │
 * ├────────────────────────────────────────────────────────────────────────┤
 * │ Level 0: Soft Reset (Application)                                     │
 * │    └─ Clears application-level errors, counters, buffers              │
 * │    └─ Preserves configuration and state machine position              │
 * │    └─ Fastest recovery, minimal disruption                            │
 * ├────────────────────────────────────────────────────────────────────────┤
 * │ Level 1: Communication Reset                                          │
 * │    └─ Resets communication parameters to power-on defaults            │
 * │    └─ PDO/SDO configuration cleared                                   │
 * │    └─ Slave remains in current EtherCAT state                         │
 * ├────────────────────────────────────────────────────────────────────────┤
 * │ Level 2: Application Reset                                            │
 * │    └─ Full application layer restart                                  │
 * │    └─ Device profile state machines reset                             │
 * │    └─ Returns to PRE-OP state typically                               │
 * ├────────────────────────────────────────────────────────────────────────┤
 * │ Level 3: State Machine Reset (ESM)                                    │
 * │    └─ EtherCAT State Machine forced to INIT                           │
 * │    └─ All Sync Managers disabled                                      │
 * │    └─ Requires full re-initialization sequence                        │
 * ├────────────────────────────────────────────────────────────────────────┤
 * │ Level 4: ESC Hardware Reset                                           │
 * │    └─ EtherCAT Slave Controller hardware reset                        │
 * │    └─ Triggers INIT with ESC reset flag                               │
 * │    └─ May require physical access or vendor command                   │
 * ├────────────────────────────────────────────────────────────────────────┤
 * │ Level 5: Full Hardware Reset (Power Cycle)                            │
 * │    └─ Complete device power cycle                                     │
 * │    └─ Requires external power management                              │
 * │    └─ Most disruptive, used as last resort                            │
 * └────────────────────────────────────────────────────────────────────────┘
 * ```
 * 
 * ## Protocol-Specific Reset Methods
 * 
 * ### CiA 402 Drive Reset
 * - Fault Reset (Controlword bit 7 rising edge)
 * - Quick Stop → Enable transition
 * - Halt/Resume control
 * 
 * ### CiA 301 NMT Commands
 * - Reset Node (all application parameters)
 * - Reset Communication (communication parameters only)
 * 
 * ### EtherCAT ESM (State Machine)
 * - AL Control writes for state transitions
 * - Error Acknowledge for error recovery
 * 
 * @see ETG.1000 EtherCAT Technology Group Specification
 * @see CiA 301 CANopen Application Layer
 * @see CiA 402 Drives and Motion Control
 */

#pragma once

#include <cstdint>
#include <cstddef>
#include <functional>
#include <optional>
#include <string>
#include <vector>

#include "tether/platform/EspCompat.hpp"

namespace EtherCAT {

// Forward declarations
class Master;

// Forward declaration for SDO injection
namespace CoE { class CoEManager; }

// ============================================================================
// Reset Level Enumeration
// ============================================================================

/**
 * @brief Reset severity levels
 * 
 * Defines the hierarchy of reset operations from least to most disruptive.
 */
enum class ResetLevel : uint8_t {
    /// Level 0: Soft/Application reset - clears errors, preserves config
    SoftReset = 0,
    
    /// Level 1: Communication parameter reset - resets PDO/SDO config
    CommunicationReset = 1,
    
    /// Level 2: Application layer reset - restarts device profile
    ApplicationReset = 2,
    
    /// Level 3: State machine reset - forces INIT state
    StateMachineReset = 3,
    
    /// Level 4: ESC hardware reset - resets EtherCAT controller
    ESCHardwareReset = 4,
    
    /// Level 5: Full hardware reset - power cycle
    HardwareReset = 5,
};

/**
 * @brief Get human-readable name for reset level
 */
const char* getResetLevelName(ResetLevel level);

/**
 * @brief Get detailed description for reset level
 */
const char* getResetLevelDescription(ResetLevel level);

// ============================================================================
// EtherCAT State Machine (ESM) Definitions
// ============================================================================

/**
 * @brief EtherCAT Application Layer states
 */
enum class ALState : uint8_t {
    Init         = 0x01,  ///< Initialization state
    PreOp        = 0x02,  ///< Pre-Operational state
    Bootstrap    = 0x03,  ///< Bootstrap state (firmware update)
    SafeOp       = 0x04,  ///< Safe-Operational state
    Op           = 0x08,  ///< Operational state
    
    /// State with error flag (OR with actual state)
    ErrorFlag    = 0x10,
};

/**
 * @brief AL Control register bit definitions
 */
namespace ALControl {
    constexpr uint16_t StateMask       = 0x000F;  ///< State bits
    constexpr uint16_t AckError        = 0x0010;  ///< Acknowledge/clear error
    constexpr uint16_t RequestId       = 0x0020;  ///< Request slave ID
    constexpr uint16_t ESCReset        = 0x0040;  ///< ESC hardware reset (specific ESCs)
}

/**
 * @brief AL Status Code categories
 */
namespace ALStatusCode {
    // No error
    constexpr uint16_t NoError                = 0x0000;
    
    // General errors (0x0001-0x000F)
    constexpr uint16_t UnspecifiedError       = 0x0001;
    constexpr uint16_t NoMemory               = 0x0002;
    
    // State transition errors (0x0011-0x001F)
    constexpr uint16_t InvalidStateChange     = 0x0011;
    constexpr uint16_t UnknownStateRequested  = 0x0012;
    constexpr uint16_t BootNotSupported       = 0x0013;
    constexpr uint16_t NoValidFirmware        = 0x0014;
    constexpr uint16_t InvalidMailboxConfig   = 0x0015;
    constexpr uint16_t InvalidMailboxConfig2  = 0x0016;
    constexpr uint16_t InvalidSMConfig        = 0x0017;
    constexpr uint16_t NoValidInputs          = 0x0018;
    constexpr uint16_t NoValidOutputs         = 0x0019;
    constexpr uint16_t SyncError              = 0x001A;
    constexpr uint16_t SMWatchdog             = 0x001B;
    constexpr uint16_t InvalidSMTypes         = 0x001C;
    constexpr uint16_t InvalidOutputConfig    = 0x001D;
    constexpr uint16_t InvalidInputConfig     = 0x001E;
    constexpr uint16_t InvalidWatchdogConfig  = 0x001F;
    
    // Application errors (0x0020-0x003F)
    constexpr uint16_t SlaveNeedsColdStart    = 0x0020;
    constexpr uint16_t SlaveNeedsInit         = 0x0021;
    constexpr uint16_t SlaveNeedsPreOp        = 0x0022;
    constexpr uint16_t SlaveNeedsSafeOp       = 0x0023;
    constexpr uint16_t InvalidInputMapping    = 0x0024;
    constexpr uint16_t InvalidOutputMapping   = 0x0025;
    constexpr uint16_t InconsistentSettings   = 0x0026;
    constexpr uint16_t FreeRunNotSupported    = 0x0027;
    constexpr uint16_t SyncNotSupported       = 0x0028;
    constexpr uint16_t FreeRunNeeds3BufferMode = 0x0029;
    constexpr uint16_t BackgroundWatchdog     = 0x002A;
    constexpr uint16_t NoValidInputsOutputs   = 0x002B;
    constexpr uint16_t FatalSyncError         = 0x002C;
    constexpr uint16_t NoSyncError            = 0x002D;
    constexpr uint16_t InvalidDCCycleTime     = 0x0030;
    constexpr uint16_t InvalidDCSync0Time     = 0x0031;
    constexpr uint16_t InvalidDCSync1Time     = 0x0032;
    constexpr uint16_t MBXAoeError            = 0x0033;
    constexpr uint16_t MBXEoeError            = 0x0034;
    constexpr uint16_t MBXCoeError            = 0x0035;
    constexpr uint16_t MBXFoeError            = 0x0036;
    constexpr uint16_t MBXSoeError            = 0x0037;
    constexpr uint16_t MBXVoeError            = 0x003E;
    
    // Vendor specific (0x8000-0xFFFF)
    constexpr uint16_t VendorSpecificStart    = 0x8000;
}

/**
 * @brief Get human-readable name for AL Status Code
 */
const char* getALStatusCodeName(uint16_t code);

// ============================================================================
// CiA 301 NMT Reset Commands
// ============================================================================

/**
 * @brief CiA 301 NMT Commands via CoE
 * 
 * In EtherCAT, NMT commands are sent via CoE (CANopen over EtherCAT)
 * using the 0x0000:0x00 broadcast SDO or specific object writes.
 */
enum class NMTCommand : uint8_t {
    StartNode           = 0x01,  ///< Start remote node (enter Operational)
    StopNode            = 0x02,  ///< Stop remote node (enter Stopped)
    EnterPreOp          = 0x80,  ///< Enter Pre-Operational
    ResetNode           = 0x81,  ///< Reset Node (application reset)
    ResetCommunication  = 0x82,  ///< Reset Communication only
};

/**
 * @brief CiA 301 Object Dictionary indexes for reset
 */
namespace CiA301Reset {
    /// Store parameters (0x1010) - Write 0x65766173 ("save") to store
    constexpr uint16_t StoreParameters     = 0x1010;
    constexpr uint32_t StoreSignature      = 0x65766173; // "save" in ASCII
    
    /// Restore parameters (0x1011) - Write 0x64616F6C ("load") to restore
    constexpr uint16_t RestoreParameters   = 0x1011;
    constexpr uint32_t RestoreSignature    = 0x64616F6C; // "load" in ASCII
    
    /// Subindexes for store/restore
    constexpr uint8_t AllParameters        = 0x01;
    constexpr uint8_t CommunicationParams  = 0x02;
    constexpr uint8_t ApplicationParams    = 0x03;
    constexpr uint8_t ManufacturerParams   = 0x04;
}

// ============================================================================
// CiA 402 Drive-Specific Reset
// ============================================================================

/**
 * @brief CiA 402 Controlword bits for reset operations
 */
namespace CiA402Reset {
    /// Fault Reset (bit 7) - Rising edge clears fault
    constexpr uint16_t FaultReset         = 0x0080;
    
    /// Halt (bit 8) - Stops motion
    constexpr uint16_t Halt               = 0x0100;
    
    /// Quick Stop disable (bit 2 = 0 triggers quick stop)
    constexpr uint16_t QuickStopActive    = 0x0000;
    constexpr uint16_t QuickStopInactive  = 0x0004;
    
    /// Enable Operation (bit 3)
    constexpr uint16_t EnableOperation    = 0x0008;
    
    /// Switch On (bits 0-3 control power stage)
    constexpr uint16_t SwitchOn           = 0x0001;
    constexpr uint16_t EnableVoltage      = 0x0002;
}

/**
 * @brief CiA 402 Drive states for reset context
 */
enum class CiA402State : uint8_t {
    NotReadyToSwitchOn    = 0x00,
    SwitchOnDisabled      = 0x40,
    ReadyToSwitchOn       = 0x21,
    SwitchedOn            = 0x23,
    OperationEnabled      = 0x27,
    QuickStopActive       = 0x07,
    FaultReactionActive   = 0x0F,
    Fault                 = 0x08,
};

// ============================================================================
// Reset Operation Results
// ============================================================================

/**
 * @brief Result of a reset operation
 */
struct ResetResult {
    bool success;                  ///< Overall success flag
    ResetLevel requested_level;    ///< Level that was requested
    ResetLevel achieved_level;     ///< Level actually achieved
    uint16_t al_status;            ///< Final AL Status after reset
    uint16_t al_status_code;       ///< AL Status Code (error details)
    uint32_t duration_us;          ///< Time taken for reset in microseconds
    std::string error_message;     ///< Human-readable error if failed
    
    /// Check if reset fully completed at requested level
    bool isComplete() const { return success && (achieved_level == requested_level); }
    
    /// Check if partial reset achieved (less than requested but non-zero)
    bool isPartial() const { return success && (achieved_level != requested_level); }
};

/**
 * @brief Callback for reset progress notification
 * 
 * @param stage Current stage description
 * @param progress Progress percentage (0-100)
 * @param slave_addr Slave address being reset (0xFFFF for broadcast)
 */
using ResetProgressCallback = std::function<void(const char* stage, uint8_t progress, uint16_t slave_addr)>;

// ============================================================================
// Slave Reset Controller
// ============================================================================

/**
 * @brief Comprehensive slave reset controller
 * 
 * Provides methods for all reset levels and protocol-specific resets.
 * 
 * @code
 * SlaveResetController reset(0);  // For slave at position 0
 * 
 * // Simple fault reset for CiA 402 drive
 * auto result = reset.faultReset();
 * if (!result.success) {
 *     TETHER_LOGE("RESET", "Fault reset failed: {}", result.error_message.c_str());
 * }
 * 
 * // Full state machine reset
 * result = reset.resetToLevel(ResetLevel::StateMachineReset);
 * 
 * // Progressive reset (tries each level until success)
 * result = reset.progressiveReset();
 * @endcode
 */
class SlaveResetController {
public:
    /**
     * @brief Construct reset controller for specific slave
     * @param coe CoEManager instance for SDO access (per-slave)
     * @param slave_index Slave index (must match CoEManager's slave index)
     */
    SlaveResetController(CoE::CoEManager& coe, uint16_t slave_index);
    
    ~SlaveResetController() = default;
    
    // ========================================================================
    // General Reset Methods
    // ========================================================================
    
    /**
     * @brief Reset slave to specific level
     * @param level Desired reset level
     * @param timeout_ms Timeout for reset operation
     * @return Reset result
     */
    ResetResult resetToLevel(ResetLevel level, uint32_t timeout_ms = 5000);
    
    /**
     * @brief Progressive reset - tries each level until success
     * 
     * Starts with soft reset and escalates to more severe levels if needed.
     * Useful for recovering from unknown error states.
     * 
     * @param max_level Maximum level to attempt
     * @param timeout_per_level_ms Timeout for each level attempt
     * @return Reset result (achieved_level shows what worked)
     */
    ResetResult progressiveReset(ResetLevel max_level = ResetLevel::ESCHardwareReset,
                                  uint32_t timeout_per_level_ms = 2000);
    
    /**
     * @brief Emergency stop and reset
     * 
     * Immediately stops all motion/output, then performs reset.
     * Implements quickest safe stop sequence.
     * 
     * @return Reset result
     */
    ResetResult emergencyStopAndReset();
    
    // ========================================================================
    // EtherCAT State Machine (ESM) Reset Methods
    // ========================================================================
    
    /**
     * @brief Acknowledge and clear error state
     * 
     * Writes AL Control with ACK bit set to clear error flag in AL Status.
     * 
     * @return true if error was cleared
     */
    bool acknowledgeError();
    
    /**
     * @brief Read current AL Status and Status Code
     * @param[out] status Current AL Status value
     * @param[out] status_code Current AL Status Code
     * @return true if read succeeded
     */
    bool readALStatus(uint16_t& status, uint16_t& status_code);
    
    /**
     * @brief Force state machine to INIT
     * @param timeout_ms Timeout for state change
     * @return true if INIT state reached
     */
    bool forceToInit(uint32_t timeout_ms = 1000);
    
    /**
     * @brief Request ESC hardware reset
     * 
     * Writes ESC reset bit to AL Control. Not supported by all ESCs.
     * 
     * @return true if reset initiated
     */
    bool requestESCReset();
    
    /**
     * @brief Transition to specific AL state
     * @param target_state Desired state
     * @param timeout_ms Timeout
     * @return true if target state reached
     */
    bool transitionToState(ALState target_state, uint32_t timeout_ms = 1000);
    
    /**
     * @brief Full re-initialization sequence
     * 
     * Performs complete INIT → PRE-OP → SAFE-OP → OP sequence with
     * proper mailbox and PDO configuration at each step.
     * 
     * @param to_op If true, goes all the way to OP state
     * @return Reset result
     */
    ResetResult fullReinitialize(bool to_op = true);
    
    // ========================================================================
    // CiA 301 NMT Reset Methods  
    // ========================================================================
    
    /**
     * @brief Send NMT Reset Node command via CoE
     * 
     * Resets the entire application layer including device profile.
     * 
     * @return true if command sent successfully
     */
    bool nmtResetNode();
    
    /**
     * @brief Send NMT Reset Communication command via CoE
     * 
     * Resets only communication parameters, preserving application state.
     * 
     * @return true if command sent successfully
     */
    bool nmtResetCommunication();
    
    /**
     * @brief Restore default parameters via object 0x1011
     * @param subindex What to restore (1=all, 2=comm, 3=app)
     * @return true if restore initiated
     */
    bool restoreDefaultParameters(uint8_t subindex = CiA301Reset::AllParameters);
    
    /**
     * @brief Clear error history (0x1003 Pre-defined Error Field)
     * @return true if cleared
     */
    bool clearErrorHistory();
    
    // ========================================================================
    // CiA 402 Drive Reset Methods
    // ========================================================================
    
    /**
     * @brief Perform CiA 402 fault reset
     * 
     * Creates rising edge on Controlword bit 7 to clear fault condition.
     * 
     * @param target_state State to transition to after fault clear
     * @return Reset result
     */
    ResetResult faultReset(CiA402State target_state = CiA402State::SwitchOnDisabled);
    
    /**
     * @brief Perform quick stop
     * 
     * Commands immediate controlled stop via Quick Stop bit.
     * 
     * @return true if quick stop initiated
     */
    bool quickStop();
    
    /**
     * @brief Halt motion (CiA 402 halt bit)
     * @return true if halt commanded
     */
    bool halt();
    
    /**
     * @brief Resume from halt
     * @return true if resume commanded
     */
    bool resumeFromHalt();
    
    /**
     * @brief Complete drive disable sequence
     * 
     * Transitions through: Op Enabled → Switched On → Ready → Switch On Disabled
     * 
     * @return Reset result
     */
    ResetResult disableDrive();
    
    /**
     * @brief Complete drive enable sequence
     * 
     * Transitions through: Switch On Disabled → Ready → Switched On → Op Enabled
     * 
     * @return Reset result
     */
    ResetResult enableDrive();
    
    /**
     * @brief Read current CiA 402 Statusword
     * @param[out] statusword Current statusword
     * @return true if read succeeded
     */
    bool readStatusword(uint16_t& statusword);
    
    /**
     * @brief Write CiA 402 Controlword
     * @param controlword Value to write
     * @return true if write succeeded
     */
    bool writeControlword(uint16_t controlword);
    
    /**
     * @brief Clear drive-specific error codes
     * 
     * Reads and clears error register (0x603F) if supported.
     * 
     * @return true if cleared
     */
    bool clearDriveErrors();
    
    // ========================================================================
    // Sync Manager and Watchdog Reset
    // ========================================================================
    
    /**
     * @brief Reset Sync Manager watchdog
     * 
     * Disables and re-enables Sync Managers to clear watchdog state.
     * 
     * @return true if reset succeeded
     */
    bool resetSyncManagerWatchdog();
    
    /**
     * @brief Clear PDI (Process Data Interface) watchdog
     * @return true if cleared
     */
    bool clearPDIWatchdog();
    
    /**
     * @brief Reconfigure Sync Managers
     * 
     * Re-reads configuration from SII and reconfigures all SMs.
     * 
     * @return true if reconfiguration succeeded
     */
    bool reconfigureSyncManagers();
    
    // ========================================================================
    // Distributed Clock Reset
    // ========================================================================
    
    /**
     * @brief Reset DC synchronization
     * 
     * Clears DC errors and restarts synchronization.
     * 
     * @return true if DC reset succeeded
     */
    bool resetDistributedClock();
    
    /**
     * @brief Clear DC sync errors
     * @return true if cleared
     */
    bool clearDCSyncErrors();
    
    // ========================================================================
    // Vendor-Specific Reset Methods
    // ========================================================================
    
    /**
     * @brief Execute vendor-specific reset command
     * 
     * Writes to vendor-specific object dictionary entry.
     * 
     * @param index Object dictionary index
     * @param subindex Subindex
     * @param data Data to write
     * @param data_len Data length
     * @return true if command sent
     */
    bool vendorSpecificReset(uint16_t index, uint8_t subindex,
                              const uint8_t* data, size_t data_len);
    
    /**
     * @brief Execute reset via VoE (Vendor over EtherCAT)
     * @param voe_data Vendor-specific data packet
     * @param voe_len Data length
     * @return true if sent
     */
    bool voeReset(const uint8_t* voe_data, size_t voe_len);
    
    // ========================================================================
    // Status and Diagnostics
    // ========================================================================
    
    /**
     * @brief Get last reset result
     */
    const ResetResult& getLastResult() const { return last_result_; }
    
    /**
     * @brief Set progress callback
     */
    void setProgressCallback(ResetProgressCallback callback);
    
    /**
     * @brief Get reset attempt count
     */
    uint32_t getResetAttemptCount() const { return reset_attempt_count_; }
    
    /**
     * @brief Get successful reset count
     */
    uint32_t getSuccessfulResetCount() const { return successful_reset_count_; }
    
    /**
     * @brief Check if slave is currently in error state
     */
    bool isInErrorState();
    
    /**
     * @brief Get detailed error description
     */
    std::string getErrorDescription();
    
private:
    CoE::CoEManager& m_coe; ///< SDO manager for SDO access
    const uint16_t slave_index_;   ///< Slave index
    ResetResult last_result_;       ///< Last reset result
    ResetProgressCallback progress_callback_;
    uint32_t reset_attempt_count_{0};
    uint32_t successful_reset_count_{0};
    
    // Internal helper methods
    bool writeALControl(uint16_t value);
    bool waitForState(ALState target, uint32_t timeout_ms);
    void reportProgress(const char* stage, uint8_t progress);
    bool sdoWrite(uint16_t index, uint8_t sub, const void* data, size_t len);
    bool sdoRead(uint16_t index, uint8_t sub, void* data, size_t len, size_t* out_len);
};

// ============================================================================
// Broadcast/Network-Wide Reset Functions
// ============================================================================
// TODO: Network-wide functions need to be redesigned for per-slave CoEManager architecture
// These functions require access to Master class which creates circular dependency
// Temporarily disabled - will be implemented in a separate module

/*
std::vector<ResetResult> resetAllSlaves(EtherCAT::Master& master, ResetLevel level, uint32_t timeout_ms = 10000);
uint16_t broadcastErrorAcknowledge(EtherCAT::Master& master);
uint16_t broadcastStateTransition(EtherCAT::Master& master, ALState target_state);
bool networkEmergencyStop(EtherCAT::Master& master);
bool reinitializeNetwork(EtherCAT::Master& master, bool to_op = true);
*/

// ============================================================================
// Reset Policies
// ============================================================================

/**
 * @brief Reset policy configuration
 */
struct ResetPolicy {
    /// Maximum automatic reset attempts before giving up
    uint8_t max_auto_attempts{3};
    
    /// Delay between reset attempts (ms)
    uint16_t retry_delay_ms{100};
    
    /// Whether to escalate reset level on failure
    bool escalate_on_failure{true};
    
    /// Starting reset level
    ResetLevel starting_level{ResetLevel::SoftReset};
    
    /// Maximum reset level to attempt
    ResetLevel max_level{ResetLevel::ApplicationReset};
    
    /// Whether to automatically retry on transient errors
    bool auto_retry_transient{true};
    
    /// For CiA 402: automatically return to op enabled after fault reset
    bool auto_reenable_drive{false};
    
    /// Callback for policy decisions
    std::function<bool(const ResetResult&, uint8_t attempt)> should_continue;
};

/**
 * @brief Apply reset according to policy
 * @param controller Reset controller
 * @param policy Reset policy
 * @return Final reset result
 */
ResetResult applyResetPolicy(SlaveResetController& controller, const ResetPolicy& policy);

} // namespace EtherCAT
