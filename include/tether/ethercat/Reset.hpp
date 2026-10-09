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
#include "tether/ethercat/FaultDetection.hpp"  // canonical ALStatusCode enum
#include "tether/ethercat/ResetTypes.hpp"

namespace EtherCAT {

// Forward declarations
class Master;

// Forward declaration for SDO injection
namespace CoE { class CoEManager; }

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

} // namespace EtherCAT
