/**
 * @file ResetTypes.hpp
 * @brief Slave reset level/state enums, results and policies.
 *
 * Split out of Reset.hpp.
 */

#pragma once

#include <cstdint>
#include <functional>
#include <string>

#include "tether/ethercat/FaultDetection.hpp"

namespace EtherCAT {

class SlaveResetController;

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

// NOTE: AL status codes live in the canonical enum EtherCAT::ALStatusCode
// (tether/ethercat/FaultDetection.hpp). The former constants namespace here
// was removed to eliminate the duplicate definition; the enum provides
// compatibility aliases for the old names (e.g. ALStatusCode::SyncError).

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
