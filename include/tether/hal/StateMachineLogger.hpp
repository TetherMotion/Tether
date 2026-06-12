/**
 * @file StateMachineLogger.hpp
 * @brief EtherCAT state machine transition logging
 *
 * Provides logging and monitoring for EtherCAT slave state machine transitions.
 */

#pragma once

#include "hal/HALTypes.hpp"
#include "hal/ILogger.hpp"
#include <memory>
#include <functional>
#include <vector>
#include <string>
#include <magic_enum/magic_enum.hpp>

namespace EtherCAT {
namespace HAL {

// ============================================================================
// EtherCAT State Definitions
// ============================================================================

/**
 * @brief EtherCAT Application Layer states (ESM)
 */
enum class ALState : uint8_t {
    Unknown = 0x00,
    Init = 0x01,
    PreOp = 0x02,
    Bootstrap = 0x03,
    SafeOp = 0x04,
    Op = 0x08,
    // Error states (state + 0x10)
    InitError = 0x11,
    PreOpError = 0x12,
    BootstrapError = 0x13,
    SafeOpError = 0x14,
    OpError = 0x18,
};

/**
 * @brief Convert AL state to string
 */
inline std::string alStateToString(ALState state) {
    auto base = static_cast<ALState>(static_cast<uint8_t>(state) & 0x0F);
    auto name = magic_enum::enum_name(base);
    if (static_cast<uint8_t>(state) & 0x10) {
        return std::string(name) + "+Error";
    }
    return std::string(name);
}

/**
 * @brief Check if state has error flag
 */
inline bool alStateHasError(ALState state) {
    return (static_cast<uint8_t>(state) & 0x10) != 0;
}

/**
 * @brief Get base state (without error flag)
 */
inline ALState alStateBase(ALState state) {
    return static_cast<ALState>(static_cast<uint8_t>(state) & 0x0F);
}

// ============================================================================
// State Transition Event
// ============================================================================

/**
 * @brief State transition event information
 */
struct StateTransition {
    uint16_t slaveIndex;        ///< Slave index (0-based)
    uint16_t configuredAddress; ///< Configured address (if known)
    ALState previousState;      ///< Previous state
    ALState newState;           ///< New state
    uint16_t alStatusCode;      ///< AL Status Code (error details)
    Timestamp timestamp;        ///< Transition timestamp
    bool requested;             ///< Transition was requested (vs spontaneous)
    
    /**
     * @brief Check if transition represents an error
     */
    bool isError() const {
        return alStateHasError(newState) || alStatusCode != 0;
    }
    
    /**
     * @brief Check if transition is a recovery from error
     */
    bool isRecovery() const {
        return alStateHasError(previousState) && !alStateHasError(newState);
    }
    
    /**
     * @brief Check if state went down (towards Init)
     */
    bool isDowngrade() const {
        return static_cast<uint8_t>(alStateBase(newState)) < 
               static_cast<uint8_t>(alStateBase(previousState));
    }
};

// ============================================================================
// AL Status Codes
// ============================================================================

/**
 * @brief AL Status Code definitions
 */
enum class ALStatusCode : uint16_t {
    NoError = 0x0000,
    UnspecifiedError = 0x0001,
    NoMemory = 0x0002,
    InvalidDeviceSetup = 0x0003,
    // ... many more codes defined in ETG.1000.6
    SIIEEPROMError = 0x0006,
    FirmwareMismatch = 0x0007,
    InvalidInputConfiguration = 0x001D,
    InvalidOutputConfiguration = 0x001E,
    InvalidSyncConfiguration = 0x001F,
    InvalidDCConfiguration = 0x0020,
    // Vendor-specific: 0x8000-0xFFFF
};

/**
 * @brief Get description for AL Status Code
 */
const char* alStatusCodeToString(uint16_t code);

// ============================================================================
// State Machine Logger Interface
// ============================================================================

/**
 * @brief Callback type for state transitions
 */
using StateTransitionCallback = std::function<void(const StateTransition& transition)>;

/**
 * @brief State machine logger configuration
 */
struct StateMachineLoggerConfig {
    LogLevel logLevel = LogLevel::Info;  ///< Minimum level to log
    bool logAllTransitions = true;       ///< Log all transitions (not just errors)
    bool logTimestamps = true;           ///< Include timestamps
    bool includeStatusCodes = true;      ///< Include AL status codes
    size_t historySize = 100;            ///< Number of transitions to keep
};

/**
 * @brief State machine logger interface
 */
class IStateMachineLogger {
public:
    virtual ~IStateMachineLogger() = default;
    
    /**
     * @brief Initialize the logger
     */
    virtual Error init(const StateMachineLoggerConfig& config = {}) = 0;
    
    /**
     * @brief Record a state transition
     */
    virtual void recordTransition(const StateTransition& transition) = 0;
    
    /**
     * @brief Convenience method to record transition
     */
    void recordTransition(uint16_t slaveIndex, ALState previous, ALState next,
                          uint16_t statusCode = 0, bool requested = false);
    
    /**
     * @brief Set callback for transitions
     */
    virtual void setCallback(StateTransitionCallback callback) = 0;
    
    /**
     * @brief Get transition history for a slave
     */
    virtual std::vector<StateTransition> getHistory(uint16_t slaveIndex) const = 0;
    
    /**
     * @brief Get all recent transitions
     */
    virtual std::vector<StateTransition> getAllHistory() const = 0;
    
    /**
     * @brief Clear history
     */
    virtual void clearHistory() = 0;
    
    /**
     * @brief Get current state for a slave
     */
    virtual ALState getCurrentState(uint16_t slaveIndex) const = 0;
    
    /**
     * @brief Get number of slaves being tracked
     */
    virtual size_t getSlaveCount() const = 0;
    
    /**
     * @brief Statistics
     */
    struct Stats {
        uint64_t totalTransitions = 0;
        uint64_t errorTransitions = 0;
        uint64_t recoveryTransitions = 0;
        uint64_t downgradeTransitions = 0;
    };
    virtual Stats getStats() const = 0;
    
    /**
     * @brief Set log level
     */
    virtual void setLogLevel(LogLevel level) = 0;
};

// ============================================================================
// Slave State Tracker
// ============================================================================

/**
 * @brief Tracks state for individual slaves
 */
class SlaveStateTracker {
public:
    explicit SlaveStateTracker(uint16_t slaveIndex, uint16_t configuredAddress = 0);
    
    /**
     * @brief Update state (call periodically with polled state)
     * @param state Current state
     * @param statusCode AL status code
     * @return true if state changed
     */
    bool updateState(ALState state, uint16_t statusCode = 0);
    
    /**
     * @brief Request state change (master-initiated)
     */
    void requestState(ALState state);
    
    /**
     * @brief Get current state
     */
    ALState getCurrentState() const { return m_currentState; }
    
    /**
     * @brief Get last requested state
     */
    ALState getRequestedState() const { return m_requestedState; }
    
    /**
     * @brief Get last status code
     */
    uint16_t getLastStatusCode() const { return m_lastStatusCode; }
    
    /**
     * @brief Check if in error state
     */
    bool hasError() const { return alStateHasError(m_currentState); }
    
    /**
     * @brief Get slave index
     */
    uint16_t getSlaveIndex() const { return m_slaveIndex; }
    
    /**
     * @brief Get configured address
     */
    uint16_t getConfiguredAddress() const { return m_configuredAddress; }
    
    /**
     * @brief Set configured address
     */
    void setConfiguredAddress(uint16_t addr) { m_configuredAddress = addr; }
    
    /**
     * @brief Get last transition
     */
    const StateTransition& getLastTransition() const { return m_lastTransition; }
    
    /**
     * @brief Set transition callback
     */
    void setCallback(StateTransitionCallback callback) { m_callback = callback; }
    
private:
    uint16_t m_slaveIndex;
    uint16_t m_configuredAddress;
    ALState m_currentState;
    ALState m_requestedState;
    uint16_t m_lastStatusCode;
    StateTransition m_lastTransition;
    StateTransitionCallback m_callback;
};

// ============================================================================
// Factory Functions
// ============================================================================

/**
 * @brief Create state machine logger
 */
std::unique_ptr<IStateMachineLogger> createStateMachineLogger();

} // namespace HAL
} // namespace EtherCAT
