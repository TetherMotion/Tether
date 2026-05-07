/**
 * @file StateMachineLogger.cpp
 * @brief EtherCAT state machine logger implementation
 *
 * Implements IStateMachineLogger and SlaveStateTracker for tracking
 * EtherCAT slave state machine transitions.
 */

#include "hal/StateMachineLogger.hpp"
#include "hal/HALTypes.hpp"
#include "hal/IClock.hpp"

#include <mutex>
#include <map>
#include <vector>

namespace EtherCAT {
namespace HAL {

// ============================================================================
// StateMachineLoggerImpl Implementation
// ============================================================================

class StateMachineLoggerImpl : public IStateMachineLogger {
public:
    StateMachineLoggerImpl() = default;

    Error init(const StateMachineLoggerConfig& config) override {
        m_config = config;
        m_historySize = config.historySize;
        return Error::OK;
    }

    void recordTransition(const StateTransition& transition) override {
        std::lock_guard<std::mutex> lock(m_mutex);
        
        // Update stats
        m_stats.totalTransitions++;
        if (transition.isError()) m_stats.errorTransitions++;
        if (transition.isRecovery()) m_stats.recoveryTransitions++;
        if (transition.isDowngrade()) m_stats.downgradeTransitions++;
        
        // Store in history
        m_allHistory.push_back(transition);
        while (m_allHistory.size() > m_historySize) {
            m_allHistory.erase(m_allHistory.begin());
        }
        
        // Store in per-slave history
        auto& slaveHistory = m_slaveHistory[transition.slaveIndex];
        slaveHistory.push_back(transition);
        while (slaveHistory.size() > m_historySize) {
            slaveHistory.erase(slaveHistory.begin());
        }
        
        // Update current state
        m_currentStates[transition.slaveIndex] = transition.newState;
        
        // Call callback
        if (m_callback) {
            m_callback(transition);
        }
    }

    void setCallback(StateTransitionCallback callback) override {
        std::lock_guard<std::mutex> lock(m_mutex);
        m_callback = callback;
    }

    std::vector<StateTransition> getHistory(uint16_t slaveIndex) const override {
        std::lock_guard<std::mutex> lock(m_mutex);
        auto it = m_slaveHistory.find(slaveIndex);
        if (it != m_slaveHistory.end()) {
            return it->second;
        }
        return {};
    }

    std::vector<StateTransition> getAllHistory() const override {
        std::lock_guard<std::mutex> lock(m_mutex);
        return m_allHistory;
    }

    void clearHistory() override {
        std::lock_guard<std::mutex> lock(m_mutex);
        m_allHistory.clear();
        m_slaveHistory.clear();
    }

    ALState getCurrentState(uint16_t slaveIndex) const override {
        std::lock_guard<std::mutex> lock(m_mutex);
        auto it = m_currentStates.find(slaveIndex);
        if (it != m_currentStates.end()) {
            return it->second;
        }
        return ALState::Unknown;
    }

    size_t getSlaveCount() const override {
        std::lock_guard<std::mutex> lock(m_mutex);
        return m_currentStates.size();
    }

    Stats getStats() const override {
        std::lock_guard<std::mutex> lock(m_mutex);
        return m_stats;
    }

    void setLogLevel(LogLevel level) override {
        m_config.logLevel = level;
    }

private:
    mutable std::mutex m_mutex;
    StateMachineLoggerConfig m_config;
    size_t m_historySize = 100;
    std::vector<StateTransition> m_allHistory;
    std::map<uint16_t, std::vector<StateTransition>> m_slaveHistory;
    std::map<uint16_t, ALState> m_currentStates;
    StateTransitionCallback m_callback;
    Stats m_stats;
};

// ============================================================================
// IStateMachineLogger Convenience Method
// ============================================================================

void IStateMachineLogger::recordTransition(uint16_t slaveIndex, ALState previous, 
                                           ALState next, uint16_t statusCode,
                                           bool requested) {
    StateTransition t;
    t.slaveIndex = slaveIndex;
    t.configuredAddress = 0;
    t.previousState = previous;
    t.newState = next;
    t.alStatusCode = statusCode;
    t.timestamp = getSystemClock().nowMicros();
    t.requested = requested;
    recordTransition(t);
}

// ============================================================================
// SlaveStateTracker Implementation
// ============================================================================

SlaveStateTracker::SlaveStateTracker(uint16_t slaveIndex, uint16_t configuredAddress)
    : m_slaveIndex(slaveIndex)
    , m_configuredAddress(configuredAddress)
    , m_currentState(ALState::Unknown)
    , m_requestedState(ALState::Unknown)
    , m_lastStatusCode(0)
    , m_callback(nullptr)
{}

bool SlaveStateTracker::updateState(ALState state, uint16_t statusCode) {
    if (state == m_currentState && statusCode == m_lastStatusCode) {
        return false;  // No change
    }
    
    m_lastTransition.slaveIndex = m_slaveIndex;
    m_lastTransition.configuredAddress = m_configuredAddress;
    m_lastTransition.previousState = m_currentState;
    m_lastTransition.newState = state;
    m_lastTransition.alStatusCode = statusCode;
    m_lastTransition.timestamp = getSystemClock().nowMicros();
    m_lastTransition.requested = (state == m_requestedState);
    
    m_currentState = state;
    m_lastStatusCode = statusCode;
    
    if (m_callback) {
        m_callback(m_lastTransition);
    }
    
    return true;
}

void SlaveStateTracker::requestState(ALState state) {
    m_requestedState = state;
}

// ============================================================================
// AL Status Code to String
// ============================================================================

const char* alStatusCodeToString(uint16_t code) {
    switch (static_cast<ALStatusCode>(code)) {
        case ALStatusCode::NoError: return "No error";
        case ALStatusCode::UnspecifiedError: return "Unspecified error";
        case ALStatusCode::NoMemory: return "No memory";
        case ALStatusCode::InvalidDeviceSetup: return "Invalid device setup";
        case ALStatusCode::SIIEEPROMError: return "SII EEPROM error";
        case ALStatusCode::FirmwareMismatch: return "Firmware mismatch";
        case ALStatusCode::InvalidInputConfiguration: return "Invalid input configuration";
        case ALStatusCode::InvalidOutputConfiguration: return "Invalid output configuration";
        case ALStatusCode::InvalidSyncConfiguration: return "Invalid sync configuration";
        case ALStatusCode::InvalidDCConfiguration: return "Invalid DC configuration";
        default:
            if (code >= 0x8000) return "Vendor-specific error";
            return "Unknown error";
    }
}

// ============================================================================
// Factory Function
// ============================================================================

std::unique_ptr<IStateMachineLogger> createStateMachineLogger() {
    return std::make_unique<StateMachineLoggerImpl>();
}

} // namespace hal
} // namespace EtherCAT
