/**
 * @file FSoEMasterConnection.hpp
 * @brief FSoE (Fail-Safe over EtherCAT) Master Connection — redesigned
 *
 * Replaces FSoEConnection with a correct, thread-safe, master-driven
 * FSoE protocol implementation that fixes all identified issues:
 * - Unified CRC (shared table-based implementation with final XOR)
 * - Master-driven protocol phases (Session → Connection → Parameter → Data)
 * - Parameter exchange with CRC verification
 * - Phase timeouts for all states
 * - Recovery from FailSafe/Error states
 * - Strict sequence validation
 * - Data length validation
 * - Per-state command validation
 * - Thread-safe via recursive mutex
 */

#pragma once

#include "fsoe/FSoEDefs.hpp"
#include "fsoe/FSoECRC.hpp"
#include <cstdint>
#include <string>
#include <functional>
#include <array>
#include <span>
#include <vector>
#include <memory>
#include <mutex>
#include <random>

namespace FSoE {

class FSoESlave;

// ============================================================================
// Configuration
// ============================================================================

struct MasterConnectionConfig {
    uint16_t slave_addr = 0;
    uint16_t slave_safety_addr = 0;     // FSoE slave safety address
    uint16_t connection_id = 0;
    uint16_t master_addr = 0;
    uint16_t watchdog_timeout_ms = 100;
    uint16_t conn_timeout_ms = 1000;
    uint16_t session_timeout_ms = 5000;
    uint16_t recovery_delay_ms = 500;
    uint8_t  safety_level = SIL::SIL2;
    uint8_t  input_size = 0;
    uint8_t  output_size = 0;
    std::array<uint8_t, 8> fail_safe_values = {0};

    bool auto_recovery_enabled = true;
    bool auto_fail_safe_on_error = true;
};

struct MasterConnectionStatus {
    uint8_t  state = ConnectionState::Reset;
    uint16_t error_code = ErrorCode::NoError;
    uint16_t session_id = 0;
    uint8_t  sequence_number = 0;
    bool     data_valid = false;
    bool     fail_safe_active = false;
    uint32_t watchdog_counter = 0;
    uint64_t last_valid_frame_ms = 0;
    uint64_t state_entered_ms = 0;

    bool isOperational() const { return state == ConnectionState::Data; }
    bool isFailSafe() const { return state == ConnectionState::FailSafe || fail_safe_active; }
    bool hasError() const { return state == ConnectionState::Error || error_code != ErrorCode::NoError; }
};

// ============================================================================
// Callback Types
// ============================================================================

using StateChangeCallback = std::function<void(uint8_t old_state, uint8_t new_state)>;
using ErrorCallback = std::function<void(uint16_t error_code)>;
using FailSafeCallback = std::function<void()>;
using DataCallback = std::function<void(const uint8_t* data, size_t len)>;

// ============================================================================
// FSoEMasterConnection — master-side FSoE connection to a single slave
// ============================================================================

class FSoEMasterConnection {
public:
    explicit FSoEMasterConnection(const MasterConnectionConfig& config);
    ~FSoEMasterConnection();

    // --- Initialization ---
    bool initialize();
    bool isInitialized() const;
    const MasterConnectionConfig& getConfig() const;

    // --- Connection Control ---
    bool startConnection();
    bool resetConnection();
    bool requestSessionReset();
    void triggerFailSafe(uint16_t error_code = ErrorCode::ApplicationError);
    bool clearError();

    // --- State Machine ---
    bool processRxFrame(const uint8_t* data, size_t len);
    size_t prepareTxFrame(uint8_t* data, size_t max_len);
    void update(uint64_t current_time_ms);

    // --- Safe Data Access ---
    bool setSafeOutputs(const uint8_t* data, size_t len);
    bool writeOutputProcessData(std::span<const uint8_t> data);
    size_t getSafeInputs(uint8_t* data, size_t len) const;
    size_t readInputProcessData(std::span<uint8_t> data) const;
    std::vector<uint8_t> inputProcessData() const;
    std::vector<uint8_t> outputProcessData() const;

    // --- Simulation helper (for testing with FSoESlave) ---
    bool exchangeWith(FSoESlave& slave, uint64_t current_time_ms);

    bool areSafeInputsValid() const;

    // --- Bit-level Safe I/O ---
    bool getSafeInputBit(uint8_t bit_index) const;
    bool setSafeOutputBit(uint8_t bit_index, bool value);
    uint8_t getSafeInputByte(uint8_t byte_index) const;
    bool setSafeOutputByte(uint8_t byte_index, uint8_t value);

    // --- Status & Diagnostics ---
    const MasterConnectionStatus& getStatus() const;
    uint8_t getState() const;
    uint16_t getErrorCode() const;
    bool isOperational() const;
    bool isFailSafe() const;
    const ConnectionStats& getStats() const;
    void resetStats();
    std::string getDiagnostics() const;

    // --- Callbacks ---
    void setStateChangeCallback(StateChangeCallback callback);
    void setErrorCallback(ErrorCallback callback);
    void setFailSafeCallback(FailSafeCallback callback);
    void setDataCallback(DataCallback callback);

private:
    // State machine handlers
    void handleSessionState(const uint8_t* data, size_t len);
    void handleConnectionState(const uint8_t* data, size_t len);
    void handleParameterState(const uint8_t* data, size_t len);
    void handleDataState(const uint8_t* data, size_t len);
    void handleFailSafeState(const uint8_t* data, size_t len);

    // Frame building
    size_t buildSessionResetFrame(uint8_t* data, size_t max_len);
    size_t buildConnectionFrame(uint8_t* data, size_t max_len);
    size_t buildParameterFrame(uint8_t* data, size_t max_len);
    size_t buildDataFrame(uint8_t* data, size_t max_len);
    size_t buildFailSafeFrame(uint8_t* data, size_t max_len);

    // Frame validation
    bool validateCRC(const uint8_t* data, size_t len) const;
    bool validateSequence(uint8_t seq);
    bool validateConnectionID(uint16_t conn_id) const;
    uint16_t extractConnectionID(const uint8_t* data, size_t len) const;

    // State transitions
    void transitionTo(uint8_t new_state);
    void handleError(uint16_t error_code);
    void checkWatchdog(uint64_t current_time_ms);
    void checkPhaseTimeout(uint64_t current_time_ms);
    void attemptAutoRecovery(uint64_t current_time_ms);

    // Parameter CRC computation
    uint16_t computeParameterCRC() const;

    MasterConnectionConfig config_;
    MasterConnectionStatus status_;
    ConnectionStats stats_;

    bool initialized_ = false;
    uint64_t current_time_ms_ = 0;

    std::array<uint8_t, 16> safe_inputs_{};
    std::array<uint8_t, 16> safe_outputs_{};
    uint8_t rx_sequence_ = 0;
    uint8_t tx_sequence_ = 0;

    // Parameter exchange state
    uint8_t current_param_index_ = 0;
    uint16_t parameter_crc_ = 0;

    // Recovery state
    uint64_t fail_safe_entered_ms_ = 0;

    // Thread safety
    mutable std::recursive_mutex mutex_;

    // Callbacks
    StateChangeCallback state_change_callback_;
    ErrorCallback error_callback_;
    FailSafeCallback fail_safe_callback_;
    DataCallback data_callback_;

    // Session ID generation
    std::mt19937 rng_{std::random_device{}()};
};

} // namespace FSoE
