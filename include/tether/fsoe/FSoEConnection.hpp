/**
 * @file FSoEConnection.hpp
 * @brief FSoE (Fail-Safe over EtherCAT) Connection Controller
 *
 * Provides FSoE black-channel safety communication over EtherCAT.
 *
 * Features:
 * - FSoE connection state machine
 * - CRC-16 verification
 * - Watchdog monitoring
 * - Session management
 * - Safe I/O data exchange
 * - Fail-safe state handling
 * - Safety diagnostics
 */

#pragma once

#include "fsoe/FSoEDefs.hpp"
#include <cstdint>
#include <string>
#include <functional>
#include <array>
#include <span>
#include <vector>
#include <memory>
#include <mutex>

namespace FSoE {

class FSoESlave;

// ============================================================================
// Configuration
// ============================================================================

/**
 * @brief FSoE connection configuration
 */
struct ConnectionConfig {
    uint16_t slave_addr = 0;            // EtherCAT slave address
    uint16_t connection_id = 0;         // FSoE connection ID
    uint16_t master_addr = 0;           // FSoE master safety address
    uint16_t watchdog_timeout_ms = 100; // Watchdog timeout
    uint16_t conn_timeout_ms = 1000;    // Connection timeout
    uint8_t  safety_level = SIL::SIL2;  // Required safety level
    uint8_t  input_size = 0;            // Safety input data size (bytes)
    uint8_t  output_size = 0;           // Safety output data size (bytes)
    std::array<uint8_t, 8> fail_safe_values = {0};  // Fail-safe output values
};

/**
 * @brief FSoE connection state
 */
struct ConnectionStatus {
    uint8_t  state = ConnectionState::Reset;
    uint16_t error_code = ErrorCode::NoError;
    uint16_t session_id = 0;
    uint8_t  sequence_number = 0;
    bool     data_valid = false;
    bool     fail_safe_active = false;
    uint32_t watchdog_counter = 0;
    uint64_t last_valid_frame_ms = 0;
    
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
// FSoE Connection Class
// ============================================================================

class FSoEConnection {
public:
    explicit FSoEConnection(const ConnectionConfig& config);
    ~FSoEConnection();
    
    // ========================================================================
    // Initialization
    // ========================================================================
    
    /**
     * @brief Initialize FSoE connection
     */
    bool initialize();
    
    /**
     * @brief Check if connection is initialized
     */
    bool isInitialized() const;
    
    /**
     * @brief Get connection configuration
     */
    const ConnectionConfig& getConfig() const { return config_; }
    
    // ========================================================================
    // Connection Control
    // ========================================================================
    
    /**
     * @brief Start connection establishment
     */
    bool startConnection();
    
    /**
     * @brief Reset connection
     */
    bool resetConnection();
    
    /**
     * @brief Request session reset
     */
    bool requestSessionReset();
    
    /**
     * @brief Trigger fail-safe state
     */
    void triggerFailSafe(uint16_t error_code = ErrorCode::ApplicationError);
    
    /**
     * @brief Clear error and attempt recovery
     */
    bool clearError();
    
    // ========================================================================
    // State Machine
    // ========================================================================
    
    /**
     * @brief Process incoming FSoE frame
     * @param data Incoming frame data
     * @param len Length of frame
     * @return true if frame was valid
     */
    bool processRxFrame(const uint8_t* data, size_t len);
    
    /**
     * @brief Prepare outgoing FSoE frame
     * @param data Buffer for frame
     * @param max_len Maximum buffer size
     * @return Actual frame size, 0 on error
     */
    size_t prepareTxFrame(uint8_t* data, size_t max_len);
    
    /**
     * @brief Update state machine (call periodically)
     * @param current_time_ms Current time in milliseconds
     */
    void update(uint64_t current_time_ms);
    
    // ========================================================================
    // Safe Data Access
    // ========================================================================
    
    /**
     * @brief Set safe output data
     * @param data Output data
     * @param len Length (must match configured output_size)
     */
    bool setSafeOutputs(const uint8_t* data, size_t len);

    /**
     * @brief Write the raw master-to-slave process image.
     */
    bool writeOutputProcessData(std::span<const uint8_t> data);
    
    /**
     * @brief Get safe input data
     * @param data Buffer for input data
     * @param len Buffer length (must be >= input_size)
     * @return Actual data length, 0 if no valid data
     */
    size_t getSafeInputs(uint8_t* data, size_t len) const;

    /**
     * @brief Read the raw slave-to-master process image.
     */
    size_t readInputProcessData(std::span<uint8_t> data) const;

    /**
     * @brief Return the last valid slave-to-master process image.
     */
    std::vector<uint8_t> inputProcessData() const;

    /**
     * @brief Return the currently configured master-to-slave process image.
     */
    std::vector<uint8_t> outputProcessData() const;

    /**
     * @brief Exchange one complete FSoE cycle against a simulated slave.
     */
    bool exchangeWith(FSoESlave& slave, uint64_t current_time_ms);
    
    /**
     * @brief Check if safe inputs are valid
     */
    bool areSafeInputsValid() const;
    
    /**
     * @brief Get fail-safe output values
     */
    const std::array<uint8_t, 8>& getFailSafeValues() const { return config_.fail_safe_values; }
    
    // ========================================================================
    // Bit-level Safe I/O Access
    // ========================================================================
    
    /**
     * @brief Get safe input bit
     */
    bool getSafeInputBit(uint8_t bit_index) const;
    
    /**
     * @brief Set safe output bit
     */
    bool setSafeOutputBit(uint8_t bit_index, bool value);
    
    /**
     * @brief Get safe input byte
     */
    uint8_t getSafeInputByte(uint8_t byte_index) const;
    
    /**
     * @brief Set safe output byte
     */
    bool setSafeOutputByte(uint8_t byte_index, uint8_t value);
    
    // ========================================================================
    // Status and Diagnostics
    // ========================================================================
    
    /**
     * @brief Get connection status
     */
    const ConnectionStatus& getStatus() const { return status_; }
    
    /**
     * @brief Get current connection state
     */
    uint8_t getState() const;
    
    /**
     * @brief Get last error code
     */
    uint16_t getErrorCode() const;
    
    /**
     * @brief Check if connection is operational
     */
    bool isOperational() const;
    
    /**
     * @brief Check if in fail-safe state
     */
    bool isFailSafe() const;
    
    /**
     * @brief Get connection statistics
     */
    const ConnectionStats& getStats() const { return stats_; }
    
    /**
     * @brief Reset statistics
     */
    void resetStats();
    
    /**
     * @brief Get diagnostic string
     */
    std::string getDiagnostics() const;
    
    // ========================================================================
    // Callbacks
    // ========================================================================
    
    void setStateChangeCallback(StateChangeCallback callback);
    void setErrorCallback(ErrorCallback callback);
    void setFailSafeCallback(FailSafeCallback callback);
    void setDataCallback(DataCallback callback);

private:
    // State machine handlers
    void handleResetState();
    void handleSessionState(const uint8_t* data, size_t len);
    void handleConnectionState(const uint8_t* data, size_t len);
    void handleParameterState(const uint8_t* data, size_t len);
    void handleDataState(const uint8_t* data, size_t len);
    
    // Frame building
    size_t buildSessionResetFrame(uint8_t* data, size_t max_len);
    size_t buildConnectionFrame(uint8_t* data, size_t max_len);
    size_t buildDataFrame(uint8_t* data, size_t max_len);
    size_t buildFailSafeFrame(uint8_t* data, size_t max_len);
    
    // Frame validation
    bool validateFrame(const uint8_t* data, size_t len);
    bool validateCRC(const uint8_t* data, size_t len);
    bool validateSequence(uint8_t seq);
    bool validateConnectionID(uint16_t conn_id);
    
    // State transitions
    void transitionTo(uint8_t new_state);
    void handleError(uint16_t error_code);
    void checkWatchdog(uint64_t current_time_ms);
    
    ConnectionConfig config_;
    ConnectionStatus status_;
    ConnectionStats stats_;
    
    bool initialized_;
    uint64_t current_time_ms_;
    
    // Safe data buffers
    std::array<uint8_t, 16> safe_inputs_;
    std::array<uint8_t, 16> safe_outputs_;
    uint8_t rx_sequence_;
    uint8_t tx_sequence_;
    
    // Callbacks
    StateChangeCallback state_change_callback_;
    ErrorCallback error_callback_;
    FailSafeCallback fail_safe_callback_;
    DataCallback data_callback_;

    mutable std::recursive_mutex mutex_;
};

// ============================================================================
// FSoE Master Class (manages multiple connections)
// ============================================================================

class FSoEMaster {
public:
    FSoEMaster();
    ~FSoEMaster();
    
    /**
     * @brief Add FSoE connection
     */
    bool addConnection(const ConnectionConfig& config);
    
    /**
     * @brief Remove FSoE connection by connection ID
     */
    bool removeConnection(uint16_t connection_id);
    
    /**
     * @brief Get connection by ID
     */
    FSoEConnection* getConnection(uint16_t connection_id);
    
    /**
     * @brief Get connection by slave address
     */
    FSoEConnection* getConnectionBySlaveAddr(uint16_t slave_addr);
    
    /**
     * @brief Start all connections
     */
    bool startAll();
    
    /**
     * @brief Reset all connections
     */
    void resetAll();
    
    /**
     * @brief Update all connections
     */
    void update(uint64_t current_time_ms);
    
    /**
     * @brief Check if all connections are operational
     */
    bool allOperational() const;
    
    /**
     * @brief Check if any connection is in fail-safe
     */
    bool anyFailSafe() const;
    
    /**
     * @brief Get number of connections
     */
    size_t getConnectionCount() const;
    
    /**
     * @brief Get master diagnostics
     */
    std::string getDiagnostics() const;

private:
    bool allOperationalUnsafe() const;
    bool anyFailSafeUnsafe() const;
    std::vector<std::unique_ptr<FSoEConnection>> connections_;
    mutable std::mutex mutex_;
};

} // namespace FSoE
