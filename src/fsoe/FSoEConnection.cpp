/**
 * @file FSoEConnection.cpp
 * @brief FSoE (Fail-Safe over EtherCAT) Connection Implementation
 */

#include "fsoe/FSoEConnection.hpp"
#include "fsoe/FSoESlave.hpp"
#include <cstring>
#include <algorithm>

namespace FSoE {

// ============================================================================
// FSoEConnection Implementation
// ============================================================================

FSoEConnection::FSoEConnection(const ConnectionConfig& config)
    : config_(config)
    , status_{}
    , stats_{}
    , initialized_(false)
    , current_time_ms_(0)
    , safe_inputs_{}
    , safe_outputs_{}
    , rx_sequence_(0x0F)
    , tx_sequence_(0)
    , state_change_callback_(nullptr)
    , error_callback_(nullptr)
    , fail_safe_callback_(nullptr)
    , data_callback_(nullptr)
{
}

FSoEConnection::~FSoEConnection() = default;

bool FSoEConnection::isInitialized() const
{
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    return initialized_;
}

// ============================================================================
// Initialization
// ============================================================================

bool FSoEConnection::initialize()
{
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    if (config_.input_size > 16 || config_.output_size > 16) {
        return false;  // Buffer size exceeded
    }

    // Validate watchdog timeout range (ETG.5100 / Object 0x6791: 50-60000 ms)
    if (config_.watchdog_timeout_ms < Limits::WatchdogTimeoutMin ||
        config_.watchdog_timeout_ms > Limits::WatchdogTimeoutMax) {
        return false;
    }

    // Validate safety address range (1-65535, 0 is invalid)
    if (config_.slave_safety_addr < Limits::SafetyAddressMin) {
        return false;
    }

    // Initialize fail-safe values in output buffer
    std::copy(config_.fail_safe_values.begin(), 
              config_.fail_safe_values.begin() + config_.output_size,
              safe_outputs_.begin());
    
    status_.state = ConnectionState::Reset;
    status_.error_code = ErrorCode::NoError;
    status_.session_id = 0;
    status_.sequence_number = 0;
    status_.data_valid = false;
    status_.fail_safe_active = false;
    
    resetStats();
    
    initialized_ = true;
    return true;
}

// ============================================================================
// Connection Control
// ============================================================================

bool FSoEConnection::startConnection()
{
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    if (!initialized_) return false;
    
    status_.state = ConnectionState::Reset;
    return true;
}

bool FSoEConnection::resetConnection()
{
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    if (!initialized_) return false;
    
    transitionTo(ConnectionState::Reset);
    status_.error_code = ErrorCode::NoError;
    status_.data_valid = false;
    rx_sequence_ = 0x0F;
    tx_sequence_ = 0;
    
    stats_.reset_events++;
    
    return true;
}

bool FSoEConnection::requestSessionReset()
{
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    if (!initialized_) return false;
    
    // Generate session ID using monotonic counter (not frame count)
    static uint16_t session_counter = 0;
    session_counter++;
    if (session_counter == 0) {
        session_counter = 1;  // Never 0
    }
    status_.session_id = session_counter;
    transitionTo(ConnectionState::Session);
    
    return true;
}

void FSoEConnection::triggerFailSafe(uint16_t error_code)
{
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    const bool was_fail_safe = status_.fail_safe_active;
    status_.fail_safe_active = true;
    status_.data_valid = false;

    if (error_code != ErrorCode::NoError) {
        status_.error_code = error_code;
    }

    // Apply fail-safe values
    std::copy(config_.fail_safe_values.begin(),
              config_.fail_safe_values.begin() + config_.output_size,
              safe_outputs_.begin());

    // Stay in Data state — fail-safe is a Data sub-mode using cmd 0x08 (ETG.5100)

    if (!was_fail_safe && fail_safe_callback_) {
        fail_safe_callback_();
    }
}

bool FSoEConnection::clearError()
{
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    if (status_.state != ConnectionState::Error &&
        !status_.fail_safe_active) {
        return false;
    }
    
    status_.error_code = ErrorCode::NoError;
    status_.fail_safe_active = false;
    
    return resetConnection();
}

// ============================================================================
// State Machine
// ============================================================================

bool FSoEConnection::processRxFrame(const uint8_t* data, size_t len)
{
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    if (!initialized_ || !data || len < MIN_FSOE_FRAME_SIZE) {
        return false;
    }
    
    stats_.frames_received++;
    
    // Parse frame with interleaved CRC verification
    uint8_t cmd = 0;
    uint8_t frame_data[MAX_SAFE_DATA_SIZE] = {0};
    size_t data_len = 0;
    uint16_t conn_id = 0;
    
    if (!CRC::parseFSoEFrame(data, len, cmd, frame_data, data_len, conn_id)) {
        stats_.crc_errors++;
        handleError(ErrorCode::CRCError);
        return false;
    }
    
    // Validate connection ID (after connection is established)
    if (status_.state == ConnectionState::Connection ||
        status_.state == ConnectionState::Parameter ||
        status_.state == ConnectionState::Data) {
        if (!validateConnectionID(conn_id)) {
            handleError(ErrorCode::ConnectionIDError);
            return false;
        }
    }
    
    // Update watchdog
    status_.last_valid_frame_ms = current_time_ms_;
    
    // Check for fail-safe response from slave in Data state
    if (cmd == Command::FailSafeData &&
        status_.state == ConnectionState::Data &&
        !status_.fail_safe_active) {
        // Extract slave error code from fail-safe response
        // Fail-safe data layout: [fail_safe_inputs...] [error_code_lo] [error_code_hi]
        if (data_len >= 2) {
            size_t error_offset = data_len - 2;
            uint16_t slave_error = static_cast<uint16_t>(
                frame_data[error_offset] | (frame_data[error_offset + 1] << 8));
            handleError(slave_error);
        } else {
            handleError(ErrorCode::ApplicationError);
        }
        return true;
    }
    
    // Store parsed data for state handlers
    // Use a member buffer to pass data to handlers
    static thread_local uint8_t rx_data[MAX_SAFE_DATA_SIZE];
    static thread_local size_t rx_data_len;
    static thread_local uint8_t rx_cmd;
    static thread_local uint16_t rx_conn_id;
    std::copy(frame_data, frame_data + data_len, rx_data);
    rx_data_len = data_len;
    rx_cmd = cmd;
    rx_conn_id = conn_id;
    
    // Process based on current state
    switch (status_.state) {
        case ConnectionState::Reset:
            handleResetState(rx_data, rx_data_len, rx_cmd);
            break;
            
        case ConnectionState::Session:
            handleSessionState(rx_data, rx_data_len, rx_cmd);
            break;
            
        case ConnectionState::Connection:
            handleConnectionState(rx_data, rx_data_len, rx_cmd);
            break;
            
        case ConnectionState::Parameter:
            handleParameterState(rx_data, rx_data_len, rx_cmd);
            break;
            
        case ConnectionState::Data:
            handleDataState(rx_data, rx_data_len, rx_cmd);
            break;
            
        case ConnectionState::Error:
            // Don't process frames in error state
            break;
    }
    
    return true;
}

size_t FSoEConnection::prepareTxFrame(uint8_t* data, size_t max_len)
{
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    if (!initialized_ || !data) return 0;
    
    size_t len = 0;
    
    switch (status_.state) {
        case ConnectionState::Reset:
        case ConnectionState::Session:
            len = buildSessionResetFrame(data, max_len);
            break;
            
        case ConnectionState::Connection:
            len = buildConnectionFrame(data, max_len);
            break;
            
        case ConnectionState::Parameter:
            len = buildParameterFrame(data, max_len);
            break;
            
        case ConnectionState::Data:
            if (status_.fail_safe_active) {
                len = buildFailSafeFrame(data, max_len);
            } else {
                len = buildDataFrame(data, max_len);
            }
            break;
            
        case ConnectionState::Error:
            // No frames in error state
            break;
    }
    
    if (len > 0) {
        stats_.frames_sent++;
    }
    
    return len;
}

void FSoEConnection::update(uint64_t current_time_ms)
{
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    current_time_ms_ = current_time_ms;
    stats_.uptime_ms = current_time_ms;
    
    // Check watchdog
    checkWatchdog(current_time_ms);
}

// ============================================================================
// State Handlers
// ============================================================================

void FSoEConnection::handleResetState(const uint8_t* data, size_t data_len, uint8_t cmd)
{
    (void)data;
    (void)data_len;
    
    if (cmd == Command::Session) {
        // Slave already responded with Session — advance to Connection
        transitionTo(ConnectionState::Connection);
    } else {
        // No valid session response — initiate session establishment
        requestSessionReset();
    }
}

void FSoEConnection::handleSessionState(const uint8_t* data, size_t data_len, uint8_t cmd)
{
    (void)data;
    (void)data_len;
    
    if (cmd == Command::Session) {
        // Session acknowledged, move to connection
        transitionTo(ConnectionState::Connection);
    }
}

void FSoEConnection::handleConnectionState(const uint8_t* data, size_t data_len, uint8_t cmd)
{
    if (cmd == Command::Connection) {
        // Validate slave's safety address from connection response safe data
        // Expected layout: [safety_addr_lo] [safety_addr_hi] [safety_level] [reserved]
        if (data_len >= 2) {
            uint16_t slave_safety_addr = static_cast<uint16_t>(data[0]) |
                                          (static_cast<uint16_t>(data[1]) << 8);
            if (slave_safety_addr != 0 && slave_safety_addr != config_.slave_safety_addr) {
                handleError(ErrorCode::ConnectionIDError);
                return;
            }
            // Validate SIL level if present (byte 2)
            if (data_len >= 3) {
                uint8_t slave_sil = data[2];
                if (slave_sil < config_.safety_level) {
                    handleError(ErrorCode::ApplicationError);
                    return;
                }
            }
        }
        if (config_.input_size > 0 || config_.output_size > 0) {
            transitionTo(ConnectionState::Parameter);
        } else {
            transitionTo(ConnectionState::Data);
        }
    } else if (cmd == Command::Parameter) {
        // Some simulated slaves acknowledge the connection by immediately
        // advancing to the parameter phase and returning a parameter response.
        if (config_.input_size > 0 || config_.output_size > 0) {
            transitionTo(ConnectionState::Parameter);
        } else {
            transitionTo(ConnectionState::Data);
        }
    }
}

void FSoEConnection::handleParameterState(const uint8_t* data, size_t data_len, uint8_t cmd)
{
    (void)data;
    (void)data_len;
    
    if (cmd == Command::Parameter) {
        transitionTo(ConnectionState::Data);
    } else if (cmd == Command::ProcessData) {
        transitionTo(ConnectionState::Data);
        // Re-process as data — need to call handleDataState with the parsed data
        // This shouldn't normally happen but handle gracefully
    }
}

void FSoEConnection::handleDataState(const uint8_t* data, size_t data_len, uint8_t cmd)
{
    if (cmd != Command::ProcessData) {
        return;
    }
    
    // Clear fail-safe if we receive normal process data
    if (status_.fail_safe_active) {
        status_.fail_safe_active = false;
    }
    
    // Extract safety input data
    if (data_len >= config_.input_size) {
        std::copy(data, data + config_.input_size, safe_inputs_.begin());
        status_.data_valid = true;
        
        if (data_callback_) {
            data_callback_(safe_inputs_.data(), config_.input_size);
        }
    }
}

// ============================================================================
// Frame Building
// ============================================================================

size_t FSoEConnection::buildSessionResetFrame(uint8_t* data, size_t max_len)
{
    // Session reset: CMD + session_id (2B) + ConnID (2B)
    uint8_t payload[2];
    payload[0] = status_.session_id & 0xFF;
    payload[1] = (status_.session_id >> 8) & 0xFF;
    size_t needed = fsoeFrameSize(2);
    if (max_len < needed) return 0;
    return CRC::buildFSoEFrame(data, Command::Session, payload, 2, config_.connection_id);
}

size_t FSoEConnection::buildConnectionFrame(uint8_t* data, size_t max_len)
{
    // Connection frame: CMD + slave_safety_addr (2B) + safety_level (1B) + param_crc (1B) + ConnID (2B)
    uint8_t payload[4];
    payload[0] = config_.slave_safety_addr & 0xFF;
    payload[1] = (config_.slave_safety_addr >> 8) & 0xFF;
    payload[2] = config_.safety_level;
    
    // Calculate parameter CRC from safety-critical configuration
    uint8_t param_buf[6];
    param_buf[0] = config_.watchdog_timeout_ms & 0xFF;
    param_buf[1] = (config_.watchdog_timeout_ms >> 8) & 0xFF;
    param_buf[2] = config_.safety_level;
    param_buf[3] = config_.input_size;
    param_buf[4] = config_.output_size;
    param_buf[5] = 0;  // Reserved
    uint16_t param_crc = CRC::calculateFSoECRC(param_buf, sizeof(param_buf));
    payload[3] = static_cast<uint8_t>(param_crc & 0xFF);
    
    size_t needed = fsoeFrameSize(4);
    if (max_len < needed) return 0;
    return CRC::buildFSoEFrame(data, Command::Connection, payload, 4, config_.connection_id);
}

size_t FSoEConnection::buildParameterFrame(uint8_t* data, size_t max_len)
{
    // Parameter frame: CMD + watchdog (2B) + safety_level (1B) + input_size (1B) + output_size (1B) + reserved (1B) + ConnID (2B)
    uint8_t payload[6];
    payload[0] = config_.watchdog_timeout_ms & 0xFF;
    payload[1] = (config_.watchdog_timeout_ms >> 8) & 0xFF;
    payload[2] = config_.safety_level;
    payload[3] = config_.input_size;
    payload[4] = config_.output_size;
    payload[5] = 0;  // Reserved
    
    size_t needed = fsoeFrameSize(6);
    if (max_len < needed) return 0;
    return CRC::buildFSoEFrame(data, Command::Parameter, payload, 6, config_.connection_id);
}

size_t FSoEConnection::buildDataFrame(uint8_t* data, size_t max_len)
{
    size_t needed = fsoeFrameSize(config_.output_size);
    if (max_len < needed) return 0;
    return CRC::buildFSoEFrame(data, Command::ProcessData,
                               safe_outputs_.data(), config_.output_size,
                               config_.connection_id);
}

size_t FSoEConnection::buildFailSafeFrame(uint8_t* data, size_t max_len)
{
    // Fail-safe frame: CMD + fail_safe_values (output_size) + error_code (2B) + ConnID (2B)
    uint8_t payload[MAX_SAFE_DATA_SIZE] = {0};
    size_t payload_len = config_.output_size + 2;  // fail-safe values + error code
    
    std::copy(config_.fail_safe_values.begin(),
              config_.fail_safe_values.begin() + config_.output_size,
              payload);
    payload[config_.output_size] = status_.error_code & 0xFF;
    payload[config_.output_size + 1] = (status_.error_code >> 8) & 0xFF;
    
    size_t needed = fsoeFrameSize(payload_len);
    if (max_len < needed) return 0;
    return CRC::buildFSoEFrame(data, Command::FailSafeData,
                               payload, payload_len, config_.connection_id);
}

// ============================================================================
// Frame Validation
// ============================================================================

bool FSoEConnection::validateFrame(const uint8_t* data, size_t len)
{
    if (!data || len < MIN_FSOE_FRAME_SIZE) {
        return false;
    }
    
    uint8_t cmd;
    size_t data_len;
    uint16_t conn_id;
    return CRC::parseFSoEFrame(data, len, cmd, nullptr, data_len, conn_id);
}

bool FSoEConnection::validateCRC(const uint8_t* data, size_t len)
{
    uint8_t cmd;
    size_t data_len;
    uint16_t conn_id;
    return CRC::parseFSoEFrame(data, len, cmd, nullptr, data_len, conn_id);
}

bool FSoEConnection::validateSequence(uint8_t seq)
{
    // ETG.5100 does not define a sequence number field.
    // Frame integrity is ensured via CRC + watchdog.
    (void)seq;
    return true;
}

bool FSoEConnection::validateConnectionID(uint16_t conn_id)
{
    return conn_id == config_.connection_id;
}

// ============================================================================
// State Transitions
// ============================================================================

void FSoEConnection::transitionTo(uint8_t new_state)
{
    if (new_state == status_.state) return;
    
    uint8_t old_state = status_.state;
    status_.state = new_state;
    
    // Clear data valid when leaving data state
    if (old_state == ConnectionState::Data && new_state != ConnectionState::Data) {
        status_.data_valid = false;
    }
    
    if (state_change_callback_) {
        state_change_callback_(old_state, new_state);
    }
}

void FSoEConnection::handleError(uint16_t error_code)
{
    status_.error_code = error_code;
    
    // Consolidated error handling: trigger fail-safe, then transition to Error
    if (!status_.fail_safe_active) {
        triggerFailSafe(error_code);
    }
    
    transitionTo(ConnectionState::Error);
    
    if (error_callback_) {
        error_callback_(error_code);
    }
}

void FSoEConnection::checkWatchdog(uint64_t current_time_ms)
{
    // Watchdog active in all communication states (not Reset/Error)
    if (status_.state == ConnectionState::Reset ||
        status_.state == ConnectionState::Error) {
        return;
    }
    
    uint64_t elapsed = current_time_ms - status_.last_valid_frame_ms;
    
    if (elapsed > config_.watchdog_timeout_ms) {
        stats_.watchdog_events++;
        handleError(ErrorCode::WatchdogError);
    }
    
    status_.watchdog_counter = static_cast<uint32_t>(elapsed);
}

// ============================================================================
// Safe Data Access
// ============================================================================

bool FSoEConnection::setSafeOutputs(const uint8_t* data, size_t len)
{
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    if (!data || len != config_.output_size) return false;
    if (status_.isFailSafe()) return false;  // Don't allow writes in fail-safe
    
    std::copy(data, data + len, safe_outputs_.begin());
    return true;
}

bool FSoEConnection::writeOutputProcessData(std::span<const uint8_t> data)
{
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    return setSafeOutputs(data.data(), data.size());
}

size_t FSoEConnection::getSafeInputs(uint8_t* data, size_t len) const
{
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    if (!data || len < config_.input_size) return 0;
    if (!status_.data_valid) return 0;
    
    std::copy(safe_inputs_.begin(), safe_inputs_.begin() + config_.input_size, data);
    return config_.input_size;
}

size_t FSoEConnection::readInputProcessData(std::span<uint8_t> data) const
{
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    return getSafeInputs(data.data(), data.size());
}

std::vector<uint8_t> FSoEConnection::inputProcessData() const
{
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    std::vector<uint8_t> data(config_.input_size, 0);
    const size_t copied = getSafeInputs(data.data(), data.size());
    data.resize(copied);
    return data;
}

std::vector<uint8_t> FSoEConnection::outputProcessData() const
{
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    return std::vector<uint8_t>(safe_outputs_.begin(),
                                safe_outputs_.begin() + config_.output_size);
}

bool FSoEConnection::exchangeWith(FSoESlave& slave, uint64_t current_time_ms)
{
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    update(current_time_ms);
    slave.update(current_time_ms);

    std::array<uint8_t, 64> tx{};
    std::array<uint8_t, 64> rx{};

    const size_t tx_len = prepareTxFrame(tx.data(), tx.size());
    if (tx_len == 0) {
        return false;
    }

    if (!slave.processRxFrame(tx.data(), tx_len)) {
        return false;
    }

    const size_t rx_len = slave.prepareTxFrame(rx.data(), rx.size());
    if (rx_len == 0) {
        return false;
    }

    return processRxFrame(rx.data(), rx_len);
}

bool FSoEConnection::getSafeInputBit(uint8_t bit_index) const
{
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    if (!status_.data_valid) return false;
    
    uint8_t byte_idx = bit_index / 8;
    uint8_t bit_pos = bit_index % 8;
    
    if (byte_idx >= config_.input_size) return false;
    
    return (safe_inputs_[byte_idx] >> bit_pos) & 1;
}

bool FSoEConnection::setSafeOutputBit(uint8_t bit_index, bool value)
{
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    if (status_.isFailSafe()) return false;
    
    uint8_t byte_idx = bit_index / 8;
    uint8_t bit_pos = bit_index % 8;
    
    if (byte_idx >= config_.output_size) return false;
    
    if (value) {
        safe_outputs_[byte_idx] |= (1 << bit_pos);
    } else {
        safe_outputs_[byte_idx] &= ~(1 << bit_pos);
    }
    
    return true;
}

uint8_t FSoEConnection::getSafeInputByte(uint8_t byte_index) const
{
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    if (!status_.data_valid || byte_index >= config_.input_size) {
        return 0;
    }
    return safe_inputs_[byte_index];
}

bool FSoEConnection::setSafeOutputByte(uint8_t byte_index, uint8_t value)
{
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    if (status_.isFailSafe() || byte_index >= config_.output_size) {
        return false;
    }
    safe_outputs_[byte_index] = value;
    return true;
}

// ============================================================================
// Diagnostics
// ============================================================================

void FSoEConnection::resetStats()
{
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    stats_.reset();
}

std::string FSoEConnection::getDiagnostics() const
{
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    std::string diag;
    
    diag += "FSoE Connection Diagnostics:\n";
    diag += "  Connection ID: " + std::to_string(config_.connection_id) + "\n";
    diag += "  Slave Address: " + std::to_string(config_.slave_addr) + "\n";
    diag += "  State: " + std::to_string(status_.state) + "\n";
    diag += "  Operational: " + std::string(status_.isOperational() ? "Yes" : "No") + "\n";
    diag += "  Fail-Safe: " + std::string(status_.isFailSafe() ? "Yes" : "No") + "\n";
    diag += "  Data Valid: " + std::string(status_.data_valid ? "Yes" : "No") + "\n";
    
    if (status_.hasError()) {
        diag += "  ERROR: 0x" + std::to_string(status_.error_code) + "\n";
    }
    
    diag += "  Session ID: " + std::to_string(status_.session_id) + "\n";
    diag += "  Watchdog: " + std::to_string(status_.watchdog_counter) + " ms\n";
    
    diag += "\nStatistics:\n";
    diag += "  Frames Sent: " + std::to_string(stats_.frames_sent) + "\n";
    diag += "  Frames Received: " + std::to_string(stats_.frames_received) + "\n";
    diag += "  CRC Errors: " + std::to_string(stats_.crc_errors) + "\n";
    diag += "  Sequence Errors: " + std::to_string(stats_.sequence_errors) + "\n";
    diag += "  Watchdog Events: " + std::to_string(stats_.watchdog_events) + "\n";
    diag += "  Reset Events: " + std::to_string(stats_.reset_events) + "\n";
    
    return diag;
}

// ============================================================================
// Callbacks
// ============================================================================

void FSoEConnection::setStateChangeCallback(StateChangeCallback callback)
{
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    state_change_callback_ = std::move(callback);
}

void FSoEConnection::setErrorCallback(ErrorCallback callback)
{
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    error_callback_ = std::move(callback);
}

void FSoEConnection::setFailSafeCallback(FailSafeCallback callback)
{
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    fail_safe_callback_ = std::move(callback);
}

void FSoEConnection::setDataCallback(DataCallback callback)
{
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    data_callback_ = std::move(callback);
}

// ============================================================================
// FSoEMaster Implementation
// ============================================================================

uint8_t FSoEConnection::getState() const
{
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    return status_.state;
}

uint16_t FSoEConnection::getErrorCode() const
{
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    return status_.error_code;
}

bool FSoEConnection::isOperational() const
{
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    return status_.isOperational();
}

bool FSoEConnection::isFailSafe() const
{
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    return status_.isFailSafe();
}

bool FSoEConnection::areSafeInputsValid() const
{
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    return status_.data_valid && status_.isOperational();
}

FSoEMaster::FSoEMaster() = default;
FSoEMaster::~FSoEMaster() = default;

bool FSoEMaster::addConnection(const ConnectionConfig& config)
{
    std::lock_guard<std::mutex> lock(mutex_);
    // Check for duplicate connection ID
    for (const auto& conn : connections_) {
        if (conn->getConfig().connection_id == config.connection_id) {
            return false;
        }
    }
    
    auto connection = std::make_unique<FSoEConnection>(config);
    if (!connection->initialize()) {
        return false;
    }
    
    connections_.push_back(std::move(connection));
    return true;
}

bool FSoEMaster::removeConnection(uint16_t connection_id)
{
    std::lock_guard<std::mutex> lock(mutex_);
    auto it = std::remove_if(connections_.begin(), connections_.end(),
        [connection_id](const std::unique_ptr<FSoEConnection>& conn) {
            return conn->getConfig().connection_id == connection_id;
        });
    
    if (it != connections_.end()) {
        connections_.erase(it, connections_.end());
        return true;
    }
    
    return false;
}

FSoEConnection* FSoEMaster::getConnection(uint16_t connection_id)
{
    std::lock_guard<std::mutex> lock(mutex_);
    for (auto& conn : connections_) {
        if (conn->getConfig().connection_id == connection_id) {
            return conn.get();
        }
    }
    return nullptr;
}

FSoEConnection* FSoEMaster::getConnectionBySlaveAddr(uint16_t slave_addr)
{
    std::lock_guard<std::mutex> lock(mutex_);
    for (auto& conn : connections_) {
        if (conn->getConfig().slave_addr == slave_addr) {
            return conn.get();
        }
    }
    return nullptr;
}

bool FSoEMaster::startAll()
{
    std::lock_guard<std::mutex> lock(mutex_);
    bool all_started = true;
    for (auto& conn : connections_) {
        if (!conn->startConnection()) {
            all_started = false;
        }
    }
    return all_started;
}

void FSoEMaster::resetAll()
{
    std::lock_guard<std::mutex> lock(mutex_);
    for (auto& conn : connections_) {
        conn->resetConnection();
    }
}

void FSoEMaster::update(uint64_t current_time_ms)
{
    std::lock_guard<std::mutex> lock(mutex_);
    for (auto& conn : connections_) {
        conn->update(current_time_ms);
    }
}

bool FSoEMaster::allOperationalUnsafe() const
{
    for (const auto& conn : connections_) {
        if (!conn->isOperational()) {
            return false;
        }
    }
    return !connections_.empty();
}

bool FSoEMaster::allOperational() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return allOperationalUnsafe();
}

bool FSoEMaster::anyFailSafeUnsafe() const
{
    for (const auto& conn : connections_) {
        if (conn->isFailSafe()) {
            return true;
        }
    }
    return false;
}

bool FSoEMaster::anyFailSafe() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return anyFailSafeUnsafe();
}

size_t FSoEMaster::getConnectionCount() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return connections_.size();
}

std::string FSoEMaster::getDiagnostics() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    std::string diag;
    
    diag += "FSoE Master Diagnostics:\n";
    diag += "  Connections: " + std::to_string(connections_.size()) + "\n";
    diag += "  All Operational: " + std::string(allOperationalUnsafe() ? "Yes" : "No") + "\n";
    diag += "  Any Fail-Safe: " + std::string(anyFailSafeUnsafe() ? "Yes" : "No") + "\n";
    
    for (size_t i = 0; i < connections_.size(); ++i) {
        diag += "\n--- Connection " + std::to_string(i) + " ---\n";
        diag += connections_[i]->getDiagnostics();
    }
    
    return diag;
}

} // namespace FSoE
