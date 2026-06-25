/**
 * @file FSoEMasterConnection.cpp
 * @brief FSoE Master Connection implementation — redesigned
 */

#include "fsoe/FSoEMasterConnection.hpp"
#include "fsoe/FSoESlave.hpp"
#include <cstring>
#include <algorithm>
#include <cstdio>

namespace FSoE {

// ============================================================================
// Construction / Initialization
// ============================================================================

FSoEMasterConnection::FSoEMasterConnection(const MasterConnectionConfig& config)
    : config_(config)
{
}

FSoEMasterConnection::~FSoEMasterConnection() = default;

bool FSoEMasterConnection::initialize()
{
    std::lock_guard<std::recursive_mutex> lock(mutex_);

    if (config_.input_size > 16 || config_.output_size > 16) {
        return false;
    }

    std::copy(config_.fail_safe_values.begin(),
              config_.fail_safe_values.begin() + config_.output_size,
              safe_outputs_.begin());

    status_ = {};
    status_.state = ConnectionState::Reset;
    status_.state_entered_ms = 0;

    rx_sequence_ = 0;
    tx_sequence_ = 0;
    current_param_index_ = 0;
    parameter_crc_ = 0;
    fail_safe_entered_ms_ = 0;

    resetStats();
    parameter_crc_ = computeParameterCRC();

    initialized_ = true;
    return true;
}

bool FSoEMasterConnection::isInitialized() const
{
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    return initialized_;
}

const MasterConnectionConfig& FSoEMasterConnection::getConfig() const
{
    return config_;
}

// ============================================================================
// Connection Control
// ============================================================================

bool FSoEMasterConnection::startConnection()
{
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    if (!initialized_) return false;

    status_.state = ConnectionState::Reset;
    status_.state_entered_ms = current_time_ms_;
    return true;
}

bool FSoEMasterConnection::resetConnection()
{
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    if (!initialized_) return false;

    transitionTo(ConnectionState::Reset);
    status_.error_code = ErrorCode::NoError;
    status_.data_valid = false;
    status_.fail_safe_active = false;
    rx_sequence_ = 0;
    tx_sequence_ = 0;
    current_param_index_ = 0;
    stats_.reset_events++;

    return true;
}

bool FSoEMasterConnection::requestSessionReset()
{
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    if (!initialized_) return false;

    // Generate non-predictable session ID
    status_.session_id = static_cast<uint16_t>(rng_() & 0xFFFF);
    if (status_.session_id == 0) {
        status_.session_id = 1;
    }

    transitionTo(ConnectionState::Session);
    return true;
}

void FSoEMasterConnection::triggerFailSafe(uint16_t error_code)
{
    std::lock_guard<std::recursive_mutex> lock(mutex_);

    const bool was_fail_safe = status_.fail_safe_active;
    status_.fail_safe_active = true;
    status_.data_valid = false;

    if (error_code != ErrorCode::NoError) {
        status_.error_code = error_code;
    }

    std::copy(config_.fail_safe_values.begin(),
              config_.fail_safe_values.begin() + config_.output_size,
              safe_outputs_.begin());

    transitionTo(ConnectionState::FailSafe);
    fail_safe_entered_ms_ = current_time_ms_;

    if (!was_fail_safe && fail_safe_callback_) {
        fail_safe_callback_();
    }
}

bool FSoEMasterConnection::clearError()
{
    std::lock_guard<std::recursive_mutex> lock(mutex_);

    if (status_.state != ConnectionState::Error &&
        status_.state != ConnectionState::FailSafe) {
        return false;
    }

    status_.error_code = ErrorCode::NoError;
    status_.fail_safe_active = false;

    return resetConnection();
}

// ============================================================================
// State Machine — processRxFrame
// ============================================================================

bool FSoEMasterConnection::processRxFrame(const uint8_t* data, size_t len)
{
    std::lock_guard<std::recursive_mutex> lock(mutex_);

    if (!initialized_ || !data || len < sizeof(FSoEHeader) + 2) {
        return false;
    }

    stats_.frames_received++;

    // Validate CRC
    if (!validateCRC(data, len)) {
        stats_.crc_errors++;
        handleError(ErrorCode::CRCError);
        return false;
    }

    // Extract and validate connection ID
    uint16_t conn_id = extractConnectionID(data, len);
    if (!validateConnectionID(conn_id)) {
        handleError(ErrorCode::ConnectionIDError);
        return false;
    }

    // Update watchdog timestamp
    status_.last_valid_frame_ms = current_time_ms_;

    // Process based on current state
    switch (status_.state) {
        case ConnectionState::Reset:
            // In Reset, we don't process incoming frames — master drives the protocol
            // by transitioning to Session in update()/prepareTxFrame()
            break;

        case ConnectionState::Session:
            handleSessionState(data, len);
            break;

        case ConnectionState::Connection:
            handleConnectionState(data, len);
            break;

        case ConnectionState::Parameter:
            handleParameterState(data, len);
            break;

        case ConnectionState::Data:
            handleDataState(data, len);
            break;

        case ConnectionState::FailSafe:
            handleFailSafeState(data, len);
            break;

        case ConnectionState::Error:
            // In Error state, only process Reset commands for recovery
            {
                const auto* header = reinterpret_cast<const FSoEHeader*>(data);
                if (header->command == Command::Reset) {
                    if (config_.auto_recovery_enabled) {
                        resetConnection();
                        stats_.successful_recoveries++;
                    }
                }
            }
            break;
    }

    return true;
}

// ============================================================================
// State Machine — prepareTxFrame
// ============================================================================

size_t FSoEMasterConnection::prepareTxFrame(uint8_t* data, size_t max_len)
{
    std::lock_guard<std::recursive_mutex> lock(mutex_);

    if (!initialized_ || !data) return 0;

    // If in Reset, auto-transition to Session to start the protocol
    if (status_.state == ConnectionState::Reset) {
        requestSessionReset();
    }

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
            len = buildDataFrame(data, max_len);
            break;

        case ConnectionState::FailSafe:
            len = buildFailSafeFrame(data, max_len);
            break;

        case ConnectionState::Error:
            // No frames in Error state
            break;
    }

    if (len > 0) {
        stats_.frames_sent++;
        if (status_.state == ConnectionState::Data ||
            status_.state == ConnectionState::Parameter) {
            tx_sequence_ = (tx_sequence_ + 1) & 0x0F;
        }
    }

    return len;
}

// ============================================================================
// Update / Watchdog
// ============================================================================

void FSoEMasterConnection::update(uint64_t current_time_ms)
{
    std::lock_guard<std::recursive_mutex> lock(mutex_);

    current_time_ms_ = current_time_ms;
    stats_.uptime_ms = current_time_ms;

    checkPhaseTimeout(current_time_ms);
    checkWatchdog(current_time_ms);

    // Auto-recovery attempt
    if (status_.state == ConnectionState::FailSafe && config_.auto_recovery_enabled) {
        attemptAutoRecovery(current_time_ms);
    }
}

void FSoEMasterConnection::checkPhaseTimeout(uint64_t current_time_ms)
{
    if (status_.state == ConnectionState::Reset ||
        status_.state == ConnectionState::Data ||
        status_.state == ConnectionState::FailSafe ||
        status_.state == ConnectionState::Error) {
        return;
    }

    uint64_t elapsed = current_time_ms - status_.state_entered_ms;
    uint16_t timeout = config_.conn_timeout_ms;

    if (status_.state == ConnectionState::Session) {
        timeout = config_.session_timeout_ms;
    }

    if (elapsed > timeout) {
        stats_.watchdog_events++;
        handleError(ErrorCode::WatchdogError);
    }
}

void FSoEMasterConnection::checkWatchdog(uint64_t current_time_ms)
{
    if (status_.state != ConnectionState::Data) return;

    uint64_t elapsed = current_time_ms - status_.last_valid_frame_ms;
    status_.watchdog_counter = static_cast<uint32_t>(elapsed);

    if (elapsed > config_.watchdog_timeout_ms) {
        stats_.watchdog_events++;
        handleError(ErrorCode::WatchdogError);
    }
}

void FSoEMasterConnection::attemptAutoRecovery(uint64_t current_time_ms)
{
    uint64_t elapsed = current_time_ms - fail_safe_entered_ms_;
    if (elapsed >= config_.recovery_delay_ms) {
        stats_.recovery_attempts++;
        resetConnection();
    }
}

// ============================================================================
// State Handlers
// ============================================================================

void FSoEMasterConnection::handleSessionState(const uint8_t* data, size_t len)
{
    const auto* header = reinterpret_cast<const FSoEHeader*>(data);

    if (header->command == Command::Session) {
        // Slave acknowledged session, move to connection
        transitionTo(ConnectionState::Connection);
    } else {
        handleError(ErrorCode::CommandError);
    }
}

void FSoEMasterConnection::handleConnectionState(const uint8_t* data, size_t len)
{
    const auto* header = reinterpret_cast<const FSoEHeader*>(data);

    if (header->command == Command::Connection) {
        // Slave acknowledged connection, move to parameter phase
        if (config_.input_size > 0 || config_.output_size > 0) {
            transitionTo(ConnectionState::Parameter);
        } else {
            transitionTo(ConnectionState::Data);
        }
    } else if (header->command == Command::ParameterResp) {
        // Slave jumped straight to parameter response
        transitionTo(ConnectionState::Parameter);
        handleParameterState(data, len);
    } else {
        handleError(ErrorCode::CommandError);
    }
}

void FSoEMasterConnection::handleParameterState(const uint8_t* data, size_t len)
{
    FSoEHeader header;
    std::memcpy(&header, data, sizeof(header));

    if (header.command == Command::ParameterResp) {
        // Move to next parameter or finish
        current_param_index_++;
        // For simplicity, we exchange a fixed set of parameters then transition
        // In a real implementation, this would iterate through all configured parameters
        if (current_param_index_ >= 1) {
            transitionTo(ConnectionState::Data);
        }
    } else if (header.command == Command::ProcessData) {
        // Slave skipped parameter phase, go straight to data
        transitionTo(ConnectionState::Data);
        handleDataState(data, len);
    } else {
        handleError(ErrorCode::CommandError);
    }
}

void FSoEMasterConnection::handleDataState(const uint8_t* data, size_t len)
{
    const auto* header = reinterpret_cast<const FSoEHeader*>(data);

    if (header->command != Command::ProcessData) {
        if (header->command == Command::Reset) {
            // Slave requested reset
            resetConnection();
            return;
        }
        handleError(ErrorCode::CommandError);
        return;
    }

    // Validate sequence
    uint8_t seq = (header->conn_id_high >> 4) & 0x0F;
    if (!validateSequence(seq)) {
        stats_.sequence_errors++;
        handleError(ErrorCode::SequenceError);
        return;
    }

    rx_sequence_ = seq;
    status_.sequence_number = seq;

    // Extract safety data
    const uint8_t* safe_data = data + sizeof(FSoEHeader);
    size_t data_len = len - sizeof(FSoEHeader) - 2;  // Minus header and CRC

    if (data_len < config_.input_size) {
        stats_.invalid_frames++;
        handleError(ErrorCode::DataLengthError);
        return;
    }

    std::copy(safe_data, safe_data + config_.input_size, safe_inputs_.begin());
    status_.data_valid = true;

    if (data_callback_) {
        data_callback_(safe_inputs_.data(), config_.input_size);
    }
}

void FSoEMasterConnection::handleFailSafeState(const uint8_t* data, size_t len)
{
    const auto* header = reinterpret_cast<const FSoEHeader*>(data);

    if (header->command == Command::Reset) {
        // Slave initiated recovery
        if (config_.auto_recovery_enabled) {
            stats_.successful_recoveries++;
            resetConnection();
        }
    }
}

// ============================================================================
// Frame Building
// ============================================================================

size_t FSoEMasterConnection::buildSessionResetFrame(uint8_t* data, size_t max_len)
{
    if (max_len < sizeof(FSoESessionReset)) return 0;

    auto* frame = reinterpret_cast<FSoESessionReset*>(data);

    frame->header.command = Command::Session;
    frame->header.conn_id_low = config_.connection_id & 0xFF;
    frame->header.conn_id_high = (config_.connection_id >> 8) & 0xFF;
    frame->session_id = status_.session_id;

    frame->crc = calculateCRC16(data, sizeof(FSoESessionReset) - 2);

    return sizeof(FSoESessionReset);
}

size_t FSoEMasterConnection::buildConnectionFrame(uint8_t* data, size_t max_len)
{
    if (max_len < sizeof(FSoEConnectionFrame)) return 0;

    auto* frame = reinterpret_cast<struct FSoEConnectionFrame*>(data);

    frame->header.command = Command::Connection;
    frame->header.conn_id_low = config_.connection_id & 0xFF;
    frame->header.conn_id_high = (config_.connection_id >> 8) & 0xFF;
    frame->conn_id = config_.connection_id;
    frame->slave_addr = config_.slave_addr;
    frame->sl_param_crc = parameter_crc_;

    frame->crc = calculateCRC16(data, sizeof(struct FSoEConnectionFrame) - 2);

    return sizeof(struct FSoEConnectionFrame);
}

size_t FSoEMasterConnection::buildParameterFrame(uint8_t* data, size_t max_len)
{
    // Build a parameter request frame
    // Frame layout: [command(1)][conn_id_low(1)][conn_id_high+seq(1)][param_id(2)][param_data(variable)][crc(2)]
    size_t param_data_size = 6;  // watchdog(2) + conn_timeout(2) + safety_level(1) + reserved(1)
    size_t frame_size = 3 + 2 + param_data_size + 2;  // header + param_id + data + crc

    if (max_len < frame_size) return 0;

    data[0] = Command::ParameterReq;
    data[1] = config_.connection_id & 0xFF;
    data[2] = ((config_.connection_id >> 8) & 0x0F) | ((tx_sequence_ & 0x0F) << 4);

    // Parameter ID
    data[3] = static_cast<uint8_t>(ParameterID::WatchdogTimeout & 0xFF);
    data[4] = static_cast<uint8_t>((ParameterID::WatchdogTimeout >> 8) & 0xFF);

    // Parameter data
    data[5] = config_.watchdog_timeout_ms & 0xFF;
    data[6] = (config_.watchdog_timeout_ms >> 8) & 0xFF;
    data[7] = config_.conn_timeout_ms & 0xFF;
    data[8] = (config_.conn_timeout_ms >> 8) & 0xFF;
    data[9] = config_.safety_level;
    data[10] = 0;  // reserved

    uint16_t crc = calculateCRC16(data, frame_size - 2);
    data[frame_size - 2] = crc & 0xFF;
    data[frame_size - 1] = (crc >> 8) & 0xFF;

    return frame_size;
}

size_t FSoEMasterConnection::buildDataFrame(uint8_t* data, size_t max_len)
{
    size_t frame_size = sizeof(FSoEHeader) + config_.output_size + 2;

    if (max_len < frame_size) return 0;

    auto* header = reinterpret_cast<FSoEHeader*>(data);

    header->command = Command::ProcessData;
    header->conn_id_low = config_.connection_id & 0xFF;
    header->conn_id_high = ((config_.connection_id >> 8) & 0x0F) | ((tx_sequence_ & 0x0F) << 4);

    std::copy(safe_outputs_.begin(), safe_outputs_.begin() + config_.output_size,
              data + sizeof(FSoEHeader));

    uint16_t crc = calculateCRC16(data, frame_size - 2);
    data[frame_size - 2] = crc & 0xFF;
    data[frame_size - 1] = (crc >> 8) & 0xFF;

    return frame_size;
}

size_t FSoEMasterConnection::buildFailSafeFrame(uint8_t* data, size_t max_len)
{
    // Fail-safe frame sends Reset command with fail-safe output values
    size_t frame_size = sizeof(FSoEHeader) + config_.output_size + 2;

    if (max_len < frame_size) return 0;

    auto* header = reinterpret_cast<FSoEHeader*>(data);

    header->command = Command::Reset;
    header->conn_id_low = config_.connection_id & 0xFF;
    header->conn_id_high = (config_.connection_id >> 8) & 0xFF;

    // Copy only output_size bytes of fail-safe values (fix: was copying all 8)
    std::copy(config_.fail_safe_values.begin(),
              config_.fail_safe_values.begin() + config_.output_size,
              data + sizeof(FSoEHeader));

    uint16_t crc = calculateCRC16(data, frame_size - 2);
    data[frame_size - 2] = crc & 0xFF;
    data[frame_size - 1] = (crc >> 8) & 0xFF;

    return frame_size;
}

// ============================================================================
// Frame Validation
// ============================================================================

bool FSoEMasterConnection::validateCRC(const uint8_t* data, size_t len) const
{
    if (len < 2) return false;

    uint16_t received_crc = static_cast<uint16_t>(data[len - 2]) |
                            (static_cast<uint16_t>(data[len - 1]) << 8);
    uint16_t calculated_crc = calculateCRC16(data, len - 2);

    return received_crc == calculated_crc;
}

bool FSoEMasterConnection::validateSequence(uint8_t seq)
{
    // Strict: only accept the next expected sequence number
    uint8_t expected = (rx_sequence_ + 1) & 0x0F;
    if (seq != expected) {
        return false;
    }
    return true;
}

bool FSoEMasterConnection::validateConnectionID(uint16_t conn_id) const
{
    return (conn_id & 0x0FFFu) == (config_.connection_id & 0x0FFFu);
}

uint16_t FSoEMasterConnection::extractConnectionID(const uint8_t* data, size_t len) const
{
    if (len < sizeof(FSoEHeader)) return 0xFFFF;

    const auto* header = reinterpret_cast<const FSoEHeader*>(data);

    if (header->command == Command::ProcessData) {
        // For process data, conn_id is in low byte + low nibble of high byte
        return static_cast<uint16_t>(header->conn_id_low |
                                      ((header->conn_id_high & 0x0F) << 8));
    } else {
        // For other frames, conn_id is full 16-bit
        return static_cast<uint16_t>(header->conn_id_low |
                                      (header->conn_id_high << 8));
    }
}

// ============================================================================
// State Transitions
// ============================================================================

void FSoEMasterConnection::transitionTo(uint8_t new_state)
{
    if (new_state == status_.state) return;

    uint8_t old_state = status_.state;
    status_.state = new_state;
    status_.state_entered_ms = current_time_ms_;

    if (old_state == ConnectionState::Data && new_state != ConnectionState::Data) {
        status_.data_valid = false;
    }

    if (state_change_callback_) {
        state_change_callback_(old_state, new_state);
    }
}

void FSoEMasterConnection::handleError(uint16_t error_code)
{
    status_.error_code = error_code;

    // Transition to Error state (persistent, not immediately overwritten)
    transitionTo(ConnectionState::Error);

    if (config_.auto_fail_safe_on_error) {
        triggerFailSafe(error_code);
    }

    if (error_callback_) {
        error_callback_(error_code);
    }
}

uint16_t FSoEMasterConnection::computeParameterCRC() const
{
    // Compute CRC over the parameter set
    std::array<uint8_t, 19> param_data{};

    param_data[0] = config_.watchdog_timeout_ms & 0xFF;
    param_data[1] = (config_.watchdog_timeout_ms >> 8) & 0xFF;
    param_data[2] = config_.conn_timeout_ms & 0xFF;
    param_data[3] = (config_.conn_timeout_ms >> 8) & 0xFF;
    param_data[4] = config_.safety_level;
    param_data[5] = config_.input_size;
    param_data[6] = config_.output_size;
    param_data[7] = config_.slave_addr & 0xFF;
    param_data[8] = (config_.slave_addr >> 8) & 0xFF;
    param_data[9] = config_.master_addr & 0xFF;
    param_data[10] = (config_.master_addr >> 8) & 0xFF;

    for (size_t i = 0; i < config_.fail_safe_values.size(); ++i) {
        param_data[11 + i] = config_.fail_safe_values[i];
    }

    return calculateCRC16(param_data.data(), param_data.size());
}

// ============================================================================
// Safe Data Access
// ============================================================================

bool FSoEMasterConnection::setSafeOutputs(const uint8_t* data, size_t len)
{
    std::lock_guard<std::recursive_mutex> lock(mutex_);

    if (!data || len != config_.output_size) return false;
    if (status_.isFailSafe()) return false;

    std::copy(data, data + len, safe_outputs_.begin());
    return true;
}

bool FSoEMasterConnection::writeOutputProcessData(std::span<const uint8_t> data)
{
    return setSafeOutputs(data.data(), data.size());
}

size_t FSoEMasterConnection::getSafeInputs(uint8_t* data, size_t len) const
{
    std::lock_guard<std::recursive_mutex> lock(mutex_);

    if (!data || len < config_.input_size) return 0;
    if (!status_.data_valid) return 0;

    std::copy(safe_inputs_.begin(), safe_inputs_.begin() + config_.input_size, data);
    return config_.input_size;
}

size_t FSoEMasterConnection::readInputProcessData(std::span<uint8_t> data) const
{
    return getSafeInputs(data.data(), data.size());
}

std::vector<uint8_t> FSoEMasterConnection::inputProcessData() const
{
    std::lock_guard<std::recursive_mutex> lock(mutex_);

    std::vector<uint8_t> data(config_.input_size, 0);
    const size_t copied = getSafeInputs(data.data(), data.size());
    data.resize(copied);
    return data;
}

std::vector<uint8_t> FSoEMasterConnection::outputProcessData() const
{
    std::lock_guard<std::recursive_mutex> lock(mutex_);

    return std::vector<uint8_t>(safe_outputs_.begin(),
                                safe_outputs_.begin() + config_.output_size);
}

bool FSoEMasterConnection::exchangeWith(FSoESlave& slave, uint64_t current_time_ms)
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

bool FSoEMasterConnection::areSafeInputsValid() const
{
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    return status_.data_valid && status_.isOperational();
}

// ============================================================================
// Bit-level Safe I/O
// ============================================================================

bool FSoEMasterConnection::getSafeInputBit(uint8_t bit_index) const
{
    std::lock_guard<std::recursive_mutex> lock(mutex_);

    if (!status_.data_valid) return false;

    uint8_t byte_idx = bit_index / 8;
    uint8_t bit_pos = bit_index % 8;

    if (byte_idx >= config_.input_size) return false;

    return (safe_inputs_[byte_idx] >> bit_pos) & 1;
}

bool FSoEMasterConnection::setSafeOutputBit(uint8_t bit_index, bool value)
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

uint8_t FSoEMasterConnection::getSafeInputByte(uint8_t byte_index) const
{
    std::lock_guard<std::recursive_mutex> lock(mutex_);

    if (!status_.data_valid || byte_index >= config_.input_size) {
        return 0;
    }
    return safe_inputs_[byte_index];
}

bool FSoEMasterConnection::setSafeOutputByte(uint8_t byte_index, uint8_t value)
{
    std::lock_guard<std::recursive_mutex> lock(mutex_);

    if (status_.isFailSafe() || byte_index >= config_.output_size) {
        return false;
    }
    safe_outputs_[byte_index] = value;
    return true;
}

// ============================================================================
// Status & Diagnostics
// ============================================================================

const MasterConnectionStatus& FSoEMasterConnection::getStatus() const
{
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    return status_;
}

uint8_t FSoEMasterConnection::getState() const
{
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    return status_.state;
}

uint16_t FSoEMasterConnection::getErrorCode() const
{
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    return status_.error_code;
}

bool FSoEMasterConnection::isOperational() const
{
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    return status_.isOperational();
}

bool FSoEMasterConnection::isFailSafe() const
{
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    return status_.isFailSafe();
}

const ConnectionStats& FSoEMasterConnection::getStats() const
{
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    return stats_;
}

void FSoEMasterConnection::resetStats()
{
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    stats_ = {};
}

std::string FSoEMasterConnection::getDiagnostics() const
{
    std::lock_guard<std::recursive_mutex> lock(mutex_);

    std::string diag;

    diag += "FSoE Master Connection Diagnostics:\n";
    diag += "  Connection ID: " + std::to_string(config_.connection_id) + "\n";
    diag += "  Slave Address: " + std::to_string(config_.slave_addr) + "\n";
    diag += "  State: " + std::to_string(status_.state) + "\n";
    diag += "  Operational: " + std::string(status_.isOperational() ? "Yes" : "No") + "\n";
    diag += "  Fail-Safe: " + std::string(status_.isFailSafe() ? "Yes" : "No") + "\n";
    diag += "  Data Valid: " + std::string(status_.data_valid ? "Yes" : "No") + "\n";

    if (status_.hasError()) {
        char buf[32];
        snprintf(buf, sizeof(buf), "0x%04X", status_.error_code);
        diag += "  ERROR: " + std::string(buf) + "\n";
    }

    diag += "  Session ID: " + std::to_string(status_.session_id) + "\n";
    diag += "  RX Sequence: " + std::to_string(rx_sequence_) + "\n";
    diag += "  TX Sequence: " + std::to_string(tx_sequence_) + "\n";
    diag += "  Watchdog: " + std::to_string(status_.watchdog_counter) + " ms\n";
    diag += "  Parameter CRC: " + std::to_string(parameter_crc_) + "\n";

    diag += "\nStatistics:\n";
    diag += "  Frames Sent: " + std::to_string(stats_.frames_sent) + "\n";
    diag += "  Frames Received: " + std::to_string(stats_.frames_received) + "\n";
    diag += "  CRC Errors: " + std::to_string(stats_.crc_errors) + "\n";
    diag += "  Sequence Errors: " + std::to_string(stats_.sequence_errors) + "\n";
    diag += "  Watchdog Events: " + std::to_string(stats_.watchdog_events) + "\n";
    diag += "  Reset Events: " + std::to_string(stats_.reset_events) + "\n";
    diag += "  Recovery Attempts: " + std::to_string(stats_.recovery_attempts) + "\n";
    diag += "  Successful Recoveries: " + std::to_string(stats_.successful_recoveries) + "\n";

    return diag;
}

// ============================================================================
// Callbacks
// ============================================================================

void FSoEMasterConnection::setStateChangeCallback(StateChangeCallback callback)
{
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    state_change_callback_ = std::move(callback);
}

void FSoEMasterConnection::setErrorCallback(ErrorCallback callback)
{
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    error_callback_ = std::move(callback);
}

void FSoEMasterConnection::setFailSafeCallback(FailSafeCallback callback)
{
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    fail_safe_callback_ = std::move(callback);
}

void FSoEMasterConnection::setDataCallback(DataCallback callback)
{
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    data_callback_ = std::move(callback);
}

} // namespace FSoE
