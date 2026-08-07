/**
 * @file FSoEMasterConnection.cpp
 * @brief FSoE Master Connection implementation — redesigned
 */

#include "fsoe/FSoEMasterConnection.hpp"
#include "fsoe/FSoESlave.hpp"
#include "fsoe/FSoECRC.hpp"
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

    rx_sequence_ = 0x0F;  // Wrap so first expected RX sequence is 0
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
    rx_sequence_ = 0x0F;  // Wrap so first expected RX sequence is 0
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

    if (!initialized_ || !data || len < CRC::MIN_FSOE_FRAME_SIZE) {
        return false;
    }

    stats_.frames_received++;

    // Parse and validate frame (CRC verification happens inside parseFSoEFrame)
    // Buffer must accommodate MAX_PARSE_DATA_SIZE bytes because the slave's
    // buildFailSafeResponse sends safeInputSize + 2 bytes (inputs + error code),
    // which can be up to 18 bytes when safeInputSize = 16.
    uint8_t cmd = 0;
    uint8_t frame_data[CRC::MAX_PARSE_DATA_SIZE] = {0};
    size_t data_len = 0;
    uint16_t conn_id = 0;

    if (!CRC::parseFSoEFrame(data, len, cmd, frame_data, data_len, conn_id)) {
        stats_.crc_errors++;
        handleError(ErrorCode::CRCError);
        return false;
    }

    // Validate connection ID
    if (!validateConnectionID(conn_id)) {
        handleError(ErrorCode::ConnectionIDError);
        return false;
    }

    // Update watchdog timestamp
    status_.last_valid_frame_ms = current_time_ms_;

    // Early detection of slave fail-safe response
    if (cmd == Command::FailSafeData &&
        status_.state != ConnectionState::FailSafe &&
        status_.state != ConnectionState::Error) {
        if (data_len >= 2) {
            size_t error_offset = (data_len >= config_.input_size + 2)
                                      ? config_.input_size : 0;
            uint16_t slave_error = static_cast<uint16_t>(
                frame_data[error_offset] | (frame_data[error_offset + 1] << 8));
            handleError(slave_error);
        } else {
            handleError(ErrorCode::ApplicationError);
        }
        return true;
    }

    // Process based on current state
    switch (status_.state) {
        case ConnectionState::Reset:
            // No frames expected in Reset — prepareTxFrame auto-transitions
            // to Session, so we should never actually be here.
            return false;

        case ConnectionState::Session:
            handleSessionState(cmd, frame_data, data_len);
            break;

        case ConnectionState::Connection:
            handleConnectionState(cmd, frame_data, data_len);
            break;

        case ConnectionState::Parameter:
            handleParameterState(cmd, frame_data, data_len);
            break;

        case ConnectionState::Data:
            handleDataState(cmd, frame_data, data_len);
            break;

        case ConnectionState::FailSafe:
            handleFailSafeState(cmd, frame_data, data_len);
            break;

        case ConnectionState::Error:
            if (cmd == Command::Reset) {
                if (config_.auto_recovery_enabled) {
                    resetConnection();
                    stats_.successful_recoveries++;
                }
            } else {
                // Ignore non-Reset commands in Error state
                return false;
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
        stats_.timeout_events++;
        handleError(ErrorCode::TimeoutError);
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

void FSoEMasterConnection::handleSessionState(uint8_t cmd, const uint8_t* data, size_t data_len)
{
    (void)data;
    (void)data_len;

    if (cmd == Command::Session) {
        transitionTo(ConnectionState::Connection);
    } else {
        handleError(ErrorCode::CommandError);
    }
}

void FSoEMasterConnection::handleConnectionState(uint8_t cmd, const uint8_t* data, size_t data_len)
{
    if (cmd == Command::Connection) {
        // Validate slave's safety address and SIL from connection response.
        // Parsed data format: [safetyAddr_lo][safetyAddr_hi][sil][reserved]
        // Require the full 4-byte payload — the slave always sends 4 bytes.
        if (data_len < 4) {
            handleError(ErrorCode::DataLengthError);
            return;
        }
        uint16_t slave_safety_addr = data[0] | (data[1] << 8);
        if (slave_safety_addr != 0 && slave_safety_addr != config_.slave_safety_addr) {
            handleError(ErrorCode::ConnectionIDError);
            return;
        }
        uint8_t slave_sil = data[2];
        if (slave_sil < config_.safety_level) {
            handleError(ErrorCode::ApplicationError);
            return;
        }
        if (config_.input_size > 0 || config_.output_size > 0) {
            transitionTo(ConnectionState::Parameter);
        } else {
            transitionTo(ConnectionState::Data);
        }
    } else {
        handleError(ErrorCode::CommandError);
    }
}

void FSoEMasterConnection::handleParameterState(uint8_t cmd, const uint8_t* data, size_t data_len)
{
    (void)data;
    (void)data_len;

    if (cmd == Command::Parameter) {
        current_param_index_++;
        if (current_param_index_ >= 1) {
            transitionTo(ConnectionState::Data);
        }
    } else if (cmd == Command::ProcessData) {
        transitionTo(ConnectionState::Data);
        handleDataState(cmd, data, data_len);
    } else {
        handleError(ErrorCode::CommandError);
    }
}

void FSoEMasterConnection::handleDataState(uint8_t cmd, const uint8_t* data, size_t data_len)
{
    if (cmd != Command::ProcessData) {
        if (cmd == Command::Reset) {
            resetConnection();
            return;
        }
        handleError(ErrorCode::CommandError);
        return;
    }

    // ETG.5100 does not define a sequence number field
    // Frame integrity is ensured via CRC + watchdog

    if (data_len < config_.input_size) {
        stats_.invalid_frames++;
        handleError(ErrorCode::DataLengthError);
        return;
    }

    std::copy(data, data + config_.input_size, safe_inputs_.begin());
    status_.data_valid = true;

    if (data_callback_) {
        data_callback_(safe_inputs_.data(), config_.input_size);
    }
}

void FSoEMasterConnection::handleFailSafeState(uint8_t cmd, const uint8_t* data, size_t data_len)
{
    if (cmd == Command::Reset) {
        if (config_.auto_recovery_enabled) {
            stats_.successful_recoveries++;
            resetConnection();
        }
    } else if (cmd == Command::FailSafeData) {
        // Slave is also in fail-safe — acknowledge by staying in fail-safe.
        // Recovery will be attempted by attemptAutoRecovery() in update().
        // Extract slave error code for diagnostics (input_size bytes of
        // fail-safe inputs followed by 2-byte error code).
        if (data_len >= config_.input_size + 2) {
            uint16_t slave_error = static_cast<uint16_t>(
                data[config_.input_size] | (data[config_.input_size + 1] << 8));
            // Update error code only if the slave reports a different error
            // than what we already have — avoids overwriting our own error.
            if (slave_error != ErrorCode::NoError &&
                slave_error != status_.error_code) {
                status_.error_code = slave_error;
            }
        }
    } else {
        // Unexpected command in FailSafe state
        handleError(ErrorCode::CommandError);
    }
}

// ============================================================================
// Frame Building
// ============================================================================

size_t FSoEMasterConnection::buildSessionResetFrame(uint8_t* data, size_t max_len)
{
    // Session reset: CMD + session_id (2B payload) + ConnID
    uint8_t payload[2] = {0, 0};
    payload[0] = static_cast<uint8_t>(status_.session_id & 0xFF);
    payload[1] = static_cast<uint8_t>((status_.session_id >> 8) & 0xFF);
    size_t needed = CRC::fsoeFrameSize(2);
    if (max_len < needed) return 0;
    return CRC::buildFSoEFrame(data, Command::Session, payload, 2, config_.connection_id);
}

size_t FSoEMasterConnection::buildConnectionFrame(uint8_t* data, size_t max_len)
{
    // Connection frame: CMD + payload(safety_addr 2B + param_crc 2B = 4B) + ConnID
    uint8_t payload[4] = {0, 0, 0, 0};
    payload[0] = config_.slave_safety_addr & 0xFF;
    payload[1] = (config_.slave_safety_addr >> 8) & 0xFF;
    payload[2] = parameter_crc_ & 0xFF;
    payload[3] = (parameter_crc_ >> 8) & 0xFF;
    size_t needed = CRC::fsoeFrameSize(4);
    if (max_len < needed) return 0;
    return CRC::buildFSoEFrame(data, Command::Connection, payload, 4, config_.connection_id);
}

size_t FSoEMasterConnection::buildParameterFrame(uint8_t* data, size_t max_len)
{
    // Parameter frame: CMD + param data (6B) + ConnID
    // Layout: [watchdog_lo] [watchdog_hi] [safety_level] [input_size] [output_size] [reserved]
    uint8_t payload[6] = {0, 0, 0, 0, 0, 0};
    payload[0] = config_.watchdog_timeout_ms & 0xFF;
    payload[1] = (config_.watchdog_timeout_ms >> 8) & 0xFF;
    payload[2] = config_.safety_level;
    payload[3] = config_.input_size;
    payload[4] = config_.output_size;
    payload[5] = 0;  // reserved
    size_t needed = CRC::fsoeFrameSize(6);
    if (max_len < needed) return 0;
    return CRC::buildFSoEFrame(data, Command::Parameter, payload, 6, config_.connection_id);
}

size_t FSoEMasterConnection::buildDataFrame(uint8_t* data, size_t max_len)
{
    // Data frame: CMD + safe_outputs + ConnID
    size_t needed = CRC::fsoeFrameSize(config_.output_size);
    if (max_len < needed) return 0;
    return CRC::buildFSoEFrame(data, Command::ProcessData,
                               safe_outputs_.data(), config_.output_size,
                               config_.connection_id);
}

size_t FSoEMasterConnection::buildFailSafeFrame(uint8_t* data, size_t max_len)
{
    // Fail-safe frame sends FailSafeData command with fail-safe output values
    size_t needed = CRC::fsoeFrameSize(config_.output_size);
    if (max_len < needed) return 0;
    return CRC::buildFSoEFrame(data, Command::FailSafeData,
                               config_.fail_safe_values.data(), config_.output_size,
                               config_.connection_id);
}

// ============================================================================
// Frame Validation
// ============================================================================

bool FSoEMasterConnection::validateCRC(const uint8_t* data, size_t len) const
{
    uint8_t cmd = 0;
    size_t data_len = 0;
    uint16_t conn_id = 0;
    return CRC::parseFSoEFrame(data, len, cmd, nullptr, data_len, conn_id);
}

bool FSoEMasterConnection::validateSequence(uint8_t seq)
{
    // ETG.5100 does not define a sequence number field.
    // Frame integrity is ensured via CRC + watchdog.
    (void)seq;
    return true;
}

bool FSoEMasterConnection::validateConnectionID(uint16_t conn_id) const
{
    return conn_id == config_.connection_id;
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

    if (config_.auto_fail_safe_on_error) {
        // Go directly to fail-safe (skip Error state)
        triggerFailSafe(error_code);
    } else {
        // Stay in Error state (persistent until explicitly cleared)
        transitionTo(ConnectionState::Error);
    }

    if (error_callback_) {
        error_callback_(error_code);
    }
}

uint16_t FSoEMasterConnection::computeParameterCRC() const
{
    // Compute CRC over the parameter set.
    // Layout: 11 fixed bytes + fail_safe_values (up to 16 bytes) = up to 27 bytes.
    std::array<uint8_t, 27> param_data{};

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

    return CRC::calculate(param_data.data(), param_data.size());
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

MasterConnectionStatus FSoEMasterConnection::getStatus() const
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

ConnectionStats FSoEMasterConnection::getStats() const
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
