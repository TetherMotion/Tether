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

// ============================================================================
// Initialization
// ============================================================================

bool FSoEConnection::initialize()
{
    if (config_.input_size > 16 || config_.output_size > 16) {
        return false;  // Buffer size exceeded
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
    if (!initialized_) return false;
    
    status_.state = ConnectionState::Reset;
    return true;
}

bool FSoEConnection::resetConnection()
{
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
    if (!initialized_) return false;
    
    // Generate new session ID
    status_.session_id = static_cast<uint16_t>(stats_.frames_sent & 0xFFFF);
    if (status_.session_id == 0) {
        status_.session_id = 1;
    }
    transitionTo(ConnectionState::Session);
    
    return true;
}

void FSoEConnection::triggerFailSafe(uint16_t error_code)
{
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
    
    transitionTo(ConnectionState::FailSafe);
    
    if (!was_fail_safe && fail_safe_callback_) {
        fail_safe_callback_();
    }
}

bool FSoEConnection::clearError()
{
    if (status_.state != ConnectionState::Error &&
        status_.state != ConnectionState::FailSafe) {
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
    
    // Validate connection ID
    const auto* header = reinterpret_cast<const FSoEHeader*>(data);
    uint16_t conn_id = 0;
    if (header->command == Command::ProcessData) {
        conn_id = static_cast<uint16_t>(header->conn_id_low |
                                        ((header->conn_id_high & 0x0F) << 8));
    } else {
        conn_id = static_cast<uint16_t>(header->conn_id_low | (header->conn_id_high << 8));
    }
    
    if (!validateConnectionID(conn_id)) {
        handleError(ErrorCode::ConnectionIDError);
        return false;
    }
    
    // Update watchdog
    status_.last_valid_frame_ms = current_time_ms_;
    
    // Process based on current state
    switch (status_.state) {
        case ConnectionState::Reset:
            handleResetState();
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
        case ConnectionState::Error:
            // Don't process frames in these states
            break;
    }
    
    return true;
}

size_t FSoEConnection::prepareTxFrame(uint8_t* data, size_t max_len)
{
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
            // Parameter frames similar to data frames
            len = buildDataFrame(data, max_len);
            break;
            
        case ConnectionState::Data:
            len = buildDataFrame(data, max_len);
            break;
            
        case ConnectionState::FailSafe:
            len = buildFailSafeFrame(data, max_len);
            break;
            
        case ConnectionState::Error:
            // No frames in error state
            break;
    }
    
    if (len > 0) {
        stats_.frames_sent++;
        if (status_.state == ConnectionState::Parameter ||
            status_.state == ConnectionState::Data ||
            status_.state == ConnectionState::FailSafe) {
            tx_sequence_++;
        }
    }
    
    return len;
}

void FSoEConnection::update(uint64_t current_time_ms)
{
    current_time_ms_ = current_time_ms;
    stats_.uptime_ms = current_time_ms;
    
    // Check watchdog
    checkWatchdog(current_time_ms);
}

// ============================================================================
// State Handlers
// ============================================================================

void FSoEConnection::handleResetState()
{
    // Start session establishment
    requestSessionReset();
}

void FSoEConnection::handleSessionState(const uint8_t* data, size_t len)
{
    const auto* header = reinterpret_cast<const FSoEHeader*>(data);
    
    if (header->command == Command::Session) {
        // Session acknowledged, move to connection
        transitionTo(ConnectionState::Connection);
    }
}

void FSoEConnection::handleConnectionState(const uint8_t* data, size_t len)
{
    const auto* header = reinterpret_cast<const FSoEHeader*>(data);
    
    if (header->command == Command::Connection ||
        header->command == Command::ParameterResp) {
        // Some simulated slaves acknowledge the connection by immediately
        // advancing to the parameter phase and returning a parameter response.
        if (config_.input_size > 0 || config_.output_size > 0) {
            transitionTo(ConnectionState::Parameter);
        } else {
            transitionTo(ConnectionState::Data);
        }
    }
}

void FSoEConnection::handleParameterState(const uint8_t* data, size_t len)
{
    // Use memcpy instead of reinterpret_cast for safety
    FSoEHeader header;
    std::memcpy(&header, data, sizeof(header));
    
    if (header.command == Command::ParameterResp) {
        transitionTo(ConnectionState::Data);
    } else if (header.command == Command::ProcessData) {
        transitionTo(ConnectionState::Data);
        handleDataState(data, len);
    }
}

void FSoEConnection::handleDataState(const uint8_t* data, size_t len)
{
    const auto* header = reinterpret_cast<const FSoEHeader*>(data);
    
    if (header->command != Command::ProcessData) {
        return;
    }
    
    // Validate sequence
    uint8_t seq = (header->conn_id_high >> 4) & 0x0F;  // Sequence in upper nibble
    if (!validateSequence(seq)) {
        stats_.sequence_errors++;
        handleError(ErrorCode::SequenceError);
        return;
    }
    
    rx_sequence_ = seq;
    
    // Extract safety data
    const uint8_t* safe_data = data + sizeof(FSoEHeader);
    size_t data_len = len - sizeof(FSoEHeader) - 2;  // Minus CRC
    
    if (data_len >= config_.input_size) {
        std::copy(safe_data, safe_data + config_.input_size, safe_inputs_.begin());
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
    if (max_len < sizeof(FSoESessionReset)) return 0;
    
    auto* frame = reinterpret_cast<FSoESessionReset*>(data);
    
    frame->header.command = Command::Session;
    frame->header.conn_id_low = config_.connection_id & 0xFF;
    frame->header.conn_id_high = (config_.connection_id >> 8) & 0xFF;
    frame->session_id = status_.session_id;
    
    // Calculate CRC
    frame->crc = CRC::calculateFSoECRC(data, sizeof(FSoESessionReset) - 2);
    
    return sizeof(FSoESessionReset);
}

size_t FSoEConnection::buildConnectionFrame(uint8_t* data, size_t max_len)
{
    if (max_len < sizeof(FSoEConnectionFrame)) return 0;
    
    auto* frame = reinterpret_cast<struct FSoEConnectionFrame*>(data);
    
    frame->header.command = Command::Connection;
    frame->header.conn_id_low = config_.connection_id & 0xFF;
    frame->header.conn_id_high = (config_.connection_id >> 8) & 0xFF;
    frame->conn_id = config_.connection_id;
    frame->slave_addr = config_.slave_addr;
    frame->sl_param_crc = 0;  // Would contain parameter CRC
    frame->reserved = 0;
    
    // Calculate CRC
    frame->crc = CRC::calculateFSoECRC(data, sizeof(struct FSoEConnectionFrame) - 2);
    
    return sizeof(struct FSoEConnectionFrame);
}

size_t FSoEConnection::buildDataFrame(uint8_t* data, size_t max_len)
{
    size_t frame_size = sizeof(FSoEHeader) + config_.output_size + 2;  // +2 for CRC
    
    if (max_len < frame_size) return 0;
    
    auto* header = reinterpret_cast<FSoEHeader*>(data);
    
    header->command = Command::ProcessData;
    header->conn_id_low = config_.connection_id & 0xFF;
    header->conn_id_high = ((config_.connection_id >> 8) & 0x0F) | ((tx_sequence_ & 0x0F) << 4);
    
    // Copy safety data
    std::copy(safe_outputs_.begin(), safe_outputs_.begin() + config_.output_size,
              data + sizeof(FSoEHeader));
    
    // Calculate CRC
    uint16_t crc = CRC::calculateFSoECRC(data, frame_size - 2);
    data[frame_size - 2] = crc & 0xFF;
    data[frame_size - 1] = (crc >> 8) & 0xFF;
    
    return frame_size;
}

size_t FSoEConnection::buildFailSafeFrame(uint8_t* data, size_t max_len)
{
    if (max_len < sizeof(FSoEFailSafe)) return 0;
    
    auto* frame = reinterpret_cast<FSoEFailSafe*>(data);
    
    frame->header.command = Command::Reset;  // Fail-safe uses reset command
    frame->header.conn_id_low = config_.connection_id & 0xFF;
    frame->header.conn_id_high = (config_.connection_id >> 8) & 0xFF;
    
    // Copy fail-safe values
    std::copy(config_.fail_safe_values.begin(), config_.fail_safe_values.end(),
              frame->fail_safe_data);
    
    // Calculate CRC
    frame->crc = CRC::calculateFSoECRC(data, sizeof(FSoEFailSafe) - 2);
    
    return sizeof(FSoEFailSafe);
}

// ============================================================================
// Frame Validation
// ============================================================================

bool FSoEConnection::validateFrame(const uint8_t* data, size_t len)
{
    if (!data || len < sizeof(FSoEHeader) + 2) {
        return false;
    }
    
    return validateCRC(data, len);
}

bool FSoEConnection::validateCRC(const uint8_t* data, size_t len)
{
    if (len < 2) return false;
    
    uint16_t received_crc = data[len - 2] | (data[len - 1] << 8);
    
    return CRC::verifyFSoECRC(data, len - 2, received_crc);
}

bool FSoEConnection::validateSequence(uint8_t seq)
{
    // Check sequence number matches expected next value
    uint8_t expected = (rx_sequence_ + 1) & 0x0F;
    return seq == expected;
}

bool FSoEConnection::validateConnectionID(uint16_t conn_id)
{
    return (conn_id & 0x0FFFu) == (config_.connection_id & 0x0FFFu);
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
    transitionTo(ConnectionState::Error);
    
    // Trigger fail-safe
    if (!status_.fail_safe_active) {
        triggerFailSafe(error_code);
    } else {
        transitionTo(ConnectionState::FailSafe);
    }
    
    if (error_callback_) {
        error_callback_(error_code);
    }
}

void FSoEConnection::checkWatchdog(uint64_t current_time_ms)
{
    if (status_.state != ConnectionState::Data) return;
    
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
    if (!data || len != config_.output_size) return false;
    if (status_.isFailSafe()) return false;  // Don't allow writes in fail-safe
    
    std::copy(data, data + len, safe_outputs_.begin());
    return true;
}

bool FSoEConnection::writeOutputProcessData(std::span<const uint8_t> data)
{
    return setSafeOutputs(data.data(), data.size());
}

size_t FSoEConnection::getSafeInputs(uint8_t* data, size_t len) const
{
    if (!data || len < config_.input_size) return 0;
    if (!status_.data_valid) return 0;
    
    std::copy(safe_inputs_.begin(), safe_inputs_.begin() + config_.input_size, data);
    return config_.input_size;
}

size_t FSoEConnection::readInputProcessData(std::span<uint8_t> data) const
{
    return getSafeInputs(data.data(), data.size());
}

std::vector<uint8_t> FSoEConnection::inputProcessData() const
{
    std::vector<uint8_t> data(config_.input_size, 0);
    const size_t copied = getSafeInputs(data.data(), data.size());
    data.resize(copied);
    return data;
}

std::vector<uint8_t> FSoEConnection::outputProcessData() const
{
    return std::vector<uint8_t>(safe_outputs_.begin(),
                                safe_outputs_.begin() + config_.output_size);
}

bool FSoEConnection::exchangeWith(FSoESlave& slave, uint64_t current_time_ms)
{
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
    if (!status_.data_valid) return false;
    
    uint8_t byte_idx = bit_index / 8;
    uint8_t bit_pos = bit_index % 8;
    
    if (byte_idx >= config_.input_size) return false;
    
    return (safe_inputs_[byte_idx] >> bit_pos) & 1;
}

bool FSoEConnection::setSafeOutputBit(uint8_t bit_index, bool value)
{
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
    if (!status_.data_valid || byte_index >= config_.input_size) {
        return 0;
    }
    return safe_inputs_[byte_index];
}

bool FSoEConnection::setSafeOutputByte(uint8_t byte_index, uint8_t value)
{
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
    stats_ = {};
}

std::string FSoEConnection::getDiagnostics() const
{
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
    diag += "  RX Sequence: " + std::to_string(rx_sequence_) + "\n";
    diag += "  TX Sequence: " + std::to_string(tx_sequence_) + "\n";
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
    state_change_callback_ = std::move(callback);
}

void FSoEConnection::setErrorCallback(ErrorCallback callback)
{
    error_callback_ = std::move(callback);
}

void FSoEConnection::setFailSafeCallback(FailSafeCallback callback)
{
    fail_safe_callback_ = std::move(callback);
}

void FSoEConnection::setDataCallback(DataCallback callback)
{
    data_callback_ = std::move(callback);
}

// ============================================================================
// FSoEMaster Implementation
// ============================================================================

FSoEMaster::FSoEMaster() = default;
FSoEMaster::~FSoEMaster() = default;

bool FSoEMaster::addConnection(const ConnectionConfig& config)
{
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
    for (auto& conn : connections_) {
        if (conn->getConfig().connection_id == connection_id) {
            return conn.get();
        }
    }
    return nullptr;
}

FSoEConnection* FSoEMaster::getConnectionBySlaveAddr(uint16_t slave_addr)
{
    for (auto& conn : connections_) {
        if (conn->getConfig().slave_addr == slave_addr) {
            return conn.get();
        }
    }
    return nullptr;
}

bool FSoEMaster::startAll()
{
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
    for (auto& conn : connections_) {
        conn->resetConnection();
    }
}

void FSoEMaster::update(uint64_t current_time_ms)
{
    for (auto& conn : connections_) {
        conn->update(current_time_ms);
    }
}

bool FSoEMaster::allOperational() const
{
    for (const auto& conn : connections_) {
        if (!conn->isOperational()) {
            return false;
        }
    }
    return !connections_.empty();
}

bool FSoEMaster::anyFailSafe() const
{
    for (const auto& conn : connections_) {
        if (conn->isFailSafe()) {
            return true;
        }
    }
    return false;
}

std::string FSoEMaster::getDiagnostics() const
{
    std::string diag;
    
    diag += "FSoE Master Diagnostics:\n";
    diag += "  Connections: " + std::to_string(connections_.size()) + "\n";
    diag += "  All Operational: " + std::string(allOperational() ? "Yes" : "No") + "\n";
    diag += "  Any Fail-Safe: " + std::string(anyFailSafe() ? "Yes" : "No") + "\n";
    
    for (size_t i = 0; i < connections_.size(); ++i) {
        diag += "\n--- Connection " + std::to_string(i) + " ---\n";
        diag += connections_[i]->getDiagnostics();
    }
    
    return diag;
}

} // namespace FSoE
