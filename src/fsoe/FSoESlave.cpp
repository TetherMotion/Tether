/**
 * @file FSoESlave.cpp
 * @brief FSoE Slave Implementation
 */

#include "fsoe/FSoESlave.hpp"
#include "fsoe/FSoECRC.hpp"
#include <algorithm>
#include <cstring>
#include <cstdio>

namespace FSoE {

// ============================================================================
// FSoESlave Implementation
// ============================================================================

FSoESlave::FSoESlave(const FSoESlaveConfig& config)
    : config_(config)
{
}

FSoESlave::~FSoESlave() = default;

bool FSoESlave::initialize() {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    
    if (initialized_) {
        return true;
    }
    
    // Validate configuration
    if (config_.safeInputSize > 16 || config_.safeOutputSize > 16) {
        return false;
    }
    
    // Initialize buffers with fail-safe values
    std::copy(config_.failSafeInputs.begin(), 
              config_.failSafeInputs.begin() + config_.safeInputSize,
              safeInputs_.begin());
    std::copy(config_.failSafeOutputs.begin(),
              config_.failSafeOutputs.begin() + config_.safeOutputSize,
              safeOutputs_.begin());
    
    // Reset state machine
    state_ = ConnectionState::Reset;
    lastError_ = ErrorCode::NoError;
    failSafeActive_ = false;
    dataValid_ = false;
    sessionId_ = 0;
    currentConnectionId_ = 0;
    expectedSequence_ = 0;
    txSequence_ = 0;
    
    // Reset statistics
    stats_.reset();
    diagnostics_.clear();
    
    initialized_ = true;
    
    logDiagnostic(ErrorCode::NoError, "FSoE slave initialized");
    
    return true;
}

bool FSoESlave::reconfigure(const FSoESlaveConfig& config) {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    
    if (state_ != ConnectionState::Reset) {
        return false;
    }
    
    config_ = config;
    initialized_ = false;
    
    return initialize();
}

const char* FSoESlave::getStateName() const {
    switch (state_.load()) {
        case ConnectionState::Reset:      return "RESET";
        case ConnectionState::Session:    return "SESSION";
        case ConnectionState::Connection: return "CONNECTION";
        case ConnectionState::Parameter:  return "PARAMETER";
        case ConnectionState::Data:       return "DATA";
        case ConnectionState::FailSafe:   return "FAIL_SAFE";
        case ConnectionState::Error:      return "ERROR";
        default:                          return "UNKNOWN";
    }
}

void FSoESlave::reset() {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    
    transitionTo(ConnectionState::Reset);
    lastError_ = ErrorCode::NoError;
    failSafeActive_ = false;
    dataValid_ = false;
    sessionId_ = 0;
    currentConnectionId_ = 0;
    expectedSequence_ = 0;
    txSequence_ = 0;
    
    // Apply fail-safe values
    applyFailSafeOutputs();
    
    stats_.sessionResets++;
    logDiagnostic(ErrorCode::NoError, "FSoE slave reset");
}

void FSoESlave::triggerFailSafe(uint16_t errorCode) {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    
    if (state_ == ConnectionState::FailSafe) {
        return;  // Already in fail-safe
    }
    
    lastError_ = errorCode;
    failSafeActive_ = true;
    dataValid_ = false;
    
    transitionTo(ConnectionState::FailSafe);
    applyFailSafeOutputs();
    
    stats_.failSafeActivations++;
    
    char msg[64];
    snprintf(msg, sizeof(msg), "Fail-safe triggered: 0x%04X", errorCode);
    logDiagnostic(errorCode, msg);
    
    if (failSafeCallback_) {
        failSafeCallback_();
    }
}

bool FSoESlave::attemptRecovery() {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    
    if (state_ != ConnectionState::FailSafe) {
        return false;
    }
    
    if (!config_.autoRecoveryEnabled) {
        return false;
    }
    
    if (errorInjection_.enabled && errorInjection_.preventRecovery) {
        return false;
    }
    
    // Check recovery callback
    if (recoveryCallback_ && !recoveryCallback_()) {
        return false;
    }
    
    stats_.recoveryAttempts++;
    
    // Attempt to restart session
    transitionTo(ConnectionState::Reset);
    lastError_ = ErrorCode::NoError;
    failSafeActive_ = false;
    
    logDiagnostic(ErrorCode::NoError, "Recovery attempt initiated");
    
    return true;
}

void FSoESlave::transitionTo(uint8_t newState) {
    uint8_t oldState = state_.load();
    
    if (oldState == newState) {
        return;
    }
    
    state_ = newState;
    stateEntryTimeMs_ = lastUpdateTimeMs_;
    
    if (stateCallback_) {
        stateCallback_(oldState, newState);
    }
    
    char msg[64];
    snprintf(msg, sizeof(msg), "State: %d -> %d", oldState, newState);
    logDiagnostic(ErrorCode::NoError, msg);
}

bool FSoESlave::processRxFrame(const uint8_t* data, size_t len) {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    
    if (!initialized_) {
        return false;
    }
    
    stats_.framesReceived++;
    
    // Check for frame drop injection
    if (errorInjection_.enabled && shouldDropFrame()) {
        stats_.invalidFrames++;
        return false;
    }
    
    // Check for forced fail-safe
    if (errorInjection_.enabled && errorInjection_.forceFailSafe) {
        triggerFailSafe(ErrorCode::ApplicationError);
        return false;
    }
    
    // Validate frame
    if (!validateFrame(data, len)) {
        stats_.invalidFrames++;
        return false;
    }
    
    stats_.validFrames++;
    lastValidFrameMs_ = lastUpdateTimeMs_;
    stats_.lastValidFrameTime = lastValidFrameMs_;
    
    // Get command from header
    uint8_t command = data[0];
    
    // Process based on current state and command
    switch (state_.load()) {
        case ConnectionState::Reset:
        case ConnectionState::Session:
            if (command == Command::Session || command == Command::Reset) {
                processSessionReset(data, len);
            } else if (command == Command::Connection) {
                processConnection(data, len);
            }
            break;
            
        case ConnectionState::Connection:
            if (command == Command::Connection) {
                processConnection(data, len);
            }
            break;
            
        case ConnectionState::Parameter:
            if (command == Command::ParameterReq) {
                processParameter(data, len);
            } else if (command == Command::ProcessData) {
                // Skip parameter phase
                transitionTo(ConnectionState::Data);
                processData(data, len);
            }
            break;
            
        case ConnectionState::Data:
            if (command == Command::ProcessData) {
                processData(data, len);
            } else if (command == Command::Reset) {
                processSessionReset(data, len);
            }
            break;
            
        case ConnectionState::FailSafe:
            if (command == Command::Reset) {
                // Master requesting reset after fail-safe
                if (config_.autoRecoveryEnabled && !errorInjection_.preventRecovery) {
                    processSessionReset(data, len);
                    stats_.successfulRecoveries++;
                }
            }
            break;
            
        case ConnectionState::Error:
            if (command == Command::Reset) {
                // Reset from error state
                reset();
            }
            break;
            
        default:
            break;
    }
    
    return true;
}

size_t FSoESlave::prepareTxFrame(uint8_t* data, size_t maxLen) {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    
    if (!initialized_ || maxLen < 8) {
        return 0;
    }
    
    size_t frameSize = 0;
    
    // Check for delayed response injection
    if (errorInjection_.enabled && errorInjection_.delayResponse) {
        // Simulate delay by not responding
        return 0;
    }
    
    switch (state_.load()) {
        case ConnectionState::Reset:
        case ConnectionState::Session:
            frameSize = buildSessionResponse(data, maxLen);
            break;
            
        case ConnectionState::Connection:
            frameSize = buildConnectionResponse(data, maxLen);
            break;
            
        case ConnectionState::Parameter:
            frameSize = buildParameterResponse(data, maxLen);
            break;
            
        case ConnectionState::Data:
            frameSize = buildDataResponse(data, maxLen);
            break;
            
        case ConnectionState::FailSafe:
            frameSize = buildFailSafeResponse(data, maxLen);
            break;
            
        default:
            break;
    }
    
    // Apply CRC error injection
    if (frameSize > 2 && errorInjection_.enabled && shouldInjectCRCError()) {
        // Corrupt the CRC
        data[frameSize - 1] ^= 0xFF;
        stats_.crcErrors++;  // Track injected errors
    }
    
    // Apply data corruption
    if (frameSize > 0 && errorInjection_.enabled && errorInjection_.corruptData) {
        applyDataCorruption(data, frameSize);
    }
    
    if (frameSize > 0) {
        stats_.framesSent++;
    }
    
    return frameSize;
}

void FSoESlave::update(uint64_t currentTimeMs) {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    
    if (!initialized_) {
        return;
    }
    
    // Calculate cycle time
    if (lastUpdateTimeMs_ > 0) {
        uint32_t cycleUs = (currentTimeMs - lastUpdateTimeMs_) * 1000;
        if (cycleUs < stats_.minCycleTimeUs) stats_.minCycleTimeUs = cycleUs;
        if (cycleUs > stats_.maxCycleTimeUs) stats_.maxCycleTimeUs = cycleUs;
        // Simple moving average
        stats_.avgCycleTimeUs = (stats_.avgCycleTimeUs * 7 + cycleUs) / 8;
    }
    
    lastUpdateTimeMs_ = currentTimeMs;
    
    // Handle watchdog
    handleWatchdog(currentTimeMs);
    
    // Handle state timeouts
    handleTimeout(currentTimeMs);
    
    // Check for simulated watchdog timeout
    if (errorInjection_.enabled && errorInjection_.simulateWatchdogTimeout) {
        if (currentTimeMs - lastValidFrameMs_ > errorInjection_.watchdogDelayMs) {
            handleError(ErrorCode::WatchdogError, config_.treatTimeoutAsCritical);
        }
    }
}

bool FSoESlave::setSafeInputs(const uint8_t* data, size_t len) {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    
    if (len > config_.safeInputSize) {
        return false;
    }
    
    std::copy(data, data + len, safeInputs_.begin());
    return true;
}

bool FSoESlave::writeInputProcessData(std::span<const uint8_t> data)
{
    return setSafeInputs(data.data(), data.size());
}

size_t FSoESlave::getSafeOutputs(uint8_t* data, size_t len) const {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    
    if (!dataValid_ || len < config_.safeOutputSize) {
        return 0;
    }
    
    std::copy(safeOutputs_.begin(), 
              safeOutputs_.begin() + config_.safeOutputSize,
              data);
    
    return config_.safeOutputSize;
}

size_t FSoESlave::readOutputProcessData(std::span<uint8_t> data) const
{
    return getSafeOutputs(data.data(), data.size());
}

std::vector<uint8_t> FSoESlave::inputProcessData() const
{
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    return std::vector<uint8_t>(safeInputs_.begin(),
                                safeInputs_.begin() + config_.safeInputSize);
}

std::vector<uint8_t> FSoESlave::outputProcessData() const
{
    std::vector<uint8_t> data(config_.safeOutputSize, 0);
    const size_t copied = getSafeOutputs(data.data(), data.size());
    data.resize(copied);
    return data;
}

bool FSoESlave::getSafeOutputBit(uint8_t bitIndex) const {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    
    uint8_t byteIndex = bitIndex / 8;
    uint8_t bitOffset = bitIndex % 8;
    
    if (byteIndex >= config_.safeOutputSize) {
        return false;
    }
    
    return (safeOutputs_[byteIndex] >> bitOffset) & 0x01;
}

bool FSoESlave::setSafeInputBit(uint8_t bitIndex, bool value) {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    
    uint8_t byteIndex = bitIndex / 8;
    uint8_t bitOffset = bitIndex % 8;
    
    if (byteIndex >= config_.safeInputSize) {
        return false;
    }
    
    if (value) {
        safeInputs_[byteIndex] |= (1 << bitOffset);
    } else {
        safeInputs_[byteIndex] &= ~(1 << bitOffset);
    }
    
    return true;
}

void FSoESlave::applyFailSafeOutputs() {
    std::copy(config_.failSafeOutputs.begin(),
              config_.failSafeOutputs.begin() + config_.safeOutputSize,
              safeOutputs_.begin());
}

// ============================================================================
// Internal Frame Validation
// ============================================================================

bool FSoESlave::validateFrame(const uint8_t* data, size_t len) {
    // Minimum frame size: header(3) + CRC(2) = 5
    if (len < 5) {
        handleError(ErrorCode::DataLengthError, false);
        stats_.dataLengthErrors++;
        return false;
    }
    
    // Validate CRC
    if (!validateCRC(data, len)) {
        return false;
    }
    
    // Get connection ID from header
    uint16_t connId = 0;
    if (data[0] == Command::ProcessData) {
        connId = static_cast<uint16_t>(data[1] | ((data[2] & 0x0F) << 8));
    } else {
        connId = static_cast<uint16_t>(data[1] | (data[2] << 8));
    }
    
    // Validate connection ID (in DATA state)
    if (state_ == ConnectionState::Data) {
        if (!validateConnectionId(connId)) {
            return false;
        }
    }
    
    return true;
}

bool FSoESlave::validateCRC(const uint8_t* data, size_t len) {
    // Check for CRC error injection
    if (errorInjection_.enabled && errorInjection_.injectCRCError) {
        // Pretend CRC failed
        handleError(ErrorCode::CRCError, config_.treatCrcErrorAsCritical);
        stats_.crcErrors++;
        return false;
    }
    
    if (len < 2) {
        return false;
    }

    const uint16_t stored_crc = static_cast<uint16_t>(data[len - 2]) |
                                (static_cast<uint16_t>(data[len - 1]) << 8);
    if (!verifyCRC16(data, len - 2, stored_crc)) {
        if (config_.strictCrcCheck) {
            handleError(ErrorCode::CRCError, config_.treatCrcErrorAsCritical);
            stats_.crcErrors++;
            return false;
        }
    }
    
    return true;
}

bool FSoESlave::validateSequence(uint8_t seqNum) {
    // Check for sequence error injection
    if (errorInjection_.enabled && errorInjection_.injectSequenceError) {
        seqNum += errorInjection_.sequenceOffset;
    }
    
    if (seqNum != expectedSequence_) {
        if (config_.strictSequenceCheck) {
            handleError(ErrorCode::SequenceError, config_.treatSequenceErrorAsCritical);
            stats_.sequenceErrors++;
            return false;
        }
    }
    
    expectedSequence_ = (seqNum + 1) & 0xFF;
    return true;
}

bool FSoESlave::validateConnectionId(uint16_t connId) {
    // Check for connection ID error injection
    if (errorInjection_.enabled && errorInjection_.injectConnIdError) {
        connId = errorInjection_.fakeConnId;
    }
    
    if ((connId & 0x0FFFu) != (currentConnectionId_ & 0x0FFFu)) {
        handleError(ErrorCode::ConnectionIDError, config_.treatConnIdErrorAsCritical);
        stats_.connectionIdErrors++;
        return false;
    }
    
    return true;
}

uint16_t FSoESlave::calculateCRC(const uint8_t* data, size_t len) {
    return calculateCRC16(data, len);
}

// ============================================================================
// Frame Processing
// ============================================================================

void FSoESlave::processSessionReset(const uint8_t* data, size_t len) {
    (void)len;
    
    // Extract session ID
    if (len >= 7) {
        sessionId_ = data[3] | (data[4] << 8);
    } else {
        sessionId_ = 0;
    }
    
    // Reset sequence counters
    expectedSequence_ = 0;
    txSequence_ = 0;
    
    // Clear error state
    lastError_ = ErrorCode::NoError;
    failSafeActive_ = false;
    
    transitionTo(ConnectionState::Session);
    
    stats_.sessionResets++;
    logDiagnostic(ErrorCode::NoError, "Session reset received");
}

void FSoESlave::processConnection(const uint8_t* data, size_t len) {
    (void)len;
    
    // Extract connection ID
    currentConnectionId_ = data[3] | (data[4] << 8);
    
    // Validate safety address if present
    if (len >= 9) {
        uint16_t safetyAddr = data[5] | (data[6] << 8);
        if (safetyAddr != 0 && safetyAddr != config_.safetyAddress) {
            handleError(ErrorCode::ConnectionIDError, true);
            return;
        }
    }
    
    transitionTo(ConnectionState::Parameter);
    
    char msg[64];
    snprintf(msg, sizeof(msg), "Connection established: ID=0x%04X", currentConnectionId_);
    logDiagnostic(ErrorCode::NoError, msg);
}

void FSoESlave::processParameter(const uint8_t* data, size_t len) {
    (void)data;
    (void)len;
    
    // Parameter exchange - for now just acknowledge
    // Real implementation would validate safety parameters
    
    transitionTo(ConnectionState::Data);
    
    logDiagnostic(ErrorCode::NoError, "Parameters accepted");
}

void FSoESlave::processData(const uint8_t* data, size_t len) {
    // Extract sequence number from data
    if (len > 3) {
        uint8_t seqNum = (data[2] >> 4) & 0x0F;  // High nibble of conn_id_high
        if (!validateSequence(seqNum)) {
            return;
        }
    }
    
    // Extract safe output data
    size_t dataOffset = 3;  // After header
    size_t dataLen = len - dataOffset - 2;  // Minus header and CRC
    
    if (dataLen >= config_.safeOutputSize) {
        std::copy(data + dataOffset, 
                  data + dataOffset + config_.safeOutputSize,
                  safeOutputs_.begin());
        
        dataValid_ = true;
        
        if (dataValidCallback_) {
            dataValidCallback_(safeOutputs_.data(), config_.safeOutputSize);
        }
    }
}

// ============================================================================
// Frame Building
// ============================================================================

size_t FSoESlave::buildSessionResponse(uint8_t* data, size_t maxLen) {
    if (maxLen < 7) return 0;
    
    data[0] = Command::Session;
    data[1] = config_.connectionId & 0xFF;
    data[2] = (config_.connectionId >> 8) & 0xFF;
    data[3] = sessionId_ & 0xFF;
    data[4] = (sessionId_ >> 8) & 0xFF;
    
    // Calculate and append CRC
    uint16_t crc = calculateCRC(data, 5);
    data[5] = crc & 0xFF;
    data[6] = (crc >> 8) & 0xFF;
    
    return 7;
}

size_t FSoESlave::buildConnectionResponse(uint8_t* data, size_t maxLen) {
    if (maxLen < 9) return 0;
    
    data[0] = Command::Connection;
    data[1] = currentConnectionId_ & 0xFF;
    data[2] = (currentConnectionId_ >> 8) & 0xFF;
    data[3] = config_.safetyAddress & 0xFF;
    data[4] = (config_.safetyAddress >> 8) & 0xFF;
    data[5] = config_.safetyLevel;
    data[6] = 0;  // Reserved
    
    // Calculate and append CRC
    uint16_t crc = calculateCRC(data, 7);
    data[7] = crc & 0xFF;
    data[8] = (crc >> 8) & 0xFF;
    
    return 9;
}

size_t FSoESlave::buildParameterResponse(uint8_t* data, size_t maxLen) {
    if (maxLen < 7) return 0;
    
    data[0] = Command::ParameterResp;
    data[1] = currentConnectionId_ & 0xFF;
    data[2] = (currentConnectionId_ >> 8) & 0xFF;
    data[3] = 0;  // Parameter ACK
    data[4] = 0;
    
    // Calculate and append CRC
    uint16_t crc = calculateCRC(data, 5);
    data[5] = crc & 0xFF;
    data[6] = (crc >> 8) & 0xFF;
    
    return 7;
}

size_t FSoESlave::buildDataResponse(uint8_t* data, size_t maxLen) {
    size_t frameLen = 3 + config_.safeInputSize + 2;  // Header + data + CRC
    
    if (maxLen < frameLen) return 0;
    
    data[0] = Command::ProcessData;
    data[1] = currentConnectionId_ & 0xFF;
    data[2] = ((currentConnectionId_ >> 8) & 0x0F) | ((txSequence_ & 0x0F) << 4);
    
    // Copy safe input data
    std::copy(safeInputs_.begin(),
              safeInputs_.begin() + config_.safeInputSize,
              data + 3);
    
    // Calculate and append CRC
    uint16_t crc = calculateCRC(data, 3 + config_.safeInputSize);
    data[3 + config_.safeInputSize] = crc & 0xFF;
    data[4 + config_.safeInputSize] = (crc >> 8) & 0xFF;
    
    txSequence_ = (txSequence_ + 1) & 0x0F;
    
    return frameLen;
}

size_t FSoESlave::buildFailSafeResponse(uint8_t* data, size_t maxLen) {
    size_t frameLen = 3 + config_.safeInputSize + 2 + 2;  // Header + data + error + CRC
    
    if (maxLen < frameLen) return 0;
    
    data[0] = Command::ProcessData | 0x80;  // Fail-safe flag
    data[1] = currentConnectionId_ & 0xFF;
    data[2] = ((currentConnectionId_ >> 8) & 0x0F) | ((txSequence_ & 0x0F) << 4);
    
    // Copy fail-safe input values
    std::copy(config_.failSafeInputs.begin(),
              config_.failSafeInputs.begin() + config_.safeInputSize,
              data + 3);
    
    // Add error code
    data[3 + config_.safeInputSize] = lastError_ & 0xFF;
    data[4 + config_.safeInputSize] = (lastError_ >> 8) & 0xFF;
    
    // Calculate and append CRC
    uint16_t crc = calculateCRC(data, 5 + config_.safeInputSize);
    data[5 + config_.safeInputSize] = crc & 0xFF;
    data[6 + config_.safeInputSize] = (crc >> 8) & 0xFF;
    
    txSequence_ = (txSequence_ + 1) & 0x0F;
    
    return frameLen;
}

// ============================================================================
// Error and Watchdog Handling
// ============================================================================

void FSoESlave::handleWatchdog(uint64_t currentTimeMs) {
    if (state_ != ConnectionState::Data) {
        return;
    }
    
    uint64_t elapsed = currentTimeMs - lastValidFrameMs_;
    
    // Track longest gap
    if (elapsed > stats_.longestGapMs) {
        stats_.longestGapMs = elapsed;
    }
    
    // Check watchdog timeout
    if (elapsed > config_.watchdogTimeoutMs) {
        handleError(ErrorCode::WatchdogError, config_.treatTimeoutAsCritical);
        stats_.watchdogTimeouts++;
    }
}

void FSoESlave::handleTimeout(uint64_t currentTimeMs) {
    uint64_t elapsed = currentTimeMs - stateEntryTimeMs_;
    
    switch (state_.load()) {
        case ConnectionState::Session:
            if (elapsed > config_.sessionTimeoutMs) {
                handleError(ErrorCode::SessionError, false);
            }
            break;
            
        case ConnectionState::Connection:
        case ConnectionState::Parameter:
            if (elapsed > config_.connectionTimeoutMs) {
                handleError(ErrorCode::TimeoutError, false);
            }
            break;
            
        case ConnectionState::FailSafe:
            // Check recovery delay
            if (config_.autoRecoveryEnabled && elapsed > config_.recoveryDelayMs) {
                attemptRecovery();
            }
            break;
            
        default:
            break;
    }
}

void FSoESlave::handleError(uint16_t errorCode, bool isCritical) {
    lastError_ = errorCode;
    
    char msg[64];
    snprintf(msg, sizeof(msg), "Error 0x%04X (critical=%d)", errorCode, isCritical);
    logDiagnostic(errorCode, msg);
    
    if (errorCallback_) {
        errorCallback_(errorCode, isCritical);
    }
    
    if (isCritical) {
        triggerFailSafe(errorCode);
    }
}

void FSoESlave::logDiagnostic(uint16_t errorCode, const char* message) {
    if (!config_.enableDiagnostics) {
        return;
    }
    
    FSoEDiagnosticEntry entry;
    entry.timestamp = lastUpdateTimeMs_;
    entry.errorCode = errorCode;
    entry.state = state_.load();
    entry.sequenceNumber = expectedSequence_;
    entry.connectionId = currentConnectionId_;
    strncpy(entry.message, message, sizeof(entry.message) - 1);
    entry.message[sizeof(entry.message) - 1] = '\0';
    
    if (diagnostics_.size() >= config_.maxErrorLogEntries) {
        diagnostics_.erase(diagnostics_.begin());
    }
    
    diagnostics_.push_back(entry);
}

// ============================================================================
// Error Injection Helpers
// ============================================================================

bool FSoESlave::shouldInjectCRCError() {
    if (!errorInjection_.injectCRCError) {
        return false;
    }
    
    if (errorInjection_.crcErrorRate == 0) {
        return true;  // Every packet
    }
    
    errorInjection_.crcErrorCounter++;
    if (errorInjection_.crcErrorCounter >= errorInjection_.crcErrorRate) {
        errorInjection_.crcErrorCounter = 0;
        return true;
    }
    
    return false;
}

bool FSoESlave::shouldDropFrame() {
    if (!errorInjection_.dropFrames) {
        return false;
    }
    
    if (errorInjection_.dropRate == 0) {
        return true;  // Drop all
    }
    
    errorInjection_.dropCounter++;
    if (errorInjection_.dropCounter >= errorInjection_.dropRate) {
        errorInjection_.dropCounter = 0;
        return true;
    }
    
    return false;
}

void FSoESlave::applyDataCorruption(uint8_t* data, size_t len) {
    if (errorInjection_.corruptByteIndex < len) {
        data[errorInjection_.corruptByteIndex] ^= errorInjection_.corruptBitMask;
    }
}

} // namespace FSoE
