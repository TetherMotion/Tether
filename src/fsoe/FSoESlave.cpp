/**
 * @file FSoESlave.cpp
 * @brief FSoE Slave Implementation
 */

#include "fsoe/FSoESlave.hpp"
#include <algorithm>
#include <cstring>
#include <cstdio>

namespace FSoE {

// ============================================================================
// CRC-16 Implementation (FSoE polynomial 0x755B)
// ============================================================================

namespace CRC {

// Pre-computed CRC-16 table for FSoE polynomial 0x755B
static const uint16_t crcTable[256] = {
    0x0000, 0x755B, 0xEAB6, 0x9FED, 0xC16D, 0xB436, 0x2BDB, 0x5E80,
    0x9EDB, 0xEB80, 0x746D, 0x0136, 0x5FB6, 0x2AED, 0xB500, 0xC05B,
    0x29B7, 0x5CEC, 0xC301, 0xB65A, 0xE8DA, 0x9D81, 0x026C, 0x7737,
    0xB76C, 0xC237, 0x5DDA, 0x2881, 0x7601, 0x035A, 0x9CB7, 0xE9EC,
    0x536E, 0x2635, 0xB9D8, 0xCC83, 0x9203, 0xE758, 0x78B5, 0x0DEE,
    0xCDB5, 0xB8EE, 0x2703, 0x5258, 0x0CD8, 0x7983, 0xE66E, 0x9335,
    0x7AD9, 0x0F82, 0x906F, 0xE534, 0xBBB4, 0xCEEF, 0x5102, 0x2459,
    0xE402, 0x9159, 0x0EB4, 0x7BEF, 0x256F, 0x5034, 0xCFD9, 0xBA82,
    0xA6DC, 0xD387, 0x4C6A, 0x3931, 0x67B1, 0x12EA, 0x8D07, 0xF85C,
    0x3807, 0x4D5C, 0xD2B1, 0xA7EA, 0xF96A, 0x8C31, 0x13DC, 0x6687,
    0x8F6B, 0xFA30, 0x65DD, 0x1086, 0x4E06, 0x3B5D, 0xA4B0, 0xD1EB,
    0x11B0, 0x64EB, 0xFB06, 0x8E5D, 0xD0DD, 0xA586, 0x3A6B, 0x4F30,
    0xF5B2, 0x80E9, 0x1F04, 0x6A5F, 0x34DF, 0x4184, 0xDE69, 0xAB32,
    0x6B69, 0x1E32, 0x81DF, 0xF484, 0xAA04, 0xDF5F, 0x40B2, 0x35E9,
    0xDC05, 0xA95E, 0x36B3, 0x43E8, 0x1D68, 0x6833, 0xF7DE, 0x8285,
    0x42DE, 0x3785, 0xA868, 0xDD33, 0x83B3, 0xF6E8, 0x6905, 0x1C5E,
    0x51B9, 0x24E2, 0xBB0F, 0xCE54, 0x90D4, 0xE58F, 0x7A62, 0x0F39,
    0xCF62, 0xBA39, 0x25D4, 0x508F, 0x0E0F, 0x7B54, 0xE4B9, 0x91E2,
    0x780E, 0x0D55, 0x92B8, 0xE7E3, 0xB963, 0xCC38, 0x53D5, 0x268E,
    0xE6D5, 0x938E, 0x0C63, 0x7938, 0x27B8, 0x52E3, 0xCD0E, 0xB855,
    0x02D7, 0x778C, 0xE861, 0x9D3A, 0xC3BA, 0xB6E1, 0x290C, 0x5C57,
    0x9C0C, 0xE957, 0x76BA, 0x03E1, 0x5D61, 0x283A, 0xB7D7, 0xC28C,
    0x2B60, 0x5E3B, 0xC1D6, 0xB48D, 0xEA0D, 0x9F56, 0x00BB, 0x75E0,
    0xB5BB, 0xC0E0, 0x5F0D, 0x2A56, 0x74D6, 0x018D, 0x9E60, 0xEB3B,
    0xF765, 0x823E, 0x1DD3, 0x6888, 0x3608, 0x4353, 0xDCBE, 0xA9E5,
    0x69BE, 0x1CE5, 0x8308, 0xF653, 0xA8D3, 0xDD88, 0x4265, 0x373E,
    0xDED2, 0xAB89, 0x3464, 0x413F, 0x1FBF, 0x6AE4, 0xF509, 0x8052,
    0x4009, 0x3552, 0xAABF, 0xDFE4, 0x8164, 0xF43F, 0x6BD2, 0x1E89,
    0xA40B, 0xD150, 0x4EBD, 0x3BE6, 0x6566, 0x103D, 0x8FD0, 0xFA8B,
    0x3AD0, 0x4F8B, 0xD066, 0xA53D, 0xFBBD, 0x8EE6, 0x110B, 0x6450,
    0x8DBC, 0xF8E7, 0x670A, 0x1251, 0x4CD1, 0x398A, 0xA667, 0xD33C,
    0x1367, 0x663C, 0xF9D1, 0x8C8A, 0xD20A, 0xA751, 0x38BC, 0x4DE7
};

uint16_t calculateFSoECRC(const uint8_t* data, size_t len) {
    uint16_t crc = 0xFFFF;  // Initial value
    
    for (size_t i = 0; i < len; i++) {
        uint8_t index = (crc ^ data[i]) & 0xFF;
        crc = (crc >> 8) ^ crcTable[index];
    }
    
    return crc ^ 0xFFFF;  // Final XOR
}

bool verifyFSoECRC(const uint8_t* data, size_t len) {
    if (len < 2) return false;
    
    // Calculate CRC over data (excluding the CRC field)
    uint16_t calculated = calculateFSoECRC(data, len - 2);
    
    // Get stored CRC (little-endian)
    uint16_t stored = data[len - 2] | (data[len - 1] << 8);
    
    return calculated == stored;
}

} // namespace CRC

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
