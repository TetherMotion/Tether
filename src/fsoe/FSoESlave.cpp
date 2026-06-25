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
// CRC-16 Implementation (FSoE Safety polynomial 0x139B7 per ETG.5100 §8.1.3.2)
// ============================================================================

namespace CRC {

// Pre-computed CRC-16 table for FSoE Safety polynomial 0x139B7 (16-bit: 0x39B7)
static const uint16_t crcTable[256] = {
    0x0000, 0x22CF, 0x36F1, 0x143E, 0x1E8D, 0x3C42, 0x287C, 0x0AB3,
    0x3D1A, 0x1FD5, 0x0BEB, 0x2924, 0x2397, 0x0158, 0x1566, 0x37A9,
    0x095B, 0x2B94, 0x3FAA, 0x1D65, 0x17D6, 0x3519, 0x2127, 0x03E8,
    0x3441, 0x168E, 0x02B0, 0x207F, 0x2ACC, 0x0803, 0x1C3D, 0x3EF2,
    0x12B6, 0x3079, 0x2447, 0x0688, 0x0C3B, 0x2EF4, 0x3ACA, 0x1805,
    0x2FAC, 0x0D63, 0x195D, 0x3B92, 0x3121, 0x13EE, 0x07D0, 0x251F,
    0x1BED, 0x3922, 0x2D1C, 0x0FD3, 0x0560, 0x27AF, 0x3391, 0x115E,
    0x26F7, 0x0438, 0x1006, 0x32C9, 0x387A, 0x1AB5, 0x0E8B, 0x2C44,
    0x256C, 0x07A3, 0x139D, 0x3152, 0x3BE1, 0x192E, 0x0D10, 0x2FDF,
    0x1876, 0x3AB9, 0x2E87, 0x0C48, 0x06FB, 0x2434, 0x300A, 0x12C5,
    0x2C37, 0x0EF8, 0x1AC6, 0x3809, 0x32BA, 0x1075, 0x044B, 0x2684,
    0x112D, 0x33E2, 0x27DC, 0x0513, 0x0FA0, 0x2D6F, 0x3951, 0x1B9E,
    0x37DA, 0x1515, 0x012B, 0x23E4, 0x2957, 0x0B98, 0x1FA6, 0x3D69,
    0x0AC0, 0x280F, 0x3C31, 0x1EFE, 0x144D, 0x3682, 0x22BC, 0x0073,
    0x3E81, 0x1C4E, 0x0870, 0x2ABF, 0x200C, 0x02C3, 0x16FD, 0x3432,
    0x039B, 0x2154, 0x356A, 0x17A5, 0x1D16, 0x3FD9, 0x2BE7, 0x0928,
    0x39B7, 0x1B78, 0x0F46, 0x2D89, 0x273A, 0x05F5, 0x11CB, 0x3304,
    0x04AD, 0x2662, 0x325C, 0x1093, 0x1A20, 0x38EF, 0x2CD1, 0x0E1E,
    0x30EC, 0x1223, 0x061D, 0x24D2, 0x2E61, 0x0CAE, 0x1890, 0x3A5F,
    0x0DF6, 0x2F39, 0x3B07, 0x19C8, 0x137B, 0x31B4, 0x258A, 0x0745,
    0x2B01, 0x09CE, 0x1DF0, 0x3F3F, 0x358C, 0x1743, 0x037D, 0x21B2,
    0x161B, 0x34D4, 0x20EA, 0x0225, 0x0896, 0x2A59, 0x3E67, 0x1CA8,
    0x225A, 0x0095, 0x14AB, 0x3664, 0x3CD7, 0x1E18, 0x0A26, 0x28E9,
    0x1F40, 0x3D8F, 0x29B1, 0x0B7E, 0x01CD, 0x2302, 0x373C, 0x15F3,
    0x1CDB, 0x3E14, 0x2A2A, 0x08E5, 0x0256, 0x2099, 0x34A7, 0x1668,
    0x21C1, 0x030E, 0x1730, 0x35FF, 0x3F4C, 0x1D83, 0x09BD, 0x2B72,
    0x1580, 0x374F, 0x2371, 0x01BE, 0x0B0D, 0x29C2, 0x3DFC, 0x1F33,
    0x289A, 0x0A55, 0x1E6B, 0x3CA4, 0x3617, 0x14D8, 0x00E6, 0x2229,
    0x0E6D, 0x2CA2, 0x389C, 0x1A53, 0x10E0, 0x322F, 0x2611, 0x04DE,
    0x3377, 0x11B8, 0x0586, 0x2749, 0x2DFA, 0x0F35, 0x1B0B, 0x39C4,
    0x0736, 0x25F9, 0x31C7, 0x1308, 0x19BB, 0x3B74, 0x2F4A, 0x0D85,
    0x3A2C, 0x18E3, 0x0CDD, 0x2E12, 0x24A1, 0x066E, 0x1250, 0x309F
};

uint16_t calculateFSoECRC(const uint8_t* data, size_t len) {
    uint16_t crc = 0x0000;  // Initial value per ETG.5100
    
    for (size_t i = 0; i < len; i++) {
        uint8_t index = (crc ^ data[i]) & 0xFF;
        crc = (crc >> 8) ^ crcTable[index];
    }
    
    return crc;
}

bool verifyFSoECRC(const uint8_t* data, size_t len, uint16_t expected_crc) {
    return calculateFSoECRC(data, len) == expected_crc;
}

size_t buildFSoEFrame(uint8_t* out, uint8_t cmd,
                       const uint8_t* data, size_t data_len,
                       uint16_t conn_id) {
    size_t chunks = (data_len + 1) / 2;
    size_t frame_size = fsoeFrameSize(data_len);

    out[0] = cmd;

    size_t offset = 1;
    for (size_t i = 0; i < chunks; i++) {
        uint8_t chunk[2] = {0, 0};
        size_t chunk_start = i * 2;
        chunk[0] = (chunk_start < data_len) ? data[chunk_start] : 0;
        chunk[1] = (chunk_start + 1 < data_len) ? data[chunk_start + 1] : 0;

        out[offset] = chunk[0];
        out[offset + 1] = chunk[1];
        uint16_t crc = calculateFSoECRC(chunk, 2);
        out[offset + 2] = crc & 0xFF;
        out[offset + 3] = (crc >> 8) & 0xFF;
        offset += 4;
    }

    out[offset] = conn_id & 0xFF;
    out[offset + 1] = (conn_id >> 8) & 0xFF;

    return frame_size;
}

bool parseFSoEFrame(const uint8_t* frame, size_t frame_len,
                     uint8_t& out_cmd,
                     uint8_t* out_data, size_t& out_data_len,
                     uint16_t& out_conn_id) {
    if (frame_len < MIN_FSOE_FRAME_SIZE) return false;

    out_cmd = frame[0];

    size_t remaining = frame_len - 1 - 2;  // subtract CMD and ConnID
    size_t chunks = remaining / 4;
    out_data_len = chunks * 2;

    size_t offset = 1;
    for (size_t i = 0; i < chunks; i++) {
        uint8_t chunk[2] = {frame[offset], frame[offset + 1]};
        uint16_t stored_crc = static_cast<uint16_t>(frame[offset + 2]) |
                              (static_cast<uint16_t>(frame[offset + 3]) << 8);
        uint16_t calc_crc = calculateFSoECRC(chunk, 2);
        if (stored_crc != calc_crc) {
            return false;
        }
        if (out_data) {
            out_data[i * 2] = chunk[0];
            out_data[i * 2 + 1] = chunk[1];
        }
        offset += 4;
    }

    out_conn_id = static_cast<uint16_t>(frame[offset]) |
                  (static_cast<uint16_t>(frame[offset + 1]) << 8);
    return true;
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

    // Validate watchdog timeout range (ETG.5100 / Object 0x6791: 50-60000 ms)
    if (config_.watchdogTimeoutMs < Limits::WatchdogTimeoutMin ||
        config_.watchdogTimeoutMs > Limits::WatchdogTimeoutMax) {
        return false;
    }

    // Validate safety address range (1-65535, 0 is invalid)
    if (config_.safetyAddress < Limits::SafetyAddressMin) {
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

    if (failSafeActive_) {
        return;  // Already in fail-safe
    }

    lastError_ = errorCode;
    failSafeActive_ = true;
    dataValid_ = false;

    // Stay in Data state but mark as fail-safe (ETG.5100 uses cmd 0x08 within Data)
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
    
    if (!failSafeActive_) {
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
            if (command == Command::Parameter) {
                processParameter(data, len);
            } else if (command == Command::ProcessData) {
                // Skip parameter phase
                transitionTo(ConnectionState::Data);
                processData(data, len);
            }
            break;
            
        case ConnectionState::Data:
            if (command == Command::ProcessData || command == Command::FailSafeData) {
                processData(data, len);
            } else if (command == Command::Reset) {
                processSessionReset(data, len);
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
            if (failSafeActive_) {
                frameSize = buildFailSafeResponse(data, maxLen);
            } else {
                frameSize = buildDataResponse(data, maxLen);
            }
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
    if (len < MIN_FSOE_FRAME_SIZE) {
        handleError(ErrorCode::DataLengthError, false);
        stats_.dataLengthErrors++;
        return false;
    }

    // Parse frame with interleaved CRC verification
    uint8_t cmd = 0;
    size_t data_len = 0;
    uint16_t conn_id = 0;

    if (!CRC::parseFSoEFrame(data, len, cmd, nullptr, data_len, conn_id)) {
        handleError(ErrorCode::CRCError, config_.treatCrcErrorAsCritical);
        stats_.crcErrors++;
        return false;
    }

    // Validate connection ID (after connection is established)
    if (state_ == ConnectionState::Connection ||
        state_ == ConnectionState::Parameter ||
        state_ == ConnectionState::Data) {
        if (!validateConnectionId(conn_id)) {
            return false;
        }
    }

    return true;
}

bool FSoESlave::validateCRC(const uint8_t* data, size_t len) {
    // CRC validation is now done inside parseFSoEFrame in validateFrame.
    // This method is kept for compatibility but delegates to parseFSoEFrame.
    if (errorInjection_.enabled && errorInjection_.injectCRCError) {
        handleError(ErrorCode::CRCError, config_.treatCrcErrorAsCritical);
        stats_.crcErrors++;
        return false;
    }

    uint8_t cmd = 0;
    size_t data_len = 0;
    uint16_t conn_id = 0;
    return CRC::parseFSoEFrame(data, len, cmd, nullptr, data_len, conn_id);
}

bool FSoESlave::validateSequence(uint8_t seqNum) {
    // ETG.5100 does not define a sequence number field.
    // Frame integrity is ensured via CRC + watchdog.
    // This method is kept for API compatibility but is a no-op.
    (void)seqNum;
    return true;
}

bool FSoESlave::validateConnectionId(uint16_t connId) {
    // Check for connection ID error injection
    if (errorInjection_.enabled && errorInjection_.injectConnIdError) {
        connId = errorInjection_.fakeConnId;
    }
    
    if (connId != currentConnectionId_) {
        handleError(ErrorCode::ConnectionIDError, config_.treatConnIdErrorAsCritical);
        stats_.connectionIdErrors++;
        return false;
    }
    
    return true;
}

uint16_t FSoESlave::calculateCRC(const uint8_t* data, size_t len) {
    return CRC::calculateFSoECRC(data, len);
}

// ============================================================================
// Frame Processing
// ============================================================================

void FSoESlave::processSessionReset(const uint8_t* data, size_t len) {
    // Parse frame to extract session ID from safe data
    uint8_t cmd = 0;
    uint8_t frame_data[MAX_SAFE_DATA_SIZE] = {0};
    size_t data_len = 0;
    uint16_t conn_id = 0;

    if (CRC::parseFSoEFrame(data, len, cmd, frame_data, data_len, conn_id)) {
        // Session ID is in the first 2 bytes of safe data
        if (data_len >= 2) {
            sessionId_ = static_cast<uint16_t>(frame_data[0]) |
                         (static_cast<uint16_t>(frame_data[1]) << 8);
        }
    }

    // Reset sequence counters (no longer used but kept for API compat)
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
    // Parse frame to extract connection data
    uint8_t cmd = 0;
    uint8_t frame_data[MAX_SAFE_DATA_SIZE] = {0};
    size_t data_len = 0;
    uint16_t conn_id = 0;

    if (!CRC::parseFSoEFrame(data, len, cmd, frame_data, data_len, conn_id)) {
        handleError(ErrorCode::CRCError, true);
        return;
    }

    // Connection ID from end of frame
    currentConnectionId_ = conn_id;

    // Validate safety address if present in safe data (first 2 bytes)
    if (data_len >= 2) {
        uint16_t safetyAddr = static_cast<uint16_t>(frame_data[0]) |
                              (static_cast<uint16_t>(frame_data[1]) << 8);
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
    // Parse frame to extract parameter data
    uint8_t cmd = 0;
    uint8_t frame_data[MAX_SAFE_DATA_SIZE] = {0};
    size_t data_len = 0;
    uint16_t conn_id = 0;

    if (!CRC::parseFSoEFrame(data, len, cmd, frame_data, data_len, conn_id)) {
        handleError(ErrorCode::CRCError, true);
        return;
    }

    // Validate safety-critical parameters from safe data
    // Expected layout: [watchdog_lo] [watchdog_hi] [safety_level] [input_size] [output_size] [reserved]
    if (data_len >= 5) {
        uint16_t watchdog = static_cast<uint16_t>(frame_data[0]) |
                            (static_cast<uint16_t>(frame_data[1]) << 8);
        uint8_t safety_level = frame_data[2];
        uint8_t input_size = frame_data[3];
        uint8_t output_size = frame_data[4];

        // Validate watchdog range
        if (watchdog < Limits::WatchdogTimeoutMin || watchdog > Limits::WatchdogTimeoutMax) {
            handleError(ErrorCode::ParameterError, true);
            return;
        }

        // Validate safety level
        if (safety_level < config_.safetyLevel) {
            handleError(ErrorCode::ParameterError, true);
            return;
        }

        // Validate data sizes match configuration
        if (input_size != config_.safeInputSize || output_size != config_.safeOutputSize) {
            handleError(ErrorCode::ParameterError, true);
            return;
        }

        // Update watchdog timeout from parameter
        config_.watchdogTimeoutMs = watchdog;
    }

    transitionTo(ConnectionState::Data);

    logDiagnostic(ErrorCode::NoError, "Parameters accepted");
}

void FSoESlave::processData(const uint8_t* data, size_t len) {
    // Parse frame to extract safe output data
    uint8_t cmd = 0;
    uint8_t frame_data[MAX_SAFE_DATA_SIZE] = {0};
    size_t data_len = 0;
    uint16_t conn_id = 0;

    if (!CRC::parseFSoEFrame(data, len, cmd, frame_data, data_len, conn_id)) {
        handleError(ErrorCode::CRCError, true);
        return;
    }

    // Handle fail-safe command within Data state
    if (cmd == Command::FailSafeData) {
        // Master is sending fail-safe data
        // Extract safe output data (first safeOutputSize bytes)
        if (data_len >= config_.safeOutputSize) {
            std::copy(frame_data, frame_data + config_.safeOutputSize,
                      safeOutputs_.begin());
        }
        dataValid_ = false;
        failSafeActive_ = true;
        applyFailSafeOutputs();

        if (failSafeCallback_) {
            failSafeCallback_();
        }
        return;
    }

    // Normal ProcessData command
    if (cmd != Command::ProcessData) {
        return;
    }

    // Extract safe output data
    // Note: fail-safe is only cleared by Reset command, not by ProcessData
    if (data_len >= config_.safeOutputSize) {
        std::copy(frame_data, frame_data + config_.safeOutputSize,
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
    // Session response: CMD + session_id (2B) + ConnID (2B)
    uint8_t payload[2];
    payload[0] = sessionId_ & 0xFF;
    payload[1] = (sessionId_ >> 8) & 0xFF;
    size_t needed = fsoeFrameSize(2);
    if (maxLen < needed) return 0;
    return CRC::buildFSoEFrame(data, Command::Session, payload, 2, config_.connectionId);
}

size_t FSoESlave::buildConnectionResponse(uint8_t* data, size_t maxLen) {
    // Connection response: CMD + safety_addr (2B) + safety_level (1B) + reserved (1B) + ConnID (2B)
    uint8_t payload[4];
    payload[0] = config_.safetyAddress & 0xFF;
    payload[1] = (config_.safetyAddress >> 8) & 0xFF;
    payload[2] = config_.safetyLevel;
    payload[3] = 0;  // Reserved
    size_t needed = fsoeFrameSize(4);
    if (maxLen < needed) return 0;
    return CRC::buildFSoEFrame(data, Command::Connection, payload, 4, currentConnectionId_);
}

size_t FSoESlave::buildParameterResponse(uint8_t* data, size_t maxLen) {
    // Parameter response: CMD + param_ack (2B) + ConnID (2B)
    uint8_t payload[2] = {0, 0};  // Parameter ACK
    size_t needed = fsoeFrameSize(2);
    if (maxLen < needed) return 0;
    return CRC::buildFSoEFrame(data, Command::Parameter, payload, 2, currentConnectionId_);
}

size_t FSoESlave::buildDataResponse(uint8_t* data, size_t maxLen) {
    size_t needed = fsoeFrameSize(config_.safeInputSize);
    if (maxLen < needed) return 0;
    return CRC::buildFSoEFrame(data, Command::ProcessData,
                               safeInputs_.data(), config_.safeInputSize,
                               currentConnectionId_);
}

size_t FSoESlave::buildFailSafeResponse(uint8_t* data, size_t maxLen) {
    // Fail-safe response: CMD + fail_safe_inputs (safeInputSize) + error_code (2B) + ConnID (2B)
    // Use a temporary buffer to combine fail-safe inputs and error code
    uint8_t payload[MAX_SAFE_DATA_SIZE] = {0};
    size_t payload_len = config_.safeInputSize + 2;  // inputs + error code

    std::copy(config_.failSafeInputs.begin(),
              config_.failSafeInputs.begin() + config_.safeInputSize,
              payload);
    payload[config_.safeInputSize] = lastError_ & 0xFF;
    payload[config_.safeInputSize + 1] = (lastError_ >> 8) & 0xFF;

    size_t needed = fsoeFrameSize(payload_len);
    if (maxLen < needed) return 0;
    return CRC::buildFSoEFrame(data, Command::FailSafeData,
                               payload, payload_len, currentConnectionId_);
}

// ============================================================================
// Error and Watchdog Handling
// ============================================================================

void FSoESlave::handleWatchdog(uint64_t currentTimeMs) {
    // Watchdog active in all communication states (not Reset/Error)
    if (state_ == ConnectionState::Reset || state_ == ConnectionState::Error) {
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
            
        case ConnectionState::Data:
            // Check recovery delay for fail-safe sub-mode
            if (failSafeActive_ && config_.autoRecoveryEnabled &&
                elapsed > config_.recoveryDelayMs) {
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
