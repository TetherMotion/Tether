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
        case ConnectionState::FailSafe:   return "FAILSAFE";
        default:                          return "UNKNOWN";
    }
}

bool FSoESlave::isFailSafe() const {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    return failSafeActive_;
}

bool FSoESlave::hasError() const {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    return state_.load() == ConnectionState::Error || lastError_ != ErrorCode::NoError;
}

uint16_t FSoESlave::getLastError() const {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    return lastError_;
}

bool FSoESlave::areSafeOutputsValid() const {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    return dataValid_ && state_.load() == ConnectionState::Data;
}

FSoESlaveStats FSoESlave::getStats() const {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    return stats_;
}

void FSoESlave::resetStats() {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    stats_.reset();
}

std::vector<FSoEDiagnosticEntry> FSoESlave::getDiagnostics() const {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    return diagnostics_;
}

void FSoESlave::clearDiagnostics() {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    diagnostics_.clear();
}

void FSoESlave::setErrorInjection(const FSoEErrorInjection& injection) {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    errorInjection_ = injection;
}

bool FSoESlave::isErrorInjectionEnabled() const {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    return errorInjection_.enabled;
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
    failSafeEnteredMs_ = lastUpdateTimeMs_;

    // ETG.5100 defines fail-safe as a sub-mode of Data (signalled via cmd 0x08).
    // Transition to Data so that handleTimeout's recovery logic is reached
    // regardless of which state triggerFailSafe was called from.
    if (state_ != ConnectionState::Data) {
        transitionTo(ConnectionState::Data);
    }

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
    failSafeEnteredMs_ = 0;
    
    logDiagnostic(ErrorCode::NoError, "Recovery attempt initiated");
    
    return true;
}

void FSoESlave::transitionTo(uint8_t newState) {
    // Invariant: caller must hold mutex_ (recursive). All state-correlated
    // fields (stateEntryTimeMs_, lastError_, failSafeActive_, dataValid_)
    // are written here under that lock so that readers observing state_ via
    // the atomic also see a consistent snapshot of the related fields.
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
            } else if (command == Command::FailSafeData) {
                processData(data, len);
            } else {
                return false;
            }
            break;

        case ConnectionState::Connection:
            if (command == Command::Connection) {
                processConnection(data, len);
            } else if (command == Command::Parameter) {
                processParameter(data, len);
            } else if (command == Command::ProcessData) {
                // Master may skip the Parameter phase when input_size=0
                // and output_size=0. Transition directly to Data.
                transitionTo(ConnectionState::Data);
                processData(data, len);
            } else if (command == Command::FailSafeData) {
                processData(data, len);
            } else {
                return false;
            }
            break;

        case ConnectionState::Parameter:
            if (command == Command::Parameter) {
                processParameter(data, len);
            } else if (command == Command::ProcessData) {
                // Skip parameter phase
                transitionTo(ConnectionState::Data);
                processData(data, len);
            } else if (command == Command::FailSafeData) {
                processData(data, len);
            } else {
                return false;
            }
            break;

        case ConnectionState::Data:
            if (command == Command::ProcessData || command == Command::FailSafeData) {
                processData(data, len);
            } else if (command == Command::Reset) {
                processSessionReset(data, len);
            } else {
                return false;
            }
            break;

        case ConnectionState::Error:
            if (command == Command::Reset) {
                // Reset from error state
                reset();
            } else {
                return false;
            }
            break;

        default:
            return false;
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
    if (len < CRC::MIN_FSOE_FRAME_SIZE) {
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
    return CRC::calculate(data, len);
}

// ============================================================================
// Frame Processing
// ============================================================================

void FSoESlave::processSessionReset(const uint8_t* data, size_t len) {
    // Parse frame to extract session ID from safe data
    uint8_t cmd = 0;
    uint8_t frame_data[CRC::MAX_PARSE_DATA_SIZE] = {0};
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
    uint8_t frame_data[CRC::MAX_PARSE_DATA_SIZE] = {0};
    size_t data_len = 0;
    uint16_t conn_id = 0;

    if (!CRC::parseFSoEFrame(data, len, cmd, frame_data, data_len, conn_id)) {
        handleError(ErrorCode::CRCError, true);
        return;
    }

    // Validate connection ID matches configured value
    if (conn_id != config_.connectionId) {
        handleError(ErrorCode::ConnectionIDError, true);
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

    // Extract parameter CRC from connection frame (bytes 2-3)
    if (data_len >= 4) {
        receivedParameterCRC_ = static_cast<uint16_t>(frame_data[2]) |
                                (static_cast<uint16_t>(frame_data[3]) << 8);
        // Verify parameter CRC if expected value is configured (non-zero)
        if (config_.expectedParameterCRC != 0 &&
            receivedParameterCRC_ != config_.expectedParameterCRC) {
            handleError(ErrorCode::ParameterError, true);
            return;
        }
    }

    transitionTo(ConnectionState::Connection);

    char msg[64];
    snprintf(msg, sizeof(msg), "Connection established: ID=0x%04X", currentConnectionId_);
    logDiagnostic(ErrorCode::NoError, msg);
}

void FSoESlave::processParameter(const uint8_t* data, size_t len) {
    // Parse frame to extract parameter data
    uint8_t cmd = 0;
    uint8_t frame_data[CRC::MAX_PARSE_DATA_SIZE] = {0};
    size_t data_len = 0;
    uint16_t conn_id = 0;

    if (!CRC::parseFSoEFrame(data, len, cmd, frame_data, data_len, conn_id)) {
        handleError(ErrorCode::CRCError, true);
        return;
    }

    // Validate safety-critical parameters from safe data
    // Layout (must match master's buildParameterFrame):
    //   [watchdog_lo] [watchdog_hi] [safety_level] [input_size] [output_size] [reserved]
    // Require the full 6-byte parameter payload (5 data bytes + 1 reserved).
    if (data_len < 6) {
        handleError(ErrorCode::DataLengthError, true);
        return;
    }

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

    // Transition to Parameter state to send Parameter response.
    // Will transition to Data when the master sends ProcessData.
    transitionTo(ConnectionState::Parameter);

    logDiagnostic(ErrorCode::NoError, "Parameters accepted");
}

void FSoESlave::processData(const uint8_t* data, size_t len) {
    // Parse frame to extract safe output data
    uint8_t cmd = 0;
    uint8_t frame_data[CRC::MAX_PARSE_DATA_SIZE] = {0};
    size_t data_len = 0;
    uint16_t conn_id = 0;

    if (!CRC::parseFSoEFrame(data, len, cmd, frame_data, data_len, conn_id)) {
        handleError(ErrorCode::CRCError, true);
        return;
    }

    // Handle fail-safe command within Data state
    if (cmd == Command::FailSafeData) {
        // Master is sending fail-safe data — enter fail-safe via triggerFailSafe
        // to properly set failSafeEnteredMs_, increment stats, and fire callback.
        // triggerFailSafe skips if already in fail-safe, which is correct.
        // Preserve existing error code; use ApplicationError if none set yet.
        triggerFailSafe(lastError_ != ErrorCode::NoError ? lastError_
                                                         : ErrorCode::ApplicationError);
        return;
    }

    // Normal ProcessData command
    if (cmd != Command::ProcessData) {
        return;
    }

    // Reject ProcessData while in fail-safe — fail-safe is only cleared by
    // Reset command. Accepting master data would overwrite fail-safe outputs.
    // Silently ignore to preserve the existing error code in the fail-safe
    // response — the master will see the slave's FailSafeData response and
    // enter fail-safe with the correct error code.
    if (failSafeActive_) {
        return;
    }

    // Validate data length — reject short ProcessData frames
    if (data_len < config_.safeOutputSize) {
        handleError(ErrorCode::DataLengthError, false);
        stats_.dataLengthErrors++;
        return;
    }

    // Extract safe output data
    std::copy(frame_data, frame_data + config_.safeOutputSize,
              safeOutputs_.begin());

    dataValid_ = true;

    if (dataValidCallback_) {
        dataValidCallback_(safeOutputs_.data(), config_.safeOutputSize);
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
    size_t needed = CRC::fsoeFrameSize(2);
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
    size_t needed = CRC::fsoeFrameSize(4);
    if (maxLen < needed) return 0;
    return CRC::buildFSoEFrame(data, Command::Connection, payload, 4, currentConnectionId_);
}

size_t FSoESlave::buildParameterResponse(uint8_t* data, size_t maxLen) {
    // Parameter response: CMD + param_ack (2B) + ConnID (2B)
    uint8_t payload[2] = {0, 0};  // Parameter ACK
    size_t needed = CRC::fsoeFrameSize(2);
    if (maxLen < needed) return 0;
    return CRC::buildFSoEFrame(data, Command::Parameter, payload, 2, currentConnectionId_);
}

size_t FSoESlave::buildDataResponse(uint8_t* data, size_t maxLen) {
    size_t needed = CRC::fsoeFrameSize(config_.safeInputSize);
    if (maxLen < needed) return 0;
    return CRC::buildFSoEFrame(data, Command::ProcessData,
                               safeInputs_.data(), config_.safeInputSize,
                               currentConnectionId_);
}

size_t FSoESlave::buildFailSafeResponse(uint8_t* data, size_t maxLen) {
    // Fail-safe response: CMD + fail_safe_inputs (safeInputSize) + error_code (2B) + ConnID (2B)
    // Use a temporary buffer to combine fail-safe inputs and error code.
    // Buffer must be large enough for safeInputSize + 2 (error code) bytes.
    // MAX_SAFE_DATA_SIZE is 16, so we need MAX_SAFE_DATA_SIZE + 2 for the worst case.
    uint8_t payload[MAX_SAFE_DATA_SIZE + 2] = {0};
    size_t payload_len = config_.safeInputSize + 2;  // inputs + error code

    std::copy(config_.failSafeInputs.begin(),
              config_.failSafeInputs.begin() + config_.safeInputSize,
              payload);
    payload[config_.safeInputSize] = lastError_ & 0xFF;
    payload[config_.safeInputSize + 1] = (lastError_ >> 8) & 0xFF;

    size_t needed = CRC::fsoeFrameSize(payload_len);
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

    // Don't re-fire watchdog once already in fail-safe
    if (failSafeActive_) {
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
            if (failSafeActive_ && config_.autoRecoveryEnabled) {
                uint64_t failSafeElapsed = currentTimeMs - failSafeEnteredMs_;
                if (failSafeElapsed > config_.recoveryDelayMs) {
                    attemptRecovery();
                }
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
