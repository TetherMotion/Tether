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

    // ETG.5100 §8.2.2.4: Connection ID 0x0000 is not permitted.
    if (config_.connectionId == 0) {
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
    sessionOctetIdx_ = 0;
    sessionOctetAdvancePending_ = false;
    connectionRxIdx_ = 0;
    connectionTxIdx_ = 0;
    connectionTxAdvancePending_ = false;
    memset(connectionBuf_, 0, sizeof(connectionBuf_));
    // Reset Parameter state multi-cycle transfer.
    paramRxIdx_ = 0;
    paramTxIdx_ = 0;
    paramTxAdvancePending_ = false;
    paramBuf_.clear();
    currentConnectionId_ = 0;
    expectedSequence_ = 0;
    txSequence_ = 0;
    last_tx_crc0_ = 0;
    last_rx_crc0_ = 0;
    tx_seq_no_ = config_.initialSeqNo;
    rx_seq_no_ = config_.initialSeqNo;
    last_tx_seq_no_ = 0;
    last_rx_seq_no_ = 0;
    statistics_.resetAll();

    // Configure diagnostics
    statistics_.setDiagnosticsEnabled(config_.enableDiagnostics);
    statistics_.setMaxEntries(config_.maxErrorLogEntries);

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
    return statistics_.getStats();
}

void FSoESlave::resetStats() {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    statistics_.resetStats();
}

std::vector<FSoEDiagnosticEntry> FSoESlave::getDiagnostics() const {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    return statistics_.getDiagnostics();
}

void FSoESlave::clearDiagnostics() {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    statistics_.clearDiagnostics();
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
    sessionOctetIdx_ = 0;
    sessionOctetAdvancePending_ = false;
    connectionRxIdx_ = 0;
    connectionTxIdx_ = 0;
    connectionTxAdvancePending_ = false;
    memset(connectionBuf_, 0, sizeof(connectionBuf_));
    // Reset Parameter state multi-cycle transfer.
    paramRxIdx_ = 0;
    paramTxIdx_ = 0;
    paramTxAdvancePending_ = false;
    paramBuf_.clear();
    currentConnectionId_ = 0;
    expectedSequence_ = 0;
    txSequence_ = 0;
    last_tx_crc0_ = 0;
    last_rx_crc0_ = 0;
    tx_seq_no_ = config_.initialSeqNo;
    rx_seq_no_ = config_.initialSeqNo;
    last_tx_seq_no_ = 0;
    last_rx_seq_no_ = 0;
    last_rx_frame_bytes_.clear();  // Clear duplicate detection state
    cached_tx_response_.clear();   // Clear TX response cache
    cached_tx_state_ = 0xFF;
    tx_cache_valid_ = false;

    // Apply fail-safe values
    applyFailSafeOutputs();
    
    statistics_.onSessionReset();
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

    // Clear TX cache — fail-safe changes the response (cmd=FailSafeData)
    cached_tx_response_.clear();
    cached_tx_state_ = 0xFF;
    tx_cache_valid_ = false;

    applyFailSafeOutputs();

    statistics_.onFailSafeActivation();

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
    
    statistics_.onRecoveryAttempt();
    
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

    // Clear TX response cache on state transition — the new state requires
    // a different response (different command, different CRC chain entry).
    cached_tx_response_.clear();
    cached_tx_state_ = 0xFF;
    tx_cache_valid_ = false;

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

    statistics_.onFrameReceived();

    // Check for frame drop injection
    if (errorInjection_.enabled && shouldDropFrame()) {
        statistics_.onInvalidFrame();
        return false;
    }

    // Check for forced fail-safe
    if (errorInjection_.enabled && errorInjection_.forceFailSafe) {
        triggerFailSafe(ErrorCode::ApplicationError);
        return false;
    }

    // Duplicate frame detection for the PDO path.
    //
    // In the PDO path, the master sends the SAME frame bytes every cycle
    // (same CRC, same seq) while in the same state.  This is correct FSoE
    // behavior: the master only rebuilds the frame on state transitions.
    // The slave must detect these duplicates and skip CRC advancement,
    // otherwise the slave's RX CRC would advance on every duplicate,
    // causing CRC divergence with the master.
    //
    // On a duplicate frame:
    // - Update the watchdog timestamp (the master is still alive)
    // - Do NOT advance the RX CRC or sequence number
    // - Do NOT re-process the command (state machine unchanged)
    // - Return true (valid frame, just a duplicate)
    if (!last_rx_frame_bytes_.empty() &&
        last_rx_frame_bytes_.size() == len &&
        std::memcmp(last_rx_frame_bytes_.data(), data, len) == 0) {
        lastValidFrameMs_ = lastUpdateTimeMs_;
        // Mark TX cache as valid — the master resent the same frame, so
        // the slave should resend its cached response (no CRC advancement).
        // This is only used in the PDO path where the master caches and
        // resends frames.
        tx_cache_valid_ = true;
        return true;
    }

    // Validate frame
    if (!validateFrame(data, len)) {
        statistics_.onInvalidFrame();
        return false;
    }

    // Store frame bytes for duplicate detection (only on successful validation)
    last_rx_frame_bytes_.assign(data, data + len);

    // Clear TX response cache — we received a NEW frame (not a duplicate),
    // so the slave should build a fresh response with the advanced CRC chain.
    // (transitionTo also clears the cache, but the state might not change
    // e.g. Session→Session with a new master frame.)
    cached_tx_response_.clear();
    cached_tx_state_ = 0xFF;
    tx_cache_valid_ = false;  // Next prepareTxFrame must build a new response

    statistics_.onValidFrame();
    lastValidFrameMs_ = lastUpdateTimeMs_;
    
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
            } else if (command == Command::Reset) {
                // Master requested reset — go back to Session
                processSessionReset(data, len);
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
            } else if (command == Command::Reset) {
                // Master requested reset — go back to Session
                processSessionReset(data, len);
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

    // Check for delayed response injection
    if (errorInjection_.enabled && errorInjection_.delayResponse) {
        // Simulate delay by not responding
        return 0;
    }

    const uint8_t current_state = state_.load();
    const bool current_fail_safe = failSafeActive_;

    // TX response caching: in the PDO path, the slave should send the SAME
    // response bytes every cycle while in the same state.  This prevents
    // CRC chain divergence when the master resends the same frame (duplicate
    // detection).  The response is rebuilt only when the state transitions
    // or the fail-safe flag changes.
    //
    // In Data state with non-fail-safe, the safe inputs may change between
    // cycles.  However, the slave should still cache its response and only
    // rebuild when it processes a NEW master frame (which is handled by
    // the duplicate detection in processRxFrame clearing the cache).  This
    // ensures the slave's TX CRC only advances when the master's TX CRC
    // advances (i.e. when both sides process a new frame).
    const bool can_cache = true;
    if (can_cache && tx_cache_valid_ && !cached_tx_response_.empty() &&
        cached_tx_state_ == current_state &&
        cached_tx_fail_safe_ == current_fail_safe) {
        // State and fail-safe flag haven't changed — resend cached response.
        // This does NOT advance last_tx_crc0_ or tx_seq_no_.
        const size_t cached_size = cached_tx_response_.size();
        if (maxLen >= cached_size) {
            std::memcpy(data, cached_tx_response_.data(), cached_size);
            statistics_.onFrameSent();
            return cached_size;
        }
        // Buffer too small — fall through to rebuild
        cached_tx_response_.clear();
        cached_tx_state_ = 0xFF;
    }

    size_t frameSize = 0;

    switch (current_state) {
        case ConnectionState::Reset:
            frameSize = buildResetResponse(data, maxLen);
            break;

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
        statistics_.onCrcError();  // Track injected errors
    }

    // Apply data corruption
    if (frameSize > 0 && errorInjection_.enabled && errorInjection_.corruptData) {
        applyDataCorruption(data, frameSize);
    }

    if (frameSize > 0) {
        statistics_.onFrameSent();
        // No seq increment — the slave's TX uses cross-direction CRC
        // inheritance (master's last TX CRC0 and seq).  The seq only
        // advances when the master sends a new frame with a new seq.
        // last_tx_seq_no_ and last_tx_crc0_ are already updated by the
        // build* function above.

        // Cache the response for future cycles (only for cacheable states)
        // Note: tx_cache_valid_ is NOT set here.  It's only set when the
        // slave receives a DUPLICATE frame (in processRxFrame).  This
        // ensures that in the direct exchange path (exchangeWith), where
        // prepareTxFrame may be called multiple times without a duplicate
        // RX, the slave always builds a fresh response with the correct
        // CRC.
        if (can_cache) {
            cached_tx_response_.assign(data, data + frameSize);
            cached_tx_state_ = current_state;
            cached_tx_fail_safe_ = current_fail_safe;
            // tx_cache_valid_ is NOT set here — only set on duplicate RX
        }
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
        statistics_.updateCycleTime(cycleUs);
    }
    
    lastUpdateTimeMs_ = currentTimeMs;
    
    // Handle watchdog
    handleWatchdog(currentTimeMs);
    
    // Handle state timeouts
    handleTimeout(currentTimeMs);
    
    // Check for simulated watchdog timeout
    if (errorInjection_.enabled && errorInjection_.simulateWatchdogTimeout) {
        if (currentTimeMs - lastValidFrameMs_ > errorInjection_.watchdogDelayMs) {
            FSoEErrorDetail detail;
            snprintf(detail.message, sizeof(detail.message),
                     "Slave simulated watchdog timeout after %llu ms (injected)",
                     static_cast<unsigned long long>(
                         currentTimeMs - lastValidFrameMs_));
            handleError(ErrorCode::WatchdogError, config_.treatTimeoutAsCritical, detail);
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
        FSoEErrorDetail detail;
        snprintf(detail.message, sizeof(detail.message),
                 "Slave received frame too short: %zu bytes (minimum %zu)",
                 len, static_cast<size_t>(CRC::MIN_FSOE_FRAME_SIZE));
        handleError(ErrorCode::DataLengthError, false, detail);
        statistics_.onDataLengthError();
        return false;
    }

    // Parse frame with interleaved CRC verification using collision-aware
    // parsing (ETG.5100 §8.1.3.4).  Reset frames (cmd=0x2A) reset the CRC
    // chain AND the sequence number:
    //   - start_crc = 0 (CRC chain reset)
    //   - seq = config_.initialSeqNo (0 for Synapticon, 1 per ETG.5100)
    // Non-Reset frames use cross-direction CRC (the master's last TX CRC0
    // and seq = the slave's last_rx_crc0_ and rx_seq_no_).
    uint8_t cmd = 0;
    size_t data_len = 0;
    uint16_t conn_id = 0;
    CRC::CrcErrorDetail crc_error_detail{};

    const bool is_reset_frame = (data[0] == Command::Reset);
    const uint16_t parse_start_crc = is_reset_frame ? 0 : last_rx_crc0_;
    const uint16_t parse_seq_no = is_reset_frame ? config_.initialSeqNo : rx_seq_no_;
    uint16_t seq_used = 0;

    if (!CRC::parseFSoEFrameWithCollisionAvoidance(
            data, len, cmd, nullptr, data_len, conn_id,
            parse_start_crc, parse_seq_no,
            is_reset_frame ? nullptr : &last_rx_crc0_,
            &seq_used, &crc_error_detail)) {
        FSoEErrorDetail detail;
        if (crc_error_detail.valid) {
            detail.crc_valid = true;
            detail.crc_segment_index = crc_error_detail.segment_index;
            detail.crc_expected = crc_error_detail.expected_crc;
            detail.crc_received = crc_error_detail.received_crc;
            detail.crc_frame_offset = crc_error_detail.frame_offset;
            snprintf(detail.message, sizeof(detail.message),
                     "Slave received wrong CRC from master: segment %d "
                     "expected 0x%04X got 0x%04X (frame offset %zu)",
                     detail.crc_segment_index,
                     detail.crc_expected, detail.crc_received,
                     detail.crc_frame_offset);
        } else {
            snprintf(detail.message, sizeof(detail.message),
                     "Slave received malformed FSoE frame from master "
                     "(unparseable)");
        }
        handleError(ErrorCode::CRCError, config_.treatCrcErrorAsCritical, detail);
        statistics_.onCrcError();
        return false;
    }

    // Validate connection ID (after connection is established).
    // ETG.5100 §8.2.2.2: Reset frames ALWAYS have Conn_Id=0 — the
    // connection has not been established (or is being reset), so there
    // is no Connection ID to check.  Skip validation for Reset frames
    // in any state, so the slave can accept a master-initiated reset
    // from Connection/Parameter/Data states without rejecting it as
    // an InvalidConnID error.
    // See: https://techoverflow.net/2026/08/12/fsoe-session-pdu-master-and-slave-structure/
    if (cmd != Command::Reset &&
        (state_ == ConnectionState::Connection ||
         state_ == ConnectionState::Parameter ||
         state_ == ConnectionState::Data)) {
        if (!validateConnectionId(conn_id)) {
            return false;
        }
    }

    // Save the seq that matched (after collision avoidance).
    // Advance rx_seq_no_ for the next expected master TX (self-inheriting
    // TX on the master side: the master increments its seq with each TX).
    last_rx_seq_no_ = seq_used;
    rx_seq_no_ = CRC::incrementSeqNo(seq_used);

    return true;
}

bool FSoESlave::validateCRC(const uint8_t* data, size_t len) {
    // CRC validation is now done inside parseFSoEFrame in validateFrame.
    // This method is kept for compatibility but delegates to parseFSoEFrame.
    if (errorInjection_.enabled && errorInjection_.injectCRCError) {
        FSoEErrorDetail detail;
        snprintf(detail.message, sizeof(detail.message),
                 "Slave CRC error injected (test mode)");
        handleError(ErrorCode::CRCError, config_.treatCrcErrorAsCritical, detail);
        statistics_.onCrcError();
        return false;
    }

    uint8_t cmd = 0;
    size_t data_len = 0;
    uint16_t conn_id = 0;
    // Cross-direction RX: verify using the master's last TX CRC0 and seq.
    const bool is_reset_frame = (!data || len == 0) ? false : (data[0] == Command::Reset);
    return CRC::parseFSoEFrameWithCollisionAvoidance(
        data, len, cmd, nullptr, data_len, conn_id,
        is_reset_frame ? 0 : last_rx_crc0_,
        is_reset_frame ? config_.initialSeqNo : rx_seq_no_);
}

bool FSoESlave::validateSequence(uint8_t seqNum) {
    // The FSoE sequence number is NOT transmitted in the frame — it is
    // folded into the CRC computation and shared between master and slave.
    // See: https://techoverflow.net/2026/08/09/fsoe-how-does-crc-inheritance-work/
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
        FSoEErrorDetail detail;
        detail.conn_id_valid = true;
        detail.expected_conn_id = currentConnectionId_;
        detail.received_conn_id = connId;
        snprintf(detail.message, sizeof(detail.message),
                 "Slave received wrong ConnectionID from master: "
                 "expected 0x%04X got 0x%04X",
                 detail.expected_conn_id, detail.received_conn_id);
        handleError(ErrorCode::ConnectionIDError, config_.treatConnIdErrorAsCritical, detail);
        statistics_.onConnectionIdError();
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
    // ETG.5100 S (D) V1.2.0, §8.2.2.3:
    // The slave does NOT echo the master's Session ID.  Instead, it
    // generates its own independent random Slave Session ID and sends
    // that back in its Session response.  The master's Session ID is
    // received but not checked (it has no safety relevance).
    // See: https://techoverflow.net/2026/08/12/fsoe-session-pdu-master-and-slave-structure/
    uint8_t cmd = 0;
    uint8_t frame_data[CRC::MAX_PARSE_DATA_SIZE] = {0};
    size_t data_len = 0;
    uint16_t conn_id = 0;

    // Extract the frame to determine the command (CRC already verified).
    // The master's Session ID bytes are intentionally NOT stored — the
    // slave uses its own independently generated Session ID.
    (void)CRC::extractFSoEFrame(data, len, cmd, frame_data, data_len, conn_id);

    // Only reset CRC inheritance state on a Reset command (0x2A).
    // A Session command (0x4E) continues the CRC chain — resetting would
    // desynchronize the slave from the master.
    //
    // For a Reset command:
    //   - Reset the CRC chain (last_tx_crc0_ = 0, last_rx_crc0_ = 0)
    //   - Reset tx_seq_no_ to config_.initialSeqNo (fresh start)
    //   - Do NOT reset rx_seq_no_ — it was already advanced by validateFrame
    //     (which used initialSeqNo for the Reset frame and set rx_seq_no_
    //     to initialSeqNo+1)
    //
    // This handles both cases:
    //   1. Slave in Reset state receiving Reset (normal handshake start):
    //      tx_seq_no_ was already initialSeqNo, no change.
    //   2. Slave in Data/Connection/Parameter state receiving Reset (recovery):
    //      tx_seq_no_ was at some advanced value, needs to be reset to
    //      initialSeqNo so the slave's next frame (Reset response) uses
    //      seq=initialSeqNo, matching the master's expected rx_seq_no_.
    if (cmd == Command::Reset) {
        expectedSequence_ = 0;
        txSequence_ = 0;
        last_tx_crc0_ = 0;
        last_rx_crc0_ = 0;
        last_tx_seq_no_ = 0;
        last_rx_seq_no_ = 0;
        tx_seq_no_ = config_.initialSeqNo;
        // rx_seq_no_ is NOT reset — validateFrame already advanced it
        last_rx_frame_bytes_.clear();  // Clear duplicate detection
        cached_tx_response_.clear();   // Clear TX response cache
        cached_tx_state_ = 0xFF;
        tx_cache_valid_ = false;
        // Generate a fresh Slave Session ID on Reset.
        // ETG.5100 §8.2.2.3: the slave generates a random Session ID once
        // per connection attempt.  Must be non-zero (0 is not a valid ID).
        sessionId_ = static_cast<uint16_t>(rng_() & 0xFFFF);
        if (sessionId_ == 0) sessionId_ = 1;
        sessionOctetIdx_ = 0;
        sessionOctetAdvancePending_ = false;
        // Reset Connection state multi-cycle transfer.
        connectionRxIdx_ = 0;
        connectionTxIdx_ = 0;
        connectionTxAdvancePending_ = false;
        memset(connectionBuf_, 0, sizeof(connectionBuf_));
        // Reset Parameter state multi-cycle transfer.
        paramRxIdx_ = 0;
        paramTxIdx_ = 0;
        paramTxAdvancePending_ = false;
        paramBuf_.clear();
    }

    // Clear error state
    lastError_ = ErrorCode::NoError;
    failSafeActive_ = false;

    // For 1-octet safety data, defer advancing the Session ID octet
    // index until AFTER the response is built.  The first Session command
    // (from Reset state) uses index 0 (low byte); the second Session
    // command (from Session state) sets a pending-advance flag so the
    // NEXT buildSessionResponse call sends the high byte (index 1).
    // ETG.5100 §8.2.2.3.
    // See: https://techoverflow.net/2026/08/12/fsoe-session-pdu-master-and-slave-structure/
    if (cmd == Command::Session && config_.safeInputSize < 2 &&
        state_.load() == ConnectionState::Session) {
        sessionOctetAdvancePending_ = true;
    }

    transitionTo(ConnectionState::Session);

    statistics_.onSessionReset();
    logDiagnostic(ErrorCode::NoError, "Session reset received");
}

void FSoESlave::processConnection(const uint8_t* data, size_t len) {
    // ETG.5100 S (D) V1.2.0, §8.2.2.4, Tables 15-17:
    // The Connection state transfers 4 bytes (Connection ID + Slave Address)
    // in SafeData.  When safety data < 4 octets, multiple cycles are needed.
    // The slave accumulates received bytes and echoes them back.
    // See: https://techoverflow.net/2026/08/12/fsoe-connection-pdu-master-and-slave-structure/
    uint8_t cmd = 0;
    uint8_t frame_data[CRC::MAX_PARSE_DATA_SIZE] = {0};
    size_t data_len = 0;
    uint16_t conn_id = 0;

    if (!CRC::extractFSoEFrame(data, len, cmd, frame_data, data_len, conn_id)) {
        FSoEErrorDetail detail;
        snprintf(detail.message, sizeof(detail.message),
                 "Slave failed to extract Connection frame from master");
        handleError(ErrorCode::CRCError, true, detail);
        return;
    }

    // Validate Conn_Id field (always strict — the Connection ID is now active).
    // ETG.5100 §8.2.2.4: Connection ID 0x0000 is not permitted.
    if (conn_id != config_.connectionId) {
        FSoEErrorDetail detail;
        detail.conn_id_valid = true;
        detail.expected_conn_id = config_.connectionId;
        detail.received_conn_id = conn_id;
        snprintf(detail.message, sizeof(detail.message),
                 "Slave received wrong ConnectionID in Connection phase: "
                 "expected 0x%04X got 0x%04X",
                 detail.expected_conn_id, detail.received_conn_id);
        handleError(ErrorCode::ConnectionIDError, true, detail);
        return;
    }

    currentConnectionId_ = conn_id;

    // 0-octet safety data: no SafeData to accumulate.  Just transition
    // to Connection state (the Conn_Id field carries the Connection ID).
    if (config_.safeOutputSize == 0) {
        transitionTo(ConnectionState::Connection);
        char msg[64];
        snprintf(msg, sizeof(msg), "Connection established: ID=0x%04X", currentConnectionId_);
        logDiagnostic(ErrorCode::NoError, msg);
        return;
    }

    // Accumulate received bytes into connectionBuf_ at connectionRxIdx_.
    // When safeOutputSize >= 4, the master sends all 4 bytes in one cycle.
    // When safeOutputSize < 4, the master sends chunks and we accumulate.
    const uint8_t rx_chunk = std::min(static_cast<uint8_t>(4), config_.safeOutputSize);
    // Reject frames with insufficient data length.
    if (data_len < rx_chunk) {
        FSoEErrorDetail detail;
        snprintf(detail.message, sizeof(detail.message),
                 "Slave Connection frame too short: got %zu bytes, expected %u",
                 data_len, rx_chunk);
        handleError(ErrorCode::DataLengthError, true, detail);
        return;
    }
    if (config_.safeOutputSize >= 4) {
        // Master sends all 4 bytes in one cycle — copy from offset 0.
        // Only copy if we haven't received all 4 bytes yet (the master
        // may send zero-padded frames after all bytes are transferred).
        if (connectionRxIdx_ < 4) {
            for (uint8_t i = 0; i < 4 && i < data_len; ++i) {
                connectionBuf_[i] = frame_data[i];
            }
            connectionRxIdx_ = 4;
        }
    } else {
        // Master sends safeOutputSize bytes per cycle — accumulate.
        if (connectionRxIdx_ < 4) {
            const uint8_t rx_off = std::min(connectionRxIdx_,
                                             static_cast<uint8_t>(4));
            const uint8_t valid = std::min(static_cast<uint8_t>(4 - rx_off),
                                            rx_chunk);
            for (uint8_t i = 0; i < valid && i < data_len; ++i) {
                connectionBuf_[rx_off + i] = frame_data[i];
            }
            connectionRxIdx_ = std::min(
                static_cast<uint8_t>(connectionRxIdx_ + rx_chunk),
                static_cast<uint8_t>(4));
        }
    }

    // Set pending TX advance flag (applied in buildConnectionResponse
    // after the echo is built, to handle PDO caching correctly).
    connectionTxAdvancePending_ = true;

    // Validate accumulated data once all 4 bytes are received.
    if (connectionRxIdx_ >= 4) {
        uint16_t frame_conn_id = static_cast<uint16_t>(connectionBuf_[0]) |
                                 (static_cast<uint16_t>(connectionBuf_[1]) << 8);
        uint16_t safetyAddr = static_cast<uint16_t>(connectionBuf_[2]) |
                              (static_cast<uint16_t>(connectionBuf_[3]) << 8);

        // ETG.5100 §8.2.2.4: Connection ID 0x0000 is not permitted.
        if (frame_conn_id != config_.connectionId) {
            FSoEErrorDetail detail;
            detail.conn_id_valid = true;
            detail.expected_conn_id = config_.connectionId;
            detail.received_conn_id = frame_conn_id;
            snprintf(detail.message, sizeof(detail.message),
                     "Slave Connection ID mismatch in SafeData: "
                     "expected 0x%04X got 0x%04X",
                     detail.expected_conn_id, detail.received_conn_id);
            handleError(ErrorCode::ConnectionIDError, true, detail);
            return;
        }
        if (safetyAddr != config_.safetyAddress) {
            FSoEErrorDetail detail;
            snprintf(detail.message, sizeof(detail.message),
                     "Slave safety address mismatch in Connection phase: "
                     "expected 0x%04X got 0x%04X",
                     config_.safetyAddress, safetyAddr);
            handleError(ErrorCode::ConnectionIDError, true, detail);
            return;
        }
    }

    transitionTo(ConnectionState::Connection);

    char msg[64];
    snprintf(msg, sizeof(msg), "Connection transfer: rx_idx=%u/4 ID=0x%04X",
             connectionRxIdx_, currentConnectionId_);
    logDiagnostic(ErrorCode::NoError, msg);
}

void FSoESlave::processParameter(const uint8_t* data, size_t len) {
    // ETG.5100 S (D) V1.2.0, §8.2.2.5, Table 18:
    //   octets 0-1: comm param length (always 2, LE)
    //   octets 2-3: FSoE watchdog (ms, LE)
    //   octets 4-5: app param length (LE)
    //   octets 6+:  app param bytes
    // The slave accumulates received bytes and echoes them back.
    // Multi-cycle: ceil((6 + appParamLen) / safeOutputSize) cycles.
    // See: https://techoverflow.net/2026/08/12/fsoe-parameter-pdu-master-and-slave-structure/
    uint8_t cmd = 0;
    uint8_t frame_data[CRC::MAX_PARSE_DATA_SIZE] = {0};
    size_t data_len = 0;
    uint16_t conn_id = 0;

    if (!CRC::extractFSoEFrame(data, len, cmd, frame_data, data_len, conn_id)) {
        FSoEErrorDetail detail;
        snprintf(detail.message, sizeof(detail.message),
                 "Slave failed to extract Parameter frame from master");
        handleError(ErrorCode::CRCError, true, detail);
        return;
    }

    // 0-octet safety data: no SafeData to accumulate.  Just transition
    // to Parameter state (no parameter data can be transferred).
    if (config_.safeOutputSize == 0) {
        transitionTo(ConnectionState::Parameter);
        logDiagnostic(ErrorCode::NoError, "Parameter state (0-octet data)");
        return;
    }

    // Reject frames shorter than the PDO's safe-data size.
    if (data_len > 0 && data_len < config_.safeOutputSize) {
        FSoEErrorDetail detail;
        snprintf(detail.message, sizeof(detail.message),
                 "Slave Parameter frame too short: got %zu bytes, expected %u",
                 data_len, config_.safeOutputSize);
        handleError(ErrorCode::DataLengthError, true, detail);
        return;
    }

    // Ensure paramBuf_ is large enough.  The total payload size is
    // 6 + appParamLen, but we don't know appParamLen until we receive
    // octets 4-5.  We use a dynamic buffer that grows as needed.
    const uint8_t rx_chunk = std::min(static_cast<uint16_t>(config_.safeOutputSize),
                                       static_cast<uint16_t>(CRC::MAX_PARSE_DATA_SIZE));

    // Determine the effective total payload length if we already know it.
    // Once we have the 6-byte header, we can compute total_len.
    size_t known_total = 0;
    if (paramBuf_.size() >= 6) {
        uint16_t app_param_len = static_cast<uint16_t>(paramBuf_[4]) |
                                 (static_cast<uint16_t>(paramBuf_[5]) << 8);
        known_total = static_cast<size_t>(6) + app_param_len;
    }

    // Accumulate received bytes into paramBuf_ at paramRxIdx_.
    // Stop accumulating once we have all bytes (known_total > 0 and
    // paramRxIdx_ >= known_total) to avoid overwriting with padding zeros.
    bool can_accumulate = (known_total == 0) ||
                          (paramRxIdx_ < known_total);
    if (can_accumulate && paramRxIdx_ < PARAM_BUF_SIZE) {
        const size_t rx_off = std::min(static_cast<size_t>(paramRxIdx_),
                                        static_cast<size_t>(PARAM_BUF_SIZE));
        size_t valid = std::min(static_cast<size_t>(PARAM_BUF_SIZE - rx_off),
                                 static_cast<size_t>(rx_chunk));
        // If we know the total, don't accumulate beyond it.
        if (known_total > 0 && rx_off + valid > known_total) {
            valid = known_total - rx_off;
        }
        // Ensure buffer is large enough.
        if (paramBuf_.size() < rx_off + valid) {
            paramBuf_.resize(rx_off + valid, 0);
        }
        for (size_t i = 0; i < valid && i < data_len; ++i) {
            paramBuf_[rx_off + i] = frame_data[i];
        }
        size_t advance = valid;
        if (known_total == 0) {
            // Don't know total yet — advance by rx_chunk.
            advance = rx_chunk;
        }
        paramRxIdx_ = std::min(static_cast<size_t>(paramRxIdx_) + advance,
                                static_cast<size_t>(PARAM_BUF_SIZE));
    }

    // Set pending TX advance flag (applied in buildParameterResponse
    // after the echo is built).
    paramTxAdvancePending_ = true;

    // Once we have received the 6-byte header (octets 0-5), we can
    // validate the comm param length and extract the app param length
    // to know the total payload size.
    if (paramRxIdx_ >= 6 && paramBuf_.size() >= 6) {
        uint16_t comm_param_len = static_cast<uint16_t>(paramBuf_[0]) |
                                  (static_cast<uint16_t>(paramBuf_[1]) << 8);
        // ETG.5100 §8.2.2.5: comm param length is always 2.
        if (comm_param_len != 2) {
            FSoEErrorDetail detail;
            snprintf(detail.message, sizeof(detail.message),
                     "Slave comm param length mismatch: expected 2 got %u",
                     comm_param_len);
            handleError(ErrorCode::ParameterError, true, detail);
            return;
        }

        // Extract watchdog (octets 2-3) and validate range.
        uint16_t watchdog = static_cast<uint16_t>(paramBuf_[2]) |
                            (static_cast<uint16_t>(paramBuf_[3]) << 8);
        if (watchdog < Limits::WatchdogTimeoutMin ||
            watchdog > Limits::WatchdogTimeoutMax) {
            FSoEErrorDetail detail;
            snprintf(detail.message, sizeof(detail.message),
                     "Slave watchdog %u out of range [%u, %u]",
                     watchdog, Limits::WatchdogTimeoutMin,
                     Limits::WatchdogTimeoutMax);
            handleError(ErrorCode::ParameterError, true, detail);
            return;
        }
        // Update the slave's watchdog from the master's configured value.
        config_.watchdogTimeoutMs = watchdog;

        // Extract app param length (octets 4-5).
        uint16_t app_param_len = static_cast<uint16_t>(paramBuf_[4]) |
                                 (static_cast<uint16_t>(paramBuf_[5]) << 8);

        // Early validation: if expected app parameters are configured,
        // check the app param length matches immediately.
        if (app_param_len != config_.expectedAppParameters.size()) {
            FSoEErrorDetail detail;
            snprintf(detail.message, sizeof(detail.message),
                     "Slave app param length mismatch: expected %zu got %u",
                     config_.expectedAppParameters.size(), app_param_len);
            handleError(ErrorCode::ParameterError, true, detail);
            return;
        }

        // Compute total expected payload size.
        size_t total_len = static_cast<size_t>(6) + app_param_len;

        // Ensure paramBuf_ is sized to the full payload.
        if (paramBuf_.size() < total_len) {
            paramBuf_.resize(total_len, 0);
        }

        // Once all bytes are received, validate the app parameters.
        if (paramRxIdx_ >= total_len) {
            // Validate app parameters against expected values.
            if (app_param_len != config_.expectedAppParameters.size()) {
                FSoEErrorDetail detail;
                snprintf(detail.message, sizeof(detail.message),
                         "Slave app param length mismatch: expected %zu got %u",
                         config_.expectedAppParameters.size(), app_param_len);
                handleError(ErrorCode::ParameterError, true, detail);
                return;
            }
            for (size_t i = 0; i < app_param_len; ++i) {
                if (paramBuf_[6 + i] != config_.expectedAppParameters[i]) {
                    FSoEErrorDetail detail;
                    snprintf(detail.message, sizeof(detail.message),
                             "Slave app param mismatch at byte %zu: "
                             "expected 0x%02X got 0x%02X",
                             i, config_.expectedAppParameters[i],
                             paramBuf_[6 + i]);
                    handleError(ErrorCode::ParameterError, true, detail);
                    return;
                }
            }
            logDiagnostic(ErrorCode::NoError, "Parameters accepted");
        }
    }

    // Transition to Parameter state to send Parameter response.
    // Will transition to Data when the master sends ProcessData.
    transitionTo(ConnectionState::Parameter);
}

void FSoESlave::processData(const uint8_t* data, size_t len) {
    // Parse frame to extract safe output data
    uint8_t cmd = 0;
    uint8_t frame_data[CRC::MAX_PARSE_DATA_SIZE] = {0};
    size_t data_len = 0;
    uint16_t conn_id = 0;

    if (!CRC::extractFSoEFrame(data, len, cmd, frame_data, data_len, conn_id)) {
        FSoEErrorDetail detail;
        snprintf(detail.message, sizeof(detail.message),
                 "Slave failed to extract Data frame from master");
        handleError(ErrorCode::CRCError, true, detail);
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
        FSoEErrorDetail detail;
        snprintf(detail.message, sizeof(detail.message),
                 "Slave ProcessData frame too short: got %zu bytes, expected %u",
                 data_len, config_.safeOutputSize);
        handleError(ErrorCode::DataLengthError, false, detail);
        statistics_.onDataLengthError();
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

size_t FSoESlave::buildResetResponse(uint8_t* data, size_t maxLen) {
    // ETG.5100 S (D) V1.2.0, §8.2.2.2:
    // Reset response: full PDO-size frame with all-zero safety data.
    // SafeData[0] = 0 (no error code — acknowledgement).
    // Conn_Id is unused and set to 0 (the connection has not been
    // established yet — there is no Connection ID to check).
    // See: https://techoverflow.net/2026/08/12/fsoe-session-pdu-master-and-slave-structure/
    //
    // The slave's Reset response uses seq = initialSeqNo + 1 (the slave
    // increments the seq after receiving the master's Reset which used
    // seq = initialSeqNo).  The CRC chain is reset (start_crc = 0).
    // After the Reset response, the next frame uses seq+1 again.
    // last_tx_crc0_ is NOT updated (stays at 0 for the next frame).
    uint8_t payload[CRC::MAX_PARSE_DATA_SIZE] = {0};
    size_t needed = CRC::fsoeFrameSize(config_.safeInputSize);
    if (maxLen < needed) return 0;
    const uint16_t reset_resp_seq = CRC::incrementSeqNo(config_.initialSeqNo);
    uint16_t seq_used = 0;
    size_t result = CRC::buildFSoEFrameWithCollisionAvoidance(
        data, Command::Reset, payload, config_.safeInputSize,
        0,  // Conn_Id = 0 in Reset state (ETG.5100 §8.2.2.2)
        0,  // start_crc = 0 (Reset resets CRC chain)
        reset_resp_seq,
        nullptr,  // don't update CRC chain (Reset resets it)
        &seq_used);
    // Set tx_seq_no_ to the seq used.  prepareTxFrame will increment it
    // for the next frame.
    tx_seq_no_ = seq_used;
    last_tx_seq_no_ = seq_used;
    return result;
}

size_t FSoESlave::buildSessionResponse(uint8_t* data, size_t maxLen) {
    // ETG.5100 S (D) V1.2.0, §8.2.2.3, Table 14:
    // Session response carries the slave's OWN random Session ID in
    // SafeData[0..1].  All other SafeData octets are 0.  Conn_Id is
    // unused and set to 0 (the connection has not been established yet).
    // See: https://techoverflow.net/2026/08/12/fsoe-session-pdu-master-and-slave-structure/
    uint8_t payload[CRC::MAX_PARSE_DATA_SIZE] = {0};
    if (config_.safeInputSize >= 2) {
        // Safety data length >= 2: both Session ID octets fit in one PDU.
        payload[0] = sessionId_ & 0xFF;
        payload[1] = (sessionId_ >> 8) & 0xFF;
    } else {
        // Safety data length == 1: transfer Session ID in two successive
        // PDUs (low byte first, then high byte).  ETG.5100 §8.2.2.3.
        payload[0] = (sessionOctetIdx_ == 0)
            ? (sessionId_ & 0xFF)
            : ((sessionId_ >> 8) & 0xFF);
    }
    size_t needed = CRC::fsoeFrameSize(config_.safeInputSize);
    if (maxLen < needed) return 0;
    uint16_t seq_used = 0;
    size_t result = CRC::buildFSoEFrameWithCollisionAvoidance(
        data, Command::Session, payload, config_.safeInputSize,
        0,  // Conn_Id = 0 in Session state (ETG.5100 §8.2.2.3)
        last_rx_crc0_, rx_seq_no_, &last_tx_crc0_, &seq_used);
    tx_seq_no_ = seq_used;
    last_tx_seq_no_ = seq_used;

    // NOTE: sessionOctetIdx_ is advanced via a deferred flag set in
    // processSessionReset().  This ensures the slave sends the correct
    // byte for the CURRENT Session command, then advances for the NEXT
    // one.  The flag is cleared after the first build so subsequent
    // cached rebuilds don't advance the index again.

    if (sessionOctetAdvancePending_) {
        sessionOctetIdx_ = (sessionOctetIdx_ + 1) & 1;
        sessionOctetAdvancePending_ = false;
    }

    return result;
}

size_t FSoESlave::buildConnectionResponse(uint8_t* data, size_t maxLen) {
    // ETG.5100 S (D) V1.2.0, §8.2.2.4, Table 17:
    // The slave echoes back the Connection ID and FSoE Slave Address.
    // When safety data < 4 octets, the echo is sent in safeInputSize-sized
    // chunks over multiple cycles, matching the received data.
    // See: https://techoverflow.net/2026/08/12/fsoe-connection-pdu-master-and-slave-structure/
    uint8_t payload[CRC::MAX_PARSE_DATA_SIZE] = {0};
    const uint8_t chunk = std::min(static_cast<uint8_t>(4), config_.safeInputSize);
    // When safeInputSize >= 4, the slave echoes from offset 0 each cycle
    // (all received bytes).  When < 4, it echoes from connectionTxIdx_.
    const uint8_t tx_off = (config_.safeInputSize >= 4)
        ? 0
        : std::min(connectionTxIdx_, static_cast<uint8_t>(4));
    // Valid bytes: can only echo bytes that have been received.
    const uint8_t rx_avail = (connectionRxIdx_ > tx_off)
        ? static_cast<uint8_t>(connectionRxIdx_ - tx_off) : 0;
    const uint8_t valid = std::min(static_cast<uint8_t>(4 - tx_off),
                                    std::min(chunk, rx_avail));
    for (uint8_t i = 0; i < valid; ++i) {
        payload[i] = connectionBuf_[tx_off + i];
    }
    // Remaining payload bytes are 0 (padding).
    size_t needed = CRC::fsoeFrameSize(config_.safeInputSize);
    if (maxLen < needed) return 0;
    uint16_t seq_used = 0;
    size_t result = CRC::buildFSoEFrameWithCollisionAvoidance(
        data, Command::Connection, payload, config_.safeInputSize,
        currentConnectionId_,
        last_rx_crc0_, rx_seq_no_, &last_tx_crc0_, &seq_used);
    tx_seq_no_ = seq_used;
    last_tx_seq_no_ = seq_used;

    // Apply deferred TX advance (set in processConnection).
    // TX advance is limited by RX progress: can't echo more bytes than
    // have been received.
    if (connectionTxAdvancePending_) {
        uint8_t max_advance = (connectionTxIdx_ < connectionRxIdx_)
            ? static_cast<uint8_t>(connectionRxIdx_ - connectionTxIdx_) : 0;
        uint8_t advance = std::min(chunk, max_advance);
        connectionTxIdx_ = std::min(
            static_cast<uint8_t>(connectionTxIdx_ + advance),
            static_cast<uint8_t>(4));
        connectionTxAdvancePending_ = false;
    }

    return result;
}

size_t FSoESlave::buildParameterResponse(uint8_t* data, size_t maxLen) {
    // ETG.5100 S (D) V1.2.0, §8.2.2.5, Tables 20/22:
    // The slave echoes back the same safety data it received.
    // Multi-cycle: echo is sent in safeInputSize-sized chunks.
    // See: https://techoverflow.net/2026/08/12/fsoe-parameter-pdu-master-and-slave-structure/
    uint8_t payload[CRC::MAX_PARSE_DATA_SIZE] = {0};
    const uint8_t chunk = std::min(static_cast<uint16_t>(config_.safeInputSize),
                                    static_cast<uint16_t>(CRC::MAX_PARSE_DATA_SIZE));
    // TX offset: clamped at paramBuf_.size().  Can only echo bytes
    // that have been received (paramTxIdx_ < paramRxIdx_).
    const size_t tx_off = std::min(static_cast<size_t>(paramTxIdx_),
                                    paramBuf_.size());
    const size_t rx_avail = (paramRxIdx_ > paramTxIdx_)
        ? static_cast<size_t>(paramRxIdx_ - paramTxIdx_) : 0;
    const size_t valid = std::min(static_cast<size_t>(chunk),
                                   std::min(paramBuf_.size() - std::min(tx_off, paramBuf_.size()),
                                            rx_avail));
    for (size_t i = 0; i < valid; ++i) {
        payload[i] = paramBuf_[tx_off + i];
    }
    // Remaining payload bytes are 0 (padding).
    size_t needed = CRC::fsoeFrameSize(config_.safeInputSize);
    if (maxLen < needed) return 0;
    uint16_t seq_used = 0;
    size_t result = CRC::buildFSoEFrameWithCollisionAvoidance(
        data, Command::Parameter, payload, config_.safeInputSize,
        currentConnectionId_,
        last_rx_crc0_, rx_seq_no_, &last_tx_crc0_, &seq_used);
    tx_seq_no_ = seq_used;
    last_tx_seq_no_ = seq_used;

    // Apply deferred TX advance (set in processParameter).
    // TX advance is limited by RX progress: can't echo more bytes than
    // have been received.
    if (paramTxAdvancePending_) {
        size_t max_advance = (paramTxIdx_ < paramRxIdx_)
            ? static_cast<size_t>(paramRxIdx_ - paramTxIdx_) : 0;
        size_t advance = std::min(static_cast<size_t>(chunk), max_advance);
        paramTxIdx_ = std::min(static_cast<size_t>(paramTxIdx_) + advance,
                                static_cast<size_t>(PARAM_BUF_SIZE));
        paramTxAdvancePending_ = false;
    }

    return result;
}

size_t FSoESlave::buildDataResponse(uint8_t* data, size_t maxLen) {
    size_t needed = CRC::fsoeFrameSize(config_.safeInputSize);
    if (maxLen < needed) return 0;
    uint16_t seq_used = 0;
    size_t result = CRC::buildFSoEFrameWithCollisionAvoidance(
        data, Command::ProcessData,
        safeInputs_.data(), config_.safeInputSize,
        currentConnectionId_,
        last_rx_crc0_, rx_seq_no_, &last_tx_crc0_, &seq_used);
    tx_seq_no_ = seq_used;
    last_tx_seq_no_ = seq_used;
    return result;
}

size_t FSoESlave::buildFailSafeResponse(uint8_t* data, size_t maxLen) {
    // Fail-safe response: CMD + fail_safe_inputs (safeInputSize) + error_code (2B) + ConnID (2B)
    // Use a temporary buffer to combine fail-safe inputs and error code.
    // Buffer must be large enough for safeInputSize + 2 (error code) bytes.
    // MAX_SAFE_DATA_SIZE is 16, so we need MAX_SAFE_DATA_SIZE + 2 for the worst case.
    uint8_t payload[MAX_SAFE_DATA_SIZE + 2] = {0};
    // Payload must be at least the fixed data length, but also accommodate
    // the fail-safe inputs + error code.
    const size_t min_payload = config_.safeInputSize;
    const size_t fs_payload = static_cast<size_t>(config_.safeInputSize) + 2;
    const size_t payload_len = fs_payload > min_payload ? fs_payload : min_payload;

    std::copy(config_.failSafeInputs.begin(),
              config_.failSafeInputs.begin() + config_.safeInputSize,
              payload);
    payload[config_.safeInputSize] = lastError_ & 0xFF;
    payload[config_.safeInputSize + 1] = (lastError_ >> 8) & 0xFF;

    size_t needed = CRC::fsoeFrameSize(payload_len);
    if (maxLen < needed) return 0;
    uint16_t seq_used = 0;
    size_t result = CRC::buildFSoEFrameWithCollisionAvoidance(
        data, Command::FailSafeData,
        payload, payload_len, currentConnectionId_,
        last_rx_crc0_, rx_seq_no_, &last_tx_crc0_, &seq_used);
    tx_seq_no_ = seq_used;
    last_tx_seq_no_ = seq_used;
    return result;
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
    statistics_.updateGap(elapsed);

    // Check watchdog timeout
    if (elapsed > config_.watchdogTimeoutMs) {
        FSoEErrorDetail detail;
        snprintf(detail.message, sizeof(detail.message),
                 "Slave watchdog timeout after %llu ms (limit %u ms)",
                 static_cast<unsigned long long>(elapsed),
                 config_.watchdogTimeoutMs);
        handleError(ErrorCode::WatchdogError, config_.treatTimeoutAsCritical, detail);
        statistics_.onWatchdogTimeout();
    }
}

void FSoESlave::handleTimeout(uint64_t currentTimeMs) {
    uint64_t elapsed = currentTimeMs - stateEntryTimeMs_;
    
    switch (state_.load()) {
        case ConnectionState::Session:
            if (elapsed > config_.sessionTimeoutMs) {
                FSoEErrorDetail detail;
                snprintf(detail.message, sizeof(detail.message),
                         "Slave session timeout after %llu ms (limit %u ms)",
                         static_cast<unsigned long long>(elapsed),
                         config_.sessionTimeoutMs);
                handleError(ErrorCode::SessionError, false, detail);
            }
            break;

        case ConnectionState::Connection:
        case ConnectionState::Parameter:
            if (elapsed > config_.connectionTimeoutMs) {
                FSoEErrorDetail detail;
                snprintf(detail.message, sizeof(detail.message),
                         "Slave phase timeout in state %u after %llu ms (limit %u ms)",
                         state_.load(),
                         static_cast<unsigned long long>(elapsed),
                         config_.connectionTimeoutMs);
                handleError(ErrorCode::TimeoutError, false, detail);
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

void FSoESlave::handleError(uint16_t errorCode, bool isCritical,
                             const FSoEErrorDetail& detail) {
    lastError_ = errorCode;

    char msg[64];
    snprintf(msg, sizeof(msg), "Error 0x%04X (critical=%d)", errorCode, isCritical);
    logDiagnostic(errorCode, msg);

    if (errorCallback_) {
        errorCallback_(errorCode, isCritical, detail);
    }

    if (isCritical) {
        triggerFailSafe(errorCode);
    }
}

void FSoESlave::logDiagnostic(uint16_t errorCode, const char* message) {
    // Update context for diagnostic entries
    statistics_.setCurrentTimestamp(lastUpdateTimeMs_);
    statistics_.setCurrentState(state_.load());
    statistics_.setCurrentSequence(expectedSequence_);
    statistics_.setCurrentConnectionId(currentConnectionId_);
    statistics_.logDiagnostic(errorCode, message);
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
