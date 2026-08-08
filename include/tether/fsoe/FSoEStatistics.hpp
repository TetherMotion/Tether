// SPDX-License-Identifier: MIT
#pragma once

/**
 * @file FSoEStatistics.hpp
 * @brief Statistics and diagnostics tracking extracted from FSoESlave
 *
 * @details
 * Encapsulates the statistics and diagnostics sub-responsibility of FSoESlave:
 *  - Frame/error/state counters (FSoESlaveStats)
 *  - Diagnostic log entries with configurable max size
 *  - Cycle time tracking (min/max/avg)
 *  - Gap timing measurement
 */

#include "fsoe/FSoEDefs.hpp"

#include <cstdint>
#include <cstring>
#include <vector>

namespace FSoE {

// ============================================================================
// FSoE Diagnostic Entry (moved from FSoESlave.hpp to break circular dependency)
// ============================================================================

struct FSoEDiagnosticEntry {
    uint64_t timestamp;
    uint16_t errorCode;
    uint8_t  state;
    uint8_t  sequenceNumber;
    uint16_t connectionId;
    char message[64];
};

// ============================================================================
// FSoE Slave Statistics (moved from FSoESlave.hpp to break circular dependency)
// ============================================================================

struct FSoESlaveStats {
    // Frame counters
    uint64_t framesReceived = 0;
    uint64_t framesSent = 0;
    uint64_t validFrames = 0;
    uint64_t invalidFrames = 0;

    // Error counters
    uint32_t crcErrors = 0;
    uint32_t sequenceErrors = 0;
    uint32_t connectionIdErrors = 0;
    uint32_t watchdogTimeouts = 0;
    uint32_t commandErrors = 0;
    uint32_t dataLengthErrors = 0;

    // State counters
    uint32_t sessionResets = 0;
    uint32_t failSafeActivations = 0;
    uint32_t recoveryAttempts = 0;
    uint32_t successfulRecoveries = 0;

    // Timing
    uint64_t lastValidFrameTime = 0;
    uint64_t longestGapMs = 0;
    uint32_t avgCycleTimeUs = 0;
    uint32_t maxCycleTimeUs = 0;
    uint32_t minCycleTimeUs = UINT32_MAX;

    void reset() {
        framesReceived = 0;
        framesSent = 0;
        validFrames = 0;
        invalidFrames = 0;
        crcErrors = 0;
        sequenceErrors = 0;
        connectionIdErrors = 0;
        watchdogTimeouts = 0;
        commandErrors = 0;
        dataLengthErrors = 0;
        sessionResets = 0;
        failSafeActivations = 0;
        recoveryAttempts = 0;
        successfulRecoveries = 0;
        lastValidFrameTime = 0;
        longestGapMs = 0;
        avgCycleTimeUs = 0;
        maxCycleTimeUs = 0;
        minCycleTimeUs = UINT32_MAX;
    }
};

class FSoEStatistics {
public:
    FSoEStatistics() = default;

    /// @brief Enable/disable diagnostic logging.
    void setDiagnosticsEnabled(bool enabled) { enableDiagnostics_ = enabled; }

    /// @brief Set max number of diagnostic entries before oldest are evicted.
    void setMaxEntries(uint32_t max) { maxEntries_ = max; }

    /// @brief Set current timestamp for diagnostic entries.
    void setCurrentTimestamp(uint64_t ts) { currentTimestamp_ = ts; }

    /// @brief Set current state for diagnostic entries.
    void setCurrentState(uint8_t state) { currentState_ = state; }

    /// @brief Set current sequence number for diagnostic entries.
    void setCurrentSequence(uint8_t seq) { currentSequence_ = seq; }

    /// @brief Set current connection ID for diagnostic entries.
    void setCurrentConnectionId(uint16_t connId) { currentConnectionId_ = connId; }

    // ---- Counter increments ------------------------------------------------

    void onFrameReceived() { stats_.framesReceived++; }
    void onFrameSent() { stats_.framesSent++; }
    void onValidFrame() { stats_.validFrames++; stats_.lastValidFrameTime = currentTimestamp_; }
    void onInvalidFrame() { stats_.invalidFrames++; }
    void onCrcError() { stats_.crcErrors++; }
    void onSequenceError() { stats_.sequenceErrors++; }
    void onConnectionIdError() { stats_.connectionIdErrors++; }
    void onWatchdogTimeout() { stats_.watchdogTimeouts++; }
    void onCommandError() { stats_.commandErrors++; }
    void onDataLengthError() { stats_.dataLengthErrors++; }
    void onSessionReset() { stats_.sessionResets++; }
    void onFailSafeActivation() { stats_.failSafeActivations++; }
    void onRecoveryAttempt() { stats_.recoveryAttempts++; }
    void onSuccessfulRecovery() { stats_.successfulRecoveries++; }

    /// @brief Update cycle time statistics.
    void updateCycleTime(uint32_t cycleUs) {
        if (cycleUs < stats_.minCycleTimeUs) stats_.minCycleTimeUs = cycleUs;
        if (cycleUs > stats_.maxCycleTimeUs) stats_.maxCycleTimeUs = cycleUs;
        stats_.avgCycleTimeUs = (stats_.avgCycleTimeUs * 7 + cycleUs) / 8;
    }

    /// @brief Update gap timing (time since last valid frame).
    void updateGap(uint64_t gapMs) {
        if (gapMs > stats_.longestGapMs) {
            stats_.longestGapMs = gapMs;
        }
    }

    // ---- Diagnostic logging ------------------------------------------------

    /// @brief Log a diagnostic entry.
    void logDiagnostic(uint16_t errorCode, const char* message);

    // ---- Accessors ---------------------------------------------------------

    FSoESlaveStats getStats() const { return stats_; }
    std::vector<FSoEDiagnosticEntry> getDiagnostics() const { return diagnostics_; }

    void resetStats() { stats_.reset(); }
    void clearDiagnostics() { diagnostics_.clear(); }

    void resetAll() {
        stats_.reset();
        diagnostics_.clear();
    }

private:
    FSoESlaveStats stats_;
    std::vector<FSoEDiagnosticEntry> diagnostics_;

    bool enableDiagnostics_ = true;
    uint32_t maxEntries_ = 100;

    // Context for diagnostic entries
    uint64_t currentTimestamp_ = 0;
    uint8_t  currentState_ = 0;
    uint8_t  currentSequence_ = 0;
    uint16_t currentConnectionId_ = 0;
};

} // namespace FSoE
