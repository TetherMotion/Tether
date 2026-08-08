/**
 * @file DCErrorBehavior.cpp
 * @brief DC Error Behavior Implementation
 */

#include "slave/dc/DCErrorBehavior.hpp"
#include <algorithm>
#include <numeric>
#include <cmath>
#include <limits>

namespace EtherCAT {
namespace slave {
namespace DC {

// ============================================================================
// DCErrorHandler Implementation
// ============================================================================

DCErrorHandler::DCErrorHandler(const DCErrorConfig& config)
    : config_(config)
{
}

void DCErrorHandler::setConfig(const DCErrorConfig& config) {
    std::lock_guard<std::mutex> lock(mutex_);
    config_ = config;
}

bool DCErrorHandler::initialize(uint64_t cycleTimeNs) {
    std::lock_guard<std::mutex> lock(mutex_);
    
    state_.reset();
    stats_.reset();
    
    state_.sync0CycleTimeNs = cycleTimeNs;
    state_.clockInitialized = true;
    state_.cycleTimeConfigured = true;
    
    return true;
}

void DCErrorHandler::reset() {
    std::lock_guard<std::mutex> lock(mutex_);
    
    state_.reset();
    jitterSamples_.clear();
    recentPackets_.clear();
    lastCycleTimeNs_ = 0;
}

bool DCErrorHandler::configureSync0(uint64_t cycleTimeNs, int32_t shiftNs) {
    std::lock_guard<std::mutex> lock(mutex_);
    
    // Check for injection
    if (injection_.enabled && injection_.injectPartialConfig) {
        // Don't complete configuration
        state_.sync0Enabled = true;
        state_.sync0CycleTimeNs = cycleTimeNs;
        // Don't set sync0Configured
        return true;
    }
    
    if (injection_.enabled && injection_.injectConfigMismatch) {
        // Configure with wrong values
        state_.sync0CycleTimeNs = cycleTimeNs * 2;  // Wrong cycle time
    } else {
        state_.sync0CycleTimeNs = cycleTimeNs;
    }
    
    state_.sync0ShiftNs = shiftNs;
    state_.sync0Enabled = true;
    state_.sync0Configured = true;
    
    return true;
}

bool DCErrorHandler::configureSync1(uint64_t cycleTimeNs, int32_t shiftNs) {
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (injection_.enabled && injection_.injectPartialConfig) {
        state_.sync1Enabled = true;
        state_.sync1CycleTimeNs = cycleTimeNs;
        return true;
    }
    
    if (injection_.enabled && injection_.injectConfigMismatch) {
        state_.sync1CycleTimeNs = cycleTimeNs * 2;
    } else {
        state_.sync1CycleTimeNs = cycleTimeNs;
    }
    
    state_.sync1ShiftNs = shiftNs;
    state_.sync1Enabled = true;
    state_.sync1Configured = true;
    
    return true;
}

bool DCErrorHandler::processSystemTimeUpdate(uint64_t newSystemTimeNs) {
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (!state_.clockInitialized) {
        reportError(DCError::ClockNotInitialized);
        return false;
    }
    
    // Apply injection
    uint64_t adjustedTime = applyTimeInjection(newSystemTimeNs);
    
    // Store previous time
    state_.lastSystemTimeNs = state_.systemTimeNs;
    state_.systemTimeNs = adjustedTime;
    
    // Check for clock jump
    if (state_.lastSystemTimeNs > 0) {
        int64_t delta = static_cast<int64_t>(adjustedTime) -
                        static_cast<int64_t>(state_.lastSystemTimeNs);

        if (!checkClockJumpUnlocked(delta)) {
            return false;
        }
    }

    // Check drift (only if we have a reference)
    if (!checkClockDriftUnlocked()) {
        return false;
    }
    
    // Calculate jitter
    if (lastCycleTimeNs_ > 0) {
        int64_t expectedDelta = static_cast<int64_t>(state_.sync0CycleTimeNs);
        int64_t actualDelta = static_cast<int64_t>(adjustedTime - lastCycleTimeNs_);
        int64_t deviation = actualDelta - expectedDelta;

        // Accumulate drift (cumulative deviation from expected cycle time)
        state_.offsetToMasterNs += deviation;
        stats_.totalDriftNs = state_.offsetToMasterNs;
        if (state_.offsetToMasterNs > stats_.maxDriftNs) {
            stats_.maxDriftNs = state_.offsetToMasterNs;
        }
        if (state_.offsetToMasterNs < stats_.minDriftNs) {
            stats_.minDriftNs = state_.offsetToMasterNs;
        }

        uint32_t jitter = static_cast<uint32_t>(std::abs(deviation));
        addJitterSample(jitter);

        if (!checkJitterUnlocked()) {
            // Non-fatal by default
        }
    }

    lastCycleTimeNs_ = adjustedTime;
    
    return true;
}

bool DCErrorHandler::processSync0(uint64_t timestampNs) {
    std::lock_guard<std::mutex> lock(mutex_);
    
    stats_.sync0Received++;
    
    // Check for injection
    if (injection_.enabled && injection_.injectSync0Missing) {
        if (injection_.sync0MissCount > 0) {
            injection_.sync0MissCount--;
            // Pretend we didn't receive it
            return true;
        }
    }
    
    // Apply delay injection
    if (injection_.enabled && injection_.injectSync0Delay) {
        timestampNs += injection_.sync0DelayNs;
    }
    
    state_.lastSync0TimeNs = timestampNs;
    state_.consecutiveMissedSync0 = 0;
    
    // Check timing
    if (state_.nextExpectedSync0Ns > 0) {
        int64_t deviation = static_cast<int64_t>(timestampNs) -
                            static_cast<int64_t>(state_.nextExpectedSync0Ns);

        int32_t clamped_dev = static_cast<int32_t>(std::clamp(deviation,
            static_cast<int64_t>(std::numeric_limits<int32_t>::min()), static_cast<int64_t>(std::numeric_limits<int32_t>::max())));

        if (std::abs(clamped_dev) > stats_.maxSync0Deviation) {
            stats_.maxSync0Deviation = std::abs(clamped_dev);
        }

        if (clamped_dev > config_.maxSync0DeviationNs) {
            reportError(DCError::Sync0Late, clamped_dev);
        } else if (clamped_dev < -config_.maxSync0DeviationNs) {
            reportError(DCError::Sync0Early, clamped_dev);
        }
    }

    // Calculate next expected
    state_.nextExpectedSync0Ns = timestampNs + state_.sync0CycleTimeNs;
    
    // Notify callback
    if (sync0Callback_) {
        sync0Callback_(timestampNs);
    }
    
    return true;
}

bool DCErrorHandler::processSync1(uint64_t timestampNs) {
    std::lock_guard<std::mutex> lock(mutex_);
    
    stats_.sync1Received++;
    
    if (injection_.enabled && injection_.injectSync1Missing) {
        if (injection_.sync1MissCount > 0) {
            injection_.sync1MissCount--;
            return true;
        }
    }
    
    state_.lastSync1TimeNs = timestampNs;
    state_.consecutiveMissedSync1 = 0;
    
    if (state_.nextExpectedSync1Ns > 0) {
        int64_t deviation = static_cast<int64_t>(timestampNs) -
                            static_cast<int64_t>(state_.nextExpectedSync1Ns);

        int32_t clamped_dev = static_cast<int32_t>(std::clamp(deviation,
            static_cast<int64_t>(std::numeric_limits<int32_t>::min()), static_cast<int64_t>(std::numeric_limits<int32_t>::max())));

        if (std::abs(clamped_dev) > stats_.maxSync1Deviation) {
            stats_.maxSync1Deviation = std::abs(clamped_dev);
        }

        if (clamped_dev > config_.maxSync1DeviationNs) {
            reportError(DCError::Sync1Late, clamped_dev);
        } else if (clamped_dev < -config_.maxSync1DeviationNs) {
            reportError(DCError::Sync1Early, clamped_dev);
        }
    }

    state_.nextExpectedSync1Ns = timestampNs + state_.sync1CycleTimeNs;
    
    if (sync1Callback_) {
        sync1Callback_(timestampNs);
    }
    
    return true;
}

bool DCErrorHandler::processPacket(uint32_t sequenceNum, uint64_t timestampNs) {
    std::lock_guard<std::mutex> lock(mutex_);
    
    stats_.packetsReceived++;
    
    // Check for packet drop injection
    if (injection_.enabled && injection_.injectPacketDrop) {
        if (injection_.packetDropRate == 0) {
            stats_.packetsDropped++;
            return false;
        }
        injection_.packetDropCounter++;
        if (injection_.packetDropCounter >= injection_.packetDropRate) {
            injection_.packetDropCounter = 0;
            stats_.packetsDropped++;
            return false;
        }
    }
    
    // Apply delay injection
    if (injection_.enabled && injection_.injectPacketDelay) {
        timestampNs += static_cast<uint64_t>(injection_.packetDelayUs) * 1000ULL;
    }

    // Check packet order
    if (!checkPacketOrderUnlocked(sequenceNum)) {
        return false;
    }
    
    // Check packet delay
    if (state_.lastPacketTimeNs > 0) {
        uint64_t delayNs = timestampNs - state_.lastPacketTimeNs;
        uint32_t delayUs = static_cast<uint32_t>(delayNs / 1000);
        
        if (delayUs > stats_.maxPacketDelayUs) {
            stats_.maxPacketDelayUs = delayUs;
        }
        
        if (delayUs > config_.maxPacketDelayUs) {
            reportError(DCError::PacketTimeout, delayUs);
        }
    }
    
    // Track packet
    state_.lastPacketSequence = sequenceNum;
    state_.lastPacketTimeNs = timestampNs;
    
    // Keep recent packets for reorder detection
    recentPackets_.push_back({sequenceNum, timestampNs});
    while (recentPackets_.size() > 10) {
        recentPackets_.pop_front();
    }
    
    return true;
}

void DCErrorHandler::update(uint64_t currentTimeNs) {
    std::lock_guard<std::mutex> lock(mutex_);
    
    // Check for clock jump injection
    if (injection_.enabled && injection_.injectClockJump && !injection_.jumpInjected) {
        if (currentTimeNs >= injection_.jumpAfterMs * 1000000ULL) {
            // Jump will be applied in next time update
            injection_.jumpInjected = true;
        }
    }
    
    // Check for missed SYNC signals
    checkSyncSignalsUnlocked(currentTimeNs);

    // Check configuration completeness
    if (hasPartialConfigurationUnlocked()) {
        reportError(DCError::ConfigIncomplete);
    }
}

bool DCErrorHandler::isConfigurationComplete() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return isConfigurationCompleteUnlocked();
}

bool DCErrorHandler::isConfigurationCompleteUnlocked() const {
    if (!state_.dcConfigured) return false;
    if (state_.sync0Enabled && !state_.sync0Configured) return false;
    if (state_.sync1Enabled && !state_.sync1Configured) return false;
    if (!state_.cycleTimeConfigured) return false;

    return true;
}

bool DCErrorHandler::hasPartialConfiguration() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return hasPartialConfigurationUnlocked();
}

bool DCErrorHandler::hasPartialConfigurationUnlocked() const {
    // Check for partial configuration (some but not all settings)
    bool hasAny = state_.sync0Enabled || state_.sync1Enabled || state_.cycleTimeConfigured;
    bool hasAll = true;

    if (state_.sync0Enabled && !state_.sync0Configured) hasAll = false;
    if (state_.sync1Enabled && !state_.sync1Configured) hasAll = false;

    return hasAny && !hasAll;
}

bool DCErrorHandler::checkClockDrift() {
    std::lock_guard<std::mutex> lock(mutex_);
    return checkClockDriftUnlocked();
}

bool DCErrorHandler::checkClockDriftUnlocked() {
    // Drift is accumulated in offsetToMasterNs by processSystemTimeUpdate
    int64_t drift = state_.offsetToMasterNs;

    if (drift > stats_.maxDriftNs) {
        stats_.maxDriftNs = drift;
    }
    if (drift < stats_.minDriftNs) {
        stats_.minDriftNs = drift;
    }

    // Check drift against limit
    if (std::abs(drift) > config_.maxClockJumpNs) {
        stats_.driftErrors++;
        reportError(DCError::ClockDriftExceeded, drift);
        return !config_.driftErrorCritical;
    }

    return true;
}

bool DCErrorHandler::checkClockJump(int64_t deltaTimeNs) {
    std::lock_guard<std::mutex> lock(mutex_);
    return checkClockJumpUnlocked(deltaTimeNs);
}

bool DCErrorHandler::checkClockJumpUnlocked(int64_t deltaTimeNs) {
    int64_t expectedDelta = static_cast<int64_t>(state_.sync0CycleTimeNs);
    int64_t deviation = deltaTimeNs - expectedDelta;

    // Detect negative time jump (clock went backwards)
    if (deltaTimeNs < 0) {
        stats_.clockJumps++;
        stats_.jumpErrors++;
        if (std::abs(deltaTimeNs) > std::abs(stats_.largestJumpNs)) {
            stats_.largestJumpNs = deltaTimeNs;
        }
        reportError(DCError::ClockNegativeJump, deltaTimeNs);
        return !config_.jumpErrorCritical;
    }

    // Detect large positive jump
    if (static_cast<uint64_t>(std::abs(deviation)) > config_.minClockJumpNs) {
        if (static_cast<uint64_t>(std::abs(deviation)) > config_.maxClockJumpNs) {
            stats_.clockJumps++;
            stats_.jumpErrors++;
            // Store the actual jump amount (deltaTimeNs), not the deviation,
            // for consistency with the negative-jump branch above.
            if (std::abs(deltaTimeNs) > std::abs(stats_.largestJumpNs)) {
                stats_.largestJumpNs = deltaTimeNs;
            }
            reportError(DCError::ClockJumpDetected, deviation);
            return !config_.jumpErrorCritical;
        }
    }

    return true;
}

bool DCErrorHandler::checkSyncSignals(uint64_t currentTimeNs) {
    std::lock_guard<std::mutex> lock(mutex_);
    return checkSyncSignalsUnlocked(currentTimeNs);
}

bool DCErrorHandler::checkSyncSignalsUnlocked(uint64_t currentTimeNs) {
    bool result = true;

    // Check SYNC0
    if (state_.sync0Enabled && state_.nextExpectedSync0Ns > 0) {
        if (currentTimeNs > state_.nextExpectedSync0Ns + config_.maxSync0DeviationNs) {
            state_.consecutiveMissedSync0++;
            stats_.sync0Missed++;

            if (state_.consecutiveMissedSync0 >= config_.maxMissedSync0) {
                stats_.syncErrors++;
                reportError(DCError::Sync0Missing, state_.consecutiveMissedSync0);
                result = !config_.syncMissingCritical;
            }

            // Update expected time
            state_.nextExpectedSync0Ns += state_.sync0CycleTimeNs;
        }
    }

    // Check SYNC1
    if (state_.sync1Enabled && state_.nextExpectedSync1Ns > 0) {
        if (currentTimeNs > state_.nextExpectedSync1Ns + config_.maxSync1DeviationNs) {
            state_.consecutiveMissedSync1++;
            stats_.sync1Missed++;

            if (state_.consecutiveMissedSync1 >= config_.maxMissedSync1) {
                stats_.syncErrors++;
                reportError(DCError::Sync1Missing, state_.consecutiveMissedSync1);
                result = !config_.syncMissingCritical;
            }

            state_.nextExpectedSync1Ns += state_.sync1CycleTimeNs;
        }
    }

    return result;
}

bool DCErrorHandler::checkJitter() {
    std::lock_guard<std::mutex> lock(mutex_);
    return checkJitterUnlocked();
}

bool DCErrorHandler::checkJitterUnlocked() {
    uint32_t jitter = calculateJitter();
    stats_.currentJitterNs = jitter;

    if (jitter > stats_.maxJitterNs) {
        stats_.maxJitterNs = jitter;
    }

    if (jitter > config_.maxJitterNs) {
        stats_.jitterErrors++;
        reportError(DCError::JitterExceeded, jitter);
        return !config_.jitterErrorCritical;
    }

    return true;
}

bool DCErrorHandler::checkPacketOrder(uint32_t sequenceNum) {
    std::lock_guard<std::mutex> lock(mutex_);
    return checkPacketOrderUnlocked(sequenceNum);
}

bool DCErrorHandler::checkPacketOrderUnlocked(uint32_t sequenceNum) {
    // Check for reordering
    if (injection_.enabled && injection_.injectPacketReorder) {
        // Inject reorder by swapping with a recent packet
        if (!recentPackets_.empty()) {
            // Pretend packet came out of order
            stats_.packetsReordered++;
        }
    }

    // Simple check: sequence should be greater than last
    if (state_.lastPacketSequence > 0) {
        if (sequenceNum != state_.lastPacketSequence + 1) {
            if (sequenceNum <= state_.lastPacketSequence) {
                // Out of order or duplicate
                stats_.packetsReordered++;
                stats_.packetErrors++;
                reportError(DCError::PacketOrderError, sequenceNum);
                return !config_.packetErrorCritical;
            } else if (sequenceNum > state_.lastPacketSequence + 1) {
                // Missed packets
                uint32_t missed = sequenceNum - state_.lastPacketSequence - 1;
                stats_.packetsDropped += missed;
                reportError(DCError::PacketMissing, missed);
            }
        }
    }

    return true;
}

uint64_t DCErrorHandler::applyTimeInjection(uint64_t timeNs) {
    if (!injection_.enabled) {
        return timeNs;
    }
    
    uint64_t result = timeNs;
    
    // Apply drift
    if (injection_.injectClockDrift) {
        // Scale drift by elapsed time
        int64_t driftNs = injection_.driftRateNsPerSecond * 
                          static_cast<int64_t>(timeNs / 1000000000ULL);
        result = static_cast<uint64_t>(static_cast<int64_t>(result) + driftNs);
        stats_.totalDriftNs = driftNs;
    }
    
    // Apply jump
    if (injection_.injectClockJump && injection_.jumpInjected) {
        result = static_cast<uint64_t>(static_cast<int64_t>(result) + injection_.jumpAmountNs);
        // Only apply once
        injection_.injectClockJump = false;
    }
    
    // Apply jitter
    if (injection_.injectJitter) {
        // Simple random jitter (would need better random in production)
        int32_t jitter = (injection_.jitterAmountNs % 2 == 0) ? 
                         injection_.jitterAmountNs : -injection_.jitterAmountNs;
        result = static_cast<uint64_t>(static_cast<int64_t>(result) + jitter);
    }
    
    return result;
}

void DCErrorHandler::reportError(uint16_t errorCode, int64_t value) {
    if (errorCallback_) {
        errorCallback_(errorCode, value);
    }
    
    if (errorHandler_) {
        errorHandler_->reportError(errorCode, ErrorCategory::DC);
    }
}

void DCErrorHandler::addJitterSample(uint32_t jitterNs) {
    jitterSamples_.push_back(jitterNs);
    
    while (jitterSamples_.size() > config_.jitterWindowSamples) {
        jitterSamples_.pop_front();
    }
}

uint32_t DCErrorHandler::calculateJitter() const {
    if (jitterSamples_.empty()) {
        return 0;
    }
    
    // Calculate average
    uint64_t sum = std::accumulate(jitterSamples_.begin(), jitterSamples_.end(), 0ULL);
    stats_.avgJitterNs = static_cast<uint32_t>(sum / jitterSamples_.size());
    
    // Return max from window as current jitter
    return *std::max_element(jitterSamples_.begin(), jitterSamples_.end());
}

} // namespace DC
} // namespace slave
} // namespace EtherCAT
