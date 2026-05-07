/**
 * @file DCErrorBehavior.hpp
 * @brief Distributed Clock Error Detection and Handling
 *
 * Provides comprehensive DC error detection for slaves including:
 * - Clock drift detection and limits
 * - Clock shift/jump detection
 * - Packet order validation
 * - Partial configuration detection
 * - SYNC signal monitoring
 * - Jitter measurement and limits
 * - Error injection for testing
 */

#pragma once

#include "slave/core/SlaveErrorBehavior.hpp"
#include <array>
#include <atomic>
#include <cstdint>
#include <functional>
#include <mutex>
#include <deque>

namespace EtherCAT {
namespace slave {
namespace DC {

// ============================================================================
// DC Error Types
// ============================================================================

/**
 * @brief DC-specific error codes
 */
namespace DCError {
    // Clock errors
    constexpr uint16_t ClockNotInitialized   = 0x0310;
    constexpr uint16_t ClockDriftExceeded    = 0x0311;
    constexpr uint16_t ClockJumpDetected     = 0x0312;
    constexpr uint16_t ClockNegativeJump     = 0x0313;
    constexpr uint16_t ClockSyncLost         = 0x0314;
    
    // Configuration errors
    constexpr uint16_t ConfigIncomplete      = 0x0320;
    constexpr uint16_t ConfigMismatch        = 0x0321;
    constexpr uint16_t CycleTimeMismatch     = 0x0322;
    constexpr uint16_t ShiftTimeMismatch     = 0x0323;
    
    // Sync signal errors
    constexpr uint16_t Sync0Missing          = 0x0330;
    constexpr uint16_t Sync0Late             = 0x0331;
    constexpr uint16_t Sync0Early            = 0x0332;
    constexpr uint16_t Sync1Missing          = 0x0333;
    constexpr uint16_t Sync1Late             = 0x0334;
    constexpr uint16_t Sync1Early            = 0x0335;
    
    // Timing errors
    constexpr uint16_t JitterExceeded        = 0x0340;
    constexpr uint16_t PropagationError      = 0x0341;
    constexpr uint16_t OffsetTooLarge        = 0x0342;
    constexpr uint16_t CycleOverrun          = 0x0343;
    
    // Packet errors
    constexpr uint16_t PacketOrderError      = 0x0350;
    constexpr uint16_t PacketMissing         = 0x0351;
    constexpr uint16_t PacketDuplicate       = 0x0352;
    constexpr uint16_t PacketTimeout         = 0x0353;
}

// ============================================================================
// DC Configuration
// ============================================================================

/**
 * @brief DC error detection configuration
 */
struct DCErrorConfig {
    // Clock limits
    int64_t maxClockDriftNsPerSecond = 100000;   ///< Max drift in ns/s (100 ppm)
    int64_t maxClockJumpNs = 1000000;            ///< Max allowed clock jump (1ms)
    uint64_t minClockJumpNs = 100000;            ///< Min jump to detect (100us)
    
    // Sync signal limits
    int32_t maxSync0DeviationNs = 50000;         ///< Max SYNC0 timing deviation (50us)
    int32_t maxSync1DeviationNs = 50000;         ///< Max SYNC1 timing deviation (50us)
    uint32_t maxMissedSync0 = 3;                 ///< Max consecutive missed SYNC0
    uint32_t maxMissedSync1 = 3;                 ///< Max consecutive missed SYNC1
    
    // Jitter limits
    uint32_t maxJitterNs = 100000;               ///< Maximum allowed jitter (100us)
    uint32_t jitterWindowSamples = 100;          ///< Samples for jitter calculation
    
    // Packet timing
    uint32_t maxPacketDelayUs = 500;             ///< Max packet delay (500us)
    uint32_t packetTimeoutMs = 100;              ///< Packet timeout (100ms)
    
    // Propagation
    uint32_t maxPropagationDelayNs = 10000000;   ///< Max propagation delay (10ms)
    uint32_t maxOffsetCorrectionNs = 1000000000; ///< Max offset correction (1s)
    
    // Error handling
    bool driftErrorCritical = false;             ///< Is drift error critical?
    bool jumpErrorCritical = true;               ///< Is jump error critical?
    bool syncMissingCritical = true;             ///< Is sync missing critical?
    bool jitterErrorCritical = false;            ///< Is jitter error critical?
    bool packetErrorCritical = true;             ///< Is packet error critical?
    bool configErrorCritical = true;             ///< Is config error critical?
};

// ============================================================================
// DC Error Injection
// ============================================================================

/**
 * @brief DC error injection configuration
 */
struct DCErrorInjection {
    bool enabled = false;
    
    // Clock manipulation
    bool injectClockDrift = false;
    int64_t driftRateNsPerSecond = 0;           ///< Injected drift rate
    
    bool injectClockJump = false;
    int64_t jumpAmountNs = 0;                   ///< Clock jump amount
    uint64_t jumpAfterMs = 0;                   ///< Inject jump after this time
    bool jumpInjected = false;
    
    // Sync manipulation
    bool injectSync0Missing = false;
    uint32_t sync0MissCount = 0;
    
    bool injectSync0Delay = false;
    int32_t sync0DelayNs = 0;
    
    bool injectSync1Missing = false;
    uint32_t sync1MissCount = 0;
    
    // Jitter injection
    bool injectJitter = false;
    uint32_t jitterAmountNs = 0;
    
    // Packet manipulation
    bool injectPacketDelay = false;
    uint32_t packetDelayUs = 0;
    
    bool injectPacketDrop = false;
    uint32_t packetDropRate = 0;
    uint32_t packetDropCounter = 0;
    
    bool injectPacketReorder = false;
    
    // Configuration manipulation
    bool injectPartialConfig = false;
    bool injectConfigMismatch = false;
    
    void reset() {
        enabled = false;
        injectClockDrift = false;
        injectClockJump = false;
        jumpInjected = false;
        injectSync0Missing = false;
        injectSync0Delay = false;
        injectSync1Missing = false;
        injectJitter = false;
        injectPacketDelay = false;
        injectPacketDrop = false;
        injectPacketReorder = false;
        injectPartialConfig = false;
        injectConfigMismatch = false;
    }
};

// ============================================================================
// DC Statistics
// ============================================================================

/**
 * @brief DC timing statistics
 */
struct DCStatistics {
    // Clock statistics
    int64_t totalDriftNs = 0;
    int64_t maxDriftNs = 0;
    int64_t minDriftNs = 0;
    uint32_t clockJumps = 0;
    int64_t largestJumpNs = 0;
    
    // Sync statistics
    uint64_t sync0Received = 0;
    uint64_t sync0Missed = 0;
    uint64_t sync1Received = 0;
    uint64_t sync1Missed = 0;
    int32_t maxSync0Deviation = 0;
    int32_t maxSync1Deviation = 0;
    
    // Jitter statistics
    uint32_t currentJitterNs = 0;
    uint32_t avgJitterNs = 0;
    uint32_t maxJitterNs = 0;
    
    // Packet statistics
    uint64_t packetsReceived = 0;
    uint64_t packetsDropped = 0;
    uint64_t packetsReordered = 0;
    uint64_t packetTimeouts = 0;
    uint32_t maxPacketDelayUs = 0;
    
    // Error counts
    uint32_t driftErrors = 0;
    uint32_t jumpErrors = 0;
    uint32_t syncErrors = 0;
    uint32_t jitterErrors = 0;
    uint32_t packetErrors = 0;
    uint32_t configErrors = 0;
    
    void reset() {
        *this = DCStatistics{};
    }
};

// ============================================================================
// DC State
// ============================================================================

/**
 * @brief DC synchronization state
 */
struct DCState {
    // Clock state
    bool clockInitialized = false;
    uint64_t systemTimeNs = 0;
    uint64_t lastSystemTimeNs = 0;
    int64_t offsetToMasterNs = 0;
    uint32_t propagationDelayNs = 0;
    
    // Sync state
    bool sync0Enabled = false;
    bool sync1Enabled = false;
    uint64_t sync0CycleTimeNs = 0;
    uint64_t sync1CycleTimeNs = 0;
    int32_t sync0ShiftNs = 0;
    int32_t sync1ShiftNs = 0;
    uint64_t lastSync0TimeNs = 0;
    uint64_t lastSync1TimeNs = 0;
    uint64_t nextExpectedSync0Ns = 0;
    uint64_t nextExpectedSync1Ns = 0;
    uint32_t consecutiveMissedSync0 = 0;
    uint32_t consecutiveMissedSync1 = 0;
    
    // Packet tracking
    uint32_t lastPacketSequence = 0;
    uint64_t lastPacketTimeNs = 0;
    
    // Configuration state
    bool dcConfigured = false;
    bool sync0Configured = false;
    bool sync1Configured = false;
    bool cycleTimeConfigured = false;
    
    void reset() {
        *this = DCState{};
    }
};

// ============================================================================
// Callback Types
// ============================================================================

using DCErrorCallback = std::function<void(uint16_t errorCode, int64_t value)>;
using DCSync0Callback = std::function<void(uint64_t timestamp)>;
using DCSync1Callback = std::function<void(uint64_t timestamp)>;

// ============================================================================
// DC Error Handler
// ============================================================================

/**
 * @brief DC Error Detection and Handling
 */
class DCErrorHandler {
public:
    explicit DCErrorHandler(const DCErrorConfig& config = DCErrorConfig{});
    ~DCErrorHandler() = default;
    
    // ========================================================================
    // Configuration
    // ========================================================================
    
    void setConfig(const DCErrorConfig& config);
    const DCErrorConfig& getConfig() const { return config_; }
    
    void setErrorHandler(ErrorBehaviorHandler* handler) { errorHandler_ = handler; }
    
    // ========================================================================
    // DC Operations
    // ========================================================================
    
    /**
     * @brief Initialize DC
     * @param cycleTimeNs Cycle time in nanoseconds
     * @return true on success
     */
    bool initialize(uint64_t cycleTimeNs);
    
    /**
     * @brief Check if DC is initialized
     */
    bool isInitialized() const { return state_.clockInitialized; }
    
    /**
     * @brief Reset DC state
     */
    void reset();
    
    /**
     * @brief Configure SYNC0
     */
    bool configureSync0(uint64_t cycleTimeNs, int32_t shiftNs);
    
    /**
     * @brief Configure SYNC1
     */
    bool configureSync1(uint64_t cycleTimeNs, int32_t shiftNs);
    
    /**
     * @brief Process DC system time update
     * @param newSystemTimeNs New system time from master
     * @return true if no errors
     */
    bool processSystemTimeUpdate(uint64_t newSystemTimeNs);
    
    /**
     * @brief Process SYNC0 event
     * @param timestampNs SYNC0 timestamp
     * @return true if no errors
     */
    bool processSync0(uint64_t timestampNs);
    
    /**
     * @brief Process SYNC1 event
     * @param timestampNs SYNC1 timestamp
     * @return true if no errors
     */
    bool processSync1(uint64_t timestampNs);
    
    /**
     * @brief Process incoming packet
     * @param sequenceNum Packet sequence number
     * @param timestampNs Packet receive timestamp
     * @return true if no errors
     */
    bool processPacket(uint32_t sequenceNum, uint64_t timestampNs);
    
    /**
     * @brief Update DC handler (call periodically)
     * @param currentTimeNs Current time in nanoseconds
     */
    void update(uint64_t currentTimeNs);
    
    // ========================================================================
    // State Access
    // ========================================================================
    
    const DCState& getState() const { return state_; }
    DCState& getState() { return state_; }
    
    /**
     * @brief Get current system time
     */
    uint64_t getSystemTime() const { return state_.systemTimeNs; }
    
    /**
     * @brief Get offset to master
     */
    int64_t getOffsetToMaster() const { return state_.offsetToMasterNs; }
    
    /**
     * @brief Check if configuration is complete
     */
    bool isConfigurationComplete() const;
    
    /**
     * @brief Check for partial configuration
     */
    bool hasPartialConfiguration() const;
    
    // ========================================================================
    // Error Checking
    // ========================================================================
    
    /**
     * @brief Check for clock drift
     * @return true if drift within limits
     */
    bool checkClockDrift();
    
    /**
     * @brief Check for clock jump
     * @param deltaTimeNs Time since last update
     * @return true if no jump detected
     */
    bool checkClockJump(int64_t deltaTimeNs);
    
    /**
     * @brief Check for missed SYNC signals
     * @param currentTimeNs Current time
     * @return true if no missed syncs
     */
    bool checkSyncSignals(uint64_t currentTimeNs);
    
    /**
     * @brief Check jitter
     * @return true if jitter within limits
     */
    bool checkJitter();
    
    /**
     * @brief Check packet order
     * @param sequenceNum Packet sequence number
     * @return true if order correct
     */
    bool checkPacketOrder(uint32_t sequenceNum);
    
    // ========================================================================
    // Callbacks
    // ========================================================================
    
    void setErrorCallback(DCErrorCallback callback) { errorCallback_ = callback; }
    void setSync0Callback(DCSync0Callback callback) { sync0Callback_ = callback; }
    void setSync1Callback(DCSync1Callback callback) { sync1Callback_ = callback; }
    
    // ========================================================================
    // Statistics
    // ========================================================================
    
    const DCStatistics& getStatistics() const { return stats_; }
    void resetStatistics() { stats_.reset(); }
    
    // ========================================================================
    // Error Injection
    // ========================================================================
    
    DCErrorInjection& getErrorInjection() { return injection_; }
    const DCErrorInjection& getErrorInjection() const { return injection_; }
    
    /**
     * @brief Apply error injection to system time
     */
    uint64_t applyTimeInjection(uint64_t timeNs);

private:
    void reportError(uint16_t errorCode, int64_t value = 0);
    void addJitterSample(uint32_t jitterNs);
    uint32_t calculateJitter() const;
    
    DCErrorConfig config_;
    DCState state_;
    mutable DCStatistics stats_;  // mutable to allow updating cached values in const methods
    DCErrorInjection injection_;
    
    ErrorBehaviorHandler* errorHandler_ = nullptr;
    
    // Jitter calculation
    std::deque<uint32_t> jitterSamples_;
    uint64_t lastCycleTimeNs_ = 0;
    
    // Packet tracking
    std::deque<std::pair<uint32_t, uint64_t>> recentPackets_;  // seq, time
    
    DCErrorCallback errorCallback_;
    DCSync0Callback sync0Callback_;
    DCSync1Callback sync1Callback_;
    
    mutable std::mutex mutex_;
};

} // namespace DC
} // namespace slave
} // namespace EtherCAT
