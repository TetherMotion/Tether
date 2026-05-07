/**
 * @file SlaveErrorBehavior.hpp
 * @brief Configurable Error Behavior Framework for EtherCAT Slaves
 *
 * Provides a comprehensive error detection and handling framework for slaves
 * with configurable critical/non-critical error responses.
 *
 * Features:
 * - Categorized error types (FSoE, DC, State Machine, Communication)
 * - Configurable error severity (critical vs non-critical)
 * - Error injection for testing
 * - Error statistics and logging
 * - Callback-based error notification
 * - Recovery strategies
 */

#pragma once

#include <array>
#include <atomic>
#include <cstdint>
#include <functional>
#include <map>
#include <mutex>
#include <string>
#include <vector>

namespace EtherCAT {
namespace slave {

// ============================================================================
// Error Categories
// ============================================================================

/**
 * @brief Error category enumeration
 */
enum class ErrorCategory : uint8_t {
    None = 0,
    Communication,   ///< Frame/packet level errors
    StateMachine,    ///< AL state machine errors
    DC,              ///< Distributed Clock errors
    FSoE,            ///< Safety protocol errors
    Profile,         ///< Profile-specific errors (CiA402, etc.)
    Watchdog,        ///< Watchdog timeout errors
    Memory,          ///< Memory/buffer errors
    Configuration,   ///< Configuration errors
    Hardware,        ///< Hardware-related errors
    Application      ///< Application-level errors
};

/**
 * @brief Get category name
 */
inline const char* getCategoryName(ErrorCategory cat) {
    switch (cat) {
        case ErrorCategory::None:           return "NONE";
        case ErrorCategory::Communication:  return "COMMUNICATION";
        case ErrorCategory::StateMachine:   return "STATE_MACHINE";
        case ErrorCategory::DC:             return "DC";
        case ErrorCategory::FSoE:           return "FSOE";
        case ErrorCategory::Profile:        return "PROFILE";
        case ErrorCategory::Watchdog:       return "WATCHDOG";
        case ErrorCategory::Memory:         return "MEMORY";
        case ErrorCategory::Configuration:  return "CONFIGURATION";
        case ErrorCategory::Hardware:       return "HARDWARE";
        case ErrorCategory::Application:    return "APPLICATION";
        default:                            return "UNKNOWN";
    }
}

// ============================================================================
// Error Codes
// ============================================================================

namespace ErrorCodes {
    // Communication errors (0x0100-0x01FF)
    constexpr uint16_t CommFrameCRC         = 0x0101;
    constexpr uint16_t CommFrameLength      = 0x0102;
    constexpr uint16_t CommFrameSequence    = 0x0103;
    constexpr uint16_t CommFrameTimeout     = 0x0104;
    constexpr uint16_t CommFrameDropped     = 0x0105;
    constexpr uint16_t CommMailboxOverflow  = 0x0106;
    constexpr uint16_t CommWKCMismatch      = 0x0107;
    
    // State machine errors (0x0200-0x02FF)
    constexpr uint16_t SMInvalidTransition  = 0x0201;
    constexpr uint16_t SMBootstrapFailed    = 0x0202;
    constexpr uint16_t SMPreOpFailed        = 0x0203;
    constexpr uint16_t SMSafeOpFailed       = 0x0204;
    constexpr uint16_t SMOpFailed           = 0x0205;
    constexpr uint16_t SMEmergencyStop      = 0x0206;
    constexpr uint16_t SMInternalError      = 0x0207;
    
    // DC errors (0x0300-0x03FF)
    constexpr uint16_t DCNotSupported       = 0x0301;
    constexpr uint16_t DCInitFailed         = 0x0302;
    constexpr uint16_t DCClockDrift         = 0x0303;
    constexpr uint16_t DCClockJump          = 0x0304;
    constexpr uint16_t DCSync0Missing       = 0x0305;
    constexpr uint16_t DCSync1Missing       = 0x0306;
    constexpr uint16_t DCPropagationError   = 0x0307;
    constexpr uint16_t DCPacketOrder        = 0x0308;
    constexpr uint16_t DCPartialConfig      = 0x0309;
    constexpr uint16_t DCJitterExceeded     = 0x030A;
    constexpr uint16_t DCOffsetTooLarge     = 0x030B;
    
    // FSoE errors (0x0400-0x04FF)
    constexpr uint16_t FSoECRCError         = 0x0401;
    constexpr uint16_t FSoESequenceError    = 0x0402;
    constexpr uint16_t FSoEWatchdogError    = 0x0403;
    constexpr uint16_t FSoEConnectionError  = 0x0404;
    constexpr uint16_t FSoESessionError     = 0x0405;
    constexpr uint16_t FSoEParameterError   = 0x0406;
    constexpr uint16_t FSoEDataLengthError  = 0x0407;
    constexpr uint16_t FSoECommChannel      = 0x0408;
    constexpr uint16_t FSoESafetyViolation  = 0x0409;
    
    // Profile errors (0x0500-0x05FF)
    constexpr uint16_t ProfileInvalidMode   = 0x0501;
    constexpr uint16_t ProfileFollowingErr  = 0x0502;
    constexpr uint16_t ProfilePositionLimit = 0x0503;
    constexpr uint16_t ProfileVelocityLimit = 0x0504;
    constexpr uint16_t ProfileTorqueLimit   = 0x0505;
    constexpr uint16_t ProfileHomingFailed  = 0x0506;
    constexpr uint16_t ProfileOvercurrent   = 0x0507;
    constexpr uint16_t ProfileOvervoltage   = 0x0508;
    constexpr uint16_t ProfileUndervoltage  = 0x0509;
    constexpr uint16_t ProfileOvertemp      = 0x050A;
    constexpr uint16_t ProfileEncoder       = 0x050B;
    constexpr uint16_t ProfileBrakeError    = 0x050C;
    
    // Watchdog errors (0x0600-0x06FF)
    constexpr uint16_t WatchdogPDO          = 0x0601;
    constexpr uint16_t WatchdogSM           = 0x0602;
    constexpr uint16_t WatchdogApplication  = 0x0603;
    
    // Memory errors (0x0700-0x07FF)
    constexpr uint16_t MemoryAllocation     = 0x0701;
    constexpr uint16_t MemoryOverflow       = 0x0702;
    constexpr uint16_t MemoryCorruption     = 0x0703;
    
    // Configuration errors (0x0800-0x08FF)
    constexpr uint16_t ConfigInvalid        = 0x0801;
    constexpr uint16_t ConfigMissing        = 0x0802;
    constexpr uint16_t ConfigMismatch       = 0x0803;
    
    // Hardware errors (0x0900-0x09FF)
    constexpr uint16_t HardwareGeneral      = 0x0901;
    constexpr uint16_t HardwarePHY          = 0x0902;
    constexpr uint16_t HardwareEEPROM       = 0x0903;
    constexpr uint16_t HardwareESC          = 0x0904;
    
    // Application errors (0x0A00-0x0AFF)
    constexpr uint16_t AppGeneral           = 0x0A01;
    constexpr uint16_t AppCallback          = 0x0A02;
    constexpr uint16_t AppAssertion         = 0x0A03;
}

// ============================================================================
// Error Entry
// ============================================================================

/**
 * @brief Error log entry
 */
struct ErrorEntry {
    uint64_t timestamp;
    uint16_t errorCode;
    ErrorCategory category;
    bool isCritical;
    bool wasHandled;
    uint8_t occurrences;
    char message[64];
    
    // Context data (varies by error type)
    union {
        struct {
            uint8_t expectedSeq;
            uint8_t receivedSeq;
        } sequence;
        struct {
            int64_t expectedOffset;
            int64_t actualOffset;
        } dcOffset;
        struct {
            int32_t targetPos;
            int32_t actualPos;
            int32_t error;
        } following;
        uint8_t raw[16];
    } context;
};

// ============================================================================
// Error Behavior Configuration
// ============================================================================

/**
 * @brief Error severity configuration
 */
struct ErrorSeverityConfig {
    bool isCritical = true;          ///< Is this error critical?
    bool triggerFailSafe = true;     ///< Trigger fail-safe on this error?
    bool logError = true;            ///< Log this error?
    bool notifyCallback = true;      ///< Call error callback?
    uint8_t maxOccurrences = 0;      ///< Max occurrences before critical (0 = immediate)
    uint16_t cooldownMs = 0;         ///< Cooldown between error reports
};

/**
 * @brief Complete error behavior configuration
 */
struct ErrorBehaviorConfig {
    // Default behavior
    bool defaultCritical = true;
    bool defaultFailSafe = true;
    
    // Per-category configuration
    std::array<ErrorSeverityConfig, 11> categoryConfig;  // One per ErrorCategory
    
    // Global settings
    bool enableErrorLogging = true;
    uint32_t maxLogEntries = 256;
    bool enableErrorStatistics = true;
    bool enableErrorInjection = false;
    
    // Recovery settings
    bool autoRecoveryEnabled = true;
    uint32_t recoveryDelayMs = 1000;
    uint32_t maxRecoveryAttempts = 3;
    
    /**
     * @brief Get default configuration
     */
    static ErrorBehaviorConfig defaults();
    
    /**
     * @brief Get strict configuration (all errors critical)
     */
    static ErrorBehaviorConfig strict();
    
    /**
     * @brief Get permissive configuration (only some errors critical)
     */
    static ErrorBehaviorConfig permissive();
};

// ============================================================================
// Error Statistics
// ============================================================================

/**
 * @brief Error statistics
 */
struct ErrorStatistics {
    // Counters by category
    std::array<uint32_t, 11> categoryCount{};
    
    // Critical vs non-critical
    uint32_t criticalErrors = 0;
    uint32_t nonCriticalErrors = 0;
    uint32_t totalErrors = 0;
    
    // Recovery
    uint32_t recoveryAttempts = 0;
    uint32_t successfulRecoveries = 0;
    
    // Timing
    uint64_t lastErrorTime = 0;
    uint64_t firstErrorTime = 0;
    
    void reset() {
        categoryCount.fill(0);
        criticalErrors = 0;
        nonCriticalErrors = 0;
        totalErrors = 0;
        recoveryAttempts = 0;
        successfulRecoveries = 0;
        lastErrorTime = 0;
        firstErrorTime = 0;
    }
};

// ============================================================================
// Error Injection Configuration
// ============================================================================

/**
 * @brief Error injection configuration for testing
 */
struct ErrorInjectionConfig {
    bool enabled = false;
    
    // Specific error injection
    uint16_t injectErrorCode = 0;
    ErrorCategory injectCategory = ErrorCategory::None;
    uint32_t injectionRate = 0;        // 0 = every call, N = every Nth call
    uint32_t injectionCounter = 0;
    
    // Random injection
    bool randomInjection = false;
    uint32_t randomProbability = 0;    // 0-10000 (0.00% - 100.00%)
    
    // Scheduled injection
    bool scheduledInjection = false;
    uint64_t injectionTimeMs = 0;
    
    // Category-specific injection
    std::array<bool, 11> injectCategory_{};  // One per ErrorCategory
    
    void reset() {
        enabled = false;
        injectErrorCode = 0;
        injectCategory = ErrorCategory::None;
        injectionRate = 0;
        injectionCounter = 0;
        randomInjection = false;
        scheduledInjection = false;
        injectCategory_.fill(false);
    }
};

// ============================================================================
// Callback Types
// ============================================================================

using ErrorCallback = std::function<void(uint16_t errorCode, ErrorCategory category, bool isCritical)>;
using RecoveryCallback = std::function<bool()>;  // Return true if recovery allowed
using FailSafeCallback = std::function<void(uint16_t errorCode)>;

// ============================================================================
// Error Behavior Handler
// ============================================================================

/**
 * @brief Error behavior handler for slaves
 */
class ErrorBehaviorHandler {
public:
    explicit ErrorBehaviorHandler(const ErrorBehaviorConfig& config = ErrorBehaviorConfig::defaults());
    ~ErrorBehaviorHandler() = default;
    
    // ========================================================================
    // Configuration
    // ========================================================================
    
    /**
     * @brief Set configuration
     */
    void setConfig(const ErrorBehaviorConfig& config);
    
    /**
     * @brief Get configuration
     */
    const ErrorBehaviorConfig& getConfig() const { return config_; }
    
    /**
     * @brief Configure specific error code
     */
    void configureError(uint16_t errorCode, const ErrorSeverityConfig& severity);
    
    /**
     * @brief Configure entire category
     */
    void configureCategory(ErrorCategory category, const ErrorSeverityConfig& severity);
    
    // ========================================================================
    // Error Reporting
    // ========================================================================
    
    /**
     * @brief Report an error
     * @param errorCode Error code
     * @param category Error category
     * @param message Optional message
     * @return true if error was critical
     */
    bool reportError(uint16_t errorCode, ErrorCategory category, const char* message = nullptr);
    
    /**
     * @brief Report error with context
     */
    bool reportErrorWithContext(uint16_t errorCode, ErrorCategory category, 
                                const void* context, size_t contextLen,
                                const char* message = nullptr);
    
    /**
     * @brief Clear current error state
     */
    void clearError();
    
    /**
     * @brief Check if error is active
     */
    bool hasError() const { return errorActive_.load(); }
    
    /**
     * @brief Check if fail-safe is active
     */
    bool isFailSafe() const { return failSafeActive_.load(); }
    
    /**
     * @brief Get last error code
     */
    uint16_t getLastError() const { return lastErrorCode_; }
    
    /**
     * @brief Get last error category
     */
    ErrorCategory getLastErrorCategory() const { return lastErrorCategory_; }
    
    // ========================================================================
    // Recovery
    // ========================================================================
    
    /**
     * @brief Attempt recovery
     */
    bool attemptRecovery();
    
    /**
     * @brief Check if recovery is possible
     */
    bool canRecover() const;
    
    // ========================================================================
    // Callbacks
    // ========================================================================
    
    void setErrorCallback(ErrorCallback callback) { errorCallback_ = callback; }
    void setRecoveryCallback(RecoveryCallback callback) { recoveryCallback_ = callback; }
    void setFailSafeCallback(FailSafeCallback callback) { failSafeCallback_ = callback; }
    
    // ========================================================================
    // Statistics and Logging
    // ========================================================================
    
    /**
     * @brief Get error statistics
     */
    const ErrorStatistics& getStatistics() const { return stats_; }
    
    /**
     * @brief Reset statistics
     */
    void resetStatistics() { stats_.reset(); }
    
    /**
     * @brief Get error log
     */
    const std::vector<ErrorEntry>& getErrorLog() const { return errorLog_; }
    
    /**
     * @brief Clear error log
     */
    void clearErrorLog() { errorLog_.clear(); }
    
    // ========================================================================
    // Error Injection (Testing)
    // ========================================================================
    
    /**
     * @brief Get error injection config (mutable)
     */
    ErrorInjectionConfig& getErrorInjection() { return injection_; }
    
    /**
     * @brief Check if injection should occur
     */
    bool shouldInjectError(ErrorCategory category = ErrorCategory::None);
    
    /**
     * @brief Inject an error (for testing)
     */
    void injectError(uint16_t errorCode, ErrorCategory category);
    
    /**
     * @brief Update handler (call periodically)
     */
    void update(uint64_t currentTimeMs);

private:
    bool shouldTriggerCritical(uint16_t errorCode, ErrorCategory category);
    void triggerFailSafe(uint16_t errorCode);
    void logError(uint16_t errorCode, ErrorCategory category, bool isCritical, const char* message);
    
    ErrorBehaviorConfig config_;
    std::map<uint16_t, ErrorSeverityConfig> errorSeverityOverrides_;
    
    // State
    std::atomic<bool> errorActive_{false};
    std::atomic<bool> failSafeActive_{false};
    uint16_t lastErrorCode_ = 0;
    ErrorCategory lastErrorCategory_ = ErrorCategory::None;
    uint32_t recoveryAttemptCount_ = 0;
    uint64_t lastRecoveryAttemptMs_ = 0;
    uint64_t currentTimeMs_ = 0;
    
    // Per-error occurrence tracking
    std::map<uint16_t, uint8_t> errorOccurrences_;
    std::map<uint16_t, uint64_t> errorCooldowns_;
    
    // Callbacks
    ErrorCallback errorCallback_;
    RecoveryCallback recoveryCallback_;
    FailSafeCallback failSafeCallback_;
    
    // Statistics and logging
    ErrorStatistics stats_;
    std::vector<ErrorEntry> errorLog_;
    
    // Injection
    ErrorInjectionConfig injection_;
    
    mutable std::mutex mutex_;
};

// ============================================================================
// Error Behavior Configuration Presets
// ============================================================================

inline ErrorBehaviorConfig ErrorBehaviorConfig::defaults() {
    ErrorBehaviorConfig config;
    
    // Configure each category
    for (size_t i = 0; i < config.categoryConfig.size(); i++) {
        config.categoryConfig[i].isCritical = true;
        config.categoryConfig[i].triggerFailSafe = true;
        config.categoryConfig[i].logError = true;
        config.categoryConfig[i].notifyCallback = true;
    }
    
    // Non-critical by default for some categories
    config.categoryConfig[static_cast<size_t>(ErrorCategory::Communication)].maxOccurrences = 3;
    config.categoryConfig[static_cast<size_t>(ErrorCategory::Watchdog)].maxOccurrences = 1;
    
    return config;
}

inline ErrorBehaviorConfig ErrorBehaviorConfig::strict() {
    ErrorBehaviorConfig config;
    
    config.defaultCritical = true;
    config.defaultFailSafe = true;
    
    for (auto& cat : config.categoryConfig) {
        cat.isCritical = true;
        cat.triggerFailSafe = true;
        cat.maxOccurrences = 0;  // Immediate
    }
    
    return config;
}

inline ErrorBehaviorConfig ErrorBehaviorConfig::permissive() {
    ErrorBehaviorConfig config;
    
    config.defaultCritical = false;
    config.defaultFailSafe = false;
    
    for (auto& cat : config.categoryConfig) {
        cat.isCritical = false;
        cat.triggerFailSafe = false;
        cat.maxOccurrences = 10;  // Allow many occurrences
    }
    
    // Still critical for safety-related
    config.categoryConfig[static_cast<size_t>(ErrorCategory::FSoE)].isCritical = true;
    config.categoryConfig[static_cast<size_t>(ErrorCategory::FSoE)].triggerFailSafe = true;
    
    return config;
}

} // namespace slave
} // namespace EtherCAT
