/**
 * @file SlaveErrorBehavior.cpp
 * @brief Error Behavior Handler Implementation
 */

#include "slave/core/SlaveErrorBehavior.hpp"
#include <algorithm>
#include <cstring>
#include <cstdio>
#include <random>

namespace EtherCAT {
namespace slave {

// ============================================================================
// ErrorBehaviorHandler Implementation
// ============================================================================

ErrorBehaviorHandler::ErrorBehaviorHandler(const ErrorBehaviorConfig& config)
    : config_(config)
{
}

void ErrorBehaviorHandler::setConfig(const ErrorBehaviorConfig& config) {
    std::lock_guard<std::mutex> lock(mutex_);
    config_ = config;
}

void ErrorBehaviorHandler::configureError(uint16_t errorCode, const ErrorSeverityConfig& severity) {
    std::lock_guard<std::mutex> lock(mutex_);
    errorSeverityOverrides_[errorCode] = severity;
}

void ErrorBehaviorHandler::configureCategory(ErrorCategory category, const ErrorSeverityConfig& severity) {
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (static_cast<size_t>(category) < config_.categoryConfig.size()) {
        config_.categoryConfig[static_cast<size_t>(category)] = severity;
    }
}

bool ErrorBehaviorHandler::reportError(uint16_t errorCode, ErrorCategory category, const char* message) {
    return reportErrorWithContext(errorCode, category, nullptr, 0, message);
}

bool ErrorBehaviorHandler::reportErrorWithContext(uint16_t errorCode, ErrorCategory category,
                                                   const void* context, size_t contextLen,
                                                   const char* message) {
    std::lock_guard<std::mutex> lock(mutex_);
    
    // Check cooldown
    auto cooldownIt = errorCooldowns_.find(errorCode);
    if (cooldownIt != errorCooldowns_.end()) {
        if (currentTimeMs_ < cooldownIt->second) {
            return false;  // Still in cooldown
        }
    }
    
    // Get severity configuration
    bool isCritical = shouldTriggerCritical(errorCode, category);
    
    // Update statistics
    stats_.totalErrors++;
    if (isCritical) {
        stats_.criticalErrors++;
    } else {
        stats_.nonCriticalErrors++;
    }
    
    if (static_cast<size_t>(category) < stats_.categoryCount.size()) {
        stats_.categoryCount[static_cast<size_t>(category)]++;
    }
    
    if (stats_.firstErrorTime == 0) {
        stats_.firstErrorTime = currentTimeMs_;
    }
    stats_.lastErrorTime = currentTimeMs_;
    
    // Update occurrence tracking
    errorOccurrences_[errorCode]++;
    
    // Get severity config for cooldown
    const ErrorSeverityConfig* severityConfig = nullptr;
    auto overrideIt = errorSeverityOverrides_.find(errorCode);
    if (overrideIt != errorSeverityOverrides_.end()) {
        severityConfig = &overrideIt->second;
    } else if (static_cast<size_t>(category) < config_.categoryConfig.size()) {
        severityConfig = &config_.categoryConfig[static_cast<size_t>(category)];
    }
    
    // Apply cooldown
    if (severityConfig && severityConfig->cooldownMs > 0) {
        errorCooldowns_[errorCode] = currentTimeMs_ + severityConfig->cooldownMs;
    }
    
    // Log error
    if (config_.enableErrorLogging && 
        (!severityConfig || severityConfig->logError)) {
        logError(errorCode, category, isCritical, message);
        
        // Add context if provided
        if (context && contextLen > 0 && !errorLog_.empty()) {
            size_t copyLen = std::min(contextLen, sizeof(errorLog_.back().context.raw));
            std::memcpy(errorLog_.back().context.raw, context, copyLen);
        }
    }
    
    // Update state
    lastErrorCode_ = errorCode;
    lastErrorCategory_ = category;
    errorActive_ = true;
    
    // Notify callback
    if (errorCallback_ && (!severityConfig || severityConfig->notifyCallback)) {
        errorCallback_(errorCode, category, isCritical);
    }
    
    // Trigger fail-safe if critical
    if (isCritical) {
        if (!severityConfig || severityConfig->triggerFailSafe) {
            triggerFailSafe(errorCode);
        }
    }
    
    return isCritical;
}

void ErrorBehaviorHandler::clearError() {
    std::lock_guard<std::mutex> lock(mutex_);
    
    errorActive_ = false;
    lastErrorCode_ = 0;
    lastErrorCategory_ = ErrorCategory::None;
}

bool ErrorBehaviorHandler::attemptRecovery() {
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (!failSafeActive_) {
        return false;  // Not in fail-safe
    }
    
    if (!config_.autoRecoveryEnabled) {
        return false;
    }
    
    // Check recovery attempts
    if (recoveryAttemptCount_ >= config_.maxRecoveryAttempts) {
        return false;
    }
    
    // Check recovery delay
    if (currentTimeMs_ - lastRecoveryAttemptMs_ < config_.recoveryDelayMs) {
        return false;
    }
    
    // Check recovery callback
    if (recoveryCallback_ && !recoveryCallback_()) {
        return false;
    }
    
    stats_.recoveryAttempts++;
    recoveryAttemptCount_++;
    lastRecoveryAttemptMs_ = currentTimeMs_;
    
    // Clear fail-safe
    failSafeActive_ = false;
    errorActive_ = false;
    lastErrorCode_ = 0;
    lastErrorCategory_ = ErrorCategory::None;
    
    // Clear occurrence tracking
    errorOccurrences_.clear();
    
    stats_.successfulRecoveries++;
    
    return true;
}

bool ErrorBehaviorHandler::canRecover() const {
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (!failSafeActive_) {
        return false;
    }
    
    if (!config_.autoRecoveryEnabled) {
        return false;
    }
    
    if (recoveryAttemptCount_ >= config_.maxRecoveryAttempts) {
        return false;
    }
    
    if (currentTimeMs_ - lastRecoveryAttemptMs_ < config_.recoveryDelayMs) {
        return false;
    }
    
    return true;
}

bool ErrorBehaviorHandler::shouldInjectError(ErrorCategory category) {
    if (!injection_.enabled) {
        return false;
    }
    
    // Check category filter
    if (category != ErrorCategory::None && 
        static_cast<size_t>(category) < injection_.injectCategory_.size()) {
        if (!injection_.injectCategory_[static_cast<size_t>(category)]) {
            return false;
        }
    }
    
    // Check scheduled injection
    if (injection_.scheduledInjection) {
        return currentTimeMs_ >= injection_.injectionTimeMs;
    }
    
    // Check random injection
    if (injection_.randomInjection) {
        static std::random_device rd;
        static std::mt19937 gen(rd());
        std::uniform_int_distribution<> dis(0, 10000);
        return dis(gen) < static_cast<int>(injection_.randomProbability);
    }
    
    // Check rate-based injection
    if (injection_.injectionRate == 0) {
        return true;  // Every call
    }
    
    injection_.injectionCounter++;
    if (injection_.injectionCounter >= injection_.injectionRate) {
        injection_.injectionCounter = 0;
        return true;
    }
    
    return false;
}

void ErrorBehaviorHandler::injectError(uint16_t errorCode, ErrorCategory category) {
    reportError(errorCode, category, "Injected error");
}

void ErrorBehaviorHandler::update(uint64_t currentTimeMs) {
    std::lock_guard<std::mutex> lock(mutex_);
    currentTimeMs_ = currentTimeMs;
    
    // Check for scheduled injection
    if (injection_.enabled && injection_.scheduledInjection) {
        if (currentTimeMs >= injection_.injectionTimeMs) {
            reportError(injection_.injectErrorCode, injection_.injectCategory, "Scheduled injection");
            injection_.scheduledInjection = false;
        }
    }
}

bool ErrorBehaviorHandler::shouldTriggerCritical(uint16_t errorCode, ErrorCategory category) {
    // Check per-error override
    auto overrideIt = errorSeverityOverrides_.find(errorCode);
    if (overrideIt != errorSeverityOverrides_.end()) {
        const auto& severity = overrideIt->second;
        
        if (severity.maxOccurrences > 0) {
            auto occIt = errorOccurrences_.find(errorCode);
            if (occIt != errorOccurrences_.end() && 
                occIt->second < severity.maxOccurrences) {
                return false;  // Not enough occurrences yet
            }
        }
        
        return severity.isCritical;
    }
    
    // Check category configuration
    if (static_cast<size_t>(category) < config_.categoryConfig.size()) {
        const auto& severity = config_.categoryConfig[static_cast<size_t>(category)];
        
        if (severity.maxOccurrences > 0) {
            auto occIt = errorOccurrences_.find(errorCode);
            if (occIt != errorOccurrences_.end() &&
                occIt->second < severity.maxOccurrences) {
                return false;
            }
        }
        
        return severity.isCritical;
    }
    
    return config_.defaultCritical;
}

void ErrorBehaviorHandler::triggerFailSafe(uint16_t errorCode) {
    failSafeActive_ = true;
    
    if (failSafeCallback_) {
        failSafeCallback_(errorCode);
    }
}

void ErrorBehaviorHandler::logError(uint16_t errorCode, ErrorCategory category,
                                     bool isCritical, const char* message) {
    // Limit log size
    if (errorLog_.size() >= config_.maxLogEntries) {
        errorLog_.erase(errorLog_.begin());
    }
    
    ErrorEntry entry;
    entry.timestamp = currentTimeMs_;
    entry.errorCode = errorCode;
    entry.category = category;
    entry.isCritical = isCritical;
    entry.wasHandled = false;
    
    auto occIt = errorOccurrences_.find(errorCode);
    entry.occurrences = (occIt != errorOccurrences_.end()) ? occIt->second : 1;
    
    if (message) {
        strncpy(entry.message, message, sizeof(entry.message) - 1);
        entry.message[sizeof(entry.message) - 1] = '\0';
    } else {
        entry.message[0] = '\0';
    }
    
    std::memset(entry.context.raw, 0, sizeof(entry.context.raw));
    
    errorLog_.push_back(entry);
}

} // namespace slave
} // namespace EtherCAT
