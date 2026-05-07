/**
 * @file SlaveErrorBehaviorTests.cpp
 * @brief Unit tests for Slave Error Behavior
 */

#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include "slave/core/SlaveErrorBehavior.hpp"

namespace EtherCAT {
namespace slave {
namespace test {

// Test fixture
class SlaveErrorBehaviorTest : public ::testing::Test {
protected:
    void SetUp() override {
        ErrorBehaviorConfig config;
        config.criticalCategories = {
            ErrorCategory::Communication,
            ErrorCategory::FSoE,
            ErrorCategory::Hardware
        };
        config.faultOnCritical = true;
        config.autoRecovery = true;
        config.maxRecoveryAttempts = 3;
        config.errorCooldownMs = 1000;
        
        handler_ = std::make_unique<ErrorBehaviorHandler>(config);
    }
    
    void TearDown() override {
        handler_.reset();
    }
    
    std::unique_ptr<ErrorBehaviorHandler> handler_;
};

// =============================================================================
// Initialization Tests
// =============================================================================

TEST_F(SlaveErrorBehaviorTest, InitializesWithDefaults) {
    ErrorBehaviorHandler handler;
    
    EXPECT_FALSE(handler.hasActiveError());
    EXPECT_FALSE(handler.hasFault());
}

TEST_F(SlaveErrorBehaviorTest, ConfigurationIsApplied) {
    ErrorBehaviorConfig config;
    config.faultOnCritical = false;
    config.maxRecoveryAttempts = 5;
    
    ErrorBehaviorHandler handler(config);
    
    EXPECT_FALSE(handler.hasFault());
}

// =============================================================================
// Error Reporting Tests
// =============================================================================

TEST_F(SlaveErrorBehaviorTest, ErrorIsReported) {
    handler_->reportError(
        ErrorCategory::Communication,
        ErrorCodes::Communication::FRAME_LOST,
        "Test frame lost"
    );
    
    EXPECT_TRUE(handler_->hasActiveError());
    EXPECT_EQ(handler_->getActiveErrorCount(), 1u);
}

TEST_F(SlaveErrorBehaviorTest, CriticalErrorCausesFault) {
    handler_->reportError(
        ErrorCategory::Communication, // Critical category
        ErrorCodes::Communication::CONNECTION_LOST,
        "Connection lost"
    );
    
    EXPECT_TRUE(handler_->hasFault());
}

TEST_F(SlaveErrorBehaviorTest, NonCriticalErrorDoesNotFault) {
    handler_->reportError(
        ErrorCategory::Application, // Non-critical
        0x1000,
        "Application warning"
    );
    
    EXPECT_FALSE(handler_->hasFault());
}

TEST_F(SlaveErrorBehaviorTest, MultipleErrorsAccumulate) {
    handler_->reportError(ErrorCategory::Application, 0x1001, "Error 1");
    handler_->reportError(ErrorCategory::Application, 0x1002, "Error 2");
    handler_->reportError(ErrorCategory::Application, 0x1003, "Error 3");
    
    EXPECT_EQ(handler_->getActiveErrorCount(), 3u);
}

// =============================================================================
// Error Clearing Tests
// =============================================================================

TEST_F(SlaveErrorBehaviorTest, ErrorsCanBeCleared) {
    handler_->reportError(ErrorCategory::Application, 0x1000, "Error");
    EXPECT_TRUE(handler_->hasActiveError());
    
    handler_->clearError(ErrorCategory::Application, 0x1000);
    EXPECT_FALSE(handler_->hasActiveError());
}

TEST_F(SlaveErrorBehaviorTest, AllErrorsCanBeCleared) {
    handler_->reportError(ErrorCategory::Application, 0x1001, "Error 1");
    handler_->reportError(ErrorCategory::Application, 0x1002, "Error 2");
    
    handler_->clearAllErrors();
    
    EXPECT_FALSE(handler_->hasActiveError());
    EXPECT_EQ(handler_->getActiveErrorCount(), 0u);
}

TEST_F(SlaveErrorBehaviorTest, FaultCanBeReset) {
    handler_->reportError(
        ErrorCategory::Communication,
        ErrorCodes::Communication::CONNECTION_LOST,
        "Fault condition"
    );
    
    EXPECT_TRUE(handler_->hasFault());
    
    handler_->resetFault();
    
    EXPECT_FALSE(handler_->hasFault());
}

// =============================================================================
// Recovery Tests
// =============================================================================

TEST_F(SlaveErrorBehaviorTest, RecoveryIsAttempted) {
    bool recoveryAttempted = false;
    
    handler_->setRecoveryCallback(ErrorCategory::Application, [&]() {
        recoveryAttempted = true;
        return true;
    });
    
    handler_->reportError(ErrorCategory::Application, 0x1000, "Error");
    handler_->attemptRecovery(ErrorCategory::Application, 0x1000);
    
    EXPECT_TRUE(recoveryAttempted);
}

TEST_F(SlaveErrorBehaviorTest, SuccessfulRecoveryClearsError) {
    handler_->setRecoveryCallback(ErrorCategory::Application, [&]() {
        return true; // Recovery successful
    });
    
    handler_->reportError(ErrorCategory::Application, 0x1000, "Error");
    bool recovered = handler_->attemptRecovery(ErrorCategory::Application, 0x1000);
    
    EXPECT_TRUE(recovered);
}

TEST_F(SlaveErrorBehaviorTest, FailedRecoveryKeepsError) {
    handler_->setRecoveryCallback(ErrorCategory::Application, [&]() {
        return false; // Recovery failed
    });
    
    handler_->reportError(ErrorCategory::Application, 0x1000, "Error");
    bool recovered = handler_->attemptRecovery(ErrorCategory::Application, 0x1000);
    
    EXPECT_FALSE(recovered);
}

TEST_F(SlaveErrorBehaviorTest, MaxRecoveryAttemptsEnforced) {
    int attempts = 0;
    handler_->setRecoveryCallback(ErrorCategory::Application, [&]() {
        attempts++;
        return false; // Always fail
    });
    
    handler_->reportError(ErrorCategory::Application, 0x1000, "Error");
    
    // Attempt recovery multiple times
    for (int i = 0; i < 10; i++) {
        handler_->attemptRecovery(ErrorCategory::Application, 0x1000);
    }
    
    // Should be limited by maxRecoveryAttempts (3)
    EXPECT_LE(attempts, 3);
}

// =============================================================================
// Error Injection Tests
// =============================================================================

TEST_F(SlaveErrorBehaviorTest, ErrorInjectionEnabled) {
    handler_->setErrorInjection(ErrorCategory::Communication, true, 0.5);
    
    // With 50% probability, should sometimes inject
    bool injected = false;
    for (int i = 0; i < 100; i++) {
        if (handler_->shouldInjectError(ErrorCategory::Communication)) {
            injected = true;
            break;
        }
    }
    
    EXPECT_TRUE(injected);
}

TEST_F(SlaveErrorBehaviorTest, ErrorInjectionDisabled) {
    handler_->setErrorInjection(ErrorCategory::Communication, false, 1.0);
    
    bool injected = handler_->shouldInjectError(ErrorCategory::Communication);
    
    EXPECT_FALSE(injected);
}

// =============================================================================
// Callback Tests
// =============================================================================

TEST_F(SlaveErrorBehaviorTest, ErrorCallbackInvoked) {
    ErrorCategory reportedCategory;
    uint32_t reportedCode = 0;
    
    handler_->setErrorCallback([&](ErrorCategory cat, uint32_t code, const std::string&) {
        reportedCategory = cat;
        reportedCode = code;
    });
    
    handler_->reportError(ErrorCategory::Hardware, 0x2000, "Hardware error");
    
    EXPECT_EQ(reportedCategory, ErrorCategory::Hardware);
    EXPECT_EQ(reportedCode, 0x2000u);
}

TEST_F(SlaveErrorBehaviorTest, FaultCallbackInvoked) {
    bool faultCallbackCalled = false;
    
    handler_->setFaultCallback([&]() {
        faultCallbackCalled = true;
    });
    
    handler_->reportError(ErrorCategory::Communication, 
                          ErrorCodes::Communication::CONNECTION_LOST,
                          "Critical error");
    
    EXPECT_TRUE(faultCallbackCalled);
}

// =============================================================================
// Statistics Tests
// =============================================================================

TEST_F(SlaveErrorBehaviorTest, StatisticsAreTracked) {
    handler_->reportError(ErrorCategory::Application, 0x1000, "Error 1");
    handler_->reportError(ErrorCategory::Application, 0x1001, "Error 2");
    
    auto stats = handler_->getStatistics();
    
    EXPECT_GE(stats.totalErrors, 2u);
}

TEST_F(SlaveErrorBehaviorTest, StatisticsCanBeReset) {
    handler_->reportError(ErrorCategory::Application, 0x1000, "Error");
    
    handler_->resetStatistics();
    
    auto stats = handler_->getStatistics();
    EXPECT_EQ(stats.totalErrors, 0u);
}

// =============================================================================
// Error Threshold Tests
// =============================================================================

TEST_F(SlaveErrorBehaviorTest, ErrorThresholdTriggersCritical) {
    ErrorBehaviorConfig config;
    config.errorOccurrenceThreshold = 3;
    config.faultOnCritical = true;
    
    ErrorBehaviorHandler handler(config);
    
    // Same error 3 times should escalate
    for (int i = 0; i < 3; i++) {
        handler.reportError(ErrorCategory::Application, 0x1000, "Repeated error");
    }
    
    // After threshold, non-critical becomes critical
}

// =============================================================================
// Category Tests
// =============================================================================

TEST_F(SlaveErrorBehaviorTest, IsCriticalCategory) {
    EXPECT_TRUE(handler_->isCriticalCategory(ErrorCategory::Communication));
    EXPECT_TRUE(handler_->isCriticalCategory(ErrorCategory::FSoE));
    EXPECT_TRUE(handler_->isCriticalCategory(ErrorCategory::Hardware));
    EXPECT_FALSE(handler_->isCriticalCategory(ErrorCategory::Application));
}

TEST_F(SlaveErrorBehaviorTest, CriticalCategoriesCanBeChanged) {
    ErrorBehaviorConfig config;
    config.criticalCategories = {ErrorCategory::Application};
    
    ErrorBehaviorHandler handler(config);
    
    EXPECT_TRUE(handler.isCriticalCategory(ErrorCategory::Application));
    EXPECT_FALSE(handler.isCriticalCategory(ErrorCategory::Communication));
}

} // namespace test
} // namespace slave
} // namespace EtherCAT
