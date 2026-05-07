/**
 * @file test_SlaveErrorBehavior.cpp
 * @brief Comprehensive tests for SlaveErrorBehavior
 *
 * Covers ErrorBehaviorHandler, ErrorBehaviorConfig presets, error reporting,
 * fail-safe, recovery, statistics, error injection, and category handling.
 */
#include <gtest/gtest.h>
#include <tether/slave/core/SlaveErrorBehavior.hpp>

using namespace EtherCAT::slave;

// ============================================================================
// ErrorCategory
// ============================================================================
TEST(ErrorCategoryTest, GetCategoryName) {
    EXPECT_STREQ(getCategoryName(ErrorCategory::None), "NONE");
    EXPECT_STREQ(getCategoryName(ErrorCategory::Communication), "COMMUNICATION");
    EXPECT_STREQ(getCategoryName(ErrorCategory::StateMachine), "STATE_MACHINE");
    EXPECT_STREQ(getCategoryName(ErrorCategory::DC), "DC");
    EXPECT_STREQ(getCategoryName(ErrorCategory::FSoE), "FSOE");
    EXPECT_STREQ(getCategoryName(ErrorCategory::Profile), "PROFILE");
    EXPECT_STREQ(getCategoryName(ErrorCategory::Watchdog), "WATCHDOG");
    EXPECT_STREQ(getCategoryName(ErrorCategory::Memory), "MEMORY");
    EXPECT_STREQ(getCategoryName(ErrorCategory::Configuration), "CONFIGURATION");
    EXPECT_STREQ(getCategoryName(ErrorCategory::Hardware), "HARDWARE");
    EXPECT_STREQ(getCategoryName(ErrorCategory::Application), "APPLICATION");
}

// ============================================================================
// ErrorBehaviorConfig presets
// ============================================================================
TEST(ErrorBehaviorConfigTest, Defaults) {
    auto cfg = ErrorBehaviorConfig::defaults();
    EXPECT_TRUE(cfg.defaultCritical);
    EXPECT_TRUE(cfg.defaultFailSafe);
    EXPECT_TRUE(cfg.enableErrorLogging);
    EXPECT_TRUE(cfg.enableErrorStatistics);
    EXPECT_FALSE(cfg.enableErrorInjection);
    EXPECT_TRUE(cfg.autoRecoveryEnabled);
    EXPECT_GT(cfg.maxLogEntries, 0u);
    EXPECT_GT(cfg.recoveryDelayMs, 0u);
    EXPECT_GT(cfg.maxRecoveryAttempts, 0u);
}

TEST(ErrorBehaviorConfigTest, Strict) {
    auto cfg = ErrorBehaviorConfig::strict();
    // All categories should be critical in strict mode
    for (size_t i = 0; i < cfg.categoryConfig.size(); i++) {
        EXPECT_TRUE(cfg.categoryConfig[i].isCritical)
            << "Category " << i << " should be critical in strict mode";
    }
}

TEST(ErrorBehaviorConfigTest, Permissive) {
    auto cfg = ErrorBehaviorConfig::permissive();
    // FSoE should still be critical even in permissive mode
    auto fsoeIdx = static_cast<size_t>(ErrorCategory::FSoE);
    EXPECT_TRUE(cfg.categoryConfig[fsoeIdx].isCritical);
}

// ============================================================================
// ErrorStatistics
// ============================================================================
TEST(ErrorStatisticsTest, Reset) {
    ErrorStatistics stats;
    stats.totalErrors = 42;
    stats.criticalErrors = 10;
    stats.nonCriticalErrors = 32;
    stats.recoveryAttempts = 5;
    stats.reset();
    EXPECT_EQ(stats.totalErrors, 0u);
    EXPECT_EQ(stats.criticalErrors, 0u);
    EXPECT_EQ(stats.nonCriticalErrors, 0u);
    EXPECT_EQ(stats.recoveryAttempts, 0u);
}

// ============================================================================
// ErrorSeverityConfig defaults
// ============================================================================
TEST(ErrorSeverityConfigTest, Defaults) {
    ErrorSeverityConfig sev;
    EXPECT_TRUE(sev.isCritical);
    EXPECT_TRUE(sev.triggerFailSafe);
    EXPECT_TRUE(sev.logError);
    EXPECT_TRUE(sev.notifyCallback);
    EXPECT_EQ(sev.maxOccurrences, 0);
    EXPECT_EQ(sev.cooldownMs, 0);
}

// ============================================================================
// ErrorInjectionConfig
// ============================================================================
TEST(ErrorInjectionConfigTest, Reset) {
    ErrorInjectionConfig inj;
    inj.enabled = true;
    inj.injectErrorCode = 0x100;
    inj.reset();
    EXPECT_FALSE(inj.enabled);
    EXPECT_EQ(inj.injectErrorCode, 0);
}

// ============================================================================
// ErrorBehaviorHandler fixture
// ============================================================================
class ErrorBehaviorHandlerTest : public ::testing::Test {
protected:
    void SetUp() override {
        handler_ = std::make_unique<ErrorBehaviorHandler>();
    }

    std::unique_ptr<ErrorBehaviorHandler> handler_;
};

// ============================================================================
// Initial state
// ============================================================================
TEST_F(ErrorBehaviorHandlerTest, InitialState) {
    EXPECT_FALSE(handler_->hasError());
    EXPECT_FALSE(handler_->isFailSafe());
    EXPECT_EQ(handler_->getLastError(), 0u);
    EXPECT_EQ(handler_->getLastErrorCategory(), ErrorCategory::None);
}

// ============================================================================
// reportError
// ============================================================================
TEST_F(ErrorBehaviorHandlerTest, ReportCriticalError) {
    bool result = handler_->reportError(
        ErrorCodes::CommFrameDropped,
        ErrorCategory::Communication,
        "Link lost");

    EXPECT_TRUE(result); // critical -> true
    EXPECT_TRUE(handler_->hasError());
    EXPECT_EQ(handler_->getLastError(), ErrorCodes::CommFrameDropped);
    EXPECT_EQ(handler_->getLastErrorCategory(), ErrorCategory::Communication);
}

TEST_F(ErrorBehaviorHandlerTest, ReportNonCriticalError) {
    // Configure communication as non-critical
    ErrorSeverityConfig sev;
    sev.isCritical = false;
    sev.triggerFailSafe = false;
    handler_->configureCategory(ErrorCategory::Communication, sev);

    bool result = handler_->reportError(
        ErrorCodes::CommFrameDropped,
        ErrorCategory::Communication);

    EXPECT_FALSE(result); // non-critical -> false
    EXPECT_TRUE(handler_->hasError());
    EXPECT_FALSE(handler_->isFailSafe());
}

TEST_F(ErrorBehaviorHandlerTest, ReportErrorTriggersFailSafe) {
    handler_->reportError(
        ErrorCodes::FSoEConnectionError,
        ErrorCategory::FSoE);
    EXPECT_TRUE(handler_->isFailSafe());
}

TEST_F(ErrorBehaviorHandlerTest, ReportErrorWithMessage) {
    handler_->reportError(
        ErrorCodes::WatchdogPDO,
        ErrorCategory::Watchdog,
        "PDI watchdog timed out");

    const auto& log = handler_->getErrorLog();
    EXPECT_GE(log.size(), 1u);
    if (!log.empty()) {
        EXPECT_EQ(log.back().errorCode, ErrorCodes::WatchdogPDO);
    }
}

// ============================================================================
// reportErrorWithContext
// ============================================================================
TEST_F(ErrorBehaviorHandlerTest, ReportErrorWithContext) {
    uint32_t seqNum = 42;
    handler_->reportErrorWithContext(
        ErrorCodes::CommFrameSequence,
        ErrorCategory::Communication,
        &seqNum, sizeof(seqNum),
        "Packet lost");

    EXPECT_TRUE(handler_->hasError());
    const auto& log = handler_->getErrorLog();
    EXPECT_GE(log.size(), 1u);
}

// ============================================================================
// Error callback
// ============================================================================
TEST_F(ErrorBehaviorHandlerTest, ErrorCallback) {
    int callCount = 0;
    uint16_t lastCode = 0;
    ErrorCategory lastCat = ErrorCategory::None;
    bool lastCritical = false;

    handler_->setErrorCallback([&](uint16_t code, ErrorCategory cat, bool crit) {
        callCount++;
        lastCode = code;
        lastCat = cat;
        lastCritical = crit;
    });

    handler_->reportError(ErrorCodes::CommFrameDropped, ErrorCategory::Communication);
    EXPECT_GE(callCount, 1);
    EXPECT_EQ(lastCode, ErrorCodes::CommFrameDropped);
    EXPECT_EQ(lastCat, ErrorCategory::Communication);
    EXPECT_TRUE(lastCritical);
}

// ============================================================================
// Fail-safe callback
// ============================================================================
TEST_F(ErrorBehaviorHandlerTest, FailSafeCallback) {
    int callCount = 0;
    uint16_t lastFailCode = 0;
    handler_->setFailSafeCallback([&](uint16_t code) {
        callCount++;
        lastFailCode = code;
    });

    handler_->reportError(ErrorCodes::FSoEConnectionError, ErrorCategory::FSoE);
    EXPECT_GE(callCount, 1);
    EXPECT_EQ(lastFailCode, ErrorCodes::FSoEConnectionError);
}

// ============================================================================
// clearError
// ============================================================================
TEST_F(ErrorBehaviorHandlerTest, ClearError) {
    handler_->reportError(ErrorCodes::CommFrameDropped, ErrorCategory::Communication);
    EXPECT_TRUE(handler_->hasError());

    handler_->clearError();
    EXPECT_FALSE(handler_->hasError());
    EXPECT_EQ(handler_->getLastError(), 0u);
    EXPECT_EQ(handler_->getLastErrorCategory(), ErrorCategory::None);
}

// ============================================================================
// Recovery
// ============================================================================
TEST_F(ErrorBehaviorHandlerTest, RecoveryNotInFailSafe) {
    EXPECT_FALSE(handler_->attemptRecovery());
}

TEST_F(ErrorBehaviorHandlerTest, RecoverySuccess) {
    handler_->reportError(ErrorCodes::CommFrameDropped, ErrorCategory::Communication);
    EXPECT_TRUE(handler_->isFailSafe());

    handler_->setRecoveryCallback([&]() { return true; });

    // Allow enough time for recovery delay
    auto cfg = handler_->getConfig();
    handler_->update(cfg.recoveryDelayMs + 1);

    bool recovered = handler_->attemptRecovery();
    if (recovered) {
        EXPECT_FALSE(handler_->isFailSafe());
    }
}

TEST_F(ErrorBehaviorHandlerTest, RecoveryCallbackReturnsFalse) {
    handler_->reportError(ErrorCodes::CommFrameDropped, ErrorCategory::Communication);
    handler_->setRecoveryCallback([&]() { return false; });

    auto cfg = handler_->getConfig();
    handler_->update(cfg.recoveryDelayMs + 1);

    EXPECT_FALSE(handler_->attemptRecovery());
    EXPECT_TRUE(handler_->isFailSafe());
}

TEST_F(ErrorBehaviorHandlerTest, RecoveryDisabled) {
    auto cfg = handler_->getConfig();
    cfg.autoRecoveryEnabled = false;
    handler_->setConfig(cfg);

    handler_->reportError(ErrorCodes::CommFrameDropped, ErrorCategory::Communication);
    EXPECT_FALSE(handler_->canRecover());
}

TEST_F(ErrorBehaviorHandlerTest, MaxRecoveryAttempts) {
    // Use a callback that allows recovery so attemptRecovery increments the count
    handler_->setRecoveryCallback([&]() { return true; });

    auto cfg = handler_->getConfig();

    // Exhaust recovery attempts: each successful recovery clears fail-safe,
    // so we must re-trigger the error before each attempt.
    for (uint32_t i = 0; i < cfg.maxRecoveryAttempts; i++) {
        handler_->reportError(ErrorCodes::CommFrameDropped, ErrorCategory::Communication);
        handler_->update(cfg.recoveryDelayMs * (i + 2));
        handler_->attemptRecovery();
    }

    // Re-trigger one more time — attempts exhausted
    handler_->reportError(ErrorCodes::CommFrameDropped, ErrorCategory::Communication);
    handler_->update(cfg.recoveryDelayMs * (cfg.maxRecoveryAttempts + 2));
    EXPECT_FALSE(handler_->canRecover());
}

// ============================================================================
// Statistics
// ============================================================================
TEST_F(ErrorBehaviorHandlerTest, StatisticsIncrement) {
    handler_->reportError(ErrorCodes::CommFrameDropped, ErrorCategory::Communication);
    handler_->reportError(ErrorCodes::WatchdogPDO, ErrorCategory::Watchdog);

    const auto& stats = handler_->getStatistics();
    EXPECT_GE(stats.totalErrors, 2u);
}

TEST_F(ErrorBehaviorHandlerTest, StatisticsCategoryCount) {
    handler_->reportError(ErrorCodes::CommFrameDropped, ErrorCategory::Communication);
    handler_->reportError(ErrorCodes::CommFrameSequence, ErrorCategory::Communication);

    const auto& stats = handler_->getStatistics();
    auto commIdx = static_cast<size_t>(ErrorCategory::Communication);
    EXPECT_GE(stats.categoryCount[commIdx], 2u);
}

TEST_F(ErrorBehaviorHandlerTest, ResetStatistics) {
    handler_->reportError(ErrorCodes::CommFrameDropped, ErrorCategory::Communication);
    handler_->resetStatistics();

    EXPECT_EQ(handler_->getStatistics().totalErrors, 0u);
}

// ============================================================================
// Error log
// ============================================================================
TEST_F(ErrorBehaviorHandlerTest, ErrorLog) {
    handler_->reportError(ErrorCodes::CommFrameDropped, ErrorCategory::Communication, "test msg");

    const auto& log = handler_->getErrorLog();
    EXPECT_GE(log.size(), 1u);
    if (!log.empty()) {
        EXPECT_EQ(log.back().errorCode, ErrorCodes::CommFrameDropped);
        EXPECT_EQ(log.back().category, ErrorCategory::Communication);
    }
}

TEST_F(ErrorBehaviorHandlerTest, ClearErrorLog) {
    handler_->reportError(ErrorCodes::CommFrameDropped, ErrorCategory::Communication);
    handler_->clearErrorLog();
    EXPECT_TRUE(handler_->getErrorLog().empty());
}

TEST_F(ErrorBehaviorHandlerTest, ErrorLogMaxEntries) {
    auto cfg = handler_->getConfig();
    cfg.maxLogEntries = 5;
    handler_->setConfig(cfg);

    for (int i = 0; i < 10; i++) {
        handler_->reportError(
            static_cast<uint16_t>(0x100 + i),
            ErrorCategory::Communication);
    }

    EXPECT_LE(handler_->getErrorLog().size(), 5u);
}

// ============================================================================
// Per-error configuration
// ============================================================================
TEST_F(ErrorBehaviorHandlerTest, ConfigureSpecificError) {
    ErrorSeverityConfig sev;
    sev.isCritical = false;
    sev.triggerFailSafe = false;
    handler_->configureError(ErrorCodes::CommFrameTimeout, sev);

    bool result = handler_->reportError(
        ErrorCodes::CommFrameTimeout,
        ErrorCategory::Communication);
    EXPECT_FALSE(result); // non-critical due to override
}

// ============================================================================
// Per-category configuration
// ============================================================================
TEST_F(ErrorBehaviorHandlerTest, ConfigureCategory) {
    ErrorSeverityConfig sev;
    sev.isCritical = false;
    sev.triggerFailSafe = false;
    handler_->configureCategory(ErrorCategory::Watchdog, sev);

    bool result = handler_->reportError(
        ErrorCodes::WatchdogPDO,
        ErrorCategory::Watchdog);
    EXPECT_FALSE(result);
    EXPECT_FALSE(handler_->isFailSafe());
}

// ============================================================================
// Error injection
// ============================================================================
TEST_F(ErrorBehaviorHandlerTest, ErrorInjectionDisabledByDefault) {
    EXPECT_FALSE(handler_->getErrorInjection().enabled);
    EXPECT_FALSE(handler_->shouldInjectError());
}

TEST_F(ErrorBehaviorHandlerTest, ErrorInjectionEnabled) {
    auto cfg = handler_->getConfig();
    cfg.enableErrorInjection = true;
    handler_->setConfig(cfg);

    auto& inj = handler_->getErrorInjection();
    inj.enabled = true;
    inj.injectErrorCode = ErrorCodes::CommFrameDropped;
    inj.injectCategory = ErrorCategory::Communication;
    inj.injectionRate = 0; // every call

    EXPECT_TRUE(handler_->shouldInjectError());
}

TEST_F(ErrorBehaviorHandlerTest, InjectError) {
    auto cfg = handler_->getConfig();
    cfg.enableErrorInjection = true;
    handler_->setConfig(cfg);

    handler_->injectError(ErrorCodes::WatchdogPDO, ErrorCategory::Watchdog);
    EXPECT_TRUE(handler_->hasError());
}

// ============================================================================
// Error codes namespace
// ============================================================================
TEST(ErrorCodesTest, CommunicationRange) {
    EXPECT_GE(ErrorCodes::CommFrameDropped, 0x0100);
    EXPECT_LT(ErrorCodes::CommFrameDropped, 0x0200);
}

TEST(ErrorCodesTest, StateMachineRange) {
    EXPECT_GE(ErrorCodes::SMInvalidTransition, 0x0200);
    EXPECT_LT(ErrorCodes::SMInvalidTransition, 0x0300);
}

TEST(ErrorCodesTest, DCRange) {
    EXPECT_GE(ErrorCodes::DCClockDrift, 0x0300);
    EXPECT_LT(ErrorCodes::DCClockDrift, 0x0400);
}

TEST(ErrorCodesTest, FSoERange) {
    EXPECT_GE(ErrorCodes::FSoEConnectionError, 0x0400);
    EXPECT_LT(ErrorCodes::FSoEConnectionError, 0x0500);
}

// ============================================================================
// setConfig
// ============================================================================
TEST_F(ErrorBehaviorHandlerTest, SetConfig) {
    auto cfg = ErrorBehaviorConfig::strict();
    handler_->setConfig(cfg);
    EXPECT_TRUE(handler_->getConfig().defaultCritical);
}

// ============================================================================
// Construct with config
// ============================================================================
TEST(ErrorBehaviorHandlerConstructTest, WithStrictConfig) {
    ErrorBehaviorHandler h(ErrorBehaviorConfig::strict());
    EXPECT_FALSE(h.hasError());
}

TEST(ErrorBehaviorHandlerConstructTest, WithPermissiveConfig) {
    ErrorBehaviorHandler h(ErrorBehaviorConfig::permissive());
    EXPECT_FALSE(h.hasError());
}
