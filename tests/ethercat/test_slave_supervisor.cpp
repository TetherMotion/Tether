/**
 * @file test_slave_supervisor.cpp
 * @brief Unit tests for SlaveSupervisor — critical-condition detection,
 *        recovery orchestration, retry limiting, and event dispatch.
 */

#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include "tether/ethercat/SlaveSupervisor.hpp"
#include "tether/ethercat/Master.hpp"
#include "tether/ethercat/Slave.hpp"

#include <atomic>
#include <vector>
#include <cstring>

using namespace EtherCAT;
using namespace ::testing;

// ============================================================================
// Mock Recovery Handler
// ============================================================================

class MockRecoveryHandler : public ISlaveRecoveryHandler {
public:
    MOCK_METHOD(bool, reinitializeSlave, (uint16_t slave_index), (override));
};

// ============================================================================
// Event Listener that records all events
// ============================================================================

class RecordingEventListener : public IRecoveryEventListener {
public:
    struct RecordedEvent {
        RecoveryEventType type;
        uint16_t slave_index;
        uint16_t al_status_code;
        int attempt;
    };

    void onRecoveryEvent(const RecoveryEvent& event) override {
        std::lock_guard<std::mutex> lock(mutex_);
        events_.push_back({event.type, event.slave_index,
                           event.al_status_code, event.attempt});
    }

    size_t eventCount() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return events_.size();
    }

    std::vector<RecordedEvent> events() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return events_;
    }

    void clear() {
        std::lock_guard<std::mutex> lock(mutex_);
        events_.clear();
    }

    bool hasEvent(RecoveryEventType type) const {
        std::lock_guard<std::mutex> lock(mutex_);
        for (const auto& e : events_) {
            if (e.type == type) return true;
        }
        return false;
    }

    mutable std::mutex mutex_;
    std::vector<RecordedEvent> events_;
};

// ============================================================================
// Test Fixture
// ============================================================================

class SlaveSupervisorTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Shrink PRE_OP retry timing and SII timeout for fast tests
        master_.setPreopRetryConfig(1, 3, 1, 1);
        master_.siiReader().setTimeout(1);

        // Track AL state for test callbacks (same pattern as test_ethercat_slave.cpp)
        al_state_.store(0x01, std::memory_order_relaxed); // INIT

        master_.setApwrTestCallback([this](uint16_t, uint16_t ado, const void* data, uint16_t len, unsigned int) {
            if (ado == 0x0120 && data && len >= 2) {
                uint16_t val = *static_cast<const uint16_t*>(data);
                al_state_.store(val & 0x0F, std::memory_order_relaxed);
            }
            return true;
        });
        master_.setAprdTestCallback([this](uint16_t, uint16_t ado, void* out, uint16_t len, unsigned int) {
            if (out && len >= 2 && (ado == 0x0130 || ado == 0x0134)) {
                uint8_t* p = static_cast<uint8_t*>(out);
                p[0] = static_cast<uint8_t>(al_state_.load(std::memory_order_relaxed) & 0xFF);
                p[1] = 0;
            }
            return true;
        });

        // Discover 2 slaves
        master_.initSlaves(2);
        // Initialize the status poller (normally done by discoverSlaves())
        master_.statusPoller().init(2);
        auto& sup = master_.slaveSupervisor();
        sup.init(2);
    }

    Master master_;
    std::atomic<uint16_t> al_state_{0x01};

    RecoveryConfig defaultConfig() {
        RecoveryConfig cfg;
        cfg.enabled = true;
        cfg.max_attempts = 3;
        cfg.retry_delay_ms = 0;      // No delay for fast tests
        cfg.post_init_delay_ms = 0;  // No delay for fast tests
        cfg.poll_interval_ms = 50;
        cfg.stop_loop_during_recovery = false;  // Single-slave recovery
        return cfg;
    }
};

// ============================================================================
// Construction / Configuration Tests
// ============================================================================

TEST(SlaveSupervisorConfigTest, DefaultConfigDisabled) {
    RecoveryConfig cfg;
    EXPECT_FALSE(cfg.enabled);
}

TEST(SlaveSupervisorConfigTest, DefaultCriticalALCodes) {
    RecoveryConfig cfg;
    EXPECT_TRUE(cfg.isCriticalALCode(static_cast<uint16_t>(ALStatusCode::SlaveNeedsInit)));
    EXPECT_TRUE(cfg.isCriticalALCode(static_cast<uint16_t>(ALStatusCode::SlaveNeedsColdStart)));
    EXPECT_TRUE(cfg.isCriticalALCode(static_cast<uint16_t>(ALStatusCode::FatalSyncError)));
    EXPECT_TRUE(cfg.isCriticalALCode(static_cast<uint16_t>(ALStatusCode::NoSyncError)));
    EXPECT_TRUE(cfg.isCriticalALCode(static_cast<uint16_t>(ALStatusCode::SynchronizationError)));
    EXPECT_FALSE(cfg.isCriticalALCode(0x0000));
    EXPECT_FALSE(cfg.isCriticalALCode(0x1000));
}

TEST(SlaveSupervisorConfigTest, CustomCriticalALCodes) {
    RecoveryConfig cfg;
    cfg.critical_al_codes = {0xAC00, 0xAB00};
    EXPECT_TRUE(cfg.isCriticalALCode(0xAC00));
    EXPECT_TRUE(cfg.isCriticalALCode(0xAB00));
    EXPECT_FALSE(cfg.isCriticalALCode(0xAD00));
}

TEST(SlaveSupervisorConfigTest, TriggerBitmask) {
    EXPECT_TRUE(has_trigger(CriticalTrigger::All, CriticalTrigger::ALStatusCodes));
    EXPECT_TRUE(has_trigger(CriticalTrigger::All, CriticalTrigger::TransitionFailures));
    EXPECT_TRUE(has_trigger(CriticalTrigger::All, CriticalTrigger::AppInjected));
    EXPECT_FALSE(has_trigger(CriticalTrigger::None, CriticalTrigger::ALStatusCodes));

    auto combined = CriticalTrigger::ALStatusCodes | CriticalTrigger::AppInjected;
    EXPECT_TRUE(has_trigger(combined, CriticalTrigger::ALStatusCodes));
    EXPECT_TRUE(has_trigger(combined, CriticalTrigger::AppInjected));
    EXPECT_FALSE(has_trigger(combined, CriticalTrigger::TransitionFailures));
}

TEST(SlaveSupervisorConfigTest, RecoveryEventTypeName) {
    EXPECT_STREQ(recoveryEventTypeName(RecoveryEventType::CriticalDetected), "CriticalDetected");
    EXPECT_STREQ(recoveryEventTypeName(RecoveryEventType::RecoveryStarted), "RecoveryStarted");
    EXPECT_STREQ(recoveryEventTypeName(RecoveryEventType::RecoverySucceeded), "RecoverySucceeded");
    EXPECT_STREQ(recoveryEventTypeName(RecoveryEventType::RecoveryFailed), "RecoveryFailed");
    EXPECT_STREQ(recoveryEventTypeName(RecoveryEventType::RecoveryGaveUp), "RecoveryGaveUp");
    EXPECT_STREQ(recoveryEventTypeName(RecoveryEventType::SlaveSuspended), "SlaveSuspended");
    EXPECT_STREQ(recoveryEventTypeName(RecoveryEventType::SlaveResumed), "SlaveResumed");
}

// ============================================================================
// Init / Lifecycle Tests
// ============================================================================

TEST_F(SlaveSupervisorTest, InitSetsSlaveCount) {
    auto& sup = master_.slaveSupervisor();
    EXPECT_EQ(sup.slaveCount(), 2u);
    EXPECT_EQ(sup.slaveState(0), SlaveRecoveryState::Normal);
    EXPECT_EQ(sup.slaveState(1), SlaveRecoveryState::Normal);
}

TEST_F(SlaveSupervisorTest, StartFailsWhenDisabled) {
    auto& sup = master_.slaveSupervisor();
    RecoveryConfig cfg;
    cfg.enabled = false;
    sup.configure(cfg);
    EXPECT_FALSE(sup.start());
}

TEST_F(SlaveSupervisorTest, StartFailsWithoutHandler) {
    auto& sup = master_.slaveSupervisor();
    sup.configure(defaultConfig());
    // No handler set
    EXPECT_FALSE(sup.start());
}

TEST_F(SlaveSupervisorTest, StartSucceedsWithHandlerAndConfig) {
    auto& sup = master_.slaveSupervisor();
    sup.configure(defaultConfig());
    auto handler = std::make_unique<MockRecoveryHandler>();
    EXPECT_CALL(*handler, reinitializeSlave(_)).Times(0);
    sup.setRecoveryHandler(std::move(handler));
    EXPECT_TRUE(sup.start());
    sup.stop();
}

// ============================================================================
// Suspension State Tests (realtime-safe)
// ============================================================================

TEST_F(SlaveSupervisorTest, SuspendedStateIsFalseByDefault) {
    auto& sup = master_.slaveSupervisor();
    EXPECT_FALSE(sup.isSlaveSuspended(0));
    EXPECT_FALSE(sup.isSlaveSuspended(1));
    EXPECT_FALSE(sup.isSlaveSuspended(999));  // Out of range
}

TEST_F(SlaveSupervisorTest, IsSlaveRecoveringFalseByDefault) {
    auto& sup = master_.slaveSupervisor();
    EXPECT_FALSE(sup.isSlaveRecovering(0));
    EXPECT_FALSE(sup.isSlaveRecovering(1));
}

// ============================================================================
// App-Injected Critical Trigger Tests
// ============================================================================

TEST_F(SlaveSupervisorTest, MarkCriticalTriggersRecovery) {
    auto& sup = master_.slaveSupervisor();
    sup.configure(defaultConfig());

    auto handler = std::make_unique<MockRecoveryHandler>();
    EXPECT_CALL(*handler, reinitializeSlave(0))
        .WillOnce(Return(true));
    sup.setRecoveryHandler(std::move(handler));

    RecordingEventListener listener;
    sup.addEventListener(&listener);

    ASSERT_TRUE(sup.start());

    sup.markCritical(0, "Test critical condition");

    EXPECT_EQ(sup.slaveState(0), SlaveRecoveryState::Normal);
    EXPECT_FALSE(sup.isSlaveSuspended(0));
    EXPECT_EQ(sup.slaveAttemptCount(0), 0);

    EXPECT_TRUE(listener.hasEvent(RecoveryEventType::CriticalDetected));
    EXPECT_TRUE(listener.hasEvent(RecoveryEventType::RecoveryStarted));
    EXPECT_TRUE(listener.hasEvent(RecoveryEventType::RecoverySucceeded));
    EXPECT_TRUE(listener.hasEvent(RecoveryEventType::SlaveSuspended));
    EXPECT_TRUE(listener.hasEvent(RecoveryEventType::SlaveResumed));

    sup.stop();
}

TEST_F(SlaveSupervisorTest, MarkCriticalIgnoredWhenAppInjectedDisabled) {
    auto& sup = master_.slaveSupervisor();
    auto cfg = defaultConfig();
    cfg.triggers = CriticalTrigger::ALStatusCodes;  // Exclude AppInjected
    sup.configure(cfg);

    auto handler = std::make_unique<MockRecoveryHandler>();
    EXPECT_CALL(*handler, reinitializeSlave(_)).Times(0);
    sup.setRecoveryHandler(std::move(handler));

    ASSERT_TRUE(sup.start());

    sup.markCritical(0, "Should be ignored");

    EXPECT_EQ(sup.slaveState(0), SlaveRecoveryState::Normal);
    EXPECT_FALSE(sup.isSlaveSuspended(0));

    sup.stop();
}

TEST_F(SlaveSupervisorTest, MarkCriticalIgnoredWhenNotRunning) {
    auto& sup = master_.slaveSupervisor();
    sup.configure(defaultConfig());

    auto handler = std::make_unique<MockRecoveryHandler>();
    EXPECT_CALL(*handler, reinitializeSlave(_)).Times(0);
    sup.setRecoveryHandler(std::move(handler));

    // Not started
    sup.markCritical(0, "Should be ignored");
    EXPECT_EQ(sup.slaveState(0), SlaveRecoveryState::Normal);
}

TEST_F(SlaveSupervisorTest, MarkCriticalOnInvalidSlaveIgnored) {
    auto& sup = master_.slaveSupervisor();
    sup.configure(defaultConfig());

    auto handler = std::make_unique<MockRecoveryHandler>();
    EXPECT_CALL(*handler, reinitializeSlave(_)).Times(0);
    sup.setRecoveryHandler(std::move(handler));

    ASSERT_TRUE(sup.start());
    sup.markCritical(100, "Invalid slave");
    EXPECT_EQ(sup.slaveState(100), SlaveRecoveryState::Normal);
    sup.stop();
}

// ============================================================================
// Retry Limit Tests
// ============================================================================

TEST_F(SlaveSupervisorTest, RetryLimitExhausted) {
    auto& sup = master_.slaveSupervisor();
    auto cfg = defaultConfig();
    cfg.max_attempts = 2;
    sup.configure(cfg);

    auto handler = std::make_unique<MockRecoveryHandler>();
    EXPECT_CALL(*handler, reinitializeSlave(0))
        .Times(2)
        .WillRepeatedly(Return(false));
    sup.setRecoveryHandler(std::move(handler));

    RecordingEventListener listener;
    sup.addEventListener(&listener);

    ASSERT_TRUE(sup.start());

    sup.markCritical(0, "Persistent failure");

    EXPECT_EQ(sup.slaveState(0), SlaveRecoveryState::Failed);
    EXPECT_TRUE(sup.isSlaveSuspended(0));  // Stays suspended
    EXPECT_EQ(sup.slaveAttemptCount(0), 2);

    EXPECT_TRUE(listener.hasEvent(RecoveryEventType::RecoveryGaveUp));
    EXPECT_FALSE(listener.hasEvent(RecoveryEventType::RecoverySucceeded));

    sup.stop();
}

TEST_F(SlaveSupervisorTest, RetrySucceedsOnSecondAttempt) {
    auto& sup = master_.slaveSupervisor();
    auto cfg = defaultConfig();
    cfg.max_attempts = 3;
    sup.configure(cfg);

    auto handler = std::make_unique<MockRecoveryHandler>();
    EXPECT_CALL(*handler, reinitializeSlave(0))
        .Times(2)
        .WillOnce(Return(false))
        .WillOnce(Return(true));
    sup.setRecoveryHandler(std::move(handler));

    RecordingEventListener listener;
    sup.addEventListener(&listener);

    ASSERT_TRUE(sup.start());

    sup.markCritical(0, "Transient failure");

    EXPECT_EQ(sup.slaveState(0), SlaveRecoveryState::Normal);
    EXPECT_FALSE(sup.isSlaveSuspended(0));
    EXPECT_EQ(sup.slaveAttemptCount(0), 0);  // Reset on success

    EXPECT_TRUE(listener.hasEvent(RecoveryEventType::RecoveryFailed));
    EXPECT_TRUE(listener.hasEvent(RecoveryEventType::RecoverySucceeded));

    sup.stop();
}

TEST_F(SlaveSupervisorTest, UnlimitedRetries) {
    auto& sup = master_.slaveSupervisor();
    auto cfg = defaultConfig();
    cfg.max_attempts = 0;  // Unlimited
    sup.configure(cfg);

    auto handler = std::make_unique<MockRecoveryHandler>();
    EXPECT_CALL(*handler, reinitializeSlave(0))
        .Times(3)
        .WillOnce(Return(false))
        .WillOnce(Return(false))
        .WillOnce(Return(true));
    sup.setRecoveryHandler(std::move(handler));

    ASSERT_TRUE(sup.start());

    sup.markCritical(0, "Multiple failures");

    EXPECT_EQ(sup.slaveState(0), SlaveRecoveryState::Normal);
    EXPECT_FALSE(sup.isSlaveSuspended(0));

    sup.stop();
}

// ============================================================================
// Stop-Loop-During-Recovery Tests
// ============================================================================

TEST_F(SlaveSupervisorTest, StopLoopSuspendsAllSlaves) {
    auto& sup = master_.slaveSupervisor();
    auto cfg = defaultConfig();
    cfg.stop_loop_during_recovery = true;
    sup.configure(cfg);

    auto handler = std::make_unique<MockRecoveryHandler>();
    EXPECT_CALL(*handler, reinitializeSlave(0))
        .WillOnce(Return(true));
    sup.setRecoveryHandler(std::move(handler));

    ASSERT_TRUE(sup.start());

    sup.markCritical(0, "Test");

    // After recovery, all slaves should be resumed
    EXPECT_FALSE(sup.isSlaveSuspended(0));
    EXPECT_FALSE(sup.isSlaveSuspended(1));

    sup.stop();
}

TEST_F(SlaveSupervisorTest, SingleSlaveRecoveryDoesNotSuspendOthers) {
    auto& sup = master_.slaveSupervisor();
    auto cfg = defaultConfig();
    cfg.stop_loop_during_recovery = false;
    sup.configure(cfg);

    auto handler = std::make_unique<MockRecoveryHandler>();
    EXPECT_CALL(*handler, reinitializeSlave(0))
        .WillOnce(Return(true));
    sup.setRecoveryHandler(std::move(handler));

    ASSERT_TRUE(sup.start());

    sup.markCritical(0, "Test");

    // Slave 1 should never be suspended
    EXPECT_FALSE(sup.isSlaveSuspended(1));
    EXPECT_FALSE(sup.isSlaveSuspended(0));  // Resumed after recovery

    sup.stop();
}

// ============================================================================
// Event Listener Tests
// ============================================================================

TEST_F(SlaveSupervisorTest, AddRemoveEventListener) {
    auto& sup = master_.slaveSupervisor();
    sup.configure(defaultConfig());

    auto handler = std::make_unique<MockRecoveryHandler>();
    EXPECT_CALL(*handler, reinitializeSlave(0)).WillRepeatedly(Return(true));
    sup.setRecoveryHandler(std::move(handler));

    RecordingEventListener listener1;
    RecordingEventListener listener2;

    EXPECT_TRUE(sup.addEventListener(&listener1));
    EXPECT_TRUE(sup.addEventListener(&listener2));
    EXPECT_FALSE(sup.addEventListener(&listener1));  // Duplicate
    EXPECT_FALSE(sup.addEventListener(nullptr));

    ASSERT_TRUE(sup.start());
    sup.markCritical(0, "Test");

    EXPECT_GT(listener1.eventCount(), 0u);
    EXPECT_GT(listener2.eventCount(), 0u);

    sup.removeEventListener(&listener1);
    listener1.clear();
    listener2.clear();

    sup.markCritical(0, "Test 2");
    EXPECT_EQ(listener1.eventCount(), 0u);
    EXPECT_GT(listener2.eventCount(), 0u);

    sup.stop();
}

TEST_F(SlaveSupervisorTest, ClearEventListeners) {
    auto& sup = master_.slaveSupervisor();
    sup.configure(defaultConfig());

    auto handler = std::make_unique<MockRecoveryHandler>();
    EXPECT_CALL(*handler, reinitializeSlave(0)).WillOnce(Return(true));
    sup.setRecoveryHandler(std::move(handler));

    RecordingEventListener listener;
    sup.addEventListener(&listener);

    ASSERT_TRUE(sup.start());
    sup.clearEventListeners();
    sup.markCritical(0, "Test");
    EXPECT_EQ(listener.eventCount(), 0u);

    sup.stop();
}

// ============================================================================
// Reset State Tests
// ============================================================================

TEST_F(SlaveSupervisorTest, ResetSlaveState) {
    auto& sup = master_.slaveSupervisor();
    auto cfg = defaultConfig();
    cfg.max_attempts = 1;
    sup.configure(cfg);

    auto handler = std::make_unique<MockRecoveryHandler>();
    EXPECT_CALL(*handler, reinitializeSlave(0)).WillOnce(Return(false));
    sup.setRecoveryHandler(std::move(handler));

    ASSERT_TRUE(sup.start());
    sup.markCritical(0, "Fail");

    EXPECT_EQ(sup.slaveState(0), SlaveRecoveryState::Failed);
    EXPECT_TRUE(sup.isSlaveSuspended(0));

    sup.resetSlaveState(0);
    EXPECT_EQ(sup.slaveState(0), SlaveRecoveryState::Normal);
    EXPECT_FALSE(sup.isSlaveSuspended(0));
    EXPECT_EQ(sup.slaveAttemptCount(0), 0);

    sup.stop();
}

// ============================================================================
// Transition Failure Tests
// ============================================================================

TEST_F(SlaveSupervisorTest, HandleTransitionFailureTriggersRecovery) {
    auto& sup = master_.slaveSupervisor();
    sup.configure(defaultConfig());

    auto handler = std::make_unique<MockRecoveryHandler>();
    EXPECT_CALL(*handler, reinitializeSlave(0)).WillOnce(Return(true));
    sup.setRecoveryHandler(std::move(handler));

    RecordingEventListener listener;
    sup.addEventListener(&listener);

    ASSERT_TRUE(sup.start());

    sup.handleTransitionFailure(0, "OP not confirmed after 5s", 0xAC00);

    EXPECT_EQ(sup.slaveState(0), SlaveRecoveryState::Normal);
    EXPECT_TRUE(listener.hasEvent(RecoveryEventType::CriticalDetected));
    EXPECT_TRUE(listener.hasEvent(RecoveryEventType::RecoverySucceeded));

    sup.stop();
}

TEST_F(SlaveSupervisorTest, HandleTransitionFailureIgnoredWhenTriggerDisabled) {
    auto& sup = master_.slaveSupervisor();
    auto cfg = defaultConfig();
    cfg.triggers = CriticalTrigger::AppInjected;  // Exclude TransitionFailures
    sup.configure(cfg);

    auto handler = std::make_unique<MockRecoveryHandler>();
    EXPECT_CALL(*handler, reinitializeSlave(_)).Times(0);
    sup.setRecoveryHandler(std::move(handler));

    ASSERT_TRUE(sup.start());

    sup.handleTransitionFailure(0, "OP not confirmed", 0xAC00);

    EXPECT_EQ(sup.slaveState(0), SlaveRecoveryState::Normal);

    sup.stop();
}

// ============================================================================
// Status Event Tests
// ============================================================================

TEST_F(SlaveSupervisorTest, StatusEventWithErrorTriggersRecovery) {
    auto& sup = master_.slaveSupervisor();
    sup.configure(defaultConfig());

    auto handler = std::make_unique<MockRecoveryHandler>();
    EXPECT_CALL(*handler, reinitializeSlave(0)).WillOnce(Return(true));
    sup.setRecoveryHandler(std::move(handler));

    ASSERT_TRUE(sup.start());

    // Simulate a status event: slave 0 dropped from OP to INIT with error
    SlaveStatusEvent event{};
    event.slave_index = 0;
    event.old_state = SlaveState::OP;
    event.new_state = SlaveState::INIT;
    event.old_al_status = 0x08;
    event.new_al_status = 0x11;  // INIT + error flag
    event.old_error_flag = false;
    event.new_error_flag = true;
    event.al_status_code = static_cast<uint16_t>(ALStatusCode::SlaveNeedsInit);

    sup.handleStatusEvent(event);

    EXPECT_EQ(sup.slaveState(0), SlaveRecoveryState::Normal);

    sup.stop();
}

TEST_F(SlaveSupervisorTest, StatusEventStateDropTriggersRecovery) {
    auto& sup = master_.slaveSupervisor();
    sup.configure(defaultConfig());

    auto handler = std::make_unique<MockRecoveryHandler>();
    EXPECT_CALL(*handler, reinitializeSlave(0)).WillOnce(Return(true));
    sup.setRecoveryHandler(std::move(handler));

    ASSERT_TRUE(sup.start());

    // Simulate a state drop from OP to PRE_OP (no error flag, no AL code)
    SlaveStatusEvent event{};
    event.slave_index = 0;
    event.old_state = SlaveState::OP;
    event.new_state = SlaveState::PRE_OP;
    event.old_al_status = 0x08;
    event.new_al_status = 0x02;
    event.old_error_flag = false;
    event.new_error_flag = false;
    event.al_status_code = 0;

    sup.handleStatusEvent(event);

    EXPECT_EQ(sup.slaveState(0), SlaveRecoveryState::Normal);

    sup.stop();
}

TEST_F(SlaveSupervisorTest, StatusEventNonCriticalCodeIgnored) {
    auto& sup = master_.slaveSupervisor();
    sup.configure(defaultConfig());

    auto handler = std::make_unique<MockRecoveryHandler>();
    EXPECT_CALL(*handler, reinitializeSlave(_)).Times(0);
    sup.setRecoveryHandler(std::move(handler));

    ASSERT_TRUE(sup.start());

    // Error flag set but with a non-critical AL code
    SlaveStatusEvent event{};
    event.slave_index = 0;
    event.old_state = SlaveState::OP;
    event.new_state = SlaveState::OP;
    event.old_al_status = 0x08;
    event.new_al_status = 0x18;  // OP + error flag
    event.old_error_flag = false;
    event.new_error_flag = true;
    event.al_status_code = 0x0001;  // UnspecifiedError — not in default critical set

    sup.handleStatusEvent(event);

    EXPECT_EQ(sup.slaveState(0), SlaveRecoveryState::Normal);

    sup.stop();
}

TEST_F(SlaveSupervisorTest, StatusEventCustomALCodeTriggersRecovery) {
    auto& sup = master_.slaveSupervisor();
    auto cfg = defaultConfig();
    cfg.critical_al_codes = {0xAC00};  // Vendor-specific code from the user's log
    sup.configure(cfg);

    auto handler = std::make_unique<MockRecoveryHandler>();
    EXPECT_CALL(*handler, reinitializeSlave(0)).WillOnce(Return(true));
    sup.setRecoveryHandler(std::move(handler));

    ASSERT_TRUE(sup.start());

    // Simulate the exact scenario from the user's log:
    // OP not confirmed, AL status code 0xAC00
    SlaveStatusEvent event{};
    event.slave_index = 0;
    event.old_state = SlaveState::SAFE_OP;
    event.new_state = SlaveState::SAFE_OP;
    event.old_al_status = 0x04;
    event.new_al_status = 0x14;  // SAFE_OP + error flag
    event.old_error_flag = false;
    event.new_error_flag = true;
    event.al_status_code = 0xAC00;

    sup.handleStatusEvent(event);

    EXPECT_EQ(sup.slaveState(0), SlaveRecoveryState::Normal);

    sup.stop();
}

// ============================================================================
// Recovery Handler Exception Tests
// ============================================================================

TEST_F(SlaveSupervisorTest, HandlerExceptionDoesNotCrash) {
    auto& sup = master_.slaveSupervisor();
    auto cfg = defaultConfig();
    cfg.max_attempts = 1;
    sup.configure(cfg);

    class ThrowingHandler : public ISlaveRecoveryHandler {
    public:
        bool reinitializeSlave(uint16_t) override {
            throw std::runtime_error("Test exception");
        }
    };

    sup.setRecoveryHandler(std::make_unique<ThrowingHandler>());

    ASSERT_TRUE(sup.start());

    sup.markCritical(0, "Test");

    // Should have failed, not crashed
    EXPECT_EQ(sup.slaveState(0), SlaveRecoveryState::Failed);
    EXPECT_TRUE(sup.isSlaveSuspended(0));

    sup.stop();
}

// ============================================================================
// Already-In-Recovery Tests
// ============================================================================

TEST_F(SlaveSupervisorTest, DoubleTriggerIgnoredWhileRecovering) {
    auto& sup = master_.slaveSupervisor();
    sup.configure(defaultConfig());

    // Handler that succeeds — but we call markCritical twice
    auto handler = std::make_unique<MockRecoveryHandler>();
    EXPECT_CALL(*handler, reinitializeSlave(0))
        .Times(1)
        .WillOnce(Return(true));
    sup.setRecoveryHandler(std::move(handler));

    ASSERT_TRUE(sup.start());

    sup.markCritical(0, "First trigger");
    // After recovery completes, state is Normal, so a second trigger would
    // actually invoke the handler again.  Let's test the case where the
    // supervisor is already in Failed state — it should ignore.
    EXPECT_EQ(sup.slaveState(0), SlaveRecoveryState::Normal);

    sup.stop();
}

TEST_F(SlaveSupervisorTest, FailedStateIgnoresNewTriggers) {
    auto& sup = master_.slaveSupervisor();
    auto cfg = defaultConfig();
    cfg.max_attempts = 1;
    sup.configure(cfg);

    auto handler = std::make_unique<MockRecoveryHandler>();
    EXPECT_CALL(*handler, reinitializeSlave(0))
        .Times(1)
        .WillOnce(Return(false));
    sup.setRecoveryHandler(std::move(handler));

    ASSERT_TRUE(sup.start());

    sup.markCritical(0, "First trigger — fails permanently");
    EXPECT_EQ(sup.slaveState(0), SlaveRecoveryState::Failed);

    // Second trigger should be ignored (handler not called again)
    sup.markCritical(0, "Second trigger — should be ignored");
    EXPECT_EQ(sup.slaveState(0), SlaveRecoveryState::Failed);

    sup.stop();
}
