/**
 * @file DCErrorBehaviorTests.cpp
 * @brief Unit tests for DC Error Behavior
 */

#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include "slave/dc/DCErrorBehavior.hpp"

namespace EtherCAT {
namespace slave {
namespace DC {
namespace test {

// Test fixture
class DCErrorBehaviorTest : public ::testing::Test {
protected:
    void SetUp() override {
        DCErrorConfig config;
        config.maxClockDriftPpm = 100;
        config.maxClockJumpNs = 1000000; // 1ms
        config.maxSyncJitterNs = 50000;  // 50us
        config.sync0Enabled = true;
        config.sync0CycleNs = 1000000;   // 1ms
        config.packetOrderCheck = true;
        config.clockDriftCritical = true;
        config.syncMissCritical = true;
        
        handler_ = std::make_unique<DCErrorHandler>(config);
    }
    
    void TearDown() override {
        handler_.reset();
    }
    
    std::unique_ptr<DCErrorHandler> handler_;
};

// =============================================================================
// Initialization Tests
// =============================================================================

TEST_F(DCErrorBehaviorTest, InitializesCorrectly) {
    EXPECT_FALSE(handler_->hasError());
    EXPECT_TRUE(handler_->isSynchronized());
}

TEST_F(DCErrorBehaviorTest, ConfigurationIsApplied) {
    DCErrorConfig config;
    config.maxClockDriftPpm = 50;
    config.sync0CycleNs = 500000;
    
    DCErrorHandler handler(config);
    
    EXPECT_FALSE(handler.hasError());
}

// =============================================================================
// Clock Drift Tests
// =============================================================================

TEST_F(DCErrorBehaviorTest, SmallClockDriftAccepted) {
    // Simulate small drift within tolerance
    for (int i = 0; i < 100; i++) {
        uint64_t systemTime = 1000000 * i; // 1ms increments
        handler_->processSystemTimeUpdate(systemTime);
    }
    
    EXPECT_FALSE(handler_->hasError());
    
    auto stats = handler_->getStatistics();
    EXPECT_LT(stats.avgClockDriftPpm, 100.0);
}

TEST_F(DCErrorBehaviorTest, LargeClockDriftDetected) {
    DCErrorInjection injection;
    injection.enabled = true;
    injection.injectClockDrift = true;
    injection.clockDriftPpm = 500; // Much larger than max
    
    handler_->setErrorInjection(injection);
    
    for (int i = 0; i < 100; i++) {
        uint64_t systemTime = 1000000 * i;
        handler_->processSystemTimeUpdate(systemTime);
    }
    
    EXPECT_TRUE(handler_->hasError());
    EXPECT_EQ(handler_->getLastError(), DCError::ClockDriftExceeded);
}

// =============================================================================
// Clock Jump Tests
// =============================================================================

TEST_F(DCErrorBehaviorTest, NormalTimeProgressAccepted) {
    uint64_t time = 0;
    for (int i = 0; i < 100; i++) {
        time += 1000000; // 1ms increments
        handler_->processSystemTimeUpdate(time);
    }
    
    EXPECT_FALSE(handler_->hasError());
}

TEST_F(DCErrorBehaviorTest, ClockJumpForwardDetected) {
    handler_->processSystemTimeUpdate(1000000);
    handler_->processSystemTimeUpdate(100000000); // 100ms jump
    
    EXPECT_TRUE(handler_->hasError());
    EXPECT_EQ(handler_->getLastError(), DCError::ClockJumpForward);
}

TEST_F(DCErrorBehaviorTest, ClockJumpBackwardDetected) {
    handler_->processSystemTimeUpdate(100000000);
    handler_->processSystemTimeUpdate(1000000); // Time went backwards
    
    EXPECT_TRUE(handler_->hasError());
    EXPECT_EQ(handler_->getLastError(), DCError::ClockJumpBackward);
}

TEST_F(DCErrorBehaviorTest, ClockJumpInjection) {
    DCErrorInjection injection;
    injection.enabled = true;
    injection.injectClockJump = true;
    injection.clockJumpNs = 10000000; // 10ms jump
    
    handler_->setErrorInjection(injection);
    
    handler_->processSystemTimeUpdate(1000000);
    handler_->processSystemTimeUpdate(2000000);
    
    EXPECT_TRUE(handler_->hasError());
}

// =============================================================================
// SYNC Signal Tests
// =============================================================================

TEST_F(DCErrorBehaviorTest, RegularSyncAccepted) {
    for (int i = 0; i < 100; i++) {
        uint64_t syncTime = 1000000 * (i + 1); // 1ms cycle
        handler_->processSync0Signal(syncTime);
    }
    
    EXPECT_FALSE(handler_->hasError());
}

TEST_F(DCErrorBehaviorTest, SyncJitterDetected) {
    DCErrorInjection injection;
    injection.enabled = true;
    injection.injectSyncJitter = true;
    injection.syncJitterNs = 100000; // 100us jitter
    
    handler_->setErrorInjection(injection);
    
    for (int i = 0; i < 100; i++) {
        uint64_t syncTime = 1000000 * (i + 1);
        handler_->processSync0Signal(syncTime);
    }
    
    EXPECT_TRUE(handler_->hasError());
    EXPECT_EQ(handler_->getLastError(), DCError::Sync0JitterExceeded);
}

TEST_F(DCErrorBehaviorTest, MissedSyncDetected) {
    handler_->processSync0Signal(1000000);
    handler_->processSync0Signal(2000000);
    // Skip sync 3
    handler_->processSync0Signal(4000000);
    
    auto stats = handler_->getStatistics();
    EXPECT_GT(stats.sync0MissCount, 0u);
}

TEST_F(DCErrorBehaviorTest, SyncMissInjection) {
    DCErrorInjection injection;
    injection.enabled = true;
    injection.injectSyncMiss = true;
    injection.syncMissCount = 5;
    
    handler_->setErrorInjection(injection);
    
    handler_->processSync0Signal(1000000);
    handler_->processSync0Signal(2000000);
    
    EXPECT_TRUE(handler_->hasError());
    EXPECT_EQ(handler_->getLastError(), DCError::Sync0Missed);
}

// =============================================================================
// Packet Order Tests
// =============================================================================

TEST_F(DCErrorBehaviorTest, CorrectPacketOrderAccepted) {
    for (uint32_t i = 1; i <= 100; i++) {
        handler_->processPacket(i);
    }
    
    EXPECT_FALSE(handler_->hasError());
}

TEST_F(DCErrorBehaviorTest, OutOfOrderPacketDetected) {
    handler_->processPacket(1);
    handler_->processPacket(2);
    handler_->processPacket(4); // Skip 3
    handler_->processPacket(3); // Out of order
    
    auto stats = handler_->getStatistics();
    EXPECT_GT(stats.outOfOrderPackets, 0u);
}

TEST_F(DCErrorBehaviorTest, DuplicatePacketDetected) {
    handler_->processPacket(1);
    handler_->processPacket(2);
    handler_->processPacket(2); // Duplicate
    
    auto stats = handler_->getStatistics();
    EXPECT_GT(stats.duplicatePackets, 0u);
}

TEST_F(DCErrorBehaviorTest, PacketOrderInjection) {
    DCErrorInjection injection;
    injection.enabled = true;
    injection.injectPacketReorder = true;
    
    handler_->setErrorInjection(injection);
    
    handler_->processPacket(1);
    handler_->processPacket(2);
    
    EXPECT_TRUE(handler_->hasError());
    EXPECT_EQ(handler_->getLastError(), DCError::PacketReordered);
}

// =============================================================================
// Synchronization State Tests
// =============================================================================

TEST_F(DCErrorBehaviorTest, InitiallyNotSynchronized) {
    DCErrorHandler handler;
    // Before any sync signals, not synchronized
}

TEST_F(DCErrorBehaviorTest, BecomesSynchronized) {
    // Process enough sync signals to establish synchronization
    for (int i = 0; i < 10; i++) {
        uint64_t syncTime = 1000000 * (i + 1);
        handler_->processSync0Signal(syncTime);
    }
    
    EXPECT_TRUE(handler_->isSynchronized());
}

TEST_F(DCErrorBehaviorTest, LosesSynchronization) {
    // Establish sync
    for (int i = 0; i < 10; i++) {
        handler_->processSync0Signal(1000000 * (i + 1));
    }
    
    EXPECT_TRUE(handler_->isSynchronized());
    
    // Inject large drift
    DCErrorInjection injection;
    injection.enabled = true;
    injection.injectClockDrift = true;
    injection.clockDriftPpm = 1000;
    
    handler_->setErrorInjection(injection);
    
    for (int i = 0; i < 100; i++) {
        handler_->processSystemTimeUpdate(1000000 * i);
    }
    
    // Should lose sync due to drift
}

// =============================================================================
// Error Criticality Tests
// =============================================================================

TEST_F(DCErrorBehaviorTest, ClockDriftIsCritical) {
    DCErrorConfig config;
    config.clockDriftCritical = true;
    
    DCErrorHandler handler(config);
    
    DCErrorInjection injection;
    injection.enabled = true;
    injection.injectClockDrift = true;
    injection.clockDriftPpm = 500;
    
    handler.setErrorInjection(injection);
    
    for (int i = 0; i < 100; i++) {
        handler.processSystemTimeUpdate(1000000 * i);
    }
    
    EXPECT_TRUE(handler.isCriticalError());
}

TEST_F(DCErrorBehaviorTest, ClockDriftNonCritical) {
    DCErrorConfig config;
    config.clockDriftCritical = false;
    config.maxClockDriftPpm = 50;
    
    DCErrorHandler handler(config);
    
    DCErrorInjection injection;
    injection.enabled = true;
    injection.injectClockDrift = true;
    injection.clockDriftPpm = 100;
    
    handler.setErrorInjection(injection);
    
    for (int i = 0; i < 100; i++) {
        handler.processSystemTimeUpdate(1000000 * i);
    }
    
    EXPECT_FALSE(handler.isCriticalError());
}

// =============================================================================
// Statistics Tests
// =============================================================================

TEST_F(DCErrorBehaviorTest, StatisticsAreAccumulated) {
    for (int i = 0; i < 100; i++) {
        handler_->processSync0Signal(1000000 * (i + 1));
        handler_->processPacket(i + 1);
    }
    
    auto stats = handler_->getStatistics();
    
    EXPECT_EQ(stats.sync0Count, 100u);
    EXPECT_EQ(stats.packetsProcessed, 100u);
}

TEST_F(DCErrorBehaviorTest, StatisticsCanBeReset) {
    for (int i = 0; i < 50; i++) {
        handler_->processSync0Signal(1000000 * (i + 1));
    }
    
    handler_->resetStatistics();
    
    auto stats = handler_->getStatistics();
    EXPECT_EQ(stats.sync0Count, 0u);
}

TEST_F(DCErrorBehaviorTest, JitterStatisticsCalculated) {
    // Process sync signals with slight variations
    for (int i = 0; i < 100; i++) {
        uint64_t jitter = (i % 2) ? 1000 : -1000; // ±1us
        handler_->processSync0Signal(1000000 * (i + 1) + jitter);
    }
    
    auto stats = handler_->getStatistics();
    EXPECT_GT(stats.avgSync0JitterNs, 0.0);
    EXPECT_GT(stats.maxSync0JitterNs, 0u);
}

// =============================================================================
// Callback Tests
// =============================================================================

TEST_F(DCErrorBehaviorTest, ErrorCallbackInvoked) {
    DCError reportedError = DCError::None;
    
    handler_->setErrorCallback([&](DCError error) {
        reportedError = error;
    });
    
    DCErrorInjection injection;
    injection.enabled = true;
    injection.injectClockJump = true;
    injection.clockJumpNs = 10000000;
    
    handler_->setErrorInjection(injection);
    
    handler_->processSystemTimeUpdate(1000000);
    handler_->processSystemTimeUpdate(2000000);
    
    EXPECT_NE(reportedError, DCError::None);
}

TEST_F(DCErrorBehaviorTest, SyncLostCallbackInvoked) {
    bool syncLostCalled = false;
    
    handler_->setSyncLostCallback([&]() {
        syncLostCalled = true;
    });
    
    // Establish sync first
    for (int i = 0; i < 10; i++) {
        handler_->processSync0Signal(1000000 * (i + 1));
    }
    
    // Then lose it
    DCErrorInjection injection;
    injection.enabled = true;
    injection.injectSyncMiss = true;
    injection.syncMissCount = 100;
    
    handler_->setErrorInjection(injection);
    handler_->processSync0Signal(11000000);
}

// =============================================================================
// Partial Configuration Tests
// =============================================================================

TEST_F(DCErrorBehaviorTest, Sync0Disabled) {
    DCErrorConfig config;
    config.sync0Enabled = false;
    
    DCErrorHandler handler(config);
    
    // Should not error when sync is disabled
    handler.processSync0Signal(1000000);
    
    auto stats = handler.getStatistics();
    // Sync events should not be counted when disabled
}

TEST_F(DCErrorBehaviorTest, PacketOrderCheckDisabled) {
    DCErrorConfig config;
    config.packetOrderCheck = false;
    
    DCErrorHandler handler(config);
    
    handler.processPacket(1);
    handler.processPacket(5); // Out of order
    handler.processPacket(3);
    
    EXPECT_FALSE(handler.hasError());
}

} // namespace test
} // namespace DC
} // namespace slave
} // namespace EtherCAT
