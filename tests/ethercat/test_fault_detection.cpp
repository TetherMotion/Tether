/**
 * @file test_fault_detection.cpp
 * @brief Comprehensive tests for FaultDetector (instance-based, no global state)
 */

#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include "tether/ethercat/FaultDetection.hpp"

#include <cstring>
#include <vector>
#include <thread>
#include <atomic>
#include <mutex>

using namespace EtherCAT;
using ::testing::_;
using ::testing::Return;
using ::testing::DoAll;
using ::testing::SetArgPointee;
using ::testing::Invoke;

// ============================================================================
// MockTransport
// ============================================================================

class MockTransport : public IFaultTransport {
public:
    MOCK_METHOD(bool, readRegister,
                (uint16_t slave_index, uint16_t reg_addr, void* data, uint16_t size),
                (override));
    MOCK_METHOD(bool, writeRegister,
                (uint16_t slave_index, uint16_t reg_addr, const void* data, uint16_t size),
                (override));
    MOCK_METHOD(uint64_t, getTimestampMs, (), (override));
    MOCK_METHOD(void, delayMs, (uint32_t ms), (override));
};

// Helper: configure readRegister to return a specific uint16_t for a given register
static auto SetRegU16(uint16_t value) {
    return [value](uint16_t /*slave*/, uint16_t /*reg*/, void* data, uint16_t size) -> bool {
        if (data && size >= 2) {
            std::memcpy(data, &value, 2);
        }
        return true;
    };
}

// Helper: configure readRegister to fail
static auto FailRead() {
    return [](uint16_t, uint16_t, void*, uint16_t) -> bool { return false; };
}

// Register addresses
static constexpr uint16_t REG_AL_STATUS      = 0x0130;
static constexpr uint16_t REG_AL_STATUS_CODE = 0x0134;
static constexpr uint16_t REG_AL_CONTROL     = 0x0120;

// ============================================================================
// Construction Tests
// ============================================================================

class FaultDetectorConstructTest : public ::testing::Test {
protected:
    MockTransport transport_;
};

TEST_F(FaultDetectorConstructTest, DefaultState) {
    FaultDetector fd(transport_);
    EXPECT_FALSE(fd.isInitialized());
    EXPECT_EQ(fd.slaveCount(), 0);
    EXPECT_FALSE(fd.anyActive());
}

TEST_F(FaultDetectorConstructTest, InitWithZeroSlaves) {
    FaultDetector fd(transport_);
    EXPECT_TRUE(fd.init(0));
    EXPECT_TRUE(fd.isInitialized());
    EXPECT_EQ(fd.slaveCount(), 0);
}

TEST_F(FaultDetectorConstructTest, InitWithOneSlaves) {
    FaultDetector fd(transport_);
    EXPECT_TRUE(fd.init(1));
    EXPECT_TRUE(fd.isInitialized());
    EXPECT_EQ(fd.slaveCount(), 1);
}

TEST_F(FaultDetectorConstructTest, InitWithMaxSlaves) {
    FaultDetector fd(transport_);
    EXPECT_TRUE(fd.init(FaultDetector::kMaxSlaves));
    EXPECT_TRUE(fd.isInitialized());
    EXPECT_EQ(fd.slaveCount(), FaultDetector::kMaxSlaves);
}

TEST_F(FaultDetectorConstructTest, InitOverMaxClamped) {
    FaultDetector fd(transport_);
    EXPECT_TRUE(fd.init(FaultDetector::kMaxSlaves + 100));
    EXPECT_TRUE(fd.isInitialized());
    EXPECT_EQ(fd.slaveCount(), FaultDetector::kMaxSlaves);
}

TEST_F(FaultDetectorConstructTest, InitIdempotent) {
    FaultDetector fd(transport_);
    EXPECT_TRUE(fd.init(4));
    EXPECT_EQ(fd.slaveCount(), 4);
    // Second init should be idempotent — keeps original count
    EXPECT_TRUE(fd.init(8));
    EXPECT_EQ(fd.slaveCount(), 4);
}

// ============================================================================
// Lifecycle Tests
// ============================================================================

class FaultDetectorLifecycleTest : public ::testing::Test {
protected:
    MockTransport transport_;
};

TEST_F(FaultDetectorLifecycleTest, ShutdownResetsState) {
    FaultDetector fd(transport_);
    fd.init(4);
    EXPECT_TRUE(fd.isInitialized());
    fd.shutdown();
    EXPECT_FALSE(fd.isInitialized());
    EXPECT_EQ(fd.slaveCount(), 0);
}

TEST_F(FaultDetectorLifecycleTest, ReInitAfterShutdown) {
    FaultDetector fd(transport_);
    fd.init(2);
    fd.shutdown();
    EXPECT_TRUE(fd.init(8));
    EXPECT_TRUE(fd.isInitialized());
    EXPECT_EQ(fd.slaveCount(), 8);
}

TEST_F(FaultDetectorLifecycleTest, ShutdownClearsCallback) {
    FaultDetector fd(transport_);
    fd.init(2);

    bool called = false;
    fd.setCallback([&](uint16_t, const SlaveFaultState&) { called = true; });

    fd.shutdown();
    fd.init(2);

    // After shutdown+reinit, callback should have been cleared
    // Set up a fault scenario
    ON_CALL(transport_, readRegister(_, _, _, _))
        .WillByDefault(Invoke([](uint16_t, uint16_t reg, void* data, uint16_t size) -> bool {
            if (data && size >= 2) {
                uint16_t val = (reg == REG_AL_STATUS) ? 0x0011u : 0x001Au;
                std::memcpy(data, &val, 2);
            }
            return true;
        }));
    ON_CALL(transport_, getTimestampMs()).WillByDefault(Return(1000));

    fd.poll(0);
    EXPECT_FALSE(called);
}

// ============================================================================
// Polling Tests — Individual Slave
// ============================================================================

class FaultDetectorPollTest : public ::testing::Test {
protected:
    MockTransport transport_;
    FaultDetector fd_{transport_};

    void SetUp() override {
        fd_.init(4);
    }
};

TEST_F(FaultDetectorPollTest, PollHealthySlave) {
    // AL_STATUS = 0x0008 (OP, no error), AL_STATUS_CODE = 0x0000 (NoError)
    ON_CALL(transport_, readRegister(0, REG_AL_STATUS, _, _))
        .WillByDefault(Invoke(SetRegU16(0x0008)));
    ON_CALL(transport_, readRegister(0, REG_AL_STATUS_CODE, _, _))
        .WillByDefault(Invoke(SetRegU16(0x0000)));
    ON_CALL(transport_, getTimestampMs()).WillByDefault(Return(100));

    auto state = fd_.poll(0);
    EXPECT_FALSE(state.has_fault);
    EXPECT_EQ(state.al_status, 0x0008);
    EXPECT_EQ(state.al_status_code, ALStatusCode::NoError);
}

TEST_F(FaultDetectorPollTest, PollFaultedSlave_ErrorBit) {
    // AL_STATUS with error bit set (bit 4)
    ON_CALL(transport_, readRegister(0, REG_AL_STATUS, _, _))
        .WillByDefault(Invoke(SetRegU16(0x0011)));  // INIT + ERROR
    ON_CALL(transport_, readRegister(0, REG_AL_STATUS_CODE, _, _))
        .WillByDefault(Invoke(SetRegU16(0x0000)));
    ON_CALL(transport_, getTimestampMs()).WillByDefault(Return(200));

    auto state = fd_.poll(0);
    EXPECT_TRUE(state.has_fault);
    EXPECT_TRUE(al_status_has_error(state.al_status));
}

TEST_F(FaultDetectorPollTest, PollFaultedSlave_StatusCode) {
    // AL_STATUS is clean but AL_STATUS_CODE has an error
    ON_CALL(transport_, readRegister(0, REG_AL_STATUS, _, _))
        .WillByDefault(Invoke(SetRegU16(0x0008)));  // OP, no error bit
    ON_CALL(transport_, readRegister(0, REG_AL_STATUS_CODE, _, _))
        .WillByDefault(Invoke(SetRegU16(static_cast<uint16_t>(ALStatusCode::SynchronizationError))));
    ON_CALL(transport_, getTimestampMs()).WillByDefault(Return(300));

    auto state = fd_.poll(0);
    EXPECT_TRUE(state.has_fault);
    EXPECT_EQ(state.al_status_code, ALStatusCode::SynchronizationError);
}

TEST_F(FaultDetectorPollTest, PollInvalidIndex) {
    auto state = fd_.poll(99);
    EXPECT_FALSE(state.has_fault);
    EXPECT_EQ(state.al_status, 0);
}

TEST_F(FaultDetectorPollTest, PollBeforeInit) {
    MockTransport t2;
    FaultDetector fd2(t2);
    // Not initialized
    auto state = fd2.poll(0);
    EXPECT_FALSE(state.has_fault);
}

TEST_F(FaultDetectorPollTest, PollReadFailure) {
    ON_CALL(transport_, readRegister(_, _, _, _))
        .WillByDefault(Return(false));
    ON_CALL(transport_, getTimestampMs()).WillByDefault(Return(0));

    auto state = fd_.poll(0);
    // Read failed → al_status stays 0, al_status_code stays NoError → no fault
    EXPECT_FALSE(state.has_fault);
}

TEST_F(FaultDetectorPollTest, PollUpdatesStoredState) {
    ON_CALL(transport_, readRegister(0, REG_AL_STATUS, _, _))
        .WillByDefault(Invoke(SetRegU16(0x0018)));  // SAFE_OP + ERROR
    ON_CALL(transport_, readRegister(0, REG_AL_STATUS_CODE, _, _))
        .WillByDefault(Invoke(SetRegU16(static_cast<uint16_t>(ALStatusCode::InvalidSyncManagerConfig))));
    ON_CALL(transport_, getTimestampMs()).WillByDefault(Return(500));

    fd_.poll(0);

    const SlaveFaultState* stored = fd_.getState(0);
    ASSERT_NE(stored, nullptr);
    EXPECT_TRUE(stored->has_fault);
    EXPECT_EQ(stored->al_status, 0x0018);
    EXPECT_EQ(stored->al_status_code, ALStatusCode::InvalidSyncManagerConfig);
}

// ============================================================================
// Poll All Tests
// ============================================================================

TEST_F(FaultDetectorPollTest, PollAllNoFaults) {
    ON_CALL(transport_, readRegister(_, REG_AL_STATUS, _, _))
        .WillByDefault(Invoke(SetRegU16(0x0008)));
    ON_CALL(transport_, readRegister(_, REG_AL_STATUS_CODE, _, _))
        .WillByDefault(Invoke(SetRegU16(0x0000)));
    ON_CALL(transport_, getTimestampMs()).WillByDefault(Return(0));

    EXPECT_EQ(fd_.pollAll(), 0);
}

TEST_F(FaultDetectorPollTest, PollAllSomeFaults) {
    // Slave 0 and 2 have faults, 1 and 3 are healthy
    ON_CALL(transport_, readRegister(_, REG_AL_STATUS, _, _))
        .WillByDefault(Invoke([](uint16_t slave, uint16_t, void* data, uint16_t size) -> bool {
            uint16_t val = (slave == 0 || slave == 2) ? 0x0011u : 0x0008u;
            if (data && size >= 2) std::memcpy(data, &val, 2);
            return true;
        }));
    ON_CALL(transport_, readRegister(_, REG_AL_STATUS_CODE, _, _))
        .WillByDefault(Invoke(SetRegU16(0x0000)));
    ON_CALL(transport_, getTimestampMs()).WillByDefault(Return(0));

    EXPECT_EQ(fd_.pollAll(), 2);
}

// ============================================================================
// Callback Tests
// ============================================================================

class FaultDetectorCallbackTest : public ::testing::Test {
protected:
    MockTransport transport_;
    FaultDetector fd_{transport_};

    void SetUp() override {
        fd_.init(4);
        ON_CALL(transport_, getTimestampMs()).WillByDefault(Return(1000));
    }

    void configureSlave(uint16_t slave, uint16_t al_status, ALStatusCode al_code) {
        ON_CALL(transport_, readRegister(slave, REG_AL_STATUS, _, _))
            .WillByDefault(Invoke(SetRegU16(al_status)));
        ON_CALL(transport_, readRegister(slave, REG_AL_STATUS_CODE, _, _))
            .WillByDefault(Invoke(SetRegU16(static_cast<uint16_t>(al_code))));
    }
};

TEST_F(FaultDetectorCallbackTest, CallbackOnNewFault) {
    uint16_t cb_slave = 0xFFFF;
    bool cb_called = false;

    fd_.setCallback([&](uint16_t slave, const SlaveFaultState& state) {
        cb_slave = slave;
        cb_called = true;
        EXPECT_TRUE(state.has_fault);
    });

    configureSlave(0, 0x0011, ALStatusCode::UnspecifiedError);
    fd_.poll(0);

    EXPECT_TRUE(cb_called);
    EXPECT_EQ(cb_slave, 0);
}

TEST_F(FaultDetectorCallbackTest, NoCallbackOnRepeatedFault) {
    int call_count = 0;

    fd_.setCallback([&](uint16_t, const SlaveFaultState&) { call_count++; });

    configureSlave(1, 0x0011, ALStatusCode::UnspecifiedError);
    fd_.poll(1);
    fd_.poll(1);
    fd_.poll(1);

    // Should only fire once (transition from healthy to faulted)
    EXPECT_EQ(call_count, 1);
}

TEST_F(FaultDetectorCallbackTest, CallbackAfterClearAndReFault) {
    int call_count = 0;

    fd_.setCallback([&](uint16_t, const SlaveFaultState&) { call_count++; });

    // First fault
    configureSlave(0, 0x0011, ALStatusCode::UnspecifiedError);
    fd_.poll(0);
    EXPECT_EQ(call_count, 1);

    // Clear fault — make transport return healthy after write
    ON_CALL(transport_, writeRegister(_, _, _, _)).WillByDefault(Return(true));
    configureSlave(0, 0x0008, ALStatusCode::NoError);
    fd_.clear(0);

    // Re-fault
    configureSlave(0, 0x0012, ALStatusCode::SynchronizationError);
    fd_.poll(0);
    EXPECT_EQ(call_count, 2);
}

TEST_F(FaultDetectorCallbackTest, NoCallbackWhenNoneSet) {
    configureSlave(0, 0x0011, ALStatusCode::UnspecifiedError);
    // No callback set — should not crash
    EXPECT_NO_FATAL_FAILURE(fd_.poll(0));
}

TEST_F(FaultDetectorCallbackTest, NullCallbackClearsIt) {
    int call_count = 0;
    fd_.setCallback([&](uint16_t, const SlaveFaultState&) { call_count++; });

    fd_.setCallback(nullptr);

    configureSlave(0, 0x0011, ALStatusCode::UnspecifiedError);
    fd_.poll(0);
    EXPECT_EQ(call_count, 0);
}

TEST_F(FaultDetectorCallbackTest, FaultCountIncrements) {
    fd_.setCallback([](uint16_t, const SlaveFaultState&) {});

    configureSlave(0, 0x0011, ALStatusCode::UnspecifiedError);
    fd_.poll(0);

    const auto* s = fd_.getState(0);
    ASSERT_NE(s, nullptr);
    EXPECT_EQ(s->fault_count, 1u);

    // Clear and re-fault
    ON_CALL(transport_, writeRegister(_, _, _, _)).WillByDefault(Return(true));
    configureSlave(0, 0x0008, ALStatusCode::NoError);
    fd_.clear(0);

    configureSlave(0, 0x0012, ALStatusCode::NoSyncError);
    fd_.poll(0);

    s = fd_.getState(0);
    ASSERT_NE(s, nullptr);
    EXPECT_EQ(s->fault_count, 2u);
}

// ============================================================================
// Fault Clearing Tests
// ============================================================================

class FaultDetectorClearTest : public ::testing::Test {
protected:
    MockTransport transport_;
    FaultDetector fd_{transport_};

    void SetUp() override {
        fd_.init(4);
        ON_CALL(transport_, getTimestampMs()).WillByDefault(Return(1000));
    }
};

TEST_F(FaultDetectorClearTest, ClearSucceeds) {
    // Initial fault
    ON_CALL(transport_, readRegister(0, REG_AL_STATUS, _, _))
        .WillByDefault(Invoke(SetRegU16(0x0011)));
    ON_CALL(transport_, readRegister(0, REG_AL_STATUS_CODE, _, _))
        .WillByDefault(Invoke(SetRegU16(static_cast<uint16_t>(ALStatusCode::UnspecifiedError))));
    fd_.poll(0);

    // Now clear — after write, slave returns healthy
    ON_CALL(transport_, writeRegister(0, REG_AL_CONTROL, _, _))
        .WillByDefault(Return(true));
    ON_CALL(transport_, readRegister(0, REG_AL_STATUS, _, _))
        .WillByDefault(Invoke(SetRegU16(0x0008)));
    ON_CALL(transport_, readRegister(0, REG_AL_STATUS_CODE, _, _))
        .WillByDefault(Invoke(SetRegU16(0x0000)));

    EXPECT_TRUE(fd_.clear(0));

    const auto* s = fd_.getState(0);
    ASSERT_NE(s, nullptr);
    EXPECT_FALSE(s->has_fault);
}

TEST_F(FaultDetectorClearTest, ClearFailsWriteFails) {
    ON_CALL(transport_, readRegister(0, REG_AL_STATUS, _, _))
        .WillByDefault(Invoke(SetRegU16(0x0011)));
    ON_CALL(transport_, readRegister(0, REG_AL_STATUS_CODE, _, _))
        .WillByDefault(Invoke(SetRegU16(0x0001)));
    fd_.poll(0);

    ON_CALL(transport_, writeRegister(_, _, _, _)).WillByDefault(Return(false));

    EXPECT_FALSE(fd_.clear(0));
}

TEST_F(FaultDetectorClearTest, ClearFailsStillFaulted) {
    ON_CALL(transport_, readRegister(0, REG_AL_STATUS, _, _))
        .WillByDefault(Invoke(SetRegU16(0x0011)));
    ON_CALL(transport_, readRegister(0, REG_AL_STATUS_CODE, _, _))
        .WillByDefault(Invoke(SetRegU16(0x0001)));
    ON_CALL(transport_, getTimestampMs()).WillByDefault(Return(100));
    fd_.poll(0);

    // Write succeeds but slave still reports fault
    ON_CALL(transport_, writeRegister(_, _, _, _)).WillByDefault(Return(true));

    EXPECT_FALSE(fd_.clear(0));
}

TEST_F(FaultDetectorClearTest, ClearInvalidIndex) {
    EXPECT_FALSE(fd_.clear(99));
}

TEST_F(FaultDetectorClearTest, ClearBeforeInit) {
    MockTransport t;
    FaultDetector fd(t);
    EXPECT_FALSE(fd.clear(0));
}

TEST_F(FaultDetectorClearTest, ClearCallsDelay) {
    ON_CALL(transport_, readRegister(_, REG_AL_STATUS, _, _))
        .WillByDefault(Invoke(SetRegU16(0x0008)));
    ON_CALL(transport_, readRegister(_, REG_AL_STATUS_CODE, _, _))
        .WillByDefault(Invoke(SetRegU16(0x0000)));
    ON_CALL(transport_, writeRegister(_, _, _, _)).WillByDefault(Return(true));

    EXPECT_CALL(transport_, delayMs(50)).Times(1);
    fd_.clear(0);
}

// ============================================================================
// Query Tests
// ============================================================================

class FaultDetectorQueryTest : public ::testing::Test {
protected:
    MockTransport transport_;
    FaultDetector fd_{transport_};

    void SetUp() override {
        fd_.init(4);
        ON_CALL(transport_, getTimestampMs()).WillByDefault(Return(0));
    }
};

TEST_F(FaultDetectorQueryTest, GetStateValid) {
    const auto* s = fd_.getState(0);
    ASSERT_NE(s, nullptr);
    EXPECT_FALSE(s->has_fault);
}

TEST_F(FaultDetectorQueryTest, GetStateInvalid) {
    EXPECT_EQ(fd_.getState(100), nullptr);
}

TEST_F(FaultDetectorQueryTest, GetStateBeforeInit) {
    MockTransport t;
    FaultDetector fd(t);
    EXPECT_EQ(fd.getState(0), nullptr);
}

TEST_F(FaultDetectorQueryTest, AnyActiveNone) {
    EXPECT_FALSE(fd_.anyActive());
}

TEST_F(FaultDetectorQueryTest, AnyActiveTrue) {
    ON_CALL(transport_, readRegister(2, REG_AL_STATUS, _, _))
        .WillByDefault(Invoke(SetRegU16(0x0011)));
    ON_CALL(transport_, readRegister(2, REG_AL_STATUS_CODE, _, _))
        .WillByDefault(Invoke(SetRegU16(0x0001)));
    fd_.poll(2);

    EXPECT_TRUE(fd_.anyActive());
}

// ============================================================================
// AL Status Error Detection
// ============================================================================

TEST(ALStatusUtil, ErrorBitDetection) {
    EXPECT_FALSE(al_status_has_error(0x0001));  // INIT
    EXPECT_FALSE(al_status_has_error(0x0002));  // PRE_OP
    EXPECT_FALSE(al_status_has_error(0x0004));  // SAFE_OP
    EXPECT_FALSE(al_status_has_error(0x0008));  // OP
    EXPECT_TRUE(al_status_has_error(0x0011));   // INIT + ERROR
    EXPECT_TRUE(al_status_has_error(0x0012));   // PRE_OP + ERROR
    EXPECT_TRUE(al_status_has_error(0x0014));   // SAFE_OP + ERROR
    EXPECT_TRUE(al_status_has_error(0x0018));   // OP + ERROR
    EXPECT_TRUE(al_status_has_error(0x0010));   // ERROR only
}

TEST(ALStatusUtil, StateExtraction) {
    EXPECT_EQ(al_status_get_state(0x0001), 1);
    EXPECT_EQ(al_status_get_state(0x0002), 2);
    EXPECT_EQ(al_status_get_state(0x0004), 4);
    EXPECT_EQ(al_status_get_state(0x0008), 8);
    EXPECT_EQ(al_status_get_state(0x0011), 1);  // Error bit doesn't affect state
    EXPECT_EQ(al_status_get_state(0x0018), 8);
}

TEST(ALStatusUtil, StateNames) {
    EXPECT_STREQ(al_status_get_state_name(0x0001), "INIT");
    EXPECT_STREQ(al_status_get_state_name(0x0002), "PRE_OP");
    EXPECT_STREQ(al_status_get_state_name(0x0003), "BOOTSTRAP");
    EXPECT_STREQ(al_status_get_state_name(0x0004), "SAFE_OP");
    EXPECT_STREQ(al_status_get_state_name(0x0008), "OP");
    EXPECT_STREQ(al_status_get_state_name(0x0000), "UNKNOWN");
    EXPECT_STREQ(al_status_get_state_name(0x0011), "INIT");  // Error flag ignored
}

// ============================================================================
// Sync Error Counting
// ============================================================================

class FaultDetectorSyncErrorTest : public ::testing::Test {
protected:
    MockTransport transport_;
    FaultDetector fd_{transport_};

    void SetUp() override {
        fd_.init(2);
        ON_CALL(transport_, getTimestampMs()).WillByDefault(Return(0));
    }
};

TEST_F(FaultDetectorSyncErrorTest, SyncErrorIncrementsOnSynchronizationError) {
    ON_CALL(transport_, readRegister(0, REG_AL_STATUS, _, _))
        .WillByDefault(Invoke(SetRegU16(0x0014)));  // SAFE_OP + ERROR
    ON_CALL(transport_, readRegister(0, REG_AL_STATUS_CODE, _, _))
        .WillByDefault(Invoke(SetRegU16(static_cast<uint16_t>(ALStatusCode::SynchronizationError))));

    fd_.poll(0);
    const auto* s = fd_.getState(0);
    ASSERT_NE(s, nullptr);
    EXPECT_EQ(s->sync_error_count, 1u);

    // Poll again — should increment again
    fd_.poll(0);
    EXPECT_EQ(s->sync_error_count, 2u);
}

TEST_F(FaultDetectorSyncErrorTest, SyncErrorIncrementsOnNoSyncError) {
    ON_CALL(transport_, readRegister(0, REG_AL_STATUS, _, _))
        .WillByDefault(Invoke(SetRegU16(0x0014)));
    ON_CALL(transport_, readRegister(0, REG_AL_STATUS_CODE, _, _))
        .WillByDefault(Invoke(SetRegU16(static_cast<uint16_t>(ALStatusCode::NoSyncError))));

    fd_.poll(0);
    EXPECT_EQ(fd_.getState(0)->sync_error_count, 1u);
}

TEST_F(FaultDetectorSyncErrorTest, SyncErrorIncrementsOnFatalSyncError) {
    ON_CALL(transport_, readRegister(0, REG_AL_STATUS, _, _))
        .WillByDefault(Invoke(SetRegU16(0x0014)));
    ON_CALL(transport_, readRegister(0, REG_AL_STATUS_CODE, _, _))
        .WillByDefault(Invoke(SetRegU16(static_cast<uint16_t>(ALStatusCode::FatalSyncError))));

    fd_.poll(0);
    EXPECT_EQ(fd_.getState(0)->sync_error_count, 1u);
}

TEST_F(FaultDetectorSyncErrorTest, NoSyncErrorOnOtherFaults) {
    ON_CALL(transport_, readRegister(0, REG_AL_STATUS, _, _))
        .WillByDefault(Invoke(SetRegU16(0x0011)));
    ON_CALL(transport_, readRegister(0, REG_AL_STATUS_CODE, _, _))
        .WillByDefault(Invoke(SetRegU16(static_cast<uint16_t>(ALStatusCode::UnspecifiedError))));

    fd_.poll(0);
    EXPECT_EQ(fd_.getState(0)->sync_error_count, 0u);
}

// ============================================================================
// Watchdog Error Counting
// ============================================================================

class FaultDetectorWatchdogTest : public ::testing::Test {
protected:
    MockTransport transport_;
    FaultDetector fd_{transport_};

    void SetUp() override {
        fd_.init(2);
        ON_CALL(transport_, getTimestampMs()).WillByDefault(Return(0));
    }
};

TEST_F(FaultDetectorWatchdogTest, WatchdogIncrementsOnSMWatchdog) {
    ON_CALL(transport_, readRegister(0, REG_AL_STATUS, _, _))
        .WillByDefault(Invoke(SetRegU16(0x0014)));
    ON_CALL(transport_, readRegister(0, REG_AL_STATUS_CODE, _, _))
        .WillByDefault(Invoke(SetRegU16(static_cast<uint16_t>(ALStatusCode::SyncManagerWatchdog))));

    fd_.poll(0);
    EXPECT_EQ(fd_.getState(0)->watchdog_count, 1u);

    fd_.poll(0);
    EXPECT_EQ(fd_.getState(0)->watchdog_count, 2u);
}

TEST_F(FaultDetectorWatchdogTest, WatchdogIncrementsOnBackgroundWatchdog) {
    ON_CALL(transport_, readRegister(0, REG_AL_STATUS, _, _))
        .WillByDefault(Invoke(SetRegU16(0x0014)));
    ON_CALL(transport_, readRegister(0, REG_AL_STATUS_CODE, _, _))
        .WillByDefault(Invoke(SetRegU16(static_cast<uint16_t>(ALStatusCode::BackgroundWatchdog))));

    fd_.poll(0);
    EXPECT_EQ(fd_.getState(0)->watchdog_count, 1u);
}

TEST_F(FaultDetectorWatchdogTest, NoWatchdogOnOtherFaults) {
    ON_CALL(transport_, readRegister(0, REG_AL_STATUS, _, _))
        .WillByDefault(Invoke(SetRegU16(0x0011)));
    ON_CALL(transport_, readRegister(0, REG_AL_STATUS_CODE, _, _))
        .WillByDefault(Invoke(SetRegU16(static_cast<uint16_t>(ALStatusCode::SynchronizationError))));

    fd_.poll(0);
    EXPECT_EQ(fd_.getState(0)->watchdog_count, 0u);
}

// ============================================================================
// Multiple Independent Instances
// ============================================================================

TEST(FaultDetectorMultiInstance, IndependentState) {
    MockTransport transport_a;
    MockTransport transport_b;

    FaultDetector fd_a(transport_a);
    FaultDetector fd_b(transport_b);

    fd_a.init(2);
    fd_b.init(4);

    EXPECT_EQ(fd_a.slaveCount(), 2);
    EXPECT_EQ(fd_b.slaveCount(), 4);

    // Fault on fd_a slave 0
    ON_CALL(transport_a, readRegister(0, REG_AL_STATUS, _, _))
        .WillByDefault(Invoke(SetRegU16(0x0011)));
    ON_CALL(transport_a, readRegister(0, REG_AL_STATUS_CODE, _, _))
        .WillByDefault(Invoke(SetRegU16(0x0001)));
    ON_CALL(transport_a, getTimestampMs()).WillByDefault(Return(0));

    // fd_b slave 0 is healthy
    ON_CALL(transport_b, readRegister(0, REG_AL_STATUS, _, _))
        .WillByDefault(Invoke(SetRegU16(0x0008)));
    ON_CALL(transport_b, readRegister(0, REG_AL_STATUS_CODE, _, _))
        .WillByDefault(Invoke(SetRegU16(0x0000)));
    ON_CALL(transport_b, getTimestampMs()).WillByDefault(Return(0));

    fd_a.poll(0);
    fd_b.poll(0);

    EXPECT_TRUE(fd_a.anyActive());
    EXPECT_FALSE(fd_b.anyActive());
}

TEST(FaultDetectorMultiInstance, IndependentCallbacks) {
    MockTransport transport_a;
    MockTransport transport_b;

    FaultDetector fd_a(transport_a);
    FaultDetector fd_b(transport_b);

    fd_a.init(1);
    fd_b.init(1);

    int count_a = 0, count_b = 0;
    fd_a.setCallback([&](uint16_t, const SlaveFaultState&) { count_a++; });
    fd_b.setCallback([&](uint16_t, const SlaveFaultState&) { count_b++; });

    // Both fault on slave 0
    auto setupFault = [](MockTransport& t) {
        ON_CALL(t, readRegister(0, REG_AL_STATUS, _, _))
            .WillByDefault(Invoke(SetRegU16(0x0011)));
        ON_CALL(t, readRegister(0, REG_AL_STATUS_CODE, _, _))
            .WillByDefault(Invoke(SetRegU16(0x0001)));
        ON_CALL(t, getTimestampMs()).WillByDefault(Return(0));
    };

    setupFault(transport_a);
    setupFault(transport_b);

    fd_a.poll(0);
    EXPECT_EQ(count_a, 1);
    EXPECT_EQ(count_b, 0);

    fd_b.poll(0);
    EXPECT_EQ(count_a, 1);
    EXPECT_EQ(count_b, 1);
}

TEST(FaultDetectorMultiInstance, ConcurrentPolling) {
    MockTransport transport_a;
    MockTransport transport_b;

    FaultDetector fd_a(transport_a);
    FaultDetector fd_b(transport_b);

    fd_a.init(4);
    fd_b.init(4);

    std::atomic<int> faults_a{0};
    std::atomic<int> faults_b{0};

    // Both return faults
    auto setupFaultAll = [](MockTransport& t) {
        ON_CALL(t, readRegister(_, REG_AL_STATUS, _, _))
            .WillByDefault(Invoke(SetRegU16(0x0011)));
        ON_CALL(t, readRegister(_, REG_AL_STATUS_CODE, _, _))
            .WillByDefault(Invoke(SetRegU16(0x001A)));
        ON_CALL(t, getTimestampMs()).WillByDefault(Return(0));
    };

    setupFaultAll(transport_a);
    setupFaultAll(transport_b);

    std::thread thread_a([&] {
        faults_a = fd_a.pollAll();
    });
    std::thread thread_b([&] {
        faults_b = fd_b.pollAll();
    });

    thread_a.join();
    thread_b.join();

    EXPECT_EQ(faults_a.load(), 4);
    EXPECT_EQ(faults_b.load(), 4);
}

// ============================================================================
// Diagnose Tests (don't crash)
// ============================================================================

class FaultDetectorDiagnoseTest : public ::testing::Test {
protected:
    MockTransport transport_;
    FaultDetector fd_{transport_};

    void SetUp() override {
        fd_.init(2);
        ON_CALL(transport_, getTimestampMs()).WillByDefault(Return(0));
    }
};

TEST_F(FaultDetectorDiagnoseTest, DiagnoseValidSlave) {
    ON_CALL(transport_, readRegister(0, REG_AL_STATUS, _, _))
        .WillByDefault(Invoke(SetRegU16(0x0014)));
    ON_CALL(transport_, readRegister(0, REG_AL_STATUS_CODE, _, _))
        .WillByDefault(Invoke(SetRegU16(static_cast<uint16_t>(ALStatusCode::SynchronizationError))));
    fd_.poll(0);

    // Should not crash
    EXPECT_NO_FATAL_FAILURE(fd_.diagnose(0));
}

TEST_F(FaultDetectorDiagnoseTest, DiagnoseInvalidSlave) {
    EXPECT_NO_FATAL_FAILURE(fd_.diagnose(99));
}

TEST_F(FaultDetectorDiagnoseTest, DiagnoseBeforeInit) {
    MockTransport t;
    FaultDetector fd(t);
    EXPECT_NO_FATAL_FAILURE(fd.diagnose(0));
}

TEST_F(FaultDetectorDiagnoseTest, DiagnoseNoSyncSlave) {
    ON_CALL(transport_, readRegister(0, REG_AL_STATUS, _, _))
        .WillByDefault(Invoke(SetRegU16(0x0014)));
    ON_CALL(transport_, readRegister(0, REG_AL_STATUS_CODE, _, _))
        .WillByDefault(Invoke(SetRegU16(static_cast<uint16_t>(ALStatusCode::NoSyncError))));
    fd_.poll(0);

    // Should not crash — diagnose should call diagnoseNoSync internally
    EXPECT_NO_FATAL_FAILURE(fd_.diagnose(0));
}

TEST_F(FaultDetectorDiagnoseTest, DiagnoseNoSyncDirect) {
    EXPECT_NO_FATAL_FAILURE(fd_.diagnoseNoSync(0));
}

TEST_F(FaultDetectorDiagnoseTest, DiagnosePrintsHumanReadableStatus) {
    // AL_STATUS has error bit; AL_STATUS_CODE set to a known value
    ON_CALL(transport_, readRegister(0, REG_AL_STATUS, _, _))
        .WillByDefault(Invoke(SetRegU16(0x0011))); // INIT + ERROR
    ON_CALL(transport_, readRegister(0, REG_AL_STATUS_CODE, _, _))
        .WillByDefault(Invoke(SetRegU16(static_cast<uint16_t>(ALStatusCode::InvalidMailboxConfig))));
    fd_.poll(0);

    testing::internal::CaptureStdout();
    fd_.diagnose(0);
    std::string out = testing::internal::GetCapturedStdout();

    // Should include the AL_STATUS_CODE line and the human-readable name
    EXPECT_NE(out.find("AL_STATUS_CODE"), std::string::npos);
    EXPECT_NE(out.find(getALStatusCodeName(ALStatusCode::InvalidMailboxConfig)), std::string::npos);
}

TEST_F(FaultDetectorDiagnoseTest, DiagnoseNoSyncPrintsDetailedGuidance) {
    ON_CALL(transport_, readRegister(0, REG_AL_STATUS, _, _))
        .WillByDefault(Invoke(SetRegU16(0x0014))); // SAFE_OP + ERROR
    ON_CALL(transport_, readRegister(0, REG_AL_STATUS_CODE, _, _))
        .WillByDefault(Invoke(SetRegU16(static_cast<uint16_t>(ALStatusCode::NoSyncError))));
    fd_.poll(0);

    testing::internal::CaptureStdout();
    fd_.diagnose(0);
    std::string out = testing::internal::GetCapturedStdout();

    // DiagnoseNoSync() prints a prominent header and recommended actions
    EXPECT_NE(out.find("NO SYNC ERROR"), std::string::npos);
    EXPECT_NE(out.find("Recommended actions"), std::string::npos);
    // The diagnostic text should reference the slave index in the suggested command
    EXPECT_NE(out.find("dc_read_sync_config(0)"), std::string::npos);
}

TEST_F(FaultDetectorDiagnoseTest, DiagnosePrintsCiA402ErrorCode) {
    // Populate stored state directly so diagnose() will print CiA 402 details
    ON_CALL(transport_, getTimestampMs()).WillByDefault(Return(0));
    fd_.init(2);

    // Seed basic AL status/code so state is considered faulted
    ON_CALL(transport_, readRegister(0, REG_AL_STATUS, _, _)).WillByDefault(Invoke(SetRegU16(0x0014)));
    ON_CALL(transport_, readRegister(0, REG_AL_STATUS_CODE, _, _)).WillByDefault(Invoke(SetRegU16(static_cast<uint16_t>(ALStatusCode::UnspecifiedError))));
    fd_.poll(0);

    // Mutate internal stored state (test-only) to include a CiA402 code
    const SlaveFaultState* s_const = fd_.getState(0);
    ASSERT_NE(s_const, nullptr);
    SlaveFaultState* s = const_cast<SlaveFaultState*>(s_const);
    s->error_code_603f = static_cast<uint16_t>(CiA402ErrorCode::OverCurrent);

    testing::internal::CaptureStdout();
    fd_.diagnose(0);
    std::string out = testing::internal::GetCapturedStdout();

    EXPECT_NE(out.find("CiA 402 Error Code"), std::string::npos);
    EXPECT_NE(out.find(getCiA402ErrorCodeName(CiA402ErrorCode::OverCurrent)), std::string::npos);
}

TEST_F(FaultDetectorDiagnoseTest, DiagnosePrintsManufacturerFault) {
    // Ensure stored state contains a decoded manufacturer fault
    ON_CALL(transport_, getTimestampMs()).WillByDefault(Return(0));
    fd_.init(2);

    ON_CALL(transport_, readRegister(0, REG_AL_STATUS, _, _)).WillByDefault(Invoke(SetRegU16(0x0011)));
    ON_CALL(transport_, readRegister(0, REG_AL_STATUS_CODE, _, _)).WillByDefault(Invoke(SetRegU16(static_cast<uint16_t>(ALStatusCode::UnspecifiedError))));
    fd_.poll(0);

    const SlaveFaultState* s_const = fd_.getState(0);
    ASSERT_NE(s_const, nullptr);
    SlaveFaultState* s = const_cast<SlaveFaultState*>(s_const);

    // Use a known manufacturer fault code that parses to Err74.1
    s->mfr_fault = ManufacturerFault::parse(741, 0, 0);

    testing::internal::CaptureStdout();
    fd_.diagnose(0);
    std::string out = testing::internal::GetCapturedStdout();

    EXPECT_NE(out.find("Manufacturer Fault"), std::string::npos);
    EXPECT_NE(out.find("Err74.1"), std::string::npos);
    EXPECT_NE(out.find("No Sync (Err74.1)"), std::string::npos);
}

// ============================================================================
// Free-function Wrapper Tests
// ============================================================================

TEST(FaultDetectorFreeFunctions, DelegateInit) {
    MockTransport t;
    FaultDetector fd(t);

    EXPECT_TRUE(fault_init(fd, 4));
    EXPECT_TRUE(fd.isInitialized());
    EXPECT_EQ(fd.slaveCount(), 4);

    fault_shutdown(fd);
    EXPECT_FALSE(fd.isInitialized());
}

TEST(FaultDetectorFreeFunctions, DelegatePoll) {
    MockTransport t;
    FaultDetector fd(t);
    fd.init(1);

    ON_CALL(t, readRegister(_, REG_AL_STATUS, _, _))
        .WillByDefault(Invoke(SetRegU16(0x0008)));
    ON_CALL(t, readRegister(_, REG_AL_STATUS_CODE, _, _))
        .WillByDefault(Invoke(SetRegU16(0x0000)));
    ON_CALL(t, getTimestampMs()).WillByDefault(Return(0));

    auto state = fault_poll(fd, 0);
    EXPECT_FALSE(state.has_fault);

    EXPECT_EQ(fault_poll_all(fd), 0);
    EXPECT_NE(fault_get_state(fd, 0), nullptr);
    EXPECT_FALSE(fault_any_active(fd));
}

TEST(FaultDetectorFreeFunctions, DelegateDiagnose) {
    MockTransport t;
    FaultDetector fd(t);
    fd.init(1);
    ON_CALL(t, getTimestampMs()).WillByDefault(Return(0));

    // Should not crash
    EXPECT_NO_FATAL_FAILURE(fault_diagnose(fd, 0));
    EXPECT_NO_FATAL_FAILURE(fault_diagnose_no_sync(fd, 0));
}

// ============================================================================
// SlaveFaultState Tests
// ============================================================================

TEST(SlaveFaultStateTest, ClearResetsFields) {
    SlaveFaultState s{};
    s.has_fault = true;
    s.al_status = 0x0011;
    s.al_status_code = ALStatusCode::UnspecifiedError;
    s.error_code_603f = 0x1234;
    s.fault_count = 10;
    s.sync_error_count = 5;

    s.clear();

    EXPECT_FALSE(s.has_fault);
    EXPECT_EQ(s.al_status, 0);
    EXPECT_EQ(s.al_status_code, ALStatusCode::NoError);
    EXPECT_EQ(s.error_code_603f, 0);
    // Note: clear() does NOT reset counters (fault_count, sync_error_count, watchdog_count)
    // as per the original SlaveFaultState::clear() implementation
    EXPECT_EQ(s.fault_count, 10u);
    EXPECT_EQ(s.sync_error_count, 5u);
}

// ============================================================================
// Timestamp Tests
// ============================================================================

TEST(FaultDetectorTimestamp, FaultTimestampRecorded) {
    MockTransport t;
    FaultDetector fd(t);
    fd.init(1);

    ON_CALL(t, readRegister(0, REG_AL_STATUS, _, _))
        .WillByDefault(Invoke(SetRegU16(0x0011)));
    ON_CALL(t, readRegister(0, REG_AL_STATUS_CODE, _, _))
        .WillByDefault(Invoke(SetRegU16(0x0001)));
    ON_CALL(t, getTimestampMs()).WillByDefault(Return(42000));

    fd.setCallback([](uint16_t, const SlaveFaultState&) {});
    fd.poll(0);

    const auto* s = fd.getState(0);
    ASSERT_NE(s, nullptr);
    EXPECT_EQ(s->fault_detected_time, 42000u);
    EXPECT_EQ(s->last_poll_time, 42000u);
}

TEST(FaultDetectorTimestamp, PollTimeUpdates) {
    MockTransport t;
    FaultDetector fd(t);
    fd.init(1);

    ON_CALL(t, readRegister(_, _, _, _)).WillByDefault(Return(false));

    uint64_t ts = 100;
    ON_CALL(t, getTimestampMs()).WillByDefault(Invoke([&]() { return ts; }));

    fd.poll(0);
    EXPECT_EQ(fd.getState(0)->last_poll_time, 100u);

    ts = 200;
    fd.poll(0);
    EXPECT_EQ(fd.getState(0)->last_poll_time, 200u);
}
