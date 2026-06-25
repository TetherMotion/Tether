/**
 * @file test_slave_status_poller.cpp
 * @brief Unit tests for SlaveStatusPoller
 */

#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include "tether/ethercat/SlaveStatusPoller.hpp"

#include <cstring>
#include <thread>
#include <atomic>
#include <mutex>
#include <condition_variable>
#include <chrono>

using namespace EtherCAT;
using ::testing::_;
using ::testing::Return;
using ::testing::Invoke;
using ::testing::DoAll;

// ============================================================================
// MockTransport — same pattern as test_fault_detection.cpp
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

static auto SetRegU16(uint16_t value) {
    return [value](uint16_t, uint16_t, void* data, uint16_t size) -> bool {
        if (data && size >= 2) {
            std::memcpy(data, &value, 2);
        }
        return true;
    };
}

static constexpr uint16_t REG_AL_STATUS      = 0x0130;
static constexpr uint16_t REG_AL_STATUS_CODE = 0x0134;

// Helper to build AL_STATUS value from state + error flag
static uint16_t alStatusVal(SlaveState state, bool error = false) {
    return static_cast<uint16_t>(state) | (error ? 0x0010 : 0);
}

// ============================================================================
// Construction / Init Tests
// ============================================================================

TEST(SlaveStatusPollerConstructTest, DefaultState) {
    MockTransport transport;
    SlaveStatusPoller poller(transport);
    EXPECT_FALSE(poller.isInitialized());
    EXPECT_EQ(poller.slaveCount(), 0);
    EXPECT_FALSE(poller.isRunning());
}

TEST(SlaveStatusPollerConstructTest, InitWithSlaves) {
    MockTransport transport;
    SlaveStatusPoller poller(transport);
    EXPECT_TRUE(poller.init(4));
    EXPECT_TRUE(poller.isInitialized());
    EXPECT_EQ(poller.slaveCount(), 4);
}

TEST(SlaveStatusPollerConstructTest, InitClampedToMax) {
    MockTransport transport;
    SlaveStatusPoller poller(transport);
    EXPECT_TRUE(poller.init(100));
    EXPECT_EQ(poller.slaveCount(), SlaveStatusPoller::kMaxSlaves);
}

TEST(SlaveStatusPollerConstructTest, InitIdempotent) {
    MockTransport transport;
    SlaveStatusPoller poller(transport);
    EXPECT_TRUE(poller.init(4));
    EXPECT_TRUE(poller.init(8));
    EXPECT_EQ(poller.slaveCount(), 4);
}

TEST(SlaveStatusPollerConstructTest, ShutdownResetsState) {
    MockTransport transport;
    SlaveStatusPoller poller(transport);
    poller.init(4);
    EXPECT_TRUE(poller.isInitialized());
    poller.shutdown();
    EXPECT_FALSE(poller.isInitialized());
    EXPECT_EQ(poller.slaveCount(), 0);
}

// ============================================================================
// Callback Registration Tests
// ============================================================================

TEST(SlaveStatusPollerCallbackTest, RegisterReturnsNonZeroHandle) {
    MockTransport transport;
    SlaveStatusPoller poller(transport);
    poller.init(2);
    auto handle = poller.registerCallback(StatusFilter{}, [](const SlaveStatusEvent&) {});
    EXPECT_NE(handle, 0u);
}

TEST(SlaveStatusPollerCallbackTest, RegisterNullCallbackReturnsZero) {
    MockTransport transport;
    SlaveStatusPoller poller(transport);
    poller.init(2);
    auto handle = poller.registerCallback(StatusFilter{}, nullptr);
    EXPECT_EQ(handle, 0u);
}

TEST(SlaveStatusPollerCallbackTest, UnregisterExistingHandle) {
    MockTransport transport;
    SlaveStatusPoller poller(transport);
    poller.init(2);
    auto handle = poller.registerCallback(StatusFilter{}, [](const SlaveStatusEvent&) {});
    EXPECT_TRUE(poller.unregisterCallback(handle));
}

TEST(SlaveStatusPollerCallbackTest, UnregisterNonExistingHandle) {
    MockTransport transport;
    SlaveStatusPoller poller(transport);
    poller.init(2);
    EXPECT_FALSE(poller.unregisterCallback(999));
    EXPECT_FALSE(poller.unregisterCallback(0));
}

TEST(SlaveStatusPollerCallbackTest, UnregisterAlreadyUnregistered) {
    MockTransport transport;
    SlaveStatusPoller poller(transport);
    poller.init(2);
    auto handle = poller.registerCallback(StatusFilter{}, [](const SlaveStatusEvent&) {});
    EXPECT_TRUE(poller.unregisterCallback(handle));
    EXPECT_FALSE(poller.unregisterCallback(handle));
}

TEST(SlaveStatusPollerCallbackTest, ClearCallbacks) {
    MockTransport transport;
    SlaveStatusPoller poller(transport);
    poller.init(2);
    poller.registerCallback(StatusFilter{}, [](const SlaveStatusEvent&) {});
    poller.registerCallback(StatusFilter{}, [](const SlaveStatusEvent&) {});
    poller.clearCallbacks();
    // Verify no callbacks fire by doing a manual poll — no crash expected
    // (indirect verification via unregister returning false)
    EXPECT_FALSE(poller.unregisterCallback(1));
    EXPECT_FALSE(poller.unregisterCallback(2));
}

// ============================================================================
// Thread Lifecycle Tests
// ============================================================================

TEST(SlaveStatusPollerThreadTest, StartStop) {
    MockTransport transport;
    SlaveStatusPoller poller(transport);
    poller.init(2);
    poller.setPollIntervalMs(10);

    ON_CALL(transport, readRegister(_, REG_AL_STATUS, _, _))
        .WillByDefault(Invoke(SetRegU16(alStatusVal(SlaveState::OP))));
    ON_CALL(transport, readRegister(_, REG_AL_STATUS_CODE, _, _))
        .WillByDefault(Invoke(SetRegU16(0)));
    ON_CALL(transport, getTimestampMs()).WillByDefault(Return(0));

    EXPECT_TRUE(poller.start());
    EXPECT_TRUE(poller.isRunning());
    poller.stop();
    EXPECT_FALSE(poller.isRunning());
}

TEST(SlaveStatusPollerThreadTest, StartWithoutInitFails) {
    MockTransport transport;
    SlaveStatusPoller poller(transport);
    EXPECT_FALSE(poller.start());
}

TEST(SlaveStatusPollerThreadTest, DoubleStartFails) {
    MockTransport transport;
    SlaveStatusPoller poller(transport);
    poller.init(2);
    poller.setPollIntervalMs(10);

    ON_CALL(transport, readRegister(_, _, _, _))
        .WillByDefault(Invoke(SetRegU16(0x0008)));
    ON_CALL(transport, getTimestampMs()).WillByDefault(Return(0));

    EXPECT_TRUE(poller.start());
    EXPECT_FALSE(poller.start());
    poller.stop();
}

TEST(SlaveStatusPollerThreadTest, StopWithoutStartIsNoop) {
    MockTransport transport;
    SlaveStatusPoller poller(transport);
    poller.init(2);
    poller.stop();  // should not crash
    EXPECT_FALSE(poller.isRunning());
}

// ============================================================================
// State Change Detection Tests (using background thread)
// ============================================================================

class SlaveStatusPollerPollTest : public ::testing::Test {
protected:
    MockTransport transport_;
    SlaveStatusPoller poller_{transport_};

    // Configurable slave state that mock returns
    std::atomic<uint16_t> slave0_al_status{alStatusVal(SlaveState::INIT)};
    std::atomic<uint16_t> slave0_al_code{0};

    void SetUp() override {
        poller_.init(1);
        poller_.setPollIntervalMs(5);

        ON_CALL(transport_, readRegister(0, REG_AL_STATUS, _, _))
            .WillByDefault(Invoke([this](uint16_t, uint16_t, void* data, uint16_t size) -> bool {
                uint16_t val = slave0_al_status.load();
                if (data && size >= 2) std::memcpy(data, &val, 2);
                return true;
            }));
        ON_CALL(transport_, readRegister(0, REG_AL_STATUS_CODE, _, _))
            .WillByDefault(Invoke([this](uint16_t, uint16_t, void* data, uint16_t size) -> bool {
                uint16_t val = slave0_al_code.load();
                if (data && size >= 2) std::memcpy(data, &val, 2);
                return true;
            }));
        ON_CALL(transport_, getTimestampMs()).WillByDefault(Return(0));
    }
};

TEST_F(SlaveStatusPollerPollTest, DetectsStateChange) {
    std::atomic<int> call_count{0};
    std::atomic<SlaveState> observed_new_state{SlaveState::INIT};

    poller_.registerCallback(
        StatusFilter(StatusTransitionFlags::AnyTransition),
        [&](const SlaveStatusEvent& ev) {
            call_count++;
            observed_new_state.store(ev.new_state);
        });

    // Start with INIT
    slave0_al_status.store(alStatusVal(SlaveState::INIT));
    poller_.start();

    // Wait for first poll cycle, then change to OP
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    slave0_al_status.store(alStatusVal(SlaveState::OP));

    // Wait for callback
    std::this_thread::sleep_for(std::chrono::milliseconds(30));
    poller_.stop();

    EXPECT_GT(call_count.load(), 0);
    EXPECT_EQ(observed_new_state.load(), SlaveState::OP);
}

TEST_F(SlaveStatusPollerPollTest, NoCallbackOnSameState) {
    std::atomic<int> call_count{0};

    poller_.registerCallback(
        StatusFilter(StatusTransitionFlags::AnyTransition),
        [&](const SlaveStatusEvent&) { call_count++; });

    // Keep state constant at OP
    slave0_al_status.store(alStatusVal(SlaveState::OP));
    poller_.start();
    std::this_thread::sleep_for(std::chrono::milliseconds(30));
    poller_.stop();

    // First poll sets cache from INIT (default) to OP — that's one transition
    // After that, no more callbacks since state doesn't change
    EXPECT_EQ(call_count.load(), 1);
}

TEST_F(SlaveStatusPollerPollTest, ErrorFlagTransition) {
    std::atomic<bool> saw_error_set{false};
    std::atomic<bool> saw_error_cleared{false};

    poller_.registerCallback(
        StatusFilter(StatusTransitionFlags::ErrorSet),
        [&](const SlaveStatusEvent& ev) {
            if (ev.new_error_flag && !ev.old_error_flag) saw_error_set.store(true);
        });

    poller_.registerCallback(
        StatusFilter(StatusTransitionFlags::ErrorCleared),
        [&](const SlaveStatusEvent& ev) {
            if (!ev.new_error_flag && ev.old_error_flag) saw_error_cleared.store(true);
        });

    slave0_al_status.store(alStatusVal(SlaveState::OP, false));
    poller_.start();

    // Let it settle in OP
    std::this_thread::sleep_for(std::chrono::milliseconds(15));

    // Set error
    slave0_al_status.store(alStatusVal(SlaveState::OP, true));
    slave0_al_code.store(0x0011);
    std::this_thread::sleep_for(std::chrono::milliseconds(15));

    // Clear error
    slave0_al_status.store(alStatusVal(SlaveState::OP, false));
    std::this_thread::sleep_for(std::chrono::milliseconds(15));

    poller_.stop();

    EXPECT_TRUE(saw_error_set.load());
    EXPECT_TRUE(saw_error_cleared.load());
}

// ============================================================================
// Filter Tests
// ============================================================================

TEST_F(SlaveStatusPollerPollTest, FilterToLowerStateOnly) {
    std::atomic<int> lower_count{0};
    std::atomic<int> higher_count{0};

    poller_.registerCallback(
        StatusFilter(StatusTransitionFlags::ToLowerState),
        [&](const SlaveStatusEvent&) { lower_count++; });

    poller_.registerCallback(
        StatusFilter(StatusTransitionFlags::ToHigherState),
        [&](const SlaveStatusEvent&) { higher_count++; });

    // Start at INIT, go to OP (higher), then drop to SAFE_OP (lower)
    slave0_al_status.store(alStatusVal(SlaveState::INIT));
    poller_.start();
    std::this_thread::sleep_for(std::chrono::milliseconds(15));

    slave0_al_status.store(alStatusVal(SlaveState::OP));
    std::this_thread::sleep_for(std::chrono::milliseconds(15));

    slave0_al_status.store(alStatusVal(SlaveState::SAFE_OP));
    std::this_thread::sleep_for(std::chrono::milliseconds(15));

    poller_.stop();

    EXPECT_GT(higher_count.load(), 0);  // INIT->OP and INIT->(cache default INIT)->OP
    EXPECT_GT(lower_count.load(), 0);  // OP->SAFE_OP
}

TEST_F(SlaveStatusPollerPollTest, FilterSpecificFromToState) {
    std::atomic<int> match_count{0};

    poller_.registerCallback(
        StatusFilter(SlaveState::SAFE_OP, SlaveState::OP, StatusTransitionFlags::AnyTransition),
        [&](const SlaveStatusEvent&) { match_count++; });

    // Start at INIT, go to SAFE_OP, then to OP
    slave0_al_status.store(alStatusVal(SlaveState::INIT));
    poller_.start();
    std::this_thread::sleep_for(std::chrono::milliseconds(15));

    slave0_al_status.store(alStatusVal(SlaveState::SAFE_OP));
    std::this_thread::sleep_for(std::chrono::milliseconds(15));

    slave0_al_status.store(alStatusVal(SlaveState::OP));
    std::this_thread::sleep_for(std::chrono::milliseconds(15));

    poller_.stop();

    // Only SAFE_OP -> OP should match (exactly 1)
    EXPECT_EQ(match_count.load(), 1);
}

TEST_F(SlaveStatusPollerPollTest, FilterSpecificSlaveOnly) {
    // Re-init with 2 slaves
    poller_.shutdown();
    poller_.init(2);

    std::atomic<int> slave1_count{0};

    poller_.registerCallback(
        StatusFilter(1, StatusTransitionFlags::AnyTransition),
        [&](const SlaveStatusEvent& ev) {
            if (ev.slave_index == 1) slave1_count++;
    });

    std::atomic<uint16_t> slave1_status{alStatusVal(SlaveState::INIT)};

    ON_CALL(transport_, readRegister(1, REG_AL_STATUS, _, _))
        .WillByDefault(Invoke([&](uint16_t, uint16_t, void* data, uint16_t size) -> bool {
            uint16_t val = slave1_status.load();
            if (data && size >= 2) std::memcpy(data, &val, 2);
            return true;
        }));
    ON_CALL(transport_, readRegister(1, REG_AL_STATUS_CODE, _, _))
        .WillByDefault(Invoke(SetRegU16(0)));

    slave0_al_status.store(alStatusVal(SlaveState::INIT));
    poller_.start();
    std::this_thread::sleep_for(std::chrono::milliseconds(15));

    // Change slave 0 — should NOT trigger callback
    slave0_al_status.store(alStatusVal(SlaveState::OP));
    std::this_thread::sleep_for(std::chrono::milliseconds(15));

    // Change slave 1 — SHOULD trigger callback
    slave1_status.store(alStatusVal(SlaveState::OP));
    std::this_thread::sleep_for(std::chrono::milliseconds(15));

    poller_.stop();

    EXPECT_EQ(slave1_count.load(), 1);
}

TEST_F(SlaveStatusPollerPollTest, MultipleCallbacksFireIndependently) {
    std::atomic<int> cb1_count{0};
    std::atomic<int> cb2_count{0};

    poller_.registerCallback(
        StatusFilter(StatusTransitionFlags::ToHigherState),
        [&](const SlaveStatusEvent&) { cb1_count++; });

    poller_.registerCallback(
        StatusFilter(StatusTransitionFlags::ToLowerState),
        [&](const SlaveStatusEvent&) { cb2_count++; });

    slave0_al_status.store(alStatusVal(SlaveState::INIT));
    poller_.start();
    std::this_thread::sleep_for(std::chrono::milliseconds(15));

    // INIT -> OP (higher)
    slave0_al_status.store(alStatusVal(SlaveState::OP));
    std::this_thread::sleep_for(std::chrono::milliseconds(15));

    // OP -> PRE_OP (lower)
    slave0_al_status.store(alStatusVal(SlaveState::PRE_OP));
    std::this_thread::sleep_for(std::chrono::milliseconds(15));

    poller_.stop();

    EXPECT_GT(cb1_count.load(), 0);
    EXPECT_GT(cb2_count.load(), 0);
}

TEST_F(SlaveStatusPollerPollTest, UnregisterStopsCallbacks) {
    std::atomic<int> call_count{0};

    auto handle = poller_.registerCallback(
        StatusFilter(StatusTransitionFlags::AnyTransition),
        [&](const SlaveStatusEvent&) { call_count++; });

    slave0_al_status.store(alStatusVal(SlaveState::INIT));
    poller_.start();
    std::this_thread::sleep_for(std::chrono::milliseconds(15));

    int count_before = call_count.load();
    poller_.unregisterCallback(handle);

    // Change state — should NOT trigger callback
    slave0_al_status.store(alStatusVal(SlaveState::OP));
    std::this_thread::sleep_for(std::chrono::milliseconds(15));

    poller_.stop();

    EXPECT_EQ(call_count.load(), count_before);
}

// ============================================================================
// Query Tests
// ============================================================================

TEST_F(SlaveStatusPollerPollTest, GetSlaveStateAfterPoll) {
    slave0_al_status.store(alStatusVal(SlaveState::OP));
    poller_.start();
    std::this_thread::sleep_for(std::chrono::milliseconds(15));
    poller_.stop();

    EXPECT_EQ(poller_.getSlaveState(0), SlaveState::OP);
}

TEST(SlaveStatusPollerQueryTest, GetSlaveStateInvalidIndex) {
    MockTransport transport;
    SlaveStatusPoller poller(transport);
    poller.init(2);
    EXPECT_EQ(poller.getSlaveState(99), SlaveState::INIT);
}

TEST(SlaveStatusPollerQueryTest, GetSlaveStateBeforeInit) {
    MockTransport transport;
    SlaveStatusPoller poller(transport);
    EXPECT_EQ(poller.getSlaveState(0), SlaveState::INIT);
}

// ============================================================================
// AL_STATUS_CODE in Event
// ============================================================================

TEST_F(SlaveStatusPollerPollTest, EventContainsAlStatusCodeOnError) {
    std::atomic<uint16_t> observed_code{0};
    std::atomic<bool> got_event{false};

    poller_.registerCallback(
        StatusFilter(StatusTransitionFlags::ErrorSet),
        [&](const SlaveStatusEvent& ev) {
            observed_code.store(ev.al_status_code);
            got_event.store(true);
        });

    slave0_al_status.store(alStatusVal(SlaveState::OP, false));
    poller_.start();
    std::this_thread::sleep_for(std::chrono::milliseconds(15));

    slave0_al_status.store(alStatusVal(SlaveState::OP, true));
    slave0_al_code.store(0x0011);
    std::this_thread::sleep_for(std::chrono::milliseconds(15));

    poller_.stop();

    EXPECT_TRUE(got_event.load());
    EXPECT_EQ(observed_code.load(), 0x0011u);
}
