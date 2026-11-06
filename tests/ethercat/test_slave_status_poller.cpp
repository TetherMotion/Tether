/**
 * @file test_slave_status_poller.cpp
 * @brief Coverage for SlaveStatusPoller — lifecycle, callback filters,
 *        poll-loop event detection and dispatch.
 */

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <mutex>
#include <vector>

#include "tether/ethercat/FaultDetection.hpp"
#include "tether/ethercat/SlaveStatusPoller.hpp"
#include "tether/ethercat/Types.hpp"

using namespace EtherCAT;

namespace {

/// Scripted fault transport: per-slave AL_STATUS/AL_STATUS_CODE values
/// the test mutates between polls; reads can also be failed.
class ScriptableFaultTransport : public IFaultTransport {
public:
    // Atomics: written by the test thread, read/incremented on the poll thread.
    std::atomic<bool> fail_reads{false};
    std::atomic<int> read_calls{0};

    void setAlStatus(uint16_t slave, uint16_t status, uint16_t code = 0) {
        std::lock_guard<std::mutex> lk(mu_);
        al_status_[slave] = status;
        al_code_[slave] = code;
    }

    bool readRegister(uint16_t slave_index, uint16_t reg_addr,
                      void* data, uint16_t size) override {
        ++read_calls;
        if (fail_reads.load()) return false;
        if (reg_addr != 0x0130 || size < 6) return false;
        std::lock_guard<std::mutex> lk(mu_);
        uint16_t status = al_status_[slave_index];
        uint16_t code = al_code_[slave_index];
        uint8_t* p = static_cast<uint8_t*>(data);
        std::memcpy(p, &status, 2);
        std::memset(p + 2, 0, 2);
        std::memcpy(p + 4, &code, 2);
        return true;
    }
    bool writeRegister(uint16_t, uint16_t, const void*, uint16_t) override {
        return true;
    }
    uint64_t getTimestampMs() override {
        return static_cast<uint64_t>(
            std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now().time_since_epoch()).count());
    }
    void delayMs(uint32_t ms) override {
        std::this_thread::sleep_for(std::chrono::milliseconds(ms));
    }

private:
    std::mutex mu_;
    uint16_t al_status_[8] = {};
    uint16_t al_code_[8] = {};
};

class SlaveStatusPollerCoverageTest : public ::testing::Test {
protected:
    ScriptableFaultTransport transport_;
    SlaveStatusPoller poller_{transport_};

    void TearDown() override { poller_.shutdown(); }

    bool waitFor(std::atomic<int>& counter, int target,
                 int timeout_ms = 2000) {
        for (int i = 0; i < timeout_ms / 10; ++i) {
            if (counter.load() >= target) return true;
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        return counter.load() >= target;
    }
};

// ============================================================================
// Lifecycle
// ============================================================================

TEST_F(SlaveStatusPollerCoverageTest, InitAndShutdownIdempotent) {
    EXPECT_TRUE(poller_.init(2));
    EXPECT_TRUE(poller_.init(2));   // second init is a no-op success
    poller_.shutdown();
    EXPECT_TRUE(poller_.init(1));   // re-init after shutdown works
}

TEST_F(SlaveStatusPollerCoverageTest, StartGuards) {
    EXPECT_FALSE(poller_.start());  // not initialized → refuse
    EXPECT_TRUE(poller_.init(0));
    EXPECT_FALSE(poller_.start());  // zero slaves → refuse
    poller_.shutdown();
    EXPECT_TRUE(poller_.init(2));
    EXPECT_TRUE(poller_.start());
    EXPECT_FALSE(poller_.start());  // already running → refuse
    poller_.stop();
}

TEST_F(SlaveStatusPollerCoverageTest, PollLoopReadsRegisters) {
    ASSERT_TRUE(poller_.init(2));
    transport_.setAlStatus(0, 0x0002);  // PRE_OP
    transport_.setAlStatus(1, 0x0004);  // SAFE_OP
    ASSERT_TRUE(poller_.start());
    for (int i = 0; i < 200 && transport_.read_calls < 4; ++i) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    poller_.stop();
    EXPECT_GE(transport_.read_calls, 4);
}

TEST_F(SlaveStatusPollerCoverageTest, GetSlaveState) {
    ASSERT_TRUE(poller_.init(2));
    transport_.setAlStatus(1, 0x0004);  // SAFE_OP
    ASSERT_TRUE(poller_.start());
    for (int i = 0; i < 200 && poller_.getSlaveState(1) != SlaveState::SAFE_OP; ++i) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    EXPECT_EQ(poller_.getSlaveState(1), SlaveState::SAFE_OP);
    poller_.stop();
    // Out-of-range index reports INIT.
    EXPECT_EQ(poller_.getSlaveState(7), SlaveState::INIT);
}

// ============================================================================
// Callbacks and dispatch
// ============================================================================

TEST_F(SlaveStatusPollerCoverageTest, NullCallbackRejected) {
    EXPECT_EQ(poller_.registerCallback({}, {}), 0u);
}

TEST_F(SlaveStatusPollerCoverageTest, UnregisterCallback) {
    auto h = poller_.registerCallback({}, [](const SlaveStatusEvent&) {});
    EXPECT_NE(h, 0u);
    EXPECT_TRUE(poller_.unregisterCallback(h));
    EXPECT_FALSE(poller_.unregisterCallback(h));
    EXPECT_FALSE(poller_.unregisterCallback(0));
}

TEST_F(SlaveStatusPollerCoverageTest, StateChangeEventDispatched) {
    ASSERT_TRUE(poller_.init(1));
    transport_.setAlStatus(0, 0x0001);  // INIT baseline

    std::atomic<int> calls{0};
    SlaveStatusEvent last{};
    std::mutex mu;
    poller_.registerCallback({}, [&](const SlaveStatusEvent& e) {
        std::lock_guard<std::mutex> lk(mu);
        last = e;
        ++calls;
    });

    ASSERT_TRUE(poller_.start());
    // Baseline poll so the INIT state is cached, then transition.
    std::this_thread::sleep_for(std::chrono::milliseconds(30));
    transport_.setAlStatus(0, 0x0002);  // INIT -> PRE_OP
    ASSERT_TRUE(waitFor(calls, 1));
    poller_.stop();

    std::lock_guard<std::mutex> lk(mu);
    EXPECT_EQ(last.slave_index, 0u);
    EXPECT_EQ(last.old_state, SlaveState::INIT);
    EXPECT_EQ(last.new_state, SlaveState::PRE_OP);
}

TEST_F(SlaveStatusPollerCoverageTest, FilterSelectsSlaveAndStates) {
    ASSERT_TRUE(poller_.init(2));
    transport_.setAlStatus(0, 0x0001);
    transport_.setAlStatus(1, 0x0001);

    std::atomic<int> hits{0};
    StatusFilter f;
    f.slave_index = 1;
    f.from_state = SlaveState::INIT;
    f.to_state = SlaveState::OP;
    poller_.registerCallback(f, [&](const SlaveStatusEvent&) { ++hits; });

    ASSERT_TRUE(poller_.start());
    std::this_thread::sleep_for(std::chrono::milliseconds(30));
    transport_.setAlStatus(0, 0x0008);  // slave 0: INIT -> OP (filtered out)
    transport_.setAlStatus(1, 0x0002);  // slave 1: INIT -> PRE_OP (wrong to_state)
    std::this_thread::sleep_for(std::chrono::milliseconds(60));
    EXPECT_EQ(hits.load(), 0);

    transport_.setAlStatus(1, 0x0008);  // slave 1: PRE_OP -> OP — from_state
                                      // is now PRE_OP, still not INIT → no hit
    std::this_thread::sleep_for(std::chrono::milliseconds(60));
    EXPECT_EQ(hits.load(), 0);
    poller_.stop();
}

TEST_F(SlaveStatusPollerCoverageTest, TransitionFlagRanks) {
    ASSERT_TRUE(poller_.init(1));
    transport_.setAlStatus(0, 0x0004);  // SAFE_OP baseline

    std::atomic<int> ups{0}, downs{0};
    StatusFilter up_f, down_f;
    up_f.transition_flags = static_cast<uint8_t>(StatusTransitionFlags::ToHigherState);
    down_f.transition_flags = static_cast<uint8_t>(StatusTransitionFlags::ToLowerState);
    poller_.registerCallback(up_f, [&](const SlaveStatusEvent&) { ++ups; });
    poller_.registerCallback(down_f, [&](const SlaveStatusEvent&) { ++downs; });

    ASSERT_TRUE(poller_.start());
    // First poll: cached INIT -> reported SAFE_OP is itself a rise.
    ASSERT_TRUE(waitFor(ups, 1));
    EXPECT_EQ(downs.load(), 0);
    transport_.setAlStatus(0, 0x0001);  // SAFE_OP -> INIT: a drop
    ASSERT_TRUE(waitFor(downs, 1));
    poller_.stop();
}

TEST_F(SlaveStatusPollerCoverageTest, ErrorFlagEvents) {
    ASSERT_TRUE(poller_.init(1));
    transport_.setAlStatus(0, 0x0002);          // PRE_OP, no error
    std::atomic<int> err_set{0}, err_clear{0};
    StatusFilter f;
    f.transition_flags = static_cast<uint8_t>(
        StatusTransitionFlags::ErrorSet | StatusTransitionFlags::ErrorCleared);
    poller_.registerCallback(f, [&](const SlaveStatusEvent& e) {
        if (e.new_error_flag) ++err_set; else ++err_clear;
    });

    ASSERT_TRUE(poller_.start());
    std::this_thread::sleep_for(std::chrono::milliseconds(30));
    transport_.setAlStatus(0, 0x0012, 0x001B);  // error bit + code
    ASSERT_TRUE(waitFor(err_set, 1));
    transport_.setAlStatus(0, 0x0002);
    ASSERT_TRUE(waitFor(err_clear, 1));
    poller_.stop();
}

TEST_F(SlaveStatusPollerCoverageTest, ReadFailureSkipsPoll) {
    ASSERT_TRUE(poller_.init(1));
    transport_.fail_reads = true;
    ASSERT_TRUE(poller_.start());
    std::this_thread::sleep_for(std::chrono::milliseconds(40));
    poller_.stop();
    EXPECT_EQ(poller_.getSlaveState(0), SlaveState::INIT);   // nothing cached
    EXPECT_GT(transport_.read_calls, 0);
}

TEST_F(SlaveStatusPollerCoverageTest, ClearCallbacksStopsDispatch) {
    ASSERT_TRUE(poller_.init(1));
    transport_.setAlStatus(0, 0x0001);
    std::atomic<int> calls{0};
    poller_.registerCallback({}, [&](const SlaveStatusEvent&) { ++calls; });
    poller_.clearCallbacks();

    ASSERT_TRUE(poller_.start());
    std::this_thread::sleep_for(std::chrono::milliseconds(30));
    transport_.setAlStatus(0, 0x0002);
    std::this_thread::sleep_for(std::chrono::milliseconds(60));
    poller_.stop();
    EXPECT_EQ(calls.load(), 0);
}

}  // namespace
