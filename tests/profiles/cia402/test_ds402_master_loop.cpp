/**
 * @file test_ds402_master_loop.cpp
 * @brief DS402Master loop-lifecycle tests — deferred motion-controller
 *        mutation while a realtime loop is running, and the scoped
 *        cyclic-loop RAII handle.
 *
 * Motion controllers are iterated by updateMotionControllers() on the
 * realtime thread.  While a loop runs, add/remove/clear enqueue ops that
 * the loop thread applies at the next cycle boundary — these tests verify
 * the deferred semantics (synchronous start(), deferred join, deferred
 * stop()) plus the synchronous fast path when no loop runs.
 */

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <functional>
#include <memory>
#include <thread>

#include "tether/ethercat/Master.hpp"
#include "tether/profiles/cia402/DS402Master.hpp"

using namespace EtherCAT;
using namespace std::chrono_literals;

namespace {

/// Counters outlive the controller — removal destroys the object.
struct ControllerStats {
    std::atomic<int> starts{0}, stops{0}, updates{0};
};

struct MockController : DS402Master::IDriveMotionController {
    explicit MockController(ControllerStats& s) : s(s) {}
    ControllerStats& s;
    bool update_result = true;
    bool start(CiA402Drive&) override { ++s.starts; return true; }
    void stop(CiA402Drive&) override { ++s.stops; }
    bool update(CiA402Drive&, double) override {
        ++s.updates;
        return update_result;
    }
};

template <typename F>
bool waitFor(F&& pred, std::chrono::milliseconds budget = 3'000ms) {
    const auto end = std::chrono::steady_clock::now() + budget;
    while (std::chrono::steady_clock::now() < end) {
        if (pred()) return true;
        std::this_thread::sleep_for(200us);
    }
    return pred();
}

class DS402LoopTest : public ::testing::Test {
protected:
    DS402Master ds402_;
    NetworkInterface iface_{};
    uint8_t mac_[6] = {0x02, 0x00, 0x00, 0x00, 0x00, 0x01};

    void SetUp() override {
        iface_.send = [](const uint8_t*, size_t) { return true; };
        ds402_.start(iface_, mac_);
        ds402_.setSlaveAsDS402(0);
        ds402_.ensureDrive(0);
    }
    void TearDown() override {
        ds402_.stopCyclicLoop();
        ds402_.stopMotionControlLoop();
        ds402_.stop();
    }

    Master::CyclicLoopConfig loopCfg() {
        auto c = Master::CyclicLoopConfig::lowLatency(500);
        c.cpu_isolation.enabled = false;  // no cpuset games in unit tests
        return c;
    }
};

TEST_F(DS402LoopTest, AddControllerWithoutLoopIsSynchronous) {
    ControllerStats st;
    ASSERT_TRUE(ds402_.addMotionController(
        0, std::make_unique<MockController>(st)));
    EXPECT_EQ(st.starts.load(), 1);
    EXPECT_TRUE(ds402_.updateMotionControllers(0.001));
    EXPECT_EQ(st.updates.load(), 1);

    // No loop running → remove stops synchronously.
    EXPECT_TRUE(ds402_.removeMotionController(0));
    EXPECT_EQ(st.stops.load(), 1);
}

TEST_F(DS402LoopTest, RejectsUnmanagedSlaveAndNull) {
    ControllerStats st;
    EXPECT_FALSE(ds402_.addMotionController(
        7, std::make_unique<MockController>(st)));
    EXPECT_FALSE(ds402_.addMotionController(0, nullptr));
    EXPECT_FALSE(ds402_.removeMotionController(42));
}

TEST_F(DS402LoopTest, AddWhileRunningJoinsOnLoopThread) {
    auto guard = ds402_.startCyclicLoopScoped(loopCfg());
    ASSERT_TRUE(static_cast<bool>(guard));

    ControllerStats st;
    ASSERT_TRUE(ds402_.addMotionController(
        0, std::make_unique<MockController>(st)));
    EXPECT_EQ(st.starts.load(), 1);   // start() stays synchronous
    // Registration is deferred — update() runs on the loop thread once the
    // next cycle drains the op queue.
    ASSERT_TRUE(waitFor([&] { return st.updates.load() > 0; }));
}

TEST_F(DS402LoopTest, RemoveWhileRunningStopsOnLoopThread) {
    ControllerStats st;
    ASSERT_TRUE(ds402_.addMotionController(
        0, std::make_unique<MockController>(st)));

    auto guard = ds402_.startCyclicLoopScoped(loopCfg());
    ASSERT_TRUE(static_cast<bool>(guard));
    ASSERT_TRUE(waitFor([&] { return st.updates.load() > 0; }));

    ASSERT_TRUE(ds402_.removeMotionController(0));   // deferred
    ASSERT_TRUE(waitFor([&] { return st.stops.load() > 0; }));

    std::this_thread::sleep_for(5ms);
    const int n = st.updates.load();
    std::this_thread::sleep_for(10ms);
    EXPECT_EQ(st.updates.load(), n);  // no further updates after removal
}

TEST_F(DS402LoopTest, RemoveUnknownWhileRunningReturnsFalse) {
    auto guard = ds402_.startCyclicLoopScoped(loopCfg());
    ASSERT_TRUE(static_cast<bool>(guard));
    EXPECT_FALSE(ds402_.removeMotionController(42));
}

TEST_F(DS402LoopTest, AddThenRemoveBeforeDrainCollapses) {
    auto guard = ds402_.startCyclicLoopScoped(loopCfg());
    ASSERT_TRUE(static_cast<bool>(guard));

    ControllerStats st;
    ASSERT_TRUE(ds402_.addMotionController(
        0, std::make_unique<MockController>(st)));
    // Remove before the pending Add is applied — ordered ops make this
    // collapse to "added then removed" on the loop thread.
    ASSERT_TRUE(ds402_.removeMotionController(0));
    ASSERT_TRUE(waitFor([&] { return st.stops.load() > 0; }));
}

TEST_F(DS402LoopTest, ClearWhileRunningStopsAllControllers) {
    ds402_.setSlaveAsDS402(1);
    ds402_.ensureDrive(1);
    ControllerStats st0, st1;
    ASSERT_TRUE(ds402_.addMotionController(
        0, std::make_unique<MockController>(st0)));
    ASSERT_TRUE(ds402_.addMotionController(
        1, std::make_unique<MockController>(st1)));

    auto guard = ds402_.startCyclicLoopScoped(loopCfg());
    ASSERT_TRUE(static_cast<bool>(guard));
    ASSERT_TRUE(waitFor([&] {
        return st0.updates.load() > 0 && st1.updates.load() > 0;
    }));

    ds402_.clearMotionControllers();   // deferred
    ASSERT_TRUE(waitFor([&] {
        return st0.stops.load() > 0 && st1.stops.load() > 0;
    }));
}

TEST_F(DS402LoopTest, ScopedGuardStopsDs402Loop) {
    {
        auto guard = ds402_.startCyclicLoopScoped(loopCfg());
        ASSERT_TRUE(static_cast<bool>(guard));
        EXPECT_TRUE(ds402_.ethercatMaster().isCyclicLoopRunning());
    }
    EXPECT_FALSE(ds402_.ethercatMaster().isCyclicLoopRunning());
}

TEST_F(DS402LoopTest, PendingOpsAppliedByExplicitStop) {
    ControllerStats st;
    ASSERT_TRUE(ds402_.addMotionController(
        0, std::make_unique<MockController>(st)));

    auto guard = ds402_.startCyclicLoopScoped(loopCfg());
    ASSERT_TRUE(static_cast<bool>(guard));
    ASSERT_TRUE(waitFor([&] { return st.updates.load() > 0; }));

    // Enqueue a remove, then stop the loop before the next drain — the
    // stop path applies leftovers so the controller is still stopped.
    ASSERT_TRUE(ds402_.removeMotionController(0));
    ds402_.stopCyclicLoop();
    EXPECT_GE(st.stops.load(), 1);
}

TEST_F(DS402LoopTest, RoleChangeWhileRunningDefersDriveErase) {
    // setSlaveAsNonDS402() flips the role synchronously but must not
    // erase the drive object under the iterating loop thread — the erase
    // is queued and applied by the next drain on the loop thread.
    ControllerStats st;
    ASSERT_TRUE(ds402_.addMotionController(
        0, std::make_unique<MockController>(st)));

    auto guard = ds402_.startCyclicLoopScoped(loopCfg());
    ASSERT_TRUE(static_cast<bool>(guard));
    ASSERT_TRUE(waitFor([&] { return st.updates.load() > 0; }));

    ds402_.setSlaveAsNonDS402(0);
    EXPECT_FALSE(ds402_.isManagedDrive(0));           // role flips now
    // The controller's stop() runs on the loop thread inside the deferred
    // EraseDrive op — even though the role gate already hides the drive,
    // stop() must still receive the drive object.
    ASSERT_TRUE(waitFor([&] { return st.stops.load() > 0; }));

    ds402_.stopCyclicLoop();   // drains any leftovers synchronously
    EXPECT_EQ(ds402_.driveAt(0), nullptr);  // drive erased by the drain
}

} // namespace
