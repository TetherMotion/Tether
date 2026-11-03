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

// ============================================================================
// Coverage additions — lifecycle, roles, accessors, tasks, recovery
// ============================================================================

TEST_F(DS402LoopTest, ConfigConstructor) {
    Master::Config cfg;
    DS402Master configured(cfg);
    EXPECT_FALSE(configured.isRunning());
}

TEST_F(DS402LoopTest, IsRunningAndCounts) {
    EXPECT_TRUE(ds402_.isRunning());
    EXPECT_EQ(ds402_.discoveredDriveCount(), 0);
    // Short timeout on a stub bus — polls then returns false.
    EXPECT_FALSE(ds402_.waitForDriveCount(1, 30));
    ds402_.stop();
    EXPECT_FALSE(ds402_.isRunning());
}

TEST_F(DS402LoopTest, DistributedClockPaths) {
    DC::DCConfig dc = DC::DCConfig::defaults();
    // Stub master — these reach Master::dc() and fail gracefully.
    ds402_.initializeDistributedClocks(dc, 1);
    ds402_.startDistributedClocks();
    ds402_.startDistributedClocks(dc, 1);
    ds402_.stopDistributedClocks();
    SUCCEED();
}

TEST_F(DS402LoopTest, SlaveRolesAndAccessors) {
    ds402_.setSlaveAsDynaDrive(2);
    EXPECT_EQ(ds402_.slaveRole(2), DS402Master::SlaveRole::DynaDrive);
    EXPECT_TRUE(ds402_.isManagedDrive(2));
    EXPECT_FALSE(ds402_.isDS402Slave(2));

    ds402_.setSlaveAsNonDS402(2);
    EXPECT_EQ(ds402_.slaveRole(2), DS402Master::SlaveRole::NonDS402);
    EXPECT_FALSE(ds402_.isManagedDrive(2));

    // Out-of-range index → NonDS402 without touching the vector.
    EXPECT_EQ(ds402_.slaveRole(60000), DS402Master::SlaveRole::NonDS402);
    EXPECT_TRUE(ds402_.isDS402Slave(0));  // fixture set slave 0 DS402
}

TEST_F(DS402LoopTest, RoleChangeWithoutLoopErasesDrive) {
    // Fixture ensured drive 0. Non-running loop → demoting to NonDS402
    // erases the drive object synchronously (removeMotionController +
    // drives_ erase in the non-loop branch).
    EXPECT_NE(ds402_.driveBySlaveIndex(0), nullptr);
    ds402_.setSlaveAsNonDS402(0);
    EXPECT_EQ(ds402_.driveAt(0), nullptr);
    EXPECT_EQ(ds402_.slaveRole(0), DS402Master::SlaveRole::NonDS402);
}

TEST_F(DS402LoopTest, DriveAccessors) {
    EXPECT_NE(ds402_.driveAt(0), nullptr);
    const DS402Master& c = ds402_;
    EXPECT_NE(c.driveAt(0), nullptr);
    EXPECT_EQ(c.driveAt(9), nullptr);
    EXPECT_EQ(ds402_.driveAt(42), nullptr);

    // ensureDrive on an unmanaged index promotes it to DS402.
    auto& d = ds402_.ensureDrive(5);
    EXPECT_EQ(d.slaveIndex(), 5);
    EXPECT_TRUE(ds402_.isDS402Slave(5));
    EXPECT_EQ(&ds402_.ensureDrive(5), &d);  // second call returns same

    EXPECT_NE(ds402_.driveBySlaveIndex(5), nullptr);
    EXPECT_NE(c.driveBySlaveIndex(5), nullptr);
    EXPECT_EQ(ds402_.driveBySlaveIndex(9), nullptr);   // unmanaged
    EXPECT_EQ(c.driveBySlaveIndex(9), nullptr);
}

TEST_F(DS402LoopTest, RemoveControllerPaths) {
    ControllerStats st;
    ASSERT_TRUE(ds402_.addMotionController(
        0, std::make_unique<MockController>(st)));
    // No loop running → removal drains eraseController immediately.
    EXPECT_TRUE(ds402_.removeMotionController(0));
    EXPECT_EQ(st.stops.load(), 1);
    EXPECT_FALSE(ds402_.removeMotionController(0));   // already gone
    EXPECT_FALSE(ds402_.removeMotionController(42));  // never existed
}

TEST_F(DS402LoopTest, AddCyclicTaskOverloads) {
    struct DummyTask : EtherCAT::ICyclicTask {
        bool update(DS402Master&, double) override { return true; }
    };
    EXPECT_FALSE(ds402_.addCyclicTask(nullptr));
    EXPECT_FALSE(ds402_.addCyclicTask(nullptr, EtherCAT::TaskPhase::MotionControl, 1));
    EXPECT_TRUE(ds402_.addCyclicTask(std::make_unique<DummyTask>()));
    EXPECT_TRUE(ds402_.addCyclicTask(std::make_unique<DummyTask>(),
                                     EtherCAT::TaskPhase::MotionControl, 7));
    ds402_.clearCyclicTasks();
}

TEST_F(DS402LoopTest, UpdateMotionControllersBranches) {
    // No controllers at all → true, no work.
    EXPECT_TRUE(ds402_.updateMotionControllers(0.001));

    ControllerStats st;
    ASSERT_TRUE(ds402_.addMotionController(
        0, std::make_unique<MockController>(st)));
    EXPECT_TRUE(ds402_.updateMotionControllers(0.001));
    EXPECT_EQ(st.updates.load(), 1);

    // Controller update returning false propagates.
    ControllerStats st2;
    ds402_.ensureDrive(5);  // promote to DS402 so the controller is managed
    auto* ctrl = new MockController(st2);
    ctrl->update_result = false;
    ASSERT_TRUE(ds402_.addMotionController(
        5, std::unique_ptr<DS402Master::IDriveMotionController>(ctrl)));
    EXPECT_FALSE(ds402_.updateMotionControllers(0.001));
}

TEST_F(DS402LoopTest, SlaveSupervisorAndRecovery) {
    EXPECT_NO_THROW(ds402_.slaveSupervisor());

    EtherCAT::RecoveryConfig rcfg;
    std::vector<DS402Master::DriveConfiguration> cfgs = {
        {.slave_index = 0, .auto_configure_mailbox = false,
         .transition_to_operational = false}};
    // configure + install handler + start on the stub master.
    ds402_.enableSlaveRecovery(rcfg, cfgs);
    ds402_.disableSlaveRecovery();
}

TEST_F(DS402LoopTest, EnableDisableWithoutDrive) {
    // No drive objects for these indices → false.
    EXPECT_FALSE(ds402_.enableDrive(9, 50));
    EXPECT_FALSE(ds402_.disableDrive(9));
    // A master with zero drives: the all-drives loops are trivially true.
    DS402Master fresh;
    EXPECT_TRUE(fresh.disableAllDrives());
    EXPECT_TRUE(fresh.enableAllDrives(10));
}

TEST_F(DS402LoopTest, StartAndStopLoopVariants) {
    // All three loop entry points install the motion-control callback
    // then delegate to Master — on the stub bus they start and stop.
    if (ds402_.startRealtimeMotionControlLoop()) {
        ds402_.stopMotionControlLoop();
    }
    if (ds402_.startPollingMotionControlLoop()) {
        ds402_.stopMotionControlLoop();
    }
    if (ds402_.startCyclicLoop(loopCfg())) {
        ds402_.stopCyclicLoop();
    }
    // Post-stop: adding a controller applies immediately again.
    ControllerStats st;
    EXPECT_TRUE(ds402_.addMotionController(
        0, std::make_unique<MockController>(st)));
}

TEST_F(DS402LoopTest, DeferredEraseDrainsOnLoopStop) {
    ControllerStats st;
    ASSERT_TRUE(ds402_.addMotionController(
        0, std::make_unique<MockController>(st)));
    auto guard = ds402_.startCyclicLoopScoped(loopCfg());
    ASSERT_TRUE(guard);
    // Demote while the loop runs → EraseDrive op is queued.
    ds402_.setSlaveAsNonDS402(0);
    guard.stop();
    ds402_.stopCyclicLoop();  // drainControllerOps applies the erase
    EXPECT_EQ(ds402_.driveAt(0), nullptr);
}

TEST_F(DS402LoopTest, ClearMotionControllersWhileRunning) {
    ControllerStats st;
    ASSERT_TRUE(ds402_.addMotionController(
        0, std::make_unique<MockController>(st)));
    auto guard = ds402_.startCyclicLoopScoped(loopCfg());
    ASSERT_TRUE(guard);
    ds402_.clearMotionControllers();  // drains stop() on the loop thread
    guard.stop();
    ds402_.stopCyclicLoop();
}
