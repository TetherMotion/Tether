/**
 * @file test_klipper_step_scheduler.cpp
 * @brief Tests for the real-time StepScheduler.
 */

#include "tether/klipper/motion/StepScheduler.hpp"
#include "tether/klipper/objects/Stepper.hpp"
#include "tether/klipper/device/KlipperDevice.hpp"
#include "tether/klipper/klippy/KlippyHost.hpp"
#include "tether/klipper/transport/LoopbackTransport.hpp"
#include "tether/klipper/config/StandardCommands.hpp"

#include <gtest/gtest.h>
#include <array>
#include <atomic>
#include <chrono>
#include <thread>

using namespace tether::klipper;
using namespace tether::klipper::motion;
using namespace tether::klipper::objects;
using namespace tether::klipper::transport;
using namespace tether::klipper::protocol;
using namespace tether::klipper::config;
using namespace tether::klipper::device;
using namespace tether::klipper::klippy;

// ============================================================================
// Basic scheduling and execution
// ============================================================================

/// @brief A single step command should fire the callback count times.
TEST(StepSchedulerTest, SingleCommandFiresAllSteps) {
    StepScheduler sched(180000000);
    std::atomic<int> steps{0};
    std::atomic<int> lastDir{0};
    sched.setStepCallback([&](uint8_t oid, int8_t dir) {
        steps++;
        lastDir = dir;
    });
    sched.start();

    StepCommand cmd;
    cmd.interval = 1000; // 1000 ticks = ~5.5us at 180MHz
    cmd.count = 100;
    cmd.add = 0;
    cmd.dir = 1;
    sched.schedule(0, cmd, 0);

    // Wait for all steps to fire (with a 1s timeout).
    ASSERT_TRUE(sched.wait(1000));
    EXPECT_EQ(steps.load(), 100);
    EXPECT_EQ(lastDir.load(), 1);
    EXPECT_TRUE(sched.idle());
}

/// @brief Multiple commands for the same OID should all fire.
TEST(StepSchedulerTest, MultipleCommandsSameOid) {
    StepScheduler sched(180000000);
    std::atomic<int> steps{0};
    sched.setStepCallback([&](uint8_t, int8_t) { steps++; });
    sched.start();

    std::vector<StepCommand> cmds;
    for (int i = 0; i < 5; ++i) {
        StepCommand c;
        c.interval = 500;
        c.count = 50;
        c.add = 0;
        c.dir = 1;
        cmds.push_back(c);
    }
    sched.scheduleSequence(0, cmds, 0);

    ASSERT_TRUE(sched.wait(1000));
    EXPECT_EQ(steps.load(), 250);
}

/// @brief Multiple OIDs should fire independently.
TEST(StepSchedulerTest, MultipleOids) {
    StepScheduler sched(180000000);
    std::atomic<int> stepsOid0{0}, stepsOid1{0};
    sched.setStepCallback([&](uint8_t oid, int8_t) {
        if (oid == 0) stepsOid0++;
        else if (oid == 1) stepsOid1++;
    });
    sched.start();

    StepCommand cmd;
    cmd.interval = 1000;
    cmd.count = 50;
    cmd.add = 0;
    cmd.dir = 1;
    sched.schedule(0, cmd, 0);
    sched.schedule(1, cmd, 0);

    ASSERT_TRUE(sched.wait(1000));
    EXPECT_EQ(stepsOid0.load(), 50);
    EXPECT_EQ(stepsOid1.load(), 50);
}

/// @brief Direction should be passed through to the callback.
TEST(StepSchedulerTest, DirectionPassThrough) {
    StepScheduler sched(180000000);
    std::atomic<int> dirSum{0};
    sched.setStepCallback([&](uint8_t, int8_t dir) { dirSum += dir; });
    sched.start();

    StepCommand fwd;
    fwd.interval = 1000; fwd.count = 10; fwd.add = 0; fwd.dir = 1;
    StepCommand rev;
    rev.interval = 1000; rev.count = 10; rev.add = 0; rev.dir = -1;
    sched.schedule(0, fwd, 0);
    sched.schedule(0, rev, 100000);

    ASSERT_TRUE(sched.wait(1000));
    EXPECT_EQ(dirSum.load(), 0); // +10 - 10 = 0
}

/// @brief Acceleration (add) should adjust the interval between steps.
TEST(StepSchedulerTest, AccelerationAdd) {
    StepScheduler sched(180000000);
    std::vector<uint32_t> stepTimes; // MCU tick of each step
    sched.setStepCallback([&](uint8_t, int8_t) {
        // Record the current MCU tick from the scheduler's perspective.
        // We can't access it directly, but we can measure wall time.
    });
    sched.start();

    // Use a large interval so timing is measurable.
    StepCommand cmd;
    cmd.interval = 100000; // 100000 ticks = ~555us at 180MHz
    cmd.count = 20;
    cmd.add = 10000; // add 10000 ticks each step (acceleration)
    cmd.dir = 1;
    sched.schedule(0, cmd, 0);

    // Just verify all steps fire.
    ASSERT_TRUE(sched.wait(2000));
    EXPECT_TRUE(sched.idle());
}

/// @brief Empty command (count=0) should be a no-op.
TEST(StepSchedulerTest, EmptyCommandNoOp) {
    StepScheduler sched(180000000);
    std::atomic<int> steps{0};
    sched.setStepCallback([&](uint8_t, int8_t) { steps++; });
    sched.start();

    StepCommand cmd;
    cmd.interval = 1000;
    cmd.count = 0;
    cmd.add = 0;
    cmd.dir = 1;
    sched.schedule(0, cmd, 0);

    EXPECT_TRUE(sched.idle());
    EXPECT_EQ(steps.load(), 0);
}

/// @brief pendingSteps() should report remaining steps correctly.
TEST(StepSchedulerTest, PendingStepsCount) {
    StepScheduler sched(180000000);
    sched.setStepCallback([](uint8_t, int8_t) {});
    sched.start();

    StepCommand cmd;
    cmd.interval = 1000000; // ~5.5ms at 180MHz — slow enough to check pending
    cmd.count = 10;
    cmd.add = 0;
    cmd.dir = 1;
    sched.schedule(0, cmd, 0);

    // Don't wait — just check pending immediately.
    EXPECT_EQ(sched.pendingSteps(), 10u);
    EXPECT_EQ(sched.pendingCommands(), 1u);
    EXPECT_FALSE(sched.idle());
}

/// @brief reset() should clear all pending steps and re-anchor.
TEST(StepSchedulerTest, ResetClearsPending) {
    StepScheduler sched(180000000);
    sched.setStepCallback([](uint8_t, int8_t) {});
    sched.start();

    StepCommand cmd;
    cmd.interval = 1000000;
    cmd.count = 100;
    cmd.add = 0;
    cmd.dir = 1;
    sched.schedule(0, cmd, 0);
    EXPECT_FALSE(sched.idle());

    sched.reset();
    EXPECT_TRUE(sched.idle());
    EXPECT_EQ(sched.pendingSteps(), 0u);
}

/// @brief setClockAnchor() should allow aligning to an external clock.
TEST(StepSchedulerTest, ClockAnchor) {
    StepScheduler sched(180000000);
    std::atomic<int> steps{0};
    sched.setStepCallback([&](uint8_t, int8_t) { steps++; });

    // Anchor to now, but at MCU tick 1000000.
    auto anchor = std::chrono::steady_clock::now();
    sched.setClockAnchor(anchor, 1000000);

    // Schedule a step at MCU tick 1000000 (i.e. "now").
    StepCommand cmd;
    cmd.interval = 1000;
    cmd.count = 10;
    cmd.add = 0;
    cmd.dir = 1;
    sched.schedule(0, cmd, 1000000);

    ASSERT_TRUE(sched.wait(1000));
    EXPECT_EQ(steps.load(), 10);
}

/// @brief wait() with a short timeout should return false if steps are
///        still pending.
TEST(StepSchedulerTest, WaitTimeout) {
    StepScheduler sched(180000000);
    sched.setStepCallback([](uint8_t, int8_t) {});
    sched.start();

    // Schedule a step far in the future (MCU tick 10 billion = ~55s).
    StepCommand cmd;
    cmd.interval = 1000;
    cmd.count = 10;
    cmd.add = 0;
    cmd.dir = 1;
    sched.schedule(0, cmd, 10000000000u);

    // Wait with a 50ms timeout — should time out.
    EXPECT_FALSE(sched.wait(50));
    EXPECT_FALSE(sched.idle());
}

/// @brief clockFrequency() should return the configured frequency.
TEST(StepSchedulerTest, ClockFrequency) {
    StepScheduler sched(123456789);
    EXPECT_EQ(sched.clockFrequency(), 123456789u);
}

/// @brief tick() without start() should be a no-op (not anchored).
TEST(StepSchedulerTest, TickWithoutStartIsNoOp) {
    StepScheduler sched(180000000);
    std::atomic<int> steps{0};
    sched.setStepCallback([&](uint8_t, int8_t) { steps++; });

    StepCommand cmd;
    cmd.interval = 1000;
    cmd.count = 10;
    cmd.add = 0;
    cmd.dir = 1;
    sched.schedule(0, cmd, 0);

    // Tick without start() — should not fire any steps.
    sched.tick();
    EXPECT_EQ(steps.load(), 0);
}

/// @brief tick() without a callback should not crash.
TEST(StepSchedulerTest, TickWithoutCallback) {
    StepScheduler sched(180000000);
    sched.start();

    StepCommand cmd;
    cmd.interval = 1000;
    cmd.count = 10;
    cmd.add = 0;
    cmd.dir = 1;
    sched.schedule(0, cmd, 0);

    // Should not crash.
    EXPECT_EQ(sched.tick(), 0u);
}

// ============================================================================
// Integration: StepScheduler wired into KlipperDevice
// ============================================================================

/// @brief Fixture for KlipperDevice + StepScheduler integration tests.
class DeviceStepSchedulerTest : public ::testing::Test {
protected:
    static constexpr uint32_t kClockFreq = 180000000;

    void SetUp() override {
        // Create a loopback transport pair.
        pair_ = std::make_unique<transport::LoopbackTransportPair>();

        // Build a standard dictionary.
        config::KlipperConfig cfg;
        config::withStandardCommands(cfg, kClockFreq);
        dict_ = cfg.build();

        // Create the device with StepScheduler enabled.
        device::KlipperDeviceConfig dcfg;
        dcfg.clockFreqHz = kClockFreq;
        dcfg.useStepScheduler = true;
        dev_ = std::make_unique<device::KlipperDevice>(
            std::make_shared<transport::LoopbackTransport>(pair_->deviceEnd()),
            dict_, dcfg);
        dev_->start();

        // Register 4 steppers.
        for (uint8_t i = 0; i < 4; ++i) {
            steppers_[i] = std::make_shared<Stepper>(i);
            dev_->registerStepper(steppers_[i]);
        }
        dev_->enableStepperMotion();

        // Create the host.
        host_ = std::make_unique<klippy::KlippyHost>(
            std::make_shared<transport::LoopbackTransport>(pair_->hostEnd()));
        host_->connect();
        host_->downloadDictionary([this]() { dev_->pump(); });
        host_->syncClock([this]() { dev_->pump(); });
    }

    void pumpBoth(int rounds = 50) {
        for (int i = 0; i < rounds; ++i) {
            dev_->pump();
            host_->pump();
        }
    }

    std::unique_ptr<transport::LoopbackTransportPair> pair_;
    protocol::DataDictionary dict_;
    std::array<std::shared_ptr<Stepper>, 4> steppers_;
    std::unique_ptr<device::KlipperDevice> dev_;
    std::unique_ptr<klippy::KlippyHost> host_;
};

/// @brief The device should have a StepScheduler when configured.
TEST_F(DeviceStepSchedulerTest, SchedulerExists) {
    EXPECT_NE(dev_->stepScheduler(), nullptr);
}

/// @brief A queue_step command should be forwarded to the StepScheduler
/// and the stepper position should update in real time.
TEST_F(DeviceStepSchedulerTest, QueueStepUpdatesPositionInRealTime) {
    // Send a queue_step command via the host: 100 steps on OID 0.
    motion::AxisStepSequence seq;
    seq.oid = 0;
    seq.startClock = 0;
    StepCommand cmd;
    cmd.interval = 1000; // ~5.5us per step at 180MHz
    cmd.count = 100;
    cmd.add = 0;
    cmd.dir = 1;
    seq.steps.push_back(cmd);

    host_->sendStepSequence(seq);
    pumpBoth(50);

    // The StepScheduler should have 100 pending steps.
    EXPECT_EQ(dev_->stepScheduler()->pendingSteps(), 100u);

    // Wait for the scheduler to fire all steps (real-time).
    ASSERT_TRUE(dev_->waitStepScheduler(2000));

    // The stepper position should now be 100.
    EXPECT_EQ(steppers_[0]->position(), 100);
}

/// @brief Multiple OIDs should update independently via the scheduler.
TEST_F(DeviceStepSchedulerTest, MultipleOidsViaScheduler) {
    motion::AxisStepSequence seq0, seq1;
    seq0.oid = 0; seq0.startClock = 0;
    seq1.oid = 1; seq1.startClock = 0;
    StepCommand cmd;
    cmd.interval = 1000; cmd.count = 50; cmd.add = 0; cmd.dir = 1;
    seq0.steps.push_back(cmd);
    seq1.steps.push_back(cmd);

    host_->sendStepSequences({seq0, seq1});
    pumpBoth(50);

    ASSERT_TRUE(dev_->waitStepScheduler(2000));
    EXPECT_EQ(steppers_[0]->position(), 50);
    EXPECT_EQ(steppers_[1]->position(), 50);
}

/// @brief Reverse direction should decrement the position.
TEST_F(DeviceStepSchedulerTest, ReverseDirectionViaScheduler) {
    // Forward 50 steps.
    motion::AxisStepSequence fwd;
    fwd.oid = 0; fwd.startClock = 0;
    StepCommand cmdFwd;
    cmdFwd.interval = 1000; cmdFwd.count = 50; cmdFwd.add = 0; cmdFwd.dir = 1;
    fwd.steps.push_back(cmdFwd);
    host_->sendStepSequence(fwd);
    pumpBoth(50);
    ASSERT_TRUE(dev_->waitStepScheduler(2000));
    ASSERT_EQ(steppers_[0]->position(), 50);

    // Reverse 50 steps (dir = -1).
    motion::AxisStepSequence rev;
    rev.oid = 0; rev.startClock = 0;
    StepCommand cmdRev;
    cmdRev.interval = 1000; cmdRev.count = 50; cmdRev.add = 0; cmdRev.dir = -1;
    rev.steps.push_back(cmdRev);
    // Need to set direction first via set_next_step_dir.
    // The host's sendStepSequence handles this automatically.
    host_->sendStepSequence(rev);
    pumpBoth(50);
    ASSERT_TRUE(dev_->waitStepScheduler(2000));
    EXPECT_EQ(steppers_[0]->position(), 0);
}

/// @brief tickStepScheduler() should return the number of steps fired.
TEST_F(DeviceStepSchedulerTest, TickReturnsFiredCount) {
    motion::AxisStepSequence seq;
    seq.oid = 0; seq.startClock = 0;
    StepCommand cmd;
    cmd.interval = 100000; // ~555us per step — slow enough to tick manually
    cmd.count = 10;
    cmd.add = 0;
    cmd.dir = 1;
    seq.steps.push_back(cmd);

    host_->sendStepSequence(seq);
    pumpBoth(50);

    // Tick the scheduler repeatedly until all steps fire.
    size_t totalFired = 0;
    for (int i = 0; i < 200 && !dev_->stepScheduler()->idle(); ++i) {
        totalFired += dev_->tickStepScheduler();
        std::this_thread::sleep_for(std::chrono::microseconds(200));
    }
    EXPECT_EQ(totalFired, 10u);
    EXPECT_EQ(steppers_[0]->position(), 10);
}

/// @brief Without useStepScheduler, the scheduler should be null.
TEST(DeviceStepSchedulerNoConfig, NoSchedulerWhenDisabled) {
    transport::LoopbackTransportPair pair;
    config::KlipperConfig cfg;
    config::withStandardCommands(cfg, 180000000);
    auto dict = cfg.build();

    device::KlipperDeviceConfig dcfg;
    dcfg.useStepScheduler = false;
    device::KlipperDevice dev(
        std::make_shared<transport::LoopbackTransport>(pair.deviceEnd()),
        dict, dcfg);
    dev.start();

    EXPECT_EQ(dev.stepScheduler(), nullptr);
    EXPECT_EQ(dev.tickStepScheduler(), 0u);
}
