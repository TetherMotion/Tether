/**
 * @file test_klipper_objects.cpp
 * @brief Tests for the object model: OidAllocator, Stepper, peripherals.
 */

#include <gtest/gtest.h>
#include "tether/klipper/objects/OidAllocator.hpp"
#include "tether/klipper/objects/Stepper.hpp"
#include "tether/klipper/objects/Peripherals.hpp"

using namespace tether::klipper::objects;

TEST(KlipperOidAllocator, SequentialAllocation) {
    OidAllocator alloc;
    EXPECT_EQ(alloc.allocate(), 0u);
    EXPECT_EQ(alloc.allocate(), 1u);
    EXPECT_EQ(alloc.allocate(), 2u);
    EXPECT_EQ(alloc.nextOid(), 3u);
}

TEST(KlipperOidAllocator, BlockAllocation) {
    OidAllocator alloc;
    EXPECT_EQ(alloc.allocateBlock(5), 0u);
    EXPECT_EQ(alloc.nextOid(), 5u);
    EXPECT_EQ(alloc.allocateBlock(3), 5u);
    EXPECT_EQ(alloc.nextOid(), 8u);
}

TEST(KlipperOidAllocator, AssignAndLookup) {
    OidAllocator alloc;
    alloc.allocate();
    alloc.assign(0, "stepper");
    EXPECT_EQ(alloc.typeOf(0), "stepper");
    EXPECT_EQ(alloc.typeOf(1), "");
}

TEST(KlipperStepper, ExecuteSteps) {
    Stepper s(0);
    s.enqueueStep({1000, 5, 0}, 0);
    EXPECT_EQ(s.pendingCommands(), 1u);
    EXPECT_FALSE(s.idle());
    uint32_t steps = s.tick(1000);
    EXPECT_EQ(steps, 2u); // steps at 0 and 1000
    EXPECT_EQ(s.position(), 2);
    steps = s.tick(5000);
    EXPECT_EQ(steps, 3u); // steps at 2000, 3000, 4000
    EXPECT_EQ(s.position(), 5);
    EXPECT_TRUE(s.idle());
}

TEST(KlipperStepper, Acceleration) {
    Stepper s(0);
    // interval=1000, count=3, add=100 (accelerating)
    s.enqueueStep({1000, 3, 100}, 0);
    s.tick(1000); // step at 0 (interval 1000), step at 1000 (interval 1100)
    EXPECT_EQ(s.position(), 2);
    s.tick(2200); // step at 2100 (interval 1200)
    EXPECT_EQ(s.position(), 3);
    EXPECT_TRUE(s.idle());
}

TEST(KlipperDigitalOut, SetValue) {
    DigitalOut d(0);
    d.setValue(1);
    EXPECT_EQ(d.value(), 1u);
    d.setValue(0);
    EXPECT_EQ(d.value(), 0u);
}

TEST(KlipperDigitalOut, ScheduleValue) {
    DigitalOut d(0);
    d.scheduleValue(1, 1000);
    EXPECT_EQ(d.pending(), 1u);
    d.tick(500);
    EXPECT_EQ(d.value(), 0u);
    d.tick(1000);
    EXPECT_EQ(d.value(), 1u);
    EXPECT_EQ(d.pending(), 0u);
}

TEST(KlipperPwmOut, SetDuty) {
    PwmOut p(0);
    p.setDuty(512, 1024);
    EXPECT_EQ(p.duty(), 512u);
    EXPECT_EQ(p.maxCycle(), 1024u);
}

TEST(KlipperEndstop, State) {
    Endstop e(0);
    EXPECT_EQ(e.state(), 0u);
    e.setState(1);
    EXPECT_EQ(e.state(), 1u);
}

TEST(KlipperTrsync, ArmAndTrigger) {
    Trsync t(0);
    EXPECT_EQ(t.state(), TrsyncState::Idle);
    t.arm(1000, 5000);
    EXPECT_EQ(t.state(), TrsyncState::Armed);
    t.trigger(2000);
    EXPECT_EQ(t.state(), TrsyncState::Triggered);
    EXPECT_EQ(t.triggerClock(), 2000u);
}

TEST(KlipperTrsync, Expire) {
    Trsync t(0);
    t.arm(1000, 5000);
    t.tick(5000);
    EXPECT_EQ(t.state(), TrsyncState::Triggered);
    EXPECT_EQ(t.triggerClock(), 5000u);
}
