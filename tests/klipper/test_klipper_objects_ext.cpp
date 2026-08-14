/**
 * @file test_klipper_objects_ext.cpp
 * @brief Extended object tests: OidAllocator exhaustion/block, Stepper multi-segment,
 *        all peripherals edge cases, Spi/I2c.
 */

#include <gtest/gtest.h>
#include "tether/klipper/objects/OidAllocator.hpp"
#include "tether/klipper/objects/Peripherals.hpp"
#include "tether/klipper/objects/Stepper.hpp"

#include <vector>

using namespace tether::klipper::objects;

// ============================================================================
// OidAllocator extended tests
// ============================================================================

TEST(KlipperOidAllocatorExt, AllocateSequential) {
    OidAllocator alloc;
    for (uint8_t i = 0; i < 10; ++i) {
        auto oid = alloc.allocate();
        ASSERT_TRUE(oid.has_value());
        EXPECT_EQ(*oid, i);
    }
    EXPECT_EQ(alloc.nextOid(), 10);
}

TEST(KlipperOidAllocatorExt, AllocateBlock) {
    OidAllocator alloc;
    auto first = alloc.allocateBlock(5);
    ASSERT_TRUE(first.has_value());
    EXPECT_EQ(*first, 0);
    EXPECT_EQ(alloc.nextOid(), 5);

    auto second = alloc.allocateBlock(3);
    ASSERT_TRUE(second.has_value());
    EXPECT_EQ(*second, 5);
    EXPECT_EQ(alloc.nextOid(), 8);
}

TEST(KlipperOidAllocatorExt, AllocateBlockTooLarge) {
    OidAllocator alloc;
    // Allocate most OIDs
    alloc.allocateBlock(250);
    // Try to allocate more than remaining
    auto result = alloc.allocateBlock(10);
    // Should return nullopt (failure)
    EXPECT_FALSE(result.has_value());
    EXPECT_TRUE(alloc.isFull());
}

TEST(KlipperOidAllocatorExt, AssignAndLookupType) {
    OidAllocator alloc;
    auto oid = alloc.allocate();
    ASSERT_TRUE(oid.has_value());
    alloc.assign(*oid, "stepper");
    EXPECT_EQ(alloc.typeOf(*oid), "stepper");
}

TEST(KlipperOidAllocatorExt, LookupUnassignedType) {
    OidAllocator alloc;
    auto oid = alloc.allocate();
    ASSERT_TRUE(oid.has_value());
    EXPECT_EQ(alloc.typeOf(*oid), ""); // Not assigned yet
}

TEST(KlipperOidAllocatorExt, LookupNonexistentOid) {
    OidAllocator alloc;
    EXPECT_EQ(alloc.typeOf(200), "");
}

TEST(KlipperOidAllocatorExt, Reset) {
    OidAllocator alloc;
    alloc.allocate();
    alloc.allocate();
    alloc.assign(0, "stepper");

    alloc.reset();
    EXPECT_EQ(alloc.nextOid(), 0);
    EXPECT_EQ(alloc.typeOf(0), "");
}

TEST(KlipperOidAllocatorExt, IsFullFalseInitially) {
    OidAllocator alloc;
    EXPECT_FALSE(alloc.isFull());
}

TEST(KlipperOidAllocatorExt, ManyAllocations) {
    OidAllocator alloc;
    for (int i = 0; i < 200; ++i) {
        auto oid = alloc.allocate();
        ASSERT_TRUE(oid.has_value());
    }
}

// ============================================================================
// Stepper extended tests
// ============================================================================

TEST(KlipperStepperExt, InitialState) {
    Stepper s(5);
    EXPECT_EQ(s.oid(), 5);
    EXPECT_EQ(s.position(), 0);
    EXPECT_TRUE(s.idle());
    EXPECT_EQ(s.pendingCommands(), 0u);
}

TEST(KlipperStepperExt, SingleStep) {
    Stepper s(0);
    StepCommand cmd{1000, 1, 0};
    s.enqueueStep(cmd, 100);
    EXPECT_FALSE(s.idle());
    EXPECT_EQ(s.pendingCommands(), 1u);

    // Tick before the step time
    EXPECT_EQ(s.tick(99), 0u);
    EXPECT_EQ(s.position(), 0);

    // Tick at the step time
    EXPECT_EQ(s.tick(100), 1u);
    EXPECT_EQ(s.position(), 1);
    EXPECT_TRUE(s.idle());
}

TEST(KlipperStepperExt, MultipleSteps) {
    Stepper s(0);
    StepCommand cmd{1000, 10, 0}; // 10 steps at 1000-tick intervals
    s.enqueueStep(cmd, 0);

    uint32_t totalSteps = 0;
    for (int i = 1; i <= 10; ++i) {
        totalSteps += s.tick(1000 * i);
    }
    EXPECT_EQ(totalSteps, 10u);
    EXPECT_EQ(s.position(), 10);
    EXPECT_TRUE(s.idle());
}

TEST(KlipperStepperExt, MultipleSegments) {
    Stepper s(0);

    // First segment: 5 steps at interval 1000, starting at clock 0
    // Steps occur at clocks: 0, 1000, 2000, 3000, 4000
    StepCommand cmd1{1000, 5, 0};
    s.enqueueStep(cmd1, 0);

    // Second segment: 3 steps at interval 500, starting at clock 5000
    // Steps occur at clocks: 5000, 5500, 6000
    StepCommand cmd2{500, 3, 0};
    s.enqueueStep(cmd2, 5000);

    EXPECT_EQ(s.pendingCommands(), 2u);

    // Tick at the exact step times to get exactly 1 step per tick
    uint32_t totalSteps = 0;
    totalSteps += s.tick(0);    // step at 0
    totalSteps += s.tick(1000); // step at 1000
    totalSteps += s.tick(2000); // step at 2000
    totalSteps += s.tick(3000); // step at 3000
    totalSteps += s.tick(4000); // step at 4000
    EXPECT_EQ(totalSteps, 5u);
    EXPECT_EQ(s.position(), 5);

    // Execute second segment
    totalSteps += s.tick(5000); // step at 5000
    totalSteps += s.tick(5500); // step at 5500
    totalSteps += s.tick(6000); // step at 6000
    EXPECT_EQ(totalSteps, 8u);
    EXPECT_EQ(s.position(), 8);
    EXPECT_TRUE(s.idle());
}

TEST(KlipperStepperExt, Acceleration) {
    Stepper s(0);
    // Start with interval 1000, 10 steps, add 100 per step
    StepCommand cmd{1000, 10, 100};
    s.enqueueStep(cmd, 0);

    // Execute all steps
    uint32_t clock = 1000;
    uint32_t totalSteps = 0;
    for (int i = 0; i < 10; ++i) {
        totalSteps += s.tick(clock);
        clock += 1000 + 100 * (i + 1); // Interval increases each step
    }
    EXPECT_EQ(totalSteps, 10u);
    EXPECT_EQ(s.position(), 10);
}

TEST(KlipperStepperExt, Deceleration) {
    Stepper s(0);
    // Start with interval 1000, 10 steps, add -100 per step
    StepCommand cmd{1000, 10, -100};
    s.enqueueStep(cmd, 0);

    // Just verify it doesn't crash and steps are taken
    uint32_t totalSteps = 0;
    uint32_t clock = 1000;
    for (int i = 0; i < 10; ++i) {
        totalSteps += s.tick(clock);
        clock += 1000; // Just advance enough
    }
    EXPECT_EQ(totalSteps, 10u);
}

TEST(KlipperStepperExt, Reset) {
    Stepper s(0);
    StepCommand cmd{1000, 5, 0};
    s.enqueueStep(cmd, 0);
    EXPECT_FALSE(s.idle());

    s.reset();
    EXPECT_EQ(s.position(), 0);
    EXPECT_TRUE(s.idle());
    EXPECT_EQ(s.pendingCommands(), 0u);
}

TEST(KlipperStepperExt, SetStepInvert) {
    Stepper s(0);
    s.setStepInvert(1);
    // No direct way to verify, but should not crash
}

TEST(KlipperStepperExt, StepperProxy) {
    StepperProxy p(7);
    EXPECT_EQ(p.oid(), 7);
}

// ============================================================================
// DigitalOut extended tests
// ============================================================================

TEST(KlipperDigitalOutExt, InitialState) {
    DigitalOut d(0);
    EXPECT_EQ(d.oid(), 0);
    EXPECT_EQ(d.value(), 0u);
    EXPECT_EQ(d.pending(), 0u);
}

TEST(KlipperDigitalOutExt, SetValue) {
    DigitalOut d(0);
    d.setValue(1);
    EXPECT_EQ(d.value(), 1u);
    d.setValue(0);
    EXPECT_EQ(d.value(), 0u);
}

TEST(KlipperDigitalOutExt, ScheduleValue) {
    DigitalOut d(0);
    d.scheduleValue(1, 1000);
    EXPECT_EQ(d.pending(), 1u);
    EXPECT_EQ(d.value(), 0u); // Not yet applied

    d.tick(999);
    EXPECT_EQ(d.value(), 0u);

    d.tick(1000);
    EXPECT_EQ(d.value(), 1u);
    EXPECT_EQ(d.pending(), 0u);
}

TEST(KlipperDigitalOutExt, MultipleScheduledValues) {
    DigitalOut d(0);
    d.scheduleValue(1, 100);
    d.scheduleValue(0, 200);
    d.scheduleValue(1, 300);

    EXPECT_EQ(d.pending(), 3u);

    d.tick(100);
    EXPECT_EQ(d.value(), 1u);
    d.tick(200);
    EXPECT_EQ(d.value(), 0u);
    d.tick(300);
    EXPECT_EQ(d.value(), 1u);
    EXPECT_EQ(d.pending(), 0u);
}

TEST(KlipperDigitalOutExt, Proxy) {
    DigitalOutProxy p(3);
    EXPECT_EQ(p.oid(), 3);
}

// ============================================================================
// PwmOut extended tests
// ============================================================================

TEST(KlipperPwmOutExt, InitialState) {
    PwmOut p(0);
    EXPECT_EQ(p.oid(), 0);
    EXPECT_EQ(p.duty(), 0u);
}

TEST(KlipperPwmOutExt, SetDuty) {
    PwmOut p(0);
    p.setDuty(512, 1024);
    EXPECT_EQ(p.duty(), 512u);
    EXPECT_EQ(p.maxCycle(), 1024u);
}

TEST(KlipperPwmOutExt, SetFullDuty) {
    PwmOut p(0);
    p.setDuty(1024, 1024);
    EXPECT_EQ(p.duty(), 1024u);
}

TEST(KlipperPwmOutExt, SetZeroDuty) {
    PwmOut p(0);
    p.setDuty(512, 1024);
    p.setDuty(0, 1024);
    EXPECT_EQ(p.duty(), 0u);
}

TEST(KlipperPwmOutExt, ScheduleDuty) {
    PwmOut p(0);
    p.scheduleDuty(256, 1000);
    p.tick(1000);
    EXPECT_EQ(p.duty(), 256u);
}

TEST(KlipperPwmOutExt, Proxy) {
    PwmOutProxy p(5);
    EXPECT_EQ(p.oid(), 5);
}

// ============================================================================
// AnalogIn tests
// ============================================================================

TEST(KlipperAnalogInExt, InitialState) {
    AnalogIn a(0);
    EXPECT_EQ(a.oid(), 0);
    EXPECT_EQ(a.lastValue(), 0u);
}

TEST(KlipperAnalogInExt, SetSample) {
    AnalogIn a(0);
    a.setSample(512);
    EXPECT_EQ(a.lastValue(), 512u);
    a.setSample(1023);
    EXPECT_EQ(a.lastValue(), 1023u);
}

TEST(KlipperAnalogInExt, Proxy) {
    AnalogInProxy p(2);
    EXPECT_EQ(p.oid(), 2);
}

// ============================================================================
// Endstop extended tests
// ============================================================================

TEST(KlipperEndstopExt, InitialState) {
    Endstop e(0);
    EXPECT_EQ(e.oid(), 0);
    EXPECT_EQ(e.state(), 0u);
}

TEST(KlipperEndstopExt, SetState) {
    Endstop e(0);
    e.setState(1);
    EXPECT_EQ(e.state(), 1u);
    e.setState(0);
    EXPECT_EQ(e.state(), 0u);
}

TEST(KlipperEndstopExt, Proxy) {
    EndstopProxy p(4);
    EXPECT_EQ(p.oid(), 4);
}

// ============================================================================
// Trsync extended tests
// ============================================================================

TEST(KlipperTrsyncExt, InitialState) {
    Trsync t(0);
    EXPECT_EQ(t.oid(), 0);
    EXPECT_EQ(t.state(), TrsyncState::Idle);
}

TEST(KlipperTrsyncExt, Arm) {
    Trsync t(0);
    t.arm(1000, 5000);
    EXPECT_EQ(t.state(), TrsyncState::Armed);
}

TEST(KlipperTrsyncExt, Trigger) {
    Trsync t(0);
    t.arm(1000, 5000);
    t.trigger(1500);
    EXPECT_EQ(t.state(), TrsyncState::Triggered);
    EXPECT_EQ(t.triggerClock(), 1500u);
}

TEST(KlipperTrsyncExt, Expire) {
    Trsync t(0);
    t.arm(1000, 5000);
    t.tick(5000); // At expire time
    EXPECT_EQ(t.state(), TrsyncState::Triggered);
}

TEST(KlipperTrsyncExt, NotTriggeredBeforeExpire) {
    Trsync t(0);
    t.arm(1000, 5000);
    t.tick(4999);
    EXPECT_EQ(t.state(), TrsyncState::Armed);
}

TEST(KlipperTrsyncExt, Reset) {
    Trsync t(0);
    t.arm(1000, 5000);
    t.trigger(1500);
    EXPECT_EQ(t.state(), TrsyncState::Triggered);

    t.reset();
    EXPECT_EQ(t.state(), TrsyncState::Idle);
}

TEST(KlipperTrsyncExt, Proxy) {
    TrsyncProxy p(6);
    EXPECT_EQ(p.oid(), 6);
}

// ============================================================================
// Spi tests
// ============================================================================

TEST(KlipperSpiExt, InitialState) {
    Spi s(0);
    EXPECT_EQ(s.oid(), 0);
}

TEST(KlipperSpiExt, Transfer) {
    Spi s(0);
    std::vector<uint8_t> mosi = {0x01, 0x02, 0x03};
    auto miso = s.transfer(mosi);
    EXPECT_EQ(miso.size(), mosi.size());
}

TEST(KlipperSpiExt, TransferEmpty) {
    Spi s(0);
    std::vector<uint8_t> mosi;
    auto miso = s.transfer(mosi);
    EXPECT_TRUE(miso.empty());
}

TEST(KlipperSpiExt, Proxy) {
    SpiProxy p(1);
    EXPECT_EQ(p.oid(), 1);
}

// ============================================================================
// I2c tests
// ============================================================================

TEST(KlipperI2cExt, InitialState) {
    I2c i(0);
    EXPECT_EQ(i.oid(), 0);
}

TEST(KlipperI2cExt, Write) {
    I2c i(0);
    std::vector<uint8_t> data = {0x01, 0x02};
    bool ok = i.write(0x50, data);
    EXPECT_TRUE(ok);
}

TEST(KlipperI2cExt, Read) {
    I2c i(0);
    auto data = i.read(0x50, 4);
    EXPECT_EQ(data.size(), 4u);
}

TEST(KlipperI2cExt, Proxy) {
    I2cProxy p(2);
    EXPECT_EQ(p.oid(), 2);
}
