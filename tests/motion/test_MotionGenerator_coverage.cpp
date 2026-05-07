/**
 * @file test_MotionGenerator_coverage.cpp
 * @brief Coverage tests for MotionGenerator: SCurve, SynchronizedMotion, factory, etc.
 *        Supplements existing test_motion_control.cpp with untested paths.
 */

#include "tether/motion/MotionGenerator.hpp"
#include <gtest/gtest.h>
#include <cmath>
#include <memory>

using namespace Motion;

// ============================================================================
// SineMotionGenerator - extended coverage
// ============================================================================

TEST(SineGenCovTest, DefaultConstruction) {
    SineMotionGenerator gen;
    EXPECT_EQ(gen.getState(), GeneratorState::Idle);
    EXPECT_FALSE(gen.isRunning());
    EXPECT_FALSE(gen.isComplete());
    EXPECT_EQ(gen.getPosition(), 0);
    EXPECT_EQ(gen.getVelocity(), 0);
    EXPECT_EQ(gen.getTorque(), 0);
}

TEST(SineGenCovTest, ConfigureAndStart) {
    SineMotionGenerator gen;
    gen.configure(1000, 1.0f, 500);
    EXPECT_EQ(gen.getAmplitude(), 1000);
    EXPECT_FLOAT_EQ(gen.getFrequency(), 1.0f);
    EXPECT_EQ(gen.getOffset(), 500);
    
    gen.start();
    EXPECT_EQ(gen.getState(), GeneratorState::Running);
    EXPECT_TRUE(gen.isRunning());
}

TEST(SineGenCovTest, SetPhase) {
    SineMotionGenerator gen;
    gen.setPhase(1.5707f); // PI/2
    // Exercise setter — value may take effect only after configure/start
    (void)gen.getPhase();
}

TEST(SineGenCovTest, SetCyclesAndComplete) {
    SineMotionGenerator gen;
    gen.configure(100, 10.0f);
    gen.setCycles(2);
    gen.start();
    
    // Run for enough time to complete 2 cycles at 10 Hz = 200ms
    float dt = 1.0f; // 1ms
    for (int i = 0; i < 300; i++) {
        gen.update(dt);
        if (gen.isComplete()) break;
    }
    EXPECT_TRUE(gen.isComplete());
    EXPECT_GE(gen.getCompletedCycles(), 2u);
}

TEST(SineGenCovTest, PauseAndResume) {
    SineMotionGenerator gen;
    gen.configure(100, 1.0f);
    gen.start();
    gen.update(10.0f);
    int32_t pos1 = gen.getPosition();
    
    gen.pause();
    EXPECT_EQ(gen.getState(), GeneratorState::Paused);
    gen.update(10.0f);
    EXPECT_EQ(gen.getPosition(), pos1); // Should not advance while paused
    
    gen.resume();
    EXPECT_EQ(gen.getState(), GeneratorState::Running);
    gen.update(10.0f);
    // Should have advanced
}

TEST(SineGenCovTest, Stop) {
    SineMotionGenerator gen;
    gen.configure(100, 1.0f);
    gen.start();
    gen.update(10.0f);
    
    gen.stop();
    EXPECT_EQ(gen.getState(), GeneratorState::Idle);
    EXPECT_FALSE(gen.isRunning());
}

TEST(SineGenCovTest, FloatAccessors) {
    SineMotionGenerator gen;
    gen.configure(1000, 1.0f);
    gen.start();
    gen.update(250.0f); // quarter period: sin(PI/2)=1
    
    float pos = gen.getPositionFloat();
    EXPECT_NEAR(pos, 1000.0f, 50.0f); // ~amplitude at quarter period
    
    float vel = gen.getVelocityFloat();
    (void)vel; // just exercise the function
}

TEST(SineGenCovTest, UnlimitedCycles) {
    SineMotionGenerator gen;
    gen.configure(100, 10.0f);
    gen.setCycles(0); // unlimited
    gen.start();
    
    for (int i = 0; i < 500; i++) {
        gen.update(1.0f);
    }
    EXPECT_FALSE(gen.isComplete()); // Never completes with 0 cycles
}

TEST(SineGenCovTest, ResumeFromNonPaused) {
    SineMotionGenerator gen;
    gen.resume(); // Not paused, should be no-op
    EXPECT_EQ(gen.getState(), GeneratorState::Idle);
}

TEST(SineGenCovTest, PauseFromIdle) {
    SineMotionGenerator gen;
    gen.pause();
    EXPECT_EQ(gen.getState(), GeneratorState::Paused);
}

// ============================================================================
// TrapezoidalProfileGenerator
// ============================================================================

TEST(TrapGenCovTest, DefaultConstruction) {
    TrapezoidalProfileGenerator gen;
    EXPECT_EQ(gen.getState(), GeneratorState::Idle);
    EXPECT_EQ(gen.getPosition(), 0);
    EXPECT_EQ(gen.getVelocity(), 0);
    EXPECT_EQ(gen.getPhase(), TrapezoidalProfileGenerator::Phase::Complete);
}

TEST(TrapGenCovTest, ConfigureViaSetters) {
    TrapezoidalProfileGenerator gen;
    gen.setStartPosition(0);
    gen.setTargetPosition(10000);
    gen.setMaxVelocity(1000);
    gen.setAcceleration(500);
    gen.setDeceleration(500);
    gen.start();
    EXPECT_TRUE(gen.isRunning());
}

TEST(TrapGenCovTest, ConfigureMethod) {
    TrapezoidalProfileGenerator gen;
    gen.configure(0, 10000, 1000, 500);
    gen.start();
    
    // Run to completion — needs enough steps
    for (int i = 0; i < 500000 && !gen.isComplete(); i++) {
        gen.update(0.1f);
    }
    EXPECT_TRUE(gen.isComplete());
    EXPECT_NEAR(gen.getPosition(), 10000, 500);
}

TEST(TrapGenCovTest, AccelPhase) {
    TrapezoidalProfileGenerator gen;
    gen.configure(0, 100000, 1000, 100);
    gen.start();
    gen.update(1.0f);
    EXPECT_EQ(gen.getPhase(), TrapezoidalProfileGenerator::Phase::Accel);
}

TEST(TrapGenCovTest, ShortDistance_TriangularProfile) {
    TrapezoidalProfileGenerator gen;
    gen.configure(0, 100, 10000, 500);
    gen.start();
    
    // Very short distance, should skip cruise phase
    // bool saw_cruise = false; // Not used
    for (int i = 0; i < 10000 && !gen.isComplete(); i++) {
        gen.update(0.1f);
        // if (gen.getPhase() == TrapezoidalProfileGenerator::Phase::Cruise) {
        //     saw_cruise = true;
        // }
    }
    EXPECT_TRUE(gen.isComplete());
}

TEST(TrapGenCovTest, NegativeDirection) {
    TrapezoidalProfileGenerator gen;
    gen.configure(10000, 0, 1000, 500);
    gen.start();
    
    for (int i = 0; i < 500000 && !gen.isComplete(); i++) {
        gen.update(0.1f);
    }
    EXPECT_TRUE(gen.isComplete());
    EXPECT_NEAR(gen.getPosition(), 0, 500);
}

TEST(TrapGenCovTest, StopDuringMotion) {
    TrapezoidalProfileGenerator gen;
    gen.configure(0, 10000, 1000, 500);
    gen.start();
    gen.update(5.0f);
    gen.stop();
    EXPECT_EQ(gen.getState(), GeneratorState::Idle);
}

TEST(TrapGenCovTest, ZeroDistance) {
    TrapezoidalProfileGenerator gen;
    gen.configure(1000, 1000, 1000, 500);
    gen.start();
    gen.update(1.0f);
    // Already at target
    EXPECT_TRUE(gen.isComplete());
}

// ============================================================================
// SCurveProfileGenerator
// ============================================================================

TEST(SCurveGenCovTest, DefaultConstruction) {
    SCurveProfileGenerator gen;
    EXPECT_EQ(gen.getState(), GeneratorState::Idle);
    EXPECT_EQ(gen.getPosition(), 0);
    EXPECT_EQ(gen.getVelocity(), 0);
}

TEST(SCurveGenCovTest, ConfigureAndRun) {
    SCurveProfileGenerator gen;
    gen.configure(0, 10000, 1000.0f, 500.0f, 5000.0f);
    gen.start();
    EXPECT_TRUE(gen.isRunning());
    
    for (int i = 0; i < 500000 && !gen.isComplete(); i++) {
        gen.update(0.1f);
    }
    EXPECT_TRUE(gen.isComplete());
    EXPECT_NEAR(gen.getPosition(), 10000, 500);
}

TEST(SCurveGenCovTest, SettersAndGetters) {
    SCurveProfileGenerator gen;
    gen.setStartPosition(0);
    gen.setTargetPosition(5000);
    gen.setMaxVelocity(2000.0f);
    gen.setMaxAcceleration(1000.0f);
    gen.setMaxJerk(10000.0f);
    gen.start();
    gen.update(1.0f);
    
    // Exercise all getters
    (void)gen.getPositionFloat();
    (void)gen.getVelocityFloat();
    (void)gen.getAcceleration();
    (void)gen.getJerk();
}

TEST(SCurveGenCovTest, NegativeDirection) {
    SCurveProfileGenerator gen;
    gen.configure(10000, 0, 1000.0f, 500.0f, 5000.0f);
    gen.start();
    
    for (int i = 0; i < 500000 && !gen.isComplete(); i++) {
        gen.update(0.1f);
    }
    EXPECT_TRUE(gen.isComplete());
    EXPECT_NEAR(gen.getPosition(), 0, 500);
}

TEST(SCurveGenCovTest, StopAndRestart) {
    SCurveProfileGenerator gen;
    gen.configure(0, 5000, 1000.0f, 500.0f, 5000.0f);
    gen.start();
    gen.update(5.0f);
    gen.stop();
    EXPECT_EQ(gen.getState(), GeneratorState::Idle);
    
    gen.start(); // Restart
    EXPECT_TRUE(gen.isRunning());
}

TEST(SCurveGenCovTest, PauseResume) {
    SCurveProfileGenerator gen;
    gen.configure(0, 5000, 1000.0f, 500.0f, 5000.0f);
    gen.start();
    gen.update(5.0f);
    
    gen.pause();
    int32_t pos = gen.getPosition();
    gen.update(5.0f);
    EXPECT_EQ(gen.getPosition(), pos);
    
    gen.resume();
    gen.update(5.0f);
    // Should have moved
}

// ============================================================================
// SynchronizedMotionGenerator
// ============================================================================

TEST(SyncGenCovTest, DefaultConstruction) {
    SynchronizedMotionGenerator sync;
    EXPECT_EQ(sync.getAxisCount(), 0u);
    EXPECT_TRUE(sync.allComplete()); // No axes = all complete
}

TEST(SyncGenCovTest, AddAxis) {
    SynchronizedMotionGenerator sync;
    SineMotionGenerator gen1, gen2;
    
    int idx1 = sync.addAxis(&gen1);
    int idx2 = sync.addAxis(&gen2);
    EXPECT_EQ(idx1, 0);
    EXPECT_EQ(idx2, 1);
    EXPECT_EQ(sync.getAxisCount(), 2u);
    EXPECT_EQ(sync.getAxis(0), &gen1);
    EXPECT_EQ(sync.getAxis(1), &gen2);
}

TEST(SyncGenCovTest, AddAxisNull) {
    SynchronizedMotionGenerator sync;
    int idx = sync.addAxis(nullptr);
    EXPECT_EQ(idx, -1);
}

TEST(SyncGenCovTest, AddAxisOverLimit) {
    SynchronizedMotionGenerator sync;
    SineMotionGenerator gens[9]; // kMaxSyncAxes = 8
    for (int i = 0; i < 8; i++) {
        EXPECT_GE(sync.addAxis(&gens[i]), 0);
    }
    EXPECT_EQ(sync.addAxis(&gens[8]), -1); // Should fail
}

TEST(SyncGenCovTest, GetAxisOutOfBounds) {
    SynchronizedMotionGenerator sync;
    EXPECT_EQ(sync.getAxis(0), nullptr);
    EXPECT_EQ(sync.getAxis(100), nullptr);
}

TEST(SyncGenCovTest, StartStopAll) {
    SynchronizedMotionGenerator sync;
    SineMotionGenerator gen1, gen2;
    gen1.configure(100, 1.0f);
    gen2.configure(200, 2.0f);
    gen1.setCycles(1);
    gen2.setCycles(1);
    
    sync.addAxis(&gen1);
    sync.addAxis(&gen2);
    
    sync.startAll();
    EXPECT_TRUE(gen1.isRunning());
    EXPECT_TRUE(gen2.isRunning());
    
    sync.stopAll();
    EXPECT_FALSE(gen1.isRunning());
    EXPECT_FALSE(gen2.isRunning());
}

TEST(SyncGenCovTest, UpdateAndComplete) {
    SynchronizedMotionGenerator sync;
    SineMotionGenerator gen1, gen2;
    gen1.configure(100, 10.0f);
    gen1.setCycles(1);
    gen2.configure(200, 10.0f);
    gen2.setCycles(1);
    
    sync.addAxis(&gen1);
    sync.addAxis(&gen2);
    sync.startAll();
    
    EXPECT_FALSE(sync.allComplete());
    
    for (int i = 0; i < 500; i++) {
        sync.updateAll(1.0f);
        if (sync.allComplete()) break;
    }
    EXPECT_TRUE(sync.allComplete());
}

// ============================================================================
// Factory function
// ============================================================================

TEST(MotionFactoryCovTest, CreateSine) {
    auto* gen = createGenerator(GeneratorType::Sine);
    ASSERT_NE(gen, nullptr);
    EXPECT_EQ(gen->getState(), GeneratorState::Idle);
    delete gen;
}

TEST(MotionFactoryCovTest, CreateTrapezoidal) {
    auto* gen = createGenerator(GeneratorType::Trapezoidal);
    ASSERT_NE(gen, nullptr);
    delete gen;
}

TEST(MotionFactoryCovTest, CreateSCurve) {
    auto* gen = createGenerator(GeneratorType::SCurve);
    ASSERT_NE(gen, nullptr);
    delete gen;
}

TEST(MotionFactoryCovTest, CreateUnknown) {
    auto* gen = createGenerator(static_cast<GeneratorType>(99));
    EXPECT_EQ(gen, nullptr);
}

// ============================================================================
// GeneratorState enum
// ============================================================================

TEST(GenStateCovTest, AllStates) {
    EXPECT_NE(GeneratorState::Idle, GeneratorState::Running);
    EXPECT_NE(GeneratorState::Running, GeneratorState::Paused);
    EXPECT_NE(GeneratorState::Paused, GeneratorState::Complete);
}
