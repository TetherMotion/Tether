/**
 * @file test_klipper_motion_dispatcher.cpp
 * @brief Direct unit tests for MotionDispatcher.
 */

#include <gtest/gtest.h>
#include "tether/klipper/motion/MotionDispatcher.hpp"
#include "tether/klipper/motion/MotionTranslator.hpp"

#include <array>
#include <atomic>

using namespace tether::klipper;

class MotionDispatcherTest : public ::testing::Test {
protected:
    void SetUp() override {
        motion::MotionDispatcher::Config cfg;
        cfg.clockFreqHz = 180000000;
        cfg.sampleIntervalSec = 0.0001;
        for (int i = 0; i < 4; ++i) {
            cfg.axisConfigs[i].stepsPerMm = 80.0;
            cfg.axisConfigs[i].invertDirection = false;
            cfg.axisOids[i] = static_cast<uint8_t>(i);
        }
        dispatcher_ = std::make_unique<motion::MotionDispatcher>(cfg);
    }

    std::unique_ptr<motion::MotionDispatcher> dispatcher_;
};

TEST_F(MotionDispatcherTest, DefaultPosition) {
    auto pos = dispatcher_->position();
    EXPECT_EQ(pos[0], 0.0);
    EXPECT_EQ(pos[1], 0.0);
    EXPECT_EQ(pos[2], 0.0);
    EXPECT_EQ(pos[3], 0.0);
}

TEST_F(MotionDispatcherTest, SetPosition) {
    dispatcher_->setPosition({10.0, 20.0, 30.0, 40.0});
    auto pos = dispatcher_->position();
    EXPECT_EQ(pos[0], 10.0);
    EXPECT_EQ(pos[1], 20.0);
    EXPECT_EQ(pos[2], 30.0);
    EXPECT_EQ(pos[3], 40.0);
}

TEST_F(MotionDispatcherTest, SetSendCallback) {
    std::atomic<size_t> callCount{0};
    dispatcher_->setSendCallback([&callCount](const std::vector<motion::AxisStepSequence>&) {
        callCount++;
        return 0;
    });
    dispatcher_->setClockProvider([]() { return 0u; });
    dispatcher_->move(10.0, 0.0, 0.0, 0.0, 50.0);
    // Position should be updated to target after move.
    auto pos = dispatcher_->position();
    EXPECT_NEAR(pos[0], 10.0, 0.01);
}

TEST_F(MotionDispatcherTest, SetClockProvider) {
    std::atomic<uint32_t> clockValue{1000};
    dispatcher_->setClockProvider([&clockValue]() { return clockValue.load(); });
    dispatcher_->setSendCallback([](const std::vector<motion::AxisStepSequence>&) { return 0; });
    dispatcher_->move(5.0, 0.0, 0.0, 0.0, 50.0);
    // Position should be updated.
    auto pos = dispatcher_->position();
    EXPECT_NEAR(pos[0], 5.0, 0.01);
}

TEST_F(MotionDispatcherTest, MoveUpdatesPosition) {
    std::atomic<size_t> callCount{0};
    dispatcher_->setSendCallback([&callCount](const std::vector<motion::AxisStepSequence>&) {
        callCount++;
        return 0;
    });
    dispatcher_->setClockProvider([]() { return 0u; });
    dispatcher_->move(10.0, 20.0, 30.0, 0.0, 50.0);
    auto pos = dispatcher_->position();
    // Position should be updated to target after move.
    EXPECT_NEAR(pos[0], 10.0, 0.01);
    EXPECT_NEAR(pos[1], 20.0, 0.01);
    EXPECT_NEAR(pos[2], 30.0, 0.01);
}

TEST_F(MotionDispatcherTest, MoveWithZeroSpeed) {
    dispatcher_->setSendCallback([](const std::vector<motion::AxisStepSequence>&) { return 0; });
    dispatcher_->setClockProvider([]() { return 0u; });
    size_t sent = dispatcher_->move(10.0, 0.0, 0.0, 0.0, 0.0);
    EXPECT_EQ(sent, 0u);
}

TEST_F(MotionDispatcherTest, SetKinematicsTransform) {
    motion::KinematicsTransform kt;
    kt.setKinematics(tether::kinematics::PrinterKinematics::CoreXY);
    dispatcher_->setKinematicsTransform(kt);
    // Verify a move still works with the transform set.
    std::atomic<size_t> callCount{0};
    dispatcher_->setSendCallback([&callCount](const std::vector<motion::AxisStepSequence>&) {
        callCount++;
        return 0;
    });
    dispatcher_->setClockProvider([]() { return 0u; });
    dispatcher_->move(5.0, 0.0, 0.0, 0.0, 50.0);
    // Position should be updated regardless of transform.
    auto pos = dispatcher_->position();
    EXPECT_NEAR(pos[0], 5.0, 0.01);
}

TEST_F(MotionDispatcherTest, MultipleMoves) {
    std::atomic<size_t> totalSent{0};
    dispatcher_->setSendCallback([&totalSent](const std::vector<motion::AxisStepSequence>& seqs) {
        totalSent += seqs.size();
        return seqs.size();
    });
    dispatcher_->setClockProvider([]() { return 0u; });
    dispatcher_->move(10.0, 0.0, 0.0, 0.0, 50.0);
    dispatcher_->move(10.0, 10.0, 0.0, 0.0, 50.0);
    dispatcher_->move(0.0, 10.0, 0.0, 0.0, 50.0);
    EXPECT_GT(totalSent.load(), 0u);
}
