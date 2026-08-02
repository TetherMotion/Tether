/**
 * @file test_klipper_multi_mcu_sync.cpp
 * @brief Direct unit tests for MultiMcuSync (MultiMcuManager & TrsyncManager).
 */

#include <gtest/gtest.h>
#include "tether/klipper/clock/MultiMcuSync.hpp"
#include "tether/klipper/clock/ClockSync.hpp"

#include <memory>

using namespace tether::klipper::clock;

class MultiMcuManagerTest : public ::testing::Test {
protected:
    void SetUp() override {
        manager_ = std::make_unique<MultiMcuManager>();
    }
    std::unique_ptr<MultiMcuManager> manager_;
};

TEST_F(MultiMcuManagerTest, InitiallyEmpty) {
    EXPECT_EQ(manager_->mcuCount(), 0u);
    EXPECT_TRUE(manager_->mcuIds().empty());
}

TEST_F(MultiMcuManagerTest, RegisterMcu) {
    auto cs = std::make_shared<ClockSync>();
    manager_->registerMcu("mcu0", 0, cs);
    EXPECT_EQ(manager_->mcuCount(), 1u);
    auto ids = manager_->mcuIds();
    EXPECT_EQ(ids.size(), 1u);
    EXPECT_EQ(ids[0], 0u);
}

TEST_F(MultiMcuManagerTest, RegisterMultipleMcus) {
    auto cs0 = std::make_shared<ClockSync>();
    auto cs1 = std::make_shared<ClockSync>();
    manager_->registerMcu("mcu0", 0, cs0);
    manager_->registerMcu("mcu1", 1, cs1);
    EXPECT_EQ(manager_->mcuCount(), 2u);
}

TEST_F(MultiMcuManagerTest, UnregisterMcu) {
    auto cs = std::make_shared<ClockSync>();
    manager_->registerMcu("mcu0", 0, cs);
    EXPECT_EQ(manager_->mcuCount(), 1u);
    manager_->unregisterMcu(0);
    EXPECT_EQ(manager_->mcuCount(), 0u);
}

TEST_F(MultiMcuManagerTest, GetMcu) {
    auto cs = std::make_shared<ClockSync>();
    manager_->registerMcu("mcu0", 0, cs);
    auto retrieved = manager_->getMcu(0);
    EXPECT_EQ(retrieved, cs);
}

TEST_F(MultiMcuManagerTest, GetNonexistentMcu) {
    auto retrieved = manager_->getMcu(99);
    EXPECT_EQ(retrieved, nullptr);
}

TEST_F(MultiMcuManagerTest, PrimaryMcuEmpty) {
    // No MCUs registered - primaryMcu behavior is implementation-defined
    // but should not crash.
    EXPECT_NO_THROW(manager_->primaryMcu());
}

TEST_F(MultiMcuManagerTest, PrimaryMcuSingle) {
    auto cs = std::make_shared<ClockSync>();
    manager_->registerMcu("mcu0", 0, cs);
    EXPECT_EQ(manager_->primaryMcu(), 0u);
}

TEST_F(MultiMcuManagerTest, HostToMcu) {
    auto cs = std::make_shared<ClockSync>();
    manager_->registerMcu("mcu0", 0, cs);
    // Without sync, hostToMcu returns 0 (no samples to extrapolate from).
    uint32_t result = manager_->hostToMcu(0, HostClock::now());
    EXPECT_EQ(result, 0u);
}

// --- TrsyncManager tests ---

class TrsyncManagerTest : public ::testing::Test {
protected:
    void SetUp() override {
        trsync_ = std::make_unique<TrsyncManager>();
    }
    std::unique_ptr<TrsyncManager> trsync_;
};

TEST_F(TrsyncManagerTest, InitiallyInactive) {
    EXPECT_FALSE(trsync_->isActive());
    EXPECT_FALSE(trsync_->isTriggered());
}

TEST_F(TrsyncManagerTest, Start) {
    trsync_->start(180000000);
    EXPECT_TRUE(trsync_->isActive());
    EXPECT_FALSE(trsync_->isTriggered());
}

TEST_F(TrsyncManagerTest, ReportTrigger) {
    trsync_->start(180000000);
    trsync_->reportTrigger(0, 1000000);
    EXPECT_TRUE(trsync_->isTriggered());
    EXPECT_EQ(trsync_->triggerMcu(), 0u);
    EXPECT_EQ(trsync_->triggerClock(), 1000000u);
}

TEST_F(TrsyncManagerTest, End) {
    trsync_->start(180000000);
    trsync_->end();
    EXPECT_FALSE(trsync_->isActive());
}

TEST_F(TrsyncManagerTest, MultipleTriggersUseFirst) {
    trsync_->start(180000000);
    trsync_->reportTrigger(0, 1000000);
    trsync_->reportTrigger(1, 2000000);
    // First trigger should be retained.
    EXPECT_EQ(trsync_->triggerMcu(), 0u);
    EXPECT_EQ(trsync_->triggerClock(), 1000000u);
}

TEST_F(TrsyncManagerTest, TriggerBeforeStart) {
    trsync_->reportTrigger(0, 1000000);
    EXPECT_FALSE(trsync_->isTriggered());
}

TEST_F(TrsyncManagerTest, EndClearsActive) {
    trsync_->start(180000000);
    trsync_->reportTrigger(0, 1000000);
    EXPECT_TRUE(trsync_->isTriggered());
    trsync_->end();
    EXPECT_FALSE(trsync_->isActive());
    // Note: triggered state may persist after end() - it is cleared on
    // the next start() call. This is by design so callers can query the
    // trigger info after the session ends.
}

TEST_F(TrsyncManagerTest, ReuseAfterEnd) {
    trsync_->start(180000000);
    trsync_->reportTrigger(0, 1000000);
    trsync_->end();
    trsync_->start(180000000);
    EXPECT_TRUE(trsync_->isActive());
    EXPECT_FALSE(trsync_->isTriggered());
}
