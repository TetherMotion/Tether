/**
 * @file test_klipper_system_stats.cpp
 * @brief Tests for injectable system stats provider.
 */

#include <gtest/gtest.h>
#include "tether/klipper/klippy/SystemStatsProvider.hpp"
#include "tether/klipper/klippy/KlippyInstance.hpp"

using namespace tether::klipper::klippy;

TEST(SystemStatsProviderTest, MockProviderReturnsFixedValues) {
    SystemStatsSnapshot snap;
    snap.sysload = 1.5;
    snap.memAvailable = 2048.0;
    snap.mcuMcups = 42;
    MockSystemStatsProvider provider(snap);

    auto result = provider.readStats();
    EXPECT_EQ(result.sysload, 1.5);
    EXPECT_EQ(result.memAvailable, 2048.0);
    EXPECT_EQ(result.mcuMcups, 42u);
}

TEST(SystemStatsProviderTest, MockProviderSetStats) {
    MockSystemStatsProvider provider;
    EXPECT_EQ(provider.readStats().sysload, 0.0);

    SystemStatsSnapshot snap;
    snap.sysload = 3.14;
    provider.setStats(snap);
    EXPECT_EQ(provider.readStats().sysload, 3.14);
}

TEST(SystemStatsProviderTest, LinuxProviderReturnsValidData) {
    LinuxSystemStatsProvider provider;
    auto snap = provider.readStats();
    // On Linux, /proc/loadavg should be readable. sysload should be >= 0.
    EXPECT_GE(snap.sysload, 0.0);
    // memAvailable should be non-negative.
    EXPECT_GE(snap.memAvailable, 0.0);
}

TEST(SystemStatsProviderTest, KlippyInstanceUsesMockProvider) {
    KlippyInstance instance;

    auto mock = std::make_shared<MockSystemStatsProvider>();
    SystemStatsSnapshot snap;
    snap.sysload = 2.5;
    snap.memAvailable = 4096.0;
    mock->setStats(snap);

    instance.setSystemStatsProvider(mock);
    instance.updateSystemStats();

    // Verify the system stats object received the mock values.
    auto& stats = instance.systemStatsObject();
    ASSERT_TRUE(stats);
    EXPECT_EQ(stats->sysload(), 2.5);
}

TEST(SystemStatsProviderTest, KlippyInstanceUsesLinuxProviderByDefault) {
    KlippyInstance instance;
    instance.updateSystemStats();
    // Should not crash and should set some values from /proc.
    auto& stats = instance.systemStatsObject();
    ASSERT_TRUE(stats);
    // sysload should be non-negative (read from /proc/loadavg).
    EXPECT_GE(stats->sysload(), 0.0);
}

TEST(SystemStatsProviderTest, KlippyInstanceCanResetProvider) {
    KlippyInstance instance;

    auto mock = std::make_shared<MockSystemStatsProvider>();
    mock->setStats({.sysload = 9.99});
    instance.setSystemStatsProvider(mock);
    instance.updateSystemStats();
    EXPECT_EQ(instance.systemStatsObject()->sysload(), 9.99);

    // Reset to default (Linux) provider.
    instance.setSystemStatsProvider(nullptr);
    instance.updateSystemStats();
    // Should now read from /proc again.
    EXPECT_GE(instance.systemStatsObject()->sysload(), 0.0);
}
