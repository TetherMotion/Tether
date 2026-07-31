/**
 * @file test_klipper_tmc_advanced.cpp
 * @brief Tests for TMC advanced features (stealthchop, spreadCycle, etc.)
 *        config parsing.
 */

#include "tether/klipper/klippy/KlippyInstance.hpp"
#include "tether/klipper/klippy/KlippyInstanceConfig.hpp"

#include <gtest/gtest.h>
#include <filesystem>
#include <fstream>
#include <string>

namespace klippy = tether::klipper::klippy;

static std::string uniqueSocketPathTmc() {
    return "/tmp/tether_test_tmc_" + std::to_string(getpid()) + ".sock";
}

static std::string createTempConfigTmc(const std::string& content) {
    static int counter = 0;
    std::string path = "/tmp/tether_test_tmc_cfg_" + std::to_string(getpid()) +
                       "_" + std::to_string(counter++) + ".cfg";
    std::ofstream f(path);
    f << content;
    f.close();
    return path;
}

TEST(TmcAdvancedConfigTest, StealthchopAndSpreadCycleParsed) {
    std::string cfgPath = createTempConfigTmc(R"(
[printer]
kinematics = cartesian
max_velocity = 300
max_accel = 2000
[tmc2209 stepper_x]
uart_pin = PA10
run_current = 0.8
hold_current = 0.5
stealthchop = 1
stealthchop_threshold = 100
spreadcycle_threshold = 50
interpolate = True
microsteps = 16
)");
    klippy::KlippyInstanceConfig cfg;
    cfg.udsConfig.socketPath = uniqueSocketPathTmc();
    cfg.settingsPath = "/tmp/tether_test_tmc_settings.cfg";
    auto instance = std::make_unique<klippy::KlippyInstance>(cfg);
    ASSERT_TRUE(instance->loadConfig(cfgPath));

    auto& s = instance->settings();
    auto it = s.tmcDrivers.find("stepper_x");
    ASSERT_NE(it, s.tmcDrivers.end());
    EXPECT_EQ(it->second.driverType, "tmc2209");
    EXPECT_TRUE(it->second.stealthchop);
    EXPECT_NEAR(it->second.stealthchopThreshold, 100.0, 1e-9);
    EXPECT_NEAR(it->second.spreadCycleThreshold, 50.0, 1e-9);
    EXPECT_TRUE(it->second.interpolate);
    EXPECT_EQ(it->second.microsteps, 16);

    std::filesystem::remove(cfgPath);
    std::filesystem::remove(cfg.settingsPath);
}

TEST(TmcAdvancedConfigTest, StallguardAndCoolstepParsed) {
    std::string cfgPath = createTempConfigTmc(R"(
[printer]
kinematics = cartesian
max_velocity = 300
max_accel = 2000
[tmc5160 stepper_y]
spi_bus = spi0
cs_pin = PA11
run_current = 1.2
stallguard = 1
stallguard_threshold = 0.5
coolstep_threshold = 30.0
chopper_timing = 2
)");
    klippy::KlippyInstanceConfig cfg;
    cfg.udsConfig.socketPath = uniqueSocketPathTmc();
    cfg.settingsPath = "/tmp/tether_test_tmc_settings2.cfg";
    auto instance = std::make_unique<klippy::KlippyInstance>(cfg);
    ASSERT_TRUE(instance->loadConfig(cfgPath));

    auto& s = instance->settings();
    auto it = s.tmcDrivers.find("stepper_y");
    ASSERT_NE(it, s.tmcDrivers.end());
    EXPECT_EQ(it->second.driverType, "tmc5160");
    EXPECT_TRUE(it->second.stallguard);
    EXPECT_NEAR(it->second.stallguardThreshold, 0.5, 1e-9);
    EXPECT_NEAR(it->second.coolstepThreshold, 30.0, 1e-9);
    EXPECT_EQ(it->second.chopperTiming, 2);
    EXPECT_FALSE(it->second.stealthchop); // Not set, should default to false

    std::filesystem::remove(cfgPath);
    std::filesystem::remove(cfg.settingsPath);
}

TEST(TmcAdvancedConfigTest, MultiHomingAndHomeCurrentParsed) {
    std::string cfgPath = createTempConfigTmc(R"(
[printer]
kinematics = cartesian
max_velocity = 300
max_accel = 2000
[tmc2209 stepper_z]
uart_pin = PA12
run_current = 0.6
multi_homing = 1
home_current = 0.3
)");
    klippy::KlippyInstanceConfig cfg;
    cfg.udsConfig.socketPath = uniqueSocketPathTmc();
    cfg.settingsPath = "/tmp/tether_test_tmc_settings3.cfg";
    auto instance = std::make_unique<klippy::KlippyInstance>(cfg);
    ASSERT_TRUE(instance->loadConfig(cfgPath));

    auto& s = instance->settings();
    auto it = s.tmcDrivers.find("stepper_z");
    ASSERT_NE(it, s.tmcDrivers.end());
    EXPECT_TRUE(it->second.multiHoming);
    EXPECT_NEAR(it->second.homeCurrent, 0.3, 1e-9);

    std::filesystem::remove(cfgPath);
    std::filesystem::remove(cfg.settingsPath);
}

TEST(TmcAdvancedConfigTest, DefaultsWhenNotSet) {
    std::string cfgPath = createTempConfigTmc(R"(
[printer]
kinematics = cartesian
max_velocity = 300
max_accel = 2000
[tmc2209 stepper_e]
uart_pin = PA13
run_current = 0.7
)");
    klippy::KlippyInstanceConfig cfg;
    cfg.udsConfig.socketPath = uniqueSocketPathTmc();
    cfg.settingsPath = "/tmp/tether_test_tmc_settings4.cfg";
    auto instance = std::make_unique<klippy::KlippyInstance>(cfg);
    ASSERT_TRUE(instance->loadConfig(cfgPath));

    auto& s = instance->settings();
    auto it = s.tmcDrivers.find("stepper_e");
    ASSERT_NE(it, s.tmcDrivers.end());
    // Advanced features should all be at defaults
    EXPECT_FALSE(it->second.stealthchop);
    EXPECT_NEAR(it->second.spreadCycleThreshold, 0.0, 1e-9);
    EXPECT_EQ(it->second.chopperTiming, 0);
    EXPECT_FALSE(it->second.stallguard);
    EXPECT_NEAR(it->second.stallguardThreshold, 0.0, 1e-9);
    EXPECT_EQ(it->second.microsteps, 0);
    EXPECT_FALSE(it->second.multiHoming);
    EXPECT_NEAR(it->second.homeCurrent, 0.0, 1e-9);

    std::filesystem::remove(cfgPath);
    std::filesystem::remove(cfg.settingsPath);
}
