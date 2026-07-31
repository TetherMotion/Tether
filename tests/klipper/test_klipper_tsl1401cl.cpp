/**
 * @file test_klipper_tsl1401cl.cpp
 * @brief Tests for TSL1401CL filament width sensor config parsing and creation.
 */

#include "tether/klipper/klippy/KlippyInstance.hpp"
#include "tether/klipper/klippy/KlippyInstanceConfig.hpp"
#include "tether/klipper/objects/Peripherals.hpp"

#include <gtest/gtest.h>
#include <filesystem>
#include <fstream>
#include <string>
#include <cmath>

namespace klippy = tether::klipper::klippy;
namespace objects = tether::klipper::objects;

static std::string uniqueSocketPathTsl() {
    return "/tmp/tether_test_tsl_" + std::to_string(getpid()) + ".sock";
}

static std::string createTempConfigTsl(const std::string& content) {
    static int counter = 0;
    std::string path = "/tmp/tether_test_tsl_cfg_" + std::to_string(getpid()) +
                       "_" + std::to_string(counter++) + ".cfg";
    std::ofstream f(path);
    f << content;
    f.close();
    return path;
}

TEST(Tsl1401clConfigTest, ConfigParsedFromSection) {
    std::string cfgPath = createTempConfigTsl(R"(
[printer]
kinematics = cartesian
max_velocity = 300
max_accel = 2000
[tsl1401cl_filament_width_sensor]
sensor_pin = PA7
nominal_width = 1.75
tolerance = 0.05
min_width = 1.5
max_width = 2.0
pixel_count = 128
pixel_spacing = 0.1
)");
    klippy::KlippyInstanceConfig cfg;
    cfg.udsConfig.socketPath = uniqueSocketPathTsl();
    cfg.settingsPath = "/tmp/tether_test_tsl_settings.cfg";
    auto instance = std::make_unique<klippy::KlippyInstance>(cfg);
    ASSERT_TRUE(instance->loadConfig(cfgPath));

    auto& s = instance->settings();
    EXPECT_TRUE(s.tsl1401clConfig.configured);
    EXPECT_EQ(s.tsl1401clConfig.sensorPin, "PA7");
    EXPECT_NEAR(s.tsl1401clConfig.nominalWidth, 1.75, 1e-9);
    EXPECT_NEAR(s.tsl1401clConfig.tolerance, 0.05, 1e-9);
    EXPECT_NEAR(s.tsl1401clConfig.minWidth, 1.5, 1e-9);
    EXPECT_NEAR(s.tsl1401clConfig.maxWidth, 2.0, 1e-9);
    EXPECT_EQ(s.tsl1401clConfig.pixelCount, 128);
    EXPECT_NEAR(s.tsl1401clConfig.pixelSpacing, 0.1, 1e-9);

    std::filesystem::remove(cfgPath);
    std::filesystem::remove(cfg.settingsPath);
}

TEST(Tsl1401clConfigTest, SensorReadsWithAdcCallback) {
    // Test the Tsl1401clFilamentSensor class directly
    objects::Tsl1401clFilamentSensor::Params params;
    params.nominalWidth = 1.75;
    params.tolerance = 0.1;
    params.minWidth = 1.5;
    params.maxWidth = 2.0;
    params.pixelCount = 128;
    params.pixelSpacing = 0.1;

    // Simulate ADC value that gives ~1.75mm width
    // 1.75mm / 0.1mm_per_pixel = 17.5 pixels
    // 17.5 / 128 * 4095 = ~560
    double targetAdc = 17.5 / 128.0 * 4095.0;
    auto adcRead = [targetAdc]() { return targetAdc; };

    objects::Tsl1401clFilamentSensor sensor(0, params, adcRead);
    double width = sensor.readWidth();
    EXPECT_FALSE(std::isnan(width));
    EXPECT_NEAR(width, 1.7, 0.2); // Should be close to 1.7-1.8mm
}

TEST(Tsl1401clConfigTest, WidthOkCheck) {
    objects::Tsl1401clFilamentSensor::Params params;
    params.nominalWidth = 1.75;
    params.tolerance = 0.1;
    params.pixelCount = 128;
    params.pixelSpacing = 0.1;

    // ADC that gives exactly nominal width
    double nominalAdc = 17.5 / 128.0 * 4095.0;
    objects::Tsl1401clFilamentSensor sensor(0, params, [nominalAdc]() { return nominalAdc; });
    EXPECT_TRUE(sensor.widthOk());

    // ADC that gives out-of-tolerance width
    double wideAdc = 25.0 / 128.0 * 4095.0; // ~2.5mm — outside max
    objects::Tsl1401clFilamentSensor sensor2(0, params, [wideAdc]() { return wideAdc; });
    double w = sensor2.readWidth();
    // 2.5mm is above maxWidth=2.0, so readWidth should return NaN
    EXPECT_TRUE(std::isnan(w));
    EXPECT_FALSE(sensor2.widthOk());
}

TEST(Tsl1401clConfigTest, WidthErrorCalculation) {
    objects::Tsl1401clFilamentSensor::Params params;
    params.nominalWidth = 1.75;
    params.tolerance = 0.1;
    params.pixelCount = 128;
    params.pixelSpacing = 0.1;

    // ADC giving 18 pixels = 1.8mm, error = 0.05mm
    double adc = 18.0 / 128.0 * 4095.0;
    objects::Tsl1401clFilamentSensor sensor(0, params, [adc]() { return adc; });
    double err = sensor.widthError();
    EXPECT_FALSE(std::isnan(err));
    EXPECT_NEAR(err, 0.05, 0.06); // 1.8 - 1.75 = 0.05, with rounding tolerance
}
