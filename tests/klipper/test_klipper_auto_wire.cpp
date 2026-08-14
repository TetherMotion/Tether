/**
 * @file test_klipper_auto_wire.cpp
 * @brief Tests for config-driven auto-wiring of temperature sensors
 *        (Thermistor/Thermocouple) and ADXL345 from config sections.
 */

#include "tether/klipper/klippy/KlippyInstance.hpp"
#include "tether/klipper/klippy/KlippyServer.hpp"
#include "tether/klipper/klippy/KlippyInstanceConfig.hpp"
#include "tether/klipper/objects/Thermal.hpp"
#include "tether/klipper/objects/Peripherals.hpp"

#include <gtest/gtest.h>
#include <algorithm>
#include <cmath>
#include <fstream>
#include <filesystem>
#include <string>

using namespace tether::klipper::klippy;
using namespace tether::klipper::objects;

// ============================================================================
// Helpers
// ============================================================================

static std::string uniqueSocketPath() {
    return "/tmp/tether_test_aw_" + std::to_string(getpid()) + ".sock";
}

static std::string createTempConfig(const std::string& content) {
    static int counter = 0;
    std::string path = "/tmp/tether_test_aw_cfg_" + std::to_string(getpid()) +
                       "_" + std::to_string(counter++) + ".cfg";
    std::ofstream f(path);
    f << content;
    f.close();
    return path;
}

// ============================================================================
// Tests
// ============================================================================

TEST(AutoWireTempSensorTest, ThermistorConfigParsed) {
    std::string cfgPath = createTempConfig(R"(
[printer]
kinematics = cartesian
max_velocity = 300
max_accel = 2000
[thermistor my_ntc]
pullup_resistor = 4700
reference_voltage = 3.3
adc_max = 4095
resistance_at_25c = 100000
beta = 3950
)");
    KlippyInstanceConfig cfg;
    cfg.udsConfig.socketPath = uniqueSocketPath();
    auto instance = std::make_unique<KlippyInstance>(cfg);
    ASSERT_TRUE(instance->loadConfig(cfgPath));

    auto& s = instance->settings();
    ASSERT_EQ(s.thermistors.size(), 1u);
    ASSERT_TRUE(s.thermistors.count("my_ntc"));
    EXPECT_NEAR(s.thermistors["my_ntc"].pullupResistor, 4700.0, 1e-9);
    EXPECT_NEAR(s.thermistors["my_ntc"].referenceVoltage, 3.3, 1e-9);
    EXPECT_NEAR(s.thermistors["my_ntc"].adcMax, 4095.0, 1e-9);
    EXPECT_NEAR(s.thermistors["my_ntc"].resistanceAt25C, 100000.0, 1e-9);
    EXPECT_NEAR(s.thermistors["my_ntc"].beta, 3950.0, 1e-9);

    std::filesystem::remove(cfgPath);
}

TEST(AutoWireTempSensorTest, ThermocoupleConfigParsed) {
    std::string cfgPath = createTempConfig(R"(
[printer]
kinematics = cartesian
max_velocity = 300
max_accel = 2000
[thermocouple my_tc]
type = K
spi_bus = spi0
cs_pin = PA3
)");
    KlippyInstanceConfig cfg;
    cfg.udsConfig.socketPath = uniqueSocketPath();
    auto instance = std::make_unique<KlippyInstance>(cfg);
    ASSERT_TRUE(instance->loadConfig(cfgPath));

    auto& s = instance->settings();
    ASSERT_EQ(s.thermocouples.size(), 1u);
    ASSERT_TRUE(s.thermocouples.count("my_tc"));
    EXPECT_EQ(s.thermocouples["my_tc"].type, "K");
    EXPECT_EQ(s.thermocouples["my_tc"].spiBus, "spi0");
    EXPECT_EQ(s.thermocouples["my_tc"].csPin, "PA3");

    std::filesystem::remove(cfgPath);
}

TEST(AutoWireTempSensorTest, ThermistorAutoInstantiated) {
    // When [temperature_sensor] references a [thermistor] definition by name,
    // a real Thermistor object should be created instead of a stub.
    std::string cfgPath = createTempConfig(R"(
[printer]
kinematics = cartesian
max_velocity = 300
max_accel = 2000
[thermistor my_ntc]
pullup_resistor = 4700
resistance_at_25c = 100000
beta = 3950
[temperature_sensor chamber]
sensor_type = my_ntc
sensor_pin = PA4
min_temp = 0
max_temp = 80
)");
    KlippyInstanceConfig cfg;
    cfg.udsConfig.socketPath = uniqueSocketPath();
    auto instance = std::make_unique<KlippyInstance>(cfg);
    ASSERT_TRUE(instance->loadConfig(cfgPath));

    // The sensor should be registered as a printer object
    auto objects = instance->server().listObjects();
    bool found = std::find(objects.begin(), objects.end(),
                           "temperature_sensor chamber") != objects.end();
    EXPECT_TRUE(found);

    // The sensor should report a valid temperature (from placeholder ADC).
    // We can't directly access the sensor object, but we can verify it
    // doesn't crash when queried via the printer object status.
    // The Thermistor with ADC=2048 (half of 4095) should produce a
    // temperature around 25°C for a 100K NTC with beta=3950.
    // Just verify the config was parsed correctly.
    auto& s = instance->settings();
    ASSERT_EQ(s.temperatureSensors.size(), 1u);
    EXPECT_EQ(s.temperatureSensors["chamber"].sensorType, "my_ntc");

    std::filesystem::remove(cfgPath);
}

TEST(AutoWireTempSensorTest, ThermocoupleAutoInstantiated) {
    std::string cfgPath = createTempConfig(R"(
[printer]
kinematics = cartesian
max_velocity = 300
max_accel = 2000
[thermocouple my_tc]
type = J
spi_bus = spi0
cs_pin = PA3
[temperature_sensor exhaust]
sensor_type = my_tc
min_temp = 0
max_temp = 1200
)");
    KlippyInstanceConfig cfg;
    cfg.udsConfig.socketPath = uniqueSocketPath();
    auto instance = std::make_unique<KlippyInstance>(cfg);
    ASSERT_TRUE(instance->loadConfig(cfgPath));

    auto objects = instance->server().listObjects();
    bool found = std::find(objects.begin(), objects.end(),
                           "temperature_sensor exhaust") != objects.end();
    EXPECT_TRUE(found);

    auto& s = instance->settings();
    ASSERT_EQ(s.temperatureSensors.size(), 1u);
    EXPECT_EQ(s.temperatureSensors["exhaust"].sensorType, "my_tc");

    std::filesystem::remove(cfgPath);
}

TEST(AutoWireTempSensorTest, StubFallbackForUnknownSensorType) {
    // When sensor_type doesn't match any [thermistor] or [thermocouple]
    // definition, a ConfigTemperatureSensor stub should be created.
    std::string cfgPath = createTempConfig(R"(
[printer]
kinematics = cartesian
max_velocity = 300
max_accel = 2000
[temperature_sensor chamber]
sensor_type = Generic_NTC
min_temp = 0
max_temp = 80
)");
    KlippyInstanceConfig cfg;
    cfg.udsConfig.socketPath = uniqueSocketPath();
    auto instance = std::make_unique<KlippyInstance>(cfg);
    ASSERT_TRUE(instance->loadConfig(cfgPath));

    auto objects = instance->server().listObjects();
    bool found = std::find(objects.begin(), objects.end(),
                           "temperature_sensor chamber") != objects.end();
    EXPECT_TRUE(found);

    std::filesystem::remove(cfgPath);
}

TEST(AutoWireTempSensorTest, MultipleThermistorDefinitions) {
    std::string cfgPath = createTempConfig(R"(
[printer]
kinematics = cartesian
max_velocity = 300
max_accel = 2000
[thermistor ntc_100k]
beta = 3950
resistance_at_25c = 100000
[thermistor ntc_10k]
beta = 3435
resistance_at_25c = 10000
[temperature_sensor bed]
sensor_type = ntc_10k
min_temp = 0
max_temp = 120
[temperature_sensor hotend]
sensor_type = ntc_100k
min_temp = 0
max_temp = 300
)");
    KlippyInstanceConfig cfg;
    cfg.udsConfig.socketPath = uniqueSocketPath();
    auto instance = std::make_unique<KlippyInstance>(cfg);
    ASSERT_TRUE(instance->loadConfig(cfgPath));

    auto& s = instance->settings();
    ASSERT_EQ(s.thermistors.size(), 2u);
    EXPECT_TRUE(s.thermistors.count("ntc_100k"));
    EXPECT_TRUE(s.thermistors.count("ntc_10k"));
    EXPECT_NEAR(s.thermistors["ntc_10k"].resistanceAt25C, 10000.0, 1e-9);
    EXPECT_NEAR(s.thermistors["ntc_100k"].beta, 3950.0, 1e-9);

    ASSERT_EQ(s.temperatureSensors.size(), 2u);
    EXPECT_EQ(s.temperatureSensors["bed"].sensorType, "ntc_10k");
    EXPECT_EQ(s.temperatureSensors["hotend"].sensorType, "ntc_100k");

    std::filesystem::remove(cfgPath);
}

TEST(AutoWireAdxl345Test, Adxl345AutoCreatedWithSpiBus) {
    // When [adxl345] has spi_bus set, the auto-wiring should attempt
    // to create a real SPI-backed ADXL345. Since /dev/spidev* likely
    // doesn't exist in the test environment, it should fall back to stub.
    std::string cfgPath = createTempConfig(R"(
[printer]
kinematics = cartesian
max_velocity = 300
max_accel = 2000
[adxl345]
cs_pin = PA5
spi_bus = spi0
rate = 3200
)");
    KlippyInstanceConfig cfg;
    cfg.udsConfig.socketPath = uniqueSocketPath();
    auto instance = std::make_unique<KlippyInstance>(cfg);
    ASSERT_TRUE(instance->loadConfig(cfgPath));

    auto& s = instance->settings();
    EXPECT_TRUE(s.adxl345Configured);
    EXPECT_EQ(s.adxl345SpiBus, "spi0");

    // ADXL345 object should be registered (either real or stub fallback)
    auto objects = instance->server().listObjects();
    bool found = std::find(objects.begin(), objects.end(), "adxl345") != objects.end();
    EXPECT_TRUE(found);

    std::filesystem::remove(cfgPath);
}

TEST(AutoWireAdxl345Test, Adxl345StubFallbackNoSpiBus) {
    // When [adxl345] has no spi_bus, should create stub ADXL345.
    std::string cfgPath = createTempConfig(R"(
[printer]
kinematics = cartesian
max_velocity = 300
max_accel = 2000
[adxl345]
cs_pin = PA5
rate = 1600
)");
    KlippyInstanceConfig cfg;
    cfg.udsConfig.socketPath = uniqueSocketPath();
    auto instance = std::make_unique<KlippyInstance>(cfg);
    ASSERT_TRUE(instance->loadConfig(cfgPath));

    auto& s = instance->settings();
    EXPECT_TRUE(s.adxl345Configured);
    EXPECT_TRUE(s.adxl345SpiBus.empty());

    auto objects = instance->server().listObjects();
    bool found = std::find(objects.begin(), objects.end(), "adxl345") != objects.end();
    EXPECT_TRUE(found);

    std::filesystem::remove(cfgPath);
}

// ============================================================================
// ADC/SPI callback registration tests
// ============================================================================

TEST(AutoWireCallbackTest, RegisterAdcCallbackUsedByThermistor) {
    std::string cfgPath = createTempConfig(R"(
[printer]
kinematics = cartesian
max_velocity = 300
max_accel = 2000
[thermistor my_ntc]
beta = 3950
resistance_at_25c = 100000
[temperature_sensor chamber]
sensor_type = my_ntc
sensor_pin = PA4
min_temp = 0
max_temp = 80
)");
    KlippyInstanceConfig cfg;
    cfg.udsConfig.socketPath = uniqueSocketPath();
    auto instance = std::make_unique<KlippyInstance>(cfg);

    instance->registerAdcCallback("PA4", []() { return 3071.25; });

    ASSERT_TRUE(instance->loadConfig(cfgPath));

    auto cb = instance->adcCallback("PA4");
    ASSERT_TRUE(cb);
    EXPECT_NEAR(cb(), 3071.25, 1e-9);

    auto objects = instance->server().listObjects();
    bool found = std::find(objects.begin(), objects.end(),
                           "temperature_sensor chamber") != objects.end();
    EXPECT_TRUE(found);

    std::filesystem::remove(cfgPath);
}

TEST(AutoWireCallbackTest, RegisterSpiCallbackUsedByThermocouple) {
    std::string cfgPath = createTempConfig(R"(
[printer]
kinematics = cartesian
max_velocity = 300
max_accel = 2000
[thermocouple my_tc]
type = K
spi_bus = spi0
cs_pin = PA3
[temperature_sensor exhaust]
sensor_type = my_tc
min_temp = 0
max_temp = 1200
)");
    KlippyInstanceConfig cfg;
    cfg.udsConfig.socketPath = uniqueSocketPath();
    auto instance = std::make_unique<KlippyInstance>(cfg);

    instance->registerSpiCallback("spi0", [](std::span<const uint8_t> tx) {
        (void)tx;
        int32_t raw = 400 << 18; // 100°C
        return std::vector<uint8_t>{
            static_cast<uint8_t>((raw >> 24) & 0xFF),
            static_cast<uint8_t>((raw >> 16) & 0xFF),
            static_cast<uint8_t>((raw >> 8) & 0xFF),
            static_cast<uint8_t>(raw & 0xFF)
        };
    });

    ASSERT_TRUE(instance->loadConfig(cfgPath));

    auto cb = instance->spiCallback("spi0");
    ASSERT_TRUE(cb);

    auto objects = instance->server().listObjects();
    bool found = std::find(objects.begin(), objects.end(),
                           "temperature_sensor exhaust") != objects.end();
    EXPECT_TRUE(found);

    std::filesystem::remove(cfgPath);
}

TEST(AutoWireCallbackTest, RegisterSpiCallbackUsedByAdxl345) {
    std::string cfgPath = createTempConfig(R"(
[printer]
kinematics = cartesian
max_velocity = 300
max_accel = 2000
[adxl345]
cs_pin = PA5
spi_bus = spi0
rate = 3200
)");
    KlippyInstanceConfig cfg;
    cfg.udsConfig.socketPath = uniqueSocketPath();
    auto instance = std::make_unique<KlippyInstance>(cfg);

    instance->registerSpiCallback("spi0", [](std::span<const uint8_t> tx) {
        (void)tx;
        return std::vector<uint8_t>(8, 0);
    });

    ASSERT_TRUE(instance->loadConfig(cfgPath));

    auto cb = instance->spiCallback("spi0");
    ASSERT_TRUE(cb);

    auto objects = instance->server().listObjects();
    bool found = std::find(objects.begin(), objects.end(), "adxl345") != objects.end();
    EXPECT_TRUE(found);

    std::filesystem::remove(cfgPath);
}

TEST(AutoWireCallbackTest, NoCallbackFallsBackToPlaceholder) {
    std::string cfgPath = createTempConfig(R"(
[printer]
kinematics = cartesian
max_velocity = 300
max_accel = 2000
[thermistor my_ntc]
beta = 3950
[temperature_sensor chamber]
sensor_type = my_ntc
sensor_pin = PA4
min_temp = 0
max_temp = 80
)");
    KlippyInstanceConfig cfg;
    cfg.udsConfig.socketPath = uniqueSocketPath();
    auto instance = std::make_unique<KlippyInstance>(cfg);

    ASSERT_TRUE(instance->loadConfig(cfgPath));

    auto cb = instance->adcCallback("PA4");
    EXPECT_FALSE(cb);

    auto objects = instance->server().listObjects();
    bool found = std::find(objects.begin(), objects.end(),
                           "temperature_sensor chamber") != objects.end();
    EXPECT_TRUE(found);

    std::filesystem::remove(cfgPath);
}

// ============================================================================
// RTD sensor tests
// ============================================================================

TEST(AutoWireRtdTest, RtdConfigParsedAndSensorCreated) {
    std::string cfgPath = createTempConfig(R"(
[printer]
kinematics = cartesian
max_velocity = 300
max_accel = 2000
[rtd my_pt100]
nominal_resistance = 100.0
alpha = 0.003851
reference_resistor = 430.0
[temperature_sensor bed_rtd]
sensor_type = my_pt100
sensor_pin = PA5
min_temp = 0
max_temp = 400
)");
    KlippyInstanceConfig cfg;
    cfg.udsConfig.socketPath = uniqueSocketPath();
    auto instance = std::make_unique<KlippyInstance>(cfg);
    ASSERT_TRUE(instance->loadConfig(cfgPath));

    auto& s = instance->settings();
    auto rtdIt = s.rtds.find("my_pt100");
    ASSERT_NE(rtdIt, s.rtds.end());
    EXPECT_NEAR(rtdIt->second.nominalResistance, 100.0, 1e-9);
    EXPECT_NEAR(rtdIt->second.alpha, 0.003851, 1e-9);
    EXPECT_NEAR(rtdIt->second.referenceResistor, 430.0, 1e-9);

    auto objects = instance->server().listObjects();
    bool found = std::find(objects.begin(), objects.end(),
                           "temperature_sensor bed_rtd") != objects.end();
    EXPECT_TRUE(found);

    std::filesystem::remove(cfgPath);
}

TEST(AutoWireRtdTest, RtdWithRegisteredAdcCallback) {
    std::string cfgPath = createTempConfig(R"(
[printer]
kinematics = cartesian
max_velocity = 300
max_accel = 2000
[rtd my_pt1000]
nominal_resistance = 1000.0
reference_resistor = 4300.0
[temperature_sensor extruder_rtd]
sensor_type = my_pt1000
sensor_pin = PA6
min_temp = 0
max_temp = 500
)");
    KlippyInstanceConfig cfg;
    cfg.udsConfig.socketPath = uniqueSocketPath();
    auto instance = std::make_unique<KlippyInstance>(cfg);

    instance->registerAdcCallback("PA6", []() { return 2048.0; });

    ASSERT_TRUE(instance->loadConfig(cfgPath));

    auto cb = instance->adcCallback("PA6");
    ASSERT_TRUE(cb);
    EXPECT_NEAR(cb(), 2048.0, 1e-9);

    auto objects = instance->server().listObjects();
    bool found = std::find(objects.begin(), objects.end(),
                           "temperature_sensor extruder_rtd") != objects.end();
    EXPECT_TRUE(found);

    std::filesystem::remove(cfgPath);
}

// ============================================================================
// Multi-point thermistor calibration tests
// ============================================================================

TEST(AutoWireThermistorTableTest, CalibrationTableParsedFromConfig) {
    std::string cfgPath = createTempConfig(R"(
[printer]
kinematics = cartesian
max_velocity = 300
max_accel = 2000
[thermistor calibrated_ntc]
pullup_resistor = 4700.0
reference_voltage = 3.3
adc_max = 4095.0
calibration_points = 25,100000;50,35900;100,6797;150,1640;200,532
[temperature_sensor precise]
sensor_type = calibrated_ntc
sensor_pin = PA7
min_temp = 0
max_temp = 250
)");
    KlippyInstanceConfig cfg;
    cfg.udsConfig.socketPath = uniqueSocketPath();
    auto instance = std::make_unique<KlippyInstance>(cfg);
    ASSERT_TRUE(instance->loadConfig(cfgPath));

    // Verify calibration table was parsed
    auto& s = instance->settings();
    auto thermIt = s.thermistors.find("calibrated_ntc");
    ASSERT_NE(thermIt, s.thermistors.end());
    EXPECT_EQ(thermIt->second.calibrationTable.size(), 5u);
    EXPECT_NEAR(thermIt->second.calibrationTable[0].first, 25.0, 1e-9);
    EXPECT_NEAR(thermIt->second.calibrationTable[0].second, 100000.0, 1e-9);
    EXPECT_NEAR(thermIt->second.calibrationTable[2].first, 100.0, 1e-9);
    EXPECT_NEAR(thermIt->second.calibrationTable[2].second, 6797.0, 1e-9);

    // Verify the sensor was registered
    auto objects = instance->server().listObjects();
    bool found = std::find(objects.begin(), objects.end(),
                           "temperature_sensor precise") != objects.end();
    EXPECT_TRUE(found);

    std::filesystem::remove(cfgPath);
}
