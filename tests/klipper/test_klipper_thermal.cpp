/**
 * @file test_klipper_thermal.cpp
 * @brief Tests for temperature control peripherals.
 */

#include "tether/klipper/objects/Thermal.hpp"

#include <gtest/gtest.h>
#include <cmath>

using namespace tether::klipper::objects;

// ============================================================================
// Thermistor tests
// ============================================================================

TEST(KlipperThermistor, ReadAt25C) {
    // At 25°C, thermistor resistance = resistanceAt25C
    // ADC value = V_ref * R_therm / (R_pullup + R_therm)
    // = 3.3 * 100000 / (4700 + 100000) = 3.3 * 0.9552 = 3.152
    // ADC = 3.152 / 3.3 * 4095 = 3909.5
    Thermistor::Params params;
    params.pullupResistor = 4700.0;
    params.referenceVoltage = 3.3;
    params.adcMax = 4095.0;
    params.resistanceAt25C = 100000.0;
    params.beta = 3950.0;

    double expectedR = 100000.0;
    double expectedV = 3.3 * expectedR / (params.pullupResistor + expectedR);
    double expectedAdc = expectedV / 3.3 * params.adcMax;

    Thermistor therm(0, params, [expectedAdc]() { return expectedAdc; });
    double temp = therm.read();
    EXPECT_NEAR(temp, 25.0, 1.0);
}

TEST(KlipperThermistor, ReadAt150C) {
    Thermistor::Params params;
    params.pullupResistor = 4700.0;
    params.referenceVoltage = 3.3;
    params.adcMax = 4095.0;
    params.resistanceAt25C = 100000.0;
    params.beta = 3950.0;

    // Calculate resistance at 150°C using beta model
    double t0 = 298.15;
    double t1 = 150.0 + 273.15;
    double r_at_150 = params.resistanceAt25C *
        std::exp(params.beta * (1.0/t1 - 1.0/t0));
    double v = 3.3 * r_at_150 / (params.pullupResistor + r_at_150);
    double adc = v / 3.3 * params.adcMax;

    Thermistor therm(0, params, [adc]() { return adc; });
    double temp = therm.read();
    EXPECT_NEAR(temp, 150.0, 2.0);
}

TEST(KlipperThermistor, ReadAtHighTemp) {
    Thermistor::Params params;
    params.pullupResistor = 4700.0;
    params.referenceVoltage = 3.3;
    params.adcMax = 4095.0;
    params.resistanceAt25C = 100000.0;
    params.beta = 3950.0;
    params.maxTemp = 500.0; // Extend range for this test

    // At 250°C: R = R25 * exp(beta * (1/T - 1/T0))
    // T = 523.15K, T0 = 298.15K
    double r_at_250 = 100000.0 * std::exp(3950.0 * (1.0/523.15 - 1.0/298.15));
    double v = 3.3 * r_at_250 / (4700.0 + r_at_250);
    double adc = v / 3.3 * 4095.0;

    Thermistor therm(0, params, [adc]() { return adc; });
    double temp = therm.read();
    EXPECT_NEAR(temp, 250.0, 5.0);
    EXPECT_TRUE(temp > 200.0);
}

TEST(KlipperThermistor, ReadAtLowTemp) {
    Thermistor::Params params;
    params.pullupResistor = 4700.0;
    params.referenceVoltage = 3.3;
    params.adcMax = 4095.0;
    params.resistanceAt25C = 100000.0;
    params.beta = 3950.0;

    // At very low temperature, resistance is very high
    // ADC value is close to max
    Thermistor therm(0, params, []() { return 4090.0; });
    double temp = therm.read();
    EXPECT_TRUE(temp < 0.0); // Should be very cold
}

TEST(KlipperThermistor, InvalidAdcReturnsNaN) {
    Thermistor::Params params;
    Thermistor therm(0, params, []() { return -1.0; });
    double temp = therm.read();
    EXPECT_TRUE(std::isnan(temp));
}

TEST(KlipperThermistor, OutOfRangeReturnsNaN) {
    Thermistor::Params params;
    params.minTemp = 0.0;
    params.maxTemp = 100.0;
    // ADC value that gives temp < 0
    Thermistor therm(0, params, []() { return 4090.0; });
    double temp = therm.read();
    EXPECT_TRUE(std::isnan(temp));
}

TEST(KlipperThermistor, Type) {
    Thermistor::Params params;
    Thermistor therm(0, params, []() { return 2048.0; });
    EXPECT_EQ(therm.type(), "thermistor");
}

TEST(KlipperThermistor, LastTemperature) {
    Thermistor::Params params;
    double adcValue = 2048.0;
    Thermistor therm(0, params, [&adcValue]() { return adcValue; });
    therm.update();
    EXPECT_FALSE(std::isnan(therm.lastTemperature()));
}

// ============================================================================
// Thermocouple tests
// ============================================================================

TEST(KlipperThermocouple, ReadNormalTemp) {
    // MAX31855: 25°C = 25 * 4 = 100 = 0x0064
    // 32-bit: temp in bits [31:18], so 25°C = 0x0064 << 18 = 0x01900000
    int32_t temp_raw = 25 * 4; // 0.25°C per bit
    int32_t raw = temp_raw << 18;
    std::vector<uint8_t> resp = {
        static_cast<uint8_t>((raw >> 24) & 0xFF),
        static_cast<uint8_t>((raw >> 16) & 0xFF),
        static_cast<uint8_t>((raw >> 8) & 0xFF),
        static_cast<uint8_t>(raw & 0xFF)
    };

    Thermocouple tc(0, Thermocouple::Type::K,
        [&](std::span<const uint8_t>) { return resp; });
    double temp = tc.read();
    EXPECT_NEAR(temp, 25.0, 0.5);
}

TEST(KlipperThermocouple, ReadNegativeTemp) {
    // -10°C = -10 * 4 = -40 = 0xFFFFFFD6 in 14-bit signed
    int16_t temp_raw = -10 * 4;
    int32_t raw = (static_cast<int32_t>(temp_raw) & 0x3FFF) << 18;
    std::vector<uint8_t> resp = {
        static_cast<uint8_t>((raw >> 24) & 0xFF),
        static_cast<uint8_t>((raw >> 16) & 0xFF),
        static_cast<uint8_t>((raw >> 8) & 0xFF),
        static_cast<uint8_t>(raw & 0xFF)
    };

    Thermocouple tc(0, Thermocouple::Type::K,
        [&](std::span<const uint8_t>) { return resp; });
    double temp = tc.read();
    EXPECT_NEAR(temp, -10.0, 1.0);
}

TEST(KlipperThermocouple, FaultReturnsNaN) {
    // Set fault bit (bit 16)
    int32_t raw = 0x00010000;
    std::vector<uint8_t> resp = {
        static_cast<uint8_t>((raw >> 24) & 0xFF),
        static_cast<uint8_t>((raw >> 16) & 0xFF),
        static_cast<uint8_t>((raw >> 8) & 0xFF),
        static_cast<uint8_t>(raw & 0xFF)
    };

    Thermocouple tc(0, Thermocouple::Type::K,
        [&](std::span<const uint8_t>) { return resp; });
    double temp = tc.read();
    EXPECT_TRUE(std::isnan(temp));
}

TEST(KlipperThermocouple, Type) {
    Thermocouple tc(0, Thermocouple::Type::K,
        [](std::span<const uint8_t>) { return std::vector<uint8_t>(4, 0); });
    EXPECT_EQ(tc.type(), "thermocouple");
}

// ============================================================================
// Heater tests
// ============================================================================

TEST(KlipperHeater, InitialState) {
    double pwmOutput = -1.0;
    double currentTemp = 25.0;
    Heater heater(0,
        [&pwmOutput](double v) { pwmOutput = v; },
        [&currentTemp]() { return currentTemp; });

    EXPECT_EQ(heater.oid(), 0);
    EXPECT_EQ(heater.target(), 0.0);
    EXPECT_TRUE(std::isnan(heater.currentTemp()));
}

TEST(KlipperHeater, SetTarget) {
    Heater heater(0, [](double){}, []() { return 25.0; });
    heater.setTarget(200.0);
    EXPECT_EQ(heater.target(), 200.0);
}

TEST(KlipperHeater, PidControl) {
    double pwmOutput = -1.0;
    double currentTemp = 25.0;
    Heater heater(0,
        [&pwmOutput](double v) { pwmOutput = v; },
        [&currentTemp]() { return currentTemp; });

    HeaterPidParams params;
    params.Kp = 0.5;
    params.Ki = 0.01;
    params.Kd = 0.0;
    params.imax = 10.0;
    params.pwmMin = 0.0;
    params.pwmMax = 1.0;
    heater.setPidParams(params);
    heater.setTarget(100.0);
    heater.setControlInterval(0.1);

    // First control iteration: error = 75, output = 0.5*75 = 37.5 -> clamped to 1.0
    double output = heater.control();
    EXPECT_NEAR(output, 1.0, 0.01);
    EXPECT_NEAR(pwmOutput, 1.0, 0.01);
}

TEST(KlipperHeater, PidAtTarget) {
    double pwmOutput = -1.0;
    Heater heater(0,
        [&pwmOutput](double v) { pwmOutput = v; },
        []() { return 100.0; });

    HeaterPidParams params;
    params.Kp = 0.5;
    params.Ki = 0.01;
    params.Kd = 0.0;
    params.imax = 10.0;
    heater.setPidParams(params);
    heater.setTarget(100.0);
    heater.setControlInterval(0.1);

    // Error = 0, output should be 0
    double output = heater.control();
    EXPECT_NEAR(output, 0.0, 0.01);
}

TEST(KlipperHeater, SafetyShutdownOnHighTemp) {
    bool shutdownCalled = false;
    std::string shutdownMsg;
    double pwmOutput = -1.0;

    Heater heater(0,
        [&pwmOutput](double v) { pwmOutput = v; },
        []() { return 350.0; }); // Over max temp

    heater.setSafetyLimits(-50.0, 300.0);
    heater.setShutdownCallback([&](const std::string& msg) {
        shutdownCalled = true;
        shutdownMsg = msg;
    });

    double output = heater.control();
    EXPECT_TRUE(shutdownCalled);
    EXPECT_NEAR(output, 0.0, 0.01);
    EXPECT_NEAR(pwmOutput, 0.0, 0.01);
    EXPECT_TRUE(shutdownMsg.find("out of range") != std::string::npos);
}

TEST(KlipperHeater, SafetyShutdownOnLowTemp) {
    bool shutdownCalled = false;
    Heater heater(0,
        [](double){},
        []() { return -100.0; }); // Below min temp

    heater.setSafetyLimits(-50.0, 300.0);
    heater.setShutdownCallback([&](const std::string& msg) {
        shutdownCalled = true;
    });

    heater.control();
    EXPECT_TRUE(shutdownCalled);
}

TEST(KlipperHeater, SafetyShutdownOnSensorFailure) {
    bool shutdownCalled = false;
    Heater heater(0,
        [](double){},
        []() { return NAN; }); // Sensor failure

    heater.setShutdownCallback([&](const std::string& msg) {
        shutdownCalled = true;
    });

    heater.control();
    EXPECT_TRUE(shutdownCalled);
}

TEST(KlipperHeater, Reset) {
    double pwmOutput = -1.0;
    Heater heater(0,
        [&pwmOutput](double v) { pwmOutput = v; },
        []() { return 25.0; });

    heater.setTarget(200.0);
    heater.setPidParams({0.5, 0.01, 0.0, 10.0, 0.0, 1.0});
    heater.control(); // Run one iteration

    heater.reset();
    EXPECT_EQ(heater.target(), 0.0);
    EXPECT_NEAR(pwmOutput, 0.0, 0.01);
}

TEST(KlipperHeater, AtTarget) {
    double currentTemp = 100.0;
    Heater heater(0, [](double){}, [&currentTemp]() { return currentTemp; });
    heater.setTarget(100.0);
    heater.control(); // Update currentTemp_

    EXPECT_TRUE(heater.atTarget(5.0));

    currentTemp = 95.0;
    heater.control();
    EXPECT_TRUE(heater.atTarget(10.0));

    currentTemp = 80.0;
    heater.control();
    EXPECT_FALSE(heater.atTarget(5.0));
}

TEST(KlipperHeater, AtTargetWhenTargetZero) {
    Heater heater(0, [](double){}, []() { return 25.0; });
    heater.setTarget(0.0);
    EXPECT_TRUE(heater.atTarget());
}

TEST(KlipperHeater, IntegralWindupClamped) {
    double pwmOutput = 0.0;
    double currentTemp = 25.0;
    Heater heater(0,
        [&pwmOutput](double v) { pwmOutput = v; },
        [&currentTemp]() { return currentTemp; });

    HeaterPidParams params;
    params.Kp = 0.0;
    params.Ki = 1.0;
    params.Kd = 0.0;
    params.imax = 5.0;
    params.pwmMin = 0.0;
    params.pwmMax = 1.0;
    heater.setPidParams(params);
    heater.setTarget(100.0);
    heater.setControlInterval(1.0);

    // Run many iterations - integral should be clamped at imax
    for (int i = 0; i < 100; ++i) {
        heater.control();
    }
    // Output should be Ki * integral = 1.0 * 5.0 = 5.0 -> clamped to 1.0
    EXPECT_NEAR(pwmOutput, 1.0, 0.01);
}

TEST(KlipperHeater, PwmClampedToMin) {
    double pwmOutput = 1.0;
    Heater heater(0,
        [&pwmOutput](double v) { pwmOutput = v; },
        []() { return 200.0; }); // Above target

    HeaterPidParams params;
    params.Kp = 1.0;
    params.Ki = 0.0;
    params.Kd = 0.0;
    params.pwmMin = 0.0;
    params.pwmMax = 1.0;
    heater.setPidParams(params);
    heater.setTarget(100.0);
    heater.setControlInterval(0.1);

    // Error = -100, output = 1.0 * -100 = -100 -> clamped to 0.0
    double output = heater.control();
    EXPECT_NEAR(output, 0.0, 0.01);
    EXPECT_NEAR(pwmOutput, 0.0, 0.01);
}

TEST(KlipperHeater, MultipleControlIterations) {
    double pwmOutput = 0.0;
    double currentTemp = 25.0;
    Heater heater(0,
        [&pwmOutput](double v) { pwmOutput = v; },
        [&currentTemp]() { return currentTemp; });

    HeaterPidParams params;
    params.Kp = 0.1;
    params.Ki = 0.001;
    params.Kd = 0.0;
    params.imax = 5.0;
    params.pwmMin = 0.0;
    params.pwmMax = 1.0;
    heater.setPidParams(params);
    heater.setTarget(60.0);
    heater.setControlInterval(0.1);

    // Simulate temperature rising
    for (int i = 0; i < 100; ++i) {
        double output = heater.control();
        // Simulate heating: temp increases proportional to output
        currentTemp += output * 2.0; // 2°C per iteration at full power
    }

    // Should be approaching target
    EXPECT_NEAR(heater.currentTemp(), 60.0, 10.0);
}
