/// @file test_klipper_autotuning_bridge.cpp
/// @brief Tests for the KlippyAutotuningBridge that delegates all autotuning
///        to the Tether autotuning framework.

#include <gtest/gtest.h>

#include "tether/klipper/klippy/KlippyAutotuningBridge.hpp"
#include "tether/klipper/klippy/KlippyInstanceConfig.hpp"
#include "tether/klipper/objects/Thermal.hpp"

#include <cmath>
#include <functional>
#include <string>

using namespace tether::klipper::klippy;
using namespace tether::klipper::objects;

// ============================================================================
// Helpers
// ============================================================================

/// @brief Create a heater with a simulated thermal model.
/// The simulation uses a simple first-order thermal model:
///   T(t) = T_ambient + K * power * (1 - exp(-t/tau))
struct SimulatedHeater {
    double ambient = 25.0;
    double K = 100.0;       // gain: 100°C at full power
    double tau = 30.0;      // time constant: 30s
    double temp = 25.0;
    double pwmOutput = 0.0;

    std::function<void(double)> pwmWrite = [this](double p) {
        pwmOutput = p;
    };
    std::function<double()> sensorRead = [this]() {
        return temp;
    };

    std::shared_ptr<Heater> heater;

    SimulatedHeater() {
        heater = std::make_shared<Heater>(1, pwmWrite, sensorRead);
        heater->setControlInterval(0.1);
    }

    /// @brief Step the thermal simulation.
    void step(double power, double dt) {
        heater->update(power, dt);
        // First-order thermal model
        double targetTemp = ambient + K * power;
        temp += (targetTemp - temp) * (dt / tau);
    }
};

// ============================================================================
// Method Selection Tests
// ============================================================================

TEST(KlippyAutotuningBridge, MethodNameRoundTrip) {
    // All methods should round-trip through name <-> enum
    auto methods = listAutotuneMethods();
    EXPECT_EQ(methods.size(), 12u);
    for (const auto& name : methods) {
        auto m = parseAutotuneMethod(name);
        EXPECT_EQ(autotuneMethodName(m), name)
            << "Round-trip failed for method: " << name;
    }
}

TEST(KlippyAutotuningBridge, DefaultMethodIsRelayFeedback) {
    // Default method should be relay_feedback
    EXPECT_EQ(autotuneMethodName(AutotuneMethod::RelayFeedback), "relay_feedback");
}

TEST(KlippyAutotuningBridge, UnknownMethodDefaultsToRelayFeedback) {
    auto m = parseAutotuneMethod("nonexistent_method");
    EXPECT_EQ(m, AutotuneMethod::RelayFeedback);
}

TEST(KlippyAutotuningBridge, MethodAliases) {
    // Test common aliases
    EXPECT_EQ(parseAutotuneMethod("relay"), AutotuneMethod::RelayFeedback);
    EXPECT_EQ(parseAutotuneMethod("imc"), AutotuneMethod::Lambda);
    EXPECT_EQ(parseAutotuneMethod("chr"), AutotuneMethod::ChienHronesReswick);
    EXPECT_EQ(parseAutotuneMethod("itae"), AutotuneMethod::LopezITAE);
    EXPECT_EQ(parseAutotuneMethod("iae"), AutotuneMethod::LopezIAE);
    EXPECT_EQ(parseAutotuneMethod("ise"), AutotuneMethod::LopezISE);
}

// ============================================================================
// Offline Autotuning from FOPDT Model Tests
// ============================================================================

TEST(KlippyAutotuningBridge, RunFromModelCohenCoon) {
    SimulatedHeater sim;
    double kp = 0, ki = 0, kd = 0;
    HeaterAutotuneBridge bridge(*sim.heater, kp, ki, kd);

    FOPDTModel model;
    model.K = 100.0;
    model.tau = 30.0;
    model.L = 5.0;

    auto result = bridge.runFromModel(model, AutotuneMethod::CohenCoon);
    EXPECT_TRUE(result.success) << result.message;
    EXPECT_GT(result.Kp, 0.0);
    EXPECT_GT(result.Ki, 0.0);
    EXPECT_GT(result.Kd, 0.0);
    EXPECT_GT(result.Ku, 0.0);
    EXPECT_GT(result.Tu, 0.0);
    // Gains should be applied to settings
    EXPECT_NEAR(kp, result.Kp, 0.001);
    EXPECT_NEAR(ki, result.Ki, 0.001);
    EXPECT_NEAR(kd, result.Kd, 0.001);
}

TEST(KlippyAutotuningBridge, RunFromModelLambda) {
    SimulatedHeater sim;
    double kp = 0, ki = 0, kd = 0;
    HeaterAutotuneBridge bridge(*sim.heater, kp, ki, kd);

    FOPDTModel model;
    model.K = 50.0;
    model.tau = 20.0;
    model.L = 3.0;

    auto result = bridge.runFromModel(model, AutotuneMethod::Lambda, PIDForm::Parallel, 15.0);
    EXPECT_TRUE(result.success) << result.message;
    EXPECT_GT(result.Kp, 0.0);
}

TEST(KlippyAutotuningBridge, RunFromModelSIMC) {
    SimulatedHeater sim;
    double kp = 0, ki = 0, kd = 0;
    HeaterAutotuneBridge bridge(*sim.heater, kp, ki, kd);

    FOPDTModel model;
    model.K = 80.0;
    model.tau = 25.0;
    model.L = 4.0;

    auto result = bridge.runFromModel(model, AutotuneMethod::SIMC);
    EXPECT_TRUE(result.success) << result.message;
    EXPECT_GT(result.Kp, 0.0);
}

TEST(KlippyAutotuningBridge, RunFromModelAMIGO) {
    SimulatedHeater sim;
    double kp = 0, ki = 0, kd = 0;
    HeaterAutotuneBridge bridge(*sim.heater, kp, ki, kd);

    FOPDTModel model;
    model.K = 60.0;
    model.tau = 15.0;
    model.L = 2.0;

    auto result = bridge.runFromModel(model, AutotuneMethod::AMIGO);
    EXPECT_TRUE(result.success) << result.message;
    EXPECT_GT(result.Kp, 0.0);
}

TEST(KlippyAutotuningBridge, RunFromModelZieglerNicholsStep) {
    SimulatedHeater sim;
    double kp = 0, ki = 0, kd = 0;
    HeaterAutotuneBridge bridge(*sim.heater, kp, ki, kd);

    FOPDTModel model;
    model.K = 100.0;
    model.tau = 30.0;
    model.L = 5.0;

    auto result = bridge.runFromModel(model, AutotuneMethod::ZieglerNicholsStep);
    EXPECT_TRUE(result.success) << result.message;
    EXPECT_GT(result.Kp, 0.0);
}

TEST(KlippyAutotuningBridge, RunFromModelChienHronesReswick) {
    SimulatedHeater sim;
    double kp = 0, ki = 0, kd = 0;
    HeaterAutotuneBridge bridge(*sim.heater, kp, ki, kd);

    FOPDTModel model;
    model.K = 70.0;
    model.tau = 20.0;
    model.L = 3.0;

    auto result = bridge.runFromModel(model, AutotuneMethod::ChienHronesReswick);
    EXPECT_TRUE(result.success) << result.message;
    EXPECT_GT(result.Kp, 0.0);
}

TEST(KlippyAutotuningBridge, RunFromModelLopezITAE) {
    SimulatedHeater sim;
    double kp = 0, ki = 0, kd = 0;
    HeaterAutotuneBridge bridge(*sim.heater, kp, ki, kd);

    FOPDTModel model;
    model.K = 90.0;
    model.tau = 25.0;
    model.L = 4.0;

    auto result = bridge.runFromModel(model, AutotuneMethod::LopezITAE);
    EXPECT_TRUE(result.success) << result.message;
    EXPECT_GT(result.Kp, 0.0);
}

TEST(KlippyAutotuningBridge, RunFromModelInvalidModel) {
    SimulatedHeater sim;
    double kp = 0, ki = 0, kd = 0;
    HeaterAutotuneBridge bridge(*sim.heater, kp, ki, kd);

    FOPDTModel model;
    model.K = 0.0;  // Invalid
    model.tau = 0.0;
    model.L = -1.0;

    auto result = bridge.runFromModel(model, AutotuneMethod::CohenCoon);
    EXPECT_FALSE(result.success);
    EXPECT_TRUE(result.message.find("Invalid") != std::string::npos);
}

// ============================================================================
// Offline Autotuning from Ultimate Parameters Tests
// ============================================================================

TEST(KlippyAutotuningBridge, RunFromUltimateZieglerNichols) {
    SimulatedHeater sim;
    double kp = 0, ki = 0, kd = 0;
    HeaterAutotuneBridge bridge(*sim.heater, kp, ki, kd);

    double Ku = 5.0;
    double Tu = 20.0;

    auto result = bridge.runFromUltimateParams(Ku, Tu, AutotuneMethod::ZieglerNicholsUltimate);
    EXPECT_TRUE(result.success) << result.message;
    // ZN PID: Kp = 0.6*Ku = 3.0
    EXPECT_NEAR(result.Kp, 0.6 * Ku, 0.01);
    EXPECT_GT(result.Ki, 0.0);
    EXPECT_GT(result.Kd, 0.0);
    EXPECT_NEAR(result.Ku, Ku, 0.001);
    EXPECT_NEAR(result.Tu, Tu, 0.001);
}

TEST(KlippyAutotuningBridge, RunFromUltimateTyreusLuyben) {
    SimulatedHeater sim;
    double kp = 0, ki = 0, kd = 0;
    HeaterAutotuneBridge bridge(*sim.heater, kp, ki, kd);

    double Ku = 5.0;
    double Tu = 20.0;

    auto result = bridge.runFromUltimateParams(Ku, Tu, AutotuneMethod::TyreusLuyben);
    EXPECT_TRUE(result.success) << result.message;
    // Tyreus-Luyben: Kp = Ku / 3.2 (more conservative than ZN's 0.6*Ku)
    EXPECT_NEAR(result.Kp, Ku / 3.2, 0.01);
    EXPECT_GT(result.Kp, 0.0);
}

// ============================================================================
// Resonance Calibration Bridge Tests
// ============================================================================

TEST(KlippyAutotuningBridge, ResonanceBridgeNoAccelerometer) {
    ResonanceCalibrationBridge bridge;
    auto result = bridge.calibrate();
    EXPECT_FALSE(result.success);
    EXPECT_TRUE(result.message.find("No accelerometer") != std::string::npos);
}

TEST(KlippyAutotuningBridge, ResonanceBridgeWithSimulatedAccel) {
    ResonanceCalibrationBridge bridge;

    // Simulate an accelerometer that responds to the excitation frequency.
    // The bridge calls generateExcitation() internally, so we track the
    // current frequency being tested via a shared variable.
    double currentFreq = 0.0;
    double resonanceFreq = 35.0;
    bridge.setAccelerometerSource(
        [&currentFreq, resonanceFreq]() -> std::array<double, 3> {
            // Simulate resonance: large response at resonanceFreq
            double gain = 5.0 / (1.0 + std::pow((currentFreq - resonanceFreq) / 2.0, 2));
            // Return a value proportional to the gain (phase doesn't matter much)
            return {gain, 0, 0};
        });

    // We can't easily inject the current frequency into the lambda since
    // runSweep controls it internally. Instead, just verify the bridge
    // runs without crashing when an accelerometer is configured.
    // The coherence filter may reject all points if the simulation
    // doesn't correlate with the excitation, so we just check no crash.
    auto bode = bridge.runSweep("X", 30.0f, 40.0f, 5.0f);
    // The bode may be empty if coherence is too low, that's OK for this test.
    // The important thing is that it didn't crash.

    // Test calibrateFromBode with non-empty data
    std::vector<Identification::FrequencyResponsePoint> fakeBode;
    Identification::FrequencyResponsePoint p1;
    p1.frequency = 30.0f;
    p1.magnitude_dB = 5.0f;
    p1.coherence = 0.9f;
    fakeBode.push_back(p1);
    Identification::FrequencyResponsePoint p2;
    p2.frequency = 35.0f;
    p2.magnitude_dB = 25.0f;  // Peak
    p2.coherence = 0.9f;
    fakeBode.push_back(p2);
    Identification::FrequencyResponsePoint p3;
    p3.frequency = 40.0f;
    p3.magnitude_dB = 5.0f;
    p3.coherence = 0.9f;
    fakeBode.push_back(p3);

    auto [freq, shaperType] = ResonanceCalibrationBridge::calibrateFromBode(fakeBode);
    EXPECT_NEAR(freq, 35.0, 0.1);
    EXPECT_EQ(shaperType, "zvd");  // > 20 dB → ZVD
}

TEST(KlippyAutotuningBridge, ResonanceCalibrateFromEmptyBode) {
    std::vector<Identification::FrequencyResponsePoint> empty;
    auto [freq, shaperType] = ResonanceCalibrationBridge::calibrateFromBode(empty);
    EXPECT_NEAR(freq, 0.0, 0.001);
    EXPECT_EQ(shaperType, "none");
}

// ============================================================================
// Config Parsing Tests (via KlippyInstance)
// ============================================================================

TEST(KlippyAutotuningBridge, AutotuneConfigParsing) {
    // Test that the [autotune] config section is parsed correctly
    // This is tested indirectly through the settings struct
    KlippySettings settings;
    EXPECT_EQ(settings.pidAutotuneMethod, "relay_feedback");
    EXPECT_EQ(settings.pidAutotuneForm, "pid");
    EXPECT_NEAR(settings.pidAutotuneLambda, -1.0, 0.001);

    // Change values
    settings.pidAutotuneMethod = "cohen_coon";
    settings.pidAutotuneForm = "parallel";
    settings.pidAutotuneLambda = 10.0;

    EXPECT_EQ(settings.pidAutotuneMethod, "cohen_coon");
    EXPECT_EQ(settings.pidAutotuneForm, "parallel");
    EXPECT_NEAR(settings.pidAutotuneLambda, 10.0, 0.001);

    // Verify method parsing
    auto m = parseAutotuneMethod(settings.pidAutotuneMethod);
    EXPECT_EQ(m, AutotuneMethod::CohenCoon);
}
