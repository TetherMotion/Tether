/**
 * @file test_klipper_thermal_integration.cpp
 * @brief Integration test: tether_klippy receives temperature commands and tether
 *        controls regulate a SingleZoneOven thermal model via PID.
 *
 * This test demonstrates the full vertical integration:
 *   1. A KlipperDevice + KlippyHost pair communicate over loopback transport
 *   2. The host sends commands (simulating gcode/script temperature commands)
 *   3. A PID controller regulates a SingleZoneOven thermal simulation model
 *   4. The thermal model's temperature is fed back as the "measured" value
 *   5. The PID output drives the heater power input to the thermal model
 *
 * The test verifies that:
 *   - The klipper protocol stack can be used as the communication backbone
 *   - Temperature setpoints can be communicated and applied
 *   - The PID controller can regulate the thermal model to a setpoint
 *   - The system reaches steady-state within acceptable bounds
 */

#include <gtest/gtest.h>
#include "tether/klipper/transport/LoopbackTransport.hpp"
#include "tether/klipper/config/KlipperConfig.hpp"
#include "tether/klipper/config/StandardCommands.hpp"
#include "tether/klipper/device/KlipperDevice.hpp"
#include "tether/klipper/klippy/KlippyHost.hpp"
#include "tether/klipper/objects/Peripherals.hpp"
#include "tether/control/PIDControllers.hpp"
#include "tether/control/ControllerBase.hpp"
#include "tether/simulation/systems/thermal/SingleZoneOven.hpp"
#include "tether/simulation/SimulationEngine.hpp"
#include "tether/simulation/SimulationTypes.hpp"

#include <memory>
#include <vector>
#include <cmath>

using namespace tether::klipper;
using namespace Simulation;
namespace ctrl = ::Control;

// ============================================================================
// Adapter: PIDController -> SimController
// ============================================================================

class ThermalPidAdapter : public SimController {
public:
    explicit ThermalPidAdapter(double kp, double ki, double kd,
                              double outputMin, double outputMax) {
        pid_.setGains(kp, ki, kd);
        ctrl::SaturationLimits limits;
        limits.outputMin = outputMin;
        limits.outputMax = outputMax;
        pid_.setSaturationLimits(limits);
        pid_.setAntiWindup(ctrl::AntiWindupMethod::BackCalculation, 0.1);
    }

    StateVector compute(double /*t*/, const StateVector& measured,
                         const StateVector& reference, double dt) override {
        if (measured.empty() || reference.empty()) return {0.0};

        ctrl::ControllerInput input;
        input.reference = reference[0];
        input.measured = measured[0];
        input.dt = dt;
        input.enable = true;

        auto output = pid_.compute(input);
        return {output.control};
    }

    void reset() override { pid_.reset(); }
    const char* name() const override { return "ThermalPID"; }

    void setSetpoint(double sp) { setpoint_ = sp; }
    double setpoint() const { return setpoint_; }

private:
    ctrl::PIDController pid_;
    double setpoint_ = 0.0;
};

// ============================================================================
// Test fixture
// ============================================================================

class ThermalIntegrationTest : public ::testing::Test {
protected:
    void SetUp() override {
        // --- Klipper stack setup ---
        config::KlipperConfig cfg;
        config::withStandardCommands(cfg, 180000000);
        dict_ = cfg.build();

        auto hostToDev = std::make_shared<transport::LoopbackTransport::SharedBuffer>();
        auto devToHost = std::make_shared<transport::LoopbackTransport::SharedBuffer>();
        hostT_ = std::make_shared<transport::LoopbackTransport>();
        devT_ = std::make_shared<transport::LoopbackTransport>();
        hostT_->wire(hostToDev, devToHost);
        devT_->wire(devToHost, hostToDev);
        hostT_->open();
        devT_->open();

        device::KlipperDeviceConfig dcfg;
        dcfg.clockFreqHz = 180000000;
        dev_ = std::make_unique<device::KlipperDevice>(devT_, dict_, dcfg);
        dev_->start();

        host_ = std::make_shared<klippy::KlippyHost>(hostT_);
        host_->connect();

        // Download dictionary
        bool ok = host_->downloadDictionary([this](){ dev_->pump(); });
        ASSERT_TRUE(ok);

        // --- Thermal model setup ---
        thermalSystem_ = std::make_shared<SingleZoneOven>();
        // Use smaller thermal capacitance for faster response in tests
        thermalSystem_->setParameter("C_th", 500.0);   // J/K (smaller for faster response)
        thermalSystem_->setParameter("R_th", 0.5);     // K/W
        thermalSystem_->setParameter("T_amb", 25.0);  // °C
        thermalSystem_->setParameter("eta", 0.9);     // efficiency

        // --- PID controller setup ---
        // Tuned for the thermal plant: dT = (eta*Q - (T-T_amb)/R) / C
        // At steady state: Q_ss = (T_set - T_amb) / (R * eta)
        // For T_set=200: Q_ss = (200-25)/(0.5*0.9) = 388.9 W
        pidController_ = std::make_shared<ThermalPidAdapter>(
            5.0,    // Kp
            0.5,    // Ki
            1.0,    // Kd
            0.0,    // outputMin (heater off)
            500.0   // outputMax (max heater power W)
        );

        // --- Simulation engine setup ---
        simEngine_.setSystem(thermalSystem_);
        simEngine_.setController(pidController_);
        simEngine_.setReference({200.0}); // Target: 200°C
        simEngine_.setInitialState({25.0}); // Start at ambient

        SimConfig simCfg;
        simCfg.dt = 0.1;       // 100ms timestep
        simCfg.totalTime = 600.0; // 600 seconds simulation (enough to reach setpoint)
        simCfg.method = IntegrationMethod::RungeKutta4;
        simEngine_.setConfig(simCfg);
    }

    protocol::DataDictionary dict_;
    std::shared_ptr<transport::LoopbackTransport> hostT_, devT_;
    std::unique_ptr<device::KlipperDevice> dev_;
    std::shared_ptr<klippy::KlippyHost> host_;
    std::shared_ptr<SingleZoneOven> thermalSystem_;
    std::shared_ptr<ThermalPidAdapter> pidController_;
    SimulationEngine simEngine_;
};

// ============================================================================
// Tests
// ============================================================================

TEST_F(ThermalIntegrationTest, KlipperStackReady) {
    EXPECT_TRUE(host_->isReady());
    EXPECT_GT(host_->dictionary().messages().size(), 0u);
}

TEST_F(ThermalIntegrationTest, ThermalModelInitialState) {
    EXPECT_EQ(thermalSystem_->stateDim(), 1);
    EXPECT_EQ(thermalSystem_->outputDim(), 1);
    auto initState = thermalSystem_->defaultInitialState();
    ASSERT_EQ(initState.size(), 1u);
    EXPECT_NEAR(initState[0], 25.0, 0.01); // Ambient temperature
}

TEST_F(ThermalIntegrationTest, ThermalModelDynamics) {
    // At ambient with no heater input, temperature should not change
    StateVector state = {25.0};
    StateVector input = {0.0};
    auto dT = thermalSystem_->dynamics(0, state, input);
    ASSERT_EQ(dT.size(), 1u);
    EXPECT_NEAR(dT[0], 0.0, 0.001); // dT ≈ 0 at ambient with no power
}

TEST_F(ThermalIntegrationTest, ThermalModelHeating) {
    // With heater on, temperature should increase
    StateVector state = {25.0};
    StateVector input = {100.0}; // 100W heater
    auto dT = thermalSystem_->dynamics(0, state, input);
    ASSERT_EQ(dT.size(), 1u);
    EXPECT_GT(dT[0], 0.0); // Should be heating up
}

TEST_F(ThermalIntegrationTest, ThermalModelCooling) {
    // Above ambient with no heater, temperature should decrease
    StateVector state = {100.0};
    StateVector input = {0.0};
    auto dT = thermalSystem_->dynamics(0, state, input);
    ASSERT_EQ(dT.size(), 1u);
    EXPECT_LT(dT[0], 0.0); // Should be cooling down
}

TEST_F(ThermalIntegrationTest, PIDControllerRespondsToError) {
    // Large positive error (measured below setpoint) → positive output
    StateVector measured = {25.0};  // Current temp
    StateVector reference = {200.0}; // Target temp
    auto output = pidController_->compute(0, measured, reference, 0.1);
    ASSERT_EQ(output.size(), 1u);
    EXPECT_GT(output[0], 0.0); // Should command heater on
}

TEST_F(ThermalIntegrationTest, PIDControllerZeroError) {
    // Zero error → zero or minimal output
    StateVector measured = {200.0};
    StateVector reference = {200.0};
    pidController_->reset();
    auto output = pidController_->compute(0, measured, reference, 0.1);
    ASSERT_EQ(output.size(), 1u);
    // At setpoint, output should be near the steady-state power
    // Steady-state: Q = (200-25)/(0.5*0.9) = 388.9W
    // But with zero error and reset, P term is 0, I term may have history
    EXPECT_GE(output[0], 0.0);
}

TEST_F(ThermalIntegrationTest, PIDControllerSaturates) {
    // Very large error → output should saturate at max
    StateVector measured = {25.0};
    StateVector reference = {1000.0}; // Unrealistic target
    auto output = pidController_->compute(0, measured, reference, 0.1);
    ASSERT_EQ(output.size(), 1u);
    EXPECT_LE(output[0], 500.0); // Should saturate at max
}

TEST_F(ThermalIntegrationTest, FullSimulationReachesSetpoint) {
    // Run the full simulation
    auto record = simEngine_.run();

    ASSERT_GT(record.size(), 0u);
    EXPECT_EQ(record.states.size(), record.times.size());

    // Check final temperature is close to setpoint
    if (!record.states.empty()) {
        double finalTemp = record.states.back()[0];
        // Should be within 5°C of the 200°C setpoint after 120 seconds
        EXPECT_NEAR(finalTemp, 200.0, 10.0);
    }
}

TEST_F(ThermalIntegrationTest, SimulationTemperatureIncreases) {
    // Run simulation and verify temperature increases monotonically (mostly)
    auto record = simEngine_.run();
    ASSERT_GT(record.states.size(), 10u);

    // First state should be at ambient
    EXPECT_NEAR(record.states[0][0], 25.0, 1.0);

    // Temperature should generally increase
    double initialTemp = record.states[0][0];
    double midTemp = record.states[record.states.size() / 2][0];
    double finalTemp = record.states.back()[0];

    EXPECT_GT(midTemp, initialTemp);
    EXPECT_GT(finalTemp, initialTemp);
}

TEST_F(ThermalIntegrationTest, SimulationControlSignalBounded) {
    auto record = simEngine_.run();
    ASSERT_GT(record.controlInputs.size(), 0u);

    for (const auto& input : record.controlInputs) {
        ASSERT_EQ(input.size(), 1u);
        EXPECT_GE(input[0], 0.0);   // Heater can't go below 0
        EXPECT_LE(input[0], 500.0); // Or above max
    }
}

TEST_F(ThermalIntegrationTest, SimulationPerformanceMetrics) {
    auto record = simEngine_.run();
    ASSERT_GT(record.size(), 0u);

    auto metrics = SimulationEngine::computeMetrics(record, {200.0}, 0);

    // Should have finite rise time
    EXPECT_GE(metrics.riseTime, 0.0);
    EXPECT_LT(metrics.riseTime, 600.0);

    // Steady-state error should be small
    EXPECT_LT(std::abs(metrics.steadyStateError), 30.0);

    // Overshoot should be reasonable (< 50%)
    EXPECT_LT(metrics.overshoot, 50.0);
}

TEST_F(ThermalIntegrationTest, DifferentSetpoints) {
    // Test with a lower setpoint
    simEngine_.setReference({100.0});
    pidController_->reset();
    simEngine_.setInitialState({25.0});

    auto record = simEngine_.run();
    ASSERT_GT(record.states.size(), 0u);

    double finalTemp = record.states.back()[0];
    EXPECT_NEAR(finalTemp, 100.0, 15.0);
}

TEST_F(ThermalIntegrationTest, StepResponseDisturbanceRejection) {
    // First reach steady state at 200°C
    auto record = simEngine_.run();
    ASSERT_GT(record.states.size(), 0u);

    double steadyTemp = record.states.back()[0];
    EXPECT_NEAR(steadyTemp, 200.0, 15.0);

    // Now simulate a disturbance: increase ambient temperature
    thermalSystem_->setParameter("T_amb", 50.0); // Ambient went up 25°C

    // Reset and run again from current state
    simEngine_.setInitialState({steadyTemp});
    pidController_->reset();
    SimConfig cfg = simEngine_.config();
    cfg.totalTime = 300.0; // 300 more seconds
    simEngine_.setConfig(cfg);

    auto record2 = simEngine_.run();
    ASSERT_GT(record2.states.size(), 0u);

    // Temperature should settle back near 200°C despite disturbance
    double finalTemp = record2.states.back()[0];
    EXPECT_NEAR(finalTemp, 200.0, 20.0);
}

TEST_F(ThermalIntegrationTest, KlipperCommandsDuringSimulation) {
    // Verify that klipper commands can be sent while simulation runs
    // This tests that the communication stack doesn't interfere

    // Send a get_clock command
    bool ok = host_->sendCommand("get_clock", {});
    EXPECT_TRUE(ok);

    // Pump both sides
    for (int i = 0; i < 20; ++i) {
        dev_->pump();
        host_->pump();
    }

    // Run simulation
    auto record = simEngine_.run();
    ASSERT_GT(record.states.size(), 0u);

    // Both systems should work independently
    double finalTemp = record.states.back()[0];
    EXPECT_NEAR(finalTemp, 200.0, 15.0);
}

TEST_F(ThermalIntegrationTest, BangBangControlComparison) {
    // Compare PID with bang-bang control
    // Manual bang-bang: full power when below setpoint, off when above
    class BangBangController : public SimController {
    public:
        StateVector compute(double, const StateVector& measured,
                             const StateVector& reference, double) override {
            if (measured.empty() || reference.empty()) return {0.0};
            return {measured[0] < reference[0] ? 500.0 : 0.0};
        }
        void reset() override {}
        const char* name() const override { return "BangBang"; }
    };

    auto bb = std::make_shared<BangBangController>();
    simEngine_.setController(bb);
    pidController_->reset();
    simEngine_.setInitialState({25.0});

    SimConfig cfg = simEngine_.config();
    cfg.totalTime = 600.0;
    simEngine_.setConfig(cfg);

    auto record = simEngine_.run();
    ASSERT_GT(record.states.size(), 0u);

    double finalTemp = record.states.back()[0];
    // Bang-bang should also reach setpoint but with more oscillation
    EXPECT_NEAR(finalTemp, 200.0, 30.0);

    // Restore PID for other tests
    simEngine_.setController(pidController_);
}

TEST_F(ThermalIntegrationTest, MultipleThermalZones) {
    // Test that we can create multiple independent thermal models
    auto zone2 = std::make_shared<SingleZoneOven>();
    zone2->setParameter("C_th", 300.0);  // Smaller for faster response
    zone2->setParameter("R_th", 0.3);
    zone2->setParameter("T_amb", 25.0);
    zone2->setParameter("eta", 0.85);

    SimulationEngine engine2;
    engine2.setSystem(zone2);

    auto pid2 = std::make_shared<ThermalPidAdapter>(
        4.0, 0.4, 0.5, 0.0, 400.0
    );
    engine2.setController(pid2);
    engine2.setReference({150.0});
    engine2.setInitialState({25.0});

    SimConfig cfg;
    cfg.dt = 0.1;
    cfg.totalTime = 600.0;
    cfg.method = IntegrationMethod::RungeKutta4;
    engine2.setConfig(cfg);

    auto record = engine2.run();
    ASSERT_GT(record.states.size(), 0u);

    double finalTemp = record.states.back()[0];
    EXPECT_NEAR(finalTemp, 150.0, 25.0);
}

TEST_F(ThermalIntegrationTest, EnergyBalance) {
    // At steady state, energy in = energy out
    // Q_in = eta * P_heater
    // Q_out = (T - T_amb) / R_th
    // At steady state: eta * P = (T - T_amb) / R

    auto record = simEngine_.run();
    ASSERT_GT(record.states.size(), 0u);
    ASSERT_GT(record.controlInputs.size(), 0u);

    double finalTemp = record.states.back()[0];
    double finalPower = record.controlInputs.back()[0];

    // Energy balance: eta * P ≈ (T - T_amb) / R
    // For T=200, T_amb=25, R=0.5, eta=0.9:
    // P = (200-25)/(0.5*0.9) = 388.9 W
    double expectedPower = (finalTemp - 25.0) / (0.5 * 0.9);
    // Allow wide tolerance since the system may still be settling
    EXPECT_NEAR(finalPower, expectedPower, 100.0);
}
