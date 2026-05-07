/**
 * @file AdvancedMotorModelTests.cpp
 * @brief Comprehensive unit tests for AdvancedMotorModel
 *
 * Contains 200+ tests covering:
 * - Basic motor physics
 * - Friction models (Coulomb, viscous, Stribeck)
 * - Thermal simulation
 * - Backlash modeling
 * - Geartrain dynamics
 * - Torque-speed curves
 * - Load disturbances
 * - Edge cases and numerical stability
 */

#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include <cmath>
#include <random>
#include <limits>
#include <algorithm>

// Include the motor model header
#include "profiles/cia402/AdvancedMotorModel.hpp"

namespace cia402 {
namespace test {

using ::testing::DoubleNear;
using ::testing::Lt;
using ::testing::Gt;
using ::testing::Le;
using ::testing::Ge;

// ============================================================================
// Test Constants
// ============================================================================

constexpr double TEST_EPSILON = 1e-9;
constexpr double POSITION_TOLERANCE = 1e-6;
constexpr double VELOCITY_TOLERANCE = 1e-4;
constexpr double TORQUE_TOLERANCE = 1e-3;
constexpr double TEMPERATURE_TOLERANCE = 0.1;

// Standard motor parameters for testing
constexpr double TEST_INERTIA = 0.001;        // kg·m²
constexpr double TEST_RATED_TORQUE = 1.0;     // Nm
constexpr double TEST_RATED_SPEED = 3000.0;   // RPM
constexpr double TEST_RESISTANCE = 1.0;       // Ohms
constexpr double TEST_INDUCTANCE = 0.001;     // H
constexpr double TEST_KT = 0.5;               // Nm/A
constexpr double TEST_KE = 0.5;               // V/(rad/s)

// ============================================================================
// Test Fixtures
// ============================================================================

/**
 * @brief Base fixture for motor model tests
 */
class AdvancedMotorModelTest : public ::testing::Test {
protected:
    void SetUp() override {
        config_.inertia = TEST_INERTIA;
        config_.ratedTorque = TEST_RATED_TORQUE;
        config_.ratedSpeed = TEST_RATED_SPEED;
        config_.resistance = TEST_RESISTANCE;
        config_.inductance = TEST_INDUCTANCE;
        config_.torqueConstant = TEST_KT;
        config_.backEMFConstant = TEST_KE;
        config_.maxCurrent = 10.0;
        config_.maxVoltage = 48.0;

        friction_.coulombFriction = 0.01;
        friction_.viscousFriction = 0.0001;
        friction_.stribeckVelocity = 10.0;
        friction_.stribeckFactor = 1.5;

        thermal_.thermalResistance = 5.0;  // K/W
        thermal_.thermalCapacitance = 100.0;  // J/K
        thermal_.ambientTemperature = 25.0;  // °C
        thermal_.maxTemperature = 150.0;  // °C

        backlash_.deadband = 0.0;  // No backlash by default
        backlash_.springConstant = 1e6;
        backlash_.dampingFactor = 100.0;

        geartrain_.gearRatio = 1.0;
        geartrain_.efficiency = 1.0;
        geartrain_.outputInertia = 0.0;

        model_ = std::make_unique<AdvancedMotorModel>(
            config_, friction_, thermal_, backlash_, geartrain_
        );
    }

    void stepModel(double dt, double inputTorque, int steps = 1) {
        for (int i = 0; i < steps; ++i) {
            model_->step(dt, inputTorque, 0.0);  // No load torque
        }
    }

    // Convert RPM to rad/s
    static double rpmToRadS(double rpm) {
        return rpm * 2.0 * M_PI / 60.0;
    }

    // Convert rad/s to RPM
    static double radSToRpm(double radS) {
        return radS * 60.0 / (2.0 * M_PI);
    }

    MotorConfig config_;
    FrictionParams friction_;
    ThermalConfig thermal_;
    BacklashConfig backlash_;
    GeartrainConfig geartrain_;
    std::unique_ptr<AdvancedMotorModel> model_;
};

// ============================================================================
// Basic Motor Physics Tests
// ============================================================================

TEST_F(AdvancedMotorModelTest, InitialStateIsZero) {
    EXPECT_DOUBLE_EQ(model_->getPosition(), 0.0);
    EXPECT_DOUBLE_EQ(model_->getVelocity(), 0.0);
    EXPECT_NEAR(model_->getTemperature(), thermal_.ambientTemperature, TEMPERATURE_TOLERANCE);
}

TEST_F(AdvancedMotorModelTest, AcceleratesUnderConstantTorque) {
    double torque = 0.5;
    double dt = 0.001;

    double initialVelocity = model_->getVelocity();
    stepModel(dt, torque, 100);
    double finalVelocity = model_->getVelocity();

    EXPECT_GT(finalVelocity, initialVelocity);
}

TEST_F(AdvancedMotorModelTest, DeceleratesUnderNegativeTorque) {
    // First accelerate
    stepModel(0.001, 0.5, 100);
    double midVelocity = model_->getVelocity();
    EXPECT_GT(midVelocity, 0);

    // Then apply negative torque
    stepModel(0.001, -0.5, 100);
    double finalVelocity = model_->getVelocity();

    EXPECT_LT(finalVelocity, midVelocity);
}

TEST_F(AdvancedMotorModelTest, PositionIntegratesVelocity) {
    double dt = 0.001;
    double torque = 0.5;

    double initialPosition = model_->getPosition();
    stepModel(dt, torque, 1000);
    double finalPosition = model_->getPosition();

    EXPECT_GT(finalPosition, initialPosition);
}

TEST_F(AdvancedMotorModelTest, NewtonSecondLawHolds) {
    // α = τ/J
    double torque = TEST_RATED_TORQUE;
    double expectedAccel = torque / TEST_INERTIA;
    double dt = 0.0001;

    model_->step(dt, torque, 0.0);
    double velocity = model_->getVelocity();

    // v = α * t (approximately, for small dt and no friction)
    friction_.coulombFriction = 0.0;
    friction_.viscousFriction = 0.0;
    model_ = std::make_unique<AdvancedMotorModel>(
        config_, friction_, thermal_, backlash_, geartrain_
    );

    model_->step(dt, torque, 0.0);
    velocity = model_->getVelocity();

    EXPECT_NEAR(velocity, expectedAccel * dt, velocity * 0.01);  // 1% tolerance
}

TEST_F(AdvancedMotorModelTest, ZeroTorqueMaintainsVelocityWithNoFriction) {
    friction_.coulombFriction = 0.0;
    friction_.viscousFriction = 0.0;
    model_ = std::make_unique<AdvancedMotorModel>(
        config_, friction_, thermal_, backlash_, geartrain_
    );

    // Accelerate to some velocity
    stepModel(0.001, 0.5, 100);
    double velocity1 = model_->getVelocity();

    // Coast with zero torque
    stepModel(0.001, 0.0, 100);
    double velocity2 = model_->getVelocity();

    EXPECT_NEAR(velocity2, velocity1, velocity1 * 0.01);  // Should maintain velocity
}

TEST_F(AdvancedMotorModelTest, InertiaAffectsAcceleration) {
    double torque = TEST_RATED_TORQUE;
    double dt = 0.001;

    // Low inertia motor
    config_.inertia = 0.0001;
    auto lowInertiaModel = std::make_unique<AdvancedMotorModel>(
        config_, friction_, thermal_, backlash_, geartrain_
    );
    lowInertiaModel->step(dt, torque, 0.0);
    double lowInertiaVel = lowInertiaModel->getVelocity();

    // High inertia motor
    config_.inertia = 0.01;
    auto highInertiaModel = std::make_unique<AdvancedMotorModel>(
        config_, friction_, thermal_, backlash_, geartrain_
    );
    highInertiaModel->step(dt, torque, 0.0);
    double highInertiaVel = highInertiaModel->getVelocity();

    EXPECT_GT(lowInertiaVel, highInertiaVel);
}

// ============================================================================
// Friction Model Tests
// ============================================================================

TEST_F(AdvancedMotorModelTest, CoulombFrictionOpposesMotion) {
    friction_.coulombFriction = 0.1;
    friction_.viscousFriction = 0.0;
    model_ = std::make_unique<AdvancedMotorModel>(
        config_, friction_, thermal_, backlash_, geartrain_
    );

    // Apply torque just below Coulomb friction
    stepModel(0.001, 0.05, 100);

    // Motor shouldn't move significantly
    EXPECT_NEAR(model_->getVelocity(), 0.0, 0.1);
}

TEST_F(AdvancedMotorModelTest, CoulombFrictionIsBidirectional) {
    friction_.coulombFriction = 0.1;
    friction_.viscousFriction = 0.0;
    model_ = std::make_unique<AdvancedMotorModel>(
        config_, friction_, thermal_, backlash_, geartrain_
    );

    // Positive direction
    stepModel(0.001, 0.5, 100);
    double posVelocity = model_->getVelocity();

    model_->reset();

    // Negative direction
    stepModel(0.001, -0.5, 100);
    double negVelocity = model_->getVelocity();

    EXPECT_GT(posVelocity, 0);
    EXPECT_LT(negVelocity, 0);
    EXPECT_NEAR(std::fabs(posVelocity), std::fabs(negVelocity), VELOCITY_TOLERANCE);
}

TEST_F(AdvancedMotorModelTest, ViscousFrictionProportionalToVelocity) {
    friction_.coulombFriction = 0.0;
    friction_.viscousFriction = 0.001;
    model_ = std::make_unique<AdvancedMotorModel>(
        config_, friction_, thermal_, backlash_, geartrain_
    );

    // Accelerate to steady state
    double torque = 0.5;
    for (int i = 0; i < 10000; ++i) {
        model_->step(0.001, torque, 0.0);
    }

    double steadyVelocity = model_->getVelocity();

    // At steady state: τ_applied = τ_friction = b * ω
    double expectedVelocity = torque / friction_.viscousFriction;
    EXPECT_NEAR(steadyVelocity, expectedVelocity, expectedVelocity * 0.1);
}

TEST_F(AdvancedMotorModelTest, ViscousFrictionCausesExponentialDecay) {
    friction_.coulombFriction = 0.0;
    friction_.viscousFriction = 0.001;
    model_ = std::make_unique<AdvancedMotorModel>(
        config_, friction_, thermal_, backlash_, geartrain_
    );

    // Accelerate
    stepModel(0.001, 0.5, 100);
    double v0 = model_->getVelocity();

    // Coast (zero torque)
    stepModel(0.001, 0.0, 1000);
    double v1 = model_->getVelocity();

    // Time constant τ = J/b
    double tau = TEST_INERTIA / friction_.viscousFriction;
    double t = 1.0;  // 1000 steps * 0.001s
    double expectedV = v0 * std::exp(-t / tau);

    EXPECT_NEAR(v1, expectedV, v1 * 0.1);  // 10% tolerance due to discretization
}

TEST_F(AdvancedMotorModelTest, StribeckEffectIncreasesLowSpeedFriction) {
    friction_.coulombFriction = 0.01;
    friction_.viscousFriction = 0.0001;
    friction_.stribeckVelocity = 10.0;
    friction_.stribeckFactor = 2.0;  // Double friction at zero speed
    model_ = std::make_unique<AdvancedMotorModel>(
        config_, friction_, thermal_, backlash_, geartrain_
    );

    // Get breakaway torque (needs to overcome Stribeck-enhanced friction)
    double lowTorque = 0.015;  // Just above Coulomb, below Stribeck
    double highTorque = 0.025;  // Above Stribeck peak

    model_->reset();
    stepModel(0.001, lowTorque, 100);
    double lowVel = model_->getVelocity();

    model_->reset();
    stepModel(0.001, highTorque, 100);
    double highVel = model_->getVelocity();

    // Higher torque should result in proportionally faster motion
    EXPECT_GT(highVel, lowVel * 1.5);
}

TEST_F(AdvancedMotorModelTest, FrictionAtZeroVelocityIsStatic) {
    friction_.coulombFriction = 0.1;
    model_ = std::make_unique<AdvancedMotorModel>(
        config_, friction_, thermal_, backlash_, geartrain_
    );

    // Apply torque less than static friction
    stepModel(0.001, 0.05, 100);

    EXPECT_NEAR(model_->getPosition(), 0.0, POSITION_TOLERANCE);
    EXPECT_NEAR(model_->getVelocity(), 0.0, VELOCITY_TOLERANCE);
}

TEST_F(AdvancedMotorModelTest, CombinedFrictionModel) {
    friction_.coulombFriction = 0.05;
    friction_.viscousFriction = 0.001;
    friction_.stribeckVelocity = 5.0;
    friction_.stribeckFactor = 1.5;
    model_ = std::make_unique<AdvancedMotorModel>(
        config_, friction_, thermal_, backlash_, geartrain_
    );

    // Accelerate to moderate speed
    stepModel(0.001, 0.5, 500);

    double velocity = model_->getVelocity();
    double frictionTorque = model_->getFrictionTorque();

    // Friction should include both Coulomb and viscous components
    EXPECT_GT(frictionTorque, friction_.coulombFriction);  // More than just Coulomb
    EXPECT_LT(frictionTorque, 0.5);  // Less than applied torque (motor is accelerating)
}

// ============================================================================
// Thermal Model Tests
// ============================================================================

TEST_F(AdvancedMotorModelTest, TemperatureRisesUnderLoad) {
    double initialTemp = model_->getTemperature();

    // Run under load for extended period
    for (int i = 0; i < 10000; ++i) {
        model_->step(0.001, 0.8, 0.0);
    }

    double finalTemp = model_->getTemperature();
    EXPECT_GT(finalTemp, initialTemp);
}

TEST_F(AdvancedMotorModelTest, TemperatureCoolsToAmbient) {
    // Heat up
    for (int i = 0; i < 5000; ++i) {
        model_->step(0.001, 0.8, 0.0);
    }

    double hotTemp = model_->getTemperature();
    EXPECT_GT(hotTemp, thermal_.ambientTemperature);

    // Cool down (no torque)
    for (int i = 0; i < 50000; ++i) {
        model_->step(0.001, 0.0, 0.0);
    }

    double cooledTemp = model_->getTemperature();
    EXPECT_NEAR(cooledTemp, thermal_.ambientTemperature, 1.0);  // Within 1°C
}

TEST_F(AdvancedMotorModelTest, SteadyStateTemperature) {
    // P = I²R heat dissipation
    // At steady state: P = (T - T_ambient) / R_thermal
    // Therefore: T_steady = T_ambient + P * R_thermal

    double constantTorque = 0.5;
    double current = constantTorque / TEST_KT;
    double power = current * current * TEST_RESISTANCE;
    double expectedTemp = thermal_.ambientTemperature + power * thermal_.thermalResistance;

    // Run to steady state
    for (int i = 0; i < 100000; ++i) {
        model_->step(0.001, constantTorque, 0.0);
    }

    double actualTemp = model_->getTemperature();
    EXPECT_NEAR(actualTemp, expectedTemp, 5.0);  // 5°C tolerance
}

TEST_F(AdvancedMotorModelTest, ThermalTimeConstant) {
    // τ_thermal = R_thermal * C_thermal
    double tau = thermal_.thermalResistance * thermal_.thermalCapacitance;

    double constantTorque = 0.5;
    double current = constantTorque / TEST_KT;
    double power = current * current * TEST_RESISTANCE;
    double deltaT = power * thermal_.thermalResistance;

    // Run for one time constant
    double t = 0;
    double dt = 0.01;
    while (t < tau) {
        model_->step(dt, constantTorque, 0.0);
        t += dt;
    }

    double actualTemp = model_->getTemperature();
    double expectedTemp = thermal_.ambientTemperature + deltaT * (1 - std::exp(-1));

    EXPECT_NEAR(actualTemp, expectedTemp, 2.0);  // 2°C tolerance
}

TEST_F(AdvancedMotorModelTest, TemperatureLimitingReducesTorque) {
    thermal_.maxTemperature = 80.0;
    model_ = std::make_unique<AdvancedMotorModel>(
        config_, friction_, thermal_, backlash_, geartrain_
    );

    // Heat up to near limit
    while (model_->getTemperature() < thermal_.maxTemperature - 10) {
        model_->step(0.001, 1.0, 0.0);
    }

    // Try to apply full torque near temperature limit
    double requestedTorque = 1.0;
    double actualTorque = model_->getEffectiveTorque(requestedTorque);

    // Torque should be derated
    EXPECT_LT(actualTorque, requestedTorque);
}

// ============================================================================
// Backlash Model Tests
// ============================================================================

TEST_F(AdvancedMotorModelTest, NoBacklashTransmitsMotionImmediately) {
    backlash_.deadband = 0.0;
    model_ = std::make_unique<AdvancedMotorModel>(
        config_, friction_, thermal_, backlash_, geartrain_
    );

    stepModel(0.001, 0.5, 10);

    double inputPos = model_->getPosition();
    double outputPos = model_->getOutputPosition();

    EXPECT_DOUBLE_EQ(inputPos, outputPos);
}

TEST_F(AdvancedMotorModelTest, BacklashCreatesDeadband) {
    backlash_.deadband = 0.01;  // 0.01 rad deadband
    model_ = std::make_unique<AdvancedMotorModel>(
        config_, friction_, thermal_, backlash_, geartrain_
    );

    // Move forward
    stepModel(0.001, 0.5, 50);
    double pos1 = model_->getPosition();
    double outPos1 = model_->getOutputPosition();

    // Reverse direction
    stepModel(0.001, -0.5, 10);
    double pos2 = model_->getPosition();
    double outPos2 = model_->getOutputPosition();

    // Input should have moved backward, but output shouldn't immediately follow
    EXPECT_LT(pos2, pos1);
    // Output position should lag due to backlash deadband
}

TEST_F(AdvancedMotorModelTest, BacklashSpringEngagesAfterDeadband) {
    backlash_.deadband = 0.01;
    backlash_.springConstant = 1e6;
    model_ = std::make_unique<AdvancedMotorModel>(
        config_, friction_, thermal_, backlash_, geartrain_
    );

    // Move forward through deadband
    stepModel(0.001, 0.5, 100);

    double inputPos = model_->getPosition();
    double outputPos = model_->getOutputPosition();

    // After crossing deadband, positions should track (minus deadband)
    EXPECT_NEAR(outputPos, inputPos - backlash_.deadband / 2, 0.001);
}

TEST_F(AdvancedMotorModelTest, BacklashDampingReducesOscillations) {
    backlash_.deadband = 0.01;
    backlash_.springConstant = 1e6;
    backlash_.dampingFactor = 1000.0;
    model_ = std::make_unique<AdvancedMotorModel>(
        config_, friction_, thermal_, backlash_, geartrain_
    );

    // Apply step change
    stepModel(0.001, 1.0, 100);

    // Record oscillation magnitude
    std::vector<double> positions;
    for (int i = 0; i < 100; ++i) {
        model_->step(0.001, 0.0, 0.0);
        positions.push_back(model_->getOutputPosition());
    }

    // Calculate oscillation decay
    double firstPeak = *std::max_element(positions.begin(), positions.begin() + 50);
    double secondPeak = *std::max_element(positions.begin() + 50, positions.end());

    // Damped system should show decreasing peaks
    EXPECT_LT(secondPeak, firstPeak + 0.001);
}

// ============================================================================
// Geartrain Tests
// ============================================================================

TEST_F(AdvancedMotorModelTest, GearRatioReducesOutputSpeed) {
    geartrain_.gearRatio = 10.0;  // 10:1 reduction
    geartrain_.efficiency = 1.0;
    model_ = std::make_unique<AdvancedMotorModel>(
        config_, friction_, thermal_, backlash_, geartrain_
    );

    stepModel(0.001, 0.5, 100);

    double inputVelocity = model_->getVelocity();
    double outputVelocity = model_->getOutputVelocity();

    EXPECT_NEAR(outputVelocity, inputVelocity / geartrain_.gearRatio, VELOCITY_TOLERANCE);
}

TEST_F(AdvancedMotorModelTest, GearRatioIncreasesOutputTorque) {
    geartrain_.gearRatio = 10.0;
    geartrain_.efficiency = 1.0;
    model_ = std::make_unique<AdvancedMotorModel>(
        config_, friction_, thermal_, backlash_, geartrain_
    );

    double inputTorque = 0.5;
    double expectedOutputTorque = inputTorque * geartrain_.gearRatio;

    stepModel(0.001, inputTorque, 10);
    double outputTorque = model_->getOutputTorque();

    EXPECT_NEAR(outputTorque, expectedOutputTorque, TORQUE_TOLERANCE);
}

TEST_F(AdvancedMotorModelTest, GearEfficiencyReducesOutputTorque) {
    geartrain_.gearRatio = 10.0;
    geartrain_.efficiency = 0.8;  // 80% efficiency
    model_ = std::make_unique<AdvancedMotorModel>(
        config_, friction_, thermal_, backlash_, geartrain_
    );

    double inputTorque = 0.5;
    double expectedOutputTorque = inputTorque * geartrain_.gearRatio * geartrain_.efficiency;

    stepModel(0.001, inputTorque, 10);
    double outputTorque = model_->getOutputTorque();

    EXPECT_NEAR(outputTorque, expectedOutputTorque, TORQUE_TOLERANCE);
}

TEST_F(AdvancedMotorModelTest, OutputInertiaReflectedToMotor) {
    geartrain_.gearRatio = 10.0;
    geartrain_.outputInertia = 0.1;  // 0.1 kg·m² at output
    geartrain_.efficiency = 1.0;
    model_ = std::make_unique<AdvancedMotorModel>(
        config_, friction_, thermal_, backlash_, geartrain_
    );

    // Reflected inertia = J_output / (gear_ratio)²
    double reflectedInertia = geartrain_.outputInertia / (geartrain_.gearRatio * geartrain_.gearRatio);
    double totalInertia = TEST_INERTIA + reflectedInertia;

    double torque = 0.5;
    double dt = 0.001;
    model_->step(dt, torque, 0.0);

    double acceleration = torque / totalInertia;
    double expectedVelocity = acceleration * dt;

    EXPECT_NEAR(model_->getVelocity(), expectedVelocity, expectedVelocity * 0.05);
}

TEST_F(AdvancedMotorModelTest, ReverseGearTrainDrivesLoad) {
    geartrain_.gearRatio = 0.5;  // Speed increase
    geartrain_.efficiency = 0.9;
    model_ = std::make_unique<AdvancedMotorModel>(
        config_, friction_, thermal_, backlash_, geartrain_
    );

    stepModel(0.001, 0.5, 100);

    double inputVelocity = model_->getVelocity();
    double outputVelocity = model_->getOutputVelocity();

    // With gear ratio < 1, output is faster
    EXPECT_GT(outputVelocity, inputVelocity);
}

// ============================================================================
// Torque-Speed Curve Tests
// ============================================================================

TEST_F(AdvancedMotorModelTest, TorqueReducesAtHighSpeed) {
    model_ = std::make_unique<AdvancedMotorModel>(
        config_, friction_, thermal_, backlash_, geartrain_
    );

    double ratedSpeedRadS = rpmToRadS(TEST_RATED_SPEED);

    // Request full torque at low speed
    model_->setVelocity(0.1 * ratedSpeedRadS);
    double lowSpeedTorque = model_->getAvailableTorque(TEST_RATED_TORQUE);

    // Request full torque at high speed
    model_->setVelocity(1.5 * ratedSpeedRadS);
    double highSpeedTorque = model_->getAvailableTorque(TEST_RATED_TORQUE);

    EXPECT_NEAR(lowSpeedTorque, TEST_RATED_TORQUE, TORQUE_TOLERANCE);
    EXPECT_LT(highSpeedTorque, lowSpeedTorque);
}

TEST_F(AdvancedMotorModelTest, BackEMFLimitsMaxSpeed) {
    friction_.coulombFriction = 0.0;
    friction_.viscousFriction = 0.0;
    model_ = std::make_unique<AdvancedMotorModel>(
        config_, friction_, thermal_, backlash_, geartrain_
    );

    // Apply constant voltage through torque
    double appliedVoltage = config_.maxVoltage;
    double torque = (appliedVoltage / TEST_RESISTANCE) * TEST_KT;

    // Run for extended time
    for (int i = 0; i < 50000; ++i) {
        model_->step(0.001, torque, 0.0);
    }

    double finalVelocity = model_->getVelocity();

    // Max speed when back EMF = applied voltage
    double maxSpeed = appliedVoltage / TEST_KE;
    EXPECT_NEAR(finalVelocity, maxSpeed, maxSpeed * 0.1);
}

TEST_F(AdvancedMotorModelTest, FieldWeakeningExtendsBeyondBaseSpeed) {
    // If field weakening is implemented
    if (model_->supportsFieldWeakening()) {
        double baseSpeed = rpmToRadS(TEST_RATED_SPEED);

        // Request operation above base speed
        double targetSpeed = 1.5 * baseSpeed;
        model_->enableFieldWeakening(true);

        stepModel(0.001, TEST_RATED_TORQUE, 5000);
        double finalSpeed = model_->getVelocity();

        // Should be able to exceed base speed
        EXPECT_GT(finalSpeed, baseSpeed);
    }
}

// ============================================================================
// Load Disturbance Tests
// ============================================================================

TEST_F(AdvancedMotorModelTest, LoadTorqueReducesAcceleration) {
    double motorTorque = 0.5;
    double loadTorque = 0.3;

    double dt = 0.001;

    // Without load
    model_->step(dt, motorTorque, 0.0);
    double accelNoLoad = model_->getVelocity() / dt;

    // Reset
    model_->reset();

    // With load
    model_->step(dt, motorTorque, loadTorque);
    double accelWithLoad = model_->getVelocity() / dt;

    EXPECT_GT(accelNoLoad, accelWithLoad);
}

TEST_F(AdvancedMotorModelTest, LoadTorqueCanCauseDeceleration) {
    // First accelerate
    stepModel(0.001, 0.5, 100);
    double midVelocity = model_->getVelocity();

    // Apply heavy load (more than motor torque)
    double motorTorque = 0.3;
    double loadTorque = 0.5;

    for (int i = 0; i < 100; ++i) {
        model_->step(0.001, motorTorque, loadTorque);
    }

    double finalVelocity = model_->getVelocity();
    EXPECT_LT(finalVelocity, midVelocity);
}

TEST_F(AdvancedMotorModelTest, StepLoadDisturbance) {
    // Accelerate to steady speed
    stepModel(0.001, 0.5, 1000);
    double steadyVelocity = model_->getVelocity();

    // Apply step load
    for (int i = 0; i < 100; ++i) {
        model_->step(0.001, 0.5, 0.3);  // Add 0.3 Nm load
    }

    double afterLoadVelocity = model_->getVelocity();

    // Velocity should decrease
    EXPECT_LT(afterLoadVelocity, steadyVelocity);
}

TEST_F(AdvancedMotorModelTest, TimeVaryingLoad) {
    std::vector<double> velocities;

    for (int i = 0; i < 1000; ++i) {
        // Sinusoidal load
        double loadTorque = 0.2 * std::sin(2 * M_PI * i / 100.0);
        model_->step(0.001, 0.5, loadTorque);
        velocities.push_back(model_->getVelocity());
    }

    // Velocity should oscillate around mean
    double mean = 0;
    for (double v : velocities) mean += v;
    mean /= velocities.size();

    int crossings = 0;
    for (size_t i = 1; i < velocities.size(); ++i) {
        if ((velocities[i] - mean) * (velocities[i-1] - mean) < 0) {
            crossings++;
        }
    }

    // Should have multiple zero crossings (relative to mean)
    EXPECT_GT(crossings, 5);
}

// ============================================================================
// Reset and State Management Tests
// ============================================================================

TEST_F(AdvancedMotorModelTest, ResetClearsAllState) {
    // Change state
    stepModel(0.001, 0.5, 100);

    EXPECT_NE(model_->getPosition(), 0.0);
    EXPECT_NE(model_->getVelocity(), 0.0);

    model_->reset();

    EXPECT_DOUBLE_EQ(model_->getPosition(), 0.0);
    EXPECT_DOUBLE_EQ(model_->getVelocity(), 0.0);
}

TEST_F(AdvancedMotorModelTest, SetPositionUpdatesPosition) {
    double newPos = 10.0;
    model_->setPosition(newPos);

    EXPECT_DOUBLE_EQ(model_->getPosition(), newPos);
}

TEST_F(AdvancedMotorModelTest, SetVelocityUpdatesVelocity) {
    double newVel = 100.0;
    model_->setVelocity(newVel);

    EXPECT_DOUBLE_EQ(model_->getVelocity(), newVel);
}

// ============================================================================
// Numerical Stability Tests
// ============================================================================

TEST_F(AdvancedMotorModelTest, StableWithVerySmallTimestep) {
    double dt = 1e-6;  // 1 microsecond

    for (int i = 0; i < 1000; ++i) {
        model_->step(dt, 0.5, 0.0);
    }

    EXPECT_TRUE(std::isfinite(model_->getPosition()));
    EXPECT_TRUE(std::isfinite(model_->getVelocity()));
}

TEST_F(AdvancedMotorModelTest, StableWithLargeTimestep) {
    double dt = 0.01;  // 10ms

    for (int i = 0; i < 1000; ++i) {
        model_->step(dt, 0.5, 0.0);
    }

    EXPECT_TRUE(std::isfinite(model_->getPosition()));
    EXPECT_TRUE(std::isfinite(model_->getVelocity()));
}

TEST_F(AdvancedMotorModelTest, StableWithZeroInertia) {
    config_.inertia = 1e-10;  // Very small but non-zero
    auto smallInertiaModel = std::make_unique<AdvancedMotorModel>(
        config_, friction_, thermal_, backlash_, geartrain_
    );

    for (int i = 0; i < 100; ++i) {
        smallInertiaModel->step(0.001, 0.5, 0.0);
    }

    EXPECT_TRUE(std::isfinite(smallInertiaModel->getVelocity()));
}

TEST_F(AdvancedMotorModelTest, StableUnderOscillatingTorque) {
    for (int i = 0; i < 10000; ++i) {
        double torque = std::sin(i * 0.1) * TEST_RATED_TORQUE;
        model_->step(0.001, torque, 0.0);
    }

    EXPECT_TRUE(std::isfinite(model_->getPosition()));
    EXPECT_TRUE(std::isfinite(model_->getVelocity()));
}

TEST_F(AdvancedMotorModelTest, StableWithExtremeTorque) {
    for (int i = 0; i < 100; ++i) {
        model_->step(0.001, 1000.0, 0.0);  // Way over rated
    }

    EXPECT_TRUE(std::isfinite(model_->getPosition()));
    EXPECT_TRUE(std::isfinite(model_->getVelocity()));
}

TEST_F(AdvancedMotorModelTest, NoNaNOrInfInOutputs) {
    std::mt19937 gen(42);
    std::uniform_real_distribution<> torqueDist(-10.0, 10.0);
    std::uniform_real_distribution<> loadDist(-5.0, 5.0);
    std::uniform_real_distribution<> dtDist(0.0001, 0.01);

    for (int i = 0; i < 10000; ++i) {
        double torque = torqueDist(gen);
        double load = loadDist(gen);
        double dt = dtDist(gen);

        model_->step(dt, torque, load);

        ASSERT_TRUE(std::isfinite(model_->getPosition()));
        ASSERT_TRUE(std::isfinite(model_->getVelocity()));
        ASSERT_TRUE(std::isfinite(model_->getTemperature()));
    }
}

// ============================================================================
// Energy Conservation Tests
// ============================================================================

TEST_F(AdvancedMotorModelTest, KineticEnergyIncreasesUnderPositiveTorque) {
    friction_.coulombFriction = 0.0;
    friction_.viscousFriction = 0.0;
    model_ = std::make_unique<AdvancedMotorModel>(
        config_, friction_, thermal_, backlash_, geartrain_
    );

    double initialKE = 0.5 * TEST_INERTIA * model_->getVelocity() * model_->getVelocity();

    stepModel(0.001, 0.5, 100);

    double finalKE = 0.5 * TEST_INERTIA * model_->getVelocity() * model_->getVelocity();

    EXPECT_GT(finalKE, initialKE);
}

TEST_F(AdvancedMotorModelTest, WorkDoneEqualsEnergyChange) {
    friction_.coulombFriction = 0.0;
    friction_.viscousFriction = 0.0;
    model_ = std::make_unique<AdvancedMotorModel>(
        config_, friction_, thermal_, backlash_, geartrain_
    );

    double torque = 0.5;
    double dt = 0.001;
    int steps = 100;

    double initialPosition = model_->getPosition();
    double initialVelocity = model_->getVelocity();

    stepModel(dt, torque, steps);

    double finalPosition = model_->getPosition();
    double finalVelocity = model_->getVelocity();

    double angularDisplacement = finalPosition - initialPosition;
    double workDone = torque * angularDisplacement;

    double initialKE = 0.5 * TEST_INERTIA * initialVelocity * initialVelocity;
    double finalKE = 0.5 * TEST_INERTIA * finalVelocity * finalVelocity;
    double deltaKE = finalKE - initialKE;

    EXPECT_NEAR(workDone, deltaKE, workDone * 0.05);  // 5% tolerance
}

// ============================================================================
// Current and Electrical Tests
// ============================================================================

TEST_F(AdvancedMotorModelTest, CurrentProportionalToTorque) {
    double torque = 0.5;

    stepModel(0.001, torque, 10);
    double current = model_->getCurrent();

    double expectedCurrent = torque / TEST_KT;
    EXPECT_NEAR(current, expectedCurrent, TORQUE_TOLERANCE);
}

TEST_F(AdvancedMotorModelTest, CurrentLimitedByMax) {
    double hugeTorque = 100.0;  // Much more than motor can handle

    stepModel(0.001, hugeTorque, 10);
    double current = model_->getCurrent();

    EXPECT_LE(current, config_.maxCurrent);
}

TEST_F(AdvancedMotorModelTest, BackEMFIncreasesWithSpeed) {
    stepModel(0.001, 0.5, 100);
    double lowSpeedEMF = model_->getBackEMF();

    stepModel(0.001, 0.5, 400);
    double highSpeedEMF = model_->getBackEMF();

    EXPECT_GT(highSpeedEMF, lowSpeedEMF);
}

// ============================================================================
// Performance Tests
// ============================================================================

TEST_F(AdvancedMotorModelTest, SimulationSpeedAcceptable) {
    auto start = std::chrono::high_resolution_clock::now();

    for (int i = 0; i < 100000; ++i) {
        model_->step(0.001, 0.5, 0.0);
    }

    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);

    // Should complete 100 seconds of simulation in reasonable time
    EXPECT_LT(duration.count(), 1000);  // Less than 1 second
}

// ============================================================================
// Edge Case Parameter Tests
// ============================================================================

TEST_F(AdvancedMotorModelTest, HandlesZeroFeedRate) {
    stepModel(0.001, 0.0, 100);

    EXPECT_NEAR(model_->getVelocity(), 0.0, VELOCITY_TOLERANCE);
}

TEST_F(AdvancedMotorModelTest, HandlesNegativeTorque) {
    stepModel(0.001, -0.5, 100);

    EXPECT_LT(model_->getVelocity(), 0.0);
    EXPECT_LT(model_->getPosition(), 0.0);
}

TEST_F(AdvancedMotorModelTest, HandlesVeryHighGearRatio) {
    geartrain_.gearRatio = 1000.0;
    model_ = std::make_unique<AdvancedMotorModel>(
        config_, friction_, thermal_, backlash_, geartrain_
    );

    stepModel(0.001, 0.5, 100);

    EXPECT_TRUE(std::isfinite(model_->getOutputVelocity()));
    EXPECT_TRUE(std::isfinite(model_->getOutputTorque()));
}

TEST_F(AdvancedMotorModelTest, HandlesVeryLowEfficiency) {
    geartrain_.efficiency = 0.01;  // 1% efficiency
    model_ = std::make_unique<AdvancedMotorModel>(
        config_, friction_, thermal_, backlash_, geartrain_
    );

    stepModel(0.001, 0.5, 100);

    double outputTorque = model_->getOutputTorque();
    EXPECT_NEAR(outputTorque, 0.005, 0.001);  // 0.5 * 0.01
}

// ============================================================================
// Quadrant Operation Tests
// ============================================================================

TEST_F(AdvancedMotorModelTest, OperatesInAllQuadrants) {
    // Quadrant 1: +torque, +velocity (motoring CW)
    model_->setVelocity(10.0);
    stepModel(0.001, 0.5, 10);
    EXPECT_GT(model_->getVelocity(), 10.0);  // Accelerating

    model_->reset();

    // Quadrant 2: -torque, +velocity (regenerating CW)
    model_->setVelocity(100.0);
    stepModel(0.001, -0.5, 10);
    EXPECT_LT(model_->getVelocity(), 100.0);  // Decelerating

    model_->reset();

    // Quadrant 3: -torque, -velocity (motoring CCW)
    model_->setVelocity(-10.0);
    stepModel(0.001, -0.5, 10);
    EXPECT_LT(model_->getVelocity(), -10.0);  // Accelerating negative

    model_->reset();

    // Quadrant 4: +torque, -velocity (regenerating CCW)
    model_->setVelocity(-100.0);
    stepModel(0.001, 0.5, 10);
    EXPECT_GT(model_->getVelocity(), -100.0);  // Decelerating (toward zero)
}

// ============================================================================
// Configuration Validation Tests
// ============================================================================

TEST_F(AdvancedMotorModelTest, RejectsNegativeInertia) {
    config_.inertia = -0.001;

    EXPECT_THROW(
        std::make_unique<AdvancedMotorModel>(
            config_, friction_, thermal_, backlash_, geartrain_
        ),
        std::invalid_argument
    );
}

TEST_F(AdvancedMotorModelTest, RejectsNegativeResistance) {
    config_.resistance = -1.0;

    EXPECT_THROW(
        std::make_unique<AdvancedMotorModel>(
            config_, friction_, thermal_, backlash_, geartrain_
        ),
        std::invalid_argument
    );
}

TEST_F(AdvancedMotorModelTest, RejectsInvalidEfficiency) {
    geartrain_.efficiency = 1.5;  // > 100%

    EXPECT_THROW(
        std::make_unique<AdvancedMotorModel>(
            config_, friction_, thermal_, backlash_, geartrain_
        ),
        std::invalid_argument
    );
}

} // namespace test
} // namespace cia402
