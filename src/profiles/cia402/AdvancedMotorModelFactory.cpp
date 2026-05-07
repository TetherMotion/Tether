/**
 * @file AdvancedMotorModelFactory.cpp
 * @brief Sensors, fault detection, and factory functions for AdvancedMotorModel
 * 
 * Split from AdvancedMotorModel.cpp for maintainability.
 */

#include "profiles/cia402/AdvancedMotorModel.hpp"
#include <algorithm>
#include <cmath>
#include <random>

namespace CiA402 {
namespace Motor {

static constexpr double PI = 3.14159265358979323846;
static constexpr double TWO_PI = 2.0 * PI;
static constexpr double RPM_TO_RADS = TWO_PI / 60.0;
static constexpr double RADS_TO_RPM = 60.0 / TWO_PI;

// ============================================================================
// Sensor Updates
// ============================================================================

void AdvancedMotorModel::updateSensors() {
    updateEncoder();
    
    bool prevPosLimit = state_.sensors.positiveLimitActive;
    bool prevNegLimit = state_.sensors.negativeLimitActive;
    bool prevHome = state_.sensors.homeActive;
    
    if (sensorConfig_.positiveLimitEnabled) {
        bool triggered = state_.load.position >= sensorConfig_.positiveLimitPosition;
        if (errorInjection_.enabled && errorInjection_.positiveLimitDisconnect) {
            triggered = sensorConfig_.positiveLimitNC;
        }
        state_.sensors.positiveLimitActive = sensorConfig_.positiveLimitNC ? !triggered : triggered;
    }
    
    if (sensorConfig_.negativeLimitEnabled) {
        bool triggered = state_.load.position <= sensorConfig_.negativeLimitPosition;
        if (errorInjection_.enabled && errorInjection_.negativeLimitDisconnect) {
            triggered = sensorConfig_.negativeLimitNC;
        }
        state_.sensors.negativeLimitActive = sensorConfig_.negativeLimitNC ? !triggered : triggered;
    }
    
    if (sensorConfig_.homeEnabled) {
        bool inHome = std::abs(state_.load.position - sensorConfig_.homePosition) < 
                      sensorConfig_.homeWidth / 2.0;
        if (errorInjection_.enabled && errorInjection_.homeDisconnect) {
            inHome = sensorConfig_.homeNC;
        }
        state_.sensors.homeActive = sensorConfig_.homeNC ? !inHome : inHome;
    }
    
    state_.sensors.indexPulse = false;
    if (sensorConfig_.indexEnabled && !errorInjection_.indexDisconnect) {
        bool crossed = (prevMotorPosition_ < sensorConfig_.indexPosition &&
                       state_.motor.position >= sensorConfig_.indexPosition) ||
                      (prevMotorPosition_ > sensorConfig_.indexPosition &&
                       state_.motor.position <= sensorConfig_.indexPosition);
        if (crossed) {
            state_.sensors.indexPulse = true;
            if (indexCallback_) {
                indexCallback_(state_.motor.position);
            }
        }
    }
    
    if ((state_.sensors.positiveLimitActive != prevPosLimit ||
         state_.sensors.negativeLimitActive != prevNegLimit) && limitCallback_) {
        limitCallback_(state_.sensors.positiveLimitActive, state_.sensors.negativeLimitActive);
    }
    
    if (state_.sensors.homeActive != prevHome && homeCallback_) {
        homeCallback_(state_.sensors.homeActive);
    }
}

void AdvancedMotorModel::updateEncoder() {
    double countsPerRad = motorParams_.mechanical.encoderResolution / TWO_PI;
    int32_t counts = static_cast<int32_t>(state_.motor.position * countsPerRad);
    
    if (errorInjection_.enabled && errorInjection_.encoderNoiseAmplitude > 0) {
        static std::random_device rd;
        static std::mt19937 gen(rd());
        std::uniform_real_distribution<> dis(-errorInjection_.encoderNoiseAmplitude,
                                              errorInjection_.encoderNoiseAmplitude);
        counts += static_cast<int32_t>(dis(gen));
    }
    
    if (errorInjection_.enabled && errorInjection_.simulateEncoderFault) {
        counts = 0;
    }
    
    state_.sensors.encoderPosition = counts;
    
    state_.sensors.outputPosition = static_cast<int32_t>(
        state_.load.position * countsPerRad / geartrainParams_.gearRatio);
}

// ============================================================================
// Fault Detection
// ============================================================================

void AdvancedMotorModel::checkFaults() {
    if (std::abs(state_.motor.current) > motorParams_.electrical.peakCurrent * 1.2) {
        triggerFault(0x2310);
    }
    
    if (state_.thermal.windingOvertemp) {
        triggerFault(0x4310);
    }
    if (state_.thermal.gearboxOvertemp) {
        triggerFault(0x4380);
    }
    
    if (controlMode_ == ControlMode::Position) {
        double error = std::abs(targetPosition_ - state_.motor.position);
        // Configurable threshold
    }
    
    if (errorInjection_.enabled && errorInjection_.simulateEncoderFault) {
        triggerFault(0x7300);
    }
}

void AdvancedMotorModel::triggerFault(uint16_t code) {
    if (!state_.hasFault) {
        state_.hasFault = true;
        state_.faultCode = code;
        
        if (faultCallback_) {
            faultCallback_(code);
        }
    }
}

// ============================================================================
// Factory Functions
// ============================================================================

namespace Factory {

MotorParams createBLDCServoMotor(double ratedTorque, double maxSpeedRPM, int encoderBits) {
    MotorParams params;
    
    params.torqueSpeed.ratedTorque = ratedTorque;
    params.torqueSpeed.peakTorque = ratedTorque * 3.0;
    params.torqueSpeed.stallTorque = ratedTorque * 1.2;
    params.torqueSpeed.cornerSpeed = maxSpeedRPM * 0.5 * RPM_TO_RADS;
    params.torqueSpeed.maxSpeed = maxSpeedRPM * RPM_TO_RADS;
    
    params.electrical.torqueConstant = ratedTorque / 5.0;
    params.electrical.backEMFConstant = params.electrical.torqueConstant;
    params.electrical.windingResistance = 1.0;
    params.electrical.ratedCurrent = 5.0;
    params.electrical.peakCurrent = 15.0;
    params.electrical.supplyVoltage = 48.0;
    
    params.mechanical.rotorInertia = ratedTorque * 0.0001;
    params.mechanical.encoderResolution = 1 << encoderBits;
    params.mechanical.friction.staticFriction = ratedTorque * 0.02;
    params.mechanical.friction.coulombFriction = ratedTorque * 0.01;
    params.mechanical.friction.viscousCoeff = ratedTorque * 0.001;
    
    return params;
}

MotorParams createStepperMotor(double holdingTorque, int stepsPerRev, int microsteps) {
    MotorParams params;
    
    params.torqueSpeed.ratedTorque = holdingTorque * 0.7;
    params.torqueSpeed.peakTorque = holdingTorque;
    params.torqueSpeed.stallTorque = holdingTorque;
    params.torqueSpeed.cornerSpeed = 10.0;
    params.torqueSpeed.maxSpeed = 200.0;
    
    params.electrical.torqueConstant = holdingTorque / 2.0;
    params.electrical.backEMFConstant = params.electrical.torqueConstant;
    params.electrical.windingResistance = 2.0;
    params.electrical.ratedCurrent = 2.0;
    params.electrical.peakCurrent = 3.0;
    
    params.mechanical.rotorInertia = 0.0003;
    params.mechanical.encoderResolution = stepsPerRev * microsteps;
    params.mechanical.friction.staticFriction = 0.05;
    params.mechanical.friction.coulombFriction = 0.02;
    
    return params;
}

GeartrainParams createPlanetaryGearbox(double ratio, int stages, bool highEfficiency) {
    GeartrainParams params;
    
    params.gearRatio = ratio;
    
    double singleStageEff = highEfficiency ? 0.97 : 0.94;
    params.forwardEfficiency = std::pow(singleStageEff, stages);
    params.backwardEfficiency = params.forwardEfficiency * 0.95;
    
    params.gearInertia = 0.00001 * ratio;
    
    params.backdrivable = true;
    
    params.friction.staticFriction = 0.1;
    params.friction.coulombFriction = 0.05;
    params.friction.viscousCoeff = 0.001;
    
    return params;
}

GeartrainParams createHarmonicDrive(double ratio) {
    GeartrainParams params;
    
    params.gearRatio = ratio;
    
    params.forwardEfficiency = 0.85;
    params.backwardEfficiency = 0.70;
    
    params.gearInertia = 0.000001 * ratio;
    
    params.backdrivable = (ratio < 50);
    params.backdriveTorqueThreshold = ratio * 0.01;
    
    return params;
}

FrictionParams createFrictionParams(double staticFriction, double coulombFriction, 
                                    double viscousCoeff) {
    FrictionParams params;
    params.staticFriction = staticFriction;
    params.coulombFriction = coulombFriction;
    params.viscousCoeff = viscousCoeff;
    params.stribeckVelocity = 0.1;
    params.stictionVelocity = 0.001;
    return params;
}

} // namespace Factory

} // namespace Motor
} // namespace CiA402
