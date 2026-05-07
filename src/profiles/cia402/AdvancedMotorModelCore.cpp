/**
 * @file AdvancedMotorModelCore.cpp
 * @brief Core AdvancedMotorModel - construction, parameters, control loops
 * 
 * Split from AdvancedMotorModel.cpp for maintainability.
 */

#include "profiles/cia402/AdvancedMotorModel.hpp"
#include <algorithm>
#include <cmath>

namespace CiA402 {
namespace Motor {

// ============================================================================
// Constants
// ============================================================================

static constexpr double PI = 3.14159265358979323846;
static constexpr double TWO_PI = 2.0 * PI;
static constexpr double RPM_TO_RADS = TWO_PI / 60.0;
static constexpr double RADS_TO_RPM = 60.0 / TWO_PI;

// ============================================================================
// FrictionParams Implementation
// ============================================================================

double FrictionParams::calculate(double velocity, double appliedTorque) const {
    double absVel = std::abs(velocity);
    
    if (absVel < stictionVelocity) {
        if (std::abs(appliedTorque) < staticFriction) {
            return appliedTorque;
        } else {
            return std::copysign(staticFriction, appliedTorque);
        }
    }
    
    double stribeckTerm = 0.0;
    if (stribeckVelocity > 0) {
        double ratio = absVel / stribeckVelocity;
        stribeckTerm = (staticFriction - coulombFriction) * std::exp(-ratio * ratio);
    }
    
    double coulombTerm = coulombFriction;
    double viscousTerm = viscousCoeff * absVel;
    
    double frictionMagnitude = coulombTerm + stribeckTerm + viscousTerm;
    
    return std::copysign(frictionMagnitude, velocity);
}

// ============================================================================
// TorqueSpeedCurve Implementation
// ============================================================================

double TorqueSpeedCurve::getAvailableTorque(double speed, bool isPeak) const {
    double absSpeed = std::abs(speed);
    double baseTorque = isPeak ? peakTorque : ratedTorque;
    
    if (absSpeed < 0.1) {
        return isPeak ? peakTorque : stallTorque;
    }
    
    if (absSpeed <= cornerSpeed) {
        return baseTorque;
    }
    
    if (absSpeed <= maxSpeed) {
        return baseTorque * (cornerSpeed / absSpeed);
    }
    
    return 0.0;
}

// ============================================================================
// AdvancedMotorModel - Construction and Parameter Setters
// ============================================================================

AdvancedMotorModel::AdvancedMotorModel() {
}

AdvancedMotorModel::AdvancedMotorModel(const MotorParams& motorParams)
    : motorParams_(motorParams)
{
}

void AdvancedMotorModel::setMotorParams(const MotorParams& params) {
    std::lock_guard<std::mutex> lock(mutex_);
    motorParams_ = params;
}

void AdvancedMotorModel::setBacklashParams(const BacklashParams& params) {
    std::lock_guard<std::mutex> lock(mutex_);
    backlashParams_ = params;
}

void AdvancedMotorModel::setGeartrainParams(const GeartrainParams& params) {
    std::lock_guard<std::mutex> lock(mutex_);
    geartrainParams_ = params;
}

void AdvancedMotorModel::setLoadParams(const LoadParams& params) {
    std::lock_guard<std::mutex> lock(mutex_);
    loadParams_ = params;
}

void AdvancedMotorModel::setThermalParams(const ThermalParams& params) {
    std::lock_guard<std::mutex> lock(mutex_);
    thermalParams_ = params;
}

void AdvancedMotorModel::setSensorConfig(const SensorConfig& config) {
    std::lock_guard<std::mutex> lock(mutex_);
    sensorConfig_ = config;
}

void AdvancedMotorModel::setControllerParams(const ControllerParams& params) {
    std::lock_guard<std::mutex> lock(mutex_);
    controllerParams_ = params;
}

void AdvancedMotorModel::setErrorInjection(const ErrorInjection& injection) {
    std::lock_guard<std::mutex> lock(mutex_);
    errorInjection_ = injection;
}

// ============================================================================
// Initialization and Reset
// ============================================================================

bool AdvancedMotorModel::initialize() {
    std::lock_guard<std::mutex> lock(mutex_);
    
    state_.reset();
    
    state_.thermal.windingTemp = thermalParams_.ambientTemperature;
    state_.thermal.motorCaseTemp = thermalParams_.ambientTemperature;
    state_.thermal.gearboxTemp = thermalParams_.ambientTemperature;
    
    state_.geartrain.backlashState = BacklashState::InBacklash;
    state_.geartrain.backlashAngle = 0.0;
    
    posIntegral_ = 0.0;
    velIntegral_ = 0.0;
    lastPosError_ = 0.0;
    lastVelError_ = 0.0;
    
    peakTorqueTimer_ = 0.0;
    usingPeakTorque_ = false;
    
    initialized_ = true;
    
    return true;
}

void AdvancedMotorModel::reset() {
    std::lock_guard<std::mutex> lock(mutex_);
    
    state_.reset();
    state_.thermal.windingTemp = thermalParams_.ambientTemperature;
    state_.thermal.motorCaseTemp = thermalParams_.ambientTemperature;
    state_.thermal.gearboxTemp = thermalParams_.ambientTemperature;
    
    targetTorque_ = 0.0;
    targetVelocity_ = 0.0;
    targetPosition_ = 0.0;
    
    posIntegral_ = 0.0;
    velIntegral_ = 0.0;
    lastPosError_ = 0.0;
    lastVelError_ = 0.0;
    
    controlMode_ = ControlMode::Disabled;
}

void AdvancedMotorModel::setInitialState(double motorPos, double loadPos) {
    std::lock_guard<std::mutex> lock(mutex_);
    
    state_.motor.position = motorPos;
    state_.load.position = loadPos;
    
    state_.geartrain.inputPosition = motorPos;
    state_.geartrain.outputPosition = loadPos;
    
    double expectedLoadPos = motorPos / geartrainParams_.gearRatio;
    state_.geartrain.backlashAngle = expectedLoadPos - loadPos;
    
    double halfBacklash = backlashParams_.totalBacklash / 2.0;
    state_.geartrain.backlashAngle = std::clamp(
        state_.geartrain.backlashAngle, -halfBacklash, halfBacklash);
    
    prevMotorPosition_ = motorPos;
    prevLoadPosition_ = loadPos;
    
    updateEncoder();
}

// ============================================================================
// Control Mode and Targets
// ============================================================================

void AdvancedMotorModel::setControlMode(ControlMode mode) {
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (mode != controlMode_) {
        posIntegral_ = 0.0;
        velIntegral_ = 0.0;
        lastPosError_ = 0.0;
        lastVelError_ = 0.0;
        
        targetPosition_ = state_.motor.position;
        targetVelocity_ = state_.motor.velocity;
        targetTorque_ = 0.0;
        
        controlMode_ = mode;
    }
}

void AdvancedMotorModel::setTargetTorque(double torque) {
    std::lock_guard<std::mutex> lock(mutex_);
    targetTorque_ = torque;
}

void AdvancedMotorModel::setTargetVelocity(double velocity) {
    std::lock_guard<std::mutex> lock(mutex_);
    targetVelocity_ = velocity;
}

void AdvancedMotorModel::setTargetPosition(double position) {
    std::lock_guard<std::mutex> lock(mutex_);
    targetPosition_ = position;
}

void AdvancedMotorModel::setExternalLoadTorque(double torque) {
    std::lock_guard<std::mutex> lock(mutex_);
    loadParams_.externalTorque = torque;
}

// ============================================================================
// Main Update Loop
// ============================================================================

void AdvancedMotorModel::update(double dt) {
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (!initialized_) {
        return;
    }
    
    checkFaults();
    
    if (state_.hasFault) {
        state_.motor.torque = 0.0;
        state_.motor.current = 0.0;
        return;
    }
    
    double torqueCmd = 0.0;
    
    switch (controlMode_) {
        case ControlMode::Position:
            torqueCmd = runPositionControl(dt);
            break;
            
        case ControlMode::Velocity:
            torqueCmd = runVelocityControl(targetVelocity_, dt);
            break;
            
        case ControlMode::Torque:
            torqueCmd = runTorqueLimit(targetTorque_);
            break;
            
        case ControlMode::Disabled:
        default:
            torqueCmd = 0.0;
            break;
    }
    
    if (errorInjection_.enabled) {
        if (errorInjection_.simulateJam) {
            torqueCmd = 0.0;
        }
    }
    
    simulateMotor(torqueCmd, dt);
    simulateBacklash(dt);
    simulateGeartrain(dt);
    simulateLoad(dt);
    
    if (thermalParams_.enabled) {
        simulateThermal(dt);
    }
    
    updateSensors();
    
    state_.simulationTime += dt;
}

void AdvancedMotorModel::clearFault() {
    std::lock_guard<std::mutex> lock(mutex_);
    state_.hasFault = false;
    state_.faultCode = 0;
}

// ============================================================================
// Control Loops
// ============================================================================

double AdvancedMotorModel::runPositionControl(double dt) {
    double posError = targetPosition_ - state_.motor.position;
    
    posIntegral_ += posError * dt;
    posIntegral_ = std::clamp(posIntegral_, 
                              -controllerParams_.posIntegralLimit,
                              controllerParams_.posIntegralLimit);
    
    double posDeriv = (posError - lastPosError_) / dt;
    lastPosError_ = posError;
    
    double velCmd = controllerParams_.posKp * posError +
                    controllerParams_.posKi * posIntegral_ +
                    controllerParams_.posKd * posDeriv;
    
    velCmd += controllerParams_.velocityFF * targetVelocity_;
    
    velCmd = std::clamp(velCmd, -controllerParams_.maxVelocityCmd,
                        controllerParams_.maxVelocityCmd);
    
    return runVelocityControl(velCmd, dt);
}

double AdvancedMotorModel::runVelocityControl(double targetVel, double dt) {
    double velError = targetVel - state_.motor.velocity;
    
    velIntegral_ += velError * dt;
    velIntegral_ = std::clamp(velIntegral_,
                              -controllerParams_.velIntegralLimit,
                              controllerParams_.velIntegralLimit);
    
    double velDeriv = (velError - lastVelError_) / dt;
    lastVelError_ = velError;
    
    double torqueCmd = controllerParams_.velKp * velError +
                       controllerParams_.velKi * velIntegral_ +
                       controllerParams_.velKd * velDeriv;
    
    return runTorqueLimit(torqueCmd);
}

double AdvancedMotorModel::runTorqueLimit(double torqueCmd) {
    double speed = std::abs(state_.motor.velocity);
    double availableTorqueCont = motorParams_.torqueSpeed.getAvailableTorque(speed, false);
    double availableTorquePeak = motorParams_.torqueSpeed.getAvailableTorque(speed, true);
    
    double absCmd = std::abs(torqueCmd);
    
    if (absCmd > availableTorqueCont) {
        usingPeakTorque_ = true;
        torqueCmd = std::clamp(torqueCmd, -availableTorquePeak, availableTorquePeak);
    } else {
        usingPeakTorque_ = false;
    }
    
    torqueCmd = std::clamp(torqueCmd, -controllerParams_.maxTorqueCmd,
                           controllerParams_.maxTorqueCmd);
    
    return torqueCmd;
}

} // namespace Motor
} // namespace CiA402
