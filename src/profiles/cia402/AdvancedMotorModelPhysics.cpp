/**
 * @file AdvancedMotorModelPhysics.cpp
 * @brief Physics simulation for AdvancedMotorModel
 * 
 * Split from AdvancedMotorModel.cpp for maintainability.
 */

#include "profiles/cia402/AdvancedMotorModel.hpp"
#include <algorithm>
#include <cmath>

namespace CiA402 {
namespace Motor {

// ============================================================================
// Physics Simulation
// ============================================================================

void AdvancedMotorModel::simulateMotor(double torqueCmd, double dt) {
    prevMotorPosition_ = state_.motor.position;
    
    double current = torqueCmd / motorParams_.electrical.torqueConstant;
    
    double maxCurrent = usingPeakTorque_ ? 
        motorParams_.electrical.peakCurrent : 
        motorParams_.electrical.ratedCurrent;
    current = std::clamp(current, -maxCurrent, maxCurrent);
    
    double motorTorque = current * motorParams_.electrical.torqueConstant;
    
    double backEMF = motorParams_.electrical.backEMFConstant * state_.motor.velocity;
    
    double powerDissipation = current * current * motorParams_.electrical.windingResistance;
    
    double frictionTorque = calculateMotorFriction(state_.motor.velocity, motorTorque);
    
    double reactionTorque = calculateBacklashTorque();
    
    double netTorque = motorTorque - frictionTorque - reactionTorque;
    
    double acceleration = netTorque / motorParams_.mechanical.rotorInertia;
    
    acceleration = std::clamp(acceleration,
                              -motorParams_.mechanical.maxAcceleration,
                              motorParams_.mechanical.maxAcceleration);
    
    double newVelocity = state_.motor.velocity + acceleration * dt;
    
    double maxSpeed = motorParams_.torqueSpeed.maxSpeed;
    newVelocity = std::clamp(newVelocity, -maxSpeed, maxSpeed);
    
    double newPosition = state_.motor.position + state_.motor.velocity * dt + 
                         0.5 * acceleration * dt * dt;
    
    state_.motor.position = newPosition;
    state_.motor.velocity = newVelocity;
    state_.motor.acceleration = acceleration;
    state_.motor.torque = motorTorque;
    state_.motor.current = current;
    state_.motor.backEMF = backEMF;
    state_.motor.powerDissipation = powerDissipation;
    state_.motor.frictionTorque = frictionTorque;
    
    if (usingPeakTorque_) {
        peakTorqueTimer_ += dt;
        if (peakTorqueTimer_ > motorParams_.torqueSpeed.peakTorqueDuration) {
            triggerFault(0x3210);
        }
    } else {
        peakTorqueTimer_ = 0.0;
    }
}

void AdvancedMotorModel::simulateBacklash(double dt) {
    if (!backlashParams_.enabled) {
        state_.geartrain.inputPosition = state_.motor.position;
        state_.geartrain.inputVelocity = state_.motor.velocity;
        state_.geartrain.backlashState = BacklashState::ContactPositive;
        state_.geartrain.backlashAngle = 0.0;
        return;
    }
    
    double motorPos = state_.motor.position;
    double gearInputPos = state_.geartrain.inputPosition;
    double relativeAngle = motorPos - gearInputPos;
    
    double halfBacklash = (backlashParams_.totalBacklash + 
                          (errorInjection_.enabled ? errorInjection_.additionalBacklash : 0.0)) / 2.0;
    
    BacklashState newState = state_.geartrain.backlashState;
    double contactTorque = 0.0;
    
    if (relativeAngle >= halfBacklash) {
        newState = BacklashState::ContactPositive;
        double penetration = relativeAngle - halfBacklash;
        double relVel = state_.motor.velocity - state_.geartrain.inputVelocity;
        contactTorque = backlashParams_.contactStiffness * penetration +
                        backlashParams_.contactDamping * relVel;
    }
    else if (relativeAngle <= -halfBacklash) {
        newState = BacklashState::ContactNegative;
        double penetration = relativeAngle + halfBacklash;
        double relVel = state_.motor.velocity - state_.geartrain.inputVelocity;
        contactTorque = backlashParams_.contactStiffness * penetration +
                        backlashParams_.contactDamping * relVel;
    }
    else {
        newState = BacklashState::InBacklash;
        double relVel = state_.motor.velocity - state_.geartrain.inputVelocity;
        contactTorque = backlashParams_.backlashViscosity * relVel;
    }
    
    state_.geartrain.backlashState = newState;
    state_.geartrain.backlashAngle = relativeAngle;
    state_.geartrain.transmittedTorque = contactTorque;
}

void AdvancedMotorModel::simulateGeartrain(double dt) {
    double inputTorque = state_.geartrain.transmittedTorque;
    double gearRatio = geartrainParams_.gearRatio;
    
    double efficiency;
    if (inputTorque * state_.geartrain.inputVelocity >= 0) {
        efficiency = geartrainParams_.forwardEfficiency;
    } else {
        if (!geartrainParams_.backdrivable) {
            double loadTorque = std::abs(state_.load.appliedTorque);
            if (loadTorque < geartrainParams_.backdriveTorqueThreshold) {
                efficiency = 0.0;
            } else {
                efficiency = geartrainParams_.backwardEfficiency;
            }
        } else {
            efficiency = geartrainParams_.backwardEfficiency;
        }
    }
    
    state_.geartrain.efficiency = efficiency;
    
    double gearOutputVel = state_.geartrain.outputVelocity;
    double gearFriction = calculateGearFriction(gearOutputVel, inputTorque / gearRatio);
    state_.geartrain.frictionTorque = gearFriction;
    
    double outputTorque = (inputTorque * efficiency / gearRatio) - gearFriction;
    
    if (errorInjection_.enabled && errorInjection_.additionalFriction > 0) {
        double addedFriction = errorInjection_.additionalFriction * 
                               std::copysign(1.0, gearOutputVel);
        outputTorque -= addedFriction;
    }
    
    if (errorInjection_.enabled && errorInjection_.simulateBrokenGear) {
        outputTorque = 0.0;
    }
    
    double reflectedInertia = geartrainParams_.gearInertia * gearRatio * gearRatio;
    
    if (state_.geartrain.backlashState != BacklashState::InBacklash) {
        state_.geartrain.inputVelocity = state_.motor.velocity;
        state_.geartrain.inputPosition = state_.motor.position - state_.geartrain.backlashAngle;
    } else {
        double coastDecel = -backlashParams_.backlashViscosity * state_.geartrain.inputVelocity / 
                           (geartrainParams_.gearInertia + 0.0001);
        state_.geartrain.inputVelocity += coastDecel * dt;
        state_.geartrain.inputPosition += state_.geartrain.inputVelocity * dt;
    }
    
    state_.geartrain.outputVelocity = state_.geartrain.inputVelocity / gearRatio;
    state_.geartrain.outputPosition = state_.geartrain.inputPosition / gearRatio;
    
    state_.load.appliedTorque = outputTorque;
}

void AdvancedMotorModel::simulateLoad(double dt) {
    prevLoadPosition_ = state_.load.position;
    
    double driveTorque = state_.load.appliedTorque;
    
    double externalTorque = loadParams_.externalTorque;
    
    if (errorInjection_.enabled) {
        externalTorque += errorInjection_.injectedLoadTorque;
        
        if (errorInjection_.simulateLoadImpact && errorInjection_.impactDuration > 0) {
            externalTorque += errorInjection_.impactTorque;
        }
    }
    
    state_.load.externalTorque = externalTorque;
    
    double frictionTorque = calculateLoadFriction(state_.load.velocity, driveTorque);
    state_.load.frictionTorque = frictionTorque;
    
    double hardStopTorque = calculateHardStopTorque();
    
    double netTorque = driveTorque - frictionTorque + externalTorque + hardStopTorque;
    
    double acceleration = netTorque / loadParams_.inertia;
    
    double newVelocity = state_.load.velocity + acceleration * dt;
    
    double newPosition = state_.load.position + state_.load.velocity * dt +
                         0.5 * acceleration * dt * dt;
    
    if (newPosition > loadParams_.positionMax) {
        newPosition = loadParams_.positionMax;
        if (newVelocity > 0) newVelocity = 0;
        state_.load.atPositiveLimit = true;
    } else {
        state_.load.atPositiveLimit = false;
    }
    
    if (newPosition < loadParams_.positionMin) {
        newPosition = loadParams_.positionMin;
        if (newVelocity < 0) newVelocity = 0;
        state_.load.atNegativeLimit = true;
    } else {
        state_.load.atNegativeLimit = false;
    }
    
    state_.load.position = newPosition;
    state_.load.velocity = newVelocity;
    state_.load.acceleration = acceleration;
}

void AdvancedMotorModel::simulateThermal(double dt) {
    double windingHeat = state_.motor.powerDissipation;
    
    double windingToCase = (state_.thermal.windingTemp - state_.thermal.motorCaseTemp) /
                           thermalParams_.windingToMotorResistance;
    
    double caseToAmbient = (state_.thermal.motorCaseTemp - thermalParams_.ambientTemperature) /
                           thermalParams_.motorThermalResistance;
    
    double windingTempChange = (windingHeat - windingToCase) * dt / thermalParams_.windingThermalMass;
    double caseTempChange = (windingToCase - caseToAmbient) * dt / thermalParams_.motorThermalMass;
    
    state_.thermal.windingTemp += windingTempChange;
    state_.thermal.motorCaseTemp += caseTempChange;
    
    double gearHeat = std::abs(state_.geartrain.frictionTorque * state_.geartrain.outputVelocity);
    double gearToAmbient = (state_.thermal.gearboxTemp - thermalParams_.ambientTemperature) /
                           thermalParams_.gearboxThermalResistance;
    double gearTempChange = (gearHeat - gearToAmbient) * dt / thermalParams_.gearboxThermalMass;
    
    state_.thermal.gearboxTemp += gearTempChange;
    
    state_.thermal.windingOvertemp = state_.thermal.windingTemp > thermalParams_.maxWindingTemp;
    state_.thermal.motorOvertemp = state_.thermal.motorCaseTemp > thermalParams_.maxMotorTemp;
    state_.thermal.gearboxOvertemp = state_.thermal.gearboxTemp > thermalParams_.maxGearboxTemp;
    
    if (errorInjection_.enabled) {
        if (errorInjection_.simulateWindingOverheat) {
            state_.thermal.windingOvertemp = true;
        }
        if (errorInjection_.simulateMotorOverheat) {
            state_.thermal.motorOvertemp = true;
        }
    }
}

// ============================================================================
// Helper Functions
// ============================================================================

double AdvancedMotorModel::calculateMotorFriction(double velocity, double appliedTorque) {
    return motorParams_.mechanical.friction.calculate(velocity, appliedTorque);
}

double AdvancedMotorModel::calculateGearFriction(double velocity, double appliedTorque) {
    return geartrainParams_.friction.calculate(velocity, appliedTorque);
}

double AdvancedMotorModel::calculateLoadFriction(double velocity, double appliedTorque) {
    return loadParams_.friction.calculate(velocity, appliedTorque);
}

double AdvancedMotorModel::calculateBacklashTorque() {
    return state_.geartrain.transmittedTorque;
}

double AdvancedMotorModel::calculateHardStopTorque() {
    double torque = 0.0;
    
    if (state_.load.position > loadParams_.positionMax) {
        double penetration = state_.load.position - loadParams_.positionMax;
        torque -= loadParams_.hardStopStiffness * penetration +
                  loadParams_.hardStopDamping * std::max(0.0, state_.load.velocity);
    }
    
    if (state_.load.position < loadParams_.positionMin) {
        double penetration = loadParams_.positionMin - state_.load.position;
        torque += loadParams_.hardStopStiffness * penetration +
                  loadParams_.hardStopDamping * std::max(0.0, -state_.load.velocity);
    }
    
    return torque;
}

} // namespace Motor
} // namespace CiA402
