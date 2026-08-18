/**
 * @file MotorModel.cpp
 * @brief Physical Motor Model Implementation
 */

#include "profiles/cia402/MotorModel.hpp"
#include <algorithm>
#include <cmath>
#include <random>

namespace CiA402 {
namespace Motor {

// ============================================================================
// Constants
// ============================================================================

static constexpr double PI = 3.14159265358979323846;
static constexpr double TWO_PI = 2.0 * PI;

// ============================================================================
// MotorModel Implementation
// ============================================================================

MotorModel::MotorModel()
    : positionPID_(std::make_unique<tether::control::PIDController>())
    , velocityPID_(std::make_unique<tether::control::PIDController>())
    , velocityOnlyPID_(std::make_unique<tether::control::PIDController>())
{
}

MotorModel::MotorModel(const MotorParams& params)
    : motorParams_(params)
    , positionPID_(std::make_unique<tether::control::PIDController>())
    , velocityPID_(std::make_unique<tether::control::PIDController>())
    , velocityOnlyPID_(std::make_unique<tether::control::PIDController>())
{
}

void MotorModel::setMotorParams(const MotorParams& params) {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    motorParams_ = params;
}

void MotorModel::setThermalParams(const ThermalParams& params) {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    thermalParams_ = params;
}

void MotorModel::setBrakingResistorParams(const BrakingResistorParams& params) {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    brakingParams_ = params;
}

void MotorModel::setEndstopConfig(const EndstopConfig& config) {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    endstopConfig_ = config;
}

void MotorModel::setPositionControllerParams(const PositionControllerParams& params) {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    posCtrlParams_ = params;
    
    // Update position PID
    positionPID_->setGains(params.posKp, params.posKi, params.posKd);
    positionPID_->setDerivativeFilter(params.posDerivativeFilter);
    positionPID_->setIntegralLimits(-params.posIntegralLimit, params.posIntegralLimit);
    
    // Update inner velocity PID
    velocityPID_->setGains(params.velKp, params.velKi, params.velKd);
    velocityPID_->setDerivativeFilter(params.velDerivativeFilter);
    velocityPID_->setIntegralLimits(-params.velIntegralLimit, params.velIntegralLimit);
}

void MotorModel::setVelocityControllerParams(const VelocityControllerParams& params) {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    velCtrlParams_ = params;
    
    velocityOnlyPID_->setGains(params.kp, params.ki, params.kd);
    velocityOnlyPID_->setDerivativeFilter(params.derivativeFilter);
    velocityOnlyPID_->setIntegralLimits(-params.integralLimit, params.integralLimit);
}

void MotorModel::setTorqueControllerParams(const TorqueControllerParams& params) {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    torqueCtrlParams_ = params;
}

bool MotorModel::initialize() {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    
    state_.reset();
    
    // Initialize controllers
    positionPID_->setGains(posCtrlParams_.posKp, posCtrlParams_.posKi, posCtrlParams_.posKd);
    positionPID_->setDerivativeFilter(posCtrlParams_.posDerivativeFilter);
    positionPID_->setIntegralLimits(-posCtrlParams_.posIntegralLimit, posCtrlParams_.posIntegralLimit);
    
    velocityPID_->setGains(posCtrlParams_.velKp, posCtrlParams_.velKi, posCtrlParams_.velKd);
    velocityPID_->setDerivativeFilter(posCtrlParams_.velDerivativeFilter);
    velocityPID_->setIntegralLimits(-posCtrlParams_.velIntegralLimit, posCtrlParams_.velIntegralLimit);
    
    velocityOnlyPID_->setGains(velCtrlParams_.kp, velCtrlParams_.ki, velCtrlParams_.kd);
    velocityOnlyPID_->setDerivativeFilter(velCtrlParams_.derivativeFilter);
    velocityOnlyPID_->setIntegralLimits(-velCtrlParams_.integralLimit, velCtrlParams_.integralLimit);
    
    // Set initial thermal state
    state_.windingTemperature = thermalParams_.ambientTemp;
    state_.motorTemperature = thermalParams_.ambientTemp;
    state_.brakingResistorTemp = thermalParams_.ambientTemp;
    
    initialized_ = true;
    
    return true;
}

void MotorModel::reset() {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    
    state_.reset();
    state_.windingTemperature = thermalParams_.ambientTemp;
    state_.motorTemperature = thermalParams_.ambientTemp;
    state_.brakingResistorTemp = thermalParams_.ambientTemp;
    
    targetPosition_ = 0;
    targetVelocity_ = 0;
    targetTorque_ = 0;
    
    prevPosition_ = 0.0;
    prevVelocity_ = 0.0;
    
    positionPID_->reset();
    velocityPID_->reset();
    velocityOnlyPID_->reset();
    
    controlMode_ = ControlMode::Disabled;
}

void MotorModel::setControlMode(ControlMode mode) {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    
    if (mode != controlMode_) {
        // Reset controllers on mode change
        positionPID_->reset();
        velocityPID_->reset();
        velocityOnlyPID_->reset();
        
        controlMode_ = mode;
    }
}

void MotorModel::setTargetPosition(int32_t position) {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    targetPosition_ = position;
}

void MotorModel::setTargetVelocity(int32_t velocity) {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    targetVelocity_ = velocity;
}

void MotorModel::setTargetTorque(int16_t torque) {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    targetTorque_ = torque;
}

void MotorModel::setLoadTorque(double torque) {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    state_.loadTorque = torque;
}

int16_t MotorModel::getActualTorque() const {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    
    // Convert to 0.1% of rated torque
    double normalized = state_.actualTorque / motorParams_.maxTorque;
    return static_cast<int16_t>(normalized * 1000.0);
}

int32_t MotorModel::getFollowingError() const {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    return targetPosition_ - state_.encoderPosition;
}

bool MotorModel::hasFault() const {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    return state_.overTorqueFault || state_.overSpeedFault || 
           state_.overTempFault || state_.followingErrorFault ||
           state_.encoderFault || state_.brakingResistorFault;
}

void MotorModel::update(double dt) {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    
    if (!initialized_) {
        return;
    }
    
    // Check for faults first
    checkFaults();
    
    if (hasFault() || controlMode_ == ControlMode::Disabled) {
        state_.commandedTorque = 0.0;
        state_.actualTorque = 0.0;
        return;
    }
    
    // Run control loop
    double torqueCmd = runControlLoop(dt);
    
    // Apply error injection
    if (errorInjection_.enabled) {
        if (errorInjection_.injectLoadTorque) {
            state_.loadTorque = errorInjection_.injectedLoadTorque;
        }
        if (errorInjection_.simulateJam) {
            torqueCmd = 0.0;
            state_.velocity = 0.0;
        }
    }
    
    // Run physics
    runPhysics(torqueCmd, dt);
    
    // Update thermal model
    if (thermalParams_.enableThermalModel) {
        updateThermal(dt);
    }
    
    // Update encoder
    updateEncoder();
    
    // Update endstops
    updateEndstops();
    
    // Check limits
    checkLimits();
}

double MotorModel::runControlLoop(double dt) {
    double torqueCmd = 0.0;
    
    switch (controlMode_) {
        case ControlMode::Position: {
            // Dual-loop position control
            // Outer loop: position -> velocity command
            tether::control::ControllerInput posInput;
            posInput.reference = countsToRadians(targetPosition_);
            posInput.measured = state_.position;
            posInput.dt = dt;
            
            tether::control::ControllerOutput posOutput = positionPID_->compute(posInput);
            
            // Limit velocity command
            double velCmd = std::clamp(posOutput.control, 
                                       -posCtrlParams_.maxVelocityCmd, 
                                       posCtrlParams_.maxVelocityCmd);
            
            // Add velocity feed-forward
            // (would need trajectory generator for proper feed-forward)
            
            // Inner loop: velocity -> torque command
            tether::control::ControllerInput velInput;
            velInput.reference = velCmd;
            velInput.measured = state_.velocity;
            velInput.dt = dt;
            
            tether::control::ControllerOutput velOutput = velocityPID_->compute(velInput);
            
            // Limit torque command
            torqueCmd = std::clamp(velOutput.control,
                                   -posCtrlParams_.maxTorqueCmd,
                                   posCtrlParams_.maxTorqueCmd);
            break;
        }
        
        case ControlMode::Velocity: {
            // Single-loop velocity control
            tether::control::ControllerInput velInput;
            velInput.reference = countsToRadians(targetVelocity_) * 
                                 TWO_PI / motorParams_.encoderResolution;
            velInput.measured = state_.velocity;
            velInput.dt = dt;
            
            tether::control::ControllerOutput velOutput = velocityOnlyPID_->compute(velInput);
            
            // Add torque feed-forward
            torqueCmd = velOutput.control + velCtrlParams_.torqueFeedforward;
            
            // Limit torque command
            torqueCmd = std::clamp(torqueCmd,
                                   -velCtrlParams_.maxTorqueCmd,
                                   velCtrlParams_.maxTorqueCmd);
            break;
        }
        
        case ControlMode::Torque: {
            // Direct torque mode
            // Convert from 0.1% of rated to actual torque
            torqueCmd = (targetTorque_ / 1000.0) * motorParams_.maxTorque;
            
            // Apply torque ramp (if needed)
            double maxChange = torqueCtrlParams_.torqueRampRate * dt;
            double diff = torqueCmd - state_.commandedTorque;
            if (std::abs(diff) > maxChange) {
                torqueCmd = state_.commandedTorque + std::copysign(maxChange, diff);
            }
            break;
        }
        
        default:
            break;
    }
    
    return torqueCmd;
}

void MotorModel::runPhysics(double commandedTorque, double dt) {
    state_.commandedTorque = commandedTorque;
    
    // Calculate friction
    state_.frictionTorque = calculateFriction(state_.velocity);
    
    // Calculate back-EMF (for electrical model)
    state_.backEMF = calculateBackEMF(state_.velocity);
    
    // Calculate actual torque (may be limited by current)
    state_.actualTorque = limitTorque(commandedTorque);
    
    // Apply error injections
    if (errorInjection_.enabled) {
        if (errorInjection_.simulateSlip) {
            state_.actualTorque *= 0.5;  // Reduced torque due to slip
        }
        if (errorInjection_.simulateBrokenBelt) {
            state_.actualTorque = 0.0;
            state_.loadTorque = 0.0;
        }
    }
    
    // Net torque
    double netTorque = state_.actualTorque - state_.frictionTorque - state_.loadTorque;
    
    // Apply gear ratio
    netTorque *= motorParams_.gearRatio * motorParams_.gearEfficiency;
    
    // Calculate acceleration (τ = J * α)
    state_.acceleration = netTorque / motorParams_.inertia;
    
    // Limit acceleration
    state_.acceleration = std::clamp(state_.acceleration,
                                     -motorParams_.maxAcceleration,
                                     motorParams_.maxAcceleration);
    
    // Integrate velocity
    prevVelocity_ = state_.velocity;
    state_.velocity += state_.acceleration * dt;
    
    // Limit velocity
    state_.velocity = std::clamp(state_.velocity,
                                 -motorParams_.maxVelocity,
                                 motorParams_.maxVelocity);
    
    // Integrate position
    prevPosition_ = state_.position;
    state_.position += state_.velocity * dt;
    
    // Handle position limits (hard stops)
    if (state_.position > motorParams_.maxPosition) {
        state_.position = motorParams_.maxPosition;
        if (state_.velocity > 0) {
            state_.velocity = 0;
        }
    } else if (state_.position < motorParams_.minPosition) {
        state_.position = motorParams_.minPosition;
        if (state_.velocity < 0) {
            state_.velocity = 0;
        }
    }
    
    // Calculate current
    state_.current = state_.actualTorque / motorParams_.torqueConstant;
    
    // Calculate power
    state_.powerDissipation = state_.current * state_.current * motorParams_.windingResistance;
}

void MotorModel::updateThermal(double dt) {
    // Simple first-order thermal model
    
    // Heat generation in windings
    double heatGeneration = state_.powerDissipation;
    
    // Heat dissipation to ambient
    double heatDissipation = (state_.windingTemperature - thermalParams_.ambientTemp) /
                             thermalParams_.thermalResistance;
    
    // Temperature change
    double tempChange = (heatGeneration - heatDissipation) * dt / 
                        thermalParams_.thermalCapacity;
    
    state_.windingTemperature += tempChange;
    
    // Motor case temperature (slower response)
    state_.motorTemperature += (state_.windingTemperature - state_.motorTemperature) * dt * 0.1;
    
    // Braking resistor temperature
    if (brakingParams_.enabled) {
        // Regenerative braking power (when velocity and torque have opposite signs)
        double regenPower = 0.0;
        if ((state_.velocity > 0 && state_.actualTorque < 0) ||
            (state_.velocity < 0 && state_.actualTorque > 0)) {
            regenPower = std::abs(state_.velocity * state_.actualTorque);
        }
        
        double brakingHeatGen = regenPower;
        double brakingHeatDiss = (state_.brakingResistorTemp - thermalParams_.ambientTemp) /
                                  brakingParams_.thermalTimeConstant;
        
        state_.brakingResistorTemp += (brakingHeatGen - brakingHeatDiss) * dt;
    }
    
    // Apply overheating injection
    if (errorInjection_.enabled && errorInjection_.simulateOverheat) {
        state_.windingTemperature = thermalParams_.maxWindingTemp + 10.0;
    }
    if (errorInjection_.enabled && errorInjection_.simulateBrakingResistorOverheat) {
        state_.brakingResistorTemp = brakingParams_.maxTemperature + 10.0;
    }
}

void MotorModel::updateEndstops() {
    double posCounts = radiansToCounts(state_.position);
    
    // Positive limit
    bool prevPositive = state_.positiveLimit;
    if (endstopConfig_.positiveEnabled) {
        if (errorInjection_.enabled && errorInjection_.simulateEndstopDisconnect) {
            state_.positiveLimit = endstopConfig_.positiveNC;  // Show as tripped
        } else if (endstopConfig_.positiveFault) {
            state_.positiveLimit = endstopConfig_.positiveNC;  // Disconnected = NC state
        } else {
            bool triggered = posCounts >= endstopConfig_.positivePosition;
            state_.positiveLimit = endstopConfig_.positiveNC ? !triggered : triggered;
        }
    } else {
        state_.positiveLimit = false;
    }
    
    // Negative limit
    bool prevNegative = state_.negativeLimit;
    if (endstopConfig_.negativeEnabled) {
        if (endstopConfig_.negativeFault) {
            state_.negativeLimit = endstopConfig_.negativeNC;
        } else {
            bool triggered = posCounts <= endstopConfig_.negativePosition;
            state_.negativeLimit = endstopConfig_.negativeNC ? !triggered : triggered;
        }
    } else {
        state_.negativeLimit = false;
    }
    
    // Home switch
    bool prevHome = state_.homeSwitch;
    if (endstopConfig_.homeEnabled) {
        if (errorInjection_.enabled && errorInjection_.simulateHomeDisconnect) {
            state_.homeSwitch = endstopConfig_.homeNC;
        } else if (endstopConfig_.homeFault) {
            state_.homeSwitch = endstopConfig_.homeNC;
        } else {
            bool inHomeZone = std::abs(posCounts - endstopConfig_.homePosition) < 
                              endstopConfig_.homeWidth / 2.0;
            state_.homeSwitch = endstopConfig_.homeNC ? !inHomeZone : inHomeZone;
        }
    } else {
        state_.homeSwitch = false;
    }
    
    // Index pulse
    state_.indexPulseDetected = false;
    if (endstopConfig_.indexEnabled) {
        if (errorInjection_.enabled && errorInjection_.simulateIndexDisconnect) {
            state_.indexPulse = false;
        } else if (endstopConfig_.indexFault) {
            state_.indexPulse = false;
        } else {
            // Check if crossed index position
            double prevCounts = radiansToCounts(prevPosition_);
            bool crossedIndex = (prevCounts < endstopConfig_.indexPosition && 
                                posCounts >= endstopConfig_.indexPosition) ||
                               (prevCounts > endstopConfig_.indexPosition && 
                                posCounts <= endstopConfig_.indexPosition);
            
            if (crossedIndex) {
                state_.indexPulseDetected = true;
                state_.indexPulse = true;
                state_.lastIndexPosition = static_cast<int32_t>(endstopConfig_.indexPosition);
                
                if (indexPulseCallback_) {
                    indexPulseCallback_(state_.lastIndexPosition);
                }
            } else {
                state_.indexPulse = false;
            }
        }
    }
    
    // Callbacks
    if ((state_.positiveLimit != prevPositive || state_.negativeLimit != prevNegative) &&
        limitCallback_) {
        limitCallback_(state_.positiveLimit, state_.negativeLimit);
    }
    
    if (state_.homeSwitch != prevHome && homeSwitchCallback_) {
        homeSwitchCallback_(state_.homeSwitch);
    }
}

void MotorModel::updateEncoder() {
    // Convert position to counts
    state_.encoderPosition = radiansToCounts(state_.position);
    
    // Apply encoder noise
    if (errorInjection_.enabled && errorInjection_.simulateEncoderNoise) {
        static std::random_device rd;
        static std::mt19937 gen(rd());
        std::uniform_real_distribution<> dis(-errorInjection_.encoderNoiseAmplitude,
                                              errorInjection_.encoderNoiseAmplitude);
        state_.encoderPosition += static_cast<int32_t>(dis(gen));
    }
    
    // Apply encoder fault
    if (errorInjection_.enabled && errorInjection_.simulateEncoderFault) {
        state_.encoderFault = true;
        state_.encoderPosition = 0;  // Lost position
    }
    
    // Calculate encoder velocity (counts/s)
    state_.encoderVelocity = static_cast<int32_t>(
        state_.velocity * motorParams_.encoderResolution / TWO_PI);
}

double MotorModel::calculateFriction(double velocity) {
    double friction = 0.0;
    
    // Viscous damping
    friction += motorParams_.viscousDamping * velocity;
    
    // Coulomb friction with stiction
    if (std::abs(velocity) < motorParams_.stictionVelocity) {
        // In stiction zone - static friction opposes motion
        friction += motorParams_.staticFriction * std::copysign(1.0, velocity);
    } else {
        // Coulomb friction
        friction += motorParams_.coulombFriction * std::copysign(1.0, velocity);
    }
    
    return friction;
}

double MotorModel::calculateBackEMF(double velocity) {
    return motorParams_.backEMFConstant * velocity;
}

double MotorModel::limitTorque(double torque) {
    // Current limit
    double maxCurrent = torqueCtrlParams_.enableCurrentLimit ? 
                        motorParams_.peakCurrent : motorParams_.ratedCurrent * 10;
    double maxTorqueFromCurrent = maxCurrent * motorParams_.torqueConstant;
    
    // Apply limits
    double limited = std::clamp(torque, -motorParams_.maxTorque, motorParams_.maxTorque);
    limited = std::clamp(limited, -maxTorqueFromCurrent, maxTorqueFromCurrent);
    
    return limited;
}

void MotorModel::checkLimits() {
    // Position at hard stop - apply reaction force
    if (state_.positiveLimit && state_.velocity > 0) {
        state_.velocity = 0;
    }
    if (state_.negativeLimit && state_.velocity < 0) {
        state_.velocity = 0;
    }
}

void MotorModel::checkFaults() {
    // Over-torque fault
    if (std::abs(state_.actualTorque) > motorParams_.maxTorque * 1.2) {
        if (!state_.overTorqueFault) {
            state_.overTorqueFault = true;
            triggerFault(0x3210);  // Over-torque
        }
    }
    
    // Over-speed fault
    if (std::abs(state_.velocity) > motorParams_.maxVelocity * 1.1) {
        if (!state_.overSpeedFault) {
            state_.overSpeedFault = true;
            triggerFault(0x3310);  // Over-speed
        }
    }
    
    // Over-temperature fault
    if (state_.windingTemperature > thermalParams_.maxWindingTemp) {
        if (!state_.overTempFault) {
            state_.overTempFault = true;
            triggerFault(0x4310);  // Over-temperature
        }
    }
    
    // Following error fault (position mode only)
    if (controlMode_ == ControlMode::Position) {
        int32_t followingError = std::abs(getFollowingError());
        // Check against configurable window (not implemented here, using 10000)
        if (followingError > 10000) {
            if (!state_.followingErrorFault) {
                state_.followingErrorFault = true;
                triggerFault(0x8611);  // Following error
            }
        }
    }
    
    // Braking resistor fault
    if (brakingParams_.enableOverheatProtection &&
        state_.brakingResistorTemp > brakingParams_.maxTemperature) {
        if (!state_.brakingResistorFault) {
            state_.brakingResistorFault = true;
            triggerFault(0x4380);  // Braking resistor overload
        }
    }
    
    // Error injection faults
    if (errorInjection_.enabled) {
        if (errorInjection_.simulateOvercurrent) {
            triggerFault(0x2310);  // Over-current
        }
        if (errorInjection_.simulateOvervoltage) {
            triggerFault(0x3210);  // Over-voltage
        }
        if (errorInjection_.simulateUndervoltage) {
            triggerFault(0x3220);  // Under-voltage
        }
    }
}

void MotorModel::triggerFault(uint16_t faultCode) {
    if (faultCallback_) {
        faultCallback_(faultCode);
    }
}

double MotorModel::countsToRadians(int32_t counts) const {
    return counts * TWO_PI / motorParams_.encoderResolution;
}

int32_t MotorModel::radiansToCounts(double radians) const {
    return static_cast<int32_t>(radians * motorParams_.encoderResolution / TWO_PI);
}

} // namespace Motor
} // namespace CiA402
