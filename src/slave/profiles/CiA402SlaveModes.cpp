/**
 * @file CiA402SlaveModes.cpp
 * @brief CiA 402 Slave - Operating Modes Implementation (PP, PV, PT, CSP, CSV, CST, HM)
 */

#include "CiA402SlaveCommon.hpp"

namespace EtherCAT {
namespace slave {

// ============================================================================
// Motor Dynamics Simulation
// ============================================================================

void CiA402Slave::simulate(uint64_t deltaNs) {
    if (!driveConfig_.enableSimulation) {
        return;
    }
    
    // Only simulate motion when operation is enabled
    if (driveState_ != CiA402State::OperationEnabled) {
        // In other states, motor is stopped
        if (driveState_ == CiA402State::QuickStopActive) {
            // Quick stop: rapid deceleration
            simulateMotion(deltaNs);
        }
        return;
    }
    
    // Update operating mode display
    operatingModeDisplay_ = operatingMode_;
    
    // Execute mode-specific motion
    switch (operatingMode_) {
        case CiA402ModeValue::ProfilePosition:
            executeProfilePosition();
            break;
        case CiA402ModeValue::ProfileVelocity:
        case CiA402ModeValue::VelocityMode:
            executeProfileVelocity();
            break;
        case CiA402ModeValue::ProfileTorque:
            executeProfileTorque();
            break;
        case CiA402ModeValue::HomingMode:
            executeHoming();
            break;
        case CiA402ModeValue::CyclicSyncPosition:
            executeCyclicSyncPosition();
            break;
        case CiA402ModeValue::CyclicSyncVelocity:
            executeCyclicSyncVelocity();
            break;
        case CiA402ModeValue::CyclicSyncTorque:
            executeCyclicSyncTorque();
            break;
        default:
            // Unknown mode, keep position
            break;
    }
    
    // Simulate motor physics
    simulateMotion(deltaNs);
    
    // Check position/velocity reached (don't override for modes that manage targetReached_ directly)
    if (operatingMode_ != CiA402ModeValue::CyclicSyncTorque &&
        operatingMode_ != CiA402ModeValue::ProfileTorque &&
        operatingMode_ != CiA402ModeValue::HomingMode) {
        targetReached_ = checkPositionReached() || checkVelocityReached();
    }
}

void CiA402Slave::simulateMotion(uint64_t deltaNs) {
    // Convert time to seconds
    double dt = static_cast<double>(deltaNs) / 1.0e9;
    if (dt <= 0 || dt > 0.1) {
        // Sanity check: skip unreasonable time deltas
        return;
    }
    
    // Simple motor model:
    // - Torque produces acceleration (a = T / J)
    // - Velocity integrates position
    // - Friction opposes motion
    
    double inertia = driveConfig_.simulatedInertia;
    double friction = driveConfig_.simulatedFriction;
    
    // Calculate acceleration from torque demand
    // Torque is in 0.1% of rated torque, convert to actual torque
    double torqueNormalized = static_cast<double>(torqueDemand_) / 1000.0;
    double torqueActual = torqueNormalized * driveConfig_.motorRatedTorque;
    
    // Apply friction (opposes velocity)
    double frictionTorque = friction * static_cast<double>(internalVelocity_);
    double netTorque = torqueActual - frictionTorque;
    
    // Acceleration (in position units/s²)
    double acceleration = netTorque / inertia;
    
    // Clamp acceleration
    double maxAccel = static_cast<double>(driveConfig_.maxAcceleration);
    acceleration = std::clamp(acceleration, -maxAccel, maxAccel);
    
    // Update velocity
    double newVelocity = static_cast<double>(internalVelocity_) + acceleration * dt;
    
    // Clamp velocity
    double maxVel = static_cast<double>(driveConfig_.maxMotorVelocity);
    newVelocity = std::clamp(newVelocity, -maxVel, maxVel);
    
    internalVelocity_ = static_cast<int32_t>(newVelocity);
    
    // Update position
    double newPosition = static_cast<double>(internalPosition_) + newVelocity * dt;
    
    // Clamp position
    newPosition = std::clamp(newPosition, 
                             static_cast<double>(driveConfig_.softwarePosLimitMin),
                             static_cast<double>(driveConfig_.softwarePosLimitMax));
    
    internalPosition_ = static_cast<int32_t>(newPosition);
    
    // Update actual values (these would come from encoders in real hardware)
    actualPosition_ = internalPosition_;
    actualVelocity_ = internalVelocity_;
    
    // Actual torque follows demand with some lag
    actualTorque_ = static_cast<int16_t>(torqueDemand_ * 0.95);
}

// ============================================================================
// Mode Execution Functions
// ============================================================================

void CiA402Slave::executeProfilePosition() {
    // Profile Position Mode (PP)
    // Generate trapezoidal profile from current position to target position
    
    // Check for new set-point
    bool newSetPoint = (controlWord_ & ControlWordBits::NewSetPoint) != 0;
    bool changeSetImmediately = (controlWord_ & ControlWordBits::ChangeSetImmediately) != 0;
    bool halt = (controlWord_ & ControlWordBits::Halt) != 0;
    
    if (halt) {
        // Halt: decelerate to stop
        velocityDemand_ = 0;
        targetReached_ = (std::abs(actualVelocity_) < static_cast<int32_t>(driveConfig_.velocityThreshold));
        
        // Compute torque to achieve deceleration
        if (actualVelocity_ > 0) {
            torqueDemand_ = static_cast<int16_t>(-maxTorque_ / 2);
        } else if (actualVelocity_ < 0) {
            torqueDemand_ = static_cast<int16_t>(maxTorque_ / 2);
        } else {
            torqueDemand_ = 0;
        }
        return;
    }
    
    if (newSetPoint && !setPointAcknowledge_) {
        // Accept new set-point
        setPointAcknowledge_ = true;
        targetReached_ = false;
        positionDemand_ = targetPosition_;
    } else if (!newSetPoint) {
        setPointAcknowledge_ = false;
    }
    
    // Calculate distance to target
    int32_t error = positionDemand_ - actualPosition_;
    
    // Simple P controller for position -> velocity
    double kp = 10.0;  // Position gain
    double velCmd = kp * static_cast<double>(error);
    
    // Clamp to profile velocity
    double maxVel = static_cast<double>(profileVelocity_);
    velCmd = std::clamp(velCmd, -maxVel, maxVel);
    
    velocityDemand_ = static_cast<int32_t>(velCmd);
    
    // Velocity -> torque (simple P controller)
    int32_t velError = velocityDemand_ - actualVelocity_;
    double kv = 0.1;  // Velocity gain
    double torqueCmd = kv * static_cast<double>(velError);
    
    // Clamp torque
    torqueCmd = std::clamp(torqueCmd, static_cast<double>(-maxTorque_), static_cast<double>(maxTorque_));
    torqueDemand_ = static_cast<int16_t>(torqueCmd);
}

void CiA402Slave::executeProfileVelocity() {
    // Profile Velocity Mode (PV) / Velocity Mode (VL)
    // Accelerate/decelerate to target velocity
    
    bool halt = (controlWord_ & ControlWordBits::Halt) != 0;
    
    int32_t targetVel = halt ? 0 : targetVelocity_;
    
    // Velocity demand ramps toward target
    int32_t velError = targetVel - velocityDemand_;
    
    // Apply acceleration/deceleration limits
    int32_t maxDelta = static_cast<int32_t>(profileAcceleration_ / 1000);  // Assuming 1kHz cycle
    if (velError > maxDelta) {
        velocityDemand_ += maxDelta;
    } else if (velError < -maxDelta) {
        velocityDemand_ -= maxDelta;
    } else {
        velocityDemand_ = targetVel;
    }
    
    // Position demand integrates velocity
    positionDemand_ += velocityDemand_ / 1000;  // Assuming 1kHz cycle
    
    // Velocity -> torque
    int32_t actualVelError = velocityDemand_ - actualVelocity_;
    double kv = 0.1;
    double torqueCmd = kv * static_cast<double>(actualVelError);
    torqueCmd = std::clamp(torqueCmd, static_cast<double>(-maxTorque_), static_cast<double>(maxTorque_));
    torqueDemand_ = static_cast<int16_t>(torqueCmd);
    
    // Target reached when velocity is stable
    targetReached_ = (std::abs(actualVelocity_ - targetVel) < static_cast<int32_t>(driveConfig_.velocityThreshold));
}

void CiA402Slave::executeProfileTorque() {
    // Profile Torque Mode (PT)
    // Apply target torque directly
    
    bool halt = (controlWord_ & ControlWordBits::Halt) != 0;
    
    if (halt) {
        torqueDemand_ = 0;
    } else {
        torqueDemand_ = targetTorque_;
    }
    
    // Update position/velocity demand from actual (no position control)
    positionDemand_ = actualPosition_;
    velocityDemand_ = actualVelocity_;
    
    targetReached_ = true;  // Torque mode is always "reached"
}

void CiA402Slave::executeCyclicSyncPosition() {
    // Cyclic Synchronous Position Mode (CSP)
    // Position demand comes directly from master each cycle
    
    // Apply position offset
    positionDemand_ = targetPosition_ + positionOffset_;
    
    // Clamp to limits
    positionDemand_ = std::clamp(positionDemand_, driveConfig_.softwarePosLimitMin,
                                  driveConfig_.softwarePosLimitMax);
    
    // Position error
    int32_t posError = positionDemand_ - actualPosition_;
    
    // Simple cascade: position -> velocity -> torque
    // Position P controller
    double kp = 50.0;  // High gain for CSP
    double velCmd = kp * static_cast<double>(posError);
    
    // Add velocity feedforward if provided
    velCmd += static_cast<double>(velocityOffset_);
    
    // Clamp velocity
    double maxVel = static_cast<double>(driveConfig_.maxMotorVelocity);
    velCmd = std::clamp(velCmd, -maxVel, maxVel);
    velocityDemand_ = static_cast<int32_t>(velCmd);
    
    // Velocity P controller
    int32_t velError = velocityDemand_ - actualVelocity_;
    double kv = 0.5;
    double torqueCmd = kv * static_cast<double>(velError);
    
    // Add torque feedforward if provided
    torqueCmd += static_cast<double>(torqueOffset_);
    
    // Clamp torque
    torqueCmd = std::clamp(torqueCmd, static_cast<double>(-maxTorque_), static_cast<double>(maxTorque_));
    torqueDemand_ = static_cast<int16_t>(torqueCmd);
    
    // Target reached based on position window
    targetReached_ = (std::abs(posError) < static_cast<int32_t>(driveConfig_.positionWindow));
}

void CiA402Slave::executeCyclicSyncVelocity() {
    // Cyclic Synchronous Velocity Mode (CSV)
    // Velocity demand comes directly from master each cycle
    
    // Apply velocity offset
    velocityDemand_ = targetVelocity_ + velocityOffset_;
    
    // Clamp velocity
    int32_t maxVel = static_cast<int32_t>(driveConfig_.maxMotorVelocity);
    velocityDemand_ = std::clamp(velocityDemand_, -maxVel, maxVel);
    
    // Update position demand by integration
    positionDemand_ += velocityDemand_ / 1000;  // Assuming 1kHz
    
    // Velocity -> torque
    int32_t velError = velocityDemand_ - actualVelocity_;
    double kv = 0.5;
    double torqueCmd = kv * static_cast<double>(velError);
    
    // Add torque feedforward
    torqueCmd += static_cast<double>(torqueOffset_);
    
    torqueCmd = std::clamp(torqueCmd, static_cast<double>(-maxTorque_), static_cast<double>(maxTorque_));
    torqueDemand_ = static_cast<int16_t>(torqueCmd);
    
    // Target reached based on velocity threshold
    targetReached_ = (std::abs(actualVelocity_ - targetVelocity_) < 
                      static_cast<int32_t>(driveConfig_.velocityThreshold));
}

void CiA402Slave::executeCyclicSyncTorque() {
    // Cyclic Synchronous Torque Mode (CST)
    // Torque demand comes directly from master each cycle
    
    // Apply torque offset
    int16_t totalTorque = targetTorque_ + torqueOffset_;
    
    // Clamp torque
    torqueDemand_ = std::clamp(totalTorque, static_cast<int16_t>(-static_cast<int16_t>(maxTorque_)), static_cast<int16_t>(maxTorque_));
    
    // Update position/velocity from actual
    positionDemand_ = actualPosition_;
    velocityDemand_ = actualVelocity_;
    
    targetReached_ = true;  // CST is always "reached"
}

void CiA402Slave::executeHoming() {
    // Homing Mode (HM)
    
    bool homingStart = (controlWord_ & ControlWordBits::HomingStart) != 0;
    
    if (homingStart && !homingActive_) {
        // Start homing
        homingActive_ = true;
        homingComplete_ = false;
    }
    
    if (!homingActive_) {
        return;
    }
    
    // Use callback if provided
    if (homingCallback_) {
        int32_t homePos = 0;
        if (homingCallback_(homingMethod_, homePos)) {
            // Homing complete
            actualPosition_ = homePos + homeOffset_;
            internalPosition_ = actualPosition_;
            positionDemand_ = actualPosition_;
            homingComplete_ = true;
            homingActive_ = false;
            targetReached_ = true;
        }
    } else {
        // Simple simulated homing: move to zero
        // This is a simplified implementation - real homing would involve
        // searching for limit switches, index pulses, etc.
        
        int32_t error = homeOffset_ - actualPosition_;
        
        if (std::abs(error) < 100) {
            // Close enough - homing complete
            actualPosition_ = homeOffset_;
            internalPosition_ = actualPosition_;
            positionDemand_ = actualPosition_;
            homingComplete_ = true;
            homingActive_ = false;
            targetReached_ = true;
        } else {
            // Move toward home
            double vel = (error > 0) ? static_cast<double>(homingSwitchSpeed_) 
                                     : static_cast<double>(-homingSwitchSpeed_);
            velocityDemand_ = static_cast<int32_t>(vel);
            
            // Velocity -> torque
            int32_t velError = velocityDemand_ - actualVelocity_;
            double kv = 0.1;
            double torqueCmd = kv * static_cast<double>(velError);
            torqueCmd = std::clamp(torqueCmd, static_cast<double>(-maxTorque_), static_cast<double>(maxTorque_));
            torqueDemand_ = static_cast<int16_t>(torqueCmd);
        }
    }
}

// ============================================================================
// Position/Velocity Window Checks
// ============================================================================

bool CiA402Slave::checkPositionReached() {
    if (operatingMode_ == CiA402ModeValue::ProfilePosition ||
        operatingMode_ == CiA402ModeValue::CyclicSyncPosition) {
        int32_t error = std::abs(positionDemand_ - actualPosition_);
        return error < static_cast<int32_t>(driveConfig_.positionWindow);
    }
    return false;
}

bool CiA402Slave::checkVelocityReached() {
    if (operatingMode_ == CiA402ModeValue::ProfileVelocity ||
        operatingMode_ == CiA402ModeValue::VelocityMode ||
        operatingMode_ == CiA402ModeValue::CyclicSyncVelocity) {
        int32_t error = std::abs(targetVelocity_ - actualVelocity_);
        return error < static_cast<int32_t>(driveConfig_.velocityThreshold);
    }
    return false;
}

}  // namespace slave
}  // namespace EtherCAT
