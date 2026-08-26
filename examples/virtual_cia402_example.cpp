/**
 * @file virtual_cia402_example.cpp
 * @brief Example: Virtual CiA 402 Slave with Physical Motor Model
 * 
 * This example demonstrates:
 * - Setting up a virtual CiA 402 slave
 * - Configuring the physical motor model
 * - Running with dual-loop position control
 * - Simulating homing operations
 * - Testing fault conditions
 * - Integration with EtherCAT master
 */

#include <iostream>
#include <chrono>
#include <thread>
#include <memory>
#include <cmath>
#include <atomic>

#include "tether/utils/SignalHandler.hpp"

// Our library includes (adjust paths as needed)
// #include "slave/profile/CiA402Slave.hpp"
// #include "profiles/cia402/MotorModel.hpp"
// #include "profiles/cia402/HomingModes.hpp"

// =============================================================================
// Configuration
// =============================================================================

namespace Config {
    // Motor parameters (typical servo motor)
    constexpr double INERTIA = 0.001;           // kg⋅m²
    constexpr double VISCOUS_DAMPING = 0.005;   // Nm/(rad/s)
    constexpr double STATIC_FRICTION = 0.1;     // Nm
    constexpr double COULOMB_FRICTION = 0.05;   // Nm
    constexpr double MAX_TORQUE = 10.0;         // Nm
    constexpr double MAX_VELOCITY = 628.0;      // rad/s (~6000 RPM)
    constexpr double MAX_ACCELERATION = 6280.0; // rad/s²
    constexpr double TORQUE_CONSTANT = 0.1;     // Nm/A
    constexpr int32_t ENCODER_RESOLUTION = 131072; // 17-bit encoder
    constexpr double GEAR_RATIO = 1.0;
    
    // Position controller (outer loop)
    constexpr double POS_KP = 100.0;
    constexpr double POS_KI = 10.0;
    constexpr double POS_KD = 5.0;
    
    // Velocity controller (inner loop)
    constexpr double VEL_KP = 1.0;
    constexpr double VEL_KI = 0.5;
    constexpr double VEL_KD = 0.01;
    
    // Simulation
    constexpr double SIM_DT = 0.001;        // 1ms simulation step
    constexpr double CYCLE_TIME = 0.001;    // 1ms EtherCAT cycle
    
    // Endstops
    constexpr int32_t POS_LIMIT = 1000000;  // counts
    constexpr int32_t NEG_LIMIT = -1000000;
    constexpr int32_t HOME_POS = 0;
    constexpr int32_t INDEX_POS = 0;
}

// =============================================================================
// Placeholder Types (for compilation without full includes)
// =============================================================================

namespace CiA402 {
namespace Motor {

struct MotorParams {
    double inertia = 0.001;
    double viscousDamping = 0.01;
    double staticFriction = 0.1;
    double coulombFriction = 0.05;
    double maxTorque = 10.0;
    double maxVelocity = 100.0;
    double maxAcceleration = 1000.0;
    double torqueConstant = 0.1;
    double backEMFConstant = 0.1;
    int32_t encoderResolution = 4096;
    double gearRatio = 1.0;
    double gearEfficiency = 0.95;
    double maxPosition = 1e9;
    double minPosition = -1e9;
    double stictionVelocity = 0.01;
    double windingResistance = 1.0;
    double ratedCurrent = 5.0;
    double peakCurrent = 15.0;
};

struct ThermalParams {
    bool enableThermalModel = false;
    double thermalResistance = 5.0;
    double thermalCapacity = 100.0;
    double ambientTemp = 25.0;
    double maxWindingTemp = 120.0;
    double maxMotorTemp = 80.0;
};

struct BrakingResistorParams {
    bool enabled = false;
    double resistance = 10.0;
    double maxPower = 500.0;
    double maxTemperature = 150.0;
    double thermalTimeConstant = 60.0;
    bool enableOverheatProtection = true;
};

struct EndstopConfig {
    bool positiveEnabled = false;
    int32_t positivePosition = 1000000;
    bool positiveNC = false;
    bool positiveFault = false;
    
    bool negativeEnabled = false;
    int32_t negativePosition = -1000000;
    bool negativeNC = false;
    bool negativeFault = false;
    
    bool homeEnabled = false;
    int32_t homePosition = 0;
    int32_t homeWidth = 100;
    bool homeNC = false;
    bool homeFault = false;
    
    bool indexEnabled = false;
    int32_t indexPosition = 0;
    bool indexFault = false;
};

struct PositionControllerParams {
    double posKp = 100.0;
    double posKi = 10.0;
    double posKd = 5.0;
    double posIntegralLimit = 100.0;
    double posDerivativeFilter = 0.1;
    
    double velKp = 1.0;
    double velKi = 0.5;
    double velKd = 0.01;
    double velIntegralLimit = 10.0;
    double velDerivativeFilter = 0.1;
    
    double maxVelocityCmd = 100.0;
    double maxTorqueCmd = 10.0;
    
    double velocityFeedforward = 0.0;
    double accelerationFeedforward = 0.0;
};

struct VelocityControllerParams {
    double kp = 1.0;
    double ki = 0.5;
    double kd = 0.01;
    double integralLimit = 10.0;
    double derivativeFilter = 0.1;
    double maxTorqueCmd = 10.0;
    double torqueFeedforward = 0.0;
};

enum class ControlMode {
    Disabled,
    Position,
    Velocity,
    Torque
};

} // namespace Motor
} // namespace CiA402

// =============================================================================
// Global State
// =============================================================================

std::atomic<bool> g_running{true};

// =============================================================================
// Virtual CiA 402 Slave Implementation
// =============================================================================

class VirtualCiA402Slave {
public:
    VirtualCiA402Slave() {
        initializeMotorModel();
    }
    
    void initializeMotorModel() {
        // Configure motor parameters
        motorParams_.inertia = Config::INERTIA;
        motorParams_.viscousDamping = Config::VISCOUS_DAMPING;
        motorParams_.staticFriction = Config::STATIC_FRICTION;
        motorParams_.coulombFriction = Config::COULOMB_FRICTION;
        motorParams_.maxTorque = Config::MAX_TORQUE;
        motorParams_.maxVelocity = Config::MAX_VELOCITY;
        motorParams_.maxAcceleration = Config::MAX_ACCELERATION;
        motorParams_.torqueConstant = Config::TORQUE_CONSTANT;
        motorParams_.encoderResolution = Config::ENCODER_RESOLUTION;
        motorParams_.gearRatio = Config::GEAR_RATIO;
        
        // Configure endstops
        endstopConfig_.positiveEnabled = true;
        endstopConfig_.positivePosition = Config::POS_LIMIT;
        endstopConfig_.negativeEnabled = true;
        endstopConfig_.negativePosition = Config::NEG_LIMIT;
        endstopConfig_.homeEnabled = true;
        endstopConfig_.homePosition = Config::HOME_POS;
        endstopConfig_.homeWidth = 200;
        endstopConfig_.indexEnabled = true;
        endstopConfig_.indexPosition = Config::INDEX_POS;
        
        // Configure position controller (dual-loop)
        posCtrlParams_.posKp = Config::POS_KP;
        posCtrlParams_.posKi = Config::POS_KI;
        posCtrlParams_.posKd = Config::POS_KD;
        posCtrlParams_.velKp = Config::VEL_KP;
        posCtrlParams_.velKi = Config::VEL_KI;
        posCtrlParams_.velKd = Config::VEL_KD;
        posCtrlParams_.maxVelocityCmd = Config::MAX_VELOCITY;
        posCtrlParams_.maxTorqueCmd = Config::MAX_TORQUE;
        
        // Configure velocity controller (single-loop)
        velCtrlParams_.kp = Config::VEL_KP;
        velCtrlParams_.ki = Config::VEL_KI;
        velCtrlParams_.kd = Config::VEL_KD;
        velCtrlParams_.maxTorqueCmd = Config::MAX_TORQUE;
        
        std::cout << "Motor model initialized" << std::endl;
        std::cout << "  Inertia: " << motorParams_.inertia << " kg⋅m²" << std::endl;
        std::cout << "  Max torque: " << motorParams_.maxTorque << " Nm" << std::endl;
        std::cout << "  Encoder: " << motorParams_.encoderResolution << " counts/rev" << std::endl;
    }
    
    // Process PDO data from master
    void processRxPDO(uint16_t controlword, int32_t targetPosition,
                      int32_t targetVelocity, int16_t targetTorque) {
        controlword_ = controlword;
        targetPosition_ = targetPosition;
        targetVelocity_ = targetVelocity;
        targetTorque_ = targetTorque;
        
        // Process state machine
        processControlword();
    }
    
    // Prepare PDO data for master
    void prepareTxPDO(uint16_t& statusword, int32_t& actualPosition,
                      int32_t& actualVelocity, int16_t& actualTorque) {
        statusword = statusword_;
        actualPosition = actualPosition_;
        actualVelocity = actualVelocity_;
        actualTorque = actualTorque_;
    }
    
    // Run simulation step
    void update(double dt) {
        // Only run motor when in Operation Enabled
        if (driveState_ == DriveState::OperationEnabled) {
            runMotorModel(dt);
        }
        
        // Update status word
        updateStatusword();
    }
    
    // Get current state
    DriveState getState() const { return driveState_; }
    int32_t getPosition() const { return actualPosition_; }
    int32_t getVelocity() const { return actualVelocity_; }
    bool hasFault() const { return hasFault_; }
    
    // Inject fault for testing
    void injectFault(uint16_t errorCode) {
        hasFault_ = true;
        errorCode_ = errorCode;
        driveState_ = DriveState::Fault;
    }
    
    void clearFault() {
        hasFault_ = false;
        errorCode_ = 0;
    }

private:
    enum class DriveState {
        NotReadyToSwitchOn,
        SwitchOnDisabled,
        ReadyToSwitchOn,
        SwitchedOn,
        OperationEnabled,
        QuickStopActive,
        FaultReactionActive,
        Fault
    };
    
    void processControlword() {
        // Fault reset
        if ((controlword_ & 0x0080) && hasFault_) {
            clearFault();
            driveState_ = DriveState::SwitchOnDisabled;
            return;
        }
        
        // State transitions based on controlword
        switch (driveState_) {
            case DriveState::SwitchOnDisabled:
                if ((controlword_ & 0x0006) == 0x0006) {
                    driveState_ = DriveState::ReadyToSwitchOn;
                }
                break;
                
            case DriveState::ReadyToSwitchOn:
                if ((controlword_ & 0x0007) == 0x0007) {
                    driveState_ = DriveState::SwitchedOn;
                }
                break;
                
            case DriveState::SwitchedOn:
                if ((controlword_ & 0x000F) == 0x000F) {
                    driveState_ = DriveState::OperationEnabled;
                }
                break;
                
            case DriveState::OperationEnabled:
                if ((controlword_ & 0x000F) != 0x000F) {
                    driveState_ = DriveState::SwitchedOn;
                }
                // Quick stop
                if ((controlword_ & 0x0004) == 0) {
                    driveState_ = DriveState::QuickStopActive;
                }
                break;
                
            case DriveState::QuickStopActive:
                // Decelerate to stop
                // When stopped, go to Switch On Disabled
                if (std::abs(actualVelocity_) < 10) {
                    driveState_ = DriveState::SwitchOnDisabled;
                }
                break;
                
            default:
                break;
        }
    }
    
    void updateStatusword() {
        statusword_ = 0;
        
        switch (driveState_) {
            case DriveState::NotReadyToSwitchOn:
                statusword_ = 0x0000;
                break;
            case DriveState::SwitchOnDisabled:
                statusword_ = 0x0040;
                break;
            case DriveState::ReadyToSwitchOn:
                statusword_ = 0x0021;
                break;
            case DriveState::SwitchedOn:
                statusword_ = 0x0023;
                break;
            case DriveState::OperationEnabled:
                statusword_ = 0x0027;
                break;
            case DriveState::QuickStopActive:
                statusword_ = 0x0007;
                break;
            case DriveState::FaultReactionActive:
                statusword_ = 0x000F;
                break;
            case DriveState::Fault:
                statusword_ = 0x0008;
                break;
        }
        
        // Add target reached flag
        if (targetReached_) {
            statusword_ |= 0x0400;
        }
        
        // Add fault flag
        if (hasFault_) {
            statusword_ |= 0x0008;
        }
    }
    
    void runMotorModel(double dt) {
        // Simple motor simulation
        // In full implementation, this would use MotorModel class
        
        // Determine control mode from operating mode
        CiA402::Motor::ControlMode mode = CiA402::Motor::ControlMode::Disabled;
        
        switch (operatingMode_) {
            case 1: // Profile Position
            case 7: // Interpolated Position
            case 8: // CSP
                mode = CiA402::Motor::ControlMode::Position;
                break;
            case 3: // Profile Velocity
            case 9: // CSV
                mode = CiA402::Motor::ControlMode::Velocity;
                break;
            case 4: // Profile Torque
            case 10: // CST
                mode = CiA402::Motor::ControlMode::Torque;
                break;
            default:
                break;
        }
        
        // Simple physics simulation
        double torqueCmd = 0.0;
        
        if (mode == CiA402::Motor::ControlMode::Position) {
            // Dual-loop position control
            double posError = (targetPosition_ - actualPosition_) / 
                              static_cast<double>(motorParams_.encoderResolution);
            
            // Outer loop: position -> velocity
            double velCmd = posCtrlParams_.posKp * posError;
            velCmd = std::clamp(velCmd, -posCtrlParams_.maxVelocityCmd, 
                               posCtrlParams_.maxVelocityCmd);
            
            // Inner loop: velocity -> torque
            double velActual = actualVelocity_ / 
                               static_cast<double>(motorParams_.encoderResolution) * 6.28318;
            double velError = velCmd - velActual;
            torqueCmd = posCtrlParams_.velKp * velError;
        }
        else if (mode == CiA402::Motor::ControlMode::Velocity) {
            // Single-loop velocity control
            double velTarget = targetVelocity_ / 
                              static_cast<double>(motorParams_.encoderResolution) * 6.28318;
            double velActual = actualVelocity_ / 
                              static_cast<double>(motorParams_.encoderResolution) * 6.28318;
            double velError = velTarget - velActual;
            torqueCmd = velCtrlParams_.kp * velError;
        }
        else if (mode == CiA402::Motor::ControlMode::Torque) {
            torqueCmd = (targetTorque_ / 1000.0) * motorParams_.maxTorque;
        }
        
        // Limit torque
        torqueCmd = std::clamp(torqueCmd, -motorParams_.maxTorque, motorParams_.maxTorque);
        
        // Calculate friction
        double velocity = actualVelocity_ / 
                         static_cast<double>(motorParams_.encoderResolution) * 6.28318;
        double friction = motorParams_.viscousDamping * velocity;
        if (std::abs(velocity) > 0.001) {
            friction += motorParams_.coulombFriction * (velocity > 0 ? 1 : -1);
        }
        
        // Net torque
        double netTorque = torqueCmd - friction;
        
        // Acceleration
        double acceleration = netTorque / motorParams_.inertia;
        acceleration = std::clamp(acceleration, -motorParams_.maxAcceleration, 
                                 motorParams_.maxAcceleration);
        
        // Integrate velocity
        velocity += acceleration * dt;
        velocity = std::clamp(velocity, -motorParams_.maxVelocity, motorParams_.maxVelocity);
        
        // Integrate position
        double position = actualPosition_;
        position += velocity * dt * motorParams_.encoderResolution / 6.28318;
        
        // Update outputs
        actualPosition_ = static_cast<int32_t>(position);
        actualVelocity_ = static_cast<int32_t>(velocity * motorParams_.encoderResolution / 6.28318);
        actualTorque_ = static_cast<int16_t>(torqueCmd / motorParams_.maxTorque * 1000);
        
        // Check target reached
        if (mode == CiA402::Motor::ControlMode::Position) {
            targetReached_ = std::abs(actualPosition_ - targetPosition_) < 100 &&
                            std::abs(actualVelocity_) < 10;
        }
    }
    
    // Configuration
    CiA402::Motor::MotorParams motorParams_;
    CiA402::Motor::ThermalParams thermalParams_;
    CiA402::Motor::EndstopConfig endstopConfig_;
    CiA402::Motor::PositionControllerParams posCtrlParams_;
    CiA402::Motor::VelocityControllerParams velCtrlParams_;
    
    // State
    DriveState driveState_ = DriveState::SwitchOnDisabled;
    int8_t operatingMode_ = 8; // CSP
    bool hasFault_ = false;
    uint16_t errorCode_ = 0;
    bool targetReached_ = false;
    
    // PDO data (inputs from master)
    uint16_t controlword_ = 0;
    int32_t targetPosition_ = 0;
    int32_t targetVelocity_ = 0;
    int16_t targetTorque_ = 0;
    
    // PDO data (outputs to master)
    uint16_t statusword_ = 0;
    int32_t actualPosition_ = 0;
    int32_t actualVelocity_ = 0;
    int16_t actualTorque_ = 0;
};

// =============================================================================
// Main Application
// =============================================================================

int main(int argc, char* argv[]) {
    std::cout << "=== Virtual CiA 402 Slave Example ===" << std::endl;
    
    Tether::Utils::SignalHandler sig_handler(g_running, false);
    
    // Create virtual slave
    VirtualCiA402Slave slave;
    
    std::cout << "\nStarting simulation..." << std::endl;
    
    // Simulation loop
    auto startTime = std::chrono::steady_clock::now();
    double simTime = 0.0;
    
    // Master commands (simulated)
    uint16_t masterControlword = 0;
    int32_t masterTargetPos = 0;
    
    // State machine: enable drive
    std::cout << "\nEnabling drive..." << std::endl;
    
    // Step 1: Shutdown (to Ready to Switch On)
    masterControlword = 0x0006;
    slave.processRxPDO(masterControlword, 0, 0, 0);
    for (int i = 0; i < 10; i++) slave.update(Config::SIM_DT);
    std::cout << "State after Shutdown: " << static_cast<int>(slave.getState()) << std::endl;
    
    // Step 2: Switch On
    masterControlword = 0x0007;
    slave.processRxPDO(masterControlword, 0, 0, 0);
    for (int i = 0; i < 10; i++) slave.update(Config::SIM_DT);
    std::cout << "State after Switch On: " << static_cast<int>(slave.getState()) << std::endl;
    
    // Step 3: Enable Operation
    masterControlword = 0x000F;
    slave.processRxPDO(masterControlword, 0, 0, 0);
    for (int i = 0; i < 10; i++) slave.update(Config::SIM_DT);
    std::cout << "State after Enable: " << static_cast<int>(slave.getState()) << std::endl;
    
    // CSP motion example
    std::cout << "\n--- CSP Motion Test ---" << std::endl;
    
    int32_t amplitude = 50000;
    double frequency = 0.5; // Hz
    double duration = 5.0;  // seconds
    
    while (g_running && simTime < duration) {
        // Generate sine wave trajectory
        masterTargetPos = static_cast<int32_t>(amplitude * sin(2.0 * M_PI * frequency * simTime));
        
        // Send command to slave
        slave.processRxPDO(masterControlword, masterTargetPos, 0, 0);
        
        // Run simulation
        slave.update(Config::SIM_DT);
        
        // Read feedback
        uint16_t statusword;
        int32_t actualPos, actualVel;
        int16_t actualTorque;
        slave.prepareTxPDO(statusword, actualPos, actualVel, actualTorque);
        
        // Print status periodically
        if (static_cast<int>(simTime * 10) % 5 == 0) {
            std::cout << "t=" << std::fixed << std::setprecision(2) << simTime 
                      << "  Target=" << masterTargetPos 
                      << "  Actual=" << actualPos 
                      << "  Error=" << (masterTargetPos - actualPos)
                      << "  Torque=" << actualTorque / 10.0 << "%" << std::endl;
        }
        
        simTime += Config::SIM_DT;
        std::this_thread::sleep_for(std::chrono::microseconds(
            static_cast<int>(Config::SIM_DT * 1000000)));
    }
    
    // Step move test
    std::cout << "\n--- Step Move Test ---" << std::endl;
    
    masterTargetPos = 100000;
    double stepDuration = 2.0;
    double stepStart = simTime;
    
    while (g_running && simTime < stepStart + stepDuration) {
        slave.processRxPDO(masterControlword, masterTargetPos, 0, 0);
        slave.update(Config::SIM_DT);
        
        uint16_t statusword;
        int32_t actualPos, actualVel;
        int16_t actualTorque;
        slave.prepareTxPDO(statusword, actualPos, actualVel, actualTorque);
        
        if (static_cast<int>((simTime - stepStart) * 20) % 10 == 0) {
            std::cout << "t=" << std::fixed << std::setprecision(2) << (simTime - stepStart)
                      << "  Pos=" << actualPos 
                      << "  Vel=" << actualVel
                      << "  Err=" << (masterTargetPos - actualPos) << std::endl;
        }
        
        simTime += Config::SIM_DT;
        std::this_thread::sleep_for(std::chrono::microseconds(1000));
    }
    
    // Fault injection test
    std::cout << "\n--- Fault Injection Test ---" << std::endl;
    
    slave.injectFault(0x3210); // Over-current
    
    uint16_t statusword;
    int32_t actualPos, actualVel;
    int16_t actualTorque;
    slave.prepareTxPDO(statusword, actualPos, actualVel, actualTorque);
    
    std::cout << "Statusword after fault: 0x" << std::hex << statusword << std::dec << std::endl;
    std::cout << "Has fault: " << (slave.hasFault() ? "yes" : "no") << std::endl;
    
    // Reset fault
    masterControlword = 0x0080; // Fault reset
    slave.processRxPDO(masterControlword, 0, 0, 0);
    slave.update(Config::SIM_DT);
    
    slave.prepareTxPDO(statusword, actualPos, actualVel, actualTorque);
    std::cout << "Statusword after reset: 0x" << std::hex << statusword << std::dec << std::endl;
    
    // Disable
    std::cout << "\n--- Shutting Down ---" << std::endl;
    masterControlword = 0x0006;
    slave.processRxPDO(masterControlword, 0, 0, 0);
    slave.update(Config::SIM_DT);
    
    std::cout << "Done!" << std::endl;
    return 0;
}
