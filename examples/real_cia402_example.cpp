/**
 * @file real_cia402_example.cpp
 * @brief Example: Linux host process controlling a real CiA 402 servo drive
 * 
 * This example demonstrates how to:
 * - Initialize EtherCAT master on Linux
 * - Scan for and configure CiA 402 drives
 * - Perform homing operation
 * - Execute profile position mode
 * - Execute cyclic synchronous position mode
 * - Handle drive faults
 */

#include <iostream>
#include <chrono>
#include <thread>
#include <atomic>
#include <cstring>
#include <cmath>

#include "tether/platform/Platform.hpp"
#include "tether/utils/SignalHandler.hpp"

// EtherCAT master includes (using SOEM or similar)
// #include "ethercat.h"

// Our library includes
// #include "Master.hpp"
// #include "CiA402Drive.hpp"

// =============================================================================
// Configuration
// =============================================================================

constexpr char NETWORK_INTERFACE[] = "eth0";
constexpr int CYCLE_TIME_US = 1000;  // 1ms cycle time
constexpr int SLAVE_ID = 1;          // First slave

// Motion parameters
constexpr int32_t MAX_VELOCITY = 100000;      // counts/s
constexpr int32_t MAX_ACCELERATION = 1000000; // counts/s²
constexpr int32_t PROFILE_VELOCITY = 50000;
constexpr int32_t PROFILE_ACCEL = 500000;

// =============================================================================
// Global State
// =============================================================================

std::atomic<bool> g_running{true};

// =============================================================================
// CiA 402 Object Dictionary Addresses
// =============================================================================

namespace OD {
    // Control and status
    constexpr uint16_t CONTROLWORD = 0x6040;
    constexpr uint16_t STATUSWORD = 0x6041;
    constexpr uint16_t MODES_OF_OPERATION = 0x6060;
    constexpr uint16_t MODES_OF_OPERATION_DISPLAY = 0x6061;
    
    // Position
    constexpr uint16_t POSITION_ACTUAL = 0x6064;
    constexpr uint16_t POSITION_DEMAND = 0x6062;
    constexpr uint16_t TARGET_POSITION = 0x607A;
    constexpr uint16_t FOLLOWING_ERROR = 0x60F4;
    
    // Velocity
    constexpr uint16_t VELOCITY_ACTUAL = 0x606C;
    constexpr uint16_t TARGET_VELOCITY = 0x60FF;
    constexpr uint16_t PROFILE_VELOCITY_OBJ = 0x6081;
    
    // Torque
    constexpr uint16_t TARGET_TORQUE = 0x6071;
    constexpr uint16_t TORQUE_ACTUAL = 0x6077;
    
    // Profile parameters
    constexpr uint16_t PROFILE_ACCELERATION = 0x6083;
    constexpr uint16_t PROFILE_DECELERATION = 0x6084;
    
    // Homing
    constexpr uint16_t HOMING_METHOD = 0x6098;
    constexpr uint16_t HOMING_SPEED_SEARCH = 0x6099;
    constexpr uint16_t HOMING_SPEED_ZERO = 0x609A;
    constexpr uint16_t HOMING_ACCELERATION = 0x609B;
}

// =============================================================================
// CiA 402 Control Word Bits
// =============================================================================

namespace ControlWord {
    constexpr uint16_t SWITCH_ON = 0x0001;
    constexpr uint16_t ENABLE_VOLTAGE = 0x0002;
    constexpr uint16_t QUICK_STOP = 0x0004;
    constexpr uint16_t ENABLE_OPERATION = 0x0008;
    constexpr uint16_t NEW_SETPOINT = 0x0010;         // PP mode
    constexpr uint16_t CHANGE_SET_IMMEDIATELY = 0x0020; // PP mode
    constexpr uint16_t RELATIVE_MOVE = 0x0040;        // PP mode
    constexpr uint16_t FAULT_RESET = 0x0080;
    constexpr uint16_t HALT = 0x0100;
    
    // Operating mode specific bits start at bit 4
    constexpr uint16_t HOMING_START = 0x0010;
}

// =============================================================================
// CiA 402 Status Word Bits
// =============================================================================

namespace StatusWord {
    constexpr uint16_t READY_TO_SWITCH_ON = 0x0001;
    constexpr uint16_t SWITCHED_ON = 0x0002;
    constexpr uint16_t OPERATION_ENABLED = 0x0004;
    constexpr uint16_t FAULT = 0x0008;
    constexpr uint16_t VOLTAGE_ENABLED = 0x0010;
    constexpr uint16_t QUICK_STOP = 0x0020;
    constexpr uint16_t SWITCH_ON_DISABLED = 0x0040;
    constexpr uint16_t WARNING = 0x0080;
    constexpr uint16_t REMOTE = 0x0200;
    constexpr uint16_t TARGET_REACHED = 0x0400;
    constexpr uint16_t INTERNAL_LIMIT = 0x0800;
    
    // Operating mode specific
    constexpr uint16_t SET_POINT_ACK = 0x1000;        // PP mode
    constexpr uint16_t HOMING_ATTAINED = 0x1000;      // HM mode
    constexpr uint16_t HOMING_ERROR = 0x2000;         // HM mode
}

// =============================================================================
// CiA 402 Operating Modes
// =============================================================================

enum class OperatingMode : int8_t {
    ProfilePosition = 1,
    ProfileVelocity = 3,
    ProfileTorque = 4,
    Homing = 6,
    InterpolatedPosition = 7,
    CyclicSyncPosition = 8,
    CyclicSyncVelocity = 9,
    CyclicSyncTorque = 10,
};

// =============================================================================
// State Machine States
// =============================================================================

enum class DriveState {
    NotReadyToSwitchOn,
    SwitchOnDisabled,
    ReadyToSwitchOn,
    SwitchedOn,
    OperationEnabled,
    QuickStopActive,
    FaultReactionActive,
    Fault,
    Unknown
};

DriveState decodeDriveState(uint16_t statusword) {
    // Mask relevant bits
    uint16_t masked = statusword & 0x006F;
    
    if ((masked & 0x004F) == 0x0000) return DriveState::NotReadyToSwitchOn;
    if ((masked & 0x004F) == 0x0040) return DriveState::SwitchOnDisabled;
    if ((masked & 0x006F) == 0x0021) return DriveState::ReadyToSwitchOn;
    if ((masked & 0x006F) == 0x0023) return DriveState::SwitchedOn;
    if ((masked & 0x006F) == 0x0027) return DriveState::OperationEnabled;
    if ((masked & 0x006F) == 0x0007) return DriveState::QuickStopActive;
    if ((masked & 0x004F) == 0x000F) return DriveState::FaultReactionActive;
    if ((masked & 0x004F) == 0x0008) return DriveState::Fault;
    
    return DriveState::Unknown;
}

std::string driveStateToString(DriveState state) {
    switch (state) {
        case DriveState::NotReadyToSwitchOn: return "Not Ready to Switch On";
        case DriveState::SwitchOnDisabled: return "Switch On Disabled";
        case DriveState::ReadyToSwitchOn: return "Ready to Switch On";
        case DriveState::SwitchedOn: return "Switched On";
        case DriveState::OperationEnabled: return "Operation Enabled";
        case DriveState::QuickStopActive: return "Quick Stop Active";
        case DriveState::FaultReactionActive: return "Fault Reaction Active";
        case DriveState::Fault: return "Fault";
        default: return "Unknown";
    }
}

// =============================================================================
// Simulated EtherCAT Interface (for compilation)
// =============================================================================

// In a real implementation, these would call the actual SOEM functions

namespace EtherCAT {

bool init(const char* ifname) {
    std::cout << "Initializing EtherCAT on interface: " << ifname << std::endl;
    // ec_init(ifname);
    return true;
}

int scanSlaves() {
    std::cout << "Scanning for slaves..." << std::endl;
    // ec_config_init(FALSE);
    // return ec_slavecount;
    return 1; // Simulated
}

bool configureSlave(int slave) {
    std::cout << "Configuring slave " << slave << std::endl;
    // Configure PDO mappings, DC, etc.
    return true;
}

bool sdoWrite(int slave, uint16_t index, uint8_t subindex, 
              void* data, int size) {
    std::cout << "SDO Write: 0x" << std::hex << index 
              << ":" << (int)subindex << std::dec << std::endl;
    // ec_SDOwrite(slave, index, subindex, FALSE, size, data, EC_TIMEOUTRXM);
    return true;
}

bool sdoRead(int slave, uint16_t index, uint8_t subindex,
             void* data, int* size) {
    std::cout << "SDO Read: 0x" << std::hex << index 
              << ":" << (int)subindex << std::dec << std::endl;
    // int s = *size;
    // ec_SDOread(slave, index, subindex, FALSE, &s, data, EC_TIMEOUTRXM);
    return true;
}

void processData() {
    // ec_send_processdata();
    // ec_receive_processdata(EC_TIMEOUTRET);
}

void close() {
    std::cout << "Closing EtherCAT" << std::endl;
    // ec_close();
}

} // namespace EtherCAT

// =============================================================================
// Drive Control Class
// =============================================================================

class CiA402DriveController {
public:
    CiA402DriveController(int slaveId) : slaveId_(slaveId) {}
    
    // Initialize drive
    bool initialize() {
        std::cout << "Initializing drive..." << std::endl;
        
        // Set operating mode to Profile Position
        int8_t mode = static_cast<int8_t>(OperatingMode::ProfilePosition);
        if (!EtherCAT::sdoWrite(slaveId_, OD::MODES_OF_OPERATION, 0, &mode, 1)) {
            return false;
        }
        
        // Set profile parameters
        if (!setProfileParameters()) {
            return false;
        }
        
        return true;
    }
    
    // State machine transition to Operation Enabled
    bool enableOperation() {
        std::cout << "Enabling operation..." << std::endl;
        
        // Transition through states
        // Switch On Disabled -> Ready to Switch On
        controlword_ = 0x0006; // Shutdown
        updatePDO();
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        
        // Ready to Switch On -> Switched On
        controlword_ = 0x0007; // Switch On
        updatePDO();
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        
        // Switched On -> Operation Enabled
        controlword_ = 0x000F; // Enable Operation
        updatePDO();
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        
        return true;
    }
    
    // Disable operation
    bool disableOperation() {
        controlword_ = 0x0006; // Shutdown
        updatePDO();
        return true;
    }
    
    // Quick stop
    void quickStop() {
        controlword_ = 0x0002; // Quick Stop
        updatePDO();
    }
    
    // Reset fault
    bool resetFault() {
        controlword_ = ControlWord::FAULT_RESET;
        updatePDO();
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        controlword_ = 0;
        updatePDO();
        return true;
    }
    
    // Homing
    bool performHoming(int8_t method = 33) {
        std::cout << "Starting homing with method " << (int)method << std::endl;
        
        // Set homing mode
        int8_t mode = static_cast<int8_t>(OperatingMode::Homing);
        EtherCAT::sdoWrite(slaveId_, OD::MODES_OF_OPERATION, 0, &mode, 1);
        
        // Set homing method
        EtherCAT::sdoWrite(slaveId_, OD::HOMING_METHOD, 0, &method, 1);
        
        // Set homing speeds
        uint32_t searchSpeed = 1000;
        uint32_t zeroSpeed = 100;
        EtherCAT::sdoWrite(slaveId_, OD::HOMING_SPEED_SEARCH, 1, &searchSpeed, 4);
        EtherCAT::sdoWrite(slaveId_, OD::HOMING_SPEED_ZERO, 2, &zeroSpeed, 4);
        
        // Enable operation and start homing
        enableOperation();
        
        controlword_ |= ControlWord::HOMING_START;
        updatePDO();
        
        // Wait for homing to complete
        int timeout = 30000; // 30 seconds
        while (timeout > 0 && g_running) {
            EtherCAT::processData();
            
            if (statusword_ & StatusWord::HOMING_ATTAINED) {
                std::cout << "Homing attained!" << std::endl;
                return true;
            }
            if (statusword_ & StatusWord::HOMING_ERROR) {
                std::cerr << "Homing error!" << std::endl;
                return false;
            }
            
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            timeout -= 10;
        }
        
        std::cerr << "Homing timeout!" << std::endl;
        return false;
    }
    
    // Profile Position Mode move
    bool moveToPosition(int32_t position, bool relative = false) {
        std::cout << "Moving to position: " << position 
                  << (relative ? " (relative)" : " (absolute)") << std::endl;
        
        // Set Profile Position mode
        int8_t mode = static_cast<int8_t>(OperatingMode::ProfilePosition);
        EtherCAT::sdoWrite(slaveId_, OD::MODES_OF_OPERATION, 0, &mode, 1);
        
        // Set target position
        targetPosition_ = position;
        
        // Set new setpoint
        controlword_ |= ControlWord::NEW_SETPOINT;
        if (relative) {
            controlword_ |= ControlWord::RELATIVE_MOVE;
        } else {
            controlword_ &= ~ControlWord::RELATIVE_MOVE;
        }
        
        updatePDO();
        
        // Wait for setpoint acknowledge
        int timeout = 100;
        while (timeout > 0) {
            EtherCAT::processData();
            if (statusword_ & StatusWord::SET_POINT_ACK) {
                break;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            timeout--;
        }
        
        // Clear new setpoint
        controlword_ &= ~ControlWord::NEW_SETPOINT;
        updatePDO();
        
        return true;
    }
    
    // Wait for motion complete
    bool waitForTarget(int timeoutMs = 10000) {
        std::cout << "Waiting for target..." << std::endl;
        
        while (timeoutMs > 0 && g_running) {
            EtherCAT::processData();
            
            if (statusword_ & StatusWord::TARGET_REACHED) {
                std::cout << "Target reached!" << std::endl;
                return true;
            }
            
            // Check for faults
            if (statusword_ & StatusWord::FAULT) {
                std::cerr << "Drive fault during motion!" << std::endl;
                return false;
            }
            
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            timeoutMs -= 10;
        }
        
        std::cerr << "Motion timeout!" << std::endl;
        return false;
    }
    
    // CSP (Cyclic Synchronous Position) mode
    bool enterCSPMode() {
        int8_t mode = static_cast<int8_t>(OperatingMode::CyclicSyncPosition);
        return EtherCAT::sdoWrite(slaveId_, OD::MODES_OF_OPERATION, 0, &mode, 1);
    }
    
    // Set target for CSP mode
    void setCSPTarget(int32_t position) {
        targetPosition_ = position;
    }
    
    // Get current state
    DriveState getState() {
        return decodeDriveState(statusword_);
    }
    
    int32_t getActualPosition() { return actualPosition_; }
    int32_t getActualVelocity() { return actualVelocity_; }
    int16_t getActualTorque() { return actualTorque_; }
    uint16_t getStatusword() { return statusword_; }
    
    // Update PDO data (call every cycle)
    void updatePDO() {
        // In real implementation, this would access the process data
        // outputs_->controlword = controlword_;
        // outputs_->target_position = targetPosition_;
        
        // Read inputs
        // statusword_ = inputs_->statusword;
        // actualPosition_ = inputs_->position_actual;
        // actualVelocity_ = inputs_->velocity_actual;
        // actualTorque_ = inputs_->torque_actual;
    }
    
private:
    bool setProfileParameters() {
        uint32_t velocity = PROFILE_VELOCITY;
        uint32_t accel = PROFILE_ACCEL;
        uint32_t decel = PROFILE_ACCEL;
        
        EtherCAT::sdoWrite(slaveId_, OD::PROFILE_VELOCITY_OBJ, 0, &velocity, 4);
        EtherCAT::sdoWrite(slaveId_, OD::PROFILE_ACCELERATION, 0, &accel, 4);
        EtherCAT::sdoWrite(slaveId_, OD::PROFILE_DECELERATION, 0, &decel, 4);
        
        return true;
    }
    
    int slaveId_;
    
    // PDO data
    uint16_t controlword_ = 0;
    uint16_t statusword_ = 0;
    int32_t targetPosition_ = 0;
    int32_t actualPosition_ = 0;
    int32_t actualVelocity_ = 0;
    int16_t actualTorque_ = 0;
};

// =============================================================================
// Cyclic Task (Real-time Loop)
// =============================================================================

void cyclicTask(CiA402DriveController& drive) {
    using clock = std::chrono::high_resolution_clock;
    auto nextCycle = clock::now();
    
    while (g_running) {
        // Process EtherCAT
        EtherCAT::processData();
        
        // Update drive state
        drive.updatePDO();
        
        // Calculate next cycle time
        nextCycle += std::chrono::microseconds(CYCLE_TIME_US);
        std::this_thread::sleep_until(nextCycle);
    }
}

// =============================================================================
// Main Application
// =============================================================================

int main(int argc, char* argv[]) {
    std::cout << "=== CiA 402 Drive Control Example ===" << std::endl;

    Tether::Platform::ensureRealtimeKernelOrExit();
    
    // Set up signal handler
    Tether::Utils::SignalHandler sig_handler(g_running, false);
    
    // Initialize EtherCAT
    if (!EtherCAT::init(NETWORK_INTERFACE)) {
        std::cerr << "Failed to initialize EtherCAT!" << std::endl;
        return 1;
    }
    
    // Scan for slaves
    int slaveCount = EtherCAT::scanSlaves();
    if (slaveCount == 0) {
        std::cerr << "No slaves found!" << std::endl;
        EtherCAT::close();
        return 1;
    }
    std::cout << "Found " << slaveCount << " slave(s)" << std::endl;
    
    // Configure slaves
    if (!EtherCAT::configureSlave(SLAVE_ID)) {
        std::cerr << "Failed to configure slave!" << std::endl;
        EtherCAT::close();
        return 1;
    }
    
    // Create drive controller
    CiA402DriveController drive(SLAVE_ID);
    
    // Initialize drive
    if (!drive.initialize()) {
        std::cerr << "Failed to initialize drive!" << std::endl;
        EtherCAT::close();
        return 1;
    }
    
    // Start cyclic task in background
    std::thread cyclicThread([&drive]() {
        if (!Tether::Platform::setCurrentThreadRealtime(-1)) {
            TETHER_LOGW("real_cia402", "cyclicThread: could not set realtime scheduling (continuing)");
        }
        cyclicTask(drive);
    });
    
    // Check current state
    DriveState state = drive.getState();
    std::cout << "Current state: " << driveStateToString(state) << std::endl;
    
    // Handle fault if present
    if (state == DriveState::Fault) {
        std::cout << "Resetting fault..." << std::endl;
        drive.resetFault();
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    
    // Enable operation
    if (!drive.enableOperation()) {
        std::cerr << "Failed to enable operation!" << std::endl;
        g_running = false;
        cyclicThread.join();
        EtherCAT::close();
        return 1;
    }
    
    std::cout << "Drive enabled. State: " << driveStateToString(drive.getState()) << std::endl;
    
    // Perform homing
    std::cout << "\n--- Homing ---" << std::endl;
    if (!drive.performHoming(37)) { // Method 37: current position
        std::cerr << "Homing failed!" << std::endl;
        drive.disableOperation();
        g_running = false;
        cyclicThread.join();
        EtherCAT::close();
        return 1;
    }
    
    // Re-enable after homing
    drive.enableOperation();
    
    // Execute some moves
    std::cout << "\n--- Profile Position Mode ---" << std::endl;
    
    // Move to position 10000
    drive.moveToPosition(10000);
    if (!drive.waitForTarget()) {
        drive.disableOperation();
        g_running = false;
        cyclicThread.join();
        EtherCAT::close();
        return 1;
    }
    
    std::cout << "Position: " << drive.getActualPosition() << std::endl;
    
    // Move back to 0
    drive.moveToPosition(0);
    if (!drive.waitForTarget()) {
        drive.disableOperation();
        g_running = false;
        cyclicThread.join();
        EtherCAT::close();
        return 1;
    }
    
    std::cout << "Position: " << drive.getActualPosition() << std::endl;
    
    // Relative move
    drive.moveToPosition(5000, true);  // Relative
    if (!drive.waitForTarget()) {
        drive.disableOperation();
        g_running = false;
        cyclicThread.join();
        EtherCAT::close();
        return 1;
    }
    
    std::cout << "Position: " << drive.getActualPosition() << std::endl;
    
    // CSP Mode example
    std::cout << "\n--- Cyclic Synchronous Position Mode ---" << std::endl;
    drive.enterCSPMode();
    
    // Generate sine wave motion
    int32_t centerPos = drive.getActualPosition();
    int32_t amplitude = 2000;
    double frequency = 0.5; // Hz
    
    auto startTime = std::chrono::steady_clock::now();
    double duration = 5.0; // seconds
    
    while (g_running) {
        auto now = std::chrono::steady_clock::now();
        double elapsed = std::chrono::duration<double>(now - startTime).count();
        
        if (elapsed > duration) break;
        
        // Calculate sine wave position
        double angle = 2.0 * M_PI * frequency * elapsed;
        int32_t target = centerPos + static_cast<int32_t>(amplitude * sin(angle));
        
        drive.setCSPTarget(target);
        
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    
    // Return to center
    drive.moveToPosition(centerPos);
    drive.waitForTarget(5000);
    
    // Disable operation
    std::cout << "\n--- Shutting Down ---" << std::endl;
    drive.disableOperation();
    
    // Stop cyclic thread
    g_running = false;
    cyclicThread.join();
    
    // Close EtherCAT
    EtherCAT::close();
    
    std::cout << "Done!" << std::endl;
    return 0;
}
