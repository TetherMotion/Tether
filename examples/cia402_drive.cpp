/**
 * @file cia402_drive.cpp
 * @brief Example: CiA 402 servo drive emulation
 * 
 * This example demonstrates creating a CiA 402 compliant servo drive slave
 * with full state machine and motion profile support.
 */

#include "slave/SlaveCore.hpp"
#include "slave/hal/LoopbackHAL.hpp"
#include "slave/profiles/CiA402Slave.hpp"
#include "slave/mailbox/CoEHandler.hpp"
#include "pcap/PcapLogger.hpp"
#include "tether/utils/SignalHandler.hpp"

#include <iostream>
#include <thread>
#include <chrono>
#include <atomic>
#include <cmath>
#include <iomanip>

std::atomic<bool> g_running{true};

class DriveSimulator {
public:
    DriveSimulator() 
        : actualPosition_(0)
        , actualVelocity_(0)
        , targetPosition_(0)
        , targetVelocity_(0)
        , maxVelocity_(10000)
        , maxAcceleration_(50000)
        , positionReached_(true)
    {}
    
    void setTargetPosition(int32_t pos) {
        targetPosition_ = pos;
        positionReached_ = false;
    }
    
    void setTargetVelocity(int32_t vel) {
        targetVelocity_ = std::min(std::max(vel, -maxVelocity_), maxVelocity_);
    }
    
    void setMaxVelocity(int32_t vel) { maxVelocity_ = vel; }
    void setMaxAcceleration(int32_t acc) { maxAcceleration_ = acc; }
    
    int32_t getActualPosition() const { return actualPosition_; }
    int32_t getActualVelocity() const { return actualVelocity_; }
    bool isPositionReached() const { return positionReached_; }
    
    void update(double dt) {
        // Simple trapezoidal motion profile
        int32_t error = targetPosition_ - actualPosition_;
        
        // Calculate desired velocity based on position error
        int32_t desiredVel = 0;
        if (std::abs(error) > 10) {  // Deadband
            // Deceleration distance
            int32_t decelDist = (actualVelocity_ * actualVelocity_) / (2 * maxAcceleration_);
            
            if (std::abs(error) <= decelDist + 100) {
                // Decelerate
                desiredVel = (error > 0) ? 
                    std::max(100, actualVelocity_ - static_cast<int32_t>(maxAcceleration_ * dt)) :
                    std::min(-100, actualVelocity_ + static_cast<int32_t>(maxAcceleration_ * dt));
            } else {
                // Accelerate or maintain speed
                desiredVel = (error > 0) ? maxVelocity_ : -maxVelocity_;
            }
        } else {
            desiredVel = 0;
            positionReached_ = true;
        }
        
        // Apply acceleration limit
        int32_t velError = desiredVel - actualVelocity_;
        int32_t maxDelta = static_cast<int32_t>(maxAcceleration_ * dt);
        
        if (velError > maxDelta) {
            actualVelocity_ += maxDelta;
        } else if (velError < -maxDelta) {
            actualVelocity_ -= maxDelta;
        } else {
            actualVelocity_ = desiredVel;
        }
        
        // Integrate velocity to position
        actualPosition_ += static_cast<int32_t>(actualVelocity_ * dt);
    }
    
    void halt() {
        targetVelocity_ = 0;
        targetPosition_ = actualPosition_;
    }
    
    void quickStop() {
        // Emergency deceleration
        actualVelocity_ = 0;
        targetVelocity_ = 0;
        targetPosition_ = actualPosition_;
    }
    
private:
    int32_t actualPosition_;
    int32_t actualVelocity_;
    int32_t targetPosition_;
    int32_t targetVelocity_;
    int32_t maxVelocity_;
    int32_t maxAcceleration_;
    bool positionReached_;
};

std::string driveStateToString(uint16_t statusWord) {
    uint8_t state = statusWord & 0x6F;
    
    if ((statusWord & 0x4F) == 0x00) return "Not ready to switch on";
    if ((statusWord & 0x6F) == 0x40) return "Switch on disabled";
    if ((statusWord & 0x6F) == 0x21) return "Ready to switch on";
    if ((statusWord & 0x6F) == 0x23) return "Switched on";
    if ((statusWord & 0x6F) == 0x27) return "Operation enabled";
    if ((statusWord & 0x6F) == 0x07) return "Quick stop active";
    if ((statusWord & 0x4F) == 0x0F) return "Fault reaction active";
    if ((statusWord & 0x4F) == 0x08) return "Fault";
    
    return "Unknown";
}

std::string operatingModeToString(int8_t mode) {
    switch (mode) {
        case 1: return "Profile Position (PP)";
        case 3: return "Profile Velocity (PV)";
        case 4: return "Profile Torque (PT)";
        case 6: return "Homing";
        case 7: return "Interpolated Position (IP)";
        case 8: return "Cyclic Sync Position (CSP)";
        case 9: return "Cyclic Sync Velocity (CSV)";
        case 10: return "Cyclic Sync Torque (CST)";
        default: return "Unknown";
    }
}

int main(int argc, char* argv[]) {
    std::cout << "EtherCAT CiA 402 Drive Example\n";
    std::cout << "==============================\n\n";
    
    Tether::Utils::SignalHandler sig_handler(g_running, false);
    
    // Create drive
    auto drive = std::make_unique<EtherCAT::Slave::CiA402Slave>(1);
    drive->setConfiguredAddress(0x1001);
    
    // Set drive parameters
    drive->setVendorId(0x00000001);
    drive->setProductCode(0x00402000);
    drive->setRevisionNumber(0x00010000);
    drive->setSerialNumber(0x00000001);
    
    // Configure SyncManagers for PDO
    // SM2: RxPDO (Controlword, Target Position, etc.)
    drive->configureSyncManager(2, 0x1100, 8, 
        EtherCAT::Slave::SyncManagerType::Output, true);
    
    // SM3: TxPDO (Statusword, Actual Position, etc.)
    drive->configureSyncManager(3, 0x1000, 8,
        EtherCAT::Slave::SyncManagerType::Input, true);
    
    // Configure FMMUs
    drive->configureFMMU(0, 0x1000, 8, 0x1100, 0x0, true, true);
    drive->configureFMMU(1, 0x1100, 8, 0x1000, 0x0, true, false);
    
    // Create physics simulator
    DriveSimulator sim;
    sim.setMaxVelocity(100000);      // counts/s
    sim.setMaxAcceleration(500000);  // counts/s²
    
    // Create HAL
    auto hal = std::make_unique<EtherCAT::Slave::DirectLoopbackHAL>(
        [&drive](const uint8_t* data, size_t len) {
            return drive->processFrame(data, len);
        }
    );
    
    std::cout << "Drive Configuration:\n";
    std::cout << "  Address:      0x1001\n";
    std::cout << "  Vendor ID:    0x00000001\n";
    std::cout << "  Product Code: 0x00402000\n";
    std::cout << "  Max Velocity: 100000 counts/s\n";
    std::cout << "  Max Accel:    500000 counts/s²\n\n";
    
    std::cout << "Press Ctrl+C to stop\n\n";
    
    // Simulation loop
    auto lastTime = std::chrono::steady_clock::now();
    const double cycleTime = 0.001;  // 1ms cycle
    
    // Demo: automatic state transitions and motion
    enum class DemoPhase {
        WaitForPreOp,
        EnableDrive,
        MoveToPosition1,
        WaitAtPosition1,
        MoveToPosition2,
        WaitAtPosition2,
        Complete
    };
    
    DemoPhase phase = DemoPhase::WaitForPreOp;
    auto phaseStartTime = std::chrono::steady_clock::now();
    int demoIteration = 0;
    
    while (g_running.load()) {
        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(now - lastTime);
        
        if (elapsed.count() >= 1000) {  // 1ms cycle
            lastTime = now;
            double dt = elapsed.count() / 1000000.0;
            
            // Update drive state machine
            drive->update();
            
            // Get drive status
            uint16_t statusWord = drive->getStatusWord();
            uint16_t controlWord = drive->getControlWord();
            int8_t opMode = drive->getModesOfOperationDisplay();
            int32_t targetPos = drive->getTargetPosition();
            
            // Update physics simulation based on drive state
            if ((statusWord & 0x6F) == 0x27) {  // Operation enabled
                if (opMode == 1) {  // Profile position mode
                    sim.setTargetPosition(targetPos);
                } else if (opMode == 8) {  // CSP mode
                    sim.setTargetPosition(targetPos);
                }
                sim.update(dt);
            } else {
                sim.halt();
            }
            
            // Update drive with simulated actual values
            drive->setActualPosition(sim.getActualPosition());
            drive->setActualVelocity(sim.getActualVelocity());
            
            // Demo state machine
            auto phaseDuration = std::chrono::duration_cast<std::chrono::seconds>(
                now - phaseStartTime).count();
            
            switch (phase) {
                case DemoPhase::WaitForPreOp:
                    // Simulate master requesting state change
                    drive->toState(EtherCAT::Slave::SlaveState::PreOp);
                    drive->toState(EtherCAT::Slave::SlaveState::SafeOp);
                    drive->toState(EtherCAT::Slave::SlaveState::Op);
                    phase = DemoPhase::EnableDrive;
                    phaseStartTime = now;
                    break;
                    
                case DemoPhase::EnableDrive:
                    // CiA 402 state machine: enable drive
                    drive->setControlWord(0x0006);  // Shutdown
                    if (phaseDuration >= 1) {
                        drive->setControlWord(0x0007);  // Switch on
                    }
                    if (phaseDuration >= 2) {
                        drive->setControlWord(0x000F);  // Enable operation
                    }
                    if (phaseDuration >= 3) {
                        drive->setModesOfOperation(1);  // Profile position
                        phase = DemoPhase::MoveToPosition1;
                        phaseStartTime = now;
                    }
                    break;
                    
                case DemoPhase::MoveToPosition1:
                    drive->setTargetPosition(100000);
                    drive->setControlWord(0x001F);  // New setpoint
                    if (sim.isPositionReached()) {
                        phase = DemoPhase::WaitAtPosition1;
                        phaseStartTime = now;
                    }
                    break;
                    
                case DemoPhase::WaitAtPosition1:
                    if (phaseDuration >= 2) {
                        phase = DemoPhase::MoveToPosition2;
                        phaseStartTime = now;
                    }
                    break;
                    
                case DemoPhase::MoveToPosition2:
                    drive->setTargetPosition(-100000);
                    drive->setControlWord(0x001F);
                    if (sim.isPositionReached()) {
                        phase = DemoPhase::WaitAtPosition2;
                        phaseStartTime = now;
                    }
                    break;
                    
                case DemoPhase::WaitAtPosition2:
                    if (phaseDuration >= 2) {
                        demoIteration++;
                        phase = DemoPhase::MoveToPosition1;
                        phaseStartTime = now;
                    }
                    break;
                    
                case DemoPhase::Complete:
                    break;
            }
            
            // Display status
            static int displayCounter = 0;
            if (++displayCounter >= 100) {  // Update display every 100ms
                displayCounter = 0;
                
                std::cout << "\r";
                std::cout << "State: " << std::setw(20) << std::left 
                          << driveStateToString(statusWord);
                std::cout << " | Mode: " << std::setw(25) << std::left
                          << operatingModeToString(opMode);
                std::cout << " | Pos: " << std::setw(8) << sim.getActualPosition();
                std::cout << " | Vel: " << std::setw(8) << sim.getActualVelocity();
                std::cout << " | Iter: " << demoIteration;
                std::cout << "     " << std::flush;
            }
        }
        
        std::this_thread::sleep_for(std::chrono::microseconds(100));
    }
    
    std::cout << "\n\nShutting down...\n";
    
    return 0;
}
