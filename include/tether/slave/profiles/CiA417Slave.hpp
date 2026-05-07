/**
 * @file CiA417Slave.hpp
 * @brief CiA 417 Lift Controller Slave Implementation
 *
 * @details
 * Implements a CiA 417 compliant lift controller slave with:
 * - Floor request handling
 * - Door control
 * - Position control
 * - Safety monitoring
 * - Load measurement
 *
 * ## Application
 *
 * This profile is designed for elevator/lift control applications
 * where precise floor positioning and safety are critical.
 */

#pragma once

#include "slave/profiles/ProfileSlave.hpp"

#include <array>
#include <functional>
#include <bitset>

namespace EtherCAT {
namespace slave {

// ============================================================================
// Lift State
// ============================================================================

enum class LiftState : uint8_t {
    Idle             = 0x00,
    MovingUp         = 0x01,
    MovingDown       = 0x02,
    Stopping         = 0x03,
    Leveling         = 0x04,
    DoorOpening      = 0x05,
    DoorOpen         = 0x06,
    DoorClosing      = 0x07,
    DoorClosed       = 0x08,
    Emergency        = 0x0F,
    Maintenance      = 0x10,
    OutOfService     = 0x11,
};

enum class DoorState : uint8_t {
    Closed           = 0x00,
    Opening          = 0x01,
    Open             = 0x02,
    Closing          = 0x03,
    Blocked          = 0x04,
    Fault            = 0x0F,
};

// ============================================================================
// CiA 417 Slave Configuration
// ============================================================================

struct CiA417SlaveConfig {
    SlaveIdentity identity = {
        .vendorId = 0x00000000,
        .productCode = 0x000001A1,
        .revisionNumber = 0x00010000,
        .serialNumber = 0x00000001,
        .deviceName = "CiA 417 Lift Controller",
    };
    
    // Lift configuration
    uint8_t numberOfFloors = 10;
    uint8_t numberOfDoors = 1;          // 1 = front only, 2 = front and rear
    
    // Position parameters (in mm)
    int32_t floorHeight = 3000;         // Default floor height: 3m
    int32_t levelingTolerance = 5;      // 5mm leveling tolerance
    
    // Speed parameters (in mm/s)
    int32_t nominalSpeed = 1000;        // 1 m/s
    int32_t levelingSpeed = 50;         // 50 mm/s leveling speed
    
    // Load
    uint16_t maxLoad = 1000;            // 1000 kg max load
    bool hasLoadSensor = true;
    
    // Safety
    bool hasSafetyCircuit = true;
    bool hasOverspeedGovernor = true;
    
    bool supportsDC = true;
    uint32_t defaultCycleTime = 1000000;
    
    SlaveLogConfig logConfig;
};

// ============================================================================
// CiA 417 Slave Class
// ============================================================================

class CiA417Slave : public ProfileSlave {
public:
    static constexpr uint8_t MAX_FLOORS = 64;
    
    explicit CiA417Slave(const CiA417SlaveConfig& config);
    ~CiA417Slave() override;
    
    const char* getProfileName() const override { return "CiA 417"; }
    uint32_t getDeviceType() const override { return 0x000001A1; }
    
    void updateTxPDO() override;
    void processRxPDO() override;
    void simulate(uint64_t deltaNs) override;
    
    // Floor control
    uint8_t getCurrentFloor() const { return currentFloor_; }
    uint8_t getTargetFloor() const { return targetFloor_; }
    void setTargetFloor(uint8_t floor);
    void addFloorRequest(uint8_t floor);
    void clearFloorRequest(uint8_t floor);
    bool hasFloorRequest(uint8_t floor) const;
    uint64_t getFloorRequests() const { return floorRequests_.to_ullong(); }
    
    // Position (in mm from ground floor)
    int32_t getPosition() const { return position_; }
    void setPosition(int32_t pos);  // For simulation
    
    // State
    LiftState getLiftState() const { return liftState_; }
    DoorState getDoorState(uint8_t door = 0) const;
    
    // Door control
    void openDoor(uint8_t door = 0);
    void closeDoor(uint8_t door = 0);
    void setDoorBlocked(uint8_t door, bool blocked);
    
    // Load (in 0.1 kg units)
    uint16_t getLoad() const { return load_; }
    void setLoad(uint16_t load);  // For simulation
    bool isOverloaded() const { return load_ > liftConfig_.maxLoad * 10; }
    
    // Safety
    bool isSafetyCircuitOk() const { return safetyCircuitOk_; }
    void setSafetyCircuit(bool ok);  // For simulation
    bool isEmergencyStop() const { return emergencyStop_; }
    void setEmergencyStop(bool stop);
    
    // Direction
    bool isMovingUp() const { return liftState_ == LiftState::MovingUp; }
    bool isMovingDown() const { return liftState_ == LiftState::MovingDown; }
    bool isMoving() const { return isMovingUp() || isMovingDown(); }
    bool isAtFloor() const;
    
    // Speed (in mm/s)
    int32_t getSpeed() const { return speed_; }
    
    // Callbacks for simulation
    using PositionCallback = std::function<void(int32_t& position, int32_t& speed)>;
    void setPositionCallback(PositionCallback callback) { positionCallback_ = callback; }
    
protected:
    void initObjectDictionary() override;
    void initPDOMappings() override;
    
private:
    CiA417SlaveConfig liftConfig_;
    
    // State
    LiftState liftState_ = LiftState::Idle;
    std::array<DoorState, 2> doorStates_ = { DoorState::Closed, DoorState::Closed };
    
    // Floor tracking
    uint8_t currentFloor_ = 0;
    uint8_t targetFloor_ = 0;
    std::bitset<MAX_FLOORS> floorRequests_;
    
    // Position and motion
    int32_t position_ = 0;              // Current position (mm)
    int32_t speed_ = 0;                 // Current speed (mm/s)
    
    // Load
    uint16_t load_ = 0;                 // Current load (0.1 kg units)
    
    // Safety
    bool safetyCircuitOk_ = true;
    bool emergencyStop_ = false;
    bool doorBlocked_[2] = { false, false };
    
    PositionCallback positionCallback_;
    
    // State machine helpers
    void updateStateMachine(uint64_t deltaNs);
    void startMovement(uint8_t targetFloor);
    void stopMovement();
    int32_t getFloorPosition(uint8_t floor) const;
    uint8_t getFloorAtPosition(int32_t pos) const;
    void selectNextTarget();
    void processControlWord(uint16_t controlWord);
    uint16_t buildStatusWord() const;
};

std::unique_ptr<CiA417Slave> createCiA417Slave(const CiA417SlaveConfig& config);

// Factory with floor count
std::unique_ptr<CiA417Slave> createLiftController(uint8_t floors);

}  // namespace slave
}  // namespace EtherCAT
