/**
 * @file CiA417Slave.cpp
 * @brief CiA 417 Lift Controller Slave Implementation
 */

#include "slave/profiles/CiA417Slave.hpp"
#include "slave/core/SlaveCore.hpp"
#include "slave/mailbox/IMailboxHandler.hpp"

#include <cstring>
#include <algorithm>
#include <cmath>

namespace EtherCAT {
namespace slave {

// ============================================================================
// CiA417Slave Implementation
// ============================================================================

CiA417Slave::CiA417Slave(const CiA417SlaveConfig& config)
    : ProfileSlave(CiAProfile::CiA417, SlaveConfig{
        .identity = config.identity,
        .supportsBootstrap = false
      })
    , liftConfig_(config)
{
    floorRequests_.reset();
}

CiA417Slave::~CiA417Slave() = default;

void CiA417Slave::initObjectDictionary() {
    ProfileSlave::registerCiA301Objects();
    // Minimal implementation - object dictionary registration would go here
}

void CiA417Slave::initPDOMappings() {
    // Minimal implementation - PDO mappings would go here
}

void CiA417Slave::updateTxPDO() {
    // Minimal implementation - transmit PDO update would go here
}

void CiA417Slave::processRxPDO() {
    // Minimal implementation - receive PDO processing would go here
}

void CiA417Slave::simulate(uint64_t deltaNs) {
    if (positionCallback_) {
        positionCallback_(position_, speed_);
    }
    updateStateMachine(deltaNs);
}

void CiA417Slave::setTargetFloor(uint8_t floor) {
    if (floor < liftConfig_.numberOfFloors) {
        targetFloor_ = floor;
        if (liftState_ == LiftState::Idle || liftState_ == LiftState::DoorClosed) {
            startMovement(floor);
        }
    }
}

void CiA417Slave::addFloorRequest(uint8_t floor) {
    if (floor < MAX_FLOORS && floor < liftConfig_.numberOfFloors) {
        floorRequests_.set(floor);
    }
}

void CiA417Slave::clearFloorRequest(uint8_t floor) {
    if (floor < MAX_FLOORS) {
        floorRequests_.reset(floor);
    }
}

bool CiA417Slave::hasFloorRequest(uint8_t floor) const {
    if (floor < MAX_FLOORS) {
        return floorRequests_.test(floor);
    }
    return false;
}

void CiA417Slave::setPosition(int32_t pos) {
    position_ = pos;
    currentFloor_ = getFloorAtPosition(pos);
}

DoorState CiA417Slave::getDoorState(uint8_t door) const {
    if (door < 2) {
        return doorStates_[door];
    }
    return DoorState::Fault;
}

void CiA417Slave::openDoor(uint8_t door) {
    if (door < liftConfig_.numberOfDoors && !isMoving()) {
        doorStates_[door] = DoorState::Opening;
    }
}

void CiA417Slave::closeDoor(uint8_t door) {
    if (door < liftConfig_.numberOfDoors) {
        if (!doorBlocked_[door]) {
            doorStates_[door] = DoorState::Closing;
        }
    }
}

void CiA417Slave::setDoorBlocked(uint8_t door, bool blocked) {
    if (door < 2) {
        doorBlocked_[door] = blocked;
        if (blocked && doorStates_[door] == DoorState::Closing) {
            doorStates_[door] = DoorState::Blocked;
        }
    }
}

void CiA417Slave::setLoad(uint16_t load) {
    load_ = load;
}

void CiA417Slave::setSafetyCircuit(bool ok) {
    safetyCircuitOk_ = ok;
    if (!ok) {
        liftState_ = LiftState::Emergency;
        speed_ = 0;
    }
}

void CiA417Slave::setEmergencyStop(bool stop) {
    emergencyStop_ = stop;
    if (stop) {
        liftState_ = LiftState::Emergency;
        speed_ = 0;
    }
}

bool CiA417Slave::isAtFloor() const {
    int32_t floorPos = getFloorPosition(currentFloor_);
    return std::abs(position_ - floorPos) <= liftConfig_.levelingTolerance;
}

void CiA417Slave::updateStateMachine(uint64_t deltaNs) {
    if (emergencyStop_ || !safetyCircuitOk_) {
        liftState_ = LiftState::Emergency;
        speed_ = 0;
        return;
    }
    
    float dt = deltaNs / 1e9f;
    
    switch (liftState_) {
        case LiftState::Idle:
            selectNextTarget();
            break;
            
        case LiftState::MovingUp:
        case LiftState::MovingDown: {
            int32_t targetPos = getFloorPosition(targetFloor_);
            int32_t distance = targetPos - position_;
            
            if (std::abs(distance) <= liftConfig_.levelingTolerance) {
                liftState_ = LiftState::Leveling;
                speed_ = 0;
            } else {
                position_ += static_cast<int32_t>(speed_ * dt);
            }
            currentFloor_ = getFloorAtPosition(position_);
            break;
        }
        
        case LiftState::Leveling:
            currentFloor_ = targetFloor_;
            clearFloorRequest(currentFloor_);
            liftState_ = LiftState::DoorOpening;
            for (uint8_t d = 0; d < std::min(liftConfig_.numberOfDoors, static_cast<uint8_t>(2)); ++d) {
                doorStates_[d] = DoorState::Opening;
            }
            break;
            
        case LiftState::DoorOpening:
            // Simulate door opening
            for (uint8_t d = 0; d < std::min(liftConfig_.numberOfDoors, static_cast<uint8_t>(2)); ++d) {
                doorStates_[d] = DoorState::Open;
            }
            liftState_ = LiftState::DoorOpen;
            break;
            
        case LiftState::DoorOpen:
            // Wait for close command
            break;
            
        case LiftState::DoorClosing:
            for (uint8_t d = 0; d < std::min(liftConfig_.numberOfDoors, static_cast<uint8_t>(2)); ++d) {
                if (!doorBlocked_[d]) {
                    doorStates_[d] = DoorState::Closed;
                }
            }
            liftState_ = LiftState::DoorClosed;
            break;
            
        case LiftState::DoorClosed:
            selectNextTarget();
            break;
            
        case LiftState::Emergency:
        case LiftState::Maintenance:
        case LiftState::OutOfService:
        case LiftState::Stopping:
            speed_ = 0;
            break;
    }
}

void CiA417Slave::startMovement(uint8_t targetFloor) {
    if (targetFloor >= liftConfig_.numberOfFloors) {
        return;
    }
    
    targetFloor_ = targetFloor;
    int32_t targetPos = getFloorPosition(targetFloor);
    
    if (targetPos > position_) {
        liftState_ = LiftState::MovingUp;
        speed_ = liftConfig_.nominalSpeed;
    } else if (targetPos < position_) {
        liftState_ = LiftState::MovingDown;
        speed_ = -liftConfig_.nominalSpeed;
    }
}

void CiA417Slave::stopMovement() {
    liftState_ = LiftState::Stopping;
    speed_ = 0;
}

int32_t CiA417Slave::getFloorPosition(uint8_t floor) const {
    return floor * liftConfig_.floorHeight;
}

uint8_t CiA417Slave::getFloorAtPosition(int32_t pos) const {
    if (pos < 0) return 0;
    uint8_t floor = static_cast<uint8_t>(pos / liftConfig_.floorHeight);
    if (floor >= liftConfig_.numberOfFloors) {
        floor = liftConfig_.numberOfFloors - 1;
    }
    return floor;
}

void CiA417Slave::selectNextTarget() {
    if (floorRequests_.none()) {
        return;
    }
    
    // Find nearest requested floor
    uint8_t nearest = currentFloor_;
    int32_t minDistance = INT32_MAX;
    
    for (uint8_t f = 0; f < liftConfig_.numberOfFloors && f < MAX_FLOORS; ++f) {
        if (floorRequests_.test(f)) {
            int32_t dist = std::abs(static_cast<int32_t>(f) - static_cast<int32_t>(currentFloor_));
            if (dist < minDistance) {
                minDistance = dist;
                nearest = f;
            }
        }
    }
    
    if (nearest != currentFloor_) {
        startMovement(nearest);
    }
}

void CiA417Slave::processControlWord(uint16_t controlWord) {
    (void)controlWord;
    // Minimal implementation
}

uint16_t CiA417Slave::buildStatusWord() const {
    uint16_t status = 0;
    status |= (static_cast<uint8_t>(liftState_) & 0x1F);
    if (safetyCircuitOk_) status |= 0x0100;
    if (emergencyStop_) status |= 0x0200;
    if (isOverloaded()) status |= 0x0400;
    return status;
}

std::unique_ptr<CiA417Slave> createCiA417Slave(const CiA417SlaveConfig& config) {
    return std::make_unique<CiA417Slave>(config);
}

std::unique_ptr<CiA417Slave> createLiftController(uint8_t floors) {
    CiA417SlaveConfig config;
    config.numberOfFloors = floors;
    return std::make_unique<CiA417Slave>(config);
}

}  // namespace slave
}  // namespace EtherCAT
