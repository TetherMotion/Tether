/**
 * @file CiA402SlavePDO.cpp
 * @brief CiA 402 Slave - PDO Mappings and Processing
 */

#include "CiA402SlaveCommon.hpp"

namespace EtherCAT {
namespace slave {

// ============================================================================
// PDO Mappings
// ============================================================================

void CiA402Slave::initPDOMappings() {
    // ========================================================================
    // RxPDO Mapping (Master -> Slave, SM2)
    // ========================================================================
    // Standard CSP/CSV/CST mapping:
    // - Control word (0x6040:0) - 16 bits
    // - Target position (0x607A:0) - 32 bits  
    // - Target velocity (0x60FF:0) - 32 bits
    // - Target torque (0x6071:0) - 16 bits
    // - Modes of operation (0x6060:0) - 8 bits
    // - Digital outputs (0x60FE:1) - 32 bits (optional, for 17-byte PDO)
    
    std::vector<uint32_t> rxPdoMapping = {
        PDOMapEntry(CiA402Index::ControlWord, 0, 16),        // 2 bytes
        PDOMapEntry(CiA402Index::TargetPosition, 0, 32),     // 4 bytes
        PDOMapEntry(CiA402Index::TargetVelocity, 0, 32),     // 4 bytes
        PDOMapEntry(CiA402Index::TargetTorque, 0, 16),       // 2 bytes
        PDOMapEntry(CiA402Index::ModesOfOperation, 0, 8),    // 1 byte
        PDOMapEntry(CiA402Index::DigitalOutputs, 1, 32),     // 4 bytes
    };
    registerPDOMapping(0x1600, rxPdoMapping);  // RxPDO mapping at 0x1600
    
    // ========================================================================
    // TxPDO Mapping (Slave -> Master, SM3)
    // ========================================================================
    // Standard CSP/CSV/CST mapping:
    // - Status word (0x6041:0) - 16 bits
    // - Position actual (0x6064:0) - 32 bits
    // - Velocity actual (0x606C:0) - 32 bits
    // - Torque actual (0x6077:0) - 16 bits  
    // - Modes of operation display (0x6061:0) - 8 bits
    // - Digital inputs (0x60FD:0) - 32 bits (optional, for 17-byte PDO)
    
    std::vector<uint32_t> txPdoMapping = {
        PDOMapEntry(CiA402Index::StatusWord, 0, 16),              // 2 bytes
        PDOMapEntry(CiA402Index::PositionActual, 0, 32),          // 4 bytes
        PDOMapEntry(CiA402Index::VelocityActual, 0, 32),          // 4 bytes
        PDOMapEntry(CiA402Index::TorqueActual, 0, 16),            // 2 bytes
        PDOMapEntry(CiA402Index::ModesOfOperationDisplay, 0, 8),  // 1 byte
        PDOMapEntry(CiA402Index::DigitalInputs, 0, 32),           // 4 bytes
    };
    registerPDOMapping(0x1A00, txPdoMapping);  // TxPDO mapping at 0x1A00
}

// ============================================================================
// TxPDO Update (Slave -> Master)
// ============================================================================

void CiA402Slave::updateTxPDO() {
    // Compute status word from current drive state
    statusWord_ = computeStatusWord();
    
    // Write status word (offset 0)
    auto* txData = getTxPDOPtr<uint8_t>(0);
    if (!txData) return;
    
    // Status word (2 bytes)
    std::memcpy(txData + pdoLayout_.statusWordOffset, &statusWord_, 2);
    
    // Position actual (4 bytes)
    std::memcpy(txData + pdoLayout_.actualPositionOffset, &actualPosition_, 4);
    
    // Velocity actual (4 bytes)
    std::memcpy(txData + pdoLayout_.actualVelocityOffset, &actualVelocity_, 4);
    
    // Torque actual (2 bytes)
    std::memcpy(txData + pdoLayout_.actualTorqueOffset, &actualTorque_, 2);
    
    // Modes of operation display (1 byte)
    txData[pdoLayout_.modeDisplayOffset] = static_cast<uint8_t>(operatingModeDisplay_);
    
    // Digital inputs (4 bytes)
    std::memcpy(txData + pdoLayout_.digitalInputsOffset, &digitalInputs_, 4);
}

// ============================================================================
// RxPDO Processing (Master -> Slave)
// ============================================================================

void CiA402Slave::processRxPDO() {
    const auto* rxData = getRxPDOPtr<uint8_t>(0);
    if (!rxData) return;
    
    // Control word (2 bytes)
    uint16_t cw;
    std::memcpy(&cw, rxData + pdoLayout_.controlWordOffset, 2);
    
    // Target position (4 bytes)
    int32_t targetPos;
    std::memcpy(&targetPos, rxData + pdoLayout_.targetPositionOffset, 4);
    targetPosition_ = std::clamp(targetPos, driveConfig_.softwarePosLimitMin,
                                 driveConfig_.softwarePosLimitMax);
    
    // Target velocity (4 bytes)
    int32_t targetVel;
    std::memcpy(&targetVel, rxData + pdoLayout_.targetVelocityOffset, 4);
    int32_t maxVel = static_cast<int32_t>(driveConfig_.maxMotorVelocity);
    targetVelocity_ = std::clamp(targetVel, -maxVel, maxVel);
    
    // Target torque (2 bytes)
    int16_t targetTrq;
    std::memcpy(&targetTrq, rxData + pdoLayout_.targetTorqueOffset, 2);
    targetTorque_ = std::clamp(targetTrq, static_cast<int16_t>(-static_cast<int16_t>(maxTorque_)), static_cast<int16_t>(maxTorque_));
    
    // Modes of operation (1 byte)
    int8_t mode = static_cast<int8_t>(rxData[pdoLayout_.modeOffset]);
    if (mode != 0 && isModeSupported(mode)) {
        operatingMode_ = mode;
    }
    
    // Digital outputs (4 bytes)
    uint32_t outputs;
    std::memcpy(&outputs, rxData + pdoLayout_.digitalOutputsOffset, 4);
    digitalOutputs_ = (digitalOutputs_ & ~digitalOutputMask_) | (outputs & digitalOutputMask_);
    
    // Process control word - this handles state machine transitions
    processControlWord(cw);
}

// ============================================================================
// EtherCAT State Change Handler
// ============================================================================

void CiA402Slave::onStateChange(SlaveState oldState, SlaveState newState) {
    ProfileSlave::onStateChange(oldState, newState);
    
    // Handle EtherCAT state transitions affecting drive state
    switch (newState) {
        case SlaveState::INIT:
            // Reset drive to initial state
            setDriveState(CiA402State::NotReadyToSwitchOn);
            controlWord_ = 0;
            targetPosition_ = actualPosition_;
            targetVelocity_ = 0;
            targetTorque_ = 0;
            break;
            
        case SlaveState::PRE_OP:
            // Drive should be in SwitchOnDisabled
            if (driveState_ == CiA402State::NotReadyToSwitchOn) {
                setDriveState(CiA402State::SwitchOnDisabled);
            }
            break;
            
        case SlaveState::SAFE_OP:
            // Prepare for operation
            // Reset targets to current position
            targetPosition_ = actualPosition_;
            positionDemand_ = actualPosition_;
            break;
            
        case SlaveState::OP:
            // Full operation enabled
            // Drive can now transition through state machine
            break;
            
        default:
            break;
    }
}

// ============================================================================
// DC Sync Handler
// ============================================================================

void CiA402Slave::onSync(int syncNum, uint64_t timestamp) {
    ProfileSlave::onSync(syncNum, timestamp);
    
    // SYNC0 is typically used for PDO exchange timing
    if (syncNum == 0) {
        // Calculate time delta
        uint64_t deltaNs = 0;
        if (lastSimTime_ > 0 && timestamp > lastSimTime_) {
            deltaNs = timestamp - lastSimTime_;
        } else {
            deltaNs = driveConfig_.defaultCycleTime;
        }
        lastSimTime_ = timestamp;
        
        // Run simulation
        simulate(deltaNs);
    }
    
    // SYNC1 can be used for additional timing if needed
}

}  // namespace slave
}  // namespace EtherCAT
