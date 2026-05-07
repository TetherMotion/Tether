/**
 * @file CiA402SlaveOD.cpp
 * @brief CiA 402 Slave - Object Dictionary Registration
 *
 * This file contains CiA 402 object dictionary registrations.
 * Note: While larger than other split files, this is kept as a single unit
 * because object registrations are declarative and splitting them across
 * files would require exposing internal OD types.
 */

#include "CiA402SlaveCommon.hpp"

namespace EtherCAT {
namespace slave {

// ============================================================================
// Object Dictionary Registration
// ============================================================================

void CiA402Slave::initObjectDictionary() {
    // Register standard CiA 301 communication objects
    ProfileSlave::registerCiA301Objects();
    
    auto& od = getObjectDictionary();
    
    // ========================================================================
    // Control/Status Objects
    // ========================================================================
    
    // 0x6040 - Control Word (UNSIGNED16, RW)
    od.registerObject(
        ODEntryInfo{
            .index = CiA402Index::ControlWord,
            .subindex = 0,
            .dataType = ObjectDictionaryDataType::Unsigned16,
            .bitLength = 16,
            .accessType = 0x3F,  // RW
            .name = "Control word",
            .defaultValue = 0
        },
        [this](uint8_t* data, size_t& len) -> SDOAbortCode {
            if (len < 2) return SDOAbortCode::DataTypeMismatch;
            std::memcpy(data, &controlWord_, 2);
            len = 2;
            return SDOAbortCode::Success;
        },
        [this](const uint8_t* data, size_t len) -> SDOAbortCode {
            if (len != 2) return SDOAbortCode::DataTypeMismatch;
            uint16_t cw;
            std::memcpy(&cw, data, 2);
            processControlWord(cw);
            return SDOAbortCode::Success;
        }
    );
    
    // 0x6041 - Status Word (UNSIGNED16, RO)
    od.registerObject(
        ODEntryInfo{
            .index = CiA402Index::StatusWord,
            .subindex = 0,
            .dataType = ObjectDictionaryDataType::Unsigned16,
            .bitLength = 16,
            .accessType = 0x3F,  // RO
            .name = "Status word",
            .defaultValue = 0
        },
        [this](uint8_t* data, size_t& len) -> SDOAbortCode {
            if (len < 2) return SDOAbortCode::DataTypeMismatch;
            uint16_t sw = computeStatusWord();
            std::memcpy(data, &sw, 2);
            len = 2;
            return SDOAbortCode::Success;
        },
        nullptr  // Read-only
    );
    
    // 0x603F - Error Code (UNSIGNED16, RO)
    od.registerObject(
        ODEntryInfo{
            .index = CiA402Index::ErrorCode,
            .subindex = 0,
            .dataType = ObjectDictionaryDataType::Unsigned16,
            .bitLength = 16,
            .accessType = 0x3F,
            .name = "Error code",
            .defaultValue = 0
        },
        [this](uint8_t* data, size_t& len) -> SDOAbortCode {
            if (len < 2) return SDOAbortCode::DataTypeMismatch;
            std::memcpy(data, &errorCode_, 2);
            len = 2;
            return SDOAbortCode::Success;
        },
        nullptr
    );
    
    // ========================================================================
    // Operating Mode Objects
    // ========================================================================
    
    // 0x6060 - Modes of Operation (INTEGER8, RW)
    od.registerObject(
        ODEntryInfo{
            .index = CiA402Index::ModesOfOperation,
            .subindex = 0,
            .dataType = ObjectDictionaryDataType::Integer8,
            .bitLength = 8,
            .accessType = 0x3F,
            .name = "Modes of operation",
            .defaultValue = 0
        },
        [this](uint8_t* data, size_t& len) -> SDOAbortCode {
            if (len < 1) return SDOAbortCode::DataTypeMismatch;
            data[0] = static_cast<uint8_t>(operatingMode_);
            len = 1;
            return SDOAbortCode::Success;
        },
        [this](const uint8_t* data, size_t len) -> SDOAbortCode {
            if (len != 1) return SDOAbortCode::DataTypeMismatch;
            int8_t mode = static_cast<int8_t>(data[0]);
            if (mode != 0 && !isModeSupported(mode)) {
                return SDOAbortCode::InvalidValue;
            }
            operatingMode_ = mode;
            // Mode takes effect only in Operation Enabled
            if (driveState_ != CiA402State::OperationEnabled) {
                operatingModeDisplay_ = mode;
            }
            return SDOAbortCode::Success;
        }
    );
    
    // 0x6061 - Modes of Operation Display (INTEGER8, RO)
    od.registerObject(
        ODEntryInfo{
            .index = CiA402Index::ModesOfOperationDisplay,
            .subindex = 0,
            .dataType = ObjectDictionaryDataType::Integer8,
            .bitLength = 8,
            .accessType = 0x3F,
            .name = "Modes of operation display",
            .defaultValue = 0
        },
        [this](uint8_t* data, size_t& len) -> SDOAbortCode {
            if (len < 1) return SDOAbortCode::DataTypeMismatch;
            data[0] = static_cast<uint8_t>(operatingModeDisplay_);
            len = 1;
            return SDOAbortCode::Success;
        },
        nullptr
    );
    
    // ========================================================================
    // Position Objects
    // ========================================================================
    
    // 0x6062 - Position Demand Value (INTEGER32, RO)
    od.registerObject(
        ODEntryInfo{
            .index = CiA402Index::PositionDemand,
            .subindex = 0,
            .dataType = ObjectDictionaryDataType::Integer32,
            .bitLength = 32,
            .accessType = 0x3F,
            .name = "Position demand value",
            .defaultValue = 0
        },
        [this](uint8_t* data, size_t& len) -> SDOAbortCode {
            if (len < 4) return SDOAbortCode::DataTypeMismatch;
            std::memcpy(data, &positionDemand_, 4);
            len = 4;
            return SDOAbortCode::Success;
        },
        nullptr
    );
    
    // 0x6064 - Position Actual Value (INTEGER32, RO)
    od.registerObject(
        ODEntryInfo{
            .index = CiA402Index::PositionActual,
            .subindex = 0,
            .dataType = ObjectDictionaryDataType::Integer32,
            .bitLength = 32,
            .accessType = 0x3F,
            .name = "Position actual value",
            .defaultValue = 0
        },
        [this](uint8_t* data, size_t& len) -> SDOAbortCode {
            if (len < 4) return SDOAbortCode::DataTypeMismatch;
            std::memcpy(data, &actualPosition_, 4);
            len = 4;
            return SDOAbortCode::Success;
        },
        nullptr
    );
    
    // 0x6065 - Following Error Window (UNSIGNED32, RW)
    od.registerObject(
        ODEntryInfo{
            .index = CiA402Index::FollowingErrorWindow,
            .subindex = 0,
            .dataType = ObjectDictionaryDataType::Unsigned32,
            .bitLength = 32,
            .accessType = 0x3F,
            .name = "Following error window",
            .defaultValue = driveConfig_.followingErrorWindow
        }
    );
    
    // 0x6067 - Position Window (UNSIGNED32, RW)
    od.registerObject(
        ODEntryInfo{
            .index = CiA402Index::PositionWindow,
            .subindex = 0,
            .dataType = ObjectDictionaryDataType::Unsigned32,
            .bitLength = 32,
            .accessType = 0x3F,
            .name = "Position window",
            .defaultValue = driveConfig_.positionWindow
        }
    );
    
    // 0x607A - Target Position (INTEGER32, RW)
    od.registerObject(
        ODEntryInfo{
            .index = CiA402Index::TargetPosition,
            .subindex = 0,
            .dataType = ObjectDictionaryDataType::Integer32,
            .bitLength = 32,
            .accessType = 0x3F,
            .name = "Target position",
            .defaultValue = 0
        },
        [this](uint8_t* data, size_t& len) -> SDOAbortCode {
            if (len < 4) return SDOAbortCode::DataTypeMismatch;
            std::memcpy(data, &targetPosition_, 4);
            len = 4;
            return SDOAbortCode::Success;
        },
        [this](const uint8_t* data, size_t len) -> SDOAbortCode {
            if (len != 4) return SDOAbortCode::DataTypeMismatch;
            int32_t pos;
            std::memcpy(&pos, data, 4);
            // Apply software limits
            pos = std::clamp(pos, driveConfig_.softwarePosLimitMin, 
                            driveConfig_.softwarePosLimitMax);
            targetPosition_ = pos;
            return SDOAbortCode::Success;
        }
    );
    
    // 0x607C - Home Offset (INTEGER32, RW)
    od.registerObject(
        ODEntryInfo{
            .index = CiA402Index::HomeOffset,
            .subindex = 0,
            .dataType = ObjectDictionaryDataType::Integer32,
            .bitLength = 32,
            .accessType = 0x3F,
            .name = "Home offset",
            .defaultValue = static_cast<uint32_t>(driveConfig_.homingOffset)
        }
    );
    
    // 0x607D - Software Position Limit (ARRAY of INTEGER32, RW)
    od.registerObject(
        ODEntryInfo{
            .index = CiA402Index::SoftwarePosLimit,
            .subindex = 0,
            .dataType = ObjectDictionaryDataType::Unsigned8,
            .bitLength = 8,
            .accessType = 0x3F,
            .name = "Software position limit - Number of entries",
            .defaultValue = 2
        }
    );
    od.registerObject(
        ODEntryInfo{
            .index = CiA402Index::SoftwarePosLimit,
            .subindex = 1,
            .dataType = ObjectDictionaryDataType::Integer32,
            .bitLength = 32,
            .accessType = 0x3F,
            .name = "Min position limit",
            .defaultValue = static_cast<uint32_t>(driveConfig_.softwarePosLimitMin)
        }
    );
    od.registerObject(
        ODEntryInfo{
            .index = CiA402Index::SoftwarePosLimit,
            .subindex = 2,
            .dataType = ObjectDictionaryDataType::Integer32,
            .bitLength = 32,
            .accessType = 0x3F,
            .name = "Max position limit",
            .defaultValue = static_cast<uint32_t>(driveConfig_.softwarePosLimitMax)
        }
    );
    
    // 0x607F - Max Profile Velocity (UNSIGNED32, RW)
    od.registerObject(
        ODEntryInfo{
            .index = CiA402Index::MaxProfileVelocity,
            .subindex = 0,
            .dataType = ObjectDictionaryDataType::Unsigned32,
            .bitLength = 32,
            .accessType = 0x3F,
            .name = "Max profile velocity",
            .defaultValue = driveConfig_.maxProfileVelocity
        }
    );
    
    // 0x6080 - Max Motor Velocity (UNSIGNED32, RW)
    od.registerObject(
        ODEntryInfo{
            .index = CiA402Index::MaxMotorVelocity,
            .subindex = 0,
            .dataType = ObjectDictionaryDataType::Unsigned32,
            .bitLength = 32,
            .accessType = 0x3F,
            .name = "Max motor velocity",
            .defaultValue = driveConfig_.maxMotorVelocity
        }
    );
    
    // 0x6081 - Profile Velocity (UNSIGNED32, RW)
    od.registerObject(
        ODEntryInfo{
            .index = CiA402Index::ProfileVelocity,
            .subindex = 0,
            .dataType = ObjectDictionaryDataType::Unsigned32,
            .bitLength = 32,
            .accessType = 0x3F,
            .name = "Profile velocity",
            .defaultValue = driveConfig_.maxProfileVelocity / 2
        },
        [this](uint8_t* data, size_t& len) -> SDOAbortCode {
            if (len < 4) return SDOAbortCode::DataTypeMismatch;
            std::memcpy(data, &profileVelocity_, 4);
            len = 4;
            return SDOAbortCode::Success;
        },
        [this](const uint8_t* data, size_t len) -> SDOAbortCode {
            if (len != 4) return SDOAbortCode::DataTypeMismatch;
            uint32_t vel;
            std::memcpy(&vel, data, 4);
            profileVelocity_ = std::min(vel, driveConfig_.maxProfileVelocity);
            return SDOAbortCode::Success;
        }
    );
    
    // 0x6083 - Profile Acceleration (UNSIGNED32, RW)
    od.registerObject(
        ODEntryInfo{
            .index = CiA402Index::ProfileAcceleration,
            .subindex = 0,
            .dataType = ObjectDictionaryDataType::Unsigned32,
            .bitLength = 32,
            .accessType = 0x3F,
            .name = "Profile acceleration",
            .defaultValue = driveConfig_.maxAcceleration / 2
        },
        [this](uint8_t* data, size_t& len) -> SDOAbortCode {
            if (len < 4) return SDOAbortCode::DataTypeMismatch;
            std::memcpy(data, &profileAcceleration_, 4);
            len = 4;
            return SDOAbortCode::Success;
        },
        [this](const uint8_t* data, size_t len) -> SDOAbortCode {
            if (len != 4) return SDOAbortCode::DataTypeMismatch;
            uint32_t acc;
            std::memcpy(&acc, data, 4);
            profileAcceleration_ = std::min(acc, driveConfig_.maxAcceleration);
            return SDOAbortCode::Success;
        }
    );
    
    // 0x6084 - Profile Deceleration (UNSIGNED32, RW)
    od.registerObject(
        ODEntryInfo{
            .index = CiA402Index::ProfileDeceleration,
            .subindex = 0,
            .dataType = ObjectDictionaryDataType::Unsigned32,
            .bitLength = 32,
            .accessType = 0x3F,
            .name = "Profile deceleration",
            .defaultValue = driveConfig_.maxDeceleration / 2
        },
        [this](uint8_t* data, size_t& len) -> SDOAbortCode {
            if (len < 4) return SDOAbortCode::DataTypeMismatch;
            std::memcpy(data, &profileDeceleration_, 4);
            len = 4;
            return SDOAbortCode::Success;
        },
        [this](const uint8_t* data, size_t len) -> SDOAbortCode {
            if (len != 4) return SDOAbortCode::DataTypeMismatch;
            uint32_t dec;
            std::memcpy(&dec, data, 4);
            profileDeceleration_ = std::min(dec, driveConfig_.maxDeceleration);
            return SDOAbortCode::Success;
        }
    );
    
    // ========================================================================
    // Velocity Objects
    // ========================================================================
    
    // 0x606B - Velocity Demand Value (INTEGER32, RO)
    od.registerObject(
        ODEntryInfo{
            .index = CiA402Index::VelocityDemand,
            .subindex = 0,
            .dataType = ObjectDictionaryDataType::Integer32,
            .bitLength = 32,
            .accessType = 0x3F,
            .name = "Velocity demand value",
            .defaultValue = 0
        },
        [this](uint8_t* data, size_t& len) -> SDOAbortCode {
            if (len < 4) return SDOAbortCode::DataTypeMismatch;
            std::memcpy(data, &velocityDemand_, 4);
            len = 4;
            return SDOAbortCode::Success;
        },
        nullptr
    );
    
    // 0x606C - Velocity Actual Value (INTEGER32, RO)
    od.registerObject(
        ODEntryInfo{
            .index = CiA402Index::VelocityActual,
            .subindex = 0,
            .dataType = ObjectDictionaryDataType::Integer32,
            .bitLength = 32,
            .accessType = 0x3F,
            .name = "Velocity actual value",
            .defaultValue = 0
        },
        [this](uint8_t* data, size_t& len) -> SDOAbortCode {
            if (len < 4) return SDOAbortCode::DataTypeMismatch;
            std::memcpy(data, &actualVelocity_, 4);
            len = 4;
            return SDOAbortCode::Success;
        },
        nullptr
    );
    
    // 0x60FF - Target Velocity (INTEGER32, RW)
    od.registerObject(
        ODEntryInfo{
            .index = CiA402Index::TargetVelocity,
            .subindex = 0,
            .dataType = ObjectDictionaryDataType::Integer32,
            .bitLength = 32,
            .accessType = 0x3F,
            .name = "Target velocity",
            .defaultValue = 0
        },
        [this](uint8_t* data, size_t& len) -> SDOAbortCode {
            if (len < 4) return SDOAbortCode::DataTypeMismatch;
            std::memcpy(data, &targetVelocity_, 4);
            len = 4;
            return SDOAbortCode::Success;
        },
        [this](const uint8_t* data, size_t len) -> SDOAbortCode {
            if (len != 4) return SDOAbortCode::DataTypeMismatch;
            int32_t vel;
            std::memcpy(&vel, data, 4);
            // Clamp to max motor velocity
            int32_t maxVel = static_cast<int32_t>(driveConfig_.maxMotorVelocity);
            targetVelocity_ = std::clamp(vel, -maxVel, maxVel);
            return SDOAbortCode::Success;
        }
    );
    
    // ========================================================================
    // Torque Objects (0x6071-0x6077)
    // ========================================================================
    
    // 0x6071 - Target Torque (INTEGER16, RW)
    od.registerObject(
        ODEntryInfo{
            .index = CiA402Index::TargetTorque,
            .subindex = 0,
            .dataType = ObjectDictionaryDataType::Integer16,
            .bitLength = 16,
            .accessType = 0x3F,
            .name = "Target torque",
            .defaultValue = 0
        },
        [this](uint8_t* data, size_t& len) -> SDOAbortCode {
            if (len < 2) return SDOAbortCode::DataTypeMismatch;
            std::memcpy(data, &targetTorque_, 2);
            len = 2;
            return SDOAbortCode::Success;
        },
        [this](const uint8_t* data, size_t len) -> SDOAbortCode {
            if (len != 2) return SDOAbortCode::DataTypeMismatch;
            int16_t torque;
            std::memcpy(&torque, data, 2);
            targetTorque_ = std::clamp(torque, 
                static_cast<int16_t>(-static_cast<int16_t>(maxTorque_)), 
                static_cast<int16_t>(maxTorque_));
            return SDOAbortCode::Success;
        }
    );
    
    // 0x6072 - Max Torque (UNSIGNED16, RW)
    od.registerObject(
        ODEntryInfo{
            .index = CiA402Index::MaxTorque,
            .subindex = 0,
            .dataType = ObjectDictionaryDataType::Unsigned16,
            .bitLength = 16,
            .accessType = 0x3F,
            .name = "Max torque",
            .defaultValue = static_cast<uint32_t>(driveConfig_.maxTorque)
        }
    );
    
    // 0x6074 - Torque Demand Value (INTEGER16, RO)
    od.registerObject(
        ODEntryInfo{
            .index = CiA402Index::TorqueDemand,
            .subindex = 0,
            .dataType = ObjectDictionaryDataType::Integer16,
            .bitLength = 16,
            .accessType = 0x3F,
            .name = "Torque demand value",
            .defaultValue = 0
        },
        [this](uint8_t* data, size_t& len) -> SDOAbortCode {
            if (len < 2) return SDOAbortCode::DataTypeMismatch;
            std::memcpy(data, &torqueDemand_, 2);
            len = 2;
            return SDOAbortCode::Success;
        },
        nullptr
    );
    
    // 0x6077 - Torque Actual Value (INTEGER16, RO)
    od.registerObject(
        ODEntryInfo{
            .index = CiA402Index::TorqueActual,
            .subindex = 0,
            .dataType = ObjectDictionaryDataType::Integer16,
            .bitLength = 16,
            .accessType = 0x3F,
            .name = "Torque actual value",
            .defaultValue = 0
        },
        [this](uint8_t* data, size_t& len) -> SDOAbortCode {
            if (len < 2) return SDOAbortCode::DataTypeMismatch;
            std::memcpy(data, &actualTorque_, 2);
            len = 2;
            return SDOAbortCode::Success;
        },
        nullptr
    );
    
    // 0x6076 - Motor Rated Torque (UNSIGNED32, RO)
    od.registerObject(
        ODEntryInfo{
            .index = CiA402Index::MotorRatedTorque,
            .subindex = 0,
            .dataType = ObjectDictionaryDataType::Unsigned32,
            .bitLength = 32,
            .accessType = 0x3F,
            .name = "Motor rated torque",
            .defaultValue = static_cast<uint32_t>(driveConfig_.motorRatedTorque)
        }
    );
    
    // ========================================================================
    // Homing Objects (0x6098-0x609A)
    // ========================================================================
    
    // 0x6098 - Homing Method (INTEGER8, RW)
    od.registerObject(
        ODEntryInfo{
            .index = CiA402Index::HomingMethod,
            .subindex = 0,
            .dataType = ObjectDictionaryDataType::Integer8,
            .bitLength = 8,
            .accessType = 0x3F,
            .name = "Homing method",
            .defaultValue = static_cast<uint32_t>(driveConfig_.homingMethod)
        },
        [this](uint8_t* data, size_t& len) -> SDOAbortCode {
            if (len < 1) return SDOAbortCode::DataTypeMismatch;
            data[0] = static_cast<uint8_t>(homingMethod_);
            len = 1;
            return SDOAbortCode::Success;
        },
        [this](const uint8_t* data, size_t len) -> SDOAbortCode {
            if (len != 1) return SDOAbortCode::DataTypeMismatch;
            homingMethod_ = static_cast<int8_t>(data[0]);
            return SDOAbortCode::Success;
        }
    );
    
    // 0x6099 - Homing Speeds (ARRAY of UNSIGNED32, RW)
    od.registerObject(
        ODEntryInfo{
            .index = CiA402Index::HomingSpeeds,
            .subindex = 0,
            .dataType = ObjectDictionaryDataType::Unsigned8,
            .bitLength = 8,
            .accessType = 0x3F,
            .name = "Homing speeds - Number of entries",
            .defaultValue = 2
        }
    );
    od.registerObject(
        ODEntryInfo{
            .index = CiA402Index::HomingSpeeds,
            .subindex = 1,
            .dataType = ObjectDictionaryDataType::Unsigned32,
            .bitLength = 32,
            .accessType = 0x3F,
            .name = "Speed during search for switch",
            .defaultValue = driveConfig_.homingSwitchSpeed
        },
        [this](uint8_t* data, size_t& len) -> SDOAbortCode {
            if (len < 4) return SDOAbortCode::DataTypeMismatch;
            std::memcpy(data, &homingSwitchSpeed_, 4);
            len = 4;
            return SDOAbortCode::Success;
        },
        [this](const uint8_t* data, size_t len) -> SDOAbortCode {
            if (len != 4) return SDOAbortCode::DataTypeMismatch;
            std::memcpy(&homingSwitchSpeed_, data, 4);
            return SDOAbortCode::Success;
        }
    );
    od.registerObject(
        ODEntryInfo{
            .index = CiA402Index::HomingSpeeds,
            .subindex = 2,
            .dataType = ObjectDictionaryDataType::Unsigned32,
            .bitLength = 32,
            .accessType = 0x3F,
            .name = "Speed during search for zero",
            .defaultValue = driveConfig_.homingZeroSpeed
        },
        [this](uint8_t* data, size_t& len) -> SDOAbortCode {
            if (len < 4) return SDOAbortCode::DataTypeMismatch;
            std::memcpy(data, &homingZeroSpeed_, 4);
            len = 4;
            return SDOAbortCode::Success;
        },
        [this](const uint8_t* data, size_t len) -> SDOAbortCode {
            if (len != 4) return SDOAbortCode::DataTypeMismatch;
            std::memcpy(&homingZeroSpeed_, data, 4);
            return SDOAbortCode::Success;
        }
    );
    
    // 0x609A - Homing Acceleration (UNSIGNED32, RW)
    od.registerObject(
        ODEntryInfo{
            .index = CiA402Index::HomingAcceleration,
            .subindex = 0,
            .dataType = ObjectDictionaryDataType::Unsigned32,
            .bitLength = 32,
            .accessType = 0x3F,
            .name = "Homing acceleration",
            .defaultValue = driveConfig_.homingAcceleration
        },
        [this](uint8_t* data, size_t& len) -> SDOAbortCode {
            if (len < 4) return SDOAbortCode::DataTypeMismatch;
            std::memcpy(data, &homingAcceleration_, 4);
            len = 4;
            return SDOAbortCode::Success;
        },
        [this](const uint8_t* data, size_t len) -> SDOAbortCode {
            if (len != 4) return SDOAbortCode::DataTypeMismatch;
            std::memcpy(&homingAcceleration_, data, 4);
            return SDOAbortCode::Success;
        }
    );
    
    // ========================================================================
    // CSP/CSV/CST Offset Objects (0x60B0-0x60B2)
    // ========================================================================
    
    // 0x60B0 - Position Offset (INTEGER32, RW)
    od.registerObject(
        ODEntryInfo{
            .index = CiA402Index::PositionOffset,
            .subindex = 0,
            .dataType = ObjectDictionaryDataType::Integer32,
            .bitLength = 32,
            .accessType = 0x3F,
            .name = "Position offset",
            .defaultValue = 0
        },
        [this](uint8_t* data, size_t& len) -> SDOAbortCode {
            if (len < 4) return SDOAbortCode::DataTypeMismatch;
            std::memcpy(data, &positionOffset_, 4);
            len = 4;
            return SDOAbortCode::Success;
        },
        [this](const uint8_t* data, size_t len) -> SDOAbortCode {
            if (len != 4) return SDOAbortCode::DataTypeMismatch;
            std::memcpy(&positionOffset_, data, 4);
            return SDOAbortCode::Success;
        }
    );
    
    // 0x60B1 - Velocity Offset (INTEGER32, RW)
    od.registerObject(
        ODEntryInfo{
            .index = CiA402Index::VelocityOffset,
            .subindex = 0,
            .dataType = ObjectDictionaryDataType::Integer32,
            .bitLength = 32,
            .accessType = 0x3F,
            .name = "Velocity offset",
            .defaultValue = 0
        },
        [this](uint8_t* data, size_t& len) -> SDOAbortCode {
            if (len < 4) return SDOAbortCode::DataTypeMismatch;
            std::memcpy(data, &velocityOffset_, 4);
            len = 4;
            return SDOAbortCode::Success;
        },
        [this](const uint8_t* data, size_t len) -> SDOAbortCode {
            if (len != 4) return SDOAbortCode::DataTypeMismatch;
            std::memcpy(&velocityOffset_, data, 4);
            return SDOAbortCode::Success;
        }
    );
    
    // 0x60B2 - Torque Offset (INTEGER16, RW)
    od.registerObject(
        ODEntryInfo{
            .index = CiA402Index::TorqueOffset,
            .subindex = 0,
            .dataType = ObjectDictionaryDataType::Integer16,
            .bitLength = 16,
            .accessType = 0x3F,
            .name = "Torque offset",
            .defaultValue = 0
        },
        [this](uint8_t* data, size_t& len) -> SDOAbortCode {
            if (len < 2) return SDOAbortCode::DataTypeMismatch;
            std::memcpy(data, &torqueOffset_, 2);
            len = 2;
            return SDOAbortCode::Success;
        },
        [this](const uint8_t* data, size_t len) -> SDOAbortCode {
            if (len != 2) return SDOAbortCode::DataTypeMismatch;
            std::memcpy(&torqueOffset_, data, 2);
            return SDOAbortCode::Success;
        }
    );
    
    // ========================================================================
    // Touch Probe Objects (0x60B8-0x60BC)
    // ========================================================================
    
    // 0x60B8 - Touch Probe Function (UNSIGNED16, RW)
    od.registerObject(
        ODEntryInfo{
            .index = CiA402Index::TouchProbeFunction,
            .subindex = 0,
            .dataType = ObjectDictionaryDataType::Unsigned16,
            .bitLength = 16,
            .accessType = 0x3F,
            .name = "Touch probe function",
            .defaultValue = 0
        },
        [this](uint8_t* data, size_t& len) -> SDOAbortCode {
            if (len < 2) return SDOAbortCode::DataTypeMismatch;
            std::memcpy(data, &touchProbeFunction_, 2);
            len = 2;
            return SDOAbortCode::Success;
        },
        [this](const uint8_t* data, size_t len) -> SDOAbortCode {
            if (len != 2) return SDOAbortCode::DataTypeMismatch;
            std::memcpy(&touchProbeFunction_, data, 2);
            return SDOAbortCode::Success;
        }
    );
    
    // 0x60B9 - Touch Probe Status (UNSIGNED16, RO)
    od.registerObject(
        ODEntryInfo{
            .index = CiA402Index::TouchProbeStatus,
            .subindex = 0,
            .dataType = ObjectDictionaryDataType::Unsigned16,
            .bitLength = 16,
            .accessType = 0x3F,
            .name = "Touch probe status",
            .defaultValue = 0
        },
        [this](uint8_t* data, size_t& len) -> SDOAbortCode {
            if (len < 2) return SDOAbortCode::DataTypeMismatch;
            std::memcpy(data, &touchProbeStatus_, 2);
            len = 2;
            return SDOAbortCode::Success;
        },
        nullptr
    );
    
    // 0x60BA - Touch Probe 1 Positive Edge Position (INTEGER32, RO)
    od.registerObject(
        ODEntryInfo{
            .index = CiA402Index::TouchProbe1PosValue,
            .subindex = 0,
            .dataType = ObjectDictionaryDataType::Integer32,
            .bitLength = 32,
            .accessType = 0x3F,
            .name = "Touch probe 1 positive edge position",
            .defaultValue = 0
        },
        [this](uint8_t* data, size_t& len) -> SDOAbortCode {
            if (len < 4) return SDOAbortCode::DataTypeMismatch;
            std::memcpy(data, &touchProbe1Pos_, 4);
            len = 4;
            return SDOAbortCode::Success;
        },
        nullptr
    );
    
    // 0x60BC - Touch Probe 2 Positive Edge Position (INTEGER32, RO)
    od.registerObject(
        ODEntryInfo{
            .index = CiA402Index::TouchProbe2PosValue,
            .subindex = 0,
            .dataType = ObjectDictionaryDataType::Integer32,
            .bitLength = 32,
            .accessType = 0x3F,
            .name = "Touch probe 2 positive edge position",
            .defaultValue = 0
        },
        [this](uint8_t* data, size_t& len) -> SDOAbortCode {
            if (len < 4) return SDOAbortCode::DataTypeMismatch;
            std::memcpy(data, &touchProbe2Pos_, 4);
            len = 4;
            return SDOAbortCode::Success;
        },
        nullptr
    );
    
    // ========================================================================
    // Following Error (0x60F4)
    // ========================================================================
    
    // 0x60F4 - Following Error Actual Value (INTEGER32, RO)
    od.registerObject(
        ODEntryInfo{
            .index = CiA402Index::FollowingErrorActual,
            .subindex = 0,
            .dataType = ObjectDictionaryDataType::Integer32,
            .bitLength = 32,
            .accessType = 0x3F,
            .name = "Following error actual value",
            .defaultValue = 0
        },
        [this](uint8_t* data, size_t& len) -> SDOAbortCode {
            if (len < 4) return SDOAbortCode::DataTypeMismatch;
            int32_t fe = getFollowingError();
            std::memcpy(data, &fe, 4);
            len = 4;
            return SDOAbortCode::Success;
        },
        nullptr
    );
    
    // ========================================================================
    // Digital I/O Objects (0x60FD-0x60FE)
    // ========================================================================
    
    // 0x60FD - Digital Inputs (UNSIGNED32, RO)
    od.registerObject(
        ODEntryInfo{
            .index = CiA402Index::DigitalInputs,
            .subindex = 0,
            .dataType = ObjectDictionaryDataType::Unsigned32,
            .bitLength = 32,
            .accessType = 0x3F,
            .name = "Digital inputs",
            .defaultValue = 0
        },
        [this](uint8_t* data, size_t& len) -> SDOAbortCode {
            if (len < 4) return SDOAbortCode::DataTypeMismatch;
            std::memcpy(data, &digitalInputs_, 4);
            len = 4;
            return SDOAbortCode::Success;
        },
        nullptr
    );
    
    // 0x60FE - Digital Outputs (ARRAY of UNSIGNED32, RW)
    od.registerObject(
        ODEntryInfo{
            .index = CiA402Index::DigitalOutputs,
            .subindex = 0,
            .dataType = ObjectDictionaryDataType::Unsigned8,
            .bitLength = 8,
            .accessType = 0x3F,
            .name = "Digital outputs - Number of entries",
            .defaultValue = 2
        }
    );
    od.registerObject(
        ODEntryInfo{
            .index = CiA402Index::DigitalOutputs,
            .subindex = 1,
            .dataType = ObjectDictionaryDataType::Unsigned32,
            .bitLength = 32,
            .accessType = 0x3F,
            .name = "Physical outputs",
            .defaultValue = 0
        },
        [this](uint8_t* data, size_t& len) -> SDOAbortCode {
            if (len < 4) return SDOAbortCode::DataTypeMismatch;
            std::memcpy(data, &digitalOutputs_, 4);
            len = 4;
            return SDOAbortCode::Success;
        },
        [this](const uint8_t* data, size_t len) -> SDOAbortCode {
            if (len != 4) return SDOAbortCode::DataTypeMismatch;
            uint32_t outputs;
            std::memcpy(&outputs, data, 4);
            digitalOutputs_ = (digitalOutputs_ & ~digitalOutputMask_) | (outputs & digitalOutputMask_);
            return SDOAbortCode::Success;
        }
    );
    od.registerObject(
        ODEntryInfo{
            .index = CiA402Index::DigitalOutputs,
            .subindex = 2,
            .dataType = ObjectDictionaryDataType::Unsigned32,
            .bitLength = 32,
            .accessType = 0x3F,
            .name = "Bit mask",
            .defaultValue = 0xFFFFFFFF
        },
        [this](uint8_t* data, size_t& len) -> SDOAbortCode {
            if (len < 4) return SDOAbortCode::DataTypeMismatch;
            std::memcpy(data, &digitalOutputMask_, 4);
            len = 4;
            return SDOAbortCode::Success;
        },
        [this](const uint8_t* data, size_t len) -> SDOAbortCode {
            if (len != 4) return SDOAbortCode::DataTypeMismatch;
            std::memcpy(&digitalOutputMask_, data, 4);
            return SDOAbortCode::Success;
        }
    );
    
    // ========================================================================
    // Supported Drive Modes (0x6502)
    // ========================================================================
    
    // 0x6502 - Supported Drive Modes (UNSIGNED32, RO)
    od.registerObject(
        ODEntryInfo{
            .index = CiA402Index::SupportedDriveModes,
            .subindex = 0,
            .dataType = ObjectDictionaryDataType::Unsigned32,
            .bitLength = 32,
            .accessType = 0x3F,
            .name = "Supported drive modes",
            .defaultValue = driveConfig_.supportedModes
        },
        [this](uint8_t* data, size_t& len) -> SDOAbortCode {
            if (len < 4) return SDOAbortCode::DataTypeMismatch;
            uint32_t modes = driveConfig_.supportedModes;
            std::memcpy(data, &modes, 4);
            len = 4;
            return SDOAbortCode::Success;
        },
        nullptr
    );

    // After dictionary initialization, transition to SwitchOnDisabled
    setDriveState(CiA402State::SwitchOnDisabled);
}

}  // namespace slave
}  // namespace EtherCAT
