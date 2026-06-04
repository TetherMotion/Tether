/**
 * @file DynaDrive.hpp
 * @brief DynaDrive (ANYdrive / rsl_drive_sdk) constants and helpers
 *
 * Vendor/Product from SII dump:
 *   Vendor ID:    0x00414E59  (ASCII "ANY")
 *   Product Code: 0x17010001
 *   Name:         "Synchron"
 */

#pragma once

#include <cstdint>
#include <string>

#include "tether/drives/DynaDrive/DynaDrivePDO.hpp"

namespace EtherCAT {
namespace Drives {

// ============================================================================
// Drive Identity
// ============================================================================

constexpr uint32_t kDynaDriveVendorId    = 0x00414E59;
constexpr uint32_t kDynaDriveProductCode = 0x17010001;

// ============================================================================
// Controlword Transition IDs (rsl_drive_sdk/fsm/Controlword.hpp)
// ============================================================================

namespace DynaDriveControlword {
    constexpr uint16_t kWarmReset                = 0x01;
    constexpr uint16_t kClearErrorsToMotorOp     = 0x02;
    constexpr uint16_t kStandbyToConfigure       = 0x03;
    constexpr uint16_t kConfigureToStandby       = 0x04;
    constexpr uint16_t kCalibrateToConfigure     = 0x05;
    constexpr uint16_t kConfigureToCalibrate     = 0x06;
    constexpr uint16_t kMotorOpToStandby         = 0x07;
    constexpr uint16_t kStandbyToMotorPreOp      = 0x08;
    constexpr uint16_t kControlOpToMotorOp       = 0x09;
    constexpr uint16_t kMotorOpToControlOp        = 0x0A;
    constexpr uint16_t kControlOpToStandby       = 0x0B;
    constexpr uint16_t kClearErrorsToStandby     = 0x0C;
}

// ============================================================================
// Operating Mode IDs (rsl_drive_sdk/mode/ModeEnum.hpp)
// ============================================================================

namespace DynaDriveMode {
    constexpr uint16_t kFreeze                  = 1;
    constexpr uint16_t kDisable                 = 2;
    constexpr uint16_t kCurrent                 = 3;
    constexpr uint16_t kMotorPosition           = 4;
    constexpr uint16_t kMotorVelocity           = 5;
    constexpr uint16_t kGearPosition            = 6;
    constexpr uint16_t kGearVelocity            = 7;
    constexpr uint16_t kJointPosition           = 8;
    constexpr uint16_t kJointVelocity           = 9;
    constexpr uint16_t kJointTorque             = 10;
    constexpr uint16_t kJointPositionVelocity   = 11;
    constexpr uint16_t kJointPositionVelocityTorque = 12;
    constexpr uint16_t kJointPositionVelocityTorquePidGains = 13;
    constexpr uint16_t kJointPositionTorque     = 16;
}

// ============================================================================
// Statusword decoding helpers
// ============================================================================

struct DynaDriveStatusword {
    uint32_t raw = 0;

    explicit DynaDriveStatusword(uint32_t data) : raw(data) {}

    uint8_t stateId() const { return static_cast<uint8_t>(raw & 0x0F); }
    uint8_t modeId() const  { return static_cast<uint8_t>((raw >> 4) & 0x0F); }

    bool isError()  const { return stateId() == 8; }
    bool isFatal()  const { return stateId() == 9; }
    bool isControlOp() const { return stateId() == 7; }
    bool isMotorOp() const { return stateId() == 6; }
    bool isStandby() const { return stateId() == 5; }

    bool hasWarningOvertemperatureBridge() const { return (raw >> 8)  & 1; }
    bool hasWarningOvertemperatureStator() const { return (raw >> 9)  & 1; }
    bool hasWarningOvertemperatureCpu()     const { return (raw >> 10) & 1; }
    bool hasErrorPdoTimeout()              const { return (raw >> 15) & 1; }
};

// ============================================================================
// State name helper
// ============================================================================

std::string dynaDriveStateName(uint8_t state_id);

} // namespace Drives
} // namespace EtherCAT
