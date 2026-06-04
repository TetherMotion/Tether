/**
 * @file DynaDrive.cpp
 * @brief DynaDrive (ANYdrive / rsl_drive_sdk) statusword and state helpers
 */

#include "tether/drives/DynaDrive.hpp"

namespace EtherCAT {
namespace Drives {

std::string dynaDriveStateName(Registers::DynaDrive::Status::StateOptions state) {
    using SO = Registers::DynaDrive::Status::StateOptions;
    switch (state) {
        case SO::NA:            return "NA";
        case SO::ColdStart:     return "ColdStart";
        case SO::WarmStart:     return "WarmStart";
        case SO::Configure:     return "Configure";
        case SO::Calibrate:     return "Calibrate";
        case SO::Standby:       return "Standby";
        case SO::MotorOp:       return "MotorOp";
        case SO::ControlOp:     return "ControlOp";
        case SO::Error:         return "Error";
        case SO::Fatal:         return "Fatal";
        case SO::MotorPreOp:    return "MotorPreOp";
        case SO::DeviceMissing: return "DeviceMissing";
        default:                return "Unknown";
    }
}

} // namespace Drives
} // namespace EtherCAT
