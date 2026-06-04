/**
 * @file DynaDrive.cpp
 * @brief DynaDrive (ANYdrive / rsl_drive_sdk) statusword and state helpers
 */

#include "tether/drives/DynaDrive.hpp"

namespace EtherCAT {
namespace Drives {

std::string dynaDriveStateName(uint8_t state_id) {
    switch (state_id) {
        case 0:  return "NA";
        case 1:  return "ColdStart";
        case 2:  return "WarmStart";
        case 3:  return "Configure";
        case 4:  return "Calibrate";
        case 5:  return "Standby";
        case 6:  return "MotorOp";
        case 7:  return "ControlOp";
        case 8:  return "Error";
        case 9:  return "Fatal";
        case 10: return "MotorPreOp";
        case 11: return "DeviceMissing";
        default: return "Unknown";
    }
}

} // namespace Drives
} // namespace EtherCAT
