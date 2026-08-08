// SPDX-License-Identifier: MIT

/**
 * @file DynaDriveController.cpp
 * @brief DynaDrive custom FSM controller implementation
 */

#include "tether/profiles/cia402/DynaDriveController.hpp"
#include "tether/profiles/cia301/CiA402Defs.hpp"
#include "tether/ethercat/CoEManager.hpp"
#include "tether/platform/Platform.hpp"

static const char* TAG = "DynaDrive";

namespace EtherCAT {

DynaDriveController::DynaDriveController(Master& master, uint16_t slave_index,
                                          uint32_t sdo_timeout_ms)
    : master_(master)
    , slave_index_(slave_index)
    , sdo_timeout_ms_(sdo_timeout_ms) {}

// ============================================================================
// State decode helpers
// ============================================================================

const char* DynaDriveController::getStateName(DynaDriveState state) {
    switch (state) {
        case DynaDriveState::NA:            return "NA";
        case DynaDriveState::ColdStart:     return "ColdStart";
        case DynaDriveState::WarmStart:     return "WarmStart";
        case DynaDriveState::Configure:     return "Configure";
        case DynaDriveState::Calibrate:     return "Calibrate";
        case DynaDriveState::Standby:       return "Standby";
        case DynaDriveState::MotorOp:       return "MotorOp";
        case DynaDriveState::ControlOp:     return "ControlOp";
        case DynaDriveState::Error:         return "Error";
        case DynaDriveState::Fatal:         return "Fatal";
        case DynaDriveState::MotorPreOp:    return "MotorPreOp";
        case DynaDriveState::DeviceMissing: return "DeviceMissing";
        default:                            return "Unknown";
    }
}

DynaDriveController::DynaDriveState DynaDriveController::decodeState(uint32_t statusword) {
    uint8_t state_id = static_cast<uint8_t>(statusword & 0x0F);
    switch (state_id) {
        case 0:  return DynaDriveState::NA;
        case 1:  return DynaDriveState::ColdStart;
        case 2:  return DynaDriveState::WarmStart;
        case 3:  return DynaDriveState::Configure;
        case 4:  return DynaDriveState::Calibrate;
        case 5:  return DynaDriveState::Standby;
        case 6:  return DynaDriveState::MotorOp;
        case 7:  return DynaDriveState::ControlOp;
        case 8:  return DynaDriveState::Error;
        case 9:  return DynaDriveState::Fatal;
        case 10: return DynaDriveState::MotorPreOp;
        case 11: return DynaDriveState::DeviceMissing;
        default: return DynaDriveState::Unknown;
    }
}

// ============================================================================
// SDO-based status / control
// ============================================================================

bool DynaDriveController::readStatusword(uint32_t& statusword) {
    auto result = master_.sdoManager(slave_index_).readU32(
        static_cast<uint16_t>(CiA402::Register::Statusword), 0,
        {.timeout_ms = sdo_timeout_ms_});
    if (!result.has_value()) return false;
    statusword = result.value();
    return true;
}

bool DynaDriveController::sendControlword(
    EtherCAT::Drives::Registers::DynaDrive::Controlword::Options controlword) {
    uint16_t cw = static_cast<uint16_t>(controlword);
    TETHER_LOGI(TAG, "Slave %u: DynaDrive sending controlword ID 0x%02X", slave_index_, cw);
    auto result = master_.sdoManager(slave_index_).writeU16(
        static_cast<uint16_t>(CiA402::Register::Controlword), 0, cw,
        {.timeout_ms = sdo_timeout_ms_});
    return result.has_value();
}

// ============================================================================
// High-level FSM transitions
// ============================================================================

bool DynaDriveController::enable(uint32_t timeout_ms) {
    TETHER_LOGI(TAG, "Slave %u: Enabling DynaDrive (target ControlOp)...", slave_index_);

    const uint32_t poll_interval = 100;
    uint32_t elapsed = 0;

    auto wait_for_state = [&](DynaDriveState target) -> bool {
        while (elapsed < timeout_ms) {
            uint32_t sw = 0;
            if (readStatusword(sw)) {
                DynaDriveState current = decodeState(sw);
                if (current == target) {
                    TETHER_LOGI(TAG, "Slave %u: Reached %s", slave_index_, getStateName(target));
                    return true;
                }
                if (current == DynaDriveState::Fatal) {
                    TETHER_LOGE(TAG, "Slave %u: Fatal state reached!", slave_index_);
                    return false;
                }
            }
            Tether::Platform::Clock::instance().delayMilliseconds(poll_interval);
            elapsed += poll_interval;
        }
        TETHER_LOGE(TAG, "Slave %u: Timeout waiting for %s", slave_index_, getStateName(target));
        return false;
    };

    // Read current state
    uint32_t statusword = 0;
    if (!readStatusword(statusword)) {
        TETHER_LOGE(TAG, "Slave %u: Failed to read DynaDrive statusword", slave_index_);
        return false;
    }
    DynaDriveState state = decodeState(statusword);
    TETHER_LOGI(TAG, "Slave %u: Current DynaDrive state = %s (0x%08X)", slave_index_, getStateName(state), statusword);

    // If in Error, clear to Standby
    if (state == DynaDriveState::Error) {
        TETHER_LOGI(TAG, "Slave %u: Clearing Error -> Standby", slave_index_);
        if (!sendControlword(EtherCAT::Drives::Registers::DynaDrive::Controlword::Options::ClearErrorsToStandby)) return false;
        Tether::Platform::Clock::instance().delayMilliseconds(500);
        if (!wait_for_state(DynaDriveState::Standby)) return false;
        state = DynaDriveState::Standby;
    }

    // Standby -> MotorPreOp (auto -> MotorOp)
    if (state == DynaDriveState::Standby) {
        TETHER_LOGI(TAG, "Slave %u: Standby -> MotorPreOp", slave_index_);
        if (!sendControlword(EtherCAT::Drives::Registers::DynaDrive::Controlword::Options::StandbyToMotorPreOp)) return false;
        Tether::Platform::Clock::instance().delayMilliseconds(500);
        if (!wait_for_state(DynaDriveState::MotorOp)) return false;
        state = DynaDriveState::MotorOp;
    }

    // MotorOp -> ControlOp
    if (state == DynaDriveState::MotorOp) {
        TETHER_LOGI(TAG, "Slave %u: MotorOp -> ControlOp", slave_index_);
        if (!sendControlword(EtherCAT::Drives::Registers::DynaDrive::Controlword::Options::MotorOpToControlOp)) return false;
        Tether::Platform::Clock::instance().delayMilliseconds(500);
        if (!wait_for_state(DynaDriveState::ControlOp)) return false;
        state = DynaDriveState::ControlOp;
    }

    if (state == DynaDriveState::ControlOp) {
        TETHER_LOGI(TAG, "Slave %u: DynaDrive enabled (ControlOp)", slave_index_);
        return true;
    }

    TETHER_LOGE(TAG, "Slave %u: Unexpected DynaDrive state %s during enable",
             slave_index_, getStateName(state));
    return false;
}

bool DynaDriveController::disable() {
    TETHER_LOGI(TAG, "Slave %u: Disabling DynaDrive (ControlOp -> Standby)", slave_index_);
    uint32_t statusword = 0;
    if (readStatusword(statusword)) {
        DynaDriveState state = decodeState(statusword);
        if (state == DynaDriveState::ControlOp) {
            return sendControlword(EtherCAT::Drives::Registers::DynaDrive::Controlword::Options::ControlOpToStandby);
        }
    }
    return true;  // Already not in ControlOp
}

bool DynaDriveController::isControlOp() {
    uint32_t statusword = 0;
    if (readStatusword(statusword)) {
        return decodeState(statusword) == DynaDriveState::ControlOp;
    }
    return false;
}

} // namespace EtherCAT
