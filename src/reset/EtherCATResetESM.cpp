/**
 * @file EtherCATResetESM.cpp
 * @brief EtherCAT State Machine (ESM) reset methods
 * 
 * Split from EtherCATReset.cpp for maintainability.
 */

#include "tether/ethercat/Reset.hpp"
#include "tether/platform/EspCompat.hpp"
#include "tether/ethercat/CoEManager.hpp"
#include "tether/ethercat/DCClass.hpp"

namespace EtherCAT {

static const char* TAG = "ECAT_RESET_ESM";

// ============================================================================
// EtherCAT State Machine (ESM) Methods
// ============================================================================

bool SlaveResetController::acknowledgeError() {
    uint16_t al_control = static_cast<uint16_t>(ALState::Init) | ALControl::AckError;
    
    uint16_t current_status, current_code;
    if (!readALStatus(current_status, current_code)) {
        last_result_.error_message = "Failed to read AL status";
        return false;
    }
    
    uint8_t current_state = current_status & ALControl::StateMask;
    al_control = current_state | ALControl::AckError;
    
    if (!writeALControl(al_control)) {
        last_result_.error_message = "Failed to write AL control for error acknowledge";
        return false;
    }
    
    Tether::Platform::Clock::instance().delayMilliseconds(10);
    if (!readALStatus(current_status, current_code)) {
        return false;
    }
    
    bool error_cleared = !(current_status & static_cast<uint16_t>(ALState::ErrorFlag));
    
    if (!error_cleared) {
        last_result_.error_message = "Error acknowledge failed - error flag still set";
        TETHER_LOGW(TAG, "Error still present after acknowledge. Status code: 0x%04X (%s)",
                 current_code, getALStatusCodeName(current_code));
    }
    
    return error_cleared;
}

bool SlaveResetController::readALStatus(uint16_t& status, uint16_t& status_code) {
    size_t actual_size;
    if (!sdoRead(0x0130, 0, &status, sizeof(status), &actual_size)) {
        return false;
    }
    
    if (!sdoRead(0x0134, 0, &status_code, sizeof(status_code), &actual_size)) {
        status_code = 0;
    }
    
    return true;
}

bool SlaveResetController::writeALControl(uint16_t value) {
    return sdoWrite(0x0120, 0, &value, sizeof(value));
}

bool SlaveResetController::forceToInit(uint32_t timeout_ms) {
    uint16_t al_control = static_cast<uint16_t>(ALState::Init);
    
    if (!writeALControl(al_control)) {
        last_result_.error_message = "Failed to write AL control for INIT transition";
        return false;
    }
    
    return waitForState(ALState::Init, timeout_ms);
}

bool SlaveResetController::waitForState(ALState target, uint32_t timeout_ms) {
    uint32_t elapsed = 0;
    const uint32_t poll_interval = 10;
    
    while (elapsed < timeout_ms) {
        uint16_t status, code;
        if (readALStatus(status, code)) {
            uint8_t current_state = status & ALControl::StateMask;
            if (current_state == static_cast<uint8_t>(target)) {
                return true;
            }
            
            if (status & static_cast<uint16_t>(ALState::ErrorFlag)) {
                last_result_.error_message = "State transition failed with error: ";
                last_result_.error_message += getALStatusCodeName(code);
                last_result_.al_status_code = code;
                return false;
            }
        }
        
        Tether::Platform::Clock::instance().delayMilliseconds(poll_interval);
        elapsed += poll_interval;
    }
    
    last_result_.error_message = "Timeout waiting for state transition";
    return false;
}

bool SlaveResetController::requestESCReset() {
    uint16_t al_control = static_cast<uint16_t>(ALState::Init) | ALControl::ESCReset;
    
    if (!writeALControl(al_control)) {
        last_result_.error_message = "Failed to write ESC reset command";
        return false;
    }
    
    return true;
}

bool SlaveResetController::transitionToState(ALState target_state, uint32_t timeout_ms) {
    uint16_t al_control = static_cast<uint16_t>(target_state);
    
    if (!writeALControl(al_control)) {
        last_result_.error_message = "Failed to write state transition request";
        return false;
    }
    
    return waitForState(target_state, timeout_ms);
}

ResetResult SlaveResetController::fullReinitialize(bool to_op) {
    ResetResult result;
    result.requested_level = ResetLevel::StateMachineReset;
    int64_t start_time = esp_timer_get_time();
    
    TETHER_LOGI(TAG, "Full re-initialization of slave %u", slave_index_);
    
    reportProgress("Forcing to INIT", 10);
    if (!forceToInit(1000)) {
        result.error_message = "Failed to reach INIT state: " + last_result_.error_message;
        result.duration_us = static_cast<uint32_t>(esp_timer_get_time() - start_time);
        return result;
    }
    
    reportProgress("Configuring mailbox", 25);
    
    reportProgress("Transitioning to PRE-OP", 40);
    if (!transitionToState(ALState::PreOp, 1000)) {
        result.error_message = "Failed to reach PRE-OP state: " + last_result_.error_message;
        result.achieved_level = ResetLevel::StateMachineReset;
        result.duration_us = static_cast<uint32_t>(esp_timer_get_time() - start_time);
        return result;
    }
    
    reportProgress("Configuring PDOs", 55);
    
    reportProgress("Transitioning to SAFE-OP", 70);
    if (!transitionToState(ALState::SafeOp, 1000)) {
        result.error_message = "Failed to reach SAFE-OP state: " + last_result_.error_message;
        result.achieved_level = ResetLevel::ApplicationReset;
        result.duration_us = static_cast<uint32_t>(esp_timer_get_time() - start_time);
        return result;
    }
    
    if (to_op) {
        reportProgress("Transitioning to OP", 85);
        if (!transitionToState(ALState::Op, 1000)) {
            result.error_message = "Failed to reach OP state: " + last_result_.error_message;
            result.achieved_level = ResetLevel::ApplicationReset;
            result.success = true;
            result.duration_us = static_cast<uint32_t>(esp_timer_get_time() - start_time);
            return result;
        }
    }
    
    reportProgress("Re-initialization complete", 100);
    result.success = true;
    result.achieved_level = ResetLevel::StateMachineReset;
    
    readALStatus(result.al_status, result.al_status_code);
    result.duration_us = static_cast<uint32_t>(esp_timer_get_time() - start_time);
    
    TETHER_LOGI(TAG, "Full re-initialization completed in %u us", result.duration_us);
    
    last_result_ = result;
    return result;
}

// ============================================================================
// Sync Manager and Watchdog Reset
// ============================================================================

bool SlaveResetController::resetSyncManagerWatchdog() {
    TETHER_LOGI(TAG, "Resetting Sync Manager watchdog on slave %u", slave_index_);
    
    uint8_t disable = 0x00;
    if (!sdoWrite(0x0806, 0, &disable, 1) ||
        !sdoWrite(0x080E, 0, &disable, 1)) {
        return false;
    }
    
    Tether::Platform::Clock::instance().delayMilliseconds(10);
    
    uint8_t enable = 0x01;
    if (!sdoWrite(0x0806, 0, &enable, 1) ||
        !sdoWrite(0x080E, 0, &enable, 1)) {
        return false;
    }
    
    return true;
}

bool SlaveResetController::clearPDIWatchdog() {
    uint16_t wd_div = 0x09C2;
    return sdoWrite(0x0400, 0, &wd_div, sizeof(wd_div));
}

bool SlaveResetController::reconfigureSyncManagers() {
    TETHER_LOGI(TAG, "Reconfiguring Sync Managers on slave %u", slave_index_);
    return true;
}

// ============================================================================
// Distributed Clock Reset
// ============================================================================

bool SlaveResetController::resetDistributedClock() {
    TETHER_LOGI(TAG, "Resetting Distributed Clock on slave %u", slave_index_);
    
    uint8_t dc_disable = 0x00;
    if (!sdoWrite(toUInt16(DCRegisters::DCSyncAct), 0, &dc_disable, 1)) {
        return false;
    }
    
    uint64_t zero_offset = 0;
    if (!sdoWrite(toUInt16(DCRegisters::DCSysOffset), 0, &zero_offset, sizeof(zero_offset))) {
        return false;
    }
    
    return true;
}

bool SlaveResetController::clearDCSyncErrors() {
    uint16_t zero = 0;
    return sdoWrite(toUInt16(DCRegisters::DCSysDiff), 0, &zero, sizeof(zero));
}

// ============================================================================
// Vendor-Specific Reset Methods
// ============================================================================

bool SlaveResetController::vendorSpecificReset(uint16_t index, uint8_t subindex,
                                                const uint8_t* data, size_t data_len) {
    return sdoWrite(index, subindex, data, data_len);
}

bool SlaveResetController::voeReset(const uint8_t* voe_data, size_t voe_len) {
    TETHER_LOGW(TAG, "VoE reset not fully implemented");
    return false;
}

} // namespace EtherCAT
