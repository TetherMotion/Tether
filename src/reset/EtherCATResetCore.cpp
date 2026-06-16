/**
 * @file EtherCATResetCore.cpp
 * @brief Core EtherCAT reset functionality - status codes, level names, controller basics
 * 
 * Split from EtherCATReset.cpp for maintainability.
 */

#include "Reset.hpp"
#include "tether/platform/EspCompat.hpp"
#include "SDOManager.hpp"
#include "profiles/cia301/CiA301Defs.hpp"
#include "profiles/cia301/CiA402Defs.hpp"

namespace EtherCAT {

static const char* TAG = "ECAT_RESET";

// ============================================================================
// Reset Level Names and Descriptions
// ============================================================================

const char* getResetLevelName(ResetLevel level) {
    switch (level) {
        case ResetLevel::SoftReset:          return "Soft Reset";
        case ResetLevel::CommunicationReset: return "Communication Reset";
        case ResetLevel::ApplicationReset:   return "Application Reset";
        case ResetLevel::StateMachineReset:  return "State Machine Reset";
        case ResetLevel::ESCHardwareReset:   return "ESC Hardware Reset";
        case ResetLevel::HardwareReset:      return "Hardware Reset";
        default:                             return "Unknown";
    }
}

const char* getResetLevelDescription(ResetLevel level) {
    switch (level) {
        case ResetLevel::SoftReset:
            return "Clears application-level errors and counters while preserving "
                   "configuration and state machine position. Fastest recovery.";
        case ResetLevel::CommunicationReset:
            return "Resets communication parameters (PDO/SDO config) to power-on "
                   "defaults while slave remains in current EtherCAT state.";
        case ResetLevel::ApplicationReset:
            return "Full application layer restart. Device profile state machines "
                   "are reset. Slave typically returns to PRE-OP state.";
        case ResetLevel::StateMachineReset:
            return "Forces EtherCAT State Machine to INIT state. All Sync Managers "
                   "are disabled. Requires full re-initialization sequence.";
        case ResetLevel::ESCHardwareReset:
            return "EtherCAT Slave Controller hardware reset. Triggers INIT with "
                   "ESC reset flag. May require vendor-specific support.";
        case ResetLevel::HardwareReset:
            return "Complete device power cycle. Most disruptive reset level. "
                   "Requires external power management capability.";
        default:
            return "Unknown reset level";
    }
}

// ============================================================================
// AL Status Code Names
// ============================================
// Implementation moved to FaultDetection.cpp to avoid duplicate symbols
// (use the public declarations in the headers).

// ============================================================================
// SlaveResetController - Construction and Basic Methods
// ============================================================================

SlaveResetController::SlaveResetController(EtherCAT::SDO::SDOManager& sdo, uint16_t slave_position)
    : m_sdo(sdo)
    , slave_addr_(slave_position)
    , use_configured_addr_(false)
    , last_result_{false, ResetLevel::SoftReset, ResetLevel::SoftReset, 0, 0, 0, ""}
{
}

SlaveResetController::SlaveResetController(EtherCAT::SDO::SDOManager& sdo, uint16_t slave_address, bool use_configured_addr)
    : m_sdo(sdo)
    , slave_addr_(slave_address)
    , use_configured_addr_(use_configured_addr)
    , last_result_{false, ResetLevel::SoftReset, ResetLevel::SoftReset, 0, 0, 0, ""}
{
}

void SlaveResetController::setProgressCallback(ResetProgressCallback callback) {
    progress_callback_ = std::move(callback);
}

void SlaveResetController::reportProgress(const char* stage, uint8_t progress) {
    if (progress_callback_) {
        progress_callback_(stage, progress, slave_addr_);
    }
}

// ============================================================================
// General Reset Methods
// ============================================================================

ResetResult SlaveResetController::resetToLevel(ResetLevel level, uint32_t timeout_ms) {
    int64_t start_time = esp_timer_get_time();
    reset_attempt_count_++;
    
    last_result_ = {false, level, ResetLevel::SoftReset, 0, 0, 0, ""};
    
    TETHER_LOGI(TAG, "Resetting slave %u to level %s", slave_addr_, getResetLevelName(level));
    reportProgress("Starting reset", 0);
    
    bool success = false;
    
    switch (level) {
        case ResetLevel::SoftReset:
            reportProgress("Acknowledging errors", 20);
            success = acknowledgeError();
            if (success) {
                reportProgress("Clearing error history", 50);
                clearErrorHistory();
                last_result_.achieved_level = ResetLevel::SoftReset;
            }
            break;
            
        case ResetLevel::CommunicationReset:
            reportProgress("Sending NMT reset communication", 30);
            success = nmtResetCommunication();
            if (success) {
                Tether::Platform::Clock::instance().delayMilliseconds(100);
                last_result_.achieved_level = ResetLevel::CommunicationReset;
            }
            break;
            
        case ResetLevel::ApplicationReset:
            reportProgress("Sending NMT reset node", 30);
            success = nmtResetNode();
            if (success) {
                Tether::Platform::Clock::instance().delayMilliseconds(500);
                reportProgress("Waiting for restart", 70);
                uint16_t status, code;
                success = readALStatus(status, code);
                if (success) {
                    last_result_.achieved_level = ResetLevel::ApplicationReset;
                }
            }
            break;
            
        case ResetLevel::StateMachineReset:
            reportProgress("Forcing to INIT state", 30);
            success = forceToInit(timeout_ms);
            if (success) {
                last_result_.achieved_level = ResetLevel::StateMachineReset;
            }
            break;
            
        case ResetLevel::ESCHardwareReset:
            reportProgress("Requesting ESC reset", 30);
            success = requestESCReset();
            if (success) {
                Tether::Platform::Clock::instance().delayMilliseconds(1000);
                reportProgress("Waiting for ESC restart", 70);
                uint16_t status, code;
                success = readALStatus(status, code);
                if (success) {
                    last_result_.achieved_level = ResetLevel::ESCHardwareReset;
                }
            }
            break;
            
        case ResetLevel::HardwareReset:
            last_result_.error_message = "Hardware reset requires external power management";
            TETHER_LOGW(TAG, "%s", last_result_.error_message.c_str());
            success = false;
            break;
    }
    
    readALStatus(last_result_.al_status, last_result_.al_status_code);
    
    last_result_.success = success;
    last_result_.duration_us = static_cast<uint32_t>(esp_timer_get_time() - start_time);
    
    if (success) {
        successful_reset_count_++;
        reportProgress("Reset complete", 100);
        TETHER_LOGI(TAG, "Reset to %s succeeded in %u us", 
                 getResetLevelName(last_result_.achieved_level),
                 last_result_.duration_us);
    } else {
        reportProgress("Reset failed", 100);
        TETHER_LOGE(TAG, "Reset to %s failed: %s", 
                 getResetLevelName(level),
                 last_result_.error_message.c_str());
    }
    
    return last_result_;
}

ResetResult SlaveResetController::progressiveReset(ResetLevel max_level, uint32_t timeout_per_level_ms) {
    TETHER_LOGI(TAG, "Starting progressive reset (max level: %s)", getResetLevelName(max_level));
    
    ResetResult result;
    
    for (int level = static_cast<int>(ResetLevel::SoftReset); 
         level <= static_cast<int>(max_level); 
         level++) {
        
        ResetLevel current_level = static_cast<ResetLevel>(level);
        TETHER_LOGI(TAG, "Trying reset level: %s", getResetLevelName(current_level));
        
        result = resetToLevel(current_level, timeout_per_level_ms);
        
        if (result.success) {
            uint16_t status, code;
            if (readALStatus(status, code)) {
                if (!(status & static_cast<uint16_t>(ALState::ErrorFlag))) {
                    TETHER_LOGI(TAG, "Progressive reset succeeded at level: %s", 
                             getResetLevelName(current_level));
                    return result;
                }
            }
        }
        
        Tether::Platform::Clock::instance().delayMilliseconds(100);
    }
    
    result.success = false;
    result.error_message = "Progressive reset exhausted all levels";
    return result;
}

ResetResult SlaveResetController::emergencyStopAndReset() {
    TETHER_LOGW(TAG, "Emergency stop and reset for slave %u", slave_addr_);
    
    ResetResult result;
    result.requested_level = ResetLevel::ApplicationReset;
    int64_t start_time = esp_timer_get_time();
    
    reportProgress("Emergency quick stop", 10);
    quickStop();
    
    reportProgress("Safe state outputs", 30);
    
    reportProgress("Application reset", 50);
    bool success = nmtResetNode();
    
    if (success) {
        Tether::Platform::Clock::instance().delayMilliseconds(500);
        uint16_t status, code;
        if (readALStatus(status, code)) {
            result.al_status = status;
            result.al_status_code = code;
            result.success = true;
            result.achieved_level = ResetLevel::ApplicationReset;
        }
    }
    
    result.duration_us = static_cast<uint32_t>(esp_timer_get_time() - start_time);
    reportProgress("Emergency stop complete", 100);
    
    last_result_ = result;
    return result;
}

// ============================================================================
// Status and Diagnostics
// ============================================================================

bool SlaveResetController::isInErrorState() {
    uint16_t status, code;
    if (readALStatus(status, code)) {
        return (status & static_cast<uint16_t>(ALState::ErrorFlag)) != 0;
    }
    return true;
}

std::string SlaveResetController::getErrorDescription() {
    uint16_t status, code;
    if (readALStatus(status, code)) {
        std::string desc = "AL Status: 0x";
        char hex[8];
        snprintf(hex, sizeof(hex), "%04X", status);
        desc += hex;
        desc += ", Code: ";
        desc += getALStatusCodeName(code);
        return desc;
    }
    return "Unable to read error status";
}

// ============================================================================
// SDO Helper Methods
// ============================================================================

bool SlaveResetController::sdoWrite(uint16_t index, uint8_t sub, const void* data, size_t len) {
    return m_sdo.writeSync(slave_addr_, index, sub, data, len, EtherCAT::SDO::kDefaultSDOTimeoutMs);
}

bool SlaveResetController::sdoRead(uint16_t index, uint8_t sub, void* data, size_t len, size_t* out_len) {
    return m_sdo.readSync(slave_addr_, index, sub, data, len, EtherCAT::SDO::kDefaultSDOTimeoutMs, out_len);
}

} // namespace EtherCAT
