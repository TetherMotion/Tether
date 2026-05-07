/**
 * @file EtherCATResetCiA.cpp
 * @brief CiA 301 NMT and CiA 402 drive reset methods
 * 
 * Split from EtherCATReset.cpp for maintainability.
 */

#include "EtherCATReset.hpp"
#include "tether/platform/EspCompat.hpp"
#include "profiles/cia301/CiA301Defs.hpp"
#include "profiles/cia301/CiA402Defs.hpp"

namespace EtherCAT {

static const char* TAG = "ECAT_RESET_CIA";

// ============================================================================
// CiA 301 NMT Reset Methods
// ============================================================================

bool SlaveResetController::nmtResetNode() {
    TETHER_LOGI(TAG, "Sending NMT Reset Node to slave %u", slave_addr_);
    return restoreDefaultParameters(CiA301Reset::AllParameters);
}

bool SlaveResetController::nmtResetCommunication() {
    TETHER_LOGI(TAG, "Sending NMT Reset Communication to slave %u", slave_addr_);
    return restoreDefaultParameters(CiA301Reset::CommunicationParams);
}

bool SlaveResetController::restoreDefaultParameters(uint8_t subindex) {
    uint32_t signature = CiA301Reset::RestoreSignature;
    return sdoWrite(CiA301Reset::RestoreParameters, subindex, &signature, sizeof(signature));
}

bool SlaveResetController::clearErrorHistory() {
    uint8_t zero = 0;
    return sdoWrite(CiA301::PreDefinedErrorField, 0, &zero, sizeof(zero));
}

// ============================================================================
// CiA 402 Drive Reset Methods
// ============================================================================

ResetResult SlaveResetController::faultReset(CiA402State target_state) {
    ResetResult result;
    result.requested_level = ResetLevel::SoftReset;
    int64_t start_time = esp_timer_get_time();
    
    TETHER_LOGI(TAG, "Performing CiA 402 fault reset on slave %u", slave_addr_);
    reportProgress("Reading statusword", 10);
    
    uint16_t statusword;
    if (!readStatusword(statusword)) {
        result.error_message = "Failed to read statusword";
        result.duration_us = static_cast<uint32_t>(esp_timer_get_time() - start_time);
        return result;
    }
    
    if (!(statusword & 0x0008)) {
        TETHER_LOGI(TAG, "Drive not in fault state, no reset needed");
        result.success = true;
        result.achieved_level = ResetLevel::SoftReset;
        result.duration_us = static_cast<uint32_t>(esp_timer_get_time() - start_time);
        return result;
    }
    
    reportProgress("Clearing fault reset bit", 30);
    uint16_t controlword = 0x0000;
    if (!writeControlword(controlword)) {
        result.error_message = "Failed to clear controlword";
        result.duration_us = static_cast<uint32_t>(esp_timer_get_time() - start_time);
        return result;
    }
    
    Tether::Platform::Clock::instance().delayMilliseconds(10);
    
    reportProgress("Setting fault reset bit", 50);
    controlword = CiA402Reset::FaultReset;
    if (!writeControlword(controlword)) {
        result.error_message = "Failed to set fault reset bit";
        result.duration_us = static_cast<uint32_t>(esp_timer_get_time() - start_time);
        return result;
    }
    
    Tether::Platform::Clock::instance().delayMilliseconds(100);
    
    reportProgress("Verifying fault cleared", 70);
    if (!readStatusword(statusword)) {
        result.error_message = "Failed to read statusword after fault reset";
        result.duration_us = static_cast<uint32_t>(esp_timer_get_time() - start_time);
        return result;
    }
    
    if (statusword & 0x0008) {
        result.error_message = "Fault reset failed - fault still present";
        result.duration_us = static_cast<uint32_t>(esp_timer_get_time() - start_time);
        return result;
    }
    
    reportProgress("Clearing fault reset bit", 85);
    controlword = 0x0000;
    writeControlword(controlword);
    
    result.success = true;
    result.achieved_level = ResetLevel::SoftReset;
    result.duration_us = static_cast<uint32_t>(esp_timer_get_time() - start_time);
    
    reportProgress("Fault reset complete", 100);
    TETHER_LOGI(TAG, "Fault reset completed in %u us", result.duration_us);
    
    last_result_ = result;
    return result;
}

bool SlaveResetController::quickStop() {
    uint16_t controlword = CiA402Reset::EnableVoltage;
    return writeControlword(controlword);
}

bool SlaveResetController::halt() {
    uint16_t controlword;
    size_t len;
    if (!sdoRead(static_cast<uint16_t>(CiA402::Register::Controlword), 0, &controlword, sizeof(controlword), &len)) {
        return false;
    }
    
    controlword |= CiA402Reset::Halt;
    return writeControlword(controlword);
}

bool SlaveResetController::resumeFromHalt() {
    uint16_t controlword;
    size_t len;
    if (!sdoRead(static_cast<uint16_t>(CiA402::Register::Controlword), 0, &controlword, sizeof(controlword), &len)) {
        return false;
    }
    
    controlword &= ~CiA402Reset::Halt;
    return writeControlword(controlword);
}

ResetResult SlaveResetController::disableDrive() {
    ResetResult result;
    result.requested_level = ResetLevel::SoftReset;
    int64_t start_time = esp_timer_get_time();
    
    TETHER_LOGI(TAG, "Disabling drive on slave %u", slave_addr_);
    
    uint16_t controlword = 0x0000;
    if (!writeControlword(controlword)) {
        result.error_message = "Failed to disable drive";
        result.duration_us = static_cast<uint32_t>(esp_timer_get_time() - start_time);
        return result;
    }
    
    result.success = true;
    result.achieved_level = ResetLevel::SoftReset;
    result.duration_us = static_cast<uint32_t>(esp_timer_get_time() - start_time);
    
    last_result_ = result;
    return result;
}

ResetResult SlaveResetController::enableDrive() {
    ResetResult result;
    result.requested_level = ResetLevel::SoftReset;
    int64_t start_time = esp_timer_get_time();
    
    TETHER_LOGI(TAG, "Enabling drive on slave %u", slave_addr_);
    
    // Step 1: Shutdown (ready to switch on)
    uint16_t controlword = CiA402Reset::QuickStopInactive | CiA402Reset::EnableVoltage;
    if (!writeControlword(controlword)) {
        result.error_message = "Failed shutdown transition";
        result.duration_us = static_cast<uint32_t>(esp_timer_get_time() - start_time);
        return result;
    }
    Tether::Platform::Clock::instance().delayMilliseconds(10);
    
    // Step 2: Switch on
    controlword |= CiA402Reset::SwitchOn;
    if (!writeControlword(controlword)) {
        result.error_message = "Failed switch on transition";
        result.duration_us = static_cast<uint32_t>(esp_timer_get_time() - start_time);
        return result;
    }
    Tether::Platform::Clock::instance().delayMilliseconds(10);
    
    // Step 3: Enable operation
    controlword |= CiA402Reset::EnableOperation;
    if (!writeControlword(controlword)) {
        result.error_message = "Failed enable operation transition";
        result.duration_us = static_cast<uint32_t>(esp_timer_get_time() - start_time);
        return result;
    }
    
    result.success = true;
    result.achieved_level = ResetLevel::SoftReset;
    result.duration_us = static_cast<uint32_t>(esp_timer_get_time() - start_time);
    
    last_result_ = result;
    return result;
}

bool SlaveResetController::readStatusword(uint16_t& statusword) {
    size_t len;
    return sdoRead(static_cast<uint16_t>(CiA402::Register::Statusword), 0, &statusword, sizeof(statusword), &len);
}

bool SlaveResetController::writeControlword(uint16_t controlword) {
    return sdoWrite(static_cast<uint16_t>(CiA402::Register::Controlword), 0, &controlword, sizeof(controlword));
}

bool SlaveResetController::clearDriveErrors() {
    uint16_t zero = 0;
    return sdoWrite(0x603F, 0, &zero, sizeof(zero));
}

} // namespace EtherCAT
