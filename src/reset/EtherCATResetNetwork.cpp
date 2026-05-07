/**
 * @file EtherCATResetNetwork.cpp
 * @brief Network-wide reset functions, broadcast operations, and reset policy
 * 
 * Split from EtherCATReset.cpp for maintainability.
 */

#include "EtherCATReset.hpp"
#include "tether/platform/EspCompat.hpp"
#include "EtherCATSDO.hpp"

namespace EtherCAT {

static const char* TAG = "ECAT_RESET_NET";

// ============================================================================
// Broadcast/Network-Wide Functions
// ============================================================================

std::vector<ResetResult> resetAllSlaves(EtherCAT::SDO::SDOManager& sdo, ResetLevel level, uint32_t timeout_ms) {
    std::vector<ResetResult> results;
    
    TETHER_LOGI(TAG, "Resetting all slaves to level: %s", getResetLevelName(level));
    
    for (uint16_t i = 0; i < 16; i++) {
        SlaveResetController controller(sdo, i);
        
        uint16_t status, code;
        if (!controller.readALStatus(status, code)) {
            break;
        }
        
        ResetResult result = controller.resetToLevel(level, timeout_ms / 16);
        results.push_back(result);
    }
    
    return results;
}

uint16_t broadcastErrorAcknowledge(EtherCAT::SDO::SDOManager& sdo) {
    TETHER_LOGI(TAG, "Broadcasting error acknowledge");
    
    uint16_t al_control = ALControl::AckError;
    return sdo.writeSync(0, 0x0120, 0, &al_control, sizeof(al_control), EtherCAT::SDO::kDefaultSDOTimeoutMs) ? 1 : 0;
}

uint16_t broadcastStateTransition(EtherCAT::SDO::SDOManager& sdo, ALState target_state) {
    TETHER_LOGI(TAG, "Broadcasting state transition to 0x%02X", static_cast<uint8_t>(target_state));
    
    uint16_t al_control = static_cast<uint16_t>(target_state);
    return sdo.writeSync(0, 0x0120, 0, &al_control, sizeof(al_control), EtherCAT::SDO::kDefaultSDOTimeoutMs) ? 1 : 0;
}

bool networkEmergencyStop(EtherCAT::SDO::SDOManager& sdo) {
    TETHER_LOGW(TAG, "NETWORK EMERGENCY STOP");
    
    uint16_t slaves_stopped = broadcastStateTransition(sdo, ALState::Init);
    
    return slaves_stopped > 0;
}

bool reinitializeNetwork(EtherCAT::SDO::SDOManager& sdo, bool to_op) {
    TETHER_LOGI(TAG, "Reinitializing entire network");
    
    broadcastStateTransition(sdo, ALState::Init);
    Tether::Platform::Clock::instance().delayMilliseconds(100);
    
    broadcastStateTransition(sdo, ALState::PreOp);
    Tether::Platform::Clock::instance().delayMilliseconds(100);
    
    broadcastStateTransition(sdo, ALState::SafeOp);
    Tether::Platform::Clock::instance().delayMilliseconds(100);
    
    if (to_op) {
        broadcastStateTransition(sdo, ALState::Op);
    }
    
    return true;
}

// ============================================================================
// Reset Policy Application
// ============================================================================

ResetResult applyResetPolicy(SlaveResetController& controller, const ResetPolicy& policy) {
    ResetResult result;
    uint8_t attempt = 0;
    ResetLevel current_level = policy.starting_level;
    
    while (attempt < policy.max_auto_attempts) {
        attempt++;
        TETHER_LOGI(TAG, "Reset attempt %u/%u at level %s",
                 attempt, policy.max_auto_attempts, getResetLevelName(current_level));
        
        result = controller.resetToLevel(current_level);
        
        if (result.success) {
            if (policy.auto_reenable_drive) {
                controller.enableDrive();
            }
            return result;
        }
        
        if (policy.should_continue && !policy.should_continue(result, attempt)) {
            break;
        }
        
        if (policy.escalate_on_failure && 
            static_cast<int>(current_level) < static_cast<int>(policy.max_level)) {
            current_level = static_cast<ResetLevel>(static_cast<int>(current_level) + 1);
            TETHER_LOGI(TAG, "Escalating to level %s", getResetLevelName(current_level));
        }
        
        Tether::Platform::Clock::instance().delayMilliseconds(policy.retry_delay_ms);
    }
    
    result.success = false;
    result.error_message = "Reset policy exhausted all attempts";
    return result;
}

} // namespace EtherCAT
