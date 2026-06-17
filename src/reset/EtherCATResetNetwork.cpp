/**
 * @file EtherCATResetNetwork.cpp
 * @brief Network-wide reset functions, broadcast operations, and reset policy
 * 
 * Split from EtherCATReset.cpp for maintainability.
 * 
 * NOTE: Network-wide functions are temporarily disabled due to circular dependency
 * with Master class. These will be reimplemented in a separate module that
 * doesn't depend on Reset.hpp.
 */

#include "Reset.hpp"
#include "tether/platform/EspCompat.hpp"
#include "tether/ethercat/CoEManager.hpp"

namespace EtherCAT {

static const char* TAG = "ECAT_RESET_NET";

// ============================================================================
// Broadcast/Network-Wide Functions
// ============================================================================
// Temporarily disabled - see note above

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
