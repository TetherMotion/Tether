#pragma once

#include "tether/ethercat/PDOManager.hpp"
#include <vector>
#include <string>
#include <array>
#include <algorithm>

namespace EtherCAT {

/**
 * @brief Result of Sync Manager validation
 */
struct SyncManagerValidationResult {
    bool valid;                     ///< Whether the configuration is valid
    std::string error_message;      ///< Error description if invalid, empty if valid
};

/**
 * @brief Validates Sync Manager configurations
 * 
 * Enforces strict non-overlapping memory rules and user-specified default configurations
 * for master/slave mailbox and buffered I/O.
 */
class SyncManagerValidation {
public:
    /**
     * @brief Validate a set of Sync Manager configurations.
     * 
     * Applies the following rules:
     * 1. Check for memory overlap between ANY enabled Sync Managers (Critical Error).
     * 2. SM0 (if enabled): Must comprise valid Mailbox Write config (Master->Slave).
     *    - Control byte must be 0x26 (MAILBOX | DIR_WRITE | IRQ_PDI | REPEAT_REQ).
     * 3. SM1 (if enabled): Must comprise valid Mailbox Read config (Slave->Master).
     *    - Control byte must be 0x22 (MAILBOX | DIR_READ | IRQ_PDI | REPEAT_REQ).
     * 4. SM2 (if enabled): Must be valid Inputs (Slave->Master).
     *    - Control byte must be 0x64 (BUFFERED | DIR_WRITE | IRQ_PDI | WATCHDOG).
     *    - NOTE: User refers to this as 'Outputs' but control implies Input direction.
     * 5. SM3 (if enabled): Must be valid Outputs (Master->Slave).
     *    - Control byte must be 0x20 (BUFFERED | DIR_READ | IRQ_PDI).
     *    - NOTE: User refers to this as 'Inputs' but control implies Output direction.
     * 
     * Addresses and Lengths are validated only for overlap and bounds, not specific values.
     * 
     * @param configs Vector of SM configs (typically 4)
     * @return ValidationResult
     */
    static SyncManagerValidationResult validate(const std::vector<EtherCAT::PDO::SyncManagerConfig>& configs);
    
    // Convenience for array
    template<size_t N>
    static SyncManagerValidationResult validate(const std::array<EtherCAT::PDO::SyncManagerConfig, N>& configs) {
        std::vector<EtherCAT::PDO::SyncManagerConfig> vec(configs.begin(), configs.end());
        return validate(vec);
    }

private:
    static bool checkOverlap(const EtherCAT::PDO::SyncManagerConfig& a, const EtherCAT::PDO::SyncManagerConfig& b);
};

} // namespace EtherCAT
