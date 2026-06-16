#pragma once

#include "tether/ethercat/DC.hpp"
#include <string>

namespace EtherCAT {
namespace DC {

/**
 * @brief Result of DC configuration validation
 */
struct DCValidationResult {
    bool valid;
    std::string error_message;
};

/**
 * @brief Helper class to validate and format DC configuration
 * 
 * Provides human-readable output of DC configuration and checks for
 * common configuration errors (zero cycle times when enabled, invalid shifts).
 */
class DCConfigurationValidation {
public:
    /**
     * @brief Validate a DC configuration structure
     * @param config The configuration to check
     * @return Validation result (valid=true if OK)
     */
    static DCValidationResult validate(const DCConfig& config);
    
    /**
     * @brief Create a human-readable string representation of the config
     * @param config The configuration to format
     * @return Multi-line string describing the configuration
     */
    static std::string toString(const DCConfig& config);
};

} // namespace DC
} // namespace EtherCAT
