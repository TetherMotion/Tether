#include "tether/ethercat/DCConfigurationValidation.hpp"
#include <sstream>
#include <iomanip>
#include <cmath>

namespace EtherCAT {
namespace DC {

DCValidationResult DCConfigurationValidation::validate(const DCConfig& config) {
    std::stringstream ss;
    bool valid = true;
    bool first_error = true;

    auto addError = [&](const std::string& msg) {
        if (!first_error) ss << "; ";
        ss << msg;
        valid = false;
        first_error = false;
    };

    // 1. Basic loop cycle time check
    if (config.cycle_period_us == 0) {
        addError("Master cycle period is 0");
    }

    // 2. Validate SYNC0
    if (config.enable_sync0) {
        if (config.sync0_cycle_time_ns == 0) {
            addError("SYNC0 enabled but cycle time is 0 (register 0x9A0 will be 0, disabling pulse)");
        }
        
        // Sanity check: Shift should not be larger than cycle
        if (config.sync0_cycle_time_ns > 0 && 
            std::abs(config.sync0_shift_ns) >= (int32_t)config.sync0_cycle_time_ns) {
            ss << (first_error ? "" : "; ") << "SYNC0 shift (" << config.sync0_shift_ns 
               << " ns) >= cycle time (" << config.sync0_cycle_time_ns << " ns)";
             valid = false;
             first_error = false;
        }
    }

    // 3. Validate SYNC1
    if (config.enable_sync1) {
        if (config.sync1_cycle_time_ns == 0) {
            addError("SYNC1 enabled but cycle time is 0 (register 0x9A4 will be 0, disabling pulse)");
        }
        
        // Logic check: If SYNC1 is enabled, usually SYNC0 is too
        if (!config.enable_sync0) {
            addError("SYNC1 enabled without SYNC0 (unusual configuration)");
        }
    }

    return {valid, ss.str()};
}

std::string DCConfigurationValidation::toString(const DCConfig& config) {
    std::stringstream ss;
    ss << "DC Configuration:\n";
    ss << "  Master Cycle : " << config.cycle_period_us << " us (" 
       << (config.cycle_period_us > 0 ? (1000000.0/config.cycle_period_us) : 0.0) << " Hz)\n";
    ss << "  Sync Interval: " << config.sync_interval_cycles << " cycles\n";
    
    // SYNC0
    ss << "  SYNC0        : " << (config.enable_sync0 ? "ENABLED" : "DISABLED");
    if (config.enable_sync0) {
        ss << " (Cycle=" << config.sync0_cycle_time_ns << " ns";
        ss << ", Shift=" << config.sync0_shift_ns << " ns)";
    }
    ss << "\n";

    // SYNC1
    ss << "  SYNC1        : " << (config.enable_sync1 ? "ENABLED" : "DISABLED");
    if (config.enable_sync1) {
        ss << " (Cycle=" << config.sync1_cycle_time_ns << " ns)";
    }

    return ss.str();
}

} // namespace DC
} // namespace EtherCAT
