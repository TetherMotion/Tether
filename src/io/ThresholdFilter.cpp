/**
 * @file ThresholdFilter.cpp
 * @brief Threshold-based change detection implementation.
 * @copyright Copyright (C) 2025-2026 Tether Authors
 */
#include "tether/io/ThresholdFilter.hpp"
#include <cmath>
#include <cstring>

namespace tether { namespace io {

const ThresholdRule* ThresholdFilter::findRule(uint64_t entryId) const {
    // Look for a specific rule for this entry
    for (const auto& r : config_.rules) {
        if (r.entryId == entryId) return &r;
    }
    // Fall back to the default rule (entryId == 0)
    for (const auto& r : config_.rules) {
        if (r.entryId == 0) return &r;
    }
    return nullptr;
}

bool ThresholdFilter::passes(uint64_t entryId, const void* oldVal,
                              const void* newVal, size_t valSize) const {
    if (config_.rules.empty()) return true;  // No rules = always pass

    const ThresholdRule* rule = findRule(entryId);
    if (!rule) {
        // No matching rule: depends on whitelist/blacklist mode
        return !config_.isWhitelist;  // whitelist: exclude; blacklist: include
    }

    switch (rule->type) {
        case ThresholdType::None:
            return true;

        case ThresholdType::Absolute: {
            // Compare as double for numeric types
            double oldD = 0.0, newD = 0.0;
            if (valSize == 4) {
                float oF, nF;
                std::memcpy(&oF, oldVal, 4);
                std::memcpy(&nF, newVal, 4);
                oldD = oF;
                newD = nF;
            } else if (valSize == 8) {
                std::memcpy(&oldD, oldVal, 8);
                std::memcpy(&newD, newVal, 8);
            } else {
                // For integer types, compare raw bytes
                return std::memcmp(oldVal, newVal, valSize) != 0;
            }
            return std::fabs(newD - oldD) > rule->threshold;
        }

        case ThresholdType::Relative: {
            double oldD = 0.0, newD = 0.0;
            if (valSize == 4) {
                float oF, nF;
                std::memcpy(&oF, oldVal, 4);
                std::memcpy(&nF, newVal, 4);
                oldD = oF;
                newD = nF;
            } else if (valSize == 8) {
                std::memcpy(&oldD, oldVal, 8);
                std::memcpy(&newD, newVal, 8);
            } else {
                return std::memcmp(oldVal, newVal, valSize) != 0;
            }
            if (std::fabs(oldD) < 1e-15) return std::fabs(newD) > rule->threshold;
            return std::fabs((newD - oldD) / oldD) > rule->threshold;
        }

        case ThresholdType::Custom: {
            auto it = customEvaluators_.find(rule->customName);
            if (it != customEvaluators_.end()) {
                return it->second(entryId, oldVal, newVal, valSize, rule->customConfig);
            }
            return true;  // Unknown custom evaluator = pass
        }
    }

    return true;
}

void ThresholdFilter::reset() {
    // No persistent state in the filter itself; per-entry state is in Session
}

}} // namespace tether::io
