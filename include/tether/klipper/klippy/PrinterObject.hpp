#pragma once

/// @file PrinterObject.hpp
/// @brief Base class for printer objects that expose status via UDS.

#include "tether/klipper/klippy/JsonValue.hpp"

#include <map>
#include <memory>
#include <string>
#include <vector>

namespace tether::klipper::klippy {

// ============================================================================
// Printer object model (for objects/list, objects/query, objects/subscribe)
// ============================================================================

/// @brief A printer object exposes named status fields.
class PrinterObject {
public:
    virtual ~PrinterObject() = default;

    /// @brief Object name (e.g. "toolhead", "extruder", "webhooks").
    virtual std::string name() const = 0;

    /// @brief Get current status fields.
    /// @param fields If empty/null, return all fields. Otherwise only requested.
    virtual std::map<std::string, JsonValue> status(
        const std::vector<std::string>& fields) const = 0;

    /// @brief List all available field names.
    virtual std::vector<std::string> availableFields() const = 0;
};

} // namespace tether::klipper::klippy
