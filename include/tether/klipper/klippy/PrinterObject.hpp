#pragma once

/// @file PrinterObject.hpp
/// @brief Base class for printer objects that expose status via UDS.

#include "tether/klipper/klippy/JsonValue.hpp"

#include <algorithm>
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

protected:
    /// @brief Helper to build a status map filtered by the requested fields.
    ///
    /// Usage:
    /// @code
    /// auto result = buildStatus(fields);
    /// result.add("temperature", JsonValue(heater_->currentTemp()));
    /// result.add("target", JsonValue(heater_->target()));
    /// return result.take();
    /// @endcode
    ///
    /// This eliminates the copy-pasted `add` lambda + filter check boilerplate
    /// that appears in 40+ printer object status() methods.
    class StatusBuilder {
    public:
        explicit StatusBuilder(const std::vector<std::string>& fields)
            : fields_(fields) {}

        /// @brief Add a field if it's in the requested list (or if all fields
        /// are requested — i.e. fields_ is empty).
        void add(const std::string& name, JsonValue value) {
            if (fields_.empty() ||
                std::find(fields_.begin(), fields_.end(), name) != fields_.end()) {
                result_[name] = std::move(value);
            }
        }

        /// @brief Move out the built result map.
        std::map<std::string, JsonValue> take() && {
            return std::move(result_);
        }

    private:
        const std::vector<std::string>& fields_;
        std::map<std::string, JsonValue> result_;
    };

    /// @brief Convenience: create a StatusBuilder for the given fields.
    StatusBuilder buildStatus(const std::vector<std::string>& fields) const {
        return StatusBuilder(fields);
    }
};

} // namespace tether::klipper::klippy
