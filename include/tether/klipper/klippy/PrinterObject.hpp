#pragma once

/**
 * @file PrinterObject.hpp
 * @brief Base class for printer objects that expose status via UDS.
 */

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

/**@brief A printer object exposes named status fields. */
class PrinterObject {
public:
    virtual ~PrinterObject() = default;

    /**
     * @brief Object name (e.g. "toolhead", "extruder", "webhooks").
     * @return Name used as the key in objects/list and objects/query.
     */
    virtual std::string name() const = 0;

    /**
     * @brief Get current status fields.
     * @param fields If empty/null, return all fields. Otherwise only requested.
     * @return Map of field name to current value, filtered per @p fields.
     */
    virtual std::map<std::string, JsonValue> status(
        const std::vector<std::string>& fields) const = 0;

    /**
     * @brief List all available field names.
     * @return Every field name status() can produce.
     */
    virtual std::vector<std::string> availableFields() const = 0;

protected:
    /**
     * @brief Helper to build a status map filtered by the requested fields.
     *
     * Usage:
     * @code
     * auto result = buildStatus(fields);
     * result.add("temperature", JsonValue(heater_->currentTemp()));
     * result.add("target", JsonValue(heater_->target()));
     * return result.take();
     * @endcode
     *
     * This eliminates the copy-pasted `add` lambda + filter check boilerplate
     * that appears in 40+ printer object status() methods.
     */
    class StatusBuilder {
    public:
        /**@param fields Requested field names; empty means "all fields". */
        explicit StatusBuilder(const std::vector<std::string>& fields)
            : fields_(fields) {}

        /**
         * @brief Add a field if it's in the requested list (or if all fields
         * are requested — i.e. fields_ is empty).
         * @param name Status field name as exposed to clients.
         * @param value Current value of the field.
         */
        void add(const std::string& name, JsonValue value) {
            if (fields_.empty() ||
                std::find(fields_.begin(), fields_.end(), name) != fields_.end()) {
                result_[name] = std::move(value);
            }
        }

        /**
         * @brief Move out the built result map.
         * @return Map of the fields that survived the filter.
         */
        std::map<std::string, JsonValue> take() && {
            return std::move(result_);
        }

    private:
        const std::vector<std::string>& fields_; /**< Requested field names. */
        std::map<std::string, JsonValue> result_; /**< Fields added so far. */
    };

    /**
     * @brief Convenience: create a StatusBuilder for the given fields.
     * @param fields Requested field names; empty means "all fields".
     * @return Builder that filters adds against @p fields.
     */
    StatusBuilder buildStatus(const std::vector<std::string>& fields) const {
        return StatusBuilder(fields);
    }
};

} // namespace tether::klipper::klippy
