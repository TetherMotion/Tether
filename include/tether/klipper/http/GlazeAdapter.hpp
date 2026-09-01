#pragma once

/// @file GlazeAdapter.hpp
/// @brief Conversion between JsonValue (klippy UDS) and glz::generic (Glaze JSON).
///
/// The existing KlippyUdsServer uses a hand-written JsonValue type for all
/// endpoint handlers. The HTTP layer uses Glaze for JSON serialization at the
/// Drogon boundary. This adapter converts between the two, allowing all 120+
/// existing endpoint handlers to be reused without modification.

#include "tether/klipper/klippy/JsonValue.hpp"

#include <glaze/glaze.hpp>

#include <map>
#include <string>
#include <vector>

namespace tether::klipper::http {

/// @brief Convert a JsonValue to a glz::generic.
/// @param val The JsonValue to convert.
/// @return The equivalent glz::generic.
inline glz::generic toJsonGeneric(const klippy::JsonValue& val) {
    using namespace klippy;
    switch (val.type()) {
        case JsonValue::Type::Null:
            return glz::generic{};
        case JsonValue::Type::Bool:
            return val.asBool();
        case JsonValue::Type::Int:
            return static_cast<double>(val.asInt());
        case JsonValue::Type::Double:
            return val.asDouble();
        case JsonValue::Type::String:
            return val.asString();
        case JsonValue::Type::Array: {
            glz::generic::array_t arr;
            for (const auto& elem : val.asArray()) {
                arr.push_back(toJsonGeneric(elem));
            }
            return arr;
        }
        case JsonValue::Type::Object: {
            glz::generic::object_t obj;
            for (const auto& [key, elem] : val.asObject()) {
                obj[key] = toJsonGeneric(elem);
            }
            return obj;
        }
    }
    return glz::generic{};
}

/// @brief Convert a glz::generic to a JsonValue.
/// @param val The glz::generic to convert.
/// @return The equivalent JsonValue.
inline klippy::JsonValue fromJsonGeneric(const glz::generic& val) {
    using namespace klippy;
    if (val.is_null()) {
        return JsonValue{};
    }
    if (auto* b = val.get_if<bool>()) {
        return JsonValue(*b);
    }
    if (auto* d = val.get_if<double>()) {
        // Check if it's a whole number
        if (*d == static_cast<double>(static_cast<int64_t>(*d)) &&
            *d >= static_cast<double>(std::numeric_limits<int64_t>::min()) &&
            *d <= static_cast<double>(std::numeric_limits<int64_t>::max())) {
            return JsonValue(static_cast<int64_t>(*d));
        }
        return JsonValue(*d);
    }
    if (auto* s = val.get_if<std::string>()) {
        return JsonValue(*s);
    }
    if (auto* arr = val.get_if<glz::generic::array_t>()) {
        std::vector<JsonValue> result;
        result.reserve(arr->size());
        for (const auto& elem : *arr) {
            result.push_back(fromJsonGeneric(elem));
        }
        return JsonValue(std::move(result));
    }
    if (auto* obj = val.get_if<glz::generic::object_t>()) {
        std::map<std::string, JsonValue> result;
        for (const auto& [key, elem] : *obj) {
            result[key] = fromJsonGeneric(elem);
        }
        return JsonValue(std::move(result));
    }
    return JsonValue{};
}

/// @brief Serialize a JsonValue to a JSON string.
///
/// Uses JsonValue::dump() directly rather than going through glz::generic,
/// which avoids a known corruption issue when converting int64_t values
/// inside nested arrays/objects through the Glaze intermediate representation.
/// @param val The JsonValue to serialize.
/// @return The JSON string.
inline std::string dumpJson(const klippy::JsonValue& val) {
    return val.dump();
}

/// @brief Parse a JSON string into a JsonValue using Glaze.
/// @param json The JSON string to parse.
/// @return The parsed JsonValue, or nullopt on error.
inline std::optional<klippy::JsonValue> parseJson(std::string_view json) {
    auto rd = glz::read_json<glz::generic>(json);
    if (!rd) return std::nullopt;
    return fromJsonGeneric(*rd);
}

} // namespace tether::klipper::http
