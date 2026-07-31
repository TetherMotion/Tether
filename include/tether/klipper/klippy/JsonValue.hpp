#pragma once

/// @file JsonValue.hpp
/// @brief Lightweight JSON value type for the UDS protocol.

#include <cstdint>
#include <map>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace tether::klipper::klippy {

// ============================================================================
// JSON value type (lightweight, avoids external JSON dependency in header)
// ============================================================================

/// @brief Lightweight JSON value for the UDS protocol.
class JsonValue {
public:
    enum class Type { Null, Bool, Int, Double, String, Array, Object };

    JsonValue() : type_(Type::Null) {}
    JsonValue(bool b) : type_(Type::Bool), bool_(b) {}
    JsonValue(int i) : type_(Type::Int), int_(i) {}
    JsonValue(int64_t i) : type_(Type::Int), int_(i) {}
    JsonValue(double d) : type_(Type::Double), double_(d) {}
    JsonValue(const char* s) : type_(Type::String), str_(s) {}
    JsonValue(std::string s) : type_(Type::String), str_(std::move(s)) {}
    JsonValue(std::vector<JsonValue> a) : type_(Type::Array), arr_(std::move(a)) {}
    JsonValue(std::map<std::string, JsonValue> o) : type_(Type::Object), obj_(std::move(o)) {}

    Type type() const { return type_; }
    bool isNull() const { return type_ == Type::Null; }
    bool isBool() const { return type_ == Type::Bool; }
    bool isInt() const { return type_ == Type::Int; }
    bool isDouble() const { return type_ == Type::Double; }
    bool isString() const { return type_ == Type::String; }
    bool isArray() const { return type_ == Type::Array; }
    bool isObject() const { return type_ == Type::Object; }

    bool asBool() const { return bool_; }
    int64_t asInt() const { return int_; }
    double asDouble() const { return type_ == Type::Int ? static_cast<double>(int_) : double_; }
    const std::string& asString() const { return str_; }
    std::vector<JsonValue>& asArray() { return arr_; }
    std::map<std::string, JsonValue>& asObject() { return obj_; }
    const std::vector<JsonValue>& asArray() const { return arr_; }
    const std::map<std::string, JsonValue>& asObject() const { return obj_; }

    /// @brief Serialise to compact JSON string.
    std::string dump() const;

    /// @brief Parse a JSON string into a JsonValue.
    static std::optional<JsonValue> parse(std::string_view json);

    /// @brief Get a field from an object, or nullptr if not present.
    const JsonValue* find(std::string_view key) const;

    /// @brief Check if object has a key.
    bool has(std::string_view key) const {
        return find(key) != nullptr;
    }

private:
    Type type_ = Type::Null;
    bool bool_ = false;
    int64_t int_ = 0;
    double double_ = 0.0;
    std::string str_;
    std::vector<JsonValue> arr_;
    std::map<std::string, JsonValue> obj_;
};

} // namespace tether::klipper::klippy
