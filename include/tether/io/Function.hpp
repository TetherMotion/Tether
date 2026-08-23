#pragma once

#include "tether/io/Protocol.hpp"
#include "tether/io/BinaryStruct.hpp"

#include <cstdint>
#include <functional>
#include <map>
#include <string>
#include <string_view>
#include <vector>

namespace tether::io {

inline constexpr size_t FUNCTION_TLV_HEADER_SIZE = 4 + 1 + 4;

/// Encodes one function argument as [position U32][type U8][length U32][value].
inline bool encodeFunctionTlv(BufWriter& writer, uint32_t position,
                              ValueType type, const uint8_t* value, size_t length) {
    if (length > MAX_VARIABLE_VALUE_SIZE || length > UINT32_MAX) {
        return false;
    }
    writer.putU32(position);
    writer.putU8(static_cast<uint8_t>(type));
    writer.putU32(static_cast<uint32_t>(length));
    writer.putBytes(value, length);
    return writer.ok();
}

/// Flags describing an annotated positional function argument.
namespace FunctionParameterFlags {
inline constexpr uint8_t Optional = 0x01;
inline constexpr uint8_t HasDefault = 0x02;
inline constexpr uint8_t HasEnum = 0x04;
inline constexpr uint8_t HasStruct = 0x08;
} // namespace FunctionParameterFlags

struct FunctionParameter {
    std::string name;
    std::string description;
    ValueType type = ValueType::Binary;
    bool optional = false;
    bool hasDefault = false;
    std::vector<uint8_t> defaultValue;
    uint64_t enumReference = 0;
    uint64_t structReference = 0;
    const StructDescriptor* structDescriptor = nullptr;
    uint32_t maxValueSize = 0;
    std::map<std::string, std::string> metadata;

    uint8_t flags() const {
        uint8_t result = 0;
        if (optional) result |= FunctionParameterFlags::Optional;
        if (hasDefault) result |= FunctionParameterFlags::HasDefault;
        if (enumReference != 0) result |= FunctionParameterFlags::HasEnum;
        if (structDescriptor != nullptr || structReference != 0) result |= FunctionParameterFlags::HasStruct;
        return result;
    }
};

struct FunctionReturn {
    bool present = false;
    std::string name;
    std::string description;
    ValueType type = ValueType::Binary;
    uint64_t enumReference = 0;
    uint64_t structReference = 0;
    const StructDescriptor* structDescriptor = nullptr;
    uint32_t maxValueSize = 0;
    std::map<std::string, std::string> metadata;
};

struct FunctionArgument {
    uint32_t position = 0;
    ValueType type = ValueType::Binary;
    std::vector<uint8_t> value;
    bool provided = false;
};

inline bool decodeFunctionTlv(BufReader& reader, FunctionArgument& argument) {
    argument.position = reader.getU32();
    argument.type = static_cast<ValueType>(reader.getU8());
    const uint32_t length = reader.getU32();
    if (!reader.ok() || length > MAX_VARIABLE_VALUE_SIZE || length > reader.remaining()) {
        reader.error = true;
        return false;
    }
    const uint8_t* value = reader.getBytes(length);
    if (!reader.ok()) return false;
    argument.value.assign(value, value + length);
    argument.provided = true;
    return true;
}

struct FunctionCallResult {
    bool success = false;
    ErrorCode error = ErrorCode::FunctionInvocationError;
    std::string errorMessage;
    std::vector<uint8_t> returnValue;
};

using FunctionCallback = std::function<FunctionCallResult(
    const std::vector<FunctionArgument>& arguments)>;

/// A fully annotated callable function in the IO registry.
struct FunctionEntry {
    uint64_t id = 0;
    std::string name;
    std::string description;
    std::string group;
    std::vector<FunctionParameter> parameters;
    FunctionReturn returnValue;
    std::map<std::string, std::string> metadata;
    FunctionCallback callback;

    /// Functions use positional calls. Once an optional argument appears,
    /// every subsequent argument must also be optional. Optional arguments
    /// must carry a default because omitted positions are materialized locally.
    bool validSignature() const {
        bool optionalSeen = false;
        for (const auto& parameter : parameters) {
            if (parameter.name.empty()) return false;
            if (parameter.optional) optionalSeen = true;
            else if (optionalSeen) return false;
            if (parameter.optional && !parameter.hasDefault) return false;
        }
        return !name.empty() && static_cast<bool>(callback);
    }

    size_t requiredParameterCount() const {
        size_t count = 0;
        for (const auto& parameter : parameters) {
            if (!parameter.optional) ++count;
        }
        return count;
    }
};

class FunctionView {
public:
    FunctionView() = default;
    explicit FunctionView(const FunctionEntry* function) : function_(function) {}

    explicit operator bool() const { return function_ != nullptr; }
    const FunctionEntry* get() const { return function_; }
    uint64_t id() const { return function_->id; }
    std::string_view name() const { return function_->name; }
    std::string_view description() const { return function_->description; }
    std::string_view group() const { return function_->group; }
    const std::vector<FunctionParameter>& parameters() const { return function_->parameters; }
    const FunctionReturn& returnValue() const { return function_->returnValue; }
    size_t requiredParameterCount() const { return function_->requiredParameterCount(); }
    size_t parameterCount() const { return function_->parameters.size(); }
    size_t metadataCount() const { return function_->metadata.size(); }
    const std::map<std::string, std::string>& metadata() const { return function_->metadata; }

    template<typename Fn>
    void forEachMetadata(Fn&& fn) const {
        for (const auto& [key, value] : function_->metadata) {
            fn(std::string_view(key), std::string_view(value));
        }
    }

    FunctionCallResult invoke(const std::vector<FunctionArgument>& arguments) const {
        return function_->callback(arguments);
    }

private:
    const FunctionEntry* function_ = nullptr;
};

} // namespace tether::io