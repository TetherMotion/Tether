#pragma once

#include "tether/io/Protocol.hpp"
#include "tether/io/BinaryStruct.hpp"

#include <cstdint>
#include <functional>
#include <map>
#include <string>
#include <string_view>
#include <vector>
#include <memory>

namespace tether::io {

inline constexpr size_t FUNCTION_TLV_HEADER_SIZE = 4 + 1 + 4;

inline constexpr uint32_t MAX_AGGREGATE_ELEMENTS = 65536;
inline constexpr uint32_t MAX_AGGREGATE_FIELDS = 65536;
inline constexpr uint32_t MAX_AGGREGATE_DEPTH = 16;

/// Recursive value schema used by function arguments, returns, and streams.
struct ValueDescriptor {
    ValueType type = ValueType::Binary;
    std::shared_ptr<const ValueDescriptor> element;
    std::vector<std::pair<std::string, std::shared_ptr<const ValueDescriptor>>> fields;

    static ValueDescriptor scalar(ValueType valueType) {
        ValueDescriptor result;
        result.type = valueType;
        return result;
    }

    static ValueDescriptor array(ValueDescriptor elementType) {
        ValueDescriptor result;
        result.type = ValueType::Array;
        result.element = std::make_shared<ValueDescriptor>(std::move(elementType));
        return result;
    }

    static ValueDescriptor structure(
        std::vector<std::pair<std::string, ValueDescriptor>> members) {
        ValueDescriptor result;
        result.type = ValueType::Struct;
        for (auto& [name, descriptor] : members) {
            result.fields.emplace_back(std::move(name),
                                       std::make_shared<ValueDescriptor>(std::move(descriptor)));
        }
        return result;
    }

    bool valid(uint32_t depth = 0) const {
        if (depth > MAX_AGGREGATE_DEPTH) return false;
        if (type == ValueType::Array) {
            return element && element->valid(depth + 1);
        }
        if (type == ValueType::Struct) {
            if (fields.size() > MAX_AGGREGATE_FIELDS) return false;
            for (const auto& [name, field] : fields) {
                if (name.empty() || !field || !field->valid(depth + 1)) return false;
            }
        }
        if (type != ValueType::Array && type != ValueType::Struct &&
            (element || !fields.empty())) return false;
        return valueTypeSize(type) != 0 || isVariableLength(type) || type == ValueType::Stream;
    }
};

inline bool descriptorContains(const ValueDescriptor& descriptor, ValueType type) {
    if (descriptor.type == type) return true;
    if (descriptor.element && descriptorContains(*descriptor.element, type)) return true;
    for (const auto& [name, field] : descriptor.fields) {
        if (field && descriptorContains(*field, type)) return true;
    }
    return false;
}

inline bool valueDescriptorWireSize(const ValueDescriptor& descriptor, size_t& size,
                                    uint32_t depth = 0) {
    if (depth > MAX_AGGREGATE_DEPTH || !descriptor.valid(depth) || size > MAX_MESSAGE_SIZE ||
        MAX_MESSAGE_SIZE - size < 1) return false;
    size += 1;
    if (descriptor.type == ValueType::Array) {
        return valueDescriptorWireSize(*descriptor.element, size, depth + 1);
    }
    if (descriptor.type == ValueType::Struct) {
        if (MAX_MESSAGE_SIZE - size < 4) return false;
        size += 4;
        for (const auto& [name, field] : descriptor.fields) {
            if (name.size() > UINT16_MAX || MAX_MESSAGE_SIZE - size < 2 ||
                MAX_MESSAGE_SIZE - size - 2 < name.size()) {
                return false;
            }
            size += 2 + name.size();
            if (!valueDescriptorWireSize(*field, size, depth + 1)) return false;
        }
    }
    return true;
}

inline bool encodeValueDescriptor(BufWriter& writer, const ValueDescriptor& descriptor,
                                  uint32_t depth = 0) {
    if (depth > MAX_AGGREGATE_DEPTH || !descriptor.valid(depth)) return false;
    writer.putU8(static_cast<uint8_t>(descriptor.type));
    if (descriptor.type == ValueType::Array) {
        if (!descriptor.element ||
            !encodeValueDescriptor(writer, *descriptor.element, depth + 1)) return false;
    } else if (descriptor.type == ValueType::Struct) {
        if (descriptor.fields.size() > MAX_AGGREGATE_FIELDS) return false;
        writer.putU32(static_cast<uint32_t>(descriptor.fields.size()));
        for (const auto& [name, field] : descriptor.fields) {
            writer.putStr16(name.c_str(), name.size());
            if (!field || !encodeValueDescriptor(writer, *field, depth + 1)) return false;
        }
    }
    return writer.ok();
}

inline bool decodeValueDescriptor(BufReader& reader, ValueDescriptor& descriptor,
                                  uint32_t depth = 0) {
    if (depth > MAX_AGGREGATE_DEPTH) { reader.error = true; return false; }
    descriptor = {};
    descriptor.type = static_cast<ValueType>(reader.getU8());
    if (!reader.ok()) return false;
    if (descriptor.type == ValueType::Array) {
        auto element = std::make_shared<ValueDescriptor>();
        if (!decodeValueDescriptor(reader, *element, depth + 1)) return false;
        descriptor.element = std::move(element);
    } else if (descriptor.type == ValueType::Struct) {
        const uint32_t count = reader.getU32();
        if (!reader.ok() || count > MAX_AGGREGATE_FIELDS) return false;
        descriptor.fields.reserve(count);
        for (uint32_t index = 0; index < count; ++index) {
            const uint16_t nameLength = reader.getU16();
            const uint8_t* name = reader.getBytes(nameLength);
            if (!reader.ok()) return false;
            auto field = std::make_shared<ValueDescriptor>();
            if (!decodeValueDescriptor(reader, *field, depth + 1)) return false;
            descriptor.fields.emplace_back(
                std::string(reinterpret_cast<const char*>(name), nameLength), std::move(field));
        }
    }
    return descriptor.valid(depth);
}

inline bool validateValuePayload(const ValueDescriptor& descriptor,
                                 const uint8_t* data, size_t length,
                                 uint32_t depth = 0) {
    if (depth > MAX_AGGREGATE_DEPTH || !descriptor.valid(depth) ||
        (length != 0 && data == nullptr)) return false;
    const size_t fixedSize = valueTypeSize(descriptor.type);
    if (descriptor.type != ValueType::Array && descriptor.type != ValueType::Struct) {
        if (fixedSize != 0) return length == fixedSize;
        if (descriptor.type == ValueType::UVarint || descriptor.type == ValueType::IVarint ||
            descriptor.type == ValueType::Enum) {
            uint32_t value = 0;
            return decodeVarint(data, length, value) != 0 &&
                   decodeVarint(data, length, value) == length;
        }
        return true;
    }

    BufReader reader(data, length);
    const uint32_t count = reader.getU32();
    if (!reader.ok() || count > MAX_AGGREGATE_ELEMENTS) return false;
    if (descriptor.type == ValueType::Array) {
        if (!descriptor.element || count > length) return false;
        for (uint32_t index = 0; index < count; ++index) {
            const uint32_t childLength = reader.getU32();
            const uint8_t* child = reader.getBytes(childLength);
            if (!reader.ok() || !validateValuePayload(*descriptor.element, child,
                                                       childLength, depth + 1)) return false;
        }
    } else {
        if (count != descriptor.fields.size()) return false;
        std::vector<bool> seen(count, false);
        for (uint32_t index = 0; index < count; ++index) {
            const uint32_t position = reader.getU32();
            const auto type = static_cast<ValueType>(reader.getU8());
            const uint32_t childLength = reader.getU32();
            const uint8_t* child = reader.getBytes(childLength);
            if (!reader.ok() || position >= count || seen[position] ||
                type != descriptor.fields[position].second->type ||
                !validateValuePayload(*descriptor.fields[position].second, child,
                                      childLength, depth + 1)) return false;
            seen[position] = true;
        }
    }
    return reader.ok() && reader.remaining() == 0;
}

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
inline constexpr uint8_t HasAggregate = 0x10;
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
    std::shared_ptr<const ValueDescriptor> valueDescriptor;
    uint32_t maxValueSize = 0;
    std::map<std::string, std::string> metadata;

    uint8_t flags() const {
        uint8_t result = 0;
        if (optional) result |= FunctionParameterFlags::Optional;
        if (valueDescriptor && descriptorContains(*valueDescriptor, ValueType::Array)) {
            result |= FunctionParameterFlags::HasAggregate;
        }
        if (hasDefault) result |= FunctionParameterFlags::HasDefault;
        if (enumReference != 0) result |= FunctionParameterFlags::HasEnum;
        if (structDescriptor != nullptr || structReference != 0 ||
            (valueDescriptor && descriptorContains(*valueDescriptor, ValueType::Struct))) {
            result |= FunctionParameterFlags::HasStruct;
        }
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
    std::shared_ptr<const ValueDescriptor> valueDescriptor;
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
            if (parameter.valueDescriptor &&
                (parameter.valueDescriptor->type != parameter.type ||
                 !parameter.valueDescriptor->valid())) return false;
            if (parameter.optional) optionalSeen = true;
            else if (optionalSeen) return false;
            if (parameter.optional && !parameter.hasDefault) return false;
            if (parameter.hasDefault && parameter.valueDescriptor &&
                !validateValuePayload(*parameter.valueDescriptor, parameter.defaultValue.data(),
                                      parameter.defaultValue.size())) return false;
        }
        if (returnValue.present && returnValue.valueDescriptor &&
            (returnValue.valueDescriptor->type != returnValue.type ||
             !returnValue.valueDescriptor->valid())) return false;
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