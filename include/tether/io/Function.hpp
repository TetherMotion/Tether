#pragma once

#include "tether/io/MachineControl.hpp"
#include "tether/io/Protocol.hpp"
#include "tether/io/Schema.hpp"

#include <algorithm>
#include <cstdint>
#include <functional>
#include <map>
#include <string>
#include <string_view>
#include <vector>
#include <memory>

namespace tether::io {

inline constexpr size_t FUNCTION_VALUE_HEADER_MAX_SIZE = MAX_VARINT_SIZE * 2;

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

/// Encodes one V6 function field as [field_key varuint][length varuint][value].
inline bool encodeFunctionValue(BufWriter& writer, uint32_t key,
                                const uint8_t* value, size_t length) {
    if (key == 0 || (length != 0 && value == nullptr)) {
        writer.overflow = true;
        return false;
    }
    if (length > MAX_VARIABLE_VALUE_SIZE || length > UINT32_MAX) {
        return false;
    }
    writer.putVarint(key);
    writer.putVarint(static_cast<uint32_t>(length));
    writer.putBytes(value, length);
    return writer.ok();
}

/// Flags describing an annotated positional function argument.
namespace FunctionParameterFlags {
inline constexpr uint8_t Optional = 0x01;
inline constexpr uint8_t HasDefault = 0x02;
inline constexpr uint8_t HasEnum = 0x04;
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
    std::shared_ptr<const ValueDescriptor> valueDescriptor;
    uint32_t maxValueSize = 0;
    std::map<std::string, std::string> metadata;
    SchemaRef schema;
    uint32_t schemaSlot = 0;
    uint32_t key = 0;

        FunctionParameter() = default;
        FunctionParameter(std::string parameterName, std::string parameterDescription,
                                            ValueType parameterType, bool parameterOptional = false,
                                            bool parameterHasDefault = false,
                                            std::vector<uint8_t> parameterDefaultValue = {})
                : name(std::move(parameterName)), description(std::move(parameterDescription)),
                    type(parameterType), optional(parameterOptional), hasDefault(parameterHasDefault),
                    defaultValue(std::move(parameterDefaultValue)) {}

    uint8_t flags() const {
        uint8_t result = 0;
        if (optional) result |= FunctionParameterFlags::Optional;
        if (valueDescriptor && descriptorContains(*valueDescriptor, ValueType::Array)) {
            result |= FunctionParameterFlags::HasAggregate;
        }
        if (hasDefault) result |= FunctionParameterFlags::HasDefault;
        if (enumReference != 0) result |= FunctionParameterFlags::HasEnum;
        return result;
    }
};

inline uint32_t functionParameterKey(const FunctionParameter& parameter, size_t position) {
    return parameter.key != 0 ? parameter.key : static_cast<uint32_t>(position + 1);
}

struct FunctionReturn {
    bool present = false;
    std::string name;
    std::string description;
    ValueType type = ValueType::Binary;
    uint64_t enumReference = 0;
    std::shared_ptr<const ValueDescriptor> valueDescriptor;
    uint32_t maxValueSize = 0;
    std::map<std::string, std::string> metadata;
    SchemaRef schema;
    uint32_t schemaSlot = 0;
};

struct FunctionArgument {
    uint32_t position = 0;
    ValueType type = ValueType::Binary;
    std::vector<uint8_t> value;
    bool provided = false;
    SchemaRef schema;
    uint32_t schemaSlot = 0;
    uint32_t key = 0;
};

inline bool decodeFunctionValue(BufReader& reader, FunctionArgument& argument) {
    argument = {};
    argument.key = reader.getVarint();
    const uint32_t length = reader.getVarint();
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

/// Invocation context supplied by the serving Session. `identity` is the
/// transport-authenticated caller (unauthenticated observer when the
/// transport did not prove one); `requestId`/`deadlineUs` are the InvokeEx
/// correlation fields (zero for CallFunctionReq).
struct FunctionInvokeContext {
    machine::SessionIdentity identity;
    uint64_t requestId = 0;
    uint64_t deadlineUs = 0;
};

using FunctionCallback = std::function<FunctionCallResult(
    const std::vector<FunctionArgument>& arguments)>;

/// Callback variant that additionally receives the caller's session context.
/// Preferred over `callback` whenever set.
using ContextualFunctionCallback = std::function<FunctionCallResult(
    const std::vector<FunctionArgument>& arguments,
    const FunctionInvokeContext& context)>;

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
    /// When set, invoked with the caller's session context instead of
    /// `callback`. Contextual functions receive the transport-authenticated
    /// identity and must enforce their own role/authority checks.
    ContextualFunctionCallback contextualCallback;

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
        return !name.empty() && (static_cast<bool>(callback) || static_cast<bool>(contextualCallback));
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

    FunctionCallResult invoke(const std::vector<FunctionArgument>& arguments,
                              const FunctionInvokeContext& context = {}) const {
        if (function_->contextualCallback)
            return function_->contextualCallback(arguments, context);
        return function_->callback(arguments);
    }

private:
    const FunctionEntry* function_ = nullptr;
};

// ---------------------------------------------------------------------------
// Peer function descriptors + symmetric invocation
// ---------------------------------------------------------------------------

/// A function signature as carried on the wire — the unit of
/// RegisterFunctionsReq and the entry format of ListFunctionsResp.
/// Unlike FunctionEntry it has no callback: invoking a peer function is a
/// remote call, not a local one.
struct FunctionDescriptor {
    uint64_t id = 0;
    std::string name;
    std::string description;
    std::string group;
    std::vector<FunctionParameter> parameters;
    FunctionReturn returnValue;
    std::map<std::string, std::string> metadata;
};

/// Parse `argumentCount` positional argument TLVs from `reader`, validate
/// them against the signature, materialize defaults for omitted optional
/// arguments, invoke the callback, and validate the return value.
/// Shared by the CallFunctionReq and InvokeExReq handlers on both ends so
/// request semantics stay identical; the caller picks the wire-level
/// error representation for result.success == false.
inline FunctionCallResult invokeFunctionChecked(const FunctionView& function,
                                                uint32_t argumentCount,
                                                BufReader& reader,
                                                const FunctionInvokeContext* context = nullptr) {
    FunctionCallResult result;
    const auto fail = [&result](const char* message) {
        result.error = ErrorCode::FunctionInvocationError;
        result.errorMessage = message;
        return result;
    };
    if (argumentCount > function.parameterCount()) {
        return fail("Too many function arguments");
    }

    std::vector<FunctionArgument> supplied(function.parameterCount());
    std::vector<bool> seen(function.parameterCount(), false);
    for (uint32_t index = 0; index < argumentCount; ++index) {
        FunctionArgument argument;
        if (!decodeFunctionValue(reader, argument)) {
            return fail("Invalid function argument value");
        }
        const auto parameterIt = std::find_if(
            function.parameters().begin(), function.parameters().end(),
            [&function, &argument](const FunctionParameter& parameter) {
                return functionParameterKey(parameter, &parameter - function.parameters().data()) ==
                       argument.key;
            });
        if (parameterIt == function.parameters().end()) {
            return fail("Unknown function argument key");
        }
        const auto position = static_cast<size_t>(parameterIt - function.parameters().begin());
        const auto& parameter = function.parameters()[position];
        if (seen[position]) {
            return fail("Duplicate function argument");
        }
        argument.position = static_cast<uint32_t>(position);
        argument.type = parameter.type;
        argument.schema = parameter.schema;
        argument.schemaSlot = parameter.schemaSlot;
        if (parameter.valueDescriptor &&
            !validateValuePayload(*parameter.valueDescriptor, argument.value.data(),
                                  argument.value.size())) {
            return fail("Invalid aggregate function argument");
        }
        const auto fixedSize = valueTypeSize(parameter.type);
        if ((fixedSize != 0 && argument.value.size() != fixedSize) ||
            (parameter.maxValueSize != 0 && argument.value.size() > parameter.maxValueSize)) {
            return fail("Invalid function argument size");
        }
        supplied[position] = std::move(argument);
        seen[position] = true;
    }
    if (!reader.ok() || reader.remaining() != 0) {
        return fail("Trailing function call data");
    }

    for (size_t position = 0; position < function.parameterCount(); ++position) {
        const auto& parameter = function.parameters()[position];
        if (seen[position]) continue;
        if (!parameter.optional || !parameter.hasDefault) {
            return fail("Missing required function argument");
        }
        auto& argument = supplied[position];
        argument.position = static_cast<uint32_t>(position);
        argument.key = functionParameterKey(parameter, position);
        argument.type = parameter.type;
        argument.schema = parameter.schema;
        argument.schemaSlot = parameter.schemaSlot;
        argument.value = parameter.defaultValue;
        argument.provided = false;
        if (parameter.valueDescriptor &&
            !validateValuePayload(*parameter.valueDescriptor, argument.value.data(),
                                  argument.value.size())) {
            return fail("Invalid default function argument");
        }
        if (parameter.maxValueSize != 0 && argument.value.size() > parameter.maxValueSize) {
            return fail("Invalid default function argument");
        }
    }

    result = function.invoke(supplied, context ? *context : FunctionInvokeContext{});
    if (!result.success) return result;

    const auto& returnValue = function.returnValue();
    const size_t fixedSize = returnValue.present ? valueTypeSize(returnValue.type) : 0;
    const bool validReturn =
        (returnValue.present && fixedSize != 0 &&
         result.returnValue.size() == fixedSize) ||
        (returnValue.present && fixedSize == 0 &&
         result.returnValue.size() <=
             (returnValue.maxValueSize != 0 ? returnValue.maxValueSize
                                            : MAX_VARIABLE_VALUE_SIZE)) ||
        (!returnValue.present && result.returnValue.empty());
    const bool validAggregateReturn =
        !returnValue.valueDescriptor ||
        validateValuePayload(*returnValue.valueDescriptor, result.returnValue.data(),
                             result.returnValue.size());
    const bool withinReturnLimit =
        returnValue.maxValueSize == 0 ||
        result.returnValue.size() <= returnValue.maxValueSize;
    if (!validReturn || !validAggregateReturn || !withinReturnLimit) {
        result.success = false;
        result.error = ErrorCode::FunctionInvocationError;
        result.errorMessage = "Invalid function return value";
        result.returnValue.clear();
    }
    return result;
}

// ---------------------------------------------------------------------------
// Function descriptor wire codec
// ---------------------------------------------------------------------------
//
// Layout (identical to one ListFunctionsResp entry):
//   [id U64][name str16][description str16][group str16]
//   [param_count U32] × {
//     [field_key U32][name str16][description str16][schema_slot U32][flags U8]
//     [default_len varint][default]                      — if flags&HasDefault
//     [metadata_count U32]([key str16][value str16])*
//   }
//   [has_return U8]([name str16][description str16][schema_slot U32]
//                    [metadata_count U32]([key str16][value str16])*)
//   [metadata_count U32]([key str16][value str16])*

namespace detail {

inline bool encodeWireValueDescriptor(BufWriter& w,
                                      const std::shared_ptr<const ValueDescriptor>& descriptor) {
    if (!descriptor) {
        w.putU8(0);
        return w.ok();
    }
    w.putU8(1);
    size_t size = 0;
    if (!valueDescriptorWireSize(*descriptor, size) || size > UINT32_MAX) return false;
    w.putU32(static_cast<uint32_t>(size));
    return encodeValueDescriptor(w, *descriptor);
}

inline bool decodeWireValueDescriptor(BufReader& r,
                                      std::shared_ptr<const ValueDescriptor>& descriptor) {
    descriptor.reset();
    if (r.getU8() == 0) return r.ok();
    const uint32_t size = r.getU32();
    const uint8_t* bytes = r.getBytes(size);
    if (!r.ok()) return false;
    BufReader sub(bytes, size);
    auto parsed = std::make_shared<ValueDescriptor>();
    if (!decodeValueDescriptor(sub, *parsed) || sub.remaining() != 0) return false;
    descriptor = std::move(parsed);
    return true;
}

inline bool encodeWireMetadata(BufWriter& w,
                               const std::map<std::string, std::string>& metadata) {
    if (metadata.size() > MAX_COLLECTION_COUNT) return false;
    w.putU32(static_cast<uint32_t>(metadata.size()));
    for (const auto& [key, value] : metadata) {
        if (key.size() > MAX_STRING_SIZE || value.size() > MAX_STRING_SIZE) return false;
        w.putStr16(key.c_str(), key.size());
        w.putStr16(value.c_str(), value.size());
    }
    return w.ok();
}

inline bool decodeWireMetadata(BufReader& r,
                               std::map<std::string, std::string>& metadata) {
    metadata.clear();
    const uint32_t count = r.getU32();
    if (!r.ok() || count > MAX_COLLECTION_COUNT) return false;
    for (uint32_t i = 0; i < count; ++i) {
        const uint16_t keyLen = r.getU16();
        const uint8_t* key = r.getBytes(keyLen);
        const uint16_t valueLen = r.getU16();
        const uint8_t* value = r.getBytes(valueLen);
        if (!r.ok()) return false;
        metadata.emplace(
            std::string(reinterpret_cast<const char*>(key), keyLen),
            std::string(reinterpret_cast<const char*>(value), valueLen));
    }
    return true;
}

inline bool encodeWireString(BufWriter& w, std::string_view value) {
    if (value.size() > MAX_STRING_SIZE) return false;
    w.putStr16(value.data(), value.size());
    return w.ok();
}

inline bool decodeWireString(BufReader& r, std::string& out) {
    const uint16_t len = r.getU16();
    const uint8_t* bytes = r.getBytes(len);
    if (!r.ok()) return false;
    out.assign(reinterpret_cast<const char*>(bytes), len);
    return true;
}

} // namespace detail

/// Encode a function descriptor in the ListFunctionsResp entry format.
inline bool encodeFunctionDescriptor(BufWriter& w, const FunctionView& function) {
    if (function.parameterCount() > MAX_COLLECTION_COUNT) return false;
    w.putU64(function.id());
    detail::encodeWireString(w, function.name());
    detail::encodeWireString(w, function.description());
    detail::encodeWireString(w, function.group());
    w.putU32(static_cast<uint32_t>(function.parameterCount()));
    for (size_t position = 0; position < function.parameterCount(); ++position) {
        const auto& parameter = function.parameters()[position];
        w.putU32(functionParameterKey(parameter, position));
        detail::encodeWireString(w, parameter.name);
        detail::encodeWireString(w, parameter.description);
        w.putU8(parameter.flags());
        w.putU32(parameter.schemaSlot);
        if (parameter.hasDefault) {
            w.putVarint(static_cast<uint32_t>(parameter.defaultValue.size()));
            w.putBytes(parameter.defaultValue.data(), parameter.defaultValue.size());
        }
        if (!detail::encodeWireMetadata(w, parameter.metadata)) return false;
    }
    const auto& result = function.returnValue();
    w.putU8(result.present ? 1 : 0);
    if (result.present) {
        detail::encodeWireString(w, result.name);
        detail::encodeWireString(w, result.description);
        w.putU32(result.schemaSlot);
        if (!detail::encodeWireMetadata(w, result.metadata)) return false;
    }
    return detail::encodeWireMetadata(w, function.metadata()) && w.ok();
}

/// Decode a descriptor produced by encodeFunctionDescriptor.
inline bool decodeFunctionDescriptor(BufReader& r, FunctionDescriptor& out) {
    out = {};
    out.id = r.getU64();
    if (!detail::decodeWireString(r, out.name) ||
        !detail::decodeWireString(r, out.description) ||
        !detail::decodeWireString(r, out.group)) return false;
    const uint32_t parameterCount = r.getU32();
    if (!r.ok() || parameterCount > MAX_COLLECTION_COUNT) return false;
    out.parameters.resize(parameterCount);
    for (auto& parameter : out.parameters) {
        parameter.key = r.getU32();
        if (parameter.key == 0 || !detail::decodeWireString(r, parameter.name) ||
            !detail::decodeWireString(r, parameter.description)) return false;
        const uint8_t flags = r.getU8();
        parameter.optional = (flags & FunctionParameterFlags::Optional) != 0;
        parameter.hasDefault = (flags & FunctionParameterFlags::HasDefault) != 0;
        parameter.schemaSlot = r.getU32();
        if (!r.ok()) return false;
        if (parameter.hasDefault) {
            const uint32_t length = r.getVarint();
            const uint8_t* bytes = r.getBytes(length);
            if (!r.ok() || length > MAX_VARIABLE_VALUE_SIZE) return false;
            parameter.defaultValue.assign(bytes, bytes + length);
        }
        if (!detail::decodeWireMetadata(r, parameter.metadata)) return false;
    }
    out.returnValue.present = r.getU8() != 0;
    if (out.returnValue.present) {
        auto& result = out.returnValue;
        if (!detail::decodeWireString(r, result.name) ||
            !detail::decodeWireString(r, result.description)) return false;
        result.schemaSlot = r.getU32();
        if (!r.ok()) return false;
        if (!detail::decodeWireMetadata(r, result.metadata)) return false;
    }
    return detail::decodeWireMetadata(r, out.metadata) && r.ok();
}

} // namespace tether::io