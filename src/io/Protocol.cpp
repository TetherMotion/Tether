#include "tether/io/Protocol.hpp"

#include <charconv>
#include <cmath>
#include <format>

namespace tether::io {

void StreamFilterSchema::defineProperty(FilterPropertyDef definition) {
    definitions_[definition.name] = std::move(definition);
}

StreamFilterSchema::Result StreamFilterSchema::validate(
    const FilterProperty& property) const {
    const auto it = definitions_.find(property.name);
    if (it == definitions_.end()) {
        return {false, FilterPropertyErrorType::ParameterUnknown,
                std::format("Unknown stream filter property '{}'.", property.name)};
    }
    const auto& definition = it->second;
    if (!definition.implemented) {
        return {false, FilterPropertyErrorType::NotImplemented,
                std::format("Stream filter property '{}' is not implemented.", property.name)};
    }
    if (definition.valueType != property.value.type) {
        return {false, FilterPropertyErrorType::WrongDataType,
                std::format("Stream filter property '{}' has the wrong type.", property.name)};
    }

    if (!definition.hasRange) return {true, FilterPropertyErrorType::None, {}};

    double value = 0.0;
    switch (property.value.type) {
        case ValueType::U8:
            if (property.value.data.size() != 1) break;
            value = property.value.data[0];
            goto check_range;
        case ValueType::U16:
            if (property.value.data.size() != 2) break;
            value = static_cast<double>(property.value.data[0] |
                                        (property.value.data[1] << 8));
            goto check_range;
        case ValueType::U32:
            if (property.value.data.size() != 4) break;
            value = property.value.data[0] |
                    (property.value.data[1] << 8) |
                    (property.value.data[2] << 16) |
                    (property.value.data[3] << 24);
            goto check_range;
        case ValueType::I8:
            if (property.value.data.size() != 1) break;
            value = static_cast<int8_t>(property.value.data[0]);
            goto check_range;
        case ValueType::I16:
            if (property.value.data.size() != 2) break;
            value = static_cast<int16_t>(property.value.data[0] |
                                         (property.value.data[1] << 8));
            goto check_range;
        case ValueType::I32: {
            if (property.value.data.size() != 4) break;
            uint32_t raw = property.value.data[0] |
                           (property.value.data[1] << 8) |
                           (property.value.data[2] << 16) |
                           (property.value.data[3] << 24);
            value = static_cast<int32_t>(raw);
            goto check_range;
        }
        case ValueType::F32: {
            if (property.value.data.size() != 4) break;
            uint32_t raw = property.value.data[0] |
                           (property.value.data[1] << 8) |
                           (property.value.data[2] << 16) |
                           (property.value.data[3] << 24);
            float decoded;
            std::memcpy(&decoded, &raw, sizeof(decoded));
            value = decoded;
            goto check_range;
        }
        case ValueType::F64: {
            if (property.value.data.size() != 8) break;
            uint64_t raw = 0;
            for (size_t index = 0; index < 8; ++index) {
                raw |= static_cast<uint64_t>(property.value.data[index]) << (8 * index);
            }
            double decoded;
            std::memcpy(&decoded, &raw, sizeof(decoded));
            value = decoded;
            goto check_range;
        }
        default:
            return {false, FilterPropertyErrorType::InvalidValue,
                    "A range requires a numeric filter value."};
    }
    return {false, FilterPropertyErrorType::InvalidValue,
            std::format("Invalid value for stream filter property '{}'.", property.name)};

check_range:
    if (!std::isfinite(value) || value < definition.minValue || value > definition.maxValue) {
        return {false, FilterPropertyErrorType::OutsideRange,
                std::format("Stream filter property '{}' is outside its allowed range.", property.name)};
    }
    return {true, FilterPropertyErrorType::None, {}};
}

bool StreamFilterSchema::empty() const {
    return definitions_.empty();
}

} // namespace tether::io
