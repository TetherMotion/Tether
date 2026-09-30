#include "tether/io/Protocol.hpp"

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
    if (definition.schemaKey != property.value.schemaKey) {
        return {false, FilterPropertyErrorType::WrongDataType,
                std::format("Stream filter property '{}' has the wrong schema.", property.name)};
    }
    return {true, FilterPropertyErrorType::None, {}};
}

bool StreamFilterSchema::empty() const {
    return definitions_.empty();
}

} // namespace tether::io
