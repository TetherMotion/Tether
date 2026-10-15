/**
 * @file value_text.hpp
 * @brief Text→SchemaValue encoding for the tether_io_client CLI/TUI.
 *
 * Grammar:
 *   - leading '{' or '[' → JSON (glaze generic) mapped to the schema
 *   - bare literal → scalar number/bool, "quoted string", hex:ABCD bytes,
 *     enum name or #index, none/null → absent optional
 */
#pragma once

#include "tether/io/SchemaValue.hpp"
#include "tether/io/TetherIOClient.hpp"

#include <expected>
#include <string>
#include <vector>

namespace tether::io::client {

/// Parse a text literal into a SchemaValue against `node`.
/// For struct/array/map/oneof/optional schemas, JSON is required unless the
/// bare-literal shorthand applies (scalar/string/bytes).
std::expected<SchemaValue, std::string>
parseValueLiteral(const SchemaGraph& graph, const SchemaNode& node,
                  const std::string& text);

/// Parse a function argument in `name=value` or `#key=value` or positional
/// `value` form — returns the encoded wire bytes plus the position/key
/// fields needed by `FunctionArg`.
struct ParsedArg {
    uint32_t position = 0;           ///< index into fn.parameters
    uint32_t key = 0;                ///< declared param key (0 = positional)
    ValueType type = ValueType::Binary;
    std::vector<uint8_t> value;      ///< wire-encoded bytes
};

/// Parse `param=value` against a function's parameter list.
std::expected<ParsedArg, std::string>
parseFunctionArg(const SchemaGraph& graph,
                 const ClientFunctionEntry& fn,
                 const std::vector<SchemaManifestEntry>& manifest,
                 const std::string& text,
                 uint32_t positionalIndex);

} // namespace tether::io::client
