/**
 * @file ParameterFormat.hpp
 * @brief Parse Klipper printf-style command/response format strings.
 *
 * @details
 * Commands and responses are declared with printf-style format strings, e.g.:
 *   "update_digital_out oid=%c value=%c"
 *   "queue_step oid=%c interval=%u count=%hu add=%hi"
 *   "spi_transfer oid=%c data=%*s"
 *
 * The first whitespace-delimited token is the message name; the remainder is a
 * sequence of "name=%spec" parameters. The format specifier determines the
 * parameter type and how it is encoded on the wire:
 *
 *   %u   uint32   (unsigned, max 5 bytes)
 *   %i   int32    (signed, max 5 bytes)
 *   %hu  uint16   (unsigned, max 3 bytes)
 *   %hi  int16    (signed, max 3 bytes)
 *   %c   byte     (unsigned, max 2 bytes)
 *   %s   string   (length-prefixed raw bytes)
 *   %*s  buffer   (length-prefixed raw bytes)
 *   %.*s progmem_buf (length-prefixed raw bytes)
 *
 * All integer types use the same signed VLQ encoding (see Vlq.hpp); the
 * declared type is documentation for the expected value range. String/buffer
 * types use a VLQ-encoded length prefix followed by raw bytes.
 */

#pragma once

#include <cstdint>
#include <cstddef>
#include <string>
#include <string_view>
#include <vector>
#include <optional>
#include <span>

namespace tether::klipper::protocol {

/// @brief Parameter wire type (derived from the format specifier).
enum class ParamType : uint8_t {
    Uint32,   ///< %u
    Int32,    ///< %i
    Uint16,   ///< %hu
    Int16,    ///< %hi
    Byte,     ///< %c
    String,   ///< %s
    Buffer,   ///< %*s
    ProgmemBuf, ///< %.*s
};

/// @brief Whether a parameter is an integer (VLQ) or a string/buffer.
inline bool isIntegerType(ParamType t) {
    return t != ParamType::String && t != ParamType::Buffer && t != ParamType::ProgmemBuf;
}

/// @brief A single declared parameter in a format string.
struct ParamSpec {
    std::string name;     ///< Parameter name (e.g. "oid", "value")
    ParamType type;       ///< Wire type
};

/// @brief A parsed format string: message name + ordered parameter list.
struct FormatSpec {
    std::string name;             ///< Message name (first token)
    std::vector<ParamSpec> params; ///< Ordered parameters

    /// @return The canonical format string (e.g. "queue_step oid=%c interval=%u count=%hu add=%hi").
    std::string toString() const;

    /// @return Number of parameters.
    size_t arity() const { return params.size(); }
};

/**
 * @brief Parse a printf-style format string into a FormatSpec.
 *
 * Accepts strings like "update_digital_out oid=%c value=%c" or
 * "queue_step oid=%c interval=%u count=%hu add=%hi".
 *
 * @return Parsed spec, or std::nullopt if the string is malformed.
 */
std::optional<FormatSpec> parseFormatString(std::string_view fmt);

/**
 * @brief Encode a single parameter value into a byte buffer.
 *
 * For integer types, @p raw is interpreted as the value's bits (int32_t for
 * signed types, uint32_t cast to int32_t for unsigned types). For string/
 * buffer types, @p str is the raw bytes (a VLQ length prefix is emitted).
 *
 * @param type   Parameter type.
 * @param intValue Integer value (for integer types).
 * @param str    String/buffer bytes (for string/buffer types).
 * @param out    Output buffer (must be large enough; <= kMaxBufferLength+5).
 * @return Number of bytes written, or 0 on error.
 */
size_t encodeParamValue(ParamType type, int32_t intValue,
                        std::span<const uint8_t> str, uint8_t* out);

/**
 * @brief Decode a single parameter value from a byte stream.
 *
 * Advances @p past the decoded parameter. For integer types the value is
 * returned in @p intValue (sign-extended; mask to 32 bits for unsigned). For
 * string/buffer types the raw bytes are returned in @p str.
 *
 * @return true on success, false on truncation.
 */
bool decodeParamValue(ParamType type, const uint8_t*& p, const uint8_t* end,
                      int32_t& intValue, std::vector<uint8_t>& str);

} // namespace tether::klipper::protocol
