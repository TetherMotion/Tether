/**
 * @file ParameterFormat.cpp
 * @brief Implementation of Klipper format-string parsing and parameter (de)serialisation.
 */

#include "tether/klipper/protocol/ParameterFormat.hpp"
#include "tether/klipper/protocol/Vlq.hpp"
#include "tether/klipper/protocol/Constants.hpp"

#include <cctype>
#include <cstring>

namespace tether::klipper::protocol {

std::string FormatSpec::toString() const {
    std::string s = name;
    for (const auto& p : params) {
        s += ' ';
        s += p.name;
        s += "=%";
        switch (p.type) {
            case ParamType::Uint32: s += 'u'; break;
            case ParamType::Int32:  s += 'i'; break;
            case ParamType::Uint16: s += "hu"; break;
            case ParamType::Int16:  s += "hi"; break;
            case ParamType::Byte:   s += 'c'; break;
            case ParamType::String: s += 's'; break;
            case ParamType::Buffer: s += "*s"; break;
            case ParamType::ProgmemBuf: s += ".*s"; break;
        }
    }
    return s;
}

namespace {

/// Skip whitespace in a string view, returning the new position.
size_t skipSpaces(std::string_view s, size_t i) {
    while (i < s.size() && std::isspace(static_cast<unsigned char>(s[i]))) ++i;
    return i;
}

/// Parse a "%spec" starting at position @p i (s[i-1] may be '%'); return the
/// ParamType and advance @p i past the spec.
std::optional<ParamType> parseSpec(std::string_view s, size_t& i) {
    // s[i] should be right after the '%'. Actually we call with i pointing at '%'.
    if (i >= s.size() || s[i] != '%') return std::nullopt;
    ++i; // consume '%'
    if (i >= s.size()) return std::nullopt;
    // %.*s
    if (s[i] == '.' && i + 2 < s.size() && s[i+1] == '*' && s[i+2] == 's') {
        i += 3;
        return ParamType::ProgmemBuf;
    }
    // %*s
    if (s[i] == '*' && i + 1 < s.size() && s[i+1] == 's') {
        i += 2;
        return ParamType::Buffer;
    }
    // %hu / %hi
    if (s[i] == 'h' && i + 1 < s.size()) {
        char c = s[i+1];
        if (c == 'u') { i += 2; return ParamType::Uint16; }
        if (c == 'i') { i += 2; return ParamType::Int16; }
        return std::nullopt;
    }
    // %u %i %c %s
    char c = s[i];
    ++i;
    switch (c) {
        case 'u': return ParamType::Uint32;
        case 'i': return ParamType::Int32;
        case 'c': return ParamType::Byte;
        case 's': return ParamType::String;
        default:  return std::nullopt;
    }
}

} // namespace

std::optional<FormatSpec> parseFormatString(std::string_view fmt) {
    size_t i = skipSpaces(fmt, 0);
    // Parse message name (first token).
    size_t nameStart = i;
    while (i < fmt.size() && !std::isspace(static_cast<unsigned char>(fmt[i]))) ++i;
    if (i == nameStart) return std::nullopt;
    FormatSpec spec;
    spec.name = std::string(fmt.substr(nameStart, i - nameStart));

    // Parse parameters: "name=%spec"
    while (i < fmt.size()) {
        i = skipSpaces(fmt, i);
        if (i >= fmt.size()) break;
        // Parameter name: up to '='
        size_t pNameStart = i;
        while (i < fmt.size() && fmt[i] != '=' && !std::isspace(static_cast<unsigned char>(fmt[i]))) ++i;
        if (i >= fmt.size() || fmt[i] != '=') return std::nullopt;
        ParamSpec ps;
        ps.name = std::string(fmt.substr(pNameStart, i - pNameStart));
        ++i; // consume '='
        auto t = parseSpec(fmt, i);
        if (!t) return std::nullopt;
        ps.type = *t;
        spec.params.push_back(std::move(ps));
    }
    return spec;
}

size_t encodeParamValue(ParamType type, int32_t intValue,
                        std::span<const uint8_t> str, uint8_t* out) {
    if (isIntegerType(type)) {
        return encodeParam(intValue, out);
    }
    // String/buffer: VLQ length prefix + raw bytes.
    if (str.size() > kMaxBufferLength) return 0;
    size_t n = encodeParam(static_cast<int32_t>(str.size()), out);
    for (size_t k = 0; k < str.size(); ++k) {
        out[n + k] = str[k];
    }
    return n + str.size();
}

bool decodeParamValue(ParamType type, const uint8_t*& p, const uint8_t* end,
                      int32_t& intValue, std::vector<uint8_t>& str) {
    if (isIntegerType(type)) {
        auto v = decodeParam(p, end);
        if (!v) return false;
        intValue = *v;
        return true;
    }
    // String/buffer: VLQ length + raw bytes.
    auto lenOpt = decodeParam(p, end);
    if (!lenOpt) return false;
    int32_t len = *lenOpt;
    if (len < 0) return false;
    if (p + len > end) return false;
    str.assign(p, p + len);
    p += len;
    return true;
}

} // namespace tether::klipper::protocol
