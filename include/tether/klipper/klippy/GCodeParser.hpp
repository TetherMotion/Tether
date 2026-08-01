#pragma once

/// @file GCodeParser.hpp
/// @brief G-code line parsing (GcodeLine struct and parseGcodeLine function)
///
/// @details
/// The parser uses the shared `GCode::Lexer` from `tether/gcode/GCodeLexer.hpp`
/// for tokenization of standard G/M/T codes and their parameters. Extended
/// Klipper commands (SET_SERVO, BED_MESH_CALIBRATE, etc.) are handled by a
/// lightweight fallback path since the Tether lexer is designed for
/// RS274/NGC syntax and doesn't recognise multi-letter command names.

#include "tether/gcode/GCodeLexer.hpp"
#include "tether/gcode/GCodeTypes.hpp"

#include <algorithm>
#include <cctype>
#include <map>
#include <optional>
#include <sstream>
#include <string>
#include <string_view>
#include <vector>

namespace tether::klipper::klippy {

// ============================================================================
// G-code command representation
// ============================================================================

/// @brief A parsed G-code line.
struct GcodeLine {
    std::string code;           ///< e.g. "G1", "M104", "G28", "BED_MESH_CALIBRATE"
    std::string comment;        ///< Comment text (after ;)
    std::string text;           ///< Remaining text after code (for M117, M23, etc.)
    std::map<char, double> params; ///< Parameters: X, Y, Z, E, F, S, T, etc.
    std::map<std::string, std::string> namedParams; ///< Named params: SERVO=..., ANGLE=...

    /// @brief Check if a parameter exists.
    bool has(char key) const { return params.count(key) > 0; }

    /// @brief Get a parameter value.
    double get(char key, double defaultVal = 0.0) const {
        auto it = params.find(key);
        return it == params.end() ? defaultVal : it->second;
    }

    /// @brief Check if a named parameter exists (case-insensitive).
    bool hasNamed(const std::string& key) const {
        return namedParams.find(toUpperKey(key)) != namedParams.end();
    }

    /// @brief Get a named parameter as string (case-insensitive).
    std::string getNamed(const std::string& key, const std::string& defaultVal = "") const {
        auto it = namedParams.find(toUpperKey(key));
        return it == namedParams.end() ? defaultVal : it->second;
    }

    /// @brief Get a named parameter as double (case-insensitive).
    double getNamedDouble(const std::string& key, double defaultVal = 0.0) const {
        auto it = namedParams.find(toUpperKey(key));
        if (it == namedParams.end()) return defaultVal;
        try { return std::stod(it->second); } catch (...) { return defaultVal; }
    }

    /// @brief Get a named parameter as int (case-insensitive).
    int getNamedInt(const std::string& key, int defaultVal = 0) const {
        auto it = namedParams.find(toUpperKey(key));
        if (it == namedParams.end()) return defaultVal;
        try { return std::stoi(it->second); } catch (...) { return defaultVal; }
    }

    /// @brief Check if this is an extended command (uppercase, contains underscore).
    bool isExtendedCommand() const {
        if (code.empty()) return false;
        if (!std::isupper(static_cast<unsigned char>(code[0]))) return false;
        return code.find('_') != std::string::npos;
    }

private:
    static std::string toUpperKey(const std::string& s) {
        std::string r = s;
        std::transform(r.begin(), r.end(), r.begin(),
                       [](unsigned char c) { return std::toupper(c); });
        return r;
    }
};

// ============================================================================
// Parsing
// ============================================================================

namespace detail {

/// @brief Check if a string starts with a G/M/T code pattern.
/// @return true if the first non-space char is G/g/M/m/T/t followed by a digit.
inline bool startsWithGcodeCommand(const std::string& s) {
    auto first = s.find_first_not_of(" \t");
    if (first == std::string::npos) return false;
    char c = std::toupper(static_cast<unsigned char>(s[first]));
    if (c != 'G' && c != 'M' && c != 'T') return false;
    // Must be followed by a digit (or for T, could be standalone)
    size_t next = first + 1;
    if (next >= s.size()) return c == 'T'; // bare T
    return std::isdigit(static_cast<unsigned char>(s[next])) ||
           (c == 'T' && std::isdigit(static_cast<unsigned char>(s[next])));
}

/// @brief Parse named parameters (KEY=VALUE) from a string.
/// Used for extended Klipper commands like SET_SERVO SERVO=foo ANGLE=45.
inline void parseNamedParams(const std::string& s,
                              std::map<std::string, std::string>& out) {
    size_t i = 0;
    while (i < s.size()) {
        // Skip whitespace
        while (i < s.size() && std::isspace(static_cast<unsigned char>(s[i]))) ++i;
        if (i >= s.size()) break;

        // Read key (letters, digits, underscore; must start with letter or _)
        size_t keyStart = i;
        if (std::isalpha(static_cast<unsigned char>(s[i])) || s[i] == '_') {
            while (i < s.size() &&
                   (std::isalnum(static_cast<unsigned char>(s[i])) || s[i] == '_'))
                ++i;
        } else {
            ++i;
            continue;
        }
        std::string key = s.substr(keyStart, i - keyStart);

        // Skip whitespace before '='
        while (i < s.size() && std::isspace(static_cast<unsigned char>(s[i]))) ++i;

        if (i < s.size() && s[i] == '=') {
            ++i; // skip '='
            while (i < s.size() && std::isspace(static_cast<unsigned char>(s[i]))) ++i;

            std::string val;
            if (i < s.size() && (s[i] == '"' || s[i] == '\'')) {
                char quote = s[i++];
                while (i < s.size() && s[i] != quote) val += s[i++];
                if (i < s.size()) ++i; // skip closing quote
            } else {
                while (i < s.size() && !std::isspace(static_cast<unsigned char>(s[i])))
                    val += s[i++];
            }

            // Uppercase the key for case-insensitive lookup
            std::transform(key.begin(), key.end(), key.begin(),
                           [](unsigned char c) { return std::toupper(c); });
            out[key] = val;
        } else {
            // Key without = — skip (not a named param)
        }
    }
}

/// @brief Parse a line using the shared GCode::Lexer for standard G/M/T codes.
/// @param codeOnly The line with comments stripped and trimmed.
/// @param comment The comment text (may be empty).
/// @return Parsed GcodeLine, or nullopt if not a valid G-code line.
inline std::optional<GcodeLine> parseWithLexer(const std::string& codeOnly,
                                                const std::string& comment) {
    GCode::Lexer lexer;
    lexer.setInput(codeOnly);

    GcodeLine result;
    result.comment = comment;

    // Collect tokens for the line
    auto tokens = lexer.tokenizeLine();

    if (tokens.empty()) return std::nullopt;

    // First token must be a WORD (G, M, or T)
    bool hasCommand = false;
    std::string codeStr;

    for (const auto& tok : tokens) {
        if (tok.type == GCode::LexerTokenType::WORD) {
            char letter = GCode::wordLetterToChar(tok.letter);
            if (letter == 'G' || letter == 'M') {
                if (!hasCommand) {
                    // Build code string: letter + number (handle subcodes like G38.2)
                    int codeInt = static_cast<int>(tok.value);
                    double frac = tok.value - codeInt;
                    if (frac > 1e-9) {
                        // Subcode (e.g. G38.2)
                        std::ostringstream ss;
                        ss << letter << codeInt << "." << static_cast<int>(frac * 10 + 0.5);
                        codeStr = ss.str();
                    } else {
                        codeStr = letter + std::to_string(codeInt);
                    }
                    result.code = codeStr;
                    hasCommand = true;
                } else {
                    // Additional G/M word — treat as parameter
                    result.params[letter] = tok.value;
                }
            } else if (letter == 'T') {
                if (!hasCommand) {
                    result.code = "T" + std::to_string(static_cast<int>(tok.value));
                    hasCommand = true;
                } else {
                    result.params[letter] = tok.value;
                }
            } else {
                // Regular parameter (X, Y, Z, E, F, S, etc.)
                result.params[letter] = tok.value;
            }
        } else if (tok.type == GCode::LexerTokenType::KEYVALUE) {
            // Named parameter (KEY=VALUE)
            std::string key = tok.kvKey;
            std::transform(key.begin(), key.end(), key.begin(),
                           [](unsigned char c) { return std::toupper(c); });
            result.namedParams[key] = tok.kvValue;
        } else if (tok.type == GCode::LexerTokenType::COMMENT) {
            // Inline comment — append to comment field
            if (!result.comment.empty()) result.comment += " ";
            result.comment += tok.text;
        } else if (tok.type == GCode::LexerTokenType::EOL ||
                   tok.type == GCode::LexerTokenType::END) {
            break;
        }
    }

    if (!hasCommand) return std::nullopt;

    // Handle M117/M23: capture remaining text after the command
    if (result.code == "M117" || result.code == "M23") {
        // Find the position after the command code in the original string
        // and capture the rest as text
        auto pos = codeOnly.find_first_of(" \t");
        if (pos != std::string::npos) {
            auto textStart = codeOnly.find_first_not_of(" \t", pos);
            if (textStart != std::string::npos) {
                result.text = codeOnly.substr(textStart);
            }
        }
        // Clear params for M117/M23 — the text is the message, not parameters
        result.params.clear();
        return result;
    }

    // The Tether lexer only tokenizes letter+number pairs (RS274/NGC style).
    // Klipper/RepRap flavor also uses bare letters (e.g. "G28 X Y" where X
    // and Y have no value). Scan the input for bare letters the lexer missed.
    // Skip the command code portion at the start.
    size_t scanStart = 0;
    // Find end of command code (first whitespace or end of string)
    while (scanStart < codeOnly.size() &&
           !std::isspace(static_cast<unsigned char>(codeOnly[scanStart])))
        ++scanStart;
    for (size_t i = scanStart; i < codeOnly.size(); ++i) {
        char c = codeOnly[i];
        if (!std::isalpha(static_cast<unsigned char>(c))) continue;
        char upper = std::toupper(static_cast<unsigned char>(c));
        // Skip if this letter is already parsed as a parameter (has a value)
        if (result.params.count(upper)) continue;
        // Check if followed by a number (letter+number pair the lexer should
        // have caught — but double-check just in case)
        size_t j = i + 1;
        while (j < codeOnly.size() && std::isspace(static_cast<unsigned char>(codeOnly[j]))) ++j;
        if (j < codeOnly.size() &&
            (std::isdigit(static_cast<unsigned char>(codeOnly[j])) ||
             codeOnly[j] == '-' || codeOnly[j] == '+' || codeOnly[j] == '.')) {
            // Letter+number — the lexer should have handled this.
            // Parse it manually as a fallback.
            size_t numStart = j;
            if (codeOnly[numStart] == '-' || codeOnly[numStart] == '+') ++numStart;
            while (numStart < codeOnly.size() &&
                   (std::isdigit(static_cast<unsigned char>(codeOnly[numStart])) ||
                    codeOnly[numStart] == '.'))
                ++numStart;
            try {
                result.params[upper] = std::stod(codeOnly.substr(j, numStart - j));
            } catch (...) {}
            i = numStart - 1;
        } else {
            // Bare letter — default value 0.0
            result.params[upper] = 0.0;
        }
    }

    return result;
}

/// @brief Parse a line as an extended Klipper command (SET_SERVO, BED_MESH_CALIBRATE, etc.)
/// @param codeOnly The line with comments stripped and trimmed.
/// @param comment The comment text (may be empty).
/// @return Parsed GcodeLine, or nullopt if not a valid extended command.
inline std::optional<GcodeLine> parseExtendedCommand(const std::string& codeOnly,
                                                      const std::string& comment) {
    GcodeLine result;
    result.comment = comment;

    // Extract the command name (letters, digits, underscores)
    size_t i = 0;
    while (i < codeOnly.size() &&
           (std::isalnum(static_cast<unsigned char>(codeOnly[i])) || codeOnly[i] == '_'))
        ++i;

    if (i == 0) return std::nullopt;

    result.code = codeOnly.substr(0, i);
    // Normalize to uppercase
    std::transform(result.code.begin(), result.code.end(),
                   result.code.begin(), [](unsigned char c) {
                       return std::toupper(c);
                   });

    // Remaining text after command
    std::string rest = codeOnly.substr(i);
    result.text = rest;

    // Parse named parameters (KEY=VALUE)
    parseNamedParams(rest, result.namedParams);

    // Also parse single-letter parameters that might appear after the command
    // (some extended commands use both named and single-letter params)
    size_t j = 0;
    while (j < rest.size()) {
        // Skip whitespace
        while (j < rest.size() && std::isspace(static_cast<unsigned char>(rest[j]))) ++j;
        if (j >= rest.size()) break;

        // Check if this is a KEY=VALUE (skip if so — already parsed)
        size_t k = j;
        if (std::isalpha(static_cast<unsigned char>(rest[k])) || rest[k] == '_') {
            while (k < rest.size() &&
                   (std::isalnum(static_cast<unsigned char>(rest[k])) || rest[k] == '_'))
                ++k;
            // Skip whitespace
            size_t eqCheck = k;
            while (eqCheck < rest.size() && std::isspace(static_cast<unsigned char>(rest[eqCheck]))) ++eqCheck;
            if (eqCheck < rest.size() && rest[eqCheck] == '=') {
                j = eqCheck + 1;
                // Skip value
                if (j < rest.size() && (rest[j] == '"' || rest[j] == '\'')) {
                    char q = rest[j++];
                    while (j < rest.size() && rest[j] != q) ++j;
                    if (j < rest.size()) ++j;
                } else {
                    while (j < rest.size() && !std::isspace(static_cast<unsigned char>(rest[j]))) ++j;
                }
                continue;
            }
        }

        // Try to parse as single-letter param (letter + optional number)
        char letter = std::toupper(static_cast<unsigned char>(rest[j]));
        if (std::isalpha(static_cast<unsigned char>(rest[j])) && j + 1 < rest.size()) {
            // Check if next is a number
            size_t numStart = j + 1;
            // Skip whitespace between letter and number
            while (numStart < rest.size() && std::isspace(static_cast<unsigned char>(rest[numStart]))) ++numStart;
            if (numStart < rest.size() &&
                (std::isdigit(static_cast<unsigned char>(rest[numStart])) ||
                 rest[numStart] == '-' || rest[numStart] == '+' || rest[numStart] == '.')) {
                size_t numEnd = numStart;
                if (rest[numEnd] == '-' || rest[numEnd] == '+') ++numEnd;
                while (numEnd < rest.size() &&
                       (std::isdigit(static_cast<unsigned char>(rest[numEnd])) ||
                        rest[numEnd] == '.'))
                    ++numEnd;
                try {
                    double val = std::stod(rest.substr(numStart, numEnd - numStart));
                    result.params[letter] = val;
                } catch (...) {}
                j = numEnd;
                continue;
            } else {
                // Bare letter (no number) — default to 0
                result.params[letter] = 0.0;
                j = numStart;
                continue;
            }
        } else if (std::isalpha(static_cast<unsigned char>(rest[j]))) {
            // Bare letter at end of string
            result.params[letter] = 0.0;
            ++j;
            continue;
        } else {
            ++j;
        }
    }

    return result;
}

} // namespace detail

/// @brief Parse a G-code line.
/// @details Uses the shared GCode::Lexer for standard G/M/T commands and a
/// lightweight fallback for extended Klipper commands (SET_SERVO, etc.).
inline std::optional<GcodeLine> parseGcodeLine(std::string_view line) {
    // Strip comments (find semicolon)
    std::string codeOnly;
    std::string comment;
    auto semi = line.find(';');
    if (semi != std::string_view::npos) {
        codeOnly = std::string(line.substr(0, semi));
        comment = std::string(line.substr(semi + 1));
    } else {
        codeOnly = std::string(line);
    }

    // Trim whitespace
    auto first = codeOnly.find_first_not_of(" \t\r\n");
    if (first == std::string::npos) return std::nullopt; // Empty line
    auto last = codeOnly.find_last_not_of(" \t\r\n");
    codeOnly = codeOnly.substr(first, last - first + 1);

    if (codeOnly.empty()) return std::nullopt;

    // Route to the appropriate parser:
    // - Standard G/M/T codes → GCode::Lexer
    // - Extended commands (SET_SERVO, BED_MESH_CALIBRATE, etc.) → fallback
    if (detail::startsWithGcodeCommand(codeOnly)) {
        return detail::parseWithLexer(codeOnly, comment);
    } else {
        return detail::parseExtendedCommand(codeOnly, comment);
    }
}

} // namespace tether::klipper::klippy
