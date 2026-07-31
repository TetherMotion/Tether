#pragma once

/// @file GCodeParser.hpp
/// @brief G-code line parsing (GcodeLine struct and parseGcodeLine function)

#include <algorithm>
#include <map>
#include <optional>
#include <regex>
#include <string>
#include <string_view>

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

/// @brief Parse a G-code line.
inline std::optional<GcodeLine> parseGcodeLine(std::string_view line) {
    // Strip comments
    std::string code;
    auto semi = line.find(';');
    if (semi != std::string_view::npos) {
        code = std::string(line.substr(0, semi));
    } else {
        code = std::string(line);
    }

    // Trim whitespace
    auto first = code.find_first_not_of(" \t\r\n");
    if (first == std::string::npos) return std::nullopt; // Empty line
    auto last = code.find_last_not_of(" \t\r\n");
    code = code.substr(first, last - first + 1);

    if (code.empty()) return std::nullopt;

    GcodeLine result;

    // Extract code (e.g. G1, M104, G28.2)
    std::regex codeRegex("^([GMgmT])(\\d+(?:\\.\\d+)?)");
    std::smatch codeMatch;
    if (std::regex_search(code, codeMatch, codeRegex)) {
        result.code = codeMatch[1].str() + codeMatch[2].str();
        // Normalize to uppercase
        result.code[0] = std::toupper(result.code[0]);
        code = code.substr(codeMatch.length());
    } else {
        // Could be a standalone T command (tool change)
        std::regex tRegex("^([Tt])(\\d+)");
        if (std::regex_search(code, codeMatch, tRegex)) {
            result.code = "T" + codeMatch[2].str();
            code = code.substr(codeMatch.length());
        } else {
            // Could be a macro name (e.g. "START_PRINT", "HOME_ALL", "SET_SERVO")
            std::regex macroRegex("^([A-Za-z_][A-Za-z0-9_]*)");
            if (std::regex_search(code, codeMatch, macroRegex)) {
                result.code = codeMatch[1].str();
                // Normalize to uppercase
                std::transform(result.code.begin(), result.code.end(),
                               result.code.begin(), [](unsigned char c) {
                                   return std::toupper(c);
                               });
                code = code.substr(codeMatch.length());
                // Capture remaining text for macro parameters
                result.text = code;

                // Parse named parameters (KEY=VALUE) for extended commands.
                // Values may be quoted strings ("..."), numbers, or bare words.
                std::regex namedRegex("([A-Za-z_][A-Za-z0-9_]*)\\s*=\\s*(\"[^\"]*\"|'[^']*'|[^\\s]+)");
                std::sregex_iterator nit(code.begin(), code.end(), namedRegex);
                std::sregex_iterator nend;
                for (; nit != nend; ++nit) {
                    std::string key = (*nit)[1].str();
                    std::string val = (*nit)[2].str();
                    // Strip surrounding quotes
                    if (val.size() >= 2 &&
                        ((val.front() == '"' && val.back() == '"') ||
                         (val.front() == '\'' && val.back() == '\''))) {
                        val = val.substr(1, val.size() - 2);
                    }
                    std::transform(key.begin(), key.end(), key.begin(),
                                   [](unsigned char c) { return std::toupper(c); });
                    result.namedParams[key] = val;
                }
            } else {
                return std::nullopt; // Not a recognized G-code
            }
        }
    }

    // For M117 and M23, capture the remaining text as the message/filename
    if (result.code == "M117" || result.code == "M23") {
        // Trim leading whitespace from the text
        auto textStart = code.find_first_not_of(" \t");
        if (textStart != std::string::npos) {
            result.text = code.substr(textStart);
        }
    } else {
        // Parse parameters (letter + optional number)
        // Some G-code commands use bare letters (e.g. "G28 X Y" where X and Y have no value)
        std::regex paramRegex("([A-Za-z])(?:\\s|$)|([A-Za-z])([-+]?\\d+(?:\\.\\d+)?)");
        std::sregex_iterator it(code.begin(), code.end(), paramRegex);
        std::sregex_iterator end;
        for (; it != end; ++it) {
            char key;
            double value;
            if (!(*it)[1].str().empty()) {
                // Bare letter (no number)
                key = std::toupper((*it)[1].str()[0]);
                value = 0.0; // Default value for bare letters
            } else {
                // Letter + number
                key = std::toupper((*it)[2].str()[0]);
                value = std::stod((*it)[3].str());
            }
            result.params[key] = value;
        }
    }

    // Store comment
    if (semi != std::string_view::npos) {
        result.comment = std::string(line.substr(semi + 1));
    }

    return result;
}

} // namespace tether::klipper::klippy
