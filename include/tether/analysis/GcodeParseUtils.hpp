/**
 * @file GcodeParseUtils.hpp
 * @brief Shared G-code parsing utilities for analysis components.
 *
 * All analysis components in tether::analysis share a common pattern:
 * strip comments, extract axis words (X/Y/Z/E/F/I/J/R/A/B/C), and
 * detect motion mode (G0/G1/G2/G3).  This header centralises those
 * helpers so each analyzer can focus on its domain logic.
 *
 * @copyright Copyright (C) 2025-2026 Tether Authors
 */
#pragma once

#include <algorithm>
#include <cmath>
#include <optional>
#include <regex>
#include <string>
#include <string_view>
#include <vector>

namespace tether::analysis {

/// @brief Strip comments (; and parenthesised) from a G-code line.
inline std::string stripComments(std::string_view line) {
    // Remove parenthesised comments first.
    std::string result{line};
    std::string::size_type pos = 0;
    while ((pos = result.find('(', pos)) != std::string::npos) {
        auto end = result.find(')', pos);
        if (end == std::string::npos) {
            result.erase(pos);
            break;
        }
        result.erase(pos, end - pos + 1);
    }
    // Remove semicolon comments.
    auto semi = result.find(';');
    if (semi != std::string::npos) result.erase(semi);
    // Trim whitespace.
    auto first = result.find_first_not_of(" \t\r\n");
    if (first == std::string::npos) return {};
    auto last = result.find_last_not_of(" \t\r\n");
    return result.substr(first, last - first + 1);
}

/// @brief Check if a stripped code line contains a G-code word (word boundary).
inline bool hasWord(std::string_view code, std::string_view word) {
    // Case-insensitive word-boundary search.
    std::string hay{code};
    std::transform(hay.begin(), hay.end(), hay.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    std::string needle{word};
    std::transform(needle.begin(), needle.end(), needle.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });

    auto pos = hay.find(needle);
    while (pos != std::string::npos) {
        // Check word boundary: preceding char must not be alphanumeric.
        bool leftOk = (pos == 0) ||
                      !std::isalnum(static_cast<unsigned char>(hay[pos - 1]));
        // Following char must not be a digit (G0 vs G01 etc. — digits are part of the word).
        bool rightOk = (pos + needle.size() >= hay.size()) ||
                       !std::isalnum(static_cast<unsigned char>(hay[pos + needle.size()]));
        if (leftOk && rightOk) return true;
        pos = hay.find(needle, pos + 1);
    }
    return false;
}

/// @brief Extract a numeric value for an axis word (e.g. X, Y, Z, E, F, I, J, R).
/// @return std::nullopt if the word is not present.
inline std::optional<double> extractValue(std::string_view code, char axis) {
    // Case-insensitive search for <axis><number>.
    for (std::size_t i = 0; i + 1 < code.size(); ++i) {
        char c = static_cast<char>(std::tolower(static_cast<unsigned char>(code[i])));
        if (c == std::tolower(static_cast<unsigned char>(axis))) {
            // Must be preceded by non-alphanumeric (word boundary).
            if (i > 0 && std::isalnum(static_cast<unsigned char>(code[i - 1])))
                continue;
            // Parse the number that follows.
            std::size_t j = i + 1;
            bool hasDigit = false;
            std::string num;
            if (j < code.size() && (code[j] == '+' || code[j] == '-')) {
                num.push_back(code[j]);
                ++j;
            }
            while (j < code.size() && (std::isdigit(static_cast<unsigned char>(code[j])) ||
                                       code[j] == '.')) {
                num.push_back(code[j]);
                if (std::isdigit(static_cast<unsigned char>(code[j]))) hasDigit = true;
                ++j;
            }
            if (hasDigit) {
                try {
                    return std::stod(num);
                } catch (...) {
                    return std::nullopt;
                }
            }
        }
    }
    return std::nullopt;
}

/// @brief Motion mode of a G-code line.
enum class MotionMode {
    None,   ///< Not a motion command
    Rapid,  ///< G0
    Feed,   ///< G1
    ArcCW,  ///< G2
    ArcCCW, ///< G3
    Cancel  ///< G80
};

/// @brief Detect the motion mode of a stripped G-code line.
inline MotionMode detectMotionMode(std::string_view code) {
    if (hasWord(code, "G0"))  return MotionMode::Rapid;
    if (hasWord(code, "G1"))  return MotionMode::Feed;
    if (hasWord(code, "G2"))  return MotionMode::ArcCW;
    if (hasWord(code, "G3"))  return MotionMode::ArcCCW;
    if (hasWord(code, "G80")) return MotionMode::Cancel;
    return MotionMode::None;
}

/// @brief A parsed G-code motion move (position + feed + extrusion).
struct ParsedMove {
    MotionMode mode = MotionMode::None;
    double x = 0, y = 0, z = 0, e = 0;
    double feedRate = 0;       ///< mm/min
    double i = 0, j = 0, r = 0; ///< Arc params
    bool hasX = false, hasY = false, hasZ = false, hasE = false;
    bool hasI = false, hasJ = false, hasR = false;
    int lineNumber = 0;
};

/// @brief Parse a single G-code line into a ParsedMove.
/// Previous values are used for missing axes.
inline ParsedMove parseLine(std::string_view rawLine, int lineNumber,
                            const ParsedMove& prev) {
    ParsedMove m = prev;
    m.mode = MotionMode::None;
    m.lineNumber = lineNumber;
    m.hasX = m.hasY = m.hasZ = m.hasE = false;
    m.hasI = m.hasJ = m.hasR = false;

    auto code = stripComments(rawLine);
    if (code.empty()) return m;

    m.mode = detectMotionMode(code);

    if (auto v = extractValue(code, 'F')) m.feedRate = *v;
    if (auto v = extractValue(code, 'X')) { m.x = *v; m.hasX = true; }
    if (auto v = extractValue(code, 'Y')) { m.y = *v; m.hasY = true; }
    if (auto v = extractValue(code, 'Z')) { m.z = *v; m.hasZ = true; }
    if (auto v = extractValue(code, 'E')) { m.e = *v; m.hasE = true; }
    if (auto v = extractValue(code, 'I')) { m.i = *v; m.hasI = true; }
    if (auto v = extractValue(code, 'J')) { m.j = *v; m.hasJ = true; }
    if (auto v = extractValue(code, 'R')) { m.r = *v; m.hasR = true; }

    return m;
}

/// @brief Euclidean distance between two moves (3D).
inline double moveDistance(const ParsedMove& a, const ParsedMove& b) {
    const double dx = b.x - a.x;
    const double dy = b.y - a.y;
    const double dz = b.z - a.z;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

/// @brief Euclidean distance between two moves (XY only).
inline double moveDistanceXY(const ParsedMove& a, const ParsedMove& b) {
    const double dx = b.x - a.x;
    const double dy = b.y - a.y;
    return std::sqrt(dx * dx + dy * dy);
}

/// @brief Check if a move is a motion command (G0-G3).
inline bool isMotion(const ParsedMove& m) {
    return m.mode == MotionMode::Rapid || m.mode == MotionMode::Feed ||
           m.mode == MotionMode::ArcCW || m.mode == MotionMode::ArcCCW;
}

/// @brief Check if a move is extruding (E increased).
inline bool isExtruding(const ParsedMove& prev, const ParsedMove& cur) {
    return cur.hasE && cur.e > prev.e;
}

/// @brief Check if a move is a retraction (E decreased).
inline bool isRetraction(const ParsedMove& prev, const ParsedMove& cur) {
    return cur.hasE && cur.e < prev.e - 0.001;
}

} // namespace tether::analysis
