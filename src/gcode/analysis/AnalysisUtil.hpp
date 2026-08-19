/// @file AnalysisUtil.hpp
/// @brief Small shared helpers for G-code analysis.

#pragma once

#include "tether/gcode/GCodeTypes.hpp"
#include "tether/gcode/motion/InterpolationStrategy.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace tether::gcode::analysis {

constexpr double kFilamentDiameterMm = 1.75;
constexpr double kFilamentDensityGPerCm3 = 1.24; // PLA
constexpr double kLayerZSnap = 0.01;              // layer grouping resolution (mm)

/// Remove comments and whitespace, leaving just the G-code words for parsing.
inline std::string stripGcodeComments(const std::string& raw) {
    std::string out;
    out.reserve(raw.size());
    bool inParen = false;
    for (char c : raw) {
        if (c == '(') {
            inParen = true;
            continue;
        }
        if (c == ')') {
            inParen = false;
            continue;
        }
        if ((c == ';' || c == '%') && !inParen) break;
        if (!inParen) out.push_back(c);
    }
    return out;
}

inline std::string toUpper(std::string s) {
    std::transform(s.begin(), s.end(), s.begin(),
                   [](unsigned char c) { return static_cast<char>(std::toupper(c)); });
    return s;
}

inline std::string trim(std::string_view s) {
    size_t start = 0;
    while (start < s.size() && std::isspace(static_cast<unsigned char>(s[start]))) ++start;
    size_t end = s.size();
    while (end > start && std::isspace(static_cast<unsigned char>(s[end - 1]))) --end;
    return std::string(s.substr(start, end - start));
}

/// Find the first word starting with `letter` and parse the numeric value.
inline std::optional<double> findWordValue(const std::string& line, char letter) {
    const char uc = static_cast<char>(std::toupper(static_cast<unsigned char>(letter)));
    for (size_t i = 0; i < line.size(); ++i) {
        if (line[i] != uc) continue;
        size_t j = i + 1;
        while (j < line.size() &&
               (std::isdigit(static_cast<unsigned char>(line[j])) ||
                line[j] == '.' || line[j] == '-' || line[j] == '+')) {
            ++j;
        }
        if (j > i + 1) {
            try {
                return std::stod(line.substr(i + 1, j - i - 1));
            } catch (...) {
                return std::nullopt;
            }
        }
    }
    return std::nullopt;
}

/// Extract a slicer feature-type comment value (e.g. ;TYPE:PERIMETER).
inline std::optional<std::string> extractFeatureTag(std::string_view raw, std::string_view prefix) {
    std::string_view s = raw;
    size_t i = 0;
    while (i < s.size() && std::isspace(static_cast<unsigned char>(s[i]))) ++i;
    if (i < s.size() && s[i] == ';') ++i;
    while (i < s.size() && std::isspace(static_cast<unsigned char>(s[i]))) ++i;

    if (s.compare(i, prefix.size(), prefix) != 0) return std::nullopt;
    i += prefix.size();
    if (i < s.size() && s[i] == ':') ++i;
    return toUpper(trim(s.substr(i)));
}

/// Per-line signed E-axis delta (mm). Tracks M82/M83, G90/G91, G92 E, etc.
inline std::vector<double> computeEdeltas(const std::vector<std::string>& gcodeLines) {
    std::vector<double> deltas(gcodeLines.size(), 0.0);
    double currentE = 0.0;
    bool absoluteE = true;

    for (size_t i = 0; i < gcodeLines.size(); ++i) {
        const std::string line = stripGcodeComments(gcodeLines[i]);

        bool sawM82Or83 = false;
        if (auto m = findWordValue(line, 'M')) {
            if (*m == 82.0) { absoluteE = true; sawM82Or83 = true; }
            else if (*m == 83.0) { absoluteE = false; sawM82Or83 = true; }
        }
        if (!sawM82Or83) {
            if (auto g = findWordValue(line, 'G')) {
                if (*g == 90.0) absoluteE = true;
                else if (*g == 91.0) absoluteE = false;
            }
        }

        if (auto g = findWordValue(line, 'G')) {
            if (*g == 92.0) {
                if (auto e = findWordValue(line, 'E')) {
                    currentE = *e;
                }
            }
        }

        if (auto g = findWordValue(line, 'G')) {
            if (*g >= 0.0 && *g <= 3.0) {
                if (auto e = findWordValue(line, 'E')) {
                    double eDelta = absoluteE ? (*e - currentE) : *e;
                    currentE += eDelta;
                    deltas[i] = eDelta;
                }
            }
        }
    }

    return deltas;
}

/// Per-line active slicer feature type, derived from TYPE/MESH/FEATURE comments.
inline std::vector<std::string> computeFeatures(const std::vector<std::string>& gcodeLines) {
    std::vector<std::string> features(gcodeLines.size(), "UNKNOWN");
    std::string current = "UNKNOWN";

    for (size_t i = 0; i < gcodeLines.size(); ++i) {
        if (auto f = extractFeatureTag(gcodeLines[i], "TYPE")) {
            current = *f;
        } else if (auto m = extractFeatureTag(gcodeLines[i], "MESH")) {
            current = "MESH:" + *m;
        } else if (auto f = extractFeatureTag(gcodeLines[i], "FEATURE")) {
            current = *f;
        }
        features[i] = current;
    }

    return features;
}

/// Map an interpolation plane to the ordered axis indices.
inline void getPlaneAxes(GCode::InterpolationPlane plane, int& u, int& v, int& w) {
    switch (plane) {
        case GCode::InterpolationPlane::XY:
            u = 0; v = 1; w = 2; break;
        case GCode::InterpolationPlane::XZ:
            u = 0; v = 2; w = 1; break;
        case GCode::InterpolationPlane::YZ:
            u = 1; v = 2; w = 0; break;
        default:
            u = 0; v = 1; w = 2; break;
    }
}

/// Access a Position coordinate by integer axis index.
inline double posCoord(const GCode::Position& p, int axis) {
    return p[static_cast<std::size_t>(axis)];
}

} // namespace tether::gcode::analysis
