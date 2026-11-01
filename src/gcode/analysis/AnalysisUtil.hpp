/**
 * @file AnalysisUtil.hpp
 * @brief Small shared helpers for G-code analysis.
 */

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

/**
 * @brief Remove comments from a raw G-code line.
 *
 * Strips parenthesized comments `( ... )`, and truncates the line at the
 * first `;` or `%` that is not nested inside parentheses (G-code `;`
 * comments run to end of line; `%` marks program boundaries in some
 * dialects). Unbalanced parentheses are tolerated — a `(` without a
 * matching `)` comments out the rest of the line.
 *
 * @param raw  Raw line as read from the file, comments included.
 * @return Line containing only executable words and whitespace; may be
 *         empty or all-whitespace for comment-only lines.
 */
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

/**
 * @brief Find the first G-code word for a letter and parse its value.
 *
 * Scans the line for the first occurrence of @p letter (compared
 * case-insensitively, so 'e' matches 'E') followed immediately by a
 * numeric literal — digits, sign, and decimal point are consumed
 * greedily. Returns the parsed value of that first word only; later
 * words with the same letter are ignored, which matches how G-code
 * interpreters treat repeated words on a line.
 *
 * A letter with no numeric tail (e.g. bare 'E' or 'E,') yields
 * std::nullopt, as does a malformed literal that std::stod cannot parse.
 * Note the scan is not token-aware — a letter inside another word (e.g.
 * 'X' inside a comment remnant) would still match, so input should be
 * pre-cleaned with stripGcodeComments().
 *
 * @param line    Comment-stripped G-code line (see stripGcodeComments()).
 * @param letter  Word letter to look for; case-insensitive.
 * @return The word's numeric value, or std::nullopt if absent/unparseable.
 */
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

/**
 * @brief Extract a slicer feature-type comment value (e.g. ;TYPE:PERIMETER).
 *
 * Matches lines of the form `;PREFIX:value` — leading whitespace before
 * the `;` and between `;` and the prefix is tolerated, and the colon after
 * the prefix is optional. The value is trimmed and uppercased so slicer
 * variants ("Perimeter", "PERIMETER") compare equal.
 *
 * @param raw     Full line including the comment marker.
 * @param prefix  Tag prefix to match (e.g. "TYPE", "MESH", "FEATURE");
 *                case-sensitive against the raw line.
 * @return Uppercased, trimmed tag value, or std::nullopt if the line does
 *         not carry the requested tag.
 */
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

/**
 * @brief Per-line signed extrusion delta in millimetres of filament.
 *
 * Replays the extrusion-mode state machine over the input lines and
 * reports, for each line, the filament length its E word adds to the
 * accumulated position. Handles:
 * - **M82 / M83**: absolute / relative extrusion mode (an M-word on the
 *   line wins over a G90/G91 on the same line, matching real firmware
 *   ordering where M82/M83 override the G90/G91 E-mode inheritance).
 * - **G90 / G91**: absolute / relative positioning, which also selects
 *   the E-mode when no explicit M82/M83 has been seen on the line.
 * - **G92 E<n>**: resets the accumulated E reference so the next
 *   absolute move deltas are measured from <n>.
 * - **G0–G3 E<n>**: the line's delta is `E - currentE` in absolute mode
 *   or `E` in relative mode; negative deltas (retractions) are preserved.
 *
 * Non-extruding lines (moves without an E word, and all other commands)
 * produce 0.0 at their index.
 *
 * @param gcodeLines  Raw G-code lines; comments are stripped internally.
 * @return Vector of signed E deltas [mm filament], one entry per input
 *         line, same order and size as @p gcodeLines.
 */
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

/**
 * @brief Per-line active slicer feature type.
 *
 * Slicers annotate sections with comments like `;TYPE:PERIMETER`,
 * `;MESH:NONMESH`, or `;FEATURE:...`. This replays those markers so every
 * line is tagged with the feature in effect at that point — a marker
 * applies from its line forward until the next marker replaces it.
 * `;MESH:x` is normalized to "MESH:x"; TYPE and FEATURE values are
 * uppercased by extractFeatureTag().
 *
 * Lines before the first marker, and files with no markers at all, are
 * tagged "UNKNOWN".
 *
 * @param gcodeLines  Raw G-code lines (comments needed — the markers are
 *                    comments themselves, so pass unstripped lines).
 * @return Vector of feature-type strings, one per input line, same order
 *         and size as @p gcodeLines.
 */
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

/**
 * @brief Map an interpolation plane to the ordered axis indices.
 *
 * Returns (u, v, w) where u/v are the in-plane axes (first/second arc
 * coordinates for G2/G3) and w is the axis normal to the plane (helical
 * axis). XY→(0,1,2), XZ→(0,2,1), YZ→(1,2,0). Unrecognized planes fall
 * back to the XY mapping.
 *
 * @param plane  Active G17/G18/G19 plane.
 * @param[out] u  First in-plane axis index.
 * @param[out] v  Second in-plane axis index.
 * @param[out] w  Plane-normal axis index.
 */
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

/**
 * @brief Access a Position coordinate by integer axis index.
 *
 * Convenience for use with the indices returned by getPlaneAxes() —
 * the caller is responsible for passing a valid index; no bounds check
 * is performed.
 *
 * @param p     Position to read.
 * @param axis  Axis index (0 = X, 1 = Y, 2 = Z, ...).
 * @return Coordinate value of @p p along @p axis [mm].
 */
inline double posCoord(const GCode::Position& p, int axis) {
    return p[static_cast<std::size_t>(axis)];
}

} // namespace tether::gcode::analysis
