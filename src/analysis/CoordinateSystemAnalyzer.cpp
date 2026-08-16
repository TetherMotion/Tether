/**
 * @file CoordinateSystemAnalyzer.cpp
 * @brief Analyze coordinate systems: work offsets, rotations, scaling, origins.
 */
#include "tether/analysis/CoordinateSystemAnalyzer.hpp"

#include <algorithm>
#include <cmath>
#include <format>
#include <set>

namespace tether::analysis {

CoordinateSystemResult CoordinateSystemAnalyzer::analyze(
    const std::vector<std::string>& gcodeLines) const {

    CoordinateSystemResult result;
    std::string activeOffset;
    std::map<std::string, WorkOffset> offsetMap;
    bool activeRotation = false;
    double maxScale = 1, minScale = 1;
    std::set<double> angleSet;
    std::set<std::string> wcsSet;

    static const char* offsetNames[] = {"G54", "G55", "G56", "G57", "G58", "G59"};

    for (int i = 0; i < static_cast<int>(gcodeLines.size()); ++i) {
        auto code = stripComments(gcodeLines[i]);
        if (code.empty()) continue;

        // === Work offsets (G54-G59) ===
        for (const char* name : offsetNames) {
            if (hasWord(code, name)) {
                if (activeOffset != name) {
                    activeOffset = name;
                    result.offsetChanges++;
                }
                if (offsetMap.find(name) == offsetMap.end()) {
                    offsetMap[name] = {name, i, 0, 0, 0, true};
                }
                break;
            }
        }

        // G10 L2 (set work offset).
        if (hasWord(code, "G10") && hasWord(code, "L2")) {
            auto p = extractValue(code, 'P');
            auto x = extractValue(code, 'X');
            auto y = extractValue(code, 'Y');
            auto z = extractValue(code, 'Z');
            if (p) {
                int pNum = static_cast<int>(*p);
                std::string offsetName = (pNum >= 1 && pNum <= 6)
                    ? offsetNames[pNum - 1]
                    : std::format("G59.{}", pNum - 6);
                result.g10Commands.push_back({i, offsetName,
                    x.value_or(0), y.value_or(0), z.value_or(0)});
                offsetMap[offsetName] = {offsetName, i,
                    x.value_or(0), y.value_or(0), z.value_or(0),
                    activeOffset == offsetName};
            }
        }

        // === Rotations (G68/G69) ===
        if (hasWord(code, "G68")) {
            auto r = extractValue(code, 'R');
            auto x = extractValue(code, 'X');
            auto y = extractValue(code, 'Y');
            double angle = r.value_or(0);
            result.rotationEvents.push_back({i, true, angle,
                x.value_or(0), y.value_or(0)});
            result.hasRotation = true;
            result.totalRotation += std::abs(angle);
            result.maxRotationAngle = std::max(result.maxRotationAngle, std::abs(angle));
            angleSet.insert(std::round(angle * 100) / 100);
            activeRotation = true;
        }
        if (hasWord(code, "G69")) {
            result.rotationEvents.push_back({i, false, 0, 0, 0});
            activeRotation = false;
        }

        // === Scaling (G51/G50) ===
        if (hasWord(code, "G51")) {
            auto xs = extractValue(code, 'X');
            auto ys = extractValue(code, 'Y');
            auto zs = extractValue(code, 'Z');
            // Also check I/J/K for some controllers.
            if (!xs) xs = extractValue(code, 'I');
            if (!ys) ys = extractValue(code, 'J');
            if (!zs) zs = extractValue(code, 'K');
            double xScale = xs.value_or(1);
            double yScale = ys.value_or(1);
            double zScale = zs.value_or(1);
            result.scaleEvents.push_back({i, true, xScale, yScale, zScale});
            result.activeScaleX = xScale;
            result.activeScaleY = yScale;
            result.activeScaleZ = zScale;
            result.scaleActiveAtEnd = true;
            maxScale = std::max({maxScale, xScale, yScale, zScale});
            minScale = std::min({minScale, xScale, yScale, zScale});
        }
        if (hasWord(code, "G50")) {
            result.scaleEvents.push_back({i, false, 1, 1, 1});
            result.activeScaleX = result.activeScaleY = result.activeScaleZ = 1;
            result.scaleActiveAtEnd = false;
        }

        // === Origins (G54-G59 + G92) ===
        bool isWcs = false;
        for (const char* name : offsetNames) {
            if (hasWord(code, name)) { isWcs = true; break; }
        }
        bool isG92 = hasWord(code, "G92");
        if (isWcs || isG92) {
            std::string wcs = isG92 ? "G92" : "";
            if (isWcs) {
                for (const char* name : offsetNames) {
                    if (hasWord(code, name)) { wcs = name; break; }
                }
            }
            auto x = extractValue(code, 'X');
            auto y = extractValue(code, 'Y');
            auto z = extractValue(code, 'Z');
            result.origins.push_back({wcs, i,
                x.value_or(0), y.value_or(0), z.value_or(0), wcs});
            wcsSet.insert(wcs);
        }
    }

    // Finalize work offsets.
    for (auto& [name, off] : offsetMap) {
        off.isActive = (off.name == activeOffset);
        result.offsets.push_back(off);
    }
    result.activeOffset = activeOffset;
    result.usesMultipleOffsets = (result.offsets.size() > 1);

    // Finalize rotations.
    result.rotationEventCount = static_cast<int>(result.rotationEvents.size());
    result.activeRotationAtEnd = activeRotation;
    result.uniqueAngles = std::vector<double>(angleSet.begin(), angleSet.end());
    result.rotationComplexityScore = std::min(100.0,
        static_cast<double>(result.rotationEventCount) * 20.0 + result.maxRotationAngle / 2.0);

    // Finalize scaling.
    result.scaleEventCount = static_cast<int>(result.scaleEvents.size());
    result.maxScale = maxScale;
    result.minScale = minScale;
    double scaleRange = maxScale - minScale;
    result.scaleComplexityScore = std::min(100.0,
        static_cast<double>(result.scaleEventCount) * 15.0 + scaleRange * 20.0);

    // Finalize origins.
    result.wcsList = std::vector<std::string>(wcsSet.begin(), wcsSet.end());
    result.wcsCount = static_cast<int>(wcsSet.size());
    if (!result.origins.empty()) {
        result.offsetRangeMinX = 1e18; result.offsetRangeMaxX = -1e18;
        result.offsetRangeMinY = 1e18; result.offsetRangeMaxY = -1e18;
        for (const auto& o : result.origins) {
            result.offsetRangeMinX = std::min(result.offsetRangeMinX, o.x);
            result.offsetRangeMaxX = std::max(result.offsetRangeMaxX, o.x);
            result.offsetRangeMinY = std::min(result.offsetRangeMinY, o.y);
            result.offsetRangeMaxY = std::max(result.offsetRangeMaxY, o.y);
        }
    }
    result.originComplexityScore = std::min(100.0,
        static_cast<double>(result.wcsCount) * 15.0 +
        static_cast<double>(result.origins.size()) * 5.0);

    // Recommendations.
    if (result.offsetChanges > 0) {
        result.recommendations.push_back(
            std::format("{} work offset changes, active: {}", result.offsetChanges,
                        result.activeOffset.empty() ? "none" : result.activeOffset));
    }
    if (result.activeRotationAtEnd) {
        result.recommendations.push_back("Rotation still active — add G69 to cancel");
    }
    if (result.scaleActiveAtEnd) {
        result.recommendations.push_back("Scale still active — add G50 to cancel");
    }
    if (result.maxScale > 2 || result.minScale < 0.5) {
        result.recommendations.push_back("Extreme scale factor — verify dimensions");
    }
    if (result.wcsCount > 3) {
        result.recommendations.push_back(
            std::format("{} coordinate systems — verify setup", result.wcsCount));
    }
    if (result.recommendations.empty()) {
        result.recommendations.push_back("No coordinate system issues detected");
    }

    return result;
}

} // namespace tether::analysis
