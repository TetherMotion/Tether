/**
 * @file CoordinateSystemAnalyzer.hpp
 * @brief Analyze coordinate systems: work offsets, rotations, scaling, origins.
 *
 * @copyright Copyright (C) 2025-2026 Tether Authors
 */
#pragma once

#include "tether/analysis/GcodeParseUtils.hpp"
#include <string>
#include <vector>

namespace tether::analysis {

/// @brief A work offset (G54-G59).
struct WorkOffset {
    std::string name;
    int lineNumber = 0;
    double x = 0, y = 0, z = 0;
    bool isActive = false;
};

/// @brief A G10 L2 command setting work offset values.
struct G10Command {
    int lineNumber = 0;
    std::string offset;
    double x = 0, y = 0, z = 0;
};

/// @brief A coordinate rotation event (G68/G69).
struct RotationEvent {
    int lineNumber = 0;
    bool activated = false;
    double angle = 0;
    double centerX = 0, centerY = 0;
};

/// @brief A coordinate scaling event (G51/G50).
struct ScaleEvent {
    int line = 0;
    bool isG51 = false;
    double xScale = 1, yScale = 1, zScale = 1;
};

/// @brief An origin offset (WCS or G92).
struct OriginOffset {
    std::string wcs;
    int line = 0;
    double x = 0, y = 0, z = 0;
    std::string source;
};

/// @brief Combined coordinate system analysis result.
struct CoordinateSystemResult {
    // Work offsets
    std::vector<WorkOffset> offsets;
    std::string activeOffset;
    int offsetChanges = 0;
    bool usesMultipleOffsets = false;
    std::vector<G10Command> g10Commands;

    // Rotations
    std::vector<RotationEvent> rotationEvents;
    bool hasRotation = false;
    int rotationEventCount = 0;
    double maxRotationAngle = 0;
    double totalRotation = 0;
    bool activeRotationAtEnd = false;
    std::vector<double> uniqueAngles;
    double rotationComplexityScore = 0;

    // Scaling
    std::vector<ScaleEvent> scaleEvents;
    int scaleEventCount = 0;
    double activeScaleX = 1, activeScaleY = 1, activeScaleZ = 1;
    bool scaleActiveAtEnd = false;
    double maxScale = 1, minScale = 1;
    double scaleComplexityScore = 0;

    // Origins
    std::vector<OriginOffset> origins;
    int wcsCount = 0;
    std::vector<std::string> wcsList;
    double offsetRangeMinX = 0, offsetRangeMaxX = 0;
    double offsetRangeMinY = 0, offsetRangeMaxY = 0;
    double originComplexityScore = 0;

    std::vector<std::string> recommendations;
};

/// @brief Analyze coordinate systems: work offsets, rotations, scaling, origins.
class CoordinateSystemAnalyzer {
public:
    /// @brief Analyze coordinate systems from G-code.
    CoordinateSystemResult analyze(const std::vector<std::string>& gcodeLines) const;
};

} // namespace tether::analysis
