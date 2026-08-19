/// @file SegmentSpeed.hpp
/// @brief Per-segment timing and speed data used by the G-code viewer and analysis tools.

#pragma once

#include <cstdint>

namespace tether::gcode {

/// @brief Per-segment speed data for miniplot visualization and analysis.
struct SegmentSpeed {
    double timeStart = 0.0;      ///< Time at start of segment (seconds)
    double duration = 0.0;       ///< Segment duration (seconds)
    int32_t blockIndex = -1;     ///< G-code block index
    int32_t lineNumber = 0;      ///< G-code line number
    double speedX = 0.0;         ///< X axis speed (mm/s)
    double speedY = 0.0;         ///< Y axis speed (mm/s)
    double speedZ = 0.0;         ///< Z axis speed (mm/s)
    double speedE = 0.0;         ///< Extruder speed (mm/s)
    double speedLinear = 0.0;    ///< Linear velocity magnitude (mm/s)
};

} // namespace tether::gcode
