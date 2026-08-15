/**
 * @file PlanningSegmentBuilder.hpp
 * @brief Convert G-code text into PlanningSegments using the Interpreter
 *
 * @details
 * This utility provides a high-level API to parse G-code text and produce
 * a vector of GCode::PlanningSegment — the canonical motion-planning
 * representation that carries start/end positions, arc geometry, feed
 * rates, and modal state (blend tolerance, plane, etc.).
 *
 * It wraps GCode::Interpreter with emit-arc-segments mode enabled so that
 * arc moves (G2/G3) are preserved as exact arc segments rather than
 * being tessellated into line segments.
 *
 * The header deliberately avoids including GCodeInterpreter.hpp to keep
 * transitive dependencies (magic_enum, Eigen, etc.) out of consumer
 * compilation units. The full Interpreter header is included only in
 * the .cpp file.
 *
 * Usage:
 * ```cpp
 * auto result = PlanningSegmentBuilder::fromText(gcodeString);
 * if (result.error.ok()) {
 *     for (const auto& seg : result.segments) { ... }
 * }
 * ```
 */

#pragma once

#include "GCodeTypes.hpp"
#include "motion/InterpolationStrategy.hpp"

#include <string>
#include <vector>

namespace GCode {

// Forward declaration — full definition in GCodeInterpreter.hpp
struct InterpreterConfig;

/// @brief Per-block metadata (line number, g-code text, motion type).
struct BlockMetadata {
    int32_t blockIndex = -1;
    int32_t lineNumber = -1;
    uint8_t motionType = 0;  ///< 0=rapid, 1=linear, 2=arcCW, 3=arcCCW, 255=non-motion
    std::string gcodeText;
};

/// @brief Result of building PlanningSegments from G-code text.
struct PlanningSegmentResult {
    std::vector<PlanningSegment> segments;  ///< Parsed motion segments
    std::vector<BlockMetadata> blocks;      ///< Per-block metadata
    Error error;                            ///< OK on success
};

/// @brief High-level utility: G-code text → PlanningSegment[].
class PlanningSegmentBuilder {
public:
    /// @brief Parse G-code text into PlanningSegments with default config.
    /// @param gcodeText Raw G-code program text
    /// @return Result with segments, block metadata, and error status
    static PlanningSegmentResult fromText(const std::string& gcodeText);

    /// @brief Parse G-code text into PlanningSegments with custom config.
    /// @param gcodeText Raw G-code program text
    /// @param config Interpreter configuration
    /// @return Result with segments, block metadata, and error status
    static PlanningSegmentResult fromText(
        const std::string& gcodeText,
        const InterpreterConfig& config);

private:
    /// Map GCode::MotionSegment::Type → SegmentMotionType
    static SegmentMotionType mapMotionType(MotionSegment::Type type);

    /// Map GCode::Plane → InterpolationPlane
    static InterpolationPlane mapPlane(Plane plane);
};

} // namespace GCode
