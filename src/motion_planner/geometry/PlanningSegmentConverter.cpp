/**
 * @file PlanningSegmentConverter.cpp
 * @brief Convert GCode::PlanningSegment[] → PiecewiseNurbsPath
 */

#include "tether/motion_planner/geometry/PlanningSegmentConverter.hpp"
#include "tether/motion_planner/blend/OutsideCircleBlender.hpp"

#include <cmath>
#include <limits>

namespace tether::motion {

using GCode::PlanningSegment;
using GCode::InterpolationPlane;

PlanningSegmentNurbsResult piecewiseNurbsFromSegments(
    const std::vector<PlanningSegment>& segments)
{
    std::vector<NurbsCurve> curves;
    std::vector<float> deviations;
    std::vector<float> extruderSpeeds;
    std::vector<double> feedRates;
    std::vector<DwellPoint> dwellPoints;
    curves.reserve(segments.size());

    // Track cumulative arc length to record dwell positions.
    double cumulativeArcLength = 0.0;

    for (const auto& seg : segments) {
        // Extract 3D start/end positions (XYZ only)
        RVec start{seg.start[0], seg.start[1], seg.start[2]};
        RVec end{seg.end[0], seg.end[1], seg.end[2]};

        // Dwell segments (G4): record the position and duration, then skip.
        // The dwell occurs at the current cumulative arc length (the
        // boundary between the previous and next NURBS pieces).
        if (seg.motionType == GCode::SegmentMotionType::Dwell) {
            DwellPoint dp;
            dp.arcLength = cumulativeArcLength;
            dp.duration = seg.segmentTime;  // G4 dwell time in seconds
            dwellPoints.push_back(dp);
            continue;
        }

        // Skip zero-length or near-zero-length segments
        double dx = end[0] - start[0];
        double dy = end[1] - start[1];
        double dz = end[2] - start[2];
        double len = std::sqrt(dx*dx + dy*dy + dz*dz);
        if (len < 1e-6) continue;

        // Per-piece attributes (stored in repurposed velocity fields)
        float segDeviation = static_cast<float>(seg.entryVelocity);
        float segExtruderSpeed = static_cast<float>(seg.exitVelocity);
        // Feed rate: convert from mm/min to mm/s. Rapid moves (G0) use
        // a large value so they are not limiting (the machine max velocity
        // will cap them). A feedRate of 0 means "not specified" → no limit.
        double segFeedRate = 0.0;
        if (seg.feedRate > 0.0) {
            segFeedRate = seg.isRapid
                ? std::numeric_limits<double>::infinity()
                : seg.feedRate / 60.0;
        }

        if (seg.isArc() && seg.arcRadius > 1e-9) {
            // Build arc NURBS
            RVec center{seg.center[0], seg.center[1], seg.center[2]};

            // Determine arc plane axes
            RVec axis1, axis2;
            switch (seg.plane) {
                case InterpolationPlane::XZ:
                    axis1 = RVec{1.0, 0.0, 0.0};
                    axis2 = RVec{0.0, 0.0, 1.0};
                    break;
                case InterpolationPlane::YZ:
                    axis1 = RVec{0.0, 1.0, 0.0};
                    axis2 = RVec{0.0, 0.0, 1.0};
                    break;
                case InterpolationPlane::XY:
                default:
                    axis1 = RVec{1.0, 0.0, 0.0};
                    axis2 = RVec{0.0, 1.0, 0.0};
                    break;
            }

            // Start angle from start position relative to center
            double startAngle = std::atan2(
                start[1] - center[1],
                start[0] - center[0]
            );
            double sweepAngle = seg.arcSweep;

            try {
                auto curve = NurbsCurve::fromArc(
                    center, seg.arcRadius, axis1, axis2,
                    startAngle, sweepAngle);
                curves.push_back(std::move(curve));
                deviations.push_back(segDeviation);
                extruderSpeeds.push_back(segExtruderSpeed);
                feedRates.push_back(segFeedRate);
                cumulativeArcLength += len;
            } catch (...) {
                // Fall back to line if arc construction fails
                try {
                    curves.push_back(NurbsCurve::fromLine(start, end));
                    deviations.push_back(segDeviation);
                    extruderSpeeds.push_back(segExtruderSpeed);
                    feedRates.push_back(segFeedRate);
                    cumulativeArcLength += len;
                } catch (...) {
                    // Skip degenerate segments
                }
            }
        } else {
            // Linear or rapid — build line NURBS
            try {
                curves.push_back(NurbsCurve::fromLine(start, end));
                deviations.push_back(segDeviation);
                extruderSpeeds.push_back(segExtruderSpeed);
                feedRates.push_back(segFeedRate);
                cumulativeArcLength += len;
            } catch (...) {
                // Skip degenerate segments
            }
        }
    }

    // ── Outside circle blend (negative G64 P) ──
    // When any segment has a negative blendTolerance (G64 P < 0), apply
    // the outside circle blend: place a circle of radius |P| at each
    // corner vertex, find the intersection with the exact path, and
    // replace the corner with the major (outside) arc.
    double blendRadius = 0.0;
    for (const auto& seg : segments) {
        if (seg.blendTolerance < 0.0) {
            blendRadius = std::abs(seg.blendTolerance);
            break;
        }
    }

    if (blendRadius > 0.0 && !curves.empty()) {
        OutsideCircleBlendConfig blendConfig;
        blendConfig.radius = blendRadius;
        PiecewiseNurbsPath originalPath(std::move(curves));
        auto blendResult = OutsideCircleBlender::blend(originalPath, blendConfig);
        if (blendResult.path && blendResult.blendedCount > 0) {
            // Replace the path with the blended version. Map per-piece
            // attributes from the original pieces to the blended pieces
            // using the sourcePieceIndices mapping.
            auto& blended = *blendResult.path;
            const auto& srcIdx = blendResult.sourcePieceIndices;
            // Save originals before resizing
            auto origDeviations = std::move(deviations);
            auto origExtruderSpeeds = std::move(extruderSpeeds);
            auto origFeedRates = std::move(feedRates);
            deviations.assign(blended.numPieces(), 0.0f);
            extruderSpeeds.assign(blended.numPieces(), 0.0f);
            feedRates.assign(blended.numPieces(),
                             std::numeric_limits<double>::infinity());
            for (std::size_t j = 0; j < blended.numPieces() && j < srcIdx.size(); ++j) {
                std::size_t src = srcIdx[j];
                if (src < origDeviations.size()) {
                    deviations[j] = origDeviations[src];
                    extruderSpeeds[j] = origExtruderSpeeds[src];
                }
                if (src < origFeedRates.size()) {
                    feedRates[j] = origFeedRates[src];
                }
            }
            return {std::move(blended),
                    std::move(deviations),
                    std::move(extruderSpeeds),
                    std::move(feedRates),
                    std::move(dwellPoints)};
        }
        // Blend didn't apply — return the original path
        return {std::move(originalPath),
                std::move(deviations),
                std::move(extruderSpeeds),
                std::move(feedRates),
                std::move(dwellPoints)};
    }

    return {PiecewiseNurbsPath(std::move(curves)),
            std::move(deviations),
            std::move(extruderSpeeds),
            std::move(feedRates),
            std::move(dwellPoints)};
}

} // namespace tether::motion
