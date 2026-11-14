/**
 * @file PlanningSegmentConverter.cpp
 * @brief Convert GCode::PlanningSegment[] → PiecewiseNurbsPath
 */

#include "tether/motion_planner/geometry/PlanningSegmentConverter.hpp"
#include "tether/motion_planner/blend/OutsideCircleBlender.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace tether::motion {

using GCode::PlanningSegment;
using GCode::InterpolationPlane;

namespace {

constexpr double kTwoPi = 2.0 * M_PI;

/**
 * @brief Lift a planar arc NURBS into a helical arc.
 *
 * A true helix is not a NURBS curve, so the perpendicular component is
 * applied by offsetting each control point along the out-of-plane axis by
 * `delta * f`, where `f` is the control point's angular fraction of the
 * total sweep (computed from its direction relative to the arc center —
 * for a symmetric rational arc span the middle control point lies exactly
 * on the bisector ray). The planar projection stays an exact circle
 * (control-point in-plane coordinates and weights are untouched), the
 * endpoints match exactly, and the perpendicular ramp deviates from a
 * true linear-in-angle helix only within each ≤π span.
 */
NurbsCurve liftArcToHelix(const NurbsCurve& arc, const RVec& center,
                          int u, int v, int w,
                          double startAngle, double sweep,
                          double startW, double delta)
{
    auto cps = arc.controlPoints();
    for (std::size_t i = 0; i < cps.size(); ++i) {
        double f;
        if (i == 0) {
            f = 0.0;
        } else if (i + 1 == cps.size()) {
            f = 1.0;
        } else {
            double dth = std::atan2(cps[i][v] - center[v],
                                    cps[i][u] - center[u]) - startAngle;
            if (sweep > 0.0) {
                while (dth < -1e-9) dth += kTwoPi;
            } else {
                while (dth > 1e-9) dth -= kTwoPi;
            }
            f = std::clamp(dth / sweep, 0.0, 1.0);
        }
        cps[i][w] = startW + delta * f;
    }
    return NurbsCurve(std::move(cps), arc.weights(), arc.knots(),
                      arc.degree());
}

/**
 * @brief Build a Bézier/NURBS piece from a spline PlanningSegment.
 *
 * CubicSpline carries 4 control points (degree-3 Bézier),
 * QuadraticSpline 3 (degree-2), NURBS segments carry their own
 * knots/weights. Uniform-weight clamped knot vector is synthesised for
 * the Bézier cases.
 */
NurbsCurve splineNurbsFromSegment(const PlanningSegment& seg)
{
    std::vector<RVec> cps;
    cps.reserve(seg.controlPoints.size());
    for (const auto& p : seg.controlPoints)
        cps.push_back(RVec{p[0], p[1], p[2]});

    if (seg.motionType == GCode::SegmentMotionType::NURBS &&
        !seg.knots.empty() &&
        seg.weights.size() == cps.size() &&
        seg.knots.size() > cps.size() + 1) {
        const int degree =
            static_cast<int>(seg.knots.size() - cps.size() - 1);
        return NurbsCurve(std::move(cps), seg.weights, seg.knots, degree);
    }

    const int degree = static_cast<int>(cps.size()) - 1;
    std::vector<double> weights(cps.size(), 1.0);
    std::vector<double> knots(2 * (degree + 1), 0.0);
    std::fill(knots.begin() + degree + 1, knots.end(), 1.0);
    return NurbsCurve(std::move(cps), std::move(weights),
                      std::move(knots), degree);
}

} // namespace

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

        // Skip zero-length or near-zero-length segments. An arc whose
        // chord vanishes is a full circle (start == end): keep it — its
        // sweep (±2π) and radius carry the real geometry.
        double dx = end[0] - start[0];
        double dy = end[1] - start[1];
        double dz = end[2] - start[2];
        double len = std::sqrt(dx*dx + dy*dy + dz*dz);
        const bool validArc = seg.isArc() &&
            seg.arcRadius > 1e-9 && std::abs(seg.arcSweep) > 1e-9;
        if (len < 1e-6 && !validArc) continue;

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

        if (validArc) {
            // Build arc NURBS
            RVec center{seg.center[0], seg.center[1], seg.center[2]};

            // In-plane axes (u,v) and perpendicular (w). The interpreter
            // expresses startAngle/sweep in (u,v) plane coordinates,
            // matching the fromArc convention
            //   p(θ) = c + r·(cosθ·û + sinθ·v̂).
            int u = 0, v = 1, w = 2;
            GCode::InterpolationStrategy::getPlaneAxes(seg.plane, u, v, w);
            RVec axis1{0.0, 0.0, 0.0}, axis2{0.0, 0.0, 0.0};
            axis1[u] = 1.0;
            axis2[v] = 1.0;

            const double startAngle = std::atan2(
                start[v] - center[v],
                start[u] - center[u]);
            // Helical lead along the perpendicular axis.
            const double helixDelta = end[w] - start[w];

            try {
                auto curve = NurbsCurve::fromArc(
                    center, seg.arcRadius, axis1, axis2,
                    startAngle, seg.arcSweep);
                if (std::abs(helixDelta) > 1e-12) {
                    curve = liftArcToHelix(curve, center, u, v, w,
                                           startAngle, seg.arcSweep,
                                           start[w], helixDelta);
                }
                curves.push_back(std::move(curve));
                deviations.push_back(segDeviation);
                extruderSpeeds.push_back(segExtruderSpeed);
                feedRates.push_back(segFeedRate);
                cumulativeArcLength += curves.back().length();
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
        } else if (seg.isSpline() && seg.controlPoints.size() >= 2) {
            // G5/G5.1/G5.2 splines — exact Bézier/NURBS pieces.
            try {
                curves.push_back(splineNurbsFromSegment(seg));
                deviations.push_back(segDeviation);
                extruderSpeeds.push_back(segExtruderSpeed);
                feedRates.push_back(segFeedRate);
                cumulativeArcLength += curves.back().length();
            } catch (...) {
                // Fall back to the chord on malformed spline data
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
