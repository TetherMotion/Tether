/**
 * @file SegmentConverter.cpp
 * @brief Implementation of SegmentConverter — MotionSegment → NurbsCurve.
 *
 * Equation (M6) refers to the exact rational-quadratic arc construction;
 * see docs/motion/GeometryFoundations.md.
 */

#include "tether/motion_planner/blend/SegmentConverter.hpp"

#include <cmath>
#include <stdexcept>

namespace tether::motion {

namespace {
using MotionPlanner::MAX_MOTION_AXES;
using MotionPlanner::MotionSegment;
using MotionPlanner::MotionSegmentType;
using MotionPlanner::ArcPlane;
} // namespace

RVec SegmentConverter::extractActive(
    const std::array<double, MAX_MOTION_AXES>& pos,
    const std::array<bool, MAX_MOTION_AXES>& activeAxes,
    std::size_t numActive) {
    // If no active axes are flagged (e.g. a dwell or zero-length move),
    // fall back to the first axis so the curve has dimension ≥ 1.
    if (numActive == 0) {
        RVec v = RVec::zero(1);
        v[0] = pos[0];
        return v;
    }
    RVec v = RVec::zero(numActive);
    std::size_t idx = 0;
    for (std::size_t i = 0; i < MAX_MOTION_AXES; ++i) {
        if (activeAxes[i]) {
            v[idx++] = pos[i];
        }
    }
    return v;
}

NurbsCurve SegmentConverter::convertLinear(const MotionSegment& seg) {
    RVec a = extractActive(seg.startPosition, seg.activeAxes, seg.numActiveAxes);
    RVec b = extractActive(seg.endPosition, seg.activeAxes, seg.numActiveAxes);
    // If start == end (zero-length), fromLine throws — but a zero-length
    // linear segment is degenerate. Let the exception propagate.
    return NurbsCurve::fromLine(a, b);
}

NurbsCurve SegmentConverter::convertArc(const MotionSegment& seg) {
    if (seg.arcRadius <= 0.0) {
        // Degenerate arc — treat as a line.
        return convertLinear(seg);
    }

    // Determine the two active plane axes from ArcPlane.
    std::size_t i1 = 0, i2 = 1;
    switch (seg.arcPlane) {
        case ArcPlane::XY: i1 = 0; i2 = 1; break;
        case ArcPlane::XZ: i1 = 0; i2 = 2; break;
        case ArcPlane::YZ: i1 = 1; i2 = 2; break;
    }

    // Build the active-axis list: the two plane axes plus any other
    // active axes (linearly interpolated along the arc). For a pure
    // planar arc, only i1 and i2 are active.
    std::vector<std::size_t> activeIdx;
    activeIdx.push_back(i1);
    activeIdx.push_back(i2);
    for (std::size_t i = 0; i < MAX_MOTION_AXES; ++i) {
        if (i == i1 || i == i2) continue;
        if (seg.activeAxes[i]) activeIdx.push_back(i);
    }
    const std::size_t dim = activeIdx.size();

    // Center in the active-axis subspace.
    RVec center = RVec::zero(dim);
    for (std::size_t k = 0; k < dim; ++k) {
        center[k] = seg.arcCenter[activeIdx[k]];
    }

    // Start angle from the start position relative to center.
    const double sx = seg.startPosition[i1] - seg.arcCenter[i1];
    const double sy = seg.startPosition[i2] - seg.arcCenter[i2];
    const double startAngle = std::atan2(sy, sx);

    // Sweep: seg.arcSweep is signed (positive for CCW, negative for CW).
    // The (M6) fromArc takes a signed sweep directly.
    double sweep = seg.arcSweep;
    // CW arcs (MotionSegmentType::ArcCW) have negative sweep per the
    // factory (arcCCW flips the sign). Ensure consistency:
    if (seg.type == MotionSegmentType::ArcCW && sweep > 0.0) {
        sweep = -sweep;
    }
    if (seg.type == MotionSegmentType::ArcCCW && sweep < 0.0) {
        sweep = -sweep;
    }

    // Orthonormal axes in the active subspace. axis1 = e_{i1},
    // axis2 = e_{i2} (unit vectors along the two plane axes).
    RVec axis1 = RVec::zero(dim);
    RVec axis2 = RVec::zero(dim);
    axis1[0] = 1.0; // i1 is the first active axis
    axis2[1] = 1.0; // i2 is the second active axis

    NurbsCurve arc = NurbsCurve::fromArc(center, seg.arcRadius,
                                         axis1, axis2, startAngle, sweep);

    // If there are extra active axes (helical motion), we need to
    // augment the arc with linear interpolation on those axes. The
    // fromArc construction only handles the 2 plane axes; the extra
    // axes are interpolated linearly across the control points.
    if (dim > 2) {
        // Replace the extra-axis coordinates of each control point with
        // a linear interpolation from start to end. We rebuild the
        // control points.
        std::vector<RVec> cps = arc.controlPoints();
        const std::size_t nCp = cps.size();
        for (std::size_t i = 0; i < nCp; ++i) {
            const double alpha = (nCp == 1) ? 0.0
                : static_cast<double>(i) / static_cast<double>(nCp - 1);
            for (std::size_t k = 2; k < dim; ++k) {
                std::size_t ax = activeIdx[k];
                cps[i][k] = seg.startPosition[ax] * (1.0 - alpha) +
                             seg.endPosition[ax] * alpha;
            }
        }
        // Reconstruct the curve with the augmented control points.
        // (The arc's weights and knots are unchanged; only the extra
        // axes are modified, which doesn't affect the rational
        // construction in the plane axes.)
        return NurbsCurve(std::move(cps), arc.weights(), arc.knots(),
                          arc.degree());
    }

    return arc;
}

NurbsCurve SegmentConverter::convertNurbs(const MotionSegment& seg) {
    if (seg.controlPoints.empty()) {
        throw std::invalid_argument(
            "SegmentConverter::convertNurbs: empty control points");
    }
    if (seg.degree == 0) {
        throw std::invalid_argument(
            "SegmentConverter::convertNurbs: degree is zero");
    }

    // Determine the active axes from the first control point vs the
    // start position. For NURBS segments, numActiveAxes may not be set;
    // fall back to comparing against the start position to find moving
    // axes. If that fails, use all axes up to the first non-zero one.
    std::array<bool, MAX_MOTION_AXES> activeAxes{};
    std::size_t numActive = 0;
    if (seg.numActiveAxes > 0) {
        activeAxes = seg.activeAxes;
        numActive = seg.numActiveAxes;
    } else {
        // Detect active axes by comparing the first and last control points.
        for (std::size_t i = 0; i < MAX_MOTION_AXES; ++i) {
            if (std::abs(seg.controlPoints.front()[i] -
                         seg.controlPoints.back()[i]) > 1e-12) {
                activeAxes[i] = true;
                ++numActive;
            }
        }
        if (numActive == 0) {
            // All axes constant — pick the first axis as active.
            activeAxes[0] = true;
            numActive = 1;
        }
    }

    // Extract the active-axis coordinates from each control point.
    std::vector<RVec> cps;
    cps.reserve(seg.controlPoints.size());
    for (const auto& cp : seg.controlPoints) {
        cps.push_back(extractActive(cp, activeAxes, numActive));
    }

    // Weights: default to all-ones if empty.
    std::vector<double> weights = seg.weights;
    if (weights.empty()) {
        weights.assign(cps.size(), 1.0);
    }

    return NurbsCurve(std::move(cps), std::move(weights),
                      seg.knots, static_cast<int>(seg.degree));
}

std::optional<NurbsCurve> SegmentConverter::convert(const MotionSegment& seg) {
    switch (seg.type) {
        case MotionSegmentType::Linear:
        case MotionSegmentType::Rapid:
            return convertLinear(seg);
        case MotionSegmentType::ArcCW:
        case MotionSegmentType::ArcCCW:
            return convertArc(seg);
        case MotionSegmentType::CubicSpline:
        case MotionSegmentType::QuadSpline:
        case MotionSegmentType::NURBS:
            return convertNurbs(seg);
        case MotionSegmentType::Dwell:
            // No geometry — skip.
            return std::nullopt;
    }
    // Unreachable, but silence compiler warnings.
    return std::nullopt;
}

std::vector<NurbsCurve> SegmentConverter::convertAll(
    const std::vector<MotionSegment>& segments) {
    std::vector<NurbsCurve> curves;
    curves.reserve(segments.size());
    for (const auto& seg : segments) {
        auto curve = convert(seg);
        if (curve) {
            curves.push_back(std::move(*curve));
        }
    }
    return curves;
}

} // namespace tether::motion
