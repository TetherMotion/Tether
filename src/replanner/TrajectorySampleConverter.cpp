/**
 * @file TrajectorySampleConverter.cpp
 * @brief Implementation of TrajectorySample → PiecewiseNurbsPath conversion
 */

#include "tether/motion_replanner/TrajectorySampleConverter.hpp"
#include "tether/motion_planner/geometry/NurbsCurve.hpp"
#include "tether/motion_planner/geometry/Vector.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <map>

namespace tether::motion::replanner {

//=============================================================================
// SegmentToPieceMap
//=============================================================================

std::optional<std::size_t> SegmentToPieceMap::pieceForSegment(
    int32_t segmentIndex) const {
    auto it = std::lower_bound(
        entries.begin(), entries.end(), segmentIndex,
        [](const std::pair<int32_t, std::size_t>& e, int32_t s) {
            return e.first < s;
        });
    if (it != entries.end() && it->first == segmentIndex) {
        return it->second;
    }
    return std::nullopt;
}

//=============================================================================
// Internal helpers
//=============================================================================

namespace {

/// Determine which axes are active (vary) across the trajectory.
/// Returns a bitmask of 9 bits: bit i set means axis i is active.
std::array<bool, 9> computeActiveAxes(
    const std::vector<GCodeExport::TrajectorySample>& samples) {
    std::array<bool, 9> active{};
    if (samples.empty()) return active;

    const auto& first = samples.front();
    for (const auto& s : samples) {
        for (int i = 0; i < 9; ++i) {
            if (std::abs(s.position[i] - first.position[i]) > 1e-12) {
                active[i] = true;
            }
        }
    }
    return active;
}

/// Count the number of active axes.
std::size_t countActive(const std::array<bool, 9>& active) {
    std::size_t n = 0;
    for (bool a : active) if (a) ++n;
    return n;
}

/// Extract an RVec of the active axes from a 9-axis position array.
RVec extractActiveRVec(const std::array<double, 9>& pos,
                       const std::array<bool, 9>& active,
                       std::size_t dim) {
    RVec v = RVec::zero(dim);
    std::size_t idx = 0;
    for (int i = 0; i < 9; ++i) {
        if (active[i]) {
            v[idx] = pos[i];
            ++idx;
        }
    }
    return v;
}

/// A group of consecutive samples belonging to one segment.
struct SegmentGroup {
    int32_t segmentIndex = -1;
    uint8_t motionType = 0;
    std::size_t startSample = 0; ///< Index into the samples vector
    std::size_t endSample = 0;   ///< Exclusive
};

/// Group samples by segmentIndex, preserving order.
std::vector<SegmentGroup> groupBySegment(
    const std::vector<GCodeExport::TrajectorySample>& samples) {
    std::vector<SegmentGroup> groups;
    if (samples.empty()) return groups;

    SegmentGroup current;
    current.segmentIndex = samples[0].segmentIndex;
    current.motionType = samples[0].motionType;
    current.startSample = 0;

    for (std::size_t i = 1; i < samples.size(); ++i) {
        if (samples[i].segmentIndex != current.segmentIndex) {
            current.endSample = i;
            groups.push_back(current);
            current.segmentIndex = samples[i].segmentIndex;
            current.motionType = samples[i].motionType;
            current.startSample = i;
        }
    }
    current.endSample = samples.size();
    groups.push_back(current);
    return groups;
}

/// Build a line NurbsCurve from the first and last sample of a group.
NurbsCurve buildLine(const std::vector<GCodeExport::TrajectorySample>& samples,
                     const SegmentGroup& group,
                     const std::array<bool, 9>& active,
                     std::size_t dim) {
    const auto& s0 = samples[group.startSample];
    const auto& s1 = samples[group.endSample - 1];
    RVec p0 = extractActiveRVec(s0.position, active, dim);
    RVec p1 = extractActiveRVec(s1.position, active, dim);
    return NurbsCurve::fromLine(p0, p1);
}

/// Fit a circle through 3 points in 2D (XY plane).
/// Returns (centerX, centerY, radius). Throws if the points are collinear.
struct Circle2D {
    double cx, cy, radius;
};

Circle2D fitCircle2D(double x0, double y0,
                     double x1, double y1,
                     double x2, double y2) {
    // Circumcenter via perpendicular bisectors.
    double d = 2.0 * (x0 * (y1 - y2) + x1 * (y2 - y0) + x2 * (y0 - y1));
    if (std::abs(d) < 1e-15) {
        throw std::runtime_error("Collinear points in circle fit");
    }
    double ux = ((x0*x0 + y0*y0) * (y1 - y2) +
                 (x1*x1 + y1*y1) * (y2 - y0) +
                 (x2*x2 + y2*y2) * (y0 - y1)) / d;
    double uy = ((x0*x0 + y0*y0) * (x2 - x1) +
                 (x1*x1 + y1*y1) * (x0 - x2) +
                 (x2*x2 + y2*y2) * (x1 - x0)) / d;
    double r = std::sqrt((x0 - ux)*(x0 - ux) + (y0 - uy)*(y0 - uy));
    return {ux, uy, r};
}

/// Compute the residual of a circle fit: max distance deviation of the
/// 3 input points from the fitted circle.
double circleResidual(const Circle2D& c,
                      double x0, double y0,
                      double x1, double y1,
                      double x2, double y2) {
    double d0 = std::abs(std::sqrt((x0-c.cx)*(x0-c.cx)+(y0-c.cy)*(y0-c.cy)) - c.radius);
    double d1 = std::abs(std::sqrt((x1-c.cx)*(x1-c.cx)+(y1-c.cy)*(y1-c.cy)) - c.radius);
    double d2 = std::abs(std::sqrt((x2-c.cx)*(x2-c.cx)+(y2-c.cy)*(y2-c.cy)) - c.radius);
    return std::max({d0, d1, d2});
}

/// Build a polyline (degree-1 multi-span NURBS) through all sample positions.
NurbsCurve buildPolyline(
    const std::vector<GCodeExport::TrajectorySample>& samples,
    const SegmentGroup& group,
    const std::array<bool, 9>& active,
    std::size_t dim) {

    std::size_t n = group.endSample - group.startSample;

    // Collect control points, skipping consecutive duplicates.
    std::vector<RVec> cps;
    cps.reserve(n);
    RVec prev = RVec::zero(dim);
    bool havePrev = false;
    for (std::size_t i = group.startSample; i < group.endSample; ++i) {
        RVec p = extractActiveRVec(samples[i].position, active, dim);
        if (!havePrev || p.distanceTo(prev) > 1e-12) {
            cps.push_back(p);
            prev = p;
            havePrev = true;
        }
    }

    if (cps.size() < 2) {
        throw std::invalid_argument(
            "Segment has fewer than 2 distinct points for polyline");
    }

    // Degree-1 NURBS: clamped knots with arc-length spacing.
    std::size_t numCPs = cps.size();
    std::vector<double> weights(numCPs, 1.0);
    std::vector<double> knots(numCPs + 2);
    knots[0] = 0.0;
    knots[1] = 0.0;
    for (std::size_t i = 0; i < numCPs - 1; ++i) {
        // Use cumulative arc length as knot spacing for a near-arc-length
        // parameterization (improves downstream quadrature convergence).
        knots[i + 2] = knots[i + 1] + cps[i].distanceTo(cps[i + 1]);
    }
    // Normalize to [0, 1].
    double total = knots.back();
    if (total > 1e-12) {
        for (double& k : knots) k /= total;
    } else {
        // Degenerate — all points coincide. Use uniform.
        for (std::size_t i = 0; i < knots.size(); ++i) {
            knots[i] = static_cast<double>(i) / (knots.size() - 1);
        }
    }

    return NurbsCurve(std::move(cps), std::move(weights), std::move(knots), 1);
}

/// Attempt to build an arc NurbsCurve from a group of samples.
/// Falls back to a polyline if the arc fit is poor or the geometry is
/// not in the XY plane.
NurbsCurve buildArcOrPolyline(
    const std::vector<GCodeExport::TrajectorySample>& samples,
    const SegmentGroup& group,
    const std::array<bool, 9>& active,
    std::size_t dim,
    double arcFitTolerance) {

    std::size_t n = group.endSample - group.startSample;

    // For a 2D XY arc, try the circumcenter fit.
    if (dim == 2 && active[0] && active[1]) {
        // Pick 3 representative points: start, middle, end.
        std::size_t i0 = group.startSample;
        std::size_t i2 = group.endSample - 1;
        std::size_t i1 = (i0 + i2) / 2;

        double x0 = samples[i0].position[0], y0 = samples[i0].position[1];
        double x1 = samples[i1].position[0], y1 = samples[i1].position[1];
        double x2 = samples[i2].position[0], y2 = samples[i2].position[1];

        try {
            Circle2D c = fitCircle2D(x0, y0, x1, y1, x2, y2);
            double res = circleResidual(c, x0, y0, x1, y1, x2, y2);

            if (res <= arcFitTolerance && c.radius > 1e-12) {
                // Build the arc via NurbsCurve::fromArc.
                RVec center = RVec::zero(2);
                center[0] = c.cx;
                center[1] = c.cy;
                RVec axis1 = RVec::zero(2);
                axis1[0] = 1.0;
                RVec axis2 = RVec::zero(2);
                axis2[1] = 1.0;

                double startAngle = std::atan2(y0 - c.cy, x0 - c.cx);
                double endAngle = std::atan2(y2 - c.cy, x2 - c.cx);

                // Determine sweep direction from motionType.
                // motionType 2 = arcCW (clockwise), 3 = arcCCW (counter-clockwise).
                double sweep = endAngle - startAngle;
                if (group.motionType == 2) {
                    // CW: sweep should be negative.
                    if (sweep > 0) sweep -= 2.0 * M_PI;
                } else {
                    // CCW: sweep should be positive.
                    if (sweep < 0) sweep += 2.0 * M_PI;
                }

                return NurbsCurve::fromArc(center, c.radius, axis1, axis2,
                                           startAngle, sweep);
            }
        } catch (const std::runtime_error&) {
            // Collinear points — fall through to polyline.
        }
    }

    // Fallback: polyline through all sample positions.
    return buildPolyline(samples, group, active, dim);
}

} // anonymous namespace

//=============================================================================
// Public API
//=============================================================================

PiecewiseNurbsPath convertTrajectory(
    const std::vector<GCodeExport::TrajectorySample>& samples,
    SegmentToPieceMap& map,
    const ConverterConfig& config) {

    if (samples.empty()) {
        throw std::invalid_argument("Cannot convert empty trajectory");
    }

    std::array<bool, 9> active = computeActiveAxes(samples);
    std::size_t dim = countActive(active);

    if (dim == 0) {
        throw std::invalid_argument(
            "No active axes in trajectory (all positions identical)");
    }

    auto groups = groupBySegment(samples);
    std::vector<NurbsCurve> pieces;
    pieces.reserve(groups.size());
    map.entries.clear();
    map.entries.reserve(groups.size());

    for (std::size_t pieceIndex = 0; pieceIndex < groups.size(); ++pieceIndex) {
        const auto& g = groups[pieceIndex];
        NurbsCurve curve =
            (g.motionType == 0 || g.motionType == 1)
                ? buildLine(samples, g, active, dim)
                : (g.motionType == 2 || g.motionType == 3)
                      ? buildArcOrPolyline(samples, g, active, dim,
                                           config.arcFitTolerance)
                      : buildPolyline(samples, g, active, dim);

        pieces.push_back(std::move(curve));
        map.entries.push_back({g.segmentIndex, pieceIndex});
    }

    return PiecewiseNurbsPath(std::move(pieces));
}

PiecewiseNurbsPath convertTrajectory(
    const std::vector<GCodeExport::TrajectorySample>& samples,
    const ConverterConfig& config) {
    SegmentToPieceMap map;
    return convertTrajectory(samples, map, config);
}

} // namespace tether::motion::replanner
