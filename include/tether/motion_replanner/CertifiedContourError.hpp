/**
 * @file CertifiedContourError.hpp
 * @brief Certified geometric contour error via pointCurveDistance (M8/M9)
 *
 * @details
 * Replaces the old tangent-projection contour error in
 * MotionReplanner::computeError with the kernel's certified
 * point-to-NURBS distance (tether::motion::pointCurveDistance).
 *
 * ## Why this is better
 *
 * The old method projected the position error onto the desired velocity
 * tangent and took the perpendicular remainder. This is only correct when
 * the path is locally straight — on curved paths (arcs, blends), the
 * tangent-line approximation introduces an error proportional to
 * κ·d² (curvature × squared distance), which can be significant at high
 * feedrates on tight arcs.
 *
 * The certified method uses tether::motion::pointCurveDistance, which
 * finds the TRUE global minimum distance from the actual position to the
 * NURBS curve via Bernstein root isolation (M8/M9). No stationary point
 * can be missed — the result is the exact geometric contour error up to
 * root-isolation width and floating-point round-off.
 *
 * ## Lag error decomposition
 *
 * Once the true closest point on the curve is known, the lag error
 * (along-path component) is computed as the arc-length difference between
 * the desired position and the closest point, with sign determined by the
 * tangent direction. This is more accurate than the old tangent-projection
 * lag because it uses the actual closest point, not a tangent-line
 * approximation.
 *
 * ## Performance
 *
 * pointCurveDistance is more expensive than tangent projection (Bernstein
 * root isolation per span). For real-time use, the caller can:
 * 1. Use the cheap tangent-projection as a fast-path alert trigger.
 * 2. Compute the certified distance only when the cheap estimate exceeds
 *    a fraction of the threshold (e.g. 0.5×).
 *
 * @see PointCurveDistance.hpp for the certified distance algorithm.
 * @see MotionReplanner::computeError for the old tangent-projection method.
 */

#pragma once

#include "tether/motion_planner/geometry/PiecewiseNurbsPath.hpp"
#include "tether/motion_planner/geometry/PointCurveDistance.hpp"
#include "tether/motion_planner/geometry/Vector.hpp"

#include <optional>
#include <cstddef>

namespace tether::motion::replanner {

/// Certified contour error result — carries the true geometric distance
/// plus the decomposition into contour (perpendicular) and lag (along-path)
/// components.
struct CertifiedContourError {
    /// True geometric distance from the actual position to the path
    /// (certified global minimum via Bernstein root isolation, M8/M9).
    double contourError = 0.0;

    /// Lag error: signed arc-length offset from the desired position to
    /// the closest point on the path. Positive = ahead, negative = behind.
    double lagError = 0.0;

    /// Combined 3D position error magnitude (Euclidean distance to the
    /// desired position, not the closest point — same as the old method
    /// for backward compatibility).
    double combinedPositionError = 0.0;

    /// The closest point on the path (S(u) achieving the minimum distance).
    RVec closestPoint{};

    /// The parameter u on the piece's NurbsCurve achieving the minimum.
    double closestParam = 0.0;

    /// Index of the piece on which the closest point was found.
    std::size_t closestPiece = 0;

    /// Whether the certified computation was used (vs. a fast-path fallback).
    bool isCertified = true;
};

/**
 * @brief Compute the certified contour error of an actual position
 *        against a piecewise NURBS path.
 *
 * @param path The desired path (from TrajectorySampleConverter).
 * @param actualPosition The measured position (active axes only, same
 *        dimension as the path).
 * @param desiredArcLength The desired arc length position (from
 *        TrajectorySample.pathPosition), used as a starting hint for
 *        locating the nearest piece and for lag error computation.
 * @return The certified contour error.
 *
 * @throws std::invalid_argument on dimension mismatch.
 */
CertifiedContourError computeCertifiedContourError(
    const PiecewiseNurbsPath& path,
    const RVec& actualPosition,
    double desiredArcLength);

/**
 * @brief Compute the certified contour error, searching only a local
 *        neighborhood of the path around the desired arc length.
 *
 * This is the faster variant for real-time use: it searches only the
 * pieces near the desired arc length (±searchWindow pieces) rather than
 * all pieces. The result is certified within the searched neighborhood;
 * if the actual position is far from the desired position, the global
 * minimum may be missed. Use the full search for offline analysis.
 *
 * @param path The desired path.
 * @param actualPosition The measured position.
 * @param desiredArcLength The desired arc length position.
 * @param searchWindow Number of pieces to search on each side of the
 *        desired position (default 1 = search 3 pieces total).
 * @return The certified contour error (within the searched neighborhood).
 */
CertifiedContourError computeCertifiedContourErrorLocal(
    const PiecewiseNurbsPath& path,
    const RVec& actualPosition,
    double desiredArcLength,
    std::size_t searchWindow = 1);

} // namespace tether::motion::replanner
