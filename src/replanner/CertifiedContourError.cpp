/**
 * @file CertifiedContourError.cpp
 * @brief Implementation of certified contour error via pointCurveDistance
 */

#include "tether/motion_replanner/CertifiedContourError.hpp"
#include "tether/motion_planner/geometry/NurbsCurve.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace tether::motion::replanner {

namespace {

/// Compute the certified contour error against a single piece, with
/// the desired arc length for lag decomposition.
CertifiedContourError computeAgainstPiece(
    const PiecewiseNurbsPath& path,
    std::size_t pieceIndex,
    const RVec& actualPosition,
    double desiredArcLength,
    double pathTotalLength) {

    const NurbsCurve& curve = path.piece(pieceIndex);

    // Certified global minimum distance from actual position to this piece.
    DistanceResult dr = pointCurveDistance(curve, actualPosition);

    CertifiedContourError result;
    result.contourError = dr.distance;
    result.closestPoint = dr.closestPoint;
    result.closestParam = dr.u;
    result.closestPiece = pieceIndex;
    result.isCertified = true;

    // Combined position error: Euclidean distance to the desired position.
    // The desired position is at the desired arc length on the path.
    double clampedS = std::clamp(desiredArcLength, 0.0, pathTotalLength);
    RVec desiredPos = path.evaluatePosition(clampedS);
    result.combinedPositionError = actualPosition.distanceTo(desiredPos);

    // Lag error: signed arc-length offset from desired to closest.
    // Compute the arc length of the closest point on the path.
    // First, find the global arc length of the closest point.
    // The closest point is at parameter u on pieceIndex.
    // We need the prefix arc length up to pieceIndex + the arc length
    // from the piece start to u.
    //
    // PiecewiseNurbsPath::locate(s) maps s → (piece, localS), but we need
    // the inverse: given (piece, u), compute the global arc length.
    // We compute it by summing piece lengths up to pieceIndex and adding
    // the arc length from the piece's start to u.
    //
    // However, NurbsCurve::arcLengthTo(u) gives the arc length from the
    // curve's knotMin to u. We need the global arc length.
    //
    // We approximate by computing the prefix lengths manually.
    double prefixLength = 0.0;
    for (std::size_t i = 0; i < pieceIndex; ++i) {
        prefixLength += path.piece(i).length();
    }
    double closestArcLength = prefixLength + curve.arcLengthTo(dr.u);

    result.lagError = closestArcLength - desiredArcLength;

    return result;
}

/// Lower bound on the distance from a point to a NURBS curve, computed
/// from the axis-aligned bounding box (AABB) of the curve's control
/// points. A NURBS curve lies within the convex hull of its control
/// points (a consequence of the partition-of-unity property of B-spline
/// basis functions), so the true minimum distance from any point to the
/// curve is ≥ the distance from that point to the AABB of the control
/// points.
///
/// This is the key pruning optimization for computeCertifiedContourError:
/// on long paths with many pieces, most pieces are far from the actual
/// position. The AABB distance is a cheap O(n·dim) computation (just
/// min/max over control points + a clamped-distance calculation), while
/// pointCurveDistance is expensive (Bézier decomposition + Bernstein
/// root isolation per span). If the AABB lower bound already exceeds
/// the current best distance, the piece cannot improve the result and
/// can be skipped.
///
/// @return A non-negative lower bound on the true distance, or 0.0 if
///         the point is inside the AABB (no useful pruning possible).
double aabbDistanceLowerBound(const NurbsCurve& curve, const RVec& p) {
    const std::size_t dim = curve.dim();
    const auto& cps = curve.controlPoints();
    if (cps.empty()) return 0.0;

    // Compute the AABB of the control points.
    std::vector<double> lo(dim), hi(dim);
    for (std::size_t d = 0; d < dim; ++d) {
        lo[d] = std::numeric_limits<double>::infinity();
        hi[d] = std::numeric_limits<double>::lowest();
    }
    for (const RVec& cp : cps) {
        for (std::size_t d = 0; d < dim; ++d) {
            const double v = cp.unchecked(d);
            lo[d] = std::min(lo[d], v);
            hi[d] = std::max(hi[d], v);
        }
    }

    // Squared distance from p to the AABB: for each axis, if p is outside
    // [lo, hi], add the squared excess; if inside, add 0.
    double d2 = 0.0;
    for (std::size_t d = 0; d < dim; ++d) {
        const double v = p.unchecked(d);
        if (v < lo[d]) {
            const double delta = lo[d] - v;
            d2 += delta * delta;
        } else if (v > hi[d]) {
            const double delta = v - hi[d];
            d2 += delta * delta;
        }
        // else: inside on this axis → 0 contribution
    }
    return std::sqrt(d2);
}

} // anonymous namespace

CertifiedContourError computeCertifiedContourError(
    const PiecewiseNurbsPath& path,
    const RVec& actualPosition,
    double desiredArcLength) {

    if (actualPosition.dim() != path.dim()) {
        throw std::invalid_argument(
            "Dimension mismatch: actual position has dim " +
            std::to_string(actualPosition.dim()) +
            ", path has dim " + std::to_string(path.dim()));
    }

    double totalLength = path.totalLength();

    // Search all pieces for the global minimum distance.
    // Start with the piece nearest the desired arc length, then expand.
    PiecewiseNurbsPath::Located loc = path.locate(desiredArcLength);

    CertifiedContourError best = computeAgainstPiece(
        path, loc.piece, actualPosition, desiredArcLength, totalLength);

    // Search all other pieces (the desired piece is usually closest, but
    // on tight corners the actual position may be closest to a neighbor).
    // Pruning: skip pieces whose AABB distance lower bound exceeds the
    // current best — the true distance cannot be smaller (see
    // aabbDistanceLowerBound for the convex-hull guarantee).
    for (std::size_t i = 0; i < path.numPieces(); ++i) {
        if (i == loc.piece) continue;

        if (aabbDistanceLowerBound(path.piece(i), actualPosition)
            >= best.contourError) {
            continue;
        }

        CertifiedContourError candidate = computeAgainstPiece(
            path, i, actualPosition, desiredArcLength, totalLength);

        if (candidate.contourError < best.contourError) {
            best = candidate;
        }
    }

    return best;
}

CertifiedContourError computeCertifiedContourErrorLocal(
    const PiecewiseNurbsPath& path,
    const RVec& actualPosition,
    double desiredArcLength,
    std::size_t searchWindow) {

    if (actualPosition.dim() != path.dim()) {
        throw std::invalid_argument(
            "Dimension mismatch: actual position has dim " +
            std::to_string(actualPosition.dim()) +
            ", path has dim " + std::to_string(path.dim()));
    }

    double totalLength = path.totalLength();
    PiecewiseNurbsPath::Located loc = path.locate(desiredArcLength);

    // Determine the search range: [loc.piece - searchWindow, loc.piece + searchWindow]
    std::size_t startPiece = (loc.piece > searchWindow)
        ? loc.piece - searchWindow : 0;
    std::size_t endPiece = std::min(loc.piece + searchWindow + 1,
                                    path.numPieces());

    CertifiedContourError best = computeAgainstPiece(
        path, loc.piece, actualPosition, desiredArcLength, totalLength);

    // Pruning: skip pieces whose AABB distance lower bound exceeds the
    // current best (see computeCertifiedContourError for rationale).
    for (std::size_t i = startPiece; i < endPiece; ++i) {
        if (i == loc.piece) continue;

        if (aabbDistanceLowerBound(path.piece(i), actualPosition)
            >= best.contourError) {
            continue;
        }

        CertifiedContourError candidate = computeAgainstPiece(
            path, i, actualPosition, desiredArcLength, totalLength);

        if (candidate.contourError < best.contourError) {
            best = candidate;
        }
    }

    return best;
}

} // namespace tether::motion::replanner
