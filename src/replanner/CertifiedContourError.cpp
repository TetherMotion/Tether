/**
 * @file CertifiedContourError.cpp
 * @brief Implementation of certified contour error via pointCurveDistance
 */

#include "tether/motion_replanner/CertifiedContourError.hpp"
#include "tether/motion_planner/geometry/NurbsCurve.hpp"

#include <algorithm>
#include <cmath>
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
    for (std::size_t i = 0; i < path.numPieces(); ++i) {
        if (i == loc.piece) continue;

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

    for (std::size_t i = startPiece; i < endPiece; ++i) {
        if (i == loc.piece) continue;

        CertifiedContourError candidate = computeAgainstPiece(
            path, i, actualPosition, desiredArcLength, totalLength);

        if (candidate.contourError < best.contourError) {
            best = candidate;
        }
    }

    return best;
}

} // namespace tether::motion::replanner
