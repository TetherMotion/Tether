/**
 * @file CertifiedCornerDetection.hpp
 * @brief Certified corner detection via CornerAnalyzer (M13)
 *
 * @details
 * Replaces the old angle-threshold-on-3-samples corner detection in
 * MotionReplanner::isCriticalPoint / classifyCriticalPoint with the
 * kernel's CornerAnalyzer (M13), which:
 *
 * - Extracts exact tangents from the NURBS arc-length derivatives
 *   (G.18–G.21), not from finite differences of sample positions.
 * - Classifies corners as Straight / Corner / Cusp using the clamped
 *   acos angle θ = acos(clamp(t_in·t_out, −1, 1)).
 * - Builds an orthonormal basis (e₁, e₂) of the tangent plane via
 *   modified Gram-Schmidt, enabling downstream blend construction.
 * - Carries curvature and jounce vectors at the junction for G²/G³
 *   blend boundary conditions.
 *
 * ## Why this is better
 *
 * The old method computed the angle from 3 consecutive TrajectorySample
 * positions via finite differences. This is:
 * - Noisy: sensitive to sample spacing and quantization.
 * - Wrong on curves: the "angle" at a junction between two arcs is not
 *   the angle of the chord through 3 samples — it's the angle between
 *   the tangent vectors at the junction.
 * - Missing classification: no Cusp detection, no plane basis.
 *
 * The certified method uses the exact NURBS derivatives at the junction,
 * giving the true turning angle and a proper geometric classification.
 *
 * @see CornerAnalysis.hpp for the CornerAnalyzer API.
 */

#pragma once

#include "tether/motion_planner/geometry/PiecewiseNurbsPath.hpp"
#include "tether/motion_planner/blend/CornerAnalysis.hpp"

#include <vector>
#include <cstddef>
#include <optional>
#include <string>

namespace tether::motion::replanner {

/// One junction analysis result, augmented with the piece indices.
struct CertifiedJunction {
    /// Index of the incoming piece (0..numPieces-2).
    std::size_t pieceInIndex = 0;

    /// Index of the outgoing piece (1..numPieces-1).
    std::size_t pieceOutIndex = 0;

    /// The full CornerAnalysis from CornerAnalyzer::analyze.
    CornerAnalysis analysis;

    /// Human-readable classification string for backward compatibility
    /// with the old MotionReplanner criticalPointType field.
    std::string kindString() const;
};

/// Result of analyzing all junctions in a path.
struct CertifiedCornerDetection {
    /// One entry per junction (numPieces - 1 entries).
    std::vector<CertifiedJunction> junctions;

    /// Count of junctions by kind.
    int straightCount = 0;
    int cornerCount = 0;
    int cuspCount = 0;

    /// Find the junction at the boundary between pieceInIndex and
    /// pieceOutIndex, or nullopt if not found.
    std::optional<CertifiedJunction> junctionAt(
        std::size_t pieceInIndex) const;
};

/**
 * @brief Analyze all junctions in a piecewise NURBS path.
 *
 * @param path The path to analyze.
 * @param minAngleRad Corners with θ < this are "Straight" (default ~1°).
 * @param maxAngleRad Corners with θ > this are "Cusp" (default ~175°).
 * @return The corner detection result with one entry per junction.
 *
 * @throws std::invalid_argument if the path has fewer than 2 pieces
 *         (no junctions to analyze).
 */
CertifiedCornerDetection detectCorners(
    const PiecewiseNurbsPath& path,
    double minAngleRad = 0.017453292519943295,  // 1°
    double maxAngleRad = 3.054326190990077);     // 175°

} // namespace tether::motion::replanner
