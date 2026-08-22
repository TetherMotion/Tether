/**
 * @file OutsideCircleBlender.hpp
 * @brief Outside (negative G64) corner blend via circle-path intersection.
 *
 * @details
 * When G64 P is negative, the tool path must go *outside* the corner,
 * creating an "ear" that bulges away from the part. The algorithm:
 *
 *   1. For each junction between two NURBS pieces, identify the corner
 *      vertex V (shared endpoint).
 *   2. Construct a circle centered at V with radius r = |G64 P|.
 *   3. Compute the analytical intersection of the *exact* path (line or
 *      arc NURBS piece) with this circle:
 *      - Line piece: point at distance r from V along the line direction.
 *      - Arc piece:  circle-circle intersection (subtract equations →
 *        linear radical axis → quadratic).
 *   4. Trim the incoming piece to end at the intersection point P1.
 *   5. Trim the outgoing piece to start at the intersection point P2.
 *   6. Insert a circular arc (NurbsCurve::fromArc) from P1 to P2 that
 *      goes the *outside* way (the major arc, sweeping > π).
 *
 * The result is a PiecewiseNurbsPath where each outside-blend corner is
 * replaced by an exact circular arc of radius |G64 P| centered at the
 * original corner vertex.
 *
 * All intersection math is closed-form / analytical:
 *   - Line-circle:  |A + t·d - V|² = r²  →  quadratic in t.
 *   - Circle-circle: |P - C|² = R² and |P - V|² = r² → radical axis
 *     (linear) + quadratic.
 *
 * Limitations:
 *   - Both adjacent pieces must be long enough to reach the circle (the
 *     intersection point must lie within the piece). If not, the corner
 *     is left unblended (exact stop).
 *   - Only line (degree-1) and arc (degree-2 rational) pieces are handled
 *     analytically. Higher-degree pieces fall back to a numerical
 *     arc-length bisection.
 */

#pragma once

#include "tether/motion_planner/geometry/NurbsCurve.hpp"
#include "tether/motion_planner/geometry/PiecewiseNurbsPath.hpp"

#include <cmath>
#include <limits>
#include <optional>
#include <vector>


namespace tether::motion {

/// @brief Configuration for the outside circle blender.
struct OutsideCircleBlendConfig {
    /// Blend radius (mm). Must be > 0. Typically |G64 P value|.
    double radius = 0.0;

    /// Minimum piece length to attempt blending (mm). Pieces shorter than
    /// this are left unblended. Default 0.01 mm.
    double minPieceLength = 0.01;

    /// Maximum fraction of a piece's length that may be trimmed for a
    /// single blend. Prevents a single corner from consuming an entire
    /// short segment. Default 0.5 (half the piece).
    double maxTrimFraction = 0.5;

    /// Tolerance for geometric comparisons (mm).
    double tol = 1e-9;

    /// Transition fraction for G2-continuous blending. When > 0, quintic
    /// Bézier transition curves are inserted between the path pieces and
    /// the circle arc, achieving G2 (curvature) continuity. The transition
    /// length is transitionFraction * radius at each end of the arc.
    /// Set to 0 for G1-only (tangent continuous, no curvature matching).
    /// Default: 0.5 (50% of radius on each side).
    double transitionFraction = 0.5;
};

/// @brief Result of an outside circle blend operation.
struct OutsideCircleBlendResult {
    /// The blended path (may be the original if no blends were applied).
    std::optional<PiecewiseNurbsPath> path;

    /// Number of corners that were successfully blended.
    int blendedCount = 0;

    /// Number of corners that were skipped (too short, no intersection, etc.).
    int skippedCount = 0;

    /// Per-corner audit: true = blended, false = skipped.
    std::vector<bool> cornerOutcomes;
};

/// @brief Outside circle blender: replaces outside corners with exact
///        circular arcs of radius |G64 P| centered at the corner vertex.
class OutsideCircleBlender {
public:
    /**
     * @brief Blend all outside corners in the path.
     *
     * @param path    The original piecewise NURBS path.
     * @param config  Blend configuration (radius, limits, tolerance).
     * @return Blend result with the new path and audit info.
     */
    static OutsideCircleBlendResult blend(
        const PiecewiseNurbsPath& path,
        const OutsideCircleBlendConfig& config);

    // ------------------------------------------------------------------
    // Geometric intersection helpers (all analytical, public for testing)
    // ------------------------------------------------------------------

    /// @brief Intersection of a line piece with a circle.
    /// @param piece   The line NURBS piece (degree 1).
    /// @param center  Circle center (the corner vertex V).
    /// @param radius  Circle radius r.
    /// @param fromEnd If true, return the intersection closest to the
    ///                piece's *end*; otherwise closest to the *start*.
    /// @return Arc-length s along the piece at the intersection, or
    ///         nullopt if no valid intersection exists.
    static std::optional<double> lineCircleIntersection(
        const NurbsCurve& piece,
        const RVec& center, double radius,
        bool fromEnd);

    /// @brief Intersection of an arc piece with a circle.
    ///
    /// The arc piece lies on a circle with center C and radius R.
    /// We solve the two-circle system |P - C|² = R² and |P - V|² = r²
    /// analytically: subtract to get the radical axis (linear), then
    /// substitute back to get a quadratic.
    ///
    /// @param piece   The arc NURBS piece (degree 2, rational).
    /// @param center  Circle center V (the corner vertex).
    /// @param radius  Circle radius r.
    /// @param fromEnd If true, return the intersection closest to the
    ///                piece's *end*; otherwise closest to the *start*.
    /// @return Arc-length s along the piece at the intersection, or
    ///         nullopt if no valid intersection exists.
    static std::optional<double> arcCircleIntersection(
        const NurbsCurve& piece,
        const RVec& center, double radius,
        bool fromEnd);

    /// @brief Extract the underlying circle (center, radius, plane axes)
    ///        from a rational quadratic NURBS arc piece.
    /// @return true if extraction succeeded, false if the piece is not
    ///         a simple circular arc.
    static bool extractCircleFromArc(
        const NurbsCurve& piece,
        RVec& outCenter, double& outRadius,
        RVec& outAxis1, RVec& outAxis2);

    /// @brief Numerical fallback: find s where |position(s) - center| = radius
    ///        using bisection on the arc-length parameter.
    static std::optional<double> numericalIntersection(
        const NurbsCurve& piece,
        const RVec& center, double radius,
        bool fromEnd);

    /// @brief Compute the outside arc from P1 to P2 centered at V.
    ///
    /// The "outside" arc is the major arc (sweep > π) that goes around
    /// the outside of the corner. We determine the sweep direction by
    /// checking which direction keeps the arc on the outside.
    ///
    /// @param P1     Start point of the arc (on the incoming piece).
    /// @param P2     End point of the arc (on the outgoing piece).
    /// @param V      Center of the arc (the corner vertex).
    /// @param d1     Incoming direction (unit, pointing toward V).
    /// @param d2     Outgoing direction (unit, pointing away from V).
    /// @param axis1  First plane axis for NurbsCurve::fromArc.
    /// @param axis2  Second plane axis for NurbsCurve::fromArc.
    /// @return The arc NURBS, or nullopt if the arc is degenerate.
    static std::optional<NurbsCurve> makeOutsideArc(
        const RVec& P1, const RVec& P2, const RVec& V,
        const RVec& d1, const RVec& d2,
        const RVec& axis1, const RVec& axis2);

    /// @brief Determine the plane (axis1, axis2) spanned by two direction
    ///        vectors at a corner.
    static void computePlaneAxes(
        const RVec& d1, const RVec& d2,
        RVec& outAxis1, RVec& outAxis2);
};

} // namespace tether::motion
