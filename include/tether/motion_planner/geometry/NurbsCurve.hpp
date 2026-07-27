/**
 * @file NurbsCurve.hpp
 * @brief NURBS curve over RVec (runtime dimension 1..5) — new geometry core
 *
 * @details
 * This is the canonical curve representation of the rewritten path-blending
 * stack (plan §4.1): lines are degree-1 curves, arcs are exact rational
 * quadratics, and B-spline/NURBS inputs pass through unchanged.
 *
 * Capabilities:
 * - Exact evaluation by De Boor in homogeneous coordinates.
 * - Parametric derivatives up to order 3 via the quotient rule on the
 *   homogeneous curve (GeometryFoundations.md eq. (G.14)–(G.16)).
 * - Arc-length derivatives (p, T, κ⃗, j⃗) in N-D, eq. (G.18)–(G.21).
 * - Arc length by adaptive 8-point Gauss–Legendre quadrature with error
 *   estimate (eq. (G.24)); degree-1 curves use an exact closed form.
 * - Arc-length inversion s→u by Newton–Raphson bracketed by bisection
 *   (eq. (G.26)); degree-1 curves use an exact closed form.
 * - Exact split at a parameter by knot insertion to full multiplicity,
 *   arc-length trimming, and Bézier decomposition.
 *
 * Thread-safety: NOT thread-safe. Arc-length results are memoized in a
 * mutable per-curve cache, so concurrent queries on one shared curve race.
 * Convention for now: one path (and its curves) per thread.
 *
 * Degenerate inputs are rejected with exceptions (std::invalid_argument /
 * std::domain_error), never undefined behavior.
 *
 * Math reference: (M3) NURBS in homogeneous form, (M4) rational derivatives,
 * (M5) arc-length derivatives up to order 3, (M6) exact circular arcs as
 * rational quadratics, (M7) splitting by knot insertion.
 * See docs/motion/BlendingAlgorithm.md for proofs.
 */
#pragma once

#include "tether/motion_planner/geometry/Vector.hpp"

#include <cstddef>
#include <utility>
#include <vector>

namespace tether::motion {

/// Arc-length derivatives of a curve at one parameter value.
/// Field validity depends on the requested order:
/// - order 0: position only
/// - order 1: + tangent (unit)
/// - order 2: + curvature vector κ⃗ = d²p/ds²
/// - order 3: + jounce vector j⃗ = d³p/ds³
struct ArcDerivatives {
    RVec position;
    RVec tangent;
    RVec curvature;
    RVec jounce;
};

class NurbsCurve {
public:
    // ========================================================================
    // Construction / validation
    // ========================================================================

    /**
     * @brief Construct a NURBS curve.
     * @param controlPoints n+1 control points, all of the same dimension 1..5
     * @param weights n+1 strictly positive weights
     * @param knots n+p+2 knot values, non-decreasing
     * @param degree curve degree p ≥ 1; requires n+1 ≥ p+1
     * @throws std::invalid_argument on any inconsistency (diagnostic, not UB)
     */
    NurbsCurve(std::vector<RVec> controlPoints, std::vector<double> weights,
               std::vector<double> knots, int degree);

    /// Degree-1 line from a to b; knots {0,0,1,1}, weights {1,1}.
    /// @throws std::invalid_argument if a == b (zero-length) or dim mismatch.
    static NurbsCurve fromLine(const RVec& a, const RVec& b);

    /**
     * @brief Exact rational quadratic circular arc.
     *
     * Convention (single, documented): the arc lies in the plane through
     * `center` spanned by the orthonormal vectors `axis1`, `axis2` ⊂ ℝ^dim.
     * Points on the arc satisfy
     *   p(θ) = center + radius·(cos θ·axis1 + sin θ·axis2),
     * θ ∈ [startAngle, startAngle + sweepAngle]  (sweep may be negative).
     *
     * Construction: per span of ≤ π, three control points with middle weight
     * cos(span/2) — exact circle, see GeometryFoundations.md proof P0.
     * Sweeps > π are split into multiple spans joined with single internal
     * knots (the result is C¹, as the spans lie on one circle).
     *
     * @throws std::invalid_argument if radius ≤ 0, sweep == 0, |sweep| > 2π,
     *         or axis1/axis2 are not orthonormal (tol 1e-9).
     */
    static NurbsCurve fromArc(const RVec& center, double radius,
                              const RVec& axis1, const RVec& axis2,
                              double startAngle, double sweepAngle);

    // ========================================================================
    // Properties
    // ========================================================================

    int degree() const noexcept { return degree_; }
    std::size_t numControlPoints() const noexcept { return controlPoints_.size(); }
    std::size_t dim() const noexcept { return dim_; }

    const std::vector<RVec>& controlPoints() const noexcept { return controlPoints_; }
    const std::vector<double>& weights() const noexcept { return weights_; }
    const std::vector<double>& knots() const noexcept { return knots_; }

    /// Parameter domain [knotMin, knotMax].
    double knotMin() const noexcept { return knots_[degree_]; }
    double knotMax() const noexcept { return knots_[knots_.size() - degree_ - 1]; }

    RVec startPoint() const { return evaluate(knotMin()); }
    RVec endPoint() const { return evaluate(knotMax()); }

    /// True for degree-1 curves (piecewise-linear; arc length is exact).
    bool isPolyline() const noexcept { return degree_ == 1; }

    // ========================================================================
    // Evaluation and derivatives
    // ========================================================================

    /// Evaluate position at u (clamped to the domain). Exact (De Boor).
    RVec evaluate(double u) const;

    /**
     * @brief Parametric derivative d^kC/du^k at u, order 0..3.
     * Orders above the curve degree return the zero vector.
     * Rational curves: quotient rule on the homogeneous curve,
     * GeometryFoundations.md eq. (G.14)–(G.16).
     */
    RVec derivative(double u, int order) const;

    /**
     * @brief Arc-length derivatives at u (eq. (G.18)–(G.21)).
     * @param order 0..3, see ArcDerivatives for field validity
     * @throws std::domain_error if order ≥ 1 and |C'(u)| is (near-)zero
     *         (degenerate parameterization — diagnostic, not UB)
     */
    ArcDerivatives arcDerivatives(double u, int order) const;

    // ========================================================================
    // Arc length (lazy, memoized; NOT thread-safe — one path per thread)
    // ========================================================================

    /// Total arc length. Memoized after the first call.
    double length() const;

    /// Arc length from knotMin() to u (u clamped). Exact for polylines;
    /// adaptive Gauss–Legendre quadrature otherwise (eq. (G.24)).
    double arcLengthTo(double u) const;

    /**
     * @brief Inverse arc length: parameter u with arcLengthTo(u) == s.
     * Newton–Raphson on f(u) = arcLengthTo(u) − s with ds/du = |C'(u)|,
     * bracketed by bisection on [knotMin, knotMax] (monotone, eq. (G.26)).
     * s is clamped to [0, length()]. Exact for polylines.
     * @throws std::domain_error on a zero-length curve.
     */
    double invertLength(double s) const;

    /// Number of adaptive-quadrature integrations performed on this curve
    /// (test/diagnostic counter proving laziness: 0 for pure line curves).
    std::size_t arcLengthComputationCount() const noexcept {
        return arcLengthComputations_;
    }

    /// Approximate memory footprint in bytes (object + heap), for tests.
    std::size_t estimatedMemoryBytes() const noexcept;

    // ========================================================================
    // Exact parameter operations
    // ========================================================================

    /**
     * @brief Split the curve at parameter u into [left, right] sub-curves.
     * Exact: u is knot-inserted to multiplicity == degree (closed form);
     * both halves share the junction control point bitwise. u is clamped
     * strictly inside the domain; splitting at a domain end returns
     * {*this-clipped, degenerate} — instead, callers should clamp u to the
     * open interval (knotMin, knotMax); out-of-range u throws.
     * @throws std::invalid_argument if u is outside the open domain.
     */
    std::pair<NurbsCurve, NurbsCurve> split(double u) const;

    /// Arc-length trim: sub-curve between lengths s0 ≤ s1 (clamped to
    /// [0, length()]). Exact up to the invertLength tolerance.
    NurbsCurve trim(double s0, double s1) const;

    /// Decompose into single-span Bézier pieces (exact knot insertion).
    /// Each piece has knots {a×(p+1), b×(p+1)} and p+1 control points.
    std::vector<NurbsCurve> bezierDecompose() const;

private:
    // Homogeneous control point: (w·P, w) with dim+1 ≤ 6 components.
    struct HomPoint {
        double c[6]; // c[0..dim-1] = w·P, c[dim] = w
    };

    int findSpan(double u) const;
    int knotMultiplicity(double u) const;
    NurbsCurve insertKnot(double u) const;

    /// De Boor in homogeneous coordinates; returns (w·C, w).
    HomPoint evaluateHomogeneous(double u) const;

    /// All parametric derivatives of the homogeneous curve up to order k
    /// (Piegl & Tiller A2.3 basis derivatives applied to (w·P, w)).
    /// out[kk][0..dim] = d^kk/du^kk (w·C, w); kk = 0..k (capped at degree).
    void homogeneousDerivatives(double u, int k, std::vector<HomPoint>& out) const;

    double clampToDomain(double u) const;

    // Degree-1 exact arc-length helpers.
    double polylineArcLengthTo(double u) const;
    double polylineInvertLength(double s) const;

    // Adaptive 8-point Gauss–Legendre with subdivision error estimate.
    double quadrature(double a, double b, double tol) const;
    double quadratureRecursive(double a, double b, double tol, int depth) const;
    double gaussLegendre8(double a, double b) const;
    double speed(double u) const { return derivative(u, 1).norm(); }

    std::vector<RVec> controlPoints_;
    std::vector<double> weights_;
    std::vector<double> knots_;
    int degree_;
    std::size_t dim_;

    // Lazy arc-length cache (mutable; not thread-safe by design).
    mutable double cachedLength_ = -1.0; // < 0 means "not computed"
    mutable std::size_t arcLengthComputations_ = 0;
};

} // namespace tether::motion
