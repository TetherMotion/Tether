/**
 * @file ProfileSplineFitter.hpp
 * @brief Adaptive B-spline interpolation through scalar samples with
 *        convex-hull constraint clamping.
 *
 * @details
 * This is the low-level math core of the ReNURBS profile representation
 * (see ReNURBS.md §4.2–§4.3). It fits a clamped B-spline of degree p through
 * a set of (u_i, q_i) samples, with two key properties:
 *
 * 1. **Interpolation within epsilon**: |B(u_i) − q_i| ≤ ε for every sample.
 *    Achieved by global B-spline interpolation (banded linear solve) followed
 *    by adaptive knot insertion where the inter-sample residual exceeds ε.
 *
 * 2. **Constraint preservation**: if an upper-limit function q_lim(u) is
 *    provided, the spline B(u) ≤ q_lim(u) − safetyMargin for *all* u in
 *    [0,1], not just at samples. Achieved by convex-hull clamping of control
 *    points per span (a B-spline segment lies in the convex hull of its p+1
 *    defining control points when weights are uniform).
 *
 * The fitter operates on plain doubles (no NurbsCurve dependency) so it can
 * be unit-tested in isolation. The ReNurbsProfileBuilder wraps the result
 * as a 1-D NurbsCurve (weights all 1).
 *
 * ## Algorithm
 *
 * ### Step 1: Global B-spline interpolation (Piegl & Tiller §9.3.1)
 *
 * - Parameterize samples by chord length: u_i = (s_i − s_0) / (s_n − s_0).
 * - Compute knot vector by averaging: for degree p with n+1 samples,
 *   interior knots U_{j+p} = (1/p) * Σ_{i=j}^{j+p-1} u_i, j = 1..n−p.
 * - Solve the banded system N^T * N * P = N^T * Q for control points P,
 *   where N is the basis function matrix. (We use a simple Gaussian
 *   elimination on the banded system for clarity; n is small, ≤ 64.)
 *
 * ### Step 2: Adaptive refinement
 *
 * - Evaluate B(u) on a dense test grid (multiplier × sample count).
 * - Compute residual r(u) = |B(u) − q_interp(u)| where q_interp is the
 *   linear interpolation of the original samples (ground truth between
 *   samples).
 * - Where r(u) > ε, insert a knot at u (Boehm insertion) and re-solve.
 * - Cap at maxControlPoints control points.
 *
 * ### Step 3: Convex-hull constraint clamping
 *
 * - For each span [u_j, u_{j+1}), the spline lies in the convex hull of
 *   control points P_{j-p+1} .. P_j (for a clamped B-spline with uniform
 *   weights).
 * - Compute the minimum of q_lim over the span (sampled, with a safety
 *   margin).
 * - Clamp each control point in the span's hull to be ≤ (min_q_lim −
 *   safetyMargin). This guarantees B(u) ≤ min_q_lim − safetyMargin on
 *   that span.
 * - After clamping, re-check interpolation; if a sample is now missed by
 *   more than ε, insert a knot near it and re-fit (the sample itself is
 *   feasible, so this converges).
 *
 * @see ReNURBS.md for the full design.
 */

#pragma once

#include <vector>
#include <cstddef>
#include <optional>
#include <stdexcept>

namespace tether::motion::profile_renurbs {

/// Result of a single spline fit.
struct SplineFitResult {
    /// Control points of the fitted B-spline (scalar values).
    std::vector<double> controlPoints;

    /// Knot vector of the fitted B-spline (clamped, non-decreasing).
    std::vector<double> knots;

    /// Degree of the fitted B-spline.
    int degree = 0;

    /// Maximum residual |B(u_i) − q_i| over all samples (after refinement).
    double maxResidual = 0.0;

    /// Maximum residual on the dense test grid (inter-sample).
    double maxInterSampleResidual = 0.0;

    /// True if the fit achieved maxResidual ≤ epsilon.
    bool withinEpsilon = false;

    /// True if the constraint clamp was applied.
    bool constraintClamped = false;

    /// True if the max control point cap was hit (graceful degradation).
    bool controlPointCapHit = false;

    /// The achieved continuity class inside a segment: C^(degree−1).
    /// (Reported as an integer for simplicity; 0 = C⁰, 1 = C¹, etc.)
    int achievedContinuity = 0;
};

/// Configuration for the spline fitter.
struct SplineFitterConfig {
    /// B-spline degree (p ≥ 1). p=1 → linear, p=3 → cubic (C²),
    /// p=5 → quintic (C⁴). Higher = smoother but more control points.
    int degree = 5;

    /// Interpolation tolerance: |B(u_i) − q_i| must be ≤ this.
    double epsilon = 1e-4;

    /// Safety margin below the limit envelope for constraint preservation.
    double safetyMargin = 1e-4;

    /// Maximum control points per segment (adaptive refinement cap).
    std::size_t maxControlPoints = 64;

    /// Dense test grid multiplier (test grid = multiplier × sample count).
    std::size_t refinementGridMultiplier = 10;

    /// Optional: clamp the lower bound of the spline (e.g. velocity ≥ 0).
    /// If set, control points are clamped to be ≥ this value.
    std::optional<double> lowerBound = std::nullopt;

    /// Optional: upper limit function — if set, the spline is clamped to
    /// be ≤ limit(u) − safetyMargin everywhere. The limit is provided as
    /// a vector sampled at the same u_i as the data (linearly interpolated
    /// between samples for inter-sample evaluation).
    std::optional<std::vector<double>> upperLimit = std::nullopt;
};

/**
 * @brief Fit a clamped B-spline through scalar samples with adaptive
 *        refinement and optional constraint clamping.
 *
 * @param u Sample parameters in [0,1], strictly increasing. Must have
 *          u[0] == 0 and u.back() == 1 (clamped endpoints).
 * @param q Sample values at the corresponding u_i.
 * @param config Fitter configuration.
 * @return The fit result (control points, knots, degree, residuals).
 * @throws std::invalid_argument if u and q have different sizes, fewer
 *         than 2 samples, u is not strictly increasing, or u[0] != 0 /
 *         u.back() != 1.
 */
SplineFitResult fitSplineThroughSamples(
    const std::vector<double>& u,
    const std::vector<double>& q,
    const SplineFitterConfig& config = {});

/**
 * @brief Evaluate a B-spline at parameter u.
 *
 * Uses de Boor's algorithm (exact evaluation). The spline is defined by
 * control points, knots, and degree — the same triple stored in
 * SplineFitResult.
 *
 * @param controlPoints Control point values (scalar).
 * @param knots Knot vector (clamped, non-decreasing).
 * @param degree B-spline degree p ≥ 1.
 * @param u Parameter in [knots[degree], knots[n+1]] where n = #CP − 1.
 * @return B(u).
 */
double evaluateBSpline(const std::vector<double>& controlPoints,
                       const std::vector<double>& knots,
                       int degree, double u);

/**
 * @brief Evaluate the first derivative of a B-spline at parameter u.
 *
 * Uses the derivative formula for B-splines (Piegl & Tiller §3.4).
 *
 * @return dB/du at u.
 */
double evaluateBSplineDerivative(const std::vector<double>& controlPoints,
                                 const std::vector<double>& knots,
                                 int degree, double u);

/**
 * @brief Insert a knot into a B-spline's representation (Boehm insertion).
 *
 * Returns the new control points and knots with the knot inserted at
 * multiplicity 1 (or increased by 1 if it already exists).
 *
 * @param controlPoints Original control points.
 * @param knots Original knot vector.
 * @param degree B-spline degree.
 * @param u Knot to insert (must be in the interior of the domain).
 * @return Pair (newControlPoints, newKnots).
 */
std::pair<std::vector<double>, std::vector<double>>
insertKnot(const std::vector<double>& controlPoints,
           const std::vector<double>& knots,
           int degree, double u);

} // namespace tether::motion::profile_renurbs
