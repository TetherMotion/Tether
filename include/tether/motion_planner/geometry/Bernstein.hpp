/**
 * @file Bernstein.hpp
 * @brief Scalar polynomials in Bernstein basis on [0,1] — basis for certification
 *
 * @details
 * A degree-n polynomial in Bernstein basis is
 *   p(t) = Σᵢ bᵢ B_{i,n}(t),  B_{i,n}(t) = C(n,i) tⁱ (1−t)^{n−i},  t ∈ [0,1]
 * (GeometryFoundations.md eq. (G.1)).
 *
 * The two properties this module exists for:
 * - **Convex hull**: for t ∈ [0,1], p(t) lies in [min bᵢ, max bᵢ] — the
 *   coefficients bound the values (eq. (G.5)).
 * - **Variation diminishing / sign changes**: a sign change in the
 *   coefficient sequence implies an odd number of roots (counted with
 *   multiplicity parity) inside the interval; strictly same-sign
 *   coefficients imply no root (eq. (G.6)).
 *
 * Math reference: (M1) Bézier/Bernstein basis, (M2) endpoint derivative
 * identities, (M9) root isolation by variation-diminishing subdivision.
 * See docs/motion/BlendingAlgorithm.md for proofs.
 *
 * `isolateRoots` exploits both by recursive subdivision: it never misses a
 * root (certified isolation) and needs no sampling. Near-multiple or
 * clustered roots that cannot be separated at the requested tolerance are
 * reported as one merged interval covering the cluster (documented
 * behavior — the interval still provably contains the roots).
 */
#pragma once

#include <cstddef>
#include <utility>
#include <vector>

namespace tether::motion {

namespace bernstein {

/// A closed parameter interval [lo, hi] ⊆ [0,1] isolating at least one root.
struct Interval {
    double lo;
    double hi;
};

/// Evaluate p(t) by De Casteljau's algorithm (eq. (G.3)).
/// @param coeffs Bernstein coefficients b_0..b_n
double evaluate(const std::vector<double>& coeffs, double t);

/// Bernstein coefficients of the derivative p'(t):
/// b'_i = n·(b_{i+1} − b_i), degree n−1 (eq. (G.4)).
std::vector<double> derivative(const std::vector<double>& coeffs);

/// Subdivide at t ∈ (0,1) into [left, right] Bernstein representations of
/// the same polynomial on [0,t] and [t,1] (De Casteljau, eq. (G.3)).
std::pair<std::vector<double>, std::vector<double>>
subdivide(const std::vector<double>& coeffs, double t);

/// Product of two Bernstein polynomials (degrees m, n → m+n), exact:
/// c_k = Σ_{i+j=k} [C(m,i)·C(n,j)/C(m+n,k)] · a_i · b_j  (eq. (G.7)).
std::vector<double> multiply(const std::vector<double>& a,
                             const std::vector<double>& b);

/// Convert power-basis coefficients [a_0..a_n] (p(x) = Σ a_i xⁱ) to
/// Bernstein coefficients of the same degree n: b_i = Σ_{j≤i} a_j·C(i,j)/C(n,j)
/// (eq. (G.8)).
std::vector<double> powerToBernstein(const std::vector<double>& power);

/// Convert Bernstein coefficients back to power basis:
/// a_m = Σ_{i≤m} b_i·C(n,i)·C(n−i,m−i)·(−1)^{m−i}  (eq. (G.9)).
std::vector<double> bernsteinToPower(const std::vector<double>& coeffs);

/**
 * @brief Isolate all real roots of p in [0,1].
 *
 * Recursive subdivision driven by the convex-hull property:
 * - coefficients strictly one-signed  ⇒ provably no root in the interval;
 * - interval narrower than `tol`      ⇒ report it (root or root cluster);
 * - otherwise                         ⇒ bisect and recurse.
 *
 * Adjacent dyadic intervals covering the same root (or an unresolvable
 * cluster, e.g. a near-double root) are merged into one reported interval.
 *
 * Certification: every root of p in [0,1] lies inside one of the returned
 * disjoint intervals; each interval has width < 2·tol. (See
 * GeometryFoundations.md §"Bernstein root isolation" for the argument.)
 *
 * @param coeffs Bernstein coefficients b_0..b_n
 * @param tol interval width goal (must be > 0); degenerate all-zero
 *        coefficient vectors return a single interval [0,1] (a zero
 *        polynomial — every point is a "root"; callers must special-case).
 */
std::vector<Interval> isolateRoots(const std::vector<double>& coeffs,
                                   double tol);

} // namespace bernstein

} // namespace tether::motion
