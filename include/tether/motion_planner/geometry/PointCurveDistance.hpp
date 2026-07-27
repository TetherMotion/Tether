/**
 * @file PointCurveDistance.hpp
 * @brief Certified global point-to-NURBS distance (no sampling, no missed roots)
 *
 * @details
 * For a rational curve S(u) = C(u)/w(u) (C vector polynomial, w scalar
 * polynomial per Bézier span) the squared distance D(u) = |S(u) − p|² is
 * stationary exactly where the *polynomial*
 *
 *   N(u) = (C − p·w) · (C′·w − C·w′)        (GeometryFoundations.md eq. (G.30))
 *
 * vanishes, because dD/du = 2·N(u)/w(u)³ and w(u) > 0 (weights are positive,
 * convex hull). All candidates for the global minimum are therefore:
 * the roots of N per span, plus the span endpoints.
 *
 * Algorithm: decompose the curve into Bézier spans (exact), build N per span
 * directly in Bernstein basis (exact products, eq. (G.7)), isolate ALL roots
 * with certified Bernstein root isolation (never misses a root), polish each
 * root with Newton steps on N, evaluate D at every candidate, take the min.
 *
 * Certification property: the returned distance is the true global minimum
 * up to root-isolation width and floating-point round-off — unlike sampling
 * approaches, no stationary point can be missed (see
 * GeometryFoundations.md §"Point-to-curve distance").
 *
 * Math reference: (M8) certified point-to-curve distance,
 * (M9) Bernstein root isolation. See docs/motion/BlendingAlgorithm.md.
 */
#pragma once

#include "tether/motion_planner/geometry/NurbsCurve.hpp"

namespace tether::motion {

struct DistanceResult {
    double distance = 0.0;   ///< min_u |S(u) − p|
    double u = 0.0;          ///< achieving parameter on the original curve
    RVec closestPoint;       ///< S(u)
};

/**
 * @brief Certified global minimum distance from point p to curve c.
 * @param c any NurbsCurve (rational or not, dimension 1..5)
 * @param p query point, same dimension as c
 * @throws std::invalid_argument on dimension mismatch
 */
DistanceResult pointCurveDistance(const NurbsCurve& c, const RVec& p);

} // namespace tether::motion
