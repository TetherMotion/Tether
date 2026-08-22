/**
 * @file OutsideCircleBlender.cpp
 * @brief Outside (negative G64) corner blend via circle-path intersection.
 */

#include "tether/motion_planner/blend/OutsideCircleBlender.hpp"
#include "tether/motion_planner/blend/BlendCurveBuilder.hpp"
#include "tether/motion_planner/blend/BoundaryConditions.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace tether::motion {

// ============================================================================
// Public API
// ============================================================================

OutsideCircleBlendResult OutsideCircleBlender::blend(
    const PiecewiseNurbsPath& path,
    const OutsideCircleBlendConfig& config)
{
    OutsideCircleBlendResult result;
    const auto& pieces = path.pieces();
    const std::size_t n = pieces.size();

    if (n < 2 || config.radius <= 0.0) {
        result.path = path;
        return result;
    }


    // We build the output piece list incrementally. For each junction
    // between piece[i] and piece[i+1], we may:
    //   - Trim the end of piece[i] and the start of piece[i+1]
    //   - Insert a blend arc between them
    //
    // Because trimming piece[i+1] affects the next junction, we process
    // junctions in order and carry forward the "remaining start trim"
    // for the next piece.

    std::vector<NurbsCurve> output;
    output.reserve(n + n / 2);  // estimate
    std::vector<std::size_t> sourceIndices;
    sourceIndices.reserve(n + n / 2);

    double carryStartTrim = 0.0;  // arc length to trim from the start of the next piece

    for (std::size_t i = 0; i < n; ++i) {
        NurbsCurve current = pieces[i];

        // Apply carried start trim (from the previous junction's blend)
        double pieceLen = current.length();
        if (carryStartTrim > 0.0 && carryStartTrim < pieceLen - config.tol) {
            current = current.trim(carryStartTrim, pieceLen);
            pieceLen = current.length();
        }
        carryStartTrim = 0.0;

        // If this is the last piece, just add it
        if (i + 1 >= n) {
            if (pieceLen > config.tol) {
                output.push_back(std::move(current));
                sourceIndices.push_back(i);
            }
            break;
        }

        // Check if this junction should be blended
        const NurbsCurve& next = pieces[i + 1];
        bool shouldBlend = true;

        // Both pieces must be long enough
        if (pieceLen < config.minPieceLength ||
            next.length() < config.minPieceLength) {
            shouldBlend = false;
        }

        // The corner vertex V is the shared endpoint
        RVec V = current.endPoint();

        // Check that the next piece starts at V (G0 continuity)
        RVec nextStart = next.startPoint();
        if (V.distanceTo(nextStart) > config.tol * 100.0) {
            shouldBlend = false;  // Not connected
        }

        if (!shouldBlend) {
            result.cornerOutcomes.push_back(false);
            result.skippedCount++;
            if (pieceLen > config.tol) {
                output.push_back(std::move(current));
                sourceIndices.push_back(i);
            }
            continue;
        }

        // Compute directions at the corner
        // d1 = incoming direction (toward V) = tangent at end of current piece
        // d2 = outgoing direction (away from V) = tangent at start of next piece
        RVec d1, d2;
        try {
            auto deriv1 = current.arcDerivatives(current.knotMax(), 1);
            d1 = deriv1.tangent;
        } catch (...) {
            // Degenerate tangent — use chord direction
            RVec dir = V - current.startPoint();
            double dn = dir.norm();
            if (dn < config.tol) { shouldBlend = false; }
            else d1 = dir / dn;
        }
        try {
            auto deriv2 = next.arcDerivatives(next.knotMin(), 1);
            d2 = deriv2.tangent;
        } catch (...) {
            RVec dir = next.endPoint() - V;
            double dn = dir.norm();
            if (dn < config.tol) { shouldBlend = false; }
            else d2 = dir / dn;
        }

        if (!shouldBlend) {
            result.cornerOutcomes.push_back(false);
            result.skippedCount++;
            if (pieceLen > config.tol) {
                output.push_back(std::move(current));
                sourceIndices.push_back(i);
            }
            continue;
        }

        // Check for collinear (no corner to blend)
        double dot = d1.dot(d2);
        double angle = std::acos(std::clamp(dot, -1.0, 1.0));
        if (angle < 1e-6) {
            // Straight line — no corner
            result.cornerOutcomes.push_back(false);
            result.skippedCount++;
            if (pieceLen > config.tol) {
                output.push_back(std::move(current));
                sourceIndices.push_back(i);
            }
            continue;
        }

        // Compute the plane axes for the arc
        RVec axis1, axis2;
        computePlaneAxes(d1, d2, axis1, axis2);

        // Find intersection of current piece with the circle (closest to end)
        double r = config.radius;
        std::optional<double> s1Opt;

        if (current.isPolyline()) {
            s1Opt = lineCircleIntersection(current, V, r, true);
        } else if (current.degree() == 2) {
            s1Opt = arcCircleIntersection(current, V, r, true);
        }
        if (!s1Opt) {
            // Fallback: numerical
            s1Opt = numericalIntersection(current, V, r, true);
        }

        // Find intersection of next piece with the circle (closest to start)
        std::optional<double> s2Opt;
        if (next.isPolyline()) {
            s2Opt = lineCircleIntersection(next, V, r, false);
        } else if (next.degree() == 2) {
            s2Opt = arcCircleIntersection(next, V, r, false);
        }
        if (!s2Opt) {
            s2Opt = numericalIntersection(next, V, r, false);
        }

        if (!s1Opt || !s2Opt) {
            // No intersection — skip this corner
            result.cornerOutcomes.push_back(false);
            result.skippedCount++;
            if (pieceLen > config.tol) {
                output.push_back(std::move(current));
                sourceIndices.push_back(i);
            }
            continue;
        }

        double s1 = *s1Opt;  // arc length along current piece (from its start)
        double s2 = *s2Opt;  // arc length along next piece (from its start)

        // Get the actual intersection points (I1 on current, I2 on next)
        RVec I1 = current.evaluate(current.invertLength(s1));
        RVec I2 = next.evaluate(next.invertLength(s2));

        // ── Compute the major arc sweep ──
        // Angles of I1, I2 on the circle (in the plane basis axis1, axis2)
        RVec VI1 = I1 - V;
        RVec VI2 = I2 - V;
        double theta1 = std::atan2(VI1.dot(axis2), VI1.dot(axis1));
        double theta2 = std::atan2(VI2.dot(axis2), VI2.dot(axis1));

        double minorSweep = theta2 - theta1;
        while (minorSweep > M_PI) minorSweep -= 2.0 * M_PI;
        while (minorSweep < -M_PI) minorSweep += 2.0 * M_PI;
        double majorSweep = (minorSweep > 0.0)
            ? minorSweep - 2.0 * M_PI
            : minorSweep + 2.0 * M_PI;
        double sweepSign = (majorSweep > 0.0) ? 1.0 : -1.0;

        // ── Determine transition offset δ ──
        double delta = 0.0;
        if (config.transitionFraction > 0.0) {
            delta = r * config.transitionFraction;
            // Clamp δ so transitions fit within both pieces
            double nextLen = next.length();
            delta = std::min(delta, s1);                    // before start of current
            delta = std::min(delta, nextLen - s2);          // past end of next
            // Respect maxTrimFraction (total trim = base trim + δ)
            double maxDeltaCurrent = pieceLen * config.maxTrimFraction
                                   - (pieceLen - s1);
            double maxDeltaNext = nextLen * config.maxTrimFraction - s2;
            delta = std::min(delta, std::max(0.0, maxDeltaCurrent));
            delta = std::min(delta, std::max(0.0, maxDeltaNext));
            // Ensure shortened arc is still a major arc (|sweep| > π)
            double deltaAngle = delta / r;
            double newSweepMag = std::abs(majorSweep) - 2.0 * deltaAngle;
            if (newSweepMag < M_PI + 0.05) {
                // Shortened too much — reduce δ
                deltaAngle = (std::abs(majorSweep) - M_PI - 0.05) / 2.0;
                delta = deltaAngle * r;
            }
        }

        if (delta > config.tol && config.transitionFraction > 0.0) {
            // ══════════════════════════════════════════════════════════════
            // G2 mode: quintic Bézier transitions + shortened circle arc
            // ══════════════════════════════════════════════════════════════
            double deltaAngle = delta / r;
            double thetaQ1 = theta1 + deltaAngle * sweepSign;
            double thetaQ2 = theta2 - deltaAngle * sweepSign;
            double newSweep = majorSweep - 2.0 * deltaAngle * sweepSign;

            // Trim points on the path pieces
            double sP1 = s1 - delta;   // on current piece
            double sP2 = s2 + delta;   // on next piece

            // Build the shortened circle arc from Q1 to Q2
            std::optional<NurbsCurve> circleArc;
            try {
                circleArc = NurbsCurve::fromArc(
                    V, r, axis1, axis2, thetaQ1, newSweep);
            } catch (...) {
                // Arc construction failed — fall back to G1
                delta = 0.0;
            }

            if (delta > config.tol && circleArc) {
                // Positions of transition endpoints
                RVec P1pos = current.evaluate(current.invertLength(sP1));
                RVec P2pos = next.evaluate(next.invertLength(sP2));
                RVec Q1 = V + axis1 * (r * std::cos(thetaQ1))
                             + axis2 * (r * std::sin(thetaQ1));
                RVec Q2 = V + axis1 * (r * std::cos(thetaQ2))
                             + axis2 * (r * std::sin(thetaQ2));

                // Extract boundary conditions
                // Entry of transition 1: end of trimmed current piece
                // Exit of transition 1: start of circle arc
                // Entry of transition 2: end of circle arc
                // Exit of transition 2: start of trimmed next piece
                BoundaryConditions bcP1, bcQ1, bcQ2, bcP2;
                try {
                    bcP1 = boundaryAt(current, sP1, true);
                    bcQ1 = boundaryAt(*circleArc, 0.0, false);
                    bcQ2 = boundaryAt(*circleArc, circleArc->length(), true);
                    bcP2 = boundaryAt(next, sP2, false);
                } catch (...) {
                    // BC extraction failed — fall back to G1
                    delta = 0.0;
                }

                if (delta > config.tol) {
                    // Speed parameters: proportional to endpoint distance
                    double dist1 = P1pos.distanceTo(Q1);
                    double dist2 = Q2.distanceTo(P2pos);
                    double alpha1 = std::max(dist1, 1e-6);
                    double beta1 = std::max(dist1, 1e-6);
                    double alpha2 = std::max(dist2, 1e-6);
                    double beta2 = std::max(dist2, 1e-6);

                    std::optional<NurbsCurve> trans1, trans2;
                    try {
                        trans1 = BlendCurveBuilder::buildQuintic(
                            bcP1, bcQ1, alpha1, beta1);
                        trans2 = BlendCurveBuilder::buildQuintic(
                            bcQ2, bcP2, alpha2, beta2);
                    } catch (...) {
                        // Quintic construction failed — fall back to G1
                        delta = 0.0;
                    }

                    if (delta > config.tol && trans1 && trans2) {
                        // ── Assemble output ──
                        // 1. Trimmed current piece
                        if (sP1 > config.tol) {
                            output.push_back(current.trim(0.0, sP1));
                            sourceIndices.push_back(i);
                        }
                        // 2. Transition 1: P1 → Q1
                        output.push_back(std::move(*trans1));
                        sourceIndices.push_back(i);
                        // 3. Circle arc: Q1 → Q2
                        output.push_back(std::move(*circleArc));
                        sourceIndices.push_back(i);
                        // 4. Transition 2: Q2 → P2
                        output.push_back(std::move(*trans2));
                        sourceIndices.push_back(i + 1);

                        result.blendedCount++;
                        result.cornerOutcomes.push_back(true);
                        carryStartTrim = sP2;
                        continue;
                    }
                }
            }
        }

        // ══════════════════════════════════════════════════════════════
        // G1 mode (or G2 fallback): direct circle arc, no transitions
        // ══════════════════════════════════════════════════════════════

        // Check trim limits: don't trim more than maxTrimFraction
        double trimCurrent = pieceLen - s1;
        double trimNext = s2;
        if (trimCurrent > pieceLen * config.maxTrimFraction ||
            trimNext > next.length() * config.maxTrimFraction) {
            result.cornerOutcomes.push_back(false);
            result.skippedCount++;
            if (pieceLen > config.tol) {
                output.push_back(std::move(current));
                sourceIndices.push_back(i);
            }
            continue;
        }

        // Trim the current piece to end at s1
        if (s1 < config.tol) {
            // The entire piece is consumed by the blend — skip it
        } else {
            NurbsCurve trimmedCurrent = current.trim(0.0, s1);
            output.push_back(std::move(trimmedCurrent));
            sourceIndices.push_back(i);
        }

        // Create the outside arc from I1 to I2 centered at V
        auto arcOpt = makeOutsideArc(I1, I2, V, d1, d2, axis1, axis2);
        if (arcOpt) {
            output.push_back(std::move(*arcOpt));
            sourceIndices.push_back(i);
            result.blendedCount++;
            result.cornerOutcomes.push_back(true);
        } else {
            // Arc creation failed — add the untrimmed current piece back
            if (s1 >= config.tol) {
                output.pop_back();
                sourceIndices.pop_back();
            }
            if (pieceLen > config.tol) {
                output.push_back(std::move(current));
                sourceIndices.push_back(i);
            }
            result.skippedCount++;
            result.cornerOutcomes.push_back(false);
            continue;
        }

        // Carry forward the start trim for the next piece
        carryStartTrim = s2;
    }

    if (output.empty()) {
        // Fallback: return the original path
        result.path = path;
    } else {
        result.path = PiecewiseNurbsPath(std::move(output));
        result.sourcePieceIndices = std::move(sourceIndices);
    }


    return result;
}

// ============================================================================
// Private helpers
// ============================================================================

std::optional<double> OutsideCircleBlender::lineCircleIntersection(
    const NurbsCurve& piece,
    const RVec& center, double radius,
    bool fromEnd)
{
    // A line piece: P(u) = A + u * (B - A), u ∈ [0, 1]
    // |P(u) - center|² = radius²
    // Let d = B - A, f = A - center
    // |f + u*d|² = radius²
    // f² + 2u(f·d) + u²(d²) = radius²
    // u²(d²) + 2u(f·d) + (f² - radius²) = 0

    RVec A = piece.startPoint();
    RVec B = piece.endPoint();
    RVec d = B - A;
    RVec f = A - center;

    double dd = d.dot(d);       // d²
    double fd = f.dot(d);       // f·d
    double ff = f.dot(f);       // f²

    if (dd < 1e-30) return std::nullopt;  // degenerate line

    // Quadratic: dd * u² + 2*fd * u + (ff - r²) = 0
    double c = ff - radius * radius;
    double disc = fd * fd - dd * c;  // discriminant

    if (disc < 0.0) return std::nullopt;  // no intersection

    double sqrtDisc = std::sqrt(disc);
    double u1 = (-fd - sqrtDisc) / dd;
    double u2 = (-fd + sqrtDisc) / dd;

    // Convert parameter u to arc length s (exact for lines: s = u * length)
    double len = piece.length();

    // Choose the intersection closest to the requested end
    if (fromEnd) {
        // We want the intersection closest to u=1 (the end of the piece)
        // Pick the larger u that's within [0, 1]
        double u = std::max(u1, u2);
        if (u < 0.0 || u > 1.0 + 1e-9) {
            // Try the other one
            u = std::min(u1, u2);
            if (u < -1e-9 || u > 1.0 + 1e-9) return std::nullopt;
        }
        return std::clamp(u, 0.0, 1.0) * len;
    } else {
        // We want the intersection closest to u=0 (the start of the piece)
        double u = std::min(u1, u2);
        if (u < -1e-9 || u > 1.0 + 1e-9) {
            u = std::max(u1, u2);
            if (u < -1e-9 || u > 1.0 + 1e-9) return std::nullopt;
        }
        return std::clamp(u, 0.0, 1.0) * len;
    }
}

std::optional<double> OutsideCircleBlender::arcCircleIntersection(
    const NurbsCurve& piece,
    const RVec& center, double radius,
    bool fromEnd)
{
    // Extract the underlying circle from the arc NURBS
    RVec C;  // arc circle center
    double R;  // arc circle radius
    RVec axis1, axis2;

    if (!extractCircleFromArc(piece, C, R, axis1, axis2)) {
        // Not a simple circular arc — fall back to numerical
        return std::nullopt;
    }

    // Circle-circle intersection:
    //   |P - C|² = R²       (arc circle)
    //   |P - V|² = r²       (blend circle, V = center)
    //
    // Subtract: |P-C|² - |P-V|² = R² - r²
    //   -2(C-V)·P + |C|² - |V|² = R² - r²
    //   (V-C)·P = (R² - r² + |V|² - |C|²) / 2
    //
    // This is the radical axis: a line. Parameterize it and substitute
    // back into one circle equation to get a quadratic.

    RVec V = center;
    double r = radius;

    // Distance between circle centers
    RVec CV = V - C;
    double d = CV.norm();

    if (d < 1e-12) {
        // Concentric circles — no intersection unless R == r (infinite)
        return std::nullopt;
    }

    // No intersection if circles are too far apart or one inside the other
    if (d > R + r + 1e-9) return std::nullopt;
    if (d < std::abs(R - r) - 1e-9) return std::nullopt;

    // Radical axis: (V-C)·P = (R² - r² + |V|² - |C|²) / 2
    // Let a = (R² - r² + d²) / (2d)  — distance from C to the radical axis along CV
    double a = (R * R - r * r + d * d) / (2.0 * d);

    // h = sqrt(R² - a²) — half-length of the intersection chord
    double hSq = R * R - a * a;
    if (hSq < 0.0) hSq = 0.0;  // numerical clamp
    double h = std::sqrt(hSq);

    // Direction from C toward V (unit)
    RVec n = CV / d;

    // Midpoint of the intersection chord
    RVec Pmid = C + n * a;

    // Perpendicular direction in the plane (axis1, axis2)
    // The intersection points are Pmid ± h * perp
    // where perp is perpendicular to n and lies in the arc's plane.

    // Find a perpendicular vector in the plane.
    // The arc lies in the plane spanned by (axis1, axis2).
    // n may not be in this plane if the blend circle center V is not
    // in the arc's plane. But for our use case, V is the endpoint of
    // the arc, so V is in the arc's plane, and C is in the plane,
    // so CV = V - C is in the plane, and n is in the plane.

    // Project n onto the arc plane to get the in-plane component
    double n_a1 = n.dot(axis1);
    double n_a2 = n.dot(axis2);
    RVec nInPlane = axis1 * n_a1 + axis2 * n_a2;
    double nInPlaneNorm = nInPlane.norm();
    if (nInPlaneNorm < 1e-12) return std::nullopt;
    nInPlane = nInPlane / nInPlaneNorm;

    // Perpendicular in the plane (rotate 90°)
    RVec perp = axis1 * (-n_a2) + axis2 * n_a1;
    double perpNorm = perp.norm();
    if (perpNorm < 1e-12) return std::nullopt;
    perp = perp / perpNorm;

    // Two intersection points (analytical, exact)
    RVec P_a = Pmid + perp * h;
    RVec P_b = Pmid - perp * h;

    // Convert each intersection point to an arc-length parameter on the
    // piece. The circle-circle intersection is analytical; the arc-length
    // conversion uses bisection (1D root finding on a smooth function).
    auto pointToArcLength = [&](const RVec& P) -> std::optional<double> {
        double len = piece.length();
        if (len < 1e-12) return std::nullopt;

        // Bisect on arc length s to find where |piece.pos(s) - P| is minimal.
        // f(s) = (piece.pos(s) - P) projected onto the arc tangent direction.
        // At the correct s, piece.pos(s) ≈ P, so f(s) ≈ 0.
        //
        // We use bisection on g(s) = (piece.pos(s) - P) · tangent(s),
        // which changes sign at the closest point.

        constexpr int N_SAMPLES = 64;
        double bestS = -1.0;
        double bestDist = std::numeric_limits<double>::max();

        // Sample to find the bracket and initial estimate
        double prevS = 0.0;
        RVec prevP = piece.startPoint();
        double prevDist = prevP.distanceTo(P);

        for (int i = 1; i <= N_SAMPLES; ++i) {
            double s = len * static_cast<double>(i) / N_SAMPLES;
            double u = piece.invertLength(s);
            RVec p = piece.evaluate(u);
            double dist = p.distanceTo(P);

            if (dist < bestDist) {
                bestDist = dist;
                bestS = s;
            }

            // Check for sign change in the projection (root bracketing)
            RVec dir1 = p - prevP;
            RVec dir2 = P - prevP;
            double cross = dir1.dot(P - p) * (P - prevP).dot(prevP - p);
            // Actually, simpler: check if the closest point is between prevS and s
            // by seeing if the distance decreases then increases
            if (prevDist < dist && i > 1) {
                // Minimum was near prevS — refine with bisection
                double lo = (i > 1) ? len * static_cast<double>(i - 2) / N_SAMPLES : 0.0;
                double hi = s;
                for (int iter = 0; iter < 50; ++iter) {
                    double mid = 0.5 * (lo + hi);
                    double uMid = piece.invertLength(mid);
                    RVec pMid = piece.evaluate(uMid);
                    double dMid = pMid.distanceTo(P);

                    double uLo = piece.invertLength(lo);
                    RVec pLo = piece.evaluate(uLo);
                    double dLo = pLo.distanceTo(P);

                    if (dLo < dMid) {
                        hi = mid;
                    } else {
                        lo = mid;
                    }
                    if (hi - lo < 1e-10) break;
                }
                double sRefined = 0.5 * (lo + hi);
                double uRefined = piece.invertLength(sRefined);
                double dRefined = piece.evaluate(uRefined).distanceTo(P);
                if (dRefined < bestDist) {
                    bestDist = dRefined;
                    bestS = sRefined;
                }
            }

            prevS = s;
            prevP = p;
            prevDist = dist;
        }

        if (bestS < 0.0 || bestDist > radius * 0.1 + 1e-3) {
            // No close match — the intersection point is not on this arc
            return std::nullopt;
        }

        return bestS;
    };

    auto s_a = pointToArcLength(P_a);
    auto s_b = pointToArcLength(P_b);

    if (!s_a && !s_b) return std::nullopt;

    double len = piece.length();

    // Choose the intersection closest to the requested end
    if (fromEnd) {
        // Closest to the end (s = len)
        double bestDist = std::numeric_limits<double>::max();
        std::optional<double> best;
        if (s_a) {
            double dist = std::abs(len - *s_a);
            if (dist < bestDist) { bestDist = dist; best = *s_a; }
        }
        if (s_b) {
            double dist = std::abs(len - *s_b);
            if (dist < bestDist) { bestDist = dist; best = *s_b; }
        }
        return best;
    } else {
        // Closest to the start (s = 0)
        double bestDist = std::numeric_limits<double>::max();
        std::optional<double> best;
        if (s_a) {
            double dist = std::abs(*s_a);
            if (dist < bestDist) { bestDist = dist; best = *s_a; }
        }
        if (s_b) {
            double dist = std::abs(*s_b);
            if (dist < bestDist) { bestDist = dist; best = *s_b; }
        }
        return best;
    }
}

bool OutsideCircleBlender::extractCircleFromArc(
    const NurbsCurve& piece,
    RVec& outCenter, double& outRadius,
    RVec& outAxis1, RVec& outAxis2)
{
    // A rational quadratic NURBS arc has 3 control points per span
    // with the middle weight = cos(span/2).
    //
    // For a single-span arc (the common case from fromArc with sweep ≤ π):
    //   P0, P1, P2 with weights w0=1, w1=cos(θ/2), w2=1
    //   - P0 and P2 are ON the circle
    //   - P1 is the intersection of the tangent lines at P0 and P2
    //         (NOT on the circle — it's the "shoulder" control point)
    //   - w1 = cos(sweep/2) where sweep is the arc's sweep angle
    //
    // The circle center is NOT the circumcenter of triangle P0-P1-P2
    // (because P1 is not on the circle). Instead:
    //   R = |P2 - P0| / (2 * sin(sweep/2)) = |P2 - P0| / (2 * sqrt(1 - w1²))
    //   Center is on the perpendicular bisector of chord P0-P2,
    //   on the OPPOSITE side from P1, at distance R * cos(sweep/2) = R * w1
    //   from the chord midpoint.

    if (piece.degree() != 2) return false;

    const auto& cps = piece.controlPoints();
    const auto& weights = piece.weights();

    std::size_t dim = piece.dim();
    if (dim < 2) return false;

    // Get the first 3 control points (first span)
    if (cps.size() < 3) return false;

    RVec P0 = cps[0];
    RVec P1 = cps[1];
    RVec P2 = cps[2];
    double w1 = weights[1];

    // The arc lies in the plane defined by P0, P1, P2
    RVec v1 = P1 - P0;
    RVec v2 = P2 - P0;

    // Compute the plane basis
    if (dim == 2) {
        outAxis1 = RVec{1.0, 0.0};
        outAxis2 = RVec{0.0, 1.0};
    } else if (dim == 3) {
        // Normal to the plane (cross product of v1 and v2)
        double nx = v1[1] * v2[2] - v1[2] * v2[1];
        double ny = v1[2] * v2[0] - v1[0] * v2[2];
        double nz = v1[0] * v2[1] - v1[1] * v2[0];
        double nNorm = std::sqrt(nx * nx + ny * ny + nz * nz);
        if (nNorm < 1e-12) return false;

        // axis1 = chord direction P0→P2 (normalized)
        RVec chord = P2 - P0;
        double chordNorm = chord.norm();
        if (chordNorm < 1e-12) return false;
        outAxis1 = chord / chordNorm;

        // axis2 = normal × axis1 (perpendicular to chord, in the plane)
        RVec normal{nx / nNorm, ny / nNorm, nz / nNorm};
        RVec a2 = RVec{
            normal[1] * outAxis1[2] - normal[2] * outAxis1[1],
            normal[2] * outAxis1[0] - normal[0] * outAxis1[2],
            normal[0] * outAxis1[1] - normal[1] * outAxis1[0]
        };
        double a2Norm = a2.norm();
        if (a2Norm < 1e-12) return false;
        outAxis2 = a2 / a2Norm;
    } else {
        return false;
    }

    // Compute the radius from the chord length and the weight.
    // w1 = cos(sweep/2), so sin(sweep/2) = sqrt(1 - w1²)
    // chord = |P2 - P0| = 2 * R * sin(sweep/2)
    // R = chord / (2 * sin(sweep/2)) = chord / (2 * sqrt(1 - w1²))
    double chordLen = (P2 - P0).norm();
    double sinHalf = std::sqrt(std::max(0.0, 1.0 - w1 * w1));
    if (sinHalf < 1e-12) return false;

    outRadius = chordLen / (2.0 * sinHalf);
    if (outRadius < 1e-12) return false;

    // Center is on the perpendicular bisector of P0-P2, at distance
    // R * cos(sweep/2) = R * w1 from the chord midpoint, on the
    // OPPOSITE side from P1.
    RVec midpoint = (P0 + P2) * 0.5;
    RVec toP1 = P1 - midpoint;
    double sideSign = toP1.dot(outAxis2);  // which side is P1 on?

    // Center is on the opposite side from P1
    double centerDist = outRadius * w1;
    if (sideSign > 0.0) {
        outCenter = midpoint - outAxis2 * centerDist;
    } else {
        outCenter = midpoint + outAxis2 * centerDist;
    }

    return true;
}

std::optional<double> OutsideCircleBlender::numericalIntersection(
    const NurbsCurve& piece,
    const RVec& center, double radius,
    bool fromEnd)
{
    // Bisection on arc length s to find |position(s) - center| = radius
    double len = piece.length();
    if (len < 1e-12) return std::nullopt;

    // We search for s where f(s) = |pos(s) - center| - radius = 0
    // Sample a few points to bracket the root
    constexpr int N_SAMPLES = 32;
    double bestS = -1.0;
    double bestDist = std::numeric_limits<double>::max();

    double prevS = 0.0;
    double prevF = (piece.evaluate(piece.invertLength(0.0)) - center).norm() - radius;

    for (int i = 1; i <= N_SAMPLES; ++i) {
        double s = len * static_cast<double>(i) / N_SAMPLES;
        double u = piece.invertLength(s);
        RVec p = piece.evaluate(u);
        double f = (p - center).norm() - radius;

        // Check for sign change (root bracketing)
        if (prevF * f <= 0.0) {
            // Bisection between prevS and s
            double lo = prevS, hi = s;
            double fLo = prevF;
            for (int iter = 0; iter < 50; ++iter) {
                double mid = 0.5 * (lo + hi);
                double uMid = piece.invertLength(mid);
                RVec pMid = piece.evaluate(uMid);
                double fMid = (pMid - center).norm() - radius;
                if (std::abs(fMid) < 1e-10) {
                    if (fromEnd) {
                        // Prefer the larger s
                        if (mid > bestS || bestS < 0.0) { bestS = mid; bestDist = std::abs(fMid); }
                    } else {
                        // Prefer the smaller s
                        if (mid < bestS || bestS < 0.0) { bestS = mid; bestDist = std::abs(fMid); }
                    }
                    break;
                }
                if (fLo * fMid < 0.0) {
                    hi = mid;
                } else {
                    lo = mid;
                    fLo = fMid;
                }
            }
            if (bestS < 0.0) {
                bestS = 0.5 * (lo + hi);
            }
        }

        // Also track the closest approach
        double dist = std::abs(f);
        if (fromEnd) {
            if (dist < bestDist && s > len * 0.5) {
                // Prefer intersections in the second half for fromEnd
            }
        }

        prevS = s;
        prevF = f;
    }

    if (bestS < 0.0) return std::nullopt;
    return bestS;
}

std::optional<NurbsCurve> OutsideCircleBlender::makeOutsideArc(
    const RVec& P1, const RVec& P2, const RVec& V,
    const RVec& d1, const RVec& d2,
    const RVec& axis1, const RVec& axis2)
{
    // The arc is centered at V with radius r = |P1 - V| = |P2 - V|.
    double r = P1.distanceTo(V);
    if (r < 1e-12) return std::nullopt;

    // Verify P2 is at the same radius
    if (std::abs(P2.distanceTo(V) - r) > r * 1e-6 + 1e-9) return std::nullopt;

    // Compute angles of P1 and P2 relative to V in the (axis1, axis2) plane
    RVec VP1 = P1 - V;
    RVec VP2 = P2 - V;
    double angle1 = std::atan2(VP1.dot(axis2), VP1.dot(axis1));
    double angle2 = std::atan2(VP2.dot(axis2), VP2.dot(axis1));

    // The "outside" arc is the major arc (sweep > π).
    // We need to determine the sweep direction (CW or CCW) that gives
    // the major arc.
    //
    // The minor arc (inside blend) would go from P1 to P2 the short way,
    // cutting the corner. The major arc (outside blend) goes the long way,
    // bulging outward.
    //
    // To determine which direction is "outside":
    // The corner vertex V is where d1 (incoming) and d2 (outgoing) meet.
    // The inside of the corner is where the path turns. The outside is
    // the opposite side.
    //
    // The cross product d1 × d2 (in 3D) or the signed area (in 2D)
    // tells us the turning direction. The outside arc goes in the
    // opposite direction.

    // Compute the signed angle from d1 to d2 in the plane
    double d1x = d1.dot(axis1), d1y = d1.dot(axis2);
    double d2x = d2.dot(axis1), d2y = d2.dot(axis2);
    double cross = d1x * d2y - d1y * d2x;  // z-component of d1 × d2

    // The minor arc (inside) sweeps in the same direction as the corner turns.
    // The major arc (outside) sweeps in the opposite direction.
    //
    // If cross > 0, the corner turns CCW (left), so:
    //   - Minor arc: CCW from P1 to P2 (sweep > 0, < π)
    //   - Major arc: CW from P1 to P2 (sweep < -π)
    //
    // If cross < 0, the corner turns CW (right), so:
    //   - Minor arc: CW from P1 to P2 (sweep < 0, > -π)
    //   - Major arc: CCW from P1 to P2 (sweep > π)

    // Compute the minor sweep (from angle1 to angle2, normalized to [-π, π])
    double minorSweep = angle2 - angle1;
    while (minorSweep > M_PI) minorSweep -= 2.0 * M_PI;
    while (minorSweep < -M_PI) minorSweep += 2.0 * M_PI;

    // The major sweep is the complement
    double majorSweep;
    if (minorSweep > 0) {
        majorSweep = minorSweep - 2.0 * M_PI;  // go the other way (negative)
    } else {
        majorSweep = minorSweep + 2.0 * M_PI;  // go the other way (positive)
    }

    // Choose the sweep that goes "outside"
    // The outside arc should have |sweep| > π
    double sweep = majorSweep;

    // Verify: the outside arc should bulge away from the corner
    // The midpoint of the outside arc should be on the opposite side
    // of the corner from the inside.
    if (std::abs(sweep) < M_PI) {
        // The major sweep should be > π in magnitude
        // If not, something is wrong — try the other direction
        sweep = (minorSweep > 0) ? minorSweep - 2.0 * M_PI : minorSweep + 2.0 * M_PI;
    }

    // Clamp sweep to valid range for fromArc: |sweep| ≤ 2π
    if (std::abs(sweep) > 2.0 * M_PI) {
        return std::nullopt;
    }
    if (std::abs(sweep) < 1e-12) {
        return std::nullopt;
    }

    try {
        return NurbsCurve::fromArc(V, r, axis1, axis2, angle1, sweep);
    } catch (...) {
        return std::nullopt;
    }
}

void OutsideCircleBlender::computePlaneAxes(
    const RVec& d1, const RVec& d2,
    RVec& outAxis1, RVec& outAxis2)
{
    // The plane is spanned by d1 and d2.
    // axis1 = d1 (normalized)
    // axis2 = component of d2 perpendicular to d1, normalized

    outAxis1 = d1.normalized();

    RVec d2_perp = d2 - outAxis1 * d2.dot(outAxis1);
    double perpNorm = d2_perp.norm();

    if (perpNorm < 1e-12) {
        // d1 and d2 are collinear — pick any perpendicular axis
        // For 2D: axis2 = (-d1.y, d1.x)
        // For 3D: use the smallest component of d1 to build a perpendicular
        std::size_t dim = d1.dim();
        if (dim == 2) {
            outAxis2 = RVec{-outAxis1[1], outAxis1[0]};
        } else if (dim == 3) {
            // Find the smallest component of axis1
            double ax = std::abs(outAxis1[0]);
            double ay = std::abs(outAxis1[1]);
            double az = std::abs(outAxis1[2]);
            RVec other;
            if (ax <= ay && ax <= az) {
                other = RVec{1.0, 0.0, 0.0};
            } else if (ay <= az) {
                other = RVec{0.0, 1.0, 0.0};
            } else {
                other = RVec{0.0, 0.0, 1.0};
            }
            RVec perp = other - outAxis1 * other.dot(outAxis1);
            outAxis2 = perp.normalized();
        } else {
            // Fallback for other dimensions
            outAxis2 = RVec::zero(dim);
            if (dim >= 2) {
                outAxis2[0] = -outAxis1[1];
                outAxis2[1] = outAxis1[0];
            }
        }
    } else {
        outAxis2 = d2_perp / perpNorm;
    }
}

} // namespace tether::motion
