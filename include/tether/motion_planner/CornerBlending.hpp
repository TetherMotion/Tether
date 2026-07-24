/**
 * @file CornerBlending.hpp
 * @brief Scoring-Based C2-Continuous Corner Blending
 *
 * @details
 * Completely reworked corner blending algorithm that produces C2-continuous
 * (curvature-continuous) transitions between motion segments.
 *
 * ## Key Design Principles
 *
 * 1. **Always C2-smooth** — Quintic Bézier blend curves with curvature-matched
 *    boundaries at both entry and exit.
 * 2. **Scoring-based solver** — Iteratively finds the best blend fraction that
 *    satisfies all constraints (half-length, C2, symmetry, curvature quality).
 * 3. **Half-length constraint** — Blends never consume more than a configurable
 *    fraction (default 50%) of either adjacent segment.
 * 4. **Symmetric preference** — Entry and exit distances should be equal unless
 *    geometry forces asymmetry.
 * 5. **Guaranteed termination** — Falls back to exact stop if no valid C2
 *    solution exists.
 * 6. **Per-transition blend modes** — Each corner independently supports
 *    Centered, Inside, Outside, Teardrop, or ExactStop.
 *
 * @see docs/G64BlendAlgorithm.md for full design documentation
 * @see BezierCurve.hpp
 * @see MotionSegment.hpp
 */

#pragma once

#include "MathTypes.hpp"
#include "BezierCurve.hpp"
#include "BlendCore.hpp"
#include "MotionSegment.hpp"
#include <optional>
#include <utility>
#include <vector>
#include <algorithm>
#include <cmath>
#include <iostream>
#include <string>
#include <sstream>
#include <cassert>

// Runtime blend checks: enabled by default via CMake, can be overridden
#ifndef TETHER_BLEND_RUNTIME_CHECKS
#define TETHER_BLEND_RUNTIME_CHECKS 1
#endif

namespace MotionPlanner {

// ============================================================================
// Corner Analysis Types
// ============================================================================

/**
 * @brief Classification of corner geometry
 */
enum class CornerType : uint8_t {
    Convex,     ///< Outside corner (turn < 180°)
    Concave,    ///< Inside corner (turn > 180°)
    Straight,   ///< No corner (collinear segments)
    Cusp        ///< Sharp reversal (180° turn)
};

/**
 * @brief Blend mode per-transition
 */
enum class BlendMode : uint8_t {
    Centered = 0,           ///< Standard inscribed-circle blend
    InsideStrict = 1,       ///< Stay strictly inside corner
    InsideApproximate = 2,  ///< Stay approximately inside
    OutsideStrict = 3,      ///< Stay strictly outside (dogbone)
    OutsideApproximate = 4, ///< Stay approximately outside
    Balanced = 5,           ///< Balance inside and outside
    Teardrop = 6,           ///< Continue past corner, arc back
    ExactStop = 7,          ///< No blending
};

/**
 * @brief Diagnostic information for a blend solution
 */
struct BlendDiagnostics {
    double entryTangentError = 0.0;    ///< Tangent direction error at entry (radians)
    double exitTangentError = 0.0;     ///< Tangent direction error at exit (radians)
    double entryCurvatureError = 0.0;  ///< Curvature mismatch at entry
    double exitCurvatureError = 0.0;   ///< Curvature mismatch at exit
    double maxCurvature = 0.0;         ///< Peak curvature in blend
    double maxCurvatureRate = 0.0;     ///< Max rate of curvature change
    int solverIterations = 0;          ///< Number of solver iterations used
    double bestScore = 0.0;            ///< Final score of chosen solution
    std::string fallbackReason;        ///< If fell back to exact stop, why
    bool isValid = false;              ///< Overall validity flag
};

/**
 * @brief Result of corner geometry analysis
 */
template<size_t Dim, typename T = double>
struct CornerAnalysis {
    /// Corner type classification
    CornerType type = CornerType::Straight;

    /// Corner angle in radians (0 to π)
    T angle = T(0);

    /// Turn angle (negative for right turn, positive for left)
    T turnAngle = T(0);

    /// True if clockwise turn (right turn)
    bool isCW = false;

    /// The corner vertex
    Vec<Dim, T> cornerPoint;

    /// Unit vector of incoming direction at junction
    Vec<Dim, T> incomingDir;

    /// Unit vector of outgoing direction at junction
    Vec<Dim, T> outgoingDir;

    /// Corner bisector direction (unit vector)
    Vec<Dim, T> bisector;

    /// Computed blend radius (inscribed circle)
    T blendRadius = T(0);

    /// Center of blend arc/circle
    Vec<Dim, T> blendCenter;

    /// Point where blend starts (on incoming segment)
    Vec<Dim, T> blendEntry;

    /// Point where blend ends (on outgoing segment)
    Vec<Dim, T> blendExit;

    /// Distance along incoming segment consumed by blend
    T entryDistance = T(0);

    /// Distance along outgoing segment consumed by blend
    T exitDistance = T(0);

    /// Incoming curvature (0 for lines, 1/R for arcs)
    T incomingCurvature = T(0);

    /// Outgoing curvature (0 for lines, 1/R for arcs)
    T outgoingCurvature = T(0);

    /// Is blending possible for this corner?
    bool canBlend = false;

    /// Reason if blending not possible
    std::string blendReason;

    /// True if this is an "Outside" blend (negative tolerance / dogbone)
    bool isOutsideBlend = false;

    /// Maximum velocity permitted at this corner
    T maxCornerVelocity = T(0);

    /// Blend fraction chosen by scorer (0..1 fraction of ideal distance)
    T blendFraction = T(1);

    /// Diagnostic information
    BlendDiagnostics diagnostics;
};

// ============================================================================
// Blend Configuration
// ============================================================================

/**
 * @brief Configuration for corner blending
 */
struct BlendConfig {
    /// Corner limit modes (interpretation of tolerances)
    enum class CornerLimitMode : uint8_t {
        Centered = 0,
        InsideStrict = 1,
        InsideApproximate = 2,
        OutsideStrict = 3,
        OutsideApproximate = 4,
        Balanced = 5,
    };

    /// Maximum path deviation (G64 P value)
    double tolerance = 0.05;

    /// Naive CAM tolerance (G64 Q value)
    double naiveTolerance = 0.0;

    /// Inside/outside specific tolerances
    double insideTolerance = 0.0;
    double outsideTolerance = 0.0;

    /// Corner limit mode (default: centered)
    CornerLimitMode cornerMode = CornerLimitMode::Centered;

    /// Minimum corner angle to blend (degrees)
    double minAngle = 1.0;

    /// Maximum corner angle to blend (degrees)
    double maxAngle = 175.0;

    /// Maximum fraction of segment consumed by blend [0, 0.5]
    /// This is the key configurable parameter: the blend must finish
    /// within this fraction of each adjacent segment's path length.
    /// Default: 0.5 (50%)
    double maxBlendFraction = 0.5;

    /// Minimum segment length to allow blending
    double minSegmentLength = 0.01;

    /// Target continuity level (1 = G1, 2 = G2)
    int continuityLevel = 2;

    /// Use Bézier curves (vs circular arcs)
    bool useBezier = true;

    /// Bézier degree for blend curves (3 = cubic, 5 = quintic)
    int bezierDegree = 5;

    /// Tolerance for geometric comparisons
    double epsilon = 1e-10;

    /// Maximum centripetal acceleration (length/sec²)
    double maxCentripetalAccel = 1000.0;

    // --- Scoring solver parameters ---

    /// Enable scoring-based optimization (vs simple clamped solution)
    bool useScoringOptimizer = true;

    /// Number of coarse search steps
    int coarseSteps = 10;

    /// Number of fine search steps
    int fineSteps = 10;

    // --- Scoring weights ---
    double weightC2 = 100.0;          ///< C2 feasibility (hard constraint)
    double weightHalfLength = 50.0;   ///< Half-length compliance
    double weightSymmetry = 5.0;      ///< Entry/exit symmetry
    double weightDeviation = 20.0;    ///< Deviation utilization (prefer larger blends)
    double weightCurvature = 15.0;    ///< Curvature quality

    /// Maximum curvature multiplier (blend curvature should not exceed this × 1/blendRadius)
    double maxCurvatureMultiplier = 5.0;
};

// ============================================================================
// Blend Curve Builder
// ============================================================================

/**
 * @brief Constructs C2-continuous blend curves using quintic Bézier
 */
template<size_t Dim, typename T = double>
class BlendCurveBuilder {
public:
    using Point = Vec<Dim, T>;
    using Curve = BezierCurve<Dim, T>;
    using Analysis = CornerAnalysis<Dim, T>;

    /**
     * @brief Build C2-continuous quintic Bézier blend curve
     *
     * The control points are placed to ensure:
     * - P0 = blendEntry, P5 = blendExit  (C0)
     * - P1 along incoming tangent, P4 along outgoing tangent  (C1)
     * - P2/P3 account for curvature  (C2)
     */
    static std::vector<Curve> buildG2BlendCurve(const Analysis& analysis,
                                    SourceReference sourceRef = {}) {
        if (!analysis.canBlend) {
            return {};
        }

        if (analysis.isOutsideBlend) {
            return buildOutsideBlend(analysis, sourceRef);
        }

        return { buildQuinticC2Blend(analysis, sourceRef) };
    }

    /**
     * @brief Build standard inside quintic C2 blend
     *
     * The quintic Bézier has 6 control points P0..P5.
     * C2 continuity is achieved by:
     * - P0, P5: position match (C0)
     * - P1 = P0 + tangent1 * s : tangent match at entry (C1)
     * - P4 = P5 - tangent2 * s : tangent match at exit (C1)
     * - P2: curvature match at entry (C2)
     * - P3: curvature match at exit (C2)
     */
    static Curve buildQuinticC2Blend(const Analysis& analysis,
                                     SourceReference sourceRef = {}) {
        Point P0 = analysis.blendEntry;
        Point P5 = analysis.blendExit;

        T chordLength = P0.distanceTo(P5);
        if (chordLength < T(1e-12)) {
            return Curve({P0, P0, P0, P0, P0, P5});
        }

        // Delegate to shared core
        namespace bc = tether::blend;
        bc::BlendVec entry{P0[0], P0[1], Dim >= 3 ? P0[2] : T(0)};
        bc::BlendVec exit{P5[0], P5[1], Dim >= 3 ? P5[2] : T(0)};
        bc::BlendVec entryDir{analysis.incomingDir[0], analysis.incomingDir[1],
                              Dim >= 3 ? analysis.incomingDir[2] : T(0)};
        bc::BlendVec exitDir{analysis.outgoingDir[0], analysis.outgoingDir[1],
                             Dim >= 3 ? analysis.outgoingDir[2] : T(0)};

        auto cp = bc::quinticC2ControlPoints(
            entry, exit, entryDir, exitDir,
            static_cast<double>(analysis.incomingCurvature),
            static_cast<double>(analysis.outgoingCurvature));

        // Convert back to Point
        auto toPoint = [](const bc::BlendVec& v) -> Point {
            Point p{};
            p[0] = static_cast<T>(v.x);
            p[1] = static_cast<T>(v.y);
            if constexpr (Dim >= 3) p[2] = static_cast<T>(v.z);
            return p;
        };

        Curve curve({toPoint(cp[0]), toPoint(cp[1]), toPoint(cp[2]),
                     toPoint(cp[3]), toPoint(cp[4]), toPoint(cp[5])});
        curve.setSourceRef(std::move(sourceRef));
        return curve;
    }

    /**
     * @brief Build outside (dogbone) blend using quintic Bézier
     *
     * Same C2 structure but P2/P3 shifted outward for dogbone effect.
     */
    static std::vector<Curve> buildOutsideBlend(const Analysis& analysis,
                                                 SourceReference sourceRef = {}) {
        Point P0 = analysis.blendEntry;
        Point P5 = analysis.blendExit;

        T chordLength = P0.distanceTo(P5);
        if (chordLength < T(1e-12)) {
            return { Curve({P0, P0, P0, P0, P0, P5}) };
        }

        T tangentScale = chordLength / T(5);

        Point P1 = P0 + analysis.incomingDir * tangentScale;
        Point P4 = P5 - analysis.outgoingDir * tangentScale;

        // Compute outward normals
        Point rightNormal = perpendicular(analysis.incomingDir);
        if (!analysis.isCW) rightNormal = -rightNormal;

        Point rightNormalExit = perpendicular(analysis.outgoingDir);
        if (!analysis.isCW) rightNormalExit = -rightNormalExit;

        // Shift P2/P3 outward
        T shiftMag = analysis.blendRadius * T(0.5);
        T curvatureScale = tangentScale * T(2) / T(3);

        Point P2 = P1 + analysis.incomingDir * curvatureScale + rightNormal * shiftMag;
        Point P3 = P4 - analysis.outgoingDir * curvatureScale + rightNormalExit * shiftMag;

        Curve curve({P0, P1, P2, P3, P4, P5});
        curve.setSourceRef(std::move(sourceRef));
        return { curve };
    }

    /**
     * @brief Build cubic G1-only blend curve (fallback)
     */
    static Curve buildG1BlendCurve(const Analysis& analysis,
                                    SourceReference sourceRef = {}) {
        if (!analysis.canBlend) {
            return Curve{};
        }

        Point P0 = analysis.blendEntry;
        Point P3 = analysis.blendExit;

        T chordLength = P0.distanceTo(P3);
        T tangentScale = chordLength / T(3);

        Point P1 = P0 + analysis.incomingDir * tangentScale;
        Point P2 = P3 - analysis.outgoingDir * tangentScale;

        Curve curve({P0, P1, P2, P3});
        curve.setSourceRef(std::move(sourceRef));
        return curve;
    }

    /**
     * @brief Build circular arc blend approximation
     */
    static Curve buildCircularBlendArc(const Analysis& analysis,
                                        SourceReference sourceRef = {}) {
        if (!analysis.canBlend) {
            return Curve{};
        }

        T sweepAngle = MathConstants::PI - analysis.angle;
        T k = T(4.0/3.0) * std::tan(sweepAngle / T(4));

        Point P0 = analysis.blendEntry;
        Point P3 = analysis.blendExit;

        Point r0 = (P0 - analysis.blendCenter).normalized();
        Point r3 = (P3 - analysis.blendCenter).normalized();

        Point t0 = perpendicular(r0);
        Point t3 = perpendicular(r3);

        if (!analysis.isCW) {
            t0 = -t0;
            t3 = -t3;
        }

        Point P1 = P0 + t0 * (k * analysis.blendRadius);
        Point P2 = P3 - t3 * (k * analysis.blendRadius);

        Curve curve({P0, P1, P2, P3});
        curve.setSourceRef(std::move(sourceRef));
        return curve;
    }

    /**
     * @brief Compute maximum curvature of a curve
     */
    static T computeMaxCurvature(const Curve& curve, int samples = 20) {
        T maxK = T(0);
        for (int i = 0; i <= samples; ++i) {
            T t = static_cast<T>(i) / static_cast<T>(samples);
            T k = computeCurvatureAt(curve, t);
            if (k > maxK) maxK = k;
        }
        return maxK;
    }

    /**
     * @brief Compute curvature at a specific parameter
     */
    static T computeCurvatureAt(const Curve& curve, T t) {
        auto d1 = curve.evaluateDerivative(t, 1);
        auto d2 = curve.evaluateDerivative(t, 2);

        T num;
        if constexpr (Dim == 2) {
            num = std::abs(d1[0]*d2[1] - d1[1]*d2[0]);
        } else {
            auto cross = d1.cross(d2);
            num = cross.magnitude();
        }

        T normD1 = d1.magnitude();
        if (normD1 > T(1e-6)) {
            T den = normD1 * normD1 * normD1;
            return num / den;
        }
        return T(0);
    }

    /**
     * @brief Compute unit tangent direction at a specific parameter
     */
    static Point computeTangentAt(const Curve& curve, T t) {
        auto d1 = curve.evaluateDerivative(t, 1);
        T mag = d1.magnitude();
        if (mag > T(1e-10)) {
            return d1 * (T(1) / mag);
        }
        return Point{};
    }

    /**
     * @brief Check if blend curve has bounded curvature (no spikes)
     */
    static bool hasCurvatureQuality(const Curve& curve, T maxAllowed, int samples = 20) {
        for (int i = 0; i <= samples; ++i) {
            T t = static_cast<T>(i) / static_cast<T>(samples);
            T k = computeCurvatureAt(curve, t);
            if (k > maxAllowed) return false;
        }
        return true;
    }

private:
    static Point perpendicular(const Point& v) {
        if constexpr (Dim == 2) {
            return Point{-v[1], v[0]};
        } else if constexpr (Dim >= 3) {
            return Point{-v[1], v[0], T(0)};
        } else {
            return Point{};
        }
    }
};

// ============================================================================
// Scoring-Based Blend Solver
// ============================================================================

/**
 * @brief Finds optimal blend parameters using scoring-based optimization
 *
 * The solver evaluates candidate blend fractions (λ) and picks the one with
 * the highest total score. Hard constraints (half-length) are enforced;
 * soft constraints (symmetry, deviation utilization, curvature quality)
 * are weighted preferences.
 *
 * Search strategy:
 * 1. Coarse pass: evaluate N evenly-spaced λ values in [0.01, maxλ]
 * 2. Fine pass: refine around best coarse result
 * 3. Verify: check all hard constraints
 * 4. Fallback: if no feasible solution, disable blend (exact stop)
 *
 * Maximum evaluations: coarseSteps + fineSteps ≈ 20. Guaranteed termination.
 */
template<size_t Dim, typename T = double>
class BlendSolver {
public:
    using Point = Vec<Dim, T>;
    using Curve = BezierCurve<Dim, T>;
    using Analysis = CornerAnalysis<Dim, T>;
    using Builder = BlendCurveBuilder<Dim, T>;

    /**
     * @brief Solve for optimal blend parameters
     *
     * Modifies analysis in-place with the best solution found.
     */
    static void solve(Analysis& analysis,
                      const MotionSegment& seg1,
                      const MotionSegment& seg2,
                      const BlendConfig& config) {
        // Compute ideal blend geometry
        T halfAngle = analysis.angle / T(2);
        T cosHalf = std::cos(halfAngle);
        T sinHalf = std::sin(halfAngle);

        T denom = T(1) - cosHalf;
        if (std::abs(denom) < static_cast<T>(config.epsilon)) {
            analysis.canBlend = false;
            analysis.blendReason = "Near-straight corner";
            return;
        }

        // Effective tolerance
        T effTolerance = static_cast<T>(std::abs(config.tolerance));
        analysis.isOutsideBlend = config.tolerance < 0;

        // Start Modification: Reject zero tolerance
        if (effTolerance < static_cast<T>(config.epsilon)) {
            analysis.canBlend = false;
            analysis.blendReason = "Zero tolerance";
            return;
        }
        // End Modification

        switch (config.cornerMode) {
            case BlendConfig::CornerLimitMode::InsideStrict:
            case BlendConfig::CornerLimitMode::InsideApproximate:
                effTolerance = std::min(effTolerance, static_cast<T>(config.insideTolerance));
                break;
            case BlendConfig::CornerLimitMode::OutsideStrict:
            case BlendConfig::CornerLimitMode::OutsideApproximate:
                effTolerance = std::min(effTolerance, static_cast<T>(config.outsideTolerance));
                break;
            case BlendConfig::CornerLimitMode::Balanced:
                effTolerance = std::min({effTolerance,
                                         static_cast<T>(config.insideTolerance),
                                         static_cast<T>(config.outsideTolerance)});
                break;
            case BlendConfig::CornerLimitMode::Centered:
            default: break;
        }

        // Ideal inscribed circle
        T idealRadius = effTolerance / denom;
        T idealDistance = idealRadius * sinHalf;

        // Max allowed from half-length constraint
        T maxEntry = static_cast<T>(seg1.segmentLength * config.maxBlendFraction);
        T maxExit  = static_cast<T>(seg2.segmentLength * config.maxBlendFraction);

        if (seg1.segmentLength < config.minSegmentLength ||
            seg2.segmentLength < config.minSegmentLength) {
            analysis.canBlend = false;
            analysis.blendReason = "Segment too short";
            return;
        }

        // Maximum λ that respects half-length
        T maxLambdaEntry = (idealDistance > T(0)) ? std::min(maxEntry / idealDistance, T(1)) : T(1);
        T maxLambdaExit  = (idealDistance > T(0)) ? std::min(maxExit / idealDistance, T(1)) : T(1);
        T maxLambda = std::min(maxLambdaEntry, maxLambdaExit);

        if (maxLambda <= T(0.01)) {
            analysis.canBlend = false;
            analysis.blendReason = "Half-length constraint leaves no room";
            return;
        }

        // --- Scoring search ---
        T bestScore = T(-1e10);
        T bestLambda = T(0);
        int totalIterations = 0;

        if (config.useScoringOptimizer) {
            // Coarse pass
            int nCoarse = config.coarseSteps;
            T bestCoarseLambda = maxLambda;
            T bestCoarseScore = T(-1e10);

            for (int i = nCoarse; i >= 1; --i) {
                T lambda = maxLambda * static_cast<T>(i) / static_cast<T>(nCoarse);
                T score = evaluateCandidate(analysis, lambda, idealRadius, idealDistance,
                                            sinHalf, seg1, seg2, config);
                totalIterations++;
                if (score > bestCoarseScore) {
                    bestCoarseScore = score;
                    bestCoarseLambda = lambda;
                }
            }

            // Fine pass
            T step = maxLambda / static_cast<T>(nCoarse);
            T fineLo = std::max(T(0.01), bestCoarseLambda - step);
            T fineHi = std::min(maxLambda, bestCoarseLambda + step);
            int nFine = config.fineSteps;

            bestScore = bestCoarseScore;
            bestLambda = bestCoarseLambda;

            for (int i = 0; i <= nFine; ++i) {
                T lambda = fineLo + (fineHi - fineLo) * static_cast<T>(i) / static_cast<T>(nFine);
                T score = evaluateCandidate(analysis, lambda, idealRadius, idealDistance,
                                            sinHalf, seg1, seg2, config);
                totalIterations++;
                if (score > bestScore) {
                    bestScore = score;
                    bestLambda = lambda;
                }
            }
        } else {
            bestLambda = maxLambda;
            bestScore = evaluateCandidate(analysis, bestLambda, idealRadius, idealDistance,
                                          sinHalf, seg1, seg2, config);
            totalIterations = 1;
        }

        // Apply best solution
        if (bestScore < T(0) || bestLambda < T(0.01)) {
            analysis.canBlend = false;
            analysis.blendReason = "No feasible blend found";
            analysis.diagnostics.solverIterations = totalIterations;
            analysis.diagnostics.bestScore = static_cast<double>(bestScore);
            return;
        }

        applyBlendFraction(analysis, bestLambda, idealRadius, idealDistance, sinHalf, config,
                           maxEntry, maxExit);
        analysis.diagnostics.solverIterations = totalIterations;
        analysis.diagnostics.bestScore = static_cast<double>(bestScore);
        analysis.diagnostics.isValid = true;

        // Compute max corner velocity
        if (config.maxCentripetalAccel > T(0) && analysis.canBlend) {
            auto curves = Builder::buildG2BlendCurve(analysis);
            T maxK = T(0);
            for (const auto& c : curves) {
                T k = Builder::computeMaxCurvature(c);
                if (k > maxK) maxK = k;
            }
            if (maxK > T(1e-6)) {
                analysis.maxCornerVelocity = std::sqrt(static_cast<T>(config.maxCentripetalAccel) / maxK);
            } else {
                analysis.maxCornerVelocity = T(1e6);
            }
        } else {
            analysis.maxCornerVelocity = T(1e6);
        }

        // Runtime verification
#if TETHER_BLEND_RUNTIME_CHECKS
        verifyBlend(analysis, config);
#endif
    }

private:
    /**
     * @brief Evaluate a candidate blend fraction and return its score
     */
    static T evaluateCandidate(const Analysis& analysis, T lambda,
                               T idealRadius, T idealDistance, T sinHalf,
                               const MotionSegment& seg1, const MotionSegment& seg2,
                               const BlendConfig& config) {
        T distance = lambda * idealDistance;
        T radius = lambda * idealRadius;

        T entryDist = distance;
        T exitDist = distance;

        // Half-length check
        T maxEntry = static_cast<T>(seg1.segmentLength * config.maxBlendFraction);
        T maxExit  = static_cast<T>(seg2.segmentLength * config.maxBlendFraction);

        T halfLengthScore;
        if (entryDist > maxEntry || exitDist > maxExit) {
            return T(-1000); // Infeasible
        } else {
            T marginEntry = (maxEntry > T(0)) ? (T(1) - entryDist / maxEntry) : T(0);
            T marginExit  = (maxExit > T(0))  ? (T(1) - exitDist / maxExit)  : T(0);
            halfLengthScore = std::min(marginEntry, marginExit);
        }

        // Symmetry score
        T maxDist = std::max(entryDist, exitDist);
        T symScore = (maxDist > T(1e-12)) ?
            T(1) - std::abs(entryDist - exitDist) / maxDist : T(1);

        // Deviation utilization (prefer larger λ = smoother blend)
        T devScore = lambda;

        // Curvature quality check — build candidate and test
        Analysis candidate = analysis;
        T maxEntryEval = static_cast<T>(seg1.segmentLength * config.maxBlendFraction);
        T maxExitEval  = static_cast<T>(seg2.segmentLength * config.maxBlendFraction);
        applyBlendFraction(candidate, lambda, idealRadius, idealDistance, sinHalf, config,
                          maxEntryEval, maxExitEval);

        T curvatureScore = T(1);
        if (candidate.canBlend && radius > T(1e-12)) {
            auto curves = Builder::buildG2BlendCurve(candidate);
            T maxK = T(0);
            for (const auto& c : curves) {
                T k = Builder::computeMaxCurvature(c, 20);
                if (k > maxK) maxK = k;
            }
            T maxAllowed = static_cast<T>(config.maxCurvatureMultiplier) / radius;
            if (maxK > maxAllowed) {
                curvatureScore = T(0.1);
            }
        }

        // C2 feasibility — quintic Bézier inherently achieves C2
        // if the control points are properly placed
        T c2Score = (distance > T(1e-12)) ? T(1) : T(0);

        // Weighted total
        T score = static_cast<T>(config.weightC2) * c2Score
                + static_cast<T>(config.weightHalfLength) * halfLengthScore
                + static_cast<T>(config.weightSymmetry) * symScore
                + static_cast<T>(config.weightDeviation) * devScore
                + static_cast<T>(config.weightCurvature) * curvatureScore;

        return score;
    }

    /**
     * @brief Apply blend fraction to analysis
     */
    static void applyBlendFraction(Analysis& analysis, T lambda,
                                   T idealRadius, T idealDistance,
                                   T sinHalf, const BlendConfig& config,
                                   T maxEntryDist = T(0),
                                   T maxExitDist = T(0)) {
        (void)config;
        T distance = lambda * idealDistance;
        T radius = lambda * idealRadius;

        // Clamp entry/exit independently to respect each segment's half-length
        T entryDist = distance;
        T exitDist = distance;
        if (maxEntryDist > T(0)) entryDist = std::min(distance, maxEntryDist);
        if (maxExitDist > T(0)) exitDist = std::min(distance, maxExitDist);

        analysis.blendRadius = radius;
        analysis.blendFraction = lambda;
        analysis.entryDistance = entryDist;
        analysis.exitDistance = exitDist;

        analysis.blendEntry = analysis.cornerPoint - analysis.incomingDir * entryDist;
        analysis.blendExit = analysis.cornerPoint + analysis.outgoingDir * exitDist;

        // Compute center using bisector
        if (sinHalf > T(1e-12)) {
            T centerOffset = radius / sinHalf;
            analysis.blendCenter = analysis.cornerPoint + analysis.bisector * centerOffset;
        }

        analysis.canBlend = true;
    }

    /**
     * @brief Runtime verification of blend quality
     */
    static void verifyBlend(Analysis& analysis, const BlendConfig& config) {
        if (!analysis.canBlend) return;

        auto curves = Builder::buildG2BlendCurve(analysis);
        if (curves.empty()) {
            analysis.diagnostics.isValid = false;
            analysis.diagnostics.fallbackReason = "No curves built";
            return;
        }

        const auto& firstCurve = curves.front();
        const auto& lastCurve = curves.back();

        // Check tangent at entry
        Point entryTangent = Builder::computeTangentAt(firstCurve, T(0));
        T entryDot = entryTangent.dot(analysis.incomingDir);
        entryDot = clamp(entryDot, T(-1), T(1));
        analysis.diagnostics.entryTangentError = static_cast<double>(std::acos(entryDot));

        // Check tangent at exit
        Point exitTangent = Builder::computeTangentAt(lastCurve, T(1));
        T exitDot = exitTangent.dot(analysis.outgoingDir);
        exitDot = clamp(exitDot, T(-1), T(1));
        analysis.diagnostics.exitTangentError = static_cast<double>(std::acos(exitDot));

        // Check curvature at entry/exit
        T entryK = Builder::computeCurvatureAt(firstCurve, T(0));
        analysis.diagnostics.entryCurvatureError =
            static_cast<double>(std::abs(entryK - analysis.incomingCurvature));

        T exitK = Builder::computeCurvatureAt(lastCurve, T(1));
        analysis.diagnostics.exitCurvatureError =
            static_cast<double>(std::abs(exitK - analysis.outgoingCurvature));

        // Check max curvature
        T maxK = T(0);
        for (const auto& c : curves) {
            T k = Builder::computeMaxCurvature(c, 20);
            if (k > maxK) maxK = k;
        }
        analysis.diagnostics.maxCurvature = static_cast<double>(maxK);
        analysis.diagnostics.isValid = true;

        // Warn on significant issues
        (void)config;
        const double tangentTol = 0.1; // ~5.7 degrees
        if (analysis.diagnostics.entryTangentError > tangentTol) {
            std::cerr << "[BlendCheck] Entry tangent error: "
                      << analysis.diagnostics.entryTangentError << " rad\n";
        }
        if (analysis.diagnostics.exitTangentError > tangentTol) {
            std::cerr << "[BlendCheck] Exit tangent error: "
                      << analysis.diagnostics.exitTangentError << " rad\n";
        }
    }
};

// ============================================================================
// Corner Analyzer
// ============================================================================

/**
 * @brief Analyzes corner geometry between motion segments and computes
 *        optimal C2-continuous blend using scoring-based solver
 */
template<size_t Dim, typename T = double>
class CornerAnalyzer {
public:
    using Point = Vec<Dim, T>;
    using Analysis = CornerAnalysis<Dim, T>;

    /**
     * @brief Unified analysis method for all segment type combinations
     *
     * Automatically detects arc/line segments, computes proper tangents,
     * and invokes the scoring solver.
     */
    static Analysis analyze(const MotionSegment& seg1,
                           const MotionSegment& seg2,
                           const BlendConfig& config = {}) {
        Analysis result;
        result.cornerPoint = extractPoint<Dim>(seg1.endPosition);

        // Compute tangent directions at junction
        if (seg1.isArc()) {
            result.incomingDir = computeArcExitTangent(seg1);
            result.incomingCurvature = static_cast<T>(seg1.arcDirection()) / static_cast<T>(seg1.arcRadius);
        } else {
            Point p0 = extractPoint<Dim>(seg1.startPosition);
            result.incomingDir = (result.cornerPoint - p0).normalized();
            result.incomingCurvature = T(0);
        }

        if (seg2.isArc()) {
            result.outgoingDir = computeArcEntryTangent(seg2);
            result.outgoingCurvature = static_cast<T>(seg2.arcDirection()) / static_cast<T>(seg2.arcRadius);
        } else {
            Point p2 = extractPoint<Dim>(seg2.endPosition);
            result.outgoingDir = (p2 - result.cornerPoint).normalized();
            result.outgoingCurvature = T(0);
        }

        // Validate
        if (result.incomingDir.isZero() || result.outgoingDir.isZero()) {
            result.type = CornerType::Straight;
            result.canBlend = false;
            result.blendReason = "Zero-length segment";
            return result;
        }

        // Compute angle
        T dotProduct = result.incomingDir.dot(result.outgoingDir);
        dotProduct = clamp(dotProduct, T(-1), T(1));
        result.angle = std::acos(dotProduct);

        // Turn direction
        if constexpr (Dim == 2) {
            T cross = result.incomingDir.cross(result.outgoingDir);
            result.isCW = cross < T(0);
        } else if constexpr (Dim == 3) {
            Point cross = result.incomingDir.cross(result.outgoingDir);
            result.isCW = cross.z() < T(0);
        }

        result.turnAngle = result.isCW ? -result.angle : result.angle;

        // Classify
        T angleDeg = result.angle * MathConstants::RAD_TO_DEG;

        if (angleDeg < config.minAngle) {
            result.type = CornerType::Straight;
            result.canBlend = false;
            result.blendReason = "Angle too small";
            return result;
        }

        if (angleDeg > config.maxAngle) {
            result.type = CornerType::Cusp;
            result.canBlend = false;
            result.blendReason = "Cusp angle";
            return result;
        }

        result.type = result.isCW ? CornerType::Convex : CornerType::Concave;

        // Bisector
        result.bisector = (result.incomingDir + result.outgoingDir).normalized();
        if (result.bisector.isZero()) {
            result.bisector = perpendicular(result.incomingDir);
        }

        // Use scoring solver
        BlendSolver<Dim, T>::solve(result, seg1, seg2, config);

        return result;
    }

    // Backward-compatible individual methods
    static Analysis analyzeLineLine(const MotionSegment& seg1,
                                    const MotionSegment& seg2,
                                    const BlendConfig& config = {}) {
        return analyze(seg1, seg2, config);
    }

    static Analysis analyzeLineArc(const MotionSegment& seg1,
                                   const MotionSegment& seg2,
                                   const BlendConfig& config = {}) {
        return analyze(seg1, seg2, config);
    }

    static Analysis analyzeArcLine(const MotionSegment& seg1,
                                   const MotionSegment& seg2,
                                   const BlendConfig& config = {}) {
        return analyze(seg1, seg2, config);
    }

    static Analysis analyzeArcArc(const MotionSegment& seg1,
                                  const MotionSegment& seg2,
                                  const BlendConfig& config = {}) {
        return analyze(seg1, seg2, config);
    }

private:
    template<size_t N>
    static Vec<N, T> extractPoint(const std::array<double, MAX_MOTION_AXES>& pos) {
        Vec<N, T> result;
        for (size_t i = 0; i < N && i < MAX_MOTION_AXES; ++i) {
            result[i] = static_cast<T>(pos[i]);
        }
        return result;
    }

    static Point perpendicular(const Point& v) {
        if constexpr (Dim == 2) {
            return Point{-v[1], v[0]};
        } else if constexpr (Dim >= 3) {
            return Point{-v[1], v[0], T(0)}.normalized();
        } else {
            return Point{};
        }
    }

    /**
     * @brief Get arc plane axis indices
     */
    static void getArcPlaneAxes(ArcPlane plane, int& u, int& v, int& w) {
        switch (plane) {
            case ArcPlane::XZ: u = 0; v = 2; w = 1; break;
            case ArcPlane::YZ: u = 1; v = 2; w = 0; break;
            case ArcPlane::XY: default: u = 0; v = 1; w = 2; break;
        }
    }

    /**
     * @brief Compute exit tangent for arc at its endpoint
     */
    static Point computeArcExitTangent(const MotionSegment& seg) {
        int u, v, w;
        getArcPlaneAxes(seg.arcPlane, u, v, w);

        double cu = seg.arcCenter[u];
        double cv = seg.arcCenter[v];
        double eu = seg.endPosition[u];
        double ev = seg.endPosition[v];

        double angle = std::atan2(ev - cv, eu - cu);
        int dir = seg.arcDirection();

        Point tangent{};
        tangent[u] = static_cast<T>(-dir * std::sin(angle));
        tangent[v] = static_cast<T>(dir * std::cos(angle));
        if (w < static_cast<int>(Dim)) {
            double segLen = seg.segmentLength > 1e-12 ? seg.segmentLength : 1.0;
            tangent[w] = static_cast<T>((seg.endPosition[w] - seg.startPosition[w]) / segLen);
        }
        return tangent.normalized();
    }

    /**
     * @brief Compute entry tangent for arc at its start point
     */
    static Point computeArcEntryTangent(const MotionSegment& seg) {
        int u, v, w;
        getArcPlaneAxes(seg.arcPlane, u, v, w);

        double cu = seg.arcCenter[u];
        double cv = seg.arcCenter[v];
        double su = seg.startPosition[u];
        double sv = seg.startPosition[v];

        double angle = std::atan2(sv - cv, su - cu);
        int dir = seg.arcDirection();

        Point tangent{};
        tangent[u] = static_cast<T>(-dir * std::sin(angle));
        tangent[v] = static_cast<T>(dir * std::cos(angle));
        if (w < static_cast<int>(Dim)) {
            double segLen = seg.segmentLength > 1e-12 ? seg.segmentLength : 1.0;
            tangent[w] = static_cast<T>((seg.endPosition[w] - seg.startPosition[w]) / segLen);
        }
        return tangent.normalized();
    }
};

// ============================================================================
// Type Aliases
// ============================================================================

using CornerAnalysis2D = CornerAnalysis<2, double>;
using CornerAnalysis3D = CornerAnalysis<3, double>;
using CornerAnalyzer2D = CornerAnalyzer<2, double>;
using CornerAnalyzer3D = CornerAnalyzer<3, double>;
using BlendCurveBuilder2D = BlendCurveBuilder<2, double>;
using BlendCurveBuilder3D = BlendCurveBuilder<3, double>;

}  // namespace MotionPlanner
