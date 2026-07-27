/**
 * @file BlendCore.hpp
 * @brief Shared algorithmic core for corner blending
 *
 * @details
 * Pure-geometry functions shared between the MotionPlanner
 * and the G64 path blender (G64CornerMode.hpp).
 *
 * This header is deliberately independent of both MotionSegment and
 * PlanningSegment. It operates on a minimal BlendVec type (x, y, z) and
 * scalar parameters. Each system converts to/from BlendVec at the call site.
 *
 * Shared algorithms:
 * - Corner angle, bisector, turn direction
 * - Blend radius from tolerance
 * - Blend entry/exit point computation
 * - Quintic Bézier C2 control point placement
 * - Half-length constraint clamping
 * - Iterative constraint reduction
 */

#pragma once

#include <array>
#include <algorithm>
#include <cmath>

namespace tether::blend {

// ============================================================================
// Minimal 3D vector — no dependencies on either system's types
// ============================================================================

struct BlendVec {
    double x = 0, y = 0, z = 0;

    BlendVec() = default;
    BlendVec(double x_, double y_, double z_ = 0) : x(x_), y(y_), z(z_) {}

    BlendVec operator+(const BlendVec& o) const { return {x+o.x, y+o.y, z+o.z}; }
    BlendVec operator-(const BlendVec& o) const { return {x-o.x, y-o.y, z-o.z}; }
    BlendVec operator*(double s) const { return {x*s, y*s, z*s}; }

    double dot(const BlendVec& o) const { return x*o.x + y*o.y + z*o.z; }

    double norm() const { return std::sqrt(x*x + y*y + z*z); }

    BlendVec normalized() const {
        double n = norm();
        if (n < 1e-15) return {};
        return {x/n, y/n, z/n};
    }

    double distanceTo(const BlendVec& o) const {
        double dx = x-o.x, dy = y-o.y, dz = z-o.z;
        return std::sqrt(dx*dx + dy*dy + dz*dz);
    }

    bool isZero() const {
        return std::abs(x) < 1e-15 && std::abs(y) < 1e-15 && std::abs(z) < 1e-15;
    }
};

inline BlendVec perpendicular(const BlendVec& v) {
    return {-v.y, v.x, 0.0};
}

inline BlendVec cross(const BlendVec& a, const BlendVec& b) {
    return {a.y*b.z - a.z*b.y,
            a.z*b.x - a.x*b.z,
            a.x*b.y - a.y*b.x};
}

// ============================================================================
// Corner geometry
// ============================================================================

struct CornerGeometry {
    double angleRad = 0;       ///< Angle between directions (0..π)
    double turnAngleRad = 0;   ///< Signed turn (negative=CW/right, positive=CCW/left)
    bool isCW = false;
    BlendVec bisector;         ///< Unit bisector pointing into the corner
    BlendVec cornerPoint;
    BlendVec incomingDir;      ///< Unit vector
    BlendVec outgoingDir;      ///< Unit vector
};

inline CornerGeometry analyzeCorner(const BlendVec& cornerPoint,
                                    const BlendVec& incomingDir,
                                    const BlendVec& outgoingDir) {
    CornerGeometry cg;
    cg.cornerPoint = cornerPoint;
    cg.incomingDir = incomingDir.normalized();
    cg.outgoingDir = outgoingDir.normalized();

    double dot = cg.incomingDir.dot(cg.outgoingDir);
    dot = std::max(-1.0, std::min(1.0, dot));
    cg.angleRad = std::acos(dot);

    BlendVec c = cross(cg.incomingDir, cg.outgoingDir);
    cg.isCW = c.z < 0;
    cg.turnAngleRad = cg.isCW ? -cg.angleRad : cg.angleRad;

    BlendVec bisum = cg.incomingDir + cg.outgoingDir;
    cg.bisector = bisum.normalized();

    return cg;
}

// ============================================================================
// Blend radius and entry/exit computation
// ============================================================================

/**
 * @brief Compute blend radius from tolerance and half-angle
 * @param tolerance Maximum path deviation (G64 P value)
 * @param halfAngle Half the corner angle in radians
 * @return Blend radius
 */
inline double blendRadiusFromTolerance(double tolerance, double halfAngle) {
    double cosHalf = std::cos(halfAngle);
    double denom = 1.0 - cosHalf;
    if (std::abs(denom) < 1e-15) return 0.0;
    return tolerance * cosHalf / denom;
}

/**
 * @brief Compute tangent distance from radius and half-angle
 */
inline double tangentDistance(double radius, double halfAngle) {
    return radius * std::tan(halfAngle);
}

/**
 * @brief Compute blend entry and exit points
 */
inline std::pair<BlendVec, BlendVec> blendEntryExit(
    const CornerGeometry& cg, double entryDist, double exitDist) {
    BlendVec entry = cg.cornerPoint - cg.incomingDir * entryDist;
    BlendVec exit  = cg.cornerPoint + cg.outgoingDir * exitDist;
    return {entry, exit};
}

/**
 * @brief Compute blend center (for arc-based fallback)
 */
inline BlendVec blendCenter(const CornerGeometry& cg, double radius, double sinHalf) {
    if (sinHalf < 1e-12) return {};
    double offset = radius / sinHalf;
    return cg.cornerPoint + cg.bisector * offset;
}

// ============================================================================
// Half-length constraint clamping
// ============================================================================

/**
 * @brief Clamp blend radius so entry/exit distances don't exceed maxBlendFraction
 * @param radius Current blend radius
 * @param halfAngle Half corner angle (radians)
 * @param maxEntryDist Max distance along incoming segment
 * @param maxExitDist Max distance along outgoing segment
 * @return Clamped radius
 */
inline double clampBlendRadius(double radius, double halfAngle,
                               double maxEntryDist, double maxExitDist) {
    double tanHalf = std::tan(halfAngle);
    if (tanHalf < 1e-12) return radius;
    double currentDist = radius * tanHalf;
    if (currentDist > maxEntryDist || currentDist > maxExitDist) {
        double maxDist = std::min(maxEntryDist, maxExitDist);
        return maxDist / tanHalf;
    }
    return radius;
}

// ============================================================================
// Quintic Bézier C2 control points
// ============================================================================

/**
 * @brief Quintic Bézier control points for C2-continuous blend
 *
 * For a quintic Bézier with control points P0..P5:
 *   C(0) = P0, C'(0) = 5*(P1-P0), C''(0) = 20*(P2-2*P1+P0)
 *   C(1) = P5, C'(1) = 5*(P5-P4), C''(1) = 20*(P5-2*P4+P3)
 *
 * Curvature at t=0: κ(0) = (4/5) * |normal(P2-2P1+P0)| / tangentScale²
 * Required normal offset = (5/4) * κ * tangentScale²
 *
 * @param entryPoint  P0
 * @param exitPoint   P5
 * @param entryDir    Unit tangent at entry
 * @param exitDir     Unit tangent at exit (pointing away from exit)
 * @param entryCurv   Curvature at entry (0 for lines, signed 1/R for arcs)
 * @param exitCurv    Curvature at exit (0 for lines, signed 1/R for arcs)
 * @return Array of 6 control points {P0, P1, P2, P3, P4, P5}
 */
inline std::array<BlendVec, 6> quinticC2ControlPoints(
    const BlendVec& entryPoint,
    const BlendVec& exitPoint,
    const BlendVec& entryDir,
    const BlendVec& exitDir,
    double entryCurv = 0.0,
    double exitCurv = 0.0) {

    BlendVec P0 = entryPoint;
    BlendVec P5 = exitPoint;

    double chordLength = P0.distanceTo(P5);
    if (chordLength < 1e-12) {
        return {P0, P0, P0, P0, P0, P5};
    }

    double s = chordLength / 5.0;  // tangentScale

    // C1: P1 along entry tangent, P4 against exit tangent
    BlendVec P1 = P0 + entryDir * s;
    BlendVec P4 = P5 - exitDir * s;

    // C2: P2 and P3 control curvature at boundaries
    // Zero curvature: collinear P0-P1-P2 and P5-P4-P3
    BlendVec P2 = P0 + entryDir * (2.0 * s);
    BlendVec P3 = P5 - exitDir * (2.0 * s);

    // Non-zero curvature: add normal offset = (5/4) * κ * s²
    if (std::abs(entryCurv) > 1e-10) {
        BlendVec norm = perpendicular(entryDir);
        double offset = (5.0 / 4.0) * entryCurv * s * s;
        P2 = P2 + norm * offset;
    }

    if (std::abs(exitCurv) > 1e-10) {
        BlendVec norm = perpendicular(exitDir);
        double offset = (5.0 / 4.0) * exitCurv * s * s;
        P3 = P3 + norm * offset;
    }

    return {P0, P1, P2, P3, P4, P5};
}

/**
 * @brief Evaluate quintic Bézier at parameter t
 */
inline BlendVec evalQuintic(const std::array<BlendVec, 6>& cp, double t) {
    double mt = 1.0 - t;
    double mt2 = mt * mt;
    double t2 = t * t;
    return cp[0] * (mt2*mt2*mt) +
           cp[1] * (5*mt2*mt2*t) +
           cp[2] * (10*mt2*t2) +
           cp[3] * (10*mt*t2*t) +
           cp[4] * (5*mt*t2*t2) +
           cp[5] * (t2*t2*t);
}

// ============================================================================
// Iterative constraint reduction
// ============================================================================

/**
 * @brief Iteratively reduce radius until constraint is satisfied or max iterations
 * @param radius Initial radius (modified in-place)
 * @param checkFn Returns true if constraint is satisfied
 * @param maxIter Maximum iterations (default 20)
 * @param reductionFactor Factor to multiply radius by each iteration (default 0.5)
 */
template<typename CheckFn>
void iterateConstraintReduction(double& radius, CheckFn checkFn,
                                int maxIter = 20, double reductionFactor = 0.5) {
    for (int i = 0; i < maxIter; ++i) {
        if (checkFn(radius)) break;
        radius *= reductionFactor;
    }
}

} // namespace tether::blend

// ============================================================================
// Segment Traits — adapters for converting segment types to BlendVec
// ============================================================================
// These are opt-in: only included when the caller has the relevant header.
// Each specialization provides:
//   position(seg, isEnd)  -> BlendVec
//   tangent(seg, isEnd)   -> BlendVec (unit tangent at start or end)
//   curvature(seg, isEnd) -> double (signed 1/R, 0 for lines)
//   length(seg)           -> double
//   isArc(seg)            -> bool
//   arcDirection(seg)     -> int (+1 CCW, -1 CW)
// ============================================================================

#include "MotionSegment.hpp"

namespace tether::blend {

template<typename Seg>
struct SegmentTraits;

// --- MotionSegment specialization (MotionPlanner namespace) ---
template<>
struct SegmentTraits<MotionPlanner::MotionSegment> {
    using Seg = MotionPlanner::MotionSegment;

    static BlendVec position(const Seg& s, bool isEnd) {
        const auto& p = isEnd ? s.endPosition : s.startPosition;
        return {p[0], p[1], p[2]};
    }

    static BlendVec tangent(const Seg& s, bool isEnd) {
        BlendVec start = position(s, false);
        BlendVec end = position(s, true);
        if (isEnd) return (end - start).normalized();
        return (end - start).normalized();
    }

    static double curvature(const Seg& s, bool /*isEnd*/) {
        if (!s.isArc() || s.arcRadius < 1e-15) return 0.0;
        return static_cast<double>(s.arcDirection()) / s.arcRadius;
    }

    static double length(const Seg& s) { return s.segmentLength; }
    static bool isArc(const Seg& s) { return s.isArc(); }
    static int arcDirection(const Seg& s) { return s.arcDirection(); }
};

} // namespace tether::blend
