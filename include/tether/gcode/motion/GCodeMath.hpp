/**
 * @file GCodeMath.hpp
 * @brief Core mathematical functions for G-Code motion calculations
 *
 * This header provides fundamental mathematical operations used across
 * the G-Code parsing, interpretation, and interpolation subsystems.
 * Previously embedded in GCodeCAPI.cpp, these functions are now
 * centralized for reuse and consistency.
 */

#pragma once

#include <cmath>
#include <cstdint>
#include <algorithm>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#ifndef M_2_PI
#define M_2_PI (2.0 * M_PI)
#endif

namespace GCode {
namespace Math {

// ============================================================================
// Basic Math Utilities
// ============================================================================

/**
 * @brief Clamp a value to a range
 * @param v Value to clamp
 * @param lo Lower bound (inclusive)
 * @param hi Upper bound (inclusive)
 * @return Clamped value
 */
inline double clamp(double v, double lo, double hi) {
    return std::max(lo, std::min(hi, v));
}

/**
 * @brief Square of a number
 */
inline double sq(double x) {
    return x * x;
}

/**
 * @brief Linear interpolation
 * @param a Start value
 * @param b End value
 * @param t Parameter [0, 1]
 */
inline double lerp(double a, double b, double t) {
    return a + (b - a) * t;
}

/**
 * @brief Normalize angle to range [-π, π]
 */
inline double normalizeAngle(double angle) {
    while (angle > M_PI) angle -= M_2_PI;
    while (angle < -M_PI) angle += M_2_PI;
    return angle;
}

/**
 * @brief Normalize angle to range [0, 2π]
 */
inline double normalizeAnglePositive(double angle) {
    while (angle >= M_2_PI) angle -= M_2_PI;
    while (angle < 0) angle += M_2_PI;
    return angle;
}

/**
 * @brief Check if value is approximately zero
 */
inline bool isNearZero(double v, double epsilon = 1e-12) {
    return std::fabs(v) < epsilon;
}

/**
 * @brief Check if two values are approximately equal
 */
inline bool nearEqual(double a, double b, double epsilon = 1e-12) {
    return std::fabs(a - b) < epsilon;
}

/**
 * @brief Safe division with zero check
 */
inline double safeDivide(double num, double denom, double defaultVal = 0.0) {
    return isNearZero(denom) ? defaultVal : num / denom;
}

/**
 * @brief Safe square root (returns 0 for negative input)
 */
inline double safeSqrt(double x) {
    return (x <= 0.0) ? 0.0 : std::sqrt(x);
}

/**
 * @brief Safe acos (clamped to [-1, 1])
 */
inline double safeAcos(double x) {
    return std::acos(clamp(x, -1.0, 1.0));
}

/**
 * @brief Safe asin (clamped to [-1, 1])
 */
inline double safeAsin(double x) {
    return std::asin(clamp(x, -1.0, 1.0));
}

// ============================================================================
// Plane / Axis Mappings
// ============================================================================

/**
 * @brief Interpolation planes for arc motion
 */
enum class Plane : uint8_t {
    XY = 0,  ///< G17 - XY plane, Z is normal
    XZ = 1,  ///< G18 - XZ plane, Y is normal
    YZ = 2   ///< G19 - YZ plane, X is normal
};

/**
 * @brief Get axis indices for a given plane
 *
 * Returns the primary (u), secondary (v), and normal (w) axis indices
 * for the specified interpolation plane.
 *
 * @param plane Interpolation plane (0=XY, 1=XZ, 2=YZ)
 * @param u Output: primary axis index (horizontal in plane)
 * @param v Output: secondary axis index (vertical in plane)
 * @param w Output: normal axis index (perpendicular to plane)
 *
 * @note For G17 (XY): u=X(0), v=Y(1), w=Z(2)
 * @note For G18 (XZ): u=X(0), v=Z(2), w=Y(1)
 * @note For G19 (YZ): u=Y(1), v=Z(2), w=X(0)
 */
inline void planeAxes(uint8_t plane, int& u, int& v, int& w) {
    switch (plane) {
        case 1: // XZ (G18)
            u = 0;  // X
            v = 2;  // Z
            w = 1;  // Y
            break;
        case 2: // YZ (G19)
            u = 1;  // Y
            v = 2;  // Z
            w = 0;  // X
            break;
        case 0: // XY (G17)
        default:
            u = 0;  // X
            v = 1;  // Y
            w = 2;  // Z
            break;
    }
}

/**
 * @brief Overload accepting Plane enum
 */
inline void planeAxes(Plane plane, int& u, int& v, int& w) {
    planeAxes(static_cast<uint8_t>(plane), u, v, w);
}

// ============================================================================
// Arc Geometry Calculations
// ============================================================================

/**
 * @brief Calculate arc sweep angle from start/end/center points
 *
 * Computes the angular sweep of an arc given its endpoints and center,
 * accounting for clockwise/counter-clockwise direction.
 *
 * @param startU Start point U coordinate (in plane)
 * @param startV Start point V coordinate (in plane)
 * @param endU End point U coordinate (in plane)
 * @param endV End point V coordinate (in plane)
 * @param centerU Center point U coordinate
 * @param centerV Center point V coordinate
 * @param clockwise True for clockwise (G2), false for CCW (G3)
 * @return Sweep angle in radians (negative for CW, positive for CCW)
 *
 * @note Full circles return ±2π depending on direction
 */
inline double arcSweepFromCenter(
    double startU, double startV,
    double endU, double endV,
    double centerU, double centerV,
    bool clockwise
) {
    const double startAngle = std::atan2(startV - centerV, startU - centerU);
    const double endAngle = std::atan2(endV - centerV, endU - centerU);
    double sweep = endAngle - startAngle;

    if (clockwise) {
        // CW: need negative sweep
        while (sweep > 0) sweep -= M_2_PI;
        if (sweep == 0.0) sweep = -M_2_PI;  // Full circle
    } else {
        // CCW: need positive sweep
        while (sweep < 0) sweep += M_2_PI;
        if (sweep == 0.0) sweep = M_2_PI;  // Full circle
    }
    return sweep;
}

/**
 * @brief Calculate point on an arc
 *
 * Given arc parameters, computes the position at parameter t ∈ [0, 1].
 *
 * @param centerU Arc center U coordinate
 * @param centerV Arc center V coordinate
 * @param radius Arc radius
 * @param startAngle Angle at t=0 (radians)
 * @param sweep Total sweep angle (radians, signed for direction)
 * @param t Parameter [0, 1]
 * @param outU Output U coordinate
 * @param outV Output V coordinate
 */
inline void arcPoint(
    double centerU, double centerV,
    double radius,
    double startAngle, double sweep,
    double t,
    double& outU, double& outV
) {
    const double angle = startAngle + t * sweep;
    outU = centerU + radius * std::cos(angle);
    outV = centerV + radius * std::sin(angle);
}

/**
 * @brief Calculate tangent direction on an arc
 *
 * @param centerU Arc center U coordinate
 * @param centerV Arc center V coordinate
 * @param radius Arc radius
 * @param startAngle Angle at t=0 (radians)
 * @param sweep Total sweep angle (radians)
 * @param t Parameter [0, 1]
 * @param tanU Output tangent U component
 * @param tanV Output tangent V component
 */
inline void arcTangent(
    double centerU, double centerV,
    double radius,
    double startAngle, double sweep,
    double t,
    double& tanU, double& tanV
) {
    (void)centerU;  // Center not needed for tangent direction
    (void)centerV;
    const double angle = startAngle + t * sweep;
    // Tangent is perpendicular to radius, direction depends on sweep sign
    const double scale = (sweep >= 0) ? 1.0 : -1.0;
    tanU = -radius * scale * std::sin(angle);
    tanV = radius * scale * std::cos(angle);
}

/**
 * @brief Calculate number of segments needed for arc with given chord deviation
 *
 * Determines how many linear segments are needed to approximate an arc
 * while keeping the maximum chord-to-arc deviation below a threshold.
 *
 * @param radius Arc radius
 * @param sweepAngle Total sweep angle (radians, absolute value used)
 * @param maxChordDeviation Maximum allowed deviation from true arc
 * @return Number of segments needed (minimum 1)
 *
 * @note Uses the formula: h = r * (1 - cos(θ/2)) where h is chord deviation
 *       Solving for θ: θ = 2 * acos(1 - h/r)
 */
inline size_t arcPointCountForDeviation(
    double radius,
    double sweepAngle,
    double maxChordDeviation
) {
    if (radius <= 0.0 || maxChordDeviation <= 0.0) return 0;

    // Maximum angle per segment that keeps chord error within tolerance
    const double ratio = clamp(1.0 - (maxChordDeviation / radius), -1.0, 1.0);
    const double anglePerSegment = 2.0 * std::acos(ratio);

    if (anglePerSegment <= 0.0) return 0;

    return static_cast<size_t>(std::ceil(std::fabs(sweepAngle) / anglePerSegment));
}

/**
 * @brief Calculate arc length
 *
 * @param radius Arc radius
 * @param sweepAngle Sweep angle (radians, absolute value used)
 * @return Arc length
 */
inline double arcLength(double radius, double sweepAngle) {
    return std::fabs(sweepAngle) * radius;
}

/**
 * @brief Calculate curvature of an arc (constant = 1/radius)
 *
 * @param radius Arc radius
 * @return Curvature (1/radius), or 0 if radius is zero/negative
 */
inline double arcCurvature(double radius) {
    return (radius > 0.0) ? (1.0 / radius) : 0.0;
}

/**
 * @brief Find arc center from two points and radius (R-format)
 *
 * Given start and end points and a radius value, computes the two possible
 * arc centers. Returns true if valid centers exist.
 *
 * @param startU Start U coordinate
 * @param startV Start V coordinate
 * @param endU End U coordinate
 * @param endV End V coordinate
 * @param radius Arc radius (absolute value)
 * @param center1U Output: first possible center U
 * @param center1V Output: first possible center V
 * @param center2U Output: second possible center U
 * @param center2V Output: second possible center V
 * @return True if valid centers computed, false if impossible geometry
 */
inline bool findArcCenters(
    double startU, double startV,
    double endU, double endV,
    double radius,
    double& center1U, double& center1V,
    double& center2U, double& center2V
) {
    const double dx = endU - startU;
    const double dy = endV - startV;
    const double chord = std::sqrt(dx * dx + dy * dy);

    // Check if radius is large enough
    if (chord > 2.0 * std::fabs(radius)) {
        return false;  // Impossible geometry
    }

    // Midpoint of chord
    const double mx = (startU + endU) * 0.5;
    const double my = (startV + endV) * 0.5;

    // Distance from midpoint to center
    const double h = safeSqrt(radius * radius - (chord * chord) / 4.0);

    // Perpendicular unit vector
    const double invChord = safeDivide(1.0, chord, 0.0);
    const double px = -dy * invChord;
    const double py = dx * invChord;

    // Two possible centers
    center1U = mx + h * px;
    center1V = my + h * py;
    center2U = mx - h * px;
    center2V = my - h * py;

    return true;
}

/**
 * @brief Select correct arc center for R-format arc
 *
 * Given two possible centers and the R sign, selects the appropriate center
 * based on whether a major arc (|R| < 0) or minor arc (|R| > 0) is specified.
 *
 * @param startU Start U coordinate
 * @param startV Start V coordinate
 * @param endU End U coordinate
 * @param endV End V coordinate
 * @param center1U First possible center U
 * @param center1V First possible center V
 * @param center2U Second possible center U
 * @param center2V Second possible center V
 * @param rValue Signed R value (negative = major arc)
 * @param clockwise True for CW (G2), false for CCW (G3)
 * @param outCenterU Output: selected center U
 * @param outCenterV Output: selected center V
 */
inline void selectArcCenter(
    double startU, double startV,
    double endU, double endV,
    double center1U, double center1V,
    double center2U, double center2V,
    double rValue,
    bool clockwise,
    double& outCenterU, double& outCenterV
) {
    const bool wantMajor = (rValue < 0.0);

    const double sweep1 = arcSweepFromCenter(startU, startV, endU, endV, center1U, center1V, clockwise);
    const double sweep2 = arcSweepFromCenter(startU, startV, endU, endV, center2U, center2V, clockwise);

    const bool major1 = std::fabs(sweep1) > (M_PI + 1e-12);
    const bool major2 = std::fabs(sweep2) > (M_PI + 1e-12);

    bool pickFirst;
    if (major1 == wantMajor && major2 != wantMajor) {
        pickFirst = true;
    } else if (major2 == wantMajor && major1 != wantMajor) {
        pickFirst = false;
    } else {
        // Both same type, pick smaller sweep
        pickFirst = (std::fabs(sweep1) <= std::fabs(sweep2));
    }

    outCenterU = pickFirst ? center1U : center2U;
    outCenterV = pickFirst ? center1V : center2V;
}

// ============================================================================
// Bezier Curve Operations
// ============================================================================

/**
 * @brief Evaluate cubic Bezier curve at parameter t
 *
 * @param p0 Control point 0
 * @param p1 Control point 1
 * @param p2 Control point 2
 * @param p3 Control point 3
 * @param t Parameter [0, 1]
 * @return Point on curve
 */
inline double cubicBezier(double p0, double p1, double p2, double p3, double t) {
    const double s = 1.0 - t;
    return s * s * s * p0 +
           3.0 * s * s * t * p1 +
           3.0 * s * t * t * p2 +
           t * t * t * p3;
}

/**
 * @brief First derivative of cubic Bezier
 */
inline double cubicBezierDerivative(double p0, double p1, double p2, double p3, double t) {
    const double s = 1.0 - t;
    return 3.0 * s * s * (p1 - p0) +
           6.0 * s * t * (p2 - p1) +
           3.0 * t * t * (p3 - p2);
}

/**
 * @brief Second derivative of cubic Bezier
 */
inline double cubicBezierSecondDerivative(double p0, double p1, double p2, double p3, double t) {
    const double s = 1.0 - t;
    return 6.0 * s * (p2 - 2.0 * p1 + p0) +
           6.0 * t * (p3 - 2.0 * p2 + p1);
}

/**
 * @brief Compute cubic Bezier control points for arc approximation
 *
 * Approximates an arc with a cubic Bezier curve. Most accurate for
 * angles up to 90 degrees; larger arcs should be split.
 *
 * @param centerU Arc center U
 * @param centerV Arc center V
 * @param radius Arc radius
 * @param startAngle Start angle (radians)
 * @param sweep Sweep angle (radians)
 * @param p0u, p0v Output: start point
 * @param p1u, p1v Output: first control point
 * @param p2u, p2v Output: second control point
 * @param p3u, p3v Output: end point
 */
inline void arcToBezier(
    double centerU, double centerV,
    double radius,
    double startAngle, double sweep,
    double& p0u, double& p0v,
    double& p1u, double& p1v,
    double& p2u, double& p2v,
    double& p3u, double& p3v
) {
    const double endAngle = startAngle + sweep;

    // Start and end points on circle
    p0u = centerU + radius * std::cos(startAngle);
    p0v = centerV + radius * std::sin(startAngle);
    p3u = centerU + radius * std::cos(endAngle);
    p3v = centerV + radius * std::sin(endAngle);

    // Control point distance for cubic Bezier arc approximation
    // k = (4/3) * tan(θ/4) where θ is the sweep angle
    const double k = (4.0 / 3.0) * std::tan(sweep / 4.0);

    // Control points perpendicular to radii
    p1u = p0u - k * radius * std::sin(startAngle);
    p1v = p0v + k * radius * std::cos(startAngle);
    p2u = p3u + k * radius * std::sin(endAngle);
    p2v = p3v - k * radius * std::cos(endAngle);
}

// ============================================================================
// Line/Segment Operations
// ============================================================================

/**
 * @brief Distance between two 2D points
 */
inline double distance2D(double x1, double y1, double x2, double y2) {
    const double dx = x2 - x1;
    const double dy = y2 - y1;
    return std::sqrt(dx * dx + dy * dy);
}

/**
 * @brief Distance between two 3D points
 */
inline double distance3D(double x1, double y1, double z1, double x2, double y2, double z2) {
    const double dx = x2 - x1;
    const double dy = y2 - y1;
    const double dz = z2 - z1;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

/**
 * @brief Point on line segment at parameter t
 */
inline void linePoint(
    double x1, double y1, double z1,
    double x2, double y2, double z2,
    double t,
    double& outX, double& outY, double& outZ
) {
    outX = lerp(x1, x2, t);
    outY = lerp(y1, y2, t);
    outZ = lerp(z1, z2, t);
}

/**
 * @brief Unit vector from point 1 to point 2
 */
inline void unitVector(
    double x1, double y1, double z1,
    double x2, double y2, double z2,
    double& ux, double& uy, double& uz
) {
    const double dx = x2 - x1;
    const double dy = y2 - y1;
    const double dz = z2 - z1;
    const double len = std::sqrt(dx * dx + dy * dy + dz * dz);
    const double invLen = safeDivide(1.0, len, 0.0);
    ux = dx * invLen;
    uy = dy * invLen;
    uz = dz * invLen;
}

// ============================================================================
// Velocity/Acceleration Calculations
// ============================================================================

/**
 * @brief Maximum reachable velocity given acceleration and distance
 *
 * Using kinematic equation: v² = v₀² + 2as
 * For starting from rest: v_max = √(2as)
 *
 * @param acceleration Acceleration (positive)
 * @param distance Distance (positive)
 * @param initialVelocity Starting velocity (default 0)
 * @return Maximum velocity achievable
 */
inline double maxVelocityForDistance(
    double acceleration,
    double distance,
    double initialVelocity = 0.0
) {
    return safeSqrt(initialVelocity * initialVelocity + 2.0 * acceleration * distance);
}

/**
 * @brief Distance required to reach target velocity
 *
 * Using kinematic equation: s = (v² - v₀²) / (2a)
 *
 * @param acceleration Acceleration (positive)
 * @param initialVelocity Starting velocity
 * @param targetVelocity Target velocity
 * @return Distance required
 */
inline double distanceForVelocityChange(
    double acceleration,
    double initialVelocity,
    double targetVelocity
) {
    return safeDivide(
        targetVelocity * targetVelocity - initialVelocity * initialVelocity,
        2.0 * acceleration,
        0.0
    );
}

/**
 * @brief Time required to reach target velocity
 *
 * Using kinematic equation: t = (v - v₀) / a
 *
 * @param acceleration Acceleration (positive)
 * @param initialVelocity Starting velocity
 * @param targetVelocity Target velocity
 * @return Time required
 */
inline double timeForVelocityChange(
    double acceleration,
    double initialVelocity,
    double targetVelocity
) {
    return safeDivide(targetVelocity - initialVelocity, acceleration, 0.0);
}

/**
 * @brief Maximum velocity at a corner given angle and acceleration
 *
 * Based on centripetal acceleration: a_c = v² / r
 * And corner geometry to estimate effective radius
 *
 * @param maxAcceleration Maximum allowed centripetal acceleration
 * @param cornerAngle Angle between segments (radians, 0 = straight)
 * @param blendRadius Blend radius if using path blending
 * @return Maximum corner velocity
 */
inline double maxCornerVelocity(
    double maxAcceleration,
    double cornerAngle,
    double blendRadius = 0.0
) {
    if (std::fabs(cornerAngle) < 1e-6) {
        return std::numeric_limits<double>::infinity();  // Straight line
    }

    // Effective radius based on corner geometry
    double effectiveRadius;
    if (blendRadius > 0.0) {
        effectiveRadius = blendRadius;
    } else {
        // Estimate based on minimum turning radius at corner
        effectiveRadius = 1.0 / (2.0 * std::sin(std::fabs(cornerAngle) / 2.0));
    }

    return safeSqrt(maxAcceleration * effectiveRadius);
}

/**
 * @brief Angle between two direction vectors
 *
 * @param ux1, uy1, uz1 First unit vector
 * @param ux2, uy2, uz2 Second unit vector
 * @return Angle in radians [0, π]
 */
inline double angleBetweenVectors(
    double ux1, double uy1, double uz1,
    double ux2, double uy2, double uz2
) {
    const double dot = ux1 * ux2 + uy1 * uy2 + uz1 * uz2;
    return safeAcos(dot);
}

} // namespace Math
} // namespace GCode
