/**
 * @file GCodeMath.hpp
 * @brief Centralized Math Functions for GCode Processing
 * 
 * Provides orthogonal projection, plane selection, Z filtering, and other
 * math utilities for GCode visualization and processing.
 */

#pragma once

#include <cmath>
#include <array>
#include <algorithm>
#include <limits>
#include <vector>

namespace GCode {
namespace Math {

// ============================================================================
// Constants
// ============================================================================

constexpr double PI = 3.14159265358979323846;
constexpr double TWO_PI = 2.0 * PI;
constexpr double EPSILON = 1e-9;

// ============================================================================
// Basic Math Functions
// ============================================================================

/**
 * @brief Clamp a value between min and max
 */
template<typename T>
constexpr T clamp(T value, T min_val, T max_val) {
    return std::max(min_val, std::min(value, max_val));
}

/**
 * @brief Linear interpolation between two values
 */
template<typename T>
constexpr T lerp(T a, T b, double t) {
    return static_cast<T>(a + t * (b - a));
}

/**
 * @brief Normalize an angle to [-PI, PI]
 */
inline double normalizeAngle(double angle) {
    while (angle > PI) angle -= TWO_PI;
    while (angle < -PI) angle += TWO_PI;
    return angle;
}

/**
 * @brief Convert degrees to radians
 */
constexpr double toRadians(double degrees) {
    return degrees * PI / 180.0;
}

/**
 * @brief Convert radians to degrees
 */
constexpr double toDegrees(double radians) {
    return radians * 180.0 / PI;
}

// ============================================================================
// 3D Vector Operations
// ============================================================================

struct Vec3 {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    
    constexpr Vec3() = default;
    constexpr Vec3(double x_, double y_, double z_) : x(x_), y(y_), z(z_) {}
    
    constexpr Vec3 operator+(const Vec3& other) const {
        return {x + other.x, y + other.y, z + other.z};
    }
    
    constexpr Vec3 operator-(const Vec3& other) const {
        return {x - other.x, y - other.y, z - other.z};
    }
    
    constexpr Vec3 operator*(double scalar) const {
        return {x * scalar, y * scalar, z * scalar};
    }
    
    constexpr Vec3 operator/(double scalar) const {
        return {x / scalar, y / scalar, z / scalar};
    }
    
    double length() const {
        return std::sqrt(x * x + y * y + z * z);
    }
    
    double lengthSquared() const {
        return x * x + y * y + z * z;
    }
    
    Vec3 normalized() const {
        double len = length();
        if (len < EPSILON) return {0, 0, 0};
        return *this / len;
    }
    
    constexpr double dot(const Vec3& other) const {
        return x * other.x + y * other.y + z * other.z;
    }
    
    constexpr Vec3 cross(const Vec3& other) const {
        return {
            y * other.z - z * other.y,
            z * other.x - x * other.z,
            x * other.y - y * other.x
        };
    }
};

// ============================================================================
// 2D Vector Operations
// ============================================================================

struct Vec2 {
    double x = 0.0;
    double y = 0.0;
    
    constexpr Vec2() = default;
    constexpr Vec2(double x_, double y_) : x(x_), y(y_) {}
    
    constexpr Vec2 operator+(const Vec2& other) const {
        return {x + other.x, y + other.y};
    }
    
    constexpr Vec2 operator-(const Vec2& other) const {
        return {x - other.x, y - other.y};
    }
    
    constexpr Vec2 operator*(double scalar) const {
        return {x * scalar, y * scalar};
    }
    
    double length() const {
        return std::sqrt(x * x + y * y);
    }
    
    Vec2 normalized() const {
        double len = length();
        if (len < EPSILON) return {0, 0};
        return {x / len, y / len};
    }
    
    constexpr double dot(const Vec2& other) const {
        return x * other.x + y * other.y;
    }
    
    // Perpendicular vector (90 degrees CCW)
    constexpr Vec2 perp() const {
        return {-y, x};
    }
};

// ============================================================================
// Projection Planes
// ============================================================================

/**
 * @brief Projection plane enumeration
 */
enum class ProjectionPlane {
    XY,  ///< Top view (looking down Z axis)
    XZ,  ///< Front view (looking down Y axis)
    YZ,  ///< Side view (looking down X axis)
};

/**
 * @brief Project a 3D point onto a 2D plane
 * 
 * @param point The 3D point to project
 * @param plane The projection plane
 * @return The projected 2D point
 */
inline Vec2 projectToPlane(const Vec3& point, ProjectionPlane plane) {
    switch (plane) {
        case ProjectionPlane::XY:
            return {point.x, point.y};
        case ProjectionPlane::XZ:
            return {point.x, point.z};
        case ProjectionPlane::YZ:
            return {point.y, point.z};
        default:
            return {point.x, point.y};
    }
}

/**
 * @brief Get the depth axis value for a projection plane
 * 
 * @param point The 3D point
 * @param plane The projection plane
 * @return The depth value (coordinate perpendicular to the projection plane)
 */
inline double getDepthForPlane(const Vec3& point, ProjectionPlane plane) {
    switch (plane) {
        case ProjectionPlane::XY:
            return point.z;
        case ProjectionPlane::XZ:
            return point.y;
        case ProjectionPlane::YZ:
            return point.x;
        default:
            return point.z;
    }
}

/**
 * @brief Unproject a 2D point back to 3D with a specified depth
 * 
 * @param point2D The 2D point
 * @param depth The depth value
 * @param plane The projection plane
 * @return The 3D point
 */
inline Vec3 unprojectFromPlane(const Vec2& point2D, double depth, ProjectionPlane plane) {
    switch (plane) {
        case ProjectionPlane::XY:
            return {point2D.x, point2D.y, depth};
        case ProjectionPlane::XZ:
            return {point2D.x, depth, point2D.y};
        case ProjectionPlane::YZ:
            return {depth, point2D.x, point2D.y};
        default:
            return {point2D.x, point2D.y, depth};
    }
}

// ============================================================================
// Z Filtering
// ============================================================================

/**
 * @brief Z filter mode
 */
enum class ZFilterMode {
    None,       ///< No filtering
    Above,      ///< Show only points above Z threshold
    Below,      ///< Show only points below Z threshold
    Between,    ///< Show only points between Z min and Z max
    Layer,      ///< Show only points within a layer (Z +/- tolerance)
};

/**
 * @brief Z filter configuration
 */
struct ZFilter {
    ZFilterMode mode = ZFilterMode::None;
    double threshold = 0.0;  ///< For Above/Below modes
    double zMin = 0.0;       ///< For Between mode
    double zMax = 0.0;       ///< For Between mode
    double layerZ = 0.0;     ///< For Layer mode
    double tolerance = 0.5;  ///< For Layer mode (layer height / 2)
    
    /**
     * @brief Check if a Z value passes the filter
     */
    bool passes(double z) const {
        switch (mode) {
            case ZFilterMode::None:
                return true;
            case ZFilterMode::Above:
                return z >= threshold;
            case ZFilterMode::Below:
                return z <= threshold;
            case ZFilterMode::Between:
                return z >= zMin && z <= zMax;
            case ZFilterMode::Layer:
                return std::abs(z - layerZ) <= tolerance;
            default:
                return true;
        }
    }
};

// ============================================================================
// Bounding Box
// ============================================================================

/**
 * @brief 3D Axis-Aligned Bounding Box
 */
struct BoundingBox {
    Vec3 min{std::numeric_limits<double>::max(),
             std::numeric_limits<double>::max(),
             std::numeric_limits<double>::max()};
    Vec3 max{std::numeric_limits<double>::lowest(),
             std::numeric_limits<double>::lowest(),
             std::numeric_limits<double>::lowest()};
    
    void expand(const Vec3& point) {
        min.x = std::min(min.x, point.x);
        min.y = std::min(min.y, point.y);
        min.z = std::min(min.z, point.z);
        max.x = std::max(max.x, point.x);
        max.y = std::max(max.y, point.y);
        max.z = std::max(max.z, point.z);
    }
    
    Vec3 center() const {
        return (min + max) * 0.5;
    }
    
    Vec3 size() const {
        return max - min;
    }
    
    bool isValid() const {
        return min.x <= max.x && min.y <= max.y && min.z <= max.z;
    }
};

// ============================================================================
// Arc Geometry
// ============================================================================

/**
 * @brief Calculate arc sweep angle from center
 * 
 * @param startU Start U coordinate in plane
 * @param startV Start V coordinate in plane
 * @param endU End U coordinate in plane
 * @param endV End V coordinate in plane
 * @param centerU Center U coordinate in plane
 * @param centerV Center V coordinate in plane
 * @param clockwise True for CW arc, false for CCW
 * @return Sweep angle in radians (negative for CW)
 */
inline double arcSweepFromCenter(
    double startU, double startV,
    double endU, double endV,
    double centerU, double centerV,
    bool clockwise
) {
    double startAngle = std::atan2(startV - centerV, startU - centerU);
    double endAngle = std::atan2(endV - centerV, endU - centerU);
    double sweep = endAngle - startAngle;
    
    if (clockwise) {
        while (sweep > 0) sweep -= TWO_PI;
        if (sweep == 0) sweep = -TWO_PI;
    } else {
        while (sweep < 0) sweep += TWO_PI;
        if (sweep == 0) sweep = TWO_PI;
    }
    return sweep;
}

/**
 * @brief Interpolate a point on an arc
 * 
 * @param centerU Center U coordinate
 * @param centerV Center V coordinate
 * @param radius Arc radius
 * @param startAngle Starting angle in radians
 * @param sweep Sweep angle in radians
 * @param t Parameter [0, 1]
 * @return Interpolated point {u, v}
 */
inline Vec2 interpolateArc(
    double centerU, double centerV,
    double radius,
    double startAngle, double sweep,
    double t
) {
    double angle = startAngle + t * sweep;
    return {
        centerU + radius * std::cos(angle),
        centerV + radius * std::sin(angle)
    };
}

/**
 * @brief Calculate the perpendicular distance from a point to a line segment
 */
inline double pointToSegmentDistance(const Vec2& point, const Vec2& segStart, const Vec2& segEnd) {
    Vec2 seg = segEnd - segStart;
    double len2 = seg.x * seg.x + seg.y * seg.y;
    
    if (len2 < EPSILON * EPSILON) {
        // Segment is effectively a point
        return (point - segStart).length();
    }
    
    // Project point onto line
    double t = clamp(((point - segStart).dot(seg)) / len2, 0.0, 1.0);
    Vec2 proj = segStart + seg * t;
    return (point - proj).length();
}

// ============================================================================
// Color Mapping
// ============================================================================

/**
 * @brief Color in RGBA format [0-1]
 */
struct Color {
    float r = 1.0f;
    float g = 1.0f;
    float b = 1.0f;
    float a = 1.0f;
    
    constexpr Color() = default;
    constexpr Color(float r_, float g_, float b_, float a_ = 1.0f)
        : r(r_), g(g_), b(b_), a(a_) {}
    
    /**
     * @brief Linear interpolation between two colors
     */
    static Color lerp(const Color& a, const Color& b, float t) {
        return {
            a.r + t * (b.r - a.r),
            a.g + t * (b.g - a.g),
            a.b + t * (b.b - a.b),
            a.a + t * (b.a - a.a)
        };
    }
    
    /**
     * @brief Create color from HSV
     */
    static Color fromHSV(float h, float s, float v, float a = 1.0f);
};

inline Color Color::fromHSV(float h, float s, float v, float a) {
    // h is [0, 360), s and v are [0, 1]
    float c = v * s;
    float x = c * (1.0f - std::abs(std::fmod(h / 60.0f, 2.0f) - 1.0f));
    float m = v - c;
    
    float r, g, b;
    if (h < 60) { r = c; g = x; b = 0; }
    else if (h < 120) { r = x; g = c; b = 0; }
    else if (h < 180) { r = 0; g = c; b = x; }
    else if (h < 240) { r = 0; g = x; b = c; }
    else if (h < 300) { r = x; g = 0; b = c; }
    else { r = c; g = 0; b = x; }
    
    return {r + m, g + m, b + m, a};
}

/**
 * @brief Color mapping mode
 */
enum class ColorMapMode {
    Solid,          ///< Single solid color
    BySpeed,        ///< Color by feed rate
    ByZHeight,      ///< Color by Z coordinate
    ByTime,         ///< Color by time in program
    ByAcceleration, ///< Color by acceleration
    ByAccuracy,     ///< Color by deviation from desired path
    ByMoveType,     ///< Different colors for rapids, feeds, arcs
};

/**
 * @brief Color map configuration
 */
struct ColorMap {
    ColorMapMode mode = ColorMapMode::Solid;
    Color solidColor{0.2f, 0.8f, 0.2f, 1.0f};
    Color lowColor{0.0f, 0.0f, 1.0f, 1.0f};    // Blue
    Color highColor{1.0f, 0.0f, 0.0f, 1.0f};   // Red
    Color rapidColor{1.0f, 0.2f, 0.2f, 0.5f};
    Color feedColor{0.2f, 0.8f, 0.2f, 1.0f};
    Color arcColor{0.2f, 0.2f, 1.0f, 1.0f};
    double minValue = 0.0;
    double maxValue = 100.0;
    
    /**
     * @brief Map a value to a color
     */
    Color mapValue(double value) const {
        if (mode == ColorMapMode::Solid) {
            return solidColor;
        }
        
        double t = clamp((value - minValue) / (maxValue - minValue + EPSILON), 0.0, 1.0);
        return Color::lerp(lowColor, highColor, static_cast<float>(t));
    }
};

// ============================================================================
// Motion Timing
// ============================================================================

/**
 * @brief Calculate time for a linear move given distance and velocity
 */
inline double timeForMove(double distance, double velocity) {
    if (velocity <= EPSILON) return 0.0;
    return distance / velocity;
}

/**
 * @brief Calculate arc length
 */
inline double arcLength(double radius, double sweepRadians) {
    return std::abs(radius * sweepRadians);
}

// ============================================================================
// Trajectory Point for Precomputation
// ============================================================================

/**
 * @brief Precomputed trajectory point with full motion data
 */
struct TrajectoryPoint {
    Vec3 position;
    Vec3 velocity;
    Vec3 acceleration;
    double time = 0.0;
    double feedRate = 0.0;
    double distanceFromStart = 0.0;
    int blockIndex = -1;
    int segmentIndex = -1;
    bool isRapid = false;
    bool isArc = false;
    
    // For coloring
    double speed() const {
        return velocity.length();
    }
    
    double accelMagnitude() const {
        return acceleration.length();
    }
};

} // namespace Math
} // namespace GCode
