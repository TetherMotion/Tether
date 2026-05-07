/**
 * @file MultiAxisPath.hpp
 * @brief Multi-axis synchronized motion path interpolation
 * 
 * @details
 * Provides coordinated multi-axis motion including:
 * - Linear interpolation (G-code G01)
 * - Circular interpolation (G-code G02/G03)
 * - Helical interpolation (3D spiral)
 * - B-spline interpolation
 * - NURBS curves
 * - Bezier curves
 * 
 * ## Coordinate Systems
 * 
 * All paths operate in a normalized parameter space [0, 1].
 * The parameter u increases monotonically along the path.
 * Time parameterization is handled by the motion profile.
 * 
 * ## Architecture
 * 
 * ```
 *  Path Definition        Time Parameterization      Axis Interpolation
 *  ┌─────────────┐       ┌─────────────────┐        ┌────────────────┐
 *  │ PathSegment │──────►│ MotionProfile   │───────►│ PathSampler    │
 *  │ (geometry)  │       │ (time mapping)  │        │ (axis setpts)  │
 *  └─────────────┘       └─────────────────┘        └────────────────┘
 * ```
 * 
 * ## Usage Example
 * 
 * ```cpp
 * // Create circular arc in XY plane
 * CircularPath arc;
 * arc.configure(CircularConfig{
 *     .center = {100, 100, 0},
 *     .start = {0, 100, 0},
 *     .end = {100, 0, 0},
 *     .plane = Plane::XY,
 *     .direction = ArcDirection::CW
 * });
 * 
 * // Sample along path
 * for (double u = 0; u <= 1.0; u += 0.01) {
 *     PathPoint pt = arc.evaluate(u);
 *     // pt.position contains {x, y, z}
 * }
 * ```
 */

#pragma once

#include "CiA402Config.hpp"
#include "MotionProfile.hpp"
#include <array>
#include <vector>
#include <memory>
#include <cmath>
#include <algorithm>

namespace CiA402 {

/**
 * @brief Maximum number of axes for multi-axis paths
 */
constexpr size_t MAX_PATH_AXES = CIA402_MAX_GROUP_AXES;

/**
 * @brief Point in multi-axis space
 */
struct PathPoint {
    std::array<double, MAX_PATH_AXES> position{};   ///< Axis positions
    std::array<double, MAX_PATH_AXES> velocity{};   ///< Axis velocities
    std::array<double, MAX_PATH_AXES> acceleration{};///< Axis accelerations
    double parameter{0.0};                           ///< Path parameter [0,1]
    double pathVelocity{0.0};                        ///< Tangential velocity
    double curvature{0.0};                           ///< Path curvature
    size_t numAxes{0};                               ///< Number of active axes
};

/**
 * @brief Arc direction for circular interpolation
 */
enum class ArcDirection {
    CW,     ///< Clockwise
    CCW     ///< Counter-clockwise
};

/**
 * @brief Plane for 2D circular interpolation
 */
enum class Plane {
    XY,     ///< X-Y plane (Z normal)
    XZ,     ///< X-Z plane (Y normal)
    YZ      ///< Y-Z plane (X normal)
};

/**
 * @brief Path segment type
 */
enum class PathType {
    Linear,
    Circular,
    Helical,
    BSpline,
    NURBS,
    Bezier,
    Polynomial
};

// ============================================================================
// Abstract Path Segment
// ============================================================================

/**
 * @brief Abstract base class for path segments
 */
class PathSegment {
public:
    virtual ~PathSegment() = default;
    
    /**
     * @brief Get path type
     */
    virtual PathType getType() const = 0;
    
    /**
     * @brief Evaluate path at parameter u
     * 
     * @param u Parameter in [0, 1]
     * @return Point on path
     */
    virtual PathPoint evaluate(double u) const = 0;
    
    /**
     * @brief Get path length
     */
    virtual double getLength() const = 0;
    
    /**
     * @brief Get number of axes
     */
    virtual size_t getNumAxes() const = 0;
    
    /**
     * @brief Get start point
     */
    virtual PathPoint getStartPoint() const { return evaluate(0.0); }
    
    /**
     * @brief Get end point
     */
    virtual PathPoint getEndPoint() const { return evaluate(1.0); }
    
    /**
     * @brief Sample path at regular intervals
     * 
     * @param numSamples Number of samples
     * @return Vector of sampled points
     */
    std::vector<PathPoint> sample(size_t numSamples) const;
    
    /**
     * @brief Calculate arc length from start to parameter u
     * 
     * @param u Parameter in [0, 1]
     * @param numSteps Integration steps
     */
    double arcLength(double u, size_t numSteps = 100) const;
    
    /**
     * @brief Find parameter u for given arc length
     * 
     * @param s Arc length from start
     * @param tolerance Search tolerance
     */
    double parameterAtLength(double s, double tolerance = 1e-6) const;
};

// ============================================================================
// Linear Interpolation
// ============================================================================

/**
 * @brief Linear path configuration
 */
struct LinearConfig {
    std::array<double, MAX_PATH_AXES> start{};
    std::array<double, MAX_PATH_AXES> end{};
    size_t numAxes{2};
};

/**
 * @brief Linear interpolation between two points
 */
class LinearPath : public PathSegment {
public:
    LinearPath() = default;
    explicit LinearPath(const LinearConfig& config);
    
    /**
     * @brief Configure linear path
     */
    void configure(const LinearConfig& config);
    
    PathType getType() const override { return PathType::Linear; }
    PathPoint evaluate(double u) const override;
    double getLength() const override;
    size_t getNumAxes() const override { return m_config.numAxes; }
    
private:
    LinearConfig m_config;
    double m_length{0.0};
};

// ============================================================================
// Circular Interpolation
// ============================================================================

/**
 * @brief Circular path configuration
 */
struct CircularConfig {
    std::array<double, 3> center{};     ///< Arc center point
    std::array<double, 3> start{};      ///< Start point
    std::array<double, 3> end{};        ///< End point (or angle)
    double radius{0.0};                  ///< Radius (calculated if 0)
    double startAngle{0.0};              ///< Start angle [rad]
    double endAngle{0.0};                ///< End angle [rad]
    Plane plane{Plane::XY};              ///< Interpolation plane
    ArcDirection direction{ArcDirection::CCW};  ///< Arc direction
    bool useAngles{false};               ///< Use angles instead of endpoints
};

/**
 * @brief Circular arc interpolation
 */
class CircularPath : public PathSegment {
public:
    CircularPath() = default;
    explicit CircularPath(const CircularConfig& config);
    
    /**
     * @brief Configure circular path
     */
    void configure(const CircularConfig& config);
    
    /**
     * @brief Configure from center and endpoints
     */
    void configureFromPoints(const std::array<double, 3>& center,
                            const std::array<double, 3>& start,
                            const std::array<double, 3>& end,
                            ArcDirection dir = ArcDirection::CCW,
                            Plane plane = Plane::XY);
    
    /**
     * @brief Configure from radius and endpoints (two solutions)
     * 
     * @param start Start point
     * @param end End point
     * @param radius Arc radius
     * @param largeArc Use larger arc (> 180°)
     * @param dir Arc direction
     * @param plane Interpolation plane
     */
    void configureFromRadius(const std::array<double, 3>& start,
                            const std::array<double, 3>& end,
                            double radius,
                            bool largeArc = false,
                            ArcDirection dir = ArcDirection::CCW,
                            Plane plane = Plane::XY);
    
    PathType getType() const override { return PathType::Circular; }
    PathPoint evaluate(double u) const override;
    double getLength() const override;
    size_t getNumAxes() const override { return 3; }
    
    /**
     * @brief Get arc angle in radians
     */
    double getArcAngle() const { return std::abs(m_endAngle - m_startAngle); }
    
    /**
     * @brief Get radius
     */
    double getRadius() const { return m_radius; }
    
private:
    void calculateAngles();
    
    CircularConfig m_config;
    double m_radius{0.0};
    double m_startAngle{0.0};
    double m_endAngle{0.0};
    int m_axisU{0};  // First in-plane axis
    int m_axisV{1};  // Second in-plane axis
    int m_axisN{2};  // Normal axis
};

// ============================================================================
// Helical Interpolation
// ============================================================================

/**
 * @brief Helical path configuration
 */
struct HelicalConfig {
    std::array<double, 3> center{};     ///< Helix center (in plane)
    double radius{100.0};                ///< Helix radius
    double pitch{10.0};                  ///< Height per revolution
    double startAngle{0.0};              ///< Start angle [rad]
    double totalAngle{2 * M_PI};         ///< Total rotation [rad]
    Plane plane{Plane::XY};              ///< Base plane
    ArcDirection direction{ArcDirection::CCW};
};

/**
 * @brief Helical (3D spiral) interpolation
 */
class HelicalPath : public PathSegment {
public:
    HelicalPath() = default;
    explicit HelicalPath(const HelicalConfig& config);
    
    void configure(const HelicalConfig& config);
    
    PathType getType() const override { return PathType::Helical; }
    PathPoint evaluate(double u) const override;
    double getLength() const override;
    size_t getNumAxes() const override { return 3; }
    
private:
    HelicalConfig m_config;
    int m_axisU{0};
    int m_axisV{1};
    int m_axisN{2};
};

// ============================================================================
// B-Spline Interpolation
// ============================================================================

/**
 * @brief B-Spline configuration
 */
struct BSplineConfig {
    std::vector<std::array<double, MAX_PATH_AXES>> controlPoints;
    std::vector<double> knots;           ///< Knot vector (empty = uniform)
    int degree{3};                       ///< Spline degree (default cubic)
    size_t numAxes{3};
};

/**
 * @brief B-Spline curve interpolation
 */
class BSplinePath : public PathSegment {
public:
    BSplinePath() = default;
    explicit BSplinePath(const BSplineConfig& config);
    
    void configure(const BSplineConfig& config);
    
    /**
     * @brief Add control point
     */
    void addControlPoint(const std::array<double, MAX_PATH_AXES>& point);
    
    /**
     * @brief Clear control points
     */
    void clearControlPoints();
    
    PathType getType() const override { return PathType::BSpline; }
    PathPoint evaluate(double u) const override;
    double getLength() const override;
    size_t getNumAxes() const override { return m_config.numAxes; }
    
    /**
     * @brief Get spline degree
     */
    int getDegree() const { return m_config.degree; }
    
private:
    /**
     * @brief Calculate basis function
     */
    double basis(int i, int p, double u) const;
    
    /**
     * @brief Calculate basis function derivative
     */
    double basisDerivative(int i, int p, double u) const;
    
    /**
     * @brief Generate uniform knot vector
     */
    void generateUniformKnots();
    
    BSplineConfig m_config;
    mutable double m_cachedLength{-1.0};
};

// ============================================================================
// NURBS Interpolation
// ============================================================================

/**
 * @brief NURBS configuration
 */
struct NURBSConfig {
    std::vector<std::array<double, MAX_PATH_AXES>> controlPoints;
    std::vector<double> weights;         ///< Control point weights
    std::vector<double> knots;           ///< Knot vector
    int degree{3};                       ///< Curve degree
    size_t numAxes{3};
};

/**
 * @brief NURBS (Non-Uniform Rational B-Spline) curve
 */
class NURBSPath : public PathSegment {
public:
    NURBSPath() = default;
    explicit NURBSPath(const NURBSConfig& config);
    
    void configure(const NURBSConfig& config);
    
    PathType getType() const override { return PathType::NURBS; }
    PathPoint evaluate(double u) const override;
    double getLength() const override;
    size_t getNumAxes() const override { return m_config.numAxes; }
    
private:
    double rationalBasis(int i, double u) const;
    double bSplineBasis(int i, int p, double u) const;
    
    NURBSConfig m_config;
    mutable double m_cachedLength{-1.0};
};

// ============================================================================
// Bezier Curve
// ============================================================================

/**
 * @brief Bezier curve configuration
 */
struct BezierConfig {
    std::vector<std::array<double, MAX_PATH_AXES>> controlPoints;
    size_t numAxes{3};
};

/**
 * @brief Bezier curve interpolation
 * 
 * Supports any degree based on number of control points.
 * Degree = numControlPoints - 1
 */
class BezierPath : public PathSegment {
public:
    BezierPath() = default;
    explicit BezierPath(const BezierConfig& config);
    
    void configure(const BezierConfig& config);
    
    /**
     * @brief Configure cubic Bezier from 4 points
     */
    void configureCubic(const std::array<double, MAX_PATH_AXES>& p0,
                       const std::array<double, MAX_PATH_AXES>& p1,
                       const std::array<double, MAX_PATH_AXES>& p2,
                       const std::array<double, MAX_PATH_AXES>& p3,
                       size_t numAxes = 3);
    
    PathType getType() const override { return PathType::Bezier; }
    PathPoint evaluate(double u) const override;
    double getLength() const override;
    size_t getNumAxes() const override { return m_config.numAxes; }
    
    /**
     * @brief Get Bezier degree
     */
    int getDegree() const { 
        return static_cast<int>(m_config.controlPoints.size()) - 1; 
    }
    
private:
    /**
     * @brief Calculate binomial coefficient
     */
    static int binomial(int n, int k);
    
    /**
     * @brief Evaluate Bernstein polynomial
     */
    static double bernstein(int n, int i, double t);
    
    BezierConfig m_config;
    mutable double m_cachedLength{-1.0};
};

// ============================================================================
// Path Sampler
// ============================================================================

/**
 * @brief Samples path segments with time parameterization
 */
class PathSampler {
public:
    PathSampler() = default;
    
    /**
     * @brief Convenience constructor
     * 
     * @param path Path to sample (unique_ptr, ownership transferred)
     * @param feedrate Desired feedrate
     */
    PathSampler(std::unique_ptr<PathSegment> path, double feedrate);
    
    /**
     * @brief Configure sampler
     * 
     * @param path Path segment to sample
     * @param profile Motion profile for time parameterization
     */
    void configure(std::shared_ptr<PathSegment> path,
                  std::shared_ptr<MotionProfile> profile);
    
    /**
     * @brief Plan motion along path
     * 
     * @param feedrate Desired feedrate (path velocity)
     */
    void plan(double feedrate);
    
    /**
     * @brief Sample at given time
     * 
     * @param time Time from start [s]
     * @return Point on path with velocities
     */
    PathPoint sample(double time) const;
    
    /**
     * @brief Sample at given time (alias)
     */
    PathPoint sampleAtTime(double time) const { return sample(time); }
    
    /**
     * @brief Check if motion is complete
     */
    bool isComplete(double time) const;
    
    /**
     * @brief Get total motion time
     */
    double getDuration() const;
    
    /**
     * @brief Get current parameter
     */
    double getParameter(double time) const;
    
private:
    std::shared_ptr<PathSegment> m_path;
    std::shared_ptr<MotionProfile> m_profile;
    std::unique_ptr<PathSegment> m_ownedPath;  ///< Owned path for convenience constructor
    double m_pathLength{0.0};
    double m_feedrate{0.0};
    double m_duration{0.0};
};

// ============================================================================
// Multi-Segment Path
// ============================================================================

/**
 * @brief Blending mode between segments
 */
enum class BlendMode {
    None,           ///< Full stop between segments
    Corner,         ///< Sharp corner (velocity limited)
    Arc,            ///< Arc blending
    Spline          ///< Spline blending
};

/**
 * @brief Multi-segment path with optional blending
 * 
 * Inherits from PathSegment so it can be used with PathSampler
 */
class MultiSegmentPath : public PathSegment {
public:
    /**
     * @brief Add segment to path
     */
    void addSegment(std::shared_ptr<PathSegment> segment);
    
    // PathSegment interface implementation
    PathType getType() const override { return PathType::Linear; }  // Multi-segment behaves like composite
    PathPoint evaluate(double u) const override;
    double getLength() const override { return getTotalLength(); }
    size_t getNumAxes() const override;
    
    /**
     * @brief Set blend mode for all transitions
     */
    void setBlendMode(BlendMode mode) { m_blendMode = mode; }
    
    /**
     * @brief Set blend radius/tolerance
     */
    void setBlendTolerance(double tolerance) { m_blendTolerance = tolerance; }
    
    /**
     * @brief Plan complete path
     * 
     * @param feedrate Default feedrate
     * @param limits Motion limits
     */
    void plan(double feedrate, const MotionLimits& limits);
    
    /**
     * @brief Sample at given time
     */
    PathPoint sample(double time) const;
    
    /**
     * @brief Get total duration
     */
    double getDuration() const { return m_totalDuration; }
    
    /**
     * @brief Get total path length
     */
    double getTotalLength() const;
    
    /**
     * @brief Get number of segments
     */
    size_t getSegmentCount() const { return m_segments.size(); }
    
    /**
     * @brief Clear all segments
     */
    void clear();
    
private:
    struct SegmentInfo {
        std::shared_ptr<PathSegment> segment;
        double startTime{0.0};
        double endTime{0.0};
        double startParam{0.0};
        double endParam{1.0};
        double feedrate{0.0};
    };
    
    std::vector<SegmentInfo> m_segments;
    BlendMode m_blendMode{BlendMode::None};
    double m_blendTolerance{1.0};
    double m_totalDuration{0.0};
};

// ============================================================================
// Factory Functions
// ============================================================================

/**
 * @brief Create path segment by type
 */
std::unique_ptr<PathSegment> createPathSegment(PathType type);

} // namespace CiA402
