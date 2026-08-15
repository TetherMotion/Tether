/**
 * @file InterpolationStrategy.hpp
 * @brief Interpolation Strategy Framework for G-Code Motion Generation
 *
 * @details
 * This file defines the abstract interface and common types for trajectory
 * interpolation strategies. Multiple numerical methods are supported:
 *
 * - **FixedTime**: Uniform time stepping
 * - **FixedDeviation**: Adaptive based on chord error
 * - **RKF45**: Runge-Kutta-Fehlberg 4(5) adaptive
 * - **DOPRI**: Dormand-Prince 5(4) adaptive
 * - **AdaptiveMidpoint**: Binary subdivision
 * - **DeCasteljau**: Bézier curve subdivision
 *
 * All strategies support:
 * - Lookahead/lookbehind velocity planning
 * - G61/G64 path mode handling
 * - Velocity/acceleration/jerk limits
 * - Inside/outside contour offset
 *
 * @see InterpolationStrategies.md for detailed mathematical documentation
 */

#pragma once

#include "../GCodeTypes.hpp"
#include <cmath>
#include <vector>
#include <memory>
#include <functional>
#include <array>
#include <limits>
#include <algorithm>
#include <string>

namespace GCode {

// ============================================================================
// Forward Declarations
// ============================================================================

class InterpolationStrategy;
class TrajectoryPlanner;
struct PlanningSegment;
struct TrajectoryPoint;

// ============================================================================
// Constants
// ============================================================================

/// Mathematical constants
namespace InterpolationConstants {
    constexpr double PI = 3.14159265358979323846;
    constexpr double TWO_PI = 2.0 * PI;
    constexpr double EPSILON = 1e-12;
    constexpr double SQRT2 = 1.41421356237309504880;
    constexpr double INV_SQRT2 = 0.70710678118654752440;
}

// ============================================================================
// Strategy Type Enumeration
// ============================================================================

/**
 * @brief Available interpolation strategies
 */
enum class InterpolationStrategyType : uint8_t {
    FixedTime = 0,          ///< Fixed time step interpolation
    FixedDeviation = 1,     ///< Adaptive chord deviation
    RKF45 = 2,              ///< Runge-Kutta-Fehlberg 4(5)
    DOPRI = 3,              ///< Dormand-Prince 5(4)
    AdaptiveMidpoint = 4,   ///< Recursive midpoint subdivision
    DeCasteljau = 5,        ///< Bézier de Casteljau algorithm
};

/**
 * @brief Path control mode (G61/G61.1/G64)
 */
enum class PathControlMode : uint8_t {
    ExactStop = 0,          ///< G61 - Full stop at each endpoint
    ExactPath = 1,          ///< G61.1 - Exact path, may not stop
    Blending = 2,           ///< G64 - Path blending/corner rounding
};

/**
 * @brief Contour offset mode
 */
enum class ContourOffsetMode : uint8_t {
    None = 0,               ///< No offset (on path)
    Left = 1,               ///< Offset left of path direction
    Right = 2,              ///< Offset right of path direction
};

/**
 * @brief Motion type for segments
 */
enum class SegmentMotionType : uint8_t {
    Rapid = 0,              ///< G0 - Rapid positioning
    Linear = 1,             ///< G1 - Linear interpolation
    ArcCW = 2,              ///< G2 - Clockwise arc
    ArcCCW = 3,             ///< G3 - Counter-clockwise arc
    CubicSpline = 4,        ///< G5 - Cubic spline
    QuadraticSpline = 5,    ///< G5.1 - Quadratic spline
    NURBS = 6,              ///< G5.2/G5.3 - NURBS
    Dwell = 7,              ///< G4 - Dwell
};

/**
 * @brief Plane selection for arc interpolation
 */
enum class InterpolationPlane : uint8_t {
    XY = 0,                 ///< G17 - XY plane (Z perpendicular)
    XZ = 1,                 ///< G18 - XZ plane (Y perpendicular)
    YZ = 2,                 ///< G19 - YZ plane (X perpendicular)
};

// ============================================================================
// Configuration Structures
// ============================================================================

/**
 * @brief Kinematic limits for motion planning
 */
struct KinematicLimits {
    double maxVelocityLinear = 6000.0;      ///< Maximum linear velocity (mm/min)
    double maxVelocityAngular = 3600.0;     ///< Maximum angular velocity (deg/min)
    double maxAcceleration = 1000.0;        ///< Maximum acceleration (mm/s²)
    double maxDeceleration = 1000.0;        ///< Maximum deceleration (mm/s²)
    double maxJerk = 10000.0;               ///< Maximum jerk (mm/s³)
    double maxCentripetalAccel = 500.0;     ///< Maximum centripetal acceleration (mm/s²)

    /// Per-axis limits (X, Y, Z, A, B, C, U, V, W)
    std::array<double, MAX_AXES> axisMaxVelocity{};
    std::array<double, MAX_AXES> axisMaxAcceleration{};
    std::array<double, MAX_AXES> axisMaxJerk{};

    KinematicLimits() {
        axisMaxVelocity.fill(6000.0);
        axisMaxAcceleration.fill(1000.0);
        axisMaxJerk.fill(10000.0);
    }
};

/**
 * @brief Configuration for interpolation strategies
 */
struct InterpolationConfig {
    // Strategy selection
    InterpolationStrategyType strategy = InterpolationStrategyType::FixedDeviation;

    // === Common Parameters ===
    double timeResolution = 0.001;          ///< Time step for fixed-time (seconds)
    double maxChordDeviation = 0.01;        ///< Maximum chord error (mm)
    double velocityTolerance = 0.01;        ///< Velocity continuity tolerance (mm/s)
    double positionTolerance = 1e-6;        ///< Position tolerance for comparisons (mm)

    // === Adaptive Method Parameters ===
    double minStepSize = 1e-8;              ///< Minimum step size (parameter or time)
    double maxStepSize = 0.1;               ///< Maximum step size
    double errorTolerance = 1e-6;           ///< Local error tolerance
    double safetyFactor = 0.9;              ///< Step size safety factor (0.8-0.95)
    int maxIterations = 1000;               ///< Maximum iterations per segment
    int maxSubdivisionDepth = 20;           ///< Maximum recursion depth

    // === Kinematic Limits ===
    KinematicLimits limits;

    // === Path Control Mode (G61/G64) ===
    PathControlMode pathMode = PathControlMode::Blending;
    double blendTolerance = 0.05;           ///< G64 P value (mm)
    double naiveCAMTolerance = 0.0;         ///< G64 Q value (mm)

    // === Contour Offset ===
    ContourOffsetMode offsetMode = ContourOffsetMode::None;
    double offsetDistance = 0.0;            ///< Offset distance (mm, positive=left)

    // === Lookahead/Lookbehind ===
    size_t lookaheadDepth = 50;             ///< Number of segments to look ahead
    size_t lookbehindDepth = 20;            ///< Number of segments to look behind

    // === S-Curve Acceleration ===
    bool useSCurve = true;                  ///< Enable S-curve (jerk limited) profiles
    bool enableJerkLimit = true;            ///< Apply jerk limiting

    // === Arc Parameters ===
    double arcRadiusTolerance = 0.005;      ///< Arc endpoint radius match tolerance (mm)
    double minArcRadius = 0.001;            ///< Minimum arc radius (mm)
    double minArcSegmentAngle = 0.01;       ///< Minimum arc segment (radians)
};

// ============================================================================
// Trajectory Point
// ============================================================================

/**
 * @brief A point along the interpolated trajectory
 */
struct TrajectoryPoint {
    double time = 0.0;                      ///< Time from trajectory start (seconds)
    Position position;                       ///< Position at this time
    Position velocity;                       ///< Velocity (units/s)
    Position acceleration;                   ///< Acceleration (units/s²)
    Position jerk;                          ///< Jerk (units/s³)

    int32_t blockIndex = -1;                ///< Source G-code block index
    int32_t segmentIndex = -1;              ///< Segment index within trajectory
    SegmentMotionType motionType = SegmentMotionType::Linear;

    double parameter = 0.0;                 ///< Curve parameter (0-1) within segment
    double pathLength = 0.0;                ///< Arc length from segment start
    double curvature = 0.0;                 ///< Local curvature (1/radius)

    bool isInterpolated = false;            ///< True if interpolated (vs endpoint)
    bool isBlendPoint = false;              ///< True if part of corner blend
    bool atVelocityLimit = false;           ///< True if velocity-limited
    bool atAccelerationLimit = false;       ///< True if acceleration-limited
    bool atJerkLimit = false;               ///< True if jerk-limited

    /// Default constructor
    TrajectoryPoint() = default;

    /// Create from position
    explicit TrajectoryPoint(const Position& pos) : position(pos) {}

    /// Euclidean distance to another point
    double distanceTo(const TrajectoryPoint& other) const {
        return position.linearDistance(other.position);
    }
};

// ============================================================================
// Motion Segment
// ============================================================================

/**
 * @brief A motion segment between two points
 */
struct PlanningSegment {
    Position start;                         ///< Start position
    Position end;                           ///< End position
    Position center;                        ///< Arc center (for arcs)

    SegmentMotionType motionType = SegmentMotionType::Linear;
    InterpolationPlane plane = InterpolationPlane::XY;

    double feedRate = 1000.0;               ///< Feed rate (mm/min)
    double arcRadius = 0.0;                 ///< Arc radius (mm)
    double arcSweep = 0.0;                  ///< Arc sweep angle (radians)
    double segmentLength = 0.0;             ///< Total path length (mm)
    double segmentTime = 0.0;               ///< Segment duration (seconds)

    int32_t blockIndex = -1;                ///< Source G-code block
    bool isRapid = false;                   ///< True for G0 moves

    /// G64 path blending tolerance (P value, mm). 0 = no blending (G61).
    /// Used by the viewer to color by corner deviation percentage.
    double blendTolerance = 0.0;

    // Spline control points (for G5, NURBS)
    std::vector<Position> controlPoints;
    std::vector<double> weights;            ///< NURBS weights
    std::vector<double> knots;              ///< NURBS knot vector

    // Velocity planning results
    double entryVelocity = 0.0;             ///< Planned entry velocity (mm/s)
    double exitVelocity = 0.0;              ///< Planned exit velocity (mm/s)
    double maxVelocity = 0.0;               ///< Maximum velocity in segment

    /// Default constructor
    PlanningSegment() = default;

    /// Check if this is an arc segment
    bool isArc() const {
        return motionType == SegmentMotionType::ArcCW ||
               motionType == SegmentMotionType::ArcCCW;
    }

    /// Check if this is a spline segment
    bool isSpline() const {
        return motionType == SegmentMotionType::CubicSpline ||
               motionType == SegmentMotionType::QuadraticSpline ||
               motionType == SegmentMotionType::NURBS;
    }

    /// Get arc direction (1 for CCW, -1 for CW)
    int arcDirection() const {
        return (motionType == SegmentMotionType::ArcCCW) ? 1 : -1;
    }
};

// ============================================================================
// Interpolation Context
// ============================================================================

/**
 * @brief Context for interpolation containing segment buffer and state
 */
struct InterpolationContext {
    InterpolationConfig config;

    // Segment buffers for lookahead/lookbehind
    std::vector<PlanningSegment> segments;
    size_t currentSegmentIndex = 0;

    // Current interpolation state
    Position currentPosition;
    Position currentVelocity;
    double currentTime = 0.0;

    // Bounds tracking
    Position minBounds;
    Position maxBounds;

    InterpolationContext() {
        for (size_t i = 0; i < MAX_AXES; ++i) {
            minBounds[i] = std::numeric_limits<double>::max();
            maxBounds[i] = std::numeric_limits<double>::lowest();
        }
    }

    /// Update bounds with a position
    void updateBounds(const Position& p) {
        for (size_t i = 0; i < MAX_AXES; ++i) {
            minBounds[i] = std::min(minBounds[i], p[i]);
            maxBounds[i] = std::max(maxBounds[i], p[i]);
        }
    }

    /// Get segment at offset from current (negative = lookbehind, positive = lookahead)
    const PlanningSegment* getSegment(int offset) const {
        int idx = static_cast<int>(currentSegmentIndex) + offset;
        if (idx < 0 || idx >= static_cast<int>(segments.size())) {
            return nullptr;
        }
        return &segments[idx];
    }

    /// Check if we have lookahead available
    bool hasLookahead(size_t count) const {
        return (currentSegmentIndex + count) < segments.size();
    }

    /// Check if we have lookbehind available
    bool hasLookbehind(size_t count) const {
        return currentSegmentIndex >= count;
    }
};

// ============================================================================
// Interpolation Result
// ============================================================================

/**
 * @brief Result of an interpolation step
 */
struct InterpolationResult {
    bool success = false;
    std::string errorMessage;

    std::vector<TrajectoryPoint> points;
    double totalDuration = 0.0;
    double totalLength = 0.0;

    Position minBounds;
    Position maxBounds;

    // Statistics
    size_t totalIterations = 0;
    size_t rejectedSteps = 0;
    double minStepSize = std::numeric_limits<double>::max();
    double maxStepSize = 0.0;
    double avgStepSize = 0.0;
};

// ============================================================================
// Abstract Interpolation Strategy Interface
// ============================================================================

/**
 * @brief Abstract base class for interpolation strategies
 *
 * All interpolation strategies must implement this interface.
 */
class InterpolationStrategy {
public:
    virtual ~InterpolationStrategy() = default;

    /**
     * @brief Get the strategy type
     */
    virtual InterpolationStrategyType type() const = 0;

    /**
     * @brief Get human-readable strategy name
     */
    virtual const char* name() const = 0;

    /**
     * @brief Configure the strategy
     * @param config Configuration parameters
     */
    virtual void configure(const InterpolationConfig& config) = 0;

    /**
     * @brief Interpolate a single segment
     * @param segment Motion segment to interpolate
     * @param ctx Interpolation context (for lookahead/lookbehind)
     * @param[out] points Output trajectory points
     * @return Result with success/failure and statistics
     */
    virtual InterpolationResult interpolateSegment(
        const PlanningSegment& segment,
        InterpolationContext& ctx,
        std::vector<TrajectoryPoint>& points
    ) = 0;

    /**
     * @brief Interpolate all segments in context
     * @param ctx Interpolation context with segments
     * @param[out] points Output trajectory points
     * @return Result with success/failure and statistics
     */
    virtual InterpolationResult interpolateAll(
        InterpolationContext& ctx,
        std::vector<TrajectoryPoint>& points
    );

    /**
     * @brief Evaluate position at parameter t (0-1) on segment
     * @param segment Motion segment
     * @param t Parameter (0 = start, 1 = end)
     * @return Position at parameter t
     */
    virtual Position evaluatePosition(const PlanningSegment& segment, double t) const;

    /**
     * @brief Evaluate velocity at parameter t on segment
     * @param segment Motion segment
     * @param t Parameter
     * @return Velocity at parameter t
     */
    virtual Position evaluateVelocity(const PlanningSegment& segment, double t) const;

    /**
     * @brief Evaluate acceleration at parameter t on segment
     * @param segment Motion segment
     * @param t Parameter
     * @return Acceleration at parameter t
     */
    virtual Position evaluateAcceleration(const PlanningSegment& segment, double t) const;

    /**
     * @brief Calculate curvature at parameter t on segment
     * @param segment Motion segment
     * @param t Parameter
     * @return Curvature (1/radius, 0 for lines)
     */
    virtual double evaluateCurvature(const PlanningSegment& segment, double t) const;

    /**
     * @brief Calculate arc length from start to parameter t
     * @param segment Motion segment
     * @param t Parameter
     * @return Arc length
     */
    virtual double arcLength(const PlanningSegment& segment, double t) const;

    /**
     * @brief Find parameter t for given arc length s
     * @param segment Motion segment
     * @param s Arc length
     * @return Parameter t
     */
    virtual double arcLengthInverse(const PlanningSegment& segment, double s) const;

protected:
    InterpolationConfig config_;

public:
    /**
     * @brief Get plane axes for arc interpolation
     * @param plane Interpolation plane
     * @param u First axis index (output)
     * @param v Second axis index (output)
     * @param w Perpendicular axis index (output)
     */
    static void getPlaneAxes(InterpolationPlane plane, int& u, int& v, int& w);

    /**
     * @brief Clamp value to range
     */
    static double clamp(double v, double lo, double hi) {
        return std::max(lo, std::min(hi, v));
    }

    /**
     * @brief Calculate required arc segments for chord deviation
     * @param radius Arc radius
     * @param sweep Arc sweep angle
     * @param maxChordError Maximum chord deviation
     * @return Number of segments needed
     */
    static size_t arcSegmentCount(double radius, double sweep, double maxChordError);
};

// ============================================================================
// Strategy Factory
// ============================================================================

/**
 * @brief Factory for creating interpolation strategies
 */
class InterpolationStrategyFactory {
public:
    /**
     * @brief Create a strategy of the specified type
     * @param type Strategy type
     * @return Unique pointer to strategy instance
     */
    static std::unique_ptr<InterpolationStrategy> create(InterpolationStrategyType type);

    /**
     * @brief Create a strategy from configuration
     * @param config Configuration (strategy type is read from config)
     * @return Unique pointer to configured strategy instance
     */
    static std::unique_ptr<InterpolationStrategy> create(const InterpolationConfig& config);
};

// ============================================================================
// Velocity Planner (Lookahead Processing)
// ============================================================================

/**
 * @brief Velocity planner with lookahead/lookbehind
 *
 * Plans entry/exit velocities for segments considering:
 * - Kinematic limits (v, a, j)
 * - Path mode (G61/G64)
 * - Corner angles
 * - Curvature constraints
 */
class VelocityPlanner {
public:
    explicit VelocityPlanner(const InterpolationConfig& config);

    /**
     * @brief Plan velocities for all segments
     * @param segments Segments to plan
     */
    void plan(std::vector<PlanningSegment>& segments);

    /**
     * @brief Calculate maximum corner velocity
     * @param prev Previous segment
     * @param next Next segment
     * @return Maximum velocity at junction
     */
    double calculateCornerVelocity(const PlanningSegment& prev, const PlanningSegment& next) const;

    /**
     * @brief Calculate maximum velocity for curvature
     * @param curvature Local curvature (1/radius)
     * @return Maximum velocity
     */
    double maxVelocityForCurvature(double curvature) const;

    /**
     * @brief Calculate reachable velocity given distance and limits
     * @param startVelocity Initial velocity
     * @param distance Available distance
     * @param acceleration Available acceleration
     * @return Maximum reachable velocity
     */
    double reachableVelocity(double startVelocity, double distance, double acceleration) const;

private:
    InterpolationConfig config_;

    void forwardPass(std::vector<PlanningSegment>& segments);
    void backwardPass(std::vector<PlanningSegment>& segments);
    double cornerAngle(const PlanningSegment& prev, const PlanningSegment& next) const;
};

} // namespace GCode
