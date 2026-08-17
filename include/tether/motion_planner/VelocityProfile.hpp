/**
 * @file VelocityProfile.hpp
 * @brief Velocity profile abstract base and sampled concrete type.
 *
 * @details
 * This file defines the velocity profile hierarchy used by the motion planner.
 * The base class `VelocityProfile` is a non-template abstract interface that
 * exposes the kinematic quantities along a path as functions of either arc
 * length or time. The concrete `SampledVelocityProfile` stores tabulated
 * points and linearly interpolates between them.
 *
 * Kinematic limit structures (`KinematicLimits`, `AxisLimits`, `PathLimits`)
 * remain in this header because they are consumed throughout the planner and
 * are intentionally kept unchanged.
 *
 * @see VelocityProfiler.hpp for the abstract profiler interface.
 * @see SampledVelocityProfile for the tabulated concrete type.
 */

#pragma once

#include "MathTypes.hpp"
#include <vector>
#include <array>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <optional>

namespace MotionPlanner {

// ============================================================================
// Kinematic Limits Configuration
// ============================================================================

/**
 * @brief Per-axis kinematic limits
 */
template<size_t NumAxes, typename T = double>
struct AxisLimits {
    /// Maximum velocity per axis (units/second)
    std::array<T, NumAxes> maxVelocity;
    
    /// Maximum acceleration per axis (units/second²)
    std::array<T, NumAxes> maxAcceleration;
    
    /// Maximum jerk per axis (units/second³) - optional
    std::array<T, NumAxes> maxJerk;
    
    /// Whether jerk limits are enabled
    bool jerkLimitEnabled = false;
    
    /**
     * @brief Default constructor with typical machine limits
     */
    AxisLimits() {
        maxVelocity.fill(T(100));      // 100 units/sec
        maxAcceleration.fill(T(500));   // 500 units/sec²
        maxJerk.fill(T(5000));          // 5000 units/sec³
    }
    
    /**
     * @brief Constructor with uniform limits
     */
    AxisLimits(T vel, T accel, T jerk = T(0)) {
        maxVelocity.fill(vel);
        maxAcceleration.fill(accel);
        maxJerk.fill(jerk);
        jerkLimitEnabled = (jerk > T(0));
    }
};

/**
 * @brief Path-level kinematic limits
 */
template<typename T = double>
struct PathLimits {
    /// Maximum tangential velocity along path
    T maxPathVelocity = T(100);
    
    /// Maximum tangential acceleration along path
    T maxPathAcceleration = T(500);
    
    /// Maximum tangential jerk along path
    T maxPathJerk = T(5000);
    
    /// Maximum centripetal acceleration (limits velocity on curves)
    T maxCentripetalAcceleration = T(500);
    
    /// Whether jerk limits are enabled
    bool jerkLimitEnabled = false;
};

/**
 * @brief Combined kinematic limits
 */
template<size_t NumAxes, typename T = double>
struct KinematicLimits {
    AxisLimits<NumAxes, T> axis;
    PathLimits<T> path;
    
    /**
     * @brief Compute maximum velocity at a path point given direction
     *
     * The velocity is limited by per-axis constraints transformed to
     * the path tangent direction.
     */
    T maxVelocityForDirection(const Vec<NumAxes, T>& tangent) const {
        T minVel = path.maxPathVelocity;
        
        for (size_t i = 0; i < NumAxes; ++i) {
            if (std::abs(tangent[i]) > MathConstants::EPSILON) {
                T axisVel = axis.maxVelocity[i] / std::abs(tangent[i]);
                minVel = std::min(minVel, axisVel);
            }
        }
        
        return minVel;
    }
    
    /**
     * @brief Compute maximum tangential acceleration for a direction and
     *        velocity, accounting for the centripetal load.
     *
     * WI-P2: If maxPathAcceleration is a magnitude limit, the available
     * tangential part is sqrt(a_path² − (v²κ)²). The centripetal load
     * v²κ is subtracted from the total acceleration budget before
     * computing the tangential limit. The dead `return 0` branch (which
     * was unreachable because v_lim already enforces v²κ ≤ a_cent) has
     * been removed.
     *
     * @param tangent Unit tangent vector at the path point.
     * @param curvature Curvature at the path point (≥ 0).
     * @param velocity Current velocity (for centripetal load computation).
     * @return Maximum tangential acceleration (≥ 0).
     */
    T maxAccelerationForDirection(const Vec<NumAxes, T>& tangent,
                                   T curvature,
                                   T velocity) const {
        // WI-P2: Centripetal load v²κ consumes part of the total
        // acceleration budget. The available tangential acceleration is
        // sqrt(a_path² − (v²κ)²) when maxPathAcceleration is a magnitude
        // limit. If the centripetal load exceeds the budget, tangential
        // acceleration is zero (all budget consumed by centripetal).
        T centripetalAccel = velocity * velocity * curvature;
        T aPath = path.maxPathAcceleration;

        T availableTangential;
        if (centripetalAccel >= aPath) {
            // Centripetal load consumes the entire budget.
            availableTangential = T(0);
        } else {
            availableTangential = std::sqrt(aPath * aPath
                                            - centripetalAccel * centripetalAccel);
        }

        // Per-axis acceleration limits (tangential projection).
        for (size_t i = 0; i < NumAxes; ++i) {
            if (std::abs(tangent[i]) > MathConstants::EPSILON) {
                T axisAccel = axis.maxAcceleration[i] / std::abs(tangent[i]);
                availableTangential = std::min(availableTangential, axisAccel);
            }
        }

        return availableTangential;
    }
};

// Backwards-compatible aliases used in tests
using KinematicLimits2D = KinematicLimits<2, double>;


// ============================================================================
// Velocity Profile Point
// ============================================================================

/**
 * @brief A point on the velocity profile
 */
struct VelocityProfilePoint {
    /// Arc length position along path
    double arcLength = 0.0;
    
    /// Velocity at this point
    double velocity = 0.0;
    
    /// Acceleration at this point (interval-average finite-difference
    /// approximation for BasicTOPPRA; analytic from carried state for
    /// JerkConstrainedTOPPRA after WI-8).
    double acceleration = 0.0;

    /// Jerk at this point (units/second³).
    /// For JerkConstrainedTOPPRA (post-WI-8): computed from the
    /// acceleration change over time, reported truthfully (not clamped).
    /// For BasicTOPPRA: zero (jerk is not constrained — theoretically
    /// infinite at switching points).
    double jerk = 0.0;
    
    /// Time to reach this point from path start
    double time = 0.0;

    /// Velocity limit at this point (the v_lim(s) used by the profiler).
    /// Populated by the profiler so downstream consumers (e.g. ReNURBS)
    /// can check constraint preservation against the *exact* limits the
    /// profiler used, without reconstructing them. Default +infinity so
    /// hand-built profiles (e.g. in tests) remain unconstrained.
    double velocityLimit = std::numeric_limits<double>::infinity();

    /// Tangential acceleration limit at this point (the a_max(s) used by
    /// the profiler). Default +infinity for backward compatibility.
    double accelerationLimit = std::numeric_limits<double>::infinity();

    /// Limiting factor (for debugging)
    enum class LimitType : uint8_t {
        None,
        ForwardAccel,
        BackwardDecel,
        Curvature,
        AxisVelocity,
        AxisAcceleration,
        FeedRate,
        Jerk
    } limitedBy = LimitType::None;
};

// ============================================================================
// Velocity Profile Abstract Base
// ============================================================================

/**
 * @brief Abstract base class for a velocity profile along a path.
 *
 * All queries are in double precision and clamped to the valid domain by
 * concrete derived classes.
 */
class VelocityProfile {
public:
    virtual ~VelocityProfile() = default;

    /// Velocity at the given arc length.
    virtual double velocityAt(double arcLength) const = 0;

    /// Acceleration at the given arc length.
    virtual double accelerationAt(double arcLength) const = 0;

    /// Jerk at the given arc length.
    virtual double jerkAt(double arcLength) const = 0;

    /// Time at the given arc length.
    virtual double timeAt(double arcLength) const = 0;

    /// Arc length at the given time.
    virtual double arcLengthAt(double time) const = 0;

    /// Total traversal time.
    virtual double totalTime() const = 0;

    /// Total path length.
    virtual double totalLength() const = 0;

    /// Tabulated points, if available. The default implementation returns
    /// an empty vector; SampledVelocityProfile and AnalyticalSSRVelocityProfile
    /// override it with their actual tabulated representation.
    virtual const std::vector<VelocityProfilePoint>& points() const {
        static const std::vector<VelocityProfilePoint> empty;
        return empty;
    }
};

// ============================================================================
// Sampled Velocity Profile
// ============================================================================

/**
 * @brief Tabulated velocity profile with linear interpolation.
 *
 * This is the concrete profile type produced by the discrete TOPPRA and
 * S-curve profilers. It stores points sampled along arc length and
 * interpolates velocity, acceleration, jerk, and time as functions of
 * arc length; arc length is recovered as a function of time by linear
 * interpolation over the time coordinate.
 */
class SampledVelocityProfile : public VelocityProfile {
public:
    using Point = VelocityProfilePoint;

    SampledVelocityProfile() = default;

    /**
     * @brief Get velocity at arc length position
     */
    double velocityAt(double arcLength) const override {
        return interpolate(&Point::velocity, arcLength);
    }

    /**
     * @brief Get acceleration at arc length position
     */
    double accelerationAt(double arcLength) const override {
        return interpolate(&Point::acceleration, arcLength);
    }

    /**
     * @brief Get jerk at arc length position
     */
    double jerkAt(double arcLength) const override {
        return interpolate(&Point::jerk, arcLength);
    }

    /**
     * @brief Get time at arc length position
     */
    double timeAt(double arcLength) const override {
        return interpolate(&Point::time, arcLength);
    }

    /**
     * @brief Get arc length at time
     */
    double arcLengthAt(double time) const override {
        if (points_.empty()) return 0.0;

        auto it = std::lower_bound(points_.begin(), points_.end(), time,
            [](const Point& p, double t) { return p.time < t; });

        if (it == points_.begin()) {
            return points_.front().arcLength;
        }
        if (it == points_.end()) {
            return points_.back().arcLength;
        }

        auto prev = it - 1;
        double alpha = (time - prev->time) / (it->time - prev->time);
        return prev->arcLength * (1.0 - alpha) + it->arcLength * alpha;
    }

    /**
     * @brief Total time to traverse the path
     */
    double totalTime() const override {
        return points_.empty() ? 0.0 : points_.back().time;
    }

    /**
     * @brief Total path length
     */
    double totalLength() const override {
        return points_.empty() ? 0.0 : points_.back().arcLength;
    }

    /**
     * @brief Access profile points
     */
    const std::vector<Point>& points() const override { return points_; }
    std::vector<Point>& points() { return points_; }

    /**
     * @brief Add a profile point
     */
    void addPoint(Point point) {
        points_.push_back(std::move(point));
    }

    /**
     * @brief Reserve space for points
     */
    void reserve(size_t n) { points_.reserve(n); }

    /**
     * @brief Clear all points
     */
    void clear() { points_.clear(); }

private:
    std::vector<Point> points_;

    using Field = double Point::*;

    double interpolate(Field field, double arcLength) const {
        if (points_.empty()) return 0.0;

        auto it = std::lower_bound(points_.begin(), points_.end(), arcLength,
            [](const Point& p, double s) { return p.arcLength < s; });

        if (it == points_.begin()) {
            return points_.front().*field;
        }
        if (it == points_.end()) {
            return points_.back().*field;
        }

        auto prev = it - 1;
        double alpha = (arcLength - prev->arcLength) / (it->arcLength - prev->arcLength);
        return (*prev).*field * (1.0 - alpha) + (*it).*field * alpha;
    }
};

// ============================================================================
// Type Aliases
// ============================================================================

template<size_t N>
using AxisLimitsN = AxisLimits<N, double>;
using AxisLimits3D = AxisLimits<3, double>;

using PathLimitsD = PathLimits<double>;

template<size_t N>
using KinematicLimitsN = KinematicLimits<N, double>;
using KinematicLimits3D = KinematicLimits<3, double>;

/// Backward-compatible alias for the old concrete sampled profile type.
using VelocityProfileD = SampledVelocityProfile;

}  // namespace MotionPlanner
