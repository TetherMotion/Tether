/**
 * @file VelocityProfile.hpp
 * @brief TOPP-RA Inspired Velocity Profiling
 *
 * @details
 * This file implements the velocity profiler that computes a time-optimal
 * velocity profile along the path subject to kinematic constraints.
 *
 * ## Algorithm (Inspired by TOPP-RA)
 *
 * 1. **Forward Pass**: Starting from initial velocity, compute maximum
 *    achievable velocity respecting acceleration limits.
 *
 * 2. **Backward Pass**: Starting from final velocity, compute maximum
 *    achievable velocity respecting deceleration limits.
 *
 * 3. **Velocity Limit Curve**: At each point, compute the maximum velocity
 *    that keeps acceleration within bounds given the path curvature.
 *
 * 4. **Final Profile**: Take minimum of forward, backward, and velocity
 *    limit curves.
 *
 * ## Constraints
 *
 * - Per-axis velocity limits
 * - Per-axis acceleration limits
 * - Per-axis jerk limits (optional)
 * - Centripetal acceleration limit (function of velocity and curvature)
 *
 * @see PiecewiseNurbsPath.hpp
 */

#pragma once

#include "MathTypes.hpp"
#include "PathAdapter.hpp"
#include <tether/motion_planner/geometry/CertifiedCurvatureSampler.hpp>
#include <tether/motion_planner/blend/PHQuinticBlendBuilder.hpp>
#include <vector>
#include <array>
#include <algorithm>
#include <cmath>
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
     * @brief Compute maximum acceleration for a direction and velocity
     *
     * Considers both tangential and centripetal components.
     */
    T maxAccelerationForDirection(const Vec<NumAxes, T>& tangent,
                                   T curvature,
                                   T velocity) const {
        // Centripetal acceleration uses some of the available acceleration
        T centripetalAccel = velocity * velocity * curvature;
        T availableTangential = path.maxPathAcceleration;
        
        if (centripetalAccel > path.maxCentripetalAcceleration) {
            // Velocity too high for this curvature
            return T(0);
        }
        
        // Per-axis acceleration limits
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
template<typename T = double>
struct VelocityProfilePoint {
    /// Arc length position along path
    T arcLength = T(0);
    
    /// Velocity at this point
    T velocity = T(0);
    
    /// Acceleration at this point
    T acceleration = T(0);
    
    /// Jerk at this point (units/second³)
    /// Populated by jerk-limited profilers; zero for basic TOPP-RA.
    T jerk = T(0);
    
    /// Time to reach this point from path start
    T time = T(0);

    /// Velocity limit at this point (the v_lim(s) used by the profiler).
    /// Populated by the profiler so downstream consumers (e.g. ReNURBS)
    /// can check constraint preservation against the *exact* limits the
    /// profiler used, without reconstructing them. Default +infinity so
    /// hand-built profiles (e.g. in tests) remain unconstrained.
    T velocityLimit = std::numeric_limits<T>::infinity();

    /// Tangential acceleration limit at this point (the a_max(s) used by
    /// the profiler). Default +infinity for backward compatibility.
    T accelerationLimit = std::numeric_limits<T>::infinity();

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
// Velocity Profile
// ============================================================================

/**
 * @brief Complete velocity profile for a path
 */
template<typename T = double>
class VelocityProfile {
public:
    using Point = VelocityProfilePoint<T>;

    VelocityProfile() = default;

    /**
     * @brief Get velocity at arc length position
     */
    T velocityAt(T arcLength) const {
        if (points_.empty()) return T(0);
        
        // Binary search for bracket
        auto it = std::lower_bound(points_.begin(), points_.end(), arcLength,
            [](const Point& p, T s) { return p.arcLength < s; });
        
        if (it == points_.begin()) {
            return points_.front().velocity;
        }
        if (it == points_.end()) {
            return points_.back().velocity;
        }
        
        // Linear interpolation
        auto prev = it - 1;
        T alpha = (arcLength - prev->arcLength) / (it->arcLength - prev->arcLength);
        return prev->velocity * (T(1) - alpha) + it->velocity * alpha;
    }

    /**
     * @brief Get acceleration at arc length position
     *
     * Linearly interpolates the acceleration stored in profile points.
     * For jerk-limited profilers, this is the analytic acceleration
     * computed during the forward/backward passes. For basic TOPP-RA,
     * this is the post-hoc estimate.
     */
    T accelerationAt(T arcLength) const {
        if (points_.empty()) return T(0);
        
        auto it = std::lower_bound(points_.begin(), points_.end(), arcLength,
            [](const Point& p, T s) { return p.arcLength < s; });
        
        if (it == points_.begin()) {
            return points_.front().acceleration;
        }
        if (it == points_.end()) {
            return points_.back().acceleration;
        }
        
        auto prev = it - 1;
        T alpha = (arcLength - prev->arcLength) / (it->arcLength - prev->arcLength);
        return prev->acceleration * (T(1) - alpha) + it->acceleration * alpha;
    }

    /**
     * @brief Get jerk at arc length position
     *
     * Linearly interpolates the jerk stored in profile points.
     * For jerk-limited profilers, this is ±j_max or 0 (by construction).
     * For basic TOPP-RA, this is zero (jerk is not constrained).
     */
    T jerkAt(T arcLength) const {
        if (points_.empty()) return T(0);
        
        auto it = std::lower_bound(points_.begin(), points_.end(), arcLength,
            [](const Point& p, T s) { return p.arcLength < s; });
        
        if (it == points_.begin()) {
            return points_.front().jerk;
        }
        if (it == points_.end()) {
            return points_.back().jerk;
        }
        
        auto prev = it - 1;
        T alpha = (arcLength - prev->arcLength) / (it->arcLength - prev->arcLength);
        return prev->jerk * (T(1) - alpha) + it->jerk * alpha;
    }

    /**
     * @brief Get time at arc length position
     */
    T timeAt(T arcLength) const {
        if (points_.empty()) return T(0);
        
        auto it = std::lower_bound(points_.begin(), points_.end(), arcLength,
            [](const Point& p, T s) { return p.arcLength < s; });
        
        if (it == points_.begin()) {
            return points_.front().time;
        }
        if (it == points_.end()) {
            return points_.back().time;
        }
        
        auto prev = it - 1;
        T alpha = (arcLength - prev->arcLength) / (it->arcLength - prev->arcLength);
        return prev->time * (T(1) - alpha) + it->time * alpha;
    }

    /**
     * @brief Get arc length at time
     *
     * Inverse of timeAt() - given a time, find the arc length position.
     */
    T arcLengthAt(T time) const {
        if (points_.empty()) return T(0);
        
        auto it = std::lower_bound(points_.begin(), points_.end(), time,
            [](const Point& p, T t) { return p.time < t; });
        
        if (it == points_.begin()) {
            return points_.front().arcLength;
        }
        if (it == points_.end()) {
            return points_.back().arcLength;
        }
        
        auto prev = it - 1;
        T alpha = (time - prev->time) / (it->time - prev->time);
        return prev->arcLength * (T(1) - alpha) + it->arcLength * alpha;
    }

    /**
     * @brief Total time to traverse the path
     */
    T totalTime() const {
        return points_.empty() ? T(0) : points_.back().time;
    }

    /**
     * @brief Total path length
     */
    T totalLength() const {
        return points_.empty() ? T(0) : points_.back().arcLength;
    }

    /**
     * @brief Access profile points
     */
    const std::vector<Point>& points() const { return points_; }
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

using VelocityProfileD = VelocityProfile<double>;

}  // namespace MotionPlanner
