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

/**
 * @brief Highest path derivative represented by a velocity profile.
 *
 * Lower-order profilers must not be interpreted as providing finite jerk or
 * snap at switching points. Consumers that need those derivatives must
 * inspect this value before querying them.
 */
enum class ProfileDerivativeOrder : uint8_t {
    Velocity,
    Acceleration,
    Jerk,
    Snap,
};

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

    /// Maximum snap (jerk derivative) per axis (units/second⁴) - optional
    std::array<T, NumAxes> maxSnap;

    /// Whether jerk limits are enabled
    bool jerkLimitEnabled = false;

    /// Whether snap limits are enabled
    bool snapLimitEnabled = false;

    /**
     * @brief Default constructor with typical machine limits
     */
    AxisLimits() {
        maxVelocity.fill(T(100));      // 100 units/sec
        maxAcceleration.fill(T(500));   // 500 units/sec²
        maxJerk.fill(T(5000));          // 5000 units/sec³
        maxSnap.fill(T(50000));         // 50000 units/sec⁴
    }

    /**
     * @brief Constructor with uniform limits
     */
    AxisLimits(T vel, T accel, T jerk = T(0), T snap = T(0)) {
        maxVelocity.fill(vel);
        maxAcceleration.fill(accel);
        maxJerk.fill(jerk);
        maxSnap.fill(snap);
        jerkLimitEnabled = (jerk > T(0));
        snapLimitEnabled = (snap > T(0));
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

    /// Maximum tangential snap (jerk derivative) along path
    T maxPathSnap = T(50000);

    /// Maximum centripetal acceleration (limits velocity on curves)
    T maxCentripetalAcceleration = T(500);

    /// Whether jerk limits are enabled
    bool jerkLimitEnabled = false;

    /// Whether snap limits are enabled
    bool snapLimitEnabled = false;
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
        if (path.maxPathAcceleration <= T(0)) return T(0);
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

    /// Jerk at this point (units/second³), when the producer represents it.
    /// A default zero is only storage compatibility for lower-order sampled
    /// profiles; callers must use `VelocityProfile::hasJerk()` before
    /// interpreting it as a physical jerk value.
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

    /// Highest path derivative represented by this profile.
    virtual ProfileDerivativeOrder derivativeOrder() const {
        return ProfileDerivativeOrder::Acceleration;
    }

    /// Whether `jerkAt()` has a physically meaningful value.
    bool hasJerk() const {
        return derivativeOrder() >= ProfileDerivativeOrder::Jerk;
    }

    /// Whether the profile is generated by a snap-limited trajectory.
    bool hasSnap() const {
        return derivativeOrder() >= ProfileDerivativeOrder::Snap;
    }

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

        // Right-continuous convention: at a repeated timestamp, select the
        // last sample at that time. This preserves the state after an
        // instantaneous representation boundary without a zero denominator.
        auto it = std::upper_bound(points_.begin(), points_.end(), time,
            [](double t, const Point& p) { return t < p.time; });

        if (it == points_.begin()) {
            return points_.front().arcLength;
        }
        if (it == points_.end()) {
            return points_.back().arcLength;
        }

        auto prev = it - 1;
        const double dt = it->time - prev->time;
        if (dt <= std::numeric_limits<double>::epsilon()) {
            return prev->arcLength;
        }
        double alpha = (time - prev->time) / dt;
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

    ProfileDerivativeOrder derivativeOrder() const override {
        return ProfileDerivativeOrder::Acceleration;
    }

private:
    std::vector<Point> points_;

    using Field = double Point::*;

    double interpolate(Field field, double arcLength) const {
        if (points_.empty()) return 0.0;

        // Use the right-most equal sample. In particular, `timeAt()` at a
        // dwell position returns the end of that dwell rather than its start.
        auto it = std::upper_bound(points_.begin(), points_.end(), arcLength,
            [](double s, const Point& p) { return s < p.arcLength; });

        if (it == points_.begin()) {
            return points_.front().*field;
        }
        if (it == points_.end()) {
            return points_.back().*field;
        }

        auto prev = it - 1;
        const double ds = it->arcLength - prev->arcLength;
        if (std::abs(ds) <= std::numeric_limits<double>::epsilon()) {
            return prev->*field;
        }
        double alpha = (arcLength - prev->arcLength) / ds;
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
