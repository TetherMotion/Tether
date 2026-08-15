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
#include "IVelocityProfiler.hpp"
#include <tether/motion_planner/geometry/CertifiedCurvatureSampler.hpp>
#include <tether/motion_planner/blend/PHQuinticBlendBuilder.hpp>
#include <vector>
#include <array>
#include <algorithm>
#include <cmath>
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
// Velocity Profiler
// ============================================================================

/**
 * @brief Computes velocity profiles using TOPP-RA inspired algorithm
 *
 * This is the basic 2nd-order TOPP-RA profiler. It produces a time-optimal
 * velocity profile with bang-bang acceleration (no jerk limiting).
 * Acceleration is discontinuous at constraint switching points.
 *
 * For jerk-limited profiling, use JerkLimitedVelocityProfiler instead.
 */
template<size_t Dim, typename T = double>
class VelocityProfiler : public IVelocityProfiler<Dim, T> {
public:
    using Path = PathAdapter<Dim, T>;
    using Profile = VelocityProfile<T>;
    using Limits = KinematicLimits<Dim, T>;
    using Point = VelocityProfilePoint<T>;

    /**
     * @brief Constructor
     *
     * @param limits Kinematic limits to apply
     */
    explicit VelocityProfiler(Limits limits = {})
        : limits_(std::move(limits)) {}

    /**
     * @brief Compute velocity profile for a path
     *
     * @param path The piecewise Bézier path
     * @param feedRate Commanded feed rate (may be limited by constraints)
     * @param startVelocity Initial velocity (default: 0)
     * @param endVelocity Target final velocity (default: 0)
     * @param numSamples Number of sample points along path
     * @param startAcceleration Initial acceleration (default: 0) - Required for replanning from moving state
     * @param startJerk Initial jerk (default: 0) - Required for replanning from moving state
     * @return Computed velocity profile
     */
    Profile computeProfile(const Path& path,
                           T feedRate,
                           T startVelocity = T(0),
                           T endVelocity = T(0),
                           size_t numSamples = 100,
                           T startAcceleration = T(0),
                           T startJerk = T(0)) override {
        Profile profile;

        if (path.numSegments() == 0) {
            return profile;
        }

        T pathLength = path.totalLength();
        if (pathLength <= T(0)) {
            return profile;
        }

        // --- Certified curvature sampler (lazy per-span, Lipschitz-bound) --
        // The velocity limit v_lim = √(a_cent / κ) uses the *certified
        // per-span max* curvature rather than the pointwise κ(s). This
        // guarantees the centripetal acceleration constraint is never
        // violated: within a span, the true κ ≤ maxKappa (certified),
        // so v ≤ √(a_cent / maxKappa) is always safe.
        //
        // Why certified sampling instead of closed-form: curvature extrema
        // of degree-5/7 Béziers have no closed-form solution (κ′ = 0 is a
        // high-degree rational equation, beyond the Abel–Ruffini barrier
        // for degree ≥ 5). See CertifiedCurvatureSampler.hpp for details.
        const tether::motion::CertifiedCurvatureSampler* curvatureSampler = nullptr;
        if (path.hasInner()) {
            curvatureSampler = &path.curvatureSampler();
        }

        // Sample path at uniform arc length intervals
        T ds = pathLength / T(numSamples - 1);

        std::vector<PathSample> samples(numSamples);

        for (size_t i = 0; i < numSamples; ++i) {
            T s = std::min(i * ds, pathLength);
            samples[i].arcLength = s;

            auto eval = path.evaluateAtArcLength(s);
            samples[i].position = eval.position;
            samples[i].tangent = eval.tangent;

            // --- PH fast path (Phase 5.4) -------------------------------
            // If the current piece has PHData, use the closed-form
            // curvature κ(ξ) = 2(uv'−u'v)/σ²(ξ) (M16) instead of the
            // certified sampler. This is faster (no sampling) and exact
            // (no certificate width). The trade-off is that the pointwise
            // κ may not be the max on the span, so the velocity limit is
            // less conservative — but for PH blends the curvature is
            // smooth and the sample density is high enough that the
            // difference is negligible.
            bool usedPH = false;
            if (path.hasInner() && path.hasPHData()) {
                const auto& inner = path.inner();
                auto loc = inner.locate(static_cast<double>(s));
                const auto& ph = path.phData(loc.piece);
                if (ph) {
                    // Map the local arc length to the PH parameter ξ ∈ [0,1].
                    // The PH curve's total arc length is polynomial; invert
                    // it to get ξ from the local arc length.
                    const double localS = loc.localS;
                    const double xi = tether::motion::PHQuinticBlendBuilder::invertArcLength(*ph, localS);
                    samples[i].curvature = static_cast<T>(
                        tether::motion::PHQuinticBlendBuilder::curvature(*ph, xi));
                    usedPH = true;
                }
            }

            if (!usedPH) {
                if (curvatureSampler) {
                    // Use the certified per-span max curvature (conservative).
                    auto cert = curvatureSampler->maxCurvatureAtArcLength(
                        static_cast<double>(s));
                    samples[i].curvature = static_cast<T>(cert.maxKappa);
                } else {
                    // Fallback: pointwise curvature (no certificate).
                    samples[i].curvature = path.curvatureAtArcLength(s);
                }
            }
        }
        
        // Compute velocity limit curve (from curvature and feed rate)
        std::vector<T> velocityLimit(numSamples);
        for (size_t i = 0; i < numSamples; ++i) {
            velocityLimit[i] = computeVelocityLimit(samples[i], feedRate);
        }
        
        // Forward pass: accelerating from start
        std::vector<T> forwardVelocity(numSamples);
        forwardVelocity[0] = std::min(startVelocity, velocityLimit[0]);
        
        for (size_t i = 1; i < numSamples; ++i) {
            T maxAccel = computeMaxAcceleration(samples[i - 1], forwardVelocity[i - 1]);
            T deltaS = samples[i].arcLength - samples[i - 1].arcLength;
            
            // v² = v₀² + 2·a·s
            T v2 = forwardVelocity[i - 1] * forwardVelocity[i - 1] + T(2) * maxAccel * deltaS;
            T achievable = std::sqrt(std::max(v2, T(0)));
            
            forwardVelocity[i] = std::min(achievable, velocityLimit[i]);
        }
        
        // Backward pass: decelerating to end
        std::vector<T> backwardVelocity(numSamples);
        backwardVelocity[numSamples - 1] = std::min(endVelocity, velocityLimit[numSamples - 1]);
        
        for (size_t i = numSamples - 1; i > 0; --i) {
            T maxDecel = computeMaxAcceleration(samples[i], backwardVelocity[i]);
            T deltaS = samples[i].arcLength - samples[i - 1].arcLength;
            
            // v₀² = v² - 2·a·s  →  v₀ = √(v² + 2·a·s) when decelerating
            T v2 = backwardVelocity[i] * backwardVelocity[i] + T(2) * maxDecel * deltaS;
            T achievable = std::sqrt(std::max(v2, T(0)));
            
            backwardVelocity[i - 1] = std::min(achievable, velocityLimit[i - 1]);
        }
        
        // Final profile: minimum of all constraints
        profile.reserve(numSamples);
        T currentTime = T(0);
        
        for (size_t i = 0; i < numSamples; ++i) {
            Point pt;
            pt.arcLength = samples[i].arcLength;
            
            // Take minimum of all velocity constraints
            T fwd = forwardVelocity[i];
            T bwd = backwardVelocity[i];
            T lim = velocityLimit[i];
            
            pt.velocity = std::min({fwd, bwd, lim});
            
            // Determine limiting factor
            if (pt.velocity == fwd && fwd < bwd && fwd < lim) {
                pt.limitedBy = Point::LimitType::ForwardAccel;
            } else if (pt.velocity == bwd && bwd < lim) {
                pt.limitedBy = Point::LimitType::BackwardDecel;
            } else if (pt.velocity == lim) {
                pt.limitedBy = Point::LimitType::Curvature;
            }
            
            // Compute time
            if (i > 0) {
                T prevVel = profile.points()[i - 1].velocity;
                T avgVel = (prevVel + pt.velocity) / T(2);
                T deltaS = pt.arcLength - profile.points()[i - 1].arcLength;
                
                if (avgVel > MathConstants::EPSILON) {
                    currentTime += deltaS / avgVel;
                }
            }
            
            pt.time = currentTime;
            
            // Compute acceleration
            if (i == 0) {
                pt.acceleration = startAcceleration;
            } else if (i > 0 && i + 1 < numSamples) {
                T prevVel = profile.points()[i - 1].velocity;
                T nextVel = forwardVelocity[i + 1];  // Estimate
                T dt = pt.time - profile.points()[i - 1].time;
                if (dt > MathConstants::EPSILON) {
                    pt.acceleration = (pt.velocity - prevVel) / dt;
                }
            }
            
            profile.addPoint(pt);
        }
        
        return profile;
    }

    /**
     * @brief Get/set kinematic limits
     */
    Limits& limits() { return limits_; }
    Limits limits() const override { return limits_; }

    ProfilerType type() const override { return ProfilerType::ToppraBasic; }
    const char* name() const override { return "VelocityProfiler (TOPP-RA basic)"; }

private:
    /**
     * @brief Internal path sample data
     */
    struct PathSample {
        T arcLength = T(0);
        Vec<Dim, T> position;
        Vec<Dim, T> tangent;
        T curvature = T(0);
    };

    /**
     * @brief Compute velocity limit at a path sample
     *
     * Limited by:
     * 1. Feed rate
     * 2. Centripetal acceleration (curvature)
     * 3. Per-axis velocity limits
     */
    T computeVelocityLimit(const PathSample& sample, T feedRate) const {
        T limit = feedRate;
        
        // Curvature limit: v² · κ ≤ a_centripetal
        if (sample.curvature > MathConstants::EPSILON) {
            T curvatureLimit = std::sqrt(
                limits_.path.maxCentripetalAcceleration / sample.curvature);
            limit = std::min(limit, curvatureLimit);
        }
        
        // Per-axis velocity limits
        T axisLimit = limits_.maxVelocityForDirection(sample.tangent);
        limit = std::min(limit, axisLimit);
        
        // Path velocity limit
        limit = std::min(limit, limits_.path.maxPathVelocity);
        
        return limit;
    }

    /**
     * @brief Compute maximum acceleration at a path sample
     */
    T computeMaxAcceleration(const PathSample& sample, T currentVelocity) const {
        return limits_.maxAccelerationForDirection(
            sample.tangent, sample.curvature, currentVelocity);
    }

    Limits limits_;
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

using VelocityProfiler2D = VelocityProfiler<2, double>;
using VelocityProfiler3D = VelocityProfiler<3, double>;

}  // namespace MotionPlanner
