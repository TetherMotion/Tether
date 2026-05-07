/**
 * @file MotionPlan.hpp
 * @brief Unified Motion Plan with Complete Traceability
 *
 * @details
 * This is the top-level interface for querying motion state at any time t.
 * It combines:
 * - Piecewise Bézier path (geometry)
 * - Velocity profile (time parameterization)
 * - S-curve profiles (jerk-limited phases)
 * - Source references (G-code traceability)
 *
 * ## Query Interface
 *
 * Given a time t, the MotionPlan returns:
 * - Position (all axes)
 * - Velocity (all axes, and path velocity)
 * - Acceleration (all axes, and path acceleration)
 * - Jerk (all axes, and path jerk)
 * - Arc length position
 * - Current G-code line reference
 *
 * ## Features
 *
 * - Bidirectional traversal (negative velocity/reverse)
 * - Real-time feed rate override support
 * - Pause/resume with state preservation
 * - Complete traceability chain
 *
 * @see PiecewisePath.hpp
 * @see VelocityProfile.hpp
 * @see SCurveProfile.hpp
 */

#pragma once

#include "MathTypes.hpp"
#include "BezierCurve.hpp"
#include "PiecewiseNURBSPath.hpp"
#include "PiecewisePath.hpp"
#include "VelocityProfile.hpp"
#include "PathBuilder.hpp"
#include "SCurveProfile.hpp"
#include "MotionSegment.hpp"
#include "SourceReference.hpp"
#include <optional>
#include <functional>
#include <memory>

namespace MotionPlanner {

// ============================================================================
// Motion State
// ============================================================================

/**
 * @brief Complete motion state at a point in time
 */
template<size_t Dim, typename T = double>
struct MotionState {
    /// Time from plan start (seconds)
    T time = T(0);
    
    /// Arc length position along path
    T arcLength = T(0);
    
    /// Position in Cartesian coordinates
    Vec<Dim, T> position;
    
    /// Velocity vector (units/second)
    Vec<Dim, T> velocity;
    
    /// Acceleration vector (units/second²)
    Vec<Dim, T> acceleration;
    
    /// Jerk vector (units/second³)
    Vec<Dim, T> jerk;
    
    /// Path (tangential) velocity magnitude
    T pathVelocity = T(0);
    
    /// Path (tangential) acceleration magnitude
    T pathAcceleration = T(0);
    
    /// Path jerk magnitude
    T pathJerk = T(0);
    
    /// Current curvature
    T curvature = T(0);
    
    /// Source reference for current position
    SourceReference sourceRef;
    
    /// Index of current path segment
    size_t segmentIndex = 0;
    
    /// Parameter within current segment [0, 1]
    T segmentParameter = T(0);
    
    /// Is motion in reverse direction?
    bool isReverse = false;
    
    /// Current feed rate override (1.0 = 100%)
    T feedOverride = T(1);
    
    /// Is motion paused?
    bool isPaused = false;
    
    /// S-curve phase (if applicable)
    SCurvePhase scurvePhase = SCurvePhase::Cruise;
};

// ============================================================================
// Motion Plan Configuration
// ============================================================================

/**
 * @brief Configuration for motion plan execution
 */
template<typename T = double>
struct MotionPlanConfig {
    /// Sample interval for internal profile (seconds)
    T sampleInterval = T(0.001);
    
    /// Enable S-curve profiles for jerk limiting
    bool enableSCurve = true;
    
    /// Enable reverse motion support
    bool enableReverse = true;
    
    /// Minimum feed rate override (0.0 = pause)
    T minFeedOverride = T(0.0);
    
    /// Maximum feed rate override
    T maxFeedOverride = T(2.0);
    
    /// Feed override ramp rate (per second)
    T feedOverrideRampRate = T(1.0);
    
    /// Position tolerance for segment transitions
    T positionTolerance = T(1e-9);
};

// ============================================================================
// Motion Plan
// ============================================================================

/**
 * @brief Unified motion plan combining path, velocity profile, and traceability
 */
template<size_t Dim, typename T = double>
class MotionPlan : public Traceable {
public:
    using Path = PiecewiseNURBSPath<Dim, T>;
    using LegacyPath = PiecewiseBezierPath<Dim, T>;
    using Profile = VelocityProfile<T>;
    using SCurve = SCurveProfile<T>;
    using State = MotionState<Dim, T>;
    using Config = MotionPlanConfig<T>;
    using Point = Vec<Dim, T>;

    MotionPlan() = default;

    /**
     * @brief Construct from path and velocity profile
     */
    MotionPlan(Path path, Profile profile, Config config = {})
        : path_(std::move(path))
        , profile_(std::move(profile))
        , config_(std::move(config)) {
        initialize();
    }

    /**
     * @brief Evaluate complete motion state at time t
     *
     * This is the primary query interface.
     */
    State evaluateAt(T t) const {
        State state;
        state.time = t;
        state.feedOverride = currentFeedOverride_;
        state.isPaused = isPaused_;
        state.isReverse = isReverse_;
        
        if (path_.numSegments() == 0) {
            return state;
        }
        
        // Handle pause state
        if (isPaused_) {
            state = lastState_;
            state.time = t;
            state.pathVelocity = T(0);
            state.pathAcceleration = T(0);
            state.pathJerk = T(0);
            state.velocity = Point{};
            state.acceleration = Point{};
            state.jerk = Point{};
            return state;
        }
        
        // Adjust time for feed override
        T effectiveTime = computeEffectiveTime(t);
        
        // Handle reverse motion
        if (isReverse_) {
            effectiveTime = profile_.totalTime() - effectiveTime;
        }
        
        // Clamp to valid range
        effectiveTime = clamp(effectiveTime, T(0), profile_.totalTime());
        
        // Get arc length at this time
        state.arcLength = profile_.arcLengthAt(effectiveTime);
        
        // Evaluate path geometry
        auto pathEval = path_.evaluateAtArcLength(state.arcLength);
        state.position = pathEval.position;
        state.segmentIndex = pathEval.segmentIndex;
        state.segmentParameter = pathEval.localParameter;
        
        // Get velocity from profile
        state.pathVelocity = profile_.velocityAt(state.arcLength);
        
        // Apply feed override
        state.pathVelocity *= currentFeedOverride_;
        
        // Reverse direction if needed
        if (isReverse_) {
            state.pathVelocity = -state.pathVelocity;
        }
        
        // Compute velocity vector
        state.velocity = pathEval.tangent * state.pathVelocity;
        
        // Get curvature
        state.curvature = path_.curvatureAtArcLength(state.arcLength);
        
        // Compute acceleration
        // a = dv/dt * tangent + v²·κ·normal
        if (config_.enableSCurve && scurveIndex_ < scurves_.size()) {
            auto scurveState = scurves_[scurveIndex_].evaluateAt(effectiveTime - scurveStartTime_);
            state.pathAcceleration = scurveState.acceleration * currentFeedOverride_;
            state.pathJerk = scurveState.jerk * currentFeedOverride_;
            state.scurvePhase = scurveState.phase;
        } else {
            // Estimate acceleration from profile
            state.pathAcceleration = estimateAcceleration(effectiveTime);
        }
        
        // Tangential acceleration
        Point tangentialAccel = pathEval.tangent * state.pathAcceleration;
        
        // Centripetal acceleration
        T v = std::abs(state.pathVelocity);
        T centripetalMag = v * v * state.curvature;
        Point normal = computeNormal(pathEval.tangent, state.arcLength);
        Point centripetalAccel = normal * centripetalMag;
        
        state.acceleration = tangentialAccel + centripetalAccel;
        
        // Jerk (simplified - mainly from S-curve)
        state.jerk = pathEval.tangent * state.pathJerk;
        
        // Get source reference
        state.sourceRef = path_.sourceRefAtArcLength(state.arcLength);
        
        return state;
    }

    /**
     * @brief Evaluate position only at time t (faster than full state)
     */
    Point positionAt(T t) const {
        if (path_.numSegments() == 0) {
            return Point{};
        }
        
        T effectiveTime = computeEffectiveTime(t);
        if (isReverse_) {
            effectiveTime = profile_.totalTime() - effectiveTime;
        }
        
        T arcLength = profile_.arcLengthAt(effectiveTime);
        return path_.evaluateAtArcLength(arcLength).position;
    }

    /**
     * @brief Get total duration of motion plan
     */
    T totalDuration() const {
        return profile_.totalTime() / std::max(currentFeedOverride_, T(0.001));
    }

    /**
     * @brief Get total path length
     */
    T totalLength() const { return path_.totalLength(); }

    /**
     * @brief Number of path segments
     */
    size_t numSegments() const { return path_.numSegments(); }

    // ========================================================================
    // Feed Rate Override
    // ========================================================================

    /**
     * @brief Set feed rate override (1.0 = 100%)
     */
    void setFeedOverride(T override) {
        currentFeedOverride_ = clamp(override, 
                                      config_.minFeedOverride, 
                                      config_.maxFeedOverride);
    }

    /**
     * @brief Get current feed rate override
     */
    T feedOverride() const { return currentFeedOverride_; }

    /**
     * @brief Ramp feed override smoothly to target
     */
    void rampFeedOverride(T target, T deltaTime) {
        target = clamp(target, config_.minFeedOverride, config_.maxFeedOverride);
        
        T maxChange = config_.feedOverrideRampRate * deltaTime;
        T diff = target - currentFeedOverride_;
        
        if (std::abs(diff) <= maxChange) {
            currentFeedOverride_ = target;
        } else {
            currentFeedOverride_ += (diff > T(0)) ? maxChange : -maxChange;
        }
    }

    // ========================================================================
    // Pause/Resume
    // ========================================================================

    /**
     * @brief Pause motion at current state
     */
    void pause(T currentTime) {
        if (!isPaused_) {
            lastState_ = evaluateAt(currentTime);
            pauseTime_ = currentTime;
            isPaused_ = true;
        }
    }

    /**
     * @brief Resume motion from paused state
     */
    void resume(T currentTime) {
        if (isPaused_) {
            timeOffset_ += currentTime - pauseTime_;
            isPaused_ = false;
        }
    }

    /**
     * @brief Check if paused
     */
    bool isPaused() const { return isPaused_; }

    // ========================================================================
    // Reverse Motion
    // ========================================================================

    /**
     * @brief Enable/disable reverse motion
     */
    void setReverse(bool reverse) {
        if (config_.enableReverse) {
            isReverse_ = reverse;
        }
    }

    /**
     * @brief Check if moving in reverse
     */
    bool isReverse() const { return isReverse_; }

    // ========================================================================
    // Progress Queries
    // ========================================================================

    /**
     * @brief Get progress as fraction [0, 1]
     */
    T progressAt(T t) const {
        T duration = totalDuration();
        return (duration > T(0)) ? clamp(t / duration, T(0), T(1)) : T(0);
    }

    /**
     * @brief Check if motion is complete
     */
    bool isComplete(T t) const {
        return t >= totalDuration();
    }

    /**
     * @brief Get remaining distance
     */
    T remainingDistance(T t) const {
        State state = evaluateAt(t);
        return totalLength() - state.arcLength;
    }

    /**
     * @brief Get remaining time
     */
    T remainingTime(T t) const {
        return std::max(totalDuration() - t, T(0));
    }

    // ========================================================================
    // Segment Queries
    // ========================================================================

    /**
     * @brief Find time at which a G-code line is reached
     */
    std::optional<T> findTimeAtLine(size_t lineNumber) const {
        // Find arc length for this line
        auto arcLength = path_.findArcLengthForLine(lineNumber);
        if (!arcLength) {
            return std::nullopt;
        }
        
        // Find time for this arc length
        return profile_.timeAt(*arcLength);
    }

    /**
     * @brief Get all G-code lines traversed in time range
     */
    std::vector<size_t> getLinesInRange(T startTime, T endTime) const {
        std::vector<size_t> lines;
        
        State start = evaluateAt(startTime);
        State end = evaluateAt(endTime);
        
        // Collect unique line numbers
        for (size_t i = start.segmentIndex; i <= end.segmentIndex && i < path_.numSegments(); ++i) {
            auto ref = path_.getSegment(i).sourceRef();
            if (ref.type() == SourceReference::Type::Single) {
                lines.push_back(ref.lineNumber());
            }
        }
        
        return lines;
    }

    // ========================================================================
    // Path Access
    // ========================================================================

    /**
     * @brief Access the underlying path
     */
    const Path& path() const { return path_; }

    /**
     * @brief Access the velocity profile
     */
    const Profile& profile() const { return profile_; }

    /**
     * @brief Access configuration
     */
    const Config& config() const { return config_; }

    // ========================================================================
    // Traceable Interface
    // ========================================================================

    const SourceReference& sourceRef() const override {
        return sourceRef_;
    }

    void setSourceRef(SourceReference ref) {
        sourceRef_ = std::move(ref);
    }

private:
    /**
     * @brief Initialize internal structures
     */
    void initialize() {
        if (config_.enableSCurve) {
            buildSCurveProfiles();
        }
    }

    /**
     * @brief Build S-curve profiles for path segments
     */
    void buildSCurveProfiles() {
        // Build S-curve profiles based on velocity profile points
        SCurveConstraints<T> constraints;
        constraints.maxVelocity = T(100);  // Would come from limits
        constraints.maxAcceleration = T(500);
        constraints.maxJerk = T(5000);
        
        SCurveProfileBuilder<T> builder(constraints);
        
        // For now, create one S-curve for the entire path
        // A full implementation would create per-segment profiles
        if (profile_.points().size() >= 2) {
            SCurve profile;
            profile.compute(path_.totalLength(), T(0), T(0), constraints);
            scurves_.push_back(std::move(profile));
        }
    }

    /**
     * @brief Compute effective time accounting for feed override
     */
    T computeEffectiveTime(T t) const {
        // Simple model: effective_time = integral of feed_override over time
        // For constant override: effective_time = t * feed_override
        // With ramping, would need to integrate
        return (t - timeOffset_) * currentFeedOverride_;
    }

    /**
     * @brief Estimate acceleration from velocity profile
     */
    T estimateAcceleration(T t) const {
        T dt = T(0.001);
        T v1 = profile_.velocityAt(profile_.arcLengthAt(t));
        T v2 = profile_.velocityAt(profile_.arcLengthAt(t + dt));
        return (v2 - v1) / dt;
    }

    /**
     * @brief Compute normal vector at arc length
     */
    Point computeNormal(const Point& tangent, T arcLength) const {
        // For 2D, rotate tangent 90°
        // For 3D, use Frenet normal from curvature
        if constexpr (Dim == 2) {
            return Point{-tangent[1], tangent[0]};
        } else if constexpr (Dim >= 3) {
            // Approximate by numerical differentiation of tangent
            T ds = T(0.001);
            Point t1 = path_.evaluateAtArcLength(arcLength - ds).tangent;
            Point t2 = path_.evaluateAtArcLength(arcLength + ds).tangent;
            Point dtds = (t2 - t1) / (T(2) * ds);
            T len = dtds.length();
            return (len > MathConstants::EPSILON) ? dtds / len : Point{};
        }
        return Point{};
    }

    Path path_;
    Profile profile_;
    Config config_;
    
    std::vector<SCurve> scurves_;
    size_t scurveIndex_ = 0;
    T scurveStartTime_ = T(0);
    
    T currentFeedOverride_ = T(1);
    T timeOffset_ = T(0);
    T pauseTime_ = T(0);
    bool isPaused_ = false;
    bool isReverse_ = false;
    
    State lastState_;
    SourceReference sourceRef_;
};

// ============================================================================
// Motion Plan Builder
// ============================================================================

/**
 * @brief Builds MotionPlan from motion segments
 */
template<size_t Dim, typename T = double>
class MotionPlanBuilder {
public:
    using Plan = MotionPlan<Dim, T>;
    using Path = PiecewiseNURBSPath<Dim, T>;
    using Profile = VelocityProfile<T>;
    using Config = MotionPlanConfig<T>;
    using Limits = KinematicLimits<Dim, T>;

    /**
     * @brief Constructor
     */
    MotionPlanBuilder(Limits limits = {}, Config config = {})
        : limits_(std::move(limits))
        , config_(std::move(config)) {}

    /**
     * @brief Build motion plan from segment list
     *
     * @param segments Motion segments (from G-code parser)
     * @param feedRate Default feed rate
     * @return Complete motion plan
     */
    Plan build(const MotionSegmentList& segments, T feedRate) {
        // Build path with corner blending
        PathBuilder<Dim, T> pathBuilder;
        auto pathResult = pathBuilder.build(segments);
        
        if (!pathResult.success || pathResult.nurbsPath.numSegments() == 0) {
            return Plan{};
        }
        
        // Compute velocity profile using NURBS path
        VelocityProfiler<Dim, T> profiler(limits_);
        Profile profile = profiler.computeProfile(pathResult.nurbsPath, feedRate);
        
        // Create motion plan with NURBS path
        Plan plan(std::move(pathResult.nurbsPath), std::move(profile), config_);
        
        // Set source reference to cover all input segments
        if (!segments.empty()) {
            std::vector<SourceReference> refs;
            for (size_t i = 0; i < segments.size(); ++i) {
                refs.push_back(segments.at(i).sourceRef);
            }
            plan.setSourceRef(SourceReference::multiple(refs));
        }
        
        return plan;
    }

    /**
     * @brief Get/set kinematic limits
     */
    Limits& limits() { return limits_; }
    const Limits& limits() const { return limits_; }

    /**
     * @brief Get/set configuration
     */
    Config& config() { return config_; }
    const Config& config() const { return config_; }

private:
    Limits limits_;
    Config config_;
};

// ============================================================================
// Type Aliases
// ============================================================================

using MotionState2D = MotionState<2, double>;
using MotionState3D = MotionState<3, double>;

using MotionPlanConfigD = MotionPlanConfig<double>;

using MotionPlan2D = MotionPlan<2, double>;
using MotionPlan3D = MotionPlan<3, double>;

using MotionPlanBuilder2D = MotionPlanBuilder<2, double>;
using MotionPlanBuilder3D = MotionPlanBuilder<3, double>;

}  // namespace MotionPlanner
