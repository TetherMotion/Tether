/**
 * @file MotionPlan.hpp
 * @brief Unified Motion Plan with Complete Traceability
 *
 * @details
 * This is the top-level interface for querying motion state at any time t.
 * It combines:
 * - Piecewise Bézier path (geometry)
 * - Velocity profile (time parameterization, from any VelocityProfiler)
 * - Source references (G-code traceability)
 *
 * The velocity profile is produced by an VelocityProfiler implementation
 * (basic TOPP-RA, jerk-limited TOPP-RA, or basic S-curve). MotionPlan
 * consumes the profile's per-point velocity, acceleration, and jerk
 * directly — it does not perform post-hoc smoothing or finite-difference
 * estimation. This ensures that the constraints verified by the profiler
 * are preserved exactly.
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
 * @see PiecewiseNurbsPath.hpp
 * @see VelocityProfile.hpp
 * @see VelocityProfiler.hpp
 */

#pragma once

#include "MathTypes.hpp"
#include "PathAdapter.hpp"
#include "VelocityProfile.hpp"
#include "VelocityProfiler.hpp"
#include "BasicTOPPRA.hpp"
#include "JerkConstrainedTOPPRA.hpp"
#include "SCurveVelocityProfiler.hpp"
#include "tether/motion_planner/profile_renurbs/ReNURBSProfile.hpp"
#include "tether/motion_planner/profile_renurbs/ReNURBSProfileBuilder.hpp"
#include "SCurveProfile.hpp"
#include "MotionSegment.hpp"
#include "SourceReference.hpp"
#include "analytical/AnalyticalTypes.hpp"
#include "analytical/AnalyticalTOPPRA.hpp"
#include "analytical/ParetoTimeEnergyOptimalVelocityPlanner.hpp"
#include "analytical/TrajectorySampler.hpp"
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

    /// ReNURBS configuration (analytical NURBS representation of the
    /// velocity/acceleration/jerk/time profiles). When enabled, the
    /// MotionPlanBuilder builds a ReNURBSProfile from the sampled
    /// VelocityProfile and stores it in the MotionPlan.
    /// Default: disabled (the sampled profile is used as-is).
    tether::motion::profile_renurbs::ReNURBSConfig renurbs;
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
    using Path = PathAdapter<Dim, T>;
    using Profile = std::shared_ptr<VelocityProfile>;
    using State = MotionState<Dim, T>;
    using Config = MotionPlanConfig<T>;
    using Point = Vec<Dim, T>;
    using Limits = KinematicLimits<Dim, T>;

    MotionPlan() = default;

    // Custom move constructor: when a MotionPlan is moved, the underlying
    // analytical trajectory source (SSR) may hold a raw pointer to the
    // path. Update that pointer to the new location after the move.
    MotionPlan(MotionPlan&& other) noexcept
        : path_(std::move(other.path_))
        , profile_(std::move(other.profile_))
        , limits_(std::move(other.limits_))
        , config_(std::move(other.config_))
        , renurbsProfile_(std::move(other.renurbsProfile_))
        , currentFeedOverride_(other.currentFeedOverride_)
        , timeOffset_(other.timeOffset_)
        , isReverse_(other.isReverse_)
        , lastState_(std::move(other.lastState_))
        , sourceRef_(std::move(other.sourceRef_)) {
        updateAnalyticalPathPointer();
    }

    MotionPlan& operator=(MotionPlan&& other) noexcept {
        if (this != &other) {
            path_ = std::move(other.path_);
            profile_ = std::move(other.profile_);
            limits_ = std::move(other.limits_);
            config_ = std::move(other.config_);
            renurbsProfile_ = std::move(other.renurbsProfile_);
            currentFeedOverride_ = other.currentFeedOverride_;
            timeOffset_ = other.timeOffset_;
            isReverse_ = other.isReverse_;
            lastState_ = std::move(other.lastState_);
            sourceRef_ = std::move(other.sourceRef_);
            updateAnalyticalPathPointer();
        }
        return *this;
    }

    /**
     * @brief Construct from path and velocity profile
     */
    MotionPlan(Path path, std::shared_ptr<VelocityProfile> profile, Config config = {})
        : path_(std::move(path))
        , profile_(std::move(profile))
        , config_(std::move(config)) {}

    /**
     * @brief Construct from path, velocity profile, and kinematic limits
     *
     * The limits are stored for reference and downstream use (e.g. by
     * the motion replanner). The velocity profile is consumed as-is —
     * no post-hoc smoothing is applied. The profiler that produced the
     * profile is responsible for respecting all constraints.
     */
    MotionPlan(Path path, std::shared_ptr<VelocityProfile> profile, Limits limits, Config config = {})
        : path_(std::move(path))
        , profile_(std::move(profile))
        , limits_(std::move(limits))
        , config_(std::move(config)) {}

    /**
     * @brief Evaluate complete motion state at time t
     *
     * This is the primary query interface. It reads velocity, acceleration,
     * and jerk directly from the velocity profile (which was produced by
     * an VelocityProfiler). No post-hoc smoothing or finite-difference
     * estimation is performed — the profile's values are used as-is,
     * preserving the constraints verified by the profiler.
     *
     * If an analytical trajectory source is available (from the
     * AnalyticalJerkLimitedTOPPRA profiler), it is used for exact/certified sampling
     * of position, velocity, and acceleration. Otherwise, the tabulated
     * velocity profile is used (backward-compatible behavior).
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
            effectiveTime = profile_->totalTime() - effectiveTime;
        }

        // Clamp to valid range
        effectiveTime = clamp(effectiveTime, T(0), profile_->totalTime());

        // Use tabulated profile sampling. The analytical source (if present)
        // is available via analyticalSource() for consumers that need
        // exact/certified sampling, but is not used internally because
        // it may reference a path pointer that becomes stale after a move.
        return evaluateAtProfiled(effectiveTime);
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
            effectiveTime = profile_->totalTime() - effectiveTime;
        }

        // Use tabulated profile (the analytical source may reference
        // a path pointer that is stale after a move; the tabulated
        // profile is always valid).
        T arcLength = profile_->arcLengthAt(effectiveTime);
        return path_.evaluateAtArcLength(arcLength).position;
    }

    /**
     * @brief Get total duration of motion plan
     */
    T totalDuration() const {
        if (path_.numSegments() == 0 || !profile_) {
            return T(0);
        }
        return profile_->totalTime() / std::max(currentFeedOverride_, T(0.001));
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
        return profile_->timeAt(*arcLength);
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
     * @brief Access the analytical trajectory source, if any.
     *
     * Returns the underlying AnalyticalTrajectorySource when the velocity
     * profile is an AnalyticalSSRVelocityProfile (e.g. from AnalyticalJerkLimitedTOPPRA
     * or ParetoTimeEnergyOptimalVelocityPlanner). Returns nullptr for
     * sampled/tabulated profiles.
     */
    std::shared_ptr<analytical::AnalyticalTrajectorySource<Dim, T>>
    analyticalSource() const {
        using AnalyticalProfile =
            analytical::AnalyticalSSRVelocityProfile<Dim, T>;
        if (!profile_) return nullptr;
        auto* avp = dynamic_cast<const AnalyticalProfile*>(profile_.get());
        if (!avp) return nullptr;
        return avp->source();
    }

    /**
     * @brief Access configuration
     */
    const Config& config() const { return config_; }

    /**
     * @brief Access kinematic limits
     */
    const Limits& limits() const { return limits_; }

    /**
     * @brief Access the optional ReNURBS profile (analytical NURBS
     *        representation of v(s), a(s), j(s), t(s)).
     * @return std::nullopt if ReNURBS was not enabled in the config.
     */
    const std::optional<tether::motion::profile_renurbs::ReNURBSProfile>&
    renurbsProfile() const { return renurbsProfile_; }

    /**
     * @brief Set the ReNURBS profile (used by MotionPlanBuilder).
     */
    void setRenurbsProfile(tether::motion::profile_renurbs::ReNURBSProfile p) {
        renurbsProfile_ = std::move(p);
    }

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
     * @brief Update the analytical trajectory source's path pointer after a
     * move. The SSR stores a raw pointer to the path; when the MotionPlan is
     * moved, the path's address changes, so the pointer must be refreshed.
     */
    void updateAnalyticalPathPointer() {
        auto source = analyticalSource();
        if (source) {
            auto sampler = std::dynamic_pointer_cast<
                analytical::TrajectorySampler<Dim, T>>(source);
            if (sampler) {
                sampler->setPath(path_);
            }
        }
    }

    /**
     * @brief Compute effective time accounting for feed override
     */
    T computeEffectiveTime(T t) const {
        return (t - timeOffset_) * currentFeedOverride_;
    }

    /**
     * @brief Evaluate state using the tabulated velocity profile.
     * This is the original (backward-compatible) evaluation path.
     */
    State evaluateAtProfiled(T effectiveTime) const {
        State state;
        state.feedOverride = currentFeedOverride_;
        state.isReverse = isReverse_;

        // Get arc length at this time
        state.arcLength = profile_->arcLengthAt(effectiveTime);

        // Evaluate path geometry
        auto pathEval = path_.evaluateAtArcLength(state.arcLength);
        state.position = pathEval.position;
        state.segmentIndex = pathEval.segmentIndex;
        state.segmentParameter = pathEval.localParameter;

        // Get velocity from profile
        state.pathVelocity = profile_->velocityAt(state.arcLength);
        state.pathVelocity *= currentFeedOverride_;

        // Reverse direction if needed
        if (isReverse_) {
            state.pathVelocity = -state.pathVelocity;
        }

        // Compute velocity vector
        state.velocity = pathEval.tangent * state.pathVelocity;

        // Get curvature
        state.curvature = path_.curvatureAtArcLength(state.arcLength);

        // Get acceleration and jerk directly from the profile.
        state.pathAcceleration = profile_->accelerationAt(state.arcLength) * currentFeedOverride_;
        // Second-order profiles intentionally do not define jerk at their
        // acceleration switches. Keep the legacy state field at zero, but do
        // not manufacture it by querying the sampled compatibility value.
        state.pathJerk = profile_->hasJerk()
            ? profile_->jerkAt(state.arcLength) * currentFeedOverride_
            : T(0);

        // Tangential acceleration
        Point tangentialAccel = pathEval.tangent * state.pathAcceleration;

        // Centripetal acceleration: a_cent = v² · κ
        T v = std::abs(state.pathVelocity);
        T centripetalMag = v * v * state.curvature;
        Point normal = computeNormal(pathEval.tangent, state.arcLength);
        Point centripetalAccel = normal * centripetalMag;

        state.acceleration = tangentialAccel + centripetalAccel;

        // Jerk vector (tangential component from profile)
        state.jerk = pathEval.tangent * state.pathJerk;

        // Get source reference
        state.sourceRef = path_.sourceRefAtArcLength(state.arcLength);

        return state;
    }

    /**
     * @brief Compute normal vector at arc length
     */
    Point computeNormal(const Point& tangent, T arcLength) const {
        if constexpr (Dim == 2) {
            return Point{-tangent[1], tangent[0]};
        } else if constexpr (Dim >= 3) {
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
    Limits limits_;
    Config config_;

    /// Optional ReNURBS profile (populated if config_.renurbs.enabled).
    std::optional<tether::motion::profile_renurbs::ReNURBSProfile> renurbsProfile_;

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
 * @brief Builds MotionPlan from motion segments.
 *
 * The builder allows choosing a velocity profiling strategy via the
 * ProfilerType enum or by providing a custom VelocityProfiler instance.
 *
 * - ProfilerType::ParetoTimeEnergy: Configurable energy/time-optimal
 *   planner with tunable cost J = ∫[w_t + w_a·a²]dt. Produces smooth
 *   trajectories via Pontryagin's maximum principle (bang-singular-bang
 *   arc structure). This is the default — it recovers time-optimal
 *   behavior when w_a = 0 and smoothly trades time for energy as w_a
 *   increases.
 *
 * - ProfilerType::ToppraBasic: Basic 2nd-order TOPP-RA (no jerk limit).
 *   Fastest trajectory; acceleration is discontinuous at switching points.
 *
 * - ProfilerType::ToppraJerkConstrained: 3rd-order TOPP-RA with jerk as a
 *   constraint inside the optimizer. Continuous acceleration; bounded jerk.
 *   Slightly slower than basic TOPP-RA.
 *
 * - ProfilerType::SCurve: Basic per-piece S-curve profiles. Jerk-limited
 *   but not time-optimal. Simpler than TOPP-RA.
 */
template<size_t Dim, typename T = double>
class MotionPlanBuilder {
public:
    using Plan = MotionPlan<Dim, T>;
    using Path = PathAdapter<Dim, T>;
    using Config = MotionPlanConfig<T>;
    using Limits = KinematicLimits<Dim, T>;
    using IProfiler = VelocityProfiler<Dim, T>;

    /**
     * @brief Constructor with profiler type selection.
     * @param limits Kinematic limits.
     * @param config Motion plan configuration.
     * @param profilerType Which profiler to use (default: ParetoTimeEnergy).
     */
    MotionPlanBuilder(Limits limits = {}, Config config = {},
                      ProfilerType profilerType = ProfilerType::ParetoTimeEnergy)
        : limits_(std::move(limits))
        , config_(std::move(config))
        , profilerType_(profilerType) {}

    /**
     * @brief Constructor with custom profiler instance.
     *
     * Allows providing a fully-configured VelocityProfiler. The builder
     * takes ownership of the profiler.
     *
     * @param profiler Custom profiler instance.
     * @param limits Kinematic limits (used for the MotionPlan; the profiler
     *               has its own copy).
     * @param config Motion plan configuration.
     */
    MotionPlanBuilder(std::unique_ptr<IProfiler> profiler,
                      Limits limits = {}, Config config = {})
        : limits_(std::move(limits))
        , config_(std::move(config))
        , customProfiler_(std::move(profiler)) {}

    /**
     * @brief Build motion plan from segment list.
     *
     * @param segments Motion segments (from G-code parser)
     * @param feedRate Default feed rate
     * @return Complete motion plan
     */
    Plan build(const MotionSegmentList& segments, T feedRate) {
        // Build path with corner blending using the new geometry core.
        PathBuilderAdapter<Dim, T> pathBuilder;
        tether::motion::BlendSpec blendSpec;
        // Use a sensible default tolerance for blending. The BlendSpec
        // default is 0, which fails validation for multi-segment paths.
        blendSpec.tolerance = 0.1;  // 0.1 mm corner tolerance
        blendSpec.continuity = tether::motion::Continuity::G2;
        blendSpec.maxBlendFraction = 0.25;
        auto pathResult = pathBuilder.build(segments, blendSpec);

        if (!pathResult.success || pathResult.path.numSegments() == 0) {
            return Plan{};
        }

        // Compute velocity profile using the selected profiler.
        std::unique_ptr<VelocityProfile> profile;
        if (customProfiler_) {
            profile = customProfiler_->computeProfile(pathResult.path, feedRate);
        } else {
            auto profiler = createProfiler(profilerType_);
            profile = profiler->computeProfile(pathResult.path, feedRate);
        }

        // Create motion plan with the adapted path and kinematic limits.
        auto profilePtr = std::shared_ptr<VelocityProfile>(std::move(profile));
        Plan plan = Plan(std::move(pathResult.path), std::move(profilePtr),
                         limits_, config_);

        // Fix: The AnalyticalJerkLimitedTOPPRA profiler's SSR stores a raw pointer to
        // the path. When pathResult.path was moved into the plan above, that
        // pointer became dangling. Update it to point to the plan's path.
        auto source = plan.analyticalSource();
        if (source) {
            auto sampler = std::dynamic_pointer_cast<
                analytical::TrajectorySampler<Dim, T>>(source);
            if (sampler) {
                sampler->setPath(plan.path());
            }
        }

        // Build ReNURBS profile if enabled in config.
        if (config_.renurbs.enabled) {
            try {
                auto renurbsProfile = tether::motion::profile_renurbs::
                    buildReNURBSProfile<Dim, T>(
                        *plan.profile(), plan.path(), limits_,
                        config_.renurbs);
                plan.setRenurbsProfile(std::move(renurbsProfile));
            } catch (const tether::motion::profile_renurbs::
                     ReNURBSCertificationError&) {
                // Certification failed; the plan is still valid with
                // just the sampled profile. The error is propagated to
                // the caller via the empty renurbsProfile_ optional.
            }
        }

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

    /**
     * @brief Get/set the profiler type (ignored if custom profiler is set).
     */
    ProfilerType profilerType() const { return profilerType_; }
    void setProfilerType(ProfilerType type) { profilerType_ = type; }

private:
    Limits limits_;
    Config config_;
    ProfilerType profilerType_ = ProfilerType::ParetoTimeEnergy;
    std::unique_ptr<IProfiler> customProfiler_;

    /// Create a profiler instance for the given type.
    std::unique_ptr<IProfiler> createProfiler(ProfilerType type) {
        switch (type) {
            case ProfilerType::ToppraBasic:
                return std::make_unique<BasicTOPPRA<Dim, T>>(limits_);
            case ProfilerType::ToppraJerkConstrained:
                return std::make_unique<JerkConstrainedTOPPRA<Dim, T>>(limits_);
            case ProfilerType::SCurve:
                return std::make_unique<SCurveVelocityProfiler<Dim, T>>(limits_);
            case ProfilerType::AnalyticalTOPPRA:
                return std::make_unique<analytical::AnalyticalTOPPRA<Dim, T>>(limits_);
            case ProfilerType::AnalyticalJerkLimitedTOPPRA:
                return std::make_unique<analytical::AnalyticalJerkLimitedTOPPRA<Dim, T>>(limits_);
            case ProfilerType::ParetoTimeEnergy:
            default:
                return std::make_unique<analytical::ParetoTimeEnergyOptimalVelocityPlanner<Dim, T>>(limits_);
        }
    }
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
