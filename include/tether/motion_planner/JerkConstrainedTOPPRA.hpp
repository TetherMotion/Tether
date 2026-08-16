/**
 * @file JerkConstrainedTOPPRA.hpp
 * @brief Jerk-limited TOPP-RA velocity profiler (3rd-order, state-carrying).
 *
 * @details
 * This profiler extends the basic TOPP-RA algorithm to include jerk as a
 * first-class constraint inside the optimization. Instead of post-hoc
 * S-curve smoothing (which breaks both optimality and feasibility), the
 * jerk limit is enforced during the forward and backward passes, producing
 * a feasible, jerk-bounded profile with:
 *
 * - **Continuous acceleration** (no step changes at switching points)
 * - **Bounded jerk** (|s⃛| ≤ j_max everywhere)
 * - **All original constraints verified** (velocity, acceleration, curvature)
 *
 * ## Algorithm (WI-8 Option B: true 3rd-order TOPP-RA)
 *
 * The basic TOPP-RA uses the 2nd-order kinematic equation:
 *   v² = v₀² + 2·a_max·Δs
 * which produces bang-bang acceleration (instantaneous switching between
 * ±a_max). The jerk-limited version replaces this with the **state-aware**
 * jerk-limited distance function from SCurveProfile:
 *   Δs = computeAccelDistanceWithState(v₀, a₀, v₁, a_max, j_max)
 * which accounts for the finite time needed to ramp acceleration up/down
 * **without forcing a = 0 at every sample point**.
 *
 * The key difference from the previous (pre-WI-8) implementation is that
 * the acceleration is carried as state in both passes. The previous
 * implementation used computeAccelDistance (symmetric S-curve starting
 * AND ending at a=0), which implicitly forced a=0 at every sample —
 * making the profile increasingly suboptimal as numSamples grew.
 *
 * ### Backward Pass (jerk-limited deceleration, state-carrying)
 *
 * Sweeping from the end of the path backward, the state (v, a) is carried
 * backward. At each sample i we compute the maximum entry velocity v_i
 * and entry acceleration a_i such that we can decelerate from (v_i, a_i)
 * to (v_{i+1}, a_{i+1}) over distance Δs, respecting jerk limits:
 *   (v_i, a_i) = maxEntryVelocityWithState(v_{i+1}, a_{i+1}, Δs, v_lim, a_max, j_max)
 * The result is also capped by v_lim(s_i).
 *
 * ### Forward Pass (jerk-limited acceleration, state-carrying)
 *
 * Sweeping from the start forward, the state (v, a) is carried forward.
 * At each sample i we compute the maximum velocity v_i and acceleration
 * a_i such that we can accelerate from (v_{i-1}, a_{i-1}) to (v_i, a_i)
 * over distance Δs, respecting jerk limits:
 *   (v_i, a_i) = maxVelocityWithState(v_{i-1}, a_{i-1}, Δs, v_lim, a_max, j_max)
 * The forward pass is also capped by the backward pass (to ensure we can
 * still stop in time).
 *
 * ### Switching Points
 *
 * At switching points where the forward and backward passes cross, the
 * acceleration transitions from a_fwd (positive, accelerating) to a_bwd
 * (negative, decelerating). This transition is jerk-limited by a
 * forward-backward smoothing pass on the acceleration profile that
 * enforces |Δa/Δt| ≤ j_max.
 *
 * ### Final Profile
 *
 * v(s) = min(forward, backward, v_lim). The acceleration at each point
 * is taken from the binding constraint (forward pass, backward pass, or
 * velocity limit), then jerk-limited smoothed. The jerk is computed from
 * the acceleration change over time — **not clamped** (WI-3: stored
 * values are reported truthfully).
 *
 * ## Time Optimality
 *
 * The state-carrying implementation is time-optimal subject to the jerk
 * constraint, up to the discretization error introduced by the sample
 * grid and the switching-point smoothing. The total time is approximately
 * independent of numSamples (unlike the pre-WI-8 implementation, whose
 * time grew with numSamples).
 *
 * @see BasicTOPPRA.hpp for the basic TOPP-RA profiler.
 * @see SCurveProfile.hpp for the state-aware jerk-limited distance functions.
 * @see VelocityProfiler.hpp for the abstract interface.
 */

#pragma once

#include "VelocityProfile.hpp"
#include "VelocityProfiler.hpp"
#include "BasicTOPPRA.hpp"
#include "SCurveProfile.hpp"
#include "PathAdapter.hpp"
#include <tether/motion_planner/geometry/CertifiedCurvatureSampler.hpp>
#include <tether/motion_planner/blend/PHQuinticBlendBuilder.hpp>

#include <algorithm>
#include <cmath>
#include <vector>

namespace MotionPlanner {

/**
 * @brief Jerk-limited TOPP-RA velocity profiler (3rd-order, state-carrying).
 *
 * Implements VelocityProfiler. Produces a feasible, jerk-bounded velocity
 * profile with jerk as a first-class constraint inside the optimization.
 * The acceleration is carried as state in both passes (WI-8 Option B),
 * making the profile approximately time-optimal subject to the jerk
 * constraint and approximately independent of the sample count.
 */
template<size_t Dim, typename T = double>
class JerkConstrainedTOPPRA : public VelocityProfiler<Dim, T> {
public:
    using Path = PathAdapter<Dim, T>;
    using Profile = VelocityProfile<T>;
    using Limits = KinematicLimits<Dim, T>;
    using Point = VelocityProfilePoint<T>;
    using SCurve = SCurveProfile<T>;

    /**
     * @brief Constructor.
     * @param limits Kinematic limits (must have valid path jerk limit).
     */
    explicit JerkConstrainedTOPPRA(Limits limits = {})
        : limits_(std::move(limits)) {}

    /**
     * @brief Compute a jerk-limited velocity profile for the given path.
     *
     * The profile respects:
     * - Per-axis velocity, acceleration, and jerk limits (WI-2)
     * - Path-level velocity, acceleration, and jerk limits
     * - Centripetal acceleration (curvature) limits
     * - Feed rate
     * - Junction velocity at tangent discontinuities (WI-4)
     *
     * Acceleration is continuous throughout; jerk is bounded by j_max.
     * The acceleration and jerk fields are reported truthfully (WI-3:
     * no clamping of stored jerk).
     */
    Profile computeProfile(
        const Path& path,
        T feedRate,
        T startVelocity = T(0),
        T endVelocity = T(0),
        size_t numSamples = 100,
        T startAcceleration = T(0),
        T startJerk = T(0)) override {

        (void)startJerk; // WI-P3: stored on first point only; not honored
                         // in the optimization (assumes a(0) = 0).
        Profile profile;
        if (path.numSegments() == 0) return profile;

        // WI-1: Validate inputs — degenerate configs must return an empty
        // (or all-rest) profile, never NaN or a velocity jump.
        if (numSamples < 2) return profile;          // ds would divide by 0
        if (feedRate <= T(0)) return profile;         // no motion commanded
        if (limits_.path.maxPathAcceleration <= T(0)) return profile;
        if (limits_.path.maxCentripetalAcceleration < T(0)) return profile;

        T pathLength = path.totalLength();
        if (pathLength <= T(0)) return profile;

        // Jerk-limited profiling requires a valid jerk limit.
        const T pathJMax = limits_.path.maxPathJerk;
        const T pathAMax = limits_.path.maxPathAcceleration;

        // WI-1: fall back to BasicTOPPRA also when jMax <= 0 (not just when
        // jerkLimitEnabled is false), and never enter the jerk passes with
        // aMax <= 0 (already guarded above, but double-check).
        const bool jerkEnabled = limits_.path.jerkLimitEnabled && pathJMax > T(0);

        // If jerk limiting is disabled, delegate to the basic profiler.
        if (!jerkEnabled) {
            BasicTOPPRA<Dim, T> basic(limits_);
            return basic.computeProfile(path, feedRate, startVelocity,
                                        endVelocity, numSamples,
                                        startAcceleration, startJerk);
        }

        // --- Sample path at uniform arc length intervals ---
        const auto* curvatureSampler = (path.hasInner())
            ? &path.curvatureSampler() : nullptr;

        T ds = pathLength / T(numSamples - 1);
        std::vector<PathSample> samples(numSamples);
        for (size_t i = 0; i < numSamples; ++i) {
            T s = std::min(i * ds, pathLength);
            samples[i].arcLength = s;
            auto eval = path.evaluateAtArcLength(s);
            samples[i].position = eval.position;
            samples[i].tangent = eval.tangent;
            samples[i].segmentIndex = eval.segmentIndex;

            // PH fast path for curvature (same as basic profiler)
            bool usedPH = false;
            if (path.hasInner() && path.hasPHData()) {
                const auto& inner = path.inner();
                auto loc = inner.locate(static_cast<double>(s));
                const auto& ph = path.phData(loc.piece);
                if (ph) {
                    const double localS = loc.localS;
                    const double xi = tether::motion::PHQuinticBlendBuilder::invertArcLength(*ph, localS);
                    samples[i].curvature = static_cast<T>(
                        tether::motion::PHQuinticBlendBuilder::curvature(*ph, xi));
                    usedPH = true;
                }
            }
            if (!usedPH) {
                if (curvatureSampler) {
                    // WI-5: Use certified per-span max curvature (conservative).
                    auto cert = curvatureSampler->maxCurvatureAtArcLength(static_cast<double>(s));
                    samples[i].curvature = static_cast<T>(cert.maxKappa);
                } else {
                    samples[i].curvature = path.curvatureAtArcLength(s);
                }
            }
        }

        // --- WI-2: Compute per-sample effective acceleration and jerk limits ---
        // Per-axis acceleration limits are folded in via
        // maxAccelerationForDirection (same as BasicTOPPRA). Per-axis jerk
        // limits are projected onto the tangent direction.
        std::vector<T> aMaxSample(numSamples);
        std::vector<T> jMaxSample(numSamples);
        for (size_t i = 0; i < numSamples; ++i) {
            // Use velocity estimate of 0 for the acceleration budget
            // (conservative — centripetal load is handled by v_lim).
            aMaxSample[i] = limits_.maxAccelerationForDirection(
                samples[i].tangent, samples[i].curvature, T(0));
            // Per-axis jerk projection: jMax_dir = min_i(axis.maxJerk[i] / |t_i|)
            // over axes with |t_i| > EPSILON, when jerkLimitEnabled.
            T jMaxDir = pathJMax;
            if (limits_.axis.jerkLimitEnabled) {
                for (size_t a = 0; a < Dim; ++a) {
                    if (std::abs(samples[i].tangent[a]) > MathConstants::EPSILON) {
                        T axisJerk = limits_.axis.maxJerk[a] / std::abs(samples[i].tangent[a]);
                        jMaxDir = std::min(jMaxDir, axisJerk);
                    }
                }
            }
            jMaxSample[i] = std::max(jMaxDir, T(1e-12)); // guard against 0
        }

        // --- Velocity limit curve v_lim(s) ---
        // WI-4: Insert junction velocity at tangent discontinuities (exact
        // path mode — no blending). When two adjacent samples cross a piece
        // boundary with a tangent discontinuity, force v = 0 at the boundary
        // (exact-stop semantics, matching LinuxCNC "exact path" mode).
        std::vector<T> vLim(numSamples);
        for (size_t i = 0; i < numSamples; ++i) {
            // WI-5: Use interval-max curvature to close the between-sample
            // curvature gap.
            PathSample intervalSample = samples[i];
            if (curvatureSampler && !path.hasPHData()) {
                T sPrev = (i > 0) ? samples[i - 1].arcLength : samples[i].arcLength;
                T sNext = (i + 1 < numSamples) ? samples[i + 1].arcLength : samples[i].arcLength;
                auto cert = curvatureSampler->maxCurvatureOverInterval(
                    static_cast<double>(sPrev), static_cast<double>(sNext));
                intervalSample.curvature = static_cast<T>(cert.maxKappa);
            }
            vLim[i] = computeVelocityLimit(intervalSample, feedRate);
        }

        // WI-4: Junction velocity at piece boundaries with tangent discontinuity.
        for (size_t i = 1; i < numSamples; ++i) {
            if (samples[i].segmentIndex != samples[i - 1].segmentIndex) {
                T junctionVel = computeJunctionVelocity(
                    samples[i - 1].tangent, samples[i].tangent);
                if (junctionVel < vLim[i]) vLim[i] = junctionVel;
                if (junctionVel < vLim[i - 1]) vLim[i - 1] = junctionVel;
            }
        }

        // ================================================================
        // WI-8: Backward pass with (v, a) state carrying
        // ================================================================
        // Sweep from end to start. State (v, a) is carried backward.
        // At each sample i, compute the maximum entry velocity v_i and
        // entry acceleration a_i such that we can decelerate from
        // (v_i, a_i) to (v_{i+1}, a_{i+1}) over distance Δs.
        std::vector<T> bwdVel(numSamples);
        std::vector<T> bwdAccel(numSamples);

        bwdVel[numSamples - 1] = std::min(endVelocity, vLim[numSamples - 1]);
        bwdAccel[numSamples - 1] = T(0); // end at rest

        for (size_t i = numSamples - 1; i > 0; --i) {
            T deltaS = samples[i].arcLength - samples[i - 1].arcLength;
            // Per-interval limits: conservative min over both endpoints.
            T aMaxInt = std::min(aMaxSample[i], aMaxSample[i - 1]);
            T jMaxInt = std::min(jMaxSample[i], jMaxSample[i - 1]);
            aMaxInt = std::min(aMaxInt, pathAMax);
            jMaxInt = std::min(jMaxInt, pathJMax);

            // Max entry velocity that allows reaching (bwdVel[i], bwdAccel[i])
            auto [v0, a0] = SCurve::maxEntryVelocityWithState(
                bwdVel[i], bwdAccel[i], deltaS, vLim[i - 1],
                aMaxInt, jMaxInt);

            bwdVel[i - 1] = std::min(v0, vLim[i - 1]);
            // If capped by v_lim, the entry acceleration should transition
            // toward 0 (cruise). Use the function's result if not capped,
            // otherwise blend toward 0.
            if (bwdVel[i - 1] < v0) {
                bwdAccel[i - 1] = T(0); // cruising at v_lim
            } else {
                bwdAccel[i - 1] = a0;
            }
        }

        // ================================================================
        // WI-8: Forward pass with (v, a) state carrying
        // ================================================================
        // Sweep from start to end. State (v, a) is carried forward.
        // At each sample i, compute the maximum velocity v_i and
        // acceleration a_i such that we can accelerate from
        // (v_{i-1}, a_{i-1}) to (v_i, a_i) over distance Δs.
        std::vector<T> fwdVel(numSamples);
        std::vector<T> fwdAccel(numSamples);
        std::vector<typename Point::LimitType> fwdCause(numSamples);

        fwdVel[0] = std::min(startVelocity, vLim[0]);
        fwdAccel[0] = startAcceleration;
        fwdCause[0] = Point::LimitType::None;

        for (size_t i = 1; i < numSamples; ++i) {
            T deltaS = samples[i].arcLength - samples[i - 1].arcLength;
            T aMaxInt = std::min(aMaxSample[i], aMaxSample[i - 1]);
            T jMaxInt = std::min(jMaxSample[i], jMaxSample[i - 1]);
            aMaxInt = std::min(aMaxInt, pathAMax);
            jMaxInt = std::min(jMaxInt, pathJMax);

            // Max velocity reachable from (fwdVel[i-1], fwdAccel[i-1])
            auto [vMax, aMax] = SCurve::maxVelocityWithState(
                fwdVel[i - 1], fwdAccel[i - 1], deltaS, vLim[i],
                aMaxInt, jMaxInt);

            // WI-7: Record which constraint is binding.
            T v = vMax;
            auto cause = Point::LimitType::ForwardAccel;
            if (vLim[i] < v) {
                v = vLim[i];
                cause = Point::LimitType::Curvature; // or FeedRate/AxisVelocity
            }
            if (bwdVel[i] < v) {
                v = bwdVel[i];
                cause = Point::LimitType::BackwardDecel;
            }

            fwdVel[i] = v;
            fwdCause[i] = cause;

            // Determine acceleration based on binding constraint.
            if (cause == Point::LimitType::ForwardAccel) {
                fwdAccel[i] = aMax;
            } else if (cause == Point::LimitType::BackwardDecel) {
                // Transition to backward pass acceleration.
                // Jerk-limited transition: limit the acceleration change.
                T dtEst = (deltaS > T(0) && v > T(0))
                    ? deltaS / v : T(1e-6);
                T maxAChange = jMaxInt * dtEst;
                T targetAccel = bwdAccel[i];
                fwdAccel[i] = std::clamp(targetAccel,
                    fwdAccel[i - 1] - maxAChange,
                    fwdAccel[i - 1] + maxAChange);
            } else {
                // Velocity limit binding — cruise at a = 0.
                T dtEst = (deltaS > T(0) && v > T(0))
                    ? deltaS / v : T(1e-6);
                T maxAChange = jMaxInt * dtEst;
                fwdAccel[i] = std::clamp(T(0),
                    fwdAccel[i - 1] - maxAChange,
                    fwdAccel[i - 1] + maxAChange);
            }
        }

        // ================================================================
        // Final profile: merge + jerk-limited smoothing + time + jerk
        // ================================================================

        // Determine raw acceleration at each sample from the binding
        // constraint (WI-3: analytic from carried state, not finite diff).
        std::vector<T> rawAccel(numSamples, T(0));
        std::vector<typename Point::LimitType> cause(numSamples);

        for (size_t i = 0; i < numSamples; ++i) {
            T vf = fwdVel[i];
            T vb = bwdVel[i];
            T vl = vLim[i];
            T v = std::min({vf, vb, vl});

            if (v == vf && vf <= vb && vf <= vl) {
                rawAccel[i] = fwdAccel[i];
                cause[i] = Point::LimitType::ForwardAccel;
            } else if (v == vb && vb <= vl) {
                rawAccel[i] = bwdAccel[i];
                cause[i] = Point::LimitType::BackwardDecel;
            } else {
                rawAccel[i] = T(0); // velocity-limited: cruise
                cause[i] = Point::LimitType::Curvature;
            }

            // Override with the forward pass cause when available (WI-7:
            // the forward pass records the cause during the pass, which is
            // more accurate than re-deriving with float equality).
            if (i > 0 && fwdVel[i] <= bwdVel[i] && fwdVel[i] <= vLim[i]) {
                cause[i] = fwdCause[i];
            }
        }
        rawAccel[0] = startAcceleration;
        cause[0] = Point::LimitType::None;

        // Compute time from the merged velocity profile (trapezoidal).
        std::vector<T> times(numSamples, T(0));
        for (size_t i = 1; i < numSamples; ++i) {
            T vPrev = std::min({fwdVel[i - 1], bwdVel[i - 1], vLim[i - 1]});
            T vCurr = std::min({fwdVel[i], bwdVel[i], vLim[i]});
            T avgVel = (vPrev + vCurr) / T(2);
            T deltaS = samples[i].arcLength - samples[i - 1].arcLength;
            if (avgVel > MathConstants::EPSILON) {
                times[i] = times[i - 1] + deltaS / avgVel;
            } else {
                times[i] = times[i - 1];
            }
        }

        // Jerk-limited smoothing of the acceleration profile (WI-8:
        // switching-point transitions). Forward-backward pass to enforce
        // |Δa/Δt| ≤ jMax in both directions.
        std::vector<T> smoothAccel(numSamples, T(0));
        smoothAccel[0] = rawAccel[0];

        // Forward pass: limit jerk going forward.
        for (size_t i = 1; i < numSamples; ++i) {
            T dt = times[i] - times[i - 1];
            if (dt < MathConstants::EPSILON) dt = MathConstants::EPSILON;
            T jMaxInt = std::min(jMaxSample[i], jMaxSample[i - 1]);
            jMaxInt = std::min(jMaxInt, pathJMax);
            T maxAChange = jMaxInt * dt;
            smoothAccel[i] = std::clamp(rawAccel[i],
                smoothAccel[i - 1] - maxAChange,
                smoothAccel[i - 1] + maxAChange);
        }

        // Backward pass: limit jerk going backward (ensures end condition).
        std::vector<T> finalAccel(numSamples, T(0));
        finalAccel[numSamples - 1] = smoothAccel[numSamples - 1];
        for (size_t i = numSamples - 1; i > 0; --i) {
            T dt = times[i] - times[i - 1];
            if (dt < MathConstants::EPSILON) dt = MathConstants::EPSILON;
            T jMaxInt = std::min(jMaxSample[i], jMaxSample[i - 1]);
            jMaxInt = std::min(jMaxInt, pathJMax);
            T maxAChange = jMaxInt * dt;
            finalAccel[i - 1] = std::clamp(smoothAccel[i - 1],
                finalAccel[i] - maxAChange,
                finalAccel[i] + maxAChange);
        }
        finalAccel[0] = startAcceleration;

        // Build the final profile.
        profile.reserve(numSamples);
        for (size_t i = 0; i < numSamples; ++i) {
            Point pt;
            pt.arcLength = samples[i].arcLength;
            pt.velocity = std::min({fwdVel[i], bwdVel[i], vLim[i]});
            pt.time = times[i];
            pt.acceleration = finalAccel[i];
            pt.limitedBy = cause[i];
            pt.velocityLimit = vLim[i];
            pt.accelerationLimit = pathAMax;

            // WI-3: Jerk from acceleration change over time — NOT clamped.
            // The jerk-limited smoothing ensures |j| ≤ jMax by construction;
            // any residual is reported truthfully so violations surface.
            if (i > 0) {
                T dt = times[i] - times[i - 1];
                if (dt > MathConstants::EPSILON) {
                    pt.jerk = (finalAccel[i] - finalAccel[i - 1]) / dt;
                }
            }

            profile.addPoint(pt);
        }

        return profile;
    }

    Limits limits() const override { return limits_; }
    ProfilerType type() const override { return ProfilerType::ToppraJerkConstrained; }
    const char* name() const override {
        return "JerkConstrainedTOPPRA (TOPP-RA + jerk constraint, 3rd-order)";
    }

private:
    /// Internal path sample
    struct PathSample {
        T arcLength = T(0);
        Vec<Dim, T> position;
        Vec<Dim, T> tangent;
        T curvature = T(0);
        size_t segmentIndex = 0;
    };

    /// Compute velocity limit at a path sample (same as basic profiler)
    T computeVelocityLimit(const PathSample& sample, T feedRate) const {
        T limit = feedRate;
        // WI-1: guard against negative maxCentripetalAcceleration (NaN).
        if (sample.curvature > MathConstants::EPSILON &&
            limits_.path.maxCentripetalAcceleration > T(0)) {
            T curvatureLimit = std::sqrt(
                limits_.path.maxCentripetalAcceleration / sample.curvature);
            limit = std::min(limit, curvatureLimit);
        }
        T axisLimit = limits_.maxVelocityForDirection(sample.tangent);
        limit = std::min(limit, axisLimit);
        limit = std::min(limit, limits_.path.maxPathVelocity);
        return limit;
    }

    /// WI-4: Compute junction velocity at a tangent discontinuity.
    /// Currently implements exact-stop semantics (v = 0 at corners),
    /// matching LinuxCNC "exact path" mode.
    T computeJunctionVelocity(const Vec<Dim, T>& tPrev,
                                const Vec<Dim, T>& tCur) const {
        T dot = T(0);
        for (size_t i = 0; i < Dim; ++i) dot += tPrev[i] * tCur[i];
        dot = std::clamp(dot, T(-1), T(1));
        T angle = std::acos(dot);
        if (angle < T(1e-6)) {
            // Tangents are parallel — no junction.
            return std::numeric_limits<T>::infinity();
        }
        // Exact-stop: velocity must be zero at the corner.
        return T(0);
    }

    Limits limits_;
};

} // namespace MotionPlanner
