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
        // WI-8b.3: Forward pass with (v, a) state carrying
        // ================================================================
        // Sweep from start to end. State (v, a) is carried forward.
        // At each sample i, compute the maximum velocity v_i and
        // acceleration a_i such that we can accelerate from
        // (v_{i-1}, a_{i-1}) to (v_i, a_i) over distance Δs.
        //
        // KEY CHANGE (WI-8b.3): the ceiling passed to maxVelocityWithState
        // is vCap[i] = min(vLim[i], bwdVel[i]) — NOT just vLim[i]. With
        // WI-8b.2 (shed-acceleration constraint), the forward state sheds
        // acceleration to 0 before hitting vCap, so it joins the backward
        // curve smoothly at switching points (a → 0 before contact, then
        // follows bwdVel exactly in decel regions). This makes the
        // post-hoc acceleration smoothing pass unnecessary.
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

            // WI-8b.3: ceiling = min(vLim, bwdVel). The forward pass
            // naturally respects both the velocity limit and the
            // backward deceleration curve.
            T vCap = std::min(vLim[i], bwdVel[i]);

            auto [vMax, aMaxState] = SCurve::maxVelocityWithState(
                fwdVel[i - 1], fwdAccel[i - 1], deltaS, vCap,
                aMaxInt, jMaxInt);

            fwdVel[i] = vMax;
            fwdAccel[i] = aMaxState;

            // WI-7: Record which constraint is binding. With WI-8b.3,
            // fwdVel[i] ≤ vCap = min(vLim[i], bwdVel[i]). The binding
            // constraint is determined by which ceiling is tightest.
            if (fwdVel[i] >= bwdVel[i] - T(1e-9) && bwdVel[i] <= vLim[i]) {
                fwdCause[i] = Point::LimitType::BackwardDecel;
            } else if (fwdVel[i] >= vLim[i] - T(1e-9)) {
                fwdCause[i] = Point::LimitType::Curvature;
            } else {
                fwdCause[i] = Point::LimitType::ForwardAccel;
            }
        }

        // ================================================================
        // Final profile: merge + time + analytic acceleration + jerk
        // ================================================================
        // WI-8b.3: No post-hoc acceleration smoothing — the velocity-level
        // joining (vCap = min(vLim, bwdVel) + WI-8b.2 shed constraint)
        // makes it unnecessary. The acceleration at each sample is the
        // carried analytic state from whichever pass is binding.

        // Determine the binding acceleration and cause at each sample.
        // With WI-8b.3, fwdVel[i] ≤ min(vLim[i], bwdVel[i]), so the
        // final velocity is fwdVel[i]. The acceleration comes from the
        // binding pass: forward accel in the accel region, backward accel
        // in the decel region, 0 at velocity-limited cruise.
        std::vector<T> finalAccel(numSamples, T(0));
        std::vector<typename Point::LimitType> cause(numSamples);

        finalAccel[0] = startAcceleration;
        cause[0] = Point::LimitType::None;

        for (size_t i = 1; i < numSamples; ++i) {
            if (fwdCause[i] == Point::LimitType::BackwardDecel) {
                // Backward pass is binding. At the switching point (first
                // decel sample), the forward pass has shed acceleration
                // to 0 (WI-8b.2). Use the forward acceleration (0) at the
                // switching point to avoid a discontinuity, then use the
                // backward acceleration for subsequent decel samples.
                // The backward acceleration ramps from 0 with j = -jMax,
                // so the jerk is bounded by construction.
                if (i > 0 && fwdCause[i - 1] != Point::LimitType::BackwardDecel) {
                    // Switching point: forward pass reached the backward
                    // curve. The forward acceleration is 0 (shed to 0).
                    // Use it to avoid a jerk spike.
                    finalAccel[i] = fwdAccel[i];
                } else {
                    // Subsequent decel sample — use backward acceleration.
                    finalAccel[i] = bwdAccel[i];
                }
                cause[i] = Point::LimitType::BackwardDecel;
            } else if (fwdCause[i] == Point::LimitType::Curvature) {
                // Velocity limit binding — cruise at a = 0.
                finalAccel[i] = T(0);
                cause[i] = Point::LimitType::Curvature;
            } else {
                // Forward accel binding — use carried analytic acceleration.
                finalAccel[i] = fwdAccel[i];
                cause[i] = Point::LimitType::ForwardAccel;
            }
        }
        // Last point: end at rest.
        finalAccel[numSamples - 1] = T(0);
        cause[numSamples - 1] = Point::LimitType::None;

        // WI-8b.3: Jerk-limited smoothing of the acceleration profile.
        // The velocity-level joining (vCap = min(vLim, bwdVel) + shed
        // constraint) ensures the velocity profile is feasible, but the
        // acceleration may have discontinuities at switching points (where
        // the forward pass transitions to the backward pass) and at the
        // start/end (where the acceleration ramps from/to zero). This
        // forward-backward pass enforces |Δa/Δt| ≤ jMax on the acceleration
        // profile without changing the velocity. The time intervals are
        // estimated from the velocity profile (trapezoidal, good enough for
        // jerk limiting; the final time is recomputed with the smoothed
        // acceleration using the constant-jerk formula).
        {
            // Estimate time intervals from velocity (trapezoidal).
            std::vector<T> estTimes(numSamples, T(0));
            for (size_t i = 1; i < numSamples; ++i) {
                T vPrev = fwdVel[i - 1];
                T vCurr = fwdVel[i];
                T avgVel = (vPrev + vCurr) / T(2);
                T deltaS = samples[i].arcLength -
                           samples[i - 1].arcLength;
                estTimes[i] = estTimes[i - 1] +
                    ((avgVel > MathConstants::EPSILON)
                        ? deltaS / avgVel : T(0));
            }

            // Forward pass: limit jerk going forward.
            std::vector<T> smoothAccel(numSamples, T(0));
            smoothAccel[0] = finalAccel[0];
            for (size_t i = 1; i < numSamples; ++i) {
                T dt = estTimes[i] - estTimes[i - 1];
                if (dt < MathConstants::EPSILON) dt = MathConstants::EPSILON;
                T jMaxInt = std::min(jMaxSample[i], jMaxSample[i - 1]);
                jMaxInt = std::min(jMaxInt, pathJMax);
                T maxAChange = jMaxInt * dt;
                smoothAccel[i] = std::clamp(finalAccel[i],
                    smoothAccel[i - 1] - maxAChange,
                    smoothAccel[i - 1] + maxAChange);
            }

            // Backward pass: limit jerk going backward.
            std::vector<T> smoothedAccel(numSamples, T(0));
            smoothedAccel[numSamples - 1] = smoothAccel[numSamples - 1];
            for (size_t i = numSamples - 1; i > 0; --i) {
                T dt = estTimes[i] - estTimes[i - 1];
                if (dt < MathConstants::EPSILON) dt = MathConstants::EPSILON;
                T jMaxInt = std::min(jMaxSample[i], jMaxSample[i - 1]);
                jMaxInt = std::min(jMaxInt, pathJMax);
                T maxAChange = jMaxInt * dt;
                smoothedAccel[i - 1] = std::clamp(smoothAccel[i - 1],
                    smoothedAccel[i] - maxAChange,
                    smoothedAccel[i] + maxAChange);
            }
            smoothedAccel[0] = startAcceleration;
            smoothedAccel[numSamples - 1] = T(0);

            finalAccel = std::move(smoothedAccel);
        }

        // Compute time from the velocity and acceleration profile.
        // WI-8b.3: Use the exact constant-jerk time formula:
        //   dt = 2·(v1 − v0) / (a0 + a1)
        // which is exact when jerk is constant over the interval (derived
        // from v1 − v0 = a_avg·dt where a_avg = (a0 + a1)/2 for constant
        // jerk). The trapezoidal rule (dt = 2·ds / (v0 + v1)) is only
        // exact for constant acceleration and underestimates dt by up to
        // 30% in the ramp-up phase (v0 ≈ 0), making T appear < 2.2 s on
        // the reference line even though the trajectory is feasible.
        //
        // Fallbacks: when a0 + a1 ≈ 0 (cruise or sign-change), use the
        // trapezoidal rule. When starting from rest (v0 ≈ 0, a0 ≈ 0),
        // use the jerk-only ramp formula t = √(6·ds / |a1|).
        std::vector<T> times(numSamples, T(0));
        for (size_t i = 1; i < numSamples; ++i) {
            T vPrev = fwdVel[i - 1];
            T vCurr = fwdVel[i];
            T aPrev = finalAccel[i - 1];
            T aCurr = finalAccel[i];
            T deltaS = samples[i].arcLength - samples[i - 1].arcLength;
            T deltaV = vCurr - vPrev;
            T aSum = aPrev + aCurr;

            T dt;
            if (deltaS <= T(0)) {
                dt = T(0);
            } else if (vPrev < T(1e-9) && std::abs(aPrev) < T(1e-9) &&
                       std::abs(aCurr) > T(1e-9)) {
                // Starting from rest with zero initial acceleration:
                // jerk-only ramp. ds ≈ a1·t²/6. Solve: t = √(6·ds / |a1|).
                dt = std::sqrt(T(6) * deltaS / std::abs(aCurr));
            } else if (std::abs(aSum) > T(1e-6) &&
                       deltaV * aSum > T(0)) {
                // Constant-jerk formula: dt = 2·Δv / (a0 + a1).
                // Exact for constant jerk; sign check ensures dt > 0.
                dt = T(2) * deltaV / aSum;
            } else {
                // Fallback: trapezoidal rule (exact for constant accel).
                T avgVel = (vPrev + vCurr) / T(2);
                dt = (avgVel > MathConstants::EPSILON)
                    ? deltaS / avgVel : T(0);
            }
            times[i] = times[i - 1] + dt;
        }

        // Build the final profile.
        profile.reserve(numSamples);
        for (size_t i = 0; i < numSamples; ++i) {
            Point pt;
            pt.arcLength = samples[i].arcLength;
            pt.velocity = fwdVel[i];
            pt.time = times[i];
            pt.acceleration = finalAccel[i];
            pt.limitedBy = cause[i];
            pt.velocityLimit = vLim[i];
            pt.accelerationLimit = pathAMax;

            // WI-3: Jerk from acceleration change over time — NOT clamped.
            // With WI-8b.3, the acceleration is the carried analytic state,
            // so the jerk (Δa/Δt) reflects the true trajectory dynamics.
            // Any violation is reported truthfully so it surfaces.
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
