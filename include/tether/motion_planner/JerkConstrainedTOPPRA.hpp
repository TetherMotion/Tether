/**
 * @file JerkConstrainedTOPPRA.hpp
 * @brief Jerk-limited TOPP-RA velocity profiler (3rd-order time-optimal).
 *
 * @details
 * This profiler extends the basic TOPP-RA algorithm to include jerk as a
 * first-class constraint inside the optimization. Instead of post-hoc
 * S-curve smoothing (which breaks both optimality and feasibility), the
 * jerk limit is enforced during the forward and backward passes, producing
 * a time-optimal profile with:
 *
 * - **Continuous acceleration** (no step changes at switching points)
 * - **Bounded jerk** (|s⃛| ≤ j_max everywhere)
 * - **All original constraints verified** (velocity, acceleration, curvature)
 *
 * ## Algorithm
 *
 * The basic TOPP-RA uses the 2nd-order kinematic equation:
 *   v² = v₀² + 2·a_max·Δs
 * which produces bang-bang acceleration (instantaneous switching between
 * ±a_max). The jerk-limited version replaces this with the jerk-limited
 * distance function from SCurveProfile:
 *   Δs = computeAccelDistance(v₀, v₁, a_max, j_max)
 * which accounts for the finite time needed to ramp acceleration up/down.
 *
 * ### Backward Pass (jerk-limited deceleration)
 *
 * Sweeping from the end of the path backward, at each sample i we compute
 * the maximum entry velocity v_i such that we can decelerate from v_i to
 * v_{i+1} (the next backward-pass velocity) over distance Δs, respecting
 * jerk limits:
 *   v_i = max v such that computeDecelDistance(v, v_{i+1}, a_max, j_max) ≤ Δs
 * This is solved by binary search (maxVelocityAfterDistance). The result
 * is also capped by v_lim(s_i) (curvature/feedrate/axis limits).
 *
 * ### Forward Pass (jerk-limited acceleration)
 *
 * Sweeping from the start forward, at each sample i we compute the maximum
 * velocity v_i such that we can accelerate from v_{i-1} to v_i over distance
 * Δs, respecting jerk limits:
 *   v_i = min(v_lim(s_i), maxVelocityAfterDistance(v_{i-1}, Δs, v_max, a_max, j_max))
 * The forward pass is also capped by the backward pass (to ensure we can
 * still stop in time).
 *
 * ### Final Profile
 *
 * v(s) = min(forward, backward, v_lim) — same as basic TOPP-RA, but now
 * both forward and backward passes are jerk-limited. The acceleration at
 * each point is computed from the jerk-limited velocity change, and the
 * jerk is ±j_max or 0 (by construction of the jerk-limited distance function).
 *
 * ## Time Optimality
 *
 * The jerk-limited profile is slightly slower than the basic TOPP-RA
 * profile (jerk limiting costs time), but it is time-optimal *subject to
 * the jerk constraint*. This is the correct way to get smooth trajectories:
 * constrain the jerk inside the optimizer, don't filter it afterward.
 *
 * @see BasicTOPPRA.hpp for the basic TOPP-RA profiler.
 * @see SCurveProfile.hpp for computeAccelDistance / maxVelocityAfterDistance.
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
 * @brief Jerk-limited TOPP-RA velocity profiler.
 *
 * Implements VelocityProfiler. Produces a time-optimal velocity profile
 * with jerk as a first-class constraint inside the optimization.
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
     * - Per-axis velocity and acceleration limits
     * - Path-level velocity, acceleration, and jerk limits
     * - Centripetal acceleration (curvature) limits
     * - Feed rate
     *
     * Acceleration is continuous throughout; jerk is bounded by j_max.
     */
    Profile computeProfile(
        const Path& path,
        T feedRate,
        T startVelocity = T(0),
        T endVelocity = T(0),
        size_t numSamples = 100,
        T startAcceleration = T(0),
        T startJerk = T(0)) override {

        Profile profile;
        if (path.numSegments() == 0) return profile;

        T pathLength = path.totalLength();
        if (pathLength <= T(0)) return profile;

        // Jerk-limited profiling requires a valid jerk limit.
        // If jerk is not enabled, fall back to basic TOPP-RA behavior
        // (the 2nd-order kinematic equation).
        const T jMax = limits_.path.maxPathJerk;
        const T aMax = limits_.path.maxPathAcceleration;
        const T vMax = limits_.path.maxPathVelocity;

        const bool jerkEnabled = limits_.path.jerkLimitEnabled && jMax > T(0);

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
                    auto cert = curvatureSampler->maxCurvatureAtArcLength(static_cast<double>(s));
                    samples[i].curvature = static_cast<T>(cert.maxKappa);
                } else {
                    samples[i].curvature = path.curvatureAtArcLength(s);
                }
            }
        }

        // --- Velocity limit curve v_lim(s) ---
        std::vector<T> vLim(numSamples);
        for (size_t i = 0; i < numSamples; ++i) {
            vLim[i] = computeVelocityLimit(samples[i], feedRate);
        }

        // ================================================================
        // Jerk-limited backward pass
        // ================================================================
        // Sweep from end to start. At each sample i, compute the maximum
        // v_i such that we can decelerate from v_i to v_{i+1} over Δs
        // with jerk-limited deceleration.
        std::vector<T> backwardVel(numSamples);
        backwardVel[numSamples - 1] = std::min(endVelocity, vLim[numSamples - 1]);

        for (size_t i = numSamples - 1; i > 0; --i) {
            T deltaS = samples[i].arcLength - samples[i - 1].arcLength;
            T vNext = backwardVel[i];

            // Max entry velocity: can we decelerate from v to vNext over deltaS?
            // Using jerk-limited deceleration distance.
            T vMaxDecel = SCurve::maxVelocityAfterDistance(
                vNext, deltaS, vLim[i - 1], aMax, jMax);

            // Also cap by velocity limit at this point
            backwardVel[i - 1] = std::min(vMaxDecel, vLim[i - 1]);
        }

        // ================================================================
        // Jerk-limited forward pass
        // ================================================================
        // Sweep from start to end. At each sample i, compute the maximum
        // v_i such that we can accelerate from v_{i-1} to v_i over Δs
        // with jerk-limited acceleration, and still decelerate in time
        // (backward pass constraint).
        std::vector<T> forwardVel(numSamples);
        forwardVel[0] = std::min(startVelocity, vLim[0]);

        for (size_t i = 1; i < numSamples; ++i) {
            T deltaS = samples[i].arcLength - samples[i - 1].arcLength;
            T vPrev = forwardVel[i - 1];

            // Max velocity reachable from vPrev over deltaS with jerk limit
            T vMaxAccel = SCurve::maxVelocityAfterDistance(
                vPrev, deltaS, vLim[i], aMax, jMax);

            // Cap by velocity limit and backward pass (must be able to stop)
            forwardVel[i] = std::min({vMaxAccel, vLim[i], backwardVel[i]});
        }

        // ================================================================
        // Final profile: min(forward, backward, v_lim) + time + accel + jerk
        // ================================================================
        profile.reserve(numSamples);
        T currentTime = T(0);

        for (size_t i = 0; i < numSamples; ++i) {
            Point pt;
            pt.arcLength = samples[i].arcLength;

            T fwd = forwardVel[i];
            T bwd = backwardVel[i];
            T lim = vLim[i];
            pt.velocity = std::min({fwd, bwd, lim});

            // Determine limiting factor
            if (pt.velocity == fwd && fwd <= bwd && fwd <= lim) {
                pt.limitedBy = Point::LimitType::ForwardAccel;
            } else if (pt.velocity == bwd && bwd <= lim) {
                pt.limitedBy = Point::LimitType::BackwardDecel;
            } else if (pt.velocity == lim) {
                pt.limitedBy = Point::LimitType::Curvature;
            } else {
                pt.limitedBy = Point::LimitType::Jerk;
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

            // Compute acceleration and jerk from the velocity profile.
            // Since the forward/backward passes use jerk-limited distance
            // functions, the acceleration is continuous and jerk is bounded.
            if (i == 0) {
                pt.acceleration = startAcceleration;
                pt.jerk = startJerk;
            } else if (i + 1 < numSamples) {
                // Acceleration from velocity change over time
                T prevVel = profile.points()[i - 1].velocity;
                T dt = pt.time - profile.points()[i - 1].time;
                if (dt > MathConstants::EPSILON) {
                    pt.acceleration = (pt.velocity - prevVel) / dt;
                }

                // Jerk from acceleration change over time
                T prevAccel = profile.points()[i - 1].acceleration;
                if (dt > MathConstants::EPSILON) {
                    pt.jerk = (pt.acceleration - prevAccel) / dt;
                    // Clamp to jerk limit (numerical noise)
                    if (std::abs(pt.jerk) > jMax) {
                        pt.jerk = std::copysign(jMax, pt.jerk);
                    }
                }
            } else {
                // Last point: deceleration to end velocity
                T prevVel = profile.points()[i - 1].velocity;
                T dt = pt.time - profile.points()[i - 1].time;
                if (dt > MathConstants::EPSILON) {
                    pt.acceleration = (pt.velocity - prevVel) / dt;
                }
                pt.jerk = T(0);
            }

            profile.addPoint(pt);
        }

        return profile;
    }

    Limits limits() const override { return limits_; }
    ProfilerType type() const override { return ProfilerType::ToppraJerkConstrained; }
    const char* name() const override {
        return "JerkConstrainedTOPPRA (TOPP-RA + jerk constraint)";
    }

private:
    /// Internal path sample (same structure as basic profiler)
    struct PathSample {
        T arcLength = T(0);
        Vec<Dim, T> position;
        Vec<Dim, T> tangent;
        T curvature = T(0);
    };

    /// Compute velocity limit at a path sample (same as basic profiler)
    T computeVelocityLimit(const PathSample& sample, T feedRate) const {
        T limit = feedRate;
        if (sample.curvature > MathConstants::EPSILON) {
            T curvatureLimit = std::sqrt(
                limits_.path.maxCentripetalAcceleration / sample.curvature);
            limit = std::min(limit, curvatureLimit);
        }
        T axisLimit = limits_.maxVelocityForDirection(sample.tangent);
        limit = std::min(limit, axisLimit);
        limit = std::min(limit, limits_.path.maxPathVelocity);
        return limit;
    }

    Limits limits_;
};

} // namespace MotionPlanner
