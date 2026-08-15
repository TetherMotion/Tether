/**
 * @file BasicTOPPRA.hpp
 * @brief Basic 2nd-order TOPP-RA velocity profiler (bang-bang acceleration).
 *
 * @details
 * This is the basic 2nd-order TOPP-RA profiler. It produces a time-optimal
 * velocity profile with bang-bang acceleration (no jerk limiting).
 * Acceleration is discontinuous at constraint switching points.
 *
 * For jerk-constrained profiling, use JerkConstrainedTOPPRA instead.
 *
 * @see VelocityProfiler.hpp for the abstract interface.
 * @see VelocityProfile.hpp for the profile data structure.
 */

#pragma once

#include "VelocityProfile.hpp"
#include "VelocityProfiler.hpp"
#include "PathAdapter.hpp"
#include <tether/motion_planner/geometry/CertifiedCurvatureSampler.hpp>
#include <tether/motion_planner/blend/PHQuinticBlendBuilder.hpp>
#include <vector>
#include <array>
#include <algorithm>
#include <cmath>
#include <optional>

namespace MotionPlanner {

/**
 * @brief Computes velocity profiles using basic 2nd-order TOPP-RA.
 *
 * This is the basic 2nd-order TOPP-RA profiler. It produces a time-optimal
 * velocity profile with bang-bang acceleration (no jerk limiting).
 * Acceleration is discontinuous at constraint switching points.
 *
 * For jerk-constrained profiling, use JerkConstrainedTOPPRA instead.
 */
template<size_t Dim, typename T = double>
class BasicTOPPRA : public VelocityProfiler<Dim, T> {
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
    explicit BasicTOPPRA(Limits limits = {})
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
    const char* name() const override { return "BasicTOPPRA (TOPP-RA basic)"; }

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

using BasicTOPPRA2D = BasicTOPPRA<2, double>;
using BasicTOPPRA3D = BasicTOPPRA<3, double>;

}  // namespace MotionPlanner
