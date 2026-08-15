/**
 * @file SCurveVelocityProfiler.hpp
 * @brief Basic S-curve velocity profiler (per-piece jerk-limited profiles).
 *
 * @details
 * This profiler produces a velocity profile by building a sequence of
 * 7-phase S-curve profiles, one per path piece (segment). It is simpler
 * than TOPP-RA and does not produce a time-optimal profile, but it
 * provides jerk-limited motion with bounded acceleration and jerk.
 *
 * ## Algorithm
 *
 * 1. For each non-zero-length path piece, compute a cruise velocity
 *    limited by curvature (v = sqrt(a_cent_max / kappa)) and the
 *    path-level max velocity.
 * 2. Build a sequence of S-curve profiles with velocity continuity:
 *    - Piece 0 starts at velocity 0 (rest).
 *    - Each piece's exit velocity is the minimum of its cruise velocity
 *      and the next piece's cruise velocity.
 *    - The last piece ends at velocity 0 (rest).
 * 3. Sample the S-curve profiles at uniform arc length intervals to
 *    produce a VelocityProfile with per-point velocity, acceleration,
 *    jerk, and time.
 *
 * ## When to Use
 *
 * - When simplicity is more important than time-optimality.
 * - When you need jerk-limited motion but don't want the complexity of
 *   the jerk-integrated TOPP-RA optimizer.
 * - For testing and validation against the TOPP-RA profilers.
 *
 * ## Limitations
 *
 * - Not time-optimal (TOPP-RA is faster for the same constraints).
 * - Per-axis jerk limits at blend boundaries are not enforced (only
 *   path-level jerk is bounded).
 * - Does not use the certified curvature sampler (uses midpoint curvature).
 *
 * @see JerkConstrainedVelocityProfiler for the time-optimal jerk-limited option.
 * @see VelocityProfiler for the basic TOPP-RA option.
 * @see IVelocityProfiler.hpp for the abstract interface.
 */

#pragma once

#include "VelocityProfile.hpp"
#include "SCurveProfile.hpp"
#include "PathAdapter.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

namespace MotionPlanner {

/**
 * @brief Basic S-curve velocity profiler.
 *
 * Implements IVelocityProfiler. Produces a jerk-limited velocity profile
 * using per-piece 7-phase S-curve profiles. Not time-optimal but simple.
 */
template<size_t Dim, typename T = double>
class SCurveVelocityProfiler : public IVelocityProfiler<Dim, T> {
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
    explicit SCurveVelocityProfiler(Limits limits = {})
        : limits_(std::move(limits)) {}

    /**
     * @brief Compute a basic S-curve velocity profile for the given path.
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

        const T jMax = limits_.path.maxPathJerk;
        const T aMax = limits_.path.maxPathAcceleration;
        const T vMax = std::min(limits_.path.maxPathVelocity, feedRate);

        if (aMax <= T(0) || jMax <= T(0)) {
            // Invalid constraints for S-curve; return empty profile
            return profile;
        }

        SCurveConstraints<T> constraints;
        constraints.maxVelocity = vMax;
        constraints.maxAcceleration = aMax;
        constraints.maxJerk = jMax;

        const auto& segments = path.segments();

        // Compute per-piece cruise velocities from curvature
        std::vector<T> pieceLengths;
        std::vector<T> cruiseVelocities;
        pieceLengths.reserve(segments.size());
        cruiseVelocities.reserve(segments.size());

        for (const auto& seg : segments) {
            if (seg.arcLength <= MathConstants::EPSILON) continue;
            T vCruise = vMax;
            T midArc = seg.cumulativeArcLength + seg.arcLength * T(0.5);
            T kappa = path.curvatureAtArcLength(midArc);
            if (kappa > MathConstants::EPSILON) {
                T vCurv = std::sqrt(
                    limits_.path.maxCentripetalAcceleration / kappa);
                vCruise = std::min(vCruise, vCurv);
            }
            pieceLengths.push_back(seg.arcLength);
            cruiseVelocities.push_back(vCruise);
        }

        if (pieceLengths.empty()) return profile;

        // Build S-curve sequence with velocity continuity
        std::vector<SCurve> scurves;
        std::vector<T> scurveStartArc;  // arc length at start of each S-curve
        scurves.reserve(pieceLengths.size());
        scurveStartArc.reserve(pieceLengths.size());

        T currentVel = startVelocity;
        T cumulativeArc = T(0);

        size_t pieceIdx = 0;
        for (size_t i = 0; i < pieceLengths.size(); ++i) {
            // Skip zero-length pieces in segments
            while (pieceIdx < segments.size() &&
                   segments[pieceIdx].arcLength <= MathConstants::EPSILON) {
                pieceIdx++;
            }
            if (pieceIdx >= segments.size()) break;

            scurveStartArc.push_back(segments[pieceIdx].cumulativeArcLength);

            T targetVel = std::min(cruiseVelocities[i], vMax);
            T nextVel = (i + 1 < pieceLengths.size())
                            ? std::min(cruiseVelocities[i + 1], vMax)
                            : endVelocity;
            T exitVel = std::min(targetVel, nextVel);

            SCurve sc;
            bool ok = sc.compute(pieceLengths[i], currentVel, exitVel, constraints);
            if (!ok || !sc.isValid()) {
                sc = SCurve{};
                ok = sc.compute(pieceLengths[i], currentVel, currentVel, constraints);
            }
            if (!ok || !sc.isValid()) {
                scurves.push_back(SCurve{});
                pieceIdx++;
                continue;
            }

            scurves.push_back(std::move(sc));
            currentVel = scurves.back().evaluateAt(
                scurves.back().totalDuration()).velocity;
            cumulativeArc += pieceLengths[i];
            pieceIdx++;
        }

        if (scurves.empty()) return profile;

        // Sample the S-curve profiles at uniform arc length intervals
        // to produce a VelocityProfile.
        T ds = pathLength / T(numSamples - 1);
        T currentTime = T(0);
        T prevVel = startVelocity;

        for (size_t i = 0; i < numSamples; ++i) {
            T s = std::min(i * ds, pathLength);
            Point pt;
            pt.arcLength = s;

            // Find which S-curve contains this arc length
            size_t scIdx = 0;
            for (size_t j = 0; j < scurves.size(); ++j) {
                T scStart = scurveStartArc[j];
                T scEnd = scStart + scurves[j].totalDistance();
                if (s >= scStart - MathConstants::EPSILON &&
                    s <= scEnd + MathConstants::EPSILON) {
                    scIdx = j;
                    break;
                }
                if (j == scurves.size() - 1) scIdx = j;
            }

            const auto& sc = scurves[scIdx];
            if (sc.isValid()) {
                T localS = s - scurveStartArc[scIdx];
                localS = std::clamp(localS, T(0), sc.totalDistance());
                // Find time at position within this S-curve
                auto tOpt = sc.findTimeAtPosition(localS);
                if (tOpt) {
                    pt.velocity = sc.evaluateAt(*tOpt).velocity;
                    pt.acceleration = sc.evaluateAt(*tOpt).acceleration;
                    pt.jerk = sc.evaluateAt(*tOpt).jerk;
                } else {
                    pt.velocity = T(0);
                }
            } else {
                pt.velocity = T(0);
            }

            // Compute time from arc length and velocity
            if (i > 0) {
                T avgVel = (prevVel + pt.velocity) / T(2);
                T deltaS = pt.arcLength - profile.points()[i - 1].arcLength;
                if (avgVel > MathConstants::EPSILON) {
                    currentTime += deltaS / avgVel;
                }
            }
            pt.time = currentTime;
            prevVel = pt.velocity;

            // Store the velocity and acceleration limits used by the
            // profiler, so downstream consumers (ReNURBS) can check
            // constraint preservation against the exact limits.
            // For the S-curve profiler, v_lim is the cruise velocity of
            // the containing piece, and a_max is the path acceleration limit.
            pt.velocityLimit = vMax;
            pt.accelerationLimit = aMax;

            profile.addPoint(pt);
        }

        return profile;
    }

    Limits limits() const override { return limits_; }
    ProfilerType type() const override { return ProfilerType::SCurve; }
    const char* name() const override {
        return "SCurveVelocityProfiler (basic S-curve)";
    }

private:
    Limits limits_;
};

} // namespace MotionPlanner
