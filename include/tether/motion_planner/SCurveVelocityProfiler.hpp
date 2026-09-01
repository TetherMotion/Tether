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
 * @see JerkConstrainedTOPPRA for the time-optimal jerk-limited option.
 * @see BasicTOPPRA for the basic TOPP-RA option.
 * @see VelocityProfiler.hpp for the abstract interface.
 */

#pragma once

#include "VelocityProfile.hpp"
#include "VelocityProfiler.hpp"
#include "SCurveProfile.hpp"
#include "PathAdapter.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

namespace MotionPlanner {

/**
 * @brief Basic S-curve velocity profiler.
 *
 * Implements VelocityProfiler. Produces a jerk-limited velocity profile
 * using per-piece 7-phase S-curve profiles. Not time-optimal but simple.
 */
template<size_t Dim, typename T = double>
class SCurveVelocityProfiler : public VelocityProfiler<Dim, T> {
public:
    using Path = PathAdapter<Dim, T>;
    using Limits = KinematicLimits<Dim, T>;
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
    std::unique_ptr<VelocityProfile> computeProfile(
        const Path& path,
        T feedRate,
        T startVelocity = T(0),
        T endVelocity = T(0),
        size_t numSamples = 100,
        T startAcceleration = T(0),
        T startJerk = T(0)) override {

        auto profile = std::make_unique<SampledVelocityProfile>();
        if (path.numSegments() == 0) return profile;
        if (numSamples < 2 || feedRate <= T(0) ||
            std::abs(startAcceleration) > T(1e-12) ||
            std::abs(startJerk) > T(1e-12)) {
            return profile;
        }

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

        // Compute per-piece cruise velocities from curvature and per-segment
        // feed rate limits (set via PathAdapter::setSegmentVelocityLimits).
        std::vector<T> pieceLengths;
        std::vector<T> cruiseVelocities;
        pieceLengths.reserve(segments.size());
        cruiseVelocities.reserve(segments.size());

        // Per-segment feed rates and corner velocities from the PathAdapter.
        // These are set by the caller via setSegmentVelocityLimits() (G-code
        // F-values) and computeCornerVelocities() (junction deviation model).
        // Without corner velocities, straight-line G-code would have zero
        // curvature and the profiler would cruise at vMax through every
        // junction — ignoring the deceleration required at corners.
        const bool hasPerSegLimits = path.hasPerSegmentVelocityLimits();
        const auto& segMaxVelocities = path.segmentMaxVelocities();
        const auto& cornerVelocities = path.cornerVelocities();

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
            // Apply per-segment feed rate limit (G-code F-values)
            if (hasPerSegLimits) {
                T vSeg = path.maxVelocityAtArcLength(midArc);
                if (vSeg < vCruise) vCruise = vSeg;
            }
            pieceLengths.push_back(seg.arcLength);
            cruiseVelocities.push_back(vCruise);
        }

        if (pieceLengths.empty()) return profile;

        // Compute per-junction exit velocities, limited by corner velocities.
        // exitVel[i] = min(cruise[i], cruise[i+1], cornerVel at junction i+1)
        // For the last segment, exitVel = endVelocity.
        std::vector<T> exitVelocities(pieceLengths.size(), vMax);
        for (size_t i = 0; i < pieceLengths.size(); ++i) {
            T targetVel = std::min(cruiseVelocities[i], vMax);
            T nextVel = (i + 1 < pieceLengths.size())
                            ? std::min(cruiseVelocities[i + 1], vMax)
                            : endVelocity;
            T exitVel = std::min(targetVel, nextVel);
            exitVelocities[i] = exitVel;
        }

        // Backward pass: propagate corner velocity limits backward so that
        // preceding segments can decelerate in time. If a sharp corner
        // forces a low exit velocity, the segment before it must also limit
        // its exit velocity (and so on, cascading backward).
        //
        // This is a conservative pass: it assumes each segment is long
        // enough to decelerate from its cruise to the next segment's exit
        // velocity. If a segment is too short, the S-curve compute will
        // fail and fall back to constant velocity — imperfect but safe.
        if (hasPerSegLimits && cornerVelocities.size() > 1) {
            // First, stamp corner velocities onto exit velocities
            size_t segIdx = 0;
            for (size_t i = 0; i < pieceLengths.size(); ++i) {
                while (segIdx < segments.size() &&
                       segments[segIdx].arcLength <= MathConstants::EPSILON) {
                    segIdx++;
                }
                if (segIdx + 1 < cornerVelocities.size()) {
                    T vCorner = static_cast<T>(cornerVelocities[segIdx + 1]);
                    if (vCorner < exitVelocities[i]) {
                        exitVelocities[i] = vCorner;
                    }
                }
                segIdx++;
            }
            // Backward propagation: limit each exit by the next exit
            for (size_t i = pieceLengths.size() - 1; i > 0; --i) {
                if (exitVelocities[i] < exitVelocities[i - 1]) {
                    // The next segment's exit is lower — we may need to
                    // decelerate through this segment. Limit our exit to
                    // the next exit (conservative: assumes we can decelerate
                    // within this segment's length).
                    T vMaxEntry = SCurve::computeDecelDistance(
                        exitVelocities[i - 1], exitVelocities[i],
                        aMax, jMax) > pieceLengths[i - 1]
                        ? exitVelocities[i]
                        : exitVelocities[i - 1];
                    if (vMaxEntry < exitVelocities[i - 1]) {
                        exitVelocities[i - 1] = vMaxEntry;
                    }
                }
            }
        }

        // Build S-curve sequence with velocity continuity
        std::vector<SCurve> scurves;
        std::vector<T> scurveStartArc;  // arc length at start of each S-curve
        std::vector<T> scurveStartTime; // exact accumulated phase time
        scurves.reserve(pieceLengths.size());
        scurveStartArc.reserve(pieceLengths.size());
        scurveStartTime.reserve(pieceLengths.size());

        T currentVel = startVelocity;
        T cumulativeArc = T(0);
        T accumulatedTime = T(0);

        size_t pieceIdx = 0;
        for (size_t i = 0; i < pieceLengths.size(); ++i) {
            // Skip zero-length pieces in segments
            while (pieceIdx < segments.size() &&
                   segments[pieceIdx].arcLength <= MathConstants::EPSILON) {
                pieceIdx++;
            }
            if (pieceIdx >= segments.size()) break;

            scurveStartArc.push_back(segments[pieceIdx].cumulativeArcLength);
            scurveStartTime.push_back(accumulatedTime);

            // Use the precomputed exit velocity (includes corner velocity
            // limits and backward-pass propagation).
            T exitVel = exitVelocities[i];

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
            accumulatedTime += scurves.back().totalDuration();
            cumulativeArc += pieceLengths[i];
            pieceIdx++;
        }

        if (scurves.empty()) return profile;

        // Sample the S-curve profiles at uniform arc length intervals
        // to produce a VelocityProfile.
        T ds = pathLength / T(numSamples - 1);
        for (size_t i = 0; i < numSamples; ++i) {
            T s = std::min(i * ds, pathLength);
            VelocityProfilePoint pt;
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
                    const auto state = sc.evaluateAt(*tOpt);
                    pt.velocity = state.velocity;
                    pt.acceleration = state.acceleration;
                    pt.jerk = state.jerk;
                    pt.time = scurveStartTime[scIdx] + *tOpt;
                } else {
                    pt.velocity = T(0);
                    pt.time = scurveStartTime[scIdx];
                }
            } else {
                pt.velocity = T(0);
                pt.time = scurveStartTime[scIdx];
            }

            // Store the velocity and acceleration limits used by the
            // profiler, so downstream consumers (ReNURBS) can check
            // constraint preservation against the exact limits.
            // For the S-curve profiler, v_lim is the cruise velocity of
            // the containing piece, and a_max is the path acceleration limit.
            pt.velocityLimit = vMax;
            pt.accelerationLimit = aMax;

            profile->addPoint(pt);
        }

        return profile;
    }

    Limits limits() const override { return limits_; }
    ProfilerType type() const override { return ProfilerType::SCurve; }
    const char* name() const override {
        return "SCurveVelocityProfiler (basic S-curve)";
    }
    ProfileDerivativeOrder derivativeOrder() const override {
        return ProfileDerivativeOrder::Jerk;
    }

private:
    Limits limits_;
};

} // namespace MotionPlanner
