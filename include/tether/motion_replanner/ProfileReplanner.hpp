/**
 * @file ProfileReplanner.hpp
 * @brief Re-plan velocity profile + S-curve transitions
 *
 * @details
 * Replaces the scalar quinticBlend limit transitions in
 * MotionReplanner::getSuggestedLimits / quinticBlend with:
 *
 * 1. **TOPP-RA velocity profile** via MotionPlanner::VelocityProfiler:
 *    forward/backward passes + curvature-aware limit curve, producing
 *    a time-optimal profile that respects per-axis and path-level
 *    kinematic limits.
 *
 * 2. **S-curve transitions** via MotionPlanner::SCurveProfile:
 *    7-phase jerk-limited profiles for smooth transitions between
 *    different feed rates (e.g. from cruise to corner feed).
 *
 * ## Why this is better
 *
 * The old replanner used a scalar C2 quintic polynomial blend for
 * transitions between different feed/accel limits. This is:
 * - Not jerk-limited: the jerk can spike at the blend boundaries.
 * - Not time-optimal: it doesn't account for the path geometry.
 * - Not curvature-aware: it doesn't slow down for corners proactively.
 *
 * The certified method uses the kernel's TOPP-RA profiler (which uses
 * CertifiedCurvatureSampler for the curvature limit curve) and S-curve
 * profiles (analytic 7-phase jerk-limited transitions).
 *
 * ## Legacy namespace note
 *
 * VelocityProfiler and SCurveProfile are in the legacy MotionPlanner
 * namespace (templated <Dim, T>). Per Architecture.md, this namespace
 * will be migrated to tether::motion in a future phase. This wrapper
 * isolates the legacy dependency so the migration is a one-file change.
 *
 * @see VelocityProfile.hpp for the TOPP-RA profiler.
 * @see SCurveProfile.hpp for the 7-phase S-curve.
 */

#pragma once

#include "tether/motion_planner/geometry/PiecewiseNurbsPath.hpp"
#include "tether/motion_planner/VelocityProfile.hpp"
#include "tether/motion_planner/SCurveProfile.hpp"
#include "tether/motion_planner/PathAdapter.hpp"

#include <vector>
#include <cstddef>
#include <optional>
#include <string>

namespace tether::motion::replanner {

/// Per-axis kinematic limits for the profile replanner.
/// This is a simplified, non-templated version of
/// MotionPlanner::KinematicLimits<Dim, T> for the common 2D/3D case.
struct ProfileLimits {
    /// Maximum path velocity (mm/s).
    double maxPathVelocity = 100.0;

    /// Maximum path acceleration (mm/s²).
    double maxPathAcceleration = 500.0;

    /// Maximum path jerk (mm/s³).
    double maxPathJerk = 5000.0;

    /// Maximum centripetal acceleration (mm/s²).
    double maxCentripetalAcceleration = 500.0;

    /// Per-axis velocity limits (mm/s). Up to 5 axes.
    std::array<double, 5> maxAxisVelocity = {100, 100, 100, 100, 100};

    /// Per-axis acceleration limits (mm/s²).
    std::array<double, 5> maxAxisAcceleration = {500, 500, 500, 500, 500};

    /// Per-axis jerk limits (mm/s³). If all zero, jerk limiting is disabled.
    std::array<double, 5> maxAxisJerk = {5000, 5000, 5000, 5000, 5000};

    /// Whether jerk limits are enabled.
    bool jerkLimitEnabled = true;
};

/// A point on the recomputed velocity profile.
struct ProfilePoint {
    double arcLength = 0.0;  ///< Arc length along path (mm)
    double velocity = 0.0;   ///< Path velocity (mm/s)
    double acceleration = 0.0; ///< Path acceleration (mm/s²)
    double time = 0.0;       ///< Time from start (s)
};

/// Result of a velocity profile re-plan.
struct ProfileReplanResult {
    /// The recomputed velocity profile, one point per sample.
    std::vector<ProfilePoint> points;

    /// Total traversal time (s).
    double totalTime = 0.0;

    /// Total path length (mm).
    double totalLength = 0.0;

    /// Maximum velocity in the profile (mm/s).
    double maxVelocity = 0.0;

    /// Whether jerk limiting was applied.
    bool jerkLimited = false;

    /// Human-readable diagnostic.
    std::string summary;
};

/**
 * @brief Re-plan the velocity profile for a path using TOPP-RA.
 *
 * @param path The desired path (from TrajectorySampleConverter or
 *        OnlineReblender::extractPath).
 * @param feedRate The commanded feed rate (mm/min).
 * @param limits Kinematic limits.
 * @param numSamples Number of sample points along the path.
 * @param startVelocity Initial velocity (mm/s, default 0).
 * @param endVelocity Final velocity (mm/s, default 0).
 * @return The recomputed profile.
 */
ProfileReplanResult replanProfile(
    const PiecewiseNurbsPath& path,
    double feedRate,
    const ProfileLimits& limits = {},
    std::size_t numSamples = 100,
    double startVelocity = 0.0,
    double endVelocity = 0.0);

/**
 * @brief Compute an S-curve transition between two velocities.
 *
 * Uses MotionPlanner::SCurveProfile to compute a 7-phase jerk-limited
 * transition from startVelocity to endVelocity over a given distance.
 *
 * @param distance The transition distance (mm).
 * @param startVelocity The starting velocity (mm/s).
 * @param endVelocity The target ending velocity (mm/s).
 * @param maxVelocity Maximum velocity during transition (mm/s).
 * @param maxAcceleration Maximum acceleration (mm/s²).
 * @param maxJerk Maximum jerk (mm/s³).
 * @return The S-curve profile, or nullopt if no valid profile exists
 *         for the given constraints.
 */
std::optional<MotionPlanner::SCurveProfile<double>> computeSCurveTransition(
    double distance,
    double startVelocity,
    double endVelocity,
    double maxVelocity,
    double maxAcceleration,
    double maxJerk);

} // namespace tether::motion::replanner
