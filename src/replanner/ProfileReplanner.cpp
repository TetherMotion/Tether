/**
 * @file ProfileReplanner.cpp
 * @brief Implementation of velocity profile re-planning + S-curve transitions
 */

#include "tether/motion_replanner/ProfileReplanner.hpp"
#include "tether/motion_planner/geometry/NurbsCurve.hpp"

#include <algorithm>
#include <cmath>
#include <sstream>
#include <stdexcept>

namespace tether::motion::replanner {

namespace {

/// Convert ProfileLimits to MotionPlanner::KinematicLimits<Dim, double>.
/// This is templated on Dim because PathAdapter and VelocityProfiler are
/// templated. We dispatch on the path dimension.
template <std::size_t Dim>
MotionPlanner::KinematicLimits<Dim, double> toKinematicLimits(
    const ProfileLimits& limits) {
    MotionPlanner::KinematicLimits<Dim, double> kl;
    kl.path.maxPathVelocity = limits.maxPathVelocity;
    kl.path.maxPathAcceleration = limits.maxPathAcceleration;
    kl.path.maxPathJerk = limits.maxPathJerk;
    kl.path.maxCentripetalAcceleration = limits.maxCentripetalAcceleration;
    kl.path.jerkLimitEnabled = limits.jerkLimitEnabled;

    for (std::size_t i = 0; i < Dim; ++i) {
        kl.axis.maxVelocity[i] = limits.maxAxisVelocity[i];
        kl.axis.maxAcceleration[i] = limits.maxAxisAcceleration[i];
        kl.axis.maxJerk[i] = limits.maxAxisJerk[i];
    }
    kl.axis.jerkLimitEnabled = limits.jerkLimitEnabled;

    return kl;
}

/// Run the VelocityProfiler for a given dimension.
template <std::size_t Dim>
ProfileReplanResult runProfiler(
    const PiecewiseNurbsPath& path,
    double feedRate,
    const ProfileLimits& limits,
    std::size_t numSamples,
    double startVelocity,
    double endVelocity) {

    MotionPlanner::PathAdapter<Dim, double> adapter(path);
    auto kl = toKinematicLimits<Dim>(limits);
    MotionPlanner::VelocityProfiler<Dim, double> profiler(kl);

    // feedRate is in mm/min; the profiler expects mm/s.
    double feedRateMmPerSec = feedRate / 60.0;

    auto profile = profiler.computeProfile(
        adapter, feedRateMmPerSec, startVelocity, endVelocity, numSamples);

    ProfileReplanResult result;
    result.totalTime = profile.totalTime();
    result.totalLength = profile.totalLength();
    result.jerkLimited = limits.jerkLimitEnabled;

    const auto& pts = profile.points();
    result.points.reserve(pts.size());
    for (const auto& p : pts) {
        ProfilePoint pp;
        pp.arcLength = p.arcLength;
        pp.velocity = p.velocity;
        pp.acceleration = p.acceleration;
        pp.time = p.time;
        result.points.push_back(pp);
        if (p.velocity > result.maxVelocity) {
            result.maxVelocity = p.velocity;
        }
    }

    std::ostringstream oss;
    oss << "Profile: " << result.points.size() << " points, "
        << result.totalTime << "s total, "
        << result.maxVelocity << " mm/s peak";
    result.summary = oss.str();

    return result;
}

} // anonymous namespace

ProfileReplanResult replanProfile(
    const PiecewiseNurbsPath& path,
    double feedRate,
    const ProfileLimits& limits,
    std::size_t numSamples,
    double startVelocity,
    double endVelocity) {

    if (numSamples == 0) {
        throw std::invalid_argument("numSamples must be > 0");
    }

    std::size_t dim = path.dim();
    if (dim < 1 || dim > 5) {
        throw std::invalid_argument(
            "Path dimension must be 1..5, got " + std::to_string(dim));
    }

    // Dispatch on dimension.
    switch (dim) {
        case 1: return runProfiler<1>(path, feedRate, limits, numSamples,
                                      startVelocity, endVelocity);
        case 2: return runProfiler<2>(path, feedRate, limits, numSamples,
                                      startVelocity, endVelocity);
        case 3: return runProfiler<3>(path, feedRate, limits, numSamples,
                                      startVelocity, endVelocity);
        case 4: return runProfiler<4>(path, feedRate, limits, numSamples,
                                      startVelocity, endVelocity);
        case 5: return runProfiler<5>(path, feedRate, limits, numSamples,
                                      startVelocity, endVelocity);
        default:
            throw std::invalid_argument("Unsupported path dimension");
    }
}

std::optional<MotionPlanner::SCurveProfile<double>> computeSCurveTransition(
    double distance,
    double startVelocity,
    double endVelocity,
    double maxVelocity,
    double maxAcceleration,
    double maxJerk) {

    MotionPlanner::SCurveConstraints<double> constraints;
    constraints.maxVelocity = maxVelocity;
    constraints.maxAcceleration = maxAcceleration;
    constraints.maxJerk = maxJerk;

    MotionPlanner::SCurveProfile<double> profile;
    bool ok = profile.compute(distance, startVelocity, endVelocity, constraints);

    if (!ok || !profile.isValid()) {
        return std::nullopt;
    }

    return profile;
}

} // namespace tether::motion::replanner
