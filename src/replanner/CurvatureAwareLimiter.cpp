/**
 * @file CurvatureAwareLimiter.cpp
 * @brief Implementation of curvature-aware proactive feed limiting
 */

#include "tether/motion_replanner/CurvatureAwareLimiter.hpp"
#include "tether/motion_planner/geometry/NurbsCurve.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace tether::motion::replanner {

double computeFeedRateLimit(double curvature, const CurvatureLimiterConfig& config) {
    if (curvature < config.minCurvature) {
        return config.maxFeedRate * config.safetyFactor;
    }

    // v = sqrt(a_cent / kappa)
    double velocity = std::sqrt(config.maxCentripetalAcceleration / curvature);
    double feedRate = velocity * 60.0; // mm/s → mm/min

    // Cap at maxFeedRate
    feedRate = std::min(feedRate, config.maxFeedRate);

    // Apply safety factor
    feedRate *= config.safetyFactor;

    return feedRate;
}

CurvatureAwareFeedLimits computeFeedLimitsFromSamples(
    const std::vector<GCodeExport::TrajectorySample>& samples,
    const CurvatureLimiterConfig& config) {

    CurvatureAwareFeedLimits result;
    result.points.reserve(samples.size());
    result.minFeedRate = std::numeric_limits<double>::max();
    result.maxFeedRate = 0.0;

    for (const auto& s : samples) {
        FeedLimitPoint p;
        p.arcLength = s.pathPosition;
        p.curvature = s.curvature;
        p.velocityLimit = (s.curvature < config.minCurvature)
            ? config.maxFeedRate / 60.0
            : std::sqrt(config.maxCentripetalAcceleration / s.curvature);
        p.velocityLimit = std::min(p.velocityLimit, config.maxFeedRate / 60.0);
        p.feedRateLimit = p.velocityLimit * 60.0 * config.safetyFactor;

        result.points.push_back(p);

        if (p.feedRateLimit < result.minFeedRate) {
            result.minFeedRate = p.feedRateLimit;
            result.minFeedRateArcLength = p.arcLength;
        }
        if (p.feedRateLimit > result.maxFeedRate) {
            result.maxFeedRate = p.feedRateLimit;
        }
    }

    if (result.points.empty()) {
        result.minFeedRate = 0.0;
    }

    return result;
}

CurvatureAwareFeedLimits computeCertifiedFeedLimits(
    const PiecewiseNurbsPath& path,
    const CurvatureLimiterConfig& config,
    std::size_t numSamples) {

    if (numSamples == 0) {
        throw std::invalid_argument("numSamples must be > 0");
    }

    CertifiedCurvatureSampler sampler(path);
    double totalLength = path.totalLength();

    CurvatureAwareFeedLimits result;
    result.points.reserve(numSamples);
    result.minFeedRate = std::numeric_limits<double>::max();
    result.maxFeedRate = 0.0;

    for (std::size_t i = 0; i < numSamples; ++i) {
        double s = (numSamples == 1)
            ? 0.0
            : static_cast<double>(i) / (numSamples - 1) * totalLength;

        // Get the certified upper bound on curvature at this arc length.
        CertifiedCurvature cc = sampler.maxCurvatureAtArcLength(s);
        double kappa = cc.maxKappa; // Conservative upper bound

        FeedLimitPoint p;
        p.arcLength = s;
        p.curvature = kappa;
        p.feedRateLimit = computeFeedRateLimit(kappa, config);
        p.velocityLimit = p.feedRateLimit / 60.0;

        result.points.push_back(p);

        if (p.feedRateLimit < result.minFeedRate) {
            result.minFeedRate = p.feedRateLimit;
            result.minFeedRateArcLength = p.arcLength;
        }
        if (p.feedRateLimit > result.maxFeedRate) {
            result.maxFeedRate = p.feedRateLimit;
        }
    }

    return result;
}

double certifiedFeedRateAt(
    const PiecewiseNurbsPath& path,
    double arcLength,
    const CurvatureLimiterConfig& config) {

    CertifiedCurvatureSampler sampler(path);
    double clampedS = std::clamp(arcLength, 0.0, path.totalLength());
    CertifiedCurvature cc = sampler.maxCurvatureAtArcLength(clampedS);
    return computeFeedRateLimit(cc.maxKappa, config);
}

} // namespace tether::motion::replanner
