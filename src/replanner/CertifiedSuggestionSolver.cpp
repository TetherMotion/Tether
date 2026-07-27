/**
 * @file CertifiedSuggestionSolver.cpp
 * @brief Implementation of certified limit suggestions via M15-pattern bisection
 */

#include "tether/motion_replanner/CertifiedSuggestionSolver.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace tether::motion::replanner {

namespace {

/// Predict the contour error at a candidate feed rate using the
/// measured error model: error(v) = measuredError × (v/measuredFeed)².
double predictedErrorAt(double candidateFeed,
                        double measuredFeed,
                        double measuredError) {
    if (measuredFeed <= 0.0) {
        return std::numeric_limits<double>::infinity();
    }
    double ratio = candidateFeed / measuredFeed;
    return measuredError * ratio * ratio;
}

/// Acceptance test: is the predicted error at this feed ≤ threshold?
bool isAccepted(double candidateFeed,
                double measuredFeed,
                double measuredError,
                double threshold) {
    return predictedErrorAt(candidateFeed, measuredFeed, measuredError)
           <= threshold;
}

} // anonymous namespace

CertifiedSuggestion solveCertifiedFeedRate(
    double measuredFeedRate,
    double measuredError,
    const SuggestionSolverConfig& config) {

    CertifiedSuggestion result;

    // Edge case: no error data.
    if (measuredError <= 0.0) {
        result.suggestedFeedRate = config.maxFeedRate * config.safetyFactor;
        result.predictedError = 0.0;
        result.accepted = true;
        result.reason = "No error — suggesting max feed";
        return result;
    }

    // Edge case: measured error already within threshold.
    if (measuredError <= config.contourErrorThreshold) {
        // Can we go faster? Search up to maxFeedRate.
        // The error model says error(v) = measuredError × (v/measuredFeed)².
        // The max accepted feed is:
        //   v_max = measuredFeed × sqrt(threshold / measuredError)
        double v_max = measuredFeedRate *
            std::sqrt(config.contourErrorThreshold / measuredError);
        v_max = std::min(v_max, config.maxFeedRate);
        result.suggestedFeedRate = v_max * config.safetyFactor;
        result.predictedError = predictedErrorAt(
            result.suggestedFeedRate, measuredFeedRate, measuredError);
        result.accepted = true;
        result.reason = "Measured error within threshold — scaled up";
        return result;
    }

    // Bisection: find the max feed in [minFeed, maxFeed] that is accepted.
    double feedLow = config.minFeedRate;
    double feedHigh = config.maxFeedRate;

    // Check if even the minimum feed is rejected.
    if (!isAccepted(feedLow, measuredFeedRate, measuredError,
                    config.contourErrorThreshold)) {
        result.suggestedFeedRate = config.minFeedRate;
        result.predictedError = predictedErrorAt(
            config.minFeedRate, measuredFeedRate, measuredError);
        result.accepted = false;
        result.reason = "All feeds rejected — using minimum";
        return result;
    }

    // Check if the maximum feed is accepted (unlikely if we're here,
    // but possible if the error model allows it).
    if (isAccepted(feedHigh, measuredFeedRate, measuredError,
                   config.contourErrorThreshold)) {
        result.suggestedFeedRate = feedHigh * config.safetyFactor;
        result.predictedError = predictedErrorAt(
            result.suggestedFeedRate, measuredFeedRate, measuredError);
        result.accepted = true;
        result.reason = "Max feed accepted";
        return result;
    }

    // Bisection: feedLow is accepted, feedHigh is rejected.
    // Find the boundary to within feedTolerance.
    int iter = 0;
    for (; iter < config.maxIterations; ++iter) {
        double mid = 0.5 * (feedLow + feedHigh);
        if (mid - feedLow < config.feedTolerance) {
            break; // Converged
        }
        if (isAccepted(mid, measuredFeedRate, measuredError,
                       config.contourErrorThreshold)) {
            feedLow = mid;
        } else {
            feedHigh = mid;
        }
    }

    // feedLow is the max accepted feed (within tolerance).
    result.suggestedFeedRate = feedLow * config.safetyFactor;
    result.predictedError = predictedErrorAt(
        result.suggestedFeedRate, measuredFeedRate, measuredError);
    result.accepted = true;
    result.iterations = iter;
    result.reason = "Bisection converged";
    return result;
}

CertifiedSuggestion solveCertifiedFeedRateWithCurvature(
    double measuredFeedRate,
    double measuredError,
    double curvatureAwareFeedLimit,
    const SuggestionSolverConfig& config) {

    // First, get the error-based suggestion.
    CertifiedSuggestion errorBased = solveCertifiedFeedRate(
        measuredFeedRate, measuredError, config);

    // Cap at the curvature-aware limit.
    if (curvatureAwareFeedLimit > 0.0 &&
        errorBased.suggestedFeedRate > curvatureAwareFeedLimit) {
        CertifiedSuggestion result = errorBased;
        result.suggestedFeedRate = curvatureAwareFeedLimit;
        result.predictedError = predictedErrorAt(
            curvatureAwareFeedLimit, measuredFeedRate, measuredError);
        result.reason = "Capped by curvature-aware limit: " + errorBased.reason;
        return result;
    }

    return errorBased;
}

} // namespace tether::motion::replanner
