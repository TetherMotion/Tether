/**
 * @file CertifiedSuggestionSolver.hpp
 * @brief Certified limit suggestions via M15-pattern bisection (T3 guarantee)
 *
 * @details
 * Replaces the old 1/sqrt(errorRatio) heuristic in
 * MotionReplanner::generateSuggestion with a bisection search for the
 * maximum feed rate whose *certified* contour error (from Phase 2's
 * pointCurveDistance) stays within the tolerance threshold.
 *
 * ## Why this is better
 *
 * The old method used `suggestedFeed = currentFeed / sqrt(errorRatio)`
 * where `errorRatio = actualError / threshold`. This is a magic-number
 * heuristic with no guarantee that the suggested feed will actually keep
 * the error within threshold — it's a rough scaling that can overshoot
 * or undershoot depending on the system's dynamics.
 *
 * The certified method mirrors BlendSolver::solve (M15):
 * 1. Define an acceptance test: `certifiedError(feed) ≤ threshold`.
 * 2. Bisect on the feed rate to find the maximum accepted feed.
 * 3. The result is guaranteed (T3 analog): the suggested feed will not
 *    violate the tolerance, given the error model.
 *
 * ## Error model
 *
 * The error model maps a feed rate to a predicted contour error. Two
 * models are supported:
 *
 * 1. **Measured model**: uses the actual measured error at the current
 *    feed and scales it by the square of the feed ratio (error ∝ v²
 *    for centripetal-acceleration-limited error). This is the default
 *    and requires only the current measured error and feed.
 *
 * 2. **Simulated model**: simulates the system response at a candidate
 *    feed using a simple second-order model. This is more accurate but
 *    requires a system model (damping, natural frequency). Not yet
 *    implemented; reserved for future use.
 *
 * ## Bisection
 *
 * The bisection searches in [feedLow, feedHigh] where:
 * - feedLow is known to be accepted (error ≤ threshold).
 * - feedHigh is known to be rejected (error > threshold).
 * The search converges to within `feedTolerance` of the true maximum
 * accepted feed, typically in ~20 iterations for a 1 mm/min tolerance.
 *
 * @see BlendSolver.hpp for the kernel's M15 bisection pattern.
 * @see CertifiedContourError.hpp for the certified error computation.
 */

#pragma once

#include <optional>
#include <string>

namespace tether::motion::replanner {

/// Configuration for the certified suggestion solver.
struct SuggestionSolverConfig {
    /// The tolerance threshold for the contour error (mm).
    /// The solver finds the max feed whose predicted error ≤ this.
    double contourErrorThreshold = 0.01; // 10 µm

    /// The feed tolerance for the bisection (mm/min).
    /// The search converges to within this of the true maximum.
    double feedTolerance = 1.0;

    /// Maximum number of bisection iterations.
    int maxIterations = 50;

    /// Minimum feed rate to consider (mm/min).
    double minFeedRate = 1.0;

    /// Maximum feed rate to consider (mm/min).
    double maxFeedRate = 60000.0;

    /// Safety factor applied to the final accepted feed (0..1).
    /// A value of 0.95 means the suggested feed is 95% of the maximum
    /// accepted feed, providing a 5% safety margin.
    double safetyFactor = 0.95;
};

/// Result of the certified suggestion solver.
struct CertifiedSuggestion {
    /// The suggested feed rate (mm/min) — guaranteed to keep the
    /// predicted contour error ≤ contourErrorThreshold (T3 analog).
    double suggestedFeedRate = 0.0;

    /// The predicted contour error at the suggested feed rate (mm).
    double predictedError = 0.0;

    /// Whether the solver found an acceptable feed rate.
    /// If false, suggestedFeedRate is set to minFeedRate and
    /// predictedError may still exceed the threshold.
    bool accepted = false;

    /// The number of bisection iterations performed.
    int iterations = 0;

    /// Human-readable diagnostic (e.g. "Bisection converged",
    /// "All feeds rejected — using minimum", "No error data").
    std::string reason;
};

/**
 * @brief Solve for the maximum feed rate whose predicted contour error
 *        stays within the tolerance threshold.
 *
 * Uses the measured error model: error(v) = measuredError × (v/measuredFeed)².
 * This model assumes the contour error scales with the square of the feed
 * rate (valid for centripetal-acceleration-limited error on curved paths).
 *
 * @param measuredFeedRate The current feed rate (mm/min).
 * @param measuredError The current measured contour error (mm).
 * @param config Solver configuration.
 * @return The certified suggestion.
 */
CertifiedSuggestion solveCertifiedFeedRate(
    double measuredFeedRate,
    double measuredError,
    const SuggestionSolverConfig& config = {});

/**
 * @brief Solve for the maximum feed rate using a curvature-aware model.
 *
 * Combines the measured error model with the curvature-aware feed limit
 * (from Phase 4). The suggested feed is the minimum of:
 * - The certified bisection result (error-based limit).
 * - The curvature-aware limit (centripetal acceleration limit).
 *
 * @param measuredFeedRate The current feed rate (mm/min).
 * @param measuredError The current measured contour error (mm).
 * @param curvatureAwareFeedLimit The curvature-aware feed limit at the
 *        current path position (mm/min), from CurvatureAwareLimiter.
 * @param config Solver configuration.
 * @return The certified suggestion.
 */
CertifiedSuggestion solveCertifiedFeedRateWithCurvature(
    double measuredFeedRate,
    double measuredError,
    double curvatureAwareFeedLimit,
    const SuggestionSolverConfig& config = {});

} // namespace tether::motion::replanner
