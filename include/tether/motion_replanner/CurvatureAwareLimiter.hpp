/**
 * @file CurvatureAwareLimiter.hpp
 * @brief Curvature-aware proactive feed limiting via CertifiedCurvatureSampler
 *
 * @details
 * The replanner's TrajectorySample already carries a `curvature` field that
 * was historically ignored. This module provides two levels of curvature-
 * aware feed limiting:
 *
 * ## Level 1: Immediate (from TrajectorySample)
 *
 * For each sample, compute v_safe = sqrt(maxCentripetalAccel / kappa) and
 * cap the suggested feed rate at v_safe. This makes the replanner proactive
 * instead of reactive — it slows down BEFORE entering a corner, not after
 * the error is observed.
 *
 * ## Level 2: Certified (from PiecewiseNurbsPath)
 *
 * Uses tether::motion::CertifiedCurvatureSampler to get a certified upper
 * bound on the maximum curvature per span (Lipschitz certificate), then
 * builds a curvature-aware velocity limit curve:
 *
 *   v_lim(s) = sqrt(a_cent_max / kappa_certified_upper(s))
 *
 * This is the same formula used by the kernel's BasicTOPPRA
 * (TOPP-RA inspired), giving the replanner access to the same certified
 * velocity limit curve without needing the full TOPP-RA machinery.
 *
 * ## Conversion to feed rate
 *
 * The replanner works in mm/min (feed rate), while the physics is in mm/s
 * (velocity). The conversion is: feedRate = velocity * 60.
 *
 * @see CertifiedCurvatureSampler.hpp for the certified curvature algorithm.
 * @see VelocityProfile.hpp for the kernel's TOPP-RA profiler.
 */

#pragma once

#include "tether/motion_planner/geometry/PiecewiseNurbsPath.hpp"
#include "tether/motion_planner/geometry/CertifiedCurvatureSampler.hpp"
#include "tether/export/TrajectoryAnalyzer.hpp"

#include <vector>
#include <cstddef>
#include <optional>

namespace tether::motion::replanner {

/// Configuration for the curvature-aware limiter.
struct CurvatureLimiterConfig {
    /// Maximum allowed centripetal acceleration (mm/s²).
    /// The feed limit is v = sqrt(a_cent / kappa).
    double maxCentripetalAcceleration = 500.0;

    /// Maximum absolute feed rate (mm/min). The curvature-aware limit
    /// is capped at this value on straight segments (kappa → 0).
    double maxFeedRate = 6000.0; // 100 mm/s

    /// Minimum curvature below which the segment is treated as straight
    /// (1/mm). Below this, the feed rate is capped at maxFeedRate.
    double minCurvature = 1e-9;

    /// Safety factor applied to the computed limit (0..1).
    /// A value of 0.9 means the suggested feed is 90% of the theoretical
    /// maximum, providing a 10% safety margin.
    double safetyFactor = 0.9;
};

/// A feed rate limit at one point along the path.
struct FeedLimitPoint {
    /// Arc length along the path (mm).
    double arcLength = 0.0;

    /// Certified curvature at this point (1/mm), or 0 if straight.
    double curvature = 0.0;

    /// Curvature-aware velocity limit (mm/s).
    double velocityLimit = 0.0;

    /// Curvature-aware feed rate limit (mm/min) = velocityLimit * 60.
    double feedRateLimit = 0.0;
};

/// Result of computing the curvature-aware feed limit curve.
struct CurvatureAwareFeedLimits {
    /// One FeedLimitPoint per sample point along the path.
    std::vector<FeedLimitPoint> points;

    /// The minimum feed rate across all points (mm/min).
    double minFeedRate = 0.0;

    /// The maximum feed rate across all points (mm/min).
    double maxFeedRate = 0.0;

    /// The arc length at which the minimum feed rate occurs (mm).
    double minFeedRateArcLength = 0.0;
};

/**
 * @brief Compute the curvature-aware feed rate limit at a single point.
 *
 * @param curvature Curvature κ at the point (1/mm).
 * @param config Limiter configuration.
 * @return The feed rate limit (mm/min).
 */
double computeFeedRateLimit(double curvature, const CurvatureLimiterConfig& config);

/**
 * @brief Compute the curvature-aware feed limit from TrajectorySample data.
 *
 * This is the "immediate" level — uses the curvature field already present
 * in each TrajectorySample. No certified curvature sampling is performed;
 * the curvature is taken as-is from the trajectory analyzer.
 *
 * @param samples The trajectory samples with curvature populated.
 * @param config Limiter configuration.
 * @return The feed limit curve, one point per sample.
 */
CurvatureAwareFeedLimits computeFeedLimitsFromSamples(
    const std::vector<GCodeExport::TrajectorySample>& samples,
    const CurvatureLimiterConfig& config = {});

/**
 * @brief Compute the certified curvature-aware feed limit curve.
 *
 * This is the "certified" level — uses CertifiedCurvatureSampler to get
 * a guaranteed upper bound on the curvature per span, then builds the
 * feed limit curve at the requested resolution.
 *
 * @param path The desired path.
 * @param config Limiter configuration.
 * @param numSamples Number of sample points along the path.
 * @return The certified feed limit curve.
 */
CurvatureAwareFeedLimits computeCertifiedFeedLimits(
    const PiecewiseNurbsPath& path,
    const CurvatureLimiterConfig& config = {},
    std::size_t numSamples = 100);

/**
 * @brief Get the certified feed rate limit at a specific arc length.
 *
 * @param path The desired path.
 * @param arcLength The arc length position (mm).
 * @param config Limiter configuration.
 * @return The feed rate limit (mm/min).
 */
double certifiedFeedRateAt(
    const PiecewiseNurbsPath& path,
    double arcLength,
    const CurvatureLimiterConfig& config = {});

} // namespace tether::motion::replanner
