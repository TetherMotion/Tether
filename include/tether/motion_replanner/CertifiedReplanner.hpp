/**
 * @file CertifiedReplanner.hpp
 * @brief Unified entry point for the certified motion replanner
 *
 * @details
 * This header provides a single include point for all the certified
 * replanner modules built in Phases 1–7, plus a backward-compatibility
 * layer that brings the new tether::motion::replanner types into the
 * legacy MotionReplanner namespace.
 *
 * ## Namespace convergence (Phase 8)
 *
 * The new certified modules live in `tether::motion::replanner` per
 * Architecture.md §3. The legacy modules (MotionReplanner, MachineTester,
 * PerformanceHeatmap, SystemIdentifier, GCodeGenerator, TestDataExporter)
 * still live in `namespace MotionReplanner` for backward compatibility
 * with existing examples and python bindings.
 *
 * This header provides `using` declarations so that code using the old
 * `MotionReplanner::` namespace can also access the new certified types
 * without changing their includes. The full namespace migration
 * (renaming `MotionReplanner` to `tether::motion::replanner` in all
 * headers) is deferred to a future phase to avoid a flag-day rename.
 *
 * ## Usage
 *
 * ```cpp
 * #include "tether/motion_replanner/CertifiedReplanner.hpp"
 *
 * // Use the new certified types:
 * using namespace tether::motion::replanner;
 *
 * auto path = convertTrajectory(samples);
 * auto err = computeCertifiedContourError(path, actualPos, 50.0);
 * auto corners = detectCorners(path);
 * auto feedLimits = computeCertifiedFeedLimits(path);
 * auto suggestion = solveCertifiedFeedRate(6000.0, err.contourError);
 * auto reblend = reblend(path);
 * auto profile = replanProfile(path, 6000.0);
 *
 * // Or via the legacy namespace (backward compat):
 * using namespace MotionReplanner;
 * // MotionReplanner::convertTrajectory, etc. also work.
 * ```
 *
 * ## Phase summary
 *
 * | Phase | Module | Key Type |
 * |-------|--------|----------|
 * | 1 | TrajectorySampleConverter | PiecewiseNurbsPath |
 * | 2 | CertifiedContourError | CertifiedContourError |
 * | 3 | CertifiedCornerDetection | CertifiedCornerDetection |
 * | 4 | CurvatureAwareLimiter | CurvatureAwareFeedLimits |
 * | 5 | CertifiedSuggestionSolver | CertifiedSuggestion |
 * | 6 | OnlineReblender | ReblendResult |
 * | 7 | ProfileReplanner | ProfileReplanResult |
 */

#pragma once

// Phase 1: TrajectorySample → PiecewiseNurbsPath bridge
#include "tether/motion_replanner/TrajectorySampleConverter.hpp"

// Phase 2: Certified contour error via pointCurveDistance
#include "tether/motion_replanner/CertifiedContourError.hpp"

// Phase 3: Certified corner detection via CornerAnalyzer
#include "tether/motion_replanner/CertifiedCornerDetection.hpp"

// Phase 4: Curvature-aware proactive feed limiting
#include "tether/motion_replanner/CurvatureAwareLimiter.hpp"

// Phase 5: Certified limit suggestions via M15-pattern bisection
#include "tether/motion_replanner/CertifiedSuggestionSolver.hpp"

// Phase 6: Online re-blending via PathBlender
#include "tether/motion_replanner/OnlineReblender.hpp"

// Phase 7: Re-plan velocity profile + S-curve transitions
#include "tether/motion_replanner/ProfileReplanner.hpp"

// ============================================================================
// Backward-compatibility: bring new types into the legacy namespace
// ============================================================================

namespace MotionReplanner {

// Phase 1
using tether::motion::replanner::SegmentToPieceMap;
using tether::motion::replanner::ConverterConfig;
using tether::motion::replanner::convertTrajectory;

// Phase 2
using tether::motion::replanner::CertifiedContourError;
using tether::motion::replanner::computeCertifiedContourError;
using tether::motion::replanner::computeCertifiedContourErrorLocal;

// Phase 3
using tether::motion::replanner::CertifiedJunction;
using tether::motion::replanner::CertifiedCornerDetection;
using tether::motion::replanner::detectCorners;

// Phase 4
using tether::motion::replanner::CurvatureLimiterConfig;
using tether::motion::replanner::FeedLimitPoint;
using tether::motion::replanner::CurvatureAwareFeedLimits;
using tether::motion::replanner::computeFeedRateLimit;
using tether::motion::replanner::computeFeedLimitsFromSamples;
using tether::motion::replanner::computeCertifiedFeedLimits;
using tether::motion::replanner::certifiedFeedRateAt;

// Phase 5
using tether::motion::replanner::SuggestionSolverConfig;
using tether::motion::replanner::CertifiedSuggestion;
using tether::motion::replanner::solveCertifiedFeedRate;
using tether::motion::replanner::solveCertifiedFeedRateWithCurvature;

// Phase 6
using tether::motion::replanner::ReblenderConfig;
using tether::motion::replanner::ReblendResult;
using tether::motion::replanner::reblend;
using tether::motion::replanner::reblendWithSpec;
using tether::motion::replanner::reblendJunctions;
using tether::motion::replanner::extractPath;

// Phase 7
using tether::motion::replanner::ProfileLimits;
using tether::motion::replanner::ProfilePoint;
using tether::motion::replanner::ProfileReplanResult;
using tether::motion::replanner::replanProfile;
using tether::motion::replanner::computeSCurveTransition;

} // namespace MotionReplanner
