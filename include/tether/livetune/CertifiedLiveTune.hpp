/**
 * @file CertifiedLiveTune.hpp
 * @brief Unified entry point for the certified motion livetune
 *
 * @details
 * This header provides a single include point for all the certified
 * livetune modules built in Phases 1–7, plus a backward-compatibility
 * layer that brings the new tether::motion::livetune types into the
 * legacy OnlineTuner namespace.
 *
 * ## Namespace convergence (Phase 8)
 *
 * The new certified modules live in `tether::motion::livetune` per
 * Architecture.md §3. The legacy modules (OnlineTuner, MachineTester,
 * PerformanceHeatmap, SystemIdentifier, GCodeGenerator, TestDataExporter)
 * still live in `namespace LiveTune` for backward compatibility
 * with existing examples and python bindings.
 *
 * This header provides `using` declarations so that code using the old
 * `LiveTune::` namespace can also access the new certified types
 * without changing their includes. The full namespace migration
 * (renaming `OnlineTuner` to `tether::motion::livetune` in all
 * headers) is deferred to a future phase to avoid a flag-day rename.
 *
 * ## Usage
 *
 * ```cpp
 * #include "tether/livetune/CertifiedLiveTune.hpp"
 *
 * // Use the new certified types:
 * using namespace tether::motion::livetune;
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
 * using namespace LiveTune;
 * // LiveTune::convertTrajectory, etc. also work.
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
#include "tether/livetune/TrajectorySampleConverter.hpp"

// Phase 2: Certified contour error via pointCurveDistance
#include "tether/livetune/CertifiedContourError.hpp"

// Phase 3: Certified corner detection via CornerAnalyzer
#include "tether/livetune/CertifiedCornerDetection.hpp"

// Phase 4: Curvature-aware proactive feed limiting
#include "tether/livetune/CurvatureAwareLimiter.hpp"

// Phase 5: Certified limit suggestions via M15-pattern bisection
#include "tether/livetune/CertifiedSuggestionSolver.hpp"

// Phase 6: Online re-blending via PathBlender
#include "tether/livetune/OnlineReblender.hpp"

// Phase 7: Re-plan velocity profile + S-curve transitions
#include "tether/livetune/ProfileReplanner.hpp"

// ============================================================================
// Backward-compatibility: bring new types into the legacy namespace
// ============================================================================

namespace LiveTune {

// Phase 1
using tether::motion::livetune::SegmentToPieceMap;
using tether::motion::livetune::ConverterConfig;
using tether::motion::livetune::convertTrajectory;

// Phase 2
using tether::motion::livetune::CertifiedContourError;
using tether::motion::livetune::computeCertifiedContourError;
using tether::motion::livetune::computeCertifiedContourErrorLocal;

// Phase 3
using tether::motion::livetune::CertifiedJunction;
using tether::motion::livetune::CertifiedCornerDetection;
using tether::motion::livetune::detectCorners;

// Phase 4
using tether::motion::livetune::CurvatureLimiterConfig;
using tether::motion::livetune::FeedLimitPoint;
using tether::motion::livetune::CurvatureAwareFeedLimits;
using tether::motion::livetune::computeFeedRateLimit;
using tether::motion::livetune::computeFeedLimitsFromSamples;
using tether::motion::livetune::computeCertifiedFeedLimits;
using tether::motion::livetune::certifiedFeedRateAt;

// Phase 5
using tether::motion::livetune::SuggestionSolverConfig;
using tether::motion::livetune::CertifiedSuggestion;
using tether::motion::livetune::solveCertifiedFeedRate;
using tether::motion::livetune::solveCertifiedFeedRateWithCurvature;

// Phase 6
using tether::motion::livetune::ReblenderConfig;
using tether::motion::livetune::ReblendResult;
using tether::motion::livetune::reblend;
using tether::motion::livetune::reblendWithSpec;
using tether::motion::livetune::reblendJunctions;
using tether::motion::livetune::extractPath;

// Phase 7
using tether::motion::livetune::ProfileLimits;
using tether::motion::livetune::ProfilePoint;
using tether::motion::livetune::ProfileReplanResult;
using tether::motion::livetune::replanProfile;
using tether::motion::livetune::computeSCurveTransition;

} // namespace LiveTune
