/**
 * @file OnlineReblender.cpp
 * @brief Implementation of online re-blending via PathBlender
 */

#include "tether/motion_replanner/OnlineReblender.hpp"

#include <sstream>
#include <stdexcept>

namespace tether::motion::replanner {

namespace {

/// Build a BlendSpec from the reblender config.
BlendSpec buildSpec(const ReblenderConfig& config, bool tight) {
    BlendSpec spec;
    spec.mode = PathMode::Blend;
    spec.tolerance = tight ? config.tightTolerance : config.defaultTolerance;
    spec.continuity = tight ? config.tightContinuity : config.defaultContinuity;
    spec.curveType = config.defaultCurveType;
    spec.maxBlendFraction = config.maxBlendFraction;
    spec.minSegmentLength = config.minSegmentLength;
    spec.validate();
    return spec;
}

/// Build a summary string from the BlendedPath.
std::string buildSummary(const BlendedPath& bp) {
    std::ostringstream oss;
    oss << "Blended: " << bp.blendedCount
        << ", ExactStop: " << bp.exactStopCount
        << ", Straight: " << bp.straightCount
        << ", Pieces: " << bp.pieces.size();
    return oss.str();
}

} // anonymous namespace

ReblendResult reblendWithSpec(
    const PiecewiseNurbsPath& path,
    const BlendSpec& spec) {

    PathBlender blender;
    BlendedPath bp = blender.blend(path, spec);

    ReblendResult result;
    result.blendedPath = std::move(bp);
    result.reblended = (result.blendedPath.blendedCount > 0);
    result.blendedCount = result.blendedPath.blendedCount;
    result.exactStopCount = result.blendedPath.exactStopCount;
    result.straightCount = result.blendedPath.straightCount;
    result.summary = buildSummary(result.blendedPath);
    return result;
}

ReblendResult reblend(
    const PiecewiseNurbsPath& path,
    const ReblenderConfig& config) {

    BlendSpec spec = buildSpec(config, false);
    return reblendWithSpec(path, spec);
}

ReblendResult reblendJunctions(
    const PiecewiseNurbsPath& path,
    const std::vector<std::size_t>& problematicJunctions,
    const ReblenderConfig& config) {

    // PathBlender currently applies one spec template to all junctions.
    // For per-junction specs, we use the tight spec if any problematic
    // junctions are specified (conservative: tighten all junctions).
    // A future PathBlender extension could accept per-junction specs.

    if (problematicJunctions.empty()) {
        return reblend(path, config);
    }

    // Validate junction indices.
    std::size_t maxJunction = path.numPieces() - 1;
    for (std::size_t idx : problematicJunctions) {
        if (idx >= maxJunction) {
            throw std::invalid_argument(
                "Junction index " + std::to_string(idx) +
                " out of range (max: " + std::to_string(maxJunction - 1) + ")");
        }
    }

    // Use the tight spec for the whole path (conservative).
    // This tightens all junctions, not just the problematic ones.
    // A future implementation could do multiple passes with different
    // specs and merge the results.
    BlendSpec tightSpec = buildSpec(config, true);
    ReblendResult result = reblendWithSpec(path, tightSpec);

    // Mark which junctions were the problematic ones in the summary.
    std::ostringstream oss;
    oss << result.summary << " (tightened junctions: ";
    for (std::size_t i = 0; i < problematicJunctions.size(); ++i) {
        if (i > 0) oss << ", ";
        oss << problematicJunctions[i];
    }
    oss << ")";
    result.summary = oss.str();

    return result;
}

std::optional<PiecewiseNurbsPath> extractPath(
    const BlendedPath& blended) {

    if (blended.pieces.empty()) {
        return std::nullopt;
    }

    return PiecewiseNurbsPath(blended.pieces);
}

} // namespace tether::motion::replanner
