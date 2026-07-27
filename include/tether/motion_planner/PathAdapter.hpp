/**
 * @file PathAdapter.hpp
 * @brief Adapter bridging the new `tether::motion` geometry core to the
 *        old `MotionPlanner` template API used by MotionPlan/VelocityProfile.
 *
 * @details
 * The new geometry core (`tether::motion::PiecewiseNurbsPath`) uses
 * runtime dimension (`RVec`) and has no source-reference tracking. The
 * old `MotionPlanner::PiecewiseNURBSPath<Dim, T>` was templated and
 * carried `SourceReference` per segment.
 *
 * This adapter wraps the new type and provides the old API so that
 * `MotionPlan` and `VelocityProfile` can use the new geometry core
 * with minimal changes. Source references are stored alongside the
 * pieces.
 *
 * The adapter also provides a `PathBuilderAdapter` that replaces the
 * old `PathBuilder<Dim, T>`, using `SegmentConverter` + `PathBlender`
 * to build the path.
 */
#pragma once

#include "tether/motion_planner/MathTypes.hpp"
#include "tether/motion_planner/MotionSegment.hpp"
#include "tether/motion_planner/SourceReference.hpp"
#include "tether/motion_planner/blend/BlendSpec.hpp"
#include "tether/motion_planner/blend/PathBlender.hpp"
#include "tether/motion_planner/blend/PHQuinticBlendBuilder.hpp"
#include "tether/motion_planner/blend/SegmentConverter.hpp"
#include "tether/motion_planner/geometry/CertifiedCurvatureSampler.hpp"
#include "tether/motion_planner/geometry/PiecewiseNurbsPath.hpp"

#include <memory>
#include <optional>
#include <vector>

namespace MotionPlanner {

/// Evaluation result matching the old `NURBSPathEvaluation<Dim, T>`.
template <size_t Dim, typename T = double>
struct PathAdapterEvaluation {
    Vec<Dim, T> position;
    Vec<Dim, T> tangent;
    Vec<Dim, T> velocity;
    Vec<Dim, T> acceleration;
    Vec<Dim, T> normal;
    Vec<Dim, T> jerk;
    T arcLength = T(0);
    T globalParameter = T(0);
    T localParameter = T(0);
    T curvature = T(0);
    size_t segmentIndex = 0;
    SourceReference sourceRef;
};

/// A segment in the adapter path, matching the old `NURBSPathSegmentInfo`.
template <size_t Dim, typename T = double>
struct PathAdapterSegment {
    SourceReference sourceRef;
    T cumulativeArcLength = T(0);
    T arcLength = T(0);
    // The underlying NURBS piece (reference into the PiecewiseNurbsPath).
    // We don't expose the curve directly; callers use the path API.
};

/// Adapter wrapping `tether::motion::PiecewiseNurbsPath` with the old
/// `PiecewiseNURBSPath<Dim, T>` API.
template <size_t Dim, typename T = double>
class PathAdapter {
public:
    using Point = Vec<Dim, T>;
    using Evaluation = PathAdapterEvaluation<Dim, T>;
    using SegmentInfo = PathAdapterSegment<Dim, T>;

    PathAdapter() = default; // empty adapter; path_ is nullopt.

    /// Construct from a `tether::motion::PiecewiseNurbsPath` and a list
    /// of source references (one per piece).
    explicit PathAdapter(tether::motion::PiecewiseNurbsPath path,
                         std::vector<SourceReference> sourceRefs = {})
        : path_(std::move(path))
        , sourceRefs_(std::move(sourceRefs)) {
        if (sourceRefs_.size() < path_->numPieces()) {
            sourceRefs_.resize(path_->numPieces());
        }
        phData_.resize(path_->numPieces()); // all empty by default
        rebuildSegmentInfo();
    }

    /// Construct with PHData sidecar (Phase 5.4 fast path).
    explicit PathAdapter(tether::motion::PiecewiseNurbsPath path,
                         std::vector<SourceReference> sourceRefs,
                         std::vector<std::optional<tether::motion::PHData>> phData)
        : path_(std::move(path))
        , sourceRefs_(std::move(sourceRefs))
        , phData_(std::move(phData)) {
        if (sourceRefs_.size() < path_->numPieces()) {
            sourceRefs_.resize(path_->numPieces());
        }
        if (phData_.size() < path_->numPieces()) {
            phData_.resize(path_->numPieces());
        }
        rebuildSegmentInfo();
    }

    // ========================================================================
    // Segment Management (old API)
    // ========================================================================

    size_t numSegments() const noexcept {
        return path_ ? path_->numPieces() : 0;
    }
    bool empty() const noexcept { return numSegments() == 0; }

    const SegmentInfo& getSegment(size_t index) const {
        return segments_.at(index);
    }

    const std::vector<SegmentInfo>& segments() const noexcept {
        return segments_;
    }

    /// Access the underlying new-type path.
    const tether::motion::PiecewiseNurbsPath& inner() const {
        return *path_;
    }

    /// True if the adapter holds a real path (not default-constructed).
    bool hasInner() const noexcept { return path_.has_value(); }

    /// Access the lazy certified curvature sampler for this path.
    /// The sampler is constructed on first call and memoized; it is
    /// bound to the underlying PiecewiseNurbsPath and samples per span
    /// on demand. See CertifiedCurvatureSampler.hpp for the algorithm.
    /// @throws std::runtime_error if the adapter is empty (no path).
    const tether::motion::CertifiedCurvatureSampler& curvatureSampler() const {
        if (!path_) {
            throw std::runtime_error(
                "PathAdapter::curvatureSampler: adapter is empty");
        }
        if (!curvatureSampler_) {
            curvatureSampler_ =
                std::make_unique<tether::motion::CertifiedCurvatureSampler>(
                    *path_);
        }
        return *curvatureSampler_;
    }

    /// PHData sidecar per piece: phData(i) is populated iff piece i is a
    /// PH quintic blend curve. When present, enables closed-form arc
    /// length / curvature (M16–M19, Phase 5.4 fast path).
    /// Returns an empty optional if the adapter is empty or the piece
    /// has no PHData.
    const std::optional<tether::motion::PHData>& phData(
        std::size_t pieceIndex) const {
        if (pieceIndex < phData_.size()) return phData_[pieceIndex];
        static const std::optional<tether::motion::PHData> empty;
        return empty;
    }

    /// True if any piece has PHData (i.e. PH fast path is available).
    bool hasPHData() const noexcept {
        for (const auto& ph : phData_) if (ph) return true;
        return false;
    }

    // ========================================================================
    // Arc Length (old API)
    // ========================================================================

    T totalLength() const noexcept {
        return path_ ? static_cast<T>(path_->totalLength()) : T(0);
    }

    T totalArcLength() const noexcept {
        return totalLength();
    }

    // ========================================================================
    // Evaluation (old API)
    // ========================================================================

    Evaluation evaluateAtArcLength(T s) const {
        return evaluateFullAtArcLength(s);
    }

    T curvatureAtArcLength(T s) const {
        return evaluateFullAtArcLength(s).curvature;
    }

    Evaluation evaluateFullAtArcLength(T s) const {
        Evaluation result;
        result.arcLength = s;

        if (!path_ || path_->numPieces() == 0) return result;

        // Clamp s to [0, totalLength].
        const T total = totalLength();
        if (s < T(0)) s = T(0);
        if (s > total) s = total;

        // Locate the piece.
        auto loc = path_->locate(static_cast<double>(s));
        const size_t segIndex = loc.piece;
        const double localS = loc.localS;

        result.segmentIndex = segIndex;
        result.localParameter = static_cast<T>(localS);
        if (segIndex < sourceRefs_.size()) {
            result.sourceRef = sourceRefs_[segIndex];
        }

        // Evaluate using the new geometry core.
        // Order 2 gives position, tangent, curvature.
        //
        // Phase 5.4 PH fast path: when this piece carries PHData, use the
        // (M19) polynomial Newton inversion (s → ξ) instead of the
        // NurbsCurve quadrature-based invertLength. The position/tangent/
        // curvature are then evaluated on the NurbsCurve at the
        // corresponding parameter u — same curve, faster inversion.
        tether::motion::ArcDerivatives derivs;
        if (segIndex < phData_.size() && phData_[segIndex]) {
            const auto& ph = *phData_[segIndex];
            const double xi = tether::motion::PHQuinticBlendBuilder::invertArcLength(ph, localS);
            const auto& piece = path_->piece(segIndex);
            const double uMin = piece.knotMin();
            const double uMax = piece.knotMax();
            const double u = uMin + xi * (uMax - uMin);
            derivs = piece.arcDerivatives(u, 2);
        } else {
            derivs = path_->evaluate(static_cast<double>(s), 2);
        }
        result.position = toVec<Dim, T>(derivs.position);
        result.tangent = toVec<Dim, T>(derivs.tangent);
        result.velocity = result.tangent;
        result.curvature = static_cast<T>(derivs.curvature.norm());
        // normal = curvature direction (if nonzero).
        const double kappaNorm = derivs.curvature.norm();
        if (kappaNorm > 1e-12) {
            tether::motion::RVec n = derivs.curvature * (1.0 / kappaNorm);
            result.normal = toVec<Dim, T>(n);
        }

        if (total > T(0)) {
            result.globalParameter = s / total;
        }

        return result;
    }

    Point evaluatePositionAtArcLength(T s) const {
        return evaluateFullAtArcLength(s).position;
    }

    SourceReference sourceRefAtArcLength(T s) const {
        return evaluateFullAtArcLength(s).sourceRef;
    }

    /// Find arc length position for a given G-code line number.
    std::optional<T> findArcLengthForLine(size_t lineNumber) const {
        for (const auto& seg : segments_) {
            if (seg.sourceRef.type() == SourceReference::Type::Single &&
                seg.sourceRef.lineNumber() == lineNumber) {
                return static_cast<T>(seg.cumulativeArcLength);
            }
        }
        return std::nullopt;
    }

    // ========================================================================
    // Iterators (old API)
    // ========================================================================

    class ForwardIterator {
    public:
        ForwardIterator(const PathAdapter* path, T arcLength)
            : path_(path), currentArcLength_(arcLength) {}
        void advance(T distance) {
            currentArcLength_ = std::clamp(currentArcLength_ + distance,
                                           T(0), path_->totalArcLength());
        }
        Point position() const {
            return path_->evaluateAtArcLength(currentArcLength_).position;
        }
        T arcLength() const { return currentArcLength_; }
        bool atEnd() const {
            return currentArcLength_ >= path_->totalArcLength();
        }
    private:
        const PathAdapter* path_;
        T currentArcLength_;
    };

    ForwardIterator forwardIterator(T startArcLength = T(0)) const {
        return ForwardIterator(this, startArcLength);
    }

private:
    std::optional<tether::motion::PiecewiseNurbsPath> path_;
    std::vector<SourceReference> sourceRefs_;
    std::vector<SegmentInfo> segments_;
    std::vector<std::optional<tether::motion::PHData>> phData_;
    mutable std::unique_ptr<tether::motion::CertifiedCurvatureSampler>
        curvatureSampler_; ///< Lazy, memoized.

    void rebuildSegmentInfo() {
        segments_.clear();
        if (!path_) return;
        segments_.reserve(path_->numPieces());
        double cumLen = 0.0;
        for (size_t i = 0; i < path_->numPieces(); ++i) {
            SegmentInfo info;
            info.cumulativeArcLength = static_cast<T>(cumLen);
            const double pieceLen = path_->piece(i).length();
            info.arcLength = static_cast<T>(pieceLen);
            if (i < sourceRefs_.size()) {
                info.sourceRef = sourceRefs_[i];
            }
            segments_.push_back(info);
            cumLen += pieceLen;
        }
    }

    /// Convert `RVec` to `Vec<Dim, T>`, zero-padding if the RVec has
    /// fewer dimensions than Dim.
    template <size_t D, typename U>
    static Vec<D, U> toVec(const tether::motion::RVec& rv) {
        Vec<D, U> result;
        const std::size_t n = std::min<std::size_t>(rv.dim(), D);
        for (std::size_t i = 0; i < n; ++i) {
            result[i] = static_cast<U>(rv[i]);
        }
        return result;
    }
};

/// Result of building a path from segments.
template <size_t Dim, typename T = double>
struct PathAdapterBuildResult {
    bool success = false;
    PathAdapter<Dim, T> path;
    std::string errorMessage;
};

/// Embed a low-dimensional NURBS curve into the full Dim-dimensional axis
/// space, placing the active-axis components at their correct positions
/// and filling the inactive axes with zeros. This is needed because
/// SegmentConverter extracts only the active axes, producing minimal-dim
/// curves whose component indices don't correspond to the global axis
/// indices.
inline tether::motion::NurbsCurve embedCurveInFullSpace(
    const tether::motion::NurbsCurve& c,
    const MotionPlanner::MotionSegment& seg,
    std::size_t targetDim) {
    // Collect the active axis indices in order.
    std::vector<std::size_t> activeIdx;
    for (std::size_t i = 0; i < MotionPlanner::MAX_MOTION_AXES; ++i) {
        if (seg.activeAxes[i]) activeIdx.push_back(i);
    }
    // The curve's components correspond to the active axes in order.
    // If the curve dim doesn't match the number of active axes, something
    // is wrong — return the curve unchanged (let the caller handle it).
    if (activeIdx.size() != c.dim()) {
        return c;
    }

    // Rebuild the control points in the full target dimension.
    // Inactive axes are filled with the segment's start position value
    // for that axis (the axis is constant throughout the segment).
    const auto& cps = c.controlPoints();
    const auto& wts = c.weights();
    const auto& knots = c.knots();
    std::vector<tether::motion::RVec> newCps(cps.size());
    for (std::size_t i = 0; i < cps.size(); ++i) {
        tether::motion::RVec p = tether::motion::RVec::zero(targetDim);
        // Fill inactive axes with the segment's start position.
        for (std::size_t a = 0; a < targetDim && a < MotionPlanner::MAX_MOTION_AXES; ++a) {
            if (!seg.activeAxes[a]) {
                p[a] = seg.startPosition[a];
            }
        }
        // Fill active axes with the curve's components.
        for (std::size_t j = 0; j < activeIdx.size() && j < cps[i].dim(); ++j) {
            if (activeIdx[j] < targetDim) {
                p[activeIdx[j]] = cps[i][j];
            }
        }
        newCps[i] = p;
    }
    return tether::motion::NurbsCurve(newCps, wts, knots, c.degree());
}

/// Adapter replacing `PathBuilder<Dim, T>`. Uses `SegmentConverter` +
/// `PathBlender` to build the path from `MotionSegmentList`.
template <size_t Dim, typename T = double>
class PathBuilderAdapter {
public:
    using BuildResult = PathAdapterBuildResult<Dim, T>;

    /// Build a path from a list of motion segments.
    /// @param segments The motion segments (from G-code parser).
    /// @param blendSpec The blend specification (tolerance, etc.).
    ///   If tolerance is 0, no blending is applied (ExactPath mode).
    /// @return The build result with the path and source references.
    BuildResult build(const MotionSegmentList& segments,
                      const tether::motion::BlendSpec& blendSpec = {}) {
        if (segments.empty()) {
            return {false, {}, "no segments"};
        }

        // Step 1: Convert segments to NURBS curves.
        std::vector<tether::motion::NurbsCurve> curves;
        std::vector<SourceReference> sourceRefs;
        try {
            for (size_t i = 0; i < segments.size(); ++i) {
                const auto& seg = segments.at(i);
                auto curve = tether::motion::SegmentConverter::convert(seg);
                if (curve) {
                    curves.push_back(std::move(*curve));
                    sourceRefs.push_back(seg.sourceRef);
                }
                // Dwell segments are skipped (no geometry).
            }
        } catch (const std::exception& e) {
            return {false, {}, std::string("SegmentConverter: ") + e.what()};
        }

        if (curves.empty()) {
            return {false, {}, "no geometry (all dwell?)"};
        }

        // Step 1b: Embed all curves in a common axis space.
        // The SegmentConverter extracts only the active axes, so different
        // segments may produce curves of different dimensions and in
        // different axis subspaces (e.g. a pure-X move gives a 1D curve
        // whose component 0 is X, while a pure-Y move gives a 1D curve
        // whose component 0 is Y). PiecewiseNurbsPath requires all pieces
        // to have the same dimension, and the components must correspond
        // to the same axes.
        //
        // Fix: embed each curve into the full Dim-dimensional axis space,
        // placing the active-axis components at their correct positions
        // and filling the inactive axes with zeros.
        if (curves.size() > 1) {
            std::size_t curveIdx = 0;
            for (size_t i = 0; i < segments.size() && curveIdx < curves.size(); ++i) {
                const auto& seg = segments.at(i);
                // Skip dwell segments (no curve produced).
                if (seg.type == MotionPlanner::MotionSegmentType::Dwell) continue;
                if (curves[curveIdx].dim() < Dim) {
                    curves[curveIdx] = embedCurveInFullSpace(
                        curves[curveIdx], seg, Dim);
                }
                ++curveIdx;
            }
        }

        if (curves.size() < 2) {
            // Single piece — no blending needed.
            tether::motion::PiecewiseNurbsPath path(std::move(curves));
            return {true, PathAdapter<Dim, T>(std::move(path),
                                               std::move(sourceRefs)), ""};
        }

        // Step 2: Build a PiecewiseNurbsPath and blend.
        // Copy curves (we need them for source ref mapping below if
        // the blend fails; the blend itself takes a const reference).
        tether::motion::PiecewiseNurbsPath rawPath(
            std::vector<tether::motion::NurbsCurve>(curves.begin(),
                                                    curves.end()));

        tether::motion::PathBlender blender;
        tether::motion::BlendedPath blended = blender.blend(rawPath,
                                                            blendSpec);

        // Step 3: Build the source reference list for the blended path.
        // The blended path has more pieces than the original (blends
        // inserted). We need to map each blended piece to a source ref.
        // Strategy: walk the audit trail and assign source refs.
        // Trimmed original pieces keep their original source ref.
        // Blend curves get a synthetic source ref combining the two
        // adjacent segments.
        std::vector<SourceReference> blendedRefs;
        size_t origIdx = 0;
        for (const auto& entry : blended.audit) {
            // The original piece `entry.cornerIndex` is trimmed.
            // Assign its source ref to the trimmed piece.
            if (entry.cornerIndex < sourceRefs.size()) {
                blendedRefs.push_back(sourceRefs[entry.cornerIndex]);
            } else {
                blendedRefs.push_back(SourceReference::synthetic("trimmed"));
            }
            // If the corner was blended, the blend curve gets a
            // combined source ref.
            if (entry.geometry.outcome ==
                tether::motion::BlendOutcome::Blended) {
                std::vector<SourceReference> refs;
                if (entry.cornerIndex < sourceRefs.size()) {
                    refs.push_back(sourceRefs[entry.cornerIndex]);
                }
                if (entry.cornerIndex + 1 < sourceRefs.size()) {
                    refs.push_back(sourceRefs[entry.cornerIndex + 1]);
                }
                blendedRefs.push_back(
                    SourceReference::multiple(refs));
            }
        }
        // Last original piece (if not consumed).
        if (!blended.audit.empty()) {
            size_t lastIdx = blended.audit.back().cornerIndex + 1;
            if (lastIdx < sourceRefs.size()) {
                blendedRefs.push_back(sourceRefs[lastIdx]);
            }
        }

        // Fallback: if the source ref mapping didn't produce enough
        // entries, pad with synthetic refs.
        while (blendedRefs.size() < blended.pieces.size()) {
            blendedRefs.push_back(
                SourceReference::synthetic("blend"));
        }

        // Ensure the PHData sidecar matches the piece count.
        while (blended.phData.size() < blended.pieces.size()) {
            blended.phData.emplace_back(); // empty optional
        }

        tether::motion::PiecewiseNurbsPath blendedPath(
            std::move(blended.pieces));

        return {true, PathAdapter<Dim, T>(std::move(blendedPath),
                                           std::move(blendedRefs),
                                           std::move(blended.phData)), ""};
    }
};

} // namespace MotionPlanner
