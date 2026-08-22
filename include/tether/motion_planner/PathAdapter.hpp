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

#include <algorithm>
#include <cmath>
#include <limits>
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

    /// Per-segment maximum velocity (from G-code F-value, mm/s).
    /// Default +infinity means "not set" — the global feedRate is used.
    T maxVelocity = std::numeric_limits<T>::infinity();

    /// Corner velocity at the segment start (junction with previous segment).
    /// Default +infinity means "no corner limit".
    T entryCornerVelocity = std::numeric_limits<T>::infinity();

    /// Corner velocity at the segment end (junction with next segment).
    /// Default +infinity means "no corner limit".
    T exitCornerVelocity = std::numeric_limits<T>::infinity();
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
                std::make_shared<tether::motion::CertifiedCurvatureSampler>(
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

    // ========================================================================
    // Per-Segment Velocity Limits (opt-in)
    // ========================================================================

    /// True if per-segment velocity limits (feed rates and/or corner
    /// velocities) have been set via setSegmentVelocityLimits() or
    /// computeCornerVelocities().
    bool hasPerSegmentVelocityLimits() const noexcept {
        return hasPerSegmentLimits_;
    }

    /// Set per-segment maximum velocities (from G-code F-values).
    /// @param feedRates One entry per segment, in mm/s. The vector size
    ///        must match numSegments(). Values <= 0 are treated as
    ///        "no limit" (infinity).
    void setSegmentVelocityLimits(const std::vector<double>& feedRates) {
        if (!path_ || feedRates.empty()) return;
        segmentMaxVelocities_.assign(path_->numPieces(),
                                     std::numeric_limits<double>::infinity());
        const size_t n = std::min(feedRates.size(), segmentMaxVelocities_.size());
        for (size_t i = 0; i < n; ++i) {
            if (feedRates[i] > 0.0) {
                segmentMaxVelocities_[i] = feedRates[i];
            }
        }
        hasPerSegmentLimits_ = true;
        updateSegmentVelocityFields();
    }

    /// Compute corner velocities at all junctions using the junction
    /// deviation model (Marlin-style):
    ///
    ///   v_corner = sqrt(a_max * delta / sin(theta/2))
    ///
    /// where theta is the junction angle between consecutive segment
    /// tangents, delta is the allowed junction deviation, and a_max is
    /// the maximum centripetal acceleration.
    ///
    /// For collinear segments (theta = 0), no corner limit is applied.
    /// For 180-degree reversals, v_corner = sqrt(a_max * delta).
    ///
    /// @param junctionDeviation Allowed junction deviation (mm). Typical
    ///        values: 0.01–0.1 mm. 0 disables cornering.
    /// @param maxCentripetalAccel Maximum centripetal acceleration (mm/s²).
    void computeCornerVelocities(double junctionDeviation,
                                 double maxCentripetalAccel) {
        if (!path_ || path_->numPieces() == 0) return;
        if (junctionDeviation <= 0.0 || maxCentripetalAccel <= 0.0) return;

        const size_t N = path_->numPieces();
        cornerVelocities_.assign(N + 1,
                                 std::numeric_limits<double>::infinity());

        // Compute tangent for each segment from NURBS control points.
        // For degree-1 (polyline): tangent = (endPoint - startPoint) / len.
        // For arcs: tangent = derivative at midpoint, normalized.
        std::vector<tether::motion::RVec> tangents(N);
        for (size_t i = 0; i < N; ++i) {
            const auto& piece = path_->piece(i);
            if (piece.isPolyline() && piece.numControlPoints() >= 2) {
                const auto& cps = piece.controlPoints();
                tether::motion::RVec diff = cps.back() - cps.front();
                double len = diff.norm();
                if (len > 1e-12) {
                    tangents[i] = diff / len;
                } else {
                    tangents[i] = tether::motion::RVec::zero(piece.dim());
                    tangents[i][0] = 1.0; // fallback
                }
            } else {
                // Arc or higher-degree: evaluate tangent at midpoint.
                try {
                    double uMid = 0.5 * (piece.knotMin() + piece.knotMax());
                    tether::motion::RVec d = piece.derivative(uMid, 1);
                    double len = d.norm();
                    if (len > 1e-12) {
                        tangents[i] = d / len;
                    } else {
                        tangents[i] = tether::motion::RVec::zero(piece.dim());
                        tangents[i][0] = 1.0;
                    }
                } catch (...) {
                    tangents[i] = tether::motion::RVec::zero(piece.dim());
                    tangents[i][0] = 1.0;
                }
            }
        }

        // Path start and end: no corner limit (boundary conditions).
        cornerVelocities_[0] = std::numeric_limits<double>::infinity();
        cornerVelocities_[N] = std::numeric_limits<double>::infinity();

        // Compute corner velocity at each interior junction.
        const double kMinSinHalfTheta = 1e-12;
        for (size_t i = 0; i + 1 < N; ++i) {
            double dot = tangents[i].dot(tangents[i + 1]);
            // Clamp dot product to [-1, 1] for numerical safety.
            dot = std::clamp(dot, -1.0, 1.0);
            // sin(theta/2) = sqrt((1 - cos(theta)) / 2) = sqrt((1 - dot) / 2)
            double sinHalfTheta = std::sqrt(std::max(0.0, (1.0 - dot) / 2.0));
            if (sinHalfTheta < kMinSinHalfTheta) {
                // Collinear — no corner limit.
                cornerVelocities_[i + 1] = std::numeric_limits<double>::infinity();
            } else {
                double vCorner = std::sqrt(
                    maxCentripetalAccel * junctionDeviation / sinHalfTheta);
                cornerVelocities_[i + 1] = vCorner;
            }
        }

        hasPerSegmentLimits_ = true;
        updateSegmentVelocityFields();
    }

    /// Get the per-segment velocity limit at arc length s.
    /// Combines the segment feed rate and the corner velocity at the
    /// nearest junction. Returns +infinity if per-segment limits are
    /// not set.
    double maxVelocityAtArcLength(T s) const {
        if (!hasPerSegmentLimits_) return std::numeric_limits<double>::infinity();
        const size_t N = segments_.size();
        if (N == 0) return std::numeric_limits<double>::infinity();

        // Clamp s to [0, totalLength].
        const T total = totalLength();
        if (s < T(0)) s = T(0);
        if (s > total) s = total;

        // Binary search for the segment containing s.
        size_t segIdx = segmentIndexAtArcLength(s);

        // Get the segment feed rate.
        double vSeg = std::numeric_limits<double>::infinity();
        if (segIdx < segmentMaxVelocities_.size()) {
            vSeg = segmentMaxVelocities_[segIdx];
        }

        // Get corner velocities at the segment boundaries.
        double vEntry = std::numeric_limits<double>::infinity();
        double vExit = std::numeric_limits<double>::infinity();
        if (!cornerVelocities_.empty()) {
            if (segIdx < cornerVelocities_.size()) {
                vEntry = cornerVelocities_[segIdx];
            }
            if (segIdx + 1 < cornerVelocities_.size()) {
                vExit = cornerVelocities_[segIdx + 1];
            }
        }

        // The velocity limit is the segment feed rate in the interior.
        // Corner velocities only apply at the exact junction between
        // segments — they are stamped onto the fine grid separately by
        // the solver's buildFineVelocityGrid() with backward/forward
        // propagation to create smooth deceleration/acceleration profiles.
        //
        // Do NOT linearly interpolate between entry and exit corner
        // velocities — that would give the corner velocity for the entire
        // segment when both corners have the same velocity, preventing
        // the solver from ever accelerating to the feed rate.
        return vSeg;
    }

    /// Get the segment index containing arc length s (binary search).
    size_t segmentIndexAtArcLength(T s) const {
        const size_t N = segments_.size();
        if (N == 0) return 0;
        if (s <= segments_[0].cumulativeArcLength) return 0;
        if (s >= segments_[N - 1].cumulativeArcLength + segments_[N - 1].arcLength)
            return N - 1;

        // Binary search.
        size_t lo = 0, hi = N - 1;
        while (lo < hi) {
            size_t mid = lo + (hi - lo) / 2;
            T segEnd = segments_[mid].cumulativeArcLength + segments_[mid].arcLength;
            if (s < segEnd) {
                hi = mid;
            } else {
                lo = mid + 1;
            }
        }
        return lo;
    }

    /// Direct access to per-segment max velocities vector (empty if not set).
    const std::vector<double>& segmentMaxVelocities() const noexcept {
        return segmentMaxVelocities_;
    }

    /// Direct access to corner velocities vector (empty if not set).
    /// Size = numSegments + 1 (one per junction, including path start/end).
    const std::vector<double>& cornerVelocities() const noexcept {
        return cornerVelocities_;
    }

    // ========================================================================
    // Dwell Points (G4 dwell commands)
    // ========================================================================

    /// Set dwell points on the path. Each dwell point forces velocity to 0
    /// at the given arc length (the planner will decelerate to a stop,
    /// pause for the dwell duration, then accelerate from 0).
    ///
    /// This method:
    /// 1. Stores the dwell points for the planner to insert dwell arcs.
    /// 2. Sets the corner velocity at the nearest segment boundary to 0
    ///    (or creates a corner velocity vector if not already set).
    ///
    /// @param dwellPoints Vector of (arcLength, duration) pairs, in
    ///                   arc-length order. Duration is in seconds.
    void setDwellPoints(const std::vector<std::pair<double, double>>& dwellPoints) {
        dwellPoints_.clear();
        dwellPoints_.reserve(dwellPoints.size());
        for (const auto& [arcLen, dur] : dwellPoints) {
            dwellPoints_.push_back({arcLen, dur});
        }

        // Ensure corner velocities are initialized (needed to force v=0
        // at dwell positions).
        if (cornerVelocities_.empty() && path_ && path_->numPieces() > 0) {
            cornerVelocities_.assign(path_->numPieces() + 1,
                                     std::numeric_limits<double>::infinity());
            hasPerSegmentLimits_ = true;
        }

        // Set corner velocity to 0 at the segment boundary nearest to
        // each dwell arc length. This forces the planner to stop.
        //
        // Corner velocity indices: cornerVelocities_[j] is at the boundary
        // between segment j-1 and segment j, i.e., at arc length =
        // segments_[j-1].cumulativeArcLength + segments_[j-1].arcLength.
        // We find the nearest boundary to the dwell arc length.
        const size_t N = segments_.size();
        if (N == 0 || cornerVelocities_.empty()) return;

        for (const auto& [arcLen, dur] : dwellPoints) {
            T s = static_cast<T>(arcLen);
            // Find the nearest segment boundary.
            // Boundaries are at: 0, end(seg0), end(seg1), ..., end(segN-1).
            // cornerVelocities_[j] corresponds to the j-th boundary.
            size_t bestJ = 0;
            double bestDist = std::abs(static_cast<double>(s));
            for (size_t j = 1; j <= N; ++j) {
                T boundary = segments_[j-1].cumulativeArcLength +
                             segments_[j-1].arcLength;
                double dist = std::abs(static_cast<double>(s - boundary));
                if (dist < bestDist) {
                    bestDist = dist;
                    bestJ = j;
                }
            }
            // Set the corner velocity at the nearest boundary to 0.
            if (bestJ < cornerVelocities_.size()) {
                cornerVelocities_[bestJ] = 0.0;
            }
        }
        updateSegmentVelocityFields();
    }

    /// Get the dwell points set on this path.
    /// @return Vector of (arcLength, duration) pairs. Empty if no dwells.
    const std::vector<std::pair<double, double>>& dwellPoints() const noexcept {
        return dwellPoints_;
    }

private:
    std::optional<tether::motion::PiecewiseNurbsPath> path_;
    std::vector<SourceReference> sourceRefs_;
    std::vector<SegmentInfo> segments_;
    std::vector<std::optional<tether::motion::PHData>> phData_;
    mutable std::shared_ptr<tether::motion::CertifiedCurvatureSampler>
        curvatureSampler_; ///< Lazy, memoized.

    /// Per-segment maximum velocity (from G-code F-values, mm/s).
    /// Empty = not set (use global feedRate only).
    std::vector<double> segmentMaxVelocities_;

    /// Per-junction corner velocity (size = numSegments + 1).
    /// Entry 0 = start of path, entry N = end of path.
    /// Empty = not set.
    std::vector<double> cornerVelocities_;

    /// Dwell points: (arcLength, duration) pairs from G4 commands.
    std::vector<std::pair<double, double>> dwellPoints_;

    /// True if per-segment velocity limits have been set.
    bool hasPerSegmentLimits_ = false;

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
        updateSegmentVelocityFields();
    }

    /// Sync the maxVelocity/entryCornerVelocity/exitCornerVelocity fields
    /// on each SegmentInfo from the internal vectors.
    void updateSegmentVelocityFields() {
        if (segments_.empty()) return;
        for (size_t i = 0; i < segments_.size(); ++i) {
            if (i < segmentMaxVelocities_.size()) {
                segments_[i].maxVelocity =
                    static_cast<T>(segmentMaxVelocities_[i]);
            }
            if (!cornerVelocities_.empty()) {
                if (i < cornerVelocities_.size()) {
                    segments_[i].entryCornerVelocity =
                        static_cast<T>(cornerVelocities_[i]);
                }
                if (i + 1 < cornerVelocities_.size()) {
                    segments_[i].exitCornerVelocity =
                        static_cast<T>(cornerVelocities_[i + 1]);
                }
            }
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
