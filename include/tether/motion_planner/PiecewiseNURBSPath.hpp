/**
 * @file PiecewiseNURBSPath.hpp
 * @brief Piecewise NURBS Path with Arc Length Parameterization
 *
 * @details
 * Replaces the Bézier-only PiecewiseBezierPath with a NURBS-native path
 * that supports arbitrary NURBS segments. For SVG export compatibility,
 * provides Bézier decomposition of all segments.
 *
 * This path representation stores NURBS segments natively and evaluates
 * using the de Boor algorithm. When Bézier curves are needed (e.g. for
 * SVG export), the decomposeToBezier() method is used.
 *
 * @see NURBSCurve.hpp
 * @see PiecewisePath.hpp (legacy Bézier-only path)
 */

#pragma once

#include "NURBSCurve.hpp"
#include "BezierCurve.hpp"
#include "PiecewisePath.hpp"
#include "SourceReference.hpp"
#include <vector>
#include <algorithm>
#include <optional>

namespace MotionPlanner {

// ============================================================================
// NURBS Segment Information
// ============================================================================

template<size_t Dim, typename T = double>
struct NURBSPathSegmentInfo {
    NURBSCurve<Dim, T> curve;
    T arcLength = T(0);
    T cumulativeArcLength = T(0);
    T startParam = T(0);
    T endParam = T(1);
    SourceReference sourceRef;
    size_t index = 0;
    NURBSArcLengthTable<T> arcLengthTable;
};

// ============================================================================
// NURBS Path Evaluation Result
// ============================================================================

template<size_t Dim, typename T = double>
struct NURBSPathEvaluation {
    Vec<Dim, T> position;
    Vec<Dim, T> velocity;
    Vec<Dim, T> acceleration;
    Vec<Dim, T> jerk;
    T arcLength = T(0);
    T globalParameter = T(0);
    T localParameter = T(0);
    size_t segmentIndex = 0;
    T curvature = T(0);
    Vec<Dim, T> tangent;
    Vec<Dim, T> normal;
    SourceReference sourceRef;
};

// ============================================================================
// Piecewise NURBS Path
// ============================================================================

template<size_t Dim, typename T = double>
class PiecewiseNURBSPath {
public:
    using Point = Vec<Dim, T>;
    using SegmentInfo = NURBSPathSegmentInfo<Dim, T>;
    using Evaluation = NURBSPathEvaluation<Dim, T>;
    using Curve = NURBSCurve<Dim, T>;
    using BezCurve = BezierCurve<Dim, T>;

    PiecewiseNURBSPath() = default;

    // ========================================================================
    // Segment Management
    // ========================================================================

    size_t appendSegment(Curve curve, SourceReference sourceRef = {}) {
        SegmentInfo info;
        info.curve = std::move(curve);
        if (sourceRef.isValid()) {
            info.sourceRef = std::move(sourceRef);
        } else {
            info.sourceRef = info.curve.sourceRef();
        }
        info.index = segments_.size();
        segments_.push_back(std::move(info));
        invalidateCache();
        return segments_.size() - 1;
    }

    /**
     * @brief Append a Bézier curve (converts to NURBS internally)
     */
    size_t appendBezierSegment(const BezCurve& bezier, SourceReference sourceRef = {}) {
        Curve nurbs(bezier);
        if (sourceRef.isValid()) {
            nurbs.setSourceRef(sourceRef);
        }
        return appendSegment(std::move(nurbs), std::move(sourceRef));
    }

    /**
     * @brief Legacy compatibility: addSegment
     */
    void addSegment(Curve curve) {
        appendSegment(std::move(curve));
    }

    void addSegment(const BezCurve& bezier) {
        appendBezierSegment(bezier);
    }

    size_t numSegments() const noexcept { return segments_.size(); }
    bool empty() const noexcept { return segments_.empty(); }

    const SegmentInfo& segment(size_t index) const { return segments_.at(index); }
    const SegmentInfo& getSegment(size_t index) const { return segments_.at(index); }
    SegmentInfo& getSegment(size_t index) { return segments_.at(index); }

    const std::vector<SegmentInfo>& segments() const noexcept { return segments_; }

    // ========================================================================
    // Arc Length Operations
    // ========================================================================

    void buildArcLengthTables(const NURBSArcLengthConfig& config = {}) {
        totalArcLength_ = T(0);

        for (auto& seg : segments_) {
            seg.arcLengthTable = seg.curve.buildArcLengthTable(config);
            seg.arcLength = seg.arcLengthTable.totalLength();
            seg.cumulativeArcLength = totalArcLength_;
            totalArcLength_ += seg.arcLength;
        }

        if (totalArcLength_ > T(0)) {
            T cumParam = T(0);
            for (auto& seg : segments_) {
                seg.startParam = cumParam;
                cumParam += seg.arcLength / totalArcLength_;
                seg.endParam = cumParam;
            }
        }

        tablesBuilt_ = true;
    }

    T totalArcLength() const noexcept { return totalArcLength_; }
    T totalLength() const noexcept { return totalArcLength_; }
    bool tablesBuilt() const noexcept { return tablesBuilt_; }

    // ========================================================================
    // Evaluation
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

        auto [segIndex, localParam] = findSegmentAtArcLength(s);
        const auto& seg = segments_[segIndex];

        result.segmentIndex = segIndex;
        result.localParameter = localParam;
        result.sourceRef = seg.sourceRef;

        // Map local parameter from [0,1] to curve domain [u_start, u_end]
        T uStart = seg.curve.domainStart();
        T uEnd = seg.curve.domainEnd();
        T u = uStart + localParam * (uEnd - uStart);

        auto [pos, vel_u, acc_u, jrk_u] = seg.curve.evaluateAll(u);
        result.position = pos;

        T speed = vel_u.magnitude();
        if (speed > static_cast<T>(MathConstants::EPSILON)) {
            result.tangent = vel_u / speed;
            result.velocity = result.tangent;

            T speedSq = speed * speed;
            Point accPerp = acc_u - vel_u * (acc_u.dot(vel_u) / speedSq);
            result.acceleration = accPerp / speedSq;

            result.curvature = seg.curve.curvature(u);
            result.normal = seg.curve.normal(u);
            result.jerk = jrk_u / (speedSq * speed);
        }

        if (totalArcLength_ > T(0)) {
            result.globalParameter = s / totalArcLength_;
        }

        return result;
    }

    Point evaluateAtParameter(T u) const {
        auto [segIndex, localParam] = findSegmentAtParameter(u);
        const auto& seg = segments_[segIndex];
        T uStart = seg.curve.domainStart();
        T uEnd = seg.curve.domainEnd();
        T curveU = uStart + localParam * (uEnd - uStart);
        return seg.curve.evaluate(curveU);
    }

    SourceReference sourceRefAtArcLength(T s) const {
        return evaluateFullAtArcLength(s).sourceRef;
    }

    /**
     * @brief Find arc length position for a given G-code line number
     * @return Arc length at the start of the matching segment, or nullopt
     */
    std::optional<T> findArcLengthForLine(size_t lineNumber) const {
        for (const auto& seg : segments_) {
            if (seg.sourceRef.type() == SourceReference::Type::Single &&
                seg.sourceRef.lineNumber() == lineNumber) {
                return seg.cumulativeArcLength;
            }
        }
        return std::nullopt;
    }

    // ========================================================================
    // Parameter/Arc Length Conversion
    // ========================================================================

    T arcLengthAtParameter(T u) const {
        if (segments_.empty()) return T(0);
        u = clamp(u, T(0), T(1));
        auto [segIndex, localParam] = findSegmentAtParameter(u);
        const auto& seg = segments_[segIndex];
        T localArcLen = seg.arcLengthTable.arcLengthAt(
            seg.curve.domainStart() + localParam * (seg.curve.domainEnd() - seg.curve.domainStart()));
        return seg.cumulativeArcLength + localArcLen;
    }

    T parameterAtArcLength(T s) const {
        if (segments_.empty() || totalArcLength_ <= T(0)) return T(0);
        return clamp(s / totalArcLength_, T(0), T(1));
    }

    // ========================================================================
    // Segment Lookup
    // ========================================================================

    std::pair<size_t, T> findSegmentAtArcLength(T s) const {
        if (segments_.empty()) return {0, T(0)};
        s = clamp(s, T(0), totalArcLength_);

        if (cachedSegmentIndex_ < segments_.size()) {
            const auto& seg = segments_[cachedSegmentIndex_];
            if (s >= seg.cumulativeArcLength &&
                s < seg.cumulativeArcLength + seg.arcLength) {
                T localS = s - seg.cumulativeArcLength;
                T localU = seg.arcLengthTable.parameterAt(localS);
                // Convert from curve domain to [0,1]
                T uStart = seg.curve.domainStart();
                T uEnd = seg.curve.domainEnd();
                T normalized = (uEnd > uStart) ? (localU - uStart) / (uEnd - uStart) : T(0);
                return {cachedSegmentIndex_, normalized};
            }
        }

        size_t left = 0;
        size_t right = segments_.size() - 1;
        while (left < right) {
            size_t mid = left + (right - left + 1) / 2;
            if (segments_[mid].cumulativeArcLength <= s) {
                left = mid;
            } else {
                right = mid - 1;
            }
        }

        cachedSegmentIndex_ = left;
        const auto& seg = segments_[left];
        T localS = s - seg.cumulativeArcLength;
        T localU = seg.arcLengthTable.parameterAt(localS);
        T uStart = seg.curve.domainStart();
        T uEnd = seg.curve.domainEnd();
        T normalized = (uEnd > uStart) ? (localU - uStart) / (uEnd - uStart) : T(0);

        return {left, normalized};
    }

    std::pair<size_t, T> findSegmentAtParameter(T u) const {
        if (segments_.empty()) return {0, T(0)};
        u = clamp(u, T(0), T(1));

        size_t left = 0;
        size_t right = segments_.size() - 1;
        while (left < right) {
            size_t mid = left + (right - left + 1) / 2;
            if (segments_[mid].startParam <= u) {
                left = mid;
            } else {
                right = mid - 1;
            }
        }

        const auto& seg = segments_[left];
        T paramRange = seg.endParam - seg.startParam;
        T localU = (paramRange > T(0)) ? (u - seg.startParam) / paramRange : T(0);
        return {left, clamp(localU, T(0), T(1))};
    }

    // ========================================================================
    // Path Properties
    // ========================================================================

    Point startPoint() const {
        if (segments_.empty()) return Point{};
        return segments_.front().curve.startPoint();
    }

    Point endPoint() const {
        if (segments_.empty()) return Point{};
        return segments_.back().curve.endPoint();
    }

    std::pair<Point, Point> boundingBox() const {
        if (segments_.empty()) return {Point{}, Point{}};
        auto [minPt, maxPt] = segments_[0].curve.boundingBox();
        for (size_t i = 1; i < segments_.size(); ++i) {
            auto [segMin, segMax] = segments_[i].curve.boundingBox();
            minPt = minPt.elementMin(segMin);
            maxPt = maxPt.elementMax(segMax);
        }
        return {minPt, maxPt};
    }

    // ========================================================================
    // Continuity Checking
    // ========================================================================

    bool isG0Continuous(T tolerance = static_cast<T>(MathConstants::EPSILON)) const {
        for (size_t i = 1; i < segments_.size(); ++i) {
            if (!segments_[i-1].curve.isG0ContinuousWith(segments_[i].curve, tolerance))
                return false;
        }
        return true;
    }

    bool isG1Continuous(T tolerance = static_cast<T>(MathConstants::EPSILON)) const {
        for (size_t i = 1; i < segments_.size(); ++i) {
            if (!segments_[i-1].curve.isG1ContinuousWith(segments_[i].curve, tolerance))
                return false;
        }
        return true;
    }

    bool isG2Continuous(T tolerance = static_cast<T>(MathConstants::EPSILON)) const {
        for (size_t i = 1; i < segments_.size(); ++i) {
            if (!segments_[i-1].curve.isG2ContinuousWith(segments_[i].curve, tolerance))
                return false;
        }
        return true;
    }

    int minContinuityLevel(T tolerance = static_cast<T>(MathConstants::EPSILON)) const {
        if (segments_.size() <= 1) return 2;
        int minLevel = 2;
        for (size_t i = 1; i < segments_.size(); ++i) {
            int level = segments_[i-1].curve.continuityLevelWith(
                segments_[i].curve, tolerance);
            minLevel = std::min(minLevel, level);
        }
        return minLevel;
    }

    // ========================================================================
    // Bézier Decomposition (for SVG export)
    // ========================================================================

    /**
     * @brief Decompose entire path into Bézier curves for SVG export
     *
     * Each NURBS segment is decomposed into its constituent Bézier curves.
     * This allows exact SVG path rendering using cubic/quadratic Bézier commands.
     */
    std::vector<BezierCurve<Dim, T>> decomposeToBezier() const {
        std::vector<BezierCurve<Dim, T>> result;
        for (const auto& seg : segments_) {
            auto beziers = seg.curve.decomposeToBezier();
            result.insert(result.end(), beziers.begin(), beziers.end());
        }
        return result;
    }

    /**
     * @brief Approximate with cubic Bézier curves (for SVG cubic path commands)
     */
    std::vector<BezierCurve<Dim, T>> approximateWithCubicBeziers(
            T maxError = T(0.01)) const {
        std::vector<BezierCurve<Dim, T>> result;
        for (const auto& seg : segments_) {
            auto cubics = seg.curve.approximateWithCubicBeziers(maxError);
            result.insert(result.end(), cubics.begin(), cubics.end());
        }
        return result;
    }

    /**
     * @brief Convert to legacy PiecewiseBezierPath for compatibility
     */
    PiecewiseBezierPath<Dim, T> toPiecewiseBezierPath(T maxError = T(0.01)) const {
        PiecewiseBezierPath<Dim, T> bezPath;
        for (const auto& seg : segments_) {
            auto cubics = seg.curve.approximateWithCubicBeziers(maxError);
            for (auto& c : cubics) {
                bezPath.appendSegment(std::move(c), seg.sourceRef);
            }
        }
        bezPath.buildArcLengthTables();
        return bezPath;
    }

    // ========================================================================
    // Iterators (legacy compatibility)
    // ========================================================================

    class LegacyForwardIterator {
    public:
        LegacyForwardIterator(const PiecewiseNURBSPath* path, size_t idx)
            : path_(path), idx_(idx) {}
        bool operator!=(const LegacyForwardIterator& other) const {
            return idx_ != other.idx_ || path_ != other.path_;
        }
        LegacyForwardIterator& operator++() { ++idx_; return *this; }
    private:
        const PiecewiseNURBSPath* path_;
        size_t idx_;
    };

    LegacyForwardIterator forwardBegin() const { return {this, 0}; }
    LegacyForwardIterator forwardEnd() const { return {this, numSegments()}; }

    // Forward iterator
    class ForwardIterator {
    public:
        ForwardIterator(const PiecewiseNURBSPath* path, T arcLength)
            : path_(path), currentArcLength_(arcLength) {}
        void advance(T distance) {
            currentArcLength_ = clamp(currentArcLength_ + distance,
                                     T(0), path_->totalArcLength());
        }
        Point position() const {
            return path_->evaluateAtArcLength(currentArcLength_).position;
        }
        T arcLength() const { return currentArcLength_; }
        bool atEnd() const { return currentArcLength_ >= path_->totalArcLength(); }
    private:
        const PiecewiseNURBSPath* path_;
        T currentArcLength_;
    };

    ForwardIterator forwardIterator(T startArcLength = T(0)) const {
        return ForwardIterator(this, startArcLength);
    }

private:
    std::vector<SegmentInfo> segments_;
    T totalArcLength_ = T(0);
    bool tablesBuilt_ = false;
    mutable size_t cachedSegmentIndex_ = 0;

    void invalidateCache() {
        tablesBuilt_ = false;
        totalArcLength_ = T(0);
        cachedSegmentIndex_ = 0;
    }
};

// ============================================================================
// Type Aliases
// ============================================================================

using PiecewiseNURBSPath2D = PiecewiseNURBSPath<2, double>;
using PiecewiseNURBSPath3D = PiecewiseNURBSPath<3, double>;

} // namespace MotionPlanner
