/**
 * @file PiecewisePath.hpp
 * @brief Piecewise Bézier Path with Arc Length Parameterization
 *
 * @details
 * This file provides a piecewise path representation that stores multiple
 * Bézier segments with precomputed arc length tables for efficient evaluation.
 *
 * ## Features
 *
 * - Storage of multiple Bézier segments (elevated to common degree)
 * - Precomputed arc length tables for each segment
 * - Global arc length parameterization across all segments
 * - Bidirectional traversal support
 * - Efficient segment lookup with caching
 *
 * ## Usage
 *
 * ```cpp
 * PiecewiseBezierPath<3> path;
 * path.appendSegment(curve1);
 * path.appendSegment(curve2);
 * path.buildArcLengthTables();
 *
 * // Evaluate at arc length s
 * Vec3 pos = path.evaluateAtArcLength(50.0);
 *
 * // Get parameter from arc length
 * double u = path.globalParameterAtArcLength(50.0);
 * ```
 *
 * @see BezierCurve.hpp
 * @see MotionSegment.hpp
 */

#pragma once

#include "BezierCurve.hpp"
#include "SourceReference.hpp"
#include <vector>
#include <optional>
#include <algorithm>
#include <stdexcept>

namespace MotionPlanner {

// ============================================================================
// Segment Information
// ============================================================================

/**
 * @brief Information about a single segment in the piecewise path
 */
template<size_t Dim, typename T = double>
struct PathSegmentInfo {
    /// The Bézier curve for this segment
    BezierCurve<Dim, T> curve;
    
    /// Arc length of this segment
    T arcLength = T(0);
    
    /// Cumulative arc length at segment start
    T cumulativeArcLength = T(0);
    
    /// Parameter range [startParam, endParam] in global parameterization
    T startParam = T(0);
    T endParam = T(1);
    
    /// Source reference for traceability
    SourceReference sourceRef;
    
    /// Segment index in path
    size_t index = 0;
    
    /// Arc length table for this segment
    ArcLengthTable<T> arcLengthTable;
};

// ============================================================================
// Path Query Result
// ============================================================================

/**
 * @brief Result of evaluating the path at a point
 */
template<size_t Dim, typename T = double>
struct PathEvaluation {
    /// Position
    Vec<Dim, T> position;
    
    /// Velocity (first derivative w.r.t. arc length)
    Vec<Dim, T> velocity;
    
    /// Acceleration (second derivative w.r.t. arc length)
    Vec<Dim, T> acceleration;
    
    /// Jerk (third derivative w.r.t. arc length)
    Vec<Dim, T> jerk;
    
    /// Arc length from path start
    T arcLength = T(0);
    
    /// Global parameter value
    T globalParameter = T(0);
    
    /// Local parameter within segment [0, 1]
    T localParameter = T(0);
    
    /// Segment index
    size_t segmentIndex = 0;
    
    /// Curvature at this point
    T curvature = T(0);
    
    /// Unit tangent vector
    Vec<Dim, T> tangent;
    
    /// Unit normal vector
    Vec<Dim, T> normal;
    
    /// Source reference for this point
    SourceReference sourceRef;
};

// ============================================================================
// Piecewise Bézier Path
// ============================================================================

/**
 * @brief A continuous path composed of multiple Bézier segments
 *
 * @tparam Dim Number of spatial dimensions
 * @tparam T Scalar type (default: double)
 */
template<size_t Dim, typename T = double>
class PiecewiseBezierPath {
public:
    /// Point type
    using Point = Vec<Dim, T>;
    
    /// Segment info type
    using SegmentInfo = PathSegmentInfo<Dim, T>;
    
    /// Evaluation result type
    using Evaluation = PathEvaluation<Dim, T>;
    
    /// Curve type
    using Curve = BezierCurve<Dim, T>;

    // ========================================================================
    // Constructors
    // ========================================================================

    PiecewiseBezierPath() = default;

    /**
     * @brief Construct from a vector of curves
     */
    explicit PiecewiseBezierPath(std::vector<Curve> curves) {
        for (auto& curve : curves) {
            appendSegment(std::move(curve));
        }
    }

    // ========================================================================
    // Segment Management
    // ========================================================================

    /**
     * @brief Append a segment to the path
     *
     * @param curve Bézier curve to append
     * @param sourceRef Optional source reference
     * @return Index of the appended segment
     */
    size_t appendSegment(Curve curve, SourceReference sourceRef = {}) {
        SegmentInfo info;
        info.curve = std::move(curve);
        // If caller provided a SourceReference use it, otherwise fall back to
        // the curve's embedded sourceRef (preserved during creation)
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
     * @brief Insert a segment at a specific index
     */
    void insertSegment(size_t index, Curve curve, SourceReference sourceRef = {}) {
        if (index > segments_.size()) {
            throw std::out_of_range("Segment index out of range");
        }
        
        SegmentInfo info;
        info.curve = std::move(curve);
        info.sourceRef = std::move(sourceRef);
        info.index = index;
        
        segments_.insert(segments_.begin() + static_cast<ptrdiff_t>(index), std::move(info));
        
        // Update indices
        for (size_t i = index + 1; i < segments_.size(); ++i) {
            segments_[i].index = i;
        }
        
        invalidateCache();
    }

    /**
     * @brief Remove a segment
     */
    void removeSegment(size_t index) {
        if (index >= segments_.size()) {
            throw std::out_of_range("Segment index out of range");
        }
        
        segments_.erase(segments_.begin() + static_cast<ptrdiff_t>(index));
        
        // Update indices
        for (size_t i = index; i < segments_.size(); ++i) {
            segments_[i].index = i;
        }
        
        invalidateCache();
    }

    /**
     * @brief Replace a segment
     */
    void replaceSegment(size_t index, Curve curve, SourceReference sourceRef = {}) {
        if (index >= segments_.size()) {
            throw std::out_of_range("Segment index out of range");
        }
        
        segments_[index].curve = std::move(curve);
        if (sourceRef.isValid()) {
            segments_[index].sourceRef = std::move(sourceRef);
        }
        
        invalidateCache();
    }

    /**
     * @brief Get number of segments
     */
    size_t numSegments() const noexcept {
        return segments_.size();
    }

    /**
     * @brief Check if path is empty
     */
    bool empty() const noexcept {
        return segments_.empty();
    }

    /**
     * @brief Get segment by index
     */
    const SegmentInfo& segment(size_t index) const {
        return segments_.at(index);
    }

    /**
     * @brief Get all segments
     */
    const std::vector<SegmentInfo>& segments() const noexcept {
        return segments_;
    }

    // ========================================================================
    // Arc Length Operations
    // ========================================================================

    /**
     * @brief Build arc length tables for all segments
     *
     * Must be called after adding segments and before arc-length queries.
     */
    void buildArcLengthTables(const ArcLengthConfig& config = {}) {
        totalArcLength_ = T(0);
        
        for (auto& seg : segments_) {
            seg.arcLengthTable = seg.curve.buildArcLengthTable(config);
            seg.arcLength = seg.arcLengthTable.totalLength();
            seg.cumulativeArcLength = totalArcLength_;
            totalArcLength_ += seg.arcLength;
        }
        
        // Compute global parameter ranges
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

    /**
     * @brief Get total arc length of the path
     */
    T totalArcLength() const noexcept {
        return totalArcLength_;
    }

    /**
     * @brief Check if arc length tables are built
     */
    bool tablesBuilt() const noexcept {
        return tablesBuilt_;
    }

    // ========================================================================
    // Position Evaluation
    // ========================================================================

    /**
     * @brief Full evaluation at arc length (compatibility)
     *
     * Historically this returned just a `Point`, but callers expect a
     * rich evaluation result (position, tangent, curvature, etc.). For
     * compatibility we return the full `Evaluation` structure here.
     */
    Evaluation evaluateAtArcLength(T s) const {
        return evaluateFullAtArcLength(s);
    }

    /**
     * @brief Convenience: get curvature at arc length
     */
    T curvatureAtArcLength(T s) const {
        return evaluateFullAtArcLength(s).curvature;
    }

    /**
     * @brief Backwards-compatible alias for appending a segment
     */
    void addSegment(Curve curve) {
        appendSegment(std::move(curve));
    }

    /**
     * @brief Backwards-compatible total length accessor
     */
    T totalLength() const noexcept {
        return totalArcLength();
    }

    /**
     * @brief Backwards-compatible accessors for code using getSegment()
     */
    const SegmentInfo& getSegment(size_t index) const { return segment(index); }
    SegmentInfo& getSegment(size_t index) { return segments_.at(index); }

    /**
     * @brief Legacy-style forward iterator helpers used by older tests
     */
    class LegacyForwardIterator {
    public:
        LegacyForwardIterator(const PiecewiseBezierPath* path, size_t idx)
            : path_(path), idx_(idx) {}

        bool operator!=(const LegacyForwardIterator& other) const {
            return idx_ != other.idx_ || path_ != other.path_;
        }

        LegacyForwardIterator& operator++() {
            ++idx_;
            return *this;
        }

    private:
        const PiecewiseBezierPath* path_;
        size_t idx_;
    };

    LegacyForwardIterator forwardBegin() const { return LegacyForwardIterator(this, 0); }
    LegacyForwardIterator forwardEnd() const { return LegacyForwardIterator(this, numSegments()); }

    /**
     * @brief Get source reference at an arc length position
     */
    SourceReference sourceRefAtArcLength(T s) const {
        return evaluateFullAtArcLength(s).sourceRef;
    }

    /**
     * @brief Evaluate position at global parameter u ∈ [0, 1]
     */
    Point evaluateAtParameter(T u) const {
        auto [segIndex, localParam] = findSegmentAtParameter(u);
        return segments_[segIndex].curve.evaluate(localParam);
    }

    /**
     * @brief Full evaluation at arc length (position + derivatives)
     */
    Evaluation evaluateFullAtArcLength(T s) const {
        Evaluation result;
        result.arcLength = s;
        
        auto [segIndex, localParam] = findSegmentAtArcLength(s);
        const auto& seg = segments_[segIndex];
        
        result.segmentIndex = segIndex;
        result.localParameter = localParam;
        result.sourceRef = seg.sourceRef;
        
        // Get position and derivatives w.r.t. curve parameter
        auto [pos, vel_u, acc_u, jrk_u] = seg.curve.evaluateAll(localParam);
        
        result.position = pos;
        
        // Convert derivatives from parameter space to arc length space
        T speed = vel_u.magnitude();
        if (speed > static_cast<T>(MathConstants::EPSILON)) {
            // ds/du = speed
            result.tangent = vel_u / speed;
            result.velocity = result.tangent;  // d/ds gives unit tangent
            
            // Second derivative: need to account for ds/du
            T speedSq = speed * speed;
            Point accPerp = acc_u - vel_u * (acc_u.dot(vel_u) / speedSq);
            result.acceleration = accPerp / speedSq;
            
            // Curvature
            result.curvature = seg.curve.curvature(localParam);
            
            // Normal (perpendicular to tangent, in direction of curvature)
            result.normal = seg.curve.normal(localParam);
            
            // Jerk (simplified)
            result.jerk = jrk_u / (speedSq * speed);
        }
        
        // Global parameter
        if (totalArcLength_ > T(0)) {
            result.globalParameter = s / totalArcLength_;
        }
        
        return result;
    }

    /**
     * @brief Full evaluation at global parameter
     */
    Evaluation evaluateFullAtParameter(T u) const {
        T s = arcLengthAtParameter(u);
        return evaluateFullAtArcLength(s);
    }

    // ========================================================================
    // Parameter/Arc Length Conversion
    // ========================================================================

    /**
     * @brief Get arc length at global parameter u
     */
    T arcLengthAtParameter(T u) const {
        if (segments_.empty()) return T(0);
        
        u = clamp(u, T(0), T(1));
        
        auto [segIndex, localParam] = findSegmentAtParameter(u);
        const auto& seg = segments_[segIndex];
        
        T localArcLen = seg.arcLengthTable.arcLengthAt(localParam);
        return seg.cumulativeArcLength + localArcLen;
    }

    /**
     * @brief Get global parameter at arc length s
     */
    T parameterAtArcLength(T s) const {
        if (segments_.empty() || totalArcLength_ <= T(0)) return T(0);
        return clamp(s / totalArcLength_, T(0), T(1));
    }

    // ========================================================================
    // Segment Lookup
    // ========================================================================

    /**
     * @brief Find segment containing arc length s
     *
     * @return Pair of (segment index, local parameter within segment)
     */
    std::pair<size_t, T> findSegmentAtArcLength(T s) const {
        if (segments_.empty()) {
            return {0, T(0)};
        }
        
        s = clamp(s, T(0), totalArcLength_);
        
        // Check cached segment first
        if (cachedSegmentIndex_ < segments_.size()) {
            const auto& seg = segments_[cachedSegmentIndex_];
            if (s >= seg.cumulativeArcLength && 
                s < seg.cumulativeArcLength + seg.arcLength) {
                T localS = s - seg.cumulativeArcLength;
                T localU = seg.arcLengthTable.parameterAt(localS);
                return {cachedSegmentIndex_, localU};
            }
        }
        
        // Binary search for segment
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
        
        return {left, localU};
    }

    /**
     * @brief Find segment containing global parameter u
     *
     * @return Pair of (segment index, local parameter within segment)
     */
    std::pair<size_t, T> findSegmentAtParameter(T u) const {
        if (segments_.empty()) {
            return {0, T(0)};
        }
        
        u = clamp(u, T(0), T(1));
        
        // Binary search for segment
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
    // Continuity Checking
    // ========================================================================

    /**
     * @brief Check G0 continuity at all segment junctions
     */
    bool isG0Continuous(T tolerance = static_cast<T>(MathConstants::EPSILON)) const {
        for (size_t i = 1; i < segments_.size(); ++i) {
            if (!segments_[i-1].curve.isG0ContinuousWith(segments_[i].curve, tolerance)) {
                return false;
            }
        }
        return true;
    }

    /**
     * @brief Check G1 continuity at all segment junctions
     */
    bool isG1Continuous(T tolerance = static_cast<T>(MathConstants::EPSILON)) const {
        for (size_t i = 1; i < segments_.size(); ++i) {
            if (!segments_[i-1].curve.isG1ContinuousWith(segments_[i].curve, tolerance)) {
                return false;
            }
        }
        return true;
    }

    /**
     * @brief Check G2 continuity at all segment junctions
     */
    bool isG2Continuous(T tolerance = static_cast<T>(MathConstants::EPSILON)) const {
        for (size_t i = 1; i < segments_.size(); ++i) {
            if (!segments_[i-1].curve.isG2ContinuousWith(segments_[i].curve, tolerance)) {
                return false;
            }
        }
        return true;
    }

    /**
     * @brief Get minimum continuity level across all junctions
     */
    int minContinuityLevel(T tolerance = static_cast<T>(MathConstants::EPSILON)) const {
        if (segments_.size() <= 1) return 2;  // Single segment is C∞
        
        int minLevel = 2;
        for (size_t i = 1; i < segments_.size(); ++i) {
            int level = segments_[i-1].curve.continuityLevelWith(
                segments_[i].curve, tolerance);
            minLevel = std::min(minLevel, level);
        }
        return minLevel;
    }

    // ========================================================================
    // Bidirectional Traversal
    // ========================================================================

    /**
     * @brief Iterator for forward traversal
     */
    class ForwardIterator {
    public:
        ForwardIterator(const PiecewiseBezierPath* path, T arcLength)
            : path_(path), currentArcLength_(arcLength) {}

        /**
         * @brief Advance by arc length
         */
        void advance(T distance) {
            currentArcLength_ = clamp(currentArcLength_ + distance, 
                                     T(0), path_->totalArcLength());
        }

        /**
         * @brief Get current position
         */
        Point position() const {
            return path_->evaluateAtArcLength(currentArcLength_).position;
        }

        /**
         * @brief Get current arc length
         */
        T arcLength() const { return currentArcLength_; }

        /**
         * @brief Check if at end
         */
        bool atEnd() const {
            return currentArcLength_ >= path_->totalArcLength();
        }

    private:
        const PiecewiseBezierPath* path_;
        T currentArcLength_;
    };

    /**
     * @brief Iterator for backward traversal
     */
    class BackwardIterator {
    public:
        BackwardIterator(const PiecewiseBezierPath* path, T arcLength)
            : path_(path), currentArcLength_(arcLength) {}

        /**
         * @brief Retreat by arc length
         */
        void retreat(T distance) {
            currentArcLength_ = clamp(currentArcLength_ - distance,
                                     T(0), path_->totalArcLength());
        }

        /**
         * @brief Get current position
         */
        Point position() const {
            return path_->evaluateAtArcLength(currentArcLength_).position;
        }

        /**
         * @brief Get current arc length
         */
        T arcLength() const { return currentArcLength_; }

        /**
         * @brief Check if at start
         */
        bool atStart() const {
            return currentArcLength_ <= T(0);
        }

    private:
        const PiecewiseBezierPath* path_;
        T currentArcLength_;
    };

    /**
     * @brief Get forward iterator starting at arc length s
     */
    ForwardIterator forwardIterator(T startArcLength = T(0)) const {
        return ForwardIterator(this, startArcLength);
    }

    /**
     * @brief Get backward iterator starting at arc length s
     */
    BackwardIterator backwardIterator(T startArcLength) const {
        return BackwardIterator(this, startArcLength);
    }

    /**
     * @brief Get backward iterator starting at end
     */
    BackwardIterator backwardIteratorFromEnd() const {
        return BackwardIterator(this, totalArcLength_);
    }

    // ========================================================================
    // Path Properties
    // ========================================================================

    /**
     * @brief Get start point of path
     */
    Point startPoint() const {
        if (segments_.empty()) return Point{};
        return segments_.front().curve.startPoint();
    }

    /**
     * @brief Get end point of path
     */
    Point endPoint() const {
        if (segments_.empty()) return Point{};
        return segments_.back().curve.endPoint();
    }

    /**
     * @brief Compute bounding box of entire path
     */
    std::pair<Point, Point> boundingBox() const {
        if (segments_.empty()) {
            return {Point{}, Point{}};
        }
        
        auto [minPt, maxPt] = segments_[0].curve.boundingBox();
        
        for (size_t i = 1; i < segments_.size(); ++i) {
            auto [segMin, segMax] = segments_[i].curve.boundingBox();
            minPt = minPt.elementMin(segMin);
            maxPt = maxPt.elementMax(segMax);
        }
        
        return {minPt, maxPt};
    }

    // ========================================================================
    // Degree Elevation
    // ========================================================================

    /**
     * @brief Elevate all segments to a common degree
     */
    void elevateToCommonDegree(size_t targetDegree) {
        for (auto& seg : segments_) {
            if (seg.curve.degree() < targetDegree) {
                seg.curve = seg.curve.elevate(targetDegree);
            }
        }
        invalidateCache();
    }

    /**
     * @brief Get maximum degree among all segments
     */
    size_t maxDegree() const {
        size_t maxDeg = 0;
        for (const auto& seg : segments_) {
            maxDeg = std::max(maxDeg, seg.curve.degree());
        }
        return maxDeg;
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

using PiecewiseBezierPath2D = PiecewiseBezierPath<2, double>;
using PiecewiseBezierPath3D = PiecewiseBezierPath<3, double>;

}  // namespace MotionPlanner
