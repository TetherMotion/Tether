/**
 * @file MotionSegment.hpp
 * @brief Motion Segment Data Structures with Lookahead/Lookbehind
 *
 * @details
 * This file defines data structures that represent parsed G-code motion
 * commands with full traceability back to source G-code lines.
 *
 * ## Key Features
 *
 * - Segment types: linear, arc, spline, dwell
 * - Doubly-linked structure for bidirectional traversal
 * - Efficient lookahead and lookbehind operations
 * - Full source reference preservation
 *
 * ## Usage
 *
 * ```cpp
 * MotionSegmentList segments;
 * segments.append(MotionSegment::linear(start, end, feedrate, sourceRef));
 * segments.append(MotionSegment::arc(start, end, center, feedrate, sourceRef));
 *
 * // Lookahead
 * auto lookahead = segments.getLookahead(currentIndex, 10);
 *
 * // Lookbehind
 * auto lookbehind = segments.getLookbehind(currentIndex, 5);
 * ```
 *
 * @see SourceReference.hpp
 * @see PathAdapter.hpp
 */

#pragma once

#include "MathTypes.hpp"
#include "SourceReference.hpp"
#include <vector>
#include <deque>
#include <memory>
#include <optional>
#include <functional>
#include <stdexcept>
#include <algorithm>

namespace MotionPlanner {

// ============================================================================
// Motion Segment Type
// ============================================================================

/**
 * @brief Type of motion segment
 */
enum class MotionSegmentType : uint8_t {
    Linear,         ///< G0/G1 - Linear motion
    ArcCW,          ///< G2 - Clockwise arc
    ArcCCW,         ///< G3 - Counter-clockwise arc
    CubicSpline,    ///< G5 - Cubic spline
    QuadSpline,     ///< G5.1 - Quadratic spline
    NURBS,          ///< G5.2/G5.3 - NURBS curve
    Dwell,          ///< G4 - Dwell (pause)
    Rapid           ///< G0 - Rapid positioning
};

/**
 * @brief Plane for arc interpolation
 */
enum class ArcPlane : uint8_t {
    XY,     ///< G17 - XY plane
    XZ,     ///< G18 - XZ plane
    YZ      ///< G19 - YZ plane
};

/**
 * @brief Path control mode
 */
enum class PathMode : uint8_t {
    ExactStop,      ///< G61 - Full stop at endpoint
    ExactPath,      ///< G61.1 - Follow exact path
    Blending        ///< G64 - Path blending/corner rounding
};

// ============================================================================
// Kinematic Limits
// ============================================================================

/**
 * @brief Kinematic limits for a segment or axis
 */
struct SegmentKinematicLimits {
    double maxVelocity = std::numeric_limits<double>::infinity();
    double maxAcceleration = std::numeric_limits<double>::infinity();
    double maxDeceleration = std::numeric_limits<double>::infinity();
    double maxJerk = std::numeric_limits<double>::infinity();
    
    /**
     * @brief Check if any limit is finite
     */
    bool hasFiniteLimits() const {
        return std::isfinite(maxVelocity) || std::isfinite(maxAcceleration) ||
               std::isfinite(maxDeceleration) || std::isfinite(maxJerk);
    }
    
    /**
     * @brief Merge with other limits (take minimum)
     */
    SegmentKinematicLimits merge(const SegmentKinematicLimits& other) const {
        SegmentKinematicLimits result;
        result.maxVelocity = std::min(maxVelocity, other.maxVelocity);
        result.maxAcceleration = std::min(maxAcceleration, other.maxAcceleration);
        result.maxDeceleration = std::min(maxDeceleration, other.maxDeceleration);
        result.maxJerk = std::min(maxJerk, other.maxJerk);
        return result;
    }
};

// ============================================================================
// Blending Configuration
// ============================================================================

/**
 * @brief Configuration for corner blending (G64)
 */
struct BlendingConfig {
    /// Path blending tolerance (G64 P value)
    double tolerance = 0.0;
    
    /// Naive CAM tolerance (G64 Q value) - simplified blending
    double naiveCamTolerance = 0.0;
    
    /// Maximum fraction of segment that can be consumed by blend [0, 0.5]
    double maxBlendFraction = 0.3;
    
    /// Minimum segment length for blending
    double minSegmentLength = 0.01;
    
    /// Continuity level to achieve (1 = G1, 2 = G2)
    int continuityLevel = 2;
};

// ============================================================================
// Motion Segment
// ============================================================================

/// Maximum number of axes
constexpr size_t MAX_MOTION_AXES = 9;

/**
 * @brief Represents a single motion segment from G-code
 *
 * This is the parsed representation of a G-code motion command, containing
 * all information needed for path construction and velocity planning.
 */
struct MotionSegment {
    // ========================================================================
    // Segment Identity
    // ========================================================================
    
    /// Unique segment ID within the program
    uint64_t id = 0;
    
    /// Motion type
    MotionSegmentType type = MotionSegmentType::Linear;
    
    /// Source reference for traceability
    SourceReference sourceRef;
    
    // ========================================================================
    // Geometry
    // ========================================================================
    
    /// Start position (all axes)
    std::array<double, MAX_MOTION_AXES> startPosition{};
    
    /// End position (all axes)
    std::array<double, MAX_MOTION_AXES> endPosition{};
    
    /// Arc center (for G2/G3)
    std::array<double, MAX_MOTION_AXES> arcCenter{};
    
    /// Arc radius (for G2/G3)
    double arcRadius = 0.0;
    
    /// Arc sweep angle in radians (positive for CCW)
    double arcSweep = 0.0;
    
    /// Arc plane (for G2/G3)
    ArcPlane arcPlane = ArcPlane::XY;
    
    /// Spline control points (for G5, NURBS)
    std::vector<std::array<double, MAX_MOTION_AXES>> controlPoints;
    
    /// NURBS weights
    std::vector<double> weights;
    
    /// NURBS knot vector
    std::vector<double> knots;
    
    /// NURBS/spline degree
    size_t degree = 0;
    
    // ========================================================================
    // Motion Parameters
    // ========================================================================
    
    /// Feedrate (mm/min for linear, deg/min for rotary)
    double feedrate = 0.0;
    /// Backwards-compatible alias used in older code/tests
    double& feedRate = feedrate;
    
    /// Dwell time (seconds, for G4)
    double dwellTime = 0.0;
    /// Backwards-compatible alias name expected by tests
    double& dwellDuration = dwellTime;
    
    /// Path mode
    PathMode pathMode = PathMode::Blending;
    
    /// Blending configuration (for G64 mode)
    BlendingConfig blending;
    
    /// Per-segment kinematic limits (optional override)
    std::optional<SegmentKinematicLimits> kinematicLimits;
    
    // ========================================================================
    // Computed Properties
    // ========================================================================
    
    /// Computed segment length (set during preprocessing)
    double segmentLength = 0.0;
    
    /// Axes that are active (moving) in this segment
    std::array<bool, MAX_MOTION_AXES> activeAxes{};
    
    /// Number of active axes
    size_t numActiveAxes = 0;
    
    // ========================================================================
    // Special member functions
    // ------------------------------------------------------------------------
    // Explicit copy/move assignment operators are provided because this struct
    // contains reference members (compatibility aliases) which prevent the
    // compiler from generating the default assignment operators.
    // ------------------------------------------------------------------------
    MotionSegment& operator=(const MotionSegment& other) {
        if (this == &other) return *this;
        id = other.id;
        type = other.type;
        sourceRef = other.sourceRef;
        startPosition = other.startPosition;
        endPosition = other.endPosition;
        arcCenter = other.arcCenter;
        arcRadius = other.arcRadius;
        arcSweep = other.arcSweep;
        arcPlane = other.arcPlane;
        controlPoints = other.controlPoints;
        weights = other.weights;
        knots = other.knots;
        degree = other.degree;
        feedrate = other.feedrate;
        dwellTime = other.dwellTime;
        pathMode = other.pathMode;
        blending = other.blending;
        kinematicLimits = other.kinematicLimits;
        segmentLength = other.segmentLength;
        activeAxes = other.activeAxes;
        numActiveAxes = other.numActiveAxes;
        return *this;
    }

    // Default constructor
    MotionSegment() = default;

    // Re-enable copy/move ctors explicitly so factory functions can return by value
    MotionSegment(const MotionSegment& other)
        : id(other.id), type(other.type), sourceRef(other.sourceRef),
          startPosition(other.startPosition), endPosition(other.endPosition),
          arcCenter(other.arcCenter), arcRadius(other.arcRadius), arcSweep(other.arcSweep),
          arcPlane(other.arcPlane), controlPoints(other.controlPoints), weights(other.weights),
          knots(other.knots), degree(other.degree), feedrate(other.feedrate), dwellTime(other.dwellTime),
          dwellDuration(dwellTime), pathMode(other.pathMode), blending(other.blending),
          kinematicLimits(other.kinematicLimits), segmentLength(other.segmentLength),
          activeAxes(other.activeAxes), numActiveAxes(other.numActiveAxes) {}

    MotionSegment(MotionSegment&& other) noexcept
        : id(other.id), type(other.type), sourceRef(std::move(other.sourceRef)),
          startPosition(std::move(other.startPosition)), endPosition(std::move(other.endPosition)),
          arcCenter(std::move(other.arcCenter)), arcRadius(other.arcRadius), arcSweep(other.arcSweep),
          arcPlane(other.arcPlane), controlPoints(std::move(other.controlPoints)), weights(std::move(other.weights)),
          knots(std::move(other.knots)), degree(other.degree), feedrate(other.feedrate), dwellTime(other.dwellTime),
          dwellDuration(dwellTime), pathMode(other.pathMode), blending(std::move(other.blending)),
          kinematicLimits(std::move(other.kinematicLimits)), segmentLength(other.segmentLength),
          activeAxes(other.activeAxes), numActiveAxes(other.numActiveAxes) {}

    MotionSegment& operator=(MotionSegment&& other) noexcept {
        if (this == &other) return *this;
        id = other.id;
        type = other.type;
        sourceRef = std::move(other.sourceRef);
        startPosition = std::move(other.startPosition);
        endPosition = std::move(other.endPosition);
        arcCenter = std::move(other.arcCenter);
        arcRadius = other.arcRadius;
        arcSweep = other.arcSweep;
        arcPlane = other.arcPlane;
        controlPoints = std::move(other.controlPoints);
        weights = std::move(other.weights);
        knots = std::move(other.knots);
        degree = other.degree;
        feedrate = other.feedrate;
        dwellTime = other.dwellTime;
        pathMode = other.pathMode;
        blending = other.blending;
        kinematicLimits = std::move(other.kinematicLimits);
        segmentLength = other.segmentLength;
        activeAxes = other.activeAxes;
        numActiveAxes = other.numActiveAxes;
        return *this;
    }

    // ========================================================================
    // Factory Methods
    // ========================================================================
    
    /**
     * @brief Create a linear motion segment
     */
    template<size_t N>
    static MotionSegment linear(const Vec<N>& start, const Vec<N>& end,
                                double feedrate, SourceReference sourceRef = {}) {
        MotionSegment seg;
        seg.type = MotionSegmentType::Linear;
        seg.feedrate = feedrate;
        seg.sourceRef = std::move(sourceRef);
        
        for (size_t i = 0; i < N && i < MAX_MOTION_AXES; ++i) {
            seg.startPosition[i] = start[i];
            seg.endPosition[i] = end[i];
            if (std::abs(end[i] - start[i]) > MathConstants::EPSILON) {
                seg.activeAxes[i] = true;
                ++seg.numActiveAxes;
            }
        }
        
        seg.computeLength();
        return seg;
    }

    // --------------------------------------------------------------------
    // Convenience overloads accepting std::array (legacy callers)
    // --------------------------------------------------------------------
    static MotionSegment linear(const std::array<double, MAX_MOTION_AXES>& start,
                                const std::array<double, MAX_MOTION_AXES>& end,
                                double feedrate, SourceReference sourceRef = {}) {
        MotionSegment seg = linear<MAX_MOTION_AXES>(Vec<MAX_MOTION_AXES>(start),
                                                         Vec<MAX_MOTION_AXES>(end),
                                                         feedrate, std::move(sourceRef));
        return seg;
    }

    template<size_t N>
    static MotionSegment rapid(const Vec<N>& start, const Vec<N>& end,
                               SourceReference sourceRef = {}) {
        auto seg = linear(start, end, 0.0, std::move(sourceRef));
        seg.type = MotionSegmentType::Rapid;
        return seg;
    }

    // Convenience overload for std::array
    static MotionSegment rapid(const std::array<double, MAX_MOTION_AXES>& start,
                               const std::array<double, MAX_MOTION_AXES>& end,
                               SourceReference sourceRef = {}) {
        return rapid<MAX_MOTION_AXES>(Vec<MAX_MOTION_AXES>(start), Vec<MAX_MOTION_AXES>(end), std::move(sourceRef));
    }

    template<size_t N>
    static MotionSegment arcCW(const Vec<N>& start, const Vec<N>& end,
                               const Vec<N>& center, double feedrate,
                               ArcPlane plane = ArcPlane::XY,
                               SourceReference sourceRef = {}) {
        MotionSegment seg;
        seg.type = MotionSegmentType::ArcCW;
        seg.feedrate = feedrate;
        seg.arcPlane = plane;
        seg.sourceRef = std::move(sourceRef);
        
        for (size_t i = 0; i < N && i < MAX_MOTION_AXES; ++i) {
            seg.startPosition[i] = start[i];
            seg.endPosition[i] = end[i];
            seg.arcCenter[i] = center[i];
        }
        
        seg.computeArcParameters();
        return seg;
    }

    // Convenience overload for std::array (center provided, feedrate provided)
    static MotionSegment arcCW(const std::array<double, MAX_MOTION_AXES>& start,
                               const std::array<double, MAX_MOTION_AXES>& end,
                               const std::array<double, MAX_MOTION_AXES>& center,
                               double feedrate, ArcPlane plane = ArcPlane::XY,
                               SourceReference sourceRef = {}) {
        return arcCW<MAX_MOTION_AXES>(Vec<MAX_MOTION_AXES>(start),
                                      Vec<MAX_MOTION_AXES>(end),
                                      Vec<MAX_MOTION_AXES>(center),
                                      feedrate, plane, std::move(sourceRef));
    }

    // Overload matching older callers that pass (start, end, center, radius, feedrate)
    static MotionSegment arcCW(const std::array<double, MAX_MOTION_AXES>& start,
                               const std::array<double, MAX_MOTION_AXES>& end,
                               const std::array<double, MAX_MOTION_AXES>& center,
                               double radius, double feedrate,
                               ArcPlane plane = ArcPlane::XY,
                               SourceReference sourceRef = {}) {
        auto seg = arcCW<MAX_MOTION_AXES>(Vec<MAX_MOTION_AXES>(start),
                                          Vec<MAX_MOTION_AXES>(end),
                                          Vec<MAX_MOTION_AXES>(center),
                                          feedrate, plane, std::move(sourceRef));
        seg.arcRadius = std::abs(radius);
        // computeArcParameters will re-evaluate sweep; keep provided radius if needed
        seg.computeArcParameters();
        return seg;
    }

    template<size_t N>
    static MotionSegment arcCCW(const Vec<N>& start, const Vec<N>& end,
                                const Vec<N>& center, double feedrate,
                                ArcPlane plane = ArcPlane::XY,
                                SourceReference sourceRef = {}) {
        auto seg = arcCW(start, end, center, feedrate, plane, std::move(sourceRef));
        seg.type = MotionSegmentType::ArcCCW;
        seg.arcSweep = -seg.arcSweep;  // Reverse direction
        return seg;
    }

    // Convenience overload for std::array
    static MotionSegment arcCCW(const std::array<double, MAX_MOTION_AXES>& start,
                                const std::array<double, MAX_MOTION_AXES>& end,
                                const std::array<double, MAX_MOTION_AXES>& center,
                                double feedrate, ArcPlane plane = ArcPlane::XY,
                                SourceReference sourceRef = {}) {
        return arcCCW<MAX_MOTION_AXES>(Vec<MAX_MOTION_AXES>(start),
                                       Vec<MAX_MOTION_AXES>(end),
                                       Vec<MAX_MOTION_AXES>(center),
                                       feedrate, plane, std::move(sourceRef));
    }

    // Overload matching older callers that pass (start, end, center, radius, feedrate)
    static MotionSegment arcCCW(const std::array<double, MAX_MOTION_AXES>& start,
                                const std::array<double, MAX_MOTION_AXES>& end,
                                const std::array<double, MAX_MOTION_AXES>& center,
                                double radius, double feedrate,
                                ArcPlane plane = ArcPlane::XY,
                                SourceReference sourceRef = {}) {
        auto seg = arcCCW<MAX_MOTION_AXES>(Vec<MAX_MOTION_AXES>(start),
                                           Vec<MAX_MOTION_AXES>(end),
                                           Vec<MAX_MOTION_AXES>(center),
                                           feedrate, plane, std::move(sourceRef));
        seg.arcRadius = std::abs(radius);
        seg.computeArcParameters();
        return seg;
    }

    /**
     * @brief Create a dwell segment
     */
    static MotionSegment dwell(double seconds, const std::array<double, MAX_MOTION_AXES>& position,
                               SourceReference sourceRef = {}) {
        MotionSegment seg;
        seg.type = MotionSegmentType::Dwell;
        seg.dwellTime = seconds;
        seg.startPosition = position;
        seg.endPosition = position;
        seg.sourceRef = std::move(sourceRef);
        seg.segmentLength = 0.0;
        return seg;
    }

    // Backwards-compatible overload with (position, seconds) ordering
    static MotionSegment dwell(const std::array<double, MAX_MOTION_AXES>& position,
                               double seconds,
                               SourceReference sourceRef = {}) {
        return dwell(seconds, position, std::move(sourceRef));
    }

    /**
     * @brief Create a NURBS motion segment
     *
     * @param start Start position
     * @param end End position (final point on NURBS curve)
     * @param poles Control points (NURBS poles)
     * @param weights NURBS weights (empty for uniform B-spline)
     * @param knots NURBS knot vector
     * @param degree NURBS curve degree
     * @param feedrate Feed rate in units/sec
     * @param sourceRef Source reference for traceability
     */
    static MotionSegment nurbs(const std::array<double, MAX_MOTION_AXES>& start,
                               const std::array<double, MAX_MOTION_AXES>& end,
                               const std::vector<std::array<double, MAX_MOTION_AXES>>& poles,
                               const std::vector<double>& weights,
                               const std::vector<double>& knots,
                               size_t nurbsDegree,
                               double feedrate,
                               SourceReference sourceRef = {}) {
        MotionSegment seg;
        seg.type = MotionSegmentType::NURBS;
        seg.startPosition = start;
        seg.endPosition = end;
        seg.controlPoints = poles;
        seg.weights = weights;
        seg.knots = knots;
        seg.degree = nurbsDegree;
        seg.feedrate = feedrate;
        seg.sourceRef = std::move(sourceRef);

        // Approximate segment length from control polygon
        double polyLen = 0.0;
        if (!poles.empty()) {
            for (size_t i = 1; i < poles.size(); ++i) {
                double sumSq = 0.0;
                for (size_t ax = 0; ax < MAX_MOTION_AXES; ++ax) {
                    double d = poles[i][ax] - poles[i-1][ax];
                    sumSq += d * d;
                }
                polyLen += std::sqrt(sumSq);
            }
        }
        seg.segmentLength = polyLen;
        return seg;
    }
    
    // ========================================================================
    // Properties
    // ========================================================================
    
    /**
     * @brief Check if this is an arc segment
     */
    bool isArc() const {
        return type == MotionSegmentType::ArcCW || type == MotionSegmentType::ArcCCW;
    }
    
    /**
     * @brief Check if this is a spline segment
     */
    bool isSpline() const {
        return type == MotionSegmentType::CubicSpline ||
               type == MotionSegmentType::QuadSpline ||
               type == MotionSegmentType::NURBS;
    }

    /**
     * @brief Check if this is a linear segment (legacy)
     */
    bool isLinear() const {
        return type == MotionSegmentType::Linear;
    }
    
    /**
     * @brief Check if this is a rapid move
     */
    bool isRapid() const {
        return type == MotionSegmentType::Rapid;
    }
    
    /**
     * @brief Check if this requires exact stop
     */
    bool requiresExactStop() const {
        return pathMode == PathMode::ExactStop;
    }
    
    /**
     * @brief Get arc direction (1 for CCW, -1 for CW)
     */
    int arcDirection() const {
        return (type == MotionSegmentType::ArcCCW) ? 1 : -1;
    }
    
    /**
     * @brief Get 3D start position
     */
    Vec3 start3D() const {
        return Vec3{startPosition[0], startPosition[1], startPosition[2]};
    }
    
    /**
     * @brief Get 3D end position
     */
    Vec3 end3D() const {
        return Vec3{endPosition[0], endPosition[1], endPosition[2]};
    }
    
    /**
     * @brief Get 3D arc center
     */
    Vec3 center3D() const {
        return Vec3{arcCenter[0], arcCenter[1], arcCenter[2]};
    }
    
private:
    /**
     * @brief Compute segment length for linear moves
     */
    void computeLength() {
        double sumSq = 0.0;
        for (size_t i = 0; i < MAX_MOTION_AXES; ++i) {
            double d = endPosition[i] - startPosition[i];
            sumSq += d * d;
        }
        segmentLength = std::sqrt(sumSq);
    }
    
    /**
     * @brief Compute arc parameters (radius, sweep)
     */
    void computeArcParameters() {
        // Get indices based on plane
        size_t i1, i2;
        switch (arcPlane) {
            case ArcPlane::XY: i1 = 0; i2 = 1; break;
            case ArcPlane::XZ: i1 = 0; i2 = 2; break;
            case ArcPlane::YZ: i1 = 1; i2 = 2; break;
        }
        
        // Compute radius from start to center
        double dx1 = startPosition[i1] - arcCenter[i1];
        double dy1 = startPosition[i2] - arcCenter[i2];
        arcRadius = std::sqrt(dx1*dx1 + dy1*dy1);
        
        // Compute angles
        double startAngle = std::atan2(dy1, dx1);
        double dx2 = endPosition[i1] - arcCenter[i1];
        double dy2 = endPosition[i2] - arcCenter[i2];
        double endAngle = std::atan2(dy2, dx2);
        
        // Compute sweep (CW = negative)
        arcSweep = endAngle - startAngle;
        if (type == MotionSegmentType::ArcCW) {
            if (arcSweep > 0) arcSweep -= 2.0 * MathConstants::PI;
        } else {
            if (arcSweep < 0) arcSweep += 2.0 * MathConstants::PI;
        }
        
        // Arc length = |sweep| * radius
        segmentLength = std::abs(arcSweep) * arcRadius;

        // Mark active axes. For an arc, BOTH plane axes (i1, i2) are always
        // active — the arc moves in both plane axes even if the start and
        // end happen to coincide on one of them (e.g. a semicircle from
        // (10,0) to (20,0) with center (15,0) passes through Y=5 but has
        // start.Y == end.Y == 0). Non-plane axes use the displacement check.
        for (size_t i = 0; i < MAX_MOTION_AXES; ++i) {
            if (i == i1 || i == i2) {
                // Plane axis: always active for arcs (unless the arc is
                // degenerate — zero radius or zero sweep).
                if (arcRadius > MathConstants::EPSILON &&
                    std::abs(arcSweep) > MathConstants::EPSILON) {
                    activeAxes[i] = true;
                    ++numActiveAxes;
                }
            } else {
                // Non-plane axis: active only if the axis actually moves.
                if (std::abs(endPosition[i] - startPosition[i]) > MathConstants::EPSILON) {
                    activeAxes[i] = true;
                    ++numActiveAxes;
                }
            }
        }
    }
};

// ============================================================================
// Motion Segment Node (for linked list)
// ============================================================================

/**
 * @brief Node in doubly-linked segment list
 */
struct MotionSegmentNode {
    MotionSegment segment;
    size_t index = 0;
    MotionSegmentNode* prev = nullptr;
    MotionSegmentNode* next = nullptr;
    
    explicit MotionSegmentNode(MotionSegment seg) : segment(std::move(seg)) {}
};

// ============================================================================
// Motion Segment List
// ============================================================================

/**
 * @brief Doubly-linked list of motion segments with lookahead/lookbehind
 *
 * Provides efficient bidirectional traversal and random access.
 */
class MotionSegmentList {
public:
    using Node = MotionSegmentNode;

    // ========================================================================
    // Constructors / Destructor
    // ========================================================================
    
    MotionSegmentList() = default;
    
    ~MotionSegmentList() {
        clear();
    }
    
    // Disable copy (use clone() if needed)
    MotionSegmentList(const MotionSegmentList&) = delete;
    MotionSegmentList& operator=(const MotionSegmentList&) = delete;
    
    // Enable move
    MotionSegmentList(MotionSegmentList&& other) noexcept {
        swap(other);
    }
    
    MotionSegmentList& operator=(MotionSegmentList&& other) noexcept {
        if (this != &other) {
            clear();
            swap(other);
        }
        return *this;
    }

    // ========================================================================
    // Modification
    // ========================================================================
    
    /**
     * @brief Append segment to end
     */
    void append(MotionSegment segment) {
        auto* node = new Node(std::move(segment));
        node->index = size_;
        node->segment.id = nextId_++;
        
        if (tail_) {
            tail_->next = node;
            node->prev = tail_;
            tail_ = node;
        } else {
            head_ = tail_ = node;
        }
        
        indexCache_.push_back(node);
        ++size_;
    }
    
    /**
     * @brief Prepend segment to beginning
     */
    void prepend(MotionSegment segment) {
        auto* node = new Node(std::move(segment));
        node->segment.id = nextId_++;
        
        if (head_) {
            head_->prev = node;
            node->next = head_;
            head_ = node;
        } else {
            head_ = tail_ = node;
        }
        
        rebuildIndexCache();
    }
    
    /**
     * @brief Insert segment after the given node
     */
    void insertAfter(Node* after, MotionSegment segment) {
        if (!after) {
            prepend(std::move(segment));
            return;
        }
        
        auto* node = new Node(std::move(segment));
        node->segment.id = nextId_++;
        
        node->prev = after;
        node->next = after->next;
        
        if (after->next) {
            after->next->prev = node;
        } else {
            tail_ = node;
        }
        after->next = node;
        
        rebuildIndexCache();
    }
    
    /**
     * @brief Remove a node
     */
    void remove(Node* node) {
        if (!node) return;
        
        if (node->prev) {
            node->prev->next = node->next;
        } else {
            head_ = node->next;
        }
        
        if (node->next) {
            node->next->prev = node->prev;
        } else {
            tail_ = node->prev;
        }
        
        delete node;
        rebuildIndexCache();
    }
    
    /**
     * @brief Clear all segments
     */
    void clear() {
        Node* current = head_;
        while (current) {
            Node* next = current->next;
            delete current;
            current = next;
        }
        head_ = tail_ = nullptr;
        size_ = 0;
        indexCache_.clear();
    }

    // ========================================================================
    // Access
    // ========================================================================
    
    /**
     * @brief Get number of segments
     */
    size_t size() const noexcept { return size_; }
    
    /**
     * @brief Check if empty
     */
    bool empty() const noexcept { return size_ == 0; }
    
    /**
     * @brief Get segment by index (O(1) with cache)
     */
    const MotionSegment& operator[](size_t index) const {
        return at(index);
    }
    
    /**
     * @brief Get segment by index with bounds checking
     */
    const MotionSegment& at(size_t index) const {
        if (index >= size_) {
            throw std::out_of_range("Segment index out of range");
        }
        return indexCache_[index]->segment;
    }
    
    /**
     * @brief Get mutable segment by index
     */
    MotionSegment& at(size_t index) {
        if (index >= size_) {
            throw std::out_of_range("Segment index out of range");
        }
        return indexCache_[index]->segment;
    }
    
    /**
     * @brief Get node by index
     */
    Node* nodeAt(size_t index) {
        if (index >= size_) return nullptr;
        return indexCache_[index];
    }
    
    const Node* nodeAt(size_t index) const {
        if (index >= size_) return nullptr;
        return indexCache_[index];
    }
    
    /**
     * @brief Get first segment
     */
    const MotionSegment& front() const {
        if (!head_) throw std::out_of_range("Empty list");
        return head_->segment;
    }
    
    /**
     * @brief Get last segment
     */
    const MotionSegment& back() const {
        if (!tail_) throw std::out_of_range("Empty list");
        return tail_->segment;
    }
    
    /**
     * @brief Get head node
     */
    Node* head() noexcept { return head_; }
    const Node* head() const noexcept { return head_; }
    
    /**
     * @brief Get tail node
     */
    Node* tail() noexcept { return tail_; }
    const Node* tail() const noexcept { return tail_; }

    // ========================================================================
    // Lookahead / Lookbehind
    // ========================================================================
    
    /**
     * @brief Get lookahead segments from index
     *
     * @param index Starting segment index
     * @param count Number of segments to look ahead
     * @return Vector of segment pointers (may be less than count)
     */
    std::vector<const MotionSegment*> getLookahead(size_t index, size_t count) const {
        std::vector<const MotionSegment*> result;
        result.reserve(count);
        
        if (index >= size_) return result;
        
        const Node* node = indexCache_[index];
        for (size_t i = 0; i < count && node; ++i) {
            result.push_back(&node->segment);
            node = node->next;
        }
        
        return result;
    }
    
    /**
     * @brief Get lookbehind segments from index
     *
     * @param index Starting segment index
     * @param count Number of segments to look behind
     * @return Vector of segment pointers (may be less than count), in reverse order
     */
    std::vector<const MotionSegment*> getLookbehind(size_t index, size_t count) const {
        std::vector<const MotionSegment*> result;
        result.reserve(count);
        
        if (index >= size_) return result;
        
        const Node* node = indexCache_[index];
        for (size_t i = 0; i < count && node; ++i) {
            result.push_back(&node->segment);
            node = node->prev;
        }
        
        return result;
    }
    
    /**
     * @brief Check if lookahead is available
     */
    bool hasLookahead(size_t index, size_t count) const {
        return (index + count) <= size_;
    }
    
    /**
     * @brief Check if lookbehind is available
     */
    bool hasLookbehind(size_t index, size_t count) const {
        return index >= count;
    }

    // ========================================================================
    // Search
    // ========================================================================
    
    /**
     * @brief Find segment by G-code line number
     *
     * @param lineNumber Source line number to find
     * @return Index if found, nullopt otherwise
     */
    std::optional<size_t> findByLineNumber(uint32_t lineNumber) const {
        for (size_t i = 0; i < size_; ++i) {
            if (indexCache_[i]->segment.sourceRef.containsLine(lineNumber)) {
                return i;
            }
        }
        return std::nullopt;
    }
    
    /**
     * @brief Find all segments from a G-code line
     */
    std::vector<size_t> findAllByLineNumber(uint32_t lineNumber) const {
        std::vector<size_t> result;
        for (size_t i = 0; i < size_; ++i) {
            if (indexCache_[i]->segment.sourceRef.containsLine(lineNumber)) {
                result.push_back(i);
            }
        }
        return result;
    }

    // ========================================================================
    // Iteration
    // ========================================================================
    
    /**
     * @brief Forward iterator
     */
    class Iterator {
    public:
        Iterator(Node* node) : node_(node) {}
        
        const MotionSegment& operator*() const { return node_->segment; }
        const MotionSegment* operator->() const { return &node_->segment; }
        
        Iterator& operator++() { 
            if (node_) node_ = node_->next;
            return *this;
        }
        
        Iterator operator++(int) {
            Iterator tmp = *this;
            ++(*this);
            return tmp;
        }
        
        bool operator==(const Iterator& other) const { return node_ == other.node_; }
        bool operator!=(const Iterator& other) const { return node_ != other.node_; }
        
        Node* node() { return node_; }
        
    private:
        Node* node_;
    };
    
    /**
     * @brief Reverse iterator
     */
    class ReverseIterator {
    public:
        ReverseIterator(Node* node) : node_(node) {}
        
        const MotionSegment& operator*() const { return node_->segment; }
        const MotionSegment* operator->() const { return &node_->segment; }
        
        ReverseIterator& operator++() {
            if (node_) node_ = node_->prev;
            return *this;
        }
        
        ReverseIterator operator++(int) {
            ReverseIterator tmp = *this;
            ++(*this);
            return tmp;
        }
        
        bool operator==(const ReverseIterator& other) const { return node_ == other.node_; }
        bool operator!=(const ReverseIterator& other) const { return node_ != other.node_; }
        
        Node* node() { return node_; }
        
    private:
        Node* node_;
    };
    
    Iterator begin() { return Iterator(head_); }
    Iterator end() { return Iterator(nullptr); }
    
    Iterator begin() const { return Iterator(head_); }
    Iterator end() const { return Iterator(nullptr); }
    
    ReverseIterator rbegin() { return ReverseIterator(tail_); }
    ReverseIterator rend() { return ReverseIterator(nullptr); }
    
    /**
     * @brief Apply function to each segment
     */
    void forEach(std::function<void(MotionSegment&, size_t)> fn) {
        Node* node = head_;
        size_t index = 0;
        while (node) {
            fn(node->segment, index++);
            node = node->next;
        }
    }
    
    void forEach(std::function<void(const MotionSegment&, size_t)> fn) const {
        const Node* node = head_;
        size_t index = 0;
        while (node) {
            fn(node->segment, index++);
            node = node->next;
        }
    }

    // ========================================================================
    // Statistics
    // ========================================================================
    
    /**
     * @brief Compute total path length
     */
    double totalLength() const {
        double total = 0.0;
        for (const auto& seg : *this) {
            total += seg.segmentLength;
        }
        return total;
    }
    
    /**
     * @brief Count segments by type
     */
    std::array<size_t, 8> countByType() const {
        std::array<size_t, 8> counts{};
        for (const auto& seg : *this) {
            counts[static_cast<size_t>(seg.type)]++;
        }
        return counts;
    }

private:
    Node* head_ = nullptr;
    Node* tail_ = nullptr;
    size_t size_ = 0;
    uint64_t nextId_ = 1;
    std::vector<Node*> indexCache_;
    
    void swap(MotionSegmentList& other) noexcept {
        std::swap(head_, other.head_);
        std::swap(tail_, other.tail_);
        std::swap(size_, other.size_);
        std::swap(nextId_, other.nextId_);
        std::swap(indexCache_, other.indexCache_);
    }
    
    void rebuildIndexCache() {
        indexCache_.clear();
        indexCache_.reserve(size_);
        Node* node = head_;
        size_t index = 0;
        while (node) {
            node->index = index++;
            indexCache_.push_back(node);
            node = node->next;
        }
        size_ = indexCache_.size();
    }
};

}  // namespace MotionPlanner
