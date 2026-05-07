/**
 * @file PathBuilder.hpp
 * @brief Converts Motion Segments to Piecewise Bézier Paths with Corner Blending
 *
 * @details
 * This file implements the PathBuilder class which takes a list of motion segments
 * and constructs a complete piecewise Bézier path with G2-continuous corner blends.
 *
 * ## Pipeline
 *
 * 1. Input: MotionSegmentList (linear moves, arcs, dwells)
 * 2. Convert each segment to Bézier representation
 * 3. Analyze corners between consecutive segments
 * 4. Insert blend curves at corners
 * 5. Output: PiecewiseBezierPath
 *
 * @see MotionSegment.hpp
 * @see CornerBlending.hpp
 * @see PiecewisePath.hpp
 */

#pragma once

#include "MathTypes.hpp"
#include "BezierCurve.hpp"
#include "NURBSCurve.hpp"
#include "PiecewisePath.hpp"
#include "PiecewiseNURBSPath.hpp"
#include "MotionSegment.hpp"
#include "CornerBlending.hpp"
#include <vector>
#include <optional>
#include <algorithm>

namespace MotionPlanner {

// ============================================================================
// Segment Conversion Utilities
// ============================================================================

/**
 * @brief Converts motion segments to Bézier curves
 */
template<size_t Dim, typename T = double>
class SegmentConverter {
public:
    using Point = Vec<Dim, T>;
    using Curve = BezierCurve<Dim, T>;
    using NCurve = NURBSCurve<Dim, T>;

    /**
     * @brief Convert a linear segment to a NURBS line
     */
    static NCurve linearToNURBS(const MotionSegment& seg) {
        Point start = extractPoint(seg.startPosition);
        Point end = extractPoint(seg.endPosition);
        auto curve = NCurve::makeLine(start, end);
        curve.setSourceRef(seg.sourceRef);
        return curve;
    }

    /**
     * @brief Convert a linear segment to a degree-1 (linear) Bézier curve
     */
    static Curve linearToBezier(const MotionSegment& seg) {
        Point start = extractPoint(seg.startPosition);
        Point end = extractPoint(seg.endPosition);
        return createLinearBezier<Dim, T>(start, end, seg.sourceRef);
    }

    /**
     * @brief Convert a circular arc to a NURBS (degree-2 rational)
     */
    static NCurve arcToNURBS(const MotionSegment& seg) {
        if (seg.arcRadius <= T(0)) {
            return linearToNURBS(seg);
        }

        Point center = extractPoint(seg.arcCenter);
        T radius = static_cast<T>(seg.arcRadius);

        // Get arc plane indices
        size_t i1 = 0, i2 = 1;
        switch (seg.arcPlane) {
            case ArcPlane::XZ: i1 = 0; i2 = 2; break;
            case ArcPlane::YZ: i1 = 1; i2 = 2; break;
            default: break;
        }

        Point start = extractPoint(seg.startPosition);
        Point v1 = start - center;
        T startAngle = std::atan2(v1[i2], v1[i1]);
        T sweepAngle = static_cast<T>(seg.arcSweep);

        if constexpr (Dim == 2) {
            auto curve = NCurve::makeCircularArc(center, radius, startAngle, sweepAngle);
            curve.setSourceRef(seg.sourceRef);
            return curve;
        } else {
            // For 3D, create a 2D arc and extend to 3D
            Vec<2, T> center2D{center[i1], center[i2]};
            auto arc2D = NURBSCurve<2, T>::makeCircularArc(center2D, radius, startAngle, sweepAngle);

            // Convert 2D arc to Dim-D
            typename NCurve::ControlPoints pts3D(arc2D.numControlPoints());
            for (size_t i = 0; i < arc2D.numControlPoints(); ++i) {
                pts3D[i] = Point{};
                pts3D[i][i1] = arc2D.controlPoint(i)[0];
                pts3D[i][i2] = arc2D.controlPoint(i)[1];
                // Interpolate other axes linearly
                for (size_t d = 0; d < Dim; ++d) {
                    if (d != i1 && d != i2) {
                        T alpha = T(i) / T(arc2D.numControlPoints() - 1);
                        pts3D[i][d] = seg.startPosition[d] * (T(1) - alpha) +
                                     seg.endPosition[d] * alpha;
                    }
                }
            }

            NCurve curve(pts3D, arc2D.weights(), arc2D.knots(), arc2D.degree());
            curve.setSourceRef(seg.sourceRef);
            return curve;
        }
    }

    /**
     * @brief Convert a NURBS motion segment directly to a NURBSCurve
     */
    static NCurve nurbsToNURBS(const MotionSegment& seg) {
        // Extract Dim-D control points from MAX_MOTION_AXES arrays
        typename NCurve::ControlPoints pts(seg.controlPoints.size());
        for (size_t i = 0; i < seg.controlPoints.size(); ++i) {
            pts[i] = extractPoint(seg.controlPoints[i]);
        }

        NCurve curve(std::move(pts), seg.weights, seg.knots, seg.degree);
        curve.setSourceRef(seg.sourceRef);
        return curve;
    }

    /**
     * @brief Convert any motion segment to NURBS
     */
    static NCurve segmentToNURBS(const MotionSegment& seg) {
        if (seg.isLinear() || seg.isRapid()) {
            return linearToNURBS(seg);
        } else if (seg.isArc()) {
            return arcToNURBS(seg);
        } else if (seg.type == MotionSegmentType::NURBS) {
            return nurbsToNURBS(seg);
        }
        // Fallback: linear from start to end
        return linearToNURBS(seg);
    }

    /**
     * @brief Convert a circular arc to a sequence of Bézier curves
     *
     * For arcs > 90°, we split into multiple cubic Bézier segments
     * for better approximation accuracy.
     */
    static std::vector<Curve> arcToBezier(const MotionSegment& seg,
                                           T maxAnglePerSegment = MathConstants::PI / T(2)) {
        std::vector<Curve> result;
        
        if (seg.arcRadius <= T(0)) {
            // Invalid arc - return linear approximation
            result.push_back(linearToBezier(seg));
            return result;
        }
        
        Point start = extractPoint(seg.startPosition);
        Point end = extractPoint(seg.endPosition);
        Point center = extractPoint(seg.arcCenter);
        
        // Calculate sweep angle
        Point v1 = start - center;
        Point v2 = end - center;
        
        T startAngle = std::atan2(v1[1], v1[0]);
        T endAngle = std::atan2(v2[1], v2[0]);
        
        bool clockwise = (seg.type == MotionSegmentType::ArcCW);
        T sweepAngle = computeSweepAngle(startAngle, endAngle, clockwise);
        
        // Number of Bézier segments needed
        T absSwingAngle = std::abs(sweepAngle);
        int numSegments = static_cast<int>(std::ceil(absSwingAngle / maxAnglePerSegment));
        numSegments = std::max(1, numSegments);
        
        T segmentAngle = sweepAngle / T(numSegments);
        T currentAngle = startAngle;
        T radius = static_cast<T>(seg.arcRadius);
        
        for (int i = 0; i < numSegments; ++i) {
            T nextAngle = currentAngle + segmentAngle;
            
            Curve arcSegment = createArcBezierSegment(
                center, radius, currentAngle, nextAngle, seg.sourceRef);
            
            result.push_back(std::move(arcSegment));
            currentAngle = nextAngle;
        }
        
        return result;
    }

    /**
     * @brief Create a single cubic Bézier approximation of a circular arc
     *
     * Uses the formula: k = (4/3) * tan(θ/4) where θ is the sweep angle
     */
    static Curve createArcBezierSegment(const Point& center, T radius,
                                         T startAngle, T endAngle,
                                         const SourceReference& sourceRef = {}) {
        // Start and end points on arc
        Point P0 = center + Point{radius * std::cos(startAngle), 
                                   radius * std::sin(startAngle)};
        Point P3 = center + Point{radius * std::cos(endAngle),
                                   radius * std::sin(endAngle)};
        
        // Tangent directions (perpendicular to radius)
        Point t0{-std::sin(startAngle), std::cos(startAngle)};
        Point t3{-std::sin(endAngle), std::cos(endAngle)};
        
        // Extend to higher dimensions if needed
        if constexpr (Dim > 2) {
            for (size_t i = 2; i < Dim; ++i) {
                P0[i] = T(0);
                P3[i] = T(0);
                t0[i] = T(0);
                t3[i] = T(0);
            }
        }
        
        // Magic number for cubic Bézier circular arc approximation
        T sweepAngle = endAngle - startAngle;
        T k = T(4.0/3.0) * std::tan(sweepAngle / T(4));
        
        Point P1 = P0 + t0 * (k * radius);
        Point P2 = P3 - t3 * (k * radius);
        
        Curve curve({P0, P1, P2, P3});
        curve.setSourceRef(sourceRef);
        return curve;
    }

    /**
     * @brief Elevate a curve's degree to a target degree
     *
     * Degree elevation preserves the curve shape while adding control points.
     */
    static Curve elevateDegree(const Curve& curve, size_t targetDegree) {
        if (curve.degree() >= targetDegree) {
            return curve;
        }
        
        // Degree elevation formula:
        // New control points Q[i] = (i/(n+1)) * P[i-1] + ((n+1-i)/(n+1)) * P[i]
        // where n is current degree
        
        Curve result = curve;
        while (result.degree() < targetDegree) {
            size_t n = result.degree();
            std::vector<Point> newPoints(n + 2);
            
            const auto& pts = result.controlPoints();
            
            newPoints[0] = pts[0];
            newPoints[n + 1] = pts[n];
            
            for (size_t i = 1; i <= n; ++i) {
                T alpha = T(i) / T(n + 1);
                newPoints[i] = pts[i - 1] * alpha + pts[i] * (T(1) - alpha);
            }
            
            result = Curve(std::move(newPoints), result.sourceRef());
        }
        
        return result;
    }

private:
    /**
     * @brief Extract N-dimensional point from position array
     */
    static Point extractPoint(const std::array<double, MAX_MOTION_AXES>& pos) {
        Point result;
        for (size_t i = 0; i < Dim && i < MAX_MOTION_AXES; ++i) {
            result[i] = static_cast<T>(pos[i]);
        }
        return result;
    }

    /**
     * @brief Compute sweep angle handling wraparound
     */
    static T computeSweepAngle(T startAngle, T endAngle, bool clockwise) {
        T sweep = endAngle - startAngle;
        
        // Normalize to [-2π, 2π]
        while (sweep > T(2) * MathConstants::PI) sweep -= T(2) * MathConstants::PI;
        while (sweep < T(-2) * MathConstants::PI) sweep += T(2) * MathConstants::PI;
        
        if (clockwise) {
            // CW: need negative sweep
            if (sweep > T(0)) sweep -= T(2) * MathConstants::PI;
        } else {
            // CCW: need positive sweep
            if (sweep < T(0)) sweep += T(2) * MathConstants::PI;
        }
        
        return sweep;
    }
};

// ============================================================================
// Path Building Result
// ============================================================================

/**
 * @brief Result of path building operation
 */
template<size_t Dim, typename T = double>
struct PathBuildResult {
    /// The built piecewise NURBS path (primary output)
    PiecewiseNURBSPath<Dim, T> nurbsPath;
    
    /// Legacy Bézier path (for compatibility)
    PiecewiseBezierPath<Dim, T> path;
    
    /// Number of original segments processed
    size_t inputSegments = 0;
    
    /// Number of curves in output path
    size_t outputCurves = 0;
    
    /// Number of corners blended
    size_t blendedCorners = 0;
    
    /// Number of corners that couldn't be blended (with reasons)
    std::vector<std::pair<size_t, std::string>> unblendedCorners;
    
    /// Build successful?
    bool success = false;
    
    /// Error message if not successful
    std::string errorMessage;
    
    /// Total path length
    T totalLength = T(0);
};

// ============================================================================
// Path Builder
// ============================================================================

/**
 * @brief Builds piecewise Bézier paths from motion segments
 */
template<size_t Dim, typename T = double>
class PathBuilder {
public:
    using Point = Vec<Dim, T>;
    using Curve = BezierCurve<Dim, T>;
    using NCurve = NURBSCurve<Dim, T>;
    using Path = PiecewiseNURBSPath<Dim, T>;
    using LegacyPath = PiecewiseBezierPath<Dim, T>;
    using Result = PathBuildResult<Dim, T>;
    using Analyzer = CornerAnalyzer<Dim, T>;
    using BlendBuilder = BlendCurveBuilder<Dim, T>;
    using Converter = SegmentConverter<Dim, T>;

    /**
     * @brief Constructor
     *
     * @param config Blend configuration
     */
    explicit PathBuilder(BlendConfig config = {})
        : config_(std::move(config)) {}

    /**
     * @brief Build path from motion segment list
     *
     * @param segments Input motion segments
     * @return Path building result with NURBS path
     */
    Result build(const MotionSegmentList& segments) {
        Result result;
        result.inputSegments = segments.size();
        
        if (segments.empty()) {
            result.success = true;
            return result;
        }
        
        // Phase 1: Convert all segments to NURBS curves
        std::vector<SegmentData> segmentData;
        segmentData.reserve(segments.size());
        
        for (size_t i = 0; i < segments.size(); ++i) {
            const MotionSegment& seg = segments.at(i);
            
            if (seg.type == MotionSegmentType::Dwell) {
                continue;
            }
            
            SegmentData data;
            data.originalIndex = i;
            data.segment = &seg;
            data.nurbsCurve = Converter::segmentToNURBS(seg);
            
            if (data.nurbsCurve.isValid()) {
                segmentData.push_back(std::move(data));
            }
        }
        
        if (segmentData.empty()) {
            result.success = true;
            return result;
        }
        
        // Phase 2: Analyze corners and compute blends
        std::vector<BlendData> blends;
        blends.reserve(segmentData.size() - 1);
        
        for (size_t i = 0; i + 1 < segmentData.size(); ++i) {
            const MotionSegment& seg1 = *segmentData[i].segment;
            const MotionSegment& seg2 = *segmentData[i + 1].segment;
            
            BlendData blend;
            blend.beforeIndex = i;
            blend.afterIndex = i + 1;
            
            blend.analysis = Analyzer::analyze(seg1, seg2, config_);
            
            if (blend.analysis.canBlend) {
                SourceReference blendRef = SourceReference::multiple({
                    seg1.sourceRef, seg2.sourceRef
                });
                
                if (config_.continuityLevel >= 2) {
                    // Build G2 blend as NURBS (quintic)
                    auto blendCurve = NCurve::makeG2BlendCurve(
                        blend.analysis.blendEntry,
                        blend.analysis.blendExit,
                        blend.analysis.incomingDir,
                        blend.analysis.outgoingDir,
                        blend.analysis.incomingCurvature,
                        blend.analysis.outgoingCurvature);
                    blendCurve.setSourceRef(std::move(blendRef));
                    blend.nurbsBlendCurves.push_back(std::move(blendCurve));
                } else if (config_.useBezier) {
                    auto bezBlend = BlendBuilder::buildG1BlendCurve(
                        blend.analysis, std::move(blendRef));
                    NCurve nurbsBlend(bezBlend);
                    blend.nurbsBlendCurves.push_back(std::move(nurbsBlend));
                } else {
                    auto bezBlend = BlendBuilder::buildCircularBlendArc(
                        blend.analysis, std::move(blendRef));
                    NCurve nurbsBlend(bezBlend);
                    blend.nurbsBlendCurves.push_back(std::move(nurbsBlend));
                }
                
                result.blendedCorners++;
            } else {
                result.unblendedCorners.push_back({
                    i, blend.analysis.blendReason
                });
            }
            
            blends.push_back(std::move(blend));
        }
        
        // Phase 3: Assemble final NURBS path
        Path nurbsPath;
        
        for (size_t i = 0; i < segmentData.size(); ++i) {
            auto& data = segmentData[i];
            
            const BlendData* blendBefore = (i > 0) ? &blends[i - 1] : nullptr;
            const BlendData* blendAfter = (i + 1 < segmentData.size()) ? &blends[i] : nullptr;
            
            // Trim segment based on blend entry/exit
            NCurve trimmed = trimNURBSSegment(
                data.nurbsCurve, blendBefore, blendAfter);
            
            nurbsPath.appendSegment(std::move(trimmed));
            result.outputCurves++;
            
            // Add blend curve(s) after this segment
            if (blendAfter && blendAfter->analysis.canBlend) {
                for (const auto& blendCurve : blendAfter->nurbsBlendCurves) {
                    nurbsPath.appendSegment(blendCurve);
                    result.outputCurves++;
                }
            }
        }
        
        // Build arc length tables
        nurbsPath.buildArcLengthTables();
        
        // Also build legacy Bézier path for compatibility
        result.path = nurbsPath.toPiecewiseBezierPath();
        
        result.nurbsPath = std::move(nurbsPath);
        result.totalLength = result.nurbsPath.totalLength();
        result.success = true;
        
        return result;
    }

    /**
     * @brief Build path with explicit corner blend decisions
     *
     * @param segments Input motion segments
     * @param blendFlags Per-corner blend enable flags
     * @return Path building result
     */
    Result buildWithFlags(const MotionSegmentList& segments,
                          const std::vector<bool>& blendFlags) {
        // Save original tolerance
        BlendConfig originalConfig = config_;
        
        // Temporarily disable blending where flagged
        Result result = build(segments);
        
        // TODO: Implement per-corner blend control
        
        config_ = originalConfig;
        return result;
    }

    /**
     * @brief Get/set blend configuration
     */
    BlendConfig& config() { return config_; }
    const BlendConfig& config() const { return config_; }

private:
    /**
     * @brief Internal segment data structure
     */
    struct SegmentData {
        size_t originalIndex = 0;
        const MotionSegment* segment = nullptr;
        NCurve nurbsCurve;
        std::vector<Curve> curves; // Legacy
    };

    /**
     * @brief Internal blend data structure
     */
    struct BlendData {
        size_t beforeIndex = 0;
        size_t afterIndex = 0;
        CornerAnalysis<Dim, T> analysis;
        std::vector<NCurve> nurbsBlendCurves;
        std::vector<Curve> blendCurves; // Legacy
    };

    /**
     * @brief Trim a NURBS segment to account for blends
     */
    NCurve trimNURBSSegment(const NCurve& curve,
                            const BlendData* blendBefore,
                            const BlendData* blendAfter) {
        if (!curve.isValid()) return curve;

        bool trimStart = blendBefore && blendBefore->analysis.canBlend;
        bool trimEnd = blendAfter && blendAfter->analysis.canBlend;

        if (!trimStart && !trimEnd) return curve;

        T startParam = T(0);
        T endParam = T(1);

        T curveLength = curve.arcLength();
        if (curveLength <= T(0)) return curve;

        if (trimStart) {
            T trimLen = blendBefore->analysis.exitDistance;
            startParam = std::min(trimLen / curveLength, T(0.5));
        }

        if (trimEnd) {
            T trimLen = blendAfter->analysis.entryDistance;
            endParam = std::max(T(1) - trimLen / curveLength, T(0.5));
        }

        if (startParam >= endParam) return curve;

        // Map [0,1] to curve domain
        T uStart = curve.domainStart();
        T uEnd = curve.domainEnd();
        T domainLen = uEnd - uStart;

        T u0 = uStart + startParam * domainLen;
        T u1 = uStart + endParam * domainLen;

        if (u0 >= u1) return curve;

        // Subdivide to extract middle portion
        if (startParam > T(0) && endParam < T(1)) {
            auto [unused, right] = curve.subdivide(u0);
            T newEndParam = (u1 - u0) / (uEnd - u0);
            newEndParam = clamp(newEndParam, T(0), T(1));
            T newU1 = right.domainStart() + newEndParam * (right.domainEnd() - right.domainStart());
            auto [middle, unused2] = right.subdivide(newU1);
            middle.setSourceRef(curve.sourceRef());
            return middle;
        } else if (startParam > T(0)) {
            auto [unused, right] = curve.subdivide(u0);
            right.setSourceRef(curve.sourceRef());
            return right;
        } else if (endParam < T(1)) {
            auto [left, unused] = curve.subdivide(u1);
            left.setSourceRef(curve.sourceRef());
            return left;
        }

        return curve;
    }

    /**
     * @brief Trim segment curves based on blends (legacy Bézier)
     */
    std::vector<Curve> trimSegmentCurves(const std::vector<Curve>& curves,
                                          const BlendData* blendBefore,
                                          const BlendData* blendAfter) {
        if (curves.empty()) return {};
        
        std::vector<Curve> result;
        
        for (size_t i = 0; i < curves.size(); ++i) {
            const Curve& curve = curves[i];
            
            bool trimStart = (i == 0) && blendBefore && blendBefore->analysis.canBlend;
            bool trimEnd = (i == curves.size() - 1) && blendAfter && blendAfter->analysis.canBlend;
            
            if (!trimStart && !trimEnd) {
                result.push_back(curve);
                continue;
            }
            
            T startParam = T(0);
            T endParam = T(1);
            
            if (trimStart) {
                T curveLength = curve.arcLength();
                if (curveLength > T(0)) {
                    T trimLength = blendBefore->analysis.entryDistance;
                    startParam = std::min(trimLength / curveLength, T(0.5));
                }
            }
            
            if (trimEnd) {
                T curveLength = curve.arcLength();
                if (curveLength > T(0)) {
                    T trimLength = blendAfter->analysis.exitDistance;
                    endParam = std::max(T(1) - trimLength / curveLength, T(0.5));
                }
            }
            
            if (startParam > T(0) || endParam < T(1)) {
                if (startParam < endParam) {
                    auto [unused1, rightPart] = curve.subdivide(startParam);
                    T newEndParam = (endParam - startParam) / (T(1) - startParam);
                    newEndParam = clamp(newEndParam, T(0), T(1));
                    auto [middlePart, unused2] = rightPart.subdivide(newEndParam);
                    middlePart.setSourceRef(curve.sourceRef());
                    result.push_back(std::move(middlePart));
                } else {
                    result.push_back(curve);
                }
            } else {
                result.push_back(curve);
            }
        }
        
        return result;
    }

    BlendConfig config_;
};

// ============================================================================
// Type Aliases
// ============================================================================

using PathBuilder2D = PathBuilder<2, double>;
using PathBuilder3D = PathBuilder<3, double>;

using PathBuildResult2D = PathBuildResult<2, double>;
using PathBuildResult3D = PathBuildResult<3, double>;

using SegmentConverter2D = SegmentConverter<2, double>;
using SegmentConverter3D = SegmentConverter<3, double>;

}  // namespace MotionPlanner
