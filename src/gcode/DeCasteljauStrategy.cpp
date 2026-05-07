/**
 * @file DeCasteljauStrategy.cpp
 * @brief Implementation of De Casteljau Bézier Interpolation Strategy
 *
 * Bézier curve interpolation using the de Casteljau algorithm with
 * adaptive subdivision for splines and arc-to-Bézier conversion.
 */

#include "gcode/motion/InterpolationStrategy.hpp"
#include <cmath>
#include <algorithm>
#include <array>

namespace GCode {

using namespace InterpolationConstants;

// ============================================================================
// De Casteljau Strategy
// ============================================================================

class DeCasteljauStrategy : public InterpolationStrategy {
public:
    InterpolationStrategyType type() const override {
        return InterpolationStrategyType::DeCasteljau;
    }

    const char* name() const override {
        return "De Casteljau";
    }

    void configure(const InterpolationConfig& config) override {
        config_ = config;
    }

    InterpolationResult interpolateSegment(
        const PlanningSegment& segment,
        InterpolationContext& ctx,
        std::vector<TrajectoryPoint>& points
    ) override {
        InterpolationResult result;
        result.success = true;
        points.clear();

        double currentTime = ctx.currentTime;

        if (segment.isSpline() && segment.controlPoints.size() >= 2) {
            // Use de Casteljau for spline segments
            std::vector<double> params;
            params.push_back(0.0);
            subdivideSpline(segment, 0.0, 1.0, params, 0, result);
            params.push_back(1.0);

            std::sort(params.begin(), params.end());
            params.erase(std::unique(params.begin(), params.end(),
                [](double a, double b) { return std::fabs(a - b) < InterpolationConstants::EPSILON; }), params.end());

            for (double t : params) {
                TrajectoryPoint pt;
                pt.time = currentTime + t * segment.segmentTime;
                pt.position = evaluateSplinePosition(segment, t);
                pt.velocity = evaluateSplineVelocity(segment, t);
                pt.parameter = t;
                pt.pathLength = arcLength(segment, t);
                pt.blockIndex = segment.blockIndex;
                pt.segmentIndex = static_cast<int32_t>(ctx.currentSegmentIndex);
                pt.motionType = segment.motionType;
                pt.isInterpolated = (t > InterpolationConstants::EPSILON && t < 1.0 - InterpolationConstants::EPSILON);

                points.push_back(pt);
                ctx.updateBounds(pt.position);
            }
        } else if (segment.isArc()) {
            // For arcs, convert to rational Bézier segments
            std::vector<BezierSegment> beziers = arcToBezier(segment);

            std::vector<double> params;
            double segmentFraction = 1.0 / beziers.size();

            for (size_t i = 0; i < beziers.size(); ++i) {
                double tBase = i * segmentFraction;
                std::vector<double> localParams;
                localParams.push_back(0.0);
                subdivideBezier(beziers[i], 0.0, 1.0, localParams, 0, result);
                localParams.push_back(1.0);

                for (double lt : localParams) {
                    params.push_back(tBase + lt * segmentFraction);
                }
            }

            std::sort(params.begin(), params.end());
            params.erase(std::unique(params.begin(), params.end(),
                [](double a, double b) { return std::fabs(a - b) < InterpolationConstants::EPSILON; }), params.end());

            for (double t : params) {
                TrajectoryPoint pt;
                pt.time = currentTime + t * segment.segmentTime;
                pt.position = evaluatePosition(segment, t);
                pt.velocity = evaluateVelocity(segment, t);
                pt.acceleration = evaluateAcceleration(segment, t);
                pt.curvature = evaluateCurvature(segment, t);
                pt.parameter = t;
                pt.pathLength = arcLength(segment, t);
                pt.blockIndex = segment.blockIndex;
                pt.segmentIndex = static_cast<int32_t>(ctx.currentSegmentIndex);
                pt.motionType = segment.motionType;
                pt.isInterpolated = (t > InterpolationConstants::EPSILON && t < 1.0 - InterpolationConstants::EPSILON);

                points.push_back(pt);
                ctx.updateBounds(pt.position);
            }
        } else {
            // For linear segments, just use endpoints (with time-based intermediate points)
            size_t numPoints = std::max<size_t>(2,
                static_cast<size_t>(std::ceil(segment.segmentTime / config_.timeResolution)) + 1);

            for (size_t i = 0; i <= numPoints - 1; ++i) {
                double t = (numPoints > 1) ? static_cast<double>(i) / (numPoints - 1) : 1.0;

                TrajectoryPoint pt;
                pt.time = currentTime + t * segment.segmentTime;
                pt.position = evaluatePosition(segment, t);
                pt.velocity = evaluateVelocity(segment, t);
                pt.acceleration = evaluateAcceleration(segment, t);
                pt.parameter = t;
                pt.pathLength = arcLength(segment, t);
                pt.blockIndex = segment.blockIndex;
                pt.segmentIndex = static_cast<int32_t>(ctx.currentSegmentIndex);
                pt.motionType = segment.motionType;
                pt.isInterpolated = (i > 0 && i < numPoints - 1);

                points.push_back(pt);
                ctx.updateBounds(pt.position);

                result.totalIterations++;
            }
        }

        ctx.currentTime += segment.segmentTime;
        ctx.currentPosition = segment.end;

        return result;
    }

private:
    struct BezierSegment {
        std::array<Position, 4> controlPoints;  // Cubic Bézier
        double weight = 1.0;  // For rational Bézier
    };

    // De Casteljau algorithm for Bézier curve evaluation
    Position deCasteljau(const std::vector<Position>& points, double t) const {
        std::vector<Position> current = points;

        while (current.size() > 1) {
            std::vector<Position> next;
            for (size_t i = 0; i < current.size() - 1; ++i) {
                Position p;
                for (size_t k = 0; k < MAX_AXES; ++k) {
                    p[k] = (1.0 - t) * current[i][k] + t * current[i + 1][k];
                }
                next.push_back(p);
            }
            current = std::move(next);
        }

        return current[0];
    }

    Position evaluateSplinePosition(const PlanningSegment& segment, double t) const {
        if (segment.controlPoints.empty()) {
            return evaluatePosition(segment, t);
        }
        return deCasteljau(segment.controlPoints, t);
    }

    Position evaluateSplineVelocity(const PlanningSegment& segment, double t) const {
        if (segment.controlPoints.size() < 2) {
            return evaluateVelocity(segment, t);
        }

        // Compute derivative via hodograph
        std::vector<Position> hodograph;
        size_t n = segment.controlPoints.size() - 1;

        for (size_t i = 0; i < n; ++i) {
            Position h;
            for (size_t k = 0; k < MAX_AXES; ++k) {
                h[k] = n * (segment.controlPoints[i + 1][k] - segment.controlPoints[i][k]);
            }
            hodograph.push_back(h);
        }

        Position deriv = deCasteljau(hodograph, t);

        // Scale by segment time to get velocity
        if (segment.segmentTime > 0) {
            for (size_t k = 0; k < MAX_AXES; ++k) {
                deriv[k] /= segment.segmentTime;
            }
        }

        return deriv;
    }

    void subdivideSpline(
        const PlanningSegment& segment,
        double t0, double t1,
        std::vector<double>& params,
        int depth,
        InterpolationResult& result
    ) {
        result.totalIterations++;

        if (depth >= config_.maxSubdivisionDepth) {
            return;
        }

        double tMid = (t0 + t1) * 0.5;

        Position p0 = evaluateSplinePosition(segment, t0);
        Position p1 = evaluateSplinePosition(segment, t1);
        Position pMid = evaluateSplinePosition(segment, tMid);

        Position pLinear;
        for (size_t i = 0; i < MAX_AXES; ++i) {
            pLinear[i] = (p0[i] + p1[i]) * 0.5;
        }

        double error = pMid.linearDistance(pLinear);

        if (error > config_.maxChordDeviation && (t1 - t0) > config_.minStepSize) {
            subdivideSpline(segment, t0, tMid, params, depth + 1, result);
            params.push_back(tMid);
            subdivideSpline(segment, tMid, t1, params, depth + 1, result);
        }
    }

    void subdivideBezier(
        const BezierSegment& bezier,
        double t0, double t1,
        std::vector<double>& params,
        int depth,
        InterpolationResult& result
    ) {
        result.totalIterations++;

        if (depth >= config_.maxSubdivisionDepth) {
            return;
        }

        double tMid = (t0 + t1) * 0.5;

        std::vector<Position> points(bezier.controlPoints.begin(), bezier.controlPoints.end());
        Position p0 = deCasteljau(points, t0);
        Position p1 = deCasteljau(points, t1);
        Position pMid = deCasteljau(points, tMid);

        Position pLinear;
        for (size_t i = 0; i < MAX_AXES; ++i) {
            pLinear[i] = (p0[i] + p1[i]) * 0.5;
        }

        double error = pMid.linearDistance(pLinear);

        if (error > config_.maxChordDeviation && (t1 - t0) > config_.minStepSize) {
            subdivideBezier(bezier, t0, tMid, params, depth + 1, result);
            params.push_back(tMid);
            subdivideBezier(bezier, tMid, t1, params, depth + 1, result);
        }
    }

    std::vector<BezierSegment> arcToBezier(const PlanningSegment& segment) const {
        // Convert arc to rational quadratic Bézier segments (max 90 degrees each)
        std::vector<BezierSegment> result;

        if (!segment.isArc() || segment.arcRadius <= 0 || segment.arcSweep == 0) {
            return result;
        }

        int u, v, w;
        getPlaneAxes(segment.plane, u, v, w);

        double totalSweep = std::fabs(segment.arcSweep);
        int numSegments = static_cast<int>(std::ceil(totalSweep / (PI / 2.0)));
        numSegments = std::max(1, numSegments);

        double sweepPerSegment = segment.arcSweep / numSegments;
        double cu = segment.center[u];
        double cv = segment.center[v];
        double su = segment.start[u];
        double sv = segment.start[v];
        double startAngle = std::atan2(sv - cv, su - cu);

        for (int i = 0; i < numSegments; ++i) {
            BezierSegment bezier;

            double theta0 = startAngle + i * sweepPerSegment;
            double theta1 = startAngle + (i + 1) * sweepPerSegment;
            double thetaMid = (theta0 + theta1) / 2.0;

            // Control points for rational quadratic Bézier approximating arc
            bezier.controlPoints[0] = segment.start;  // Will be overwritten
            bezier.controlPoints[0][u] = cu + segment.arcRadius * std::cos(theta0);
            bezier.controlPoints[0][v] = cv + segment.arcRadius * std::sin(theta0);

            // Middle control point (on tangent intersection)
            double halfAngle = std::fabs(sweepPerSegment) / 2.0;
            // Note: k is calculated but not used directly for px/py calculation
            // double k = std::tan(halfAngle);
            double px = cu + segment.arcRadius * std::cos(thetaMid) / std::cos(halfAngle);
            double py = cv + segment.arcRadius * std::sin(thetaMid) / std::cos(halfAngle);

            bezier.controlPoints[1] = bezier.controlPoints[0];
            bezier.controlPoints[1][u] = px;
            bezier.controlPoints[1][v] = py;

            bezier.controlPoints[2] = segment.start;
            bezier.controlPoints[2][u] = cu + segment.arcRadius * std::cos(theta1);
            bezier.controlPoints[2][v] = cv + segment.arcRadius * std::sin(theta1);

            bezier.controlPoints[3] = bezier.controlPoints[2];  // Duplicate for cubic

            // Weight for rational Bézier
            bezier.weight = std::cos(halfAngle);

            result.push_back(bezier);
        }

        return result;
    }
};

// Factory registration helper - defined in InterpolationStrategy.cpp
std::unique_ptr<InterpolationStrategy> createDeCasteljauStrategy() {
    return std::make_unique<DeCasteljauStrategy>();
}

} // namespace GCode
