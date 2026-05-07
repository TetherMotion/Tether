/**
 * @file AdaptiveMidpointStrategy.cpp
 * @brief Implementation of Adaptive Midpoint Interpolation Strategy
 *
 * Recursive midpoint subdivision strategy that adapts to curvature
 * by subdividing where chord deviation exceeds tolerance.
 */

#include "gcode/motion/InterpolationStrategy.hpp"
#include <cmath>
#include <algorithm>

namespace GCode {

using namespace InterpolationConstants;

// ============================================================================
// Adaptive Midpoint Strategy
// ============================================================================

class AdaptiveMidpointStrategy : public InterpolationStrategy {
public:
    InterpolationStrategyType type() const override {
        return InterpolationStrategyType::AdaptiveMidpoint;
    }

    const char* name() const override {
        return "Adaptive Midpoint";
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

        // Recursive subdivision
        std::vector<double> params;
        params.push_back(0.0);
        subdivide(segment, 0.0, 1.0, params, 0, result);
        params.push_back(1.0);

        // Sort and remove duplicates
        std::sort(params.begin(), params.end());
        params.erase(std::unique(params.begin(), params.end(),
            [](double a, double b) { return std::fabs(a - b) < InterpolationConstants::EPSILON; }), params.end());

        // Generate points at each parameter
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

        ctx.currentTime += segment.segmentTime;
        ctx.currentPosition = segment.end;

        return result;
    }

private:
    void subdivide(
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

        Position p0 = evaluatePosition(segment, t0);
        Position p1 = evaluatePosition(segment, t1);
        Position pMid = evaluatePosition(segment, tMid);

        // Linear midpoint
        Position pLinear;
        for (size_t i = 0; i < MAX_AXES; ++i) {
            pLinear[i] = (p0[i] + p1[i]) * 0.5;
        }

        // Deviation from linear interpolation
        double error = pMid.linearDistance(pLinear);

        if (error > config_.maxChordDeviation && (t1 - t0) > config_.minStepSize) {
            // Subdivide further
            subdivide(segment, t0, tMid, params, depth + 1, result);
            params.push_back(tMid);
            subdivide(segment, tMid, t1, params, depth + 1, result);
        }
    }
};

// Factory registration helper - defined in InterpolationStrategy.cpp
std::unique_ptr<InterpolationStrategy> createAdaptiveMidpointStrategy() {
    return std::make_unique<AdaptiveMidpointStrategy>();
}

} // namespace GCode
