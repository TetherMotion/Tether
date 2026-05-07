/**
 * @file FixedTimeStrategy.cpp
 * @brief Implementation of Fixed Time Interpolation Strategy
 *
 * Fixed time stepping strategy that generates trajectory points
 * at uniform time intervals.
 */

#include "gcode/motion/InterpolationStrategy.hpp"
#include <cmath>

namespace GCode {

// ============================================================================
// Fixed Time Strategy
// ============================================================================

class FixedTimeStrategy : public InterpolationStrategy {
public:
    InterpolationStrategyType type() const override {
        return InterpolationStrategyType::FixedTime;
    }

    const char* name() const override {
        return "Fixed Time";
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

        double dt = config_.timeResolution;
        if (dt <= 0) dt = 0.001;

        size_t numPoints = 2;
        if (segment.segmentTime > dt) {
            numPoints = static_cast<size_t>(std::ceil(segment.segmentTime / dt)) + 1;
        }

        double currentTime = ctx.currentTime;

        for (size_t i = 0; i <= numPoints - 1; ++i) {
            double t = (numPoints > 1) ? static_cast<double>(i) / (numPoints - 1) : 1.0;
            double pointTime = currentTime + t * segment.segmentTime;

            TrajectoryPoint pt;
            pt.time = pointTime;
            pt.position = evaluatePosition(segment, t);
            pt.velocity = evaluateVelocity(segment, t);
            pt.acceleration = evaluateAcceleration(segment, t);
            pt.curvature = evaluateCurvature(segment, t);
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

        ctx.currentTime += segment.segmentTime;
        ctx.currentPosition = segment.end;

        return result;
    }
};

// Factory registration helper - defined in InterpolationStrategy.cpp
std::unique_ptr<InterpolationStrategy> createFixedTimeStrategy() {
    return std::make_unique<FixedTimeStrategy>();
}

} // namespace GCode
