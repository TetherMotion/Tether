/**
 * @file FixedDeviationStrategy.cpp
 * @brief Implementation of Fixed Deviation Interpolation Strategy
 *
 * Adaptive strategy that adjusts point density based on chord deviation
 * to maintain geometric accuracy.
 */

#include "gcode/motion/InterpolationStrategy.hpp"
#include <cmath>

namespace GCode {

// ============================================================================
// Fixed Deviation Strategy
// ============================================================================

class FixedDeviationStrategy : public InterpolationStrategy {
public:
    InterpolationStrategyType type() const override {
        return InterpolationStrategyType::FixedDeviation;
    }

    const char* name() const override {
        return "Fixed Deviation";
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

        size_t numPoints;

        if (segment.isArc()) {
            // Calculate points needed for chord deviation
            numPoints = arcSegmentCount(
                segment.arcRadius,
                segment.arcSweep,
                config_.maxChordDeviation
            );
            numPoints = std::max<size_t>(numPoints, 8);  // Minimum 8 points for arcs
        } else {
            // For lines, use fixed time with minimum points
            double dt = config_.timeResolution;
            if (dt <= 0) dt = 0.001;
            numPoints = std::max<size_t>(2,
                static_cast<size_t>(std::ceil(segment.segmentTime / dt)) + 1);
        }

        double currentTime = ctx.currentTime;

        for (size_t i = 0; i <= numPoints; ++i) {
            double t = (numPoints > 0) ? static_cast<double>(i) / numPoints : 1.0;
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
            pt.isInterpolated = (i > 0 && i < numPoints);

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
std::unique_ptr<InterpolationStrategy> createFixedDeviationStrategy() {
    return std::make_unique<FixedDeviationStrategy>();
}

} // namespace GCode
