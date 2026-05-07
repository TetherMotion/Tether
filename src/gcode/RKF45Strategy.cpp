/**
 * @file RKF45Strategy.cpp
 * @brief Implementation of Runge-Kutta-Fehlberg 4(5) Interpolation Strategy
 *
 * Adaptive step-size RKF45 method with embedded error estimation
 * for automatic step size control.
 */

#include "gcode/motion/InterpolationStrategy.hpp"
#include <cmath>

namespace GCode {

using namespace InterpolationConstants;

// ============================================================================
// RKF45 Strategy (Runge-Kutta-Fehlberg 4(5))
// ============================================================================

class RKF45Strategy : public InterpolationStrategy {
public:
    InterpolationStrategyType type() const override {
        return InterpolationStrategyType::RKF45;
    }

    const char* name() const override {
        return "RKF45";
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

        // RKF45 Butcher tableau coefficients
        static constexpr double a2 = 1.0/4.0;
        static constexpr double a3 = 3.0/8.0;
        static constexpr double a4 = 12.0/13.0;
        static constexpr double a5 = 1.0;
        static constexpr double a6 = 1.0/2.0;

        static constexpr double b21 = 1.0/4.0;
        static constexpr double b31 = 3.0/32.0, b32 = 9.0/32.0;
        static constexpr double b41 = 1932.0/2197.0, b42 = -7200.0/2197.0, b43 = 7296.0/2197.0;
        static constexpr double b51 = 439.0/216.0, b52 = -8.0, b53 = 3680.0/513.0, b54 = -845.0/4104.0;
        static constexpr double b61 = -8.0/27.0, b62 = 2.0, b63 = -3544.0/2565.0, b64 = 1859.0/4104.0, b65 = -11.0/40.0;

        // 4th order coefficients
        static constexpr double c1 = 25.0/216.0, c3 = 1408.0/2565.0, c4 = 2197.0/4104.0, c5 = -1.0/5.0;

        // 5th order coefficients (for error estimation)
        static constexpr double d1 = 16.0/135.0, d3 = 6656.0/12825.0, d4 = 28561.0/56430.0;
        static constexpr double d5 = -9.0/50.0, d6 = 2.0/55.0;

        // Suppress unused variable warnings for Butcher tableau coefficients
        (void)b21; (void)b31; (void)b32;
        (void)b41; (void)b42; (void)b43;
        (void)b51; (void)b52; (void)b53; (void)b54;
        (void)b61; (void)b62; (void)b63; (void)b64; (void)b65;

        double t = 0.0;
        double h = config_.maxStepSize;
        double hMin = config_.minStepSize;
        double hMax = config_.maxStepSize;
        double tolerance = config_.errorTolerance;
        double safety = config_.safetyFactor;

        double currentTime = ctx.currentTime;

        // Add start point
        TrajectoryPoint startPt;
        startPt.time = currentTime;
        startPt.position = segment.start;
        startPt.velocity = evaluateVelocity(segment, 0.0);
        startPt.parameter = 0.0;
        startPt.blockIndex = segment.blockIndex;
        startPt.segmentIndex = static_cast<int32_t>(ctx.currentSegmentIndex);
        startPt.motionType = segment.motionType;
        points.push_back(startPt);
        ctx.updateBounds(startPt.position);

        int iterations = 0;
        while (t < 1.0 && iterations < config_.maxIterations) {
            iterations++;

            // Adjust step size for final step
            if (t + h > 1.0) {
                h = 1.0 - t;
            }

            // Compute k values (derivatives at intermediate points)
            // For trajectory, we're integrating parameter t -> position
            // f(t, y) returns velocity at parameter t
            Position k1 = evaluateDerivative(segment, t);
            Position k2 = evaluateDerivative(segment, t + a2 * h);
            Position k3 = evaluateDerivative(segment, t + a3 * h);
            Position k4 = evaluateDerivative(segment, t + a4 * h);
            Position k5 = evaluateDerivative(segment, t + a5 * h);
            Position k6 = evaluateDerivative(segment, t + a6 * h);

            // 4th order solution
            Position y4;
            for (size_t i = 0; i < MAX_AXES; ++i) {
                y4[i] = c1 * k1[i] + c3 * k3[i] + c4 * k4[i] + c5 * k5[i];
            }

            // 5th order solution
            Position y5;
            for (size_t i = 0; i < MAX_AXES; ++i) {
                y5[i] = d1 * k1[i] + d3 * k3[i] + d4 * k4[i] + d5 * k5[i] + d6 * k6[i];
            }

            // Error estimate (difference between 4th and 5th order)
            double error = 0.0;
            for (size_t i = 0; i < 3; ++i) {  // Only check XYZ for geometric error
                double diff = std::fabs(y5[i] - y4[i]) * h;
                error = std::max(error, diff);
            }

            // Step acceptance and adaptation
            if (error <= tolerance || h <= hMin) {
                // Accept step
                t += h;

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
                pt.isInterpolated = true;

                points.push_back(pt);
                ctx.updateBounds(pt.position);

                result.minStepSize = std::min(result.minStepSize, h);
                result.maxStepSize = std::max(result.maxStepSize, h);
            } else {
                result.rejectedSteps++;
            }

            // Adapt step size
            if (error > InterpolationConstants::EPSILON) {
                double factor = safety * std::pow(tolerance / error, 0.2);
                factor = clamp(factor, 0.1, 5.0);
                h = clamp(h * factor, hMin, hMax);
            }

            result.totalIterations++;
        }

        // Ensure we end exactly at t=1
        if (points.empty() || points.back().parameter < 1.0 - InterpolationConstants::EPSILON) {
            TrajectoryPoint endPt;
            endPt.time = currentTime + segment.segmentTime;
            endPt.position = segment.end;
            endPt.velocity = evaluateVelocity(segment, 1.0);
            endPt.parameter = 1.0;
            endPt.blockIndex = segment.blockIndex;
            endPt.segmentIndex = static_cast<int32_t>(ctx.currentSegmentIndex);
            endPt.motionType = segment.motionType;
            points.push_back(endPt);
            ctx.updateBounds(endPt.position);
        }

        ctx.currentTime += segment.segmentTime;
        ctx.currentPosition = segment.end;

        return result;
    }

private:
    Position evaluateDerivative(const PlanningSegment& segment, double t) const {
        // Derivative of position with respect to parameter
        // For arc-length parametrization, this equals velocity normalized by segment time
        Position vel = evaluateVelocity(segment, t);
        Position deriv;
        for (size_t i = 0; i < MAX_AXES; ++i) {
            deriv[i] = vel[i] * segment.segmentTime;
        }
        return deriv;
    }
};

// Factory registration helper - defined in InterpolationStrategy.cpp
std::unique_ptr<InterpolationStrategy> createRKF45Strategy() {
    return std::make_unique<RKF45Strategy>();
}

} // namespace GCode
