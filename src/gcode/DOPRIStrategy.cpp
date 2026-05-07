/**
 * @file DOPRIStrategy.cpp
 * @brief Implementation of Dormand-Prince 5(4) Interpolation Strategy
 *
 * Adaptive step-size DOPRI method with FSAL (First Same As Last)
 * optimization for efficient error estimation.
 */

#include "gcode/motion/InterpolationStrategy.hpp"
#include <cmath>

namespace GCode {

using namespace InterpolationConstants;

// ============================================================================
// DOPRI Strategy (Dormand-Prince 5(4))
// ============================================================================

class DOPRIStrategy : public InterpolationStrategy {
public:
    InterpolationStrategyType type() const override {
        return InterpolationStrategyType::DOPRI;
    }

    const char* name() const override {
        return "DOPRI";
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

        // Dormand-Prince coefficients
        static constexpr double a2 = 1.0/5.0;
        static constexpr double a3 = 3.0/10.0;
        static constexpr double a4 = 4.0/5.0;
        static constexpr double a5 = 8.0/9.0;
        static constexpr double a6 = 1.0;
        static constexpr double a7 = 1.0;

        static constexpr double b21 = 1.0/5.0;
        static constexpr double b31 = 3.0/40.0, b32 = 9.0/40.0;
        static constexpr double b41 = 44.0/45.0, b42 = -56.0/15.0, b43 = 32.0/9.0;
        static constexpr double b51 = 19372.0/6561.0, b52 = -25360.0/2187.0, b53 = 64448.0/6561.0, b54 = -212.0/729.0;
        static constexpr double b61 = 9017.0/3168.0, b62 = -355.0/33.0, b63 = 46732.0/5247.0, b64 = 49.0/176.0, b65 = -5103.0/18656.0;
        static constexpr double b71 = 35.0/384.0, b73 = 500.0/1113.0, b74 = 125.0/192.0, b75 = -2187.0/6784.0, b76 = 11.0/84.0;

        // 5th order coefficients (used for stepping)
        static constexpr double c1 = 35.0/384.0, c3 = 500.0/1113.0, c4 = 125.0/192.0;
        static constexpr double c5 = -2187.0/6784.0, c6 = 11.0/84.0;

        // 4th order coefficients (for error estimation)
        static constexpr double d1 = 5179.0/57600.0, d3 = 7571.0/16695.0, d4 = 393.0/640.0;
        static constexpr double d5 = -92097.0/339200.0, d6 = 187.0/2100.0, d7 = 1.0/40.0;

        // Suppress unused variable warnings for Butcher tableau coefficients
        (void)b21; (void)b31; (void)b32;
        (void)b41; (void)b42; (void)b43;
        (void)b51; (void)b52; (void)b53; (void)b54;
        (void)b61; (void)b62; (void)b63; (void)b64; (void)b65;
        (void)b71; (void)b73; (void)b74; (void)b75; (void)b76;

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

        // FSAL: k1 of next step = k7 of current step
        Position k1 = evaluateDerivative(segment, 0.0);

        int iterations = 0;
        while (t < 1.0 && iterations < config_.maxIterations) {
            iterations++;

            if (t + h > 1.0) {
                h = 1.0 - t;
            }

            // Compute k values
            Position k2 = evaluateDerivative(segment, t + a2 * h);
            Position k3 = evaluateDerivative(segment, t + a3 * h);
            Position k4 = evaluateDerivative(segment, t + a4 * h);
            Position k5 = evaluateDerivative(segment, t + a5 * h);
            Position k6 = evaluateDerivative(segment, t + a6 * h);
            Position k7 = evaluateDerivative(segment, t + a7 * h);

            // Error estimate
            double error = 0.0;
            for (size_t i = 0; i < 3; ++i) {
                double y5_i = c1 * k1[i] + c3 * k3[i] + c4 * k4[i] + c5 * k5[i] + c6 * k6[i];
                double y4_i = d1 * k1[i] + d3 * k3[i] + d4 * k4[i] + d5 * k5[i] + d6 * k6[i] + d7 * k7[i];
                double diff = std::fabs(y5_i - y4_i) * h;
                error = std::max(error, diff);
            }

            if (error <= tolerance || h <= hMin) {
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

                // FSAL: reuse k7 as k1 for next step
                k1 = k7;

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

        // Ensure endpoint
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
        Position vel = evaluateVelocity(segment, t);
        Position deriv;
        for (size_t i = 0; i < MAX_AXES; ++i) {
            deriv[i] = vel[i] * segment.segmentTime;
        }
        return deriv;
    }
};

// Factory registration helper - defined in InterpolationStrategy.cpp
std::unique_ptr<InterpolationStrategy> createDOPRIStrategy() {
    return std::make_unique<DOPRIStrategy>();
}

} // namespace GCode
