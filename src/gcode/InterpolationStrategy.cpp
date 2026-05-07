/**
 * @file InterpolationStrategy.cpp
 * @brief Implementation of Interpolation Strategy Framework
 *
 * Implements:
 * - Base InterpolationStrategy methods
 * - VelocityPlanner
 * - InterpolationStrategyFactory
 *
 * Strategy implementations are in separate files:
 * - FixedTimeStrategy.cpp
 * - FixedDeviationStrategy.cpp
 * - RKF45Strategy.cpp
 * - DOPRIStrategy.cpp
 * - AdaptiveMidpointStrategy.cpp
 * - DeCasteljauStrategy.cpp
 */

#include "gcode/motion/InterpolationStrategy.hpp"
#include <cmath>
#include <stdexcept>
#include <numeric>

namespace GCode {

// Forward declarations for factory functions defined in separate strategy files
std::unique_ptr<InterpolationStrategy> createFixedTimeStrategy();
std::unique_ptr<InterpolationStrategy> createFixedDeviationStrategy();
std::unique_ptr<InterpolationStrategy> createRKF45Strategy();
std::unique_ptr<InterpolationStrategy> createDOPRIStrategy();
std::unique_ptr<InterpolationStrategy> createAdaptiveMidpointStrategy();
std::unique_ptr<InterpolationStrategy> createDeCasteljauStrategy();

using namespace InterpolationConstants;

// ============================================================================
// InterpolationStrategy Base Implementation
// ============================================================================

void InterpolationStrategy::getPlaneAxes(InterpolationPlane plane, int& u, int& v, int& w) {
    switch (plane) {
        case InterpolationPlane::XZ:
            u = 0; v = 2; w = 1;
            break;
        case InterpolationPlane::YZ:
            u = 1; v = 2; w = 0;
            break;
        case InterpolationPlane::XY:
        default:
            u = 0; v = 1; w = 2;
            break;
    }
}

size_t InterpolationStrategy::arcSegmentCount(double radius, double sweep, double maxChordError) {
    if (radius <= 0 || maxChordError <= 0) return 1;
    double x = clamp(1.0 - (maxChordError / radius), -1.0, 1.0);
    double anglePerSegment = 2.0 * std::acos(x);
    if (anglePerSegment <= 0) return 1;
    return static_cast<size_t>(std::ceil(std::fabs(sweep) / anglePerSegment));
}

Position InterpolationStrategy::evaluatePosition(const PlanningSegment& segment, double t) const {
    t = clamp(t, 0.0, 1.0);
    Position result;

    if (segment.isArc() && segment.arcRadius > 0 && segment.arcSweep != 0) {
        // Arc interpolation
        int u, v, w;
        getPlaneAxes(segment.plane, u, v, w);

        double cu = segment.center[u];
        double cv = segment.center[v];
        double su = segment.start[u];
        double sv = segment.start[v];

        double startAngle = std::atan2(sv - cv, su - cu);
        double angle = startAngle + t * segment.arcSweep;

        // Interpolate all axes linearly first
        for (size_t k = 0; k < MAX_AXES; ++k) {
            result[k] = segment.start[k] + (segment.end[k] - segment.start[k]) * t;
        }

        // Override the arc plane axes
        result[u] = cu + segment.arcRadius * std::cos(angle);
        result[v] = cv + segment.arcRadius * std::sin(angle);
    } else if (segment.isSpline() && segment.controlPoints.size() >= 2) {
        // Spline interpolation via de Casteljau (implemented in DeCasteljauStrategy)
        // For base class, fall back to linear
        for (size_t k = 0; k < MAX_AXES; ++k) {
            result[k] = segment.start[k] + (segment.end[k] - segment.start[k]) * t;
        }
    } else {
        // Linear interpolation
        for (size_t k = 0; k < MAX_AXES; ++k) {
            result[k] = segment.start[k] + (segment.end[k] - segment.start[k]) * t;
        }
    }

    return result;
}

Position InterpolationStrategy::evaluateVelocity(const PlanningSegment& segment, double t) const {
    t = clamp(t, 0.0, 1.0);
    Position result;

    if (segment.segmentTime <= 0) {
        return result;  // Zero velocity
    }

    if (segment.isArc() && segment.arcRadius > 0 && segment.arcSweep != 0) {
        int u, v, w;
        getPlaneAxes(segment.plane, u, v, w);

        double cu = segment.center[u];
        double cv = segment.center[v];
        double su = segment.start[u];
        double sv = segment.start[v];

        double startAngle = std::atan2(sv - cv, su - cu);
        double angle = startAngle + t * segment.arcSweep;
        double omega = segment.arcSweep / segment.segmentTime;  // Angular velocity

        // Linear velocity for non-arc axes
        for (size_t k = 0; k < MAX_AXES; ++k) {
            result[k] = (segment.end[k] - segment.start[k]) / segment.segmentTime;
        }

        // Arc velocity (tangent to circle)
        result[u] = -segment.arcRadius * std::sin(angle) * omega;
        result[v] = segment.arcRadius * std::cos(angle) * omega;
    } else {
        // Linear: constant velocity
        for (size_t k = 0; k < MAX_AXES; ++k) {
            result[k] = (segment.end[k] - segment.start[k]) / segment.segmentTime;
        }
    }

    return result;
}

Position InterpolationStrategy::evaluateAcceleration(const PlanningSegment& segment, double t) const {
    t = clamp(t, 0.0, 1.0);
    Position result;

    if (segment.segmentTime <= 0) {
        return result;
    }

    if (segment.isArc() && segment.arcRadius > 0 && segment.arcSweep != 0) {
        int u, v, w;
        getPlaneAxes(segment.plane, u, v, w);

        double cu = segment.center[u];
        double cv = segment.center[v];
        double su = segment.start[u];
        double sv = segment.start[v];

        double startAngle = std::atan2(sv - cv, su - cu);
        double angle = startAngle + t * segment.arcSweep;
        double omega = segment.arcSweep / segment.segmentTime;

        // Centripetal acceleration (toward center)
        double accel = segment.arcRadius * omega * omega;
        result[u] = -accel * std::cos(angle);
        result[v] = -accel * std::sin(angle);
    }
    // Linear segments have zero acceleration (constant velocity)

    return result;
}

double InterpolationStrategy::evaluateCurvature(const PlanningSegment& segment, double /*t*/) const {
    if (segment.isArc() && segment.arcRadius > InterpolationConstants::EPSILON) {
        return 1.0 / segment.arcRadius;
    }
    return 0.0;  // Lines have zero curvature
}

double InterpolationStrategy::arcLength(const PlanningSegment& segment, double t) const {
    t = clamp(t, 0.0, 1.0);
    return t * segment.segmentLength;
}

double InterpolationStrategy::arcLengthInverse(const PlanningSegment& segment, double s) const {
    if (segment.segmentLength <= InterpolationConstants::EPSILON) return 0.0;
    return clamp(s / segment.segmentLength, 0.0, 1.0);
}

InterpolationResult InterpolationStrategy::interpolateAll(
    InterpolationContext& ctx,
    std::vector<TrajectoryPoint>& points
) {
    InterpolationResult result;
    result.success = true;
    points.clear();

    for (size_t i = 0; i < ctx.segments.size(); ++i) {
        ctx.currentSegmentIndex = i;
        std::vector<TrajectoryPoint> segmentPoints;

        auto segResult = interpolateSegment(ctx.segments[i], ctx, segmentPoints);
        if (!segResult.success) {
            result.success = false;
            result.errorMessage = segResult.errorMessage;
            return result;
        }

        // Append points (skip first if not first segment to avoid duplicates)
        size_t startIdx = (i > 0 && !points.empty()) ? 1 : 0;
        for (size_t j = startIdx; j < segmentPoints.size(); ++j) {
            points.push_back(segmentPoints[j]);
        }

        result.totalIterations += segResult.totalIterations;
        result.rejectedSteps += segResult.rejectedSteps;
        result.minStepSize = std::min(result.minStepSize, segResult.minStepSize);
        result.maxStepSize = std::max(result.maxStepSize, segResult.maxStepSize);
    }

    if (!points.empty()) {
        result.totalDuration = points.back().time;
        result.minBounds = ctx.minBounds;
        result.maxBounds = ctx.maxBounds;
    }

    return result;
}

// ============================================================================
// Velocity Planner Implementation
// ============================================================================

VelocityPlanner::VelocityPlanner(const InterpolationConfig& config)
    : config_(config) {}

void VelocityPlanner::plan(std::vector<PlanningSegment>& segments) {
    if (segments.empty()) return;

    // Initialize velocities based on feed rates
    for (auto& seg : segments) {
        seg.maxVelocity = seg.feedRate / 60.0;  // Convert mm/min to mm/s

        if (seg.isRapid) {
            seg.maxVelocity = config_.limits.maxVelocityLinear / 60.0;
        }

        // Apply curvature limit
        if (seg.isArc() && seg.arcRadius > InterpolationConstants::EPSILON) {
            double curvatureLimit = maxVelocityForCurvature(1.0 / seg.arcRadius);
            seg.maxVelocity = std::min(seg.maxVelocity, curvatureLimit);
        }

        seg.entryVelocity = seg.maxVelocity;
        seg.exitVelocity = seg.maxVelocity;
    }

    // Apply path mode constraints
    if (config_.pathMode == PathControlMode::ExactStop) {
        // G61: Full stop at every endpoint
        for (auto& seg : segments) {
            seg.entryVelocity = 0.0;
            seg.exitVelocity = 0.0;
        }
    } else {
        // Calculate corner velocities
        for (size_t i = 0; i < segments.size() - 1; ++i) {
            double cornerVel = calculateCornerVelocity(segments[i], segments[i + 1]);
            segments[i].exitVelocity = std::min(segments[i].exitVelocity, cornerVel);
            segments[i + 1].entryVelocity = std::min(segments[i + 1].entryVelocity, cornerVel);
        }

        // First segment starts from rest (or previous velocity if known)
        segments.front().entryVelocity = 0.0;

        // Last segment ends at rest
        segments.back().exitVelocity = 0.0;
    }

    // Backward pass: limit by deceleration capability
    backwardPass(segments);

    // Forward pass: limit by acceleration capability
    forwardPass(segments);
}

double VelocityPlanner::calculateCornerVelocity(
    const PlanningSegment& prev,
    const PlanningSegment& next
) const {
    if (config_.pathMode == PathControlMode::ExactStop) {
        return 0.0;
    }

    double angle = cornerAngle(prev, next);

    if (config_.pathMode == PathControlMode::ExactPath) {
        // G61.1: Zero velocity if direction changes
        if (angle > PI / 180.0) {  // More than 1 degree
            return 0.0;
        }
        return std::min(prev.maxVelocity, next.maxVelocity);
    }

    // G64: Path blending
    if (angle < InterpolationConstants::EPSILON) {
        // Collinear - full speed
        return std::min(prev.maxVelocity, next.maxVelocity);
    }

    // Calculate blend radius for given tolerance
    double tolerance = config_.blendTolerance;
    if (tolerance <= 0) tolerance = 0.01;

    // Corner radius from tolerance: r = tolerance / (1 - cos(angle/2))
    double halfAngle = angle / 2.0;
    double cosHalf = std::cos(halfAngle);
    double blendRadius = tolerance / (1.0 - cosHalf + InterpolationConstants::EPSILON);

    // Centripetal acceleration limit
    double maxCentripetal = config_.limits.maxCentripetalAccel;
    double cornerVel = std::sqrt(maxCentripetal * blendRadius);

    return std::min({cornerVel, prev.maxVelocity, next.maxVelocity});
}

double VelocityPlanner::maxVelocityForCurvature(double curvature) const {
    if (curvature <= InterpolationConstants::EPSILON) {
        return config_.limits.maxVelocityLinear / 60.0;
    }

    double radius = 1.0 / curvature;
    double maxCentripetal = config_.limits.maxCentripetalAccel;

    return std::sqrt(maxCentripetal * radius);
}

double VelocityPlanner::reachableVelocity(
    double startVelocity,
    double distance,
    double acceleration
) const {
    // v² = v₀² + 2as
    double vSquared = startVelocity * startVelocity + 2.0 * acceleration * distance;
    return (vSquared > 0) ? std::sqrt(vSquared) : 0.0;
}

void VelocityPlanner::forwardPass(std::vector<PlanningSegment>& segments) {
    double accel = config_.limits.maxAcceleration;

    for (size_t i = 0; i < segments.size(); ++i) {
        auto& seg = segments[i];

        if (i > 0) {
            seg.entryVelocity = std::min(seg.entryVelocity, segments[i - 1].exitVelocity);
        }

        double reachable = reachableVelocity(seg.entryVelocity, seg.segmentLength, accel);
        seg.exitVelocity = std::min(seg.exitVelocity, reachable);
    }
}

void VelocityPlanner::backwardPass(std::vector<PlanningSegment>& segments) {
    double decel = config_.limits.maxDeceleration;

    for (int i = static_cast<int>(segments.size()) - 1; i >= 0; --i) {
        auto& seg = segments[i];

        if (i < static_cast<int>(segments.size()) - 1) {
            seg.exitVelocity = std::min(seg.exitVelocity, segments[i + 1].entryVelocity);
        }

        double reachable = reachableVelocity(seg.exitVelocity, seg.segmentLength, decel);
        seg.entryVelocity = std::min(seg.entryVelocity, reachable);
    }
}

double VelocityPlanner::cornerAngle(const PlanningSegment& prev, const PlanningSegment& next) const {
    // Calculate tangent directions at junction
    Position prevTangent;
    Position nextTangent;

    // Previous segment exit tangent
    if (prev.isArc()) {
        int u, v, w;
        InterpolationStrategy::getPlaneAxes(prev.plane, u, v, w);

        double cu = prev.center[u];
        double cv = prev.center[v];
        double eu = prev.end[u];
        double ev = prev.end[v];

        double angle = std::atan2(ev - cv, eu - cu);
        int dir = prev.arcDirection();

        // Tangent is perpendicular to radius
        prevTangent[u] = -dir * std::sin(angle);
        prevTangent[v] = dir * std::cos(angle);
        prevTangent[w] = (prev.end[w] - prev.start[w]) / (prev.segmentLength + InterpolationConstants::EPSILON);
    } else {
        for (size_t k = 0; k < MAX_AXES; ++k) {
            prevTangent[k] = (prev.end[k] - prev.start[k]) / (prev.segmentLength + InterpolationConstants::EPSILON);
        }
    }

    // Next segment entry tangent
    if (next.isArc()) {
        int u, v, w;
        InterpolationStrategy::getPlaneAxes(next.plane, u, v, w);

        double cu = next.center[u];
        double cv = next.center[v];
        double su = next.start[u];
        double sv = next.start[v];

        double angle = std::atan2(sv - cv, su - cu);
        int dir = next.arcDirection();

        nextTangent[u] = -dir * std::sin(angle);
        nextTangent[v] = dir * std::cos(angle);
        nextTangent[w] = (next.end[w] - next.start[w]) / (next.segmentLength + InterpolationConstants::EPSILON);
    } else {
        for (size_t k = 0; k < MAX_AXES; ++k) {
            nextTangent[k] = (next.end[k] - next.start[k]) / (next.segmentLength + InterpolationConstants::EPSILON);
        }
    }

    // Normalize
    double prevMag = std::sqrt(prevTangent[0]*prevTangent[0] +
                               prevTangent[1]*prevTangent[1] +
                               prevTangent[2]*prevTangent[2]);
    double nextMag = std::sqrt(nextTangent[0]*nextTangent[0] +
                               nextTangent[1]*nextTangent[1] +
                               nextTangent[2]*nextTangent[2]);

    if (prevMag < InterpolationConstants::EPSILON || nextMag < InterpolationConstants::EPSILON) {
        return 0.0;
    }

    for (size_t k = 0; k < 3; ++k) {
        prevTangent[k] /= prevMag;
        nextTangent[k] /= nextMag;
    }

    // Angle between tangents
    double dot = prevTangent[0]*nextTangent[0] +
                 prevTangent[1]*nextTangent[1] +
                 prevTangent[2]*nextTangent[2];

    dot = InterpolationStrategy::clamp(dot, -1.0, 1.0);

    return std::acos(dot);
}

// ============================================================================
// Factory Implementation
// ============================================================================

std::unique_ptr<InterpolationStrategy> InterpolationStrategyFactory::create(
    InterpolationStrategyType type
) {
    switch (type) {
        case InterpolationStrategyType::FixedTime:
            return createFixedTimeStrategy();
        case InterpolationStrategyType::FixedDeviation:
            return createFixedDeviationStrategy();
        case InterpolationStrategyType::RKF45:
            return createRKF45Strategy();
        case InterpolationStrategyType::DOPRI:
            return createDOPRIStrategy();
        case InterpolationStrategyType::AdaptiveMidpoint:
            return createAdaptiveMidpointStrategy();
        case InterpolationStrategyType::DeCasteljau:
            return createDeCasteljauStrategy();
        default:
            return createFixedDeviationStrategy();
    }
}

std::unique_ptr<InterpolationStrategy> InterpolationStrategyFactory::create(
    const InterpolationConfig& config
) {
    auto strategy = create(config.strategy);
    strategy->configure(config);
    return strategy;
}

} // namespace GCode
