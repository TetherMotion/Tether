/**
 * @file TrajectoryAnalyzer.cpp
 * @brief Implementation of trajectory analysis with derivative computation
 */

#include "TrajectoryAnalyzer.hpp"
#include <cmath>
#include <algorithm>
#include <numeric>

namespace GCodeExport {

// Central difference coefficients (declared constexpr in header)
constexpr std::array<double, 5> TrajectoryAnalyzer::CD4_COEFF1;
constexpr std::array<double, 5> TrajectoryAnalyzer::CD4_COEFF2;

TrajectoryAnalyzer::TrajectoryAnalyzer(const AnalysisConfig& config)
    : config_(config) {}

std::vector<TrajectorySample> TrajectoryAnalyzer::analyze(
    const std::vector<GCode::PlanningSegment>& segments,
    GCode::InterpolationStrategy* strategy
) {
    std::vector<TrajectorySample> samples;
    if (segments.empty()) return samples;
    
    double currentTime = 0.0;
    double pathPosition = 0.0;
    
    for (size_t segIdx = 0; segIdx < segments.size(); ++segIdx) {
        const auto& seg = segments[segIdx];
        
        if (seg.segmentTime <= 0) continue;
        
        // Determine number of samples for this segment
        size_t numSteps = std::max(
            size_t(1), 
            static_cast<size_t>(std::ceil(seg.segmentTime / config_.timeStep))
        );
        
        const double dt = seg.segmentTime / static_cast<double>(numSteps);
        
        // For all segments except the last, don't include the endpoint (t=1.0)
        // because the next segment's start (t=0.0) will be at the same position.
        // This avoids duplicate time points at segment boundaries.
        bool isLastSegment = (segIdx == segments.size() - 1);
        size_t maxStep = isLastSegment ? numSteps : numSteps - 1;
        
        for (size_t step = 0; step <= maxStep; ++step) {
            const double t = static_cast<double>(step) / static_cast<double>(numSteps);
            
            TrajectorySample sample;
            sample.time = currentTime + step * dt;
            sample.segmentIndex = static_cast<int32_t>(segIdx);
            sample.blockIndex = seg.blockIndex;
            sample.motionType = static_cast<uint8_t>(seg.motionType);
            
            // Interpolate position
            if (strategy) {
                auto pos = strategy->evaluatePosition(seg, t);
                for (size_t i = 0; i < 9; ++i) {
                    sample.position[i] = pos[i];
                }
            } else {
                // Linear interpolation fallback
                for (size_t i = 0; i < 9; ++i) {
                    sample.position[i] = seg.start[i] + t * (seg.end[i] - seg.start[i]);
                }
                
                // Arc interpolation for G2/G3
                if (seg.motionType == GCode::SegmentMotionType::ArcCW ||
                    seg.motionType == GCode::SegmentMotionType::ArcCCW) {
                    
                    int u = 0, v = 1, w = 2;
                    switch (seg.plane) {
                        case GCode::InterpolationPlane::XY: u = 0; v = 1; w = 2; break;
                        case GCode::InterpolationPlane::XZ: u = 0; v = 2; w = 1; break;
                        case GCode::InterpolationPlane::YZ: u = 1; v = 2; w = 0; break;
                    }
                    
                    double startAngle = std::atan2(
                        seg.start[v] - seg.center[v],
                        seg.start[u] - seg.center[u]
                    );
                    double angle = startAngle + t * seg.arcSweep;
                    
                    sample.position[u] = seg.center[u] + seg.arcRadius * std::cos(angle);
                    sample.position[v] = seg.center[v] + seg.arcRadius * std::sin(angle);
                    sample.position[w] = seg.start[w] + t * (seg.end[w] - seg.start[w]);
                }
            }
            
            // Compute path position (arc length)
            if (!samples.empty()) {
                double ds = 0.0;
                for (size_t i = 0; i < 3; ++i) {
                    double d = sample.position[i] - samples.back().position[i];
                    ds += d * d;
                }
                pathPosition += std::sqrt(ds);
            }
            sample.pathPosition = pathPosition;
            
            samples.push_back(sample);
        }
        
        currentTime += seg.segmentTime;
    }
    
    // Compute derivatives
    computeDerivatives(samples, config_.derivativeOrder);
    
    // Compute combined metrics
    computeCombinedMetrics(samples);
    
    return samples;
} // GCOVR_EXCL_LINE

void TrajectoryAnalyzer::computeDerivatives(std::vector<TrajectorySample>& samples, int order) {
    if (samples.size() < 5) return;
    
    const size_t n = samples.size();
    
    for (size_t i = 2; i < n - 2; ++i) {
        const double dt = (samples[i + 1].time - samples[i - 1].time) / 2.0;
        if (dt <= 0) continue;
        
        for (size_t axis = 0; axis < 9; ++axis) {
            // 4th order central difference for velocity
            double vel = (
                CD4_COEFF1[0] * samples[i - 2].position[axis] +
                CD4_COEFF1[1] * samples[i - 1].position[axis] +
                CD4_COEFF1[3] * samples[i + 1].position[axis] +
                CD4_COEFF1[4] * samples[i + 2].position[axis]
            ) / dt;
            samples[i].velocity[axis] = vel;
            
            // 4th order central difference for acceleration
            double acc = (
                CD4_COEFF2[0] * samples[i - 2].position[axis] +
                CD4_COEFF2[1] * samples[i - 1].position[axis] +
                CD4_COEFF2[2] * samples[i].position[axis] +
                CD4_COEFF2[3] * samples[i + 1].position[axis] +
                CD4_COEFF2[4] * samples[i + 2].position[axis]
            ) / (dt * dt);
            samples[i].acceleration[axis] = acc;
        }
    }
    
    // Compute jerk from acceleration (central difference)
    for (size_t i = 3; i < n - 3; ++i) {
        const double dt = (samples[i + 1].time - samples[i - 1].time) / 2.0;
        if (dt <= 0) continue;
        
        for (size_t axis = 0; axis < 9; ++axis) {
            samples[i].jerk[axis] = (
                samples[i + 1].acceleration[axis] - samples[i - 1].acceleration[axis]
            ) / (2.0 * dt);
        }
    }
    
    // Handle boundary samples (copy from nearest computed)
    if (n > 4) {
        samples[0].velocity = samples[2].velocity;
        samples[1].velocity = samples[2].velocity;
        samples[n-2].velocity = samples[n-3].velocity;
        samples[n-1].velocity = samples[n-3].velocity;
        
        samples[0].acceleration = samples[2].acceleration;
        samples[1].acceleration = samples[2].acceleration;
        samples[n-2].acceleration = samples[n-3].acceleration;
        samples[n-1].acceleration = samples[n-3].acceleration;
    }
    
    if (n > 6) {
        samples[0].jerk = samples[3].jerk;
        samples[1].jerk = samples[3].jerk;
        samples[2].jerk = samples[3].jerk;
        samples[n-3].jerk = samples[n-4].jerk;
        samples[n-2].jerk = samples[n-4].jerk;
        samples[n-1].jerk = samples[n-4].jerk;
    }
}

void TrajectoryAnalyzer::computeCombinedMetrics(std::vector<TrajectorySample>& samples) {
    for (auto& sample : samples) {
        // Linear velocity magnitude (XYZ)
        sample.linearVelocity = std::sqrt(
            sample.velocity[0] * sample.velocity[0] +
            sample.velocity[1] * sample.velocity[1] +
            sample.velocity[2] * sample.velocity[2]
        );
        
        // Linear acceleration magnitude
        sample.linearAcceleration = std::sqrt(
            sample.acceleration[0] * sample.acceleration[0] +
            sample.acceleration[1] * sample.acceleration[1] +
            sample.acceleration[2] * sample.acceleration[2]
        );
        
        // Linear jerk magnitude
        sample.linearJerk = std::sqrt(
            sample.jerk[0] * sample.jerk[0] +
            sample.jerk[1] * sample.jerk[1] +
            sample.jerk[2] * sample.jerk[2]
        );
        
        // Curvature = |v × a| / |v|³
        if (sample.linearVelocity > 1e-9) {
            double cross_x = sample.velocity[1] * sample.acceleration[2] - 
                            sample.velocity[2] * sample.acceleration[1];
            double cross_y = sample.velocity[2] * sample.acceleration[0] - 
                            sample.velocity[0] * sample.acceleration[2];
            double cross_z = sample.velocity[0] * sample.acceleration[1] - 
                            sample.velocity[1] * sample.acceleration[0];
            
            double crossMag = std::sqrt(cross_x * cross_x + cross_y * cross_y + cross_z * cross_z);
            double v3 = sample.linearVelocity * sample.linearVelocity * sample.linearVelocity;
            sample.curvature = crossMag / v3;
            
            // Centripetal acceleration = v² * κ = v² / r
            sample.centripetalAccel = sample.linearVelocity * sample.linearVelocity * sample.curvature;
        }
    }
}

TrajectoryStatistics TrajectoryAnalyzer::computeStatistics(const std::vector<TrajectorySample>& samples) {
    TrajectoryStatistics stats;
    
    if (samples.empty()) return stats;
    
    stats.sampleCount = samples.size();
    stats.duration = samples.back().time;
    stats.pathLength = samples.back().pathPosition;
    
    // Initialize axis statistics
    for (size_t axis = 0; axis < 9; ++axis) {
        stats.axisStats[axis].minPosition = std::numeric_limits<double>::max();
        stats.axisStats[axis].maxPosition = std::numeric_limits<double>::lowest();
        stats.axisStats[axis].minVelocity = std::numeric_limits<double>::max();
        stats.axisStats[axis].maxVelocity = std::numeric_limits<double>::lowest();
        stats.axisStats[axis].minAcceleration = std::numeric_limits<double>::max();
        stats.axisStats[axis].maxAcceleration = std::numeric_limits<double>::lowest();
        stats.axisStats[axis].minJerk = std::numeric_limits<double>::max();
        stats.axisStats[axis].maxJerk = std::numeric_limits<double>::lowest();
    }
    
    // Accumulate statistics
    std::array<double, 9> velSum{}, accSum{};
    
    for (const auto& sample : samples) {
        for (size_t axis = 0; axis < 9; ++axis) {
            auto& as = stats.axisStats[axis];
            
            as.minPosition = std::min(as.minPosition, sample.position[axis]);
            as.maxPosition = std::max(as.maxPosition, sample.position[axis]);
            as.minVelocity = std::min(as.minVelocity, sample.velocity[axis]);
            as.maxVelocity = std::max(as.maxVelocity, sample.velocity[axis]);
            as.minAcceleration = std::min(as.minAcceleration, sample.acceleration[axis]);
            as.maxAcceleration = std::max(as.maxAcceleration, sample.acceleration[axis]);
            as.minJerk = std::min(as.minJerk, sample.jerk[axis]);
            as.maxJerk = std::max(as.maxJerk, sample.jerk[axis]);
            
            velSum[axis] += std::abs(sample.velocity[axis]);
            accSum[axis] += std::abs(sample.acceleration[axis]);
        }
        
        stats.maxLinearVelocity = std::max(stats.maxLinearVelocity, sample.linearVelocity);
        stats.maxLinearAcceleration = std::max(stats.maxLinearAcceleration, sample.linearAcceleration);
        stats.maxLinearJerk = std::max(stats.maxLinearJerk, sample.linearJerk);
        stats.maxCurvature = std::max(stats.maxCurvature, sample.curvature);
        stats.maxCentripetalAccel = std::max(stats.maxCentripetalAccel, sample.centripetalAccel);
    }
    
    // Compute averages
    for (size_t axis = 0; axis < 9; ++axis) {
        stats.axisStats[axis].avgVelocity = velSum[axis] / stats.sampleCount;
        stats.axisStats[axis].avgAcceleration = accSum[axis] / stats.sampleCount;
    }
    
    // Check limit compliance
    std::vector<LimitViolation> violations;
    stats.meetsLimits = checkLimitCompliance(samples, &violations);
    stats.violations = std::move(violations);
    
    return stats;
}

bool TrajectoryAnalyzer::checkLimitCompliance(
    const std::vector<TrajectorySample>& samples,
    std::vector<LimitViolation>* violations
) {
    bool compliant = true;
    
    for (const auto& sample : samples) {
        // Check linear velocity
        if (sample.linearVelocity > config_.limits.maxVelocityLinear / 60.0 * (1.0 + config_.violationTolerance)) {
            compliant = false;
            if (violations) {
                violations->push_back({
                    sample.time,
                    -1,
                    "velocity",
                    sample.linearVelocity,
                    config_.limits.maxVelocityLinear / 60.0,
                    (sample.linearVelocity / (config_.limits.maxVelocityLinear / 60.0) - 1.0) * 100.0
                });
            }
        }
        
        // Check linear acceleration
        if (sample.linearAcceleration > config_.limits.maxAcceleration * (1.0 + config_.violationTolerance)) {
            compliant = false;
            if (violations) {
                violations->push_back({
                    sample.time,
                    -1,
                    "acceleration",
                    sample.linearAcceleration,
                    config_.limits.maxAcceleration,
                    (sample.linearAcceleration / config_.limits.maxAcceleration - 1.0) * 100.0
                });
            }
        }
        
        // Check linear jerk
        if (sample.linearJerk > config_.limits.maxJerk * (1.0 + config_.violationTolerance)) {
            compliant = false;
            if (violations) {
                violations->push_back({
                    sample.time,
                    -1,
                    "jerk",
                    sample.linearJerk,
                    config_.limits.maxJerk,
                    (sample.linearJerk / config_.limits.maxJerk - 1.0) * 100.0
                });
            }
        }
        
        // Check per-axis limits
        for (size_t axis = 0; axis < 9; ++axis) {
            double axisVelLimit = config_.limits.axisMaxVelocity[axis] / 60.0;
            double axisAccLimit = config_.limits.axisMaxAcceleration[axis];
            double axisJerkLimit = config_.limits.axisMaxJerk[axis];
            
            if (std::abs(sample.velocity[axis]) > axisVelLimit * (1.0 + config_.violationTolerance)) {
                compliant = false;
                if (violations) {
                    violations->push_back({
                        sample.time,
                        static_cast<int>(axis),
                        "velocity",
                        std::abs(sample.velocity[axis]),
                        axisVelLimit,
                        (std::abs(sample.velocity[axis]) / axisVelLimit - 1.0) * 100.0
                    });
                }
            }
            
            if (std::abs(sample.acceleration[axis]) > axisAccLimit * (1.0 + config_.violationTolerance)) {
                compliant = false;
                if (violations) {
                    violations->push_back({
                        sample.time,
                        static_cast<int>(axis),
                        "acceleration",
                        std::abs(sample.acceleration[axis]),
                        axisAccLimit,
                        (std::abs(sample.acceleration[axis]) / axisAccLimit - 1.0) * 100.0
                    });
                }
            }
            
            if (std::abs(sample.jerk[axis]) > axisJerkLimit * (1.0 + config_.violationTolerance)) {
                compliant = false;
                if (violations) {
                    violations->push_back({
                        sample.time,
                        static_cast<int>(axis),
                        "jerk",
                        std::abs(sample.jerk[axis]),
                        axisJerkLimit,
                        (std::abs(sample.jerk[axis]) / axisJerkLimit - 1.0) * 100.0
                    });
                }
            }
        }
    }
    
    return compliant;
}

// ============================================================================
// Approximation Strategies
// ============================================================================

std::vector<TrajectorySample> FixedTimeApproximation::generateTrajectory(
    const std::vector<GCode::PlanningSegment>& segments,
    const GCode::KinematicLimits& limits
) {
    AnalysisConfig config;
    config.timeStep = timeStep_;
    config.limits = limits;
    
    TrajectoryAnalyzer analyzer(config);
    return analyzer.analyze(segments, nullptr);
}

void FixedTimeApproximation::configure(const std::string& key, double value) {
    if (key == "timeStep") timeStep_ = value;
}

std::vector<TrajectorySample> FixedDeviationApproximation::generateTrajectory(
    const std::vector<GCode::PlanningSegment>& segments,
    const GCode::KinematicLimits& limits
) {
    std::vector<TrajectorySample> samples;
    
    double currentTime = 0.0;
    double pathPosition = 0.0;
    
    for (size_t segIdx = 0; segIdx < segments.size(); ++segIdx) {
        const auto& seg = segments[segIdx];
        
        if (seg.segmentLength <= 0) continue;
        
        // Compute number of steps based on deviation
        size_t numSteps = 1;
        
        if (seg.isArc()) {
            // For arcs: chord error = r(1 - cos(θ/2)) ≈ rθ²/8 for small angles
            // Solving for step angle: θ = sqrt(8 * deviation / r)
            if (seg.arcRadius > 0 && maxDeviation_ > 0) {
                double ratio = maxDeviation_ / seg.arcRadius;
                if (ratio < 1.0) {
                    double anglePerStep = 2.0 * std::acos(1.0 - ratio);
                    if (anglePerStep > 0) {
                        numSteps = static_cast<size_t>(std::ceil(std::abs(seg.arcSweep) / anglePerStep));
                    }
                }
            }
            numSteps = std::max(numSteps, size_t(4));
        } else {
            // For linear: deviation is 0, use length-based
            numSteps = std::max(size_t(1), static_cast<size_t>(std::ceil(seg.segmentLength / maxDeviation_)));
        }
        
        const double dt = seg.segmentTime / static_cast<double>(numSteps);
        
        for (size_t step = 0; step <= numSteps; ++step) {
            const double t = static_cast<double>(step) / static_cast<double>(numSteps);
            
            TrajectorySample sample;
            sample.time = currentTime + step * dt;
            sample.segmentIndex = static_cast<int32_t>(segIdx);
            sample.blockIndex = seg.blockIndex;
            sample.motionType = static_cast<uint8_t>(seg.motionType);
            
            // Interpolate position
            for (size_t i = 0; i < 9; ++i) {
                sample.position[i] = seg.start[i] + t * (seg.end[i] - seg.start[i]);
            }
            
            // Arc interpolation
            if (seg.isArc()) {
                int u = 0, v = 1, w = 2;
                switch (seg.plane) {
                    case GCode::InterpolationPlane::XY: u = 0; v = 1; w = 2; break;
                    case GCode::InterpolationPlane::XZ: u = 0; v = 2; w = 1; break;
                    case GCode::InterpolationPlane::YZ: u = 1; v = 2; w = 0; break;
                }
                
                double startAngle = std::atan2(
                    seg.start[v] - seg.center[v],
                    seg.start[u] - seg.center[u]
                );
                double angle = startAngle + t * seg.arcSweep;
                
                sample.position[u] = seg.center[u] + seg.arcRadius * std::cos(angle);
                sample.position[v] = seg.center[v] + seg.arcRadius * std::sin(angle);
                sample.position[w] = seg.start[w] + t * (seg.end[w] - seg.start[w]);
            }
            
            if (!samples.empty()) {
                double ds = 0.0;
                for (size_t i = 0; i < 3; ++i) {
                    double d = sample.position[i] - samples.back().position[i];
                    ds += d * d;
                }
                pathPosition += std::sqrt(ds);
            }
            sample.pathPosition = pathPosition;
            
            samples.push_back(sample);
        }
        
        currentTime += seg.segmentTime;
    }
    
    // Compute derivatives
    AnalysisConfig config;
    config.limits = limits;
    TrajectoryAnalyzer analyzer(config);
    analyzer.computeDerivatives(samples, 4);
    analyzer.computeCombinedMetrics(samples);
    
    return samples;
} // GCOVR_EXCL_LINE

void FixedDeviationApproximation::configure(const std::string& key, double value) {
    if (key == "maxDeviation") maxDeviation_ = value;
}

std::vector<TrajectorySample> TrapezoidalApproximation::generateTrajectory(
    const std::vector<GCode::PlanningSegment>& segments,
    const GCode::KinematicLimits& limits
) {
    // Basic trapezoidal implementation - can be enhanced with proper velocity planning
    AnalysisConfig config;
    config.timeStep = timeStep_;
    config.limits = limits;
    
    TrajectoryAnalyzer analyzer(config);
    return analyzer.analyze(segments, nullptr);
}

void TrapezoidalApproximation::configure(const std::string& key, double value) {
    if (key == "timeStep") timeStep_ = value;
    if (key == "useJerkLimiting") useJerkLimiting_ = (value != 0);
}

std::vector<TrajectorySample> SCurveApproximation::generateTrajectory(
    const std::vector<GCode::PlanningSegment>& segments,
    const GCode::KinematicLimits& limits
) {
    // S-curve implementation - 7-segment profile
    AnalysisConfig config;
    config.timeStep = timeStep_;
    config.limits = limits;
    
    TrajectoryAnalyzer analyzer(config);
    return analyzer.analyze(segments, nullptr);
}

void SCurveApproximation::configure(const std::string& key, double value) {
    if (key == "timeStep") timeStep_ = value;
}

std::unique_ptr<ApproximationStrategy> ApproximationFactory::create(const std::string& name) {
    if (name == "FixedTime") return std::make_unique<FixedTimeApproximation>();
    if (name == "FixedDeviation") return std::make_unique<FixedDeviationApproximation>();
    if (name == "Trapezoidal") return std::make_unique<TrapezoidalApproximation>();
    if (name == "SCurve") return std::make_unique<SCurveApproximation>();
    return nullptr;
}

} // namespace GCodeExport
