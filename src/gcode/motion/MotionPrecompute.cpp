/**
 * @file MotionPrecompute.cpp
 * @brief Motion Precomputation Framework Implementation
 */

#include "gcode/motion/MotionPrecompute.hpp"
#include <cmath>
#include <sstream>
#include <iomanip>

namespace GCode {
namespace Motion {

using namespace Math;

// ============================================================================
// PrecomputeStats Implementation
// ============================================================================

std::string PrecomputeStats::summary() const {
    std::ostringstream ss;
    ss << std::fixed << std::setprecision(2);
    ss << "Precomputation Statistics:\n";
    ss << "  Total time:        " << totalTime.count() / 1000.0 << " ms\n";
    ss << "  Parse time:        " << parseTime.count() / 1000.0 << " ms\n";
    ss << "  Interpolation:     " << interpolationTime.count() / 1000.0 << " ms\n";
    ss << "  Post-processing:   " << postProcessTime.count() / 1000.0 << " ms\n";
    ss << "  Input segments:    " << inputSegments << " (" << linearSegments << " linear, " << arcSegments << " arc)\n";
    ss << "  Output points:     " << outputPoints << "\n";
    ss << "  Points/second:     " << std::setprecision(0) << pointsPerSecond() << "\n";
    return ss.str();
}

// ============================================================================
// FixedTimeStrategy Implementation
// ============================================================================

void FixedTimeStrategy::interpolateLinear(
    const Vec3& start,
    const Vec3& end,
    double feedRate,
    std::vector<TrajectoryPoint>& output
) {
    Vec3 delta = end - start;
    double distance = delta.length();
    double velocity = feedRate / 60.0;  // mm/s
    double duration = (velocity > EPSILON) ? distance / velocity : 0.0;
    
    if (duration < EPSILON) {
        TrajectoryPoint pt;
        pt.position = end;
        pt.feedRate = feedRate;
        pt.distanceFromStart = distance;
        output.push_back(pt);
        return;
    }
    
    int numSteps = std::max(1, static_cast<int>(std::ceil(duration / timeStep_)));
    // double dt = duration / numSteps; // Not used
    
    for (int i = 0; i <= numSteps; ++i) {
        double t = static_cast<double>(i) / numSteps;
        TrajectoryPoint pt;
        pt.position = start + delta * t;
        pt.feedRate = feedRate;
        pt.distanceFromStart = distance * t;
        pt.isRapid = false;
        pt.isArc = false;
        output.push_back(pt);
    }
}

void FixedTimeStrategy::interpolateArc(
    const Vec3& start,
    const Vec3& end,
    const Vec3& center,
    double radius,
    double sweep,
    int plane,
    double feedRate,
    std::vector<TrajectoryPoint>& output
) {
    double arcLen = Math::arcLength(radius, sweep);
    double velocity = feedRate / 60.0;  // mm/s
    double duration = (velocity > EPSILON) ? arcLen / velocity : 0.0;
    
    // Get plane axes - clamp to valid range
    int plane_idx = (plane >= 0 && plane <= 2) ? plane : 0;
    int u, v, w;
    switch (plane_idx) {
        case 1: u = 0; v = 2; w = 1; break;  // XZ
        case 2: u = 1; v = 2; w = 0; break;  // YZ
        default: u = 0; v = 1; w = 2; break; // XY
    }
    
    // Use array indexing instead of pointer arithmetic
    const double center_coords[3] = {center.x, center.y, center.z};
    const double start_coords[3] = {start.x, start.y, start.z};
    const double end_coords[3] = {end.x, end.y, end.z};
    
    double cu = center_coords[u];
    double cv = center_coords[v];
    double su = start_coords[u];
    double sv = start_coords[v];
    double startAngle = std::atan2(sv - cv, su - cu);
    
    int numSteps = std::max(2, static_cast<int>(std::ceil(duration / timeStep_)));
    
    // Also ensure enough steps for arc fidelity
    double anglePerStep = std::abs(sweep) / numSteps;
    if (anglePerStep > 0.1) {  // Max ~6 degrees per step
        numSteps = std::max(numSteps, static_cast<int>(std::ceil(std::abs(sweep) / 0.1)));
    }
    
    for (int i = 0; i <= numSteps; ++i) {
        double t = static_cast<double>(i) / numSteps;
        double angle = startAngle + t * sweep;
        
        TrajectoryPoint pt;
        double coords[3];
        coords[u] = cu + radius * std::cos(angle);
        coords[v] = cv + radius * std::sin(angle);
        // Linear interpolation for the perpendicular axis (helical)
        coords[w] = start_coords[w] + t * (end_coords[w] - start_coords[w]);
        
        pt.position = Vec3(coords[0], coords[1], coords[2]);
        pt.feedRate = feedRate;
        pt.distanceFromStart = arcLen * t;
        pt.isRapid = false;
        pt.isArc = true;
        output.push_back(pt);
    }
}

// ============================================================================
// FixedDeviationStrategy Implementation
// ============================================================================

void FixedDeviationStrategy::interpolateLinear(
    const Vec3& start,
    const Vec3& end,
    double feedRate,
    std::vector<TrajectoryPoint>& output
) {
    // Linear segments need no subdivision for deviation (they're already straight)
    Vec3 delta = end - start;
    double distance = delta.length();
    
    // Just add start and end
    TrajectoryPoint startPt;
    startPt.position = start;
    startPt.feedRate = feedRate;
    startPt.distanceFromStart = 0.0;
    output.push_back(startPt);
    
    TrajectoryPoint endPt;
    endPt.position = end;
    endPt.feedRate = feedRate;
    endPt.distanceFromStart = distance;
    output.push_back(endPt);
}

void FixedDeviationStrategy::interpolateArc(
    const Vec3& start,
    const Vec3& end,
    const Vec3& center,
    double radius,
    double sweep,
    int plane,
    double feedRate,
    std::vector<TrajectoryPoint>& output
) {
    // Calculate number of segments needed for desired chord deviation
    // Chord deviation: d = r * (1 - cos(θ/2)) where θ is angle per segment
    // Solving for θ: θ = 2 * acos(1 - d/r)
    double maxAngle = 2.0 * std::acos(clamp(1.0 - maxDeviation_ / radius, -1.0, 1.0));
    int numSteps = std::max(2, static_cast<int>(std::ceil(std::abs(sweep) / maxAngle)));
    
    // Get plane axes - clamp to valid range
    int plane_idx = (plane >= 0 && plane <= 2) ? plane : 0;
    int u, v, w;
    switch (plane_idx) {
        case 1: u = 0; v = 2; w = 1; break;
        case 2: u = 1; v = 2; w = 0; break;
        default: u = 0; v = 1; w = 2; break;
    }
    
    // Use array indexing instead of pointer arithmetic
    const double center_coords[3] = {center.x, center.y, center.z};
    const double start_coords[3] = {start.x, start.y, start.z};
    const double end_coords[3] = {end.x, end.y, end.z};
    
    double cu = center_coords[u];
    double cv = center_coords[v];
    double su = start_coords[u];
    double sv = start_coords[v];
    double startAngle = std::atan2(sv - cv, su - cu);
    double arcLen = Math::arcLength(radius, sweep);
    
    for (int i = 0; i <= numSteps; ++i) {
        double t = static_cast<double>(i) / numSteps;
        double angle = startAngle + t * sweep;
        
        TrajectoryPoint pt;
        double coords[3];
        coords[u] = cu + radius * std::cos(angle);
        coords[v] = cv + radius * std::sin(angle);
        coords[w] = start_coords[w] + t * (end_coords[w] - start_coords[w]);
        
        pt.position = Vec3(coords[0], coords[1], coords[2]);
        pt.feedRate = feedRate;
        pt.distanceFromStart = arcLen * t;
        pt.isArc = true;
        output.push_back(pt);
    }
}

// ============================================================================
// AdaptiveStrategy Implementation
// ============================================================================

void AdaptiveStrategy::interpolateLinear(
    const Vec3& start,
    const Vec3& end,
    double feedRate,
    std::vector<TrajectoryPoint>& output
) {
    // Linear segments don't need adaptive subdivision
    Vec3 delta = end - start;
    double distance = delta.length();
    
    TrajectoryPoint startPt;
    startPt.position = start;
    startPt.feedRate = feedRate;
    startPt.distanceFromStart = 0.0;
    output.push_back(startPt);
    
    TrajectoryPoint endPt;
    endPt.position = end;
    endPt.feedRate = feedRate;
    endPt.distanceFromStart = distance;
    output.push_back(endPt);
}

void AdaptiveStrategy::interpolateArc(
    const Vec3& start,
    const Vec3& end,
    const Vec3& center,
    double radius,
    double sweep,
    int plane,
    double feedRate,
    std::vector<TrajectoryPoint>& output
) {
    // Adaptive subdivision based on curvature
    // Smaller radius = higher curvature = more points needed
    double curvature = 1.0 / radius;
    
    // Base step size inversely proportional to curvature
    double baseStep = clamp(tolerance_ / curvature, minStep_, maxStep_);
    
    // Calculate required steps
    double arcLen = Math::arcLength(radius, sweep);
    int numSteps = std::max(2, static_cast<int>(std::ceil(arcLen / baseStep)));
    
    // Get plane axes - clamp to valid range
    int plane_idx = (plane >= 0 && plane <= 2) ? plane : 0;
    int u, v, w;
    switch (plane_idx) {
        case 1: u = 0; v = 2; w = 1; break;
        case 2: u = 1; v = 2; w = 0; break;
        default: u = 0; v = 1; w = 2; break;
    }
    
    // Use array indexing instead of pointer arithmetic
    const double center_coords[3] = {center.x, center.y, center.z};
    const double start_coords[3] = {start.x, start.y, start.z};
    const double end_coords[3] = {end.x, end.y, end.z};
    
    double cu = center_coords[u];
    double cv = center_coords[v];
    double su = start_coords[u];
    double sv = start_coords[v];
    double startAngle = std::atan2(sv - cv, su - cu);
    
    for (int i = 0; i <= numSteps; ++i) {
        double t = static_cast<double>(i) / numSteps;
        double angle = startAngle + t * sweep;
        
        TrajectoryPoint pt;
        double coords[3];
        coords[u] = cu + radius * std::cos(angle);
        coords[v] = cv + radius * std::sin(angle);
        coords[w] = start_coords[w] + t * (end_coords[w] - start_coords[w]);
        
        pt.position = Vec3(coords[0], coords[1], coords[2]);
        pt.feedRate = feedRate;
        pt.distanceFromStart = arcLen * t;
        pt.isArc = true;
        output.push_back(pt);
    }
}

// ============================================================================
// MotionPrecomputer Implementation
// ============================================================================

MotionPrecomputer::MotionPrecomputer()
    : strategy_(std::make_unique<FixedTimeStrategy>(0.01))
{
}

MotionPrecomputer::~MotionPrecomputer() = default;

void MotionPrecomputer::setStrategy(std::unique_ptr<InterpolationStrategy> strategy) {
    if (strategy) {
        strategy_ = std::move(strategy);
    }
}

std::string MotionPrecomputer::strategyName() const {
    return strategy_ ? strategy_->name() : "None";
}

std::vector<TrajectoryPoint> MotionPrecomputer::precompute(
    const std::vector<MotionSegment>& segments
) {
    using Clock = std::chrono::high_resolution_clock;
    auto totalStart = Clock::now();
    
    stats_ = PrecomputeStats{};
    stats_.inputSegments = segments.size();
    
    std::vector<TrajectoryPoint> output;
    output.reserve(segments.size() * 100);  // Estimate
    
    auto interpStart = Clock::now();
    
    double totalDistance = 0.0;
    double totalTime = 0.0;
    
    for (size_t i = 0; i < segments.size(); ++i) {
        const auto& seg = segments[i];
        size_t startIdx = output.size();
        
        double effectiveFeedRate = seg.feedRate;
        if (seg.isRapid && effectiveFeedRate <= 0) {
            effectiveFeedRate = rapidFeedRate_;
        }
        
        if (seg.isArc) {
            stats_.arcSegments++;
            strategy_->interpolateArc(
                seg.start, seg.end, seg.center,
                seg.radius, seg.sweep, seg.plane,
                effectiveFeedRate, output
            );
        } else {
            stats_.linearSegments++;
            strategy_->interpolateLinear(
                seg.start, seg.end,
                effectiveFeedRate, output
            );
        }
        
        // Update metadata for newly added points
        for (size_t j = startIdx; j < output.size(); ++j) {
            output[j].blockIndex = seg.blockIndex;
            output[j].segmentIndex = static_cast<int>(i);
            output[j].isRapid = seg.isRapid;
            output[j].distanceFromStart += totalDistance;
            
            // Calculate time
            double velocity = effectiveFeedRate / 60.0;  // mm/s
            if (j > startIdx && velocity > EPSILON) {
                double dt = (output[j].position - output[j-1].position).length() / velocity;
                totalTime += dt;
            }
            output[j].time = totalTime;
        }
        
        // Update total distance
        if (!output.empty() && startIdx > 0) {
            totalDistance = output.back().distanceFromStart;
        }
    }
    
    stats_.interpolationTime = std::chrono::duration_cast<std::chrono::microseconds>(
        Clock::now() - interpStart);
    
    // Post-processing: calculate velocities and accelerations
    auto postStart = Clock::now();
    if (calcVelocity_ || calcAcceleration_) {
        calculateDerivatives(output);
    }
    stats_.postProcessTime = std::chrono::duration_cast<std::chrono::microseconds>(
        Clock::now() - postStart);
    
    stats_.outputPoints = output.size();
    stats_.totalTime = std::chrono::duration_cast<std::chrono::microseconds>(
        Clock::now() - totalStart);
    
    return output;
}

void MotionPrecomputer::calculateDerivatives(std::vector<TrajectoryPoint>& points) {
    if (points.size() < 2) return;
    
    for (size_t i = 0; i < points.size(); ++i) {
        if (calcVelocity_) {
            if (i == 0) {
                // Forward difference
                double dt = points[1].time - points[0].time;
                if (dt > EPSILON) {
                    points[i].velocity = (points[1].position - points[0].position) / dt;
                }
            } else if (i == points.size() - 1) {
                // Backward difference
                double dt = points[i].time - points[i-1].time;
                if (dt > EPSILON) {
                    points[i].velocity = (points[i].position - points[i-1].position) / dt;
                }
            } else {
                // Central difference
                double dt = points[i+1].time - points[i-1].time;
                if (dt > EPSILON) {
                    points[i].velocity = (points[i+1].position - points[i-1].position) / dt;
                }
            }
        }
        
        if (calcAcceleration_ && points.size() >= 3) {
            if (i == 0 || i == points.size() - 1) {
                points[i].acceleration = Vec3(0, 0, 0);
            } else {
                double dt1 = points[i].time - points[i-1].time;
                double dt2 = points[i+1].time - points[i].time;
                if (dt1 > EPSILON && dt2 > EPSILON) {
                    Vec3 v1 = (points[i].position - points[i-1].position) / dt1;
                    Vec3 v2 = (points[i+1].position - points[i].position) / dt2;
                    double dtAvg = (dt1 + dt2) / 2.0;
                    points[i].acceleration = (v2 - v1) / dtAvg;
                }
            }
        }
    }
}

// ============================================================================
// Factory Functions
// ============================================================================

std::unique_ptr<InterpolationStrategy> createStrategy(
    const std::string& name,
    double param
) {
    if (name == "FixedTime" || name == "fixedtime") {
        return std::make_unique<FixedTimeStrategy>(param);
    } else if (name == "FixedDeviation" || name == "fixeddeviation") {
        return std::make_unique<FixedDeviationStrategy>(param);
    } else if (name == "Adaptive" || name == "adaptive") {
        return std::make_unique<AdaptiveStrategy>(0.001, 0.1, param);
    }
    // Default
    return std::make_unique<FixedTimeStrategy>(param);
}

} // namespace Motion
} // namespace GCode
