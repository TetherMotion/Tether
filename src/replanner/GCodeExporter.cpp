/**
 * @file GCodeExporter.cpp
 * @brief GCodeExporter implementation for trajectory export
 * 
 * Split from GCodeGenerator.cpp for maintainability.
 */

#include "GCodeGenerator.hpp"
#include <algorithm>
#include <cmath>

namespace MotionReplanner {

//=============================================================================
// GCodeExporter Implementation
//=============================================================================

GCodeExporter::GCodeExporter(const GCodeOptions& opts)
    : opts_(opts) {}

GCodeProgram GCodeExporter::exportTrajectory(const std::vector<PositionSample>& samples) {
    GCodeProgram prog(opts_);
    prog.addProgramStart();
    
    if (samples.empty()) {
        prog.addProgramEnd();
        return prog;
    }
    
    // Move to first point
    const auto& first = samples[0];
    prog.addRapidZ(opts_.safeZ);
    prog.addRapidXY(first.position[0], first.position[1]);
    prog.addRapidZ(first.position[2]);
    
    double lastFeed = 0;
    
    for (size_t i = 1; i < samples.size(); ++i) {
        const auto& sample = samples[i];
        
        // Calculate velocity for feed rate
        double dt = sample.timestamp - samples[i-1].timestamp;
        double dx = sample.position[0] - samples[i-1].position[0];
        double dy = sample.position[1] - samples[i-1].position[1];
        double dz = sample.position[2] - samples[i-1].position[2];
        double dist = std::sqrt(dx*dx + dy*dy + dz*dz);
        
        double feedRate = (dt > 0) ? (dist / dt * 60.0) : opts_.defaultFeedRate;
        feedRate = std::max(1.0, std::min(opts_.rapidRate, feedRate));
        
        // Only output feed rate if changed significantly
        if (std::abs(feedRate - lastFeed) / feedRate > 0.05) {
            prog.addLinear(sample.position[0], sample.position[1], sample.position[2], feedRate);
            lastFeed = feedRate;
        } else {
            prog.addLinear(sample.position[0], sample.position[1], sample.position[2]);
        }
    }
    
    prog.addRapidZ(opts_.safeZ);
    prog.addProgramEnd();
    
    return prog;
}

GCodeProgram GCodeExporter::exportTrajectoryWithArcs(
    const std::vector<PositionSample>& samples, double arcTolerance) {
    
    GCodeProgram prog(opts_);
    prog.addProgramStart();
    
    if (samples.empty()) {
        prog.addProgramEnd();
        return prog;
    }
    
    // Move to first point
    prog.addRapidZ(opts_.safeZ);
    prog.addRapidXY(samples[0].position[0], samples[0].position[1]);
    prog.addRapidZ(samples[0].position[2]);
    
    size_t i = 1;
    while (i < samples.size()) {
        size_t arcEnd;
        double cx, cy, r;
        bool cw;
        
        if (fitArc(samples, i, arcEnd, cx, cy, r, cw)) {
            // Output arc
            if (cw) {
                prog.addArcCW(samples[arcEnd].position[0], samples[arcEnd].position[1],
                             cx - samples[i-1].position[0], cy - samples[i-1].position[1],
                             opts_.defaultFeedRate);
            } else {
                prog.addArcCCW(samples[arcEnd].position[0], samples[arcEnd].position[1],
                              cx - samples[i-1].position[0], cy - samples[i-1].position[1],
                              opts_.defaultFeedRate);
            }
            i = arcEnd + 1;
        } else {
            // Output linear
            prog.addLinear(samples[i].position[0], samples[i].position[1],
                          samples[i].position[2], opts_.defaultFeedRate);
            i++;
        }
    }
    
    prog.addRapidZ(opts_.safeZ);
    prog.addProgramEnd();
    
    return prog;
}

bool GCodeExporter::fitArc(const std::vector<PositionSample>& samples, size_t start, size_t& end,
                           double& cx, double& cy, double& r, bool& clockwise) {
    // Simple arc fitting - check if 3+ points lie on a circle
    if (start + 2 >= samples.size()) return false;
    
    // Get three points
    double x1 = samples[start].position[0], y1 = samples[start].position[1];
    double x2 = samples[start + 1].position[0], y2 = samples[start + 1].position[1];
    double x3 = samples[start + 2].position[0], y3 = samples[start + 2].position[1];
    
    // Calculate circumcenter
    double d = 2 * (x1 * (y2 - y3) + x2 * (y3 - y1) + x3 * (y1 - y2));
    if (std::abs(d) < 1e-9) return false;  // Collinear
    
    double ux = ((x1*x1 + y1*y1) * (y2 - y3) + (x2*x2 + y2*y2) * (y3 - y1) + (x3*x3 + y3*y3) * (y1 - y2)) / d;
    double uy = ((x1*x1 + y1*y1) * (x3 - x2) + (x2*x2 + y2*y2) * (x1 - x3) + (x3*x3 + y3*y3) * (x2 - x1)) / d;
    
    r = std::sqrt((x1 - ux) * (x1 - ux) + (y1 - uy) * (y1 - uy));
    
    // Check if radius is reasonable
    if (r < opts_.arcMinRadius || r > 10000) return false;
    
    // Determine direction
    double cross = (x2 - x1) * (y3 - y1) - (y2 - y1) * (x3 - x1);
    clockwise = (cross < 0);
    
    cx = ux;
    cy = uy;
    
    // Find how many more points fit the arc
    end = start + 2;
    for (size_t i = start + 3; i < samples.size(); ++i) {
        double px = samples[i].position[0];
        double py = samples[i].position[1];
        double dist = std::sqrt((px - cx) * (px - cx) + (py - cy) * (py - cy));
        
        if (std::abs(dist - r) > opts_.arcTolerance) break;
        end = i;
    }
    
    return (end > start + 2);  // Need at least 3 points
}

GCodeProgram GCodeExporter::exportTestResult(const TestResult& result) {
    GCodeProgram prog(opts_);
    prog.addProgramStart();
    
    prog.addComment("Test: " + result.testName);
    prog.addComment("Type: " + result.testType);
    
    // Export desired trajectory
    prog.addComment("--- Desired Trajectory ---");
    for (const auto& sample : result.desiredSamples) {
        prog.addLinear(sample.position[0], sample.position[1], sample.position[2],
                      opts_.defaultFeedRate);
    }
    
    prog.addProgramEnd();
    return prog;
}

} // namespace MotionReplanner
