/**
 * @file GCodePatterns.cpp
 * @brief TestPatternGenerator implementation for G-code test patterns
 * 
 * Split from GCodeGenerator.cpp for maintainability.
 */

#include "tether/motion_replanner/GCodeGenerator.hpp"
#include <algorithm>
#include <cmath>

namespace MotionReplanner {

//=============================================================================
// TestPatternGenerator Implementation
//=============================================================================

TestPatternGenerator::TestPatternGenerator(const GCodeOptions& opts)
    : opts_(opts) {}

GCodeProgram TestPatternGenerator::generateSingleAxisTest(const SingleAxisTestConfig& config) {
    GCodeProgram prog(opts_);
    prog.addProgramStart();
    
    std::string testName;
    switch (config.type) {
        case SingleAxisTestType::Sinusoid: testName = "Sinusoid"; break;
        case SingleAxisTestType::Ramp: testName = "Ramp"; break;
        case SingleAxisTestType::SCurve: testName = "S-Curve"; break;
        case SingleAxisTestType::Step: testName = "Step"; break;
        case SingleAxisTestType::Triangular: testName = "Triangular"; break;
        case SingleAxisTestType::Trapezoidal: testName = "Trapezoidal"; break;
    }
    
    const char* axisNames = "XYZABCUVW";
    prog.addComment("Single axis test: " + testName + " on " + axisNames[config.axis] +
                    ", Amplitude: " + std::to_string(config.amplitude) + "mm");
    
    // Move to start position
    double startPos[3] = {0, 0, opts_.safeZ};
    startPos[config.axis % 3] = config.centerPosition - config.amplitude;
    prog.addRapid(startPos[0], startPos[1], startPos[2]);
    
    switch (config.type) {
        case SingleAxisTestType::Sinusoid:
            return generateSinusoidTest(config.axis, config.centerPosition,
                                        config.amplitude, config.frequency,
                                        config.cycles, config.velocity);
        case SingleAxisTestType::Ramp:
            return generateRampTest(config.axis, 
                                   config.centerPosition - config.amplitude,
                                   config.centerPosition + config.amplitude,
                                   config.velocity, config.cycles);
        default:
            break;
    }
    
    prog.addProgramEnd();
    return prog;
}

GCodeProgram TestPatternGenerator::generateMultiAxisTest(const MultiAxisTestConfig& config) {
    switch (config.type) {
        case MultiAxisTestType::Circle:
            return generateCircleTest(config.center[0], config.center[1], config.center[2],
                                      config.radiusU, config.feedRate, config.revolutions);
        case MultiAxisTestType::Ellipse:
            return generateEllipseTest(config.center[0], config.center[1], config.center[2],
                                       config.radiusU, config.radiusV, config.rotationAngle,
                                       config.feedRate, config.revolutions);
        case MultiAxisTestType::Helix:
            return generateHelixTest(config.center[0], config.center[1], config.center[2],
                                     config.radiusU, config.pitchW, config.feedRate,
                                     config.revolutions);
        case MultiAxisTestType::Square:
            return generateSquareTest(config.center[0], config.center[1], config.center[2],
                                      config.radiusU * 2, config.feedRate, config.revolutions);
        case MultiAxisTestType::RoundedSquare:
            return generateRoundedSquareTest(config.center[0], config.center[1], config.center[2],
                                             config.radiusU * 2, config.cornerRadius,
                                             config.feedRate, config.revolutions);
        default:
            break;
    }
    
    return GCodeProgram(opts_);
}

GCodeProgram TestPatternGenerator::generateCircleTest(
    double centerX, double centerY, double centerZ,
    double radius, double feedRate, int revolutions) {
    
    GCodeProgram prog(opts_);
    prog.addProgramStart();
    
    prog.addComment("Circle test: R=" + std::to_string(radius) + 
                    " F=" + std::to_string(feedRate));
    
    // Move to start position (right side of circle)
    double startX = centerX + radius;
    double startY = centerY;
    
    prog.addRapidZ(opts_.safeZ);
    prog.addRapidXY(startX, startY);
    prog.addRapidZ(centerZ);
    
    // Full circles using G2/G3
    for (int rev = 0; rev < revolutions; ++rev) {
        // Use two semicircles for full circle
        prog.addArcCW(centerX - radius, centerY, -radius, 0, feedRate);
        prog.addArcCW(centerX + radius, centerY, radius, 0, feedRate);
    }
    
    prog.addRapidZ(opts_.safeZ);
    prog.addProgramEnd();
    
    return prog;
}

GCodeProgram TestPatternGenerator::generateEllipseTest(
    double centerX, double centerY, double centerZ,
    double radiusX, double radiusY, double rotation,
    double feedRate, int revolutions) {
    
    GCodeProgram prog(opts_);
    prog.addProgramStart();
    
    prog.addComment("Ellipse test: Rx=" + std::to_string(radiusX) +
                    " Ry=" + std::to_string(radiusY));
    
    double rotRad = rotation * M_PI / 180.0;
    double cosR = std::cos(rotRad);
    double sinR = std::sin(rotRad);
    
    // Start at angle 0
    double startX = centerX + radiusX * cosR;
    double startY = centerY + radiusX * sinR;
    
    prog.addRapidZ(opts_.safeZ);
    prog.addRapidXY(startX, startY);
    prog.addRapidZ(centerZ);
    
    // Approximate ellipse with line segments
    int segments = 72;
    for (int rev = 0; rev < revolutions; ++rev) {
        for (int i = 1; i <= segments; ++i) {
            double angle = 2.0 * M_PI * i / segments;
            double localX = radiusX * std::cos(angle);
            double localY = radiusY * std::sin(angle);
            
            double x = centerX + localX * cosR - localY * sinR;
            double y = centerY + localX * sinR + localY * cosR;
            
            prog.addLinearXY(x, y, feedRate);
        }
    }
    
    prog.addRapidZ(opts_.safeZ);
    prog.addProgramEnd();
    
    return prog;
}

GCodeProgram TestPatternGenerator::generateHelixTest(
    double centerX, double centerY, double startZ,
    double radius, double pitch, double feedRate, int revolutions) {
    
    GCodeProgram prog(opts_);
    prog.addProgramStart();
    
    prog.addComment("Helix test: R=" + std::to_string(radius) +
                    " Pitch=" + std::to_string(pitch));
    
    double startX = centerX + radius;
    
    prog.addRapidZ(opts_.safeZ);
    prog.addRapidXY(startX, centerY);
    prog.addRapidZ(startZ);
    
    // Helical arc
    double currentZ = startZ;
    int segmentsPerRev = 4;  // 4 quarter arcs per revolution
    
    for (int rev = 0; rev < revolutions; ++rev) {
        for (int seg = 0; seg < segmentsPerRev; ++seg) {
            double endAngle = (seg + 1) * M_PI / 2;
            double endX = centerX + radius * std::cos(endAngle);
            double endY = centerY + radius * std::sin(endAngle);
            double endZ = currentZ + pitch / segmentsPerRev;
            
            double startAngle = seg * M_PI / 2;
            double i = -radius * std::cos(startAngle);
            double j = -radius * std::sin(startAngle);
            
            GCodeBlock block;
            block.gCode = "G2";
            block.position[0] = endX; block.hasPosition[0] = true;
            block.position[1] = endY; block.hasPosition[1] = true;
            block.position[2] = endZ; block.hasPosition[2] = true;
            block.i = i; block.j = j; block.hasIJK = true;
            block.feedRate = feedRate;
            prog.addBlock(block);
            
            currentZ = endZ;
        }
    }
    
    prog.addRapidZ(opts_.safeZ);
    prog.addProgramEnd();
    
    return prog;
}

GCodeProgram TestPatternGenerator::generateSinusoidTest(
    int axis, double center, double amplitude,
    double frequency, int cycles, double feedRate) {
    
    GCodeProgram prog(opts_);
    prog.addProgramStart();
    
    const char* axisNames = "XYZABCUVW";
    prog.addComment("Sinusoid test: " + std::string(1, axisNames[axis]) +
                    " A=" + std::to_string(amplitude) +
                    " f=" + std::to_string(frequency) + "Hz");
    
    // Calculate total distance for velocity-based timing
    double period = 1.0 / frequency;
    double totalTime = cycles * period;
    double approxDistance = cycles * 4 * amplitude;  // Approximate
    double velocityMmPerSec = feedRate / 60.0;
    
    int points = static_cast<int>(approxDistance / (velocityMmPerSec * 0.001));  // ~1ms spacing
    points = std::max(100, std::min(10000, points));
    
    // Move to start
    double startPos[3] = {0, 0, opts_.safeZ};
    startPos[axis % 3] = center;
    
    prog.addRapidZ(opts_.safeZ);
    prog.addRapid(startPos[0], startPos[1], startPos[2]);
    
    // Generate sinusoid points
    double dx = 0, dy = 0, dz = 0;
    switch (axis) {
        case 0: /* axisPtr = &dx; */ break;
        case 1: /* axisPtr = &dy; */ break;
        case 2: /* axisPtr = &dz; */ break;
    }
    
    double travelAxis = (axis == 0) ? 1 : 0;  // Move along perpendicular axis
    double travelLength = cycles * 50.0;  // 50mm per cycle
    
    for (int i = 0; i <= points; ++i) {
        double t = static_cast<double>(i) / points * totalTime;
        double travel = static_cast<double>(i) / points * travelLength;
        
        double sinVal = amplitude * std::sin(2.0 * M_PI * frequency * t);
        
        double x = (travelAxis == 0) ? travel : (axis == 0 ? center + sinVal : 0);
        double y = (travelAxis == 1) ? travel : (axis == 1 ? center + sinVal : 0);
        double z = (axis == 2) ? center + sinVal : opts_.safeZ;
        
        if (axis == 0) x = center + sinVal;
        else if (axis == 1) y = center + sinVal;
        else z = center + sinVal;
        
        if (travelAxis == 0) x = travel;
        else y = travel;
        
        prog.addLinear(x, y, z, feedRate);
    }
    
    prog.addRapidZ(opts_.safeZ);
    prog.addProgramEnd();
    
    return prog;
}

GCodeProgram TestPatternGenerator::generateRampTest(
    int axis, double start, double end,
    double feedRate, int repeats) {
    
    GCodeProgram prog(opts_);
    prog.addProgramStart();
    
    const char* axisNames = "XYZABCUVW";
    prog.addComment("Ramp test: " + std::string(1, axisNames[axis]) +
                    " " + std::to_string(start) + " to " + std::to_string(end));
    
    double pos[3] = {0, 0, opts_.safeZ};
    pos[axis % 3] = start;
    
    prog.addRapid(pos[0], pos[1], pos[2]);
    
    for (int rep = 0; rep < repeats; ++rep) {
        // Forward
        pos[axis % 3] = end;
        prog.addLinear(pos[0], pos[1], pos[2], feedRate);
        
        prog.addDwell(0.1);
        
        // Back
        pos[axis % 3] = start;
        prog.addLinear(pos[0], pos[1], pos[2], feedRate);
        
        prog.addDwell(0.1);
    }
    
    prog.addRapidZ(opts_.safeZ);
    prog.addProgramEnd();
    
    return prog;
}

GCodeProgram TestPatternGenerator::generateSquareTest(
    double centerX, double centerY, double centerZ,
    double size, double feedRate, int revolutions) {
    
    GCodeProgram prog(opts_);
    prog.addProgramStart();
    
    prog.addComment("Square test: Size=" + std::to_string(size));
    
    double half = size / 2;
    
    // Corners
    double corners[4][2] = {
        {centerX + half, centerY + half},
        {centerX - half, centerY + half},
        {centerX - half, centerY - half},
        {centerX + half, centerY - half}
    };
    
    prog.addRapidZ(opts_.safeZ);
    prog.addRapidXY(corners[0][0], corners[0][1]);
    prog.addRapidZ(centerZ);
    
    for (int rev = 0; rev < revolutions; ++rev) {
        for (int i = 1; i <= 4; ++i) {
            int idx = i % 4;
            prog.addLinearXY(corners[idx][0], corners[idx][1], feedRate);
        }
    }
    
    prog.addRapidZ(opts_.safeZ);
    prog.addProgramEnd();
    
    return prog;
}

GCodeProgram TestPatternGenerator::generateRoundedSquareTest(
    double centerX, double centerY, double centerZ,
    double size, double cornerRadius, double feedRate, int revolutions) {
    
    GCodeProgram prog(opts_);
    prog.addProgramStart();
    
    prog.addComment("Rounded square test: Size=" + std::to_string(size) +
                    " R=" + std::to_string(cornerRadius));
    
    double half = size / 2;
    double r = std::min(cornerRadius, half * 0.9);  // Limit corner radius
    
    // Start point (middle of right edge)
    prog.addRapidZ(opts_.safeZ);
    prog.addRapidXY(centerX + half, centerY);
    prog.addRapidZ(centerZ);
    
    for (int rev = 0; rev < revolutions; ++rev) {
        // Top-right corner approach
        prog.addLinearXY(centerX + half, centerY + half - r, feedRate);
        // Top-right corner arc
        prog.addArcCW(centerX + half - r, centerY + half, 0, r, feedRate);
        
        // Top edge
        prog.addLinearXY(centerX - half + r, centerY + half, feedRate);
        // Top-left corner arc
        prog.addArcCW(centerX - half, centerY + half - r, -r, 0, feedRate);
        
        // Left edge
        prog.addLinearXY(centerX - half, centerY - half + r, feedRate);
        // Bottom-left corner arc
        prog.addArcCW(centerX - half + r, centerY - half, 0, -r, feedRate);
        
        // Bottom edge
        prog.addLinearXY(centerX + half - r, centerY - half, feedRate);
        // Bottom-right corner arc
        prog.addArcCW(centerX + half, centerY - half + r, r, 0, feedRate);
        
        // Complete right edge
        prog.addLinearXY(centerX + half, centerY, feedRate);
    }
    
    prog.addRapidZ(opts_.safeZ);
    prog.addProgramEnd();
    
    return prog;
}

GCodeProgram TestPatternGenerator::generateFrictionTest(
    int axis, double distance,
    const std::vector<double>& feedRates, int repeatsPerRate) {
    
    GCodeProgram prog(opts_);
    prog.addProgramStart();
    
    const char* axisNames = "XYZABCUVW";
    prog.addComment("Friction test: Axis " + std::string(1, axisNames[axis]));
    
    double pos[3] = {0, 0, opts_.safeZ};
    
    for (double feedRate : feedRates) {
        prog.addComment("Feed rate: " + std::to_string(feedRate) + " mm/min");
        
        for (int rep = 0; rep < repeatsPerRate; ++rep) {
            // Forward
            pos[axis % 3] = distance;
            prog.addLinear(pos[0], pos[1], pos[2], feedRate);
            prog.addDwell(0.5);
            
            // Back
            pos[axis % 3] = 0;
            prog.addLinear(pos[0], pos[1], pos[2], feedRate);
            prog.addDwell(0.5);
        }
    }
    
    prog.addRapidZ(opts_.safeZ);
    prog.addProgramEnd();
    
    return prog;
}

GCodeProgram TestPatternGenerator::generateWorkspaceSweep(
    const HeatmapConfig& heatmapConfig, double testRadius, double feedRate) {
    
    GCodeProgram prog(opts_);
    prog.addProgramStart();
    
    prog.addComment("Workspace sweep for performance mapping");
    
    double xRange = heatmapConfig.maxBounds[0] - heatmapConfig.minBounds[0];
    double yRange = heatmapConfig.maxBounds[1] - heatmapConfig.minBounds[1];
    double zRange = heatmapConfig.maxBounds[2] - heatmapConfig.minBounds[2];
    
    int numX = static_cast<int>(xRange / heatmapConfig.resolution3D);
    int numY = static_cast<int>(yRange / heatmapConfig.resolution3D);
    int numZ = std::max(1, static_cast<int>(zRange / (heatmapConfig.resolution3D * 2)));
    
    for (int iz = 0; iz < numZ; ++iz) {
        double z = heatmapConfig.minBounds[2] + (iz + 0.5) * zRange / numZ;
        
        prog.addComment("Z level: " + std::to_string(z));
        
        for (int iy = 0; iy < numY; ++iy) {
            double y = heatmapConfig.minBounds[1] + (iy + 0.5) * yRange / numY;
            
            for (int ix = 0; ix < numX; ++ix) {
                double x = heatmapConfig.minBounds[0] + (ix + 0.5) * xRange / numX;
                
                // Move to position
                prog.addRapidZ(opts_.safeZ);
                prog.addRapidXY(x + testRadius, y);
                prog.addRapidZ(z);
                
                // Small circle
                prog.addArcCW(x - testRadius, y, -testRadius, 0, feedRate);
                prog.addArcCW(x + testRadius, y, testRadius, 0, feedRate);
            }
        }
    }
    
    prog.addRapidZ(opts_.safeZ);
    prog.addRapidXY(0, 0);
    prog.addProgramEnd();
    
    return prog;
}

GCodeProgram TestPatternGenerator::generateCalibrationSequence(const TestSequence& sequence) {
    GCodeProgram prog(opts_);
    prog.addProgramStart();
    
    prog.addComment("Calibration sequence: " + sequence.name);
    prog.addComment(sequence.description);
    prog.addBlankLine();
    
    // Generate all single axis tests
    for (const auto& config : sequence.singleAxisTests) {
        prog.addComment("=== Single axis test ===");
        auto subProg = generateSingleAxisTest(config);
        for (const auto& block : subProg.blocks()) {
            prog.addBlock(block);
        }
        prog.addDwell(1.0);
    }
    
    // Generate all multi axis tests
    for (const auto& config : sequence.multiAxisTests) {
        prog.addComment("=== Multi axis test ===");
        auto subProg = generateMultiAxisTest(config);
        for (const auto& block : subProg.blocks()) {
            prog.addBlock(block);
        }
        prog.addDwell(1.0);
    }
    
    prog.addProgramEnd();
    return prog;
}

} // namespace MotionReplanner
