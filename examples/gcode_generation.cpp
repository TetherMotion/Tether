/**
 * @file gcode_generation.cpp
 * @brief Example: G-Code test pattern generation
 * 
 * Demonstrates how to generate G-Code for various machine testing patterns
 * including circles, squares, sinusoids, friction tests, and full workspace sweeps.
 */

#include "GCodeGenerator.hpp"
#include "MachineTester.hpp"
#include <iostream>
#include <iomanip>
#include <fstream>

using namespace MotionReplanner;

void saveGCode(const GCodeProgram& program, const std::string& filename) {
    GCodeExporter exporter;
    
    if (exporter.exportToFile(program, filename)) {
        std::cout << "  Saved: " << filename << "\n";
        std::cout << "    Lines: " << program.blocks.size() << "\n";
        std::cout << "    Duration: " << std::fixed << std::setprecision(1) 
                  << program.estimateDuration() << "s\n";
    } else {
        std::cerr << "  Error saving: " << filename << "\n";
    }
}

int main() {
    std::cout << "=== Motion Replanner: G-Code Generation Example ===\n\n";
    
    // Default options
    GCodeOptions options;
    options.dialect = GCodeDialect::LinuxCNC;
    options.feedRate = 1000.0;  // mm/min
    options.rapidFeedRate = 3000.0;
    options.spindleSpeed = 12000;
    options.safeZ = 10.0;
    options.includeComments = true;
    options.useAbsoluteCoords = true;
    
    TestPatternGenerator generator(options);
    
    // --- Circle Test ---
    std::cout << "Generating Circle Tests...\n";
    
    CircleTestConfig circleConfig;
    circleConfig.centerX = 150.0;
    circleConfig.centerY = 100.0;
    circleConfig.radius = 50.0;
    circleConfig.feedRate = 500.0;
    circleConfig.repetitions = 3;
    circleConfig.clockwise = true;
    
    auto circleProgram = generator.generateCircleTest(circleConfig);
    saveGCode(circleProgram, "test_circle_cw.ngc");
    
    circleConfig.clockwise = false;
    circleProgram = generator.generateCircleTest(circleConfig);
    saveGCode(circleProgram, "test_circle_ccw.ngc");
    
    // --- Ellipse Test ---
    std::cout << "\nGenerating Ellipse Tests...\n";
    
    EllipseTestConfig ellipseConfig;
    ellipseConfig.centerX = 150.0;
    ellipseConfig.centerY = 100.0;
    ellipseConfig.semiAxisA = 60.0;
    ellipseConfig.semiAxisB = 30.0;
    ellipseConfig.feedRate = 400.0;
    
    // Generate ellipses at different rotations
    for (double rotation : {0.0, 30.0, 45.0, 60.0, 90.0}) {
        ellipseConfig.rotationDeg = rotation;
        auto ellipseProgram = generator.generateEllipseTest(ellipseConfig);
        
        std::ostringstream filename;
        filename << "test_ellipse_" << static_cast<int>(rotation) << "deg.ngc";
        saveGCode(ellipseProgram, filename.str());
    }
    
    // --- Square Test ---
    std::cout << "\nGenerating Square Test...\n";
    
    SquareTestConfig squareConfig;
    squareConfig.originX = 100.0;
    squareConfig.originY = 50.0;
    squareConfig.sideLength = 100.0;
    squareConfig.feedRate = 800.0;
    squareConfig.cornerPause = 0.1;  // 100ms pause at corners
    squareConfig.repetitions = 5;
    
    auto squareProgram = generator.generateSquareTest(squareConfig);
    saveGCode(squareProgram, "test_square.ngc");
    
    // --- Sinusoid Test (single axis) ---
    std::cout << "\nGenerating Sinusoid Tests...\n";
    
    SinusoidTestConfig sinConfig;
    sinConfig.amplitude = 50.0;
    sinConfig.wavelength = 100.0;
    sinConfig.cycles = 5;
    sinConfig.feedRate = 600.0;
    
    sinConfig.axis = Axis::X;
    sinConfig.startPosition = 50.0;
    auto sinProgramX = generator.generateSinusoidTest(sinConfig);
    saveGCode(sinProgramX, "test_sinusoid_x.ngc");
    
    sinConfig.axis = Axis::Y;
    sinConfig.startPosition = 30.0;
    auto sinProgramY = generator.generateSinusoidTest(sinConfig);
    saveGCode(sinProgramY, "test_sinusoid_y.ngc");
    
    // --- Ramp Test ---
    std::cout << "\nGenerating Ramp Tests...\n";
    
    RampTestConfig rampConfig;
    rampConfig.startPosition = 10.0;
    rampConfig.endPosition = 290.0;
    rampConfig.rampUpDistance = 50.0;
    rampConfig.rampDownDistance = 50.0;
    rampConfig.maxFeedRate = 2000.0;
    
    rampConfig.axis = Axis::X;
    auto rampProgramX = generator.generateRampTest(rampConfig);
    saveGCode(rampProgramX, "test_ramp_x.ngc");
    
    // --- S-Curve Test ---
    std::cout << "\nGenerating S-Curve Tests...\n";
    
    SCurveTestConfig scurveConfig;
    scurveConfig.startPosition = 10.0;
    scurveConfig.endPosition = 290.0;
    scurveConfig.jerkLimit = 5000.0;
    scurveConfig.maxAcceleration = 500.0;
    scurveConfig.maxVelocity = 100.0;
    
    scurveConfig.axis = Axis::X;
    auto scurveProgramX = generator.generateSCurveTest(scurveConfig);
    saveGCode(scurveProgramX, "test_scurve_x.ngc");
    
    // --- Helix Test ---
    std::cout << "\nGenerating Helix Test...\n";
    
    HelixTestConfig helixConfig;
    helixConfig.centerX = 150.0;
    helixConfig.centerY = 100.0;
    helixConfig.radius = 40.0;
    helixConfig.startZ = 50.0;
    helixConfig.endZ = 10.0;
    helixConfig.turns = 4;
    helixConfig.feedRate = 300.0;
    
    auto helixProgram = generator.generateHelixTest(helixConfig);
    saveGCode(helixProgram, "test_helix.ngc");
    
    // --- Lissajous Test ---
    std::cout << "\nGenerating Lissajous Patterns...\n";
    
    LissajousTestConfig lissajousConfig;
    lissajousConfig.centerX = 150.0;
    lissajousConfig.centerY = 100.0;
    lissajousConfig.amplitudeX = 50.0;
    lissajousConfig.amplitudeY = 40.0;
    lissajousConfig.cycles = 2;
    lissajousConfig.feedRate = 400.0;
    
    // Different frequency ratios create different patterns
    std::vector<std::pair<int, int>> ratios = {{1, 2}, {2, 3}, {3, 4}, {3, 5}};
    
    for (const auto& [fx, fy] : ratios) {
        lissajousConfig.frequencyRatioX = fx;
        lissajousConfig.frequencyRatioY = fy;
        
        auto lissProgram = generator.generateLissajousTest(lissajousConfig);
        
        std::ostringstream filename;
        filename << "test_lissajous_" << fx << "_" << fy << ".ngc";
        saveGCode(lissProgram, filename.str());
    }
    
    // --- Friction Test ---
    std::cout << "\nGenerating Friction Test...\n";
    
    FrictionTestConfig frictionConfig;
    frictionConfig.axis = Axis::X;
    frictionConfig.startPosition = 50.0;
    frictionConfig.endPosition = 250.0;
    frictionConfig.velocitySteps = {10.0, 25.0, 50.0, 100.0, 200.0, 500.0, 1000.0};
    frictionConfig.dwellTime = 0.5;  // 500ms between steps
    
    auto frictionProgram = generator.generateFrictionTest(frictionConfig);
    saveGCode(frictionProgram, "test_friction_x.ngc");
    
    // --- Full Workspace Sweep ---
    std::cout << "\nGenerating Workspace Sweep...\n";
    
    // Define workspace bounds
    double xMin = 0, xMax = 300;
    double yMin = 0, yMax = 200;
    double zHeight = 5.0;
    
    auto sweepProgram = generator.generateWorkspaceSweep(
        xMin, xMax, yMin, yMax, zHeight,
        10,   // X divisions
        8,    // Y divisions
        500.0 // Feed rate
    );
    saveGCode(sweepProgram, "test_workspace_sweep.ngc");
    
    // --- Multi-dialect export ---
    std::cout << "\nGenerating Multi-Dialect Exports...\n";
    
    // Export the circle test in different dialects
    struct DialectInfo {
        GCodeDialect dialect;
        std::string name;
        std::string ext;
    };
    
    std::vector<DialectInfo> dialects = {
        {GCodeDialect::LinuxCNC, "LinuxCNC", "ngc"},
        {GCodeDialect::Fanuc,    "Fanuc",    "nc"},
        {GCodeDialect::Grbl,     "Grbl",     "gcode"},
        {GCodeDialect::Mach3,    "Mach3",    "tap"},
        {GCodeDialect::Marlin,   "Marlin",   "gcode"},
    };
    
    for (const auto& d : dialects) {
        options.dialect = d.dialect;
        TestPatternGenerator dialectGen(options);
        
        circleConfig.clockwise = true;
        auto dialectCircle = dialectGen.generateCircleTest(circleConfig);
        
        std::ostringstream filename;
        filename << "circle_" << d.name << "." << d.ext;
        saveGCode(dialectCircle, filename.str());
    }
    
    std::cout << "\n=== G-Code Generation Complete ===\n";
    std::cout << "\nGenerated test patterns can be loaded into your CNC controller\n";
    std::cout << "or simulated in tools like CAMotics, NCViewer, etc.\n";
    
    return 0;
}
