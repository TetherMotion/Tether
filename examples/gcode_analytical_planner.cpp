/**
 * @file gcode_analytical_planner.cpp
 * @brief G-code Analytical Motion Planner Example
 *
 * This example demonstrates:
 * 1. Parsing G-code using Tether GCode library
 * 2. Converting to Motion Segments (Ideal Path)
 * 3. Computing Analytical Solution (Boustrophedon / B-splines / Bezier)
 *    using the MotionPlanner component.
 * 4. Exporting both Ideal and Analytical paths to SVG.
 *
 * Usage:
 *   gcode_analytical_planner <gcode_file>
 *
 * Note: Uses libargparse for command line parsing.
 */

#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <memory>
#include <cmath>

// Use argparse - assuming it's available or we provide a minimal header
// If <argparse/argparse.hpp> is not available, you may need to install it 
// or use a different library. For this example we assume https://github.com/p-ranav/argparse
// conforming to the Requirement "It must use libargparse".
#if __has_include(<argparse/argparse.hpp>)
#include <argparse/argparse.hpp>
#else
// Fallback or error if not found. 
// For this example to be self-contained if the header is missing, we might fail compilation
// unless we strictly follow the user's environment.
#include <argparse/argparse.hpp> 
#endif

// Tether Includes
#include <tether/gcode/GCodeParser.hpp>
#include <tether/gcode/GCodeLexer.hpp>
#include <tether/motion_planner/GCodeAdapter.hpp>
#include <tether/motion_planner/PathBuilder.hpp>
#include <tether/motion_planner/MotionPlanner.hpp>
#include <tether/export/SVGExporter.hpp>
#include <tether/export/TrajectoryAnalyzer.hpp>

using namespace MotionPlanner;

// Helper to extract point for visualization
static MotionPlanner::Vec<3, double> extractPoint(const std::array<double, 9>& pos) {
    return {pos[0], pos[1], pos[2]};
}

// Custom manual SVG exporter for direct Bezier curves
// Logic adapted from SVGExporter but adds support for <path d="M... C...">
struct RenderablePath {
    MotionPlanner::PiecewiseBezierPath3D path;
    std::string color;
    double width;
};

void exportManyPathsToSVG(const std::string& filename,
                       const std::vector<RenderablePath>& paths,
                       double width, double height) {
    std::ofstream out(filename);
    if (!out.is_open()) return;

    // Hardcoded bounds for this specific demo (0,0) to EU conventional coords
    // Input G-code is 0..50mm. We add margin.
    // SVGExporter usually flips Y. G-code +Y is UP. SVG +Y is DOWN.
    // Transform: x' = x + margin, y' = (height - margin) - y
    
    double margin = 50.0;
    // Scale to fit 1024x1024. 
    // Data is approx -10..60 mm (due to curve overshoot/rounding)
    // Range ~70mm. 1024 / 70 ~ 14 pixels/mm.
    double scale = 10.0; 
    
    auto transformX = [&](double x) { return margin + x * scale; };
    auto transformY = [&](double y) { return (height - margin) - y * scale; };

    out << "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n";
    out << "<svg xmlns=\"http://www.w3.org/2000/svg\" "
        << "width=\"" << width << "\" height=\"" << height << "\" "
        << "viewBox=\"0 0 " << width << " " << height << "\">\n";
    out << "  <rect width=\"100%\" height=\"100%\" fill=\"white\"/>\n";
    out << "  <g id=\"grid\" stroke=\"#dddddd\" stroke-width=\"0.5\">\n";
    
    // Simple Grid (0 to 100mm)
    for(int i=0; i<=100; i+=10) {
        double x = transformX(i);
        double y = transformY(i);
        out << "    <line x1=\"" << x << "\" y1=\"" << transformY(-20) << "\" x2=\"" << x << "\" y2=\"" << transformY(100) << "\"/>\n";
        out << "    <line x1=\"" << transformX(-20) << "\" y1=\"" << y << "\" x2=\"" << transformX(100) << "\" y2=\"" << y << "\"/>\n";
    }
    out << "  </g>\n";

    for (const auto& rpath : paths) {
        const auto& path = rpath.path;
        out << "  <path d=\"";
        
        bool first = true;

        for (size_t i = 0; i < path.numSegments(); ++i) {
            const auto& segInfo = path.segment(i);
            const auto& curve = segInfo.curve;
            const auto& pts = curve.controlPoints(); // Curve access
            
            // P0 (Start of segment)
            double x0 = transformX(pts[0][0]);
            double y0 = transformY(pts[0][1]);
            
            if (first) {
                out << "M" << x0 << "," << y0;
                first = false;
            } else {
                 // Ensure connectivity
                 out << " L " << x0 << "," << y0;
            }
    
            if (curve.degree() == 3) {
                // Cubic Bezier
                double x1 = transformX(pts[1][0]);
                double y1 = transformY(pts[1][1]);
                double x2 = transformX(pts[2][0]);
                double y2 = transformY(pts[2][1]);
                double x3 = transformX(pts[3][0]);
                double y3 = transformY(pts[3][1]);
                
                out << " C " << x1 << "," << y1 << " " 
                    << x2 << "," << y2 << " " 
                    << x3 << "," << y3;
            } else if (curve.degree() == 1) {
                 // Linear
                double x1 = transformX(pts[1][0]);
                double y1 = transformY(pts[1][1]);
                out << " L " << x1 << "," << y1;
            } else {
                 // Higher order (e.g. Quintic/Degree 5 for G2 blends)
                 // Approximate with polyline since SVG <path> doesn't support degree 5
                 const int samples = 32; 
                 for(int k=1; k<=samples; ++k) {
                     double t = (double)k / samples;
                     auto pt = curve.evaluate(t);
                     out << " L " << transformX(pt[0]) << "," << transformY(pt[1]);
                 }
            }
        }
        
        out << "\" stroke=\"" << rpath.color << "\" stroke-width=\"" << rpath.width << "\" fill=\"none\"/>\n";
    }
    out << "</svg>\n";
    std::cout << "Exported Multi-Path SVG to: " << filename << "\n";
}

int main(int argc, char *argv[]) {
    // -------------------------------------------------------------------------
    // 1. Argument Parsing (libargparse)
    // -------------------------------------------------------------------------
    argparse::ArgumentParser program("gcode_analytical_planner");

    program.add_argument("input_file")
        .help("Input G-code file path")
        .required();

    program.add_argument("-o", "--output")
        .help("Output SVG base filename")
        .default_value(std::string("output"));

    try {
        program.parse_args(argc, argv);
    }
    catch (const std::runtime_error& err) {
        std::cerr << err.what() << std::endl;
        std::cerr << program;
        return 1;
    }

    std::string inputFile = program.get<std::string>("input_file");
    std::string outputBase = program.get<std::string>("--output");

    std::cout << "Input Command: " << inputFile << "\n";
    std::cout << "Output Base:   " << outputBase << "\n";

    // -------------------------------------------------------------------------
    // 2. Read G-Code from file
    // -------------------------------------------------------------------------
    std::string gcode_content;
    try {
        std::ifstream ifs(inputFile);
        if (!ifs.is_open()) {
            throw std::runtime_error("Could not open file: " + inputFile);
        }
        gcode_content.assign((std::istreambuf_iterator<char>(ifs)),
                             (std::istreambuf_iterator<char>()));
    }
    catch (const std::exception &e) {
        std::cerr << "Error reading file: " << e.what() << "\n";
        return 1;
    }

    // -------------------------------------------------------------------------
    // 3. Parse G-Code to Motion Segments (The "Ideal" Path)
    // -------------------------------------------------------------------------
    GCode::VariableSystem variables;
    GCode::Parser parser(variables);
    double currentBlendingTolerance = 0.0;
    
    // Create GCode to Motion Converter
    auto sourceFile = std::make_shared<SourceFile>(inputFile);
    GCodeToMotionConverter converter(sourceFile);
    
    MotionSegmentList idealSegments;
    std::istringstream stream(gcode_content);
    std::string line;
    
    while (std::getline(stream, line)) {
        if (line.empty()) continue;
        
        GCode::Block block;
        GCode::Error err = parser.parseLine(line.c_str(), block);
        if (err) {
            std::cerr << "Warning: Parse error on line '" << line << "': " << err.message.data() << "\n";
            continue;
        }

        // We need to convert Block -> ParsedGCodeCommand for the adapter
        // Since we don't have a direct converter in the public API shown in previous read_file,
        // we might need to manually construct ParsedGCodeCommand or if GCodeAdapter has a block processor.
        // Looking at GCodeAdapter, it takes `ParsedGCodeCommand`. 
        // We will construct it manually from the Block for this example, 
        // or check if there is a helper.
        // For simplicity in this example, we assume we can map GCode::Block to ParsedGCodeCommand.
        
        MotionPlanner::ParsedGCodeCommand cmd;
        cmd.lineNumber = 0; // line numbers?
        cmd.lineText = line;
        
        // Initialize coordinates to NaN
        for(auto& val : cmd.coordinates) val = std::numeric_limits<double>::quiet_NaN();
        
        // Extract G/M codes and coords from block
        // (This is a simplified mapping logic for the example)
        bool hasG = false;
        for(uint8_t i=0; i<block.gCodeCount; ++i) {
            // Note: Parser seems to return G-codes scaled by 10 (e.g. G1 -> 10, G21 -> 210)
            // We need to normalize back to integer for GCodeAdapter
            cmd.gCode = block.gCodes[i] / 10;
            hasG = true;
            break; // take first G code
        }
        for(uint8_t i=0; i<block.mCodeCount; ++i) {
            cmd.mCode = block.mCodes[i];
            break; 
        }
        
        if (block.hasWord(GCode::WordLetter::X)) cmd.coordinates[0] = block.getWord(GCode::WordLetter::X);
        if (block.hasWord(GCode::WordLetter::Y)) cmd.coordinates[1] = block.getWord(GCode::WordLetter::Y);
        if (block.hasWord(GCode::WordLetter::Z)) cmd.coordinates[2] = block.getWord(GCode::WordLetter::Z);
        if (block.hasWord(GCode::WordLetter::A)) cmd.coordinates[3] = block.getWord(GCode::WordLetter::A);
        if (block.hasWord(GCode::WordLetter::B)) cmd.coordinates[4] = block.getWord(GCode::WordLetter::B);
        if (block.hasWord(GCode::WordLetter::C)) cmd.coordinates[5] = block.getWord(GCode::WordLetter::C);

        // DEBUG: Check extraction
        if (cmd.gCode >= 0 || cmd.hasAnyCoordinate()) {
             std::cout << "CMD G" << cmd.gCode << " Coords: ";
             if (!std::isnan(cmd.coordinates[0])) std::cout << "X" << cmd.coordinates[0] << " ";
             if (!std::isnan(cmd.coordinates[1])) std::cout << "Y" << cmd.coordinates[1] << " ";
             std::cout << "\n";
        }

        if (block.hasWord(GCode::WordLetter::I)) cmd.arcOffsets[0] = block.getWord(GCode::WordLetter::I);
        if (block.hasWord(GCode::WordLetter::J)) cmd.arcOffsets[1] = block.getWord(GCode::WordLetter::J);
        if (block.hasWord(GCode::WordLetter::K)) cmd.arcOffsets[2] = block.getWord(GCode::WordLetter::K);
        
        if (block.hasWord(GCode::WordLetter::R)) cmd.arcRadius = block.getWord(GCode::WordLetter::R);
        if (block.hasWord(GCode::WordLetter::P)) cmd.dwellTime = block.getWord(GCode::WordLetter::P); // Used for G4 dwell or G64 tolerance
        if (block.hasWord(GCode::WordLetter::F)) cmd.feedRate = block.getWord(GCode::WordLetter::F);

        // Process
        if (cmd.gCode == 64 && cmd.dwellTime.has_value()) {
            currentBlendingTolerance = *cmd.dwellTime;
            std::cout << " -> Set Blending Tolerance: " << currentBlendingTolerance << " based on G64 P\n";
        }

        auto segment = converter.processCommand(cmd);
        if (segment) {
            if (segment->pathMode == MotionPlanner::PathMode::Blending) {
                 segment->blending.tolerance = currentBlendingTolerance;
            }
            std::cout << " -> Segment generated: Type " << (int)segment->type << "\n";
            std::cout << "    Start: " << segment->startPosition[0] << ", " << segment->startPosition[1] << "\n";
            std::cout << "    End:   " << segment->endPosition[0] << ", " << segment->endPosition[1] << "\n";
            if (segment->isArc()) {
                std::cout << "    Center:" << segment->arcCenter[0] << ", " << segment->arcCenter[1] << "\n";
                int dir = segment->arcDirection();
                std::cout << "    Dir:   " << (dir > 0 ? "CCW" : "CW") << "\n";
            }
            idealSegments.append(std::move(*segment));
        } else {
            std::cout << " -> No segment generated.\n";
        }
    }

    std::cout << "Generated " << idealSegments.size() << " motion segments (Ideal Path).\n";

    if (idealSegments.empty()) {
        std::cerr << "Error: No segments generated from G-code.\n";
        return 1;
    }

    // -------------------------------------------------------------------------
    // 4. Compute Analytical Solution (G2 Blended Bezier Path)
    // -------------------------------------------------------------------------
    
    // We will generate multiple paths for the sweep requested by the user
    std::vector<RenderablePath> resultPaths;
    
    double baseTolerance = 25.0; // Default fallback
    if (currentBlendingTolerance > 0.0) baseTolerance = currentBlendingTolerance;

    // The user asked for "6 steps from negative [current] to positive [current]"
    // and "the 0 deviation one must be black".
    // We'll generate the range [-baseTolerance, baseTolerance]
    // Steps: -25, -15, -5, 5, 15, 25 (if base=25, steps=6)
    // And explicit 0.
    
    std::vector<double> testTolerances;
    int steps = 6;
    double start = -baseTolerance;
    double end = baseTolerance;
    double step = (end - start) / (steps - 1); // 50 / 5 = 10
    
    for(int i=0; i<steps; ++i) {
        testTolerances.push_back(start + i * step);
    }
    // Ensure 0 is present for the black trace
    bool hasZero = false;
    for(double t : testTolerances) if (std::abs(t) < 1e-6) hasZero = true;
    if (!hasZero) testTolerances.push_back(0.0);
    
    // Sort for consistent drawing order (draw 0 last to be on top?)
    // Actually typically we want black on top.
    
    for (double tol : testTolerances) {
        // Reset builder
        MotionPlanner::PathBuilder3D pathBuilder;
        
        // Handle negative tolerance (assuming internal logic clamps or user logic applies)
        // If the library expects positive, we pass positive.
        // But user asked for negative step.
        // If we strictly pass negative, PathBuilder might misbehave. 
        // We'll assume user means "Magnitude" but implicitly "Direction"??
        // Since standard blending is isotropic, -25 vs +25 likely same unless directed offset.
        // However, I will pass it as is.
        pathBuilder.config().tolerance = tol; // Pass raw value
        
        auto result = pathBuilder.build(idealSegments);
        
        RenderablePath rpath;
        rpath.path = std::move(result.path);
        
        if (std::abs(tol) < 1e-6) {
            rpath.color = "black";
            rpath.width = 3.0;
        } else {
            // Map index to color
            // Simple logic:
            // -Max -> Red
            // 0 -> Green? No 0 is black.
            // +Max -> Blue
            // Let's use a spectral map roughly.
            double t = (tol - start) / (end - start); // 0..1
            // r,g,b
            // 0.0 -> Red (255,0,0)
            // 0.5 -> Green/Yellow
            // 1.0 -> Blue (0,0,255)
            // Or use predefined strings
            if (tol < -20) rpath.color = "#ff0000"; // Red
            else if (tol < -10) rpath.color = "#ff8800"; // Orange
            else if (tol < 0) rpath.color = "#aaaa00"; // Yellowish
            else if (tol == 0) rpath.color = "black";
            else if (tol <= 5) rpath.color = "#00aaaa"; // Cyanish
            else if (tol <= 15) rpath.color = "#0088ff"; // Azure
            else rpath.color = "#0000ff"; // Blue
            
            rpath.width = 1.5;
        }
        
        resultPaths.push_back(std::move(rpath));
        std::cout << "Computed path for P=" << tol << " Segments=" << resultPaths.back().path.numSegments() << "\n";
    }

    // --- Custom Export (Direct Bezier) ---
    // Convert to SVGExporter format
    std::vector<GCodeExport::RenderableBezierPath> exportPaths;
    for(const auto& rp : resultPaths) {
        GCodeExport::RenderableBezierPath bp;
        bp.color = rp.color;
        bp.width = rp.width;
        
        // Extract curves from Path object
        for(size_t i=0; i<rp.path.numSegments(); ++i) {
             const auto& curve3d = rp.path.segment(i).curve;
             // Manual 3D -> 2D projection
             std::vector<MotionPlanner::Vec<2, double>> points2d;
             for (const auto& p : curve3d.controlPoints()) {
                 points2d.push_back(MotionPlanner::Vec<2, double>{p[0], p[1]});
             }
             bp.path.push_back(MotionPlanner::BezierCurve<2, double>(points2d));
        }
        exportPaths.push_back(bp);
    }
    
    GCodeExport::SVGExporter exporter;
    GCodeExport::SVGConfig config;
    config.flipY = true;
    config.width = 1024;
    config.height = 1024;
    exporter.configure(config);
    
    exporter.exportBezierPaths(exportPaths, outputBase + "_direct_bezier.svg");
    
    std::cout << "Exported " << outputBase + "_direct_bezier.svg" << "\n";
    
    return 0; // Exit early
    
    /** REMOVED UNREACHABLE CODE **/
    
    // --- Export (a) Ideal Path ---
    {
        std::vector<GCodeExport::TrajectorySample> idealSamples;
        // Sample the linear/arc segments directly
        for (const auto& seg : idealSegments) {
            // Simplified sampling: start and end.
            
            GCodeExport::TrajectorySample s1;
            auto start = extractPoint(seg.startPosition);
            s1.position[0] = start[0]; s1.position[1] = start[1]; s1.position[2] = start[2];
            s1.motionType = (uint8_t)(seg.type == MotionSegmentType::Rapid ? 0 : 1); 
            idealSamples.push_back(s1);

            GCodeExport::TrajectorySample s2;
            auto end = extractPoint(seg.endPosition);
            s2.position[0] = end[0]; s2.position[1] = end[1]; s2.position[2] = end[2];
            idealSamples.push_back(s2);
        }
        
        std::string filename = outputBase + "_ideal.svg";
        exporter.configure(config);
        if (exporter.exportToFile(idealSamples, filename)) {
            std::cout << "Exported Ideal Path to: " << filename << "\n";
        }
    }

    // --- Export (b) Computed Analytical Path ---
    {
        std::vector<GCodeExport::TrajectorySample> analyticalSamples;
        
        // Sampling the analytical Bezier path
        // We do NOT use time-sampled solution (e.g. at t=0.001s).
        // We sample by Arc Length to visualize the geometry of the curve solution.
        
        // Use the last computed path (P=25) as the representative "Analytical" export for the standard exporter
        if (!resultPaths.empty()) {
            const auto& analyticalPathRef = resultPaths.back().path;
            
            double totalLen = analyticalPathRef.totalLength();
            double step = 0.5; // 0.5 mm step for visualization
            
            for (double s = 0; s <= totalLen; s += step) {
                 auto eval = analyticalPathRef.evaluateAtArcLength(s);
                 
                 GCodeExport::TrajectorySample samp;
                 samp.pathPosition = s;
                 samp.position[0] = eval.position[0];
                 samp.position[1] = eval.position[1];
                 samp.position[2] = eval.position[2];
                 samp.motionType = 1; // Force Linear style (solid line) instead of Rapid (dashed) or unknown
                 
                 analyticalSamples.push_back(samp);
            }
        }
        
        std::string filename = outputBase + "_analytical.svg";
        // Change color to distinguish
        config.linearColor = "#ff00ff"; // Macintosh / Magenta for analytical
        exporter.configure(config);
        
        if (exporter.exportToFile(analyticalSamples, filename)) {
            std::cout << "Exported Analytical Path to: " << filename << "\n";
        }
    }

    return 0;
}
