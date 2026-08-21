/**
 * @file gcode_path_compare.cpp
 * @brief Example: parse a G-code file, plan its motion, and export an SVG
 *        showing the original G-code path (gray) overlaid with the planned
 *        NURBS path (red).
 *
 * This example demonstrates the full Tether pipeline:
 *   1. Parse G-code → PlanningSegments
 *   2. Convert segments → PiecewiseNurbsPath (the "planned" path)
 *   3. Export both the raw G-code segments and the NURBS path to a single SVG
 *
 * The SVG uses:
 *   - Gray for the original G-code path (from PlanningSegments)
 *   - Red for the Tether-planned NURBS path
 *
 * Usage:
 *   gcode_path_compare <gcode_file> [-o <output.svg>] \
 *       [--max-velocity V] [--max-acceleration A] [--max-jerk J]
 *
 * Defaults: v=200 mm/s, a=2000 mm/s², j=20000 mm/s³
 */

#include <tether/gcode/PlanningSegmentBuilder.hpp>
#include <tether/gcode/GCodeInterpreter.hpp>
#include <tether/motion_planner/geometry/PiecewiseNurbsPath.hpp>
#include <tether/motion_planner/geometry/PlanningSegmentConverter.hpp>
#include <tether/motion_planner/PathAdapter.hpp>
#include <tether/motion_planner/analytical/ParetoTimeEnergyOptimalVelocityPlanner.hpp>
#include <tether/export/SVGExporter.hpp>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

using GCode::PlanningSegment;
using GCodeExport::SVGExporter;
using GCodeExport::SVGConfig;
using GCodeExport::RenderableBezierPath;

namespace {

/// Pre-filter Klipper extended commands that the Tether parser doesn't
/// understand (same logic as GCodeProcessor / wss_diag).
bool isKlipperCommand(std::string_view line) {
    size_t i = 0;
    while (i < line.size() && (line[i] == ' ' || line[i] == '\t')) i++;
    if (i >= line.size()) return false;
    if (line[i] == ';' || line[i] == '(') return false;
    char c0 = line[i];
    if (i + 1 < line.size()) {
        char c1 = line[i + 1];
        if (std::isdigit(static_cast<unsigned char>(c1)) ||
            c1 == ' ' || c1 == '\t' || c1 == '\r' || c1 == '\n') return false;
    }
    if (c0 < 'A' || c0 > 'Z') return false;
    size_t cmdEnd = i;
    while (cmdEnd < line.size() && ((line[cmdEnd] >= 'A' && line[cmdEnd] <= 'Z') || line[cmdEnd] == '_')) cmdEnd++;
    if (cmdEnd - i < 2) return false;
    if (cmdEnd < line.size()) {
        char after = line[cmdEnd];
        if (after != ' ' && after != '\t' && after != '\r' && after != '\n' && after != '=') return false;
    }
    return true;
}

bool hasAxisWordsWithoutValues(std::string_view line) {
    size_t commentPos = line.find_first_of(";(");
    if (commentPos != std::string_view::npos)
        line = line.substr(0, commentPos);
    size_t i = 0;
    while (i < line.size() && (line[i] == ' ' || line[i] == '\t')) i++;
    if (i >= line.size()) return false;
    if (line[i] != 'M' && line[i] != 'G') return false;
    i++;
    while (i < line.size() && std::isdigit(static_cast<unsigned char>(line[i]))) i++;
    bool foundWordWithoutValue = false;
    while (i < line.size()) {
        char c = line[i];
        if ((c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z')) {
            if (c == 'E' || c == 'X' || c == 'Y' || c == 'Z' || c == 'F' || c == 'S' || c == 'P' || c == 'R' || c == 'T') {
                i++;
                if (i < line.size() && (std::isdigit(static_cast<unsigned char>(line[i])) || line[i] == '.' || line[i] == '-' || line[i] == '+')) {
                    while (i < line.size() && (std::isdigit(static_cast<unsigned char>(line[i])) || line[i] == '.' || line[i] == '-' || line[i] == '+' || line[i] == 'e' || line[i] == 'E')) i++;
                } else {
                    foundWordWithoutValue = true;
                }
            } else {
                i++;
            }
        } else {
            i++;
        }
    }
    return foundWordWithoutValue;
}

std::string filterKlipperCommands(const std::string& gcodeText) {
    std::string result;
    result.reserve(gcodeText.size());
    size_t pos = 0;
    while (pos < gcodeText.size()) {
        size_t lineEnd = gcodeText.find('\n', pos);
        std::string_view line = (lineEnd == std::string::npos)
            ? std::string_view(gcodeText.data() + pos)
            : std::string_view(gcodeText.data() + pos, lineEnd - pos);
        if (isKlipperCommand(line) || hasAxisWordsWithoutValues(line)) {
            result += "; [filtered] ";
            result += line;
        } else {
            result += line;
        }
        if (lineEnd != std::string::npos) { result += '\n'; pos = lineEnd + 1; }
        else break;
    }
    return result;
}

/// Convert PlanningSegments to NurbsCurves for the gray "raw G-code" path.
/// Each segment becomes a single linear NurbsCurve (degree 1).
std::vector<tether::motion::NurbsCurve> segmentsToNurbsCurves(
    const std::vector<PlanningSegment>& segments)
{
    std::vector<tether::motion::NurbsCurve> curves;
    curves.reserve(segments.size());
    for (const auto& seg : segments) {
        // Skip segments with zero XY displacement (e.g. Z-only moves).
        double dx = seg.end[0] - seg.start[0];
        double dy = seg.end[1] - seg.start[1];
        if (std::hypot(dx, dy) < 1e-12) continue;
        // Create a degree-1 NURBS (line) from start to end.
        // Use only X/Y for the 2D SVG projection.
        tether::motion::RVec start{seg.start[0], seg.start[1]};
        tether::motion::RVec end{seg.end[0], seg.end[1]};
        auto curve = tether::motion::NurbsCurve::fromLine(start, end);
        curves.push_back(std::move(curve));
    }
    return curves;
}

} // anonymous namespace

int main(int argc, char* argv[]) {
    if (argc < 2) {
        std::cerr << "Usage: " << argv[0]
                  << " <gcode_file> [-o <output.svg>]"
                  << " [--max-velocity V] [--max-acceleration A] [--max-jerk J]\n"
                  << "\nDefaults: v=200 mm/s, a=2000 mm/s², j=20000 mm/s³\n";
        return 1;
    }

    std::string gcodeFile = argv[1];
    std::string outputFile = "path_compare.svg";
    double maxVelocity = 200.0;
    double maxAcceleration = 2000.0;
    double maxJerk = 20000.0;

    for (int i = 2; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "-o" && i + 1 < argc) {
            outputFile = argv[++i];
        } else if (arg == "--max-velocity" && i + 1 < argc) {
            maxVelocity = std::stod(argv[++i]);
        } else if (arg == "--max-acceleration" && i + 1 < argc) {
            maxAcceleration = std::stod(argv[++i]);
        } else if (arg == "--max-jerk" && i + 1 < argc) {
            maxJerk = std::stod(argv[++i]);
        }
    }

    // ── Read G-code file ──
    std::ifstream file(gcodeFile);
    if (!file) {
        std::cerr << "ERROR: Cannot open " << gcodeFile << "\n";
        return 1;
    }
    std::stringstream ss;
    ss << file.rdbuf();
    std::string gcodeText = ss.str();
    std::cout << "G-code file: " << gcodeFile << " (" << gcodeText.size() << " bytes)\n";
    std::cout << "Limits: v=" << maxVelocity << " a=" << maxAcceleration << " j=" << maxJerk << "\n";

    // ── Step 1: Parse G-code → PlanningSegments ──
    std::string filtered = filterKlipperCommands(gcodeText);
    auto parseResult = GCode::PlanningSegmentBuilder::fromText(filtered);
    if (!parseResult.error.ok()) {
        std::cerr << "Parse failed, retrying with stopOnError=false\n";
        GCode::InterpreterConfig retryConfig;
        retryConfig.stopOnError = false;
        auto retryResult = GCode::PlanningSegmentBuilder::fromText(filtered, retryConfig);
        if (!retryResult.segments.empty()) {
            parseResult = std::move(retryResult);
        }
    }
    auto& segments = parseResult.segments;
    std::cout << "Parsed " << segments.size() << " segments\n";

    if (segments.empty()) {
        std::cerr << "ERROR: No motion segments found.\n";
        return 1;
    }

    // ── Step 2: Build NURBS path (the "planned" path) ──
    auto nurbsResult = tether::motion::piecewiseNurbsFromSegments(segments);
    std::cout << "NURBS path: " << nurbsResult.path.numPieces() << " pieces, "
              << nurbsResult.path.totalLength() << " mm\n";

    // ── Step 3: Run the Pareto planner to verify the path is plannable ──
    // (This also validates that the WSS can be generated.)
    MotionPlanner::PathAdapter<3, double> pathAdapter(nurbsResult.path);
    if (!nurbsResult.feedRates.empty()) {
        pathAdapter.setSegmentVelocityLimits(nurbsResult.feedRates);
        pathAdapter.computeCornerVelocities(0.05, maxAcceleration);
    }

    MotionPlanner::KinematicLimits<3, double> limits;
    limits.path.maxPathVelocity = maxVelocity;
    limits.path.maxPathAcceleration = maxAcceleration;
    limits.path.maxPathJerk = maxJerk;
    limits.path.jerkLimitEnabled = (maxJerk > 0.0);
    for (int i = 0; i < 3; ++i) {
        limits.axis.maxVelocity[i] = maxVelocity;
        limits.axis.maxAcceleration[i] = maxAcceleration;
        limits.axis.maxJerk[i] = maxJerk;
    }
    limits.axis.jerkLimitEnabled = limits.path.jerkLimitEnabled;

    MotionPlanner::analytical::ParetoTimeEnergyOptimalVelocityPlanner<3, double> profiler(limits);
    std::size_t numSamples = std::min<std::size_t>(
        20000, std::max<std::size_t>(200, pathAdapter.numSegments() * 20));
    auto velocityProfile = profiler.computeProfile(
        pathAdapter, maxVelocity, 0.0, 0.0, numSamples);

    if (!velocityProfile) {
        std::cerr << "WARNING: ParetoPlanner returned null velocity profile.\n";
    } else {
        auto wss = profiler.weightedSource();
        if (wss) {
            std::cout << "WSS: " << wss->arcs().size() << " arcs, "
                      << "totalTime=" << wss->totalTime() << "s\n";
        }
    }

    // ── Step 4: Export SVG with both paths overlaid ──
    // Gray: raw G-code path (from PlanningSegments)
    // Red:  Tether-planned NURBS path
    std::vector<RenderableBezierPath> paths;

    // Gray path: raw G-code segments as linear NURBS curves
    auto rawCurves = segmentsToNurbsCurves(segments);
    RenderableBezierPath grayPath;
    grayPath.path = std::move(rawCurves);
    grayPath.color = "#999999";
    grayPath.width = 2.0;
    paths.push_back(std::move(grayPath));

    // Red path: planned NURBS path (decomposed into Bézier curves)
    std::vector<tether::motion::NurbsCurve> plannedBeziers;
    for (std::size_t i = 0; i < nurbsResult.path.numPieces(); ++i) {
        auto pieces = nurbsResult.path.piece(i).bezierDecompose();
        for (auto& p : pieces) {
            plannedBeziers.push_back(std::move(p));
        }
    }
    RenderableBezierPath redPath;
    redPath.path = std::move(plannedBeziers);
    redPath.color = "#cc0000";
    redPath.width = 1.0;
    paths.push_back(std::move(redPath));

    SVGConfig cfg;
    cfg.width = 1200;
    cfg.height = 1200;
    cfg.flipY = true;
    cfg.showGrid = true;
    cfg.gridSpacing = 10.0;
    cfg.gridColor = "#eeeeee";

    SVGExporter exporter(cfg);
    if (exporter.exportBezierPaths(paths, outputFile)) {
        std::cout << "Exported path comparison to: " << outputFile << "\n";
        return 0;
    }

    std::cerr << "ERROR: Failed to export SVG\n";
    return 1;
}
