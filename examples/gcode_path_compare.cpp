/**
 * @file gcode_path_compare.cpp
 * @brief Example: parse a G-code file, optionally blend corners with G64
 *        path deviation tolerance, and export an SVG showing the original
 *        G-code path (gray) overlaid with the Tether-planned path (red).
 *
 * This example demonstrates the Tether path-planning pipeline:
 *   1. Parse G-code → PlanningSegments
 *   2. Convert segments → PiecewiseNurbsPath (raw, sharp corners)
 *   3. Optionally blend corners via PathBlender with a G64 deviation
 *      tolerance (or --exact-stop for G61 mode)
 *   4. Export both the raw G-code path and the planned path to a single SVG
 *
 * The SVG uses:
 *   - Gray for the original G-code path (from PlanningSegments)
 *   - Red for the Tether-planned path (blended or exact-stop)
 *
 * Usage:
 *   gcode_path_compare <gcode_file> [options]
 *
 * Options:
 *   -o, --output FILE       Output SVG file (default: path_compare.svg)
 *   --g64-tolerance TOL     G64 path deviation tolerance in mm (default: 0.05)
 *   --exact-stop            G61 exact stop mode (no corner blending)
 */

#include <argparse/argparse.hpp>

#include <tether/gcode/PlanningSegmentBuilder.hpp>
#include <tether/gcode/GCodeInterpreter.hpp>
#include <tether/motion_planner/geometry/PiecewiseNurbsPath.hpp>
#include <tether/motion_planner/geometry/PlanningSegmentConverter.hpp>
#include <tether/motion_planner/blend/PathBlender.hpp>
#include <tether/motion_planner/blend/BlendSpec.hpp>
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
/// Each segment becomes a single linear NurbsCurve (degree 1), XY projection.
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
        tether::motion::RVec start{seg.start[0], seg.start[1]};
        tether::motion::RVec end{seg.end[0], seg.end[1]};
        auto curve = tether::motion::NurbsCurve::fromLine(start, end);
        curves.push_back(std::move(curve));
    }
    return curves;
}

} // anonymous namespace

int main(int argc, char** argv) {
    argparse::ArgumentParser program("gcode_path_compare");
    program.add_description(
        "Parse a G-code file, optionally blend corners with G64 path "
        "deviation tolerance, and export an SVG showing the original "
        "G-code path (gray) overlaid with the Tether-planned path (red).");

    program.add_argument("gcode_file")
        .help("Input G-code file path");

    program.add_argument("-o", "--output")
        .default_value(std::string("path_compare.svg"))
        .help("Output SVG file path (default: path_compare.svg)");

    program.add_argument("--g64-tolerance")
        .default_value(0.05)
        .scan<'g', double>()
        .help("G64 path deviation tolerance in mm (default: 0.05)");

    program.add_argument("--exact-stop")
        .flag()
        .help("G61 exact stop mode — no corner blending");

    try {
        program.parse_args(argc, argv);
    } catch (const std::runtime_error& err) {
        std::cerr << err.what() << "\n" << program;
        return 1;
    }

    std::string gcodeFile = program.get<std::string>("gcode_file");
    std::string outputFile = program.get<std::string>("--output");
    bool exactStop = program.get<bool>("--exact-stop");
    double g64Tolerance = program.get<double>("--g64-tolerance");

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
    if (exactStop) {
        std::cout << "Mode: exact stop (G61, no blending)\n";
    } else {
        std::cout << "Mode: G64 blending, tolerance = " << g64Tolerance << " mm\n";
    }

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

    // ── Step 2: Build raw NURBS path from segments ──
    auto nurbsResult = tether::motion::piecewiseNurbsFromSegments(segments);
    std::cout << "Raw NURBS path: " << nurbsResult.path.numPieces() << " pieces, "
              << nurbsResult.path.totalLength() << " mm\n";

    // ── Step 3: Blend corners (or exact stop) ──
    // The planned path is either:
    //   - The blended path (G64 with deviation tolerance), or
    //   - The raw path as-is (G61 exact stop)
    std::vector<tether::motion::NurbsCurve> plannedPieces;

    if (exactStop) {
        // No blending — use the raw pieces directly
        for (std::size_t i = 0; i < nurbsResult.path.numPieces(); ++i) {
            plannedPieces.push_back(nurbsResult.path.piece(i));
        }
        std::cout << "Planned path: " << plannedPieces.size()
                  << " pieces (exact stop, no blending)\n";
    } else {
        // Blend corners with PathBlender
        tether::motion::BlendSpec blendSpec;
        blendSpec.mode = tether::motion::PathMode::Blend;
        blendSpec.tolerance = g64Tolerance;

        tether::motion::PathBlender blender;
        auto blended = blender.blend(nurbsResult.path, blendSpec);

        plannedPieces = blended.pieces;
        std::cout << "Planned path: " << plannedPieces.size()
                  << " pieces (blended: " << blended.blendedCount
                  << " corners, " << blended.exactStopCount
                  << " exact-stop fallbacks, "
                  << blended.straightCount << " straight)\n";
    }

    if (plannedPieces.empty()) {
        std::cerr << "ERROR: Planned path is empty.\n";
        return 1;
    }

    // ── Step 4: Export SVG with both paths overlaid ──
    // Gray: raw G-code path (from PlanningSegments)
    // Red:  Tether-planned path (blended or exact-stop)
    std::vector<RenderableBezierPath> paths;

    // Gray path: raw G-code segments as linear NURBS curves
    auto rawCurves = segmentsToNurbsCurves(segments);
    RenderableBezierPath grayPath;
    grayPath.path = std::move(rawCurves);
    grayPath.color = "#999999";
    grayPath.width = 2.0;
    paths.push_back(std::move(grayPath));

    // Red path: planned path (decomposed into Bézier curves for SVG)
    std::vector<tether::motion::NurbsCurve> plannedBeziers;
    for (const auto& piece : plannedPieces) {
        auto decomposed = piece.bezierDecompose();
        for (auto& p : decomposed) {
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
