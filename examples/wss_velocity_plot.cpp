/**
 * @file wss_velocity_plot.cpp
 * @brief Example: parse a G-code file, run the Pareto time-energy optimal
 *        velocity planner, and export velocity and acceleration curves as
 *        SVG plots from the resulting WSS (Weighted Switching Structure).
 *
 * This example demonstrates:
 *   1. Parse G-code → PlanningSegments
 *   2. Convert segments → PiecewiseNurbsPath
 *   3. Run ParetoTimeEnergyOptimalVelocityPlanner → WSS arcs
 *   4. Evaluate velocity and acceleration analytically at sampled times
 *   5. Export two SVG plots: velocity(t) and acceleration(t)
 *
 * The WSS arcs are evaluated in closed form:
 *   BANG:     v(τ) = v0 + a0·τ + ½·η·τ²,  a(τ) = a0 + η·τ
 *   SINGULAR: v(τ) = v0 + a*·τ,           a(τ) = a*
 *   WALL:     v(τ) = v0 (constant),        a(τ) = 0
 *   DWELL:    v = 0, a = 0
 *
 * Usage:
 *   wss_velocity_plot <gcode_file> [-o <output_prefix>] \
 *       [--max-velocity V] [--max-acceleration A] [--max-jerk J] \
 *       [--samples N] [--g64-tolerance T] [--transition-fraction F]
 *
 * Path blending (default: exact stop, no blending):
 *   --g64-tolerance T        G64 deviation tolerance (mm). Positive = inside
 *                            Bézier blend, negative = outside circle blend
 *                            (radius = |T|). If not specified, exact stop.
 *   --transition-fraction F  Transition fraction for G2 outside blend
 *                            (default: 0.5). 0 = G1 only.
 *
 * Outputs:
 *   <prefix>_velocity.svg     — velocity vs time plot (mm/s vs s)
 *   <prefix>_acceleration.svg — acceleration vs time plot (mm/s² vs s)
 *
 * Defaults: v=200 mm/s, a=2000 mm/s², j=20000 mm/s³, samples=2000, exact stop
 */

#include <argparse/argparse.hpp>

#include <tether/gcode/PlanningSegmentBuilder.hpp>
#include <tether/gcode/GCodeInterpreter.hpp>
#include <tether/motion_planner/geometry/PiecewiseNurbsPath.hpp>
#include <tether/motion_planner/geometry/PlanningSegmentConverter.hpp>
#include <tether/motion_planner/blend/PathBlender.hpp>
#include <tether/motion_planner/blend/BlendSpec.hpp>
#include <tether/motion_planner/blend/OutsideCircleBlender.hpp>
#include <tether/motion_planner/PathAdapter.hpp>
#include <tether/motion_planner/analytical/ParetoTimeEnergyOptimalVelocityPlanner.hpp>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <string>
#include <vector>

using MotionPlanner::analytical::WeightedArc;
using MotionPlanner::analytical::WeightedArcType;

namespace {

/// Pre-filter Klipper extended commands (same as gcode_path_compare / wss_diag).
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

/// Evaluate path velocity at time t from WSS arcs (closed-form).
/// BANG:     v(τ) = v0 + a0·τ + ½·η·τ²
/// SINGULAR: v(τ) = v0 + a*·τ
/// WALL:     v(τ) = v0
/// DWELL:    v = 0
double evalVelocity(const WeightedArc& arc, double tau) {
    tau = std::clamp(tau, 0.0, arc.duration);
    switch (arc.type) {
        case WeightedArcType::BANG_PLUS:
        case WeightedArcType::BANG_MINUS:
            return arc.v0 + arc.a0 * tau + 0.5 * arc.eta * tau * tau;
        case WeightedArcType::SINGULAR:
            return arc.v0 + arc.a_star * tau;
        case WeightedArcType::WALL:
            return arc.v0;
        case WeightedArcType::DWELL:
            return 0.0;
    }
    return 0.0;
}

/// Evaluate path acceleration at time t from WSS arcs (closed-form).
/// BANG:     a(τ) = a0 + η·τ
/// SINGULAR: a(τ) = a*
/// WALL:     a(τ) = 0
/// DWELL:    a = 0
double evalAcceleration(const WeightedArc& arc, double tau) {
    tau = std::clamp(tau, 0.0, arc.duration);
    switch (arc.type) {
        case WeightedArcType::BANG_PLUS:
        case WeightedArcType::BANG_MINUS:
            return arc.a0 + arc.eta * tau;
        case WeightedArcType::SINGULAR:
            return arc.a_star;
        case WeightedArcType::WALL:
            return 0.0;
        case WeightedArcType::DWELL:
            return 0.0;
    }
    return 0.0;
}

/// Find the arc containing time t via binary search on t0.
const WeightedArc& findArc(const std::vector<WeightedArc>& arcs, double t) {
    size_t lo = 0, hi = arcs.size() - 1;
    while (lo < hi) {
        size_t mid = (lo + hi + 1) / 2;
        if (arcs[mid].t0 <= t) lo = mid;
        else hi = mid - 1;
    }
    return arcs[lo];
}

/// Sample (time, value) pairs from the WSS arcs.
struct Sample { double t; double v; double a; };

std::vector<Sample> sampleWss(const std::vector<WeightedArc>& arcs,
                               double totalTime, size_t numSamples)
{
    std::vector<Sample> samples;
    samples.reserve(numSamples);
    if (arcs.empty() || totalTime <= 0.0) return samples;

    for (size_t i = 0; i < numSamples; ++i) {
        double t = totalTime * static_cast<double>(i) / static_cast<double>(numSamples - 1);
        const auto& arc = findArc(arcs, t);
        double tau = t - arc.t0;
        double v = evalVelocity(arc, tau);
        double a = evalAcceleration(arc, tau);
        samples.push_back({t, v, a});
    }
    return samples;
}

/// Write a 2D line plot as SVG.
/// Renders axes, grid, labels, and a polyline for the data.
bool writePlotSvg(const std::string& filename,
                  const std::string& title,
                  const std::string& yLabel,
                  const std::string& xLabel,
                  const std::vector<Sample>& samples,
                  double Sample::* yField,
                  const std::string& lineColor,
                  double yMinHint = 0.0,
                  double yMaxHint = 0.0)
{
    if (samples.empty()) return false;

    // Compute data ranges
    double tMin = samples.front().t;
    double tMax = samples.back().t;
    double yMin = yMinHint, yMax = yMaxHint;
    if (yMin == 0.0 && yMax == 0.0) {
        yMin = std::numeric_limits<double>::max();
        yMax = std::numeric_limits<double>::lowest();
        for (const auto& s : samples) {
            double y = s.*yField;
            if (y < yMin) yMin = y;
            if (y > yMax) yMax = y;
        }
    }
    if (yMin == yMax) { yMin -= 1; yMax += 1; }
    // Add 5% padding
    double yRange = yMax - yMin;
    yMin -= yRange * 0.05;
    yMax += yRange * 0.05;

    // SVG layout
    const double W = 1200, H = 600;
    const double marginL = 80, marginR = 40, marginT = 50, marginB = 60;
    const double plotW = W - marginL - marginR;
    const double plotH = H - marginT - marginB;

    auto sx = [&](double t) {
        return marginL + (t - tMin) / (tMax - tMin) * plotW;
    };
    auto sy = [&](double y) {
        return marginT + (1.0 - (y - yMin) / (yMax - yMin)) * plotH;
    };

    // "Nice" tick step
    auto niceStep = [](double range, int targetTicks) -> double {
        double raw = range / targetTicks;
        double mag = std::pow(10.0, std::floor(std::log10(raw)));
        double norm = raw / mag;
        double step;
        if (norm < 1.5) step = 1;
        else if (norm < 3) step = 2;
        else if (norm < 7) step = 5;
        else step = 10;
        return step * mag;
    };

    double xStep = niceStep(tMax - tMin, 10);
    double yStep = niceStep(yMax - yMin, 8);

    std::ofstream out(filename);
    if (!out) return false;

    out << std::fixed << std::setprecision(3);
    out << "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n";
    out << "<svg xmlns=\"http://www.w3.org/2000/svg\" "
        << "width=\"" << W << "\" height=\"" << H << "\" "
        << "viewBox=\"0 0 " << W << " " << H << "\">\n";
    out << "  <rect width=\"100%\" height=\"100%\" fill=\"#1a1a2e\"/>\n";

    // Title
    out << "  <text x=\"" << (W / 2) << "\" y=\"25\" "
        << "fill=\"#e0e0e0\" font-size=\"16\" font-family=\"sans-serif\" "
        << "text-anchor=\"middle\">" << title << "</text>\n";

    // Grid + Y tick labels
    out << "  <g stroke=\"#2a2a4e\" stroke-width=\"0.5\">\n";
    for (double y = std::ceil(yMin / yStep) * yStep; y <= yMax; y += yStep) {
        double py = sy(y);
        out << "    <line x1=\"" << marginL << "\" y1=\"" << py
            << "\" x2=\"" << (W - marginR) << "\" y2=\"" << py << "\"/>\n";
    }
    for (double t = std::ceil(tMin / xStep) * xStep; t <= tMax; t += xStep) {
        double px = sx(t);
        out << "    <line x1=\"" << px << "\" y1=\"" << marginT
            << "\" x2=\"" << px << "\" y2=\"" << (H - marginB) << "\"/>\n";
    }
    out << "  </g>\n";

    // Axes
    out << "  <g stroke=\"#666688\" stroke-width=\"1.0\" fill=\"none\">\n";
    out << "    <line x1=\"" << marginL << "\" y1=\"" << marginT
        << "\" x2=\"" << marginL << "\" y2=\"" << (H - marginB) << "\"/>\n";
    out << "    <line x1=\"" << marginL << "\" y1=\"" << (H - marginB)
        << "\" x2=\"" << (W - marginR) << "\" y2=\"" << (H - marginB) << "\"/>\n";
    // Zero line if in range
    if (yMin < 0 && yMax > 0) {
        out << "    <line x1=\"" << marginL << "\" y1=\"" << sy(0)
            << "\" x2=\"" << (W - marginR) << "\" y2=\"" << sy(0)
            << "\" stroke=\"#555577\" stroke-dasharray=\"4,4\"/>\n";
    }
    out << "  </g>\n";

    // Tick labels
    out << "  <g fill=\"#9999bb\" font-size=\"11\" font-family=\"monospace\">\n";
    for (double y = std::ceil(yMin / yStep) * yStep; y <= yMax; y += yStep) {
        double py = sy(y);
        out << "    <text x=\"" << (marginL - 8) << "\" y=\"" << (py + 4)
            << "\" text-anchor=\"end\">" << y << "</text>\n";
    }
    for (double t = std::ceil(tMin / xStep) * xStep; t <= tMax; t += xStep) {
        double px = sx(t);
        out << "    <text x=\"" << px << "\" y=\"" << (H - marginB + 18)
            << "\" text-anchor=\"middle\">" << t << "</text>\n";
    }
    out << "  </g>\n";

    // Axis labels
    out << "  <text x=\"" << (marginL - 50) << "\" y=\"" << (H / 2)
        << "\" fill=\"#ccccdd\" font-size=\"13\" font-family=\"sans-serif\" "
        << "text-anchor=\"middle\" transform=\"rotate(-90 "
        << (marginL - 50) << " " << (H / 2) << ")\">" << yLabel << "</text>\n";
    out << "  <text x=\"" << (W / 2) << "\" y=\"" << (H - 15)
        << "\" fill=\"#ccccdd\" font-size=\"13\" font-family=\"sans-serif\" "
        << "text-anchor=\"middle\">" << xLabel << "</text>\n";

    // Data polyline
    out << "  <polyline points=\"";
    for (size_t i = 0; i < samples.size(); ++i) {
        if (i > 0) out << " ";
        out << sx(samples[i].t) << "," << sy(samples[i].*yField);
    }
    out << "\" fill=\"none\" stroke=\"" << lineColor
        << "\" stroke-width=\"1.5\"/>\n";

    out << "</svg>\n";
    return out.good();
}

} // anonymous namespace

int main(int argc, char* argv[]) {
    argparse::ArgumentParser program("wss_velocity_plot");
    program.add_description(
        "Parse a G-code file, run the Pareto time-energy optimal velocity "
        "planner, and export velocity(t) and acceleration(t) as SVG plots "
        "by evaluating WSS arcs in closed form.\n\n"
        "Outputs: <prefix>_velocity.svg, <prefix>_acceleration.svg");

    program.add_argument("gcode_file")
        .help("Input G-code file path");

    program.add_argument("-o", "--output")
        .default_value(std::string("wss_plot"))
        .help("Output file prefix (default: wss_plot)");

    program.add_argument("--max-velocity")
        .default_value(200.0)
        .scan<'g', double>()
        .help("Max path velocity in mm/s (default: 200)");

    program.add_argument("--max-acceleration")
        .default_value(2000.0)
        .scan<'g', double>()
        .help("Max path acceleration in mm/s² (default: 2000)");

    program.add_argument("--max-jerk")
        .default_value(20000.0)
        .scan<'g', double>()
        .help("Max path jerk in mm/s³ (default: 20000)");

    program.add_argument("--samples")
        .default_value(2000)
        .scan<'i', int>()
        .help("Number of sample points (default: 2000)");

    program.add_argument("--g64-tolerance")
        .scan<'g', double>()
        .help("G64 path deviation tolerance in mm. Positive = inside "
              "Bézier blend, negative = outside circle blend (radius = |T|). "
              "If not specified, exact stop (no blending).");

    program.add_argument("--transition-fraction")
        .default_value(0.5)
        .scan<'g', double>()
        .help("Transition fraction for G2 outside blend (default: 0.5). "
              "Fraction of blend radius used for quintic transition curves. "
              "0 = G1 only (tangent continuous), >0 = G2 (curvature continuous). "
              "Only used with negative --g64-tolerance.");

    try {
        program.parse_args(argc, argv);
    } catch (const std::runtime_error& err) {
        std::cerr << err.what() << "\n" << program;
        return 1;
    }

    std::string gcodeFile = program.get<std::string>("gcode_file");
    std::string outputPrefix = program.get<std::string>("--output");
    double maxVelocity = program.get<double>("--max-velocity");
    double maxAcceleration = program.get<double>("--max-acceleration");
    double maxJerk = program.get<double>("--max-jerk");
    size_t numSamples = static_cast<size_t>(program.get<int>("--samples"));
    double transitionFraction = program.get<double>("--transition-fraction");
    bool hasG64Tolerance = program.is_used("--g64-tolerance");
    double g64Tolerance = hasG64Tolerance
        ? program.get<double>("--g64-tolerance") : 0.0;

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
    if (hasG64Tolerance) {
        if (g64Tolerance < 0.0) {
            std::cout << "Mode: outside circle blend, radius=" << std::abs(g64Tolerance)
                      << " mm, transitionFraction=" << transitionFraction << "\n";
        } else {
            std::cout << "Mode: inside Bézier blend, tolerance=" << g64Tolerance << " mm\n";
        }
    } else {
        std::cout << "Mode: exact stop (no blending)\n";
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

    // ── Step 2: Build NURBS path ──
    auto nurbsResult = tether::motion::piecewiseNurbsFromSegments(segments);

    // Apply path blending based on CLI arguments.
    // Default: exact stop (no blending). Only blend if --g64-tolerance was
    // explicitly provided.
    tether::motion::PiecewiseNurbsPath plannedPath = std::move(nurbsResult.path);
    std::vector<double> plannedFeedRates = std::move(nurbsResult.feedRates);

    if (hasG64Tolerance) {
        if (g64Tolerance < 0.0) {
            // Outside circle blend
            tether::motion::OutsideCircleBlendConfig blendConfig;
            blendConfig.radius = std::abs(g64Tolerance);
            blendConfig.transitionFraction = transitionFraction;
            auto blendResult = tether::motion::OutsideCircleBlender::blend(
                plannedPath, blendConfig);
            if (blendResult.path && blendResult.blendedCount > 0) {
                plannedPath = std::move(*blendResult.path);
                plannedFeedRates.assign(plannedPath.numPieces(), maxVelocity);
                std::cout << "Outside circle blend: " << blendResult.blendedCount
                          << " corners blended, "
                          << plannedPath.numPieces() << " pieces\n";
            }
        } else {
            // Inside Bézier blend
            tether::motion::BlendSpec blendSpec;
            blendSpec.mode = tether::motion::PathMode::Blend;
            blendSpec.tolerance = g64Tolerance;
            tether::motion::PathBlender blender;
            auto blended = blender.blend(plannedPath, blendSpec);
            if (!blended.pieces.empty()) {
                plannedPath = tether::motion::PiecewiseNurbsPath(
                    std::move(blended.pieces));
                plannedFeedRates.assign(plannedPath.numPieces(), maxVelocity);
                std::cout << "Inside Bézier blend: " << blended.blendedCount
                          << " corners blended, "
                          << plannedPath.numPieces() << " pieces\n";
            }
        }
    }

    std::cout << "NURBS path: " << plannedPath.numPieces() << " pieces, "
              << plannedPath.totalLength() << " mm\n";

    // ── Step 3: Run Pareto planner → WSS ──
    MotionPlanner::PathAdapter<3, double> pathAdapter(plannedPath);
    if (!plannedFeedRates.empty()) {
        pathAdapter.setSegmentVelocityLimits(plannedFeedRates);
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
    std::size_t plannerSamples = std::min<std::size_t>(
        20000, std::max<std::size_t>(200, pathAdapter.numSegments() * 20));
    auto velocityProfile = profiler.computeProfile(
        pathAdapter, maxVelocity, 0.0, 0.0, plannerSamples);

    if (!velocityProfile) {
        std::cerr << "ERROR: ParetoPlanner returned null velocity profile.\n";
        return 1;
    }

    auto wss = profiler.weightedSource();
    if (!wss || wss->arcs().empty()) {
        std::cerr << "ERROR: No WSS arcs generated.\n";
        return 1;
    }

    const auto& arcs = wss->arcs();
    double totalTime = wss->totalTime();
    std::cout << "WSS: " << arcs.size() << " arcs, totalTime=" << totalTime << "s\n";

    // ── Step 4: Sample WSS analytically ──
    auto samples = sampleWss(arcs, totalTime, numSamples);
    std::cout << "Sampled " << samples.size() << " points\n";

    // Print some stats
    double maxV = 0, maxA = 0;
    for (const auto& s : samples) {
        maxV = std::max(maxV, std::abs(s.v));
        maxA = std::max(maxA, std::abs(s.a));
    }
    std::cout << "Max velocity: " << maxV << " mm/s\n";
    std::cout << "Max acceleration: " << maxA << " mm/s²\n";

    // ── Step 5: Export SVG plots ──
    std::string velFile = outputPrefix + "_velocity.svg";
    std::string accFile = outputPrefix + "_acceleration.svg";

    if (writePlotSvg(velFile, "Velocity vs Time", "Velocity (mm/s)", "Time (s)",
                     samples, &Sample::v, "#00cc66")) {
        std::cout << "Exported velocity plot to: " << velFile << "\n";
    } else {
        std::cerr << "ERROR: Failed to write " << velFile << "\n";
        return 1;
    }

    if (writePlotSvg(accFile, "Acceleration vs Time", "Acceleration (mm/s²)", "Time (s)",
                     samples, &Sample::a, "#ff6600")) {
        std::cout << "Exported acceleration plot to: " << accFile << "\n";
    } else {
        std::cerr << "ERROR: Failed to write " << accFile << "\n";
        return 1;
    }

    return 0;
}
