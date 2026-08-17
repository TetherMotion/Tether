/**
 * @file gcode_motion_benchmark.cpp
 * @brief End-to-end G-code motion processing pipeline benchmark.
 *
 * @details
 * Replicates the full WebGCodeViewer processing pipeline using only Tether
 * primitives, timing every stage and sub-stage that is instrumented:
 *
 *   Step 0: File I/O (read file into memory)
 *   Step 1: Parse G-code → PlanningSegments + block metadata
 *           (PlanningSegmentBuilder::fromText, with stopOnError=false retry)
 *   Step 2: Per-segment corner deviation (CornerAnalyzer::analyze)
 *           + extrusion ratio and extruder speed computation
 *   Step 3: Build NURBS path (piecewiseNurbsFromSegments)
 *   Step 3b: Pareto time-energy-optimal velocity profile
 *           (ParetoTimeEnergyOptimalVelocityPlanner::computeProfile)
 *   Step 3c: Analytical extrusion/pressure-advance planners
 *           (linear, power-law, Cross-WLF, thermal observer, LTI,
 *            overlap-add LPV, ARX LPV, state-space LPV, flow-adaptive heater)
 *   Step 4 (optional): Dense sampling (TrajectoryAnalyzer::analyze)
 *                       + computeStatistics
 *
 * ReNURBS fitting is intentionally omitted because the Pareto profiler now
 * outputs an analytical SSR/WSS profile.
 *
 * Usage:
 *   gcode_motion_benchmark <gcode_file> [--dense] [--constraint-cache-size N]
 *                          [--max-velocity V] [--max-accel A] [--max-jerk J]
 *
 * --dense                  : Run Step 4 (dense sampling via TrajectoryAnalyzer).
 * --constraint-cache-size N : Pareto solver constraint cache size (default 20000, capped).
 * --max-velocity  : Path velocity limit mm/s (default 200).
 * --max-accel     : Path acceleration limit mm/s² (default 2000).
 * --max-jerk      : Path jerk limit mm/s³ (default 20000; 0 disables jerk).
 */

#include "tether/gcode/PlanningSegmentBuilder.hpp"
#include "tether/gcode/GCodeInterpreter.hpp"
#include "tether/gcode/motion/G64CornerMode.hpp"
#include "tether/motion_planner/PathAdapter.hpp"
#include "tether/motion_planner/analytical/ParetoTimeEnergyOptimalVelocityPlanner.hpp"
#include "tether/motion_planner/geometry/PlanningSegmentConverter.hpp"
#include "tether/motion_planner/analytical/extrusion/AnalyticalExtrusionTypes.hpp"
#include "tether/motion_planner/analytical/extrusion/AnalyticalLinearPressureAdvance.hpp"
#include "tether/motion_planner/analytical/extrusion/AnalyticalPowerLawPressureAdvance.hpp"
#include "tether/motion_planner/analytical/extrusion/AnalyticalCrossWLFPressureAdvance.hpp"
#include "tether/motion_planner/analytical/extrusion/AnalyticalMeltZoneThermalObserver.hpp"
#include "tether/motion_planner/analytical/extrusion/AnalyticalLTIDeconvolution.hpp"
#include "tether/motion_planner/analytical/extrusion/AnalyticalOverlapAddLPV.hpp"
#include "tether/motion_planner/analytical/extrusion/AnalyticalARXLPVInverse.hpp"
#include "tether/motion_planner/analytical/extrusion/AnalyticalStateSpaceLPV.hpp"
#include "tether/motion_planner/analytical/extrusion/AnalyticalFlowAdaptiveHeater.hpp"
#include "tether/control/extrusion/PressureFlowLut.hpp"
#include "tether/control/extrusion/CrossWlfRheology.hpp"
#include "tether/export/TrajectoryAnalyzer.hpp"

#include <Eigen/Dense>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <optional>
#include <sstream>
#include <string>
#include <string_view>
#include <sys/resource.h>
#include <vector>

using Clock = std::chrono::steady_clock;
using Duration = std::chrono::duration<double, std::milli>;

using GCode::PlanningSegment;
using GCode::PlanningSegmentBuilder;
using GCode::PlanningSegmentResult;
using GCode::CornerAnalyzer;
using MotionPlanner::PathAdapter;
using MotionPlanner::analytical::ParetoTimeEnergyOptimalVelocityPlanner;
using MotionPlanner::analytical::CostWeights;
using MotionPlanner::analytical::WeightedSwitchingStructure;
using MotionPlanner::KinematicLimits;
using MotionPlanner::VelocityProfile;
using MotionPlanner::SampledVelocityProfile;
using MotionPlanner::analytical::extrusion::ExtrusionTrajectory;
using MotionPlanner::analytical::extrusion::AnalyticalLinearPressureAdvance;
using MotionPlanner::analytical::extrusion::AnalyticalLinearPressureAdvanceParams;
using MotionPlanner::analytical::extrusion::AnalyticalPowerLawPressureAdvance;
using MotionPlanner::analytical::extrusion::AnalyticalPowerLawPressureAdvanceParams;
using MotionPlanner::analytical::extrusion::AnalyticalCrossWLFPressureAdvance;
using MotionPlanner::analytical::extrusion::AnalyticalCrossWLFPressureAdvanceParams;
using MotionPlanner::analytical::extrusion::AnalyticalMeltZoneThermalObserver;
using MotionPlanner::analytical::extrusion::AnalyticalThermalParams;
using MotionPlanner::analytical::extrusion::AnalyticalLTIDeconvolution;
using MotionPlanner::analytical::extrusion::AnalyticalLTIDeconvParams;
using MotionPlanner::analytical::extrusion::AnalyticalOverlapAddLPV;
using MotionPlanner::analytical::extrusion::AnalyticalOverlapAddLPVParams;
using MotionPlanner::analytical::extrusion::AnalyticalARXLPVInverse;
using MotionPlanner::analytical::extrusion::AnalyticalARXLPVParams;
using MotionPlanner::analytical::extrusion::AnalyticalStateSpaceLPV;
using MotionPlanner::analytical::extrusion::AnalyticalStateSpaceLPVParams;
using MotionPlanner::analytical::extrusion::AnalyticalFlowAdaptiveHeater;
using MotionPlanner::analytical::extrusion::AnalyticalFlowAdaptiveHeaterParams;
using tether::control::extrusion::PressureFlowLut;
using tether::control::extrusion::CrossWlfParams;
using tether::control::extrusion::NozzleGeometry;
using GCodeExport::TrajectoryAnalyzer;
using GCodeExport::AnalysisConfig;
using GCodeExport::TrajectorySample;
using GCodeExport::TrajectoryStatistics;

namespace {

// ── Timing helpers ───────────────────────────────────────────────────────────

class Timer {
public:
    void start() { start_ = Clock::now(); }
    double ms() const { return Duration(Clock::now() - start_).count(); }
    Clock::time_point start_;
};

/// A recorded stage result for the summary table.
struct StageRow {
    std::string label;
    double ms = 0.0;
    size_t count = 0;        // items produced (0 = N/A)
    std::string note;        // optional extra info
};

class StageReport {
public:
    /// Time a stage by lambda; record label + item count.
    template<typename F>
    auto run(const std::string& label, size_t count, F&& f) -> decltype(f()) {
        Timer t; t.start();
        auto r = f();
        rows_.push_back({label, t.ms(), count, {}});
        return r;
    }
    /// Time a stage by lambda; record label only (count filled by caller).
    template<typename F>
    auto run(const std::string& label, F&& f) -> decltype(f()) {
        Timer t; t.start();
        auto r = f();
        rows_.push_back({label, t.ms(), 0, {}});
        return r;
    }
    /// Add a raw row (for sub-stages measured manually inside a run).
    void add(const std::string& label, double ms, size_t count = 0,
             const std::string& note = "") {
        rows_.push_back({label, ms, count, note});
    }
    const std::vector<StageRow>& rows() const { return rows_; }
private:
    std::vector<StageRow> rows_;
};

// ── Memory helpers ───────────────────────────────────────────────────────────

size_t peak_rss_bytes() {
    struct rusage u;
    getrusage(RUSAGE_SELF, &u);
    return static_cast<size_t>(u.ru_maxrss) * 1024; // KiB → bytes on Linux
}

std::string fmt_mem(size_t bytes) {
    std::ostringstream o;
    o << std::fixed << std::setprecision(2);
    if (bytes < 1024)               o << bytes << " B";
    else if (bytes < 1024ull<<20)   o << bytes / 1024.0 << " KiB";
    else if (bytes < 1024ull<<30)   o << bytes / (1024.0*1024.0) << " MiB";
    else                            o << bytes / (1024.0*1024.0*1024.0) << " GiB";
    return o.str();
}

std::string fmt_bytes(size_t bytes) { return fmt_mem(bytes); }

// ── File helpers ─────────────────────────────────────────────────────────────

size_t file_size(const std::string& path) {
    std::ifstream f(path, std::ios::binary | std::ios::ate);
    return f.is_open() ? static_cast<size_t>(f.tellg()) : 0;
}

size_t count_lines(const std::string& path) {
    std::ifstream f(path);
    return static_cast<size_t>(std::count(
        std::istreambuf_iterator<char>(f),
        std::istreambuf_iterator<char>(), '\n'));
}

std::string read_file(const std::string& path) {
    std::ifstream f(path, std::ios::binary);
    if (!f.is_open()) return {};
    f.seekg(0, std::ios::end);
    size_t sz = static_cast<size_t>(f.tellg());
    f.seekg(0, std::ios::beg);
    std::string buf(sz, '\0');
    f.read(&buf[0], sz);
    return buf;
}

// ── Klipper command pre-filter (mirrors GCodeProcessor::filterKlipperCommands) ─

bool is_klipper_extended(std::string_view line) {
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
    while (cmdEnd < line.size() &&
           ((line[cmdEnd] >= 'A' && line[cmdEnd] <= 'Z') || line[cmdEnd] == '_'))
        cmdEnd++;
    if (cmdEnd - i < 2) return false;
    if (cmdEnd < line.size()) {
        char after = line[cmdEnd];
        if (after != ' ' && after != '\t' && after != '\r' &&
            after != '\n' && after != '=') return false;
    }
    return true;
}

bool has_axis_words_without_values(std::string_view line) {
    size_t c = line.find_first_of(";(");
    if (c != std::string_view::npos) line = line.substr(0, c);
    size_t i = 0;
    while (i < line.size() && (line[i] == ' ' || line[i] == '\t')) i++;
    if (i >= line.size() || (line[i] != 'M' && line[i] != 'G')) return false;
    char codeLetter = line[i++];
    while (i < line.size() && std::isdigit(static_cast<unsigned char>(line[i]))) i++;
    int codeNum = 0;
    try { codeNum = std::stoi(std::string(line.substr(1, i - 1))); } catch (...) {}
    if (codeLetter == 'M' && codeNum == 117) {
        while (i < line.size() && (line[i] == ' ' || line[i] == '\t')) i++;
        return i < line.size();
    }
    auto isWord = [](char ch) {
        return ch=='X'||ch=='Y'||ch=='Z'||ch=='A'||ch=='B'||ch=='C'||ch=='U'||
               ch=='V'||ch=='W'||ch=='E'||ch=='F'||ch=='S'||ch=='P'||ch=='R'||
               ch=='T'||ch=='I'||ch=='J'||ch=='K'||ch=='D'||ch=='H'||ch=='L'||
               ch=='O'||ch=='N'||ch=='Q';
    };
    bool found = false;
    while (i < line.size()) {
        while (i < line.size() && (line[i] == ' ' || line[i] == '\t')) i++;
        if (i >= line.size()) break;
        char ch = line[i];
        if (isWord(ch)) {
            i++;
            if (i < line.size() && (std::isdigit(static_cast<unsigned char>(line[i])) ||
                line[i] == '.' || line[i] == '-' || line[i] == '+')) {
                while (i < line.size() && !std::isspace(static_cast<unsigned char>(line[i])) &&
                       !isWord(line[i])) i++;
            } else {
                found = true;
            }
        } else {
            i++;
        }
    }
    return found;
}

std::string filter_klipper(const std::string& gcode) {
    std::string out;
    out.reserve(gcode.size());
    size_t pos = 0;
    while (pos < gcode.size()) {
        size_t le = gcode.find('\n', pos);
        std::string_view line = (le == std::string::npos)
            ? std::string_view(gcode.data() + pos)
            : std::string_view(gcode.data() + pos, le - pos);
        if (is_klipper_extended(line) || has_axis_words_without_values(line)) {
            out += "; [filtered] ";
            out += line;
        } else {
            out += line;
        }
        if (le != std::string::npos) { out += '\n'; pos = le + 1; }
        else break;
    }
    return out;
}

// ── Corner deviation + extruder speed (mirrors GCodeProcessor) ───────────────

void compute_corner_deviation(std::vector<PlanningSegment>& segs) {
    for (size_t i = 0; i < segs.size(); ++i) {
        auto& seg = segs[i];
        if (seg.blendTolerance <= 0.0 || i == 0) {
            seg.entryVelocity = 0.0;
            continue;
        }
        const auto& prev = segs[i - 1];
        auto analysis = CornerAnalyzer::analyze(prev, seg);
        seg.entryVelocity = analysis.deviationPercentage;
    }
}

void compute_extruder_speed(std::vector<PlanningSegment>& segs) {
    for (auto& seg : segs) {
        double eDelta = seg.exitVelocity; // repurposed E-delta storage
        if (seg.isRapid || std::abs(eDelta) < 1e-12 || seg.segmentTime < 1e-9) {
            seg.exitVelocity = 0.0;
            continue;
        }
        seg.exitVelocity = std::abs(eDelta) / seg.segmentTime; // mm/s
    }
}

/// Compute the average extrusion ratio (E mm / path mm) for extruding moves.
/// This must be called *before* compute_extruder_speed() overwrites exitVelocity.
double compute_average_extrusion_ratio(const std::vector<PlanningSegment>& segs) {
    double totalE = 0.0;
    double totalLen = 0.0;
    for (const auto& seg : segs) {
        if (seg.isRapid || seg.segmentLength < 1e-9) continue;
        double eDelta = seg.exitVelocity; // E-delta before it is repurposed
        if (std::abs(eDelta) < 1e-12) continue;
        totalE += eDelta;
        totalLen += seg.segmentLength;
    }
    return (totalLen > 1e-9) ? (totalE / totalLen) : 0.0;
}

/// Generate N evenly spaced sample times in [0, totalTime].
std::vector<double> linspace_times(double totalTime, std::size_t n) {
    std::vector<double> times;
    if (n == 0 || totalTime <= 0.0) return times;
    times.reserve(n);
    for (std::size_t i = 0; i < n; ++i) {
        double alpha = (n == 1) ? 0.0
            : static_cast<double>(i) / static_cast<double>(n - 1);
        times.push_back(alpha * totalTime);
    }
    return times;
}

// ── Analytical extrusion / pressure-advance benchmark ────────────────────────

void run_analytical_extrusion_benchmarks(
    StageReport& report,
    const WeightedSwitchingStructure<3, double>* wss,
    double avgExtrusionRatio,
    std::size_t numSamples = 1000) {

    if (!wss || wss->arcs().empty()) {
        report.add("Step 3c: ExtrusionTrajectory (SKIPPED)", 0.0, 0,
                   "no WSS available");
        return;
    }

    // Build the extrusion trajectory (uniform average ratio for the benchmark).
    Timer tm; tm.start();
    ExtrusionTrajectory<3, double> traj(*wss, avgExtrusionRatio);
    double traj_ms = tm.ms();
    if (traj.numArcs() == 0 || traj.totalTime() <= 0.0) {
        report.add("Step 3c: ExtrusionTrajectory (FAILED)", 0.0, 0,
                   "empty trajectory");
        return;
    }
    report.add("Step 3c: ExtrusionTrajectory", traj_ms, traj.numArcs());

    double totalT = traj.totalTime();
    std::vector<double> times = linspace_times(totalT, numSamples);

    // 1. Linear PressureAdvance
    {
        AnalyticalLinearPressureAdvanceParams params;
        params.pressureAdvance = 0.045;
        params.smoothTime = 0.0;
        Timer tm; tm.start();
        AnalyticalLinearPressureAdvance<3> pa(traj, params);
        for (double t : times) { (void)pa.offsetAtTime(t); }
        report.add("Step 3c.1: Linear PressureAdvance", tm.ms(), numSamples);
    }

    // 2. Power-law PressureAdvance
    {
        AnalyticalPowerLawPressureAdvanceParams params;
        params.baseGain = 0.012;
        params.flowIndex = 0.5;
        Timer tm; tm.start();
        AnalyticalPowerLawPressureAdvance<3> pa(traj, params);
        for (double t : times) { (void)pa.offsetAtTime(t); }
        report.add("Step 3c.2: PowerLaw PressureAdvance", tm.ms(), numSamples);
    }

    // 3. Melt-zone thermal observer (also used by Cross-WLF below)
    AnalyticalMeltZoneThermalObserver<3>* thermalObs = nullptr;
    {
        AnalyticalThermalParams params;
        params.heaterPWM = 0.5;
        Timer tm; tm.start();
        static thread_local AnalyticalMeltZoneThermalObserver<3> thermal(traj, params);
        thermal.initialize(210.0);
        double construct_ms = tm.ms();
        thermalObs = &thermal;
        tm.start();
        for (double t : times) { (void)thermal.meltTempAt(t); }
        report.add("Step 3c.3: MeltZone ThermalObserver",
                   construct_ms + tm.ms(), numSamples);
    }

    // 4. Cross-WLF PressureAdvance
    {
        CrossWlfParams cwParams;
        NozzleGeometry geom;
        auto lut = std::make_shared<PressureFlowLut>();
        std::vector<double> flowAxis = {0.0, 0.5, 1.0, 2.0, 5.0, 10.0, 20.0, 50.0};
        std::vector<double> tempAxis = {180.0, 200.0, 210.0, 220.0, 240.0};
        lut->build(cwParams, geom, flowAxis, tempAxis);

        AnalyticalCrossWLFPressureAdvanceParams params;
        params.compressibilityOverArea = 1e-5;
        Timer tm; tm.start();
        AnalyticalCrossWLFPressureAdvance<3> pa(traj, lut, params, thermalObs);
        for (double t : times) { (void)pa.offsetAtTime(t); }
        report.add("Step 3c.4: CrossWLF PressureAdvance", tm.ms(), numSamples);
    }

    // 5. LTI deconvolution
    {
        std::vector<double> h = {0.0, 0.5, 0.3, 0.15, 0.08, 0.04, 0.02, 0.01};
        AnalyticalLTIDeconvParams params;
        params.lambda = 1e-4;
        Timer tm; tm.start();
        AnalyticalLTIDeconvolution<3> deconv(traj, h, 1000.0, params);
        for (double t : times) { (void)deconv.inputAtTime(t, false); }
        report.add("Step 3c.5: LTI Deconvolution", tm.ms(), numSamples);
    }

    // 6. Overlap-add LPV
    {
        AnalyticalOverlapAddLPVParams params;
        params.lambda = 1e-4;
        Timer tm; tm.start();
        AnalyticalOverlapAddLPV<3> lpv(traj, params);
        std::vector<double> h1 = {0.0, 0.5, 0.3, 0.15, 0.08, 0.04, 0.02, 0.01};
        std::vector<double> h2 = {0.0, 0.7, 0.2, 0.05, 0.02, 0.01, 0.005, 0.002};
        lpv.addOperatingPoint(10.0, h1, 1000.0);
        lpv.addOperatingPoint(300.0, h2, 1000.0);
        for (double t : times) { (void)lpv.inputAtTime(t, false); }
        report.add("Step 3c.6: OverlapAdd LPV", tm.ms(), numSamples);
    }

    // 7. ARX LPV inverse
    {
        AnalyticalARXLPVParams params;
        params.na = 1;
        params.nb = 0;
        Timer tm; tm.start();
        AnalyticalARXLPVInverse<3> filter(traj, params);
        filter.addModelPoint(10.0, {2.0}, {5.0}, 0.0);
        filter.addModelPoint(300.0, {1.0}, {8.0}, 0.0);
        for (double t : times) { (void)filter.inputAtTime(t); }
        report.add("Step 3c.7: ARX LPV Inverse", tm.ms(), numSamples);
    }

    // 8. State-space LPV
    {
        AnalyticalStateSpaceLPVParams params;
        params.stateDim = 2;
        params.lambda = 1e-6;
        Timer tm; tm.start();
        AnalyticalStateSpaceLPV<3> estimator(traj, params);

        Eigen::MatrixXd A(2, 2), B(2, 1), C(1, 2);
        A << -1.0, 0.0, 0.0, -2.0;
        B << 1.0, 0.0;
        C << 1.0, 0.0;
        estimator.addModelPoint({10.0, A, B, C});

        A << -0.5, 0.0, 0.0, -1.0;
        B << 1.5, 0.0;
        estimator.addModelPoint({300.0, A, B, C});

        for (double t : times) {
            if (t < 0.01 || t > totalT * 0.99) continue;
            (void)estimator.inputAtTime(t);
        }
        report.add("Step 3c.8: StateSpace LPV", tm.ms(), numSamples);
    }

    // 9. Flow-adaptive heater (analytical, no explicit sampling)
    {
        AnalyticalFlowAdaptiveHeaterParams params;
        params.targetTempC = 210.0;
        Timer tm; tm.start();
        AnalyticalFlowAdaptiveHeater<3> heater(traj, params);
        for (double t : times) { (void)heater.temperatureDeltaAtTime(t); }
        report.add("Step 3c.9: FlowAdaptive Heater", tm.ms(), numSamples);
    }
}

// ── Pretty printing ──────────────────────────────────────────────────────────

void print_separator() {
    std::cout << std::string(78, '-') << '\n';
}

void print_banner() {
    std::cout << "\n";
    std::cout << "==============================================================================\n";
    std::cout << "               G-CODE MOTION PIPELINE BENCHMARK (Tether)\n";
    std::cout << "==============================================================================\n";
}

void print_table(const std::vector<StageRow>& rows) {
    // Column widths: label 46 | time 12 | count 12 | note
    std::cout << '\n';
    std::cout << std::left  << std::setw(46) << "Stage"
              << std::right << std::setw(12) << "Time (ms)"
              << std::right << std::setw(12) << "Items" << '\n';
    print_separator();
    double total = 0.0;
    for (const auto& r : rows) {
        std::cout << std::left  << std::setw(46) << r.label
                  << std::right << std::setw(12) << std::fixed
                  << std::setprecision(2) << r.ms;
        if (r.count > 0)
            std::cout << std::right << std::setw(12) << r.count;
        else
            std::cout << std::right << std::setw(12) << "-";
        std::cout << '\n';
        if (!r.note.empty())
            std::cout << "    " << r.note << '\n';
        total += r.ms;
    }
    print_separator();
    std::cout << std::left  << std::setw(46) << "TOTAL"
              << std::right << std::setw(12) << std::fixed
              << std::setprecision(2) << total
              << std::right << std::setw(12) << "" << '\n';
}

// ── CLI options ──────────────────────────────────────────────────────────────

struct Options {
    std::string file;
    bool dense = false;
    size_t constraintCacheSize = 20000;
    size_t max_dense_samples = 1000000;  ///< Cap dense samples to prevent OOM
    double max_velocity = 200.0;     // mm/s
    double max_accel = 2000.0;       // mm/s²
    double max_jerk = 20000.0;       // mm/s³ (0 = disable)
};

bool parse_args(int argc, char* argv[], Options& opt) {
    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];
        auto next = [&](const char* name) -> const char* {
            if (i + 1 >= argc) {
                std::cerr << "Error: " << name << " requires a value\n";
                return nullptr;
            }
            return argv[++i];
        };
        if (a == "--dense")              opt.dense = true;
        else if (a == "--constraint-cache-size")
            { const char* v = next("--constraint-cache-size"); if (!v) return false; opt.constraintCacheSize = std::strtoull(v, nullptr, 10); }
        else if (a == "--max-samples")   { const char* v = next("--max-samples"); if (!v) return false; opt.max_dense_samples = std::strtoull(v, nullptr, 10); }
        else if (a == "--max-velocity")  { const char* v = next("--max-velocity"); if (!v) return false; opt.max_velocity = std::strtod(v, nullptr); }
        else if (a == "--max-accel")     { const char* v = next("--max-accel"); if (!v) return false; opt.max_accel = std::strtod(v, nullptr); }
        else if (a == "--max-jerk")      { const char* v = next("--max-jerk"); if (!v) return false; opt.max_jerk = std::strtod(v, nullptr); }
        else if (a == "-h" || a == "--help") {
            std::cout <<
                "Usage: gcode_motion_benchmark <gcode_file> [options]\n"
                "\n"
                "Options:\n"
                "  --dense                   Run Step 4 (dense sampling via TrajectoryAnalyzer)\n"
                "  --constraint-cache-size N Pareto solver constraint cache size (default 20000)\n"
                "  --max-samples N           Cap dense sample count (default 1000000; 0=uncapped)\n"
                "  --max-velocity V  Path velocity limit mm/s (default 200)\n"
                "  --max-accel A     Path acceleration limit mm/s² (default 2000)\n"
                "  --max-jerk J      Path jerk limit mm/s³ (default 20000; 0 disables)\n";
            return false;
        } else if (a.size() > 0 && a[0] == '-') {
            std::cerr << "Error: unknown option '" << a << "'\n";
            return false;
        } else {
            if (!opt.file.empty()) {
                std::cerr << "Error: multiple input files specified\n";
                return false;
            }
            opt.file = a;
        }
    }
    if (opt.file.empty()) {
        std::cerr << "Usage: gcode_motion_benchmark <gcode_file> [options]\n"
                     "Run with --help for details.\n";
        return false;
    }
    return true;
}

} // namespace

int main(int argc, char* argv[]) {
    Options opt;
    if (!parse_args(argc, argv, opt)) return opt.file.empty() ? 0 : 1;

    const size_t fsize = file_size(opt.file);
    if (fsize == 0) {
        std::cerr << "Error: cannot open or empty file: " << opt.file << '\n';
        return 1;
    }
    const size_t lines = count_lines(opt.file);

    print_banner();
    std::cout << "File:         " << opt.file << '\n';
    std::cout << "Size:         " << fmt_bytes(fsize) << " (" << lines << " lines)\n";
    std::cout << "Limits:       v=" << opt.max_velocity << " mm/s, "
              << "a=" << opt.max_accel << " mm/s², "
              << "j=" << opt.max_jerk << " mm/s³\n";
    std::cout << "Pareto constraint cache size: " << opt.constraintCacheSize
              << (opt.dense ? "  (dense sampling ON)" : "  (dense sampling OFF)") << '\n';
    std::cout << "Peak RSS at start: " << fmt_mem(peak_rss_bytes()) << '\n';

    StageReport report;
    Timer total; total.start();

    // ── Step 0: File I/O ─────────────────────────────────────────────────────
    std::string content = report.run("Step 0: File I/O (read into memory)", [&] {
        return read_file(opt.file);
    });
    if (content.empty()) {
        std::cerr << "Error: failed to read file: " << opt.file << '\n';
        return 1;
    }

    // ── Step 0b: Klipper command pre-filter ──────────────────────────────────
    std::string filtered = report.run("Step 0b: filter Klipper commands", [&] {
        return filter_klipper(content);
    });
    // Free the raw content to reduce peak RSS.
    content.clear();
    content.shrink_to_fit();

    // ── Step 1: Parse G-code → PlanningSegments ──────────────────────────────
    PlanningSegmentResult parseResult;
    Timer step1; step1.start();
    double parse_ms = 0.0, retry_ms = 0.0;
    {
        Timer t; t.start();
        parseResult = PlanningSegmentBuilder::fromText(filtered);
        parse_ms = t.ms();
    }
    // Retry with stopOnError=false on parse failure (mirrors GCodeProcessor).
    if (!parseResult.error.ok()) {
        Timer t; t.start();
        GCode::InterpreterConfig retryConfig;
        retryConfig.stopOnError = false;
        auto retry = PlanningSegmentBuilder::fromText(filtered, retryConfig);
        retry_ms = t.ms();
        if (!retry.segments.empty()) {
            parseResult = std::move(retry);
        }
    }
    report.add("Step 1a: parse (strict)", parse_ms);
    if (retry_ms > 0.0)
        report.add("Step 1b: parse (retry, stopOnError=false)", retry_ms,
                   parseResult.segments.size());

    if (!parseResult.error.ok() && parseResult.segments.empty()) {
        std::cerr << "Error: G-code parse failed — "
                  << parseResult.error.message.data() << " (line "
                  << parseResult.error.line << ")\n";
        return 1;
    }
    if (parseResult.segments.empty()) {
        std::cerr << "Error: no motion segments found ("
                  << parseResult.blocks.size() << " blocks parsed)\n";
        return 1;
    }

    auto& segments = parseResult.segments;
    std::cout << "\nParsed: " << segments.size() << " segments, "
              << parseResult.blocks.size() << " blocks"
              << (parseResult.error.ok() ? "" : " (with non-fatal parse warnings)")
              << '\n';

    // ── Step 2: Corner deviation + extrusion ratio + extruder speed ─────────
    Timer t2; t2.start();
    compute_corner_deviation(segments);
    double corner_ms = t2.ms();
    t2.start();
    double avgExtrusionRatio = compute_average_extrusion_ratio(segments);
    double ratio_ms = t2.ms();
    t2.start();
    compute_extruder_speed(segments);
    double extruder_ms = t2.ms();
    report.add("Step 2a: corner deviation (CornerAnalyzer)", corner_ms,
               segments.size());
    report.add("Step 2b: average extrusion ratio extraction", ratio_ms,
               segments.size(),
               "avg_ratio=" + std::to_string(avgExtrusionRatio));
    report.add("Step 2c: extruder speed computation", extruder_ms,
               segments.size());

    // ── Step 3: Build NURBS path ─────────────────────────────────────────────
    std::optional<tether::motion::PlanningSegmentNurbsResult> nurbsResult;
    try {
        Timer t; t.start();
        nurbsResult = tether::motion::piecewiseNurbsFromSegments(segments);
        report.add("Step 3: piecewiseNurbsFromSegments", t.ms(),
                   nurbsResult->path.numPieces());
    } catch (const std::exception& e) {
        std::cerr << "Error: NURBS construction failed: " << e.what() << '\n';
        return 1;
    }
    const double pathLength = nurbsResult->path.totalLength();
    std::cout << "NURBS path: " << nurbsResult->path.numPieces()
              << " pieces, total length " << std::fixed
              << std::setprecision(2) << pathLength << " mm\n";

    // ── Step 3b: Pareto time-energy-optimal velocity profile ─────────────────
    // Uses Tether's ParetoTimeEnergyOptimalVelocityPlanner (the default
    // profiler in MotionPlanBuilder) instead of BasicTOPPRA. Solves the
    // weighted-cost optimal control problem J = ∫[w_t + w_a·a²]dt via
    // Pontryagin's maximum principle, producing BANG + SINGULAR arcs.
    // With w_a = 0 it degenerates to pure time-optimal (TOPPRA).
    // Skip for very large files (mirrors GCodeProcessor's kMaxSegmentsForReNurbs).
    constexpr std::size_t kMaxSegmentsForReNurbs = 1'000'000;
    std::shared_ptr<VelocityProfile> velocityProfile;

    if (nurbsResult->path.numPieces() <= kMaxSegmentsForReNurbs) {
        PathAdapter<3, double> pathAdapter(std::move(nurbsResult->path));

        KinematicLimits<3, double> limits;
        limits.path.maxPathVelocity = opt.max_velocity;
        limits.path.maxPathAcceleration = opt.max_accel;
        limits.path.maxPathJerk = opt.max_jerk;
        limits.path.jerkLimitEnabled = (opt.max_jerk > 0.0);
        for (int i = 0; i < 3; ++i) {
            limits.axis.maxVelocity[i] = opt.max_velocity;
            limits.axis.maxAcceleration[i] = opt.max_accel;
            limits.axis.maxJerk[i] = opt.max_jerk;
        }
        limits.axis.jerkLimitEnabled = limits.path.jerkLimitEnabled;

        // Use the first segment's feed rate (mm/min → mm/s) capped by max.
        double feedRate = opt.max_velocity;
        for (const auto& seg : segments) {
            if (seg.feedRate > 0.0) {
                feedRate = std::min(feedRate, seg.feedRate / 60.0);
                break;
            }
        }
        std::size_t constraintCacheSize = std::min(opt.constraintCacheSize,
            std::max<std::size_t>(200, pathAdapter.numSegments() * 20));

        // Pareto planner with default weights (w_t=1, w_a=0 → time-optimal).
        CostWeights weights;
        weights.w_t = 1.0;
        weights.w_a = 0.0;
        ParetoTimeEnergyOptimalVelocityPlanner<3, double> profiler(limits, weights);
        try {
            Timer t; t.start();
            velocityProfile = std::shared_ptr<VelocityProfile>(
                profiler.computeProfile(pathAdapter, feedRate, 0.0, 0.0, constraintCacheSize));
            auto sampled = dynamic_cast<SampledVelocityProfile*>(velocityProfile.get());
            report.add("Step 3b: Pareto computeProfile", t.ms(),
                       sampled ? sampled->points().size() : 0);
        } catch (const std::exception& e) {
            report.add("Step 3b: Pareto computeProfile (FAILED)", 0.0, 0,
                       std::string("exception: ") + e.what());
        }

        // ── Step 3c: Analytical extrusion / pressure-advance planners ────────
        auto wss = profiler.weightedSource();
        try {
            run_analytical_extrusion_benchmarks(report, wss.get(),
                                                avgExtrusionRatio, 1000);
        } catch (const std::exception& e) {
            report.add("Step 3c: Analytical extrusion (FAILED)", 0.0, 0,
                       std::string("exception: ") + e.what());
        }
    } else {
        report.add("Step 3b: Pareto (SKIPPED — too many pieces)", 0.0, 0,
            "pieces=" + std::to_string(nurbsResult->path.numPieces()) +
            " > limit " + std::to_string(kMaxSegmentsForReNurbs));
        report.add("Step 3c: Analytical extrusion (SKIPPED)", 0.0, 0, {});
    }

    // ── Step 4 (optional): Dense sampling ────────────────────────────────────
    if (opt.dense) {
        AnalysisConfig ac;
        ac.timeStep = 0.001;
        ac.derivativeOrder = 4;
        ac.maxSamples = opt.max_dense_samples;  // cap to prevent OOM
        ac.limits.maxAcceleration = opt.max_accel;
        ac.limits.maxDeceleration = opt.max_accel;
        ac.limits.maxJerk = opt.max_jerk;
        ac.limits.maxVelocityLinear = opt.max_velocity * 60.0;
        TrajectoryAnalyzer analyzer(ac);

        std::vector<TrajectorySample> samples;
        try {
            Timer t; t.start();
            samples = analyzer.analyze(segments, nullptr);
            report.add("Step 4a: TrajectoryAnalyzer::analyze", t.ms(),
                       samples.size());
        } catch (const std::exception& e) {
            report.add("Step 4a: TrajectoryAnalyzer::analyze (FAILED)", 0.0, 0,
                       std::string("exception: ") + e.what());
        }

        if (!samples.empty()) {
            TrajectoryStatistics stats{};
            try {
                Timer t; t.start();
                stats = analyzer.computeStatistics(samples);
                report.add("Step 4b: computeStatistics", t.ms(), stats.sampleCount);
            } catch (const std::exception& e) {
                report.add("Step 4b: computeStatistics (FAILED)", 0.0, 0,
                           std::string("exception: ") + e.what());
            }
            std::cout << "Dense samples: " << samples.size()
                      << ", duration " << std::fixed << std::setprecision(2)
                      << stats.duration << " s, path "
                      << stats.pathLength << " mm\n";
        }
    } else {
        report.add("Step 4: dense sampling (SKIPPED — use --dense)", 0.0, 0, {});
    }

    // ── Summary ──────────────────────────────────────────────────────────────
    double total_ms = total.ms();
    print_table(report.rows());

    std::cout << "\nSummary:\n";
    std::cout << "  Total time:   " << std::fixed << std::setprecision(2)
              << total_ms << " ms"
              << " (" << std::setprecision(3) << total_ms / 1000.0 << " s)\n";
    std::cout << "  Peak RSS:     " << fmt_mem(peak_rss_bytes()) << '\n';
    double lines_per_s = lines / (total_ms / 1000.0);
    double mb_per_s = (fsize / (1024.0 * 1024.0)) / (total_ms / 1000.0);
    std::cout << "  Throughput:   " << std::fixed << std::setprecision(0)
              << lines_per_s << " lines/s, "
              << std::setprecision(2) << mb_per_s << " MiB/s\n";
    std::cout << "\n==============================================================================\n";
    return 0;
}
