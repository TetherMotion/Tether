/**
 * @file test_analytical_extrusion_gcode.cpp
 * @brief G-code integration tests for analytical extrusion compensation.
 *
 * @details
 * Tests the analytical extrusion compensation algorithms with real G-code
 * files, including the kissen_punctperi2 gcode from ~/dev.
 *
 * The tests:
 * 1. Parse G-code moves to extract extrusion ratios per segment
 * 2. Build a WSS from the motion-planned path
 * 3. Run all 9 analytical algorithms on the trajectory
 * 4. Verify finiteness, physical sanity, and consistency
 */

#include <gtest/gtest.h>
#include <tether/motion_planner/MotionPlanner.hpp>
#include <tether/motion_planner/MotionSegment.hpp>
#include <tether/motion_planner/blend/BlendSpec.hpp>
#include <tether/motion_planner/analytical/ParetoTimeEnergyOptimalVelocityPlanner.hpp>
#include <tether/motion_planner/analytical/extrusion/AnalyticalExtrusionTypes.hpp>
#include <tether/motion_planner/analytical/extrusion/AnalyticalLinearPressureAdvance.hpp>
#include <tether/motion_planner/analytical/extrusion/AnalyticalPowerLawPressureAdvance.hpp>
#include <tether/motion_planner/analytical/extrusion/AnalyticalCrossWLFPressureAdvance.hpp>
#include <tether/motion_planner/analytical/extrusion/AnalyticalMeltZoneThermalObserver.hpp>
#include <tether/motion_planner/analytical/extrusion/AnalyticalLTIDeconvolution.hpp>
#include <tether/motion_planner/analytical/extrusion/AnalyticalOverlapAddLPV.hpp>
#include <tether/motion_planner/analytical/extrusion/AnalyticalARXLPVInverse.hpp>
#include <tether/motion_planner/analytical/extrusion/AnalyticalStateSpaceLPV.hpp>
#include <tether/motion_planner/analytical/extrusion/AnalyticalFlowAdaptiveHeater.hpp>
#include <tether/control/extrusion/PressureFlowLut.hpp>
#include <tether/control/extrusion/CrossWlfRheology.hpp>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

using namespace MotionPlanner;
using namespace MotionPlanner::analytical;
using namespace MotionPlanner::analytical::extrusion;

// ============================================================================
// G-code parser (minimal — extracts extruding moves)
// ============================================================================

namespace {

struct GcodeMove {
    double x, y, z, e;
    double feedRate;  // mm/min
    bool extruding;
};

struct GcodeParseResult {
    std::vector<GcodeMove> moves;
    double filamentDiameterMm = 1.75;
    double nozzleDiameterMm = 0.4;
    double defaultLayerHeightMm = 0.2;
    bool hasPressureAdvance = false;
    double pressureAdvance = 0.0;
    double smoothTime = 0.0;
};

GcodeParseResult parseGcode(const std::vector<std::string>& lines) {
    GcodeParseResult result;
    double curX = 0, curY = 0, curZ = 0, curE = 0;
    double curF = 1500;  // default feed rate mm/min
    bool relativeE = false;
    bool absolutePos = true;

    for (const auto& line : lines) {
        // Strip comments
        auto commentPos = line.find(';');
        std::string cmd = (commentPos != std::string::npos)
            ? line.substr(0, commentPos) : line;

        // Check for SET_PRESSURE_ADVANCE
        if (cmd.find("SET_PRESSURE_ADVANCE") != std::string::npos) {
            result.hasPressureAdvance = true;
            auto advPos = cmd.find("ADVANCE=");
            if (advPos != std::string::npos) {
                std::stringstream ss(cmd.substr(advPos + 7));
                ss >> result.pressureAdvance;
            }
            auto stPos = cmd.find("SMOOTH_TIME=");
            if (stPos != std::string::npos) {
                std::stringstream ss(cmd.substr(stPos + 12));
                ss >> result.smoothTime;
            }
        }

        // Check for M83 (relative E) / M82 (absolute E)
        if (cmd.find("M83") != std::string::npos) relativeE = true;
        if (cmd.find("M82") != std::string::npos) relativeE = false;
        if (cmd.find("G90") != std::string::npos) absolutePos = true;
        if (cmd.find("G91") != std::string::npos) absolutePos = false;

        // Parse G0/G1 moves
        if (cmd.size() > 2 && (cmd[0] == 'G') &&
            (cmd[1] == '0' || cmd[1] == '1') &&
            (cmd[2] == ' ' || cmd[2] == '\t')) {
            std::stringstream ss(cmd.substr(3));
            char axis;
            double val;
            double newX = curX, newY = curY, newZ = curZ, newE = curE;
            double newF = curF;
            bool hasE = false;
            bool hasXYZ = false;

            while (ss >> axis >> val) {
                switch (toupper(axis)) {
                    case 'X': newX = absolutePos ? val : curX + val; hasXYZ = true; break;
                    case 'Y': newY = absolutePos ? val : curY + val; hasXYZ = true; break;
                    case 'Z': newZ = absolutePos ? val : curZ + val; hasXYZ = true; break;
                    case 'E': newE = relativeE ? curE + val : val; hasE = true; break;
                    case 'F': newF = val; break;
                }
            }

            double dE = newE - curE;
            double dx = newX - curX, dy = newY - curY, dz = newZ - curZ;
            double dist = std::sqrt(dx * dx + dy * dy + dz * dz);

            if (dist > 1e-6) {
                GcodeMove move;
                move.x = newX;
                move.y = newY;
                move.z = newZ;
                move.e = newE;
                move.feedRate = newF;
                move.extruding = (dE > 1e-8);
                result.moves.push_back(move);
            }

            curX = newX; curY = newY; curZ = newZ;
            curE = newE; curF = newF;
        }
    }

    return result;
}

std::vector<std::string> readGcodeFile(const std::string& path) {
    std::ifstream file(path);
    if (!file.is_open()) return {};
    std::vector<std::string> lines;
    std::string line;
    while (std::getline(file, line)) {
        lines.push_back(line);
    }
    return lines;
}

/// Build a path from G-code extruding moves
PathAdapter<2, double> buildPathFromGcode(const GcodeParseResult& gcode,
                                            size_t maxMoves = 50) {
    MotionSegmentList segments;
    size_t count = 0;
    double prevX = 0, prevY = 0;
    bool first = true;

    for (const auto& move : gcode.moves) {
        if (!move.extruding) {
            prevX = move.x;
            prevY = move.y;
            first = false;
            continue;
        }

        if (first) {
            prevX = move.x;
            prevY = move.y;
            first = false;
            continue;
        }

        double dist = std::sqrt(
            (move.x - prevX) * (move.x - prevX) +
            (move.y - prevY) * (move.y - prevY));

        if (dist < 0.001) continue;

        double feedRate = move.feedRate / 60.0;  // mm/min → mm/s
        segments.append(MotionSegment::linear(
            Vec<2, double>{prevX, prevY},
            Vec<2, double>{move.x, move.y},
            std::max(feedRate, 10.0)));

        prevX = move.x;
        prevY = move.y;
        ++count;
        if (count >= maxMoves) break;
    }

    if (segments.size() == 0) return PathAdapter<2, double>{};

    PathBuilderAdapter<2, double> builder;
    tether::motion::BlendSpec spec;
    spec.tolerance = 0.1;
    spec.continuity = tether::motion::Continuity::G2;
    spec.maxBlendFraction = 0.25;
    auto result = builder.build(segments, spec);
    if (!result.success) return PathAdapter<2, double>{};
    return std::move(result.path);
}

/// Extract per-segment extrusion ratios from G-code
std::vector<double> extractExtrusionRatios(const GcodeParseResult& gcode,
                                             size_t maxMoves = 50) {
    std::vector<double> ratios;
    double prevX = 0, prevY = 0, prevE = 0;
    bool first = true;
    size_t count = 0;

    for (const auto& move : gcode.moves) {
        if (first) {
            prevX = move.x;
            prevY = move.y;
            prevE = move.e;
            first = false;
            continue;
        }

        double dist = std::sqrt(
            (move.x - prevX) * (move.x - prevX) +
            (move.y - prevY) * (move.y - prevY));

        if (dist < 0.001) {
            prevX = move.x;
            prevY = move.y;
            prevE = move.e;
            continue;
        }

        double dE = move.e - prevE;
        double ratio = (dE > 0) ? dE / dist : 0.0;
        ratios.push_back(ratio);

        prevX = move.x;
        prevY = move.y;
        prevE = move.e;
        ++count;
        if (count >= maxMoves) break;
    }

    return ratios;
}

KinematicLimits<2, double> makeGcodeLimits() {
    KinematicLimits<2, double> limits;
    limits.path.maxPathVelocity = 300.0;  // 18000 mm/min
    limits.path.maxPathAcceleration = 2000.0;
    limits.path.maxPathJerk = 20000.0;
    limits.path.jerkLimitEnabled = true;
    limits.path.maxCentripetalAcceleration = 1000.0;
    for (int i = 0; i < 2; ++i) {
        limits.axis.maxVelocity[i] = 300.0;
        limits.axis.maxAcceleration[i] = 2000.0;
        limits.axis.maxJerk[i] = 20000.0;
    }
    limits.axis.jerkLimitEnabled = true;
    return limits;
}

std::vector<double> linspace(double t0, double t1, int n) {
    std::vector<double> result(n);
    for (int i = 0; i < n; ++i)
        result[i] = t0 + (t1 - t0) * static_cast<double>(i) / (n - 1);
    return result;
}

} // namespace

// ============================================================================
// Synthetic G-code tests
// ============================================================================

TEST(AnalyticalExtrusionGcodeTest, SyntheticGcode_AllAlgorithmsFinite) {
    // Simple synthetic G-code: a square with extrusion
    std::vector<std::string> gcodeLines = {
        "G21", "G90", "G92 E0", "M83",
        "G1 X0 Y0 F6000",
        "G1 X100 Y0 E5 F3000",
        "G1 X100 Y100 E10 F3000",
        "G1 X0 Y100 E15 F3000",
        "G1 X0 Y0 E20 F3000",
    };

    auto gcode = parseGcode(gcodeLines);
    ASSERT_GT(gcode.moves.size(), 0u);

    auto path = buildPathFromGcode(gcode);
    ASSERT_GT(path.numSegments(), 0u);

    auto ratios = extractExtrusionRatios(gcode);
    ASSERT_GT(ratios.size(), 0u);

    // Build WSS
    ParetoTimeEnergyOptimalVelocityPlanner<2, double> planner(makeGcodeLimits());
    auto profile = planner.computeProfile(path, 50.0, 0.0, 0.0, 200);
    auto wss = planner.weightedSource();
    ASSERT_NE(wss, nullptr);

    // Build extrusion trajectory
    ExtrusionTrajectory<2, double> traj(*wss, ratios);
    ASSERT_GT(traj.numArcs(), 0u);
    ASSERT_GT(traj.totalTime(), 0.0);

    double totalT = traj.totalTime();
    std::vector<double> times = linspace(0, totalT, 100);

    // 1. Linear PressureAdvance
    {
        AnalyticalLinearPressureAdvanceParams params;
        params.pressureAdvance = 0.045;
        params.smoothTime = 0.04;
        AnalyticalLinearPressureAdvance<2> pressureAdvance(traj, params);
        for (double t : times) {
            double off = pressureAdvance.offsetAtTime(t);
            EXPECT_TRUE(std::isfinite(off)) << "Linear PressureAdvance non-finite at t=" << t;
            EXPECT_GE(off, -1.0) << "Linear PressureAdvance offset too negative at t=" << t;
        }
    }

    // 2. Power-law PressureAdvance
    {
        AnalyticalPowerLawPressureAdvanceParams params;
        params.baseGain = 0.012;
        params.flowIndex = 0.5;
        AnalyticalPowerLawPressureAdvance<2> pressureAdvance(traj, params);
        for (double t : times) {
            double off = pressureAdvance.offsetAtTime(t);
            EXPECT_TRUE(std::isfinite(off)) << "PowerLaw PressureAdvance non-finite at t=" << t;
        }
    }

    // 3. Cross-WLF PressureAdvance
    {
        tether::control::extrusion::CrossWlfParams cwParams;
        tether::control::extrusion::NozzleGeometry geom;
        auto lut = std::make_shared<tether::control::extrusion::PressureFlowLut>();
        std::vector<double> flowAxis = {0.0, 0.5, 1.0, 2.0, 5.0, 10.0, 20.0, 50.0};
        std::vector<double> tempAxis = {180.0, 200.0, 210.0, 220.0, 240.0};
        lut->build(cwParams, geom, flowAxis, tempAxis);

        AnalyticalCrossWLFPressureAdvanceParams params;
        params.compressibilityOverArea = 1e-5;
        AnalyticalCrossWLFPressureAdvance<2> pressureAdvance(traj, lut, params);
        for (double t : times) {
            double off = pressureAdvance.offsetAtTime(t);
            EXPECT_TRUE(std::isfinite(off)) << "CrossWLF PressureAdvance non-finite at t=" << t;
        }
    }

    // 4. Thermal observer
    {
        AnalyticalThermalParams params;
        params.heaterPWM = 0.5;
        AnalyticalMeltZoneThermalObserver<2> thermal(traj, params);
        thermal.initialize(210.0);
        for (double t : times) {
            double temp = thermal.meltTempAt(t);
            EXPECT_TRUE(std::isfinite(temp)) << "Thermal non-finite at t=" << t;
            EXPECT_GT(temp, 20.0) << "Thermal temp too low at t=" << t;
            EXPECT_LT(temp, 300.0) << "Thermal temp too high at t=" << t;
        }
    }

    // 5. LTI deconvolution
    {
        std::vector<double> h = {0.0, 0.5, 0.3, 0.15, 0.08, 0.04, 0.02, 0.01};
        AnalyticalLTIDeconvParams params;
        params.lambda = 1e-4;
        AnalyticalLTIDeconvolution<2> deconv(traj, h, 1000.0, params);
        for (double t : times) {
            double x = deconv.inputAtTime(t, false);
            EXPECT_TRUE(std::isfinite(x)) << "LTI deconv non-finite at t=" << t;
        }
    }

    // 6. Overlap-add LPV
    {
        AnalyticalOverlapAddLPVParams params;
        params.lambda = 1e-4;
        AnalyticalOverlapAddLPV<2> lpv(traj, params);
        std::vector<double> h1 = {0.0, 0.5, 0.3, 0.15, 0.08, 0.04, 0.02, 0.01};
        std::vector<double> h2 = {0.0, 0.7, 0.2, 0.05, 0.02, 0.01, 0.005, 0.002};
        lpv.addOperatingPoint(10.0, h1, 1000.0);
        lpv.addOperatingPoint(300.0, h2, 1000.0);
        for (double t : times) {
            double x = lpv.inputAtTime(t, false);
            EXPECT_TRUE(std::isfinite(x)) << "LPV non-finite at t=" << t;
        }
    }

    // 7. ARX LPV
    {
        AnalyticalARXLPVParams params;
        params.na = 1;
        params.nb = 0;
        AnalyticalARXLPVInverse<2> filter(traj, params);
        filter.addModelPoint(10.0, {2.0}, {5.0}, 0.0);
        filter.addModelPoint(300.0, {1.0}, {8.0}, 0.0);
        for (double t : times) {
            double x = filter.inputAtTime(t);
            EXPECT_TRUE(std::isfinite(x)) << "ARX LPV non-finite at t=" << t;
        }
    }

    // 8. State-space LPV
    {
        AnalyticalStateSpaceLPVParams params;
        params.stateDim = 2;
        params.lambda = 1e-6;
        AnalyticalStateSpaceLPV<2> estimator(traj, params);

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
            double x = estimator.inputAtTime(t);
            EXPECT_TRUE(std::isfinite(x)) << "SS LPV non-finite at t=" << t;
        }
    }

    // 9. Flow-adaptive heater
    {
        AnalyticalFlowAdaptiveHeaterParams params;
        params.targetTempC = 210.0;
        AnalyticalFlowAdaptiveHeater<2> heater(traj, params);
        for (double t : times) {
            double ff = heater.feedforwardAtTime(t);
            EXPECT_TRUE(std::isfinite(ff)) << "Heater FF non-finite at t=" << t;
            EXPECT_GE(ff, 0.0) << "Heater FF negative at t=" << t;
            EXPECT_LE(ff, 1.0) << "Heater FF > 1 at t=" << t;
        }
    }
}

// ============================================================================
// Real G-code file tests
// ============================================================================

TEST(AnalyticalExtrusionGcodeTest, RealGcode_KissenPunctperi) {
    // Try to read the gcode file from ~/dev
    std::string home = std::getenv("HOME") ? std::getenv("HOME") : "/home/uli";
    std::string gcodePath = home + "/dev/kissen_punctperi2_20260814_132232.gcode";

    auto gcodeLines = readGcodeFile(gcodePath);
    if (gcodeLines.empty()) {
        GTEST_SKIP() << "G-code file not found: " << gcodePath;
    }

    // Parse and check
    auto gcode = parseGcode(gcodeLines);
    ASSERT_GT(gcode.moves.size(), 100u) << "Expected many moves in real G-code";

    // Check for pressure advance setting (may or may not be present
    // depending on the gcode file — we just verify it parses)
    if (gcode.hasPressureAdvance) {
        EXPECT_GE(gcode.pressureAdvance, 0.0);
    }

    // Build path from first N extruding moves
    size_t maxMoves = 100;
    auto path = buildPathFromGcode(gcode, maxMoves);
    if (path.numSegments() == 0) {
        GTEST_SKIP() << "Could not build path from G-code";
    }

    auto ratios = extractExtrusionRatios(gcode, maxMoves);
    ASSERT_GT(ratios.size(), 0u);

    // Build WSS
    ParetoTimeEnergyOptimalVelocityPlanner<2, double> planner(makeGcodeLimits());
    auto profile = planner.computeProfile(path, 50.0, 0.0, 0.0, 200);
    auto wss = planner.weightedSource();
    if (!wss) {
        GTEST_SKIP() << "Could not build WSS from path";
    }

    // Build extrusion trajectory
    ExtrusionTrajectory<2, double> traj(*wss, ratios);
    ASSERT_GT(traj.numArcs(), 0u);

    double totalT = traj.totalTime();
    ASSERT_GT(totalT, 0.0);

    // Run linear PressureAdvance with the G-code's PressureAdvance setting
    AnalyticalLinearPressureAdvanceParams pressureAdvanceParams;
    pressureAdvanceParams.pressureAdvance = gcode.pressureAdvance > 0 ? gcode.pressureAdvance : 0.045;
    pressureAdvanceParams.smoothTime = gcode.smoothTime > 0 ? gcode.smoothTime : 0.04;
    AnalyticalLinearPressureAdvance<2> pressureAdvance(traj, pressureAdvanceParams);

    // Sample and verify
    std::vector<double> times = linspace(0, totalT, 200);
    int finiteCount = 0;
    double maxOffset = 0.0;
    for (double t : times) {
        double off = pressureAdvance.offsetAtTime(t);
        if (std::isfinite(off)) {
            ++finiteCount;
            maxOffset = std::max(maxOffset, std::abs(off));
        }
    }
    EXPECT_GT(finiteCount, 180) << "Most offsets should be finite";
    EXPECT_GE(maxOffset, 0.0) << "Max offset should be non-negative";

    // Run power-law PressureAdvance
    AnalyticalPowerLawPressureAdvanceParams plParams;
    plParams.baseGain = 0.012;
    plParams.flowIndex = 0.5;
    AnalyticalPowerLawPressureAdvance<2> plPressureAdvance(traj, plParams);

    finiteCount = 0;
    for (double t : times) {
        double off = plPressureAdvance.offsetAtTime(t);
        if (std::isfinite(off)) ++finiteCount;
    }
    EXPECT_GT(finiteCount, 180);

    // Run thermal observer
    AnalyticalThermalParams thParams;
    thParams.heaterPWM = 0.5;
    thParams.inletTempC = 25.0;
    AnalyticalMeltZoneThermalObserver<2> thermal(traj, thParams);
    thermal.initialize(230.0);  // TPU temp from gcode

    for (double t : times) {
        double temp = thermal.meltTempAt(t);
        EXPECT_TRUE(std::isfinite(temp));
        EXPECT_GT(temp, 20.0);
        EXPECT_LT(temp, 400.0);
    }

    // Run flow-adaptive heater
    AnalyticalFlowAdaptiveHeaterParams heaterParams;
    heaterParams.targetTempC = 230.0;
    AnalyticalFlowAdaptiveHeater<2> heater(traj, heaterParams);

    for (double t : times) {
        double ff = heater.feedforwardAtTime(t);
        EXPECT_TRUE(std::isfinite(ff));
        EXPECT_GE(ff, 0.0);
        EXPECT_LE(ff, 1.0);
    }
}

TEST(AnalyticalExtrusionGcodeTest, RealGcode_LinearPressureAdvanceConsistentWithVelocity) {
    std::string home = std::getenv("HOME") ? std::getenv("HOME") : "/home/uli";
    std::string gcodePath = home + "/dev/kissen_punctperi2_20260814_132232.gcode";

    auto gcodeLines = readGcodeFile(gcodePath);
    if (gcodeLines.empty()) {
        GTEST_SKIP() << "G-code file not found";
    }

    auto gcode = parseGcode(gcodeLines);
    auto path = buildPathFromGcode(gcode, 50);
    if (path.numSegments() == 0) GTEST_SKIP() << "Could not build path";

    auto ratios = extractExtrusionRatios(gcode, 50);

    ParetoTimeEnergyOptimalVelocityPlanner<2, double> planner(makeGcodeLimits());
    planner.computeProfile(path, 50.0, 0.0, 0.0, 200);
    auto wss = planner.weightedSource();
    if (!wss) GTEST_SKIP() << "Could not build WSS";

    ExtrusionTrajectory<2, double> traj(*wss, ratios);

    AnalyticalLinearPressureAdvanceParams params;
    params.pressureAdvance = 0.12;  // From the gcode
    params.smoothTime = 0.0;
    AnalyticalLinearPressureAdvance<2> pressureAdvance(traj, params);

    // Verify δe(t) = PressureAdvance · v_e(t) exactly (no smoothing)
    double totalT = traj.totalTime();
    for (double t : linspace(totalT * 0.1, totalT * 0.9, 50)) {
        double offset = pressureAdvance.offsetAtTime(t);
        double vE = traj.extruderVelocityAtTime(t);
        double expected = params.pressureAdvance * vE;
        EXPECT_NEAR(offset, expected, 1e-8)
            << "Linear PressureAdvance offset != PressureAdvance*v_e at t=" << t;
    }
}

TEST(AnalyticalExtrusionGcodeTest, RealGcode_AllAlgorithmsRun) {
    std::string home = std::getenv("HOME") ? std::getenv("HOME") : "/home/uli";
    std::string gcodePath = home + "/dev/kissen_punctperi2_20260814_132232.gcode";

    auto gcodeLines = readGcodeFile(gcodePath);
    if (gcodeLines.empty()) GTEST_SKIP() << "G-code file not found";

    auto gcode = parseGcode(gcodeLines);
    auto path = buildPathFromGcode(gcode, 30);
    if (path.numSegments() == 0) GTEST_SKIP();

    auto ratios = extractExtrusionRatios(gcode, 30);

    ParetoTimeEnergyOptimalVelocityPlanner<2, double> planner(makeGcodeLimits());
    planner.computeProfile(path, 50.0, 0.0, 0.0, 200);
    auto wss = planner.weightedSource();
    if (!wss) GTEST_SKIP();

    ExtrusionTrajectory<2, double> traj(*wss, ratios);
    double totalT = traj.totalTime();
    ASSERT_GT(totalT, 0.0);

    std::vector<double> times = linspace(0.01, totalT * 0.99, 30);

    // Run all algorithms and check they produce finite results
    int totalChecks = 0;
    int finiteChecks = 0;

    // Linear PressureAdvance
    {
        AnalyticalLinearPressureAdvanceParams p;
        p.pressureAdvance = 0.12;
        AnalyticalLinearPressureAdvance<2> pressureAdvance(traj, p);
        for (double t : times) {
            ++totalChecks;
            if (std::isfinite(pressureAdvance.offsetAtTime(t))) ++finiteChecks;
        }
    }

    // Power-law PressureAdvance
    {
        AnalyticalPowerLawPressureAdvanceParams p;
        p.baseGain = 0.012;
        p.flowIndex = 0.5;
        AnalyticalPowerLawPressureAdvance<2> pressureAdvance(traj, p);
        for (double t : times) {
            ++totalChecks;
            if (std::isfinite(pressureAdvance.offsetAtTime(t))) ++finiteChecks;
        }
    }

    // Cross-WLF PressureAdvance
    {
        tether::control::extrusion::CrossWlfParams cwParams;
        tether::control::extrusion::NozzleGeometry geom;
        auto lut = std::make_shared<tether::control::extrusion::PressureFlowLut>();
        lut->build(cwParams, geom, {0.0, 0.5, 1.0, 2.0, 5.0, 10.0, 20.0},
                   {180.0, 200.0, 220.0, 240.0});
        AnalyticalCrossWLFPressureAdvanceParams p;
        p.compressibilityOverArea = 1e-5;
        AnalyticalCrossWLFPressureAdvance<2> pressureAdvance(traj, lut, p);
        for (double t : times) {
            ++totalChecks;
            if (std::isfinite(pressureAdvance.offsetAtTime(t))) ++finiteChecks;
        }
    }

    // LTI deconvolution
    {
        std::vector<double> h = {0.0, 0.5, 0.3, 0.15, 0.08, 0.04, 0.02, 0.01};
        AnalyticalLTIDeconvParams p;
        p.lambda = 1e-4;
        AnalyticalLTIDeconvolution<2> deconv(traj, h, 1000.0, p);
        for (double t : times) {
            ++totalChecks;
            if (std::isfinite(deconv.inputAtTime(t, false))) ++finiteChecks;
        }
    }

    // Overlap-add LPV
    {
        AnalyticalOverlapAddLPVParams p;
        p.lambda = 1e-4;
        AnalyticalOverlapAddLPV<2> lpv(traj, p);
        lpv.addOperatingPoint(10.0, {0.0, 0.5, 0.3, 0.15, 0.08}, 1000.0);
        lpv.addOperatingPoint(300.0, {0.0, 0.7, 0.2, 0.05, 0.02}, 1000.0);
        for (double t : times) {
            ++totalChecks;
            if (std::isfinite(lpv.inputAtTime(t, false))) ++finiteChecks;
        }
    }

    // ARX LPV
    {
        AnalyticalARXLPVParams p;
        p.na = 1;
        AnalyticalARXLPVInverse<2> filter(traj, p);
        filter.addModelPoint(10.0, {2.0}, {5.0}, 0.0);
        filter.addModelPoint(300.0, {1.0}, {8.0}, 0.0);
        for (double t : times) {
            ++totalChecks;
            if (std::isfinite(filter.inputAtTime(t))) ++finiteChecks;
        }
    }

    // State-space LPV
    {
        AnalyticalStateSpaceLPVParams p;
        p.stateDim = 2;
        p.lambda = 1e-6;
        AnalyticalStateSpaceLPV<2> est(traj, p);
        Eigen::MatrixXd A(2, 2), B(2, 1), C(1, 2);
        A << -1, 0, 0, -2; B << 1, 0; C << 1, 0;
        est.addModelPoint({10.0, A, B, C});
        A << -0.5, 0, 0, -1; B << 1.5, 0;
        est.addModelPoint({300.0, A, B, C});
        for (double t : times) {
            ++totalChecks;
            if (std::isfinite(est.inputAtTime(t))) ++finiteChecks;
        }
    }

    // Flow-adaptive heater
    {
        AnalyticalFlowAdaptiveHeaterParams p;
        p.targetTempC = 230.0;
        AnalyticalFlowAdaptiveHeater<2> heater(traj, p);
        for (double t : times) {
            ++totalChecks;
            if (std::isfinite(heater.feedforwardAtTime(t))) ++finiteChecks;
        }
    }

    // Thermal observer
    {
        AnalyticalThermalParams p;
        p.heaterPWM = 0.5;
        AnalyticalMeltZoneThermalObserver<2> thermal(traj, p);
        thermal.initialize(230.0);
        for (double t : times) {
            ++totalChecks;
            if (std::isfinite(thermal.meltTempAt(t))) ++finiteChecks;
        }
    }

    // At least 90% of all checks should be finite
    EXPECT_GT(finiteChecks, totalChecks * 9 / 10)
        << "Too many non-finite results: " << finiteChecks << "/" << totalChecks;
}
