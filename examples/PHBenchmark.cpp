/**
 * @file PHBenchmark.cpp
 * @brief Phase 5.4 benchmark: PH-on vs PH-off planning time on a multi-corner path.
 *
 * Builds a zigzag path with N corners, plans it once with the default
 * (Bezier G2) blend and once with PH forced on, and reports the
 * planning-time ratio. The result is printed to stdout for recording
 * in docs/motion/AlgorithmComparison.md.
 *
 * This is an informational benchmark, not a hard CI gate.
 */

#include <tether/motion_planner/MotionPlanner.hpp>
#include <tether/motion_planner/MotionSegment.hpp>
#include <tether/motion_planner/blend/BlendSpec.hpp>
#include <tether/motion_planner/VelocityProfile.hpp>
#include <tether/motion_planner/MotionPlan.hpp>

#include <chrono>
#include <iostream>
#include <vector>

using namespace MotionPlanner;
using namespace std::chrono;

/// Build a zigzag path with `numCorners` 90° corners.
/// Each leg is 10 units long; the path zigzags in X and Y.
MotionSegmentList buildZigzag(size_t numCorners) {
    MotionSegmentList segments;
    double x = 0, y = 0;
    for (size_t i = 0; i < numCorners + 1; ++i) {
        double nx = (i % 2 == 0) ? x + 10 : x;
        double ny = (i % 2 == 1) ? y + 10 : y;
        if (i == 0) { nx = 10; ny = 0; }
        segments.append(MotionSegment::linear(Vec2{x, y}, Vec2{nx, ny}, 100.0));
        x = nx; y = ny;
    }
    return segments;
}

/// Plan the path with the given curve type and return the planning time in ms.
/// Also reports the blend construction time and velocity planning time separately.
struct PlanTiming {
    double blendMs = 0;
    double velocityMs = 0;
    double totalMs = 0;
};

PlanTiming planWith(const MotionSegmentList& segments,
                tether::motion::BlendCurveType curveType,
                size_t numProfileSamples = 200) {
    PlanTiming timing;
    tether::motion::BlendSpec spec;
    spec.tolerance = 0.1;
    spec.maxBlendFraction = 0.25;
    spec.curveType = curveType;

    auto t0 = high_resolution_clock::now();

    PathBuilderAdapter<2, double> pathBuilder;
    auto pathResult = pathBuilder.build(segments, spec);

    auto t1 = high_resolution_clock::now();
    timing.blendMs = duration_cast<microseconds>(t1 - t0).count() / 1000.0;

    KinematicLimits<2, double> limits;
    VelocityProfiler<2, double> profiler(limits);
    auto profile = profiler.computeProfile(pathResult.path, 100.0,
                                           0.0, 0.0, numProfileSamples);

    MotionPlanConfig<double> config;
    MotionPlan2D plan(std::move(pathResult.path), std::move(profile), config);

    auto t2 = high_resolution_clock::now();
    timing.velocityMs = duration_cast<microseconds>(t2 - t1).count() / 1000.0;
    timing.totalMs = duration_cast<microseconds>(t2 - t0).count() / 1000.0;
    return timing;
}

int main(int argc, char** argv) {
    const size_t numCorners = (argc > 1) ? std::stoul(argv[1]) : 100;
    const size_t runs = (argc > 2) ? std::stoul(argv[2]) : 3;

    std::cout << "Phase 5.4 Benchmark: PH-on vs PH-off\n";
    std::cout << "Corners: " << numCorners << "\n";
    std::cout << "Runs: " << runs << " (reporting median)\n\n";

    std::vector<double> bezBlend, phBlend, bezVel, phVel, bezTotal, phTotal;
    for (size_t r = 0; r < runs; ++r) {
        auto segs1 = buildZigzag(numCorners);
        auto tBez = planWith(segs1, tether::motion::BlendCurveType::BezierGk);
        bezBlend.push_back(tBez.blendMs);
        bezVel.push_back(tBez.velocityMs);
        bezTotal.push_back(tBez.totalMs);

        auto segs2 = buildZigzag(numCorners);
        auto tPH = planWith(segs2, tether::motion::BlendCurveType::PHQuintic);
        phBlend.push_back(tPH.blendMs);
        phVel.push_back(tPH.velocityMs);
        phTotal.push_back(tPH.totalMs);

        std::cout << "  run " << (r+1)
                  << ": Bezier blend=" << tBez.blendMs << "ms"
                  << " vel=" << tBez.velocityMs << "ms"
                  << " total=" << tBez.totalMs << "ms"
                  << " | PH blend=" << tPH.blendMs << "ms"
                  << " vel=" << tPH.velocityMs << "ms"
                  << " total=" << tPH.totalMs << "ms\n";
    }

    auto median = [](std::vector<double>& v) {
        std::sort(v.begin(), v.end());
        return v[v.size() / 2];
    };
    double bb = median(bezBlend), pb = median(phBlend);
    double bv = median(bezVel), pv = median(phVel);
    double bt = median(bezTotal), pt = median(phTotal);

    std::cout << "\nMedian results:\n";
    std::cout << "  Blend construction:  Bezier=" << bb << "ms  PH=" << pb << "ms  ratio=" << pb/bb << "\n";
    std::cout << "  Velocity planning:   Bezier=" << bv << "ms  PH=" << pv << "ms  ratio=" << (bv > 0 ? pv/bv : 0) << "\n";
    std::cout << "  Total:               Bezier=" << bt << "ms  PH=" << pt << "ms  ratio=" << pt/bt << "\n";

    std::cout << "\n(For the docs: a " << numCorners << "-corner zigzag, "
              << "PH total planning took " << pt << "ms vs Bezier "
              << bt << "ms (ratio " << pt/bt << "); "
              << "blend construction " << pb << "ms vs " << bb << "ms (ratio " << pb/bb << "); "
              << "velocity planning " << pv << "ms vs " << bv << "ms.)\n";

    return 0;
}
