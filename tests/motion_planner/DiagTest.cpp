#include <gtest/gtest.h>
#include <tether/motion_planner/MotionPlanner.hpp>
#include <tether/motion_planner/MotionSegment.hpp>
#include <tether/motion_planner/blend/BlendSpec.hpp>
#include <tether/motion_planner/analytical/ParetoTimeEnergyOptimalVelocityPlanner.hpp>
#include <cstdio>

using namespace MotionPlanner;
using namespace MotionPlanner::analytical;

TEST(DiagTest, TrapezoidDiag) {
    const double L = 100.0, V = 100.0, A = 500.0/3.0;
    MotionSegmentList segments;
    segments.append(MotionSegment::linear(
        Vec<2, double>{0.0, 0.0}, Vec<2, double>{L, 0.0}, 100.0));
    PathBuilderAdapter<2, double> builder;
    tether::motion::BlendSpec spec;
    spec.tolerance = 0.1;
    spec.continuity = tether::motion::Continuity::G2;
    spec.maxBlendFraction = 0.25;
    auto result = builder.build(segments, spec);
    ASSERT_TRUE(result.success);
    auto& path = result.path;
    std::fprintf(stderr, "path length=%.3f numSegs=%zu\n", path.totalLength(), path.numSegments());
    for (size_t i = 0; i < path.numSegments(); ++i) {
        std::fprintf(stderr, "  seg[%zu] cumS=%.3f len=%.3f\n", i,
                     path.segments()[i].cumulativeArcLength,
                     path.segments()[i].arcLength);
    }

    KinematicLimits<2, double> limits;
    limits.path.maxPathVelocity = V;
    limits.path.maxPathAcceleration = A;
    limits.path.maxPathJerk = 0.0;
    limits.path.jerkLimitEnabled = false;
    limits.path.maxCentripetalAcceleration = A;
    for (int i = 0; i < 2; ++i) {
        limits.axis.maxVelocity[i] = V;
        limits.axis.maxAcceleration[i] = A;
        limits.axis.maxJerk[i] = 0.0;
    }
    limits.axis.jerkLimitEnabled = false;

    CostWeights w;
    w.w_t = 1.0;
    w.w_a = 0.0;

    ParetoTimeEnergyOptimalVelocityPlanner<2> profiler(limits, w);
    auto profile = profiler.computeProfile(path, V, 0.0, 0.0, 500);
    auto wss = profiler.weightedSource();
    ASSERT_NE(wss, nullptr);
    std::fprintf(stderr, "a*=%.6f cost=%.6f T=%.6f arcs=%zu\n",
                 profiler.optimalAStar(), profiler.costValue(),
                 wss->totalTime(), wss->arcs().size());
    for (size_t i = 0; i < wss->arcs().size() && i < 20; ++i) {
        const auto& a = wss->arcs()[i];
        std::fprintf(stderr, "  arc[%zu] type=%s s0=%.3f s1=%.3f v0=%.3f a0=%.3f eta=%.3f a*=%.3f dur=%.6f\n",
                     i, weightedArcTypeName(a.type), a.s0, a.s1, a.v0, a.a0, a.eta, a.a_star, a.duration);
    }
}
