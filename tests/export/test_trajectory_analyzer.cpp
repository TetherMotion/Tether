#include <gtest/gtest.h>

#include "export/TrajectoryAnalyzer.hpp"

#include "gcode/motion/InterpolationStrategy.hpp"

#include <cmath>

namespace {

constexpr double kEps = 1e-9;

GCode::PlanningSegment makeLinearSegment(
    double x0,
    double x1,
    double segmentTime,
    int32_t blockIndex,
    GCode::SegmentMotionType motionType = GCode::SegmentMotionType::Linear
) {
    GCode::PlanningSegment seg;
    seg.start[0] = x0;
    seg.end[0] = x1;
    seg.motionType = motionType;
    seg.segmentTime = segmentTime;
    seg.segmentLength = std::abs(x1 - x0);
    seg.blockIndex = blockIndex;
    seg.plane = GCode::InterpolationPlane::XY;
    return seg;
}

GCode::PlanningSegment makeArcSegment(
    GCode::InterpolationPlane plane,
    double segmentTime,
    int32_t blockIndex
) {
    // Unit quarter-circle arc on the specified plane, CCW.
    // Start at (1,0) in (u,v) coordinates, end at (0,1).
    GCode::PlanningSegment seg;
    seg.motionType = GCode::SegmentMotionType::ArcCCW;
    seg.plane = plane;
    seg.segmentTime = segmentTime;
    seg.blockIndex = blockIndex;

    int u = 0, v = 1, w = 2;
    switch (plane) {
        case GCode::InterpolationPlane::XY: u = 0; v = 1; w = 2; break;
        case GCode::InterpolationPlane::XZ: u = 0; v = 2; w = 1; break;
        case GCode::InterpolationPlane::YZ: u = 1; v = 2; w = 0; break;
    }

    seg.center[u] = 0.0;
    seg.center[v] = 0.0;
    seg.center[w] = 5.0;  // constant perpendicular axis

    seg.start[u] = 1.0;
    seg.start[v] = 0.0;
    seg.start[w] = 5.0;

    seg.end[u] = 0.0;
    seg.end[v] = 1.0;
    seg.end[w] = 5.0;

    seg.arcRadius = 1.0;
    seg.arcSweep = M_PI / 2.0;
    seg.segmentLength = seg.arcRadius * std::abs(seg.arcSweep);
    return seg;
}

class StrategyThatOffsetsX final : public GCode::InterpolationStrategy {
public:
    GCode::InterpolationStrategyType type() const override {
        return GCode::InterpolationStrategyType::FixedTime;
    }
    const char* name() const override { return "StrategyThatOffsetsX"; }
    void configure(const GCode::InterpolationConfig&) override {}
    GCode::InterpolationResult interpolateSegment(
        const GCode::PlanningSegment&, GCode::InterpolationContext&, std::vector<GCode::TrajectoryPoint>&
    ) override {
        return {};
    }

    GCode::Position evaluatePosition(const GCode::PlanningSegment& segment, double t) const override {
        // Linear interpolation + offset so the analyzer's strategy path is exercised deterministically.
        GCode::Position pos;
        for (size_t i = 0; i < 9; ++i) {
            pos[i] = segment.start[i] + t * (segment.end[i] - segment.start[i]);
        }
        pos[0] += 123.0;
        return pos;
    }
};

}  // namespace

TEST(TrajectoryAnalyzerStandaloneTest, AnalyzeEmptySegmentsReturnsEmpty) {
    GCodeExport::TrajectoryAnalyzer analyzer;
    auto samples = analyzer.analyze({});
    EXPECT_TRUE(samples.empty());

    // Cover inline config accessors in the header.
    GCodeExport::AnalysisConfig cfg;
    cfg.timeStep = 0.123;
    analyzer.configure(cfg);
    EXPECT_NEAR(analyzer.config().timeStep, 0.123, 1e-12);
}

TEST(TrajectoryAnalyzerStandaloneTest, AnalyzeSkipsZeroTimeSegments) {
    GCodeExport::AnalysisConfig cfg;
    cfg.timeStep = 0.5;
    GCodeExport::TrajectoryAnalyzer analyzer(cfg);

    auto seg = makeLinearSegment(0.0, 1.0, 0.0, 7);
    auto samples = analyzer.analyze({seg});
    EXPECT_TRUE(samples.empty());
}

TEST(TrajectoryAnalyzerStandaloneTest, AnalyzeTwoLinearSegmentsAvoidsDuplicateBoundaryAndComputesDerivatives) {
    GCodeExport::AnalysisConfig cfg;
    cfg.timeStep = 0.5;
    cfg.violationTolerance = 0.0;
    GCodeExport::TrajectoryAnalyzer analyzer(cfg);

    auto s0 = makeLinearSegment(0.0, 1.0, 1.0, 10);
    auto s1 = makeLinearSegment(1.0, 2.0, 1.0, 11);

    auto samples = analyzer.analyze({s0, s1}, nullptr);
    ASSERT_EQ(samples.size(), 5u);

    // 0.0, 0.5, 1.0, 1.5, 2.0
    EXPECT_NEAR(samples.front().time, 0.0, kEps);
    EXPECT_NEAR(samples[2].time, 1.0, kEps);
    EXPECT_NEAR(samples.back().time, 2.0, kEps);

    // Positions are monotonic and path position accumulates.
    EXPECT_LT(samples[0].pathPosition, samples.back().pathPosition);
    EXPECT_NEAR(samples[0].position[0], 0.0, kEps);
    EXPECT_NEAR(samples.back().position[0], 2.0, kEps);

    // Derivatives should be constant for linear motion.
    for (const auto& sample : samples) {
        EXPECT_NEAR(sample.velocity[0], 1.0, 1e-12);
        EXPECT_NEAR(sample.acceleration[0], 0.0, 1e-12);
        EXPECT_NEAR(sample.linearVelocity, 1.0, 1e-12);
        EXPECT_NEAR(sample.curvature, 0.0, 1e-12);
        EXPECT_NEAR(sample.centripetalAccel, 0.0, 1e-12);
    }

    std::vector<GCodeExport::LimitViolation> violations;
    EXPECT_TRUE(analyzer.checkLimitCompliance(samples, &violations));
    EXPECT_TRUE(violations.empty());
}

TEST(TrajectoryAnalyzerStandaloneTest, StrategyBranchUsesEvaluatePosition) {
    GCodeExport::AnalysisConfig cfg;
    cfg.timeStep = 1.0;
    GCodeExport::TrajectoryAnalyzer analyzer(cfg);

    StrategyThatOffsetsX strategy;
    auto seg = makeLinearSegment(0.0, 10.0, 1.0, 12);
    auto samples = analyzer.analyze({seg}, &strategy);

    ASSERT_FALSE(samples.empty());
    EXPECT_NEAR(samples.front().position[0], 123.0, kEps);
    EXPECT_NEAR(samples.back().position[0], 133.0, kEps);
}

TEST(TrajectoryAnalyzerStandaloneTest, ArcInterpolationCoversAllPlanes) {
    GCodeExport::AnalysisConfig cfg;
    cfg.timeStep = 0.5;
    GCodeExport::TrajectoryAnalyzer analyzer(cfg);

    std::vector<GCode::PlanningSegment> segs;
    segs.push_back(makeArcSegment(GCode::InterpolationPlane::XY, 1.0, 20));
    segs.push_back(makeArcSegment(GCode::InterpolationPlane::XZ, 1.0, 21));
    segs.push_back(makeArcSegment(GCode::InterpolationPlane::YZ, 1.0, 22));

    auto samples = analyzer.analyze(segs, nullptr);
    ASSERT_GE(samples.size(), 5u);

    // Midpoint of the first arc (XY) should be approximately at angle pi/4.
    // Find a sample in segment 0 at around t=0.5; with cfg.timeStep=0.5 and 1s segment, it exists.
    auto it = std::find_if(samples.begin(), samples.end(), [](const auto& s) {
        return s.segmentIndex == 0 && std::abs(s.time - 0.5) < 1e-12;
    });
    ASSERT_TRUE(it != samples.end());
    EXPECT_NEAR(it->position[0], std::cos(M_PI / 4.0), 1e-12);
    EXPECT_NEAR(it->position[1], std::sin(M_PI / 4.0), 1e-12);
    EXPECT_NEAR(it->position[2], 5.0, 1e-12);
}

TEST(TrajectoryAnalyzerStandaloneTest, DerivativesEarlyReturnAndDtNonPositiveAreHandled) {
    GCodeExport::TrajectoryAnalyzer analyzer;

    // < 5 samples -> early return.
    std::vector<GCodeExport::TrajectorySample> few(4);
    for (size_t i = 0; i < few.size(); ++i) {
        few[i].time = static_cast<double>(i);
        few[i].position[0] = static_cast<double>(i);
    }
    analyzer.computeDerivatives(few, 4);
    for (const auto& s : few) {
        EXPECT_NEAR(s.velocity[0], 0.0, kEps);
        EXPECT_NEAR(s.acceleration[0], 0.0, kEps);
    }

    // dt <= 0 -> skip computation (times are identical).
    std::vector<GCodeExport::TrajectorySample> badTime(5);
    for (auto& s : badTime) {
        s.time = 1.0;
        s.position[0] = 42.0;
    }
    analyzer.computeDerivatives(badTime, 4);
    for (const auto& s : badTime) {
        EXPECT_NEAR(s.velocity[0], 0.0, kEps);
        EXPECT_NEAR(s.acceleration[0], 0.0, kEps);
    }
}

TEST(TrajectoryAnalyzerStandaloneTest, LimitComplianceReportsCombinedAndAxisViolations) {
    GCodeExport::AnalysisConfig cfg;
    cfg.violationTolerance = 0.0;
    cfg.limits.maxVelocityLinear = 60.0;  // 1 mm/s
    cfg.limits.maxAcceleration = 2.0;
    cfg.limits.maxJerk = 3.0;
    cfg.limits.axisMaxVelocity.fill(60.0);
    cfg.limits.axisMaxAcceleration.fill(2.0);
    cfg.limits.axisMaxJerk.fill(3.0);

    GCodeExport::TrajectoryAnalyzer analyzer(cfg);

    GCodeExport::TrajectorySample s;
    s.time = 1.25;
    s.linearVelocity = 2.0;
    s.linearAcceleration = 3.0;
    s.linearJerk = 4.0;
    s.velocity[0] = 2.0;
    s.acceleration[0] = 3.0;
    s.jerk[0] = 4.0;

    std::vector<GCodeExport::LimitViolation> violations;
    EXPECT_FALSE(analyzer.checkLimitCompliance({s}, &violations));
    EXPECT_EQ(violations.size(), 6u);
    for (const auto& v : violations) {
        EXPECT_NEAR(v.time, 1.25, kEps);
        EXPECT_GT(v.overshoot, 0.0);
    }

    // Also cover the nullptr output path.
    EXPECT_FALSE(analyzer.checkLimitCompliance({s}, nullptr));
}

TEST(TrajectoryAnalyzerStandaloneTest, ApproximationFactoryAndStrategiesWork) {
    // Factory creation
    auto fixedTime = GCodeExport::ApproximationFactory::create("FixedTime");
    auto fixedDev = GCodeExport::ApproximationFactory::create("FixedDeviation");
    auto trap = GCodeExport::ApproximationFactory::create("Trapezoidal");
    auto scurve = GCodeExport::ApproximationFactory::create("SCurve");
    EXPECT_NE(fixedTime, nullptr);
    EXPECT_NE(fixedDev, nullptr);
    EXPECT_NE(trap, nullptr);
    EXPECT_NE(scurve, nullptr);
    EXPECT_EQ(GCodeExport::ApproximationFactory::create("Nope"), nullptr);

    // Cover inline strategy names (header-only lines in src/export/TrajectoryAnalyzer.hpp).
    EXPECT_STREQ(fixedTime->name(), "FixedTime");
    EXPECT_STREQ(fixedDev->name(), "FixedDeviation");
    EXPECT_STREQ(trap->name(), "Trapezoidal");
    EXPECT_STREQ(scurve->name(), "SCurve");

    auto available = GCodeExport::ApproximationFactory::availableStrategies();
    EXPECT_EQ(available.size(), 4u);

    GCode::KinematicLimits limits;

    // Fixed-time configuration branch
    fixedTime->configure("timeStep", 0.5);

    // Fixed-deviation configuration branch
    fixedDev->configure("maxDeviation", 0.1);

    // Cover header-only setter.
    auto* fixedDevConcrete = dynamic_cast<GCodeExport::FixedDeviationApproximation*>(fixedDev.get());
    ASSERT_NE(fixedDevConcrete, nullptr);
    fixedDevConcrete->setMaxDeviation(0.1);

    // Trapezoidal configuration branches
    trap->configure("timeStep", 0.5);
    trap->configure("useJerkLimiting", 0.0);

    // S-curve configuration branch
    scurve->configure("timeStep", 0.5);

    // Build segments that hit both linear and arc branches in FixedDeviationApproximation.
    std::vector<GCode::PlanningSegment> segments;
    segments.push_back(makeLinearSegment(0.0, 1.0, 1.0, 30));
    segments.push_back(makeArcSegment(GCode::InterpolationPlane::XY, 1.0, 31));
    segments.push_back(makeArcSegment(GCode::InterpolationPlane::XZ, 1.0, 32));
    segments.push_back(makeArcSegment(GCode::InterpolationPlane::YZ, 1.0, 33));

    auto a = fixedTime->generateTrajectory(segments, limits);
    auto b = fixedDev->generateTrajectory(segments, limits);
    auto c = trap->generateTrajectory(segments, limits);
    auto d = scurve->generateTrajectory(segments, limits);

    EXPECT_FALSE(a.empty());
    EXPECT_FALSE(b.empty());
    EXPECT_FALSE(c.empty());
    EXPECT_FALSE(d.empty());
}
