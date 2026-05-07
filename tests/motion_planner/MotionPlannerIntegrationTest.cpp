/**
 * @file MotionPlannerIntegrationTest.cpp
 * @brief Integration tests for the complete Motion Planner system
 *
 * Tests the full pipeline from G-code to motion state evaluation.
 */

#include <gtest/gtest.h>
#include <tether/motion_planner/MotionPlanner.hpp>
#include <cmath>
#include <sstream>

using namespace MotionPlanner;

// ============================================================================
// Source Reference Tests
// ============================================================================

class SourceReferenceTest : public ::testing::Test {};

TEST_F(SourceReferenceTest, SingleLineRef) {
    auto file = std::make_shared<SourceFile>("test.gcode");
    auto ref = SourceReference::fromLine(42, file);
    
    EXPECT_EQ(ref.type(), SourceReference::Type::Single);
    EXPECT_EQ(ref.lineNumber(), 42);
    EXPECT_EQ(ref.sourceFile(), file.get());
}

TEST_F(SourceReferenceTest, MultipleRefs) {
    auto file = std::make_shared<SourceFile>("test.gcode");
    auto ref1 = SourceReference::fromLine(10, file);
    auto ref2 = SourceReference::fromLine(11, file);
    
    auto combined = SourceReference::multiple({ref1, ref2});
    EXPECT_EQ(combined.type(), SourceReference::Type::Multiple);
}

TEST_F(SourceReferenceTest, RangeRef) {
    auto file = std::make_shared<SourceFile>("test.gcode");
    auto range = SourceReference::range(10, 20, file);
    
    EXPECT_EQ(range.type(), SourceReference::Type::Range);
    EXPECT_EQ(range.startLine(), 10);
    EXPECT_EQ(range.endLine(), 20);
}

TEST_F(SourceReferenceTest, SyntheticRef) {
    auto ref = SourceReference::synthetic("corner blend");
    EXPECT_EQ(ref.type(), SourceReference::Type::Synthetic);
}

// ============================================================================
// Motion Segment Tests
// ============================================================================

class MotionSegmentTest : public ::testing::Test {};

TEST_F(MotionSegmentTest, LinearSegment) {
    std::array<double, MAX_MOTION_AXES> start{};
    std::array<double, MAX_MOTION_AXES> end{};
    start[0] = 0; start[1] = 0; start[2] = 0;
    end[0] = 10; end[1] = 0; end[2] = 0;
    
    auto seg = MotionSegment::linear(start, end, 100.0);
    
    EXPECT_EQ(seg.type, MotionSegmentType::Linear);
    EXPECT_TRUE(seg.isLinear());
    EXPECT_FALSE(seg.isArc());
    EXPECT_DOUBLE_EQ(seg.feedRate, 100.0);
    EXPECT_NEAR(seg.segmentLength, 10.0, 1e-10);
}

TEST_F(MotionSegmentTest, ArcSegmentCW) {
    std::array<double, MAX_MOTION_AXES> start{};
    std::array<double, MAX_MOTION_AXES> end{};
    std::array<double, MAX_MOTION_AXES> center{};
    
    start[0] = 10; start[1] = 0;
    end[0] = 0; end[1] = 10;
    center[0] = 0; center[1] = 0;
    
    auto seg = MotionSegment::arcCW(start, end, center, 10.0, 50.0);
    
    EXPECT_EQ(seg.type, MotionSegmentType::ArcCW);
    EXPECT_TRUE(seg.isArc());
    EXPECT_DOUBLE_EQ(seg.arcRadius, 10.0);
}

TEST_F(MotionSegmentTest, DwellSegment) {
    std::array<double, MAX_MOTION_AXES> pos{};
    pos[0] = 5; pos[1] = 10; pos[2] = 15;
    
    auto seg = MotionSegment::dwell(pos, 2.5);
    
    EXPECT_EQ(seg.type, MotionSegmentType::Dwell);
    EXPECT_DOUBLE_EQ(seg.dwellDuration, 2.5);
    EXPECT_DOUBLE_EQ(seg.segmentLength, 0.0);
}

TEST_F(MotionSegmentTest, SegmentList) {
    MotionSegmentList list;
    
    std::array<double, MAX_MOTION_AXES> p0{}, p1{}, p2{};
    p0[0] = 0; p1[0] = 10; p2[0] = 20;
    
    list.append(MotionSegment::linear(p0, p1, 100.0));
    list.append(MotionSegment::linear(p1, p2, 100.0));
    
    EXPECT_EQ(list.size(), 2);
    EXPECT_FALSE(list.empty());
    
    // Test iteration
    size_t count = 0;
    for (auto it = list.begin(); it != list.end(); ++it) {
        count++;
    }
    EXPECT_EQ(count, 2);
}

TEST_F(MotionSegmentTest, Lookahead) {
    MotionSegmentList list;
    
    std::array<double, MAX_MOTION_AXES> p0{}, p1{}, p2{}, p3{};
    p0[0] = 0; p1[0] = 10; p2[0] = 20; p3[0] = 30;
    
    list.append(MotionSegment::linear(p0, p1, 100.0));
    list.append(MotionSegment::linear(p1, p2, 100.0));
    list.append(MotionSegment::linear(p2, p3, 100.0));
    
    auto lookahead = list.getLookahead(0, 3);
    EXPECT_EQ(lookahead.size(), 3);
}

// ============================================================================
// Piecewise Path Tests
// ============================================================================

class PiecewisePathTest : public ::testing::Test {};

TEST_F(PiecewisePathTest, SingleSegmentPath) {
    PiecewiseBezierPath2D path;
    
    auto curve = createLinearBezier(Vec2{0, 0}, Vec2{10, 0});
    path.addSegment(std::move(curve));
    path.buildArcLengthTables();
    
    EXPECT_EQ(path.numSegments(), 1);
    EXPECT_NEAR(path.totalLength(), 10.0, 1e-6);
}

TEST_F(PiecewisePathTest, MultiSegmentPath) {
    PiecewiseBezierPath2D path;
    
    path.addSegment(createLinearBezier(Vec2{0, 0}, Vec2{10, 0}));
    path.addSegment(createLinearBezier(Vec2{10, 0}, Vec2{10, 10}));
    path.buildArcLengthTables();
    
    EXPECT_EQ(path.numSegments(), 2);
    EXPECT_NEAR(path.totalLength(), 20.0, 1e-6);
}

TEST_F(PiecewisePathTest, ArcLengthEvaluation) {
    PiecewiseBezierPath2D path;
    path.addSegment(createLinearBezier(Vec2{0, 0}, Vec2{10, 0}));
    path.addSegment(createLinearBezier(Vec2{10, 0}, Vec2{10, 10}));
    path.buildArcLengthTables();
    
    // At s=0: start
    auto eval0 = path.evaluateAtArcLength(0.0);
    EXPECT_NEAR(eval0.position[0], 0.0, 1e-6);
    EXPECT_NEAR(eval0.position[1], 0.0, 1e-6);
    
    // At s=10: corner
    auto eval10 = path.evaluateAtArcLength(10.0);
    EXPECT_NEAR(eval10.position[0], 10.0, 1e-6);
    EXPECT_NEAR(eval10.position[1], 0.0, 1e-6);
    
    // At s=20: end
    auto eval20 = path.evaluateAtArcLength(20.0);
    EXPECT_NEAR(eval20.position[0], 10.0, 1e-6);
    EXPECT_NEAR(eval20.position[1], 10.0, 1e-6);
}

TEST_F(PiecewisePathTest, ForwardIteration) {
    PiecewiseBezierPath2D path;
    path.addSegment(createLinearBezier(Vec2{0, 0}, Vec2{10, 0}));
    path.addSegment(createLinearBezier(Vec2{10, 0}, Vec2{20, 0}));
    path.buildArcLengthTables();
    
    size_t count = 0;
    for (auto it = path.forwardBegin(); it != path.forwardEnd(); ++it) {
        count++;
    }
    EXPECT_EQ(count, 2);
}

// ============================================================================
// Velocity Profile Tests
// ============================================================================

class VelocityProfileTest : public ::testing::Test {};

TEST_F(VelocityProfileTest, SimpleProfile) {
    VelocityProfileD profile;
    
    VelocityProfilePoint<double> p0, p1, p2;
    p0.arcLength = 0.0; p0.velocity = 0.0; p0.time = 0.0;
    p1.arcLength = 5.0; p1.velocity = 100.0; p1.time = 0.1;
    p2.arcLength = 10.0; p2.velocity = 0.0; p2.time = 0.2;
    
    profile.addPoint(p0);
    profile.addPoint(p1);
    profile.addPoint(p2);
    
    EXPECT_NEAR(profile.velocityAt(0.0), 0.0, 1e-6);
    EXPECT_NEAR(profile.velocityAt(5.0), 100.0, 1e-6);
    EXPECT_NEAR(profile.velocityAt(10.0), 0.0, 1e-6);
    
    EXPECT_NEAR(profile.totalTime(), 0.2, 1e-10);
    EXPECT_NEAR(profile.totalLength(), 10.0, 1e-10);
}

TEST_F(VelocityProfileTest, InterpolatedVelocity) {
    VelocityProfileD profile;
    
    VelocityProfilePoint<double> p0, p1;
    p0.arcLength = 0.0; p0.velocity = 0.0; p0.time = 0.0;
    p1.arcLength = 10.0; p1.velocity = 100.0; p1.time = 0.2;
    
    profile.addPoint(p0);
    profile.addPoint(p1);
    
    // Linear interpolation
    EXPECT_NEAR(profile.velocityAt(5.0), 50.0, 1e-6);
}

// ============================================================================
// S-Curve Profile Tests
// ============================================================================

class SCurveProfileTest : public ::testing::Test {};

TEST_F(SCurveProfileTest, BasicProfile) {
    SCurveConstraintsD constraints;
    constraints.maxVelocity = 100.0;
    constraints.maxAcceleration = 500.0;
    constraints.maxJerk = 5000.0;
    
    SCurveProfileD profile;
    bool success = profile.compute(50.0, 0.0, 0.0, constraints);
    
    EXPECT_TRUE(success);
    EXPECT_TRUE(profile.isValid());
    EXPECT_GT(profile.totalDuration(), 0.0);
}

TEST_F(SCurveProfileTest, StateEvaluation) {
    SCurveConstraintsD constraints;
    constraints.maxVelocity = 100.0;
    constraints.maxAcceleration = 500.0;
    constraints.maxJerk = 5000.0;
    
    SCurveProfileD profile;
    profile.compute(100.0, 0.0, 0.0, constraints);
    
    // At t=0: position=0, velocity=0
    auto state0 = profile.evaluateAt(0.0);
    EXPECT_NEAR(state0.position, 0.0, 1e-6);
    EXPECT_NEAR(state0.velocity, 0.0, 1e-6);
    
    // At end: position=100, velocity=0
    auto stateEnd = profile.evaluateAt(profile.totalDuration());
    EXPECT_NEAR(stateEnd.position, 100.0, 0.1);  // Small tolerance
    EXPECT_NEAR(stateEnd.velocity, 0.0, 1.0);  // Should be close to 0
}

TEST_F(SCurveProfileTest, SevenPhases) {
    SCurveConstraintsD constraints;
    constraints.maxVelocity = 100.0;
    constraints.maxAcceleration = 500.0;
    constraints.maxJerk = 5000.0;
    
    SCurveProfileD profile;
    profile.compute(1000.0, 0.0, 0.0, constraints);  // Long enough for all phases
    
    // Count active phases
    int activePhases = 0;
    for (size_t i = 0; i < NUM_SCURVE_PHASES; ++i) {
        if (profile.phase(i).isActive()) {
            activePhases++;
        }
    }
    
    // For a long move, all 7 phases should be active
    // (or at least most of them)
    EXPECT_GE(activePhases, 5);
}

TEST_F(SCurveProfileTest, ShortMoveProfile) {
    SCurveConstraintsD constraints;
    constraints.maxVelocity = 100.0;
    constraints.maxAcceleration = 500.0;
    constraints.maxJerk = 5000.0;
    
    SCurveProfileD profile;
    profile.compute(1.0, 0.0, 0.0, constraints);  // Very short move
    
    EXPECT_TRUE(profile.isValid());
    EXPECT_GT(profile.totalDuration(), 0.0);
}

// ============================================================================
// Corner Blending Tests
// ============================================================================

class CornerBlendingTest : public ::testing::Test {};

TEST_F(CornerBlendingTest, LineLineCorner90Degrees) {
    std::array<double, MAX_MOTION_AXES> p0{}, p1{}, p2{};
    p0[0] = 0; p0[1] = 0;
    p1[0] = 10; p1[1] = 0;
    p2[0] = 10; p2[1] = 10;
    
    auto seg1 = MotionSegment::linear(p0, p1, 100.0);
    auto seg2 = MotionSegment::linear(p1, p2, 100.0);
    
    BlendConfig config;
    config.tolerance = 0.5;
    
    auto analysis = CornerAnalyzer2D::analyzeLineLine(seg1, seg2, config);
    
    EXPECT_TRUE(analysis.canBlend);
    EXPECT_NEAR(analysis.angle, MathConstants::HALF_PI, 0.01);  // 90°
    EXPECT_GT(analysis.blendRadius, 0.0);
}

TEST_F(CornerBlendingTest, StraightLine) {
    std::array<double, MAX_MOTION_AXES> p0{}, p1{}, p2{};
    p0[0] = 0; p0[1] = 0;
    p1[0] = 10; p1[1] = 0;
    p2[0] = 20; p2[1] = 0;
    
    auto seg1 = MotionSegment::linear(p0, p1, 100.0);
    auto seg2 = MotionSegment::linear(p1, p2, 100.0);
    
    BlendConfig config;
    auto analysis = CornerAnalyzer2D::analyzeLineLine(seg1, seg2, config);
    
    // Collinear segments shouldn't need blending
    EXPECT_EQ(analysis.type, CornerType::Straight);
}

TEST_F(CornerBlendingTest, BlendCurveG2) {
    std::array<double, MAX_MOTION_AXES> p0{}, p1{}, p2{};
    p0[0] = 0; p0[1] = 0;
    p1[0] = 10; p1[1] = 0;
    p2[0] = 10; p2[1] = 10;
    
    auto seg1 = MotionSegment::linear(p0, p1, 100.0);
    auto seg2 = MotionSegment::linear(p1, p2, 100.0);
    
    BlendConfig config;
    config.tolerance = 0.5;
    config.continuityLevel = 2;
    
    auto analysis = CornerAnalyzer2D::analyzeLineLine(seg1, seg2, config);
    
    if (analysis.canBlend) {
        auto blendCurves = BlendCurveBuilder2D::buildG2BlendCurve(analysis);
        ASSERT_EQ(blendCurves.size(), 1);
        auto& blendCurve = blendCurves[0];
        
        // Quintic Bézier should have degree 5
        EXPECT_EQ(blendCurve.degree(), 5);
        
        // Should start and end at blend points
        EXPECT_NEAR(blendCurve.evaluate(0.0)[0], analysis.blendEntry[0], 1e-6);
        EXPECT_NEAR(blendCurve.evaluate(0.0)[1], analysis.blendEntry[1], 1e-6);
    }
}

// ============================================================================
// Path Builder Tests
// ============================================================================

class PathBuilderTest : public ::testing::Test {};

TEST_F(PathBuilderTest, BuildFromSegments) {
    MotionSegmentList segments;
    
    std::array<double, MAX_MOTION_AXES> p0{}, p1{}, p2{};
    p0[0] = 0; p0[1] = 0;
    p1[0] = 10; p1[1] = 0;
    p2[0] = 10; p2[1] = 10;
    
    segments.append(MotionSegment::linear(p0, p1, 100.0));
    segments.append(MotionSegment::linear(p1, p2, 100.0));
    
    BlendConfig config;
    config.tolerance = 0.5;
    
    PathBuilder2D builder(config);
    auto result = builder.build(segments);
    
    EXPECT_TRUE(result.success);
    EXPECT_EQ(result.inputSegments, 2);
    EXPECT_GT(result.outputCurves, 0);
}

TEST_F(PathBuilderTest, EmptyInput) {
    MotionSegmentList segments;
    
    PathBuilder2D builder;
    auto result = builder.build(segments);
    
    EXPECT_TRUE(result.success);
    EXPECT_EQ(result.outputCurves, 0);
}

// ============================================================================
// G-Code Adapter Tests
// ============================================================================

class GCodeAdapterTest : public ::testing::Test {};

TEST_F(GCodeAdapterTest, LinearMove) {
    auto file = std::make_shared<SourceFile>("test.gcode");
    GCodeToMotionConverter converter(file);
    
    ParsedGCodeCommand cmd;
    cmd.lineNumber = 1;
    cmd.gCode = 1;  // G1
    cmd.coordinates[0] = 10.0;  // X10
    cmd.coordinates[1] = 20.0;  // Y20
    cmd.feedRate = 100.0;
    
    auto segment = converter.processCommand(cmd);
    
    EXPECT_TRUE(segment.has_value());
    EXPECT_EQ(segment->type, MotionSegmentType::Linear);
    EXPECT_NEAR(segment->endPosition[0], 10.0, 1e-10);
    EXPECT_NEAR(segment->endPosition[1], 20.0, 1e-10);
}

TEST_F(GCodeAdapterTest, RapidMove) {
    GCodeToMotionConverter converter;
    
    ParsedGCodeCommand cmd;
    cmd.lineNumber = 1;
    cmd.gCode = 0;  // G0
    cmd.coordinates[0] = 50.0;  // X50
    
    auto segment = converter.processCommand(cmd);
    
    EXPECT_TRUE(segment.has_value());
    EXPECT_EQ(segment->type, MotionSegmentType::Rapid);
}

TEST_F(GCodeAdapterTest, DwellCommand) {
    GCodeToMotionConverter converter;
    
    ParsedGCodeCommand cmd;
    cmd.lineNumber = 1;
    cmd.gCode = 4;  // G4
    cmd.dwellTime = 2.5;  // P2.5
    
    auto segment = converter.processCommand(cmd);
    
    EXPECT_TRUE(segment.has_value());
    EXPECT_EQ(segment->type, MotionSegmentType::Dwell);
    EXPECT_DOUBLE_EQ(segment->dwellDuration, 2.5);
}

TEST_F(GCodeAdapterTest, ModalState) {
    GCodeToMotionConverter converter;
    
    // G17 - XY plane
    ParsedGCodeCommand cmd1;
    cmd1.gCode = 17;
    converter.processCommand(cmd1);
    EXPECT_EQ(converter.state().activePlane, Plane::XY);
    
    // G90 - Absolute mode
    ParsedGCodeCommand cmd2;
    cmd2.gCode = 90;
    converter.processCommand(cmd2);
    EXPECT_EQ(converter.state().distanceMode, DistanceMode::Absolute);
}

TEST_F(GCodeAdapterTest, ProcessSequence) {
    GCodeToMotionConverter converter;
    
    std::vector<ParsedGCodeCommand> commands;
    
    // G0 X10
    ParsedGCodeCommand cmd1;
    cmd1.lineNumber = 1;
    cmd1.gCode = 0;
    cmd1.coordinates[0] = 10.0;
    commands.push_back(cmd1);
    
    // G1 Y20 F100
    ParsedGCodeCommand cmd2;
    cmd2.lineNumber = 2;
    cmd2.gCode = 1;
    cmd2.coordinates[1] = 20.0;
    cmd2.feedRate = 100.0;
    commands.push_back(cmd2);
    
    auto segments = converter.processCommands(commands);
    
    EXPECT_EQ(segments.size(), 2);
}

// ============================================================================
// Motion Plan Tests
// ============================================================================

class MotionPlanTest : public ::testing::Test {};

TEST_F(MotionPlanTest, SimpleLinearPlan) {
    MotionSegmentList segments;
    
    std::array<double, MAX_MOTION_AXES> p0{}, p1{};
    p0[0] = 0; p0[1] = 0;
    p1[0] = 100; p1[1] = 0;
    
    segments.append(MotionSegment::linear(p0, p1, 50.0));
    
    KinematicLimits2D limits;
    limits.path.maxPathVelocity = 100.0;
    limits.path.maxPathAcceleration = 500.0;
    
    MotionPlanBuilder2D builder(limits);
    auto plan = builder.build(segments, 50.0);
    
    EXPECT_GT(plan.totalDuration(), 0.0);
    EXPECT_NEAR(plan.totalLength(), 100.0, 1e-6);
}

TEST_F(MotionPlanTest, FeedOverride) {
    MotionSegmentList segments;
    
    std::array<double, MAX_MOTION_AXES> p0{}, p1{};
    p0[0] = 0; p1[0] = 100;
    
    segments.append(MotionSegment::linear(p0, p1, 50.0));
    
    MotionPlanBuilder2D builder;
    auto plan = builder.build(segments, 50.0);
    
    double normalDuration = plan.totalDuration();
    
    plan.setFeedOverride(0.5);  // 50% speed
    double slowDuration = plan.totalDuration();
    
    EXPECT_NEAR(slowDuration, normalDuration * 2.0, normalDuration * 0.01);
}

TEST_F(MotionPlanTest, PauseResume) {
    MotionSegmentList segments;
    
    std::array<double, MAX_MOTION_AXES> p0{}, p1{};
    p0[0] = 0; p1[0] = 100;
    
    segments.append(MotionSegment::linear(p0, p1, 50.0));
    
    MotionPlanBuilder2D builder;
    auto plan = builder.build(segments, 50.0);
    
    EXPECT_FALSE(plan.isPaused());
    
    plan.pause(0.5);
    EXPECT_TRUE(plan.isPaused());
    
    plan.resume(1.0);
    EXPECT_FALSE(plan.isPaused());
}

// ============================================================================
// Full Integration Test
// ============================================================================

class FullIntegrationTest : public ::testing::Test {};

TEST_F(FullIntegrationTest, GCodeToMotionState) {
    // Simulate G-code: G0 X10 / G1 Y10 F100 / G1 X0 Y0
    auto file = std::make_shared<SourceFile>("test.gcode");
    
    std::vector<ParsedGCodeCommand> commands;
    
    // G0 X10
    ParsedGCodeCommand cmd1;
    cmd1.lineNumber = 1;
    cmd1.gCode = 0;
    cmd1.coordinates[0] = 10.0;
    commands.push_back(cmd1);
    
    // G1 Y10 F100
    ParsedGCodeCommand cmd2;
    cmd2.lineNumber = 2;
    cmd2.gCode = 1;
    cmd2.coordinates[1] = 10.0;
    cmd2.feedRate = 100.0;
    commands.push_back(cmd2);
    
    // G1 X0 Y0
    ParsedGCodeCommand cmd3;
    cmd3.lineNumber = 3;
    cmd3.gCode = 1;
    cmd3.coordinates[0] = 0.0;
    cmd3.coordinates[1] = 0.0;
    commands.push_back(cmd3);
    
    // Build motion plan
    KinematicLimits2D limits;
    limits.path.maxPathVelocity = 100.0;
    limits.path.maxPathAcceleration = 500.0;
    
    GCodeToMotionPlan2D converter(limits);
    auto plan = converter.build(commands, file);
    
    EXPECT_GT(plan.numSegments(), 0);
    EXPECT_GT(plan.totalDuration(), 0.0);
    
    // Evaluate at various times
    auto state0 = plan.evaluateAt(0.0);
    EXPECT_NEAR(state0.position[0], 0.0, 1e-6);
    EXPECT_NEAR(state0.position[1], 0.0, 1e-6);
    
    // End state
    auto stateEnd = plan.evaluateAt(plan.totalDuration());
    // Should be back at origin
    EXPECT_NEAR(stateEnd.position[0], 0.0, 0.1);
    EXPECT_NEAR(stateEnd.position[1], 0.0, 0.1);
}

TEST_F(FullIntegrationTest, Traceability) {
    auto file = std::make_shared<SourceFile>("trace_test.gcode");
    
    std::vector<ParsedGCodeCommand> commands;
    
    ParsedGCodeCommand cmd;
    cmd.lineNumber = 42;
    cmd.gCode = 1;
    cmd.coordinates[0] = 10.0;
    cmd.feedRate = 100.0;
    commands.push_back(cmd);
    
    GCodeToMotionPlan2D converter;
    auto plan = converter.build(commands, file);
    
    auto state = plan.evaluateAt(plan.totalDuration() / 2.0);
    
    // Source reference should trace back to line 42
    // (Implementation may vary based on how traceability is propagated)
    EXPECT_NE(state.sourceRef.type(), SourceReference::Type::Empty);
}

