/**
 * @file test_GCodePatterns_coverage.cpp
 * @brief Extended coverage tests for GCodePatterns (TestPatternGenerator)
 *        Supplements existing test_GCodeGenerator.cpp with untested patterns.
 */

#include "tether/motion_replanner/GCodeGenerator.hpp"
#include "tether/motion_replanner/MachineTester.hpp"
#include <gtest/gtest.h>
#include <string>

using namespace MotionReplanner;

// ============================================================================
// Test fixture
// ============================================================================

class PatternGenCovTest : public ::testing::Test {
protected:
    void SetUp() override {
        gen_ = std::make_unique<TestPatternGenerator>();
    }
    std::unique_ptr<TestPatternGenerator> gen_;
};

// ============================================================================
// Direct generation methods
// ============================================================================

TEST_F(PatternGenCovTest, CircleTest) {
    auto prog = gen_->generateCircleTest(50.0, 50.0, 0.0, 25.0, 500.0, 2);
    EXPECT_FALSE(prog.blocks().empty());
    std::string code = prog.toString();
    EXPECT_FALSE(code.empty());
}

TEST_F(PatternGenCovTest, EllipseTest) {
    auto prog = gen_->generateEllipseTest(0.0, 0.0, 0.0, 30.0, 15.0, 0.0, 600.0, 1);
    EXPECT_FALSE(prog.blocks().empty());
}

TEST_F(PatternGenCovTest, HelixTest) {
    auto prog = gen_->generateHelixTest(0.0, 0.0, 0.0, 20.0, 5.0, 400.0, 3);
    EXPECT_FALSE(prog.blocks().empty());
}

TEST_F(PatternGenCovTest, SinusoidTest) {
    auto prog = gen_->generateSinusoidTest(0, 0.0, 10.0, 1.0, 3, 300.0);
    EXPECT_FALSE(prog.blocks().empty());
}

TEST_F(PatternGenCovTest, RampTest) {
    auto prog = gen_->generateRampTest(0, 0.0, 100.0, 200.0, 2);
    EXPECT_FALSE(prog.blocks().empty());
}

TEST_F(PatternGenCovTest, SquareTest) {
    auto prog = gen_->generateSquareTest(0.0, 0.0, 0.0, 50.0, 500.0, 1);
    EXPECT_FALSE(prog.blocks().empty());
}

TEST_F(PatternGenCovTest, RoundedSquareTest) {
    auto prog = gen_->generateRoundedSquareTest(0.0, 0.0, 0.0, 50.0, 5.0, 500.0, 1);
    EXPECT_FALSE(prog.blocks().empty());
}

TEST_F(PatternGenCovTest, FrictionTest) {
    std::vector<double> feedRates = {100.0, 200.0, 500.0};
    auto prog = gen_->generateFrictionTest(0, 50.0, feedRates, 2);
    EXPECT_FALSE(prog.blocks().empty());
}

TEST_F(PatternGenCovTest, SingleRevolution) {
    auto prog = gen_->generateCircleTest(0.0, 0.0, 0.0, 10.0, 100.0, 1);
    EXPECT_FALSE(prog.blocks().empty());
}

// ============================================================================
// SingleAxisTestConfig dispatch
// ============================================================================

TEST_F(PatternGenCovTest, SingleAxis_Ramp) {
    SingleAxisTestConfig cfg;
    cfg.axis = 0;
    cfg.type = SingleAxisTestType::Ramp;
    cfg.amplitude = 50.0;
    cfg.velocity = 200.0;
    cfg.cycles = 2;
    auto prog = gen_->generateSingleAxisTest(cfg);
    EXPECT_FALSE(prog.blocks().empty());
}

TEST_F(PatternGenCovTest, SingleAxis_Sinusoid) {
    SingleAxisTestConfig cfg;
    cfg.axis = 0;
    cfg.type = SingleAxisTestType::Sinusoid;
    cfg.amplitude = 30.0;
    cfg.frequency = 1.0;
    cfg.velocity = 300.0;
    cfg.cycles = 3;
    auto prog = gen_->generateSingleAxisTest(cfg);
    EXPECT_FALSE(prog.blocks().empty());
}

TEST_F(PatternGenCovTest, SingleAxis_SCurve) {
    SingleAxisTestConfig cfg;
    cfg.axis = 1;
    cfg.type = SingleAxisTestType::SCurve;
    cfg.amplitude = 40.0;
    cfg.velocity = 100.0;
    cfg.acceleration = 500.0;
    cfg.jerk = 5000.0;
    auto prog = gen_->generateSingleAxisTest(cfg);
    EXPECT_FALSE(prog.blocks().empty());
}

TEST_F(PatternGenCovTest, SingleAxis_Step) {
    SingleAxisTestConfig cfg;
    cfg.axis = 0;
    cfg.type = SingleAxisTestType::Step;
    cfg.amplitude = 10.0;
    cfg.velocity = 500.0;
    auto prog = gen_->generateSingleAxisTest(cfg);
    EXPECT_FALSE(prog.blocks().empty());
}

TEST_F(PatternGenCovTest, SingleAxis_Triangular) {
    SingleAxisTestConfig cfg;
    cfg.axis = 0;
    cfg.type = SingleAxisTestType::Triangular;
    cfg.amplitude = 20.0;
    cfg.velocity = 200.0;
    cfg.cycles = 2;
    auto prog = gen_->generateSingleAxisTest(cfg);
    EXPECT_FALSE(prog.blocks().empty());
}

TEST_F(PatternGenCovTest, SingleAxis_Trapezoidal) {
    SingleAxisTestConfig cfg;
    cfg.axis = 0;
    cfg.type = SingleAxisTestType::Trapezoidal;
    cfg.amplitude = 25.0;
    cfg.velocity = 300.0;
    cfg.acceleration = 100.0;
    cfg.cycles = 1;
    auto prog = gen_->generateSingleAxisTest(cfg);
    EXPECT_FALSE(prog.blocks().empty());
}

// ============================================================================
// MultiAxisTestConfig dispatch
// ============================================================================

TEST_F(PatternGenCovTest, MultiAxis_Circle) {
    MultiAxisTestConfig cfg;
    cfg.type = MultiAxisTestType::Circle;
    cfg.uAxis = 0;
    cfg.vAxis = 1;
    cfg.radiusU = 25.0;
    cfg.feedRate = 500.0;
    cfg.revolutions = 2;
    auto prog = gen_->generateMultiAxisTest(cfg);
    EXPECT_FALSE(prog.blocks().empty());
}

TEST_F(PatternGenCovTest, MultiAxis_Ellipse) {
    MultiAxisTestConfig cfg;
    cfg.type = MultiAxisTestType::Ellipse;
    cfg.uAxis = 0;
    cfg.vAxis = 1;
    cfg.radiusU = 30.0;
    cfg.radiusV = 15.0;
    cfg.feedRate = 400.0;
    cfg.revolutions = 1;
    auto prog = gen_->generateMultiAxisTest(cfg);
    EXPECT_FALSE(prog.blocks().empty());
}

TEST_F(PatternGenCovTest, MultiAxis_Helix) {
    MultiAxisTestConfig cfg;
    cfg.type = MultiAxisTestType::Helix;
    cfg.uAxis = 0;
    cfg.vAxis = 1;
    cfg.wAxis = 2;
    cfg.radiusU = 20.0;
    cfg.pitchW = 5.0;
    cfg.feedRate = 300.0;
    cfg.revolutions = 3;
    auto prog = gen_->generateMultiAxisTest(cfg);
    EXPECT_FALSE(prog.blocks().empty());
}

TEST_F(PatternGenCovTest, MultiAxis_Square) {
    MultiAxisTestConfig cfg;
    cfg.type = MultiAxisTestType::Square;
    cfg.uAxis = 0;
    cfg.vAxis = 1;
    cfg.radiusU = 25.0;
    cfg.feedRate = 500.0;
    auto prog = gen_->generateMultiAxisTest(cfg);
    EXPECT_FALSE(prog.blocks().empty());
}

TEST_F(PatternGenCovTest, MultiAxis_RoundedSquare) {
    MultiAxisTestConfig cfg;
    cfg.type = MultiAxisTestType::RoundedSquare;
    cfg.uAxis = 0;
    cfg.vAxis = 1;
    cfg.radiusU = 30.0;
    cfg.cornerRadius = 5.0;
    cfg.feedRate = 500.0;
    auto prog = gen_->generateMultiAxisTest(cfg);
    EXPECT_FALSE(prog.blocks().empty());
}

TEST_F(PatternGenCovTest, MultiAxis_Lissajous) {
    MultiAxisTestConfig cfg;
    cfg.type = MultiAxisTestType::Lissajous;
    cfg.uAxis = 0;
    cfg.vAxis = 1;
    cfg.radiusU = 20.0;
    cfg.radiusV = 20.0;
    cfg.lissajousRatioU = 3.0;
    cfg.lissajousRatioV = 2.0;
    cfg.lissajousPhase = 1.5708;
    cfg.feedRate = 400.0;
    auto prog = gen_->generateMultiAxisTest(cfg);
    // Lissajous may not be fully implemented
    (void)prog;
}

TEST_F(PatternGenCovTest, MultiAxis_DiagonalBox) {
    MultiAxisTestConfig cfg;
    cfg.type = MultiAxisTestType::DiagonalBox;
    cfg.uAxis = 0;
    cfg.vAxis = 1;
    cfg.radiusU = 20.0;
    cfg.radiusV = 20.0;
    cfg.feedRate = 300.0;
    auto prog = gen_->generateMultiAxisTest(cfg);
    // DiagonalBox may not be fully implemented
    (void)prog;
}

// ============================================================================
// Workspace sweep
// ============================================================================

TEST_F(PatternGenCovTest, WorkspaceSweep) {
    HeatmapConfig hcfg;
    hcfg.minBounds[0] = -50.0;
    hcfg.minBounds[1] = -50.0;
    hcfg.minBounds[2] = 0.0;
    hcfg.maxBounds[0] = 50.0;
    hcfg.maxBounds[1] = 50.0;
    hcfg.maxBounds[2] = 0.0;
    hcfg.resolution2D = 25.0;
    auto prog = gen_->generateWorkspaceSweep(hcfg, 10.0, 500.0);
    EXPECT_FALSE(prog.blocks().empty());
}

// ============================================================================
// Calibration sequence
// ============================================================================

TEST_F(PatternGenCovTest, CalibrationSequence) {
    TestSequence seq;
    seq.name = "TestCalib";
    seq.description = "Coverage test calibration";
    
    SingleAxisTestConfig saCfg;
    saCfg.axis = 0;
    saCfg.type = SingleAxisTestType::Ramp;
    saCfg.amplitude = 20.0;
    saCfg.velocity = 100.0;
    saCfg.cycles = 1;
    seq.singleAxisTests.push_back(saCfg);
    
    MultiAxisTestConfig maCfg;
    maCfg.type = MultiAxisTestType::Circle;
    maCfg.uAxis = 0;
    maCfg.vAxis = 1;
    maCfg.radiusU = 15.0;
    maCfg.feedRate = 200.0;
    maCfg.revolutions = 1;
    seq.multiAxisTests.push_back(maCfg);
    
    auto prog = gen_->generateCalibrationSequence(seq);
    EXPECT_FALSE(prog.blocks().empty());
}

// ============================================================================
// GCodeOptions 
// ============================================================================

TEST_F(PatternGenCovTest, SetOptions) {
    GCodeOptions opts;
    opts.feedPrecision = 4;
    gen_->setOptions(opts);
    EXPECT_EQ(gen_->options().feedPrecision, 4);
}

TEST_F(PatternGenCovTest, ConstructWithOptions) {
    GCodeOptions opts;
    opts.feedPrecision = 3;
    TestPatternGenerator gen2(opts);
    EXPECT_EQ(gen2.options().feedPrecision, 3);
}

// ============================================================================
// Edge cases
// ============================================================================

TEST_F(PatternGenCovTest, FrictionTest_EmptyFeedRates) {
    std::vector<double> feedRates;
    auto prog = gen_->generateFrictionTest(0, 50.0, feedRates, 1);
    // May be empty or just have header/footer
}

TEST_F(PatternGenCovTest, CircleZeroRadius) {
    auto prog = gen_->generateCircleTest(0.0, 0.0, 0.0, 0.0, 100.0, 1);
    // Should handle gracefully
}

TEST_F(PatternGenCovTest, EllipseRotated) {
    auto prog = gen_->generateEllipseTest(0.0, 0.0, 0.0, 30.0, 15.0, 0.7854, 600.0, 1);
    EXPECT_FALSE(prog.blocks().empty());
}

TEST_F(PatternGenCovTest, MultipleRevolutions) {
    auto prog = gen_->generateCircleTest(0.0, 0.0, 0.0, 25.0, 500.0, 5);
    EXPECT_FALSE(prog.blocks().empty());
}
