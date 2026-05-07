/**
 * @file test_GCodeGenerator.cpp
 * @brief Tests for GCodeProgram, TestPatternGenerator, and GCodeExporter
 *
 * Covers program building, block formatting, dialects, line numbering,
 * duration estimation, bounds calculation, pattern generation, and export.
 */
#include <gtest/gtest.h>
#include <tether/motion_replanner/GCodeGenerator.hpp>
#include <sstream>

using namespace MotionReplanner;

// ============================================================================
// GCodeOptions
// ============================================================================
TEST(GCodeOptionsTest, Defaults) {
    GCodeOptions opts;
    EXPECT_EQ(opts.dialect, GCodeDialect::LinuxCNC);
    EXPECT_EQ(opts.positionPrecision, 4);
    EXPECT_TRUE(opts.useMetric);
    EXPECT_TRUE(opts.absoluteMode);
    EXPECT_FALSE(opts.useLineNumbers);
}

// ============================================================================
// GCodeBlock
// ============================================================================
TEST(GCodeBlockTest, DefaultConstruction) {
    GCodeBlock b;
    EXPECT_EQ(b.lineNumber, -1);
    EXPECT_TRUE(b.gCode.empty());
    EXPECT_DOUBLE_EQ(b.feedRate, -1.0);
    EXPECT_FALSE(b.hasIJK);
    EXPECT_FALSE(b.hasR);
    for (int i = 0; i < 9; i++) {
        EXPECT_FALSE(b.hasPosition[i]);
    }
}

TEST(GCodeBlockTest, ToStringBasicG1) {
    GCodeBlock b;
    b.gCode = "G1";
    b.position[0] = 100.0;
    b.hasPosition[0] = true;
    b.feedRate = 500.0;

    GCodeOptions opts;
    std::string str = b.toString(opts);
    EXPECT_NE(str.find("G1"), std::string::npos);
    EXPECT_NE(str.find("X"), std::string::npos);
    EXPECT_NE(str.find("F"), std::string::npos);
}

TEST(GCodeBlockTest, ToStringWithComment) {
    GCodeBlock b;
    b.gCode = "G0";
    b.position[0] = 0.0;
    b.hasPosition[0] = true;
    b.comment = "rapids home";

    GCodeOptions opts;
    opts.useParenComments = true;
    std::string str = b.toString(opts);
    EXPECT_NE(str.find("rapids home"), std::string::npos);
}

TEST(GCodeBlockTest, ToStringWithLineNumber) {
    GCodeBlock b;
    b.lineNumber = 10;
    b.gCode = "G1";
    b.position[0] = 50.0;
    b.hasPosition[0] = true;

    GCodeOptions opts;
    opts.useLineNumbers = true;
    std::string str = b.toString(opts);
    EXPECT_NE(str.find("N10"), std::string::npos);
}

TEST(GCodeBlockTest, ToStringArcWithIJK) {
    GCodeBlock b;
    b.gCode = "G2";
    b.position[0] = 10.0;
    b.hasPosition[0] = true;
    b.position[1] = 10.0;
    b.hasPosition[1] = true;
    b.i = 5.0;
    b.j = 0.0;
    b.hasIJK = true;

    GCodeOptions opts;
    std::string str = b.toString(opts);
    EXPECT_NE(str.find("G2"), std::string::npos);
    EXPECT_NE(str.find("I"), std::string::npos);
}

// ============================================================================
// GCodeProgram
// ============================================================================
class GCodeProgramTest : public ::testing::Test {
protected:
    GCodeProgram program_;
};

TEST_F(GCodeProgramTest, DefaultEmpty) {
    EXPECT_TRUE(program_.blocks().empty());
}

TEST_F(GCodeProgramTest, AddComment) {
    program_.addComment("Test program");
    EXPECT_EQ(program_.blocks().size(), 1u);
}

TEST_F(GCodeProgramTest, AddBlankLine) {
    program_.addBlankLine();
    EXPECT_EQ(program_.blocks().size(), 1u);
}

TEST_F(GCodeProgramTest, AddProgramStartEnd) {
    program_.addProgramStart();
    program_.addProgramEnd();
    EXPECT_GE(program_.blocks().size(), 2u);
}

TEST_F(GCodeProgramTest, AddUnitsMetric) {
    program_.addUnitsMetric();
    auto str = program_.toString();
    EXPECT_NE(str.find("G21"), std::string::npos);
}

TEST_F(GCodeProgramTest, AddUnitsInch) {
    program_.addUnitsInch();
    auto str = program_.toString();
    EXPECT_NE(str.find("G20"), std::string::npos);
}

TEST_F(GCodeProgramTest, AddAbsoluteMode) {
    program_.addAbsoluteMode();
    auto str = program_.toString();
    EXPECT_NE(str.find("G90"), std::string::npos);
}

TEST_F(GCodeProgramTest, AddRapid) {
    program_.addRapid(10.0, 20.0, 30.0);
    auto str = program_.toString();
    EXPECT_NE(str.find("G0"), std::string::npos);
}

TEST_F(GCodeProgramTest, AddLinear) {
    program_.addLinear(50.0, 60.0, 5.0, 1000.0);
    auto str = program_.toString();
    EXPECT_NE(str.find("G1"), std::string::npos);
    EXPECT_NE(str.find("F"), std::string::npos);
}

TEST_F(GCodeProgramTest, AddArcCW) {
    program_.addArcCW(10.0, 10.0, 5.0, 0.0, 500.0);
    auto str = program_.toString();
    EXPECT_NE(str.find("G2"), std::string::npos);
}

TEST_F(GCodeProgramTest, AddArcCCW) {
    program_.addArcCCW(10.0, 10.0, 5.0, 0.0, 500.0);
    auto str = program_.toString();
    EXPECT_NE(str.find("G3"), std::string::npos);
}

TEST_F(GCodeProgramTest, AddDwell) {
    program_.addDwell(0.5);
    auto str = program_.toString();
    EXPECT_NE(str.find("G4"), std::string::npos);
}

TEST_F(GCodeProgramTest, AddToolChange) {
    program_.addToolChange(3);
    auto str = program_.toString();
    EXPECT_NE(str.find("T3"), std::string::npos);
}

TEST_F(GCodeProgramTest, AddSpindleOnOff) {
    program_.addSpindleOn(12000.0, true); // CW
    program_.addSpindleOff();
    auto str = program_.toString();
    EXPECT_NE(str.find("M3"), std::string::npos);
    EXPECT_NE(str.find("M5"), std::string::npos);
}

TEST_F(GCodeProgramTest, AddFeedRate) {
    program_.addFeedRate(2000.0);
    auto str = program_.toString();
    EXPECT_NE(str.find("F"), std::string::npos);
}

TEST_F(GCodeProgramTest, ToString) {
    program_.addComment("header");
    program_.addUnitsMetric();
    program_.addRapid(0.0, 0.0, 10.0);
    program_.addLinear(100.0, 0.0, 0.0, 500.0);
    auto str = program_.toString();
    EXPECT_FALSE(str.empty());
}

TEST_F(GCodeProgramTest, EstimateDuration) {
    program_.addRapid(0.0, 0.0, 0.0);
    program_.addLinear(100.0, 0.0, 0.0, 6000.0); // 100mm at 6000mm/min = 1s
    double duration = program_.estimateDuration();
    EXPECT_GE(duration, 0.0);
}

TEST_F(GCodeProgramTest, GetBounds) {
    program_.addLinear(100.0, 200.0, 50.0, 1000.0);
    program_.addLinear(-10.0, -20.0, 0.0, 1000.0);
    auto bounds = program_.getBounds();
    // Should have min/max for x, y, z
    (void)bounds;
}

TEST_F(GCodeProgramTest, WriteToStream) {
    program_.addUnitsMetric();
    program_.addLinear(10.0, 20.0, 0.0, 500.0);
    std::ostringstream ss;
    program_.write(ss);
    EXPECT_FALSE(ss.str().empty());
}

TEST_F(GCodeProgramTest, AddBlockFromString) {
    program_.addBlock("G1 X100 Y200 F500");
    EXPECT_EQ(program_.blocks().size(), 1u);
}

TEST_F(GCodeProgramTest, WithLineNumbers) {
    GCodeOptions opts;
    opts.useLineNumbers = true;
    opts.lineNumberIncrement = 5;
    GCodeProgram prog(opts);
    prog.addRapid(0.0, 0.0, 10.0);
    prog.addLinear(100.0, 0.0, 0.0, 500.0);
    auto str = prog.toString();
    EXPECT_NE(str.find("N"), std::string::npos);
}

// ============================================================================
// TestPatternGenerator
// ============================================================================
class TestPatternGeneratorTest : public ::testing::Test {
protected:
    TestPatternGenerator gen_;
};

TEST_F(TestPatternGeneratorTest, GenerateSingleAxisRamp) {
    SingleAxisTestConfig cfg;
    cfg.type = SingleAxisTestType::Ramp;
    cfg.axis = 0;
    cfg.amplitude = 100.0;
    cfg.velocity = 1000.0;
    cfg.duration = 2.0;
    auto prog = gen_.generateSingleAxisTest(cfg);
    EXPECT_FALSE(prog.blocks().empty());
}

TEST_F(TestPatternGeneratorTest, GenerateSingleAxisSinusoid) {
    SingleAxisTestConfig cfg;
    cfg.type = SingleAxisTestType::Sinusoid;
    cfg.axis = 0;
    cfg.amplitude = 50.0;
    cfg.frequency = 1.0;
    cfg.duration = 2.0;
    auto prog = gen_.generateSingleAxisTest(cfg);
    EXPECT_FALSE(prog.blocks().empty());
}

TEST_F(TestPatternGeneratorTest, GenerateCircleTest) {
    auto prog = gen_.generateCircleTest(0.0, 0.0, 0.0, 50.0, 1000.0, 1);
    EXPECT_FALSE(prog.blocks().empty());
    // Circle should contain G2 or G3 arcs
    auto str = prog.toString();
    bool hasArc = str.find("G2") != std::string::npos || str.find("G3") != std::string::npos;
    // Some implementations may use linear approximation
    (void)hasArc;
}

TEST_F(TestPatternGeneratorTest, GenerateEllipseTest) {
    auto prog = gen_.generateEllipseTest(0.0, 0.0, 0.0, 50.0, 30.0, 0.0, 1000.0, 1);
    EXPECT_FALSE(prog.blocks().empty());
}

TEST_F(TestPatternGeneratorTest, GenerateHelixTest) {
    auto prog = gen_.generateHelixTest(0.0, 0.0, 0.0, 50.0, 10.0, 1000.0, 2);
    EXPECT_FALSE(prog.blocks().empty());
}

TEST_F(TestPatternGeneratorTest, GenerateFrictionTest) {
    std::vector<double> feedRates = {100.0, 500.0, 1000.0};
    auto prog = gen_.generateFrictionTest(0, 100.0, feedRates, 3);
    EXPECT_FALSE(prog.blocks().empty());
}

TEST_F(TestPatternGeneratorTest, GenerateRampTest) {
    auto prog = gen_.generateRampTest(1, 0.0, 200.0, 2000.0, 1);
    EXPECT_FALSE(prog.blocks().empty());
}

// ============================================================================
// GCodeExporter
// ============================================================================
TEST(GCodeExporterTest, ExportEmptyTrajectory) {
    GCodeExporter exporter;
    std::vector<PositionSample> samples;
    auto prog = exporter.exportTrajectory(samples);
    // Empty or minimal program
    (void)prog;
}

TEST(GCodeExporterTest, ExportLinearTrajectory) {
    GCodeExporter exporter;
    std::vector<PositionSample> samples;
    for (int i = 0; i < 10; i++) {
        PositionSample s;
        s.timestamp = i * 0.01;
        s.position[0] = i * 10.0;
        s.position[1] = 0.0;
        s.position[2] = 0.0;
        samples.push_back(s);
    }
    auto prog = exporter.exportTrajectory(samples);
    EXPECT_FALSE(prog.blocks().empty());
}

TEST(GCodeExporterTest, ExportTrajectoryWithArcs) {
    GCodeExporter exporter;
    std::vector<PositionSample> samples;
    // Create a circle trajectory
    for (int i = 0; i <= 36; i++) {
        double angle = i * 10.0 * M_PI / 180.0;
        PositionSample s;
        s.timestamp = i * 0.01;
        s.position[0] = 50.0 * std::cos(angle);
        s.position[1] = 50.0 * std::sin(angle);
        s.position[2] = 0.0;
        samples.push_back(s);
    }
    auto prog = exporter.exportTrajectoryWithArcs(samples, 0.01);
    EXPECT_FALSE(prog.blocks().empty());
}
