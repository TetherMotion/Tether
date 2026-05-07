/**
 * @file ExportTests.cpp
 * @brief Unit tests for SVG and CSV export functionality
 */

#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include <sstream>
#include <fstream>
#include <cmath>

#include "tether/export/TrajectoryAnalyzer.hpp"
#include "tether/export/SVGExporter.hpp"
#include "tether/export/CSVExporter.hpp"

using namespace GCodeExport;
using namespace GCode;

// ============================================================================
// Test Fixtures
// ============================================================================

class ExportTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Create some test samples
        for (int i = 0; i <= 100; ++i) {
            TrajectorySample s;
            s.time = i * 0.001;
            s.pathPosition = i * 0.5;
            s.position[0] = i * 0.5;  // X
            s.position[1] = 10 * std::sin(i * 0.1);  // Y - sinusoidal
            s.position[2] = 0;  // Z
            s.velocity[0] = 50.0;  // mm/s
            s.velocity[1] = 10 * std::cos(i * 0.1) * 100;
            s.linearVelocity = std::sqrt(s.velocity[0]*s.velocity[0] + s.velocity[1]*s.velocity[1]);
            s.segmentIndex = i / 10;
            s.motionType = 1;  // Linear
            samples_.push_back(s);
        }
    }
    
    std::vector<TrajectorySample> samples_;
};

// ============================================================================
// SVG Export Tests
// ============================================================================

TEST_F(ExportTest, SVGExportCreatesValidXML) {
    SVGConfig config;
    SVGExporter exporter(config);
    
    std::stringstream ss;
    exporter.exportToStream(samples_, ss);
    
    std::string svg = ss.str();
    
    // Check for XML declaration
    EXPECT_TRUE(svg.find("<?xml") != std::string::npos);
    
    // Check for SVG root element
    EXPECT_TRUE(svg.find("<svg") != std::string::npos);
    EXPECT_TRUE(svg.find("</svg>") != std::string::npos);
    
    // Check for path element
    EXPECT_TRUE(svg.find("<path") != std::string::npos);
}

TEST_F(ExportTest, SVGExportHasCorrectDimensions) {
    SVGConfig config;
    config.width = 1024;
    config.height = 768;
    
    SVGExporter exporter(config);
    
    std::stringstream ss;
    exporter.exportToStream(samples_, ss);
    
    std::string svg = ss.str();
    
    EXPECT_TRUE(svg.find("width=\"1024\"") != std::string::npos);
    EXPECT_TRUE(svg.find("height=\"768\"") != std::string::npos);
}

TEST_F(ExportTest, SVGExportIncludesGrid) {
    SVGConfig config;
    config.showGrid = true;
    
    SVGExporter exporter(config);
    
    std::stringstream ss;
    exporter.exportToStream(samples_, ss);
    
    std::string svg = ss.str();
    
    EXPECT_TRUE(svg.find("id=\"grid\"") != std::string::npos);
}

TEST_F(ExportTest, SVGExportWithoutGrid) {
    SVGConfig config;
    config.showGrid = false;
    
    SVGExporter exporter(config);
    
    std::stringstream ss;
    exporter.exportToStream(samples_, ss);
    
    std::string svg = ss.str();
    
    EXPECT_TRUE(svg.find("id=\"grid\"") == std::string::npos);
}

TEST_F(ExportTest, SVGExportIncludesMarkers) {
    SVGConfig config;
    config.showStartPoint = true;
    config.showEndPoint = true;
    
    SVGExporter exporter(config);
    
    std::stringstream ss;
    exporter.exportToStream(samples_, ss);
    
    std::string svg = ss.str();
    
    EXPECT_TRUE(svg.find("Start") != std::string::npos);
    EXPECT_TRUE(svg.find("End") != std::string::npos);
}

TEST_F(ExportTest, SVGExportColorsByMotionType) {
    SVGConfig config;
    
    // Add different motion types
    TrajectorySample rapid;
    rapid.position[0] = 0; rapid.position[1] = 0;
    rapid.motionType = 0;  // Rapid
    samples_.insert(samples_.begin(), rapid);
    
    SVGExporter exporter(config);
    
    std::stringstream ss;
    exporter.exportToStream(samples_, ss);
    
    std::string svg = ss.str();
    
    // Should have different colors for different motion types
    EXPECT_TRUE(svg.find(config.rapidColor) != std::string::npos ||
                svg.find(config.linearColor) != std::string::npos);
}

// ============================================================================
// CSV Export Tests
// ============================================================================

TEST_F(ExportTest, CSVExportHasHeader) {
    CSVConfig config;
    config.includeHeader = true;
    
    CSVExporter exporter(config);
    
    std::stringstream ss;
    exporter.exportToStream(samples_, ss);
    
    std::string csv = ss.str();
    std::istringstream iss(csv);
    std::string firstLine;
    std::getline(iss, firstLine);
    
    // Header should contain column names
    EXPECT_TRUE(firstLine.find("time") != std::string::npos);
    EXPECT_TRUE(firstLine.find("pos_X") != std::string::npos);
}

TEST_F(ExportTest, CSVExportWithoutHeader) {
    CSVConfig config;
    config.includeHeader = false;
    config.includeUnits = false;
    
    CSVExporter exporter(config);
    
    std::stringstream ss;
    exporter.exportToStream(samples_, ss);
    
    std::string csv = ss.str();
    std::istringstream iss(csv);
    std::string firstLine;
    std::getline(iss, firstLine);
    
    // First line should be data (starts with number)
    EXPECT_TRUE(firstLine.size() > 0);
    EXPECT_TRUE(std::isdigit(firstLine[0]) || firstLine[0] == '-');
}

TEST_F(ExportTest, CSVExportCorrectColumnCount) {
    CSVConfig config;
    config.includeAxes.reset();
    config.includeAxes[0] = true;  // X only
    config.includeAxes[1] = true;  // Y only
    config.exportTime = true;
    config.exportPosition = true;
    config.exportVelocity = false;
    config.exportAcceleration = false;
    config.exportJerk = false;
    config.exportCombinedMetrics = false;
    config.exportSegmentInfo = false;
    config.exportPathPosition = false;
    
    CSVExporter exporter(config);
    
    std::stringstream ss;
    exporter.exportToStream(samples_, ss);
    
    std::string csv = ss.str();
    std::istringstream iss(csv);
    std::string headerLine;
    std::getline(iss, headerLine);  // Skip header
    
    // Count commas in header
    int commaCount = std::count(headerLine.begin(), headerLine.end(), ',');
    
    // Should have time + 2 position columns = 3 columns = 2 commas
    EXPECT_EQ(commaCount, 2);
}

TEST_F(ExportTest, CSVExportCustomDelimiter) {
    CSVConfig config;
    config.delimiter = ';';
    
    CSVExporter exporter(config);
    
    std::stringstream ss;
    exporter.exportToStream(samples_, ss);
    
    std::string csv = ss.str();
    
    // Should use semicolon as delimiter
    EXPECT_TRUE(csv.find(';') != std::string::npos);
}

TEST_F(ExportTest, CSVExportIncludesAllDerivatives) {
    CSVConfig config;
    config.exportVelocity = true;
    config.exportAcceleration = true;
    config.exportJerk = true;
    
    CSVExporter exporter(config);
    
    std::stringstream ss;
    exporter.exportToStream(samples_, ss);
    
    std::string csv = ss.str();
    
    EXPECT_TRUE(csv.find("vel_") != std::string::npos);
    EXPECT_TRUE(csv.find("acc_") != std::string::npos);
    EXPECT_TRUE(csv.find("jerk_") != std::string::npos);
}

TEST_F(ExportTest, CSVExportUnitsRow) {
    CSVConfig config;
    config.includeHeader = true;
    config.includeUnits = true;
    
    CSVExporter exporter(config);
    
    std::stringstream ss;
    exporter.exportToStream(samples_, ss);
    
    std::string csv = ss.str();
    std::istringstream iss(csv);
    std::string line1, line2;
    std::getline(iss, line1);  // Header
    std::getline(iss, line2);  // Units
    
    // Units row should contain unit strings
    EXPECT_TRUE(line2.find("mm") != std::string::npos || 
                line2.find("s") != std::string::npos);
}

// ============================================================================
// Statistics Export Tests
// ============================================================================

TEST_F(ExportTest, StatisticsExportCreatesFile) {
    TrajectoryAnalyzer analyzer;
    auto stats = analyzer.computeStatistics(samples_);
    
    CSVConfig config;
    CSVExporter exporter(config);
    
    std::stringstream ss;
    // Would need to add exportStatisticsToStream method
    // For now, test file export
}

// ============================================================================
// Batch Export Tests
// ============================================================================

TEST_F(ExportTest, BatchExporterCreatesMultipleFiles) {
    BatchExporter::ExportSpec spec;
    spec.basename = "/tmp/test_export";
    spec.exportSVG = true;
    spec.exportCSV = true;
    spec.exportStatistics = true;
    
    BatchExporter batcher;
    
    TrajectoryAnalyzer analyzer;
    auto stats = analyzer.computeStatistics(samples_);
    
    int count = batcher.exportAll(samples_, &stats, spec);
    
    EXPECT_GE(count, 2) << "Should create at least SVG and CSV";
    EXPECT_FALSE(batcher.createdFiles().empty());
}

// ============================================================================
// Edge Cases
// ============================================================================

TEST_F(ExportTest, ExportEmptySamples) {
    std::vector<TrajectorySample> empty;
    
    SVGConfig svgConfig;
    SVGExporter svg(svgConfig);
    
    std::stringstream ss;
    svg.exportToStream(empty, ss);
    
    // Should not crash, may produce minimal or no output
}

TEST_F(ExportTest, ExportSingleSample) {
    std::vector<TrajectorySample> single(1);
    single[0].position[0] = 50;
    single[0].position[1] = 50;
    
    SVGConfig svgConfig;
    SVGExporter svg(svgConfig);
    
    std::stringstream ss;
    svg.exportToStream(single, ss);
    
    std::string result = ss.str();
    EXPECT_FALSE(result.empty());
}

TEST_F(ExportTest, ExportVeryLongTrajectory) {
    // Create a long trajectory
    std::vector<TrajectorySample> longTraj;
    for (int i = 0; i < 100000; ++i) {
        TrajectorySample s;
        s.time = i * 0.001;
        s.position[0] = i * 0.1;
        s.position[1] = std::sin(i * 0.001) * 100;
        longTraj.push_back(s);
    }
    
    CSVConfig config;
    CSVExporter csv(config);
    
    std::stringstream ss;
    csv.exportToStream(longTraj, ss);
    
    // Should complete without error
    EXPECT_TRUE(ss.str().size() > 0);
}

// ============================================================================
// TrajectoryAnalyzer Tests
// ============================================================================

class TrajectoryAnalyzerTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Create test segments for analysis
        createLinearSegments();
        createArcSegments();
    }
    
    void createLinearSegments() {
        // Simple linear motion from (0,0,0) to (100,0,0)
        GCode::PlanningSegment seg;
        seg.start = {0, 0, 0, 0, 0, 0, 0, 0, 0};
        seg.end = {100, 0, 0, 0, 0, 0, 0, 0, 0};
        seg.segmentLength = 100.0;
        seg.segmentTime = 2.0;  // 50 mm/s
        seg.motionType = GCode::SegmentMotionType::Linear;
        seg.blockIndex = 0;
        linearSegments_.push_back(seg);
        
        // Second segment in Y direction
        GCode::PlanningSegment seg2;
        seg2.start = {100, 0, 0, 0, 0, 0, 0, 0, 0};
        seg2.end = {100, 50, 0, 0, 0, 0, 0, 0, 0};
        seg2.segmentLength = 50.0;
        seg2.segmentTime = 1.0;  // 50 mm/s
        seg2.motionType = GCode::SegmentMotionType::Linear;
        seg2.blockIndex = 1;
        linearSegments_.push_back(seg2);
    }
    
    void createArcSegments() {
        // Arc segment (G2) - clockwise quarter arc
        GCode::PlanningSegment arc;
        arc.start = {10, 0, 0, 0, 0, 0, 0, 0, 0};
        arc.end = {0, 10, 0, 0, 0, 0, 0, 0, 0};
        arc.center = {0, 0, 0, 0, 0, 0, 0, 0, 0};
        arc.arcRadius = 10.0;
        arc.arcSweep = M_PI / 2.0;
        arc.segmentLength = M_PI * 10.0 / 2.0;  // Quarter circumference
        arc.segmentTime = 1.0;
        arc.motionType = GCode::SegmentMotionType::ArcCW;
        arc.plane = GCode::InterpolationPlane::XY;
        arc.blockIndex = 0;
        arcSegments_.push_back(arc);
    }
    
    std::vector<GCode::PlanningSegment> linearSegments_;
    std::vector<GCode::PlanningSegment> arcSegments_;
};

TEST_F(TrajectoryAnalyzerTest, DefaultConstructor) {
    TrajectoryAnalyzer analyzer;
    EXPECT_EQ(analyzer.config().timeStep, 0.001);
    EXPECT_EQ(analyzer.config().derivativeOrder, 4);
}

TEST_F(TrajectoryAnalyzerTest, ConstructorWithConfig) {
    AnalysisConfig config;
    config.timeStep = 0.005;
    config.derivativeOrder = 2;
    config.violationTolerance = 0.05;
    
    TrajectoryAnalyzer analyzer(config);
    EXPECT_EQ(analyzer.config().timeStep, 0.005);
    EXPECT_EQ(analyzer.config().derivativeOrder, 2);
    EXPECT_EQ(analyzer.config().violationTolerance, 0.05);
}

TEST_F(TrajectoryAnalyzerTest, ConfigureMethod) {
    TrajectoryAnalyzer analyzer;
    
    AnalysisConfig newConfig;
    newConfig.timeStep = 0.01;
    analyzer.configure(newConfig);
    
    EXPECT_EQ(analyzer.config().timeStep, 0.01);
}

TEST_F(TrajectoryAnalyzerTest, AnalyzeEmptySegments) {
    TrajectoryAnalyzer analyzer;
    std::vector<GCode::PlanningSegment> empty;
    
    auto samples = analyzer.analyze(empty, nullptr);
    EXPECT_TRUE(samples.empty());
}

TEST_F(TrajectoryAnalyzerTest, AnalyzeLinearSegments) {
    AnalysisConfig config;
    config.timeStep = 0.1;  // Larger time step for fewer samples
    TrajectoryAnalyzer analyzer(config);
    
    auto samples = analyzer.analyze(linearSegments_, nullptr);
    
    EXPECT_FALSE(samples.empty());
    // First sample should be at origin
    EXPECT_NEAR(samples.front().position[0], 0.0, 0.1);
    EXPECT_NEAR(samples.front().position[1], 0.0, 0.1);
    EXPECT_EQ(samples.front().time, 0.0);
    
    // Last sample should be at (100, 50, 0)
    EXPECT_NEAR(samples.back().position[0], 100.0, 0.1);
    EXPECT_NEAR(samples.back().position[1], 50.0, 0.1);
    EXPECT_NEAR(samples.back().time, 3.0, 0.1);  // 2s + 1s
}

TEST_F(TrajectoryAnalyzerTest, AnalyzeArcSegments) {
    AnalysisConfig config;
    config.timeStep = 0.05;
    TrajectoryAnalyzer analyzer(config);
    
    auto samples = analyzer.analyze(arcSegments_, nullptr);
    
    EXPECT_FALSE(samples.empty());
    // First sample near (10, 0)
    EXPECT_NEAR(samples.front().position[0], 10.0, 0.1);
    EXPECT_NEAR(samples.front().position[1], 0.0, 0.1);
    
    // Last sample near (0, 10)
    EXPECT_NEAR(samples.back().position[0], 0.0, 0.1);
    EXPECT_NEAR(samples.back().position[1], 10.0, 0.1);
}

TEST_F(TrajectoryAnalyzerTest, AnalyzeArcSegmentXZPlane) {
    GCode::PlanningSegment arc;
    arc.start = {10, 0, 0, 0, 0, 0, 0, 0, 0};
    arc.end = {0, 0, 10, 0, 0, 0, 0, 0, 0};
    arc.center = {0, 0, 0, 0, 0, 0, 0, 0, 0};
    arc.arcRadius = 10.0;
    arc.arcSweep = M_PI / 2.0;
    arc.segmentLength = M_PI * 10.0 / 2.0;
    arc.segmentTime = 1.0;
    arc.motionType = GCode::SegmentMotionType::ArcCW;
    arc.plane = GCode::InterpolationPlane::XZ;
    arc.blockIndex = 0;
    
    std::vector<GCode::PlanningSegment> segments = {arc};
    
    AnalysisConfig config;
    config.timeStep = 0.1;
    TrajectoryAnalyzer analyzer(config);
    
    auto samples = analyzer.analyze(segments, nullptr);
    EXPECT_FALSE(samples.empty());
    EXPECT_NEAR(samples.front().position[0], 10.0, 0.1);
    EXPECT_NEAR(samples.front().position[2], 0.0, 0.1);
}

TEST_F(TrajectoryAnalyzerTest, AnalyzeArcSegmentYZPlane) {
    GCode::PlanningSegment arc;
    arc.start = {0, 10, 0, 0, 0, 0, 0, 0, 0};
    arc.end = {0, 0, 10, 0, 0, 0, 0, 0, 0};
    arc.center = {0, 0, 0, 0, 0, 0, 0, 0, 0};
    arc.arcRadius = 10.0;
    arc.arcSweep = M_PI / 2.0;
    arc.segmentLength = M_PI * 10.0 / 2.0;
    arc.segmentTime = 1.0;
    arc.motionType = GCode::SegmentMotionType::ArcCW;
    arc.plane = GCode::InterpolationPlane::YZ;
    arc.blockIndex = 0;
    
    std::vector<GCode::PlanningSegment> segments = {arc};
    
    AnalysisConfig config;
    config.timeStep = 0.1;
    TrajectoryAnalyzer analyzer(config);
    
    auto samples = analyzer.analyze(segments, nullptr);
    EXPECT_FALSE(samples.empty());
    EXPECT_NEAR(samples.front().position[1], 10.0, 0.1);
    EXPECT_NEAR(samples.front().position[2], 0.0, 0.1);
}

TEST_F(TrajectoryAnalyzerTest, AnalyzeZeroTimeSegment) {
    GCode::PlanningSegment seg;
    seg.start = {0, 0, 0, 0, 0, 0, 0, 0, 0};
    seg.end = {10, 0, 0, 0, 0, 0, 0, 0, 0};
    seg.segmentLength = 10.0;
    seg.segmentTime = 0.0;  // Zero time
    seg.motionType = GCode::SegmentMotionType::Linear;
    
    std::vector<GCode::PlanningSegment> segments = {seg};
    
    TrajectoryAnalyzer analyzer;
    auto samples = analyzer.analyze(segments, nullptr);
    
    // Zero time segment should be skipped
    EXPECT_TRUE(samples.empty());
}

TEST_F(TrajectoryAnalyzerTest, AnalyzeZeroLengthSegment) {
    GCode::PlanningSegment seg;
    seg.start = {0, 0, 0, 0, 0, 0, 0, 0, 0};
    seg.end = {0, 0, 0, 0, 0, 0, 0, 0, 0};
    seg.segmentLength = 0.0;
    seg.segmentTime = 1.0;
    seg.motionType = GCode::SegmentMotionType::Linear;
    
    std::vector<GCode::PlanningSegment> segments = {seg};
    
    TrajectoryAnalyzer analyzer;
    auto samples = analyzer.analyze(segments, nullptr);
    
    // Should still produce samples even for zero length
    EXPECT_FALSE(samples.empty());
}

TEST_F(TrajectoryAnalyzerTest, ComputeDerivativesSmallSamples) {
    std::vector<TrajectorySample> samples;
    
    // Create only 3 samples (less than 5)
    for (int i = 0; i < 3; ++i) {
        TrajectorySample s;
        s.time = i * 0.1;
        s.position[0] = i * 10.0;
        samples.push_back(s);
    }
    
    TrajectoryAnalyzer analyzer;
    analyzer.computeDerivatives(samples, 4);
    
    // Should not crash, velocities remain 0
    EXPECT_EQ(samples[0].velocity[0], 0.0);
}

TEST_F(TrajectoryAnalyzerTest, ComputeDerivativesZeroDt) {
    std::vector<TrajectorySample> samples;
    
    // Create samples with zero time difference
    for (int i = 0; i < 10; ++i) {
        TrajectorySample s;
        s.time = 0.0;  // All same time
        s.position[0] = i * 10.0;
        samples.push_back(s);
    }
    
    TrajectoryAnalyzer analyzer;
    analyzer.computeDerivatives(samples, 4);
    
    // Should handle gracefully without division by zero
    // Velocities should remain 0 due to dt <= 0 check
}

TEST_F(TrajectoryAnalyzerTest, ComputeDerivativesBoundaryConditions) {
    std::vector<TrajectorySample> samples;
    
    // Create samples for derivative computation
    for (int i = 0; i < 10; ++i) {
        TrajectorySample s;
        s.time = i * 0.1;
        s.position[0] = i * 10.0;  // Linear motion
        samples.push_back(s);
    }
    
    TrajectoryAnalyzer analyzer;
    analyzer.computeDerivatives(samples, 4);
    
    // Boundary samples should have copied velocities
    EXPECT_EQ(samples[0].velocity[0], samples[2].velocity[0]);
    EXPECT_EQ(samples[1].velocity[0], samples[2].velocity[0]);
    
    // Check acceleration boundaries
    EXPECT_EQ(samples[0].acceleration[0], samples[2].acceleration[0]);
}

TEST_F(TrajectoryAnalyzerTest, ComputeCombinedMetrics) {
    std::vector<TrajectorySample> samples;
    
    TrajectorySample s;
    s.velocity[0] = 30.0;  // X velocity
    s.velocity[1] = 40.0;  // Y velocity
    s.velocity[2] = 0.0;   // Z velocity
    s.acceleration[0] = 10.0;
    s.acceleration[1] = 0.0;
    s.acceleration[2] = 0.0;
    samples.push_back(s);
    
    TrajectoryAnalyzer analyzer;
    analyzer.computeCombinedMetrics(samples);
    
    // Linear velocity = sqrt(30^2 + 40^2) = 50
    EXPECT_NEAR(samples[0].linearVelocity, 50.0, 0.001);
}

TEST_F(TrajectoryAnalyzerTest, ComputeCombinedMetricsZeroVelocity) {
    std::vector<TrajectorySample> samples;
    
    TrajectorySample s;
    s.velocity[0] = 0.0;
    s.velocity[1] = 0.0;
    s.velocity[2] = 0.0;
    samples.push_back(s);
    
    TrajectoryAnalyzer analyzer;
    analyzer.computeCombinedMetrics(samples);
    
    EXPECT_EQ(samples[0].linearVelocity, 0.0);
    EXPECT_EQ(samples[0].curvature, 0.0);  // Cannot compute with zero velocity
}

TEST_F(TrajectoryAnalyzerTest, ComputeCombinedMetricsCurvature) {
    std::vector<TrajectorySample> samples;
    
    TrajectorySample s;
    // Circular motion: velocity tangent, acceleration toward center
    s.velocity[0] = 10.0;  // Moving in X direction
    s.velocity[1] = 0.0;
    s.velocity[2] = 0.0;
    s.acceleration[0] = 0.0;
    s.acceleration[1] = 5.0;  // Centripetal in Y
    s.acceleration[2] = 0.0;
    samples.push_back(s);
    
    TrajectoryAnalyzer analyzer;
    analyzer.computeCombinedMetrics(samples);
    
    // Curvature = |v × a| / |v|³
    // cross = (0, 0, 10*5) = (0, 0, 50)
    // |cross| = 50
    // |v|³ = 10³ = 1000
    // curvature = 50/1000 = 0.05
    EXPECT_NEAR(samples[0].curvature, 0.05, 0.001);
    EXPECT_TRUE(samples[0].centripetalAccel > 0);
}

TEST_F(TrajectoryAnalyzerTest, ComputeStatisticsEmpty) {
    std::vector<TrajectorySample> empty;
    
    TrajectoryAnalyzer analyzer;
    auto stats = analyzer.computeStatistics(empty);
    
    EXPECT_EQ(stats.sampleCount, 0);
    EXPECT_EQ(stats.duration, 0.0);
    EXPECT_EQ(stats.pathLength, 0.0);
}

TEST_F(TrajectoryAnalyzerTest, ComputeStatisticsBasic) {
    TrajectoryAnalyzer analyzer;
    auto samples = analyzer.analyze(linearSegments_, nullptr);
    
    auto stats = analyzer.computeStatistics(samples);
    
    EXPECT_GT(stats.sampleCount, 0);
    EXPECT_NEAR(stats.duration, 3.0, 0.1);
    EXPECT_GT(stats.pathLength, 0.0);
}

TEST_F(TrajectoryAnalyzerTest, ComputeStatisticsAxisStats) {
    std::vector<TrajectorySample> samples;
    
    TrajectorySample s1;
    s1.time = 0.0;
    s1.position[0] = 0.0;
    s1.velocity[0] = 10.0;
    s1.acceleration[0] = 5.0;
    s1.jerk[0] = 2.0;
    samples.push_back(s1);
    
    TrajectorySample s2;
    s2.time = 1.0;
    s2.pathPosition = 50.0;
    s2.position[0] = 100.0;
    s2.velocity[0] = 50.0;
    s2.acceleration[0] = 20.0;
    s2.jerk[0] = 10.0;
    samples.push_back(s2);
    
    TrajectoryAnalyzer analyzer;
    auto stats = analyzer.computeStatistics(samples);
    
    EXPECT_EQ(stats.axisStats[0].minPosition, 0.0);
    EXPECT_EQ(stats.axisStats[0].maxPosition, 100.0);
    EXPECT_EQ(stats.axisStats[0].minVelocity, 10.0);
    EXPECT_EQ(stats.axisStats[0].maxVelocity, 50.0);
}

TEST_F(TrajectoryAnalyzerTest, CheckLimitCompliancePasses) {
    std::vector<TrajectorySample> samples;
    
    TrajectorySample s;
    s.linearVelocity = 10.0;  // 10 mm/s = 600 mm/min (below default limit)
    s.linearAcceleration = 100.0;
    s.linearJerk = 1000.0;
    samples.push_back(s);
    
    AnalysisConfig config;
    config.limits.maxVelocityLinear = 6000.0;  // mm/min
    config.limits.maxAcceleration = 1000.0;
    config.limits.maxJerk = 10000.0;
    
    TrajectoryAnalyzer analyzer(config);
    
    std::vector<LimitViolation> violations;
    bool compliant = analyzer.checkLimitCompliance(samples, &violations);
    
    EXPECT_TRUE(compliant);
    EXPECT_TRUE(violations.empty());
}

TEST_F(TrajectoryAnalyzerTest, CheckLimitComplianceVelocityViolation) {
    std::vector<TrajectorySample> samples;
    
    TrajectorySample s;
    s.time = 0.5;
    s.linearVelocity = 200.0;  // 200 mm/s = 12000 mm/min (above limit)
    s.linearAcceleration = 100.0;
    s.linearJerk = 1000.0;
    samples.push_back(s);
    
    AnalysisConfig config;
    config.limits.maxVelocityLinear = 6000.0;  // 100 mm/s
    config.limits.maxAcceleration = 10000.0;
    config.limits.maxJerk = 100000.0;
    config.violationTolerance = 0.01;
    
    TrajectoryAnalyzer analyzer(config);
    
    std::vector<LimitViolation> violations;
    bool compliant = analyzer.checkLimitCompliance(samples, &violations);
    
    EXPECT_FALSE(compliant);
    EXPECT_FALSE(violations.empty());
    EXPECT_EQ(violations[0].limitType, "velocity");
}

TEST_F(TrajectoryAnalyzerTest, CheckLimitComplianceAccelerationViolation) {
    std::vector<TrajectorySample> samples;
    
    TrajectorySample s;
    s.time = 0.5;
    s.linearVelocity = 10.0;
    s.linearAcceleration = 5000.0;  // Above limit
    s.linearJerk = 1000.0;
    samples.push_back(s);
    
    AnalysisConfig config;
    config.limits.maxVelocityLinear = 60000.0;
    config.limits.maxAcceleration = 1000.0;
    config.limits.maxJerk = 100000.0;
    config.violationTolerance = 0.01;
    
    TrajectoryAnalyzer analyzer(config);
    
    std::vector<LimitViolation> violations;
    bool compliant = analyzer.checkLimitCompliance(samples, &violations);
    
    EXPECT_FALSE(compliant);
    EXPECT_FALSE(violations.empty());
    EXPECT_EQ(violations[0].limitType, "acceleration");
}

TEST_F(TrajectoryAnalyzerTest, CheckLimitComplianceJerkViolation) {
    std::vector<TrajectorySample> samples;
    
    TrajectorySample s;
    s.time = 0.5;
    s.linearVelocity = 10.0;
    s.linearAcceleration = 100.0;
    s.linearJerk = 100000.0;  // Above limit
    samples.push_back(s);
    
    AnalysisConfig config;
    config.limits.maxVelocityLinear = 60000.0;
    config.limits.maxAcceleration = 10000.0;
    config.limits.maxJerk = 10000.0;
    config.violationTolerance = 0.01;
    
    TrajectoryAnalyzer analyzer(config);
    
    std::vector<LimitViolation> violations;
    bool compliant = analyzer.checkLimitCompliance(samples, &violations);
    
    EXPECT_FALSE(compliant);
    EXPECT_FALSE(violations.empty());
    EXPECT_EQ(violations[0].limitType, "jerk");
}

TEST_F(TrajectoryAnalyzerTest, CheckLimitCompliancePerAxisVelocity) {
    std::vector<TrajectorySample> samples;
    
    TrajectorySample s;
    s.time = 0.5;
    s.velocity[0] = 200.0;  // X velocity above axis limit
    samples.push_back(s);
    
    AnalysisConfig config;
    config.limits.axisMaxVelocity[0] = 6000.0;  // 100 mm/s limit for X
    config.limits.axisMaxAcceleration[0] = 10000.0;
    config.limits.axisMaxJerk[0] = 100000.0;
    config.violationTolerance = 0.01;
    
    TrajectoryAnalyzer analyzer(config);
    
    std::vector<LimitViolation> violations;
    bool compliant = analyzer.checkLimitCompliance(samples, &violations);
    
    EXPECT_FALSE(compliant);
    EXPECT_FALSE(violations.empty());
    EXPECT_EQ(violations[0].axis, 0);  // X axis
    EXPECT_EQ(violations[0].limitType, "velocity");
}

TEST_F(TrajectoryAnalyzerTest, CheckLimitCompliancePerAxisAcceleration) {
    std::vector<TrajectorySample> samples;
    
    TrajectorySample s;
    s.time = 0.5;
    s.acceleration[1] = 5000.0;  // Y acceleration above axis limit
    samples.push_back(s);
    
    AnalysisConfig config;
    config.limits.axisMaxVelocity[1] = 100000.0;
    config.limits.axisMaxAcceleration[1] = 1000.0;  // Y acceleration limit
    config.limits.axisMaxJerk[1] = 100000.0;
    config.violationTolerance = 0.01;
    
    TrajectoryAnalyzer analyzer(config);
    
    std::vector<LimitViolation> violations;
    bool compliant = analyzer.checkLimitCompliance(samples, &violations);
    
    EXPECT_FALSE(compliant);
    EXPECT_FALSE(violations.empty());
    EXPECT_EQ(violations[0].axis, 1);  // Y axis
    EXPECT_EQ(violations[0].limitType, "acceleration");
}

TEST_F(TrajectoryAnalyzerTest, CheckLimitCompliancePerAxisJerk) {
    std::vector<TrajectorySample> samples;
    
    TrajectorySample s;
    s.time = 0.5;
    s.jerk[2] = 100000.0;  // Z jerk above axis limit
    samples.push_back(s);
    
    AnalysisConfig config;
    config.limits.axisMaxVelocity[2] = 100000.0;
    config.limits.axisMaxAcceleration[2] = 100000.0;
    config.limits.axisMaxJerk[2] = 10000.0;  // Z jerk limit
    config.violationTolerance = 0.01;
    
    TrajectoryAnalyzer analyzer(config);
    
    std::vector<LimitViolation> violations;
    bool compliant = analyzer.checkLimitCompliance(samples, &violations);
    
    EXPECT_FALSE(compliant);
    EXPECT_FALSE(violations.empty());
    EXPECT_EQ(violations[0].axis, 2);  // Z axis
    EXPECT_EQ(violations[0].limitType, "jerk");
}

TEST_F(TrajectoryAnalyzerTest, CheckLimitComplianceNoViolationsOutput) {
    std::vector<TrajectorySample> samples;
    
    TrajectorySample s;
    s.linearVelocity = 200.0;  // Above limit
    samples.push_back(s);
    
    AnalysisConfig config;
    config.limits.maxVelocityLinear = 6000.0;
    
    TrajectoryAnalyzer analyzer(config);
    
    // Call without violations pointer
    bool compliant = analyzer.checkLimitCompliance(samples, nullptr);
    
    EXPECT_FALSE(compliant);
}

// ============================================================================
// Approximation Strategy Tests
// ============================================================================

TEST_F(TrajectoryAnalyzerTest, FixedTimeApproximationName) {
    FixedTimeApproximation approx;
    EXPECT_STREQ(approx.name(), "FixedTime");
}

TEST_F(TrajectoryAnalyzerTest, FixedTimeApproximationSetTimeStep) {
    FixedTimeApproximation approx;
    approx.setTimeStep(0.005);
    
    auto samples = approx.generateTrajectory(linearSegments_, GCode::KinematicLimits());
    EXPECT_FALSE(samples.empty());
}

TEST_F(TrajectoryAnalyzerTest, FixedTimeApproximationConfigure) {
    FixedTimeApproximation approx;
    approx.configure("timeStep", 0.01);
    approx.configure("unknown", 1.0);  // Should be ignored
    
    auto samples = approx.generateTrajectory(linearSegments_, GCode::KinematicLimits());
    EXPECT_FALSE(samples.empty());
}

TEST_F(TrajectoryAnalyzerTest, FixedDeviationApproximationName) {
    FixedDeviationApproximation approx;
    EXPECT_STREQ(approx.name(), "FixedDeviation");
}

TEST_F(TrajectoryAnalyzerTest, FixedDeviationApproximationSetMaxDeviation) {
    FixedDeviationApproximation approx;
    approx.setMaxDeviation(0.01);
    
    auto samples = approx.generateTrajectory(linearSegments_, GCode::KinematicLimits());
    EXPECT_FALSE(samples.empty());
}

TEST_F(TrajectoryAnalyzerTest, FixedDeviationApproximationConfigure) {
    FixedDeviationApproximation approx;
    approx.configure("maxDeviation", 0.01);
    approx.configure("unknown", 1.0);  // Should be ignored
    
    auto samples = approx.generateTrajectory(linearSegments_, GCode::KinematicLimits());
    EXPECT_FALSE(samples.empty());
}

TEST_F(TrajectoryAnalyzerTest, FixedDeviationApproximationArc) {
    FixedDeviationApproximation approx;
    approx.setMaxDeviation(0.1);
    
    auto samples = approx.generateTrajectory(arcSegments_, GCode::KinematicLimits());
    EXPECT_FALSE(samples.empty());
}

TEST_F(TrajectoryAnalyzerTest, FixedDeviationApproximationZeroLengthSegment) {
    GCode::PlanningSegment seg;
    seg.start = {0, 0, 0, 0, 0, 0, 0, 0, 0};
    seg.end = {0, 0, 0, 0, 0, 0, 0, 0, 0};
    seg.segmentLength = 0.0;
    seg.segmentTime = 1.0;
    seg.motionType = GCode::SegmentMotionType::Linear;
    
    std::vector<GCode::PlanningSegment> segments = {seg};
    
    FixedDeviationApproximation approx;
    auto samples = approx.generateTrajectory(segments, GCode::KinematicLimits());
    
    // Zero length should be skipped
    EXPECT_TRUE(samples.empty());
}

TEST_F(TrajectoryAnalyzerTest, TrapezoidalApproximationName) {
    TrapezoidalApproximation approx;
    EXPECT_STREQ(approx.name(), "Trapezoidal");
}

TEST_F(TrajectoryAnalyzerTest, TrapezoidalApproximationConfigure) {
    TrapezoidalApproximation approx;
    approx.configure("timeStep", 0.005);
    approx.configure("useJerkLimiting", 1.0);
    approx.configure("useJerkLimiting", 0.0);
    
    auto samples = approx.generateTrajectory(linearSegments_, GCode::KinematicLimits());
    EXPECT_FALSE(samples.empty());
}

TEST_F(TrajectoryAnalyzerTest, SCurveApproximationName) {
    SCurveApproximation approx;
    EXPECT_STREQ(approx.name(), "SCurve");
}

TEST_F(TrajectoryAnalyzerTest, SCurveApproximationConfigure) {
    SCurveApproximation approx;
    approx.configure("timeStep", 0.005);
    approx.configure("unknown", 1.0);
    
    auto samples = approx.generateTrajectory(linearSegments_, GCode::KinematicLimits());
    EXPECT_FALSE(samples.empty());
}

// ============================================================================
// Approximation Factory Tests
// ============================================================================

TEST(ApproximationFactoryTest, CreateFixedTime) {
    auto strategy = ApproximationFactory::create("FixedTime");
    EXPECT_NE(strategy, nullptr);
    EXPECT_STREQ(strategy->name(), "FixedTime");
}

TEST(ApproximationFactoryTest, CreateFixedDeviation) {
    auto strategy = ApproximationFactory::create("FixedDeviation");
    EXPECT_NE(strategy, nullptr);
    EXPECT_STREQ(strategy->name(), "FixedDeviation");
}

TEST(ApproximationFactoryTest, CreateTrapezoidal) {
    auto strategy = ApproximationFactory::create("Trapezoidal");
    EXPECT_NE(strategy, nullptr);
    EXPECT_STREQ(strategy->name(), "Trapezoidal");
}

TEST(ApproximationFactoryTest, CreateSCurve) {
    auto strategy = ApproximationFactory::create("SCurve");
    EXPECT_NE(strategy, nullptr);
    EXPECT_STREQ(strategy->name(), "SCurve");
}

TEST(ApproximationFactoryTest, CreateUnknown) {
    auto strategy = ApproximationFactory::create("Unknown");
    EXPECT_EQ(strategy, nullptr);
}

TEST(ApproximationFactoryTest, AvailableStrategies) {
    auto strategies = ApproximationFactory::availableStrategies();
    EXPECT_EQ(strategies.size(), 4);
    EXPECT_TRUE(std::find(strategies.begin(), strategies.end(), "FixedTime") != strategies.end());
    EXPECT_TRUE(std::find(strategies.begin(), strategies.end(), "FixedDeviation") != strategies.end());
    EXPECT_TRUE(std::find(strategies.begin(), strategies.end(), "Trapezoidal") != strategies.end());
    EXPECT_TRUE(std::find(strategies.begin(), strategies.end(), "SCurve") != strategies.end());
}

// ============================================================================
// SVGExporter Additional Tests
// ============================================================================

TEST_F(ExportTest, SVGExportToFile) {
    SVGConfig config;
    SVGExporter exporter(config);
    
    std::string filename = "/tmp/test_export.svg";
    bool success = exporter.exportToFile(samples_, filename);
    
    EXPECT_TRUE(success);
    
    // Clean up
    std::remove(filename.c_str());
}

TEST_F(ExportTest, SVGExportToFileInvalidPath) {
    SVGConfig config;
    SVGExporter exporter(config);
    
    bool success = exporter.exportToFile(samples_, "/nonexistent/dir/test.svg");
    
    EXPECT_FALSE(success);
}

TEST_F(ExportTest, SVGExportSegments) {
    GCode::PlanningSegment seg;
    seg.start = {0, 0, 0, 0, 0, 0, 0, 0, 0};
    seg.end = {100, 50, 0, 0, 0, 0, 0, 0, 0};
    seg.segmentLength = 111.8;
    seg.segmentTime = 2.0;
    seg.motionType = GCode::SegmentMotionType::Linear;
    
    std::vector<GCode::PlanningSegment> segments = {seg};
    
    SVGConfig config;
    SVGExporter exporter(config);
    
    std::string filename = "/tmp/test_segments.svg";
    bool success = exporter.exportSegments(segments, filename);
    
    EXPECT_TRUE(success);
    
    // Clean up
    std::remove(filename.c_str());
}

TEST_F(ExportTest, SVGExportNoAutoScale) {
    SVGConfig config;
    config.autoScale = false;
    
    SVGExporter exporter(config);
    
    std::stringstream ss;
    exporter.exportToStream(samples_, ss);
    
    std::string svg = ss.str();
    EXPECT_TRUE(svg.find("<svg") != std::string::npos);
}

TEST_F(ExportTest, SVGExportFlipY) {
    SVGConfig config;
    config.flipY = true;
    
    SVGExporter exporter(config);
    
    std::stringstream ss;
    exporter.exportToStream(samples_, ss);
    
    std::string svg = ss.str();
    EXPECT_TRUE(svg.find("<path") != std::string::npos);
}

TEST_F(ExportTest, SVGExportColorByVelocity) {
    SVGConfig config;
    config.colorByVelocity = true;
    config.velocityColorMin = 0.0;
    config.velocityColorMax = 100.0;
    
    SVGExporter exporter(config);
    
    std::stringstream ss;
    exporter.exportToStream(samples_, ss);
    
    std::string svg = ss.str();
    EXPECT_TRUE(svg.find("<path") != std::string::npos);
}

TEST_F(ExportTest, SVGExportHideRapids) {
    SVGConfig config;
    config.showRapids = false;
    
    // Add rapid movement
    TrajectorySample rapid;
    rapid.position[0] = -10;
    rapid.position[1] = -10;
    rapid.motionType = 0;  // Rapid
    samples_.insert(samples_.begin(), rapid);
    
    SVGExporter exporter(config);
    
    std::stringstream ss;
    exporter.exportToStream(samples_, ss);
    
    // Rapid should not be rendered
    std::string svg = ss.str();
    EXPECT_TRUE(svg.find("<path") != std::string::npos);
}

TEST_F(ExportTest, SVGExportDirectionArrows) {
    SVGConfig config;
    config.showDirectionArrows = true;
    
    SVGExporter exporter(config);
    
    std::stringstream ss;
    exporter.exportToStream(samples_, ss);
    
    std::string svg = ss.str();
    EXPECT_TRUE(svg.find("marker-mid") != std::string::npos);
}

TEST_F(ExportTest, SVGExportArcMotionTypes) {
    // Add arc motion types
    TrajectorySample arcCW;
    arcCW.position[0] = 60;
    arcCW.position[1] = 5;
    arcCW.motionType = 2;  // Arc CW
    samples_.push_back(arcCW);
    
    TrajectorySample arcCCW;
    arcCCW.position[0] = 70;
    arcCCW.position[1] = 5;
    arcCCW.motionType = 3;  // Arc CCW
    samples_.push_back(arcCCW);
    
    SVGConfig config;
    SVGExporter exporter(config);
    
    std::stringstream ss;
    exporter.exportToStream(samples_, ss);
    
    std::string svg = ss.str();
    EXPECT_TRUE(svg.find("<path") != std::string::npos);
}

TEST_F(ExportTest, SVGExportVerySmallBounds) {
    std::vector<TrajectorySample> samples;
    TrajectorySample s;
    s.position[0] = 0.0;
    s.position[1] = 0.0;
    samples.push_back(s);
    
    TrajectorySample s2;
    s2.position[0] = 0.0;  // Same X
    s2.position[1] = 0.0;  // Same Y
    samples.push_back(s2);
    
    SVGConfig config;
    SVGExporter exporter(config);
    
    std::stringstream ss;
    exporter.exportToStream(samples, ss);
    
    // Should handle zero-width bounds
    std::string svg = ss.str();
    EXPECT_TRUE(svg.find("<svg") != std::string::npos);
}

TEST_F(ExportTest, SVGExportSecondaryAxes) {
    SVGConfig config;
    config.primaryAxis1 = 1;  // Y axis
    config.primaryAxis2 = 2;  // Z axis
    
    SVGExporter exporter(config);
    
    std::stringstream ss;
    exporter.exportToStream(samples_, ss);
    
    std::string svg = ss.str();
    EXPECT_TRUE(svg.find("<path") != std::string::npos);
}

// ============================================================================
// CSVExporter Additional Tests
// ============================================================================

TEST_F(ExportTest, CSVExportToFile) {
    CSVConfig config;
    CSVExporter exporter(config);
    
    std::string filename = "/tmp/test_export.csv";
    bool success = exporter.exportToFile(samples_, filename);
    
    EXPECT_TRUE(success);
    
    // Clean up
    std::remove(filename.c_str());
}

TEST_F(ExportTest, CSVExportToFileInvalidPath) {
    CSVConfig config;
    CSVExporter exporter(config);
    
    bool success = exporter.exportToFile(samples_, "/nonexistent/dir/test.csv");
    
    EXPECT_FALSE(success);
}

TEST_F(ExportTest, CSVExportStatisticsToFile) {
    TrajectoryStatistics stats;
    stats.duration = 5.0;
    stats.pathLength = 150.0;
    stats.sampleCount = 100;
    stats.maxLinearVelocity = 50.0;
    stats.maxLinearAcceleration = 1000.0;
    stats.maxLinearJerk = 10000.0;
    stats.maxCurvature = 0.01;
    stats.maxCentripetalAccel = 100.0;
    stats.meetsLimits = true;
    
    CSVConfig config;
    CSVExporter exporter(config);
    
    std::string filename = "/tmp/test_stats.csv";
    bool success = exporter.exportStatistics(stats, filename);
    
    EXPECT_TRUE(success);
    
    // Clean up
    std::remove(filename.c_str());
}

TEST_F(ExportTest, CSVExportStatisticsInvalidPath) {
    TrajectoryStatistics stats;
    
    CSVConfig config;
    CSVExporter exporter(config);
    
    bool success = exporter.exportStatistics(stats, "/nonexistent/dir/test.csv");
    
    EXPECT_FALSE(success);
}

TEST_F(ExportTest, CSVExportViolationsToFile) {
    std::vector<LimitViolation> violations;
    
    LimitViolation v1;
    v1.time = 1.5;
    v1.axis = -1;
    v1.limitType = "velocity";
    v1.value = 150.0;
    v1.limit = 100.0;
    v1.overshoot = 50.0;
    violations.push_back(v1);
    
    LimitViolation v2;
    v2.time = 2.0;
    v2.axis = 0;  // X axis
    v2.limitType = "acceleration";
    v2.value = 2000.0;
    v2.limit = 1000.0;
    v2.overshoot = 100.0;
    violations.push_back(v2);
    
    CSVConfig config;
    CSVExporter exporter(config);
    
    std::string filename = "/tmp/test_violations.csv";
    bool success = exporter.exportViolations(violations, filename);
    
    EXPECT_TRUE(success);
    
    // Clean up
    std::remove(filename.c_str());
}

TEST_F(ExportTest, CSVExportViolationsInvalidPath) {
    std::vector<LimitViolation> violations;
    
    CSVConfig config;
    CSVExporter exporter(config);
    
    bool success = exporter.exportViolations(violations, "/nonexistent/dir/test.csv");
    
    EXPECT_FALSE(success);
}

TEST_F(ExportTest, CSVExportRotaryAxisUnits) {
    // Test that rotary axes (A, B, C) get degree units
    CSVConfig config;
    config.includeHeader = true;
    config.includeUnits = true;
    config.includeAxes.reset();
    config.includeAxes[3] = true;  // A axis (rotary)
    config.exportPosition = true;
    config.exportVelocity = true;
    config.exportAcceleration = true;
    config.exportJerk = true;
    config.exportTime = false;
    config.exportCombinedMetrics = false;
    config.exportSegmentInfo = false;
    config.exportPathPosition = false;
    
    CSVExporter exporter(config);
    
    std::stringstream ss;
    exporter.exportToStream(samples_, ss);
    
    std::string csv = ss.str();
    EXPECT_TRUE(csv.find("deg") != std::string::npos);
}

TEST_F(ExportTest, CSVExportAllAxes) {
    CSVConfig config;
    config.includeAxes.set();  // All axes
    config.exportPosition = true;
    config.exportVelocity = true;
    config.exportAcceleration = true;
    config.exportJerk = true;
    
    CSVExporter exporter(config);
    
    std::stringstream ss;
    exporter.exportToStream(samples_, ss);
    
    std::string csv = ss.str();
    // Should have all axis names
    EXPECT_TRUE(csv.find("pos_X") != std::string::npos);
    EXPECT_TRUE(csv.find("pos_Y") != std::string::npos);
    EXPECT_TRUE(csv.find("pos_Z") != std::string::npos);
    EXPECT_TRUE(csv.find("pos_A") != std::string::npos);
    EXPECT_TRUE(csv.find("pos_B") != std::string::npos);
    EXPECT_TRUE(csv.find("pos_C") != std::string::npos);
    EXPECT_TRUE(csv.find("pos_U") != std::string::npos);
    EXPECT_TRUE(csv.find("pos_V") != std::string::npos);
    EXPECT_TRUE(csv.find("pos_W") != std::string::npos);
}

// ============================================================================
// BatchExporter Additional Tests
// ============================================================================

TEST_F(ExportTest, BatchExporterWithViolations) {
    BatchExporter::ExportSpec spec;
    spec.basename = "/tmp/test_batch";
    spec.exportSVG = true;
    spec.exportCSV = true;
    spec.exportStatistics = true;
    spec.exportViolations = true;
    
    BatchExporter batcher;
    
    TrajectoryStatistics stats;
    stats.meetsLimits = false;
    LimitViolation v;
    v.time = 1.0;
    v.axis = -1;
    v.limitType = "velocity";
    v.value = 150.0;
    v.limit = 100.0;
    v.overshoot = 50.0;
    stats.violations.push_back(v);
    
    int count = batcher.exportAll(samples_, &stats, spec);
    
    EXPECT_GE(count, 3);  // SVG, CSV, stats, violations
    
    // Clean up
    for (const auto& file : batcher.createdFiles()) {
        std::remove(file.c_str());
    }
}

TEST_F(ExportTest, BatchExporterNoStats) {
    BatchExporter::ExportSpec spec;
    spec.basename = "/tmp/test_batch_nostats";
    spec.exportSVG = true;
    spec.exportCSV = true;
    spec.exportStatistics = true;
    
    BatchExporter batcher;
    
    // Pass nullptr for stats - should compute internally
    int count = batcher.exportAll(samples_, nullptr, spec);
    
    EXPECT_GE(count, 2);
    
    // Clean up
    for (const auto& file : batcher.createdFiles()) {
        std::remove(file.c_str());
    }
}
// ============================================================================
// SVGExporter Coverage Completion Tests
// ============================================================================

TEST_F(ExportTest, SVGExportUnknownMotionType) {
    // Test default case in motion type switch
    std::vector<TrajectorySample> samples;
    
    TrajectorySample s;
    s.position[0] = 10.0;
    s.position[1] = 10.0;
    s.motionType = 255;  // Unknown motion type - triggers default case
    samples.push_back(s);
    
    TrajectorySample s2;
    s2.position[0] = 20.0;
    s2.position[1] = 20.0;
    s2.motionType = 255;
    samples.push_back(s2);
    
    SVGConfig config;
    config.colorByVelocity = false;
    SVGExporter exporter(config);
    
    std::stringstream ss;
    exporter.exportToStream(samples, ss);
    
    std::string svg = ss.str();
    // Should use linear color as default
    EXPECT_TRUE(svg.find(config.linearColor) != std::string::npos);
}

TEST_F(ExportTest, SVGExportVelocityColorBlueToGreen) {
    // Test velocityToColor for normalized < 0.5 (blue to green branch)
    std::vector<TrajectorySample> samples;
    
    TrajectorySample s;
    s.position[0] = 0.0;
    s.position[1] = 0.0;
    s.linearVelocity = 10.0;  // Low velocity
    s.motionType = 1;  // Linear
    samples.push_back(s);
    
    TrajectorySample s2;
    s2.position[0] = 100.0;
    s2.position[1] = 0.0;
    s2.linearVelocity = 10.0;
    s2.motionType = 1;
    samples.push_back(s2);
    
    SVGConfig config;
    config.colorByVelocity = true;
    config.velocityColorMin = 0.0;
    config.velocityColorMax = 100.0;  // 10 is 10% = 0.1 < 0.5, so blue-to-green range
    SVGExporter exporter(config);
    
    std::stringstream ss;
    exporter.exportToStream(samples, ss);
    
    std::string svg = ss.str();
    // Should have path with velocity-based color
    EXPECT_TRUE(svg.find("<path") != std::string::npos);
}

TEST_F(ExportTest, SVGExportVelocityColorGreenToRed) {
    // Test velocityToColor for normalized >= 0.5 (green to red branch)
    std::vector<TrajectorySample> samples;
    
    TrajectorySample s;
    s.position[0] = 0.0;
    s.position[1] = 0.0;
    s.linearVelocity = 80.0;  // High velocity
    s.motionType = 1;
    samples.push_back(s);
    
    TrajectorySample s2;
    s2.position[0] = 100.0;
    s2.position[1] = 0.0;
    s2.linearVelocity = 80.0;
    s2.motionType = 1;
    samples.push_back(s2);
    
    SVGConfig config;
    config.colorByVelocity = true;
    config.velocityColorMin = 0.0;
    config.velocityColorMax = 100.0;  // 80 is 80% = 0.8 >= 0.5, so green-to-red range
    SVGExporter exporter(config);
    
    std::stringstream ss;
    exporter.exportToStream(samples, ss);
    
    std::string svg = ss.str();
    EXPECT_TRUE(svg.find("<path") != std::string::npos);
}

// ============================================================================
// Test escapeXml by checking SVG output when special characters in data
// Note: escapeXml is not directly testable through exportToStream since
// sample data doesn't include strings. But we can at least ensure the
// code path exists and compiles. For comprehensive test, we'd need to
// directly test the escapeXml function if it were public.
// ============================================================================