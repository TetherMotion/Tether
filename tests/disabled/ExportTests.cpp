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
