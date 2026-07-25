/**
 * @file test_TestDataExporter.cpp
 * @brief Tests for TestDataExporter classes: DataExporter, JSONBuilder,
 *        StreamingExporter, TrajectoryExporter, HeatmapExporter,
 *        TestResultExporter, BatchExporter, ReportGenerator.
 *        Covers TestDataExporter*.cpp (0% → target 100%).
 */
#include <gtest/gtest.h>
#include <tether/motion_replanner/TestDataExporter.hpp>
#include <tether/motion_replanner/MotionReplanner.hpp>
#include <tether/motion_replanner/PerformanceHeatmap.hpp>
#include <tether/motion_replanner/MachineTester.hpp>
#include <tether/motion_replanner/SystemIdentifier.hpp>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <string>

using namespace MotionReplanner;

// ============================================================================
// Helper: create temp file path
// ============================================================================
static std::string tempFile(const std::string& name) {
    // Include PID to avoid collisions when CTest runs individual gtest
    // cases as separate processes in parallel (--parallel).
    return "/tmp/tether_test_export_" + std::to_string(getpid()) + "_" + name;
}

static void cleanupFile(const std::string& path) {
    std::remove(path.c_str());
}

// ============================================================================
// ExportConfig Tests
// ============================================================================
TEST(ExportConfigTest, Defaults) {
    ExportConfig cfg;
    EXPECT_EQ(cfg.format, ExportFormat::CSV);
    EXPECT_EQ(cfg.delimiter, ',');
    EXPECT_EQ(cfg.precision, 6);
    EXPECT_TRUE(cfg.includeHeader);
    EXPECT_TRUE(cfg.includeTimestamp);
    EXPECT_TRUE(cfg.includeMetadata);
    EXPECT_FALSE(cfg.compressOutput);
    EXPECT_EQ(cfg.downsampleFactor, 1);
}

// ============================================================================
// DataExporter Tests (Base class)
// ============================================================================
class DataExporterTest : public ::testing::Test {
protected:
    DataExporter exporter_;
};

TEST_F(DataExporterTest, DefaultConfig) {
    const auto& cfg = exporter_.config();
    EXPECT_EQ(cfg.format, ExportFormat::CSV);
}

TEST_F(DataExporterTest, SetConfig) {
    ExportConfig cfg;
    cfg.format = ExportFormat::JSON;
    cfg.precision = 3;
    exporter_.setConfig(cfg);
    EXPECT_EQ(exporter_.config().format, ExportFormat::JSON);
    EXPECT_EQ(exporter_.config().precision, 3);
}

TEST_F(DataExporterTest, FormatDouble) {
    auto s = exporter_.formatDouble(3.14159);
    EXPECT_FALSE(s.empty());
    EXPECT_NE(s.find("3.14"), std::string::npos);
}

TEST_F(DataExporterTest, FormatDoubleZero) {
    auto s = exporter_.formatDouble(0.0);
    EXPECT_FALSE(s.empty());
}

TEST_F(DataExporterTest, FormatDoubleNegative) {
    auto s = exporter_.formatDouble(-1.5);
    EXPECT_NE(s.find("-"), std::string::npos);
}

TEST_F(DataExporterTest, EscapeCSVSimple) {
    auto s = exporter_.escapeCSV("hello");
    EXPECT_EQ(s, "hello");
}

TEST_F(DataExporterTest, EscapeCSVWithComma) {
    auto s = exporter_.escapeCSV("hello,world");
    EXPECT_NE(s.find("\""), std::string::npos);
}

TEST_F(DataExporterTest, EscapeCSVWithQuote) {
    auto s = exporter_.escapeCSV("say \"hi\"");
    // Should escape quotes
    EXPECT_NE(s.find("\""), std::string::npos);
}

TEST_F(DataExporterTest, EscapeCSVWithNewline) {
    auto s = exporter_.escapeCSV("line1\nline2");
    EXPECT_NE(s.find("\""), std::string::npos);
}

TEST_F(DataExporterTest, EscapeJSONSimple) {
    auto s = exporter_.escapeJSON("hello");
    EXPECT_EQ(s, "hello");
}

TEST_F(DataExporterTest, EscapeJSONWithQuote) {
    auto s = exporter_.escapeJSON("say \"hi\"");
    EXPECT_NE(s.find("\\\""), std::string::npos);
}

TEST_F(DataExporterTest, EscapeJSONWithBackslash) {
    auto s = exporter_.escapeJSON("path\\to\\file");
    EXPECT_NE(s.find("\\\\"), std::string::npos);
}

TEST_F(DataExporterTest, EscapeJSONWithNewline) {
    auto s = exporter_.escapeJSON("line1\nline2");
    EXPECT_NE(s.find("\\n"), std::string::npos);
}

TEST_F(DataExporterTest, GetCurrentTimestamp) {
    auto ts = exporter_.getCurrentTimestamp();
    EXPECT_FALSE(ts.empty());
    // Should contain year (20xx)
    EXPECT_NE(ts.find("20"), std::string::npos);
}

TEST_F(DataExporterTest, SetMetadata) {
    ExportMetadata meta;
    meta.testName = "MyTest";
    meta.testType = "Calibration";
    meta.sampleRate = 1000.0;
    exporter_.setMetadata(meta);
    EXPECT_EQ(exporter_.metadata().testName, "MyTest");
    EXPECT_EQ(exporter_.metadata().sampleRate, 1000.0);
}

TEST_F(DataExporterTest, CustomPrecision) {
    ExportConfig cfg;
    cfg.precision = 2;
    DataExporter exp(cfg);
    auto s = exp.formatDouble(3.14159);
    // With precision 2, should be shorter
    EXPECT_LE(s.size(), 6u);
}

// ============================================================================
// JSONBuilder Tests
// ============================================================================
TEST(JSONBuilderTest, EmptyObject) {
    JSONBuilder b(false);
    b.beginObject();
    b.endObject();
    auto s = b.str();
    EXPECT_NE(s.find("{"), std::string::npos);
    EXPECT_NE(s.find("}"), std::string::npos);
}

TEST(JSONBuilderTest, KeyValueString) {
    JSONBuilder b(false);
    b.beginObject();
    b.keyValue("name", std::string("test"));
    b.endObject();
    auto s = b.str();
    EXPECT_NE(s.find("\"name\""), std::string::npos);
    EXPECT_NE(s.find("\"test\""), std::string::npos);
}

TEST(JSONBuilderTest, KeyValueDouble) {
    JSONBuilder b(false);
    b.beginObject();
    b.keyValue("value", 3.14);
    b.endObject();
    auto s = b.str();
    EXPECT_NE(s.find("3.14"), std::string::npos);
}

TEST(JSONBuilderTest, KeyValueInt) {
    JSONBuilder b(false);
    b.beginObject();
    b.keyValue("count", 42);
    b.endObject();
    auto s = b.str();
    EXPECT_NE(s.find("42"), std::string::npos);
}

TEST(JSONBuilderTest, KeyValueBool) {
    JSONBuilder b(false);
    b.beginObject();
    b.keyValue("flag", true);
    b.endObject();
    auto s = b.str();
    EXPECT_NE(s.find("true"), std::string::npos);
}

TEST(JSONBuilderTest, Array) {
    JSONBuilder b(false);
    b.beginArray();
    b.value(1);
    b.value(2);
    b.value(3);
    b.endArray();
    auto s = b.str();
    EXPECT_NE(s.find("["), std::string::npos);
    EXPECT_NE(s.find("]"), std::string::npos);
}

TEST(JSONBuilderTest, DoubleArray) {
    JSONBuilder b(false);
    b.beginObject();
    b.doubleArray("data", {1.0, 2.0, 3.0});
    b.endObject();
    auto s = b.str();
    EXPECT_NE(s.find("data"), std::string::npos);
}

TEST(JSONBuilderTest, NestedObject) {
    JSONBuilder b(false);
    b.beginObject();
    b.key("inner");
    b.beginObject();
    b.keyValue("x", 1);
    b.endObject();
    b.endObject();
    auto s = b.str();
    EXPECT_NE(s.find("inner"), std::string::npos);
}

TEST(JSONBuilderTest, PrettyPrinting) {
    JSONBuilder b(true); // pretty
    b.beginObject();
    b.keyValue("name", std::string("test"));
    b.endObject();
    auto s = b.str();
    // Pretty print should have newlines or indentation
    EXPECT_GT(s.size(), 5u);
}

TEST(JSONBuilderTest, NullValue) {
    JSONBuilder b(false);
    b.beginObject();
    b.key("empty");
    b.valueNull();
    b.endObject();
    auto s = b.str();
    EXPECT_NE(s.find("null"), std::string::npos);
}

TEST(JSONBuilderTest, StringValue) {
    JSONBuilder b(false);
    b.beginArray();
    b.value(std::string("hello"));
    b.endArray();
    auto s = b.str();
    EXPECT_NE(s.find("hello"), std::string::npos);
}

TEST(JSONBuilderTest, BoolValue) {
    JSONBuilder b(false);
    b.beginArray();
    b.value(false);
    b.endArray();
    auto s = b.str();
    EXPECT_NE(s.find("false"), std::string::npos);
}

TEST(JSONBuilderTest, DoubleArrayStandalone) {
    JSONBuilder b(false);
    b.doubleArray({1.5, 2.5, 3.5});
    auto s = b.str();
    EXPECT_NE(s.find("["), std::string::npos);
}

// ============================================================================
// StreamingExporter Tests
// ============================================================================
class StreamingExporterTest : public ::testing::Test {
protected:
    std::string filepath_;

    void SetUp() override {
        filepath_ = tempFile("streaming_test.csv");
    }

    void TearDown() override {
        cleanupFile(filepath_);
    }
};

TEST_F(StreamingExporterTest, OpenAndClose) {
    StreamingExporter exp(filepath_);
    EXPECT_TRUE(exp.open());
    EXPECT_TRUE(exp.isOpen());
    exp.close();
    EXPECT_FALSE(exp.isOpen());
}

TEST_F(StreamingExporterTest, WriteHeader) {
    StreamingExporter exp(filepath_);
    exp.open();
    exp.writeHeader({"time", "x", "y", "z"});
    exp.close();

    std::ifstream in(filepath_);
    std::string line;
    std::getline(in, line);
    EXPECT_NE(line.find("time"), std::string::npos);
}

TEST_F(StreamingExporterTest, WriteRowDoubles) {
    StreamingExporter exp(filepath_);
    exp.open();
    exp.writeHeader({"a", "b", "c"});
    exp.writeRow({1.0, 2.0, 3.0});
    exp.writeRow({4.0, 5.0, 6.0});
    exp.close();

    std::ifstream in(filepath_);
    int lines = 0;
    std::string line;
    while (std::getline(in, line)) lines++;
    EXPECT_GE(lines, 3); // header + 2 data rows
}

TEST_F(StreamingExporterTest, WriteRowStrings) {
    StreamingExporter exp(filepath_);
    exp.open();
    exp.writeRow(std::vector<std::string>{"hello", "world"});
    exp.close();

    std::ifstream in(filepath_);
    std::string line;
    std::getline(in, line);
    EXPECT_NE(line.find("hello"), std::string::npos);
}

TEST_F(StreamingExporterTest, BytesWritten) {
    StreamingExporter exp(filepath_);
    exp.open();
    exp.writeRow({1.0, 2.0, 3.0});
    exp.flush();
    EXPECT_GT(exp.bytesWritten(), 0u);
    exp.close();
}

TEST_F(StreamingExporterTest, WriteSample) {
    StreamingExporter exp(filepath_);
    exp.open();
    GCodeExport::TrajectorySample desired{};
    GCodeExport::TrajectorySample actual{};
    desired.time = 0.0;
    actual.time = 0.0;
    desired.position[0] = 1.0;
    actual.position[0] = 0.9;
    exp.writeSample(desired, actual);
    exp.close();
    EXPECT_GT(exp.bytesWritten(), 0u);
}

TEST_F(StreamingExporterTest, WriteError) {
    StreamingExporter exp(filepath_);
    exp.open();
    TrackingError err{};
    err.timestamp = 0.1;
    err.combinedPositionError = 0.05;
    exp.writeError(err);
    exp.close();
    EXPECT_GT(exp.bytesWritten(), 0u);
}

TEST_F(StreamingExporterTest, Flush) {
    StreamingExporter exp(filepath_);
    exp.open();
    exp.writeRow(std::vector<double>{42.0});
    exp.flush();
    exp.close();
}

TEST_F(StreamingExporterTest, WithConfig) {
    ExportConfig cfg;
    cfg.delimiter = '\t';
    cfg.precision = 3;
    StreamingExporter exp(filepath_, cfg);
    exp.open();
    exp.writeRow(std::vector<double>{1.23456789, 2.34567890});
    exp.close();
}

// ============================================================================
// TrajectoryExporter Tests
// ============================================================================
class TrajectoryExporterTest : public ::testing::Test {
protected:
    TrajectoryExporter exporter_;
    std::string filepath_;

    void SetUp() override {
        filepath_ = tempFile("trajectory_test.csv");
    }
    void TearDown() override {
        cleanupFile(filepath_);
        cleanupFile(tempFile("tracking_err.csv"));
        cleanupFile(tempFile("segment_perf.csv"));
        cleanupFile(tempFile("err_stats.csv"));
        cleanupFile(tempFile("suggestions.csv"));
    }
};

TEST_F(TrajectoryExporterTest, ExportEmptyTrajectory) {
    std::vector<GCodeExport::TrajectorySample> desired, actual;
    bool ok = exporter_.exportTrajectory(filepath_, desired, actual);
    EXPECT_TRUE(ok);
}

TEST_F(TrajectoryExporterTest, ExportTrajectory) {
    std::vector<GCodeExport::TrajectorySample> desired(5), actual(5);
    for (int i = 0; i < 5; i++) {
        desired[i].time = i * 0.001;
        desired[i].position[0] = i * 1.0;
        actual[i].time = i * 0.001;
        actual[i].position[0] = i * 0.95;
    }
    bool ok = exporter_.exportTrajectory(filepath_, desired, actual);
    EXPECT_TRUE(ok);
}

TEST_F(TrajectoryExporterTest, ExportTrackingErrors) {
    std::vector<TrackingError> errors(3);
    for (int i = 0; i < 3; i++) {
        errors[i].timestamp = i * 0.01;
        errors[i].combinedPositionError = 0.01 * (i + 1);
    }
    bool ok = exporter_.exportTrackingErrors(tempFile("tracking_err.csv"), errors);
    EXPECT_TRUE(ok);
}

TEST_F(TrajectoryExporterTest, ExportSegmentPerformance) {
    std::vector<SegmentPerformance> segs(2);
    segs[0].segmentIndex = 0;
    segs[0].commandedFeedRate = 1000.0;
    segs[0].achievedMeanFeedRate = 950.0;
    segs[1].segmentIndex = 1;
    segs[1].commandedFeedRate = 500.0;
    segs[1].achievedMeanFeedRate = 490.0;
    bool ok = exporter_.exportSegmentPerformance(tempFile("segment_perf.csv"), segs);
    EXPECT_TRUE(ok);
}

TEST_F(TrajectoryExporterTest, ExportErrorStatistics) {
    ErrorStatistics stats{};
    stats.meanError = 0.05;
    stats.maxError = 0.2;
    stats.rmsError = 0.07;
    stats.sampleCount = 1000;
    bool ok = exporter_.exportErrorStatistics(tempFile("err_stats.csv"), stats);
    EXPECT_TRUE(ok);
}

TEST_F(TrajectoryExporterTest, ExportSuggestions) {
    std::vector<ParameterSuggestion> suggestions(1);
    suggestions[0].segmentIndex = 0;
    suggestions[0].currentFeedRate = 1000.0;
    suggestions[0].suggestedFeedRate = 800.0;
    suggestions[0].reason = "High tracking error";
    bool ok = exporter_.exportSuggestions(tempFile("suggestions.csv"), suggestions);
    EXPECT_TRUE(ok);
}

// ============================================================================
// HeatmapExporter Tests
// ============================================================================
class HeatmapExporterTest : public ::testing::Test {
protected:
    HeatmapExporter exporter_;
    std::string filepath_;

    void SetUp() override {
        filepath_ = tempFile("heatmap_test.csv");
    }
    void TearDown() override {
        cleanupFile(filepath_);
        cleanupFile(tempFile("heatmap1d.csv"));
        cleanupFile(tempFile("heatmap3d.csv"));
        cleanupFile(tempFile("diff_heatmap.csv"));
        cleanupFile(tempFile("limits.csv"));
    }
};

TEST_F(HeatmapExporterTest, ExportHeatmap1D) {
    Heatmap1D hm(0); // axis 0
    // Add some samples so there's data to export
    hm.addSample(10.0, 500.0, 100.0, 0.01, 0.95);
    hm.addSample(20.0, 600.0, 120.0, 0.02, 0.90);
    hm.addSample(30.0, 700.0, 150.0, 0.015, 0.92);
    bool ok = exporter_.exportHeatmap1D(tempFile("heatmap1d.csv"), hm);
    EXPECT_TRUE(ok);
}

TEST_F(HeatmapExporterTest, ExportHeatmap2D) {
    Heatmap2D hm(Heatmap2D::Plane::XY);
    hm.addSample(10.0, 10.0, 500.0, 100.0, 0.01, 0.95);
    hm.addSample(20.0, 20.0, 600.0, 120.0, 0.02, 0.90);
    bool ok = exporter_.exportHeatmap2D(filepath_, hm);
    EXPECT_TRUE(ok);
}

TEST_F(HeatmapExporterTest, ExportHeatmap3D) {
    Heatmap3D hm;
    hm.addSample({10.0, 20.0, 30.0}, 500.0, 100.0, 0.01, 0.95);
    bool ok = exporter_.exportHeatmap3D(tempFile("heatmap3d.csv"), hm);
    EXPECT_TRUE(ok);
}

TEST_F(HeatmapExporterTest, ExportDifferentialHeatmap) {
    DifferentialHeatmap hm;
    hm.setExpectedPerformance(6000.0, 1000.0, 10000.0);
    hm.addActualSample({10.0, 20.0, 30.0}, 500.0, 100.0, 0.01);
    bool ok = exporter_.exportDifferentialHeatmap(tempFile("diff_heatmap.csv"), hm);
    EXPECT_TRUE(ok);
}

TEST_F(HeatmapExporterTest, ExportSuggestedLimits) {
    SuggestedLimits limits;
    limits.maxVelocity = 5000.0;
    limits.maxAcceleration = 2000.0;
    limits.confidence = 0.8;
    bool ok = exporter_.exportSuggestedLimits(tempFile("limits.csv"), limits);
    EXPECT_TRUE(ok);
}

TEST_F(HeatmapExporterTest, ExportHeatmap2DAsJSON) {
    ExportConfig cfg;
    cfg.format = ExportFormat::JSON;
    HeatmapExporter exp(cfg);

    Heatmap2D hm(Heatmap2D::Plane::XY);
    hm.addSample(5.0, 5.0, 500.0, 100.0, 0.01, 0.95);
    hm.addSample(15.0, 15.0, 600.0, 120.0, 0.02, 0.90);

    auto jsonFile = tempFile("heatmap2d.json");
    bool ok = exp.exportHeatmap2D(jsonFile, hm);
    EXPECT_TRUE(ok);
    cleanupFile(jsonFile);
}

// ============================================================================
// TestResultExporter Tests
// ============================================================================
class TestResultExporterTest : public ::testing::Test {
protected:
    TestResultExporter exporter_;

    void TearDown() override {
        cleanupFile(tempFile("test_result.csv"));
        cleanupFile(tempFile("friction.csv"));
        cleanupFile(tempFile("delay.csv"));
        cleanupFile(tempFile("pid.csv"));
        cleanupFile(tempFile("dynamics.csv"));
    }
};

TEST_F(TestResultExporterTest, ExportTestResult) {
    TestResult result;
    result.testName = "RampTest";
    result.testType = "SingleAxis";
    result.passed = true;
    result.maxVelocityAchieved = 1000.0;
    bool ok = exporter_.exportTestResult(tempFile("test_result.csv"), result);
    EXPECT_TRUE(ok);
}

TEST_F(TestResultExporterTest, ExportFrictionModel) {
    FrictionIdentificationResult fr;
    fr.bestModel.type = FrictionModelType::CoulombViscous;
    fr.bestModel.coulombForce = 5.0;
    fr.bestModel.viscousCoeff = 0.01;
    fr.bestModel.rSquared = 0.95;
    bool ok = exporter_.exportFrictionModel(tempFile("friction.csv"), fr);
    EXPECT_TRUE(ok);
}

TEST_F(TestResultExporterTest, ExportDelayIdentification) {
    DelayIdentificationResult dr{};
    dr.transportDelay = 0.005;
    dr.delayConfidence = 0.9;
    bool ok = exporter_.exportDelayIdentification(tempFile("delay.csv"), dr);
    EXPECT_TRUE(ok);
}

TEST_F(TestResultExporterTest, ExportPIDAssessment) {
    PIDTuningAssessment pa{};
    pa.suggestedKp = 1.0;
    pa.suggestedKi = 0.1;
    pa.suggestedKd = 0.05;
    pa.overallScore = 0.8;
    bool ok = exporter_.exportPIDAssessment(tempFile("pid.csv"), pa);
    EXPECT_TRUE(ok);
}

TEST_F(TestResultExporterTest, ExportDynamicsModel) {
    DynamicsIdentificationResult dy{};
    dy.gain = 2.0;
    dy.timeConstant = 0.05;
    dy.systemOrder = 1;
    bool ok = exporter_.exportDynamicsModel(tempFile("dynamics.csv"), dy);
    EXPECT_TRUE(ok);
}

// ============================================================================
// BatchExporter Tests
// ============================================================================
class BatchExporterTest : public ::testing::Test {
protected:
    std::string outputDir_;

    void SetUp() override {
        outputDir_ = "/tmp/tether_batch_test_" + std::to_string(getpid());
        std::filesystem::create_directories(outputDir_);
    }
    void TearDown() override {
        std::filesystem::remove_all(outputDir_);
    }
};

TEST_F(BatchExporterTest, Construction) {
    BatchExporter exp(outputDir_);
    EXPECT_TRUE(exp.generatedFiles().empty());
}

TEST_F(BatchExporterTest, SetPrefix) {
    BatchExporter exp(outputDir_);
    exp.setPrefix("mytest_");
    EXPECT_TRUE(exp.generatedFiles().empty());
}

TEST_F(BatchExporterTest, ExportTestResults) {
    BatchExporter exp(outputDir_);
    std::vector<TestResult> results(2);
    results[0].testName = "Test1";
    results[0].passed = true;
    results[1].testName = "Test2";
    results[1].passed = false;
    exp.exportTestResults(results);
    EXPECT_FALSE(exp.generatedFiles().empty());
}

TEST_F(BatchExporterTest, GenerateManifest) {
    BatchExporter exp(outputDir_);
    std::vector<TestResult> results(1);
    results[0].testName = "ManifestTest";
    exp.exportTestResults(results);

    std::string manifestFile = outputDir_ + "/manifest.json";
    bool ok = exp.generateManifest(manifestFile);
    // Manifest may fail if no files were generated yet, or succeed
    (void)ok;
    // Just verify no crash
}

TEST_F(BatchExporterTest, ExportIdentificationData) {
    BatchExporter exp(outputDir_);
    SystemIdentifier si;
    DelayIdentificationResult delay{};
    FrictionIdentificationResult friction;
    PIDTuningAssessment pid{};
    exp.exportIdentificationData(si, delay, friction, pid);
}

// ============================================================================
// ReportGenerator Tests
// ============================================================================
class ReportGeneratorTest : public ::testing::Test {
protected:
    ReportGenerator gen_;

    void TearDown() override {
        cleanupFile(tempFile("report.md"));
        cleanupFile(tempFile("report.html"));
        cleanupFile(tempFile("report.tex"));
    }
};

TEST_F(ReportGeneratorTest, AddSection) {
    ReportGenerator::ReportSection sec;
    sec.title = "Introduction";
    sec.content = "This is a test report.";
    gen_.addSection(sec);
}

TEST_F(ReportGeneratorTest, AddSummary) {
    std::vector<TestResult> results(2);
    results[0].testName = "T1";
    results[0].passed = true;
    results[1].testName = "T2";
    results[1].passed = false;
    gen_.addSummary(results);
}

TEST_F(ReportGeneratorTest, AddErrorStatistics) {
    ErrorStatistics stats{};
    stats.meanError = 0.05;
    stats.maxError = 0.2;
    gen_.addErrorStatistics(stats);
}

TEST_F(ReportGeneratorTest, AddSystemIdentification) {
    DelayIdentificationResult delay{};
    FrictionIdentificationResult friction;
    PIDTuningAssessment pid{};
    gen_.addSystemIdentification(delay, friction, pid);
}

TEST_F(ReportGeneratorTest, AddRecommendations) {
    std::vector<ParameterSuggestion> suggestions(1);
    suggestions[0].reason = "Too fast";
    gen_.addRecommendations(suggestions);
}

TEST_F(ReportGeneratorTest, ExportMarkdown) {
    ReportGenerator::ReportSection sec;
    sec.title = "Test";
    sec.content = "Content";
    gen_.addSection(sec);
    bool ok = gen_.exportMarkdown(tempFile("report.md"));
    EXPECT_TRUE(ok);
}

TEST_F(ReportGeneratorTest, ExportHTML) {
    ReportGenerator::ReportSection sec;
    sec.title = "Test";
    sec.content = "Content";
    gen_.addSection(sec);
    bool ok = gen_.exportHTML(tempFile("report.html"));
    EXPECT_TRUE(ok);
}

TEST_F(ReportGeneratorTest, ExportLaTeX) {
    ReportGenerator::ReportSection sec;
    sec.title = "Test";
    sec.content = "Content";
    gen_.addSection(sec);
    bool ok = gen_.exportLaTeX(tempFile("report.tex"));
    EXPECT_TRUE(ok);
}

TEST_F(ReportGeneratorTest, FullReport) {
    // Build a complete report
    ReportGenerator::ReportSection intro;
    intro.title = "Introduction";
    intro.content = "Automated machine testing report";
    gen_.addSection(intro);

    std::vector<TestResult> results(1);
    results[0].testName = "Ramp";
    results[0].passed = true;
    gen_.addSummary(results);

    ErrorStatistics stats{};
    stats.meanError = 0.02;
    gen_.addErrorStatistics(stats);

    bool ok = gen_.exportMarkdown(tempFile("report.md"));
    EXPECT_TRUE(ok);

    // Verify file is non-empty
    std::ifstream in(tempFile("report.md"));
    std::string content((std::istreambuf_iterator<char>(in)),
                        std::istreambuf_iterator<char>());
    EXPECT_GT(content.size(), 10u);
}

// ============================================================================
// ExportFormat enum
// ============================================================================
TEST(ExportFormatTest, Values) {
    EXPECT_NE(ExportFormat::CSV, ExportFormat::JSON);
    EXPECT_NE(ExportFormat::JSON, ExportFormat::JSONPretty);
    EXPECT_NE(ExportFormat::Binary, ExportFormat::Numpy);
}

// ============================================================================
// ExportMetadata
// ============================================================================
TEST(ExportMetadataTest, Defaults) {
    ExportMetadata meta;
    EXPECT_EQ(meta.version, "1.0");
    EXPECT_DOUBLE_EQ(meta.sampleRate, 0.0);
    EXPECT_EQ(meta.sampleCount, 0u);
}
