/**
 * @file test_SvgExporter.cpp
 * @brief Tests for the SvgExporter SVG vector graphics export module.
 */

#include "tether/motion_replanner/SvgExporter.hpp"
#include "tether/motion_replanner/PathEvaluator.hpp"
#include "tether/motion_replanner/PathRelativeFFT.hpp"

#include <gtest/gtest.h>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <vector>

using namespace MotionReplanner;
using namespace tether::motion::replanner;
using GCodeExport::TrajectorySample;

namespace {

TrajectorySample makeSample(double t, double x, double y,
                            int32_t seg, uint8_t motionType,
                            double pathPos = 0.0) {
    TrajectorySample s;
    s.time = t;
    s.pathPosition = pathPos;
    s.position = {x, y, 0, 0, 0, 0, 0, 0, 0};
    s.velocity = {0, 0, 0, 0, 0, 0, 0, 0, 0};
    s.acceleration = {0, 0, 0, 0, 0, 0, 0, 0, 0};
    s.segmentIndex = seg;
    s.motionType = motionType;
    s.curvature = 0.0;
    return s;
}

std::vector<TrajectorySample> makeLinePath(int n = 50) {
    std::vector<TrajectorySample> samples;
    double dt = 0.01;
    double dx = 100.0 / static_cast<double>(n - 1);
    double dy = 10.0 / static_cast<double>(n - 1);
    for (int i = 0; i < n; ++i) {
        double x = i * dx;
        double y = i * dy;
        double s = std::sqrt(x * x + y * y);
        samples.push_back(makeSample(i * dt, x, y, 0, 1, s));
    }
    return samples;
}

std::vector<TrajectorySample> makeOffsetActual(
    const std::vector<TrajectorySample>& desired, double offset) {
    std::vector<TrajectorySample> actual = desired;
    double nx = -0.0995, ny = 0.995;
    for (auto& s : actual) {
        s.position[0] += offset * nx;
        s.position[1] += offset * ny;
    }
    return actual;
}

/// Check that a file exists and is non-empty.
bool fileExistsAndNonEmpty(const std::string& path) {
    std::ifstream f(path, std::ios::binary | std::ios::ate);
    return f.is_open() && f.tellg() > 0;
}

/// Check that a file contains a specific substring.
bool fileContains(const std::string& path, const std::string& needle) {
    std::ifstream f(path);
    std::string content((std::istreambuf_iterator<char>(f)),
                        std::istreambuf_iterator<char>());
    return content.find(needle) != std::string::npos;
}

/// Create a unique temp directory for test output.
std::string makeTempDir() {
    auto dir = std::filesystem::temp_directory_path() /
               ("svg_test_" + std::to_string(std::rand()));
    std::filesystem::create_directories(dir);
    return dir.string();
}

} // anonymous namespace

//=============================================================================
// SVG structure tests
//=============================================================================

TEST(SvgExporter, HeaderAndFooter) {
    auto desired = makeLinePath();
    auto actual = makeOffsetActual(desired, 0.5);

    SvgExporter exporter;
    std::string path = makeTempDir() + "/test_header.svg";

    EXPECT_TRUE(exporter.exportPlot(path, SvgPlotType::TrajectoryXY, desired, actual));
    EXPECT_TRUE(fileExistsAndNonEmpty(path));

    // SVG should have XML header
    EXPECT_TRUE(fileContains(path, "<?xml version=\"1.0\""));
    // SVG should have svg root element
    EXPECT_TRUE(fileContains(path, "<svg xmlns=\"http://www.w3.org/2000/svg\""));
    // SVG should have closing tag
    EXPECT_TRUE(fileContains(path, "</svg>"));
}

TEST(SvgExporter, TrajectoryXY) {
    auto desired = makeLinePath();
    auto actual = makeOffsetActual(desired, 0.5);

    SvgExporter exporter;
    std::string path = makeTempDir() + "/test_xy.svg";

    EXPECT_TRUE(exporter.exportPlot(path, SvgPlotType::TrajectoryXY, desired, actual));
    EXPECT_TRUE(fileExistsAndNonEmpty(path));
    // Should contain polylines for both paths
    EXPECT_TRUE(fileContains(path, "<polyline"));
}

TEST(SvgExporter, TrajectoryXZ) {
    auto desired = makeLinePath();
    auto actual = makeOffsetActual(desired, 0.5);

    SvgExporter exporter;
    std::string path = makeTempDir() + "/test_xz.svg";

    EXPECT_TRUE(exporter.exportPlot(path, SvgPlotType::TrajectoryXZ, desired, actual));
    EXPECT_TRUE(fileExistsAndNonEmpty(path));
}

TEST(SvgExporter, TrajectoryYZ) {
    auto desired = makeLinePath();
    auto actual = makeOffsetActual(desired, 0.5);

    SvgExporter exporter;
    std::string path = makeTempDir() + "/test_yz.svg";

    EXPECT_TRUE(exporter.exportPlot(path, SvgPlotType::TrajectoryYZ, desired, actual));
    EXPECT_TRUE(fileExistsAndNonEmpty(path));
}

TEST(SvgExporter, Trajectory3D) {
    auto desired = makeLinePath();
    auto actual = makeOffsetActual(desired, 0.5);

    SvgExporter exporter;
    std::string path = makeTempDir() + "/test_3d.svg";

    EXPECT_TRUE(exporter.exportPlot(path, SvgPlotType::Trajectory3D, desired, actual));
    EXPECT_TRUE(fileExistsAndNonEmpty(path));
}

TEST(SvgExporter, ErrorVsPathLength) {
    auto desired = makeLinePath();
    auto actual = makeOffsetActual(desired, 0.5);

    SvgExporter exporter;
    std::string path = makeTempDir() + "/test_error_vs_path.svg";

    EXPECT_TRUE(exporter.exportPlot(path, SvgPlotType::ErrorVsPathLength, desired, actual));
    EXPECT_TRUE(fileExistsAndNonEmpty(path));
}

TEST(SvgExporter, ErrorVsTime) {
    auto desired = makeLinePath();
    auto actual = makeOffsetActual(desired, 0.5);

    SvgExporter exporter;
    std::string path = makeTempDir() + "/test_error_vs_time.svg";

    EXPECT_TRUE(exporter.exportPlot(path, SvgPlotType::ErrorVsTime, desired, actual));
    EXPECT_TRUE(fileExistsAndNonEmpty(path));
}

TEST(SvgExporter, ErrorHistogram) {
    auto desired = makeLinePath();
    auto actual = makeOffsetActual(desired, 0.5);

    EvaluatorConfig evalConfig;
    evalConfig.useCertifiedContourError = false;
    PathEvaluator evaluator(evalConfig);
    auto quant = evaluator.evaluateQuantitative(desired, actual);

    SvgExporter exporter;
    std::string path = makeTempDir() + "/test_histogram.svg";

    EXPECT_TRUE(exporter.exportPlot(path, SvgPlotType::ErrorHistogram, desired, actual, &quant));
    EXPECT_TRUE(fileExistsAndNonEmpty(path));
    // Should contain rect elements for histogram bars
    EXPECT_TRUE(fileContains(path, "<rect"));
}

TEST(SvgExporter, ErrorEnvelope) {
    auto desired = makeLinePath();
    auto actual = makeOffsetActual(desired, 0.5);

    SvgConfig svgConfig;
    svgConfig.envelopeScale = 20.0;
    SvgExporter exporter(svgConfig);
    std::string path = makeTempDir() + "/test_envelope.svg";

    EXPECT_TRUE(exporter.exportPlot(path, SvgPlotType::ErrorEnvelope, desired, actual));
    EXPECT_TRUE(fileExistsAndNonEmpty(path));
}

TEST(SvgExporter, SpectralPlot) {
    auto desired = makeLinePath(128);
    // Add oscillation perpendicular to path
    std::vector<TrajectorySample> actual = desired;
    double nx = -0.0995, ny = 0.995;
    for (std::size_t i = 0; i < actual.size(); ++i) {
        double offset = 0.1 * std::sin(2.0 * M_PI * 0.1 * desired[i].pathPosition);
        actual[i].position[0] += offset * nx;
        actual[i].position[1] += offset * ny;
    }

    FFTConfig fftConfig;
    fftConfig.useCertifiedContourError = false;
    PathRelativeFFT fftEval(fftConfig);
    auto spectral = fftEval.evaluate(desired, actual);

    SvgExporter exporter;
    std::string path = makeTempDir() + "/test_spectral.svg";

    EXPECT_TRUE(exporter.exportPlot(path, SvgPlotType::SpectralMagnitude,
                                    desired, actual, nullptr, &spectral));
    EXPECT_TRUE(fileExistsAndNonEmpty(path));
}

TEST(SvgExporter, VelocityProfile) {
    auto desired = makeLinePath();
    auto actual = makeOffsetActual(desired, 0.5);

    SvgExporter exporter;
    std::string path = makeTempDir() + "/test_velocity.svg";

    EXPECT_TRUE(exporter.exportPlot(path, SvgPlotType::VelocityProfile, desired, actual));
    EXPECT_TRUE(fileExistsAndNonEmpty(path));
}

TEST(SvgExporter, AccelerationProfile) {
    auto desired = makeLinePath();
    auto actual = makeOffsetActual(desired, 0.5);

    SvgExporter exporter;
    std::string path = makeTempDir() + "/test_accel.svg";

    EXPECT_TRUE(exporter.exportPlot(path, SvgPlotType::AccelerationProfile, desired, actual));
    EXPECT_TRUE(fileExistsAndNonEmpty(path));
}

TEST(SvgExporter, PhasePortrait) {
    auto desired = makeLinePath();
    auto actual = makeOffsetActual(desired, 0.5);

    SvgExporter exporter;
    std::string path = makeTempDir() + "/test_phase.svg";

    EXPECT_TRUE(exporter.exportPlot(path, SvgPlotType::PhasePortrait, desired, actual));
    EXPECT_TRUE(fileExistsAndNonEmpty(path));
}

//=============================================================================
// Batch export tests
//=============================================================================

TEST(SvgExporter, ExportAllPlots) {
    auto desired = makeLinePath();
    auto actual = makeOffsetActual(desired, 0.5);

    EvaluatorConfig evalConfig;
    evalConfig.useCertifiedContourError = false;
    PathEvaluator evaluator(evalConfig);
    auto quant = evaluator.evaluateQuantitative(desired, actual);

    FFTConfig fftConfig;
    fftConfig.useCertifiedContourError = false;
    PathRelativeFFT fftEval(fftConfig);
    auto spectral = fftEval.evaluate(desired, actual);

    SvgExporter exporter;
    std::string dir = makeTempDir();

    auto files = exporter.exportAllPlots(dir, "test", desired, actual, quant, spectral);

    // Should generate multiple files
    EXPECT_GT(files.size(), 5u);

    // All files should exist
    for (const auto& f : files) {
        EXPECT_TRUE(fileExistsAndNonEmpty(f)) << "Missing: " << f;
    }
}

TEST(SvgExporter, Dashboard) {
    auto desired = makeLinePath();
    auto actual = makeOffsetActual(desired, 0.5);

    EvaluatorConfig evalConfig;
    evalConfig.useCertifiedContourError = false;
    PathEvaluator evaluator(evalConfig);
    auto quant = evaluator.evaluateQuantitative(desired, actual);

    FFTConfig fftConfig;
    fftConfig.useCertifiedContourError = false;
    PathRelativeFFT fftEval(fftConfig);
    auto spectral = fftEval.evaluate(desired, actual);

    SvgExporter exporter;
    std::string path = makeTempDir() + "/test_dashboard.svg";

    EXPECT_TRUE(exporter.exportDashboard(path, desired, actual, quant, spectral));
    EXPECT_TRUE(fileExistsAndNonEmpty(path));
    // Dashboard should contain multiple sub-plot groups
    EXPECT_TRUE(fileContains(path, "<g transform="));
}

//=============================================================================
// Configuration tests
//=============================================================================

TEST(SvgExporter, CustomConfig) {
    SvgConfig config;
    config.width = 800;
    config.height = 600;
    config.desiredColor = "#FF0000";
    config.actualColor = "#00FF00";
    config.includeGrid = false;
    config.includeLegend = false;

    SvgExporter exporter(config);

    auto desired = makeLinePath();
    auto actual = makeOffsetActual(desired, 0.5);

    std::string path = makeTempDir() + "/test_custom.svg";
    EXPECT_TRUE(exporter.exportPlot(path, SvgPlotType::TrajectoryXY, desired, actual));
    EXPECT_TRUE(fileExistsAndNonEmpty(path));

    // Should contain the custom colors
    EXPECT_TRUE(fileContains(path, "#FF0000"));
    EXPECT_TRUE(fileContains(path, "#00FF00"));

    // Should have the custom dimensions
    EXPECT_TRUE(fileContains(path, "width=\"800\""));
    EXPECT_TRUE(fileContains(path, "height=\"600\""));
}

TEST(SvgExporter, DeviationEnvelope) {
    SvgConfig config;
    config.showDeviationEnvelope = true;
    config.envelopeScale = 50.0;

    SvgExporter exporter(config);

    auto desired = makeLinePath();
    auto actual = makeOffsetActual(desired, 0.1);

    std::string path = makeTempDir() + "/test_devenv.svg";
    EXPECT_TRUE(exporter.exportPlot(path, SvgPlotType::TrajectoryXY, desired, actual));
    EXPECT_TRUE(fileExistsAndNonEmpty(path));
}

TEST(SvgExporter, EmptyInput) {
    std::vector<TrajectorySample> empty;

    SvgExporter exporter;
    std::string path = makeTempDir() + "/test_empty.svg";

    // Should not crash on empty input
    EXPECT_TRUE(exporter.exportPlot(path, SvgPlotType::TrajectoryXY, empty, empty));
}
