/**
 * @file test_KdeSvgRenderer.cpp
 * @brief Coverage for KdeSvgRenderer — KDE heatmap/conditional/marginal
 *        SVG rendering plus file export helpers, driven by a synthetic
 *        KdeEvaluation grid.
 */

#include <gtest/gtest.h>

#include <cstdio>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>

#include "tether/motion_replanner/KdeDerivativeAnalyzer.hpp"
#include "tether/motion_replanner/KdeSvgRenderer.hpp"
#include "tether/motion_replanner/SvgCanvas.hpp"
#include "tether/motion_replanner/SvgExporter.hpp"

namespace {

namespace repl = tether::motion::replanner;
using MotionReplanner::KdeSvgRenderer;
using MotionReplanner::SvgConfig;
using MotionReplanner::SvgCanvas;

/// Synthetic 5x4 density grid with a clear peak at (2, 1).
repl::KdeEvaluation makeEval() {
    repl::KdeEvaluation e;
    e.hasSufficientData = true;
    e.derivatives = {0.1, 0.2, 0.3, 0.4, 0.5};
    e.deviations  = {0.01, 0.02, 0.03, 0.04};
    e.arcLengths  = {0, 10, 20, 30, 40};
    e.times       = {0, 0.1, 0.2, 0.3, 0.4};

    e.grid.xBins = e.derivatives;
    e.grid.yBins = e.deviations;
    e.grid.bandwidthX = 0.05;
    e.grid.bandwidthY = 0.005;
    e.grid.sampleCount = 200;
    const size_t nx = e.grid.xBins.size(), ny = e.grid.yBins.size();
    e.grid.density.assign(nx * ny, 0.0);
    for (size_t ix = 0; ix < nx; ++ix)
        for (size_t iy = 0; iy < ny; ++iy)
            e.grid.density[e.grid.index(ix, iy)] =
                1.0 / (1.0 + std::abs(int(ix) - 2) + std::abs(int(iy) - 1));

    e.derivativeMarginal.mean = 0.3;  e.derivativeMarginal.stdDev = 0.1;
    e.derivativeMarginal.min = 0.1;   e.derivativeMarginal.max = 0.5;
    e.deviationMarginal.mean = 0.025; e.deviationMarginal.stdDev = 0.01;

    repl::ConditionalStats cs;
    cs.xValue = 0.3; cs.meanY = 0.02; cs.stdY = 0.005; cs.medianY = 0.02;
    cs.p05Y = 0.01;  cs.p25Y = 0.015; cs.p75Y = 0.025;  cs.p95Y = 0.03;
    cs.modeY = 0.02; cs.mass = 0.4;
    e.conditional = {cs, cs, cs};
    e.conditional[1].xValue = 0.4;
    e.conditional[2].xValue = 0.5;

    repl::DeviationThreshold th;
    th.derivativeValue = 0.35; th.probability = 0.05; th.tolerance = 0.05;
    th.description = "5% risk of deviation > 0.05mm";
    e.thresholds = {th};

    e.pearsonCorrelation = 0.72;
    e.modeDerivative = 0.3;  e.modeDeviation = 0.02;
    e.maxDensity = 1.0;
    return e;
}

class KdeSvgRendererTest : public ::testing::Test {
protected:
    SvgConfig config_;
    SvgCanvas canvas_{config_};
    KdeSvgRenderer renderer_{config_, canvas_};
    repl::KdeEvaluation eval_ = makeEval();
    std::string tmpdir_ = "/tmp/tether_test_kdesvg_" +
                          std::to_string(getpid());
    void TearDown() override {
        std::filesystem::remove_all(tmpdir_);
    }
};

// ---------------------------------------------------------------------------
// Stream renderers
// ---------------------------------------------------------------------------

TEST_F(KdeSvgRendererTest, RenderHeatmapProducesSvg) {
    // render* emits inner SVG content only — the <svg> wrapper is added
    // by the export* wrappers via canvas_.writeHeader/writeFooter.
    std::ostringstream out;
    renderer_.renderHeatmap(out, 800, 600, eval_);
    const std::string s = out.str();
    EXPECT_NE(s.find("<rect"), std::string::npos);
    EXPECT_GT(s.size(), 500u);
}

TEST_F(KdeSvgRendererTest, RenderConditionalProducesSvg) {
    std::ostringstream out;
    renderer_.renderConditional(out, 800, 600, eval_);
    const std::string s = out.str();
    EXPECT_FALSE(s.empty());
    EXPECT_NE(s.find("<"), std::string::npos);
}

TEST_F(KdeSvgRendererTest, RenderMarginalsProduceSvg) {
    std::ostringstream ox, oy;
    renderer_.renderMarginalX(ox, 800, 200, eval_);
    renderer_.renderMarginalY(oy, 200, 600, eval_);
    EXPECT_FALSE(ox.str().empty());
    EXPECT_FALSE(oy.str().empty());
    EXPECT_NE(ox.str().find("<"), std::string::npos);
    EXPECT_NE(oy.str().find("<"), std::string::npos);
}

TEST_F(KdeSvgRendererTest, RenderersHandleEmptyGrid) {
    repl::KdeEvaluation empty;
    std::ostringstream out;
    renderer_.renderHeatmap(out, 400, 300, empty);
    renderer_.renderConditional(out, 400, 300, empty);
    renderer_.renderMarginalX(out, 400, 200, empty);
    renderer_.renderMarginalY(out, 200, 300, empty);
    SUCCEED();  // must not crash on empty data
}

// ---------------------------------------------------------------------------
// File export helpers
// ---------------------------------------------------------------------------

TEST_F(KdeSvgRendererTest, ExportHeatmapWritesFile) {
    std::filesystem::create_directories(tmpdir_);
    const std::string f = tmpdir_ + "/heatmap.svg";
    EXPECT_TRUE(renderer_.exportHeatmap(f, eval_));
    EXPECT_TRUE(std::filesystem::exists(f));
    EXPECT_GT(std::filesystem::file_size(f), 500u);
    std::ifstream in(f);
    std::stringstream ss;
    ss << in.rdbuf();
    EXPECT_NE(ss.str().find("<svg"), std::string::npos);
    EXPECT_NE(ss.str().find("</svg>"), std::string::npos);
}

TEST_F(KdeSvgRendererTest, ExportConditionalWritesFile) {
    std::filesystem::create_directories(tmpdir_);
    const std::string f = tmpdir_ + "/cond.svg";
    EXPECT_TRUE(renderer_.exportConditional(f, eval_));
    EXPECT_TRUE(std::filesystem::exists(f));
}

TEST_F(KdeSvgRendererTest, ExportDashboardWritesFile) {
    std::filesystem::create_directories(tmpdir_);
    const std::string f = tmpdir_ + "/dash.svg";
    EXPECT_TRUE(renderer_.exportDashboard(f, eval_));
    EXPECT_TRUE(std::filesystem::exists(f));
}

TEST_F(KdeSvgRendererTest, ExportAllReturnsWrittenPaths) {
    std::filesystem::create_directories(tmpdir_);
    auto paths = renderer_.exportAll(tmpdir_, "kde_", eval_);
    EXPECT_FALSE(paths.empty());
    for (const auto& p : paths)
        EXPECT_TRUE(std::filesystem::exists(p)) << p;
}

TEST_F(KdeSvgRendererTest, ExportToBadPathFails) {
    EXPECT_FALSE(renderer_.exportHeatmap("/nonexistent_dir_xyz/f.svg", eval_));
}

// ---------------------------------------------------------------------------
// densityColor mapping
// ---------------------------------------------------------------------------

TEST_F(KdeSvgRendererTest, DensityColorMapsRange) {
    const std::string lo = renderer_.densityColor(0.0);
    const std::string mid = renderer_.densityColor(0.5);
    const std::string hi = renderer_.densityColor(1.0);
    EXPECT_FALSE(lo.empty());
    EXPECT_NE(lo, hi);
    // Out-of-range values clamp rather than crash.
    EXPECT_FALSE(renderer_.densityColor(-1.0).empty());
    EXPECT_FALSE(renderer_.densityColor(2.0).empty());
}

}  // namespace
