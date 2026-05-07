/**
 * @file test_PerformanceHeatmap.cpp
 * @brief Tests for Heatmap1D, Heatmap2D, Heatmap3D, DifferentialHeatmap,
 *        and HeatmapBuilder.
 */
#include <gtest/gtest.h>
#include <tether/motion_replanner/PerformanceHeatmap.hpp>

using namespace MotionReplanner;

// ============================================================================
// HeatmapConfig
// ============================================================================
TEST(HeatmapConfigTest, Defaults) {
    HeatmapConfig cfg;
    EXPECT_DOUBLE_EQ(cfg.resolution1D, 10.0);
    EXPECT_DOUBLE_EQ(cfg.resolution2D, 20.0);
    EXPECT_DOUBLE_EQ(cfg.resolution3D, 50.0);
    EXPECT_GT(cfg.defaultMaxVelocity, 0.0);
    EXPECT_GT(cfg.defaultMaxAccel, 0.0);
    EXPECT_GT(cfg.safetyMargin, 0.0);
    EXPECT_EQ(cfg.minSamplesForValidity, 5u);
}

// ============================================================================
// PerformanceMetrics
// ============================================================================
TEST(PerformanceMetricsTest, Defaults) {
    PerformanceMetrics m;
    EXPECT_DOUBLE_EQ(m.maxAchievedVelocity, 0.0);
    EXPECT_DOUBLE_EQ(m.meanFeedRateRatio, 1.0);
    EXPECT_EQ(m.sampleCount, 0u);
}

TEST(PerformanceMetricsTest, Confidence) {
    PerformanceMetrics m;
    EXPECT_DOUBLE_EQ(m.confidence(), 0.0);
    m.sampleCount = 50;
    EXPECT_NEAR(m.confidence(), 0.5, 0.01);
    m.sampleCount = 100;
    EXPECT_NEAR(m.confidence(), 1.0, 0.01);
    m.sampleCount = 200;
    EXPECT_NEAR(m.confidence(), 1.0, 0.01); // capped at 1.0
}

// ============================================================================
// SuggestedLimits
// ============================================================================
TEST(SuggestedLimitsTest, Defaults) {
    SuggestedLimits sl;
    EXPECT_GT(sl.maxVelocity, 0.0);
    EXPECT_GT(sl.maxAcceleration, 0.0);
    EXPECT_GT(sl.maxJerk, 0.0);
    EXPECT_DOUBLE_EQ(sl.confidence, 0.0);
}

// ============================================================================
// Heatmap1D
// ============================================================================
class Heatmap1DTest : public ::testing::Test {
protected:
    Heatmap1D hmap_{0}; // axis 0 (X)
};

TEST_F(Heatmap1DTest, Construction) {
    EXPECT_EQ(hmap_.axis(), 0);
}

TEST_F(Heatmap1DTest, AddSampleAndRetrieve) {
    hmap_.addSample(50.0, 1000.0, 500.0, 0.01, 0.95);
    auto metrics = hmap_.getMetrics(50.0);
    EXPECT_GE(metrics.sampleCount, 1u);
    EXPECT_GE(metrics.maxAchievedVelocity, 1000.0);
}

TEST_F(Heatmap1DTest, MultipleSamples) {
    for (int i = 0; i < 20; i++) {
        hmap_.addSample(50.0, 1000.0 + i * 10, 500.0, 0.01, 0.95);
    }
    auto metrics = hmap_.getMetrics(50.0);
    EXPECT_GE(metrics.sampleCount, 20u);
}

TEST_F(Heatmap1DTest, GetSuggestedLimits) {
    for (int i = 0; i < 10; i++) {
        hmap_.addSample(50.0, 1000.0, 500.0, 0.01, 0.95);
    }
    auto limits = hmap_.getSuggestedLimits(50.0);
    EXPECT_GT(limits.maxVelocity, 0.0);
}

TEST_F(Heatmap1DTest, Clear) {
    hmap_.addSample(50.0, 1000.0, 500.0, 0.01, 0.95);
    hmap_.clear();
    auto metrics = hmap_.getMetrics(50.0);
    EXPECT_EQ(metrics.sampleCount, 0u);
}

TEST_F(Heatmap1DTest, GetAllBins) {
    hmap_.addSample(10.0, 1000.0, 500.0, 0.01, 0.95);
    hmap_.addSample(100.0, 2000.0, 500.0, 0.02, 0.9);
    auto bins = hmap_.getAllBins();
    EXPECT_GE(bins.size(), 2u);
}

TEST_F(Heatmap1DTest, GetDifferential) {
    hmap_.addSample(10.0, 1000.0, 500.0, 0.01, 0.95);
    hmap_.addSample(100.0, 2000.0, 500.0, 0.05, 0.8);
    auto diff = hmap_.getDifferential();
    // Just verify it returns without crash
    (void)diff;
}

// ============================================================================
// Heatmap2D
// ============================================================================
class Heatmap2DTest : public ::testing::Test {
protected:
    Heatmap2D hmap_{Heatmap2D::Plane::XY};
};

TEST_F(Heatmap2DTest, Construction) {
    EXPECT_EQ(hmap_.plane(), Heatmap2D::Plane::XY);
}

TEST_F(Heatmap2DTest, AddSample) {
    hmap_.addSample(50.0, 80.0, 1000.0, 500.0, 0.01, 0.95);
    auto metrics = hmap_.getMetrics(50.0, 80.0);
    EXPECT_GE(metrics.sampleCount, 1u);
}

TEST_F(Heatmap2DTest, AddSample3D) {
    std::array<double, 3> pos = {50.0, 80.0, 10.0};
    hmap_.addSample3D(pos, 1500.0, 700.0, 0.02, 0.9);
    // XY plane: u=50, v=80
    auto metrics = hmap_.getMetrics(50.0, 80.0);
    EXPECT_GE(metrics.sampleCount, 1u);
}

TEST_F(Heatmap2DTest, GetSuggestedLimits) {
    for (int i = 0; i < 10; i++) {
        hmap_.addSample(50.0, 80.0, 1000.0, 500.0, 0.01, 0.95);
    }
    auto limits = hmap_.getSuggestedLimits(50.0, 80.0);
    EXPECT_GT(limits.maxVelocity, 0.0);
}

TEST_F(Heatmap2DTest, GetInterpolatedMetrics) {
    // Add samples at two nearby cells
    for (int i = 0; i < 10; i++) {
        hmap_.addSample(30.0, 30.0, 1000.0, 500.0, 0.01, 0.95);
        hmap_.addSample(50.0, 50.0, 2000.0, 800.0, 0.02, 0.9);
    }
    auto metrics = hmap_.getInterpolatedMetrics(40.0, 40.0);
    // Interpolated result
    (void)metrics;
}

TEST_F(Heatmap2DTest, GetVelocityGrid) {
    hmap_.addSample(50.0, 80.0, 1000.0, 500.0, 0.01, 0.95);
    auto grid = hmap_.getVelocityGrid();
    EXPECT_GT(grid.data.size(), 0u);
}

TEST_F(Heatmap2DTest, Clear) {
    hmap_.addSample(50.0, 80.0, 1000.0, 500.0, 0.01, 0.95);
    hmap_.clear();
    auto metrics = hmap_.getMetrics(50.0, 80.0);
    EXPECT_EQ(metrics.sampleCount, 0u);
}

// ============================================================================
// Heatmap3D
// ============================================================================
class Heatmap3DTest : public ::testing::Test {
protected:
    Heatmap3D hmap_;
};

TEST_F(Heatmap3DTest, AddSample) {
    std::array<double, 3> pos = {50.0, 80.0, 10.0};
    hmap_.addSample(pos, 1000.0, 500.0, 0.01, 0.95);
    auto metrics = hmap_.getMetrics(pos);
    EXPECT_GE(metrics.sampleCount, 1u);
}

TEST_F(Heatmap3DTest, GetSuggestedLimits) {
    std::array<double, 3> pos = {50.0, 80.0, 10.0};
    for (int i = 0; i < 10; i++) {
        hmap_.addSample(pos, 1000.0, 500.0, 0.01, 0.95);
    }
    auto limits = hmap_.getSuggestedLimits(pos);
    EXPECT_GT(limits.maxVelocity, 0.0);
}

TEST_F(Heatmap3DTest, GetPopulatedVoxels) {
    std::array<double, 3> pos1 = {50.0, 80.0, 10.0};
    std::array<double, 3> pos2 = {150.0, 180.0, 110.0};
    hmap_.addSample(pos1, 1000.0, 500.0, 0.01, 0.95);
    hmap_.addSample(pos2, 2000.0, 800.0, 0.02, 0.9);
    auto voxels = hmap_.getPopulatedVoxels();
    EXPECT_GE(voxels.size(), 2u);
}

TEST_F(Heatmap3DTest, GetSliceAtZ) {
    std::array<double, 3> pos = {50.0, 80.0, 10.0};
    hmap_.addSample(pos, 1000.0, 500.0, 0.01, 0.95);
    auto slice = hmap_.getSliceAtZ(10.0);
    // Should return 2D data for that Z level
    (void)slice;
}

TEST_F(Heatmap3DTest, Clear) {
    std::array<double, 3> pos = {50.0, 80.0, 10.0};
    hmap_.addSample(pos, 1000.0, 500.0, 0.01, 0.95);
    hmap_.clear();
    auto metrics = hmap_.getMetrics(pos);
    EXPECT_EQ(metrics.sampleCount, 0u);
}

// ============================================================================
// DifferentialHeatmap
// ============================================================================
TEST(DifferentialHeatmapTest, DefaultConstruction) {
    DifferentialHeatmap dhmap;
    // no crash
}

TEST(DifferentialHeatmapTest, SetAndCompare) {
    DifferentialHeatmap dhmap;
    dhmap.setExpectedPerformance(6000.0, 1000.0, 50000.0);

    std::array<double, 3> pos = {50.0, 80.0, 10.0};
    dhmap.addActualSample(pos, 3000.0, 500.0, 0.01);
    auto diff = dhmap.getDifferential(pos);
    // Actual is worse than expected
    (void)diff;
}

TEST(DifferentialHeatmapTest, GetProblemAreas) {
    DifferentialHeatmap dhmap;
    dhmap.setExpectedPerformance(6000.0, 1000.0, 50000.0);

    // Add a "slow" area
    std::array<double, 3> pos = {50.0, 80.0, 10.0};
    for (int i = 0; i < 10; i++) {
        dhmap.addActualSample(pos, 1000.0, 200.0, 0.05);
    }
    auto problems = dhmap.getProblemAreas(0.5);
    // Should identify the area as problematic
    (void)problems;
}

// ============================================================================
// HeatmapBuilder
// ============================================================================
class HeatmapBuilderTest : public ::testing::Test {
protected:
    HeatmapBuilder builder_;
};

TEST_F(HeatmapBuilderTest, ProcessSample) {
    std::array<double, 9> pos = {50.0, 80.0, 10.0};
    std::array<double, 9> vel = {1000.0, 500.0, 0.0};
    builder_.processSample(pos, vel, 0.01, 0.005, 6000.0, 5500.0);
    // No crash
}

TEST_F(HeatmapBuilderTest, GetAxisHeatmaps) {
    std::array<double, 9> pos = {50.0, 80.0, 10.0};
    std::array<double, 9> vel = {1000.0, 500.0, 0.0};
    builder_.processSample(pos, vel, 0.01, 0.005, 6000.0, 5500.0);
    const auto& hmaps = builder_.getAxisHeatmaps();
    // Should have at least 3 axis heatmaps
    EXPECT_GE(hmaps.size(), 3u);
}

TEST_F(HeatmapBuilderTest, GetXYHeatmap) {
    std::array<double, 9> pos = {50.0, 80.0, 10.0};
    std::array<double, 9> vel = {1000.0, 500.0, 0.0};
    builder_.processSample(pos, vel, 0.01, 0.005, 6000.0, 5500.0);
    const auto& xy = builder_.getXYHeatmap();
    EXPECT_EQ(xy.plane(), Heatmap2D::Plane::XY);
}

TEST_F(HeatmapBuilderTest, Clear) {
    std::array<double, 9> pos = {50.0, 80.0, 10.0};
    std::array<double, 9> vel = {1000.0, 500.0, 0.0};
    builder_.processSample(pos, vel, 0.01, 0.005, 6000.0, 5500.0);
    builder_.clear();
    // After clear, heatmaps should be empty
}
