/**
 * @file test_heatmap.cpp
 * @brief Unit tests for PerformanceHeatmap classes
 */

#include <gtest/gtest.h>
#include "PerformanceHeatmap.hpp"
#include <cmath>

using namespace MotionReplanner;

class Heatmap1DTest : public ::testing::Test {
protected:
    void SetUp() override {
        config.xBins = 10;
        config.minSamplesForValid = 1;
    }
    
    HeatmapConfig config;
};

TEST_F(Heatmap1DTest, BasicBinning) {
    Heatmap1D heatmap(Axis::X, 0.0, 100.0, config);
    
    PerformanceMetrics pm;
    pm.position = {25.0, 0.0, 0.0};
    pm.trackingError = 0.01;
    pm.achievedVelocity = 50.0;
    pm.commandedVelocity = 50.0;
    
    heatmap.addSample(pm);
    
    const auto& data = heatmap.getData();
    EXPECT_EQ(data.size(), 10);
    
    // Sample at 25.0 should be in bin 2 (0-10, 10-20, 20-30...)
    EXPECT_EQ(data[2].sampleCount, 1);
    EXPECT_NEAR(data[2].avgTrackingError, 0.01, 1e-6);
}

TEST_F(Heatmap1DTest, MultipleSamplesInBin) {
    Heatmap1D heatmap(Axis::X, 0.0, 100.0, config);
    
    // Add multiple samples to same bin
    for (int i = 0; i < 5; ++i) {
        PerformanceMetrics pm;
        pm.position = {25.0 + i * 0.1, 0.0, 0.0};  // All in bin 2
        pm.trackingError = 0.01 * (i + 1);
        pm.achievedVelocity = 50.0;
        pm.commandedVelocity = 50.0;
        
        heatmap.addSample(pm);
    }
    
    const auto& data = heatmap.getData();
    EXPECT_EQ(data[2].sampleCount, 5);
    EXPECT_NEAR(data[2].avgTrackingError, 0.03, 1e-6);  // Mean of 0.01, 0.02, 0.03, 0.04, 0.05
}

TEST_F(Heatmap1DTest, OutOfBoundsHandling) {
    Heatmap1D heatmap(Axis::X, 0.0, 100.0, config);
    
    // Add sample outside bounds
    PerformanceMetrics pm;
    pm.position = {150.0, 0.0, 0.0};  // Beyond max
    pm.trackingError = 0.01;
    pm.achievedVelocity = 50.0;
    pm.commandedVelocity = 50.0;
    
    heatmap.addSample(pm);
    
    // Should be clamped to last bin or ignored (implementation dependent)
    const auto& data = heatmap.getData();
    
    // Check total sample count
    size_t totalSamples = 0;
    for (const auto& cell : data) {
        totalSamples += cell.sampleCount;
    }
    // Either 0 (ignored) or 1 (clamped)
    EXPECT_LE(totalSamples, 1);
}

class Heatmap2DTest : public ::testing::Test {
protected:
    void SetUp() override {
        config.xBins = 10;
        config.yBins = 10;
        config.minSamplesForValid = 1;
    }
    
    HeatmapConfig config;
};

TEST_F(Heatmap2DTest, BasicBinning) {
    Heatmap2D heatmap(Plane::XY, 0.0, 100.0, 0.0, 100.0, config);
    
    PerformanceMetrics pm;
    pm.position = {25.0, 35.0, 0.0};
    pm.trackingError = 0.01;
    pm.achievedVelocity = 50.0;
    pm.commandedVelocity = 50.0;
    
    heatmap.addSample(pm);
    
    const auto& data = heatmap.getData();
    EXPECT_EQ(data.size(), 10);
    EXPECT_EQ(data[0].size(), 10);
    
    // Sample at (25, 35) should be in cell (2, 3)
    EXPECT_EQ(data[2][3].sampleCount, 1);
}

TEST_F(Heatmap2DTest, DifferentPlanes) {
    // XZ plane
    Heatmap2D heatmapXZ(Plane::XZ, 0.0, 100.0, 0.0, 50.0, config);
    
    PerformanceMetrics pm;
    pm.position = {25.0, 999.0, 15.0};  // Y is ignored for XZ plane
    pm.trackingError = 0.01;
    pm.achievedVelocity = 50.0;
    pm.commandedVelocity = 50.0;
    
    heatmapXZ.addSample(pm);
    
    const auto& data = heatmapXZ.getData();
    
    // X bin: 25/10 = 2, Z bin: 15/5 = 3
    EXPECT_EQ(data[2][3].sampleCount, 1);
}

TEST_F(Heatmap2DTest, SuggestedLimits) {
    Heatmap2D heatmap(Plane::XY, 0.0, 100.0, 0.0, 100.0, config);
    
    // Add samples with varying performance
    for (int x = 0; x < 100; x += 10) {
        for (int y = 0; y < 100; y += 10) {
            PerformanceMetrics pm;
            pm.position = {static_cast<double>(x) + 5, static_cast<double>(y) + 5, 0.0};
            
            // Simulate worse performance at edges
            double distFromCenter = std::sqrt(std::pow(x - 50, 2) + std::pow(y - 50, 2));
            pm.achievedVelocity = 100.0 - distFromCenter * 0.5;
            pm.commandedVelocity = 100.0;
            pm.trackingError = 0.01 + distFromCenter * 0.0005;
            
            heatmap.addSample(pm);
        }
    }
    
    auto suggestions = heatmap.getSuggestedLimits();
    
    // Should have lower suggested velocity for edge regions
    // Center region should allow higher velocity
    EXPECT_GT(suggestions.maxSuggestedVelocity, 0.0);
    EXPECT_LE(suggestions.maxSuggestedVelocity, 100.0);
}

class Heatmap3DTest : public ::testing::Test {
protected:
    void SetUp() override {
        config.xBins = 5;
        config.yBins = 5;
        config.zBins = 5;
        config.minSamplesForValid = 1;
    }
    
    HeatmapConfig config;
};

TEST_F(Heatmap3DTest, SparseStorage) {
    Heatmap3D heatmap(0.0, 100.0, 0.0, 100.0, 0.0, 100.0, config);
    
    // Add only a few samples
    PerformanceMetrics pm;
    pm.position = {25.0, 35.0, 45.0};
    pm.trackingError = 0.01;
    pm.achievedVelocity = 50.0;
    pm.commandedVelocity = 50.0;
    
    heatmap.addSample(pm);
    
    const auto& data = heatmap.getData();
    
    // Sparse storage: only filled voxels are stored
    EXPECT_EQ(data.size(), 1);
}

TEST_F(Heatmap3DTest, MultipleVoxels) {
    Heatmap3D heatmap(0.0, 100.0, 0.0, 100.0, 0.0, 100.0, config);
    
    // Add samples to different voxels
    std::vector<std::array<double, 3>> positions = {
        {10.0, 10.0, 10.0},
        {50.0, 50.0, 50.0},
        {90.0, 90.0, 90.0},
    };
    
    for (const auto& pos : positions) {
        PerformanceMetrics pm;
        pm.position = pos;
        pm.trackingError = 0.01;
        pm.achievedVelocity = 50.0;
        pm.commandedVelocity = 50.0;
        
        heatmap.addSample(pm);
    }
    
    const auto& data = heatmap.getData();
    EXPECT_EQ(data.size(), 3);
}

class DifferentialHeatmapTest : public ::testing::Test {
protected:
    void SetUp() override {
        config.xBins = 5;
        config.yBins = 5;
        config.minSamplesForValid = 1;
    }
    
    HeatmapConfig config;
};

TEST_F(DifferentialHeatmapTest, IdenticalMaps) {
    Heatmap2D expected(Plane::XY, 0.0, 100.0, 0.0, 100.0, config);
    Heatmap2D actual(Plane::XY, 0.0, 100.0, 0.0, 100.0, config);
    
    // Add identical samples to both
    for (int x = 0; x < 5; ++x) {
        for (int y = 0; y < 5; ++y) {
            PerformanceMetrics pm;
            pm.position = {x * 20.0 + 10.0, y * 20.0 + 10.0, 0.0};
            pm.trackingError = 0.01;
            pm.achievedVelocity = 50.0;
            pm.commandedVelocity = 50.0;
            
            expected.addSample(pm);
            actual.addSample(pm);
        }
    }
    
    DifferentialHeatmap diff(expected, actual);
    
    const auto& data = diff.getData();
    
    // All ratios should be 1.0 (identical)
    for (const auto& row : data) {
        for (const auto& cell : row) {
            if (cell.expectedValid && cell.actualValid) {
                EXPECT_NEAR(cell.velocityRatio, 1.0, 1e-6);
                EXPECT_NEAR(cell.errorDifference, 0.0, 1e-6);
            }
        }
    }
}

TEST_F(DifferentialHeatmapTest, DegradedPerformance) {
    Heatmap2D expected(Plane::XY, 0.0, 100.0, 0.0, 100.0, config);
    Heatmap2D actual(Plane::XY, 0.0, 100.0, 0.0, 100.0, config);
    
    // Expected: good performance
    PerformanceMetrics pmExpected;
    pmExpected.position = {50.0, 50.0, 0.0};
    pmExpected.trackingError = 0.01;
    pmExpected.achievedVelocity = 100.0;
    pmExpected.commandedVelocity = 100.0;
    expected.addSample(pmExpected);
    
    // Actual: degraded performance
    PerformanceMetrics pmActual;
    pmActual.position = {50.0, 50.0, 0.0};
    pmActual.trackingError = 0.05;
    pmActual.achievedVelocity = 80.0;
    pmActual.commandedVelocity = 100.0;
    actual.addSample(pmActual);
    
    DifferentialHeatmap diff(expected, actual);
    
    const auto& data = diff.getData();
    
    // Check the cell containing (50, 50)
    int xBin = 2;  // 50 / 20 = 2
    int yBin = 2;
    
    EXPECT_NEAR(data[xBin][yBin].velocityRatio, 0.8, 1e-6);  // 80/100
    EXPECT_NEAR(data[xBin][yBin].errorDifference, 0.04, 1e-6);  // 0.05 - 0.01
}

// Test HeatmapBuilder
TEST(HeatmapBuilderTest, BuildAll) {
    HeatmapConfig config;
    config.xBins = 5;
    config.yBins = 5;
    config.zBins = 5;
    
    std::vector<PerformanceMetrics> data;
    
    // Generate test data
    for (int i = 0; i < 100; ++i) {
        PerformanceMetrics pm;
        pm.position = {
            static_cast<double>(i % 10) * 10.0,
            static_cast<double>((i / 10) % 10) * 10.0,
            static_cast<double>(i / 100) * 10.0
        };
        pm.trackingError = 0.01;
        pm.achievedVelocity = 50.0;
        pm.commandedVelocity = 50.0;
        
        data.push_back(pm);
    }
    
    HeatmapBuilder builder(config);
    builder.setWorkspaceBounds(0.0, 100.0, 0.0, 100.0, 0.0, 100.0);
    
    for (const auto& pm : data) {
        builder.addSample(pm);
    }
    
    auto heatmap1D_X = builder.buildHeatmap1D(Axis::X);
    auto heatmap2D_XY = builder.buildHeatmap2D(Plane::XY);
    auto heatmap3D = builder.buildHeatmap3D();
    
    EXPECT_EQ(heatmap1D_X.getData().size(), 5);
    EXPECT_EQ(heatmap2D_XY.getData().size(), 5);
    EXPECT_GT(heatmap3D.getData().size(), 0);
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
