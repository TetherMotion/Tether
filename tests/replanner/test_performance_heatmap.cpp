#include <gtest/gtest.h>

#include "tether/motion_replanner/PerformanceHeatmap.hpp"

#include <array>
#include <cmath>
#include <limits>

using MotionReplanner::DifferentialHeatmap;
using MotionReplanner::Heatmap1D;
using MotionReplanner::Heatmap2D;
using MotionReplanner::Heatmap3D;
using MotionReplanner::HeatmapBuilder;
using MotionReplanner::HeatmapConfig;
using MotionReplanner::PerformanceMetrics;
using MotionReplanner::SuggestedLimits;

namespace {

HeatmapConfig makeSmallConfig() {
    HeatmapConfig cfg;
    cfg.minBounds = {0.0, 0.0, 0.0};
    cfg.maxBounds = {2.0, 2.0, 2.0};
    cfg.resolution1D = 1.0;
    cfg.resolution2D = 1.0;
    cfg.resolution3D = 1.0;
    cfg.defaultMaxVelocity = 100.0;
    cfg.defaultMaxAccel = 10.0;
    cfg.defaultMaxJerk = 1000.0;
    cfg.safetyMargin = 0.1;
    cfg.minSamplesForValidity = 5;
    cfg.expectedVelocity = 50.0;
    cfg.expectedAccel = 5.0;
    return cfg;
}

} // namespace

TEST(PerformanceHeatmap, Heatmap1DEmptyBinsAreHandled) {
    HeatmapConfig cfg = makeSmallConfig();
    cfg.minBounds = {0.0, 0.0, 0.0};
    cfg.maxBounds = {0.0, 0.0, 0.0}; // range=0 => 0 bins
    Heatmap1D hm(0, cfg);

    hm.addSample(0.0, 10.0, 1.0, 0.0, 1.0);
    auto metrics = hm.getMetrics(0.0);
    EXPECT_EQ(metrics.sampleCount, 0u);

    auto suggested = hm.getSuggestedLimits(0.0);
    EXPECT_DOUBLE_EQ(suggested.maxVelocity, cfg.defaultMaxVelocity);
    EXPECT_DOUBLE_EQ(suggested.maxAcceleration, cfg.defaultMaxAccel);
    EXPECT_DOUBLE_EQ(suggested.maxJerk, cfg.defaultMaxJerk);
}

TEST(PerformanceHeatmap, Heatmap1DSuggestedLimitsInsufficientData) {
    HeatmapConfig cfg = makeSmallConfig();
    Heatmap1D hm(0, cfg);

    for (size_t i = 0; i < cfg.minSamplesForValidity - 1; ++i) {
        hm.addSample(0.5, 20.0, 2.0, 0.0, 1.0);
    }

    auto suggested = hm.getSuggestedLimits(0.5);
    EXPECT_DOUBLE_EQ(suggested.maxVelocity, cfg.defaultMaxVelocity);
    EXPECT_DOUBLE_EQ(suggested.maxAcceleration, cfg.defaultMaxAccel);
    EXPECT_DOUBLE_EQ(suggested.maxJerk, cfg.defaultMaxJerk);
    EXPECT_DOUBLE_EQ(suggested.confidence, 0.0);
    EXPECT_EQ(suggested.limitingFactor, "Insufficient data");
}

TEST(PerformanceHeatmap, Heatmap1DSuggestedLimitsTrackingErrorFactorAndClamp) {
    HeatmapConfig cfg = makeSmallConfig();
    cfg.safetyMargin = 0.1; // safetyFactor=0.9
    Heatmap1D hm(0, cfg);

    // Enough samples to be valid; ensure maxAchievedVelocity is 80.
    for (size_t i = 0; i < cfg.minSamplesForValidity; ++i) {
        hm.addSample(0.5, 80.0, 8.0, 0.02, 1.0); // meanTrackingError=0.02 => errorFactor=0.5
    }

    auto suggested = hm.getSuggestedLimits(0.5);
    EXPECT_EQ(suggested.limitingFactor, "Tracking error");
    EXPECT_NEAR(suggested.maxVelocity, 80.0 * 0.9 * 0.5, 1e-9);
    EXPECT_NEAR(suggested.maxAcceleration, 8.0 * 0.9 * 0.5, 1e-9);
    EXPECT_DOUBLE_EQ(suggested.maxJerk, cfg.defaultMaxJerk);
}

TEST(PerformanceHeatmap, Heatmap1DSuggestedLimitsAchievedPerformancePath) {
    HeatmapConfig cfg = makeSmallConfig();
    Heatmap1D hm(0, cfg);

    for (size_t i = 0; i < cfg.minSamplesForValidity; ++i) {
        hm.addSample(0.5, 70.0, 7.0, 0.0, 1.0);
    }

    auto suggested = hm.getSuggestedLimits(0.5);
    EXPECT_EQ(suggested.limitingFactor, "Achieved performance");
    EXPECT_GT(suggested.confidence, 0.0);
}

TEST(PerformanceHeatmap, Heatmap1DPositionClampsToFirstAndLastBin) {
    HeatmapConfig cfg = makeSmallConfig();
    cfg.minBounds = {0.0, 0.0, 0.0};
    cfg.maxBounds = {2.0, 2.0, 2.0};
    cfg.resolution1D = 1.0; // bins at [0..1], [1..2]
    Heatmap1D hm(0, cfg);

    // Put a distinct sample in the first bin.
    for (size_t i = 0; i < cfg.minSamplesForValidity; ++i) {
        hm.addSample(0.25, 10.0, 1.0, 0.0, 1.0);
    }
    // Put a distinct sample in the last bin.
    for (size_t i = 0; i < cfg.minSamplesForValidity; ++i) {
        hm.addSample(1.75, 99.0, 9.0, 0.0, 1.0);
    }

    auto first = hm.getMetrics(-100.0);
    auto last = hm.getMetrics(1e9);
    EXPECT_GE(first.maxAchievedVelocity, 10.0);
    EXPECT_GE(last.maxAchievedVelocity, 99.0);
}

TEST(PerformanceHeatmap, Heatmap1DGetAllBinsReportsBinsAndSuggestedLimits) {
    HeatmapConfig cfg = makeSmallConfig();
    cfg.minSamplesForValidity = 1;
    Heatmap1D hm(0, cfg);

    // Add one sample in each bin (range 0..2, resolution 1 => 2 bins).
    hm.addSample(0.25, 10.0, 1.0, 0.0, 1.0);
    hm.addSample(1.75, 20.0, 2.0, 0.0, 1.0);

    auto bins = hm.getAllBins();
    ASSERT_EQ(bins.size(), 2u);
    EXPECT_DOUBLE_EQ(bins[0].width, cfg.resolution1D);
    EXPECT_DOUBLE_EQ(bins[1].width, cfg.resolution1D);
    EXPECT_GT(bins[0].suggested.maxVelocity, 0.0);
    EXPECT_GT(bins[1].suggested.maxVelocity, 0.0);
}

TEST(PerformanceHeatmap, Heatmap1DDifferentialOnlyReturnsValidBins) {
    HeatmapConfig cfg = makeSmallConfig();
    Heatmap1D hm(0, cfg);

    // Populate only one bin with enough samples.
    for (size_t i = 0; i < cfg.minSamplesForValidity; ++i) {
        hm.addSample(0.5, 60.0, 6.0, 0.0, 1.0);
    }

    auto diffs = hm.getDifferential();
    ASSERT_EQ(diffs.size(), 1u);
    EXPECT_NEAR(diffs[0].velocityDiff, 60.0 - cfg.expectedVelocity, 1e-9);
    EXPECT_NEAR(diffs[0].accelDiff, 6.0 - cfg.expectedAccel, 1e-9);
    EXPECT_NEAR(diffs[0].performanceRatio, 60.0 / cfg.expectedVelocity, 1e-9);
}

TEST(PerformanceHeatmap, Heatmap1DDifferentialExpectedVelocityZeroFallsBackToOne) {
    HeatmapConfig cfg = makeSmallConfig();
    cfg.expectedVelocity = 0.0;
    Heatmap1D hm(0, cfg);

    for (size_t i = 0; i < cfg.minSamplesForValidity; ++i) {
        hm.addSample(0.5, 12.0, 1.0, 0.0, 1.0);
    }

    auto diffs = hm.getDifferential();
    ASSERT_EQ(diffs.size(), 1u);
    EXPECT_DOUBLE_EQ(diffs[0].performanceRatio, 1.0);
}

TEST(PerformanceHeatmap, Heatmap1DClearResetsBins) {
    HeatmapConfig cfg = makeSmallConfig();
    Heatmap1D hm(0, cfg);
    for (size_t i = 0; i < cfg.minSamplesForValidity; ++i) {
        hm.addSample(0.5, 60.0, 6.0, 0.0, 1.0);
    }
    hm.clear();
    auto metrics = hm.getMetrics(0.5);
    EXPECT_EQ(metrics.sampleCount, 0u);
    EXPECT_DOUBLE_EQ(metrics.maxAchievedVelocity, 0.0);
}

TEST(PerformanceHeatmap, Heatmap2DInterpolationAndGridsAndClear) {
    HeatmapConfig cfg = makeSmallConfig();
    Heatmap2D hm(Heatmap2D::Plane::XY, cfg);

    // 2x2 grid with centers at 0.5 and 1.5.
    const std::array<std::pair<double, double>, 4> centers = {
        std::make_pair(0.5, 0.5),
        std::make_pair(1.5, 0.5),
        std::make_pair(0.5, 1.5),
        std::make_pair(1.5, 1.5),
    };
    const std::array<double, 4> vels = {10.0, 20.0, 30.0, 40.0};
    for (size_t i = 0; i < centers.size(); ++i) {
        for (size_t n = 0; n < cfg.minSamplesForValidity; ++n) {
            hm.addSample(centers[i].first, centers[i].second, vels[i], 2.0, 0.0, 1.0);
        }
    }

    // Use half-cell coordinates (0.5) to exercise blending across 4 neighbors.
    auto interp = hm.getInterpolatedMetrics(0.5, 0.5);
    EXPECT_NEAR(interp.maxAchievedVelocity, 25.0, 1e-9);
    EXPECT_NEAR(interp.maxAchievedAccel, 2.0, 1e-9);

    auto gridV = hm.getVelocityGrid();
    EXPECT_EQ(gridV.rows * gridV.cols, gridV.data.size());
    EXPECT_EQ(gridV.rows, 2u);
    EXPECT_EQ(gridV.cols, 2u);

    auto gridA = hm.getAccelGrid();
    EXPECT_EQ(gridA.data.size(), 4u);

    auto gridE = hm.getErrorGrid();
    EXPECT_EQ(gridE.data.size(), 4u);

    auto cells = hm.getAllCells();
    EXPECT_EQ(cells.size(), 4u);
    EXPECT_DOUBLE_EQ(cells[0].width, cfg.resolution2D);
    EXPECT_DOUBLE_EQ(cells[0].height, cfg.resolution2D);

    hm.clear();
    auto after = hm.getMetrics(0.5, 0.5);
    EXPECT_EQ(after.sampleCount, 0u);
}

TEST(PerformanceHeatmap, Heatmap2DPlaneAxesAnd3DProjection) {
    HeatmapConfig cfg = makeSmallConfig();
    Heatmap2D xz(Heatmap2D::Plane::XZ, cfg);
    std::array<double, 3> pos = {1.5, 999.0, 0.5};
    for (size_t n = 0; n < cfg.minSamplesForValidity; ++n) {
        xz.addSample3D(pos, 12.0, 3.0, 0.0, 1.0);
    }
    auto m = xz.getMetrics(pos[0], pos[2]);
    EXPECT_GE(m.maxAchievedVelocity, 12.0);
}

TEST(PerformanceHeatmap, Heatmap2DInvalidPlaneEnumUsesDefaultAxes) {
    HeatmapConfig cfg = makeSmallConfig();
    Heatmap2D hm(static_cast<Heatmap2D::Plane>(99), cfg);
    auto grid = hm.getVelocityGrid();
    EXPECT_EQ(grid.rows, 2u);
    EXPECT_EQ(grid.cols, 2u);
}

TEST(PerformanceHeatmap, Heatmap2DEmptyGridReturnsDefaultsAndEmptyResults) {
    HeatmapConfig cfg = makeSmallConfig();
    cfg.minBounds = {0.0, 0.0, 0.0};
    cfg.maxBounds = {0.0, 0.0, 0.0};
    Heatmap2D hm(Heatmap2D::Plane::XY, cfg);

    auto m = hm.getMetrics(0.0, 0.0);
    EXPECT_EQ(m.sampleCount, 0u);

    auto s = hm.getSuggestedLimits(0.0, 0.0);
    EXPECT_DOUBLE_EQ(s.maxVelocity, cfg.defaultMaxVelocity);
    EXPECT_DOUBLE_EQ(s.maxAcceleration, cfg.defaultMaxAccel);

    auto interp = hm.getInterpolatedMetrics(0.0, 0.0);
    EXPECT_EQ(interp.sampleCount, 0u);

    EXPECT_TRUE(hm.getAllCells().empty());
}

TEST(PerformanceHeatmap, Heatmap3DInterpolationVolumesSlicesAndClear) {
    HeatmapConfig cfg = makeSmallConfig();
    Heatmap3D hm(cfg);

    // Populate all 8 voxels (2x2x2). Centers are at 0.5 and 1.5.
    const std::array<double, 2> c = {0.5, 1.5};
    double vel = 10.0;
    for (double z : c) {
        for (double y : c) {
            for (double x : c) {
                std::array<double, 3> p = {x, y, z};
                for (size_t n = 0; n < cfg.minSamplesForValidity; ++n) {
                    hm.addSample(p, vel, 2.0, 0.0, 1.0);
                }
                vel += 10.0;
            }
        }
    }

    // Use half-voxel coordinates (0.5) to exercise blending across 8 neighbors.
    auto interp = hm.getInterpolatedMetrics({0.5, 0.5, 0.5});
    // Average of 10..80 step 10 => (10+80)/2 = 45.
    EXPECT_NEAR(interp.maxAchievedVelocity, 45.0, 1e-9);
    EXPECT_NEAR(interp.maxAchievedAccel, 2.0, 1e-9);

    auto all = hm.getAllVoxels();
    EXPECT_EQ(all.size(), 8u);
    auto populated = hm.getPopulatedVoxels();
    EXPECT_EQ(populated.size(), 8u);

    auto slice = hm.getSliceAtZ(0.1);
    auto sliceCells = slice.getAllCells();
    EXPECT_EQ(sliceCells.size(), 4u);

    auto volV = hm.getVelocityVolume();
    EXPECT_EQ(volV.data.size(), volV.nx * volV.ny * volV.nz);
    EXPECT_EQ(volV.data.size(), 8u);

    auto volE = hm.getErrorVolume();
    EXPECT_EQ(volE.data.size(), 8u);

    hm.clear();
    EXPECT_TRUE(hm.getPopulatedVoxels().empty());
}

TEST(PerformanceHeatmap, Heatmap3DMissingVoxelsAndEmptyInterpolation) {
    HeatmapConfig cfg = makeSmallConfig();
    Heatmap3D hm(cfg);

    // Empty interpolation uses the internal empty voxel path.
    auto emptyInterp = hm.getInterpolatedMetrics({0.5, 0.5, 0.5});
    EXPECT_EQ(emptyInterp.sampleCount, 0u);
    EXPECT_DOUBLE_EQ(emptyInterp.maxAchievedVelocity, 0.0);

    // Insert a single voxel, then query another voxel to hit the not-found path.
    std::array<double, 3> p0 = {0.5, 0.5, 0.5};
    for (size_t n = 0; n < cfg.minSamplesForValidity; ++n) {
        hm.addSample(p0, 10.0, 1.0, 0.0, 1.0);
    }

    auto missing = hm.getMetrics({1.5, 1.5, 1.5});
    EXPECT_EQ(missing.sampleCount, 0u);

    auto defaults = hm.getSuggestedLimits({1.5, 1.5, 1.5});
    EXPECT_DOUBLE_EQ(defaults.maxVelocity, cfg.defaultMaxVelocity);
    EXPECT_DOUBLE_EQ(defaults.confidence, 0.0);

    // getAllVoxels should exercise both populated and unpopulated branches.
    auto all = hm.getAllVoxels();
    EXPECT_EQ(all.size(), 8u);
    auto populated = hm.getPopulatedVoxels();
    EXPECT_EQ(populated.size(), 1u);

    // Slice at a Z layer we did not populate should be empty.
    auto slice = hm.getSliceAtZ(1.5);
    for (const auto& cell : slice.getAllCells()) {
        EXPECT_EQ(cell.metrics.sampleCount, 0u);
    }
}

TEST(PerformanceHeatmap, DifferentialHeatmapExpectedFunctionAndProblemAreasAndGrid) {
    HeatmapConfig cfg = makeSmallConfig();
    DifferentialHeatmap diff(cfg);
    diff.setExpectedPerformance(100.0, 10.0, 1000.0);

    diff.setExpectedFunction([](const std::array<double, 3>&) {
        PerformanceMetrics m;
        m.maxAchievedVelocity = 200.0;
        m.maxAchievedAccel = 20.0;
        return m;
    });

    std::array<double, 3> p = {0.5, 0.5, 0.5};
    for (size_t n = 0; n < cfg.minSamplesForValidity; ++n) {
        diff.addActualSample(p, 50.0, 5.0, 0.02);
    }

    auto d = diff.getDifferential(p);
    EXPECT_NEAR(d.velocityRatio, 50.0 / 200.0, 1e-9);
    EXPECT_TRUE(d.underperforming);
    EXPECT_TRUE(d.overErroring);

    auto problems = diff.getProblemAreas(0.8);
    ASSERT_EQ(problems.size(), 1u);

    auto grid = diff.getVelocityRatioGrid(Heatmap2D::Plane::XY, 0.0);
    EXPECT_FALSE(grid.data.empty());
    for (double v : grid.data) {
        // Some cells are empty (0/expected => 0); populated ones should be <= 1.
        EXPECT_LE(v, 1.0);
    }
}

TEST(PerformanceHeatmap, DifferentialHeatmapNoExpectedFunctionAndZeroExpectedVelocity) {
    HeatmapConfig cfg = makeSmallConfig();
    DifferentialHeatmap diff(cfg);
    diff.setExpectedPerformance(0.0, 0.0, 0.0);

    std::array<double, 3> p = {0.5, 0.5, 0.5};
    for (size_t n = 0; n < cfg.minSamplesForValidity; ++n) {
        diff.addActualSample(p, 123.0, 4.0, 0.0);
    }

    auto d = diff.getDifferential(p);
    EXPECT_DOUBLE_EQ(d.velocityRatio, 1.0);
    EXPECT_DOUBLE_EQ(d.accelRatio, 1.0);
    EXPECT_FALSE(d.underperforming);
    EXPECT_FALSE(d.overErroring);

    EXPECT_TRUE(diff.getProblemAreas(0.8).empty());
}

TEST(PerformanceHeatmap, HeatmapBuilderProcessesSamplesAndReconfigures) {
    HeatmapConfig cfg = makeSmallConfig();
    HeatmapBuilder builder(cfg);

    std::array<double, 9> pos{};
    std::array<double, 9> vel{};
    pos[0] = 0.5; pos[1] = 0.5; pos[2] = 0.5;
    vel[0] = 1.0; vel[1] = 2.0; vel[2] = 3.0;

    // Cover feedRatio branch with commandedFeedRate=0.
    builder.processSample(pos, vel, 0.0, 0.0, 0.0, 123.0);

    // And normal branch.
    builder.processSample(pos, vel, 0.01, 0.02, 100.0, 50.0);

    // Ensure some data landed in the 3D heatmap.
    auto v = builder.get3DHeatmap().getPopulatedVoxels();
    EXPECT_FALSE(v.empty());

    // Cover inline getters for mutable access (header-only lines).
    EXPECT_EQ(builder.axisHeatmaps().size(), 9u);
    (void)builder.xyHeatmap();
    (void)builder.xzHeatmap();
    (void)builder.yzHeatmap();
    (void)builder.heatmap3D();

    // Cover inline const getters (header-only lines).
    const HeatmapBuilder& constBuilder = builder;
    EXPECT_EQ(constBuilder.getAxisHeatmaps().size(), 9u);
    (void)constBuilder.getXYHeatmap();
    (void)constBuilder.getXZHeatmap();
    (void)constBuilder.getYZHeatmap();
    (void)constBuilder.get3DHeatmap();

    builder.clear();
    EXPECT_TRUE(builder.get3DHeatmap().getPopulatedVoxels().empty());

    // Reconfigure to exercise configure paths.
    HeatmapConfig cfg2 = cfg;
    cfg2.resolution1D = 0.5;
    cfg2.resolution2D = 0.5;
    cfg2.resolution3D = 0.5;
    builder.configure(cfg2);
}
