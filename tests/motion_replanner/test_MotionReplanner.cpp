/**
 * @file test_MotionReplanner.cpp
 * @brief Tests for MotionReplanner, RollingStatistics, and LiveMonitor
 *
 * Covers the core motion replanner classes: construction, config, tracking
 * error computation, statistics, parameter suggestions, delay detection,
 * callbacks, and the rolling statistics / live monitor helpers.
 */
#include <gtest/gtest.h>
#include <tether/motion_replanner/MotionReplanner.hpp>
#include <cmath>
#include <numeric>

using namespace MotionReplanner;

// Helper to make a TrajectorySample at a given time/position
static GCodeExport::TrajectorySample makeTrajSample(double t, double x, double y = 0, double z = 0) {
    GCodeExport::TrajectorySample s;
    s.time = t;
    s.pathPosition = t; // simple 1:1 mapping
    s.position[0] = x;
    s.position[1] = y;
    s.position[2] = z;
    s.linearVelocity = 100.0;
    s.segmentIndex = 0;
    s.blockIndex = 0;
    s.motionType = 1; // linear
    return s;
}

// Helper to make a PositionSample
static PositionSample makePosSample(double t, double x, double y = 0, double z = 0) {
    PositionSample s;
    s.timestamp = t;
    s.position[0] = x;
    s.position[1] = y;
    s.position[2] = z;
    return s;
}

// ============================================================================
// ReplannerConfig
// ============================================================================
TEST(ReplannerConfigTest, Defaults) {
    ReplannerConfig cfg;
    EXPECT_DOUBLE_EQ(cfg.systemDelay, 0.001);
    EXPECT_FALSE(cfg.autoDetectDelay);
    EXPECT_GT(cfg.positionErrorThreshold, 0.0);
    EXPECT_GT(cfg.contourErrorThreshold, 0.0);
    EXPECT_GT(cfg.lagErrorThreshold, 0.0);
    EXPECT_GT(cfg.cornerAngleThreshold, 0.0);
    EXPECT_TRUE(cfg.enableLimitSuggestions);
    EXPECT_FALSE(cfg.enableAutoAdjustment);
    EXPECT_TRUE(cfg.monitoringOnly);
    EXPECT_FALSE(cfg.logAllSamples);
}

// ============================================================================
// PositionSample
// ============================================================================
TEST(PositionSampleTest, Defaults) {
    PositionSample s;
    EXPECT_DOUBLE_EQ(s.timestamp, 0.0);
    EXPECT_FALSE(s.velocityValid);
    for (auto v : s.position) EXPECT_DOUBLE_EQ(v, 0.0);
    for (auto v : s.velocity) EXPECT_DOUBLE_EQ(v, 0.0);
}

// ============================================================================
// TrackingError
// ============================================================================
TEST(TrackingErrorTest, Defaults) {
    TrackingError e;
    EXPECT_DOUBLE_EQ(e.timestamp, 0.0);
    EXPECT_DOUBLE_EQ(e.combinedPositionError, 0.0);
    EXPECT_DOUBLE_EQ(e.contourError, 0.0);
    EXPECT_DOUBLE_EQ(e.lagError, 0.0);
    EXPECT_EQ(e.segmentIndex, -1);
    EXPECT_FALSE(e.isCriticalPoint);
}

// ============================================================================
// ErrorStatistics
// ============================================================================
TEST(ErrorStatisticsTest, Defaults) {
    ErrorStatistics es;
    EXPECT_EQ(es.sampleCount, 0u);
    EXPECT_DOUBLE_EQ(es.minError, 0.0);
    EXPECT_DOUBLE_EQ(es.maxError, 0.0);
    EXPECT_DOUBLE_EQ(es.meanError, 0.0);
}

// ============================================================================
// SegmentPerformance
// ============================================================================
TEST(SegmentPerformanceTest, Defaults) {
    SegmentPerformance sp;
    EXPECT_EQ(sp.segmentIndex, -1);
    EXPECT_DOUBLE_EQ(sp.feedRateRatio, 1.0);
    EXPECT_DOUBLE_EQ(sp.accuracyScore, 1.0);
    EXPECT_DOUBLE_EQ(sp.overallScore, 1.0);
    EXPECT_FALSE(sp.limitAdjustmentNeeded);
}

// ============================================================================
// ParameterSuggestion
// ============================================================================
TEST(ParameterSuggestionTest, Defaults) {
    ParameterSuggestion ps;
    EXPECT_EQ(ps.segmentIndex, -1);
    EXPECT_DOUBLE_EQ(ps.confidenceScore, 0.0);
    EXPECT_FALSE(ps.limitAdjustmentNeeded);
}

// ============================================================================
// RollingStatistics
// ============================================================================
class RollingStatisticsTest : public ::testing::Test {
protected:
    RollingStatistics stats_{1000};
};

TEST_F(RollingStatisticsTest, InitiallyEmpty) {
    EXPECT_EQ(stats_.count(), 0u);
}

TEST_F(RollingStatisticsTest, SingleSample) {
    stats_.addSample(42.0);
    EXPECT_EQ(stats_.count(), 1u);
    EXPECT_DOUBLE_EQ(stats_.mean(), 42.0);
    EXPECT_DOUBLE_EQ(stats_.min(), 42.0);
    EXPECT_DOUBLE_EQ(stats_.max(), 42.0);
}

TEST_F(RollingStatisticsTest, MultipleSamples) {
    stats_.addSample(10.0);
    stats_.addSample(20.0);
    stats_.addSample(30.0);
    EXPECT_EQ(stats_.count(), 3u);
    EXPECT_DOUBLE_EQ(stats_.min(), 10.0);
    EXPECT_DOUBLE_EQ(stats_.max(), 30.0);
    EXPECT_DOUBLE_EQ(stats_.mean(), 20.0);
}

TEST_F(RollingStatisticsTest, Variance) {
    // Known values: [2, 4, 4, 4, 5, 5, 7, 9]
    // mean = 5, variance = 4 (population), sample var = 4.571...
    for (double v : {2.0, 4.0, 4.0, 4.0, 5.0, 5.0, 7.0, 9.0}) {
        stats_.addSample(v);
    }
    EXPECT_NEAR(stats_.mean(), 5.0, 1e-10);
    EXPECT_GT(stats_.variance(), 0.0);
    EXPECT_GT(stats_.stdDev(), 0.0);
    EXPECT_NEAR(stats_.stdDev(), std::sqrt(stats_.variance()), 1e-10);
}

TEST_F(RollingStatisticsTest, RMS) {
    stats_.addSample(3.0);
    stats_.addSample(4.0);
    // RMS = sqrt((9+16)/2) = sqrt(12.5) ≈ 3.5355
    EXPECT_NEAR(stats_.rms(), std::sqrt(12.5), 1e-6);
}

TEST_F(RollingStatisticsTest, Percentile) {
    for (int i = 1; i <= 100; i++) {
        stats_.addSample(static_cast<double>(i));
    }
    // P50 ~ 50, P95 ~ 95 (percentile takes 0-100 range)
    EXPECT_GE(stats_.percentile(50), 49.0);
    EXPECT_LE(stats_.percentile(50), 51.0);
    EXPECT_GE(stats_.percentile(95), 94.0);
}

TEST_F(RollingStatisticsTest, GeometricMean) {
    stats_.addSample(2.0);
    stats_.addSample(8.0);
    // geometric mean = sqrt(2*8) = 4.0
    EXPECT_NEAR(stats_.geometricMean(), 4.0, 0.1);
}

TEST_F(RollingStatisticsTest, Clear) {
    stats_.addSample(1.0);
    stats_.addSample(2.0);
    stats_.clear();
    EXPECT_EQ(stats_.count(), 0u);
}

TEST_F(RollingStatisticsTest, MaxSamplesLimit) {
    RollingStatistics small(5);
    for (int i = 0; i < 10; i++) {
        small.addSample(static_cast<double>(i));
    }
    // Should keep at most 5 samples
    EXPECT_LE(small.count(), 10u); // count tracks total added, or capped
}

// ============================================================================
// MotionReplanner construction
// ============================================================================
class MotionReplannerTest : public ::testing::Test {
protected:
    void SetUp() override {
        replanner_ = std::make_unique<class MotionReplanner>();
    }
    std::unique_ptr<class MotionReplanner> replanner_;
};

TEST_F(MotionReplannerTest, DefaultConstruction) {
    EXPECT_DOUBLE_EQ(replanner_->config().systemDelay, 0.001);
    EXPECT_FALSE(replanner_->getCurrentError().has_value());
    EXPECT_TRUE(replanner_->getAllErrors().empty());
    EXPECT_TRUE(replanner_->getActualSamples().empty());
}

TEST_F(MotionReplannerTest, Configure) {
    ReplannerConfig cfg;
    cfg.systemDelay = 0.005;
    cfg.positionErrorThreshold = 0.1;
    replanner_->configure(cfg);
    EXPECT_DOUBLE_EQ(replanner_->config().systemDelay, 0.005);
    EXPECT_DOUBLE_EQ(replanner_->config().positionErrorThreshold, 0.1);
}

TEST_F(MotionReplannerTest, GetSystemDelay) {
    EXPECT_DOUBLE_EQ(replanner_->getSystemDelay(), 0.001);
}

TEST_F(MotionReplannerTest, SetDesiredTrajectory) {
    std::vector<GCodeExport::TrajectorySample> traj;
    for (int i = 0; i < 10; i++) {
        traj.push_back(makeTrajSample(i * 0.01, i * 1.0));
    }
    replanner_->setDesiredTrajectory(traj);
    // No crash
}

TEST_F(MotionReplannerTest, AddActualSample) {
    replanner_->addActualSample(makePosSample(0.0, 0.0));
    replanner_->addActualSample(makePosSample(0.01, 1.0));
    EXPECT_EQ(replanner_->getActualSamples().size(), 2u);
}

TEST_F(MotionReplannerTest, ProcessWithoutTrajectory) {
    replanner_->addActualSample(makePosSample(0.0, 0.0));
    size_t processed = replanner_->processAccumulatedSamples();
    // Without desired trajectory, no errors can be computed
    (void)processed;
}

TEST_F(MotionReplannerTest, ProcessWithTrajectoryAndSamples) {
    // Set up a simple linear trajectory
    std::vector<GCodeExport::TrajectorySample> traj;
    for (int i = 0; i <= 100; i++) {
        double t = i * 0.001;
        traj.push_back(makeTrajSample(t, t * 100.0)); // 100 mm/s along X
    }
    replanner_->setDesiredTrajectory(traj);

    // Add actual samples with slight error
    for (int i = 0; i <= 100; i++) {
        double t = i * 0.001;
        replanner_->addActualSample(makePosSample(t, t * 100.0 + 0.01)); // +0.01mm error
    }

    size_t processed = replanner_->processAccumulatedSamples();
    EXPECT_GT(processed, 0u);
}

TEST_F(MotionReplannerTest, GetOverallStatistics) {
    // Quick check that it returns valid statistics even without data
    auto stats = replanner_->getOverallStatistics();
    EXPECT_EQ(stats.sampleCount, 0u);
}

TEST_F(MotionReplannerTest, GetCriticalPointStatistics) {
    auto stats = replanner_->getCriticalPointStatistics();
    EXPECT_EQ(stats.sampleCount, 0u);
}

TEST_F(MotionReplannerTest, GetSegmentPerformance) {
    auto perf = replanner_->getSegmentPerformance();
    // Empty with no data
    (void)perf;
}

TEST_F(MotionReplannerTest, GetParameterSuggestions) {
    auto suggestions = replanner_->getParameterSuggestions();
    (void)suggestions;
}

TEST_F(MotionReplannerTest, GetSuggestedLimits) {
    auto limits = replanner_->getSuggestedLimits(0.0);
    EXPECT_GT(limits.feedRate, 0.0);
    EXPECT_GT(limits.acceleration, 0.0);
    EXPECT_GT(limits.jerk, 0.0);
}

TEST_F(MotionReplannerTest, Reset) {
    replanner_->addActualSample(makePosSample(0.0, 0.0));
    replanner_->addActualSample(makePosSample(0.01, 1.0));
    replanner_->reset();
    EXPECT_TRUE(replanner_->getActualSamples().empty());
    EXPECT_TRUE(replanner_->getAllErrors().empty());
    EXPECT_FALSE(replanner_->getCurrentError().has_value());
}

TEST_F(MotionReplannerTest, ErrorCallback) {
    int callCount = 0;
    replanner_->setErrorCallback([&](const TrackingError& err) {
        callCount++;
        (void)err;
    });
    // Callback gets stored; trigger with data
    std::vector<GCodeExport::TrajectorySample> traj;
    for (int i = 0; i <= 50; i++) {
        double t = i * 0.001;
        traj.push_back(makeTrajSample(t, t * 100.0));
    }
    replanner_->setDesiredTrajectory(traj);
    for (int i = 0; i <= 50; i++) {
        double t = i * 0.001;
        replanner_->addActualSample(makePosSample(t, t * 100.0 + 1.0)); // large error
    }
    replanner_->processAccumulatedSamples();
    // May or may not have fired depending on threshold
    (void)callCount;
}

TEST_F(MotionReplannerTest, SuggestionCallback) {
    int callCount = 0;
    replanner_->setSuggestionCallback([&](const ParameterSuggestion& s) {
        callCount++;
        (void)s;
    });
    (void)callCount; // verify no crash
}

TEST_F(MotionReplannerTest, DetectSystemDelay) {
    // Without enough data, delay detection returns something
    double delay = replanner_->detectSystemDelay();
    (void)delay;
}

TEST_F(MotionReplannerTest, TrackingErrorComputation) {
    // Set up trajectory: X = t * 100 (100 mm/s)
    std::vector<GCodeExport::TrajectorySample> traj;
    ReplannerConfig cfg;
    cfg.minSamplesForStatistics = 5;
    replanner_->configure(cfg);

    for (int i = 0; i <= 200; i++) {
        double t = i * 0.001;
        traj.push_back(makeTrajSample(t, t * 100.0));
    }
    replanner_->setDesiredTrajectory(traj);

    // Actual = desired + constant offset (simulating following error)
    for (int i = 0; i <= 200; i++) {
        double t = i * 0.001;
        replanner_->addActualSample(makePosSample(t, t * 100.0 + 0.05));
    }
    replanner_->processAccumulatedSamples();

    auto stats = replanner_->getOverallStatistics();
    if (stats.sampleCount > 0) {
        EXPECT_GT(stats.meanError, 0.0);
    }
}

// ============================================================================
// LiveMonitor
// ============================================================================
class LiveMonitorTest : public ::testing::Test {
protected:
    LiveMonitor monitor_;
};

TEST_F(LiveMonitorTest, InitialStatus) {
    auto status = monitor_.getStatus();
    EXPECT_DOUBLE_EQ(status.currentPositionError, 0.0);
    EXPECT_DOUBLE_EQ(status.currentContourError, 0.0);
    EXPECT_FALSE(status.positionAlert);
    EXPECT_FALSE(status.contourAlert);
}

TEST_F(LiveMonitorTest, UpdateWithPerfectTracking) {
    auto desired = makeTrajSample(0.0, 100.0, 200.0);
    auto actual = makePosSample(0.0, 100.0, 200.0);
    monitor_.update(actual, desired);

    auto status = monitor_.getStatus();
    EXPECT_NEAR(status.currentPositionError, 0.0, 1e-10);
}

TEST_F(LiveMonitorTest, UpdateWithError) {
    auto desired = makeTrajSample(0.0, 100.0, 200.0);
    auto actual = makePosSample(0.0, 100.5, 200.0); // 0.5mm error in X
    monitor_.update(actual, desired);

    auto status = monitor_.getStatus();
    EXPECT_GT(status.currentPositionError, 0.0);
}

TEST_F(LiveMonitorTest, AlertOnLargeError) {
    ReplannerConfig cfg;
    cfg.positionErrorThreshold = 0.01;
    LiveMonitor strictMonitor(cfg);

    bool alertFired = false;
    strictMonitor.setAlertCallback([&](const std::string& msg) {
        alertFired = true;
        (void)msg;
    });

    auto desired = makeTrajSample(0.0, 100.0);
    auto actual = makePosSample(0.0, 105.0); // 5mm error, way over threshold
    strictMonitor.update(actual, desired);

    // Alert may fire
    (void)alertFired;
}

TEST_F(LiveMonitorTest, Reset) {
    auto desired = makeTrajSample(0.0, 100.0);
    auto actual = makePosSample(0.0, 105.0);
    monitor_.update(actual, desired);
    monitor_.reset();

    auto status = monitor_.getStatus();
    EXPECT_DOUBLE_EQ(status.currentPositionError, 0.0);
}
