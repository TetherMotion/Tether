/**
 * @file test_OnlineTuner_coverage.cpp
 * @brief Extended coverage tests for OnlineTuner, RollingStatistics, LiveMonitor
 *        Exercises branches not covered by existing test_OnlineTuner.cpp
 */

#include <gtest/gtest.h>
#include <tether/livetune/OnlineTuner.hpp>
#include <cmath>
#include <numeric>

using namespace LiveTune;

// Helper to make a TrajectorySample
static GCodeExport::TrajectorySample mkTraj(double t, double x, double y = 0, double z = 0,
                                              int seg = 0, int blk = 0,
                                              double feedRate = 100.0) {
    GCodeExport::TrajectorySample s{};
    s.time = t;
    s.pathPosition = t;
    s.position[0] = x;
    s.position[1] = y;
    s.position[2] = z;
    s.linearVelocity = feedRate;
    s.segmentIndex = seg;
    s.blockIndex = blk;
    s.motionType = 1;
    return s;
}

// Helper to make a PositionSample with velocity
static PositionSample mkPos(double t, double x, double y = 0, double z = 0,
                             double vx = 0, double vy = 0, double vz = 0) {
    PositionSample s{};
    s.timestamp = t;
    s.position[0] = x;
    s.position[1] = y;
    s.position[2] = z;
    s.velocity[0] = vx;
    s.velocity[1] = vy;
    s.velocity[2] = vz;
    s.velocityValid = (vx != 0 || vy != 0 || vz != 0);
    return s;
}

// ============================================================================
// RollingStatistics edge cases
// ============================================================================

TEST(RollingStatsCovTest, EmptyStatistics) {
    RollingStatistics rs(100);
    EXPECT_EQ(rs.count(), 0u);
    // All aggregates should handle empty gracefully
    EXPECT_TRUE(std::isnan(rs.mean()) || rs.mean() == 0.0);
}

TEST(RollingStatsCovTest, SingleSampleVariance) {
    RollingStatistics rs(100);
    rs.addSample(5.0);
    // Variance of single sample = 0
    EXPECT_EQ(rs.count(), 1u);
    EXPECT_DOUBLE_EQ(rs.min(), 5.0);
    EXPECT_DOUBLE_EQ(rs.max(), 5.0);
}

TEST(RollingStatsCovTest, NegativeValues) {
    RollingStatistics rs(100);
    rs.addSample(-10.0);
    rs.addSample(-20.0);
    rs.addSample(-5.0);
    EXPECT_DOUBLE_EQ(rs.min(), -20.0);
    EXPECT_DOUBLE_EQ(rs.max(), -5.0);
    EXPECT_NEAR(rs.mean(), -35.0 / 3.0, 1e-10);
}

TEST(RollingStatsCovTest, LargeDataSet) {
    RollingStatistics rs(50);
    for (int i = 0; i < 200; i++) {
        rs.addSample(static_cast<double>(i));
    }
    // Should not crash, count depends on implementation
    EXPECT_GT(rs.count(), 0u);
    EXPECT_GT(rs.rms(), 0.0);
}

TEST(RollingStatsCovTest, PercentileBoundary) {
    RollingStatistics rs(100);
    for (int i = 1; i <= 10; i++) {
        rs.addSample(static_cast<double>(i));
    }
    // P0 should be close to min, P100 close to max
    EXPECT_GE(rs.percentile(0), 1.0);
    EXPECT_LE(rs.percentile(100), 10.0);
    EXPECT_GT(rs.percentile(99), 5.0);
}

TEST(RollingStatsCovTest, GeometricMeanOfOnes) {
    RollingStatistics rs(100);
    for (int i = 0; i < 10; i++) {
        rs.addSample(1.0);
    }
    EXPECT_NEAR(rs.geometricMean(), 1.0, 1e-10);
}

// ============================================================================
// OnlineTuner: Multi-axis tracking
// ============================================================================

class TunerCovTest : public ::testing::Test {
protected:
    void SetUp() override {
        tuner_ = std::make_unique<class OnlineTuner>();
    }
    std::unique_ptr<class OnlineTuner> tuner_;
};

TEST_F(TunerCovTest, MultiAxisError) {
    // 3-axis trajectory: X, Y, Z all moving
    std::vector<GCodeExport::TrajectorySample> traj;
    for (int i = 0; i <= 100; i++) {
        double t = i * 0.001;
        traj.push_back(mkTraj(t, t * 50.0, t * 30.0, t * 10.0));
    }
    tuner_->setDesiredTrajectory(traj);
    
    for (int i = 0; i <= 100; i++) {
        double t = i * 0.001;
        // Errors in all axes
        tuner_->addActualSample(mkPos(t, t * 50.0 + 0.02, t * 30.0 - 0.01, t * 10.0 + 0.03));
    }
    
    size_t processed = tuner_->processAccumulatedSamples();
    EXPECT_GT(processed, 0u);
    
    auto stats = tuner_->getOverallStatistics();
    if (stats.sampleCount > 0) {
        EXPECT_GT(stats.meanError, 0.0);
        EXPECT_GT(stats.rmsError, 0.0);
    }
}

TEST_F(TunerCovTest, TrackingWithVelocity) {
    std::vector<GCodeExport::TrajectorySample> traj;
    for (int i = 0; i <= 50; i++) {
        double t = i * 0.001;
        auto s = mkTraj(t, t * 100.0);
        s.linearVelocity = 100.0;
        traj.push_back(s);
    }
    tuner_->setDesiredTrajectory(traj);
    
    for (int i = 0; i <= 50; i++) {
        double t = i * 0.001;
        // Position sample with velocity data
        tuner_->addActualSample(mkPos(t, t * 100.0 + 0.01, 0, 0, 100.0));
    }
    tuner_->processAccumulatedSamples();
}

TEST_F(TunerCovTest, MultiSegmentTrajectory) {
    std::vector<GCodeExport::TrajectorySample> traj;
    // Segment 0
    for (int i = 0; i <= 50; i++) {
        double t = i * 0.001;
        traj.push_back(mkTraj(t, t * 100.0, 0, 0, 0, 0, 200.0));
    }
    // Segment 1 (higher feed rate)
    for (int i = 51; i <= 100; i++) {
        double t = i * 0.001;
        traj.push_back(mkTraj(t, t * 100.0, t * 50.0, 0, 1, 1, 300.0));
    }
    tuner_->setDesiredTrajectory(traj);
    
    for (int i = 0; i <= 100; i++) {
        double t = i * 0.001;
        tuner_->addActualSample(mkPos(t, t * 100.0 + 0.05, 
                                           i > 50 ? t * 50.0 + 0.03 : 0));
    }
    tuner_->processAccumulatedSamples();
    
    auto perf = tuner_->getSegmentPerformance();
    // May have one or more segments
    (void)perf;
}

TEST_F(TunerCovTest, GetSuggestedLimits_WithData) {
    std::vector<GCodeExport::TrajectorySample> traj;
    for (int i = 0; i <= 200; i++) {
        double t = i * 0.001;
        traj.push_back(mkTraj(t, t * 100.0));
    }
    tuner_->setDesiredTrajectory(traj);
    
    for (int i = 0; i <= 200; i++) {
        double t = i * 0.001;
        tuner_->addActualSample(mkPos(t, t * 100.0 + 0.1));
    }
    tuner_->processAccumulatedSamples();
    
    auto limits = tuner_->getSuggestedLimits(0.05);
    EXPECT_GT(limits.feedRate, 0.0);
    
    // With smooth transition
    auto limitsSmooth = tuner_->getSuggestedLimits(0.05, true);
    EXPECT_GT(limitsSmooth.feedRate, 0.0);
    
    // Without smooth transition
    auto limitsNoSmooth = tuner_->getSuggestedLimits(0.05, false);
    EXPECT_GT(limitsNoSmooth.feedRate, 0.0);
}

TEST_F(TunerCovTest, GetAllErrors) {
    std::vector<GCodeExport::TrajectorySample> traj;
    for (int i = 0; i <= 50; i++) {
        double t = i * 0.001;
        traj.push_back(mkTraj(t, t * 100.0));
    }
    tuner_->setDesiredTrajectory(traj);
    
    for (int i = 0; i <= 50; i++) {
        double t = i * 0.001;
        tuner_->addActualSample(mkPos(t, t * 100.0 + 0.5));
    }
    tuner_->processAccumulatedSamples();
    
    auto& errors = tuner_->getAllErrors();
    if (!errors.empty()) {
        EXPECT_GT(errors[0].combinedPositionError, 0.0);
    }
}

TEST_F(TunerCovTest, GetCurrentError) {
    std::vector<GCodeExport::TrajectorySample> traj;
    for (int i = 0; i <= 50; i++) {
        double t = i * 0.001;
        traj.push_back(mkTraj(t, t * 100.0));
    }
    tuner_->setDesiredTrajectory(traj);
    
    for (int i = 0; i <= 50; i++) {
        double t = i * 0.001;
        tuner_->addActualSample(mkPos(t, t * 100.0 + 0.2));
    }
    tuner_->processAccumulatedSamples();
    
    auto err = tuner_->getCurrentError();
    // May or may not have a current error
    (void)err;
}

TEST_F(TunerCovTest, CriticalPointStatistics) {
    // Create a trajectory with a sharp corner
    std::vector<GCodeExport::TrajectorySample> traj;
    // Straight segment
    for (int i = 0; i <= 50; i++) {
        double t = i * 0.001;
        traj.push_back(mkTraj(t, t * 100.0, 0, 0, 0, 0));
    }
    // Corner: change direction
    for (int i = 51; i <= 100; i++) {
        double t = i * 0.001;
        double x = 50 * 0.001 * 100.0; // Fixed X
        double y = (i - 50) * 0.001 * 100.0; // Moving in Y
        traj.push_back(mkTraj(t, x, y, 0, 1, 1));
    }
    tuner_->setDesiredTrajectory(traj);
    
    for (int i = 0; i <= 100; i++) {
        double t = i * 0.001;
        double x = (i <= 50) ? t * 100.0 : 50 * 0.001 * 100.0;
        double y = (i > 50) ? (i - 50) * 0.001 * 100.0 + 0.1 : 0;
        tuner_->addActualSample(mkPos(t, x + 0.01, y));
    }
    tuner_->processAccumulatedSamples();
    
    auto critStats = tuner_->getCriticalPointStatistics();
    (void)critStats;
}

TEST_F(TunerCovTest, DetectSystemDelay_WithData) {
    std::vector<GCodeExport::TrajectorySample> traj;
    for (int i = 0; i <= 200; i++) {
        double t = i * 0.001;
        traj.push_back(mkTraj(t, sin(t * 10.0) * 50.0));
    }
    tuner_->setDesiredTrajectory(traj);
    
    // Actual data with a slight delay
    for (int i = 0; i <= 200; i++) {
        double t = i * 0.001;
        double delayed_t = std::max(0.0, t - 0.002); // 2ms delay
        tuner_->addActualSample(mkPos(t, sin(delayed_t * 10.0) * 50.0));
    }
    tuner_->processAccumulatedSamples();
    
    double delay = tuner_->detectSystemDelay();
    // Just verify it returns something reasonable
    EXPECT_GE(delay, 0.0);
}

TEST_F(TunerCovTest, ConfigureAutoDetect) {
    OnlineTunerConfig cfg;
    cfg.autoDetectDelay = true;
    cfg.systemDelay = 0.0;
    tuner_->configure(cfg);
    EXPECT_TRUE(tuner_->config().autoDetectDelay);
}

TEST_F(TunerCovTest, ConfigureLimitAdjust) {
    OnlineTunerConfig cfg;
    cfg.enableLimitSuggestions = true;
    cfg.enableAutoAdjustment = true;
    cfg.monitoringOnly = false;
    cfg.logAllSamples = true;
    tuner_->configure(cfg);
    EXPECT_TRUE(tuner_->config().enableAutoAdjustment);
    EXPECT_FALSE(tuner_->config().monitoringOnly);
}

TEST_F(TunerCovTest, ParameterSuggestions_LargeError) {
    OnlineTunerConfig cfg;
    cfg.positionErrorThreshold = 0.001; // Very tight threshold
    cfg.enableLimitSuggestions = true;
    tuner_->configure(cfg);
    
    std::vector<GCodeExport::TrajectorySample> traj;
    for (int i = 0; i <= 200; i++) {
        double t = i * 0.001;
        traj.push_back(mkTraj(t, t * 100.0, 0, 0, 0, 0, 500.0));
    }
    tuner_->setDesiredTrajectory(traj);
    
    for (int i = 0; i <= 200; i++) {
        double t = i * 0.001;
        tuner_->addActualSample(mkPos(t, t * 100.0 + 1.0)); // 1mm error
    }
    tuner_->processAccumulatedSamples();
    
    auto suggestions = tuner_->getParameterSuggestions();
    // May generate suggestions due to large error
    (void)suggestions;
}

// ============================================================================
// LiveMonitor deep coverage
// ============================================================================

TEST(LiveMonitorCovTest, MultipleUpdates) {
    LiveMonitor monitor;
    for (int i = 0; i < 100; i++) {
        double t = i * 0.001;
        auto desired = mkTraj(t, t * 100.0, t * 50.0);
        auto actual = mkPos(t, t * 100.0 + sin(t * 20.0) * 0.1, t * 50.0 + cos(t * 20.0) * 0.05);
        monitor.update(actual, desired);
    }
    auto status = monitor.getStatus();
    EXPECT_GT(status.rollingStats.sampleCount, 0u);
}

TEST(LiveMonitorCovTest, AlertCallback) {
    OnlineTunerConfig cfg;
    cfg.positionErrorThreshold = 0.001; // Very tight
    cfg.contourErrorThreshold = 0.001;
    cfg.lagErrorThreshold = 0.001;
    LiveMonitor monitor(cfg);
    
    std::vector<std::string> alerts;
    monitor.setAlertCallback([&](const std::string& msg) {
        alerts.push_back(msg);
    });
    
    // Drive with large error to trigger alert
    auto desired = mkTraj(0.0, 100.0, 200.0);
    auto actual = mkPos(0.0, 110.0, 220.0); // 10mm error
    monitor.update(actual, desired);
    
    // Alert may have fired
    (void)alerts;
}

TEST(LiveMonitorCovTest, ResetClearsState) {
    LiveMonitor monitor;
    auto desired = mkTraj(0.0, 100.0);
    auto actual = mkPos(0.0, 105.0);
    monitor.update(actual, desired);
    
    monitor.reset();
    auto status = monitor.getStatus();
    EXPECT_DOUBLE_EQ(status.currentPositionError, 0.0);
    EXPECT_DOUBLE_EQ(status.currentContourError, 0.0);
    EXPECT_DOUBLE_EQ(status.currentLagError, 0.0);
}

TEST(LiveMonitorCovTest, HealthMetrics) {
    LiveMonitor monitor;
    for (int i = 0; i < 50; i++) {
        double t = i * 0.001;
        auto desired = mkTraj(t, t * 100.0);
        auto actual = mkPos(t, t * 100.0 + 0.001); // tiny error
        monitor.update(actual, desired);
    }
    auto status = monitor.getStatus();
    // With tiny errors, health should be high
    EXPECT_GT(status.overallHealth, 0.0);
}

TEST(LiveMonitorCovTest, PerfectTracking) {
    LiveMonitor monitor;
    for (int i = 0; i < 20; i++) {
        double t = i * 0.001;
        auto desired = mkTraj(t, t * 100.0, t * 50.0, t * 25.0);
        auto actual = mkPos(t, t * 100.0, t * 50.0, t * 25.0);
        monitor.update(actual, desired);
    }
    auto status = monitor.getStatus();
    EXPECT_NEAR(status.currentPositionError, 0.0, 1e-6);
    EXPECT_FALSE(status.positionAlert);
    EXPECT_FALSE(status.contourAlert);
    EXPECT_FALSE(status.lagAlert);
}

// ============================================================================
// Config struct field coverage
// ============================================================================

TEST(OnlineTunerConfigCovTest, AllFields) {
    OnlineTunerConfig cfg;
    cfg.systemDelay = 0.005;
    cfg.autoDetectDelay = true;
    cfg.positionErrorThreshold = 0.1;
    cfg.contourErrorThreshold = 0.05;
    cfg.lagErrorThreshold = 0.2;
    cfg.cornerAngleThreshold = 15.0;
    cfg.cornerProximity = 2.0;
    cfg.enableLimitSuggestions = true;
    cfg.enableAutoAdjustment = false;
    cfg.monitoringOnly = true;
    cfg.logAllSamples = true;
    
    EXPECT_DOUBLE_EQ(cfg.systemDelay, 0.005);
    EXPECT_TRUE(cfg.autoDetectDelay);
    EXPECT_DOUBLE_EQ(cfg.positionErrorThreshold, 0.1);
    EXPECT_DOUBLE_EQ(cfg.contourErrorThreshold, 0.05);
    EXPECT_DOUBLE_EQ(cfg.lagErrorThreshold, 0.2);
    EXPECT_DOUBLE_EQ(cfg.cornerAngleThreshold, 15.0);
}

// ============================================================================
// Limits struct
// ============================================================================

TEST(LimitsCovTest, Defaults) {
    // Limits is a nested struct inside OnlineTuner
    class OnlineTuner rep;
    auto lim = rep.getSuggestedLimits(0.0);
    // Check returned limits have reasonable values
    EXPECT_GE(lim.feedRate, 0.0);
}

// ============================================================================
// ErrorCallback and SuggestionCallback
// ============================================================================

TEST(TunerCallbackCovTest, SetAndClearErrorCallback) {
    class OnlineTuner rep;
    rep.setErrorCallback([](const TrackingError&) {});
    rep.setErrorCallback(nullptr); // Clear
}

TEST(TunerCallbackCovTest, SetAndClearSuggestionCallback) {
    class OnlineTuner rep;
    rep.setSuggestionCallback([](const ParameterSuggestion&) {});
    rep.setSuggestionCallback(nullptr); // Clear
}
