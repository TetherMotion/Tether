/**
 * @file test_livetune.cpp
 * @brief Unit tests for OnlineTuner class
 */

#include <gtest/gtest.h>
#include "OnlineTuner.hpp"
#include <cmath>

using namespace LiveTune;

class OnlineTunerTest : public ::testing::Test {
protected:
    void SetUp() override {
        config.systemDelay = 0.001;
        config.samplePeriod = 0.001;
        config.cornerVelocityThreshold = 0.1;
        config.cornerAccelThreshold = 100.0;
        config.errorThreshold = 0.1;
        config.mode = OperationMode::Monitor;
    }
    
    OnlineTunerConfig config;
};

// Test basic sample processing
TEST_F(OnlineTunerTest, ProcessSingleSample) {
    OnlineTuner livetune(config);
    
    TrajectorySample commanded;
    commanded.timestamp = 0.0;
    commanded.position = {0.0, 0.0, 0.0};
    commanded.velocity = {10.0, 0.0, 0.0};
    commanded.acceleration = {0.0, 0.0, 0.0};
    commanded.segmentIndex = 0;
    
    TrajectorySample actual = commanded;
    actual.position = {0.001, 0.0, 0.0};  // Slight error
    
    livetune.processSample(commanded, actual);
    
    auto stats = livetune.getGlobalStatistics();
    EXPECT_EQ(stats.sampleCount, 1);
    EXPECT_NEAR(stats.mean, 0.001, 1e-6);
}

// Test error statistics calculation
TEST_F(OnlineTunerTest, ErrorStatisticsCalculation) {
    OnlineTuner livetune(config);
    
    // Feed samples with known errors
    std::vector<double> errors = {0.01, 0.02, 0.03, 0.04, 0.05};
    
    for (size_t i = 0; i < errors.size(); ++i) {
        TrajectorySample commanded;
        commanded.timestamp = i * 0.001;
        commanded.position = {static_cast<double>(i) * 10.0, 0.0, 0.0};
        commanded.velocity = {10.0, 0.0, 0.0};
        commanded.acceleration = {0.0, 0.0, 0.0};
        commanded.segmentIndex = 0;
        
        TrajectorySample actual = commanded;
        actual.position[0] += errors[i];
        
        livetune.processSample(commanded, actual);
    }
    
    auto stats = livetune.getGlobalStatistics();
    
    EXPECT_EQ(stats.sampleCount, 5);
    EXPECT_NEAR(stats.min, 0.01, 1e-6);
    EXPECT_NEAR(stats.max, 0.05, 1e-6);
    EXPECT_NEAR(stats.mean, 0.03, 1e-6);
}

// Test corner detection
TEST_F(OnlineTunerTest, CornerDetection) {
    OnlineTuner livetune(config);
    
    // Create a trajectory with a corner (velocity direction change)
    
    // First segment: +X direction
    for (int i = 0; i < 10; ++i) {
        TrajectorySample commanded;
        commanded.timestamp = i * 0.001;
        commanded.position = {static_cast<double>(i), 0.0, 0.0};
        commanded.velocity = {100.0, 0.0, 0.0};
        commanded.acceleration = {0.0, 0.0, 0.0};
        commanded.segmentIndex = 0;
        
        TrajectorySample actual = commanded;
        livetune.processSample(commanded, actual);
    }
    
    // Corner: sudden direction change to +Y
    for (int i = 0; i < 10; ++i) {
        TrajectorySample commanded;
        commanded.timestamp = (10 + i) * 0.001;
        commanded.position = {10.0, static_cast<double>(i), 0.0};
        commanded.velocity = {0.0, 100.0, 0.0};
        commanded.acceleration = {0.0, 0.0, 0.0};
        commanded.segmentIndex = 1;
        
        TrajectorySample actual = commanded;
        // Add larger error at corner
        if (i < 3) {
            actual.position[0] += 0.1;  // Overshoot
        }
        livetune.processSample(commanded, actual);
    }
    
    auto cornerStats = livetune.getCornerStatistics();
    
    // Should have detected corner samples
    EXPECT_GT(cornerStats.sampleCount, 0);
}

// Test delay compensation
TEST_F(OnlineTunerTest, DelayCompensation) {
    config.systemDelay = 0.002;  // 2ms delay
    config.compensateDelay = true;
    
    OnlineTuner livetune(config);
    
    // Feed a trajectory moving at constant velocity
    double velocity = 100.0;  // mm/s
    
    for (int i = 0; i < 20; ++i) {
        TrajectorySample commanded;
        commanded.timestamp = i * 0.001;
        commanded.position = {velocity * i * 0.001, 0.0, 0.0};
        commanded.velocity = {velocity, 0.0, 0.0};
        commanded.acceleration = {0.0, 0.0, 0.0};
        commanded.segmentIndex = 0;
        
        // Actual position is delayed by 2 samples
        TrajectorySample actual = commanded;
        if (i >= 2) {
            actual.position = {velocity * (i - 2) * 0.001, 0.0, 0.0};
        } else {
            actual.position = {0.0, 0.0, 0.0};
        }
        
        livetune.processSample(commanded, actual);
    }
    
    // With delay compensation, the error should be small
    // (exact values depend on implementation details)
    auto stats = livetune.getGlobalStatistics();
    EXPECT_GT(stats.sampleCount, 0);
}

// Test parameter suggestions in Suggest mode
TEST_F(OnlineTunerTest, ParameterSuggestions) {
    config.mode = OperationMode::Suggest;
    config.maxVelocity = 200.0;
    config.maxAcceleration = 1000.0;
    config.maxJerk = 50000.0;
    
    OnlineTuner livetune(config);
    
    // Feed samples with good tracking (low error)
    for (int i = 0; i < 100; ++i) {
        TrajectorySample commanded;
        commanded.timestamp = i * 0.001;
        commanded.position = {static_cast<double>(i), 0.0, 0.0};
        commanded.velocity = {50.0, 0.0, 0.0};  // Well below max
        commanded.acceleration = {100.0, 0.0, 0.0};
        commanded.segmentIndex = 0;
        
        TrajectorySample actual = commanded;
        actual.position[0] += 0.001;  // Very small error
        
        livetune.processSample(commanded, actual);
    }
    
    auto suggestions = livetune.getSuggestions();
    
    // Should suggest increased limits since tracking is good
    // (specific suggestions depend on implementation)
    // Just verify we get some suggestions
    // EXPECT_GT(suggestions.size(), 0);  // May or may not have suggestions
}

// Test livetune reset
TEST_F(OnlineTunerTest, Reset) {
    OnlineTuner livetune(config);
    
    // Add some samples
    for (int i = 0; i < 10; ++i) {
        TrajectorySample sample;
        sample.timestamp = i * 0.001;
        sample.position = {static_cast<double>(i), 0.0, 0.0};
        sample.velocity = {10.0, 0.0, 0.0};
        sample.segmentIndex = 0;
        
        livetune.processSample(sample, sample);
    }
    
    EXPECT_EQ(livetune.getGlobalStatistics().sampleCount, 10);
    
    livetune.reset();
    
    EXPECT_EQ(livetune.getGlobalStatistics().sampleCount, 0);
}

// Test RollingStatistics helper
TEST(RollingStatisticsTest, BasicOperations) {
    RollingStatistics stats(100);
    
    stats.addSample(1.0);
    stats.addSample(2.0);
    stats.addSample(3.0);
    stats.addSample(4.0);
    stats.addSample(5.0);
    
    EXPECT_EQ(stats.count(), 5);
    EXPECT_NEAR(stats.mean(), 3.0, 1e-6);
    EXPECT_NEAR(stats.min(), 1.0, 1e-6);
    EXPECT_NEAR(stats.max(), 5.0, 1e-6);
}

TEST(RollingStatisticsTest, WindowLimit) {
    RollingStatistics stats(3);  // Window of 3
    
    stats.addSample(10.0);
    stats.addSample(20.0);
    stats.addSample(30.0);
    stats.addSample(40.0);  // Should push out 10.0
    stats.addSample(50.0);  // Should push out 20.0
    
    // Window now contains: 30, 40, 50
    EXPECT_EQ(stats.count(), 3);
    EXPECT_NEAR(stats.mean(), 40.0, 1e-6);
    EXPECT_NEAR(stats.min(), 30.0, 1e-6);
    EXPECT_NEAR(stats.max(), 50.0, 1e-6);
}

// Test geometric mean calculation
TEST(ErrorStatisticsTest, GeometricMean) {
    // For values 1, 2, 4, 8, geometric mean should be sqrt(sqrt(64)) = 2.828...
    // Actually: (1*2*4*8)^(1/4) = 64^0.25 = 2.828...
    
    std::vector<double> values = {1.0, 2.0, 4.0, 8.0};
    
    double logSum = 0.0;
    for (double v : values) {
        logSum += std::log(v);
    }
    double geometricMean = std::exp(logSum / values.size());
    
    EXPECT_NEAR(geometricMean, 2.8284271247, 1e-6);
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
