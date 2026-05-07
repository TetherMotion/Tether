/**
 * @file PlotSamplingTests.cpp
 * @brief Unit tests for trajectory sampling functions used in 2D plotting
 * 
 * Tests the efficient sampling, interpolation, and query functions added
 * to support high-quality 2D plot generation in the GCodeFlow UI.
 */

#include <gtest/gtest.h>
#include <cmath>
#include <vector>

// Include the FFI headers (through the GCodeFlow build)
// For testing, we'll include the C++ types directly
#include "tether/gcode/GCodeParser.hpp"
#include "tether/gcode/GCodeTypes.hpp"
#include "tether/gcode/motion/InterpolationStrategy.hpp"

namespace {

/**
 * @brief Helper to create a simple linear motion trajectory
 */
class TrajectoryTestHelper {
public:
    static std::vector<GCode::TrajectoryPoint> createLinearTrajectory(
        double start_x, double end_x,
        double duration,
        size_t num_points)
    {
        std::vector<GCode::TrajectoryPoint> points;
        points.reserve(num_points);
        
        for (size_t i = 0; i < num_points; ++i) {
            GCode::TrajectoryPoint pt;
            double frac = static_cast<double>(i) / (num_points - 1);
            pt.time = frac * duration;
            pt.position.coords[0] = start_x + frac * (end_x - start_x);
            pt.position.coords[1] = 0.0;
            pt.position.coords[2] = 0.0;
            pt.velocity.coords[0] = (end_x - start_x) / duration;
            pt.velocity.coords[1] = 0.0;
            pt.velocity.coords[2] = 0.0;
            pt.acceleration = GCode::Position{};
            pt.blockIndex = 0;
            pt.segmentIndex = 0;
            pt.motionType = GCode::SegmentMotionType::Linear;
            pt.isInterpolated = true;
            
            points.push_back(pt);
        }
        
        return points;
    }
    
    /**
     * @brief Helper to interpolate trajectory at a specific time
     */
    static GCode::TrajectoryPoint queryAtTime(
        const std::vector<GCode::TrajectoryPoint>& points,
        double time_seconds)
    {
        if (points.empty()) {
            return GCode::TrajectoryPoint{};
        }
        
        if (time_seconds <= points.front().time) {
            return points.front();
        }
        
        if (time_seconds >= points.back().time) {
            return points.back();
        }
        
        // Binary search for interval
        size_t left = 0;
        size_t right = points.size() - 1;
        
        while (left < right) {
            size_t mid = (left + right) / 2;
            if (points[mid].time < time_seconds) {
                left = mid + 1;
            } else {
                right = mid;
            }
        }
        
        if (left == 0) {
            return points[0];
        }
        
        const auto& pt1 = points[left - 1];
        const auto& pt2 = points[left];
        
        double dt = pt2.time - pt1.time;
        if (dt < 1e-9) {
            return pt2;
        }
        
        double frac = (time_seconds - pt1.time) / dt;
        
        GCode::TrajectoryPoint result;
        result.time = time_seconds;
        
        for (size_t i = 0; i < GCode::MAX_AXES; ++i) {
            result.position.coords[i] = pt1.position.coords[i] + 
                frac * (pt2.position.coords[i] - pt1.position.coords[i]);
            result.velocity.coords[i] = pt1.velocity.coords[i] + 
                frac * (pt2.velocity.coords[i] - pt1.velocity.coords[i]);
            result.acceleration.coords[i] = pt1.acceleration.coords[i] + 
                frac * (pt2.acceleration.coords[i] - pt1.acceleration.coords[i]);
        }
        
        result.blockIndex = pt1.blockIndex;
        result.segmentIndex = pt1.segmentIndex;
        result.motionType = pt1.motionType;
        result.isInterpolated = true;
        
        return result;
    }
    
    /**
     * @brief Sample trajectory at regular intervals
     */
    static std::vector<GCode::TrajectoryPoint> sampleAtInterval(
        const std::vector<GCode::TrajectoryPoint>& points,
        double interval_seconds)
    {
        std::vector<GCode::TrajectoryPoint> result;
        
        if (points.empty() || interval_seconds <= 0.0) {
            return result;
        }
        
        double current_time = 0.0;
        double end_time = points.back().time;
        
        result.reserve(static_cast<size_t>(end_time / interval_seconds) + 2);
        
        while (current_time <= end_time) {
            result.push_back(queryAtTime(points, current_time));
            current_time += interval_seconds;
        }
        
        // Always include last point
        if (result.empty() || std::abs(result.back().time - end_time) > 1e-6) {
            result.push_back(queryAtTime(points, end_time));
        }
        
        return result;
    }
};

} // anonymous namespace

// =============================================================================
// Test Cases
// =============================================================================

/**
 * @test Verify that queryAtTime returns correct interpolated values
 */
TEST(PlotSamplingTest, QueryAtTimeLinearInterpolation) {
    auto points = TrajectoryTestHelper::createLinearTrajectory(0.0, 100.0, 10.0, 11);
    
    // Query at start
    auto pt_start = TrajectoryTestHelper::queryAtTime(points, 0.0);
    EXPECT_NEAR(pt_start.position.coords[0], 0.0, 1e-6);
    EXPECT_NEAR(pt_start.time, 0.0, 1e-6);
    
    // Query at midpoint
    auto pt_mid = TrajectoryTestHelper::queryAtTime(points, 5.0);
    EXPECT_NEAR(pt_mid.position.coords[0], 50.0, 1e-6);
    EXPECT_NEAR(pt_mid.time, 5.0, 1e-6);
    
    // Query at end
    auto pt_end = TrajectoryTestHelper::queryAtTime(points, 10.0);
    EXPECT_NEAR(pt_end.position.coords[0], 100.0, 1e-6);
    EXPECT_NEAR(pt_end.time, 10.0, 1e-6);
    
    // Query at intermediate point (t=3.5)
    auto pt_inter = TrajectoryTestHelper::queryAtTime(points, 3.5);
    EXPECT_NEAR(pt_inter.position.coords[0], 35.0, 1e-6);
    EXPECT_NEAR(pt_inter.time, 3.5, 1e-6);
}

/**
 * @test Verify queryAtTime clamps to valid range
 */
TEST(PlotSamplingTest, QueryAtTimeClampsBounds) {
    auto points = TrajectoryTestHelper::createLinearTrajectory(0.0, 100.0, 10.0, 11);
    
    // Query before start
    auto pt_before = TrajectoryTestHelper::queryAtTime(points, -1.0);
    EXPECT_NEAR(pt_before.position.coords[0], 0.0, 1e-6);
    EXPECT_NEAR(pt_before.time, 0.0, 1e-6);
    
    // Query after end
    auto pt_after = TrajectoryTestHelper::queryAtTime(points, 15.0);
    EXPECT_NEAR(pt_after.position.coords[0], 100.0, 1e-6);
    EXPECT_NEAR(pt_after.time, 10.0, 1e-6);
}

/**
 * @test Verify sampleAtInterval produces correct number of samples
 */
TEST(PlotSamplingTest, SampleAtIntervalCount) {
    auto points = TrajectoryTestHelper::createLinearTrajectory(0.0, 100.0, 10.0, 11);
    
    // Sample at 1 second intervals
    auto samples = TrajectoryTestHelper::sampleAtInterval(points, 1.0);
    EXPECT_EQ(samples.size(), 11); // 0, 1, 2, ..., 10
    
    // Sample at 0.5 second intervals
    samples = TrajectoryTestHelper::sampleAtInterval(points, 0.5);
    EXPECT_EQ(samples.size(), 21); // 0, 0.5, 1.0, ..., 10.0
    
    // Sample at 2.5 second intervals
    samples = TrajectoryTestHelper::sampleAtInterval(points, 2.5);
    EXPECT_EQ(samples.size(), 5); // 0, 2.5, 5.0, 7.5, 10.0
}

/**
 * @test Verify sampleAtInterval produces correct values
 */
TEST(PlotSamplingTest, SampleAtIntervalValues) {
    auto points = TrajectoryTestHelper::createLinearTrajectory(0.0, 100.0, 10.0, 11);
    auto samples = TrajectoryTestHelper::sampleAtInterval(points, 2.0);
    
    ASSERT_GE(samples.size(), 6);
    
    EXPECT_NEAR(samples[0].position.coords[0], 0.0, 1e-6);
    EXPECT_NEAR(samples[1].position.coords[0], 20.0, 1e-6);
    EXPECT_NEAR(samples[2].position.coords[0], 40.0, 1e-6);
    EXPECT_NEAR(samples[3].position.coords[0], 60.0, 1e-6);
    EXPECT_NEAR(samples[4].position.coords[0], 80.0, 1e-6);
    EXPECT_NEAR(samples[5].position.coords[0], 100.0, 1e-6);
}

/**
 * @test Verify velocity interpolation is correct
 */
TEST(PlotSamplingTest, VelocityInterpolation) {
    auto points = TrajectoryTestHelper::createLinearTrajectory(0.0, 100.0, 10.0, 11);
    
    // Expected velocity: 100 mm / 10 s = 10 mm/s
    double expected_vel = 10.0;
    
    auto pt_mid = TrajectoryTestHelper::queryAtTime(points, 5.0);
    EXPECT_NEAR(pt_mid.velocity.coords[0], expected_vel, 1e-6);
    
    // Sample at various times
    auto samples = TrajectoryTestHelper::sampleAtInterval(points, 1.0);
    for (const auto& sample : samples) {
        EXPECT_NEAR(sample.velocity.coords[0], expected_vel, 1e-6);
    }
}

/**
 * @test Verify empty trajectory handling
 */
TEST(PlotSamplingTest, EmptyTrajectoryHandling) {
    std::vector<GCode::TrajectoryPoint> empty_points;
    
    auto result = TrajectoryTestHelper::queryAtTime(empty_points, 5.0);
    EXPECT_EQ(result.time, 0.0);
    
    auto samples = TrajectoryTestHelper::sampleAtInterval(empty_points, 1.0);
    EXPECT_TRUE(samples.empty());
}

/**
 * @test Verify single point trajectory
 */
TEST(PlotSamplingTest, SinglePointTrajectory) {
    std::vector<GCode::TrajectoryPoint> points(1);
    points[0].time = 0.0;
    points[0].position.coords[0] = 50.0;
    points[0].position.coords[1] = 0.0;
    points[0].position.coords[2] = 0.0;
    points[0].velocity = GCode::Position{};
    points[0].acceleration = GCode::Position{};
    points[0].blockIndex = 0;
    points[0].segmentIndex = 0;
    points[0].motionType = GCode::SegmentMotionType::Linear;
    
    auto result = TrajectoryTestHelper::queryAtTime(points, 0.0);
    EXPECT_NEAR(result.position.coords[0], 50.0, 1e-6);
    
    auto samples = TrajectoryTestHelper::sampleAtInterval(points, 1.0);
    EXPECT_EQ(samples.size(), 1);
    EXPECT_NEAR(samples[0].position.coords[0], 50.0, 1e-6);
}

/**
 * @test Verify precision of interpolation
 */
TEST(PlotSamplingTest, InterpolationPrecision) {
    auto points = TrajectoryTestHelper::createLinearTrajectory(0.0, 1000.0, 100.0, 101);
    
    // Test many intermediate points
    for (double t = 0.0; t <= 100.0; t += 0.1) {
        auto pt = TrajectoryTestHelper::queryAtTime(points, t);
        double expected_pos = t * 10.0; // velocity is 10 mm/s
        EXPECT_NEAR(pt.position.coords[0], expected_pos, 0.01);
    }
}

/**
 * @test Verify block index preservation during interpolation
 */
TEST(PlotSamplingTest, BlockIndexPreservation) {
    auto points = TrajectoryTestHelper::createLinearTrajectory(0.0, 100.0, 10.0, 11);
    
    // Set different block indices
    for (size_t i = 0; i < points.size(); ++i) {
        points[i].blockIndex = static_cast<int32_t>(i / 2);
    }
    
    // Query should return block index from previous point
    auto pt = TrajectoryTestHelper::queryAtTime(points, 5.5);
    EXPECT_EQ(pt.blockIndex, 2); // Should be from point at index 5
}

/**
 * @test Verify multi-axis interpolation
 */
TEST(PlotSamplingTest, MultiAxisInterpolation) {
    std::vector<GCode::TrajectoryPoint> points(11);
    
    for (size_t i = 0; i < points.size(); ++i) {
        double frac = static_cast<double>(i) / 10.0;
        points[i].time = frac * 10.0;
        points[i].position.coords[0] = frac * 100.0; // X: 0 -> 100
        points[i].position.coords[1] = frac * 50.0;  // Y: 0 -> 50
        points[i].position.coords[2] = frac * 25.0;  // Z: 0 -> 25
        points[i].blockIndex = 0;
        points[i].segmentIndex = 0;
        points[i].motionType = GCode::SegmentMotionType::Linear;
    }
    
    auto pt = TrajectoryTestHelper::queryAtTime(points, 5.0);
    EXPECT_NEAR(pt.position.coords[0], 50.0, 1e-6);
    EXPECT_NEAR(pt.position.coords[1], 25.0, 1e-6);
    EXPECT_NEAR(pt.position.coords[2], 12.5, 1e-6);
}

/**
 * @test Performance test for large trajectories
 */
TEST(PlotSamplingTest, PerformanceLargeTrajectory) {
    // Create a large trajectory (10000 points)
    auto points = TrajectoryTestHelper::createLinearTrajectory(0.0, 10000.0, 1000.0, 10000);
    
    // Sample at high frequency (1000 samples)
    auto start = std::chrono::high_resolution_clock::now();
    auto samples = TrajectoryTestHelper::sampleAtInterval(points, 1.0);
    auto end = std::chrono::high_resolution_clock::now();
    
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    
    EXPECT_EQ(samples.size(), 1001);
    EXPECT_LT(duration.count(), 100); // Should complete in < 100ms
    
    // Verify correctness of samples
    for (size_t i = 0; i < std::min(samples.size(), size_t(10)); ++i) {
        EXPECT_NEAR(samples[i].position.coords[0], static_cast<double>(i) * 10.0, 1e-6);
    }
}

/**
 * @test Test sampling with very small intervals
 */
TEST(PlotSamplingTest, SmallIntervalSampling) {
    auto points = TrajectoryTestHelper::createLinearTrajectory(0.0, 10.0, 1.0, 11);
    
    // Sample at 0.001 second intervals (1ms)
    auto samples = TrajectoryTestHelper::sampleAtInterval(points, 0.001);
    
    EXPECT_GE(samples.size(), 1000);
    
    // Verify first few samples
    for (size_t i = 0; i < std::min(samples.size(), size_t(10)); ++i) {
        double expected_pos = static_cast<double>(i) * 0.01;
        EXPECT_NEAR(samples[i].position.coords[0], expected_pos, 1e-3);
    }
}

