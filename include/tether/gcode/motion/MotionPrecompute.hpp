/**
 * @file MotionPrecompute.hpp
 * @brief Motion Precomputation Framework
 * 
 * Framework for precomputing motion trajectory points with timing tracking
 * and multiple interpolation strategies.
 */

#pragma once

#include "../GCodeMath.hpp"
#include <vector>
#include <functional>
#include <chrono>
#include <memory>
#include <string>

namespace GCode {
namespace Motion {

using namespace Math;

// ============================================================================
// Interpolation Strategy Interface
// ============================================================================

/**
 * @brief Base class for interpolation strategies
 */
class InterpolationStrategy {
public:
    virtual ~InterpolationStrategy() = default;
    
    /**
     * @brief Get the name of this strategy
     */
    virtual std::string name() const = 0;
    
    /**
     * @brief Interpolate points along a linear segment
     * 
     * @param start Start position
     * @param end End position
     * @param feedRate Feed rate in mm/min
     * @param output Vector to append points to
     */
    virtual void interpolateLinear(
        const Vec3& start,
        const Vec3& end,
        double feedRate,
        std::vector<TrajectoryPoint>& output
    ) = 0;
    
    /**
     * @brief Interpolate points along an arc segment
     * 
     * @param start Start position
     * @param end End position
     * @param center Arc center
     * @param radius Arc radius
     * @param sweep Sweep angle in radians
     * @param plane Arc plane (XY=0, XZ=1, YZ=2)
     * @param feedRate Feed rate in mm/min
     * @param output Vector to append points to
     */
    virtual void interpolateArc(
        const Vec3& start,
        const Vec3& end,
        const Vec3& center,
        double radius,
        double sweep,
        int plane,
        double feedRate,
        std::vector<TrajectoryPoint>& output
    ) = 0;
};

// ============================================================================
// Fixed Time Step Strategy
// ============================================================================

/**
 * @brief Interpolation with fixed time intervals
 */
class FixedTimeStrategy : public InterpolationStrategy {
public:
    explicit FixedTimeStrategy(double timeStep = 0.01)
        : timeStep_(timeStep) {}
    
    std::string name() const override { return "FixedTime"; }
    
    void interpolateLinear(
        const Vec3& start,
        const Vec3& end,
        double feedRate,
        std::vector<TrajectoryPoint>& output
    ) override;
    
    void interpolateArc(
        const Vec3& start,
        const Vec3& end,
        const Vec3& center,
        double radius,
        double sweep,
        int plane,
        double feedRate,
        std::vector<TrajectoryPoint>& output
    ) override;
    
    void setTimeStep(double ts) { timeStep_ = ts; }
    double timeStep() const { return timeStep_; }
    
private:
    double timeStep_;
};

// ============================================================================
// Fixed Deviation Strategy
// ============================================================================

/**
 * @brief Interpolation with fixed maximum deviation from ideal path
 */
class FixedDeviationStrategy : public InterpolationStrategy {
public:
    explicit FixedDeviationStrategy(double maxDeviation = 0.1)
        : maxDeviation_(maxDeviation) {}
    
    std::string name() const override { return "FixedDeviation"; }
    
    void interpolateLinear(
        const Vec3& start,
        const Vec3& end,
        double feedRate,
        std::vector<TrajectoryPoint>& output
    ) override;
    
    void interpolateArc(
        const Vec3& start,
        const Vec3& end,
        const Vec3& center,
        double radius,
        double sweep,
        int plane,
        double feedRate,
        std::vector<TrajectoryPoint>& output
    ) override;
    
    void setMaxDeviation(double dev) { maxDeviation_ = dev; }
    double maxDeviation() const { return maxDeviation_; }
    
private:
    double maxDeviation_;
};

// ============================================================================
// Adaptive Strategy (RKF45-like)
// ============================================================================

/**
 * @brief Adaptive interpolation that adjusts step size based on curvature/acceleration
 */
class AdaptiveStrategy : public InterpolationStrategy {
public:
    AdaptiveStrategy(double minStep = 0.001, double maxStep = 0.1, double tolerance = 0.01)
        : minStep_(minStep), maxStep_(maxStep), tolerance_(tolerance) {}
    
    std::string name() const override { return "Adaptive"; }
    
    void interpolateLinear(
        const Vec3& start,
        const Vec3& end,
        double feedRate,
        std::vector<TrajectoryPoint>& output
    ) override;
    
    void interpolateArc(
        const Vec3& start,
        const Vec3& end,
        const Vec3& center,
        double radius,
        double sweep,
        int plane,
        double feedRate,
        std::vector<TrajectoryPoint>& output
    ) override;
    
private:
    double minStep_;
    double maxStep_;
    double tolerance_;
};

// ============================================================================
// Timing Statistics
// ============================================================================

/**
 * @brief Statistics for precomputation timing
 */
struct PrecomputeStats {
    std::chrono::microseconds totalTime{0};
    std::chrono::microseconds parseTime{0};
    std::chrono::microseconds interpolationTime{0};
    std::chrono::microseconds postProcessTime{0};
    
    size_t inputSegments = 0;
    size_t outputPoints = 0;
    size_t linearSegments = 0;
    size_t arcSegments = 0;
    
    double pointsPerSecond() const {
        if (totalTime.count() == 0) return 0.0;
        return static_cast<double>(outputPoints) * 1e6 / totalTime.count();
    }
    
    std::string summary() const;
};

// ============================================================================
// Motion Segment Input
// ============================================================================

/**
 * @brief Motion segment for precomputation input
 */
struct MotionSegment {
    Vec3 start;
    Vec3 end;
    Vec3 center;      // For arcs
    double radius = 0.0;
    double sweep = 0.0;
    double feedRate = 1000.0;  // mm/min
    int plane = 0;    // 0=XY, 1=XZ, 2=YZ
    int blockIndex = -1;
    bool isRapid = false;
    bool isArc = false;
};

// ============================================================================
// Motion Precomputer
// ============================================================================

/**
 * @brief Main precomputation engine
 */
class MotionPrecomputer {
public:
    MotionPrecomputer();
    ~MotionPrecomputer();
    
    /**
     * @brief Set the interpolation strategy
     */
    void setStrategy(std::unique_ptr<InterpolationStrategy> strategy);
    
    /**
     * @brief Get the current strategy name
     */
    std::string strategyName() const;
    
    /**
     * @brief Precompute trajectory from motion segments
     * 
     * @param segments Input motion segments
     * @return Precomputed trajectory points
     */
    std::vector<TrajectoryPoint> precompute(const std::vector<MotionSegment>& segments);
    
    /**
     * @brief Get statistics from last precomputation
     */
    const PrecomputeStats& stats() const { return stats_; }
    
    /**
     * @brief Set rapid feed rate (used when segment has isRapid=true and feedRate=0)
     */
    void setRapidFeedRate(double rate) { rapidFeedRate_ = rate; }
    
    /**
     * @brief Enable/disable velocity calculation
     */
    void setCalculateVelocity(bool enable) { calcVelocity_ = enable; }
    
    /**
     * @brief Enable/disable acceleration calculation
     */
    void setCalculateAcceleration(bool enable) { calcAcceleration_ = enable; }
    
private:
    std::unique_ptr<InterpolationStrategy> strategy_;
    PrecomputeStats stats_;
    double rapidFeedRate_ = 6000.0;  // mm/min
    bool calcVelocity_ = true;
    bool calcAcceleration_ = true;
    
    void calculateDerivatives(std::vector<TrajectoryPoint>& points);
};

// ============================================================================
// Factory Functions
// ============================================================================

/**
 * @brief Create an interpolation strategy by name
 * 
 * @param name Strategy name: "FixedTime", "FixedDeviation", "Adaptive"
 * @param param Primary parameter (time step, deviation, or tolerance)
 * @return Unique pointer to the strategy
 */
std::unique_ptr<InterpolationStrategy> createStrategy(
    const std::string& name,
    double param = 0.01
);

} // namespace Motion
} // namespace GCode
