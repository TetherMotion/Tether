/**
 * @file TrajectoryAnalyzer.hpp
 * @brief Trajectory analysis for velocity, acceleration, and jerk computation
 * 
 * Provides comprehensive analysis of trajectory points including:
 * - Per-axis velocity, acceleration, and jerk computation
 * - Kinematic limit compliance checking
 * - Statistical analysis
 */

#pragma once

#include "gcode/motion/InterpolationStrategy.hpp"
#include <vector>
#include <array>
#include <string>
#include <functional>
#include <optional>

struct GCodeMotionSegment;

namespace GCodeExport {

/**
 * @brief A single trajectory sample with all derivatives
 */
struct TrajectorySample {
    double time = 0.0;                          ///< Time from start (seconds)
    double pathPosition = 0.0;                  ///< Arc length from start (mm)
    
    // Per-axis data (X, Y, Z, A, B, C, U, V, W)
    std::array<double, 9> position{};           ///< Position (mm or deg)
    std::array<double, 9> velocity{};           ///< Velocity (mm/s or deg/s)
    std::array<double, 9> acceleration{};       ///< Acceleration (mm/s² or deg/s²)
    std::array<double, 9> jerk{};               ///< Jerk (mm/s³ or deg/s³)
    
    // Combined metrics
    double linearVelocity = 0.0;                ///< 3D linear velocity magnitude (mm/s)
    double linearAcceleration = 0.0;            ///< 3D linear acceleration magnitude
    double linearJerk = 0.0;                    ///< 3D linear jerk magnitude
    double curvature = 0.0;                     ///< Local curvature (1/mm)
    double centripetalAccel = 0.0;              ///< Centripetal acceleration
    
    // Segment info
    int32_t segmentIndex = -1;
    int32_t blockIndex = -1;
    uint8_t motionType = 0;                     ///< 0=rapid, 1=linear, 2=arcCW, 3=arcCCW
};

/**
 * @brief Violation record for limit compliance checking
 */
struct LimitViolation {
    double time;
    int axis;                                   ///< -1 for combined, 0-8 for individual axes
    std::string limitType;                      ///< "velocity", "acceleration", "jerk"
    double value;
    double limit;
    double overshoot;                           ///< Percentage over limit
};

/**
 * @brief Statistics for a trajectory
 */
struct TrajectoryStatistics {
    double duration = 0.0;                      ///< Total time (seconds)
    double pathLength = 0.0;                    ///< Total path length (mm)
    size_t sampleCount = 0;
    
    // Per-axis statistics
    struct AxisStats {
        double minPosition = 0.0;
        double maxPosition = 0.0;
        double minVelocity = 0.0;
        double maxVelocity = 0.0;
        double minAcceleration = 0.0;
        double maxAcceleration = 0.0;
        double minJerk = 0.0;
        double maxJerk = 0.0;
        double avgVelocity = 0.0;
        double avgAcceleration = 0.0;
    };
    std::array<AxisStats, 9> axisStats{};
    
    // Combined statistics
    double maxLinearVelocity = 0.0;
    double maxLinearAcceleration = 0.0;
    double maxLinearJerk = 0.0;
    double maxCurvature = 0.0;
    double maxCentripetalAccel = 0.0;
    
    // Corner error analysis
    struct CornerError {
        size_t cornerIndex = 0;                 ///< Index of corner in segment sequence
        double idealCornerX = 0.0;              ///< Ideal (programmed) corner X
        double idealCornerY = 0.0;              ///< Ideal (programmed) corner Y
        double idealCornerZ = 0.0;              ///< Ideal (programmed) corner Z
        double maxDeviation = 0.0;              ///< Maximum deviation from ideal path (mm)
        double insideDeviation = 0.0;           ///< Maximum deviation toward inside (mm)
        double outsideDeviation = 0.0;          ///< Maximum deviation toward outside (mm)
        double minDistanceFromCorner = 0.0;     ///< Closest approach to corner vertex
        double cornerAngle = 0.0;               ///< Corner angle (degrees)
        double entryVelocity = 0.0;             ///< Velocity entering corner blend
        double exitVelocity = 0.0;              ///< Velocity exiting corner blend
        double minVelocity = 0.0;               ///< Minimum velocity during corner
    };
    std::vector<CornerError> cornerErrors;
    double totalCornerError = 0.0;              ///< Sum of all corner deviations
    double maxCornerError = 0.0;                ///< Maximum single corner deviation
    
    // Limit violations
    std::vector<LimitViolation> violations;
    bool meetsLimits = true;
};

/**
 * @brief Configuration for trajectory analysis
 */
struct AnalysisConfig {
    // Sampling
    double timeStep = 0.001;                    ///< Sample time step (seconds)
    bool useAdaptiveSampling = false;
    double maxChordError = 0.005;               ///< For adaptive sampling (mm)
    
    // Derivative computation
    int derivativeOrder = 4;                    ///< Central difference order (2, 4, 6)
    
    // Kinematic limits for compliance checking
    GCode::KinematicLimits limits;
    
    // Tolerance for violation detection
    double violationTolerance = 0.01;           ///< 1% over limit before flagging
};

/**
 * @brief Analyzes trajectories and computes all derivatives
 */
class TrajectoryAnalyzer {
public:
    explicit TrajectoryAnalyzer(const AnalysisConfig& config = {});
    
    /**
     * @brief Analyze segments and generate trajectory samples
     * @param segments Motion segments from interpreter
     * @param strategy Interpolation strategy to use
     * @return Vector of trajectory samples with all derivatives
     */
    std::vector<TrajectorySample> analyze(
        const std::vector<GCode::PlanningSegment>& segments,
        GCode::InterpolationStrategy* strategy = nullptr
    );
    
    /**
     * @brief Analyze from C API segments
     */
    std::vector<TrajectorySample> analyzeFromCAPI(
        const struct ::GCodeMotionSegment* segments,
        size_t segmentCount
    );
    
    /**
     * @brief Compute statistics for trajectory samples
     */
    TrajectoryStatistics computeStatistics(const std::vector<TrajectorySample>& samples);
    
    /**
     * @brief Check if trajectory meets kinematic limits
     * @param samples Trajectory samples
     * @param[out] violations Optional output for violation details
     * @return true if all limits are met
     */
    bool checkLimitCompliance(
        const std::vector<TrajectorySample>& samples,
        std::vector<LimitViolation>* violations = nullptr
    );
    
    /**
     * @brief Configure analyzer
     */
    void configure(const AnalysisConfig& config) { config_ = config; }
    const AnalysisConfig& config() const { return config_; }
    
private:
    AnalysisConfig config_;

public:
    /**
     * @brief Compute derivatives using finite differences
     * @param samples Input samples (positions must be filled)
     * @param order Derivative order (2, 4, or 6-point)
     */
    void computeDerivatives(std::vector<TrajectorySample>& samples, int order);
    
    /**
     * @brief Compute combined metrics (linear velocity, curvature, etc.)
     */
    void computeCombinedMetrics(std::vector<TrajectorySample>& samples);
    
    /**
     * @brief 4th order central difference coefficients for 1st derivative
     */
    static constexpr std::array<double, 5> CD4_COEFF1 = {1.0/12.0, -2.0/3.0, 0.0, 2.0/3.0, -1.0/12.0};
    
    /**
     * @brief 4th order central difference coefficients for 2nd derivative
     */
    static constexpr std::array<double, 5> CD4_COEFF2 = {-1.0/12.0, 4.0/3.0, -5.0/2.0, 4.0/3.0, -1.0/12.0};
};

/**
 * @brief Approximation strategy interface for trajectory generation
 */
class ApproximationStrategy {
public:
    virtual ~ApproximationStrategy() = default; // GCOVR_EXCL_LINE
    
    /**
     * @brief Get strategy name
     */
    virtual const char* name() const = 0;
    
    /**
     * @brief Generate trajectory samples from segments
     */
    virtual std::vector<TrajectorySample> generateTrajectory(
        const std::vector<GCode::PlanningSegment>& segments,
        const GCode::KinematicLimits& limits
    ) = 0;
    
    /**
     * @brief Configure strategy-specific parameters
     */
    virtual void configure(const std::string& key, double value) = 0;
};

/**
 * @brief Fixed time step approximation
 */
class FixedTimeApproximation : public ApproximationStrategy {
public:
    const char* name() const override { return "FixedTime"; } // GCOVR_EXCL_LINE
    
    std::vector<TrajectorySample> generateTrajectory(
        const std::vector<GCode::PlanningSegment>& segments,
        const GCode::KinematicLimits& limits
    ) override;
    
    void configure(const std::string& key, double value) override;
    
    void setTimeStep(double dt) { timeStep_ = dt; }
    
private:
    double timeStep_ = 0.001;  // 1ms default
};

/**
 * @brief Fixed deviation (chord error) approximation
 */
class FixedDeviationApproximation : public ApproximationStrategy {
public:
    const char* name() const override { return "FixedDeviation"; } // GCOVR_EXCL_LINE
    
    std::vector<TrajectorySample> generateTrajectory(
        const std::vector<GCode::PlanningSegment>& segments,
        const GCode::KinematicLimits& limits
    ) override;
    
    void configure(const std::string& key, double value) override;
    
    void setMaxDeviation(double deviation) { maxDeviation_ = deviation; }
    
private:
    double maxDeviation_ = 0.005;  // 5µm default
};

/**
 * @brief Trapezoidal velocity profile with jerk limiting
 */
class TrapezoidalApproximation : public ApproximationStrategy {
public:
    const char* name() const override { return "Trapezoidal"; } // GCOVR_EXCL_LINE
    
    std::vector<TrajectorySample> generateTrajectory(
        const std::vector<GCode::PlanningSegment>& segments,
        const GCode::KinematicLimits& limits
    ) override;
    
    void configure(const std::string& key, double value) override;
    
private:
    double timeStep_ = 0.001;
    bool useJerkLimiting_ = true;
};

/**
 * @brief S-curve (7-segment) velocity profile
 */
class SCurveApproximation : public ApproximationStrategy {
public:
    const char* name() const override { return "SCurve"; } // GCOVR_EXCL_LINE
    
    std::vector<TrajectorySample> generateTrajectory(
        const std::vector<GCode::PlanningSegment>& segments,
        const GCode::KinematicLimits& limits
    ) override;
    
    void configure(const std::string& key, double value) override;
    
private:
    double timeStep_ = 0.001;
};

/**
 * @brief Factory for creating approximation strategies
 */
class ApproximationFactory {
public:
    static std::unique_ptr<ApproximationStrategy> create(const std::string& name);
    
    static std::vector<std::string> availableStrategies() {
        return {"FixedTime", "FixedDeviation", "Trapezoidal", "SCurve"};
    }
};

} // namespace GCodeExport
