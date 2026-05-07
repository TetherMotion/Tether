/**
 * @file MachineTester.hpp
 * @brief Structured machine performance testing framework
 * 
 * Provides comprehensive tests to determine machine capabilities:
 * - Single axis tests: ramps, sinusoids, S-curves
 * - Multi-axis tests: circles, ellipsoids, helixes
 * - System identification: delay, friction, PID tuning
 * - Automated test sequences with varying parameters
 */

#pragma once

#include "MotionReplanner.hpp"
#include "PerformanceHeatmap.hpp"
#include <vector>
#include <array>
#include <memory>
#include <string>
#include <functional>
#include <optional>

namespace MotionReplanner {

//=============================================================================
// Test Types and Configurations
//=============================================================================

/**
 * @brief Types of single-axis tests
 */
enum class SingleAxisTestType {
    Ramp,               ///< Linear ramp up/down
    Sinusoid,           ///< Sinusoidal motion
    SCurve,             ///< S-curve velocity profile
    Step,               ///< Step response for PID analysis
    Triangular,         ///< Triangle wave
    Trapezoidal         ///< Trapezoidal velocity profile
};

/**
 * @brief Types of multi-axis tests
 */
enum class MultiAxisTestType {
    Circle,             ///< Circular motion in plane
    Ellipse,            ///< Elliptical motion
    Helix,              ///< Helical motion (circle + Z)
    Lissajous,          ///< Lissajous curve
    Square,             ///< Square path with corners
    RoundedSquare,      ///< Square with rounded corners
    DiagonalBox         ///< Box with diagonals
};

/**
 * @brief Configuration for a single-axis test
 */
struct SingleAxisTestConfig {
    int axis = 0;                         ///< Axis index (0=X, 1=Y, etc.)
    SingleAxisTestType type = SingleAxisTestType::Sinusoid;
    
    double amplitude = 50.0;              ///< mm
    double frequency = 1.0;               ///< Hz
    double velocity = 1000.0;             ///< mm/min (for ramps)
    double acceleration = 500.0;          ///< mm/s² (for S-curves)
    double jerk = 5000.0;                 ///< mm/s³ (for S-curves)
    
    double duration = 5.0;                ///< seconds
    int cycles = 5;                       ///< Number of cycles (for periodic)
    
    double centerPosition = 0.0;          ///< Center of motion
};

/**
 * @brief Configuration for a multi-axis test
 */
struct MultiAxisTestConfig {
    MultiAxisTestType type = MultiAxisTestType::Circle;
    
    int uAxis = 0;                        ///< First axis (typically X)
    int vAxis = 1;                        ///< Second axis (typically Y)
    int wAxis = 2;                        ///< Third axis for 3D (typically Z)
    
    double radiusU = 50.0;                ///< Radius in U direction
    double radiusV = 50.0;                ///< Radius in V direction (same as U for circle)
    double pitchW = 0.0;                  ///< Pitch per revolution for helix
    
    double rotationAngle = 0.0;           ///< Rotation of pattern (degrees)
    double feedRate = 1000.0;             ///< mm/min
    
    double duration = 10.0;               ///< seconds
    int revolutions = 5;                  ///< Number of revolutions
    
    std::array<double, 3> center = {0.0, 0.0, 0.0};
    
    double cornerRadius = 5.0;            ///< For rounded square
    double lissajousRatioU = 1.0;         ///< Frequency ratio for Lissajous
    double lissajousRatioV = 2.0;
    double lissajousPhase = 90.0;         ///< Phase difference (degrees)
};

/**
 * @brief Configuration for friction identification test
 */
struct FrictionTestConfig {
    int axis = 0;
    
    std::vector<double> velocities = {10, 50, 100, 200, 500, 1000, 2000};  ///< mm/min
    double distance = 100.0;              ///< mm per velocity level
    int repeats = 3;                      ///< Repetitions per velocity
    
    bool bidirectional = true;            ///< Test both directions
    double settleTime = 0.5;              ///< seconds between moves
};

/**
 * @brief Configuration for delay identification test  
 */
struct DelayTestConfig {
    int axis = 0;
    
    double stepAmplitude = 10.0;          ///< mm
    double stepVelocity = 5000.0;         ///< mm/min (fast for clear response)
    int stepCount = 10;                   ///< Number of steps
    double settleTime = 0.2;              ///< seconds between steps
    
    double maxExpectedDelay = 0.01;       ///< seconds (10ms max search)
    double delayResolution = 0.0001;      ///< 0.1ms resolution
};

/**
 * @brief Configuration for PID tuning test
 */
struct PIDTestConfig {
    int axis = 0;
    
    double stepAmplitude = 5.0;           ///< mm
    std::vector<double> stepVelocities = {100, 500, 1000, 2000, 5000};
    
    // Chirp signal for frequency response
    double chirpStartFreq = 0.5;          ///< Hz
    double chirpEndFreq = 50.0;           ///< Hz
    double chirpAmplitude = 2.0;          ///< mm
    double chirpDuration = 20.0;          ///< seconds
    
    int repeats = 3;
};

/**
 * @brief Result of a single test run
 */
struct TestResult {
    std::string testName;
    std::string testType;
    double timestamp = 0.0;
    
    // General metrics
    double maxVelocityAchieved = 0.0;     ///< mm/min
    double maxAccelerationAchieved = 0.0; ///< mm/s²
    double maxJerkAchieved = 0.0;         ///< mm/s³
    
    // Error metrics
    ErrorStatistics positionError;
    ErrorStatistics contourError;         ///< For multi-axis
    
    // Specific test results
    double detectedDelay = 0.0;           ///< For delay tests
    double staticFriction = 0.0;          ///< N or current
    double viscousFriction = 0.0;         ///< N/(mm/s)
    double coulombFriction = 0.0;         ///< N
    
    // PID analysis
    double overshoot = 0.0;               ///< %
    double riseTime = 0.0;                ///< seconds
    double settlingTime = 0.0;            ///< seconds (to 2%)
    double steadyStateError = 0.0;        ///< mm
    
    // Multi-axis circularity
    double circularityError = 0.0;        ///< mm
    double radiusError = 0.0;             ///< mm
    double phaseError = 0.0;              ///< degrees between axes
    
    // Raw data for export
    std::vector<PositionSample> desiredSamples;
    std::vector<PositionSample> actualSamples;
    
    bool passed = true;
    std::string failureReason;
};

//=============================================================================
// Test Sequence
//=============================================================================

/**
 * @brief A sequence of tests to run
 */
struct TestSequence {
    std::string name;
    std::string description;
    
    std::vector<SingleAxisTestConfig> singleAxisTests;
    std::vector<MultiAxisTestConfig> multiAxisTests;
    std::optional<FrictionTestConfig> frictionTest;
    std::optional<DelayTestConfig> delayTest;
    std::optional<PIDTestConfig> pidTest;
    
    // Sweep parameters
    bool sweepAmplitudes = false;
    std::vector<double> amplitudes = {10, 25, 50, 75, 100};
    
    bool sweepFrequencies = false;
    std::vector<double> frequencies = {0.5, 1.0, 2.0, 5.0, 10.0};
    
    bool sweepFeedRates = false;
    std::vector<double> feedRates = {500, 1000, 2000, 3000, 5000};
};

//=============================================================================
// Machine Tester Class
//=============================================================================

/**
 * @brief Main class for machine performance testing
 */
class MachineTester {
public:
    MachineTester();
    
    /**
     * @brief Callbacks for machine interface
     */
    using CommandCallback = std::function<bool(const std::vector<PositionSample>& trajectory)>;
    using FeedbackCallback = std::function<std::vector<PositionSample>()>;
    using StatusCallback = std::function<void(const std::string& message)>;
    
    void setCommandCallback(CommandCallback cb) { commandCallback_ = cb; }
    void setFeedbackCallback(FeedbackCallback cb) { feedbackCallback_ = cb; }
    void setStatusCallback(StatusCallback cb) { statusCallback_ = cb; }
    
    /**
     * @brief Run a single-axis test
     */
    TestResult runSingleAxisTest(const SingleAxisTestConfig& config);
    
    /**
     * @brief Run a multi-axis test
     */
    TestResult runMultiAxisTest(const MultiAxisTestConfig& config);
    
    /**
     * @brief Run friction identification
     */
    TestResult runFrictionTest(const FrictionTestConfig& config);
    
    /**
     * @brief Run delay identification
     */
    TestResult runDelayTest(const DelayTestConfig& config);
    
    /**
     * @brief Run PID tuning analysis
     */
    TestResult runPIDTest(const PIDTestConfig& config);
    
    /**
     * @brief Run a complete test sequence
     */
    std::vector<TestResult> runTestSequence(const TestSequence& sequence);
    
    /**
     * @brief Create standard test sequences
     */
    static TestSequence createQuickCalibration();
    static TestSequence createFullCalibration();
    static TestSequence createAxisCharacterization(int axis);
    static TestSequence createWorkspaceMapping(const HeatmapConfig& heatmapConfig);
    
    /**
     * @brief Get the heatmap built from test results
     */
    const HeatmapBuilder& getHeatmapBuilder() const { return heatmapBuilder_; }
    HeatmapBuilder& heatmapBuilder() { return heatmapBuilder_; }
    
    /**
     * @brief Get all test results
     */
    const std::vector<TestResult>& getResults() const { return results_; }
    
    /**
     * @brief Clear all results
     */
    void clearResults() { results_.clear(); }
    
private:
    CommandCallback commandCallback_;
    FeedbackCallback feedbackCallback_;
    StatusCallback statusCallback_;
    
    HeatmapBuilder heatmapBuilder_;
    std::vector<TestResult> results_;
    
    // Trajectory generation helpers
    std::vector<PositionSample> generateSinusoid(const SingleAxisTestConfig& config);
    std::vector<PositionSample> generateRamp(const SingleAxisTestConfig& config);
    std::vector<PositionSample> generateSCurve(const SingleAxisTestConfig& config);
    std::vector<PositionSample> generateStep(const SingleAxisTestConfig& config);
    std::vector<PositionSample> generateTriangular(const SingleAxisTestConfig& config);
    std::vector<PositionSample> generateTrapezoidal(const SingleAxisTestConfig& config);
    
    std::vector<PositionSample> generateCircle(const MultiAxisTestConfig& config);
    std::vector<PositionSample> generateEllipse(const MultiAxisTestConfig& config);
    std::vector<PositionSample> generateHelix(const MultiAxisTestConfig& config);
    std::vector<PositionSample> generateLissajous(const MultiAxisTestConfig& config);
    std::vector<PositionSample> generateSquare(const MultiAxisTestConfig& config);
    std::vector<PositionSample> generateRoundedSquare(const MultiAxisTestConfig& config);
    
    // Analysis helpers
    void analyzeStepResponse(TestResult& result, 
                            const std::vector<PositionSample>& desired,
                            const std::vector<PositionSample>& actual);
    void analyzeCircularity(TestResult& result,
                           const std::vector<PositionSample>& actual,
                           const MultiAxisTestConfig& config);
    void analyzeFriction(TestResult& result,
                        const std::vector<std::pair<double, std::vector<PositionSample>>>& data);
    double findDelay(const std::vector<PositionSample>& desired,
                    const std::vector<PositionSample>& actual,
                    double maxDelay, double resolution);
    
    void reportStatus(const std::string& message);
};

//=============================================================================
// Friction Model
//=============================================================================

/**
 * @brief Friction model parameters
 */
struct FrictionModel {
    double staticFriction = 0.0;          ///< Force to overcome at standstill
    double coulombFriction = 0.0;         ///< Constant friction force during motion
    double viscousFriction = 0.0;         ///< Velocity-proportional friction
    double stribeckVelocity = 0.0;        ///< Stribeck effect velocity threshold
    double stribeckExponent = 2.0;        ///< Stribeck curve exponent
    
    /**
     * @brief Calculate friction force at a given velocity
     */
    double frictionForce(double velocity) const {
        double absVel = std::abs(velocity);
        
        // Stribeck curve: F = Fc + (Fs - Fc) * exp(-(v/vs)^n) + Fv * v
        double stribeckFactor = std::exp(-std::pow(absVel / stribeckVelocity, stribeckExponent));
        double frictionMag = coulombFriction + 
                            (staticFriction - coulombFriction) * stribeckFactor +
                            viscousFriction * absVel;
        
        return (velocity >= 0) ? frictionMag : -frictionMag;
    }
    
    /**
     * @brief Fit model to velocity-force data
     */
    static FrictionModel fitFromData(
        const std::vector<double>& velocities,
        const std::vector<double>& forces);
};

//=============================================================================
// PID Analysis Results
//=============================================================================

/**
 * @brief Detailed PID controller analysis
 */
struct PIDAnalysis {
    // Time domain metrics
    double riseTime = 0.0;                ///< 10% to 90%
    double settlingTime = 0.0;            ///< To within 2%
    double overshoot = 0.0;               ///< Percentage
    double peakTime = 0.0;                ///< Time to peak
    double steadyStateError = 0.0;
    
    // Frequency domain metrics
    double bandwidth = 0.0;               ///< -3dB bandwidth (Hz)
    double phaseMargin = 0.0;             ///< degrees
    double gainMargin = 0.0;              ///< dB
    double resonanceFreq = 0.0;           ///< Hz
    double resonancePeak = 0.0;           ///< dB
    
    // Suggested improvements
    bool needsMoreP = false;
    bool needsLessP = false;
    bool needsMoreI = false;
    bool needsLessI = false;
    bool needsMoreD = false;
    bool needsLessD = false;
    
    std::string diagnosis;
    std::string recommendation;
};

} // namespace MotionReplanner
