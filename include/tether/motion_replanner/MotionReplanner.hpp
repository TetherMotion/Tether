/**
 * @file MotionReplanner.hpp
 * @brief Comprehensive online motion replanning based on closed-loop feedback
 * 
 * Features:
 * - Closed-loop feedback processing with configurable system delay compensation
 * - Error statistics computation (min/max/mean/geometric mean/std dev)
 * - Critical point detection (corners, direction changes, etc.)
 * - Performance monitoring mode
 * - Parameter suggestion and adaptive limit adjustment
 * - C2-smooth limit transitions for safe online adjustments
 */

#pragma once

#include "tether/export/TrajectoryAnalyzer.hpp"
#include <vector>
#include <deque>
#include <array>
#include <memory>
#include <functional>
#include <optional>
#include <map>
#include <mutex>
#include <cmath>

namespace MotionReplanner {

//=============================================================================
// Data Structures
//=============================================================================

/**
 * @brief A timestamped position sample from the control system
 */
struct PositionSample {
    double timestamp = 0.0;               ///< System time (seconds)
    std::array<double, 9> position{};     ///< Position (X,Y,Z,A,B,C,U,V,W)
    std::array<double, 9> velocity{};     ///< Velocity (optional, may be derived)
    bool velocityValid = false;           ///< Whether velocity was measured or derived
};

/**
 * @brief Tracking error at a single point
 */
struct TrackingError {
    double timestamp = 0.0;
    double pathPosition = 0.0;            ///< Arc length from start
    
    std::array<double, 9> positionError{};
    std::array<double, 9> velocityError{};
    
    double combinedPositionError = 0.0;   ///< 3D position error magnitude
    double contourError = 0.0;            ///< Error perpendicular to path
    double lagError = 0.0;                ///< Error along path direction
    
    int32_t segmentIndex = -1;
    int32_t blockIndex = -1;
    bool isCriticalPoint = false;         ///< Corner, direction change, etc.
    std::string criticalPointType;        ///< "corner", "arc_start", "arc_end", etc.
};

/**
 * @brief Statistical summary of errors
 */
struct ErrorStatistics {
    double minError = 0.0;
    double maxError = 0.0;
    double meanError = 0.0;
    double geometricMean = 0.0;
    double stdDev = 0.0;
    double rmsError = 0.0;
    double p95Error = 0.0;                ///< 95th percentile
    double p99Error = 0.0;                ///< 99th percentile
    size_t sampleCount = 0;
    
    // For critical points specifically
    double maxCornerError = 0.0;
    double meanCornerError = 0.0;
    size_t cornerCount = 0;
};

/**
 * @brief Segment-level performance analysis
 */
struct SegmentPerformance {
    int32_t segmentIndex = -1;
    int32_t blockIndex = -1;
    
    // Error statistics for this segment
    ErrorStatistics positionStats;
    ErrorStatistics contourStats;
    ErrorStatistics lagStats;
    
    // Achieved vs commanded
    double commandedFeedRate = 0.0;       ///< mm/min
    double achievedMeanFeedRate = 0.0;
    double achievedMinFeedRate = 0.0;
    double achievedMaxFeedRate = 0.0;
    
    double commandedAccel = 0.0;
    double achievedMaxAccel = 0.0;
    
    // Performance ratio (0-1, where 1 = perfect)
    double feedRateRatio = 1.0;
    double accuracyScore = 1.0;           ///< Based on contour error
    double overallScore = 1.0;
    
    // Suggested limits for this segment
    double suggestedFeedRate = 0.0;
    double suggestedAccel = 0.0;
    double suggestedJerk = 0.0;
    bool limitAdjustmentNeeded = false;
};

/**
 * @brief Suggested parameter change with smooth transition
 */
struct ParameterSuggestion {
    int32_t segmentIndex = -1;
    double startPathPosition = 0.0;
    double endPathPosition = 0.0;
    
    // Current values
    double currentFeedRate = 0.0;
    double currentAccel = 0.0;
    double currentJerk = 0.0;
    
    // Suggested values
    double suggestedFeedRate = 0.0;
    double suggestedAccel = 0.0;
    double suggestedJerk = 0.0;
    
    // Transition parameters for C2 smoothness
    double transitionLength = 0.0;        ///< mm, for smooth ramp
    double transitionTime = 0.0;          ///< seconds
    
    std::string reason;
    double confidenceScore = 0.0;         ///< 0-1
    bool limitAdjustmentNeeded = false;
};

/**
 * @brief Configuration for the motion replanner
 */
struct ReplannerConfig {
    // System delay compensation
    double systemDelay = 0.001;           ///< Constant system delay (seconds)
    bool autoDetectDelay = false;         ///< Enable automatic delay detection
    double delayDetectionWindow = 0.1;    ///< Window for cross-correlation (seconds)
    
    // Error thresholds
    double positionErrorThreshold = 0.05; ///< mm - flag if exceeded
    double contourErrorThreshold = 0.02;  ///< mm - critical for surface finish
    double lagErrorThreshold = 0.1;       ///< mm - acceptable along-path error
    
    // Corner detection
    double cornerAngleThreshold = 10.0;   ///< degrees - angle change to be a corner
    double cornerProximity = 0.5;         ///< mm - region around corner for analysis
    
    // Limit adjustment
    bool enableLimitSuggestions = true;
    bool enableAutoAdjustment = false;    ///< Actually modify limits online
    double maxFeedRateIncrease = 1.2;     ///< Max factor to increase (e.g., 1.2 = 20%)
    double maxFeedRateDecrease = 0.5;     ///< Min factor (0.5 = reduce to 50%)
    double safetyMargin = 0.1;            ///< 10% margin from measured capability
    
    // Maximum allowed limits (user configured)
    double absoluteMaxVelocity = 10000.0; ///< mm/min
    double absoluteMaxAccel = 5000.0;     ///< mm/s²
    double absoluteMaxJerk = 50000.0;     ///< mm/s³
    
    // Smoothing for online adjustments
    double limitTransitionTime = 0.1;     ///< seconds for C2-smooth transitions
    double minTransitionLength = 5.0;     ///< mm minimum transition length
    
    // Statistical analysis
    size_t minSamplesForStatistics = 10;
    double statisticsWindowTime = 1.0;    ///< Rolling window for live stats
    
    // Monitoring mode
    bool monitoringOnly = true;           ///< Only monitor, don't suggest changes
    bool logAllSamples = false;           ///< Keep all samples (memory intensive)
};

//=============================================================================
// Motion Replanner Class
//=============================================================================

/**
 * @brief Online motion replanner with closed-loop feedback analysis
 */
class MotionReplanner {
public:
    explicit MotionReplanner(const ReplannerConfig& config = {});
    
    /**
     * @brief Set the desired trajectory (from G-code)
     */
    void setDesiredTrajectory(const std::vector<GCodeExport::TrajectorySample>& desired);
    
    /**
     * @brief Add a new actual position sample (called at each control cycle)
     * @param sample The measured position with timestamp
     */
    void addActualSample(const PositionSample& sample);
    
    /**
     * @brief Process accumulated samples and compute errors
     * @return Number of new error samples computed
     */
    size_t processAccumulatedSamples();
    
    /**
     * @brief Get current tracking error (most recent)
     */
    std::optional<TrackingError> getCurrentError() const;
    
    /**
     * @brief Get error statistics for the entire trajectory so far
     */
    ErrorStatistics getOverallStatistics() const;
    
    /**
     * @brief Get error statistics for critical points only
     */
    ErrorStatistics getCriticalPointStatistics() const;
    
    /**
     * @brief Get per-segment performance analysis
     */
    std::vector<SegmentPerformance> getSegmentPerformance() const;
    
    /**
     * @brief Get parameter suggestions for problem segments
     */
    std::vector<ParameterSuggestion> getParameterSuggestions() const;
    
    /**
     * @brief Get suggested limits at a specific path position
     * @param pathPosition Arc length from start
     * @param useSmoothTransition Apply C2-smooth transition
     */
    struct Limits {
        double feedRate;
        double acceleration;
        double jerk;
    };
    Limits getSuggestedLimits(double pathPosition, bool useSmoothTransition = true) const;
    
    /**
     * @brief Get the detected/configured system delay
     */
    double getSystemDelay() const { return detectedDelay_; }
    
    /**
     * @brief Force delay detection from current data
     * @return Detected delay in seconds
     */
    double detectSystemDelay();
    
    /**
     * @brief Reset all accumulated data
     */
    void reset();
    
    /**
     * @brief Get all tracking errors (if logging enabled)
     */
    const std::vector<TrackingError>& getAllErrors() const { return trackingErrors_; }
    
    /**
     * @brief Get all actual samples (thread-safe copy)
     *
     * Returns a copy rather than a const reference so the caller cannot
     * observe the deque while another thread is mutating it.
     */
    std::deque<PositionSample> getActualSamples() const {
        std::lock_guard<std::mutex> lk(samples_mutex_);
        return actualSamples_;
    }
    
    /**
     * @brief Configuration access
     */
    void configure(const ReplannerConfig& config) { config_ = config; }
    const ReplannerConfig& config() const { return config_; }
    
    /**
     * @brief Set callback for real-time error notification
     */
    using ErrorCallback = std::function<void(const TrackingError&)>;
    void setErrorCallback(ErrorCallback callback) { errorCallback_ = callback; }
    
    /**
     * @brief Set callback for limit suggestion notification
     */
    using SuggestionCallback = std::function<void(const ParameterSuggestion&)>;
    void setSuggestionCallback(SuggestionCallback callback) { suggestionCallback_ = callback; }

private:
    ReplannerConfig config_;
    
    // Desired trajectory
    std::vector<GCodeExport::TrajectorySample> desiredTrajectory_;
    
    // Actual samples (ring buffer for memory efficiency).
    // Guarded by samples_mutex_ because addActualSample() may be called from
    // a feedback/RT thread while processAccumulatedSamples() / detectSystemDelay()
    // / getActualSamples() run on another thread. Without synchronization the
    // deque's internal map can be invalidated mid-iteration (use-after-free).
    std::deque<PositionSample> actualSamples_;
    mutable std::mutex samples_mutex_;
    size_t nextDesiredIndex_ = 0;
    
    // Computed errors
    std::vector<TrackingError> trackingErrors_;
    
    // Segment performance cache
    mutable std::map<int32_t, SegmentPerformance> segmentPerformance_;
    mutable bool segmentPerfDirty_ = true;
    
    // System delay
    double detectedDelay_ = 0.001;
    
    // Callbacks
    ErrorCallback errorCallback_;
    SuggestionCallback suggestionCallback_;
    
    // Helper functions
    size_t findNearestDesiredIndex(double timestamp) const;
    TrackingError computeError(const PositionSample& actual, 
                               const GCodeExport::TrajectorySample& desired) const;
    bool isCriticalPoint(size_t desiredIndex) const;
    std::string classifyCriticalPoint(size_t desiredIndex) const;
    
    void updateSegmentPerformance(const TrackingError& error);
    ParameterSuggestion generateSuggestion(const SegmentPerformance& perf) const;
    
    // C2 smooth transition helpers
    double smoothStep(double t) const;  // Hermite interpolation
    double quinticBlend(double t) const; // C2 continuous
};

//=============================================================================
// Rolling Statistics Helper
//=============================================================================

/**
 * @brief Efficient rolling statistics calculator
 */
class RollingStatistics {
public:
    explicit RollingStatistics(size_t maxSamples = 1000);
    
    void addSample(double value);
    void clear();
    
    size_t count() const { return count_; }
    double min() const { return min_; }
    double max() const { return max_; }
    double mean() const { return mean_; }
    double variance() const;
    double stdDev() const { return std::sqrt(variance()); }
    double geometricMean() const;
    double rms() const;
    
    // Percentiles require keeping all samples
    double percentile(double p) const;
    
private:
    size_t maxSamples_;
    size_t count_ = 0;
    double min_ = std::numeric_limits<double>::max();
    double max_ = std::numeric_limits<double>::lowest();
    double mean_ = 0.0;
    double m2_ = 0.0;  // For Welford's online variance
    double sumSquares_ = 0.0;
    double logSum_ = 0.0;
    size_t  positiveCount_ = 0;  // Count of samples > 0 (for geometric mean)
    
    std::vector<double> samples_; // For percentiles
};

//=============================================================================
// Live Monitoring Mode
//=============================================================================

/**
 * @brief Lightweight monitoring mode for continuous operation
 * 
 * Tracks performance without keeping full history.
 * Suitable for production monitoring.
 */
class LiveMonitor {
public:
    struct MonitorStatus {
        // Current error
        double currentPositionError = 0.0;
        double currentContourError = 0.0;
        double currentLagError = 0.0;
        
        // Rolling statistics
        ErrorStatistics rollingStats;
        
        // Alert status
        bool positionAlert = false;
        bool contourAlert = false;
        bool lagAlert = false;
        std::string alertMessage;
        
        // Performance metrics
        double feedRateEfficiency = 1.0;  // Achieved / commanded
        double overallHealth = 1.0;       // 0-1 composite score
    };
    
    explicit LiveMonitor(const ReplannerConfig& config = {});
    
    /**
     * @brief Update with new sample pair
     */
    void update(const PositionSample& actual, 
                const GCodeExport::TrajectorySample& desired);
    
    /**
     * @brief Get current status
     */
    MonitorStatus getStatus() const;
    
    /**
     * @brief Reset rolling statistics
     */
    void reset();
    
    /**
     * @brief Set alert callback
     */
    using AlertCallback = std::function<void(const std::string&)>;
    void setAlertCallback(AlertCallback callback) { alertCallback_ = callback; }
    
private:
    ReplannerConfig config_;
    RollingStatistics positionStats_;
    RollingStatistics contourStats_;
    RollingStatistics lagStats_;
    
    double currentPositionError_ = 0.0;
    double currentContourError_ = 0.0;
    double currentLagError_ = 0.0;
    
    AlertCallback alertCallback_;
};

} // namespace MotionReplanner
