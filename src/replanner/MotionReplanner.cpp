/**
 * @file MotionReplanner.cpp
 * @brief Implementation of online motion replanning with closed-loop feedback
 */

#include "MotionReplanner.hpp"
#include <algorithm>
#include <numeric>
#include <cmath>

namespace MotionReplanner {

//=============================================================================
// RollingStatistics Implementation
//=============================================================================

RollingStatistics::RollingStatistics(size_t maxSamples)
    : maxSamples_(maxSamples) {
    samples_.reserve(maxSamples);
}

void RollingStatistics::addSample(double value) {
    count_++;
    
    // Min/max
    min_ = std::min(min_, value);
    max_ = std::max(max_, value);
    
    // Welford's online algorithm for mean and variance
    double delta = value - mean_;
    mean_ += delta / count_;
    double delta2 = value - mean_;
    m2_ += delta * delta2;
    
    // Sum of squares for RMS
    sumSquares_ += value * value;
    
    // Log sum for geometric mean (only for positive values)
    if (value > 0) {
        logSum_ += std::log(value);
        positiveCount_++;
    }
    
    // Keep samples for percentile calculation
    if (samples_.size() < maxSamples_) {
        samples_.push_back(value);
    }
}

void RollingStatistics::clear() {
    count_ = 0;
    min_ = std::numeric_limits<double>::max();
    max_ = std::numeric_limits<double>::lowest();
    mean_ = 0.0;
    m2_ = 0.0;
    sumSquares_ = 0.0;
    logSum_ = 0.0;
    positiveCount_ = 0;
    samples_.clear();
}

double RollingStatistics::variance() const {
    if (count_ < 2) return 0.0;
    return m2_ / (count_ - 1);
}

double RollingStatistics::geometricMean() const {
    // Geometric mean is only defined for positive values. If no positive
    // samples were added, return 0.0 instead of the incorrect exp(0)=1.0.
    if (positiveCount_ == 0) return 0.0;
    return std::exp(logSum_ / static_cast<double>(positiveCount_));
}

double RollingStatistics::rms() const {
    if (count_ == 0) return 0.0;
    return std::sqrt(sumSquares_ / count_);
}

double RollingStatistics::percentile(double p) const {
    if (samples_.empty()) return 0.0;
    
    std::vector<double> sorted = samples_;
    std::sort(sorted.begin(), sorted.end());
    
    double index = (p / 100.0) * (sorted.size() - 1);
    size_t lower = static_cast<size_t>(std::floor(index));
    size_t upper = static_cast<size_t>(std::ceil(index));
    
    if (lower == upper) return sorted[lower];
    
    double frac = index - lower;
    return sorted[lower] * (1.0 - frac) + sorted[upper] * frac;
}

//=============================================================================
// MotionReplanner Implementation
//=============================================================================

MotionReplanner::MotionReplanner(const ReplannerConfig& config)
    : config_(config), detectedDelay_(config.systemDelay) {}

void MotionReplanner::setDesiredTrajectory(
    const std::vector<GCodeExport::TrajectorySample>& desired) {
    desiredTrajectory_ = desired;
    reset();
}

void MotionReplanner::reset() {
    std::lock_guard<std::mutex> lk(samples_mutex_);
    actualSamples_.clear();
    trackingErrors_.clear();
    segmentPerformance_.clear();
    segmentPerfDirty_ = true;
    nextDesiredIndex_ = 0;
}

void MotionReplanner::addActualSample(const PositionSample& sample) {
    std::lock_guard<std::mutex> lk(samples_mutex_);
    // Add to buffer
    actualSamples_.push_back(sample);
    
    // Limit buffer size to save memory
    const size_t maxBufferSize = 10000;
    while (actualSamples_.size() > maxBufferSize) {
        actualSamples_.pop_front();
    }
}

size_t MotionReplanner::processAccumulatedSamples() {
    // Snapshot the samples under the lock, then release it before running
    // the per-sample loop. This avoids holding samples_mutex_ while user
    // callbacks (errorCallback_, suggestionCallback_) run, which could
    // otherwise deadlock if a callback calls back into this replanner
    // (e.g. addActualSample).
    std::deque<PositionSample> samples_snapshot;
    {
        std::lock_guard<std::mutex> lk(samples_mutex_);
        if (desiredTrajectory_.empty() || actualSamples_.empty()) {
            return 0;
        }
        samples_snapshot = actualSamples_;
    }

    size_t processedCount = 0;

    for (const auto& actual : samples_snapshot) {
        // Apply delay compensation
        double adjustedTime = actual.timestamp - detectedDelay_;
        if (adjustedTime < 0) continue;

        // Find nearest desired sample
        size_t desiredIdx = findNearestDesiredIndex(adjustedTime);
        if (desiredIdx >= desiredTrajectory_.size()) continue;

        const auto& desired = desiredTrajectory_[desiredIdx];

        // Compute tracking error
        TrackingError error = computeError(actual, desired);
        error.segmentIndex = desired.segmentIndex;
        error.blockIndex = desired.blockIndex;

        // Check if this is a critical point
        error.isCriticalPoint = isCriticalPoint(desiredIdx);
        if (error.isCriticalPoint) {
            error.criticalPointType = classifyCriticalPoint(desiredIdx);
        }

        // Store error
        if (config_.logAllSamples || error.isCriticalPoint) {
            trackingErrors_.push_back(error);
        }

        // Update segment performance
        updateSegmentPerformance(error);

        // Notify callback
        if (errorCallback_) {
            errorCallback_(error);
        }

        // Check for alerts
        if (!config_.monitoringOnly && config_.enableLimitSuggestions) {
            if (error.combinedPositionError > config_.positionErrorThreshold ||
                error.contourError > config_.contourErrorThreshold) {

                auto perf = segmentPerformance_[error.segmentIndex];
                auto suggestion = generateSuggestion(perf);
                if (suggestion.limitAdjustmentNeeded && suggestionCallback_) {
                    suggestionCallback_(suggestion);
                }
            }
        }

        processedCount++;
    }

    // Clear processed samples (keep some for delay detection)
    if (!config_.autoDetectDelay) {
        std::lock_guard<std::mutex> lk(samples_mutex_);
        actualSamples_.clear();
    }

    return processedCount;
}

size_t MotionReplanner::findNearestDesiredIndex(double timestamp) const {
    if (desiredTrajectory_.empty()) return 0;
    
    // Binary search for nearest time
    auto it = std::lower_bound(
        desiredTrajectory_.begin(), desiredTrajectory_.end(),
        timestamp,
        [](const GCodeExport::TrajectorySample& s, double t) {
            return s.time < t;
        }
    );
    
    if (it == desiredTrajectory_.end()) {
        return desiredTrajectory_.size() - 1;
    }
    
    if (it == desiredTrajectory_.begin()) {
        return 0;
    }
    
    // Return closer of the two adjacent samples
    auto prev = std::prev(it);
    double diffPrev = std::abs(prev->time - timestamp);
    double diffCurr = std::abs(it->time - timestamp);
    
    return (diffPrev < diffCurr) ? 
           std::distance(desiredTrajectory_.begin(), prev) :
           std::distance(desiredTrajectory_.begin(), it);
}

TrackingError MotionReplanner::computeError(
    const PositionSample& actual,
    const GCodeExport::TrajectorySample& desired) const {
    
    TrackingError error;
    error.timestamp = actual.timestamp;
    error.pathPosition = desired.pathPosition;
    
    // Position error per axis
    for (size_t i = 0; i < 9; ++i) {
        error.positionError[i] = actual.position[i] - desired.position[i];
        if (actual.velocityValid) {
            error.velocityError[i] = actual.velocity[i] - desired.velocity[i];
        }
    }
    
    // Combined 3D position error
    error.combinedPositionError = std::sqrt(
        error.positionError[0] * error.positionError[0] +
        error.positionError[1] * error.positionError[1] +
        error.positionError[2] * error.positionError[2]
    );
    
    // Decompose into contour error (perpendicular) and lag error (along path)
    // Using the velocity direction as the tangent
    double velMag = std::sqrt(
        desired.velocity[0] * desired.velocity[0] +
        desired.velocity[1] * desired.velocity[1] +
        desired.velocity[2] * desired.velocity[2]
    );
    
    if (velMag > 1e-9) {
        // Unit tangent vector
        double tx = desired.velocity[0] / velMag;
        double ty = desired.velocity[1] / velMag;
        double tz = desired.velocity[2] / velMag;
        
        // Project error onto tangent (lag error)
        error.lagError = error.positionError[0] * tx +
                         error.positionError[1] * ty +
                         error.positionError[2] * tz;
        
        // Contour error is the remaining perpendicular component
        double lagVecX = error.lagError * tx;
        double lagVecY = error.lagError * ty;
        double lagVecZ = error.lagError * tz;
        
        double contourX = error.positionError[0] - lagVecX;
        double contourY = error.positionError[1] - lagVecY;
        double contourZ = error.positionError[2] - lagVecZ;
        
        error.contourError = std::sqrt(contourX * contourX + 
                                       contourY * contourY + 
                                       contourZ * contourZ);
    } else {
        // At standstill, all error is position error
        error.contourError = error.combinedPositionError;
        error.lagError = 0.0;
    }
    
    return error;
}

bool MotionReplanner::isCriticalPoint(size_t desiredIndex) const {
    if (desiredIndex < 2 || desiredIndex >= desiredTrajectory_.size() - 2) {
        return false;
    }
    
    const auto& prev = desiredTrajectory_[desiredIndex - 1];
    const auto& curr = desiredTrajectory_[desiredIndex];
    const auto& next = desiredTrajectory_[desiredIndex + 1];
    
    // Compute direction change angle
    double v1x = curr.position[0] - prev.position[0];
    double v1y = curr.position[1] - prev.position[1];
    double v1z = curr.position[2] - prev.position[2];
    
    double v2x = next.position[0] - curr.position[0];
    double v2y = next.position[1] - curr.position[1];
    double v2z = next.position[2] - curr.position[2];
    
    double mag1 = std::sqrt(v1x*v1x + v1y*v1y + v1z*v1z);
    double mag2 = std::sqrt(v2x*v2x + v2y*v2y + v2z*v2z);
    
    if (mag1 < 1e-9 || mag2 < 1e-9) return false;
    
    double dot = (v1x*v2x + v1y*v2y + v1z*v2z) / (mag1 * mag2);
    dot = std::clamp(dot, -1.0, 1.0);
    double angleDeg = std::acos(dot) * 180.0 / M_PI;
    
    // Check for segment boundary
    bool segmentChange = prev.segmentIndex != curr.segmentIndex ||
                         curr.segmentIndex != next.segmentIndex;
    
    // Check for motion type change
    bool motionTypeChange = prev.motionType != curr.motionType ||
                            curr.motionType != next.motionType;
    
    return angleDeg > config_.cornerAngleThreshold || segmentChange || motionTypeChange;
}

std::string MotionReplanner::classifyCriticalPoint(size_t desiredIndex) const {
    if (desiredIndex < 1 || desiredIndex >= desiredTrajectory_.size() - 1) {
        return "boundary";
    }
    
    const auto& prev = desiredTrajectory_[desiredIndex - 1];
    const auto& curr = desiredTrajectory_[desiredIndex];
    const auto& next = desiredTrajectory_[desiredIndex + 1];
    
    if (prev.segmentIndex != curr.segmentIndex) {
        // Transitioning into a new segment
        switch (curr.motionType) {
            case 0: return "rapid_start";
            case 1: return "linear_start";
            case 2: return "arc_cw_start";
            case 3: return "arc_ccw_start";
            default: return "segment_start";
        }
    }
    
    if (curr.segmentIndex != next.segmentIndex) {
        switch (curr.motionType) {
            case 2: return "arc_cw_end";
            case 3: return "arc_ccw_end";
            default: return "segment_end";
        }
    }
    
    return "corner";
}

void MotionReplanner::updateSegmentPerformance(const TrackingError& error) {
    int32_t segIdx = error.segmentIndex;
    if (segIdx < 0) return;
    
    auto& perf = segmentPerformance_[segIdx];
    if (perf.segmentIndex < 0) {
        perf.segmentIndex = segIdx;
        perf.blockIndex = error.blockIndex;
    }
    
    // Update position error statistics (simplified running stats)
    perf.positionStats.sampleCount++;
    double n = static_cast<double>(perf.positionStats.sampleCount);
    
    if (perf.positionStats.sampleCount == 1) {
        perf.positionStats.minError = error.combinedPositionError;
        perf.positionStats.maxError = error.combinedPositionError;
        perf.positionStats.meanError = error.combinedPositionError;
        perf.positionStats.rmsError = error.combinedPositionError * error.combinedPositionError;
    } else {
        perf.positionStats.minError = std::min(perf.positionStats.minError, 
                                               error.combinedPositionError);
        perf.positionStats.maxError = std::max(perf.positionStats.maxError, 
                                               error.combinedPositionError);
        
        double delta = error.combinedPositionError - perf.positionStats.meanError;
        perf.positionStats.meanError += delta / n;
        perf.positionStats.rmsError += error.combinedPositionError * error.combinedPositionError;
    }
    
    // Contour error statistics
    perf.contourStats.sampleCount++;
    if (perf.contourStats.sampleCount == 1) {
        perf.contourStats.maxError = error.contourError;
        perf.contourStats.meanError = error.contourError;
    } else {
        perf.contourStats.maxError = std::max(perf.contourStats.maxError, error.contourError);
        double delta = error.contourError - perf.contourStats.meanError;
        perf.contourStats.meanError += delta / static_cast<double>(perf.contourStats.sampleCount);
    }
    
    // Corner statistics
    if (error.isCriticalPoint) {
        perf.positionStats.cornerCount++;
        if (perf.positionStats.cornerCount == 1) {
            perf.positionStats.maxCornerError = error.combinedPositionError;
            perf.positionStats.meanCornerError = error.combinedPositionError;
        } else {
            perf.positionStats.maxCornerError = std::max(perf.positionStats.maxCornerError,
                                                         error.combinedPositionError);
            double delta = error.combinedPositionError - perf.positionStats.meanCornerError;
            perf.positionStats.meanCornerError += delta / static_cast<double>(perf.positionStats.cornerCount);
        }
    }
    
    segmentPerfDirty_ = true;
}

std::optional<TrackingError> MotionReplanner::getCurrentError() const {
    if (trackingErrors_.empty()) {
        return std::nullopt;
    }
    return trackingErrors_.back();
}

ErrorStatistics MotionReplanner::getOverallStatistics() const {
    ErrorStatistics stats;
    RollingStatistics rolling;
    
    for (const auto& error : trackingErrors_) {
        rolling.addSample(error.combinedPositionError);
    }
    
    stats.sampleCount = rolling.count();
    if (stats.sampleCount > 0) {
        stats.minError = rolling.min();
        stats.maxError = rolling.max();
        stats.meanError = rolling.mean();
        stats.stdDev = rolling.stdDev();
        stats.geometricMean = rolling.geometricMean();
        stats.rmsError = rolling.rms();
        stats.p95Error = rolling.percentile(95);
        stats.p99Error = rolling.percentile(99);
    }
    
    // Corner statistics
    RollingStatistics cornerRolling;
    for (const auto& error : trackingErrors_) {
        if (error.isCriticalPoint) {
            cornerRolling.addSample(error.combinedPositionError);
        }
    }
    
    stats.cornerCount = cornerRolling.count();
    if (stats.cornerCount > 0) {
        stats.maxCornerError = cornerRolling.max();
        stats.meanCornerError = cornerRolling.mean();
    }
    
    return stats;
}

ErrorStatistics MotionReplanner::getCriticalPointStatistics() const {
    ErrorStatistics stats;
    RollingStatistics rolling;
    
    for (const auto& error : trackingErrors_) {
        if (error.isCriticalPoint) {
            rolling.addSample(error.combinedPositionError);
        }
    }
    
    stats.sampleCount = rolling.count();
    stats.cornerCount = rolling.count();
    
    if (stats.sampleCount > 0) {
        stats.minError = rolling.min();
        stats.maxError = rolling.max();
        stats.meanError = rolling.mean();
        stats.stdDev = rolling.stdDev();
        stats.geometricMean = rolling.geometricMean();
        stats.rmsError = rolling.rms();
        stats.maxCornerError = rolling.max();
        stats.meanCornerError = rolling.mean();
    }
    
    return stats;
}

std::vector<SegmentPerformance> MotionReplanner::getSegmentPerformance() const {
    std::vector<SegmentPerformance> result;
    result.reserve(segmentPerformance_.size());
    
    for (const auto& [idx, perf] : segmentPerformance_) {
        result.push_back(perf);
    }
    
    // Sort by segment index
    std::sort(result.begin(), result.end(),
              [](const SegmentPerformance& a, const SegmentPerformance& b) {
                  return a.segmentIndex < b.segmentIndex;
              });
    
    return result;
}

std::vector<ParameterSuggestion> MotionReplanner::getParameterSuggestions() const {
    std::vector<ParameterSuggestion> suggestions;
    
    if (config_.monitoringOnly || !config_.enableLimitSuggestions) {
        return suggestions;
    }
    
    for (const auto& [idx, perf] : segmentPerformance_) {
        auto suggestion = generateSuggestion(perf);
        if (suggestion.limitAdjustmentNeeded) {
            suggestions.push_back(suggestion);
        }
    }
    
    return suggestions;
}

ParameterSuggestion MotionReplanner::generateSuggestion(
    const SegmentPerformance& perf) const {
    
    ParameterSuggestion suggestion;
    suggestion.segmentIndex = perf.segmentIndex;
    suggestion.currentFeedRate = perf.commandedFeedRate;
    suggestion.currentAccel = perf.commandedAccel;
    
    // Check if adjustment is needed
    bool needsReduction = perf.contourStats.maxError > config_.contourErrorThreshold ||
                          perf.positionStats.maxCornerError > config_.positionErrorThreshold * 2;
    
    bool canIncrease = perf.contourStats.maxError < config_.contourErrorThreshold * 0.5 &&
                       perf.feedRateRatio > 0.95;
    
    if (needsReduction) {
        // Calculate reduction factor based on error magnitude
        double errorRatio = perf.contourStats.maxError / config_.contourErrorThreshold;
        double reductionFactor = 1.0 / std::sqrt(errorRatio);
        reductionFactor = std::max(reductionFactor, config_.maxFeedRateDecrease);
        
        suggestion.suggestedFeedRate = perf.commandedFeedRate * reductionFactor;
        suggestion.suggestedAccel = perf.commandedAccel * reductionFactor;
        suggestion.limitAdjustmentNeeded = true;
        suggestion.reason = "Contour error exceeds threshold";
        suggestion.confidenceScore = std::min(1.0, perf.contourStats.sampleCount / 100.0);
        
    } else if (canIncrease && !config_.monitoringOnly) {
        // Can potentially increase limits
        double headroom = config_.contourErrorThreshold / perf.contourStats.maxError;
        double increaseFactor = std::sqrt(headroom) * (1.0 - config_.safetyMargin);
        increaseFactor = std::min(increaseFactor, config_.maxFeedRateIncrease);
        
        suggestion.suggestedFeedRate = std::min(
            perf.commandedFeedRate * increaseFactor,
            config_.absoluteMaxVelocity
        );
        suggestion.suggestedAccel = std::min(
            perf.commandedAccel * increaseFactor,
            config_.absoluteMaxAccel
        );
        
        if (suggestion.suggestedFeedRate > perf.commandedFeedRate * 1.05) {
            suggestion.limitAdjustmentNeeded = true;
            suggestion.reason = "Performance headroom allows increase";
            suggestion.confidenceScore = std::min(1.0, perf.contourStats.sampleCount / 200.0);
        }
    }
    
    // Calculate transition parameters
    suggestion.transitionTime = config_.limitTransitionTime;
    suggestion.transitionLength = std::max(
        config_.minTransitionLength,
        suggestion.currentFeedRate / 60.0 * suggestion.transitionTime
    );
    
    return suggestion;
}

MotionReplanner::Limits MotionReplanner::getSuggestedLimits(
    double pathPosition, bool useSmoothTransition) const {
    
    Limits limits;
    limits.feedRate = config_.absoluteMaxVelocity;
    limits.acceleration = config_.absoluteMaxAccel;
    limits.jerk = config_.absoluteMaxJerk;
    
    // Find applicable suggestions
    auto suggestions = getParameterSuggestions();
    
    for (const auto& sugg : suggestions) {
        if (pathPosition >= sugg.startPathPosition && 
            pathPosition <= sugg.endPathPosition) {
            
            if (useSmoothTransition) {
                // Apply C2 smooth transition
                double localT = (pathPosition - sugg.startPathPosition) / 
                               (sugg.endPathPosition - sugg.startPathPosition);
                double blend = quinticBlend(localT);
                
                limits.feedRate = sugg.currentFeedRate + 
                                  blend * (sugg.suggestedFeedRate - sugg.currentFeedRate);
                limits.acceleration = sugg.currentAccel +
                                      blend * (sugg.suggestedAccel - sugg.currentAccel);
            } else {
                limits.feedRate = sugg.suggestedFeedRate;
                limits.acceleration = sugg.suggestedAccel;
            }
            break;
        }
    }
    
    return limits;
}

double MotionReplanner::detectSystemDelay() {
    // Snapshot under the lock so the cross-correlation loop sees a stable
    // view of the samples even if addActualSample() runs concurrently.
    std::deque<PositionSample> samples_snapshot;
    {
        std::lock_guard<std::mutex> lk(samples_mutex_);
        samples_snapshot = actualSamples_;
    }
    if (samples_snapshot.size() < 100 || desiredTrajectory_.empty()) {
        return config_.systemDelay;
    }
    
    // Cross-correlation to find delay
    // Compare actual velocity profile with desired velocity profile
    
    const double maxDelay = config_.delayDetectionWindow;
    const double delayStep = 0.0001;  // 0.1ms resolution
    
    double bestDelay = 0.0;
    double bestCorrelation = -1e9;
    
    for (double testDelay = 0; testDelay <= maxDelay; testDelay += delayStep) {
        double correlation = 0.0;
        size_t count = 0;
        
        for (const auto& actual : samples_snapshot) {
            double adjustedTime = actual.timestamp - testDelay;
            if (adjustedTime < 0) continue;
            
            size_t desiredIdx = findNearestDesiredIndex(adjustedTime);
            if (desiredIdx >= desiredTrajectory_.size()) continue;
            
            const auto& desired = desiredTrajectory_[desiredIdx];
            
            // Correlate velocity magnitudes
            double actualVel = std::sqrt(
                actual.velocity[0] * actual.velocity[0] +
                actual.velocity[1] * actual.velocity[1] +
                actual.velocity[2] * actual.velocity[2]
            );
            double desiredVel = std::sqrt(
                desired.velocity[0] * desired.velocity[0] +
                desired.velocity[1] * desired.velocity[1] +
                desired.velocity[2] * desired.velocity[2]
            );
            
            correlation += actualVel * desiredVel;
            count++;
        }
        
        if (count > 0) {
            correlation /= count;
            if (correlation > bestCorrelation) {
                bestCorrelation = correlation;
                bestDelay = testDelay;
            }
        }
    }
    
    detectedDelay_ = bestDelay;
    return detectedDelay_;
}

double MotionReplanner::smoothStep(double t) const {
    // Hermite interpolation (C1 smooth)
    t = std::clamp(t, 0.0, 1.0);
    return t * t * (3.0 - 2.0 * t);
}

double MotionReplanner::quinticBlend(double t) const {
    // C2 continuous blend (zero velocity and acceleration at endpoints)
    t = std::clamp(t, 0.0, 1.0);
    return t * t * t * (t * (t * 6.0 - 15.0) + 10.0);
}

//=============================================================================
// LiveMonitor Implementation
//=============================================================================

LiveMonitor::LiveMonitor(const ReplannerConfig& config)
    : config_(config),
      positionStats_(1000),
      contourStats_(1000),
      lagStats_(1000) {}

void LiveMonitor::update(const PositionSample& actual,
                         const GCodeExport::TrajectorySample& desired) {
    // Compute errors
    double posErr = 0.0;
    for (size_t i = 0; i < 3; ++i) {
        double d = actual.position[i] - desired.position[i];
        posErr += d * d;
    }
    currentPositionError_ = std::sqrt(posErr);
    
    // Decompose into contour and lag (simplified)
    double velMag = std::sqrt(
        desired.velocity[0] * desired.velocity[0] +
        desired.velocity[1] * desired.velocity[1] +
        desired.velocity[2] * desired.velocity[2]
    );
    
    if (velMag > 1e-9) {
        double ex = actual.position[0] - desired.position[0];
        double ey = actual.position[1] - desired.position[1];
        double ez = actual.position[2] - desired.position[2];
        
        double tx = desired.velocity[0] / velMag;
        double ty = desired.velocity[1] / velMag;
        double tz = desired.velocity[2] / velMag;
        
        currentLagError_ = ex * tx + ey * ty + ez * tz;
        
        double lagX = currentLagError_ * tx;
        double lagY = currentLagError_ * ty;
        double lagZ = currentLagError_ * tz;
        
        double contX = ex - lagX;
        double contY = ey - lagY;
        double contZ = ez - lagZ;
        
        currentContourError_ = std::sqrt(contX*contX + contY*contY + contZ*contZ);
    } else {
        currentContourError_ = currentPositionError_;
        currentLagError_ = 0.0;
    }
    
    // Update rolling statistics
    positionStats_.addSample(currentPositionError_);
    contourStats_.addSample(currentContourError_);
    lagStats_.addSample(std::abs(currentLagError_));
    
    // Check for alerts
    bool alert = false;
    std::string alertMsg;
    
    if (currentPositionError_ > config_.positionErrorThreshold) {
        alert = true;
        alertMsg += "Position error: " + std::to_string(currentPositionError_) + "mm ";
    }
    if (currentContourError_ > config_.contourErrorThreshold) {
        alert = true;
        alertMsg += "Contour error: " + std::to_string(currentContourError_) + "mm ";
    }
    if (std::abs(currentLagError_) > config_.lagErrorThreshold) {
        alert = true;
        alertMsg += "Lag error: " + std::to_string(currentLagError_) + "mm ";
    }
    
    if (alert && alertCallback_) {
        alertCallback_(alertMsg);
    }
}

LiveMonitor::MonitorStatus LiveMonitor::getStatus() const {
    MonitorStatus status;
    
    status.currentPositionError = currentPositionError_;
    status.currentContourError = currentContourError_;
    status.currentLagError = currentLagError_;
    
    status.rollingStats.sampleCount = positionStats_.count();
    status.rollingStats.minError = positionStats_.min();
    status.rollingStats.maxError = positionStats_.max();
    status.rollingStats.meanError = positionStats_.mean();
    status.rollingStats.stdDev = positionStats_.stdDev();
    status.rollingStats.rmsError = positionStats_.rms();
    
    status.positionAlert = currentPositionError_ > config_.positionErrorThreshold;
    status.contourAlert = currentContourError_ > config_.contourErrorThreshold;
    status.lagAlert = std::abs(currentLagError_) > config_.lagErrorThreshold;
    
    // Health score: 1.0 = perfect, 0.0 = at threshold
    double posHealth = std::max(0.0, 1.0 - currentPositionError_ / config_.positionErrorThreshold);
    double contHealth = std::max(0.0, 1.0 - currentContourError_ / config_.contourErrorThreshold);
    status.overallHealth = std::min(posHealth, contHealth);
    
    return status;
}

void LiveMonitor::reset() {
    positionStats_.clear();
    contourStats_.clear();
    lagStats_.clear();
    currentPositionError_ = 0.0;
    currentContourError_ = 0.0;
    currentLagError_ = 0.0;
}

} // namespace MotionReplanner
