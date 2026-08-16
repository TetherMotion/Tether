/**
 * @file AccelerationProfileAnalyzer.hpp
 * @brief Analyze acceleration profile and estimate acceleration-limited time.
 *
 * @copyright Copyright (C) 2025-2026 Tether Authors
 */
#pragma once

#include "tether/analysis/GcodeParseUtils.hpp"
#include <string>
#include <vector>

namespace tether::analysis {

/// @brief A single acceleration data point.
struct AccelerationPoint {
    int line = 0;
    double acceleration = 0; ///< mm/s²
    enum class Type { Acceleration, Deceleration, Steady, Jerk } type = Type::Steady;
    enum class Magnitude { Low, Medium, High, Extreme } magnitude = Magnitude::Low;
};

/// @brief Per-segment time data (sampled).
struct AccelTimeSegment {
    int lineNumber = 0;
    double unlimitedTime = 0;  ///< seconds (no accel limit)
    double limitedTime = 0;    ///< seconds (with accel limit)
    bool isAccelLimited = false;
};

/// @brief Combined acceleration profile analysis result.
struct AccelerationProfileResult {
    // Acceleration profile
    std::vector<AccelerationPoint> points;
    double avgAcceleration = 0;
    double maxAcceleration = 0;
    int accelerationCount = 0;
    int decelerationCount = 0;
    int jerkCount = 0;
    double smoothnessScore = 100;

    // Acceleration-limited time estimation
    double limitedTime = 0;          ///< seconds
    double unlimitedTime = 0;        ///< seconds
    double accelerationOverhead = 0; ///< seconds
    double overheadPercentage = 0;
    int directionChanges = 0;
    int stops = 0;
    double avgLimitedSpeed = 0;      ///< mm/min
    std::vector<AccelTimeSegment> segments;

    std::vector<std::string> recommendations;
};

/// @brief Analyze acceleration profile and estimate acceleration-limited time.
class AccelerationProfileAnalyzer {
public:
    struct Params {
        double maxAccelMmPerS2 = 1000;
        double maxJerkMmPerS3 = 500;
        int sampleInterval = 100;
    };

    explicit AccelerationProfileAnalyzer() : params_() {}
    explicit AccelerationProfileAnalyzer(Params params) : params_(params) {}

    /// @brief Analyze acceleration profile from G-code.
    AccelerationProfileResult analyze(const std::vector<std::string>& gcodeLines) const;

    const Params& params() const { return params_; }
    void setParams(Params p) { params_ = p; }

private:
    Params params_;
};

} // namespace tether::analysis
