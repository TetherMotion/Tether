/**
 * @file RetractionAnalyzer.hpp
 * @brief Analyze and optimize retraction settings from G-code.
 *
 * @copyright Copyright (C) 2025-2026 Tether Authors
 */
#pragma once

#include "tether/analysis/GcodeParseUtils.hpp"
#include <string>
#include <vector>

namespace tether::analysis {

/// @brief A single retraction event.
struct RetractionEvent {
    int lineNumber = 0;
    double distance = 0;   ///< mm
    double speed = 0;      ///< mm/min
    double zHop = 0;       ///< mm
    double x = 0, y = 0, z = 0;
    bool isDeretraction = false;
};

/// @brief Recommended retraction settings.
struct RetractionSettings {
    double distance = 0;    ///< mm
    double speed = 0;       ///< mm/min
    double extraRestart = 0; ///< mm
    double zHop = 0;        ///< mm
};

/// @brief A retraction optimization recommendation.
struct RetractionRecommendation {
    std::string parameter;
    double current = 0;
    double recommended = 0;
    std::string reason;
    std::string improvement;
};

/// @brief Combined retraction analysis result.
struct RetractionResult {
    // Basic analysis
    std::vector<RetractionEvent> events;
    int count = 0;
    double avgDistance = 0;
    double minDistance = 0;
    double maxDistance = 0;
    double avgSpeed = 0;
    double totalRetractionTime = 0; ///< seconds
    int zHopCount = 0;
    std::vector<double> uniqueDistances;
    std::vector<double> uniqueSpeeds;

    // Optimization
    RetractionSettings currentSettings;
    std::vector<RetractionRecommendation> recommendations;
    double stringingReduction = 0;
    double optimizationScore = 100;

    // Distance optimization
    double recommendedDistance = 0;
    enum class ExtruderType { Direct, Bowden, Unknown } extruderType = ExtruderType::Unknown;
    double stringingRiskReduction = 0;

    // Speed optimization
    double currentSpeedMmPerS = 0;
    double recommendedSpeedMmPerS = 0;
    double slipRisk = 0;      ///< 0-100
    double stringingRisk = 0; ///< 0-100
    double speedScore = 100;

    // Frequency analysis
    int totalLines = 0;
    double retractionsPer100Lines = 0;
    double avgDistanceBetweenRetractions = 0;
    int clusterCount = 0;
    int maxConsecutiveRetractions = 0;
    double frequencyScore = 100;

    // Acceleration analysis
    double avgRetractSpeed = 0;  ///< mm/s
    double maxRetractSpeed = 0;  ///< mm/s
    double avgAcceleration = 0;  ///< mm/s²
    double maxAcceleration = 0;  ///< mm/s²
    int highAccelCount = 0;
    double qualityScore = 100;

    std::vector<std::string> advice;
};

/// @brief Analyze and optimize retraction settings from G-code.
class RetractionAnalyzer {
public:
    struct Params {
        std::string filamentType = "PLA";
        enum class ExtruderType { Direct, Bowden, Unknown } extruderType =
            ExtruderType::Unknown;
        double filamentDiameterMm = 1.75;
        double maxRecommendedAccel = 3000;
    };

    explicit RetractionAnalyzer() : params_() {}
    explicit RetractionAnalyzer(Params params) : params_(params) {}

    /// @brief Analyze retraction settings from G-code.
    RetractionResult analyze(const std::vector<std::string>& gcodeLines) const;

    const Params& params() const { return params_; }
    void setParams(Params p) { params_ = p; }

private:
    Params params_;

    RetractionSettings recommendedSettings(const std::string& filamentType) const;
};

} // namespace tether::analysis
