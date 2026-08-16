/**
 * @file RetractionAnalyzer.cpp
 * @brief Analyze and optimize retraction settings from G-code.
 */
#include "tether/analysis/RetractionAnalyzer.hpp"

#include <algorithm>
#include <numeric>
#include <cmath>
#include <format>
#include <set>

namespace tether::analysis {

RetractionSettings RetractionAnalyzer::recommendedSettings(
    const std::string& filamentType) const {

    if (filamentType == "ABS")
        return {3.0, 1500, 0.1, 0.2};
    if (filamentType == "PETG")
        return {4.0, 1200, 0.2, 0.4};
    if (filamentType == "TPU")
        return {0.8, 600, 0.0, 0.0};
    if (filamentType == "Nylon")
        return {3.5, 1200, 0.15, 0.3};
    // PLA default
    return {1.5, 1800, 0.0, 0.0};
}

RetractionResult RetractionAnalyzer::analyze(
    const std::vector<std::string>& gcodeLines) const {

    RetractionResult result;
    ParsedMove prev;
    prev.x = prev.y = prev.z = prev.e = 0;
    prev.feedRate = 0;
    double zBeforeHop = 0;
    double feedRate = 0;
    double retractionSpeed = 0;
    int lastRetractionLine = -1;
    int zHops = 0;

    // Frequency tracking.
    double distanceSinceLastRetraction = 0;
    std::vector<double> distancesBetweenRetractions;
    int consecutiveRetractions = 0;
    int maxConsecutiveRetractions = 0;
    int clusterCount = 0;
    bool inCluster = false;

    // Speed tracking.
    std::vector<double> retractSpeeds;

    for (int i = 0; i < static_cast<int>(gcodeLines.size()); ++i) {
        ParsedMove m = parseLine(gcodeLines[i], i, prev);

        if (m.hasE && isMotion(m)) {
            double eDelta = m.e - prev.e;

            if (eDelta < -0.001) {
                // Retraction.
                RetractionEvent ev;
                ev.lineNumber = i;
                ev.distance = std::abs(eDelta);
                ev.speed = m.feedRate;
                ev.zHop = std::abs(m.z - zBeforeHop);
                ev.x = m.x; ev.y = m.y; ev.z = m.z;
                ev.isDeretraction = false;
                result.events.push_back(ev);

                retractSpeeds.push_back(m.feedRate / 60.0);
                lastRetractionLine = i;

                // Frequency tracking.
                if (distanceSinceLastRetraction > 0)
                    distancesBetweenRetractions.push_back(distanceSinceLastRetraction);
                distanceSinceLastRetraction = 0;
                consecutiveRetractions++;
                maxConsecutiveRetractions = std::max(maxConsecutiveRetractions, consecutiveRetractions);
                if (consecutiveRetractions > 2 && !inCluster) {
                    clusterCount++;
                    inCluster = true;
                }
            } else if (eDelta > 0.001 && !m.hasX && !m.hasY && !result.events.empty()) {
                // Deretraction.
                const auto& last = result.events.back();
                if (!last.isDeretraction) {
                    RetractionEvent ev;
                    ev.lineNumber = i;
                    ev.distance = eDelta;
                    ev.speed = m.feedRate;
                    ev.zHop = 0;
                    ev.x = m.x; ev.y = m.y; ev.z = m.z;
                    ev.isDeretraction = true;
                    result.events.push_back(ev);
                }
            } else {
                consecutiveRetractions = 0;
                inCluster = false;
            }
        }

        // Z-hop detection.
        if (m.hasZ && !m.hasX && !m.hasY && m.hasE) {
            zBeforeHop = prev.z;
        }
        if (m.hasZ && lastRetractionLine >= i - 2 && i - lastRetractionLine <= 2) {
            zHops++;
        }

        // Track distance between retractions.
        if (isMotion(m)) {
            double dist = moveDistanceXY(prev, m);
            distanceSinceLastRetraction += dist;
        }

        prev = m;
    }

    // Compute basic statistics.
    std::vector<RetractionEvent> retractions;
    for (const auto& e : result.events)
        if (!e.isDeretraction) retractions.push_back(e);

    result.count = static_cast<int>(retractions.size());
    if (!retractions.empty()) {
        std::vector<double> distances, speeds;
        for (const auto& r : retractions) {
            distances.push_back(r.distance);
            speeds.push_back(r.speed);
        }

        result.avgDistance = std::accumulate(distances.begin(), distances.end(), 0.0) / distances.size();
        result.minDistance = *std::min_element(distances.begin(), distances.end());
        result.maxDistance = *std::max_element(distances.begin(), distances.end());
        result.avgSpeed = std::accumulate(speeds.begin(), speeds.end(), 0.0) / speeds.size();

        for (const auto& r : retractions) {
            if (r.speed > 0)
                result.totalRetractionTime += (r.distance / (r.speed / 60.0)) * 2.0;
        }

        result.zHopCount = static_cast<int>(
            std::count_if(retractions.begin(), retractions.end(),
                          [](const RetractionEvent& r) { return r.zHop > 0.01; }));

        // Unique distances and speeds.
        std::set<double> uniqueDists, uniqueSpeeds;
        for (double d : distances) uniqueDists.insert(std::round(d * 100) / 100);
        for (double s : speeds) uniqueSpeeds.insert(std::round(s));
        result.uniqueDistances = std::vector<double>(uniqueDists.begin(), uniqueDists.end());
        result.uniqueSpeeds = std::vector<double>(uniqueSpeeds.begin(), uniqueSpeeds.end());
    }

    // === Optimization ===
    result.currentSettings.distance = result.avgDistance;
    result.currentSettings.speed = result.avgSpeed;
    result.currentSettings.zHop = (zHops > 0) ? 0.2 : 0;

    auto recSettings = recommendedSettings(params_.filamentType);

    if (std::abs(result.currentSettings.distance - recSettings.distance) > 0.5) {
        result.recommendations.push_back({
            "distance", result.currentSettings.distance, recSettings.distance,
            std::format("{} optimal retraction distance", params_.filamentType),
            "Reduce stringing by ~30%"
        });
    }
    if (std::abs(result.currentSettings.speed - recSettings.speed) > 200) {
        result.recommendations.push_back({
            "speed", result.currentSettings.speed, recSettings.speed,
            std::format("{} optimal retraction speed", params_.filamentType),
            "Better filament control"
        });
    }
    if (recSettings.zHop > 0 && result.currentSettings.zHop == 0) {
        result.recommendations.push_back({
            "zHop", result.currentSettings.zHop, recSettings.zHop,
            std::format("{} benefits from Z-hop", params_.filamentType),
            "Prevent nozzle marks on travel"
        });
    }

    result.stringingReduction = result.recommendations.empty() ? 0
        : std::min(80.0, static_cast<double>(result.recommendations.size()) * 25);
    result.optimizationScore = std::max(0.0, 100.0 - static_cast<double>(result.recommendations.size()) * 20);

    // === Distance optimization ===
    auto detectedType = params_.extruderType;
    if (detectedType == Params::ExtruderType::Unknown) {
        if (result.avgDistance > 3) detectedType = Params::ExtruderType::Bowden;
        else if (result.avgDistance > 0) detectedType = Params::ExtruderType::Direct;
    }
    result.extruderType = static_cast<RetractionResult::ExtruderType>(detectedType);
    result.recommendedDistance = (detectedType == Params::ExtruderType::Bowden) ? 4.0 : 0.8;
    result.stringingRiskReduction = std::min(100.0,
        std::abs(result.avgDistance - result.recommendedDistance) * 20);

    // === Speed optimization ===
    if (!retractSpeeds.empty()) {
        result.currentSpeedMmPerS = retractSpeeds[0];
        result.avgRetractSpeed = std::accumulate(retractSpeeds.begin(), retractSpeeds.end(), 0.0) / retractSpeeds.size();
        result.maxRetractSpeed = *std::max_element(retractSpeeds.begin(), retractSpeeds.end());
    }
    result.recommendedSpeedMmPerS = (detectedType == Params::ExtruderType::Bowden) ? 25.0 : 35.0;
    result.slipRisk = (result.currentSpeedMmPerS > 40)
        ? std::min(100.0, (result.currentSpeedMmPerS - 40) * 5) : 0;
    result.stringingRisk = (result.currentSpeedMmPerS < 20)
        ? std::min(100.0, (20 - result.currentSpeedMmPerS) * 5) : 0;
    result.speedScore = std::max(0.0, 100.0 - result.slipRisk - result.stringingRisk);

    // === Frequency analysis ===
    result.totalLines = static_cast<int>(gcodeLines.size());
    result.retractionsPer100Lines = (result.totalLines > 0)
        ? (static_cast<double>(result.count) / result.totalLines) * 100.0 : 0;
    result.avgDistanceBetweenRetractions = distancesBetweenRetractions.empty() ? 0
        : std::accumulate(distancesBetweenRetractions.begin(), distancesBetweenRetractions.end(), 0.0)
          / distancesBetweenRetractions.size();
    result.clusterCount = clusterCount;
    result.maxConsecutiveRetractions = maxConsecutiveRetractions;

    double freqScore = 100;
    if (result.retractionsPer100Lines > 10) freqScore -= 20;
    if (clusterCount > 5) freqScore -= 20;
    if (maxConsecutiveRetractions > 3) freqScore -= 15;
    if (result.avgDistanceBetweenRetractions < 5 && result.count > 10) freqScore -= 20;
    result.frequencyScore = std::max(0.0, freqScore);

    // === Acceleration analysis ===
    if (!retractSpeeds.empty()) {
        std::vector<double> accels;
        for (double s : retractSpeeds) accels.push_back(s * 10);
        result.avgAcceleration = std::accumulate(accels.begin(), accels.end(), 0.0) / accels.size();
        result.maxAcceleration = *std::max_element(accels.begin(), accels.end());
        result.highAccelCount = static_cast<int>(
            std::count_if(accels.begin(), accels.end(),
                          [&](double a) { return a > params_.maxRecommendedAccel; }));
        result.qualityScore = std::max(0.0,
            100.0 - result.highAccelCount * 5.0 -
            (result.maxAcceleration / params_.maxRecommendedAccel - 1.0) * 30.0);
    }

    // === Advice ===
    if (result.count == 0) {
        result.advice.push_back("No retractions detected — enable retraction in slicer");
    } else {
        result.advice.push_back(
            std::format("{} retractions, avg {:.2f}mm", result.count, result.avgDistance));
    }
    for (const auto& rec : result.recommendations) {
        result.advice.push_back(
            std::format("{}: {:.2f} → {:.2f} ({})", rec.parameter, rec.current, rec.recommended, rec.reason));
    }
    if (result.recommendations.empty() && result.count > 0) {
        result.advice.push_back("Retraction settings are well-optimized");
    }

    return result;
}

} // namespace tether::analysis
