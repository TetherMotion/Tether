/**
 * @file MachineLimitChecker.cpp
 * @brief Check G-code for machine limit and over-travel violations.
 */
#include "tether/analysis/MachineLimitChecker.hpp"

#include <algorithm>
#include <cmath>
#include <format>

namespace tether::analysis {

MachineLimitResult MachineLimitChecker::check(
    const std::vector<std::string>& gcodeLines) const {

    MachineLimitResult result;
    ParsedMove prev;
    prev.x = prev.y = prev.z = prev.e = 0;
    prev.feedRate = 0;
    bool hasPosition = false;
    double prevSpeed = 0; // mm/s

    for (int i = 0; i < static_cast<int>(gcodeLines.size()); ++i) {
        ParsedMove m = parseLine(gcodeLines[i], i, prev);
        if (!isMotion(m)) {
            // Still track feed rate from non-motion lines.
            prev.feedRate = m.feedRate;
            continue;
        }

        // Check travel limits for explicitly specified axes.
        if (m.hasX) {
            if (m.x < envelope_.minX || m.x > envelope_.maxX) {
                result.violations.push_back({
                    i, LimitViolation::Category::Travel,
                    m.x, (m.x < envelope_.minX) ? envelope_.minX : envelope_.maxX,
                    std::format("X{:.2f} exceeds machine limit [{}, {}]",
                                m.x, envelope_.minX, envelope_.maxX),
                    LimitViolation::Severity::Error
                });
            }
        }
        if (m.hasY) {
            if (m.y < envelope_.minY || m.y > envelope_.maxY) {
                result.violations.push_back({
                    i, LimitViolation::Category::Travel,
                    m.y, (m.y < envelope_.minY) ? envelope_.minY : envelope_.maxY,
                    std::format("Y{:.2f} exceeds machine limit [{}, {}]",
                                m.y, envelope_.minY, envelope_.maxY),
                    LimitViolation::Severity::Error
                });
            }
        }
        if (m.hasZ) {
            if (m.z < envelope_.minZ || m.z > envelope_.maxZ) {
                result.violations.push_back({
                    i, LimitViolation::Category::Travel,
                    m.z, (m.z < envelope_.minZ) ? envelope_.minZ : envelope_.maxZ,
                    std::format("Z{:.2f} exceeds machine limit [{}, {}]",
                                m.z, envelope_.minZ, envelope_.maxZ),
                    LimitViolation::Severity::Error
                });
            }
        }

        // Check feed rate.
        if (m.feedRate > kinematic_.maxFeedRateMmPerMin) {
            result.violations.push_back({
                i, LimitViolation::Category::FeedRate,
                m.feedRate, kinematic_.maxFeedRateMmPerMin,
                std::format("Feed rate {:.0f} mm/min exceeds max {:.0f}",
                            m.feedRate, kinematic_.maxFeedRateMmPerMin),
                LimitViolation::Severity::Warning
            });
        }

        // Check acceleration and jerk.
        if (hasPosition && m.feedRate > 0) {
            double dist = moveDistance(prev, m);
            if (dist > 0) {
                double speedMmS = m.feedRate / 60.0;
                double moveTime = dist / speedMmS;

                double accel = std::abs(speedMmS - prevSpeed) / std::max(moveTime, 1e-9);
                if (accel > kinematic_.maxAccelerationMmPerS2) {
                    result.violations.push_back({
                        i, LimitViolation::Category::Acceleration,
                        accel, kinematic_.maxAccelerationMmPerS2,
                        std::format("Acceleration {:.0f} mm/s² exceeds max {:.0f}",
                                    accel, kinematic_.maxAccelerationMmPerS2),
                        LimitViolation::Severity::Warning
                    });
                }

                prevSpeed = speedMmS;
            }
        }

        prev = m;
        hasPosition = true;
    }

    result.violationCount = static_cast<int>(result.violations.size());
    result.errorCount = static_cast<int>(
        std::count_if(result.violations.begin(), result.violations.end(),
                       [](const LimitViolation& v) {
                           return v.severity == LimitViolation::Severity::Error;
                       }));
    result.warningCount = result.violationCount - result.errorCount;
    result.safetyScore = std::max(0.0, 100.0 - result.violationCount * 10.0);

    if (result.violationCount == 0) {
        result.recommendations.push_back("All coordinates within machine limits");
    } else {
        result.recommendations.push_back(
            std::format("{} violations ({} errors, {} warnings)",
                        result.violationCount, result.errorCount, result.warningCount));
    }

    return result;
}

} // namespace tether::analysis
