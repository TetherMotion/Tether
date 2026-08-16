/**
 * @file MachineLimitChecker.hpp
 * @brief Check G-code for machine limit and over-travel violations.
 *
 * @copyright Copyright (C) 2025-2026 Tether Authors
 */
#pragma once

#include "tether/analysis/GcodeParseUtils.hpp"
#include <string>
#include <vector>

namespace tether::analysis {

/// @brief Machine travel envelope [min, max] per axis.
struct MachineEnvelope {
    double minX = -500, maxX = 500;
    double minY = -500, maxY = 500;
    double minZ = -500, maxZ = 500;
    double minA = 0, maxA = 0; ///< Optional rotary A
    double minB = 0, maxB = 0; ///< Optional rotary B
    double minC = 0, maxC = 0; ///< Optional rotary C
    bool hasA = false, hasB = false, hasC = false;
};

/// @brief Machine kinematic limits.
struct MachineKinematicLimits {
    double maxFeedRateMmPerMin = 6000;
    double maxAccelerationMmPerS2 = 3000;
    double maxJerkMmPerS3 = 20000;
};

/// @brief A single limit violation.
struct LimitViolation {
    int lineNumber = 0;
    enum class Category { FeedRate, Acceleration, Jerk, Travel } category;
    double actual = 0;
    double limit = 0;
    std::string message;
    enum class Severity { Warning, Error } severity = Severity::Warning;
};

/// @brief Result of machine limit checking.
struct MachineLimitResult {
    std::vector<LimitViolation> violations;
    int violationCount = 0;
    int errorCount = 0;
    int warningCount = 0;
    double safetyScore = 100.0; ///< 0-100, higher is safer
    std::vector<std::string> recommendations;
};

/// @brief Check G-code for machine limit violations (feed, accel, jerk, travel).
class MachineLimitChecker {
public:
    using Envelope = MachineEnvelope;
    using KinematicLimits = MachineKinematicLimits;

    explicit MachineLimitChecker(Envelope envelope = {},
                                  KinematicLimits kinematic = {})
        : envelope_(envelope), kinematic_(kinematic) {}

    /// @brief Check G-code lines for all limit violations.
    MachineLimitResult check(const std::vector<std::string>& gcodeLines) const;

    const Envelope& envelope() const { return envelope_; }
    const KinematicLimits& kinematic() const { return kinematic_; }
    void setEnvelope(Envelope e) { envelope_ = e; }
    void setKinematic(KinematicLimits k) { kinematic_ = k; }

private:
    Envelope envelope_;
    KinematicLimits kinematic_;
};

} // namespace tether::analysis
