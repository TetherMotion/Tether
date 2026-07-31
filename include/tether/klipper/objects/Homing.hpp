/**
 * @file Homing.hpp
 * @brief Homing sequence orchestrator and probe support.
 *
 * Provides:
 *   - HomingSequence: coordinates multi-axis homing with endstop sampling
 *   - Probe: probe peripheral for bed leveling and Z homing
 *   - HomingState: tracks homing progress and results
 */

#pragma once

#include <array>
#include <cstdint>
#include <functional>
#include <map>
#include <memory>
#include <string>
#include <vector>

namespace tether::klipper::objects {

// ============================================================================
// Homing state
// ============================================================================

enum class HomingPhase {
    Idle,
    Seeking,    ///< Moving towards endstop at search speed
    Bouncing,   ///< Moved back after trigger, re-approaching at slower speed
    Settling,   ///< Setting position after homing
    Complete,
    Failed
};

inline const char* homingPhaseToString(HomingPhase p) {
    switch (p) {
        case HomingPhase::Idle:     return "idle";
        case HomingPhase::Seeking:  return "seeking";
        case HomingPhase::Bouncing: return "bouncing";
        case HomingPhase::Settling: return "settling";
        case HomingPhase::Complete: return "complete";
        case HomingPhase::Failed:   return "failed";
    }
    return "unknown";
}

// ============================================================================
// Endstop state (for homing)
// ============================================================================

struct EndstopState {
    bool triggered = false;
    uint32_t triggerClock = 0;
    double positionAtTrigger = 0.0;
};

// ============================================================================
// Homing sequence
// ============================================================================

/// @brief Configuration for a single-axis homing sequence.
struct HomingAxisConfig {
    std::string name;              ///< Axis name (e.g. "x", "y", "z")
    int stepperIndex = 0;          ///< Stepper index for this axis
    double searchSpeed = 50.0;     ///< mm/s, speed for initial seek
    double bounceSpeed = 10.0;     ///< mm/s, speed for second approach
    double bounceDistance = 5.0;   ///< mm, distance to back off after trigger
    double homePosition = 0.0;     ///< Position to set after homing
    bool positiveDirection = false; ///< True if home is at positive end
    double endstopThreshold = 0.1; ///< Position threshold for endstop detection
};

/// @brief Result of a homing operation.
struct HomingResult {
    bool success = false;
    std::string axis;
    double finalPosition = 0.0;
    double triggerPosition = 0.0;
    std::string errorMessage;
};

/// @brief Homing sequence orchestrator.
///
/// Coordinates multi-axis homing by:
/// 1. Moving each axis towards its endstop at search speed
/// 2. When endstop triggers, backing off by bounceDistance
/// 3. Re-approaching at bounceSpeed for precision
/// 4. Setting the axis position to homePosition
class HomingSequence {
public:
    using EndstopCheckFunc = std::function<bool(const std::string& axis)>;
    using MoveFunc = std::function<void(const std::string& axis,
                                         double speed, bool positive)>;
    using SetPositionFunc = std::function<void(const std::string& axis,
                                                double position)>;
    using GetPositionFunc = std::function<double(const std::string& axis)>;
    using WaitFunc = std::function<void(double seconds)>;

    HomingSequence(EndstopCheckFunc endstopCheck,
                   MoveFunc move,
                   SetPositionFunc setPosition,
                   GetPositionFunc getPosition,
                   WaitFunc wait)
        : endstopCheck_(std::move(endstopCheck))
        , move_(std::move(move))
        , setPosition_(std::move(setPosition))
        , getPosition_(std::move(getPosition))
        , wait_(std::move(wait)) {}

    /// @brief Home a single axis.
    HomingResult homeAxis(const HomingAxisConfig& config) {
        HomingResult result;
        result.axis = config.name;

        // Phase 1: Seek endstop at search speed
        phase_ = HomingPhase::Seeking;
        move_(config.name, config.searchSpeed, config.positiveDirection);

        // Wait for endstop trigger
        double totalWait = 0.0;
        double maxWait = 30.0; // 30 second timeout
        while (!endstopCheck_(config.name)) {
            wait_(0.01);
            totalWait += 0.01;
            if (totalWait > maxWait) {
                phase_ = HomingPhase::Failed;
                result.errorMessage = "Homing timeout on axis " + config.name;
                return result;
            }
        }

        double triggerPos = getPosition_(config.name);
        result.triggerPosition = triggerPos;

        // Phase 2: Bounce back
        phase_ = HomingPhase::Bouncing;
        move_(config.name, config.bounceSpeed, !config.positiveDirection);
        wait_(config.bounceDistance / config.bounceSpeed);

        // Phase 3: Re-approach at slower speed
        move_(config.name, config.bounceSpeed, config.positiveDirection);
        totalWait = 0.0;
        while (!endstopCheck_(config.name)) {
            wait_(0.01);
            totalWait += 0.01;
            if (totalWait > maxWait) {
                phase_ = HomingPhase::Failed;
                result.errorMessage = "Homing bounce timeout on axis " + config.name;
                return result;
            }
        }

        // Phase 4: Set position
        phase_ = HomingPhase::Settling;
        setPosition_(config.name, config.homePosition);

        phase_ = HomingPhase::Complete;
        result.success = true;
        result.finalPosition = config.homePosition;
        return result;
    }

    /// @brief Home multiple axes (sequentially).
    std::vector<HomingResult> homeAxes(const std::vector<HomingAxisConfig>& configs) {
        std::vector<HomingResult> results;
        for (const auto& cfg : configs) {
            results.push_back(homeAxis(cfg));
            if (!results.back().success) break;
        }
        return results;
    }

    /// @brief Get current homing phase.
    HomingPhase phase() const { return phase_; }

private:
    EndstopCheckFunc endstopCheck_;
    MoveFunc move_;
    SetPositionFunc setPosition_;
    GetPositionFunc getPosition_;
    WaitFunc wait_;
    HomingPhase phase_ = HomingPhase::Idle;
};

// ============================================================================
// Probe peripheral
// ============================================================================

/// @brief Probe peripheral for bed leveling and Z homing.
class Probe {
public:
    using PinReadFunc = std::function<bool()>;

    Probe(uint8_t oid, PinReadFunc pinRead)
        : oid_(oid)
        , pinRead_(std::move(pinRead)) {}

    uint8_t oid() const { return oid_; }

    /// @brief Check if probe is currently triggered.
    bool triggered() const { return pinRead_(); }

    /// @brief Probe at a specific XY position.
    /// @param zSpeed Z speed in mm/s (negative = down).
    /// @param zMaxDistance Maximum Z travel distance.
    /// @param zFunc Function to move Z axis: (speed, distance) -> triggered
    /// @return Probe result: Z position where probe triggered, or NaN if not triggered.
    double probe(double zSpeed, double zMaxDistance,
                 std::function<double(double, double)> zMove) {
        return zMove(zSpeed, zMaxDistance);
    }

    /// @brief Set Z offset (calibrated probe-to-nozzle offset).
    void setZOffset(double offset) { zOffset_ = offset; }

    /// @brief Get Z offset.
    double zOffset() const { return zOffset_; }

    /// @brief Set probe as virtual endstop for Z homing.
    void setVirtualEndstop(bool enabled) { virtualEndstop_ = enabled; }

    /// @brief Check if probe is used as virtual endstop.
    bool isVirtualEndstop() const { return virtualEndstop_; }

private:
    uint8_t oid_;
    PinReadFunc pinRead_;
    double zOffset_ = 0.0;
    bool virtualEndstop_ = false;
};

} // namespace tether::klipper::objects
