/**
 * @file MotionDispatcher.hpp
 * @brief Bridge G-code move callbacks to the Klipper protocol motion path.
 *
 * @details
 * MotionDispatcher connects the host-emulator's G-code `move` callback to the
 * real Klipper wire protocol:
 *
 *   move(x, y, z, e, speed_mm_s)
 *     -> MotionSegment (linear, from current to target position)
 *     -> MotionPlanBuilder -> MotionPlan<4>
 *     -> MotionTranslator<4> -> std::vector<AxisStepSequence>
 *     -> sendCallback (typically KlippyHost::sendStepSequences)
 *
 * This is the missing link between KlippyInstance (the host emulator / object
 * model) and KlippyHost + KlipperDevice (the protocol + motion execution).
 *
 * Usage:
 * @code
 *   MotionDispatcher::Config dcfg{
 *       .axisConfigs = {{{80,0},{80,0},{400,0},{500,0}}},
 *       .axisOids    = {0, 1, 2, 3},
 *       .clockFreqHz = 180000000,
 *       .sampleIntervalSec = 0.0001,
 *   };
 *   MotionDispatcher disp(dcfg);
 *   disp.setSendCallback([&](const auto& seqs){ return host.sendStepSequences(seqs); });
 *   disp.setClockProvider([&](){ return host.clockSync().mcuClockNow(); });
 *   disp.setKinematicsTransform(kt);
 *   // Wire into KlippyInstance:
 *   cb.move = [&](double x,double y,double z,double e,double s){ disp.move(x,y,z,e,s); };
 * @endcode
 */

#pragma once

#include "tether/klipper/motion/MotionTranslator.hpp"
#include "tether/motion_planner/MotionPlan.hpp"
#include "tether/motion_planner/MotionSegment.hpp"

#include <algorithm>
#include <array>
#include <cstdint>
#include <functional>
#include <vector>

namespace tether::klipper::motion {

/// @brief Configuration for the MotionDispatcher.
struct MotionDispatcherConfig {
    std::array<AxisConfig, 4> axisConfigs = {{
        {80.0, false}, {80.0, false}, {400.0, false}, {500.0, false}
    }};
    std::array<uint8_t, 4> axisOids = {0, 1, 2, 3};
    uint32_t clockFreqHz = 180000000;
    /// Sample interval for MotionTranslator discretization (seconds).
    double sampleIntervalSec = 0.0001;
    /// Kinematic limits for the MotionPlanBuilder.
    MotionPlanner::KinematicLimits<4, double> limits{};
#if TETHER_ENABLE_PRESSURE_ADVANCE
    /// Pressure advance configuration for the extruder axis.
    PressureAdvanceConfig pressureAdvance{};
#endif
};

/// @brief Bridges G-code move callbacks to the Klipper queue_step protocol path.
class MotionDispatcher {
public:
    using Config = MotionDispatcherConfig;

    /// @brief Callback that ships translated step sequences to the device.
    /// @return Number of queue_step commands actually enqueued.
    using SendCallback = std::function<size_t(const std::vector<AxisStepSequence>&)>;

    /// @brief Returns the current MCU clock (ticks) used as the startClock.
    using ClockProvider = std::function<uint32_t()>;

    explicit MotionDispatcher(Config config = Config{})
        : config_(std::move(config))
        , translator_(config_.axisConfigs, config_.axisOids) {
        // Reasonable default limits if caller left them zeroed.
        std::ranges::replace_if(config_.limits.axis.maxVelocity,    [](auto v) { return v <= 0; }, 200.0);
        std::ranges::replace_if(config_.limits.axis.maxAcceleration, [](auto v) { return v <= 0; }, 2000.0);
        std::ranges::replace_if(config_.limits.axis.maxJerk,         [](auto v) { return v <= 0; }, 20000.0);
        if (config_.limits.path.maxPathVelocity <= 0)      config_.limits.path.maxPathVelocity = 200.0;
        if (config_.limits.path.maxPathAcceleration <= 0)  config_.limits.path.maxPathAcceleration = 2000.0;
#if TETHER_ENABLE_PRESSURE_ADVANCE
        translator_.setPressureAdvanceConfig(config_.pressureAdvance);
#endif
    }

    /// @brief Set the callback that sends step sequences to the device.
    void setSendCallback(SendCallback cb) { send_ = std::move(cb); }

    /// @brief Set the clock provider (returns current MCU clock in ticks).
    void setClockProvider(ClockProvider cp) { clock_ = std::move(cp); }

    /// @brief Set the kinematics transform (Cartesian/CoreXY/Delta/...).
    void setKinematicsTransform(const KinematicsTransform& kt) {
        translator_.setKinematicsTransform(kt);
    }

#if TETHER_ENABLE_PRESSURE_ADVANCE
    /// @brief Update pressure advance config at runtime.
    /// Allows G-code (M900 / SET_PRESSURE_ADVANCE) to enable/disable
    /// and tune PA without recreating the dispatcher.
    void setPressureAdvanceConfig(const PressureAdvanceConfig& pa) {
        translator_.setPressureAdvanceConfig(pa);
    }

    /// @brief Get the current pressure advance config.
    const PressureAdvanceConfig& pressureAdvanceConfig() const {
        return translator_.pressureAdvanceConfig();
    }
#endif

    /// @brief Set/override the current position (mm). Used after homing/G92.
    void setPosition(std::array<double, 4> pos) { current_ = pos; }

    /// @return The current logical position (mm).
    std::array<double, 4> position() const { return current_; }

    /// @brief Execute a linear move to (x, y, z, e) at speed_mm_s (mm/s).
    /// @return Number of queue_step commands dispatched, or 0 if no send
    ///         callback is set / no motion is needed.
    size_t move(double x, double y, double z, double e, double speed_mm_s) {
        if (!send_) return 0;

        std::array<double, 4> target{x, y, z, e};
        bool anyMove = false;
        for (size_t i = 0; i < 4; ++i) {
            if (std::abs(target[i] - current_[i]) > 1e-9) { anyMove = true; break; }
        }
        if (!anyMove) return 0;

        // MotionSegment expects feedrate in mm/min; speed_mm_s is mm/s.
        double feedMmMin = std::max(speed_mm_s * 60.0, 1.0);

        MotionPlanner::MotionSegmentList segments;
        MotionPlanner::Vec<4> startVec{current_[0], current_[1], current_[2], current_[3]};
        MotionPlanner::Vec<4> endVec{target[0], target[1], target[2], target[3]};
        segments.append(MotionPlanner::MotionSegment::linear(startVec, endVec, feedMmMin));

        MotionPlanner::MotionPlanBuilder<4, double> builder(config_.limits);
        auto plan = builder.build(segments, feedMmMin);
        if (plan.totalDuration() <= 0.0) {
            current_ = target;
            return 0;
        }

        uint32_t startClock = clock_ ? clock_() : 0;
        auto seqs = translator_.translate(plan, config_.clockFreqHz,
                                          config_.sampleIntervalSec, startClock);
        size_t dispatched = send_(seqs);
        current_ = target;
        return dispatched;
    }

private:
    Config config_;
    MotionTranslator<4, double> translator_;
    SendCallback send_;
    ClockProvider clock_;
    std::array<double, 4> current_{0, 0, 0, 0};
};

} // namespace tether::klipper::motion
