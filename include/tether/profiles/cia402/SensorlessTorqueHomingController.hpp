#pragma once

#include <atomic>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <type_traits>
#include <vector>

#include "tether/control/PIDControllers.hpp"
#include "tether/profiles/cia301/CiA402Defs.hpp"
#include "tether/profiles/cia402/CiA402Drive.hpp"
#include "tether/profiles/cia402/DS402Master.hpp"
#include "tether/platform/EspCompat.hpp"

namespace EtherCAT {

/**
 * @brief Generic sensorless torque homing for any CiA 402 drive.
 *
 * Drives the slave in Cyclic Sync Torque (CST) mode using a closed-loop
 * velocity PI controller. The torque command is clamped to a configurable
 * low percentage of rated torque. When the drive stops moving, isHomed()
 * becomes true and the computed home position is available via
 * homePosition(). The controller can therefore be used as a virtual homing
 * switch: it reports the position at which the mechanical stop was found
 * without itself touching the drive's 0-point.
 *
 * Optional multi-pass fine homing is available: after the first stall the
 * drive backs off and re-approaches the stop the configured number of times,
 * recording the position at each stall. The final home position can be the
 * mean or the sum of these recorded positions. The first (coarse) approach
 * runs at target_velocity; back-off and re-approach passes run at
 * fine_velocity when configured, otherwise at target_velocity. All passes
 * share the same torque limit.
 *
 * The class is templated on the drive-specific packed RxPDO/TxPDO structs.
 * The RxPDO must expose controlword, modes_of_operation and target_torque;
 * the TxPDO must expose statusword, modes_of_operation_display and
 * speed_feedback. position_actual is optional, but required for multi-pass
 * fine homing and for homePosition(). This is verified at compile time.
 *
 * Mode of operation is verified both at startup (after a short delay to let
 * the slave switch to CST) and continuously while the controller is running.
 * A per-phase watchdog (phase_timeout) fails the controller if a single
 * approach or back-off phase makes no progress for too long — this covers
 * the "axis already at the stop / torque too low to move" case, which would
 * otherwise hang forever waiting for movement that never happens.
 */
template<typename RxPDO, typename TxPDO>
class SensorlessTorqueHomingController : public DS402Master::IDriveMotionController {
public:
    enum class PassAggregation { Mean, Sum };
    enum class PassState { Approaching, BackingOff, Done };

    static constexpr int kMaxFineHomingPasses = 64;

    static_assert(requires(RxPDO& p) {
        p.controlword;
        p.modes_of_operation;
        p.target_torque;
    }, "RxPDO must provide controlword, modes_of_operation and target_torque");

    static_assert(requires(TxPDO& p) {
        p.statusword;
        p.modes_of_operation_display;
        p.speed_feedback;
    }, "TxPDO must provide statusword, modes_of_operation_display and speed_feedback");

    struct Config {
        double target_velocity = 2000.0;   // counts/s (magnitude), first pass
        int direction = 1;                 // +1 or -1
        double max_torque_percent = 1.0;   // % of rated
        double kp = 0.05;
        double ki = 0.005;
        double stall_velocity = 100.0;     // counts/s
        double stall_time = 1.0;           // seconds below threshold
        int fine_homing_passes = 1;        // total stall recordings, 1..kMaxFineHomingPasses
        double backoff_distance = 5000.0;  // counts
        double fine_velocity = 0.0;        // counts/s for back-off/re-approach; 0 = target_velocity
        double phase_timeout = 30.0;       // max seconds per approach/back-off phase; 0 = disabled
        PassAggregation pass_aggregation = PassAggregation::Mean;
    };

    explicit SensorlessTorqueHomingController(const Config& config)
        : config_(config)
        , reference_velocity_(static_cast<double>(config_.direction) * config_.target_velocity)
        , fine_magnitude_(config_.fine_velocity > 0.0 ? config_.fine_velocity : config_.target_velocity)
        , fine_reference_(static_cast<double>(config_.direction) * fine_magnitude_)
        , backoff_reference_(-fine_reference_)
        , current_reference_(reference_velocity_)
        , max_permille_(std::min(1000.0, config_.max_torque_percent * 10.0))
    {
        config_.fine_homing_passes =
            std::clamp(config_.fine_homing_passes, 1, kMaxFineHomingPasses);
    }

    bool start(CiA402Drive& drive) override
    {
        if (drive.rxPDO<RxPDO>() == nullptr) {
            setFailure("RxPDO not mapped");
            return false;
        }
        if (drive.txPDO<TxPDO>() == nullptr) {
            setFailure("TxPDO not mapped");
            return false;
        }

        if (std::abs(reference_velocity_) <= 0.0) {
            setFailure("target velocity must be non-zero");
            return false;
        }

        if (!drive.setModeCST()) {
            setFailure("Failed to set Cyclic Sync Torque mode");
            return false;
        }

        pi_.setGains(config_.kp, config_.ki);
        pi_.setIntegralLimits(-max_permille_, max_permille_);
        pi_.setSaturationLimits({
            -max_permille_,
            max_permille_,
            -max_permille_,
            max_permille_,
            -std::numeric_limits<double>::max(),
            std::numeric_limits<double>::max(),
            std::numeric_limits<double>::max(),
        });
        pi_.setAntiWindup(tether::control::AntiWindupMethod::Clamping, 0.0);
        pi_.reset();

        homed_.store(false, std::memory_order_release);
        failed_.store(false, std::memory_order_release);
        failure_message_.store("", std::memory_order_release);
        home_position_.store(0, std::memory_order_release);
        home_position_valid_.store(false, std::memory_order_release);
        virtual_switch_position_.store(0, std::memory_order_release);
        virtual_switch_position_valid_.store(false, std::memory_order_release);
        virtual_switch_active_.store(false, std::memory_order_release);
        pass_positions_.clear();
        pass_positions_.reserve(static_cast<size_t>(config_.fine_homing_passes));
        has_position_data_ = false;
        started_moving_ = false;
        mode_ok_ = false;
        stall_timer_ = 0.0;
        startup_timer_ = 0.0;
        phase_timer_ = 0.0;
        state_ = PassState::Approaching;
        current_reference_ = reference_velocity_;
        backoff_has_position_ = false;
        backoff_start_position_ = 0;
        backoff_timer_ = 0.0;

        TETHER_LOGI("sensorless_homing",
                    "Started on slave {}: reference={} counts/s, max_torque={}%, passes={}",
                    drive.slaveIndex(), reference_velocity_, config_.max_torque_percent,
                    config_.fine_homing_passes);
        return true;
    }

    void stop(CiA402Drive& drive) override
    {
        // Zero the torque command so a stale value is not left in the PDO
        // buffer if the controller is removed while the loop is still
        // flushing process data.
        if (auto* rx = drive.rxPDO<RxPDO>()) {
            rx->target_torque = 0;
        }
    }

    bool update(CiA402Drive& drive, double dt_seconds) override
    {
        auto* rx = drive.rxPDO<RxPDO>();
        auto* tx = drive.txPDO<TxPDO>();
        if (rx == nullptr || tx == nullptr) {
            setFailure("PDO buffers became invalid");
            return true;
        }

        rx->controlword = static_cast<uint16_t>(CiA402::ControlWord::ENABLE_OPERATION);
        rx->modes_of_operation = CiA402::OperatingMode::CyclicSyncTorque;

        if constexpr (requires(RxPDO& p) { p.target_position; }) {
            rx->target_position = 0;
        }
        if constexpr (requires(RxPDO& p) { p.target_velocity; }) {
            rx->target_velocity = 0;
        }
        if constexpr (requires(RxPDO& p) { p.max_profile_velocity; }) {
            rx->max_profile_velocity = 100000u;
        }
        if constexpr (requires(RxPDO& p) { p.touch_probe_function; }) {
            rx->touch_probe_function = 0;
        }

        startup_timer_ += dt_seconds;

        if (!mode_ok_ && !failed_.load(std::memory_order_acquire) &&
            startup_timer_ >= kModeVerifyDelaySeconds) {
            if (tx->modes_of_operation_display != CiA402::OperatingMode::CyclicSyncTorque) {
                setFailure("Drive is not in Cyclic Sync Torque mode");
            } else if ((tx->statusword & CiA402::StatuswordBits::OperationEnabled) == 0) {
                setFailure("Drive is not operation-enabled");
            } else {
                mode_ok_ = true;
            }
        }

        if (failed_.load(std::memory_order_acquire)) {
            rx->target_torque = 0;
            return true;
        }

        if (mode_ok_) {
            if (tx->modes_of_operation_display != CiA402::OperatingMode::CyclicSyncTorque) {
                setFailure("Drive left Cyclic Sync Torque mode");
                rx->target_torque = 0;
                return true;
            }
            if ((tx->statusword & CiA402::StatuswordBits::OperationEnabled) == 0) {
                setFailure("Drive lost operation-enabled state");
                rx->target_torque = 0;
                return true;
            }
        }

        if (state_ == PassState::Done) {
            rx->target_torque = 0;
            return true;
        }

        // Per-phase watchdog: an approach or back-off that makes no progress
        // (e.g. axis already at the stop, torque too low to move) must fail
        // instead of hanging forever.
        if (config_.phase_timeout > 0.0) {
            phase_timer_ += dt_seconds;
            if (phase_timer_ >= config_.phase_timeout) {
                setFailure("Homing phase timed out — no progress");
                rx->target_torque = 0;
                return true;
            }
        }

        const double speed = static_cast<double>(tx->speed_feedback);

        if (state_ == PassState::Approaching) {
            // Arm stall detection only once the axis is actually moving in
            // the approach direction. Requiring a matching sign prevents a
            // phantom stall during the speed reversal when transitioning
            // from BackingOff to Approaching.
            if (!started_moving_ && std::abs(speed) > config_.stall_velocity &&
                speed * current_reference_ > 0.0) {
                started_moving_ = true;
                TETHER_LOGI("sensorless_homing",
                            "Slave {} started moving: speed={} counts/s",
                            drive.slaveIndex(), speed);
            }

            if (started_moving_) {
                if (std::abs(speed) <= config_.stall_velocity) {
                    stall_timer_ += dt_seconds;
                    if (stall_timer_ >= config_.stall_time) {
                        recordPass(drive, tx, rx);
                        return true;
                    }
                } else {
                    stall_timer_ = 0.0;
                }
            }
        } else if (state_ == PassState::BackingOff) {
            if (backoffDone(tx, dt_seconds)) {
                enterApproach();
            }
        }

        tether::control::ControllerInput input;
        input.reference = current_reference_;
        input.measured = speed;
        input.dt = dt_seconds;
        input.enable = true;

        const auto output = pi_.compute(input);
        rx->target_torque = static_cast<int16_t>(std::llround(std::clamp(
            output.control, -max_permille_, max_permille_)));
        return true;
    }

    /// True once the final pass is complete.
    bool isHomed() const { return homed_.load(std::memory_order_acquire); }

    /// True if the virtual homing switch is active (same as isHomed()).
    bool isVirtualSwitchActive() const { return virtual_switch_active_.load(std::memory_order_acquire); }

    /// True if a computed home position is available.
    bool hasHomePosition() const { return home_position_valid_.load(std::memory_order_acquire); }

    /// True if a virtual switch position is available.
    bool hasVirtualSwitchPosition() const { return virtual_switch_position_valid_.load(std::memory_order_acquire); }

    /// The computed home position (mean or sum of the recorded pass positions).
    int64_t homePosition() const { return home_position_.load(std::memory_order_acquire); }

    /// Position reported by the virtual homing switch (same as homePosition()).
    int64_t virtualSwitchPosition() const { return virtual_switch_position_.load(std::memory_order_acquire); }

    bool hasFailed() const { return failed_.load(std::memory_order_acquire); }
    const char* failureMessage() const { return failure_message_.load(std::memory_order_acquire); }

private:
    void setFailure(const char* message)
    {
        if (!failed_.exchange(true, std::memory_order_acq_rel)) {
            failure_message_.store(message, std::memory_order_release);
            TETHER_LOGE("sensorless_homing", "Failed: {}", message);
        }
    }

    void recordPass(CiA402Drive& drive, const TxPDO* tx, RxPDO* rx)
    {
        constexpr bool kHasPosition = requires(TxPDO& p) { p.position_actual; };
        if (config_.fine_homing_passes > 1 && !kHasPosition) {
            setFailure("position_actual is required for multi-pass fine homing");
            rx->target_torque = 0;
            return;
        }

        int64_t position = 0;
        if constexpr (kHasPosition) {
            position = static_cast<int64_t>(tx->position_actual);
            has_position_data_ = true;
        }

        pass_positions_.push_back(position);
        TETHER_LOGI("sensorless_homing",
                    "Slave {} pass {}/{} recorded at {}",
                    drive.slaveIndex(), pass_positions_.size(),
                    config_.fine_homing_passes, position);

        stall_timer_ = 0.0;
        started_moving_ = false;

        if (pass_positions_.size() >= static_cast<size_t>(config_.fine_homing_passes)) {
            finishHoming(drive, rx);
        } else {
            enterBackoff(drive, tx);
        }
    }

    void enterBackoff(CiA402Drive& drive, const TxPDO* tx)
    {
        state_ = PassState::BackingOff;
        current_reference_ = backoff_reference_;
        started_moving_ = false;
        stall_timer_ = 0.0;
        phase_timer_ = 0.0;
        pi_.reset();

        if constexpr (requires(TxPDO& p) { p.position_actual; }) {
            backoff_has_position_ = true;
            backoff_start_position_ = tx->position_actual;
        } else {
            backoff_has_position_ = false;
            backoff_timer_ = 0.0;
        }

        TETHER_LOGI("sensorless_homing",
                    "Slave {} backing off for next pass",
                    drive.slaveIndex());
    }

    bool backoffDone(const TxPDO* tx, double dt_seconds)
    {
        if constexpr (requires(TxPDO& p) { p.position_actual; }) {
            if (backoff_has_position_) {
                // Subtract in double to avoid int32 overflow at wraparound.
                return std::abs(static_cast<double>(tx->position_actual) -
                                static_cast<double>(backoff_start_position_)) >=
                       std::abs(config_.backoff_distance);
            }
        }

        backoff_timer_ += dt_seconds;
        const double needed_time = std::abs(config_.backoff_distance / current_reference_);
        return backoff_timer_ >= needed_time;
    }

    void enterApproach()
    {
        state_ = PassState::Approaching;
        current_reference_ = fine_reference_;
        started_moving_ = false;
        stall_timer_ = 0.0;
        phase_timer_ = 0.0;
        pi_.reset();
    }

    void finishHoming(CiA402Drive& drive, RxPDO* rx)
    {
        int64_t total = 0;
        for (int64_t p : pass_positions_) {
            total += p;
        }

        int64_t result = 0;
        if (config_.pass_aggregation == PassAggregation::Mean) {
            result = static_cast<int64_t>(std::llround(
                static_cast<double>(total) / static_cast<double>(pass_positions_.size())));
        } else {
            result = total;
        }

        home_position_.store(result, std::memory_order_release);
        home_position_valid_.store(has_position_data_, std::memory_order_release);
        virtual_switch_position_.store(result, std::memory_order_release);
        virtual_switch_position_valid_.store(has_position_data_, std::memory_order_release);
        virtual_switch_active_.store(true, std::memory_order_release);
        homed_.store(true, std::memory_order_release);
        state_ = PassState::Done;
        rx->target_torque = 0;

        TETHER_LOGI("sensorless_homing",
                    "Slave {} homed at {} ({} passes, aggregation={})",
                    drive.slaveIndex(), result, pass_positions_.size(),
                    config_.pass_aggregation == PassAggregation::Mean ? "mean" : "sum");
    }

    Config config_;
    double reference_velocity_;   // first (coarse) approach reference
    double fine_magnitude_;       // magnitude used for back-off and re-approach
    double fine_reference_;       // re-approach reference (passes 2..N)
    double backoff_reference_;    // back-off reference (-fine_reference_)
    double current_reference_;    // active PI setpoint for the current phase
    double max_permille_;
    tether::control::PIController pi_;
    std::atomic<bool> homed_{false};
    std::atomic<bool> failed_{false};
    std::atomic<const char*> failure_message_{""};
    std::atomic<int64_t> home_position_{0};
    std::atomic<bool> home_position_valid_{false};
    std::atomic<int64_t> virtual_switch_position_{0};
    std::atomic<bool> virtual_switch_position_valid_{false};
    std::atomic<bool> virtual_switch_active_{false};
    std::vector<int64_t> pass_positions_;
    bool has_position_data_ = false;
    bool started_moving_ = false;
    bool mode_ok_ = false;
    double stall_timer_ = 0.0;
    double startup_timer_ = 0.0;
    double phase_timer_ = 0.0;
    PassState state_ = PassState::Approaching;
    bool backoff_has_position_ = false;
    int32_t backoff_start_position_ = 0;
    double backoff_timer_ = 0.0;
    static constexpr double kModeVerifyDelaySeconds = 0.2;
};

} // namespace EtherCAT
