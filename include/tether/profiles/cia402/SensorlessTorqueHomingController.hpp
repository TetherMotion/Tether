#pragma once

#include <atomic>
#include <cmath>
#include <cstdint>
#include <limits>
#include <type_traits>

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
 * becomes true and the application can call the drive's homing routine
 * (e.g. CiA402Drive::homeToCurrentPosition()).
 *
 * The class is templated on the drive-specific packed RxPDO/TxPDO structs.
 * The RxPDO must expose controlword, modes_of_operation and target_torque;
 * the TxPDO must expose statusword, modes_of_operation_display and
 * speed_feedback. This is verified at compile time with static_assert.
 *
 * Mode of operation is verified both at startup (after a short delay to let
 * the slave switch to CST) and continuously while the controller is running.
 */
template<typename RxPDO, typename TxPDO>
class SensorlessTorqueHomingController : public DS402Master::IDriveMotionController {
public:
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
        double target_velocity = 2000.0; // counts/s (magnitude)
        int direction = 1;               // +1 or -1
        double max_torque_percent = 1.0; // % of rated
        double kp = 0.05;
        double ki = 0.005;
        double stall_velocity = 100.0;   // counts/s
        double stall_time = 1.0;         // seconds below threshold
    };

    explicit SensorlessTorqueHomingController(const Config& config)
        : config_(config)
        , reference_velocity_(static_cast<double>(config.direction) * config.target_velocity)
        , max_permille_(std::min(1000.0, config.max_torque_percent * 10.0))
    {
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
        started_moving_ = false;
        mode_ok_ = false;
        stall_timer_ = 0.0;
        startup_timer_ = 0.0;

        TETHER_LOGI("sensorless_homing",
                    "Started on slave {}: reference={} counts/s, max_torque={}%",
                    drive.slaveIndex(), reference_velocity_, config_.max_torque_percent);
        return true;
    }

    void stop(CiA402Drive&) override
    {
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

        if (homed_.load(std::memory_order_acquire)) {
            rx->target_torque = 0;
            return true;
        }

        const double speed = static_cast<double>(tx->speed_feedback);

        if (!started_moving_ && std::abs(speed) > config_.stall_velocity) {
            started_moving_ = true;
            TETHER_LOGI("sensorless_homing",
                        "Slave {} started moving: speed={} counts/s",
                        drive.slaveIndex(), speed);
        }

        if (started_moving_) {
            if (std::abs(speed) <= config_.stall_velocity) {
                stall_timer_ += dt_seconds;
                if (stall_timer_ >= config_.stall_time) {
                    TETHER_LOGI("sensorless_homing",
                                "Slave {} stall detected (speed={} counts/s, time={} s); homed",
                                drive.slaveIndex(), speed, stall_timer_);
                    homed_.store(true, std::memory_order_release);
                    rx->target_torque = 0;
                    return true;
                }
            } else {
                stall_timer_ = 0.0;
            }
        }

        tether::control::ControllerInput input;
        input.reference = reference_velocity_;
        input.measured = speed;
        input.dt = dt_seconds;
        input.enable = true;

        const auto output = pi_.compute(input);
        rx->target_torque = static_cast<int16_t>(std::llround(std::clamp(
            output.control, -max_permille_, max_permille_)));
        return true;
    }

    bool isHomed() const { return homed_.load(std::memory_order_acquire); }
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

    Config config_;
    double reference_velocity_;
    double max_permille_;
    tether::control::PIController pi_;
    std::atomic<bool> homed_{false};
    std::atomic<bool> failed_{false};
    std::atomic<const char*> failure_message_{""};
    bool started_moving_ = false;
    bool mode_ok_ = false;
    double stall_timer_ = 0.0;
    double startup_timer_ = 0.0;
    static constexpr double kModeVerifyDelaySeconds = 0.2;
};

} // namespace EtherCAT
