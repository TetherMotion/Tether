#pragma once

#include <atomic>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <limits>
#include <type_traits>
#include <vector>

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

    /// How a stall (axis reached the mechanical stop) is detected.
    enum class StallDetection {
        /// |speed_feedback| stays below stall_velocity for stall_time.
        /// Noisy on drives whose 0x606C is derived from position at the
        /// cycle rate — kept for compatibility and drives with clean
        /// velocity estimation.
        Speed,
        /// Position does not change by more than stall_position_counts
        /// within stall_window, sustained for stall_time.  position_actual
        /// is a clean counter, so this is the most noise-immune method.
        Position,
        /// |torque_actual| stays at/above stall_torque_permille for
        /// stall_time — the drive is pressing against the stop.
        Torque,
    };

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
        bool use_csv_mode = false;         // true: drive-internal velocity loop (CSV),
                                           // false: bounded CST torque command
        StallDetection stall_detection = StallDetection::Position;
        double stall_velocity = 100.0;     // counts/s (StallDetection::Speed)
        double stall_window = 0.1;         // s, position-delta window (Position)
        double stall_position_counts = 20.0; // counts per stall_window (Position)
        double stall_torque_permille = -1.0; // Torque mode; <0 = 80% of max_torque
        double stall_time = 1.0;           // seconds below threshold
        uint32_t home_position_avg_samples = 250; // positions averaged per pass
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

        constexpr bool kHasPosition = requires(TxPDO& p) { p.position_actual; };
        constexpr bool kHasTorque = requires(TxPDO& p) { p.torque_actual; };
        if (config_.stall_detection == StallDetection::Position && !kHasPosition) {
            setFailure("position_actual is required for position stall detection");
            return false;
        }
        if (config_.stall_detection == StallDetection::Torque && !kHasTorque) {
            setFailure("torque_actual is required for torque stall detection");
            return false;
        }

        const bool mode_ok = config_.use_csv_mode ? drive.setModeCSV()
                                                  : drive.setModeCST();
        if (!mode_ok) {
            setFailure(config_.use_csv_mode
                           ? "Failed to set Cyclic Sync Velocity mode"
                           : "Failed to set Cyclic Sync Torque mode");
            return false;
        }

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

        const size_t hist_cap = static_cast<size_t>(
            std::clamp(config_.stall_window, 0.001, 10.0) * 10000.0) + 2;
        pos_hist_.assign(hist_cap, PosSample{0.0, 0});
        pos_hist_head_ = 0;
        pos_hist_tail_ = 0;
        pos_hist_count_ = 0;

        const size_t avg_cap = static_cast<size_t>(
            std::clamp(config_.home_position_avg_samples, 1u, 4096u));
        pos_avg_ring_.assign(avg_cap, 0);
        pos_avg_head_ = 0;
        pos_avg_count_ = 0;

        const char* trace_path = std::getenv("TETHER_SENSORLESS_HOMING_TRACE");
        if (trace_path != nullptr && *trace_path != '\0') {
            trace_file_ = std::fopen(trace_path, "w");
            if (trace_file_ != nullptr) {
                std::setvbuf(trace_file_, nullptr, _IOFBF, 1 << 20);
                std::fputs("t_s,state,ref,speed,position,torque_actual,cmd\n",
                           trace_file_);
                trace_.reserve(kTraceChunkSize);
            }
        }

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
            zeroCommand(rx);
        }
        flushTrace();
    }

    bool update(CiA402Drive& drive, double dt_seconds) override
    {
        auto* rx = drive.rxPDO<RxPDO>();
        auto* tx = drive.txPDO<TxPDO>();
        if (rx == nullptr || tx == nullptr) {
            setFailure("PDO buffers became invalid");
            return true;
        }

        const int8_t op_mode = config_.use_csv_mode
            ? static_cast<int8_t>(CiA402::OperatingMode::CyclicSyncVelocity)
            : static_cast<int8_t>(CiA402::OperatingMode::CyclicSyncTorque);

        rx->controlword = static_cast<uint16_t>(CiA402::ControlWord::EnableOperation);
        rx->modes_of_operation = op_mode;

        if constexpr (requires(RxPDO& p) { p.target_position; }) {
            rx->target_position = 0;
        }
        if (!config_.use_csv_mode) {
            if constexpr (requires(RxPDO& p) { p.target_velocity; }) {
                rx->target_velocity = 0;
            }
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
            if (tx->modes_of_operation_display != op_mode) {
                setFailure(config_.use_csv_mode
                               ? "Drive is not in Cyclic Sync Velocity mode"
                               : "Drive is not in Cyclic Sync Torque mode");
            } else if ((tx->statusword & CiA402::StatuswordBits::OperationEnabled) == 0) {
                setFailure("Drive is not operation-enabled");
            } else {
                mode_ok_ = true;
            }
        }

        if (failed_.load(std::memory_order_acquire)) {
            zeroCommand(rx);
            return true;
        }

        if (mode_ok_) {
            if (tx->modes_of_operation_display != op_mode) {
                setFailure("Drive left the commanded operating mode");
                zeroCommand(rx);
                return true;
            }
            if ((tx->statusword & CiA402::StatuswordBits::OperationEnabled) == 0) {
                setFailure("Drive lost operation-enabled state");
                zeroCommand(rx);
                return true;
            }
        }

        if (state_ == PassState::Done) {
            zeroCommand(rx);
            return true;
        }

        // Per-phase watchdog: an approach or back-off that makes no progress
        // (e.g. axis already at the stop, torque too low to move) must fail
        // instead of hanging forever.
        if (config_.phase_timeout > 0.0) {
            phase_timer_ += dt_seconds;
            if (phase_timer_ >= config_.phase_timeout) {
                setFailure("Homing phase timed out — no progress");
                zeroCommand(rx);
                return true;
            }
        }

        const double speed = static_cast<double>(tx->speed_feedback);

        double windowed_dpos = 0.0;
        if constexpr (requires(TxPDO& p) { p.position_actual; }) {
            const int64_t pos_now = static_cast<int64_t>(tx->position_actual);
            pushPositionSample(startup_timer_, pos_now);
            pushAvgSample(pos_now);
            windowed_dpos = windowedPositionDelta(pos_now);
        }

        if (state_ == PassState::Approaching) {
            // Arm stall detection only once the axis is actually moving in
            // the approach direction. Requiring a matching sign prevents a
            // phantom stall during the reversal when transitioning from
            // BackingOff to Approaching.
            if (!started_moving_) {
                bool moved = false;
                switch (config_.stall_detection) {
                    case StallDetection::Speed:
                        moved = std::abs(speed) > config_.stall_velocity &&
                                speed * current_reference_ > 0.0;
                        break;
                    case StallDetection::Position:
                    case StallDetection::Torque:
                        if constexpr (requires(TxPDO& p) { p.position_actual; }) {
                            moved = std::abs(windowed_dpos) > config_.stall_position_counts &&
                                    windowed_dpos * current_reference_ > 0.0;
                        } else {
                            moved = std::abs(speed) > config_.stall_velocity &&
                                    speed * current_reference_ > 0.0;
                        }
                        break;
                }
                if (moved) {
                    started_moving_ = true;
                    TETHER_LOGI("sensorless_homing",
                                "Slave {} started moving: speed={} counts/s",
                                drive.slaveIndex(), speed);
                }
            }

            if (started_moving_) {
                bool stalled_now = false;
                switch (config_.stall_detection) {
                    case StallDetection::Speed:
                        stalled_now = std::abs(speed) <= config_.stall_velocity;
                        break;
                    case StallDetection::Position:
                        stalled_now =
                            std::abs(windowed_dpos) <= config_.stall_position_counts;
                        break;
                    case StallDetection::Torque: {
                        double tq = 0.0;
                        if constexpr (requires(TxPDO& p) { p.torque_actual; }) {
                            tq = static_cast<double>(tx->torque_actual);
                        }
                        const double thr = config_.stall_torque_permille >= 0.0
                            ? config_.stall_torque_permille
                            : 0.8 * max_permille_;
                        stalled_now = std::abs(tq) >= thr;
                        break;
                    }
                }

                if (stalled_now) {
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

        if (config_.use_csv_mode) {
            // Drive-internal velocity loop: just stream the velocity setpoint.
            // The torque limit must be enforced via 0x60E0/0x60E1 (SDO or a
            // PDO that carries the torque-limit fields).
            if constexpr (requires(RxPDO& p) { p.target_velocity; }) {
                rx->target_velocity =
                    static_cast<int32_t>(std::llround(current_reference_));
            }
            if constexpr (requires(RxPDO& p) { p.target_torque; }) {
                rx->target_torque = 0;
            }
            traceSample(tx, rx->target_torque, speed);
            return true;
        }

        rx->target_torque = static_cast<int16_t>(std::llround(
            std::copysign(max_permille_, current_reference_)));

        traceSample(tx, rx->target_torque, speed);
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

    ~SensorlessTorqueHomingController() override { flushTrace(); }

private:
    struct TraceSample {
        double t;
        double ref;
        double speed;
        int32_t position;
        int16_t torque_actual;
        int16_t cmd;
        int8_t state;
    };

    static constexpr size_t kTraceChunkSize = 2048;

    void traceSample(const TxPDO* tx, int16_t cmd, double speed)
    {
        if (trace_file_ == nullptr) {
            return;
        }
        TraceSample s{};
        s.t = startup_timer_;
        s.ref = current_reference_;
        s.speed = speed;
        s.cmd = cmd;
        s.state = static_cast<int8_t>(state_);
        if constexpr (requires(TxPDO& p) { p.position_actual; }) {
            s.position = tx->position_actual;
        }
        if constexpr (requires(TxPDO& p) { p.torque_actual; }) {
            s.torque_actual = tx->torque_actual;
        }
        trace_.push_back(s);
        if (trace_.size() >= kTraceChunkSize) {
            flushTrace();
        }
    }

    void flushTrace()
    {
        if (trace_file_ == nullptr) {
            return;
        }
        for (const TraceSample& s : trace_) {
            std::fprintf(trace_file_, "%.3f,%d,%.1f,%.1f,%d,%d,%d\n",
                         s.t, static_cast<int>(s.state), s.ref, s.speed,
                         s.position, static_cast<int>(s.torque_actual),
                         static_cast<int>(s.cmd));
        }
        trace_.clear();
        std::fflush(trace_file_);
    }


    struct PosSample {
        double t;
        int64_t pos;
    };

    void pushPositionSample(double t, int64_t pos)
    {
        if (pos_hist_.empty()) {
            return;
        }
        pos_hist_[pos_hist_head_] = PosSample{t, pos};
        pos_hist_head_ = (pos_hist_head_ + 1) % pos_hist_.size();
        if (pos_hist_count_ < pos_hist_.size()) {
            ++pos_hist_count_;
        } else {
            pos_hist_tail_ = (pos_hist_tail_ + 1) % pos_hist_.size();
        }
        const double cutoff = t - config_.stall_window;
        while (pos_hist_count_ > 0 && pos_hist_[pos_hist_tail_].t < cutoff) {
            pos_hist_tail_ = (pos_hist_tail_ + 1) % pos_hist_.size();
            --pos_hist_count_;
        }
    }

    /// Position change over the trailing stall_window seconds
    /// (or the oldest available sample while the window fills).
    double windowedPositionDelta(int64_t pos_now) const
    {
        if (pos_hist_count_ == 0) {
            return 0.0;
        }
        return static_cast<double>(pos_now - pos_hist_[pos_hist_tail_].pos);
    }

    void clearPositionHistory()
    {
        pos_hist_head_ = 0;
        pos_hist_tail_ = 0;
        pos_hist_count_ = 0;
    }

    void pushAvgSample(int64_t pos)
    {
        if (pos_avg_ring_.empty()) {
            return;
        }
        pos_avg_ring_[pos_avg_head_] = pos;
        pos_avg_head_ = (pos_avg_head_ + 1) % pos_avg_ring_.size();
        if (pos_avg_count_ < pos_avg_ring_.size()) {
            ++pos_avg_count_;
        }
    }

    /// Mean of the last home_position_avg_samples positions, or pos_now if
    /// averaging is disabled / no samples collected yet.
    int64_t averagedPosition(int64_t pos_now) const
    {
        if (pos_avg_count_ == 0) {
            return pos_now;
        }
        int64_t total = 0;
        for (size_t i = 0; i < pos_avg_count_; ++i) {
            total += pos_avg_ring_[i];
        }
        return static_cast<int64_t>(std::llround(
            static_cast<double>(total) / static_cast<double>(pos_avg_count_)));
    }

    static void zeroCommand(RxPDO* rx)
    {
        if constexpr (requires(RxPDO& p) { p.target_torque; }) {
            rx->target_torque = 0;
        }
        if constexpr (requires(RxPDO& p) { p.target_velocity; }) {
            rx->target_velocity = 0;
        }
    }

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
            zeroCommand(rx);
            return;
        }

        int64_t position = 0;
        if constexpr (kHasPosition) {
            position = averagedPosition(static_cast<int64_t>(tx->position_actual));
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
        clearPositionHistory();

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
        clearPositionHistory();
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
        zeroCommand(rx);

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
    double current_reference_;    // active setpoint for the current phase
    double max_permille_;
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
    std::vector<PosSample> pos_hist_;
    size_t pos_hist_head_ = 0;
    size_t pos_hist_tail_ = 0;
    size_t pos_hist_count_ = 0;
    std::vector<int64_t> pos_avg_ring_;
    size_t pos_avg_head_ = 0;
    size_t pos_avg_count_ = 0;
    std::vector<TraceSample> trace_;
    std::FILE* trace_file_ = nullptr;
    static constexpr double kModeVerifyDelaySeconds = 0.2;
};

} // namespace EtherCAT
