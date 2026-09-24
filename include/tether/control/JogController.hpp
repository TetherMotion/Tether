/**
 * @file JogController.hpp
 * @brief Generic failsafe jogging controller for N axes.
 *
 * JogController provides operator-style manual motion (continuous jog and
 * bounded incremental moves) for an arbitrary number of axes.
 *
 * ## Architecture
 *
 * The controller is deliberately split into three layers so it stays
 * drive- and application-agnostic:
 *
 *   1. **Command/lease layer** (this class) — validates and clamps
 *      commands, owns the lease state machine, and arbitrates between
 *      continuous, incremental and stopping behaviour.
 *   2. **Ramp layer** (`IJogRamper`) — per-axis trajectory generator
 *      producing the position/rate profile.  The default
 *      `TrapezoidJogRamper` implements accel-limited trapezoidal ramps;
 *      hosts may inject a different generator (S-curve, jerk-limited,
 *      drive-internal profile, ...) via `Config::ramperFactory`.
 *   3. **Application layer** (`JogApplyFn`) — a callback the host installs
 *      to consume each axis's ramped setpoint on the cyclic thread
 *      (write it to a CSP target, a pulse generator, a simulator...).
 *      The controller never touches a target position itself.
 *
 * ## Failsafe model
 *
 * Continuous jogging is *lease-based*.  Every jog() call refreshes a
 * deadline; the cyclic update() — not the commanding thread — enforces
 * it.  If commands stop arriving for any reason (client crash, network
 * partition, half-open TCP), the ramper is told to stop and the axis
 * decelerates within the lease window.  Incremental move() requests are
 * inherently failsafe because each one is a single bounded displacement.
 *
 * All limits are enforced server-side: rate is clamped to `maxRate`, the
 * lease is clamped to `maxLeaseMs`, and incremental requests are clamped
 * to `maxIncrement`.  A client cannot exceed them by sending larger values.
 *
 * ## Threading
 *
 * jog()/move()/stop() may be called from any thread (typically IO session
 * threads); they serialize on a short per-axis mutex and publish through
 * atomic mailboxes.  update() runs on the single cyclic/motion thread,
 * is fully lock-free, and is the only accessor of the ramper objects —
 * they are single-writer.  Output position/rate are mirrored into atomics
 * for cross-thread queries.
 *
 * @copyright Copyright (C) 2025-2026 Tether Authors
 */
#pragma once

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <deque>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

namespace tether::control {

// ============================================================================
// IJogRamper — pluggable per-axis trajectory generator
// ============================================================================

/**
 * @brief Ramp generator used by JogController for one axis.
 *
 * A ramper converts high-level jog demands into a position/rate profile:
 *
 *   - setRate(v)    — track a velocity command (continuous jog)
 *   - setTarget(p)  — ramp to an absolute position (incremental jog)
 *   - stop()        — decelerate to rest (lease expiry, stop request)
 *
 * Implementations are driven exclusively from JogController::update() on
 * the cyclic thread — they need not be thread-safe themselves.
 */
class IJogRamper {
public:
    virtual ~IJogRamper() = default;

    /// Track a velocity command (units/s).  Switches to velocity mode.
    virtual void setRate(double rate) = 0;
    /// Ramp to an absolute jog position.  Switches to position mode.
    virtual void setTarget(double position) = 0;
    /// Decelerate to rest and hold position.
    virtual void stop() = 0;
    /// Advance the profile by dt seconds.
    virtual void update(double dt) = 0;

    /// Current profile position.
    virtual double position() const = 0;
    /// Current profile velocity.
    virtual double rate() const = 0;
    /// Current position target (position mode; position() otherwise).
    virtual double target() const = 0;
    /// True when the ramper has no work left: at rest after stop() or
    /// after reaching a position target.  Never true in velocity mode.
    virtual bool finished() const = 0;
    /// Re-base the profile position (e.g. after homing).  Keeps the phase.
    virtual void reset(double position) = 0;
};

// ============================================================================
// TrapezoidJogRamper — default accel-limited implementation
// ============================================================================

/**
 * @brief Default ramper: accel-limited velocity slewing and trapezoidal
 *        point-to-point moves (v = sqrt(2·a·d) deceleration law).
 */
class TrapezoidJogRamper : public IJogRamper {
public:
    struct Config {
        double maxRate = 50.0;   ///< |rate| cap, units/s
        double accel   = 500.0;  ///< rate slew limit, units/s^2
        double epsilon = 1e-4;   ///< completion tolerance, units
    };

    explicit TrapezoidJogRamper(const Config& cfg) : m_cfg(cfg) {}

    void setRate(double rate) override {
        m_phase = Phase::Velocity;
        m_targetRate = rate;
    }

    void setTarget(double position) override {
        m_phase = Phase::Position;
        m_targetPos = position;
    }

    void stop() override {
        if (m_phase != Phase::Idle) m_phase = Phase::Decel;
    }

    void update(double dt) override {
        switch (m_phase) {
        case Phase::Velocity:
            slew(m_targetRate, dt);
            break;

        case Phase::Position: {
            const double err = m_targetPos - m_position;
            const double dir = err < 0.0 ? -1.0 : 1.0;
            slew(dir * std::min(m_cfg.maxRate,
                                std::sqrt(2.0 * m_cfg.accel * std::abs(err))),
                 dt);
            // Consume the remainder exactly once the step covers it.
            const double step = m_rate * dt;
            if (std::abs(err - step) <= m_cfg.epsilon) {
                m_position = m_targetPos;
                m_rate = 0.0;
                m_phase = Phase::Idle;
                return;
            }
            break;
        }

        case Phase::Decel:
            slew(0.0, dt);
            if (std::abs(m_rate) <= std::max(m_cfg.epsilon,
                                             m_cfg.accel * dt)) {
                m_rate = 0.0;
                m_phase = Phase::Idle;
            }
            break;

        case Phase::Idle:
        default:
            m_rate = 0.0;
            return;
        }
        m_position += m_rate * dt;
    }

    double position() const override { return m_position; }
    double rate() const override { return m_rate; }
    double target() const override {
        return m_phase == Phase::Position ? m_targetPos : m_position;
    }
    bool finished() const override { return m_phase == Phase::Idle; }

    void reset(double position) override {
        const double off = position - m_position;
        m_position = position;
        m_targetPos += off;
    }

private:
    /// Slew m_rate toward cmd at the accel limit.
    void slew(double cmd, double dt) {
        const double maxStep = m_cfg.accel * dt;
        m_rate += std::clamp(cmd - m_rate, -maxStep, maxStep);
    }

    enum class Phase { Idle, Velocity, Position, Decel };

    Config m_cfg;
    Phase  m_phase{Phase::Idle};
    double m_position{0.0};
    double m_rate{0.0};
    double m_targetRate{0.0};
    double m_targetPos{0.0};
};

// ============================================================================
// JogSetpoint / JogApplyFn — the application abstraction
// ============================================================================

/// One cycle of ramped jog output for one axis, handed to the applicator.
struct JogSetpoint {
    size_t  axis;         ///< Axis index
    uint8_t mode;         ///< JogController::Mode as uint8_t
    bool    active;       ///< mode != Idle
    double  position;     ///< Ramped jog position (units)
    double  rate;         ///< Ramped jog rate (units/s)
    double  remaining;    ///< Incremental distance left (units)
    uint32_t leaseLeftMs; ///< Continuous mode: ms until lease expiry
    int64_t nowMs;        ///< Controller clock at this update
};

/**
 * @brief Host-provided consumer of ramped jog setpoints.
 *
 * Called synchronously from JogController::update() on the cyclic thread
 * once per axis per cycle — implementations must be bounded and RT-cheap
 * (e.g. write into a PDO image, a pulse accumulator, or a test capture).
 */
using JogApplyFn = std::function<void(const JogSetpoint&)>;

// ============================================================================
// JogController
// ============================================================================

class JogController {
public:
    /// Axis runtime mode (also emitted in the IO state snapshot).
    enum class Mode : uint8_t {
        Idle        = 0, ///< Holding position, no jog command
        Continuous  = 1, ///< Leased velocity jog in progress
        Incremental = 2, ///< Bounded displacement in progress
        Stopping    = 3, ///< Lease expired / stop requested — ramping down
    };

    /// Server-enforced limits and UI presets for one axis.
    struct AxisConfig {
        std::string name;                 ///< "x", "y", "z", "e", ...
        std::string unit = "mm";          ///< Display/engineering unit
        double      maxRate = 50.0;       ///< |rate| clamp, units/s
        double      accel   = 500.0;      ///< Rate slew limit, units/s^2
        uint32_t    maxLeaseMs = 500;     ///< Hard lease cap, ms
        double      maxIncrement = 10.0;  ///< |move()| clamp, units
        std::vector<double> ratePresets      = {1.0, 10.0, 50.0};  ///< UI rate chips
        std::vector<double> incrementPresets = {0.1, 1.0, 10.0};   ///< UI step chips
    };

    /// Named subset of axes with optional preset overrides (UI grouping).
    struct Group {
        std::string          name;              ///< "linear", "extruder", ...
        std::vector<size_t>  axes;              ///< Indices into Config::axes
        std::vector<double>  ratePresets;       ///< Empty = inherit per-axis
        std::vector<double>  incrementPresets;  ///< Empty = inherit per-axis
    };

    /// Factory creating one ramper per axis.  Default: TrapezoidJogRamper
    /// built from the axis limits.
    using RamperFactory =
        std::function<std::unique_ptr<IJogRamper>(const AxisConfig&)>;

    struct Config {
        std::vector<AxisConfig> axes;
        std::vector<Group>      groups;
        uint32_t defaultLeaseMs = 300;   ///< Lease when client passes 0
        double   stopEpsilon = 1e-4;     ///< |rate| below which Stopping→Idle

        /// Optional custom ramper factory (empty = TrapezoidJogRamper).
        RamperFactory ramperFactory;

        static Config getDefault() {
            Config cfg;
            cfg.axes = {AxisConfig{.name = "x"}, AxisConfig{.name = "y"},
                        AxisConfig{.name = "z", .maxRate = 10.0, .accel = 100.0,
                                   .incrementPresets = {0.05, 0.5, 5.0}}};
            cfg.groups = {Group{.name = "all", .axes = {0, 1, 2}}};
            return cfg;
        }
    };

    /// Read-only per-axis snapshot for IO transport (POD, fixed layout).
    struct AxisSnapshot {
        uint8_t  mode;          ///< Mode as uint8_t
        uint8_t  active;        ///< mode != Idle
        uint8_t  _pad[6];
        double   rate;          ///< Ramped output rate, units/s
        double   position;      ///< Accumulated jog position, units
        double   remaining;     ///< Incremental distance left, units
        uint32_t leaseLeftMs;   ///< Continuous mode: ms until expiry
        uint32_t maxLeaseMs;    ///< Configured cap
        double   maxRate;       ///< Configured cap
        double   maxIncrement;  ///< Configured cap
    };

    explicit JogController(const Config& cfg) : m_cfg(cfg), m_axes(cfg.axes.size()) {
        for (size_t i = 0; i < cfg.axes.size(); ++i) {
            m_axes[i].ramper = cfg.ramperFactory
                                   ? cfg.ramperFactory(cfg.axes[i])
                                   : makeTrapezoidRamper(cfg.axes[i]);
        }
    }

    JogController() : JogController(Config::getDefault()) {}

    /**
     * @brief Install the setpoint applicator.
     *
     * The callback runs on the update() thread once per axis per cycle.
     * Keep it bounded and RT-cheap — it is where the host should write
     * the ramped setpoint into its own output path (PDO, pulses, sim).
     */
    void setApplyCallback(JogApplyFn fn) { m_apply = std::move(fn); }

    // ------------------------------------------------------------------
    // Configuration / metadata
    // ------------------------------------------------------------------

    const Config& config() const { return m_cfg; }
    size_t axisCount() const { return m_axes.size(); }
    const AxisConfig& axisConfig(size_t i) const { return m_cfg.axes.at(i); }

    // ------------------------------------------------------------------
    // Commands — safe from any thread
    // ------------------------------------------------------------------

    /**
     * @brief Continuous jog lease refresh.
     *
     * @param axis     Axis index.
     * @param rate     Commanded rate in units/s — clamped to ±maxRate
     *                 server-side.  0 keeps the lease alive while holding.
     * @param leaseMs  Requested lease; clamped to [1, maxLeaseMs]; 0 selects
     *                 Config::defaultLeaseMs.
     * @param nowMs    Caller's millisecond clock (same source as update()).
     * @return false on invalid axis or non-finite rate.
     */
    bool jog(size_t axis, double rate, uint32_t leaseMs, int64_t nowMs) {
        if (axis >= m_axes.size() || !std::isfinite(rate)) return false;
        auto& a = m_axes[axis];
        const auto& cfg = m_cfg.axes[axis];
        const uint32_t lease = std::clamp(
            leaseMs == 0 ? m_cfg.defaultLeaseMs : leaseMs, 1u, cfg.maxLeaseMs);
        std::lock_guard lock(a.cmdMutex);
        a.cmdRate.store(std::clamp(rate, -cfg.maxRate, cfg.maxRate),
                        std::memory_order_relaxed);
        a.leaseEndMs.store(nowMs + static_cast<int64_t>(lease),
                           std::memory_order_relaxed);
        // A continuous command takes ownership: drop any pending increment.
        a.pendingIncrement.store(0.0, std::memory_order_relaxed);
        a.mode.store(static_cast<uint8_t>(Mode::Continuous),
                     std::memory_order_release);
        return true;
    }

    /**
     * @brief Bounded incremental move.
     *
     * The request is clamped to ±maxIncrement and accumulated into the
     * axis's pending-increment mailbox (consumed by update() into the
     * ramper's position target).  Accepted only while the axis is Idle or
     * already Incremental — a continuous jog (or its ramp-down) owns the
     * axis and rejects the request.
     *
     * @return The applied (clamped) displacement, or 0.0 when rejected.
     */
    double move(size_t axis, double distance, int64_t /*nowMs*/) {
        if (axis >= m_axes.size() || !std::isfinite(distance)) return 0.0;
        auto& a = m_axes[axis];
        const double applied = std::clamp(
            distance, -m_cfg.axes[axis].maxIncrement, m_cfg.axes[axis].maxIncrement);
        std::lock_guard lock(a.cmdMutex);
        const Mode mode =
            static_cast<Mode>(a.mode.load(std::memory_order_acquire));
        if (mode == Mode::Continuous || mode == Mode::Stopping)
            return 0.0;
        a.pendingIncrement.fetch_add(applied, std::memory_order_relaxed);
        a.mode.store(static_cast<uint8_t>(Mode::Incremental),
                     std::memory_order_release);
        return applied;
    }

    /// Request a ramped stop for one axis (convenience — the lease is the
    /// real failsafe).  No-op when already Idle.
    void stop(size_t axis) {
        if (axis >= m_axes.size()) return;
        auto& a = m_axes[axis];
        std::lock_guard lock(a.cmdMutex);
        if (static_cast<Mode>(a.mode.load(std::memory_order_acquire)) !=
            Mode::Idle)
            a.mode.store(static_cast<uint8_t>(Mode::Stopping),
                         std::memory_order_release);
    }

    /// Stop every axis.
    void stopAll() {
        for (size_t i = 0; i < m_axes.size(); ++i) stop(i);
    }

    /**
     * @brief Re-base the accumulated jog position (e.g. after a homing or
     *        coordinate change).  Applied by the next update() cycle.
     */
    void resetPosition(size_t axis, double position = 0.0) {
        if (axis >= m_axes.size()) return;
        m_axes[axis].resetRequest.store(position, std::memory_order_relaxed);
        m_axes[axis].resetPending.store(true, std::memory_order_release);
    }

    // ------------------------------------------------------------------
    // Cyclic update — call from the motion thread every cycle
    // ------------------------------------------------------------------

    /**
     * @brief Advance all axes by dt seconds, enforce lease expiry, drive
     *        the ramplers and push setpoints to the applicator.
     *
     * @param nowMs Millisecond clock — must be the same time source passed
     *              to jog() and monotonic (not wall clock).
     * @param dt    Cycle period in seconds.
     */
    void update(int64_t nowMs, double dt) {
        for (size_t i = 0; i < m_axes.size(); ++i) {
            auto& a = m_axes[i];
            const Mode mode =
                static_cast<Mode>(a.mode.load(std::memory_order_acquire));

            // Deferred position re-base (resetPosition()).
            if (a.resetPending.load(std::memory_order_acquire)) {
                a.ramper->reset(
                    a.resetRequest.load(std::memory_order_relaxed));
                a.resetPending.store(false, std::memory_order_release);
            }

            switch (mode) {
            case Mode::Continuous:
                if (nowMs >= a.leaseEndMs.load(std::memory_order_relaxed)) {
                    // Lease expired — decelerate at the profile limit.
                    a.mode.store(static_cast<uint8_t>(Mode::Stopping),
                                 std::memory_order_release);
                    a.ramper->stop();
                } else {
                    a.ramper->setRate(
                        a.cmdRate.load(std::memory_order_relaxed));
                }
                break;

            case Mode::Incremental: {
                // Consume the pending-increment mailbox into the ramper's
                // absolute target (atomic — a concurrent move() can add).
                const double add =
                    a.pendingIncrement.exchange(0.0, std::memory_order_relaxed);
                if (add != 0.0)
                    a.ramper->setTarget(a.ramper->target() + add);
                if (a.ramper->finished() &&
                    a.pendingIncrement.load(std::memory_order_relaxed) == 0.0)
                    a.mode.store(static_cast<uint8_t>(Mode::Idle),
                                 std::memory_order_release);
                break;
            }

            case Mode::Stopping:
                a.ramper->stop();
                if (a.ramper->finished())
                    a.mode.store(static_cast<uint8_t>(Mode::Idle),
                                 std::memory_order_release);
                break;

            case Mode::Idle:
            default:
                break;
            }

            a.ramper->update(dt);

            // Post-transition mode (lease expiry may have stored Stopping).
            const Mode modeOut =
                static_cast<Mode>(a.mode.load(std::memory_order_acquire));

            // Mirror outputs for cross-thread queries.
            const double pos = a.ramper->position();
            const double rate = a.ramper->rate();
            const double remaining =
                modeOut == Mode::Incremental ? a.ramper->target() - pos : 0.0;
            const uint32_t leaseLeft =
                modeOut == Mode::Continuous
                    ? static_cast<uint32_t>(std::max<int64_t>(
                          0, a.leaseEndMs.load(std::memory_order_relaxed) -
                                 nowMs))
                    : 0;
            a.position.store(pos, std::memory_order_relaxed);
            a.rate.store(rate, std::memory_order_relaxed);
            a.remaining.store(remaining, std::memory_order_relaxed);
            a.leaseLeft.store(leaseLeft, std::memory_order_relaxed);

            // Hand the ramped setpoint to the applicator.
            if (m_apply) {
                m_apply(JogSetpoint{i, static_cast<uint8_t>(modeOut),
                                    modeOut != Mode::Idle, pos, rate,
                                    remaining, leaseLeft, nowMs});
            }
        }
    }

    // ------------------------------------------------------------------
    // State queries (safe from any thread)
    // ------------------------------------------------------------------

    Mode axisMode(size_t axis) const {
        return static_cast<Mode>(
            m_axes.at(axis).mode.load(std::memory_order_acquire));
    }
    bool   axisActive(size_t axis) const { return axisMode(axis) != Mode::Idle; }
    double axisRate(size_t axis) const {
        return m_axes.at(axis).rate.load(std::memory_order_relaxed);
    }
    double axisPosition(size_t axis) const {
        return m_axes.at(axis).position.load(std::memory_order_relaxed);
    }
    /// Remaining incremental displacement (0 for continuous/idle).
    double axisRemaining(size_t axis) const {
        return m_axes.at(axis).remaining.load(std::memory_order_relaxed);
    }
    /// Milliseconds until the continuous lease expires (0 when not leased).
    uint32_t axisLeaseLeftMs(size_t axis, int64_t nowMs) const {
        const auto& a = m_axes.at(axis);
        if (axisMode(axis) != Mode::Continuous) return 0;
        const int64_t left =
            a.leaseEndMs.load(std::memory_order_relaxed) - nowMs;
        return left > 0 ? static_cast<uint32_t>(left) : 0;
    }

    /// Fill a fixed-layout snapshot for IO transport.
    void fillSnapshot(size_t axis, AxisSnapshot& out, int64_t nowMs) const {
        const auto& cfg = m_cfg.axes.at(axis);
        out.mode = static_cast<uint8_t>(axisMode(axis));
        out.active = out.mode != static_cast<uint8_t>(Mode::Idle) ? 1 : 0;
        out.rate = axisRate(axis);
        out.position = axisPosition(axis);
        out.remaining = axisRemaining(axis);
        out.leaseLeftMs = axisLeaseLeftMs(axis, nowMs);
        out.maxLeaseMs = cfg.maxLeaseMs;
        out.maxRate = cfg.maxRate;
        out.maxIncrement = cfg.maxIncrement;
    }

    /**
     * @brief Machine-readable description of axes, groups, and limits for
     *        clients (UI renders itself from this — nothing is hardcoded).
     */
    std::string describeJson() const {
        std::string j;
        char buf[256];
        j += "{\"version\":1,\"defaultLeaseMs\":";
        j += std::to_string(m_cfg.defaultLeaseMs);
        j += ",\"axes\":[";
        for (size_t i = 0; i < m_cfg.axes.size(); ++i) {
            const auto& a = m_cfg.axes[i];
            std::snprintf(buf, sizeof(buf),
                          "%s{\"name\":\"%s\",\"unit\":\"%s\",\"maxRate\":%g,"
                          "\"accel\":%g,\"maxLeaseMs\":%u,\"maxIncrement\":%g,"
                          "\"ratePresets\":%s,\"incrementPresets\":%s}",
                          i ? "," : "", jsonEscape(a.name).c_str(),
                          jsonEscape(a.unit).c_str(),
                          a.maxRate, a.accel, a.maxLeaseMs, a.maxIncrement,
                          numberArray(a.ratePresets).c_str(),
                          numberArray(a.incrementPresets).c_str());
            j += buf;
        }
        j += "],\"groups\":[";
        for (size_t g = 0; g < m_cfg.groups.size(); ++g) {
            const auto& grp = m_cfg.groups[g];
            std::string ax = "[";
            for (size_t k = 0; k < grp.axes.size(); ++k) {
                if (k) ax += ',';
                ax += '"';
                ax += grp.axes[k] < m_cfg.axes.size()
                          ? jsonEscape(m_cfg.axes[grp.axes[k]].name)
                          : "?";
                ax += '"';
            }
            ax += ']';
            std::snprintf(buf, sizeof(buf),
                          "%s{\"name\":\"%s\",\"axes\":%s,"
                          "\"ratePresets\":%s,\"incrementPresets\":%s}",
                          g ? "," : "", jsonEscape(grp.name).c_str(),
                          ax.c_str(),
                          numberArray(grp.ratePresets).c_str(),
                          numberArray(grp.incrementPresets).c_str());
            j += buf;
        }
        j += "]}";
        return j;
    }

private:
    /// Default ramper factory: trapezoid ramps at the axis limits.
    static std::unique_ptr<IJogRamper> makeTrapezoidRamper(const AxisConfig& a) {
        TrapezoidJogRamper::Config rc;
        rc.maxRate = a.maxRate;
        rc.accel = a.accel;
        return std::make_unique<TrapezoidJogRamper>(rc);
    }

    struct AxisState {
        /// Serializes jog()/move()/stop() writers only — update() never
        /// takes this lock.
        std::mutex           cmdMutex;
        std::atomic<uint8_t> mode{static_cast<uint8_t>(Mode::Idle)};
        std::atomic<double>  cmdRate{0.0};       ///< Continuous commanded rate
        std::atomic<int64_t> leaseEndMs{0};      ///< Continuous lease deadline
        std::atomic<double>  pendingIncrement{0.0};  ///< move() mailbox
        std::atomic<double>  resetRequest{0.0};  ///< resetPosition() mailbox
        std::atomic<bool>    resetPending{false};
        /// Trajectory generator — update() thread only (single writer).
        std::unique_ptr<IJogRamper> ramper;
        // Mirrors for cross-thread queries (update-thread written):
        std::atomic<double>  position{0.0};
        std::atomic<double>  rate{0.0};
        std::atomic<double>  remaining{0.0};
        std::atomic<uint32_t> leaseLeft{0};
        AxisState() = default;
        AxisState(const AxisState&) = delete;
        AxisState& operator=(const AxisState&) = delete;
    };

    /// Escape characters that would break a JSON string literal.
    static std::string jsonEscape(const std::string& s) {
        std::string out;
        out.reserve(s.size());
        for (const char c : s) {
            if (c == '"' || c == '\\') out += '\\';
            if (static_cast<unsigned char>(c) >= 0x20) out += c;
        }
        return out;
    }

    static std::string numberArray(const std::vector<double>& values) {
        std::string out = "[";
        char num[32];
        for (size_t i = 0; i < values.size(); ++i) {
            std::snprintf(num, sizeof(num), "%s%g", i ? "," : "", values[i]);
            out += num;
        }
        out += ']';
        return out;
    }

    Config                m_cfg;
    std::deque<AxisState> m_axes;  ///< Non-movable states; deque = stable addresses
    JogApplyFn            m_apply;
};

} // namespace tether::control
