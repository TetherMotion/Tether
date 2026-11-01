/**
 * @file LoopPacer.hpp
 * @brief Single-discipline pacing for a synchronous loop, with timing stats
 *
 * A LoopPacer owns one loop's schedule: call endTick() once per iteration
 * with the measured work duration; it records the duration, asks the
 * installed IPacingPolicy for the next tick's start time, sleeps until
 * then, and records how late the tick actually began.
 *
 * ## Policies (pluggable; subclass `IPacingPolicy` for custom)
 *
 *   - `FreeRunPolicy`      — no pacing; every tick starts immediately
 *   - `FixedRatePolicy`    — absolute grid advanced by `period` per
 *     tick. Behind schedule → immediate fire (natural catch-up/burst);
 *     lag beyond `resyncAfterUs` → grid rebased to now (the "drop",
 *     counted as a resync). `resyncAfterUs=0` → never rebase.
 *   - `SnapToGridPolicy`   — phase-locked: the next tick is always the
 *     first grid point at/after now plus `lateToleranceUs`. Overruns
 *     wait for the next slot — slots are never caught up and never burst.
 *
 * ## Statistics
 *
 * LoopStats tracks per-tick work (last/EMA/max + a WindowStats<N> window
 * for percentiles), overruns vs the policy period, wake-lateness of tick
 * starts, and the policy's own resync/catch-up counters. All stats are
 * diagnostics — they never alter pacing behavior.
 *
 * ## Threading
 *
 * Single-threaded: the loop thread calls endTick(); stats may be read
 * concurrently (each getter copies under a mutex).
 */
#pragma once

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <thread>

#include "tether/common/WindowStats.hpp"

namespace tether::control {

/**
 * @brief Decides when the next loop tick starts.
 *
 * schedule() is invoked once per tick, between iterations, on the loop
 * thread. Returning a past/now time point means "fire immediately".
 */
class IPacingPolicy {
public:
    using TimePoint = std::chrono::steady_clock::time_point;

    virtual ~IPacingPolicy() = default;

    /**
     * @brief Absolute start time for the next tick.
     * @param now  time at the end of the current tick's work
     * @return the scheduled next-tick start (≤ now = fire immediately)
     */
    virtual TimePoint schedule(TimePoint now) = 0;
    /// Target period in µs; 0 = unbounded (free run). Feeds overrun stats.
    /// @return Nominal tick period [µs]
    virtual double periodUs() const = 0;
    /// Schedule rebases/drops since construction or reset().
    /// @return Number of times the schedule was rebased
    virtual uint64_t resyncs() const { return 0; }
    /// Ticks fired behind their grid point to catch up.
    /// @return Number of catch-up ticks fired
    virtual uint64_t catchUpTicks() const { return 0; }
    /// Re-anchor internal state (e.g. after a loop pause/reset).
    /// @param now Current time to re-anchor the schedule to
    virtual void reset(TimePoint now) = 0;
};

/// No pacing — next tick fires immediately. periodUs() == 0.
class FreeRunPolicy final : public IPacingPolicy {
public:
    TimePoint schedule(TimePoint now) override { return now; }
    double periodUs() const override { return 0.0; }
    void reset(TimePoint) override {}
};

/**
 * @brief Fixed absolute-rate grid with optional rebase on deep lag.
 *
 * The grid advances one period per tick regardless of wall time — a loop
 * that falls behind fires immediately until the schedule recovers
 * (burst catch-up), or until lag exceeds resyncAfterUs, at which point
 * the grid is rebased to now (late slots dropped, counted as resync).
 */
class FixedRatePolicy final : public IPacingPolicy {
public:
    /**
     * @param periodUs        target tick period, > 0
     * @param resyncAfterUs   lag beyond which the grid is rebased;
     *                        0 = never rebase (unbounded catch-up)
     * @throw std::invalid_argument on non-positive/non-finite period or
     *        negative resyncAfterUs
     */
    explicit FixedRatePolicy(double periodUs, double resyncAfterUs = 0.0)
        : periodUs_(periodUs), resyncAfterUs_(resyncAfterUs) {
        if (!(periodUs > 0.0) || !std::isfinite(periodUs))
            throw std::invalid_argument("FixedRatePolicy: periodUs must be > 0");
        if (resyncAfterUs < 0.0 || !std::isfinite(resyncAfterUs))
            throw std::invalid_argument("FixedRatePolicy: resyncAfterUs < 0");
    }

    TimePoint schedule(TimePoint now) override {
        if (!anchored_) {
            next_ = now;
            anchored_ = true;
        }
        next_ += std::chrono::duration_cast<TimePoint::duration>(
            std::chrono::duration<double, std::micro>(periodUs_));
        if (next_ <= now) {
            ++catchUpTicks_;
            const double lagUs =
                std::chrono::duration<double, std::micro>(now - next_).count();
            if (resyncAfterUs_ > 0.0 && lagUs > resyncAfterUs_) {
                next_ = now;
                ++resyncs_;
            }
        }
        return next_;
    }

    double periodUs() const override { return periodUs_; }
    uint64_t resyncs() const override { return resyncs_; }
    uint64_t catchUpTicks() const override { return catchUpTicks_; }
    void reset(TimePoint now) override {
        next_ = now;
        anchored_ = false;
        resyncs_ = 0;
        catchUpTicks_ = 0;
    }

private:
    double periodUs_;
    double resyncAfterUs_;
    TimePoint next_{};
    bool anchored_ = false;
    uint64_t resyncs_ = 0;
    uint64_t catchUpTicks_ = 0;
};

/**
 * @brief Phase-locked grid: ticks always start on an absolute slot.
 *
 * Overruns are never caught up — after a late tick the next start is the
 * first slot at/after `now` (plus a small grace). A tick arriving within
 * lateToleranceUs of its slot still fires immediately (counted as a
 * catch-up). Slots silently skipped by an overrun are dropped — the loop
 * phase stays pinned to the wall grid.
 */
class SnapToGridPolicy final : public IPacingPolicy {
public:
    /**
     * @param periodUs         grid period, > 0
     * @param lateToleranceUs  a tick arriving this late still fires now
     *                         instead of waiting for the next slot; ≥ 0
     * @throw std::invalid_argument on non-positive period or negative
     *        tolerance
     */
    explicit SnapToGridPolicy(double periodUs, double lateToleranceUs = 100.0)
        : periodUs_(periodUs), toleranceUs_(lateToleranceUs) {
        if (!(periodUs > 0.0) || !std::isfinite(periodUs))
            throw std::invalid_argument("SnapToGridPolicy: periodUs must be > 0");
        if (lateToleranceUs < 0.0 || !std::isfinite(lateToleranceUs))
            throw std::invalid_argument("SnapToGridPolicy: lateToleranceUs < 0");
    }

    TimePoint schedule(TimePoint now) override {
        if (!anchored_) {
            origin_ = now;
            anchored_ = true;
            return now;  // anchor on the first tick — not a catch-up
        }
        const double elapsedUs =
            std::chrono::duration<double, std::micro>(now - origin_).count();
        const double slots = elapsedUs / periodUs_;
        const uint64_t whole = static_cast<uint64_t>(std::floor(slots));
        const double fracUs = (slots - whole) * periodUs_;
        if (fracUs <= toleranceUs_) {
            // Inside the grace window after a slot — fire now.
            ++catchUpTicks_;
            return now;
        }
        const auto target = origin_ + std::chrono::duration_cast<TimePoint::duration>(
            std::chrono::duration<double, std::micro>((whole + 1) * periodUs_));
        return target;
    }

    double periodUs() const override { return periodUs_; }
    void reset(TimePoint now) override {
        origin_ = now;
        anchored_ = false;
        catchUpTicks_ = 0;
    }
    uint64_t catchUpTicks() const override { return catchUpTicks_; }

private:
    double periodUs_;
    double toleranceUs_;
    TimePoint origin_{};
    bool anchored_ = false;
    uint64_t catchUpTicks_ = 0;
};

/**
 * @brief Owns one loop's pacing policy and its timing statistics.
 *
 * @tparam WindowN  samples kept in the tick/late-start windows for
 *                  percentile queries (1024 ≈ 1 s at 1 kHz)
 */
template <size_t WindowN = 1024>
class LoopPacer {
public:
    using Clock = std::chrono::steady_clock;
    using TimePoint = Clock::time_point;
    using NowFn = std::function<TimePoint()>;
    using SleepFn = std::function<void(TimePoint)>;

    /// Per-tick pacing diagnostics. Snapshot via stats().
    struct LoopStats {
        uint64_t ticks = 0;
        double lastTickUs = 0;     ///< work duration of the last tick
        double emaTickUs = 0;      ///< EMA of work duration
        double maxTickUs = 0;
        uint64_t overruns = 0;     ///< ticks whose work exceeded the period
        double maxOverrunUs = 0;
        uint64_t lateStarts = 0;   ///< starts later than lateThresholdUs
        double lastLateStartUs = 0;
        double maxLateStartUs = 0;
        /// Effective wall rate implied by the EMA tick (Hz; 0 if none).
        /// @return 1e6/emaTickUs, or 0 before the first tick
        double effectiveHz() const { return emaTickUs > 0 ? 1e6 / emaTickUs : 0.0; }
        /// Effective real-time factor for a loop advancing `simDtUs` of
        /// simulated time per tick (0 if no EMA yet).
        /// @param simDtUs Simulated time advanced per tick [µs]
        /// @return simDtUs/emaTickUs — &gt;1 means faster than real time
        double effectiveRtf(double simDtUs) const {
            return emaTickUs > 0 ? simDtUs / emaTickUs : 0.0;
        }
    };

    /**
     * @param policy            pacing law (must be non-null)
     * @param emaAlpha          EMA blend for tick time, in (0,1]
     * @param lateThresholdUs   a start later than this counts as lateStarts
     * @param now               clock source (tests inject a fake)
     * @param sleep             how to wait for a target (tests no-op it)
     * @throw std::invalid_argument on null policy or alpha outside (0,1]
     */
    explicit LoopPacer(std::unique_ptr<IPacingPolicy> policy,
                       double emaAlpha = 0.01, double lateThresholdUs = 100.0,
                       NowFn now = Clock::now,
                       SleepFn sleep = [](TimePoint t) {
                           std::this_thread::sleep_until(t);
                       })
        : policy_(std::move(policy)),
          emaAlpha_(emaAlpha),
          lateThresholdUs_(lateThresholdUs),
          now_(std::move(now)),
          sleep_(std::move(sleep)) {
        if (!policy_)
            throw std::invalid_argument("LoopPacer: null policy");
        if (!(emaAlpha > 0.0 && emaAlpha <= 1.0) || !std::isfinite(emaAlpha))
            throw std::invalid_argument("LoopPacer: emaAlpha must be in (0,1]");
        if (lateThresholdUs < 0.0 || !std::isfinite(lateThresholdUs))
            throw std::invalid_argument("LoopPacer: lateThresholdUs < 0");
    }

    LoopPacer(const LoopPacer&) = delete;
    LoopPacer& operator=(const LoopPacer&) = delete;

    /**
     * @brief End-of-tick: record `workUs`, schedule and sleep to the next
     *        tick, then record the actual start lateness.
     * @param workUs  measured duration of the tick's work in µs
     * @return µs the upcoming tick started after its scheduled time
     *         (0 = on time or free-run)
     */
    double endTick(double workUs) {
        {
            std::lock_guard lk(stats_mtx_);
            ++stats_.ticks;
            stats_.lastTickUs = workUs;
            stats_.emaTickUs = stats_.ticks == 1
                                   ? workUs
                                   : stats_.emaTickUs * (1.0 - emaAlpha_) +
                                         workUs * emaAlpha_;
            if (workUs > stats_.maxTickUs) stats_.maxTickUs = workUs;
            tickWindow_.add(workUs);
            const double period = policy_->periodUs();
            if (period > 0.0 && workUs > period) {
                ++stats_.overruns;
                const double over = workUs - period;
                if (over > stats_.maxOverrunUs) stats_.maxOverrunUs = over;
            }
        }

        const auto target = policy_->schedule(now_());
        if (target > now_()) sleep_(target);
        const double lateUs =
            std::chrono::duration<double, std::micro>(now_() - target).count();
        const double lateness = lateUs > 0 ? lateUs : 0.0;

        {
            std::lock_guard lk(stats_mtx_);
            stats_.lastLateStartUs = lateness;
            if (lateness > stats_.maxLateStartUs) stats_.maxLateStartUs = lateness;
            if (lateness > lateThresholdUs_) ++stats_.lateStarts;
            lateWindow_.add(lateness);
        }
        return lateness;
    }

    /// Consistent snapshot of cumulative stats (windows are separate).
    /// @return Copy of all counters and aggregates
    LoopStats stats() const {
        std::lock_guard lk(stats_mtx_);
        return stats_;
    }

    /// Tick-work duration window (p50/p95/p99 via percentile()).
    /// Copied under the stats mutex — safe to query from any thread.
    /// @return Copy of the rolling tick-duration window
    common::WindowStats<WindowN> tickWindow() const {
        std::lock_guard lk(stats_mtx_);
        return tickWindow_;
    }
    /// Start-lateness window.
    /// @return Copy of the rolling start-lateness window
    common::WindowStats<WindowN> lateWindow() const {
        std::lock_guard lk(stats_mtx_);
        return lateWindow_;
    }

    /// Policy-reported schedule rebases.
    /// @return Rebase count forwarded from the installed policy
    uint64_t resyncs() const { return policy_->resyncs(); }
    /// Policy-reported catch-up fires.
    /// @return Catch-up count forwarded from the installed policy
    uint64_t catchUpTicks() const { return policy_->catchUpTicks(); }
    /// Target period of the installed policy (µs; 0 = free run).
    /// @return Nominal tick period [µs]
    double periodUs() const { return policy_->periodUs(); }

    /// Zero all stats and re-anchor the policy at the current time.
    void reset() {
        policy_->reset(now_());
        std::lock_guard lk(stats_mtx_);
        stats_ = {};
        tickWindow_.reset();
        lateWindow_.reset();
    }

private:
    std::unique_ptr<IPacingPolicy> policy_;
    double emaAlpha_;
    double lateThresholdUs_;
    NowFn now_;
    SleepFn sleep_;

    mutable std::mutex stats_mtx_;
    LoopStats stats_;
    common::WindowStats<WindowN> tickWindow_;
    common::WindowStats<WindowN> lateWindow_;
};

}  // namespace tether::control
