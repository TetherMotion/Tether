#pragma once

/**
 * @file StageTimer.hpp
 * @brief Per-stage timing inside a tick: deltas between successive checkpoints.
 */

#include <chrono>
#include <functional>

namespace tether::common {

/**
 * @brief Moving-checkpoint timer for instrumenting pipeline stages.
 *
 * Carries one time point; checkpoint() reads the clock ONCE, returns
 * microseconds since the previous checkpoint, and re-marks with the same
 * read.  Replaces the `us = since(t); t = now();` statement pair — half the
 * clock reads per boundary, and a forgotten re-mark is impossible by
 * construction.
 *
 * Checkpoints are positional, not named: the caller maps checkpoint order to
 * its own fields by assignment.  Realtime-safe — no allocation, no strings,
 * no lookups.
 *
 * @code
 *   StageTimer st;                  // marks now
 *   observe();
 *   tick.obs_us  = st.checkpoint();
 *   execute();
 *   tick.exec_us = st.checkpoint();
 *   tick.tick_us = st.elapsed();    // total since construction
 * @endcode
 *
 * The clock is injectable (std::function, same convention as
 * tether::control::LoopPacer) so tests can drive a manual time base.
 */
class StageTimer {
public:
    using Clock     = std::chrono::steady_clock;
    using TimePoint = Clock::time_point;
    using NowFn     = std::function<TimePoint()>;

    /**
     * @brief Construct and mark the start point.
     * @param now  clock source; defaults to steady_clock::now
     */
    explicit StageTimer(NowFn now = Clock::now)
        : now_(std::move(now)), start_(now_()), last_(start_) {}

    /**
     * @brief Read the clock, return the stage delta, re-mark.
     * @return microseconds since construction, reset(), or the previous
     *         checkpoint — whichever marked last
     */
    double checkpoint() {
        const TimePoint t = now_();
        const double us =
            std::chrono::duration<double, std::micro>(t - last_).count();
        last_ = t;
        return us;
    }

    /**
     * @brief Total span covered by the timer.
     * @return microseconds since construction or the last reset()
     */
    double elapsed() const {
        return std::chrono::duration<double, std::micro>(now_() - start_)
            .count();
    }

    /// Rebase both marks to now.  Use when resuming after a pause so the
    /// held interval is not attributed to the next stage.
    void reset() { start_ = last_ = now_(); }

private:
    NowFn     now_;
    TimePoint start_;
    TimePoint last_;
};

}  // namespace tether::common
