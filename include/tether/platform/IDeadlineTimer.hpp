/**
 * @file IDeadlineTimer.hpp
 * @brief Absolute-deadline wait abstraction for real-time cyclic threads.
 *
 * Unlike IPlatformTimer (which models an asynchronous ISR-style callback and
 * therefore requires a producer thread + event + consumer thread), a deadline
 * timer blocks the CALLING thread until the next absolute deadline.  On Linux
 * this is a single clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME) syscall —
 * the minimum possible cost for kernel-timed periodic sleep.
 *
 * Contract:
 *   - start()/startAt() arm the schedule (not copyable across threads).
 *   - waitNext() blocks the calling thread until the next deadline and then
 *     advances the internal schedule by exactly period_ns.  Missed deadlines
 *     (overruns) are skipped and reported in Tick::missed; the schedule never
 *     resynchronises to "now", so a periodic task has zero drift.
 *   - requestStop() makes waitNext() return false.  The stop is cooperative:
 *     it is observed on the next wake-up (at most one period of latency).
 *   - Implementations must not allocate or take mutexes inside waitNext().
 */
#pragma once

#include <cstdint>
#include <memory>

namespace EtherCAT {
namespace Platform {

class IDeadlineTimer {
public:
    struct Tick {
        uint64_t deadline_ns = 0;  ///< absolute time the tick was scheduled for
        uint64_t woke_ns     = 0;  ///< actual wake-up timestamp (monotonic)
        uint32_t missed      = 0;  ///< deadlines skipped due to overrun
    };

    virtual ~IDeadlineTimer() = default;

    /** Arm the schedule; first deadline = now + period_ns. */
    virtual bool start(uint64_t period_ns) = 0;

    /** Arm the schedule with an explicit first deadline (phase alignment). */
    virtual bool startAt(uint64_t first_deadline_ns, uint64_t period_ns) = 0;

    /**
     * @brief Block the calling thread until the next deadline.
     * @return false when stopped (requestStop) or on timer failure.
     */
    virtual bool waitNext(Tick& out) = 0;

    /** Cooperative stop — observed on the next deadline wake-up. */
    virtual void requestStop() = 0;

    virtual bool     isRunning() const = 0;
    virtual uint64_t periodNs()  const = 0;
};

/**
 * @brief Create the platform deadline timer.
 *
 * Linux : clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME) — one syscall/cycle.
 * ESP32 : vTaskDelayUntil() — same absolute-deadline semantics.
 */
std::unique_ptr<IDeadlineTimer> createDeadlineTimer();

/**
 * @brief Create a hybrid sleep+spin deadline timer.
 *
 * Sleeps (clock_nanosleep) until deadline - spin_window_ns, then busy-waits
 * on a vDSO clock_gettime read for the remainder.  Cuts wake-up jitter to
 * single-digit microseconds on isolated cores at the cost of one busy CPU
 * for spin_window_ns per cycle.
 *
 * @param spin_window_ns  Spin duration at the tail of each period
 *                        (e.g. 50'000 for the last 50 µs).  0 == pure sleep.
 */
std::unique_ptr<IDeadlineTimer> createHybridDeadlineTimer(uint64_t spin_window_ns);

} // namespace Platform
} // namespace EtherCAT
