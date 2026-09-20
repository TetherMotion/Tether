#pragma once

/**
 * @file CyclicExecutive.hpp
 * @brief Single-thread, deadline-driven cyclic executive for EtherCAT exchange.
 *
 * Unlike RealtimeLoop (timer thread -> event -> worker hop per cycle), the
 * executive's worker thread sleeps directly on an absolute deadline via
 * IDeadlineTimer (clock_nanosleep/TIMER_ABSTIME on Linux): one syscall and
 * zero context-switch hops per cycle.
 *
 * The loop runs an ordered phase pipeline each cycle:
 *
 *   [DC sync]* -> PreExchange -> Exchange -> PostExchange -> MotionControl -> Diagnostics
 *
 *   *only when DCPlacement::Inline and the decimation counter hits.
 *
 * DC synchronisation can instead run on a dedicated, higher-priority
 * deadline-driven thread (DCPlacement::DedicatedThread) — the recommended
 * mode for drives that emergency-stop on missed sync — or be omitted
 * entirely (DCPlacement::Disabled).
 *
 * Threads and tasks are created at start()/addTask() time; the hot path
 * performs no allocation, no mutex, and no condition variable.
 */

#include <array>
#include <atomic>
#include <cstdint>
#include <functional>
#include <memory>
#include <vector>

#include "tether/platform/IDeadlineTimer.hpp"
#include "tether/hal/IThreading.hpp"
#include "tether/ethercat/CyclicTaskScheduler.hpp"  // TaskPhase
#include "tether/ethercat/RealtimeJitterMonitor.hpp"

namespace EtherCAT {

class CyclicExecutive {
public:
    using TaskFn   = std::function<bool()>;
    using TimeFunc = std::function<uint64_t()>;   ///< monotonic time in ns

    /// How the worker waits for its next deadline.
    enum class SleepMode : uint8_t {
        Nanosleep,    ///< clock_nanosleep(TIMER_ABSTIME) — lowest CPU
        HybridSpin,   ///< sleep + busy-spin tail (spin_window_us) — lowest jitter
    };

    /// Where DC synchronisation frames are emitted.
    enum class DCPlacement : uint8_t {
        Disabled,         ///< No DC sync driven by this executive
        DedicatedThread,  ///< Separate high-priority deadline-driven thread
                          ///< (fault-isolated from the cyclic path)
        Inline,           ///< Same thread, first action of every
                          ///< dc_interval_cycles-th cycle
    };

    /// Kernel scheduling class for the cyclic thread.
    enum class SchedClass : uint8_t {
        Fifo,       ///< SCHED_FIFO @ priority — needs CAP_SYS_NICE
        Deadline,   ///< SCHED_DEADLINE CBS (runtime/deadline/period) —
                    ///< works on vanilla kernels, bounds runaway-RT damage.
                    ///< Falls back to Fifo with a logged warning.
    };

    struct Config {
        uint32_t cycle_period_us = 1000;   ///< Cyclic period in µs

        int      priority     = 80;        ///< SCHED_FIFO priority of cyclic thread
        int      cpu_affinity = -1;        ///< CPU to pin cyclic thread (-1 = any)
        size_t   stack_size   = 262144;    ///< Cyclic thread stack bytes
        SleepMode sleep_mode  = SleepMode::Nanosleep;
        uint32_t spin_window_us = 50;      ///< HybridSpin: busy-wait tail in µs

        /// Scheduling class for the cyclic thread (DC thread always uses
        /// SCHED_FIFO — its budget is a single short frame).
        SchedClass sched_class = SchedClass::Fifo;
        /// SCHED_DEADLINE parameters (ns).  0 → runtime = period/2,
        /// deadline = period.  Only used when sched_class == Deadline.
        uint64_t dl_runtime_ns  = 0;
        uint64_t dl_deadline_ns = 0;

        /// Pre-fault the cyclic thread's stack at thread start (avoids
        /// first-touch page faults mid-cycle).  Bytes to descend.
        uint32_t stack_prefault_bytes = 128 * 1024;
        /// Kill the kernel's ~50 µs timer slack on this thread — only
        /// matters when SCHED_FIFO could not be acquired (RT tasks skip
        /// slack already), but it is free insurance on the degraded path.
        bool     low_timer_slack = true;

        // ---- DC synchronisation ----
        DCPlacement dc_placement      = DCPlacement::DedicatedThread;
        uint32_t    dc_interval_cycles = 10;   ///< Inline decimation / dedicated
                                               ///< thread period = period × this
        int         dc_priority       = 90;    ///< Dedicated DC thread priority
        int         dc_cpu_affinity   = -1;

        /// Jitter thresholds (auto-derived by defaults())
        JitterConfig jitter    = JitterConfig::defaults(1000);
        JitterConfig dc_jitter = JitterConfig::defaults(10000);

        /// If true, the loop keeps running when a task/exchange returns false
        /// (error counted).  If false, the first failure stops the loop.
        bool continue_on_error = true;

        static Config defaults(uint32_t cycle_us = 1000, uint32_t dc_cycles = 10) {
            Config c;
            c.cycle_period_us    = cycle_us;
            c.dc_interval_cycles = dc_cycles;
            c.jitter             = JitterConfig::defaults(cycle_us);
            c.dc_jitter          = JitterConfig::defaults(cycle_us * dc_cycles);
            return c;
        }
    };

    struct Stats {
        uint64_t cycle_count        = 0;
        uint64_t exchange_errors    = 0;
        uint64_t task_errors        = 0;
        uint64_t missed_deadlines   = 0;   ///< skipped deadline count (overruns)
        uint32_t max_cycle_work_us  = 0;   ///< longest in-cycle work burst
        JitterStats jitter;                ///< wake-up period jitter (cyclic)

        uint64_t   dc_sync_count    = 0;
        uint64_t   dc_sync_errors   = 0;
        JitterStats dc_jitter;             ///< dedicated DC thread jitter
    };

    /**
     * @param exchange    Called once per cycle at the Exchange phase
     *                    (e.g. the LRW process-image exchange).  May be empty.
     * @param dc_sync     Called per DCPlacement (may be null when Disabled).
     * @param time_source Monotonic nanoseconds for jitter stats (may be null —
     *                    falls back to the platform clock).
     */
    CyclicExecutive(TaskFn exchange, TaskFn dc_sync, TimeFunc time_source,
                    const Config& config = Config::defaults());
    ~CyclicExecutive();

    CyclicExecutive(const CyclicExecutive&) = delete;
    CyclicExecutive& operator=(const CyclicExecutive&) = delete;

    /**
     * @brief Register a task for a phase.  Call BEFORE start() — registration
     *        is not realtime-safe (it appends to a vector).
     * @return false if the phase is Exchange (reserved for the exchange fn)
     *         or the loop is already running.
     */
    bool addTask(TaskPhase phase, TaskFn fn);

    bool start();
    void stop();
    bool isRunning() const { return running_.load(std::memory_order_acquire); }

    /// Enable/disable the exchange step (tasks still run).  Default: enabled.
    void setExchangeEnabled(bool en) { exchange_enabled_.store(en, std::memory_order_release); }

    Stats getStats() const;

private:
    void cyclicMain();
    void dcMain();
    bool runPhase(size_t phase_idx);
    std::unique_ptr<Platform::IDeadlineTimer> makeTimer() const;

    uint64_t nowNs() const { return time_source_ ? time_source_() : platformNowNs(); }
    static uint64_t platformNowNs();

    static constexpr size_t kNumPhases = 5;   // Pre/Ex/Post/Motion/Diag
    static size_t phaseIndex(TaskPhase p);

    TaskFn   exchange_;
    TaskFn   dc_sync_;
    TimeFunc time_source_;
    Config   config_;

    std::array<std::vector<TaskFn>, kNumPhases> phase_tasks_;

    // Cyclic thread
    std::unique_ptr<Platform::IDeadlineTimer> timer_;
    std::unique_ptr<HAL::IThread>             thread_;
    std::unique_ptr<RealtimeJitterMonitor>    jitter_monitor_;

    // Dedicated DC thread (DCPlacement::DedicatedThread)
    std::unique_ptr<Platform::IDeadlineTimer> dc_timer_;
    std::unique_ptr<HAL::IThread>             dc_thread_;
    std::unique_ptr<RealtimeJitterMonitor>    dc_jitter_monitor_;

    std::atomic<bool> running_{false};
    std::atomic<bool> exchange_enabled_{true};

    // Stats — written only by the RT threads, read by getStats()
    std::atomic<uint64_t> cycle_count_{0};
    std::atomic<uint64_t> exchange_errors_{0};
    std::atomic<uint64_t> task_errors_{0};
    std::atomic<uint64_t> missed_deadlines_{0};
    std::atomic<uint32_t> max_cycle_work_us_{0};
    std::atomic<uint64_t> dc_sync_count_{0};
    std::atomic<uint64_t> dc_sync_errors_{0};
};

} // namespace EtherCAT
