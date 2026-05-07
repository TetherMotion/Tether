#pragma once

/**
 * @file RealtimeJitterMonitor.hpp
 * @brief Self-diagnosis for periodic realtime threads
 *
 * Monitors whether a periodic task meets its timing deadlines by
 * measuring inter-cycle wall-clock deltas and comparing them against
 * configurable warning and critical thresholds.
 *
 * Designed to be embedded inside each realtime thread (PDO, DC).
 * Call `recordCycle()` once per iteration with the current monotonic
 * timestamp; the monitor tracks jitter statistics and periodically
 * logs a diagnostic summary.
 *
 * Usage:
 * @code
 *   JitterConfig cfg = JitterConfig::defaults(1000);  // 1 kHz thread
 *   RealtimeJitterMonitor mon(cfg, "pdo");
 *
 *   while (running) {
 *       mon.recordCycle(now_ns());
 *       // ... do work ...
 *   }
 *   auto stats = mon.getStats();
 * @endcode
 */

#include <cstdint>
#include <mutex>

namespace EtherCAT {

// ============================================================================
// Configuration
// ============================================================================

/**
 * @brief Configuration for realtime jitter monitoring
 *
 * All time values are in microseconds unless stated otherwise.
 */
struct JitterConfig {
    uint32_t expected_period_us;      ///< Expected nominal cycle period
    uint32_t warning_threshold_us;    ///< Warn when |actual − expected| exceeds this
    uint32_t critical_threshold_us;   ///< Critical alert when |actual − expected| exceeds this
    uint32_t log_interval_cycles;     ///< Log a diagnostic summary every N cycles (0 = disabled)

    /**
     * @brief Construct defaults for a given nominal period
     *
     * - Warning  = 50 % of the period
     * - Critical = 200 % of the period
     * - Log every 10 000 cycles (≈ 10 s at 1 kHz)
     *
     * @param period_us  Nominal cycle period in microseconds
     */
    static JitterConfig defaults(uint32_t period_us = 1000) {
        return JitterConfig{
            .expected_period_us    = period_us,
            .warning_threshold_us  = period_us / 2,
            .critical_threshold_us = period_us * 2,
            .log_interval_cycles   = 10000
        };
    }
};

// ============================================================================
// Statistics snapshot
// ============================================================================

/**
 * @brief Thread-safe snapshot of jitter statistics
 */
struct JitterStats {
    uint64_t cycle_count     = 0;   ///< Total cycles recorded
    uint32_t max_jitter_us   = 0;   ///< Maximum observed |actual − expected|
    uint32_t avg_jitter_us   = 0;   ///< Exponentially weighted moving average of jitter
    uint64_t warning_count   = 0;   ///< Cycles where jitter exceeded warning threshold
    uint64_t critical_count  = 0;   ///< Cycles where jitter exceeded critical threshold
    uint32_t last_period_us  = 0;   ///< Measured period of the most recent cycle
    bool     realtime_ok     = true; ///< false if any critical overrun has occurred
};

// ============================================================================
// Monitor
// ============================================================================

/**
 * @brief Realtime self-diagnosis — monitors whether a periodic task
 * meets its timing deadlines.
 *
 * Thread-safety: `getStats()` and `reset()` are safe to call from any
 * thread while `recordCycle()` runs in the realtime thread.
 */
class RealtimeJitterMonitor {
public:
    /**
     * @brief Construct a jitter monitor
     *
     * @param config      Thresholds and logging configuration
     * @param thread_name Short label used in log messages (e.g. "pdo", "dc")
     */
    explicit RealtimeJitterMonitor(const JitterConfig& config,
                                   const char* thread_name = "rt");

    /**
     * @brief Record a new cycle occurrence
     *
     * Call at the top of each loop iteration with the current monotonic
     * time.  Internally measures inter-cycle period, computes jitter,
     * updates statistics, and logs warnings when thresholds are exceeded.
     *
     * @param now_ns  Current monotonic time in nanoseconds
     */
    void recordCycle(uint64_t now_ns);

    /**
     * @brief Thread-safe snapshot of accumulated statistics
     */
    JitterStats getStats() const;

    /**
     * @brief Reset all statistics to zero
     *
     * Not typically needed in production; useful in unit tests.
     */
    void reset();

private:
    JitterConfig config_;
    const char*  name_;

    bool     has_prev_ = false;  ///< Whether we have a previous timestamp
    uint64_t prev_ns_  = 0;     ///< Timestamp of previous cycle

    JitterStats stats_;
    mutable std::mutex mutex_;
};

} // namespace EtherCAT
