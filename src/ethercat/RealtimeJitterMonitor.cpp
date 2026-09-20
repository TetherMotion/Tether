/**
 * @file RealtimeJitterMonitor.cpp
 * @brief Implementation of the realtime self-diagnosis jitter monitor
 *
 * Lock-free design: recordCycle() runs on the realtime thread and performs
 * only atomic operations — it can never be blocked by a diagnostics thread
 * calling getStats()/reset().
 */

#include "tether/ethercat/RealtimeJitterMonitor.hpp"
#include "tether/platform/Platform.hpp"

#include <algorithm>
#include <cmath>

namespace EtherCAT {

static const char* TAG = "rt_jitter";

// ============================================================================
// Constructor
// ============================================================================

RealtimeJitterMonitor::RealtimeJitterMonitor(const JitterConfig& config,
                                             const char* thread_name)
    : config_(config)
    , name_(thread_name ? thread_name : "rt")
{
}

// ============================================================================
// Record a cycle
// ============================================================================

void RealtimeJitterMonitor::recordCycle(uint64_t now_ns) {
    const uint64_t cycle = cycle_count_.fetch_add(1, std::memory_order_relaxed) + 1;

    // First cycle — just record the timestamp, no jitter to compute yet
    if (!has_prev_.load(std::memory_order_relaxed)) {
        prev_ns_.store(now_ns, std::memory_order_relaxed);
        has_prev_.store(true, std::memory_order_relaxed);
        return;
    }

    // Compute inter-cycle delta
    const uint64_t prev   = prev_ns_.load(std::memory_order_relaxed);
    const uint64_t delta_ns = (now_ns >= prev) ? (now_ns - prev) : 0;
    const uint32_t delta_us = static_cast<uint32_t>(delta_ns / 1000);
    last_period_us_.store(delta_us, std::memory_order_relaxed);
    prev_ns_.store(now_ns, std::memory_order_relaxed);

    // Jitter = |actual_period - expected_period|
    const uint32_t jitter_us = (delta_us >= config_.expected_period_us)
        ? (delta_us - config_.expected_period_us)
        : (config_.expected_period_us - delta_us);

    // Update max (single writer — plain RMW is fine)
    const uint32_t cur_max = max_jitter_us_.load(std::memory_order_relaxed);
    if (jitter_us > cur_max) {
        max_jitter_us_.store(jitter_us, std::memory_order_relaxed);
    }

    // Update EWMA (weight = 7/8 old + 1/8 new)
    avg_jitter_us_.store(
        (avg_jitter_us_.load(std::memory_order_relaxed) * 7 + jitter_us) / 8,
        std::memory_order_relaxed);

    // Check warning threshold
    if (jitter_us > config_.warning_threshold_us) {
        const uint64_t warnings =
            warning_count_.fetch_add(1, std::memory_order_relaxed) + 1;

        // Log only the first few warnings and then periodically to avoid spam
        if (warnings <= 3 ||
            (config_.log_interval_cycles > 0 &&
             cycle % config_.log_interval_cycles == 0)) {
            TETHER_LOGW(TAG, "[{}] Jitter WARNING: {} us (threshold {} us, cycle #{})",
                        name_, jitter_us, config_.warning_threshold_us,
                        static_cast<unsigned long long>(cycle));
        }
    }

    // Check critical threshold
    if (jitter_us > config_.critical_threshold_us) {
        critical_count_.fetch_add(1, std::memory_order_relaxed);
        realtime_ok_.store(false, std::memory_order_relaxed);

        // Always log critical overruns (they should be rare)
        TETHER_LOGE(TAG, "[{}] Jitter CRITICAL: {} us (threshold {} us, cycle #{}, period {} us)",
                    name_, jitter_us, config_.critical_threshold_us,
                    static_cast<unsigned long long>(cycle),
                    delta_us);
    }

    // Periodic diagnostic summary
    if (config_.log_interval_cycles > 0 &&
        cycle > 1 &&
        cycle % config_.log_interval_cycles == 0) {
        TETHER_LOGI(TAG, "[{}] Jitter report: cycles={} avg={} us max={} us warn={} crit={} rt_ok={}",
                    name_,
                    static_cast<unsigned long long>(cycle),
                    avg_jitter_us_.load(std::memory_order_relaxed),
                    max_jitter_us_.load(std::memory_order_relaxed),
                    static_cast<unsigned long long>(
                        warning_count_.load(std::memory_order_relaxed)),
                    static_cast<unsigned long long>(
                        critical_count_.load(std::memory_order_relaxed)),
                    realtime_ok_.load(std::memory_order_relaxed) ? "YES" : "NO");
    }
}

// ============================================================================
// Stats snapshot
// ============================================================================

JitterStats RealtimeJitterMonitor::getStats() const {
    JitterStats s;
    s.cycle_count    = cycle_count_.load(std::memory_order_relaxed);
    s.max_jitter_us  = max_jitter_us_.load(std::memory_order_relaxed);
    s.avg_jitter_us  = avg_jitter_us_.load(std::memory_order_relaxed);
    s.warning_count  = warning_count_.load(std::memory_order_relaxed);
    s.critical_count = critical_count_.load(std::memory_order_relaxed);
    s.last_period_us = last_period_us_.load(std::memory_order_relaxed);
    s.realtime_ok    = realtime_ok_.load(std::memory_order_relaxed);
    return s;
}

// ============================================================================
// Reset
// ============================================================================

void RealtimeJitterMonitor::reset() {
    cycle_count_.store(0, std::memory_order_relaxed);
    max_jitter_us_.store(0, std::memory_order_relaxed);
    avg_jitter_us_.store(0, std::memory_order_relaxed);
    warning_count_.store(0, std::memory_order_relaxed);
    critical_count_.store(0, std::memory_order_relaxed);
    last_period_us_.store(0, std::memory_order_relaxed);
    realtime_ok_.store(true, std::memory_order_relaxed);
    prev_ns_.store(0, std::memory_order_relaxed);
    has_prev_.store(false, std::memory_order_relaxed);
}

} // namespace EtherCAT
