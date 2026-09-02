/**
 * @file RealtimeJitterMonitor.cpp
 * @brief Implementation of the realtime self-diagnosis jitter monitor
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
    std::lock_guard<std::mutex> lk(mutex_);

    stats_.cycle_count++;

    // First cycle — just record the timestamp, no jitter to compute yet
    if (!has_prev_) {
        prev_ns_  = now_ns;
        has_prev_ = true;
        return;
    }

    // Compute inter-cycle delta
    const uint64_t delta_ns = (now_ns >= prev_ns_) ? (now_ns - prev_ns_) : 0;
    const uint32_t delta_us = static_cast<uint32_t>(delta_ns / 1000);
    stats_.last_period_us = delta_us;
    prev_ns_ = now_ns;

    // Jitter = |actual_period - expected_period|
    const uint32_t jitter_us = (delta_us >= config_.expected_period_us)
        ? (delta_us - config_.expected_period_us)
        : (config_.expected_period_us - delta_us);

    // Update max
    if (jitter_us > stats_.max_jitter_us) {
        stats_.max_jitter_us = jitter_us;
    }

    // Update EWMA (weight = 7/8 old + 1/8 new)
    stats_.avg_jitter_us = (stats_.avg_jitter_us * 7 + jitter_us) / 8;

    // Check warning threshold
    if (jitter_us > config_.warning_threshold_us) {
        stats_.warning_count++;

        // Log only the first few warnings and then periodically to avoid spam
        if (stats_.warning_count <= 3 ||
            (config_.log_interval_cycles > 0 &&
             stats_.cycle_count % config_.log_interval_cycles == 0)) {
            TETHER_LOGW(TAG, "[{}] Jitter WARNING: {} us (threshold {} us, cycle #{})",
                        name_, jitter_us, config_.warning_threshold_us,
                        static_cast<unsigned long long>(stats_.cycle_count));
        }
    }

    // Check critical threshold
    if (jitter_us > config_.critical_threshold_us) {
        stats_.critical_count++;
        stats_.realtime_ok = false;

        // Always log critical overruns (they should be rare)
        TETHER_LOGE(TAG, "[{}] Jitter CRITICAL: {} us (threshold {} us, cycle #{}, period {} us)",
                    name_, jitter_us, config_.critical_threshold_us,
                    static_cast<unsigned long long>(stats_.cycle_count),
                    delta_us);
    }

    // Periodic diagnostic summary
    if (config_.log_interval_cycles > 0 &&
        stats_.cycle_count > 1 &&
        stats_.cycle_count % config_.log_interval_cycles == 0) {
        TETHER_LOGI(TAG, "[{}] Jitter report: cycles={} avg={} us max={} us warn={} crit={} rt_ok={}",
                    name_,
                    static_cast<unsigned long long>(stats_.cycle_count),
                    stats_.avg_jitter_us,
                    stats_.max_jitter_us,
                    static_cast<unsigned long long>(stats_.warning_count),
                    static_cast<unsigned long long>(stats_.critical_count),
                    stats_.realtime_ok ? "YES" : "NO");
    }
}

// ============================================================================
// Stats snapshot
// ============================================================================

JitterStats RealtimeJitterMonitor::getStats() const {
    std::lock_guard<std::mutex> lk(mutex_);
    return stats_;
}

// ============================================================================
// Reset
// ============================================================================

void RealtimeJitterMonitor::reset() {
    std::lock_guard<std::mutex> lk(mutex_);
    stats_    = JitterStats{};
    prev_ns_  = 0;
    has_prev_ = false;
}

} // namespace EtherCAT
