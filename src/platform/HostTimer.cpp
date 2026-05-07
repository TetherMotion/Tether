/**
 * @file HostTimer.cpp
 * @brief Linux/host std::thread-based implementation of IPlatformTimer
 * 
 * Uses high-resolution std::chrono timers for precise periodic callbacks.
 * Suitable for testing and host-side simulation of DC synchronization.
 */

#include "tether/platform/IPlatformTimer.hpp"
#include "tether/platform/EspCompat.hpp"

#include <thread>
#include <atomic>
#include <chrono>
#include <cstring>

namespace EtherCAT {
namespace Platform {

static const char* TAG = "host_timer";

/**
 * @brief Host timer implementation using std::thread + chrono
 */
class HostTimer : public IPlatformTimer {
public:
    HostTimer() = default;
    
    ~HostTimer() override {
        stop();
    }
    
    bool configure(const TimerConfig& config) override {
        if (isRunning()) {
            TETHER_LOGE(TAG, "Cannot configure while running");
            return false;
        }
        
        config_ = config;
        configured_ = true;
        return true;
    }
    
    bool start() override {
        if (!configured_) {
            TETHER_LOGE(TAG, "Timer not configured");
            return false;
        }
        
        if (running_.load()) {
            return true;  // Already running
        }
        
        running_.store(true);
        max_jitter_us_ = 0;
        avg_jitter_us_ = 0;
        cycle_count_ = 0;
        
        // Launch high-priority timer thread
        timer_thread_ = std::thread(&HostTimer::timerThreadFunc, this);
        
        // Try to set thread priority (requires privileges on Linux)
#ifdef __linux__
        pthread_t native_handle = timer_thread_.native_handle();
        struct sched_param param;
        std::memset(&param, 0, sizeof(param));
        
        int policy = SCHED_FIFO;
        int max_prio = sched_get_priority_max(policy);
        param.sched_priority = (config_.priority > 0) ? 
                               std::min(config_.priority, max_prio) : 
                               (max_prio > 10 ? max_prio - 10 : max_prio / 2);
        
        int ret = pthread_setschedparam(native_handle, policy, &param);
        if (ret == 0) {
            TETHER_LOGI(TAG, "Timer thread set to SCHED_FIFO priority %d", param.sched_priority);
        } else {
            TETHER_LOGW(TAG, "Failed to set thread priority (requires CAP_SYS_NICE or root): %s", 
                     strerror(ret));
        }
#endif
        
        TETHER_LOGI(TAG, "Timer started at %lu Hz", 1000000UL / config_.period_us);
        return true;
    }
    
    void stop() override {
        if (!running_.load()) {
            return;
        }
        
        running_.store(false);
        
        if (timer_thread_.joinable()) {
            timer_thread_.join();
        }
        
        TETHER_LOGI(TAG, "Timer stopped after %llu cycles", (unsigned long long)cycle_count_.load());
    }
    
    bool isRunning() const override {
        return running_.load();
    }
    
    uint32_t getActualPeriodUs() const override {
        return config_.period_us;
    }
    
    bool getJitterStats(uint32_t& max_jitter_us, uint32_t& avg_jitter_us) const override {
        max_jitter_us = max_jitter_us_.load();
        avg_jitter_us = avg_jitter_us_.load();
        return cycle_count_.load() > 0;
    }
    
private:
    /**
     * @brief Timer thread function
     * 
     * Uses std::chrono::high_resolution_clock for precise timing.
     * Implements a drift-correcting loop to maintain accurate period.
     */
    void timerThreadFunc() {
        using namespace std::chrono;
        
        const auto period = microseconds(config_.period_us);
        auto next_wakeup = high_resolution_clock::now() + period;
        
        while (running_.load()) {
            // Sleep until next cycle
            std::this_thread::sleep_until(next_wakeup);
            
            // Measure actual wakeup time
            const auto woke_at = high_resolution_clock::now();
            const auto jitter_duration = duration_cast<microseconds>(woke_at - next_wakeup);
            const int64_t jitter_us = jitter_duration.count();
            const uint32_t abs_jitter = static_cast<uint32_t>(jitter_us > 0 ? jitter_us : -jitter_us);
            
            // Update jitter statistics
            uint32_t current_max = max_jitter_us_.load();
            while (abs_jitter > current_max && 
                   !max_jitter_us_.compare_exchange_weak(current_max, abs_jitter)) {
                // Retry if another thread updated max_jitter_us_
            }
            
            // Simple moving average for avg jitter
            uint32_t current_avg = avg_jitter_us_.load();
            uint32_t new_avg = (current_avg * 7 + abs_jitter) / 8;
            avg_jitter_us_.store(new_avg);
            
            cycle_count_.fetch_add(1);
            
            // Invoke user callback
            if (config_.callback) {
                config_.callback(config_.user_data);
            }
            
            // Calculate next wakeup time (drift correction)
            if (config_.auto_reload) {
                next_wakeup += period;
                
                // If we've fallen behind significantly, resync to now
                const auto now = high_resolution_clock::now();
                if (next_wakeup < now - period) {
                    static uint32_t behind_count = 0;
                    behind_count++;
                    if (behind_count <= 3 || (behind_count % 1000) == 0) {
                        TETHER_LOGW(TAG, "Timer fell behind by %lld us, resyncing (count=%u)",
                                 (long long)duration_cast<microseconds>(now - next_wakeup).count(),
                                 behind_count);
                    }
                    next_wakeup = now + period;
                }
            } else {
                break;  // One-shot mode
            }
        }
    }
    
    TimerConfig config_;
    bool configured_ = false;
    std::atomic<bool> running_{false};
    std::thread timer_thread_;
    
    std::atomic<uint64_t> cycle_count_{0};
    std::atomic<uint32_t> max_jitter_us_{0};
    std::atomic<uint32_t> avg_jitter_us_{0};
};

std::unique_ptr<IPlatformTimer> createHostTimer() {
    return std::make_unique<HostTimer>();
}

// Factory function for platform abstraction
std::unique_ptr<IPlatformTimer> createPlatformTimer() {
    return std::make_unique<HostTimer>();
}

} // namespace Platform
} // namespace EtherCAT
