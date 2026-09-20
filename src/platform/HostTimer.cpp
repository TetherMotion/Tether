/**
 * @file HostTimer.cpp
 * @brief Linux/host std::thread-based implementation of IPlatformTimer
 * 
 * Uses high-resolution std::chrono timers for precise periodic callbacks.
 * Suitable for testing and host-side simulation of DC synchronization.
 */

#include "tether/platform/IPlatformTimer.hpp"
#include "tether/platform/IDeadlineTimer.hpp"
#include "tether/platform/EspCompat.hpp"

#include <thread>
#include <atomic>
#include <chrono>
#include <cstring>
#ifdef __linux__
#include <ctime>
#include <cerrno>
#include <pthread.h>
#endif

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
        stop(false);
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
            TETHER_LOGI(TAG, "Timer thread set to SCHED_FIFO priority {}", param.sched_priority);
        } else {
            TETHER_LOGW(TAG, "Failed to set thread priority (requires CAP_SYS_NICE or root): {}", 
                     strerror(ret));
        }
#endif
        
        TETHER_LOGI(TAG, "Timer started at {} Hz", 1000000UL / config_.period_us);
        return true;
    }
    
    void stop(bool verbose) override {
        if (!running_.load()) {
            return;
        }
        
        running_.store(false);
        
        if (timer_thread_.joinable()) {
            timer_thread_.join();
        }
        
        if (verbose) {
            TETHER_LOGI(TAG, "Timer stopped after {} cycles", (unsigned long long)cycle_count_.load());
        }
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
                    static std::atomic<uint32_t> behind_count{0};
                    const uint32_t count = behind_count.fetch_add(1, std::memory_order_relaxed) + 1;
                    if (count <= 3 || (count % 1000) == 0) {
                        TETHER_LOGW(TAG, "Timer fell behind by {} us, resyncing (count={})",
                                 (long long)duration_cast<microseconds>(now - next_wakeup).count(),
                                 count);
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

// ============================================================================
// Deadline timer — sleeps the CALLING thread on an absolute deadline.
// ============================================================================

/**
 * @brief Generic deadline timer: absolute std::chrono steady_clock deadline.
 *
 * One virtual call + one kernel sleep per cycle; no producer thread, no
 * event, no mutex.  On Linux the steady_clock deadline maps 1:1 onto
 * clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME).
 */
class HostDeadlineTimer : public IDeadlineTimer {
public:
    bool start(uint64_t period_ns) override {
        return startAt(nowNs() + period_ns, period_ns);
    }

    bool startAt(uint64_t first_deadline_ns, uint64_t period_ns) override {
        if (period_ns == 0) return false;
        period_ns_ = period_ns;
        next_ns_   = first_deadline_ns;
        running_.store(true, std::memory_order_release);
        return true;
    }

    bool waitNext(Tick& out) override {
        if (!running_.load(std::memory_order_acquire)) return false;

        sleepUntilNs(next_ns_);

        const uint64_t woke = nowNs();
        out.deadline_ns = next_ns_;
        out.woke_ns     = woke;

        // Advance the fixed schedule; skip (and count) overrun deadlines so
        // an overrun never turns into a burst of back-to-back wake-ups.
        uint64_t next = next_ns_ + period_ns_;
        uint32_t missed = 0;
        while (next <= woke) {
            next += period_ns_;
            ++missed;
        }
        out.missed = missed;
        next_ns_   = next;
        return running_.load(std::memory_order_acquire);
    }

    void requestStop() override {
        running_.store(false, std::memory_order_release);
    }

    bool     isRunning() const override { return running_.load(std::memory_order_acquire); }
    uint64_t periodNs()  const override { return period_ns_; }

protected:
    static uint64_t nowNs() {
#ifdef __linux__
        struct timespec ts;
        clock_gettime(CLOCK_MONOTONIC, &ts);
        return static_cast<uint64_t>(ts.tv_sec) * 1'000'000'000ULL +
               static_cast<uint64_t>(ts.tv_nsec);
#else
        return static_cast<uint64_t>(
            std::chrono::duration_cast<std::chrono::nanoseconds>(
                std::chrono::steady_clock::now().time_since_epoch()).count());
#endif
    }

    virtual void sleepUntilNs(uint64_t deadline_ns) {
#ifdef __linux__
        struct timespec ts;
        ts.tv_sec  = static_cast<time_t>(deadline_ns / 1'000'000'000ULL);
        ts.tv_nsec = static_cast<long>(deadline_ns % 1'000'000'000ULL);
        int ret;
        do {
            ret = clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &ts, nullptr);
        } while (ret == EINTR && running_.load(std::memory_order_relaxed));
#else
        std::this_thread::sleep_until(
            std::chrono::steady_clock::time_point(std::chrono::nanoseconds(deadline_ns)));
#endif
    }

    std::atomic<bool> running_{false};
    uint64_t period_ns_ = 0;
    uint64_t next_ns_   = 0;
};

/**
 * @brief Hybrid deadline timer: kernel sleep + userspace spin tail.
 *
 * Sleeps until (deadline - spin_window_ns), then busy-polls the vDSO
 * CLOCK_MONOTONIC (no syscall in the spin) until the exact deadline.
 * Trades spin_window_ns of CPU per cycle for near-zero wake-up latency.
 */
class HybridDeadlineTimer : public HostDeadlineTimer {
public:
    explicit HybridDeadlineTimer(uint64_t spin_window_ns)
        : spin_window_ns_(spin_window_ns) {}

protected:
    void sleepUntilNs(uint64_t deadline_ns) override {
        const uint64_t spin = spin_window_ns_;
        if (spin > 0 && deadline_ns > spin) {
            HostDeadlineTimer::sleepUntilNs(deadline_ns - spin);
            while (nowNs() < deadline_ns &&
                   running_.load(std::memory_order_relaxed)) {
                // clock_gettime is vDSO on Linux — pure userspace read.
            }
        } else {
            HostDeadlineTimer::sleepUntilNs(deadline_ns);
        }
    }

private:
    uint64_t spin_window_ns_;
};

std::unique_ptr<IDeadlineTimer> createDeadlineTimer() {
    return std::make_unique<HostDeadlineTimer>();
}

std::unique_ptr<IDeadlineTimer> createHybridDeadlineTimer(uint64_t spin_window_ns) {
    return std::make_unique<HybridDeadlineTimer>(spin_window_ns);
}

} // namespace Platform
} // namespace EtherCAT
