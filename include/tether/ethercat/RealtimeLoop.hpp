#pragma once

/**
 * @file RealtimeLoop.hpp
 * @brief Two-thread realtime loop for independent PDO exchange and DC synchronization
 *
 * This class owns **two** independent realtime threads, each with its own
 * platform timer, event, and jitter monitor:
 *
 *  1. **PDO thread** — runs at `cycle_period_us` (default 1 kHz) and calls the
 *     PDO exchange callback every cycle when PDO is enabled.
 *  2. **DC thread** — runs at `cycle_period_us * sync_interval_cycles` (default
 *     10 ms / 100 Hz) and calls the DC synchronisation callback.
 *
 * Both threads include automatic self-diagnosis via `RealtimeJitterMonitor`,
 * which detects missed deadlines and logs warnings / critical alerts when
 * configurable thresholds are exceeded.
 *
 * The class is independent from the DC module — it simply calls provided
 * callbacks at the configured rates.
 *
 * Usage:
 * @code
 *   auto pdo_fn  = [&pdo]() { return pdo.exchange(); };
 *   auto sync_fn = [&dc]()  { return dc.sendSyncFrame(); };
 *   auto time_fn = [&dc]()  { return dc.getMasterTimeNs(); };
 *
 *   RealtimeLoop loop(pdo_fn, sync_fn, time_fn, config);
 *   loop.setPDOEnabled(true);
 *   loop.start();
 *   // ...
 *   auto diag = loop.getDiagnostics();
 *   loop.stop();
 * @endcode
 */

#include <cstdint>
#include <functional>
#include <atomic>
#include <memory>

#include "tether/platform/IPlatformTimer.hpp"
#include "tether/hal/IThreading.hpp"
#include "tether/ethercat/RealtimeJitterMonitor.hpp"

namespace EtherCAT {

class RealtimeLoop {
public:
    /// Callback signatures
    using ExchangeFunc = std::function<bool()>;
    using SyncFunc     = std::function<bool()>;
    using TimeFunc     = std::function<uint64_t()>;

    /// Loop configuration
    struct Config {
        uint32_t cycle_period_us      = 1000;  ///< PDO timer period in µs (default 1 kHz)
        uint32_t sync_interval_cycles = 10;    ///< DC sync period = cycle_period_us × this

        /// Jitter thresholds for the PDO thread (auto-derived from cycle_period_us)
        JitterConfig pdo_jitter = JitterConfig::defaults(1000);
        /// Jitter thresholds for the DC thread (auto-derived from sync period)
        JitterConfig dc_jitter  = JitterConfig::defaults(10000);

        /**
         * @brief Construct defaults and auto-derive jitter thresholds
         *
         * @param cycle_us  PDO cycle period in µs
         * @param sync_cyc  Number of PDO cycles between DC sync frames
         */
        static Config defaults(uint32_t cycle_us = 1000, uint32_t sync_cyc = 10) {
            Config c;
            c.cycle_period_us      = cycle_us;
            c.sync_interval_cycles = sync_cyc;
            c.pdo_jitter           = JitterConfig::defaults(cycle_us);
            c.dc_jitter            = JitterConfig::defaults(cycle_us * sync_cyc);
            return c;
        }
    };

    /// Legacy loop statistics (backward-compatible snapshot via getStats())
    struct Stats {
        uint64_t cycle_count     = 0;   ///< PDO thread cycle count
        uint64_t sync_count      = 0;   ///< DC thread sync count
        uint64_t pdo_error_count = 0;   ///< PDO exchange error count
        uint32_t max_jitter_us   = 0;   ///< Max jitter of the PDO thread
        uint32_t avg_jitter_us   = 0;   ///< Avg jitter of the PDO thread
    };

    /// Extended per-thread diagnostics
    struct ThreadDiagnostics {
        JitterStats pdo_jitter;  ///< Full jitter statistics for the PDO thread
        JitterStats dc_jitter;   ///< Full jitter statistics for the DC thread
    };

    /**
     * @brief Construct a realtime loop with two independent threads
     *
     * @param pdo_exchange  Called every PDO cycle when PDO is enabled
     * @param dc_sync       Called every DC sync cycle
     * @param time_source   Returns current monotonic time in nanoseconds
     * @param config        Timing and jitter configuration
     */
    RealtimeLoop(ExchangeFunc pdo_exchange,
                          SyncFunc     dc_sync,
                          TimeFunc     time_source,
                          const Config& config = Config::defaults());

    ~RealtimeLoop();

    // Non-copyable, non-movable
    RealtimeLoop(const RealtimeLoop&) = delete;
    RealtimeLoop& operator=(const RealtimeLoop&) = delete;
    RealtimeLoop(RealtimeLoop&&) = delete;
    RealtimeLoop& operator=(RealtimeLoop&&) = delete;

    /**
     * @brief Start both realtime threads (PDO + DC)
     *
     * Creates timers, threads, and events for both the PDO and DC loops.
     *
     * @return true on success, false if already running or resource creation fails
     */
    bool start();

    /**
     * @brief Stop both realtime threads
     *
     * Stops timers, joins threads, releases resources.
     */
    void stop();

    /// Check if the loop is currently running (both threads)
    bool isRunning() const { return running_.load(std::memory_order_acquire); }

    /// Enable / disable PDO exchange in the PDO thread
    void setPDOEnabled(bool enable) { pdo_enabled_.store(enable, std::memory_order_release); }

    /// Check if PDO exchange is enabled
    bool isPDOEnabled() const { return pdo_enabled_.load(std::memory_order_acquire); }

    /// Thread-safe snapshot of current statistics (backward-compatible)
    Stats getStats() const;

    /// Thread-safe snapshot of per-thread jitter diagnostics
    ThreadDiagnostics getDiagnostics() const;

private:
    // Timer callbacks (ISR-safe: only signal events)
    static bool pdoTimerCallback(void* user_data);
    static bool dcTimerCallback(void* user_data);

    // Thread entry points
    static void pdoTaskEntry(void* param);
    static void dcTaskEntry(void* param);

    // Helpers
    bool startPDOThread();
    bool startDCThread();
    void stopPDOThread();
    void stopDCThread();

    ExchangeFunc pdo_exchange_;
    SyncFunc     dc_sync_;
    TimeFunc     time_source_;
    Config       config_;

    // ── PDO thread resources ──
    std::unique_ptr<Platform::IPlatformTimer> pdo_timer_;
    std::unique_ptr<HAL::IThread>             pdo_thread_;
    std::unique_ptr<HAL::IEvent>              pdo_event_;
    std::unique_ptr<RealtimeJitterMonitor>    pdo_jitter_monitor_;

    // ── DC thread resources ──
    std::unique_ptr<Platform::IPlatformTimer> dc_timer_;
    std::unique_ptr<HAL::IThread>             dc_thread_;
    std::unique_ptr<HAL::IEvent>              dc_event_;
    std::unique_ptr<RealtimeJitterMonitor>    dc_jitter_monitor_;

    std::atomic<bool> running_{false};
    std::atomic<bool> pdo_enabled_{false};

    // Legacy stats (combined view for backward compatibility).
    // Atomic counters are used so the realtime PDO/DC threads can update them
    // without taking a mutex (which would risk priority inversion against the
    // application thread reading getStats()). Relaxed ordering is sufficient
    // because each counter is independently monotonic; getStats() may observe
    // a slightly inconsistent snapshot across counters, which is acceptable
    // for diagnostics.
    std::atomic<uint64_t> pdo_cycle_count_{0};
    std::atomic<uint64_t> pdo_error_count_{0};
    std::atomic<uint64_t> dc_sync_count_{0};
};

} // namespace EtherCAT
