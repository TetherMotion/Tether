#pragma once

/**
 * @file DummyRealtimeExchangeLoop.hpp
 * @brief Lightweight cyclic PDO exchange loop to keep sync manager watchdogs alive
 *
 * This class runs a background thread that calls a user-provided exchange
 * function (typically `master.pdo().exchangeAll()`) at a configurable fixed
 * frequency.  It is intended for tools and utilities that need to keep the
 * EtherCAT sync manager watchdogs from expiring while performing slow SDO
 * operations (e.g. writing FNI/RSP/SDD configuration data), but do not need
 * the full RealtimeLoop infrastructure (platform timers, DC sync, jitter
 * monitoring).
 *
 * Usage:
 * @code
 *   EtherCAT::Master master;
 *   // ... configure slaves, reach SAFE-OP/OP ...
 *
 *   EtherCAT::DummyRealtimeExchangeLoop loop([&master]() {
 *       master.pdo().exchangeAll();
 *   });
 *   loop.start();   // 1 kHz by default
 *
 *   // ... perform slow SDO operations ...
 *
 *   loop.stop();
 * @endcode
 */

#include <atomic>
#include <chrono>
#include <functional>
#include <thread>

namespace EtherCAT {

class DummyRealtimeExchangeLoop {
public:
    /// Callback invoked every cycle.  Return value is ignored.
    using ExchangeFunc = std::function<void()>;

    /// Configuration
    struct Config {
        /// Cycle period in microseconds (default: 1000 = 1 kHz).
        uint32_t cycle_period_us = 1000;

        /// Thread name (informational, for logging if desired).
        const char* name = "dummy_rt_exchange";
    };

    /**
     * @brief Construct the loop with default config (1 kHz).
     *
     * @param exchange_fn  Called every cycle (e.g. `master.pdo().exchangeAll()`).
     */
    explicit DummyRealtimeExchangeLoop(ExchangeFunc exchange_fn)
        : exchange_fn_(std::move(exchange_fn)) {}

    /**
     * @brief Construct the loop with custom config.
     *
     * @param exchange_fn  Called every cycle (e.g. `master.pdo().exchangeAll()`).
     * @param config       Timing and naming configuration.
     */
    DummyRealtimeExchangeLoop(ExchangeFunc exchange_fn, Config config)
        : exchange_fn_(std::move(exchange_fn)),
          config_(config) {}

    ~DummyRealtimeExchangeLoop() {
        stop();
    }

    // Non-copyable, non-movable
    DummyRealtimeExchangeLoop(const DummyRealtimeExchangeLoop&) = delete;
    DummyRealtimeExchangeLoop& operator=(const DummyRealtimeExchangeLoop&) = delete;
    DummyRealtimeExchangeLoop(DummyRealtimeExchangeLoop&&) = delete;
    DummyRealtimeExchangeLoop& operator=(DummyRealtimeExchangeLoop&&) = delete;

    /// Start the exchange thread.  Returns false if already running.
    bool start() {
        if (running_.load(std::memory_order_acquire)) return false;
        running_.store(true, std::memory_order_release);
        thread_ = std::thread([this]() { runLoop(); });
        return true;
    }

    /// Stop the exchange thread and join it.
    void stop() {
        if (!running_.load(std::memory_order_acquire)) return;
        running_.store(false, std::memory_order_release);
        if (thread_.joinable()) thread_.join();
    }

    /// Check if the loop is currently running.
    bool isRunning() const { return running_.load(std::memory_order_acquire); }

    /// Get the configured cycle period in microseconds.
    uint32_t cyclePeriodUs() const { return config_.cycle_period_us; }

    /// Get the number of cycles completed so far.
    uint64_t cycleCount() const { return cycle_count_.load(std::memory_order_relaxed); }

private:
    void runLoop() {
        const auto period = std::chrono::microseconds(config_.cycle_period_us);
        auto next_time = std::chrono::steady_clock::now();
        while (running_.load(std::memory_order_relaxed)) {
            exchange_fn_();
            cycle_count_.fetch_add(1, std::memory_order_relaxed);
            next_time += period;
            std::this_thread::sleep_until(next_time);
        }
    }

    ExchangeFunc exchange_fn_;
    Config config_;
    std::thread thread_;
    std::atomic<bool> running_{false};
    std::atomic<uint64_t> cycle_count_{0};
};

} // namespace EtherCAT
