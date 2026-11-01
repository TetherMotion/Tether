/**
 * @file IClock.hpp
 * @brief Clock and timing abstraction interfaces
 *
 * Provides interfaces for system clocks, high-resolution timing,
 * and periodic timers across different platforms.
 */

#pragma once

#include "hal/HALTypes.hpp"
#include <functional>
#include <memory>

namespace EtherCAT {
namespace HAL {

// ============================================================================
// Clock Interface
// ============================================================================

/**
 * @brief Abstract clock interface for time measurement
 */
class IClock {
public:
    virtual ~IClock() = default;
    
    /**
     * @brief Get current time in microseconds
     * @return Monotonic timestamp in microseconds
     * 
     * The returned value is monotonically increasing and suitable
     * for measuring elapsed time. The epoch is platform-defined.
     */
    virtual Timestamp nowMicros() = 0;
    
    /**
     * @brief Get current time in milliseconds
     *
     * Convenience wrapper over nowMicros() (integer-division truncated),
     * sharing its monotonic epoch and guarantees. Override only if the
     * platform offers a cheaper or higher-precision millisecond source.
     *
     * @return Monotonic timestamp in milliseconds.
     */
    virtual Timestamp nowMillis() {
        return nowMicros() / 1000;
    }
    
    /**
     * @brief Get current time in nanoseconds (if supported)
     * @return Nanosecond timestamp or 0 if not supported
     */
    virtual Timestamp nowNanos() {
        return nowMicros() * 1000;  // Default: convert from micros
    }
    
    /**
     * @brief Get system (wall clock) time in milliseconds since Unix epoch
     * @return Milliseconds since 1970-01-01 00:00:00 UTC
     */
    virtual Timestamp systemTimeMillis() = 0;
    
    /**
     * @brief Get clock resolution in nanoseconds
     *
     * The smallest time increment the clock can distinguish — the
     * granularity of the underlying hardware/OS timer, not the nominal
     * unit of the returned timestamps. Useful for judging whether a
     * measured interval is meaningful or within quantization noise.
     *
     * @return Clock tick granularity in nanoseconds.
     */
    virtual Nanoseconds resolution() = 0;
    
    /**
     * @brief Busy-wait delay (blocking, high precision)
     * @param us Microseconds to delay
     */
    virtual void delayMicros(Microseconds us) = 0;
    
    /**
     * @brief Sleep delay (may yield to other tasks)
     * @param ms Milliseconds to sleep
     */
    virtual void delayMillis(Milliseconds ms) = 0;
};

// ============================================================================
// Periodic Timer Interface
// ============================================================================

/**
 * @brief Timer callback type
 */
using TimerCallback = std::function<void()>;

/**
 * @brief Abstract periodic timer interface
 */
class IPeriodicTimer {
public:
    virtual ~IPeriodicTimer() = default;
    
    /**
     * @brief Initialize the timer
     * @param frequencyHz Timer frequency in Hz
     * @return true on success
     */
    virtual bool init(uint32_t frequencyHz) = 0;
    
    /**
     * @brief Set timer period directly
     * @param periodUs Period in microseconds
     * @return true on success
     */
    virtual bool setPeriodMicros(Microseconds periodUs) {
        if (periodUs <= 0) return false;
        return init(static_cast<uint32_t>(1000000 / periodUs));
    }
    
    /**
     * @brief Start the timer
     */
    virtual void start() = 0;
    
    /**
     * @brief Stop the timer
     */
    virtual void stop() = 0;
    
    /**
     * @brief Check if timer is running
     *
     * @return true between a successful start() and the next stop();
     *         callbacks only fire while running.
     */
    virtual bool isRunning() const = 0;
    
    /**
     * @brief Wait for next timer cycle (blocking)
     * 
     * Blocks the calling thread until the next timer tick occurs.
     * This is the preferred method for implementing cyclic tasks.
     */
    virtual void waitForCycle() = 0;
    
    /**
     * @brief Set callback for timer events
     * @param callback Function to call on each tick
     * 
     * Note: The callback runs in timer context (ISR or high-priority task).
     * Keep it short and non-blocking.
     */
    virtual void setCallback(TimerCallback callback) = 0;
    
    /**
     * @brief Get current period in microseconds
     *
     * The period the hardware is actually programmed for — when init()
     * was called with a frequency the platform can only approximate, this
     * may differ slightly from `1e6 / frequencyHz`.
     *
     * @return Programmed tick period in microseconds.
     */
    virtual Microseconds getPeriodMicros() const = 0;
    
    /**
     * @brief Get timer statistics
     */
    struct Stats {
        uint64_t tickCount = 0; /**< Number of timer ticks */
        Microseconds maxJitter = 0; /**< Maximum jitter observed */
        Microseconds avgJitter = 0; /**< Average jitter */
        uint32_t missedTicks = 0; /**< Number of missed/late ticks */
    };
    
    virtual Stats getStats() const = 0;
    virtual void resetStats() = 0;
};

// ============================================================================
// One-Shot Timer Interface
// ============================================================================

/**
 * @brief Abstract one-shot timer interface
 */
class IOneShotTimer {
public:
    virtual ~IOneShotTimer() = default;
    
    /**
     * @brief Start timer with callback
     * @param delayUs Delay in microseconds
     * @param callback Function to call when timer expires
     * @return true on success
     */
    virtual bool start(Microseconds delayUs, TimerCallback callback) = 0;
    
    /**
     * @brief Cancel pending timer
     * @return true if timer was cancelled, false if already expired
     */
    virtual bool cancel() = 0;
    
    /**
     * @brief Check if timer is pending
     *
     * @return true while a start()ed timer is armed and has not yet
     *         fired or been cancelled; false after expiry, cancel(), or
     *         before the first start().
     */
    virtual bool isPending() const = 0;
    
    /**
     * @brief Get remaining time
     * @return Microseconds until expiry, 0 if expired or not pending
     */
    virtual Microseconds remaining() const = 0;
};

// ============================================================================
// Stopwatch Utility
// ============================================================================

/**
 * @brief Simple stopwatch for measuring elapsed time
 */
class Stopwatch {
public:
    explicit Stopwatch(IClock& clock) : m_clock(clock), m_start(0), m_running(false) {}
    
    void start() {
        m_start = m_clock.nowMicros();
        m_running = true;
    }
    
    void stop() {
        if (m_running) {
            m_elapsed = m_clock.nowMicros() - m_start;
            m_running = false;
        }
    }
    
    void reset() {
        m_start = 0;
        m_elapsed = 0;
        m_running = false;
    }
    
    Microseconds elapsedMicros() const {
        if (m_running) {
            return m_clock.nowMicros() - m_start;
        }
        return m_elapsed;
    }
    
    Milliseconds elapsedMillis() const {
        return elapsedMicros() / 1000;
    }
    
    bool isRunning() const { return m_running; }
    
private:
    IClock& m_clock;
    Timestamp m_start;
    Microseconds m_elapsed = 0;
    bool m_running;
};

// ============================================================================
// Factory Interface
// ============================================================================

/**
 * @brief Factory for creating clock and timer instances
 */
class IClockFactory {
public:
    virtual ~IClockFactory() = default;
    
    /**
     * @brief Get the system clock instance
     *
     * @return Reference to the platform clock singleton. The reference
     *         remains valid for the factory's lifetime; repeated calls
     *         return the same instance.
     */
    virtual IClock& getSystemClock() = 0;
    
    /**
     * @brief Create a periodic timer
     *
     * The returned timer is inert until init() + start(). Caller owns
     * the instance; destroying it must stop any pending tick delivery.
     *
     * @return New periodic timer, or nullptr if the platform cannot
     *         allocate more timers.
     */
    virtual std::unique_ptr<IPeriodicTimer> createPeriodicTimer() = 0;
    
    /**
     * @brief Create a one-shot timer
     *
     * Caller owns the instance; destroying it must cancel any pending
     * expiry callback.
     *
     * @return New one-shot timer, or nullptr if the platform cannot
     *         allocate more timers.
     */
    virtual std::unique_ptr<IOneShotTimer> createOneShotTimer() = 0;
};

// ============================================================================
// Global Access
// ============================================================================

/**
 * @brief Get platform-specific clock factory
 *
 * Lazily instantiates the platform's factory singleton on first call
 * (Linux, Windows, embedded port, or the test double installed via
 * setClockFactory()).
 *
 * @return Reference to the active IClockFactory singleton — valid until
 *         resetClockFactory() or process teardown.
 */
IClockFactory& getClockFactory();

/**
 * @brief Get system clock (convenience)
 *
 * Equivalent to getClockFactory().getSystemClock().
 *
 * @return Reference to the platform's monotonic clock singleton.
 */
inline IClock& getSystemClock() {
    return getClockFactory().getSystemClock();
}

/**
 * @brief Set custom clock factory (for testing)
 *
 * Replaces the platform singleton so all subsequent getClockFactory()/
 * getSystemClock() calls and every component built on them use the
 * injected implementation — the standard way to run deterministic
 * simulated time in unit tests. Install before first use; existing
 * references obtained earlier keep pointing at the previous factory.
 *
 * @param factory  Replacement factory; ownership is transferred. Must
 *                 not be null.
 */
void setClockFactory(std::unique_ptr<IClockFactory> factory);

/**
 * @brief Reset clock factory to platform default (for testing)
 *
 * Destroys the current clock factory singleton, allowing it to be
 * re-created with platform defaults on next access. Primarily used
 * in unit tests to ensure clean state between test cases.
 */
void resetClockFactory();

} // namespace HAL
} // namespace EtherCAT
