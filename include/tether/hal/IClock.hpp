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
     */
    virtual Microseconds getPeriodMicros() const = 0;
    
    /**
     * @brief Get timer statistics
     */
    struct Stats {
        uint64_t tickCount = 0;     ///< Number of timer ticks
        Microseconds maxJitter = 0; ///< Maximum jitter observed
        Microseconds avgJitter = 0; ///< Average jitter
        uint32_t missedTicks = 0;   ///< Number of missed/late ticks
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
     */
    virtual IClock& getSystemClock() = 0;
    
    /**
     * @brief Create a periodic timer
     */
    virtual std::unique_ptr<IPeriodicTimer> createPeriodicTimer() = 0;
    
    /**
     * @brief Create a one-shot timer
     */
    virtual std::unique_ptr<IOneShotTimer> createOneShotTimer() = 0;
};

// ============================================================================
// Global Access
// ============================================================================

/**
 * @brief Get platform-specific clock factory
 */
IClockFactory& getClockFactory();

/**
 * @brief Get system clock (convenience)
 */
inline IClock& getSystemClock() {
    return getClockFactory().getSystemClock();
}

/**
 * @brief Set custom clock factory (for testing)
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
