#pragma once

/**
 * @file IPlatformTimer.hpp
 * @brief Platform-independent timer interface for DC realtime loop
 * 
 * This interface abstracts hardware timer differences between platforms:
 * - ESP32: gptimer with ISR callback
 * - Linux/host: std::thread + chrono high-resolution timer
 * - Other RTOS: Compatible timer implementations
 * 
 * The timer is designed for precise periodic callbacks at frequencies
 * up to 10kHz (100μs period), with typical usage at 1kHz (1ms period).
 */

#include <cstdint>
#include <functional>
#include <memory>

namespace EtherCAT {
namespace Platform {

/**
 * @brief Timer callback function signature
 * 
 * The callback should execute quickly (< 100μs typical) and avoid blocking.
 * It may run in interrupt context (ESP32) or high-priority thread (host).
 * 
 * @param user_data User-provided context pointer
 * @return true if high-priority task should be woken (for RTOS platforms)
 */
using TimerCallback = std::function<bool(void* user_data)>;

/**
 * @brief Timer configuration
 */
struct TimerConfig {
    uint32_t period_us;         ///< Timer period in microseconds (e.g., 1000 for 1kHz)
    TimerCallback callback;     ///< Function to call on each timer expiration
    void* user_data;            ///< User context passed to callback
    int priority;               ///< Thread/interrupt priority (platform-specific, 0 = default)
    bool auto_reload;           ///< Automatically reload timer after expiration
};

/**
 * @brief Platform timer interface
 * 
 * Implementations must provide:
 * - Precise periodic callbacks
 * - Sub-millisecond resolution (microsecond preferred)
 * - Low jitter (< 10μs typical for 1kHz operation)
 * - Thread-safe start/stop operations
 */
class IPlatformTimer {
public:
    virtual ~IPlatformTimer() = default;
    
    /**
     * @brief Configure the timer
     * 
     * Must be called before start(). May be called multiple times to reconfigure.
     * 
     * @param config Timer configuration
     * @return true on success
     */
    virtual bool configure(const TimerConfig& config) = 0;
    
    /**
     * @brief Start the timer
     * 
     * Begins periodic callback invocation at the configured frequency.
     * 
     * @return true on success
     */
    virtual bool start() = 0;
    
    /**
     * @brief Stop the timer
     * 
     * Stops callback invocation. The timer can be restarted with start().
     */
    virtual void stop() = 0;
    
    /**
     * @brief Check if timer is currently running
     * 
     * @return true if timer is active
     */
    virtual bool isRunning() const = 0;
    
    /**
     * @brief Get actual achieved timer period (may differ from requested due to hardware limits)
     * 
     * @return Actual period in microseconds
     */
    virtual uint32_t getActualPeriodUs() const = 0;
    
    /**
     * @brief Get measured jitter statistics
     * 
     * @param max_jitter_us Maximum observed deviation from target period (output parameter)
     * @param avg_jitter_us Average deviation from target period (output parameter)
     * @return true if statistics are available
     */
    virtual bool getJitterStats(uint32_t& max_jitter_us, uint32_t& avg_jitter_us) const = 0;
};

/**
 * @brief Create platform-specific timer instance
 * 
 * Factory function that returns the appropriate timer implementation for the current platform:
 * - ESP32: GPTimer-based implementation
 * - Linux/host: std::thread + chrono implementation
 * - Other: Platform-specific implementation
 * 
 * @return Unique pointer to platform timer, or nullptr on failure
 */
std::unique_ptr<IPlatformTimer> createPlatformTimer();

} // namespace Platform
} // namespace EtherCAT
