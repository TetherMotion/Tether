/**
 * @file Platform.hpp
 * @brief Platform abstraction layer for Tether library
 * 
 * This provides platform-independent interfaces for:
 * - Logging (replaces esp_log.h)
 * - Timing (replaces esp_timer.h) 
 * - Delays (replaces esp_rom_delay_us, vTaskDelay)
 * - Ethernet handle abstraction
 */

#pragma once

#include <cstdint>
#include <chrono>
#include <functional>

#include "logging/Logger.hpp"

namespace Tether {
namespace Platform {

//=============================================================================
// Timing Interface
//=============================================================================

/**
 * @brief Platform-independent timing interface
 */
class Clock {
public:
    static Clock& instance();
    
    /**
     * @brief Get current time in microseconds since boot
     */
    int64_t getMicroseconds() const;
    
    /**
     * @brief Get current time in milliseconds since boot
     */
    int64_t getMilliseconds() const;
    
    /**
     * @brief Blocking delay in microseconds
     * @note May be approximate on non-RTOS systems
     */
    void delayMicroseconds(uint32_t us) const;
    
    /**
     * @brief Blocking delay in milliseconds
     */
    void delayMilliseconds(uint32_t ms) const;
    
    /**
     * @brief Yield to other tasks (RTOS) or no-op (bare metal)
     */
    void yield() const;
    
    // Allow setting custom implementations
    using GetTimeFn = std::function<int64_t()>;
    using DelayFn = std::function<void(uint32_t)>;
    using YieldFn = std::function<void()>;
    
    void setGetMicroseconds(GetTimeFn fn) { getMicros_ = fn; }
    GetTimeFn getGetMicroseconds() const { return getMicros_; }
    void setDelayMicroseconds(DelayFn fn) { delayMicros_ = fn; }
    void setYield(YieldFn fn) { yieldFn_ = fn; }
    
private:
    Clock();
    GetTimeFn getMicros_;
    DelayFn delayMicros_;
    YieldFn yieldFn_;
    std::chrono::steady_clock::time_point startTime_;
};

//=============================================================================
// Ethernet Handle Abstraction
//=============================================================================

/**
 * @brief Abstract handle type for ethernet operations
 * 
 * On ESP32 this wraps the platform Ethernet handle, on Linux it's a socket fd, etc.
 */
using EthernetHandle = void*;

/**
 * @brief Ethernet frame transmission function type
 */
using TransmitFn = std::function<bool(EthernetHandle, const uint8_t*, size_t)>;

/**
 * @brief Abstract ethernet interface
 */
class EthernetInterface {
public:
    virtual ~EthernetInterface() = default;
    
    virtual bool transmit(const uint8_t* frame, size_t length) = 0;
    virtual EthernetHandle getHandle() const = 0;
    virtual const uint8_t* getMacAddress() const = 0;
};

//=============================================================================
// Platform Initialization
//=============================================================================

/**
 * @brief Initialize platform layer (call once at startup)
 * 
 * On ESP32: sets up ESP-IDF handlers
 * On Linux: sets up std::chrono based timing
 */
void initialize();

/**
 * @brief Check if running on ESP32
 */
bool isEsp32();

/**
 * @brief Check if running on Linux
 */
bool isLinux();

/**
 * @brief Attempt to set the calling thread to realtime (SCHED_FIFO) on Linux
 * @param priority Linux realtime priority (1..99). If <= 0 a sensible default is used.
 * @return true on success or when the platform does not support realtime scheduling,
 *         false if an attempt was made and failed (e.g. insufficient privileges).
 */
bool setCurrentThreadRealtime(int priority = -1);

} // namespace Platform
} // namespace Tether
