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
#include <string>

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

//=============================================================================
// Realtime Kernel Detection
//=============================================================================

/**
 * @brief Preempt model compiled into the kernel (from uname)
 */
enum class PreemptModel {
    PreemptRt,        ///< PREEMPT_RT (hard realtime)
    PreemptDynamic,   ///< PREEMPT_DYNAMIC (mode selectable at runtime)
    PreemptFull,      ///< Static PREEMPT (full preempt, low-latency desktop)
    PreemptVoluntary, ///< PREEMPT_VOLUNTARY
    PreemptNone,      ///< PREEMPT_NONE
    Unknown,
};

/**
 * @brief Classified realtime level (lowest to highest)
 */
enum class RealtimeClass {
    None,          ///< preempt=none or unknown
    Voluntary,     ///< preempt=voluntary or lazy
    LowLatency,    ///< full preempt (desktop low-latency, not hard RT)
    HardRealtime,  ///< PREEMPT_RT
};

/**
 * @brief Caller-selectable realtime requirement level
 */
enum class RealtimeRequirement {
    None,          ///< Detect and log only, never exit
    LowLatency,    ///< Require at least full preempt; error if below
    HardRealtime,  ///< Require PREEMPT_RT; warn if only low-latency, error if below
};

/**
 * @brief Result of realtime kernel detection
 */
struct RealtimeKernelInfo {
    bool is_realtime = false;             ///< true only for PREEMPT_RT
    bool is_low_latency = false;          ///< true for full preempt or PREEMPT_RT
    RealtimeClass realtime_class = RealtimeClass::None;
    PreemptModel build_model = PreemptModel::Unknown;   ///< from uname
    PreemptModel active_model = PreemptModel::Unknown;  ///< active mode
    std::string active_preempt_mode;      ///< "full", "lazy", "voluntary", "none", "unknown", "n/a"
    std::string sysname;
    std::string kernel_release;
    std::string kernel_version;
    std::string detection_source;
};

/**
 * @brief Detect the kernel's realtime capabilities
 * @return RealtimeKernelInfo with build model, active dynamic mode, and classified level
 */
RealtimeKernelInfo detectRealtimeKernel();

/**
 * @brief Ensure the kernel meets the requested realtime level, or exit
 * @param req Requirement level (default: HardRealtime)
 * @return RealtimeKernelInfo on success
 *
 * Behavior by requirement level:
 * - HardRealtime: pass if PREEMPT_RT; WARN if low-latency desktop; ERROR+exit if below
 * - LowLatency:   pass if full preempt or better; ERROR+exit if below
 * - None:         always pass (detect and log only)
 */
RealtimeKernelInfo ensureRealtimeKernelOrExit(RealtimeRequirement req = RealtimeRequirement::HardRealtime);

} // namespace Platform
} // namespace Tether
