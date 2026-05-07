/**
 * @file Platform.cpp
 * @brief Default (portable) platform implementation
 * 
 * This provides std::chrono based timing and printf-based logging.
 * ESP32-specific implementation is in hal/esp32/ESP32Platform.cpp
 */

#include "tether/platform/Platform.hpp"

#include <thread>

namespace Tether {
namespace Platform {

//=============================================================================
// Clock Implementation
//=============================================================================

Clock& Clock::instance() {
    static Clock clock;
    return clock;
}

Clock::Clock() : startTime_(std::chrono::steady_clock::now()) {
    // Default implementations using std::chrono
    getMicros_ = [this]() -> int64_t {
        auto now = std::chrono::steady_clock::now();
        return std::chrono::duration_cast<std::chrono::microseconds>(
            now - startTime_).count();
    };
    
    delayMicros_ = [](uint32_t us) {
        if (us < 1000) {
            // Busy wait for short delays
            auto start = std::chrono::steady_clock::now();
            while (std::chrono::duration_cast<std::chrono::microseconds>(
                std::chrono::steady_clock::now() - start).count() < us) {
                // Spin
            }
        } else {
            // Use sleep for longer delays
            std::this_thread::sleep_for(std::chrono::microseconds(us));
        }
    };
    
    yieldFn_ = []() {
        std::this_thread::yield();
    };
}

int64_t Clock::getMicroseconds() const {
    return getMicros_();
}

int64_t Clock::getMilliseconds() const {
    return getMicros_() / 1000;
}

void Clock::delayMicroseconds(uint32_t us) const {
    delayMicros_(us);
}

void Clock::delayMilliseconds(uint32_t ms) const {
    delayMicros_(ms * 1000);
}

void Clock::yield() const {
    if (yieldFn_) {
        yieldFn_();
    }
}

//=============================================================================
// Platform Detection
//=============================================================================

void initialize() {
    // Default initialization - nothing special needed for portable code
}

bool isEsp32() {
#ifdef ESP_PLATFORM
    return true;
#else
    return false;
#endif
}

bool isLinux() {
#ifdef __linux__
    return true;
#else
    return false;
#endif
}

bool setCurrentThreadRealtime(int priority) {
#ifdef __linux__
    // Default priority if caller didn't specify
    if (priority <= 0) priority = 80;

    // Clamp to system limits for SCHED_FIFO
    int prio_max = sched_get_priority_max(SCHED_FIFO);
    int prio_min = sched_get_priority_min(SCHED_FIFO);
    if (prio_max <= 0) {
        TETHER_LOGW("Platform", "Failed to query SCHED_FIFO priority range; refusing to set realtime");
        return false;
    }
    if (priority > prio_max) priority = prio_max;
    if (priority < prio_min) priority = prio_min;

    struct sched_param param;
    param.sched_priority = priority;
    int ret = pthread_setschedparam(pthread_self(), SCHED_FIFO, &param);
    if (ret != 0) {
        if (ret == EPERM) {
            TETHER_LOGW("Platform", "setCurrentThreadRealtime: insufficient permissions to set SCHED_FIFO (CAP_SYS_NICE required)");
        } else {
            TETHER_LOGW("Platform", "setCurrentThreadRealtime: pthread_setschedparam failed (%d)", ret);
        }
        return false;
    }

    TETHER_LOGI("Platform", "setCurrentThreadRealtime: SCHED_FIFO priority=%d applied", param.sched_priority);
    return true;
#else
    // Non-Linux platforms: best-effort no-op (ESP32 FreeRTOS scheduling handled elsewhere)
    (void)priority;
    return true;
#endif
}
} // namespace Platform
} // namespace Tether
