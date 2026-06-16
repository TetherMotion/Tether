/**
 * @file dc_time_source.cpp
 * @brief Platform-abstracted time source for EtherCAT DC synchronization
 *
 * This file provides weak symbol implementations for the time source interface.
 * The default implementation uses ESP32's esp_timer for microsecond resolution.
 *
 * To override for a different platform or higher precision source:
 * 1. Create a new source file with your platform's implementation
 * 2. Define the functions WITHOUT __attribute__((weak))
 * 3. Link your file instead of or alongside this one
 *
 * Example override for Linux host testing:
 * @code
 * extern "C" uint64_t ecdc_get_master_time_ns() {
 *     struct timespec ts;
 *     clock_gettime(CLOCK_MONOTONIC, &ts);
 *     return (uint64_t)ts.tv_sec * 1000000000ULL + ts.tv_nsec;
 * }
 * @endcode
 */

#include "tether/ethercat/DC.hpp"
#include "tether/platform/EspCompat.hpp"

#ifndef ESP_PLATFORM
#include <time.h>
#endif

static const char* TAG = "dc_time";

/**
 * @brief Get current master time in nanoseconds (weak default implementation)
 *
 * ESP32: Uses esp_timer_get_time() which provides microsecond resolution since boot.
 * Host: Uses clock_gettime(CLOCK_MONOTONIC) for nanosecond resolution.
 *
 * For higher precision, override with a hardware timer or external clock source.
 */
extern "C" __attribute__((weak))
uint64_t ecdc_get_master_time_ns()
{
#ifdef ESP_PLATFORM
    // esp_timer_get_time() returns microseconds since boot
    const int64_t us = esp_timer_get_time();
    return static_cast<uint64_t>(us) * 1000ULL;
#else
    // Host: use POSIX clock_gettime with CLOCK_MONOTONIC
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return static_cast<uint64_t>(ts.tv_sec) * 1000000000ULL + 
           static_cast<uint64_t>(ts.tv_nsec);
#endif
}

/**
 * @brief Initialize platform-specific time source (weak default implementation)
 *
 * ESP32: The ESP32 esp_timer is automatically initialized during system startup,
 * so this default implementation simply logs and returns success.
 *
 * Host: CLOCK_MONOTONIC doesn't need initialization.
 *
 * Override this for platforms that need explicit time source initialization.
 */
extern "C" __attribute__((weak))
bool ecdc_init_time_source()
{
#ifdef ESP_PLATFORM
    TETHER_LOGI(TAG, "Using ESP32 time source (esp_timer, μs resolution)");
#else
    TETHER_LOGI(TAG, "Using host time source (CLOCK_MONOTONIC, ns resolution)");
#endif
    return true;
}

/**
 * @brief Deinitialize platform-specific time source (weak default implementation)
 *
 * The ESP32 esp_timer doesn't need explicit deinitialization,
 * so this default implementation is a no-op.
 *
 * Override this for platforms that need cleanup.
 */
extern "C" __attribute__((weak))
void ecdc_deinit_time_source()
{
    // No cleanup needed for esp_timer
}
