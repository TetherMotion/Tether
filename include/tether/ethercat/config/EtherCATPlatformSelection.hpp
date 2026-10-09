/**
 * @file EtherCATPlatformSelection.hpp
 * @brief EtherCAT config: PLATFORM SELECTION
 *
 * Split out of EtherCATConfig.hpp.
 */

#pragma once

// Pull in the generated top-level feature flags (TETHER_ENABLE_*).
#if __has_include("tether/TetherConfig.hpp")
#include "tether/TetherConfig.hpp"
#endif

// ============================================================================
// PLATFORM SELECTION
// ============================================================================

/**
 * @brief Select the platform for filesystem operations (FoE)
 * 
 * Options:
 * - ECAT_PLATFORM_ESP32_LITTLEFS: ESP32 with LittleFS (recommended for ESP32)
 * - ECAT_PLATFORM_ESP32_SPIFFS: ESP32 with SPIFFS (legacy)
 * - ECAT_PLATFORM_POSIX: Standard POSIX file I/O (Linux testing)
 * - ECAT_PLATFORM_NONE: No filesystem (FoE disabled or custom implementation)
 * 
 * @note Auto-detected based on ESP_PLATFORM if not explicitly set
 */
#define ECAT_PLATFORM_ESP32_LITTLEFS    1
#define ECAT_PLATFORM_ESP32_SPIFFS      2
#define ECAT_PLATFORM_POSIX             3
#define ECAT_PLATFORM_NONE              0

#ifndef ECAT_PLATFORM_FILESYSTEM
#if defined(ESP_PLATFORM)
    #define ECAT_PLATFORM_FILESYSTEM    ECAT_PLATFORM_ESP32_LITTLEFS
#elif defined(__linux__) || defined(__unix__)
    #define ECAT_PLATFORM_FILESYSTEM    ECAT_PLATFORM_POSIX
#else
    #define ECAT_PLATFORM_FILESYSTEM    ECAT_PLATFORM_NONE
#endif
#endif

/**
 * @brief LittleFS partition label (ESP32 only)
 * 
 * The partition label in partitions.csv where LittleFS is mounted.
 * Default: "storage"
 */
#ifndef ECAT_LITTLEFS_PARTITION_LABEL
#define ECAT_LITTLEFS_PARTITION_LABEL   "storage"
#endif

/**
 * @brief LittleFS mount point (ESP32 only)
 * 
 * The VFS mount path for LittleFS.
 * Default: "/littlefs"
 */
#ifndef ECAT_LITTLEFS_MOUNT_POINT
#define ECAT_LITTLEFS_MOUNT_POINT       "/littlefs"
#endif
