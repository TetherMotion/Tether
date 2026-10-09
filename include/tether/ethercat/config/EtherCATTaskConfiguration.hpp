/**
 * @file EtherCATTaskConfiguration.hpp
 * @brief EtherCAT config: TASK CONFIGURATION
 *
 * Split out of EtherCATConfig.hpp.
 */

#pragma once

// Pull in the generated top-level feature flags (TETHER_ENABLE_*).
#if __has_include("tether/TetherConfig.hpp")
#include "tether/TetherConfig.hpp"
#endif

// ============================================================================
// TASK CONFIGURATION
// ============================================================================

/**
 * @brief SDO processing task priority
 * 
 * FreeRTOS priority for the background SDO task.
 * Should be lower than realtime task but higher than idle.
 * 
 * Default: 5
 */
#ifndef ECAT_SDO_TASK_PRIORITY
#define ECAT_SDO_TASK_PRIORITY          5
#endif

/**
 * @brief SDO task stack size (bytes)
 * 
 * Default: 4096
 */
#ifndef ECAT_SDO_TASK_STACK_SIZE
#define ECAT_SDO_TASK_STACK_SIZE        4096
#endif

/**
 * @brief SDO task CPU core (ESP32)
 * 
 * Which core to pin the SDO task to. Use tskNO_AFFINITY for no pinning.
 * 
 * Default: 0
 */
#ifndef ECAT_SDO_TASK_CORE
#define ECAT_SDO_TASK_CORE              0
#endif

/**
 * @brief FoE processing task priority
 * 
 * Default: 4 (lower than SDO)
 */
#ifndef ECAT_FOE_TASK_PRIORITY
#define ECAT_FOE_TASK_PRIORITY          4
#endif

/**
 * @brief FoE task stack size (bytes)
 * 
 * Default: 8192 (needs more for file operations)
 */
#ifndef ECAT_FOE_TASK_STACK_SIZE
#define ECAT_FOE_TASK_STACK_SIZE        8192
#endif

/**
 * @brief EoE processing task priority
 * 
 * Default: 6 (higher than SDO for network responsiveness)
 */
#ifndef ECAT_EOE_TASK_PRIORITY
#define ECAT_EOE_TASK_PRIORITY          6
#endif

/**
 * @brief EoE task stack size (bytes)
 * 
 * Default: 4096
 */
#ifndef ECAT_EOE_TASK_STACK_SIZE
#define ECAT_EOE_TASK_STACK_SIZE        4096
#endif

/**
 * @brief Realtime (DC) task priority
 * 
 * Should be the highest priority task.
 * 
 * Default: configMAX_PRIORITIES - 1
 */
#ifndef ECAT_DC_TASK_PRIORITY
#define ECAT_DC_TASK_PRIORITY           (configMAX_PRIORITIES - 1)
#endif

/**
 * @brief Realtime task stack size (bytes)
 * 
 * Default: 4096
 */
#ifndef ECAT_DC_TASK_STACK_SIZE
#define ECAT_DC_TASK_STACK_SIZE         4096
#endif

/**
 * @brief Realtime task CPU core (ESP32)
 * 
 * @note Recommendation: Pin to core 1, keep core 0 for WiFi/BT
 * 
 * Default: 1
 */
#ifndef ECAT_DC_TASK_CORE
#define ECAT_DC_TASK_CORE               1
#endif
