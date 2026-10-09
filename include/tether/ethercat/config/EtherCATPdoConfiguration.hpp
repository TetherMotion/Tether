/**
 * @file EtherCATPdoConfiguration.hpp
 * @brief EtherCAT config: PDO CONFIGURATION
 *
 * Split out of EtherCATConfig.hpp.
 */

#pragma once

// Pull in the generated top-level feature flags (TETHER_ENABLE_*).
#if __has_include("tether/TetherConfig.hpp")
#include "tether/TetherConfig.hpp"
#endif

// ============================================================================
// PDO CONFIGURATION
// ============================================================================

/**
 * @brief Maximum number of slaves for PDO mapping
 *
 * Limits memory allocation for slave configuration arrays.
 * This is a Tether-internal limit — if the actual slave count exceeds
 * this value, Tether will reject the configuration with an error that
 * explicitly names Tether's internal limit as the constraint.
 *
 * @note Recommendation: Set to actual expected slave count + small margin
 *
 * Memory impact: ~64 bytes per slot
 * Range: 1-247 (EtherCAT limit)
 * Default: 64
 */
#ifndef ECAT_PDO_MAX_SLAVES
#define ECAT_PDO_MAX_SLAVES             64
#endif

/**
 * @brief Maximum total PDO entries across all slaves
 *
 * Limits the total number of PDO mapping entries in the PDOMapping.
 * This is a Tether-internal limit — if the total entries exceed this
 * value, Tether will reject the mapping with an error that explicitly
 * names Tether's internal limit as the constraint.
 *
 * @note Recommendation: 16-32 for typical applications, higher for complex setups
 *
 * Memory impact: ~32 bytes per entry
 * Default: 32
 */
#ifndef ECAT_PDO_MAX_ENTRIES
#define ECAT_PDO_MAX_ENTRIES            32
#endif

/**
 * @brief Maximum PDO entries per slave
 *
 * Limits the number of PDO objects that can be mapped per slave.
 *
 * @note Recommendation: Most slaves have 2-8 PDOs. Set higher for complex slaves.
 *
 * Memory impact: ~32 bytes per entry per slave
 * Default: 16
 */
#ifndef ECAT_PDO_MAX_ENTRIES_PER_SLAVE
#define ECAT_PDO_MAX_ENTRIES_PER_SLAVE  16
#endif

/**
 * @brief Maximum PDO data buffer size (bytes)
 *
 * Size of the internal PDO data exchange buffer. Must accommodate the
 * largest single PDO data transfer. This is a Tether-internal limit —
 * if a PDO exceeds this size, Tether will reject it with an error that
 * explicitly names Tether's internal buffer as the limiting factor.
 *
 * @note Recommendation: 1024 covers most applications. Increase for very large PDOs.
 *
 * Memory impact: 2x this value (double buffering)
 * Default: 1024
 */
#ifndef ECAT_PDO_MAX_BUFFER_SIZE
#define ECAT_PDO_MAX_BUFFER_SIZE        1024
#endif

/**
 * @brief Maximum total PDO data size per slave (bytes)
 *
 * Buffer size for PDO data exchange. Must accommodate largest PDO mapping.
 *
 * @note Recommendation: Sum of all PDO sizes for your largest slave + padding
 * 
 * Memory impact: 2x this value per slave (double buffering)
 * Default: 256
 */
#ifndef ECAT_PDO_MAX_DATA_SIZE
#define ECAT_PDO_MAX_DATA_SIZE          256
#endif
