/**
 * @file EtherCATDcConfiguration.hpp
 * @brief EtherCAT config: DC CONFIGURATION
 *
 * Split out of EtherCATConfig.hpp.
 */

#pragma once

// Pull in the generated top-level feature flags (TETHER_ENABLE_*).
#if __has_include("tether/TetherConfig.hpp")
#include "tether/TetherConfig.hpp"
#endif

// ============================================================================
// DC CONFIGURATION
// ============================================================================

/**
 * @brief DC sync cycle time (nanoseconds)
 * 
 * Target cycle time for DC synchronization. Common values:
 * - 1000000 (1ms) - Standard industrial
 * - 500000 (500µs) - Fast I/O
 * - 250000 (250µs) - High-performance motion
 * 
 * @warning Shorter cycles require faster hardware and optimized code
 * 
 * Default: 1000000 (1ms)
 */
#ifndef ECAT_DC_CYCLE_TIME_NS
#define ECAT_DC_CYCLE_TIME_NS           1000000
#endif

/**
 * @brief DC drift compensation enabled
 * 
 * Enable automatic compensation for clock drift between master and slaves.
 * 
 * Default: 1
 */
#ifndef ECAT_DC_DRIFT_COMPENSATION
#define ECAT_DC_DRIFT_COMPENSATION      1
#endif

/**
 * @brief DC maximum allowed drift (nanoseconds)
 * 
 * Maximum acceptable clock drift before triggering resynchronization.
 * 
 * Default: 10000 (10µs)
 */
#ifndef ECAT_DC_MAX_DRIFT_NS
#define ECAT_DC_MAX_DRIFT_NS            10000
#endif
