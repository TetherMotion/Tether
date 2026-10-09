/**
 * @file EtherCATDebugAndLogging.hpp
 * @brief EtherCAT config: DEBUG AND LOGGING
 *
 * Split out of EtherCATConfig.hpp.
 */

#pragma once

// Pull in the generated top-level feature flags (TETHER_ENABLE_*).
#if __has_include("tether/TetherConfig.hpp")
#include "tether/TetherConfig.hpp"
#endif

// ============================================================================
// DEBUG AND LOGGING
// ============================================================================

/**
 * @brief Enable verbose EtherCAT logging
 * 
 * When enabled, logs detailed information about EtherCAT operations.
 * Useful for debugging but adds overhead.
 * 
 * Default: 0 (disabled in production)
 */
#ifndef ECAT_DEBUG_LOGGING
#define ECAT_DEBUG_LOGGING              0
#endif

/**
 * @brief Enable frame dump logging
 * 
 * When enabled, logs hex dumps of EtherCAT frames.
 * Very verbose - use only for protocol debugging.
 * 
 * Default: 0
 */
#ifndef ECAT_DEBUG_FRAME_DUMP
#define ECAT_DEBUG_FRAME_DUMP           0
#endif

/**
 * @brief ESI (EtherCAT Slave Information) XML parser support
 *
 * When enabled (set to 1 by CMake when the tether_esi library is built),
 * ESIFile can parse XML files and the ESI-based Master/Slave overloads
 * are fully functional. When disabled (0), constructing an ESIFile from
 * a file path triggers a critical error (std::abort) because the ESI
 * parser library is not linked.
 *
 * Defined by CMake: tether_esi sets TETHER_HAVE_ESI=1 on all its
 * dependents. If tether_esi is not built, this defaults to 0.
 */
#ifndef TETHER_HAVE_ESI
#define TETHER_HAVE_ESI               0
#endif

/**
 * @brief Enable conditional debug gating framework
 *
 * When enabled (default), the DebugGate system is compiled in, allowing
 * debug output to be conditionally activated/deactivated based on
 * start/stop conditions (state transitions, checkpoints, register/CoE reads).
 * When disabled, all DebugGate code compiles to no-ops with zero overhead.
 *
 * Can be overridden via CMake: -DTETHER_DEBUG_GATE_ENABLED=OFF
 */
#ifndef TETHER_DEBUG_GATE_ENABLED
#define TETHER_DEBUG_GATE_ENABLED       1
#endif

/**
 * @brief Log tag for EtherCAT modules
 *
 * Used with TETHER_LOG* macros for platform-independent logging.
 */
#ifndef ECAT_LOG_TAG
#define ECAT_LOG_TAG                    "ECAT"
#endif
