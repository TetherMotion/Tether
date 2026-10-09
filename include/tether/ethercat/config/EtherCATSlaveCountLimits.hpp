/**
 * @file EtherCATSlaveCountLimits.hpp
 * @brief EtherCAT config: SLAVE COUNT LIMITS (Tether-internal)
 *
 * Split out of EtherCATConfig.hpp.
 */

#pragma once

// Pull in the generated top-level feature flags (TETHER_ENABLE_*).
#if __has_include("tether/TetherConfig.hpp")
#include "tether/TetherConfig.hpp"
#endif

// ============================================================================
// SLAVE COUNT LIMITS (Tether-internal)
// ============================================================================

/**
 * @brief Maximum slaves for fault detection
 *
 * Limits the FaultDetector's internal slave state array. This is a
 * Tether-internal limit — if the actual slave count exceeds this value,
 * Tether will silently clamp to this limit. Increase if you have more
 * slaves and need fault detection for all of them.
 *
 * Default: 64
 */
#ifndef ECAT_FAULT_DETECTION_MAX_SLAVES
#define ECAT_FAULT_DETECTION_MAX_SLAVES 64
#endif

/**
 * @brief Maximum slaves for status polling
 *
 * Limits the SlaveStatusPoller's internal slave array. This is a
 * Tether-internal limit.
 *
 * Default: 64
 */
#ifndef ECAT_STATUS_POLLER_MAX_SLAVES
#define ECAT_STATUS_POLLER_MAX_SLAVES   64
#endif

/**
 * @brief Maximum slaves for Distributed Clocks
 *
 * Limits the DC subsystem's per-slave time info array. This is a
 * Tether-internal limit.
 *
 * Default: 64
 */
#ifndef ECAT_DC_MAX_SLAVES
#define ECAT_DC_MAX_SLAVES              64
#endif
