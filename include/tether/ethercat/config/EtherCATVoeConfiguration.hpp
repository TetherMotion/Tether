/**
 * @file EtherCATVoeConfiguration.hpp
 * @brief EtherCAT config: VOE CONFIGURATION
 *
 * Split out of EtherCATConfig.hpp.
 */

#pragma once

// Pull in the generated top-level feature flags (TETHER_ENABLE_*).
#if __has_include("tether/TetherConfig.hpp")
#include "tether/TetherConfig.hpp"
#endif

// ============================================================================
// VOE CONFIGURATION
// ============================================================================

/**
 * @brief VoE maximum data size (bytes)
 * 
 * Maximum payload size for vendor-specific mailbox messages.
 * 
 * @note Recommendation: Match your vendor's protocol requirements
 * 
 * Default: 256
 */
#ifndef ECAT_VOE_MAX_DATA_SIZE
#define ECAT_VOE_MAX_DATA_SIZE          256
#endif

/**
 * @brief VoE request queue depth
 * 
 * Number of VoE requests that can be queued.
 * 
 * Default: 8
 */
#ifndef ECAT_VOE_QUEUE_DEPTH
#define ECAT_VOE_QUEUE_DEPTH            8
#endif

/**
 * @brief VoE timeout (milliseconds)
 * 
 * Default: 1000
 */
#ifndef ECAT_VOE_TIMEOUT_MS
#define ECAT_VOE_TIMEOUT_MS             1000
#endif
