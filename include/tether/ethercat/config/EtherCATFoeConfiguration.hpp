/**
 * @file EtherCATFoeConfiguration.hpp
 * @brief EtherCAT config: FOE CONFIGURATION
 *
 * Split out of EtherCATConfig.hpp.
 */

#pragma once

// Pull in the generated top-level feature flags (TETHER_ENABLE_*).
#if __has_include("tether/TetherConfig.hpp")
#include "tether/TetherConfig.hpp"
#endif

// ============================================================================
// FOE CONFIGURATION
// ============================================================================

/**
 * @brief FoE transfer buffer size (bytes)
 * 
 * Size of the buffer used for FoE file transfers. Larger buffers improve
 * throughput but use more RAM.
 * 
 * @note Recommendation: 512-1024 for good balance. Match slave's mailbox size if known.
 * 
 * Memory impact: 2x this value (read + write buffers)
 * Default: 512
 */
#ifndef ECAT_FOE_BUFFER_SIZE
#define ECAT_FOE_BUFFER_SIZE            512
#endif

/**
 * @brief FoE maximum filename length
 * 
 * Maximum length for FoE filenames including null terminator.
 * 
 * Default: 64
 */
#ifndef ECAT_FOE_MAX_FILENAME
#define ECAT_FOE_MAX_FILENAME           64
#endif

/**
 * @brief FoE transfer timeout (milliseconds)
 * 
 * Timeout for each FoE packet exchange. Total transfer timeout is
 * this value multiplied by number of packets.
 * 
 * @note Recommendation: 3000-5000ms for firmware updates (flash write is slow)
 * 
 * Default: 5000
 */
#ifndef ECAT_FOE_TIMEOUT_MS
#define ECAT_FOE_TIMEOUT_MS             5000
#endif

/**
 * @brief FoE password for protected transfers
 * 
 * Default password sent with FoE requests. Many slaves ignore this.
 * 
 * Default: 0 (no password)
 */
#ifndef ECAT_FOE_DEFAULT_PASSWORD
#define ECAT_FOE_DEFAULT_PASSWORD       0
#endif

/**
 * @brief Maximum concurrent FoE transfers
 * 
 * Number of simultaneous FoE transfers supported.
 * 
 * @note Recommendation: 1 is usually sufficient. Increase for parallel updates.
 * 
 * Memory impact: ~1KB per transfer slot
 * Default: 1
 */
#ifndef ECAT_FOE_MAX_TRANSFERS
#define ECAT_FOE_MAX_TRANSFERS          1
#endif
