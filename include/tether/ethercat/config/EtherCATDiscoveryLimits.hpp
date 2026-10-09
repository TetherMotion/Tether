/**
 * @file EtherCATDiscoveryLimits.hpp
 * @brief EtherCAT config: DISCOVERY LIMITS (Tether-internal)
 *
 * Split out of EtherCATConfig.hpp.
 */

#pragma once

// Pull in the generated top-level feature flags (TETHER_ENABLE_*).
#if __has_include("tether/TetherConfig.hpp")
#include "tether/TetherConfig.hpp"
#endif

// ============================================================================
// DISCOVERY LIMITS (Tether-internal)
// ============================================================================

/**
 * @brief Discovery mailbox write length (bytes)
 *
 * Mailbox write size used during slave discovery when SII-based
 * configuration is not available. This is a Tether-internal default
 * — some slaves may require larger or smaller mailboxes.
 *
 * Default: 128
 */
#ifndef ECAT_DISCOVERY_MBX_WRITE_LEN
#define ECAT_DISCOVERY_MBX_WRITE_LEN    128
#endif

/**
 * @brief Discovery mailbox read length (bytes)
 *
 * Mailbox read size used during slave discovery.
 *
 * Default: 128
 */
#ifndef ECAT_DISCOVERY_MBX_READ_LEN
#define ECAT_DISCOVERY_MBX_READ_LEN     128
#endif
