/**
 * @file EtherCATSiiParserLimits.hpp
 * @brief EtherCAT config: SII PARSER LIMITS (Tether-internal)
 *
 * Split out of EtherCATConfig.hpp.
 */

#pragma once

// Pull in the generated top-level feature flags (TETHER_ENABLE_*).
#if __has_include("tether/TetherConfig.hpp")
#include "tether/TetherConfig.hpp"
#endif

// ============================================================================
// SII PARSER LIMITS (Tether-internal)
// ============================================================================

/**
 * @brief Maximum number of strings in SII string category
 *
 * Limits how many indexed strings the SII parser can store.
 * This is a Tether-internal limit — if a slave's SII has more strings,
 * the excess strings are silently dropped.
 *
 * Default: 32
 */
#ifndef ECAT_SII_MAX_STRINGS
#define ECAT_SII_MAX_STRINGS            32
#endif

/**
 * @brief Maximum SII string buffer size (bytes)
 *
 * Total storage for all SII string data. This is a Tether-internal
 * limit — if the total string data exceeds this, excess is silently
 * dropped.
 *
 * Default: 2048
 */
#ifndef ECAT_SII_MAX_STRING_BUFFER
#define ECAT_SII_MAX_STRING_BUFFER      2048
#endif
