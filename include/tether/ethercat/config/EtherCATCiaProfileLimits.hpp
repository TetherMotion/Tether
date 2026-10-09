/**
 * @file EtherCATCiaProfileLimits.hpp
 * @brief EtherCAT config: CIA PROFILE LIMITS (Tether-internal)
 *
 * Split out of EtherCATConfig.hpp.
 */

#pragma once

// Pull in the generated top-level feature flags (TETHER_ENABLE_*).
#if __has_include("tether/TetherConfig.hpp")
#include "tether/TetherConfig.hpp"
#endif

// ============================================================================
// CIA PROFILE LIMITS (Tether-internal)
// ============================================================================

/**
 * @brief Maximum managed CiA402 drives
 *
 * Limits the CiA402 DriveManager's internal drive array. This is a
 * Tether-internal limit.
 *
 * Default: 8
 */
#ifndef ECAT_CIA402_MAX_MANAGED_DRIVES
#define ECAT_CIA402_MAX_MANAGED_DRIVES  8
#endif

/**
 * @brief CiA402 maximum PDO buffer size (bytes)
 *
 * Maximum PDO buffer size per CiA402 drive. This is a Tether-internal
 * limit — if a drive's PDO mapping exceeds this, it will be rejected.
 *
 * Default: 256
 */
#ifndef ECAT_CIA402_MAX_PDO_BUFFER_SIZE
#define ECAT_CIA402_MAX_PDO_BUFFER_SIZE 256
#endif

/**
 * @brief Maximum identified slaves (CiA 301)
 *
 * Limits the batch slave identification array. This is a
 * Tether-internal limit.
 *
 * Default: 16
 */
#ifndef ECAT_CIA301_MAX_IDENTIFIED_SLAVES
#define ECAT_CIA301_MAX_IDENTIFIED_SLAVES 16
#endif

/**
 * @brief CiA 301 identity string buffer size (bytes)
 *
 * Total storage for slave identity strings (name, hardware version,
 * software version, manufacturer). This is a Tether-internal limit —
 * if the total string data exceeds this, excess is silently dropped.
 *
 * Default: 512
 */
#ifndef ECAT_CIA301_MAX_IDENTITY_STRING_BUFFER
#define ECAT_CIA301_MAX_IDENTITY_STRING_BUFFER 512
#endif
