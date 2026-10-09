/**
 * @file EtherCATValidation.hpp
 * @brief EtherCAT config: VALIDATION
 *
 * Split out of EtherCATConfig.hpp.
 */

#pragma once

// Pull in the generated top-level feature flags (TETHER_ENABLE_*).
#if __has_include("tether/TetherConfig.hpp")
#include "tether/TetherConfig.hpp"
#endif

// ============================================================================
// VALIDATION
// ============================================================================

// Validate feature dependencies
#if ECAT_FEATURE_FOE_ENABLED && !ECAT_FEATURE_SDO_ENABLED
    #warning "FoE requires SDO support. Enabling SDO."
    #undef ECAT_FEATURE_SDO_ENABLED
    #define ECAT_FEATURE_SDO_ENABLED 1
#endif

#if ECAT_FEATURE_VOE_ENABLED && !ECAT_FEATURE_SDO_ENABLED
    #warning "VoE requires SDO support. Enabling SDO."
    #undef ECAT_FEATURE_SDO_ENABLED
    #define ECAT_FEATURE_SDO_ENABLED 1
#endif

#if ECAT_FEATURE_EOE_ENABLED && !ECAT_FEATURE_SDO_ENABLED
    #warning "EoE requires SDO support. Enabling SDO."
    #undef ECAT_FEATURE_SDO_ENABLED
    #define ECAT_FEATURE_SDO_ENABLED 1
#endif

// Validate buffer sizes
#if ECAT_PDO_MAX_SLAVES > 247
    #error "ECAT_PDO_MAX_SLAVES cannot exceed 247 (EtherCAT limit)"
#endif

#if ECAT_FOE_BUFFER_SIZE < 128
    #error "ECAT_FOE_BUFFER_SIZE must be at least 128 bytes"
#endif

#if ECAT_EOE_FRAGMENT_SIZE < 64
    #error "ECAT_EOE_FRAGMENT_SIZE must be at least 64 bytes"
#endif
