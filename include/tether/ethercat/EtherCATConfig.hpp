/**
 * @file EtherCATConfig.hpp
 * @brief Centralized configuration for EtherCAT stack features
 * 
 * @details
 * This file contains all user-configurable options for the EtherCAT stack.
 * Modify these settings to enable/disable features and tune performance
 * for your specific application requirements.
 * 
 * ## Configuration Categories
 * 
 * 1. **Feature Enables** - Turn protocols on/off to save code space
 * 2. **Buffer Sizes** - Tune memory usage vs. capability trade-offs
 * 3. **Timing Parameters** - Adjust timeouts and retry behavior
 * 4. **Platform Selection** - Choose platform-specific implementations
 * 
 * ## Memory Impact Summary
 * 
 * | Feature | Code Size | RAM Usage | Recommendation |
 * |---------|-----------|-----------|----------------|
 * | PDO     | ~4 KB     | ~2 KB     | Always enable for realtime |
 * | SDO     | ~6 KB     | ~1 KB     | Enable for configuration |
 * | FoE     | ~8 KB     | ~4 KB     | Enable for firmware updates |
 * | VoE     | ~3 KB     | ~1 KB     | Enable for vendor features |
 * | EoE     | ~10 KB    | ~8 KB     | Enable for IP networking |
 * | DC      | ~5 KB     | ~0.5 KB   | Enable for synchronized motion |
 * 
 * ## Quick Start Profiles
 * 
 * ### Minimal (Motion Control Only)
 * @code
 * #define ECAT_FEATURE_PDO_ENABLED    1
 * #define ECAT_FEATURE_SDO_ENABLED    1
 * #define ECAT_FEATURE_DC_ENABLED     1
 * #define ECAT_FEATURE_FOE_ENABLED    0
 * #define ECAT_FEATURE_VOE_ENABLED    0
 * #define ECAT_FEATURE_EOE_ENABLED    0
 * @endcode
 * 
 * ### Full Featured
 * @code
 * #define ECAT_FEATURE_PDO_ENABLED    1
 * #define ECAT_FEATURE_SDO_ENABLED    1
 * #define ECAT_FEATURE_DC_ENABLED     1
 * #define ECAT_FEATURE_FOE_ENABLED    1
 * #define ECAT_FEATURE_VOE_ENABLED    1
 * #define ECAT_FEATURE_EOE_ENABLED    1
 * @endcode
 * 
 * ### Firmware Update Station
 * @code
 * #define ECAT_FEATURE_PDO_ENABLED    0
 * #define ECAT_FEATURE_SDO_ENABLED    1
 * #define ECAT_FEATURE_DC_ENABLED     0
 * #define ECAT_FEATURE_FOE_ENABLED    1
 * #define ECAT_FEATURE_VOE_ENABLED    0
 * #define ECAT_FEATURE_EOE_ENABLED    0
 * @endcode
 */

#pragma once

// Pull in the generated top-level feature flags (TETHER_ENABLE_*). This makes
// the generated config visible to every TU that includes the EtherCAT config,
// so feature-gated declarations have the same shape in the library and in
// consumer code. Guarded so the header still works without a generated config.
#if __has_include("tether/TetherConfig.hpp")
#include "tether/TetherConfig.hpp"
#endif

// Individual configuration sections (split out of this header).

#include "tether/ethercat/config/EtherCATStatistics.hpp"
#include "tether/ethercat/config/EtherCATSiiEepromSupport.hpp"
#include "tether/ethercat/config/EtherCATUdpEncapsulation.hpp"
#include "tether/ethercat/config/EtherCATFeatureEnables.hpp"
#include "tether/ethercat/config/EtherCATPlatformSelection.hpp"
#include "tether/ethercat/config/EtherCATPdoConfiguration.hpp"
#include "tether/ethercat/config/EtherCATSdoConfiguration.hpp"
#include "tether/ethercat/config/EtherCATFoeConfiguration.hpp"
#include "tether/ethercat/config/EtherCATVoeConfiguration.hpp"
#include "tether/ethercat/config/EtherCATEoeConfiguration.hpp"
#include "tether/ethercat/config/EtherCATDcConfiguration.hpp"
#include "tether/ethercat/config/EtherCATTimingAndPerformance.hpp"
#include "tether/ethercat/config/EtherCATSlaveCountLimits.hpp"
#include "tether/ethercat/config/EtherCATSiiParserLimits.hpp"
#include "tether/ethercat/config/EtherCATDiscoveryLimits.hpp"
#include "tether/ethercat/config/EtherCATCiaProfileLimits.hpp"
#include "tether/ethercat/config/EtherCATDebugAndLogging.hpp"
#include "tether/ethercat/config/EtherCATTaskConfiguration.hpp"
#include "tether/ethercat/config/EtherCATValidation.hpp"
