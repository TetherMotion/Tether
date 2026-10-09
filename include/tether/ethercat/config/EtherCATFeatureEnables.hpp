/**
 * @file EtherCATFeatureEnables.hpp
 * @brief EtherCAT config: FEATURE ENABLES
 *
 * Split out of EtherCATConfig.hpp.
 */

#pragma once

// Pull in the generated top-level feature flags (TETHER_ENABLE_*).
#if __has_include("tether/TetherConfig.hpp")
#include "tether/TetherConfig.hpp"
#endif

// ============================================================================
// FEATURE ENABLES
// ============================================================================
// Set to 1 to enable, 0 to disable. Disabled features are compiled out
// completely, saving both code space and RAM.

/**
 * @brief Enable Process Data Object (PDO) support
 * 
 * PDOs are the primary mechanism for realtime cyclic data exchange.
 * Required for most motion control and I/O applications.
 * 
 * @note Recommendation: Enable unless you only need configuration/diagnostics
 * 
 * Dependencies: None
 * Code size: ~4 KB
 * RAM usage: ~2 KB (scales with number of slaves and PDO sizes)
 */
#ifndef ECAT_FEATURE_PDO_ENABLED
#define ECAT_FEATURE_PDO_ENABLED    1
#endif

/**
 * @brief Enable Service Data Object (SDO) support
 * 
 * SDOs provide non-realtime access to the slave's object dictionary.
 * Used for configuration, diagnostics, and parameter access.
 * 
 * @note Recommendation: Enable for most applications
 * 
 * Dependencies: Mailbox protocol (SM0/SM1)
 * Code size: ~6 KB
 * RAM usage: ~1 KB (queue + buffers)
 */
#ifndef ECAT_FEATURE_SDO_ENABLED
#define ECAT_FEATURE_SDO_ENABLED    1
#endif

/**
 * @brief Enable Distributed Clocks (DC) support
 * 
 * DC provides synchronized timing across all slaves in the network.
 * Essential for coordinated multi-axis motion control.
 * 
 * @note Recommendation: Enable for motion control, disable for simple I/O
 * 
 * Dependencies: Hardware timer (GPTimer on ESP32)
 * Code size: ~5 KB
 * RAM usage: ~0.5 KB
 */
#ifndef ECAT_FEATURE_DC_ENABLED
#define ECAT_FEATURE_DC_ENABLED     1
#endif

/**
 * @brief Enable File over EtherCAT (FoE) support
 * 
 * FoE allows file transfers to/from slaves. Primary uses:
 * - Firmware updates (bootloader mode)
 * - Configuration file upload/download
 * - Log file retrieval
 * 
 * @note Recommendation: Enable if slaves support firmware updates
 * 
 * Dependencies: SDO (mailbox), Platform filesystem abstraction
 * Code size: ~8 KB
 * RAM usage: ~4 KB (file buffer + state)
 */
#ifndef ECAT_FEATURE_FOE_ENABLED
#define ECAT_FEATURE_FOE_ENABLED    1
#endif

/**
 * @brief Enable Vendor-specific over EtherCAT (VoE) support
 * 
 * VoE provides a vendor-specific mailbox channel for proprietary
 * features not covered by standard protocols. Uses include:
 * - Proprietary diagnostic protocols
 * - Custom firmware features
 * - Vendor-specific configuration
 * 
 * @note Recommendation: Enable only if your slaves use VoE
 * 
 * Dependencies: SDO (mailbox)
 * Code size: ~3 KB
 * RAM usage: ~1 KB
 */
#ifndef ECAT_FEATURE_VOE_ENABLED
#define ECAT_FEATURE_VOE_ENABLED    1
#endif

/**
 * @brief Enable Ethernet over EtherCAT (EoE) support
 * 
 * EoE tunnels standard Ethernet/IP traffic through the EtherCAT network.
 * This allows slaves to have virtual Ethernet ports for:
 * - Remote configuration via web interface
 * - TCP/IP diagnostics
 * - Integration with IT networks
 * 
 * @warning EoE adds significant overhead. Only enable if required.
 * 
 * @note Recommendation: Disable for pure motion control applications
 * 
 * Dependencies: SDO (mailbox), Network stack (lwIP on ESP32)
 * Code size: ~10 KB
 * RAM usage: ~8 KB (frame buffers + state)
 */
#ifndef ECAT_FEATURE_EOE_ENABLED
#define ECAT_FEATURE_EOE_ENABLED    1
#endif
