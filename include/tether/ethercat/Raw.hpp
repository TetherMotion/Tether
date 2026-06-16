#pragma once

/**
 * @file Raw.hpp
 * @brief EtherCAT Raw Master - Main Public API
 * 
 * @details
 * This header provides the top-level API for the EtherCAT raw master
 * implementation. The "raw" master is a lightweight implementation that
 * provides direct control over EtherCAT primitives without the overhead
 * of a full master stack.
 * 
 * ## Architecture Overview
 * 
 * ```
 *                         Application
 *                              │
 *     ┌────────────────────────┼────────────────────────┐
 *     │              Raw.hpp (This file)        │
 *     │              - StartMasterTask()                │
 *     │              - HandleRxFrame()                  │
 *     └────────────────────────┬────────────────────────┘
 *                              │
 *     ┌────────────────────────┼────────────────────────────────┐
 *     │                    Sub-modules                          │
 *     │  ┌──────────┐ ┌──────────┐ ┌──────────┐ ┌──────────┐   │
 *     │  │   DC     │ │   PDO    │ │   SDO    │ │ Mailbox  │   │
 *     │  │ (sync)   │ │ (cyclic) │ │ (async)  │ │ FoE/VoE  │   │
 *     │  └──────────┘ └──────────┘ └──────────┘ │   EoE    │   │
 *     │                                         └──────────┘   │
 *     └────────────────────────┬───────────────────────────────┘
 *                              │
 *     ┌────────────────────────┼────────────────────────┐
 *     │              raw/internal.hpp                   │
 *     │              - Transport primitives             │
 *     │              - Frame building                   │
 *     └────────────────────────┬────────────────────────┘
 *                              │
 *     ┌────────────────────────┼────────────────────────┐
 *     │              ESP-IDF Ethernet Driver            │
 *     │              (esp_eth.h)                        │
 *     └─────────────────────────────────────────────────┘
 * ```
 * 
 * ## Startup Sequence
 * 
 * 1. Initialize Ethernet hardware and get platform-specific handle
 * 2. Call `EtherCAT::StartMasterTask(eth_handle)`
 * 3. The master task automatically:
 *    - Discovers slaves on the network (BRD AL_STATUS)
 *    - Transitions slaves to PRE_OP state
 *    - Reads device identity via CoE SDO
 *    - Initializes DC synchronization
 *    - Starts the 1kHz realtime loop
 * 
 * ## Sub-module APIs
 * 
 * For detailed control, use the sub-module headers:
 * - **DC.hpp**: Distributed Clock synchronization
 * - **PDOManager.hpp**: Process Data Object (cyclic data)
 * - **SDOManager.hpp**: Service Data Object (async parameters)
 * - **FoE.hpp**: File over EtherCAT (firmware updates, file transfer)
 * - **VoE.hpp**: Vendor over EtherCAT (vendor-specific messaging)
 * - **EoE.hpp**: Ethernet over EtherCAT (IP connectivity)
 * 
 * @see DC.hpp for DC synchronization API
 * @see PDOManager.hpp for PDO configuration and mapping
 * @see SDOManager.hpp for asynchronous SDO access
 * @see FoE.hpp for file transfer API
 * @see VoE.hpp for vendor-specific communication
 * @see EoE.hpp for virtual Ethernet interface
 */

#include <cstddef>
#include <cstdint>

#include "tether/platform/EspCompat.hpp"
#ifdef ESP_PLATFORM
#include "esp_eth_driver.h"
#endif

// Include configuration first
#include "TetherConfig.hpp"

// Class-based master API (new)
#include "Master.hpp"

// Core sub-module APIs
#include "DC.hpp"
#include "PDOManager.hpp"
#include "SDOManager.hpp"

// Mailbox protocol APIs (conditionally included based on config)
#if ECAT_FEATURE_FOE_ENABLED
#include "FoE.hpp"
#endif

#if ECAT_FEATURE_VOE_ENABLED
#include "VoE.hpp"
#endif

#if ECAT_FEATURE_EOE_ENABLED
#include "EoE.hpp"
#endif

namespace EtherCAT {

// All EtherCAT master functionality is now in the class-based API.
// Use Master instances directly — no free-function API is provided.

} // namespace EtherCAT
