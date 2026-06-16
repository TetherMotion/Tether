#pragma once

/**
 * @file EtherCATRaw.hpp
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
 *     │              EtherCATRaw.hpp (This file)        │
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
 * - **EtherCATDC.hpp**: Distributed Clock synchronization
 * - **EtherCATPDO.hpp**: Process Data Object (cyclic data)
 * - **EtherCATSDO.hpp**: Service Data Object (async parameters)
 * - **EtherCATFoE.hpp**: File over EtherCAT (firmware updates, file transfer)
 * - **EtherCATVoE.hpp**: Vendor over EtherCAT (vendor-specific messaging)
 * - **EtherCATEoE.hpp**: Ethernet over EtherCAT (IP connectivity)
 * 
 * @see EtherCATDC.hpp for DC synchronization API
 * @see EtherCATPDO.hpp for PDO configuration and mapping
 * @see EtherCATSDO.hpp for asynchronous SDO access
 * @see EtherCATFoE.hpp for file transfer API
 * @see EtherCATVoE.hpp for vendor-specific communication
 * @see EtherCATEoE.hpp for virtual Ethernet interface
 */

#include <cstddef>
#include <cstdint>

#include "tether/platform/EspCompat.hpp"
#ifdef ESP_PLATFORM
#include "esp_eth_driver.h"
#endif

// Include configuration first
#include "EtherCATConfig.hpp"

// Class-based master API (new)
#include "EtherCATMaster.hpp"

// Core sub-module APIs
#include "EtherCATDC.hpp"
#include "EtherCATPDO.hpp"
#include "EtherCATSDO.hpp"

// Mailbox protocol APIs (conditionally included based on config)
#if ECAT_FEATURE_FOE_ENABLED
#include "EtherCATFoE.hpp"
#endif

#if ECAT_FEATURE_VOE_ENABLED
#include "EtherCATVoE.hpp"
#endif

#if ECAT_FEATURE_EOE_ENABLED
#include "EtherCATEoE.hpp"
#endif

namespace EtherCAT {

// All EtherCAT master functionality is now in the class-based API.
// Use Master instances directly — no free-function API is provided.

} // namespace EtherCAT
