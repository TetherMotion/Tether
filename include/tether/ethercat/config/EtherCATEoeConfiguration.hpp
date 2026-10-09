/**
 * @file EtherCATEoeConfiguration.hpp
 * @brief EtherCAT config: EOE CONFIGURATION
 *
 * Split out of EtherCATConfig.hpp.
 */

#pragma once

// Pull in the generated top-level feature flags (TETHER_ENABLE_*).
#if __has_include("tether/TetherConfig.hpp")
#include "tether/TetherConfig.hpp"
#endif

// ============================================================================
// EOE CONFIGURATION
// ============================================================================

/**
 * @brief EoE frame buffer count
 * 
 * Number of Ethernet frame buffers for EoE. More buffers improve throughput
 * but use more RAM.
 * 
 * @note Recommendation: 4-8 for typical use, increase for high-throughput
 * 
 * Memory impact: 1518 bytes per buffer (max Ethernet frame)
 * Default: 4
 */
#ifndef ECAT_EOE_FRAME_BUFFER_COUNT
#define ECAT_EOE_FRAME_BUFFER_COUNT     4
#endif

/**
 * @brief EoE maximum frame size (bytes)
 * 
 * Maximum Ethernet frame size to handle. Standard is 1518 bytes.
 * 
 * Default: 1518
 */
#ifndef ECAT_EOE_MAX_FRAME_SIZE
#define ECAT_EOE_MAX_FRAME_SIZE         1518
#endif

/**
 * @brief EoE fragment size (bytes)
 * 
 * Size of each EoE fragment in mailbox. Must fit in mailbox with headers.
 * 
 * @note Recommendation: Match slave's mailbox size minus ~32 bytes for headers
 * 
 * Default: 256
 */
#ifndef ECAT_EOE_FRAGMENT_SIZE
#define ECAT_EOE_FRAGMENT_SIZE          256
#endif

/**
 * @brief EoE fragment reassembly timeout (milliseconds)
 * 
 * Timeout for receiving all fragments of an Ethernet frame.
 * 
 * Default: 1000
 */
#ifndef ECAT_EOE_FRAGMENT_TIMEOUT_MS
#define ECAT_EOE_FRAGMENT_TIMEOUT_MS    1000
#endif

/**
 * @brief Enable EoE IP address assignment
 * 
 * When enabled, the master can assign IP addresses to slaves.
 * 
 * Default: 1
 */
#ifndef ECAT_EOE_IP_ASSIGNMENT_ENABLED
#define ECAT_EOE_IP_ASSIGNMENT_ENABLED  1
#endif

/**
 * @brief EoE virtual network interface name
 * 
 * Name of the virtual network interface created for EoE (where supported).
 * 
 * Default: "eoe0"
 */
#ifndef ECAT_EOE_INTERFACE_NAME
#define ECAT_EOE_INTERFACE_NAME         "eoe0"
#endif
