/**
 * @file EtherCATTimingAndPerformance.hpp
 * @brief EtherCAT config: TIMING AND PERFORMANCE
 *
 * Split out of EtherCATConfig.hpp.
 */

#pragma once

// Pull in the generated top-level feature flags (TETHER_ENABLE_*).
#if __has_include("tether/TetherConfig.hpp")
#include "tether/TetherConfig.hpp"
#endif

// ============================================================================
// TIMING AND PERFORMANCE
// ============================================================================

/**
 * @brief Mailbox polling interval (milliseconds)
 * 
 * How often to poll slave mailboxes for responses when not using interrupts.
 * 
 * @note Recommendation: 1-5ms. Lower = faster response, higher CPU usage
 * 
 * Default: 1
 */
#ifndef ECAT_MAILBOX_POLL_INTERVAL_MS
#define ECAT_MAILBOX_POLL_INTERVAL_MS   1
#endif

/**
 * @brief Ethernet frame timeout (milliseconds)
 * 
 * Timeout waiting for EtherCAT frame to return from the network.
 * 
 * @note Recommendation: 10-50ms. Longer for noisy/long networks.
 * 
 * Default: 20
 */
#ifndef ECAT_FRAME_TIMEOUT_MS
#define ECAT_FRAME_TIMEOUT_MS           20
#endif

/**
 * @brief State change timeout (milliseconds)
 * 
 * Timeout waiting for slave state machine transitions.
 * 
 * Default: 5000
 */
#ifndef ECAT_STATE_CHANGE_TIMEOUT_MS
#define ECAT_STATE_CHANGE_TIMEOUT_MS    5000
#endif

/**
 * @brief Maximum TX retries for EtherCAT frame transmission
 *
 * Number of times Tether will retransmit an EtherCAT frame if the
 * raw socket send fails. This is a Tether-internal limit.
 *
 * Default: 3
 */
#ifndef ECAT_TX_MAX_RETRIES
#define ECAT_TX_MAX_RETRIES             3
#endif

/**
 * @brief TX retry delay (microseconds)
 *
 * Delay between TX retry attempts. This is a Tether-internal parameter.
 *
 * Default: 50
 */
#ifndef ECAT_TX_RETRY_DELAY_US
#define ECAT_TX_RETRY_DELAY_US          50
#endif
