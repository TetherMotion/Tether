/**
 * @file EtherCATStatistics.hpp
 * @brief EtherCAT config: STATISTICS (compile-time switch — controllable via CMake)
 *
 * Split out of EtherCATConfig.hpp.
 */

#pragma once

// Pull in the generated top-level feature flags (TETHER_ENABLE_*).
#if __has_include("tether/TetherConfig.hpp")
#include "tether/TetherConfig.hpp"
#endif

// ============================================================================
// STATISTICS (compile-time switch — controllable via CMake)
// ============================================================================
// Set via CMake option TETHER_ENABLE_ETHERCAT_STATS (ON/OFF).
// When disabled, all statistics collection is compiled out.

#ifndef TETHER_ENABLE_ETHERCAT_STATS
#define TETHER_ENABLE_ETHERCAT_STATS 1
#endif
