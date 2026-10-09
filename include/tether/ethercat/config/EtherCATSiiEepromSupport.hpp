/**
 * @file EtherCATSiiEepromSupport.hpp
 * @brief EtherCAT config: SII EEPROM SUPPORT (compile-time switch)
 *
 * Split out of EtherCATConfig.hpp.
 */

#pragma once

// Pull in the generated top-level feature flags (TETHER_ENABLE_*).
#if __has_include("tether/TetherConfig.hpp")
#include "tether/TetherConfig.hpp"
#endif

// ============================================================================
// SII EEPROM SUPPORT (compile-time switch)
// ============================================================================
// Set via CMake option TETHER_ENABLE_SII (ON/OFF).
// When disabled, the SII manager and all SII parsing is compiled out.

#ifndef TETHER_ENABLE_SII
#define TETHER_ENABLE_SII 1
#endif
