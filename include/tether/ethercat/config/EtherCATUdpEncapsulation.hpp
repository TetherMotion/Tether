/**
 * @file EtherCATUdpEncapsulation.hpp
 * @brief EtherCAT config: UDP ENCAPSULATION (compile-time switch)
 *
 * Split out of EtherCATConfig.hpp.
 */

#pragma once

// Pull in the generated top-level feature flags (TETHER_ENABLE_*).
#if __has_include("tether/TetherConfig.hpp")
#include "tether/TetherConfig.hpp"
#endif

// ============================================================================
// UDP ENCAPSULATION (compile-time switch)
// ============================================================================
// Set to 1 to enable EtherCAT-over-UDP encapsulation (Ethernet/IPv4/UDP port
// 34980).  When disabled (default), all UDP encapsulation code is compiled
// out and the master uses direct EtherCAT Ethernet frames (EtherType 0x88A4)
// with zero overhead.  Enable only if your slaves require UDP tunneling.
//
// Can be overridden via CMake: -DTETHER_ENABLE_UDP_ENCAPSULATION=ON

#ifndef TETHER_ENABLE_UDP_ENCAPSULATION
#define TETHER_ENABLE_UDP_ENCAPSULATION 0
#endif
