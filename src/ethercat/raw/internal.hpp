/**
 * @file internal.hpp
 * @brief Internal EtherCAT protocol definitions (umbrella header)
 *
 * This header has been split into focused, modular headers:
 * - RawWireFormat.hpp    — Packed wire-format structures (EthernetHeader, datagrams, mailbox/SDO structs)
 * - RawConstants.hpp     — Register address enums, SII/EEPROM constants, mailbox/SDO constants
 * - RawTransportDecls.hpp — Function declarations for transport, SII, SDO
 *
 * This umbrella header preserves backward compatibility.
 *
 * @internal
 * This is an internal header used by the EtherCAT raw module implementation.
 * Application code should use the public headers (Raw.hpp, PDOManager.hpp, etc.)
 */

#pragma once

#include "tether/platform/Platform.hpp"
#include "tether/ethercat/Types.hpp"
#include "tether/ethercat/SMRegisters.hpp"
#include <bit>
#include <cinttypes>
#include <cstring>
#include <memory>

#include "RawWireFormat.hpp"
#include "RawConstants.hpp"
#include "RawTransportDecls.hpp"
