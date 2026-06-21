/**
 * @file internal.hpp
 * @brief Internal EtherCAT protocol definitions (umbrella header)
 *
 * This header has been split into focused, modular headers:
 * - RawWireFormat.hpp    — Packed wire-format structures
 * - RawConstants.hpp     — Register address enums, SII/EEPROM constants
 * - RawTransportDecls.hpp — Function declarations for transport, SII, SDO
 *
 * This umbrella header preserves backward compatibility.
 *
 * @internal
 * This is an internal header used by the EtherCAT raw module implementation.
 * Application code should use the public headers (Raw.hpp, PDOManager.hpp, etc.)
 *
 * @details
 * This header contains:
 * - Wire-format structures for EtherCAT frames and datagrams (packed)
 * - Protocol constants and register addresses
 * - Low-level transport functions (send/receive datagrams)
 * - CoE (CANopen over EtherCAT) mailbox structures
 * 
 * ## EtherCAT Frame Structure
 * 
 * An EtherCAT frame consists of:
 * ```
 * ┌────────────────┬─────────────────┬────────────────┬─────┬────────────────┐
 * │ Ethernet Hdr   │ EtherCAT Header │  Datagram 1    │ ... │  Datagram N    │
 * │ (14 bytes)     │ (2 bytes)       │ (10+data+2)    │     │ (10+data+2)    │
 * └────────────────┴─────────────────┴────────────────┴─────┴────────────────┘
 *          │               │                 │
 *          │               │                 └── EtherCATDatagramHeader
 *          │               │                     + payload + WKC
 *          │               └── Length[11] + Reserved[1] + Type[4]
 *          └── Dst MAC + Src MAC + EtherType (0x88A4)
 * ```
 * 
 * ## Datagram Commands
 * 
 * | Cmd  | Name | Description |
 * |------|------|-------------|
 * | 0x01 | APRD | Auto-increment Position Read |
 * | 0x02 | APWR | Auto-increment Position Write |
 * | 0x04 | FPRD | Configured Address Read |
 * | 0x05 | FPWR | Configured Address Write |
 * | 0x07 | BRD  | Broadcast Read |
 * | 0x08 | BWR  | Broadcast Write |
 * | 0x0C | LRW  | Logical Read/Write |
 * 
 * ## Important Notes
 * 
 * - All multi-byte fields in packed structures are little-endian (marked with _le suffix)
 * - Big-endian fields (Ethernet header) are marked with _be suffix
 * - Use static_assert to verify structure sizes match wire format
 * - Never use C++ bitfields for protocol structures (implementation-defined ordering)
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

// Forward-declare Master for function parameters
namespace EtherCAT { class Master; }
