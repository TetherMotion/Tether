// SPDX-License-Identifier: MIT
#pragma once

#include <cstdint>

namespace EtherCAT { namespace ESCReg {

// ============================================================================
// Information Registers (0x0000-0x000F)
// ============================================================================
constexpr uint16_t Type                     = 0x0000;
constexpr uint16_t Revision                 = 0x0001;
constexpr uint16_t Build                    = 0x0002;
constexpr uint16_t FMMUCount                = 0x0004;
constexpr uint16_t SyncManagerCount         = 0x0005;
constexpr uint16_t RAMSize                  = 0x0006;
constexpr uint16_t PortDescriptor           = 0x0007;
constexpr uint16_t Features                 = 0x0008;

// ============================================================================
// Configured Station Address (0x0010-0x0013)
// ============================================================================
constexpr uint16_t ConfiguredStationAddress = 0x0010;
constexpr uint16_t ConfiguredStationAlias   = 0x0012;

// ============================================================================
// Data Link (DL) Registers (0x0100-0x011F)
// ============================================================================
constexpr uint16_t DLControl                = 0x0100;
constexpr uint16_t DLStatus                 = 0x0110;

// ============================================================================
// Application Layer (AL) Registers (0x0120-0x013F)
// ============================================================================
constexpr uint16_t ALControl                = 0x0120;
constexpr uint16_t ALStatus                 = 0x0130;
constexpr uint16_t ALStatusCode             = 0x0134;

// ============================================================================
// PDI Registers (0x0140-0x015F)
// ============================================================================
constexpr uint16_t PDIControl               = 0x0140;
constexpr uint16_t PDIConfig                = 0x0150;

// ============================================================================
// Watchdog Registers (0x0400-0x0443)
// ============================================================================
constexpr uint16_t WatchdogDivider          = 0x0400;
constexpr uint16_t PDIWatchdog              = 0x0410;
constexpr uint16_t SyncManagerWatchdog      = 0x0420;
constexpr uint16_t WatchdogStatus           = 0x0440;
constexpr uint16_t WatchdogCounter          = 0x0442;
constexpr uint16_t PDIWatchdogCounter       = 0x0443;

// ============================================================================
// SII / EEPROM Registers (0x0500-0x050F)
// ============================================================================
constexpr uint16_t SIIConfig                = 0x0500;
constexpr uint16_t SIIControl               = 0x0502;
constexpr uint16_t SIIAddress               = 0x0504;
constexpr uint16_t SIIData                  = 0x0508;

// ============================================================================
// FMMU Registers (0x0600+, 16 bytes each)
// ============================================================================
constexpr uint16_t FMMU0                    = 0x0600;
constexpr uint16_t FMMUSize                 = 16;

// ============================================================================
// Sync Manager Registers (0x0800+, 8 bytes each)
// ============================================================================
constexpr uint16_t SM0                      = 0x0800;
constexpr uint16_t SM1                      = 0x0808;
constexpr uint16_t SMSize                   = 8;

// ============================================================================
// Distributed Clock (DC) Time Registers (0x0900-0x0935)
// ============================================================================
constexpr uint16_t DCReceiveTime            = 0x0900;
constexpr uint16_t DCReceiveTimePort1       = 0x0918;
constexpr uint16_t DCSystemTime             = 0x0910;
constexpr uint16_t DCSystemTimeOffset       = 0x0920;
constexpr uint16_t DCSystemTimeDelay        = 0x0928;
constexpr uint16_t DCSystemTimeDifference   = 0x092C;
constexpr uint16_t DCSpeedCounterStart      = 0x0930;
constexpr uint16_t DCSpeedCounterDiff       = 0x0932;
constexpr uint16_t DCSystemTimeDiffFilter   = 0x0934;
constexpr uint16_t DCControlLoop            = 0x0935;

// ============================================================================
// DC SYNC Registers (0x0980-0x09A4)
// ============================================================================
constexpr uint16_t DCSyncActivation         = 0x0980;
constexpr uint16_t DCSyncStartTime          = 0x0990;
constexpr uint16_t DCSync0CycleTime         = 0x09A0;
constexpr uint16_t DCSync1CycleTime         = 0x09A4;

// ============================================================================
// DC Latch Registers (0x09A8-0x09CF)
// ============================================================================
constexpr uint16_t DCLatchControl           = 0x09A8;
constexpr uint16_t DCLatchStatus            = 0x09AE;
constexpr uint16_t DCLatch0Time             = 0x09B0;
constexpr uint16_t DCLatch0TimeNeg          = 0x09B8;
constexpr uint16_t DCLatch1Time             = 0x09C0;
constexpr uint16_t DCLatch1TimeNeg          = 0x09C8;

// ============================================================================
// Process Data RAM (0x1000+)
// ============================================================================
constexpr uint16_t ProcessDataRAM           = 0x1000;

}} // namespace EtherCAT::ESCReg
