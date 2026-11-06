/**
 * @file Constants.hpp
 * @brief Wire constants from the Modbus specification (plan §3/ANNEX O).
 */
#pragma once

#include <chrono>
#include <cstdint>

namespace tether::modbus {

// ---- wire limits ----------------------------------------------------------
inline constexpr uint16_t kMaxPduSize      = 253;
inline constexpr uint16_t kMaxRtuAduSize   = 256;   // unit(1)+pdu(253)+crc(2)
inline constexpr uint16_t kMbapHeaderSize  = 7;     // txn2+proto2+len2+unit1
inline constexpr uint16_t kMaxMbapAduSize  = kMbapHeaderSize + kMaxPduSize;

// ---- addressing ------------------------------------------------------------
inline constexpr uint8_t  kBroadcastUnit   = 0;
inline constexpr uint8_t  kMinUnit         = 1;
inline constexpr uint8_t  kMaxNormalUnit   = 247;
inline constexpr uint8_t  kMaxUnit         = 255;

// ---- function-code ceilings (spec table 3.3) -------------------------------
inline constexpr uint16_t kMaxReadBitsQty   = 2000; // FC1/FC2
inline constexpr uint16_t kMaxReadRegsQty   = 125;  // FC3/FC4
inline constexpr uint16_t kMaxWriteBitsQty  = 1968; // FC15
inline constexpr uint16_t kMaxWriteRegsQty  = 123;  // FC16
inline constexpr uint16_t kMaxRwReadQty     = 125;  // FC23 read part
inline constexpr uint16_t kMaxRwWriteQty    = 121;  // FC23 write part
inline constexpr uint16_t kMaxFifoQty       = 31;   // FC24
inline constexpr uint32_t kAddressSpace     = 65536;

// ---- CRC-16/MODBUS ----------------------------------------------------------
inline constexpr uint16_t kCrcPolynomial   = 0xA001; // reflected 0x8005
inline constexpr uint16_t kCrcInitial      = 0xFFFF;

// ---- RTU timing (Modbus over Serial Line §2.5) -----------------------------
// Above 19200 baud both gaps use FIXED recommended values.
inline constexpr std::chrono::microseconds kRtuFixedT15{750};
inline constexpr std::chrono::microseconds kRtuFixedT35{1750};
inline constexpr uint32_t kRtuFixedTimingBaud = 19200;

// ---- defaults ---------------------------------------------------------------
inline constexpr uint32_t kDefaultResponseTimeoutMs = 1000;
inline constexpr uint16_t kDefaultTcpPort           = 502;
inline constexpr uint32_t kDefaultConnectTimeoutMs  = 5000;
inline constexpr uint32_t kDefaultKeepaliveMs       = 30000;
inline constexpr uint16_t kDefaultQueueDepth        = 256;
inline constexpr uint8_t  kDefaultTcpInflight       = 8;
inline constexpr uint32_t kDefaultBroadcastDelayMs  = 100;

} // namespace tether::modbus
