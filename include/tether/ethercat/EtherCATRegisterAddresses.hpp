/**
 * @file EtherCATRegisterAddresses.hpp
 * @brief EtherCAT types: EtherCAT Register Addresses
 *
 * Split out of Types.hpp.
 */

#pragma once

#include <array>
#include <cstdint>
#include <cstring>
#include <functional>

namespace EtherCAT {

// ============================================================================
// EtherCAT Register Addresses
// ============================================================================

namespace reg {

// Identification registers
constexpr uint16_t TYPE           = 0x0000;  ///< Type register (8 bytes)
constexpr uint16_t REVISION       = 0x0001;  ///< Revision register
constexpr uint16_t BUILD          = 0x0002;  ///< Build register (2 bytes)
constexpr uint16_t FMMU_COUNT     = 0x0004;  ///< Number of FMMUs
constexpr uint16_t SM_COUNT       = 0x0005;  ///< Number of Sync Managers
constexpr uint16_t RAM_SIZE       = 0x0006;  ///< RAM size (KB)
constexpr uint16_t PORT_DESC      = 0x0007;  ///< Port descriptor

// DL Control registers
constexpr uint16_t DL_CONTROL     = 0x0100;  ///< DL Control (4 bytes)
constexpr uint16_t DL_STATUS      = 0x0110;  ///< DL Status (2 bytes)

// Application Layer registers
constexpr uint16_t AL_CONTROL     = 0x0120;  ///< AL Control (2 bytes)
constexpr uint16_t AL_STATUS      = 0x0130;  ///< AL Status (2 bytes)
constexpr uint16_t AL_STATUS_CODE = 0x0134;  ///< AL Status Code (2 bytes)

// Watchdog registers
constexpr uint16_t WD_DIV         = 0x0400;  ///< Watchdog Divider
constexpr uint16_t WD_TIME_PDI    = 0x0410;  ///< Watchdog Time PDI
constexpr uint16_t WD_TIME_PDATA  = 0x0420;  ///< Watchdog Time Process Data
constexpr uint16_t WD_STATUS      = 0x0440;  ///< Watchdog Status
constexpr uint16_t WD_CNT_PDI     = 0x0442;  ///< Watchdog Counter PDI
constexpr uint16_t WD_CNT_PDATA   = 0x0443;  ///< Watchdog Counter Process Data

// SII/EEPROM registers
constexpr uint16_t SII_CONTROL    = 0x0502;  ///< SII Control
constexpr uint16_t SII_ADDRESS    = 0x0504;  ///< SII Address
constexpr uint16_t SII_DATA       = 0x0508;  ///< SII Data (8 bytes)

// FMMU registers (0x0600 + n*16)
constexpr uint16_t FMMU_BASE      = 0x0600;  ///< FMMU 0 base address
constexpr uint16_t FMMU_SIZE      = 16;      ///< Size of each FMMU config

// Sync Manager registers (0x0800 + n*8)
constexpr uint16_t SM_BASE        = 0x0800;  ///< SM 0 base address
constexpr uint16_t SM_SIZE        = 8;       ///< Size of each SM config

// DC registers
constexpr uint16_t DC_RCV_TIME_PORT0 = 0x0900;  ///< Receive time port 0
constexpr uint16_t DC_RCV_TIME_PORT1 = 0x0904;  ///< Receive time port 1
constexpr uint16_t DC_SYS_TIME       = 0x0910;  ///< System Time (8 bytes)
constexpr uint16_t DC_SYS_TIME_OFF   = 0x0920;  ///< System Time Offset (8 bytes)
constexpr uint16_t DC_SYS_TIME_DELAY = 0x0928;  ///< System Time Delay (4 bytes)
constexpr uint16_t DC_SYS_TIME_DIFF  = 0x092C;  ///< System Time Difference (4 bytes)
constexpr uint16_t DC_SPEED_CNT_START = 0x0930; ///< Speed Counter Start
constexpr uint16_t DC_SPEED_CNT_DIFF  = 0x0932; ///< Speed Counter Diff
constexpr uint16_t DC_FILTER_DEPTH   = 0x0934;  ///< Filter Depth
constexpr uint16_t DC_SYNC_ACTIVATION = 0x0981; ///< DC Sync Activation
constexpr uint16_t DC_SYNC0_CYCLE    = 0x09A0;  ///< SYNC0 cycle time
constexpr uint16_t DC_SYNC1_CYCLE    = 0x09A4;  ///< SYNC1 cycle time

} // namespace reg

} // namespace EtherCAT
