/**
 * @file EtherCATTypes.hpp
 * @brief Common EtherCAT type definitions shared across modules
 * 
 * This header provides shared type definitions that are used by multiple
 * EtherCAT modules including the packet router, retry logic, and slave emulator.
 * 
 * The types are designed to be portable between ESP32 target and host testing.
 */

#pragma once

#include <cstdint>
#include <cstring>
#include <array>
#include <functional>

#ifdef ESP_PLATFORM
#include "esp_eth_driver.h"
#endif

namespace EtherCAT {

// ============================================================================
// Protocol Constants
// ============================================================================

/// EtherType for EtherCAT frames (big-endian on wire)
constexpr uint16_t kEtherTypeEtherCAT = 0x88A4;

/// Maximum EtherCAT datagram data size
constexpr size_t kMaxDatagramDataSize = 1486;

/// Maximum Ethernet frame size
constexpr size_t kMaxFrameSize = 1518;

/// EtherCAT destination MAC (broadcast)
constexpr std::array<uint8_t, 6> kEtherCATBroadcastMAC = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

// ============================================================================
// Common Types & Errors
// ============================================================================

using EthHandle = void*;
using ErrorCode = int;
constexpr ErrorCode EC_SUCCESS = 0;
constexpr ErrorCode EC_FAILURE = -1;

// ============================================================================
// Network Abstraction
// ============================================================================

/**
 * @brief Abstract network interface for EtherCAT communication
 */
struct NetworkInterface {
    /**
     * @brief Send a packet
     * @param data Pointer to packet data
     * @param len Length of packet
     * @return true on success
     */
    std::function<bool(const uint8_t* data, size_t len)> send;

    /**
     * @brief Receive a packet (optional/polling)
     * @param buffer Output buffer
     * @param max_len Maximum length to read
     * @param[out] out_len Actual length read
     * @return true on success
     */
    std::function<bool(uint8_t* buffer, size_t max_len, size_t* out_len)> receive;

    /**
     * @brief Optional opaque native handle owned by the platform adapter.
     */
    void* native_handle = nullptr;

#ifdef ESP_PLATFORM
    static NetworkInterface fromEspHandle(esp_eth_handle_t eth_handle) {
        NetworkInterface iface;
        iface.native_handle = eth_handle;
        return iface;
    }
#endif
};

// ============================================================================
// EtherCAT Command Types
// ============================================================================

/**
 * @brief EtherCAT datagram command types
 */
enum class Command : uint8_t {
    NOP  = 0x00,  ///< No operation
    APRD = 0x01,  ///< Auto Position Read
    APWR = 0x02,  ///< Auto Position Write
    APRW = 0x03,  ///< Auto Position Read/Write
    FPRD = 0x04,  ///< Configured Address Read
    FPWR = 0x05,  ///< Configured Address Write
    FPRW = 0x06,  ///< Configured Address Read/Write
    BRD  = 0x07,  ///< Broadcast Read
    BWR  = 0x08,  ///< Broadcast Write
    BRW  = 0x09,  ///< Broadcast Read/Write
    LRD  = 0x0A,  ///< Logical Read
    LWR  = 0x0B,  ///< Logical Write
    LRW  = 0x0C,  ///< Logical Read/Write
    ARMW = 0x0D,  ///< Auto Read Multiple Write
    FRMW = 0x0E,  ///< Configured Read Multiple Write
};

/**
 * @brief Convert command to string name
 */
inline const char* commandToString(Command cmd) {
    switch (cmd) {
        case Command::NOP:  return "NOP";
        case Command::APRD: return "APRD";
        case Command::APWR: return "APWR";
        case Command::APRW: return "APRW";
        case Command::FPRD: return "FPRD";
        case Command::FPWR: return "FPWR";
        case Command::FPRW: return "FPRW";
        case Command::BRD:  return "BRD";
        case Command::BWR:  return "BWR";
        case Command::BRW:  return "BRW";
        case Command::LRD:  return "LRD";
        case Command::LWR:  return "LWR";
        case Command::LRW:  return "LRW";
        case Command::ARMW: return "ARMW";
        case Command::FRMW: return "FRMW";
        default: return "UNK";
    }
}

/**
 * @brief Check if command is a read operation
 */
inline bool isReadCommand(Command cmd) {
    return cmd == Command::APRD || cmd == Command::FPRD || 
           cmd == Command::BRD || cmd == Command::LRD ||
           cmd == Command::APRW || cmd == Command::FPRW ||
           cmd == Command::BRW || cmd == Command::LRW;
}

/**
 * @brief Check if command is a write operation
 */
inline bool isWriteCommand(Command cmd) {
    return cmd == Command::APWR || cmd == Command::FPWR || 
           cmd == Command::BWR || cmd == Command::LWR ||
           cmd == Command::APRW || cmd == Command::FPRW ||
           cmd == Command::BRW || cmd == Command::LRW;
}

/**
 * @brief Received datagram structure
 * 
 * Passed through the RX queue when a response is received.
 */
struct RxDatagram {
    uint8_t idx;              ///< Matching index from request
    Command cmd;              ///< Command type
    uint16_t adp;             ///< Address position
    uint16_t ado;             ///< Address offset
    uint16_t datalen;         ///< Data length
    uint16_t wkc;             ///< Working Counter (incremented by each slave that processes)
    uint8_t data[256];        ///< Payload data
};

// ============================================================================
// Wire-Format Structures (Packed)
// ============================================================================

// Backwards-compatible alias used by older code/tests that reference Raw::EtherCATCommand
namespace Raw {
    using EtherCATCommand = Command;
}

/**
 * @brief Ethernet frame header
 */
struct __attribute__((packed)) EthernetHeader {
    uint8_t dst[6];         ///< Destination MAC address
    uint8_t src[6];         ///< Source MAC address
    uint16_t etherType_be;  ///< EtherType in big-endian
};
static_assert(sizeof(EthernetHeader) == 14, "EthernetHeader must be 14 bytes");

/**
 * @brief EtherCAT frame header (2 bytes after Ethernet header)
 */
struct __attribute__((packed)) FrameHeader {
    uint16_t raw_le;  ///< [0:10]=length, [11]=reserved, [12:15]=type (must be 1)
    
    uint16_t length() const { return raw_le & 0x07FF; }
    uint16_t type() const { return (raw_le >> 12) & 0x0F; }
    
    void setLength(uint16_t len) {
        raw_le = (raw_le & 0xF800) | (len & 0x07FF);
    }
    void setType(uint16_t t) {
        raw_le = (raw_le & 0x0FFF) | ((t & 0x0F) << 12);
    }
    void set(uint16_t len, uint16_t type = 1) {
        raw_le = (len & 0x07FF) | ((type & 0x0F) << 12);
    }
};
static_assert(sizeof(FrameHeader) == 2, "FrameHeader must be 2 bytes");

/**
 * @brief EtherCAT datagram header (10 bytes)
 */
struct __attribute__((packed)) DatagramHeader {
    Command cmd;        ///< Command type
    uint8_t idx;        ///< Index for request/response matching
    uint16_t adp_le;    ///< Address Position
    uint16_t ado_le;    ///< Address Offset
    uint16_t lenFlags_le; ///< [0:10]=length, [14]=C, [15]=M
    uint16_t irq_le;    ///< Interrupt request
    
    // Accessors
    uint16_t dataLength() const { return lenFlags_le & 0x07FF; }
    bool more() const { return (lenFlags_le & 0x8000) != 0; }
    bool circulating() const { return (lenFlags_le & 0x4000) != 0; }
    
    void setDataLength(uint16_t len) {
        lenFlags_le = (lenFlags_le & 0xF800) | (len & 0x07FF);
    }
    void setMore(bool m) {
        if (m) lenFlags_le |= 0x8000;
        else lenFlags_le &= ~0x8000;
    }
    void setCirculating(bool c) {
        if (c) lenFlags_le |= 0x4000;
        else lenFlags_le &= ~0x4000;
    }
    
    // Get 32-bit logical address (for LRD/LWR/LRW)
    uint32_t logicalAddress() const {
        return (static_cast<uint32_t>(ado_le) << 16) | adp_le;
    }
    void setLogicalAddress(uint32_t addr) {
        adp_le = addr & 0xFFFF;
        ado_le = (addr >> 16) & 0xFFFF;
    }
};
static_assert(sizeof(DatagramHeader) == 10, "DatagramHeader must be 10 bytes");

/**
 * @brief Complete datagram with header, data, and WKC
 * 
 * This is a variable-size structure. The data array size is a maximum.
 */
struct Datagram {
    DatagramHeader header;
    uint8_t data[kMaxDatagramDataSize];
    uint16_t wkc = 0;
    
    // Actual data size based on header
    size_t totalSize() const {
        return sizeof(DatagramHeader) + header.dataLength() + sizeof(uint16_t);
    }
    
    // Copy data into the datagram
    void setData(const void* src, size_t len) {
        if (len > kMaxDatagramDataSize) len = kMaxDatagramDataSize;
        std::memcpy(data, src, len);
        header.setDataLength(static_cast<uint16_t>(len));
    }
    
    // Get pointer to WKC (after data)
    uint16_t* wkcPtr() {
        return reinterpret_cast<uint16_t*>(data + header.dataLength());
    }
    const uint16_t* wkcPtr() const {
        return reinterpret_cast<const uint16_t*>(data + header.dataLength());
    }
    
    // Read WKC from wire position
    uint16_t readWkc() const {
        uint16_t w;
        std::memcpy(&w, data + header.dataLength(), sizeof(w));
        return w;
    }
    
    // Write WKC to wire position
    void writeWkc(uint16_t w) {
        std::memcpy(data + header.dataLength(), &w, sizeof(w));
        wkc = w;
    }
};

/**
 * @brief Lightweight datagram reference for parsing received frames
 * 
 * Points into a frame buffer without copying data.
 */
struct DatagramView {
    Command cmd;
    uint8_t idx;
    uint16_t adp;
    uint16_t ado;
    uint16_t dataLength;
    const uint8_t* data;
    uint16_t wkc;
    bool more;
    
    // Get 32-bit logical address
    uint32_t logicalAddress() const {
        return (static_cast<uint32_t>(ado) << 16) | adp;
    }
    
    // Parse from wire format
    static DatagramView parse(const uint8_t* ptr, size_t available) {
        DatagramView v{};
        if (available < sizeof(DatagramHeader)) {
            return v;
        }
        
        // Use memcpy instead of reinterpret_cast for safety
        DatagramHeader hdr;
        std::memcpy(&hdr, ptr, sizeof(hdr));
        v.cmd = hdr.cmd;
        v.idx = hdr.idx;
        v.adp = hdr.adp_le;
        v.ado = hdr.ado_le;
        v.dataLength = hdr.dataLength();
        v.more = hdr.more();
        
        if (available < sizeof(DatagramHeader) + v.dataLength + 2) {
            v.dataLength = 0;
            return v;
        }
        
        v.data = ptr + sizeof(DatagramHeader);
        std::memcpy(&v.wkc, v.data + v.dataLength, sizeof(v.wkc));
        
        return v;
    }
    
    // Total size in bytes
    size_t totalSize() const {
        return sizeof(DatagramHeader) + dataLength + sizeof(uint16_t);
    }
};

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

// ============================================================================
// AL Status Codes
// ============================================================================

namespace alcode {

constexpr uint16_t NO_ERROR               = 0x0000;
constexpr uint16_t UNSPECIFIED_ERROR      = 0x0001;
constexpr uint16_t NO_MEMORY              = 0x0002;
constexpr uint16_t INVALID_REQ_STATE_CHG  = 0x0011;
constexpr uint16_t UNKNOWN_REQ_STATE      = 0x0012;
constexpr uint16_t BOOTSTRAP_NOT_SUPP     = 0x0013;
constexpr uint16_t NO_VALID_FIRMWARE      = 0x0014;
constexpr uint16_t INVALID_MAILBOX_CFG1   = 0x0016;
constexpr uint16_t INVALID_MAILBOX_CFG2   = 0x0017;
constexpr uint16_t INVALID_SM_CFG         = 0x0018;
constexpr uint16_t NO_VALID_INPUTS        = 0x0019;
constexpr uint16_t NO_VALID_OUTPUTS       = 0x001A;
constexpr uint16_t SYNC_ERROR             = 0x001B;
constexpr uint16_t SM_WATCHDOG            = 0x001C;
constexpr uint16_t INVALID_SM_TYPES       = 0x001D;
constexpr uint16_t INVALID_OUTPUT_CFG     = 0x001E;
constexpr uint16_t INVALID_INPUT_CFG      = 0x001F;
constexpr uint16_t INVALID_WATCHDOG_CFG   = 0x0020;
constexpr uint16_t SLAVE_NEEDS_COLD_START = 0x0021;
constexpr uint16_t SLAVE_NEEDS_INIT       = 0x0022;
constexpr uint16_t SLAVE_NEEDS_PREOP      = 0x0023;
constexpr uint16_t SLAVE_NEEDS_SAFEOP     = 0x0024;
constexpr uint16_t INVALID_INPUT_MAP      = 0x0025;
constexpr uint16_t INVALID_OUTPUT_MAP     = 0x0026;
constexpr uint16_t INCONSISTENT_SETTINGS  = 0x0027;
constexpr uint16_t FREERUN_NOT_SUPPORTED  = 0x0028;
constexpr uint16_t SYNC_NOT_SUPPORTED     = 0x0029;
constexpr uint16_t FREERUN_3BUF_NEEDED    = 0x002A;
constexpr uint16_t BG_WATCHDOG            = 0x002B;
constexpr uint16_t NO_VALID_INPUTS_OUTPUTS= 0x002C;
constexpr uint16_t FATAL_SYNC_ERROR       = 0x002D;
constexpr uint16_t NO_SYNC_ERROR          = 0x002E;  ///< "Err74.1" - No sync
constexpr uint16_t INVALID_DC_SYNC_CFG    = 0x0030;
constexpr uint16_t INVALID_DC_LATCH_CFG   = 0x0031;
constexpr uint16_t PLL_ERROR              = 0x0032;
constexpr uint16_t DC_SYNC_IO_ERROR       = 0x0033;
constexpr uint16_t DC_SYNC_TIMEOUT        = 0x0034;
constexpr uint16_t DC_INVALID_SYNC_CYCLE  = 0x0035;
constexpr uint16_t DC_SYNC0_CYCLE_ERROR   = 0x0036;
constexpr uint16_t DC_SYNC1_CYCLE_ERROR   = 0x0037;
constexpr uint16_t MBX_AOE_ERROR          = 0x0041;
constexpr uint16_t MBX_EOE_ERROR          = 0x0042;
constexpr uint16_t MBX_COE_ERROR          = 0x0043;
constexpr uint16_t MBX_FOE_ERROR          = 0x0044;
constexpr uint16_t MBX_SOE_ERROR          = 0x0045;
constexpr uint16_t MBX_VOE_ERROR          = 0x004F;
constexpr uint16_t EEPROM_NO_ACCESS       = 0x0050;
constexpr uint16_t EEPROM_ERROR           = 0x0051;
constexpr uint16_t SLAVE_RESTARTED        = 0x0060;
constexpr uint16_t DEVICE_ID_UPDATE_ERR   = 0x0061;
constexpr uint16_t APPLICATION_CTRL_ERR   = 0x00F0;

/// Get string description for AL status code
inline const char* alStatusCodeToString(uint16_t code) {
    switch (code) {
        case NO_ERROR: return "No error";
        case UNSPECIFIED_ERROR: return "Unspecified error";
        case NO_MEMORY: return "No memory";
        case INVALID_REQ_STATE_CHG: return "Invalid requested state change";
        case UNKNOWN_REQ_STATE: return "Unknown requested state";
        case BOOTSTRAP_NOT_SUPP: return "Bootstrap not supported";
        case NO_VALID_FIRMWARE: return "No valid firmware";
        case INVALID_MAILBOX_CFG1: return "Invalid mailbox configuration (SM0)";
        case INVALID_MAILBOX_CFG2: return "Invalid mailbox configuration (SM1)";
        case INVALID_SM_CFG: return "Invalid sync manager configuration";
        case NO_VALID_INPUTS: return "No valid inputs available";
        case NO_VALID_OUTPUTS: return "No valid outputs available";
        case SYNC_ERROR: return "Synchronization error";
        case SM_WATCHDOG: return "Sync manager watchdog";
        case INVALID_SM_TYPES: return "Invalid sync manager types";
        case INVALID_OUTPUT_CFG: return "Invalid output configuration";
        case INVALID_INPUT_CFG: return "Invalid input configuration";
        case INVALID_WATCHDOG_CFG: return "Invalid watchdog configuration";
        case SLAVE_NEEDS_COLD_START: return "Slave needs cold start";
        case SLAVE_NEEDS_INIT: return "Slave needs INIT state";
        case SLAVE_NEEDS_PREOP: return "Slave needs PRE-OP state";
        case SLAVE_NEEDS_SAFEOP: return "Slave needs SAFE-OP state";
        case INVALID_INPUT_MAP: return "Invalid input mapping";
        case INVALID_OUTPUT_MAP: return "Invalid output mapping";
        case INCONSISTENT_SETTINGS: return "Inconsistent settings";
        case FREERUN_NOT_SUPPORTED: return "Freerun not supported";
        case SYNC_NOT_SUPPORTED: return "Sync not supported";
        case FREERUN_3BUF_NEEDED: return "Freerun needs 3 buffers";
        case BG_WATCHDOG: return "Background watchdog";
        case NO_VALID_INPUTS_OUTPUTS: return "No valid inputs and outputs";
        case FATAL_SYNC_ERROR: return "Fatal sync error";
        case NO_SYNC_ERROR: return "No sync error (Err74.1)";
        case INVALID_DC_SYNC_CFG: return "Invalid DC sync configuration";
        case INVALID_DC_LATCH_CFG: return "Invalid DC latch configuration";
        case PLL_ERROR: return "PLL error";
        case DC_SYNC_IO_ERROR: return "DC sync I/O error";
        case DC_SYNC_TIMEOUT: return "DC sync timeout";
        case DC_INVALID_SYNC_CYCLE: return "Invalid DC sync cycle time";
        case DC_SYNC0_CYCLE_ERROR: return "DC SYNC0 cycle error";
        case DC_SYNC1_CYCLE_ERROR: return "DC SYNC1 cycle error";
        case MBX_AOE_ERROR: return "Mailbox AoE error";
        case MBX_EOE_ERROR: return "Mailbox EoE error";
        case MBX_COE_ERROR: return "Mailbox CoE error";
        case MBX_FOE_ERROR: return "Mailbox FoE error";
        case MBX_SOE_ERROR: return "Mailbox SoE error";
        case MBX_VOE_ERROR: return "Mailbox VoE error";
        case EEPROM_NO_ACCESS: return "EEPROM no access";
        case EEPROM_ERROR: return "EEPROM error";
        case SLAVE_RESTARTED: return "Slave restarted locally";
        case DEVICE_ID_UPDATE_ERR: return "Device ID update error";
        case APPLICATION_CTRL_ERR: return "Application controller error";
        default: return "Unknown error";
    }
}

} // namespace alcode

// ============================================================================
// Slave State
// ============================================================================

enum class SlaveState : uint8_t {
    INIT    = 0x01,
    PRE_OP  = 0x02,
    BOOT    = 0x03,
    SAFE_OP = 0x04,
    OP      = 0x08
};

inline const char* slaveStateToString(SlaveState state) {
    switch (state) {
        case SlaveState::INIT:    return "INIT";
        case SlaveState::PRE_OP:  return "PRE-OP";
        case SlaveState::BOOT:    return "BOOT";
        case SlaveState::SAFE_OP: return "SAFE-OP";
        case SlaveState::OP:      return "OP";
        default: return "UNKNOWN";
    }
}

} // namespace EtherCAT

