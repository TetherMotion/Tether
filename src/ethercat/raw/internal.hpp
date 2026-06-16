/**
 * @file raw_internal.hpp
 * @brief Internal EtherCAT protocol definitions and low-level transport primitives
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

#include <cinttypes>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <memory> 

#include "tether/platform/Platform.hpp"
#include "tether/ethercat/Types.hpp"

// Forward-declare Master for function parameters
namespace EtherCAT { class Master; }

namespace EtherCAT {
namespace Raw {

// Use types from public header
using EtherCATCommand = EtherCAT::Command;
using RxDatagram = EtherCAT::RxDatagram;

// ============================================================================
// Constants
// ============================================================================

/**
 * @brief Reserved index for fire-and-forget datagrams
 * 
 * Datagrams sent with this index are not expected to have their responses
 * processed. The RX handler will discard responses with this index to
 * prevent queue pollution from fire-and-forget operations like RxPDO writes.
 */
constexpr uint8_t FIRE_AND_FORGET_IDX = 0xFE;

// ============================================================================
// Log Deduplication
// ============================================================================

/**
 * @brief Log message with key-based deduplication
 * @param level Log level: 0=INFO, 1=WARN, 2=ERROR, 3=DEBUG
 * @param key Unique key for deduplication
 * @param msg Message to log
 * 
 * Prevents flooding the log with repeated identical messages.
 */
void log_dedup_key(int level, const char *key, const char *msg);

/**
 * @brief Log message with message-based deduplication
 * @param level Log level: 0=INFO, 1=WARN, 2=ERROR, 3=DEBUG
 * @param msg Message to log (also used as deduplication key)
 */
void log_dedup(int level, const char *msg);

// ============================================================================
// Byte Order Utilities
// ============================================================================

/**
 * @brief Swap bytes of a 16-bit value
 * @param v Input value
 * @return Byte-swapped value
 */
static inline uint16_t bswap16(uint16_t v)
{
    return __builtin_bswap16(v);
}

/**
 * @brief Convert host-order to big-endian (network byte order for Ethernet header)
 * @param host Host-order value
 * @return Big-endian value
 * 
 * ESP32 is little-endian; Ethernet header fields (EtherType) are big-endian.
 */
static inline uint16_t be16(uint16_t host)
{
    // ESP32 is little-endian; Ethernet header fields are big-endian.
    return bswap16(host);
}

// Backwards-compatible alias expected by some modules
static inline uint16_t host_to_be16(uint16_t host) { return be16(host); }

/**
 * @brief Convert little-endian to host order (no-op on ESP32)
 * @param le Little-endian value
 * @return Host-order value
 * 
 * EtherCAT protocol uses little-endian for all multi-byte fields.
 */
static inline uint16_t le16_to_host(uint16_t le)
{
    // ESP32 is little-endian.
    return le;
}

/**
 * @brief Convert host order to little-endian (no-op on ESP32)
 */
static inline uint16_t host_to_le16(uint16_t host)
{
    return host;
}

/**
 * @brief Convert 32-bit little-endian to host order
 */
static inline uint32_t le32_to_host(uint32_t le)
{
    return le;
}

/**
 * @brief Convert host order to 32-bit little-endian
 */
static inline uint32_t host_to_le32(uint32_t host)
{
    return host;
}

// ============================================================================
// Protocol Constants
// ============================================================================

/**
 * @brief EtherType values for frame identification
 */
enum class EtherType : uint16_t {
    EtherCAT = 0x88A4,  ///< EtherCAT frame identification
};

// EtherCATCommand is defined in Types.hpp and aliased above

// ============================================================================
// Wire-Format Structures (Packed)
// ============================================================================

/**
 * @brief Standard Ethernet frame header
 * 
 * All EtherCAT frames start with this 14-byte Ethernet header.
 * The EtherType field is 0x88A4 for EtherCAT.
 */
struct __attribute__((packed)) EthernetHeader {
    uint8_t dst[6];         ///< Destination MAC address
    uint8_t src[6];         ///< Source MAC address  
    uint16_t etherType_be;  ///< EtherType in big-endian (0x88A4 for EtherCAT)
};
static_assert(sizeof(EthernetHeader) == 14, "EthernetHeader must be 14 bytes");

/**
 * @brief EtherCAT frame header bitfield structure
 * 
 * Follows the Ethernet header; describes the EtherCAT payload.
 */
struct __attribute__((packed)) EtherCATFrameHeaderBits {
    uint16_t length : 11;   ///< Total payload length in bytes (all datagrams)
    uint16_t reserved : 1;  ///< Reserved, must be 0
    uint16_t type : 4;      ///< Protocol type, must be 1 for EtherCAT PDU
};
static_assert(sizeof(EtherCATFrameHeaderBits) == 2, "EtherCATFrameHeaderBits must be 2 bytes");

/**
 * @brief EtherCAT frame header union for raw access
 */
union __attribute__((packed)) EtherCATFrameHeader {
    EtherCATFrameHeaderBits bits;  ///< Structured access
    uint16_t raw_le;               ///< Raw little-endian access
};
static_assert(sizeof(EtherCATFrameHeader) == 2, "EtherCATFrameHeader must be 2 bytes");

/**
 * @brief Datagram length and flags bitfield
 */
struct __attribute__((packed)) EtherCATDatagramLenFlagsBits {
    uint16_t length : 11;   ///< Datagram data length in bytes
    uint16_t reserved : 3;  ///< Reserved bits
    uint16_t c : 1;         ///< Circulating frame flag
    uint16_t m : 1;         ///< More datagrams follow flag
};
static_assert(sizeof(EtherCATDatagramLenFlagsBits) == 2, "EtherCATDatagramLenFlagsBits must be 2 bytes");

/**
 * @brief Datagram length and flags union
 */
union __attribute__((packed)) EtherCATDatagramLenFlags {
    EtherCATDatagramLenFlagsBits bits;  ///< Structured access
    uint16_t raw_le;                     ///< Raw little-endian access
};
static_assert(sizeof(EtherCATDatagramLenFlags) == 2, "EtherCATDatagramLenFlags must be 2 bytes");

/**
 * @brief EtherCAT datagram header (10 bytes)
 * 
 * Each datagram in an EtherCAT frame starts with this header,
 * followed by the data payload and a 2-byte Working Counter (WKC).
 * 
 * @note The meaning of ADP/ADO depends on the command:
 * - APxx: ADP = auto-increment address (negative offset), ADO = register offset
 * - FPxx: ADP = configured address, ADO = register offset
 * - Bxx:  ADP = ignored (0), ADO = register offset
 * - Lxx:  ADP:ADO combined = 32-bit logical address
 */
struct __attribute__((packed)) EtherCATDatagramHeader {
    EtherCATCommand cmd;            ///< Command type
    uint8_t idx;                    ///< Index for request/response matching
    uint16_t adp_le;                ///< Address Position (slave/position dependent)
    uint16_t ado_le;                ///< Address Offset (typically register address)
    EtherCATDatagramLenFlags lenFlags;  ///< Length and flags
    uint16_t irq_le;                ///< Interrupt request (usually 0)
};
static_assert(sizeof(EtherCATDatagramHeader) == 10, "EtherCATDatagramHeader must be 10 bytes");

/**
 * @brief Combined frame header for single-datagram frames
 * 
 * Convenience structure for the common case of sending a single datagram.
 */
struct __attribute__((packed)) EtherCATSingleDgramFrameHeader {
    EthernetHeader eth;         ///< Ethernet header
    EtherCATFrameHeader ec;     ///< EtherCAT frame header
    EtherCATDatagramHeader dg;  ///< Datagram header
};

// ============================================================================
// EtherCAT Register Addresses
// ============================================================================

/**
 * @brief EtherCAT slave register addresses
 * 
 * These are the standard ESC (EtherCAT Slave Controller) register addresses
 * used for slave configuration and status.
 */
enum : uint16_t {
    EC_REG_AL_CONTROL = 0x0120,     ///< Application Layer Control (write to change state)
    EC_REG_AL_STATUS = 0x0130,      ///< Application Layer Status (current state)
    EC_REG_AL_STATUS_CODE = 0x0134, ///< AL Status Code (error details)
    
    // Watchdog registers (critical for maintaining OP state)
    EC_REG_WD_DIV = 0x0400,         ///< Watchdog Divider (2 bytes, default 0x09C2 = 2498 = 100us base)
    EC_REG_WD_TIME_PDI = 0x0410,    ///< Watchdog Time PDI (2 bytes, x 100us units)
    EC_REG_WD_TIME_PDATA = 0x0420,  ///< Watchdog Time Process Data (2 bytes, x 100us units)
    EC_REG_WD_STATUS = 0x0440,      ///< Watchdog Status PDI (1 byte)
    EC_REG_WD_CNT_PDI = 0x0442,     ///< Watchdog Counter PDI (1 byte)
    EC_REG_WD_CNT_PDATA = 0x0443,   ///< Watchdog Counter Process Data (1 byte)
    
    EC_REG_EEPCTL = 0x0502,         ///< EEPROM Control/Status
    EC_REG_EEPSTAT = 0x0502,        ///< EEPROM Status (same register)
    EC_REG_EEPDAT = 0x0508,         ///< EEPROM Data
    EC_REG_SM0 = 0x0800,            ///< Sync Manager 0 configuration
    EC_REG_SM1 = 0x0808,            ///< Sync Manager 1 configuration
    EC_REG_SM0STAT = EC_REG_SM0 + 0x05,  ///< SM0 Status byte
    EC_REG_SM1STAT = EC_REG_SM1 + 0x05,  ///< SM1 Status byte
};

/**
 * @brief SII (Slave Information Interface) constants
 * 
 * Used for reading configuration from slave EEPROM.
 */
enum : uint16_t {
    ECT_SII_START = 0x0040,       ///< Start word address of SII categories
    ECT_SII_CAT_STRING = 10,      ///< String category type
};

/**
 * @brief EEPROM command values
 */
enum : uint16_t {
    EC_ECMD_NOP = 0x0000,   ///< No operation
    EC_ECMD_READ = 0x0100,  ///< Read command
};

/**
 * @brief EEPROM status flags
 */
enum : uint16_t {
    EC_ESTAT_R64 = 0x0040,    ///< 64-bit read capability
    EC_ESTAT_NACK = 0x2000,   ///< Negative acknowledge (error)
    EC_ESTAT_EMASK = 0x7800,  ///< Error mask
    EC_ESTAT_BUSY = 0x8000,   ///< EEPROM busy flag
};

/**
 * @brief SII word addresses for mailbox configuration
 */
enum : uint16_t {
    ECT_SII_RXMBXADR = 0x0018,  ///< RX mailbox address (master→slave)
    ECT_SII_TXMBXADR = 0x001a,  ///< TX mailbox address (slave→master)
    ECT_SII_MBXPROTO = 0x001c,  ///< Supported mailbox protocols
};

/**
 * @brief Default Sync Manager configuration values
 */
enum : uint32_t {
    EC_DEFAULTMBXSM0 = 0x00010026,  ///< Default SM0 (mailbox in / M→S) config
    EC_DEFAULTMBXSM1 = 0x00010022,  ///< Default SM1 (mailbox out / S→M) config
};

/**
 * @brief EEPROM command structure
 */
struct __attribute__((packed)) EepromCmd {
    uint16_t comm_le;  ///< Command
    uint16_t addr_le;  ///< Word address
    uint16_t d2_le;    ///< Additional data
};
static_assert(sizeof(EepromCmd) == 6, "EepromCmd must be 6 bytes");

// ============================================================================
// Mailbox Protocol Types
// ============================================================================

/**
 * @brief Mailbox types (low nibble of TYPE byte)
 */
enum : uint8_t {
    EC_MBXT_ERR = 0x00,     ///< Error response
    EC_MBXT_AOE = 0x01,     ///< ADS over EtherCAT
    EC_MBXT_EOE = 0x02,     ///< Ethernet over EtherCAT
    EC_MBXT_COE = 0x03,     ///< CANopen over EtherCAT
    EC_MBXT_FOE = 0x04,     ///< File over EtherCAT
    EC_MBXT_SOE = 0x05,     ///< Servo over EtherCAT
    EC_MBXT_VOE = 0x0F,     ///< Vendor over EtherCAT
};

/**
 * @brief CoE service types
 */
enum : uint8_t {
    EC_COES_SDOREQ = 0x02,  ///< SDO Request service
    EC_COES_SDORES = 0x03,  ///< SDO Response service
};

/**
 * @brief SDO command specifiers
 */
enum : uint8_t {
    EC_SDO_DOWN_REQ = 0x20,    ///< Download (write) initiate request
    EC_SDO_DOWN_SEG_REQ = 0x00, ///< Segmented download request
    EC_SDO_UP_REQ = 0x40,      ///< Upload (read) initiate request
    EC_SDO_SEG_UP_REQ = 0x60,  ///< Segmented upload request
    EC_SDO_ABORT = 0x80,       ///< SDO abort transfer
};

/**
 * @brief Sync Manager register structure
 * 
 * Each Sync Manager has 8 bytes of configuration registers.
 */
struct __attribute__((packed)) SyncManagerRegs {
    uint16_t physStart_le;  ///< Physical start address in ESC memory
    uint16_t length_le;     ///< Buffer length in bytes
    uint8_t control;        ///< Control register (direction, mode)
    uint8_t status;         ///< Status register
    uint8_t activate;       ///< Activate register (enable/disable)
    uint8_t pdiControl;     ///< PDI control
};
static_assert(sizeof(SyncManagerRegs) == 8, "SyncManagerRegs must be 8 bytes");

// ============================================================================
// Sync Manager status bit masks (ETG.1000.4, Figure 36 — TSYNCMAN.status)
// ============================================================================

enum : uint8_t {
    EC_SM_STATUS_WRITE_EVENT      = 0x01, ///< Bit 0: Write event (master wrote to SM)
    EC_SM_STATUS_READ_EVENT       = 0x02, ///< Bit 1: Read event (master read from SM)
    EC_SM_STATUS_MBXFULL          = 0x08, ///< Bit 3: Mailbox/buffer full (slave finished)
    EC_SM_STATUS_READ_BUFFER_FULL = 0x40, ///< Bit 6: Read buffer full
    EC_SM_STATUS_WRITE_BUFFER_FULL= 0x80, ///< Bit 7: Write buffer full
};

/**
 * @brief Compute the ESC register address of a Sync Manager's Status byte.
 *
 * Each Sync Manager occupies 8 bytes of register space, starting at 0x0800:
 *   Base  = 0x0800 + (sm_index * 8)
 *   Status= Base + 5
 *
 * @param sm_index Sync Manager index (0..15)
 * @return ESC register address for the SM status byte
 */
static inline uint16_t sm_status_address(uint8_t sm_index)
{
    return static_cast<uint16_t>(EC_REG_SM0 + (sm_index * 8u) + 5u);
}

/**
 * @brief Mailbox header structure
 * 
 * All mailbox protocols (CoE, FoE, SoE, etc.) use this 6-byte header.
 */
struct __attribute__((packed)) MbxHeader {
    uint16_t length_le;   ///< Bytes after mailbox header
    uint16_t address_le;  ///< Source/destination address
    uint8_t priority;     ///< Message priority
    uint8_t mbxtype;      ///< Type (low nibble) + counter (high nibble)
};
static_assert(sizeof(MbxHeader) == 6, "MbxHeader must be 6 bytes");

/**
 * @brief Create mailbox type byte with counter
 * @param type_low_nibble Mailbox type (0-15)
 * @param cnt Counter value (0-15) for sequence tracking
 * @return Combined type/counter byte
 * 
 * The EtherCAT mailbox header encodes a counter in the high nibble of TYPE.
 */
static inline uint8_t mbx_type_with_cnt(uint8_t type_low_nibble, uint8_t cnt)
{
    // EtherCAT mailbox header encodes the counter in the high nibble of the TYPE byte.
    // SOEM macro: MBX_HDR_SET_CNT(cnt) == (cnt << 4) and ORs it into mbxtype.
    return static_cast<uint8_t>(((cnt & 0x07u) << 4) | (type_low_nibble & 0x0Fu));
}

/**
 * @brief CANopen over EtherCAT header
 * 
 * @note We avoid C/C++ bitfields here because bitfield ordering is 
 * implementation-defined and can silently produce invalid protocol headers.
 */
struct __attribute__((packed)) CoeHeader {
    uint16_t raw_le;  ///< Raw 16-bit value: number[9] | reserved[3] | service[4]
};
static_assert(sizeof(CoeHeader) == 2, "CoeHeader must be 2 bytes");

/**
 * @brief Construct CoE header raw value
 * @param number SDO/PDO number (9 bits)
 * @param service Service type (4 bits)
 * @return Raw little-endian header value
 */
static inline uint16_t coe_make_raw(uint16_t number, uint8_t service)
{
    return static_cast<uint16_t>((number & 0x01FFu) | ((static_cast<uint16_t>(service) & 0x0Fu) << 12));
}

/**
 * @brief SDO initiate upload request structure
 */
struct __attribute__((packed)) SdoInitUploadReq {
    uint8_t cmd;            ///< Command byte (0x40 for initiate upload)
    uint16_t index_le;      ///< Object dictionary index
    uint8_t sub;            ///< Subindex
    uint8_t reserved[4];    ///< Reserved bytes
};
static_assert(sizeof(SdoInitUploadReq) == 8, "SdoInitUploadReq must be 8 bytes");

/**
 * @brief SDO initiate upload response structure
 */
struct __attribute__((packed)) SdoInitUploadRes {
    uint8_t cmd;                 ///< Command byte with size indication
    uint16_t index_le;           ///< Object dictionary index (echoed)
    uint8_t sub;                 ///< Subindex (echoed)
    uint32_t data_or_size_le;    ///< Expedited: data; Segmented: total size
};
static_assert(sizeof(SdoInitUploadRes) == 8, "SdoInitUploadRes must be 8 bytes");

/**
 * @brief SDO initiate download request structure
 */
struct __attribute__((packed)) SdoInitDownloadReq {
    uint8_t cmd;            ///< Command byte (0x20 + size indicators)
    uint16_t index_le;      ///< Object dictionary index
    uint8_t sub;            ///< Subindex
    uint32_t data_le;       ///< Data for expedited transfer (1-4 bytes)
};
static_assert(sizeof(SdoInitDownloadReq) == 8, "SdoInitDownloadReq must be 8 bytes");

/**
 * @brief SDO initiate download response structure
 */
struct __attribute__((packed)) SdoInitDownloadRes {
    uint8_t cmd;            ///< Command byte (0x60)
    uint16_t index_le;      ///< Object dictionary index (echoed)
    uint8_t sub;            ///< Subindex (echoed)
    uint8_t reserved[4];    ///< Reserved bytes
};
static_assert(sizeof(SdoInitDownloadRes) == 8, "SdoInitDownloadRes must be 8 bytes");

/**
 * @brief SDO abort transfer structure
 */
struct __attribute__((packed)) SdoAbort {
    uint8_t cmd;              ///< Command byte (0x80)
    uint16_t index_le;        ///< Object that caused abort
    uint8_t sub;              ///< Subindex
    uint32_t abortCode_le;    ///< Abort code (see SDOManager.hpp)
};
static_assert(sizeof(SdoAbort) == 8, "SdoAbort must be 8 bytes");

// ============================================================================
// Response/Receive Structures
// ============================================================================

// RxDatagram is now defined in Types.hpp

/**
 * @brief Minimal scan frame structure
 * 
 * Used for BRD (Broadcast Read) of AL Status to count slaves.
 * WKC indicates number of slaves in the network.
 */
struct __attribute__((packed)) EtherCATScanFrame {
    EthernetHeader eth;
    EtherCATFrameHeader ec;
    EtherCATDatagramHeader dg;
    uint8_t data[2];      ///< AL Status data
    uint16_t wkc_le;      ///< Working Counter
};
static_assert(sizeof(EtherCATScanFrame) == 14 + 2 + 10 + 2 + 2, "Unexpected EtherCATScanFrame size");

// ============================================================================
// Higher-Level Helpers
// ============================================================================

/**
 * @brief Convert slave index to ADP (auto-increment position) value
 * @param slave_index Zero-based slave index
 * @return ADP for use in APxx commands
 */
uint16_t adp_for_slave_index(uint16_t slave_index);

// ============================================================================
// EEPROM/SII and Mailbox Configuration
// ============================================================================

/**
 * @brief Configure mailbox Sync Managers from slave EEPROM (SII)
 *
 * Reads mailbox addresses and sizes from the slave's SII and configures
 * SM0 (Receive/MbxIn) and SM1 (Send/MbxOut).
 *
 * Per standard EtherCAT convention:
 * - SM0 = Receive mailbox (MbxIn) = Master→Slave = std_rx in SII
 * - SM1 = Send mailbox (MbxOut) = Slave→Master = std_tx in SII
 *
 * @param[out] out_wr_addr Receive mailbox address (MbxIn, Master→Slave, SM0)
 * @param[out] out_wr_len Receive mailbox size
 * @param[out] out_rd_addr Send mailbox address (MbxOut, Slave→Master, SM1)
 * @param[out] out_rd_len Send mailbox size
 * @param[out] out_mbx_proto Supported protocols bitmap
 * @return true on success
 */
bool configure_mailbox_from_sii(
    Master& master,
    uint16_t slave_index,
    uint16_t *out_wr_addr,
    uint16_t *out_wr_len,
    uint16_t *out_rd_addr,
    uint16_t *out_rd_len,
    uint16_t *out_mbx_proto);

/**
 * @brief Read a string from slave SII (EEPROM)
 */
bool sii_read_string(Master& master, uint16_t slave_index, uint16_t string_number, char *out, size_t out_cap);

// ============================================================================
// CoE SDO Communication
// ============================================================================

/**
 * @brief CoE SDO upload (read object dictionary entry)
 * 
 * Handles expedited and segmented transfers automatically.
 * 
 * @param[inout] inout_mbx_cnt Mailbox counter (incremented each call)
 * @param index Object dictionary index
 * @param sub Subindex
 * @param[out] out Data buffer
 * @param out_cap Buffer capacity
 * @param[out] out_len Actual bytes read
 * @return true on success
 */
bool coe_sdo_upload(
    Master& master,
    uint16_t adp,
    uint8_t *inout_mbx_cnt,
    uint16_t mbx_write_addr,
    uint16_t mbx_write_len,
    uint16_t mbx_read_addr,
    uint16_t mbx_read_len,
    uint16_t index,
    uint8_t sub,
    uint8_t *out,
    size_t out_cap,
    size_t *out_len,
    bool diag_enabled = false);

/**
 * @brief CANopen SDO download (write) operation
 * 
 * @param eth Ethernet handle
 * @param src_mac Source MAC address
 * @param adp Auto-increment address (slave position)
 * @param[in,out] inout_mbx_cnt Mailbox counter (updated on return)
 * @param mbx_write_addr Mailbox write (slave RX) address
 * @param mbx_write_len Mailbox write length
 * @param mbx_read_addr Mailbox read (slave TX) address
 * @param mbx_read_len Mailbox read length
 * @param index Object dictionary index
 * @param sub Subindex
 * @param data Data to write
 * @param data_len Data length (expedited: 1-4 bytes)
 * @return true on success
 */
bool coe_sdo_download(
    Master& master,
    uint16_t adp,
    uint8_t *inout_mbx_cnt,
    uint16_t mbx_write_addr,
    uint16_t mbx_write_len,
    uint16_t mbx_read_addr,
    uint16_t mbx_read_len,
    uint16_t index,
    uint8_t sub,
    const uint8_t *data,
    size_t data_len,
    bool diag_enabled = false);

} // namespace Raw
} // namespace EtherCAT
