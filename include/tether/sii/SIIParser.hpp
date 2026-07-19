/**
 * @file SIIParser.hpp
 * @brief EtherCAT Slave Information Interface (SII) Parser
 * 
 * @details
 * This module provides a comprehensive parser for the EtherCAT SII EEPROM,
 * as defined in ETG.2010 (EtherCAT Slave Information Interface Specification).
 * 
 * The SII EEPROM contains all configuration data needed to initialize an
 * EtherCAT slave, including:
 * - Device identity (Vendor ID, Product Code, etc.)
 * - Mailbox configuration
 * - Sync Manager configuration
 * - FMMU configuration
 * - PDO mapping information
 * - General device information
 * - Distributed Clock configuration
 * 
 * ## SII EEPROM Structure
 * 
 * ```
 * Word Address   Content
 * ───────────────────────────────────────────────────
 * 0x0000-0x0007  EEPROM Configuration Area
 * 0x0008-0x003F  Device Identity Area
 * 0x0040+        Category Area (variable categories)
 * ```
 * 
 * ## Usage Example
 * 
 * ```cpp
 * #include "sii/SIIParser.hpp"
 * 
 * EtherCAT::SII::SIIReader reader(eth_handle, mac_addr);
 * EtherCAT::SII::SIIData sii_data;
 * 
 * if (reader.readSII(slave_index, sii_data)) {
 *     // Use parsed SII data
 *     printf("Vendor: 0x%08X\n", sii_data.identity.vendor_id);
 *     printf("Product: 0x%08X\n", sii_data.identity.product_code);
 *     
 *     // Configure Sync Managers from SII
 *     for (const auto& sm : sii_data.sync_managers) {
 *         // Apply SM configuration...
 *     }
 * }
 * ```
 * 
 * @see ETG.2010 EtherCAT Slave Information Interface Specification
 */

#pragma once

#include <cstdint>
#include <cstddef>
#include <cstring>
#include <array>
#include <vector>
#include <bit>

#include "tether/ethercat/SMRegisters.hpp"
#include "tether/ethercat/TetherConfig.hpp"

namespace EtherCAT {
namespace SII {

// ============================================================================
// SII Constants
// ============================================================================

/**
 * @brief SII Word Addresses (from ETG.2010)
 */
enum SIIWordAddress : uint16_t {
    // EEPROM Configuration Area (0x0000-0x0007)
    SII_PDI_CONTROL              = 0x0000,  ///< PDI Control
    SII_PDI_CONFIG               = 0x0001,  ///< PDI Configuration
    SII_SYNC_IMPULSE_LEN         = 0x0002,  ///< Sync Impulse Length
    SII_PDI_CONFIG2              = 0x0003,  ///< PDI Configuration 2
    SII_ALIAS_ADDRESS            = 0x0004,  ///< Configured Station Alias
    SII_RESERVED_0005            = 0x0005,  ///< Reserved
    SII_RESERVED_0006            = 0x0006,  ///< Reserved
    SII_CHECKSUM                 = 0x0007,  ///< CRC (words 0-6)
    
    // Device Identity Area (0x0008-0x000F)
    SII_VENDOR_ID                = 0x0008,  ///< Vendor ID (32-bit, words 0x08-0x09)
    SII_PRODUCT_CODE             = 0x000A,  ///< Product Code (32-bit, words 0x0A-0x0B)
    SII_REVISION                 = 0x000C,  ///< Revision Number (32-bit, words 0x0C-0x0D)
    SII_SERIAL_NUMBER            = 0x000E,  ///< Serial Number (32-bit, words 0x0E-0x0F)
    
    // Bootstrap Mailbox (0x0014-0x001D)
    SII_BOOTSTRAP_RX_MBX_OFFSET  = 0x0014,  ///< Bootstrap Receive Mailbox Offset (MbxIn — Master->Slave, SM1)
    SII_BOOTSTRAP_RX_MBX_SIZE    = 0x0015,  ///< Bootstrap Receive Mailbox Size   (MbxIn — Master->Slave, SM1)
    SII_BOOTSTRAP_TX_MBX_OFFSET  = 0x0016,  ///< Bootstrap Send Mailbox Offset    (MbxOut — Slave->Master, SM0)
    SII_BOOTSTRAP_TX_MBX_SIZE    = 0x0017,  ///< Bootstrap Send Mailbox Size      (MbxOut — Slave->Master, SM0)
    
    // Standard Mailbox (0x0018-0x001D)
    SII_STD_RX_MBX_OFFSET        = 0x0018,  ///< Standard Receive Mailbox Offset (MbxIn — Master->Slave, SM1)
    SII_STD_RX_MBX_SIZE          = 0x0019,  ///< Standard Receive Mailbox Size   (MbxIn — Master->Slave, SM1)
    SII_STD_TX_MBX_OFFSET        = 0x001A,  ///< Standard Send Mailbox Offset    (MbxOut — Slave->Master, SM0)
    SII_STD_TX_MBX_SIZE          = 0x001B,  ///< Standard Send Mailbox Size      (MbxOut — Slave->Master, SM0)
    SII_MAILBOX_PROTOCOLS        = 0x001C,  ///< Supported Mailbox Protocols
    
    // Size Information (0x002E-0x002F)
    SII_SIZE_INFO                = 0x002E,  ///< Size information
    SII_VERSION                  = 0x002F,  ///< Version
    
    // Category Area Start
    SII_CATEGORY_START           = 0x0040,  ///< Start of category area
};

/**
 * @brief SII Category Types (from ETG.2010 Table 17)
 */
enum SIICategoryType : uint16_t {
    CAT_NOP              = 0,      ///< No operation (padding)
    CAT_STRINGS          = 10,     ///< Strings category
    CAT_DATA_TYPES       = 20,     ///< DataTypes (not commonly used)
    CAT_GENERAL          = 30,     ///< General Device Information
    CAT_FMMU             = 40,     ///< FMMU Configuration
    CAT_SYNC_MANAGER     = 41,     ///< Sync Manager Configuration
    CAT_FMMU_EX          = 42,     ///< FMMU Extended (not commonly used)
    CAT_SYNC_UNIT        = 43,     ///< Sync Unit (DC)
    CAT_TXPDO            = 50,     ///< TxPDO Description
    CAT_RXPDO            = 51,     ///< RxPDO Description
    CAT_DC               = 60,     ///< Distributed Clocks Configuration
    CAT_END              = 0xFFFF, ///< End of categories marker
};

/**
 * @brief Mailbox Protocol Flags (from ETG.2010)
 */
enum MailboxProtocol : uint16_t {
    MBX_PROTO_AOE   = 0x0001,  ///< ADS over EtherCAT
    MBX_PROTO_EOE   = 0x0002,  ///< Ethernet over EtherCAT
    MBX_PROTO_COE   = 0x0004,  ///< CANopen over EtherCAT
    MBX_PROTO_FOE   = 0x0008,  ///< File over EtherCAT
    MBX_PROTO_SOE   = 0x0010,  ///< Servo over EtherCAT
    MBX_PROTO_VOE   = 0x0020,  ///< Vendor over EtherCAT
};

/**
 * @brief Sync Manager Types (from ETG.2010)
 */
enum SyncManagerType : uint8_t {
    SM_TYPE_UNUSED      = 0,   ///< Not used
    SM_TYPE_MBX_WRITE   = 1,   ///< Mailbox Write (master → slave)
    SM_TYPE_MBX_READ    = 2,   ///< Mailbox Read (slave → master)
    SM_TYPE_PROCESS_OUT = 3,   ///< Process Data Output (RxPDO)
    SM_TYPE_PROCESS_IN  = 4,   ///< Process Data Input (TxPDO)
};

/**
 * @brief FMMU Types
 */
enum FMMUType : uint8_t {
    FMMU_TYPE_UNUSED   = 0,    ///< Not used
    FMMU_TYPE_OUTPUT   = 1,    ///< Outputs (RxPDO)
    FMMU_TYPE_INPUT    = 2,    ///< Inputs (TxPDO)
    FMMU_TYPE_MBX_SYNC = 3,    ///< Mailbox synchronization
};

// ============================================================================
// SII Data Structures
// ============================================================================

/**
 * @brief Device Identity from SII
 */
struct SIIIdentity {
    uint32_t vendor_id{0};        ///< Vendor ID (ETG assigned)
    uint32_t product_code{0};     ///< Vendor-specific product code
    uint32_t revision_number{0};  ///< Revision (major.minor in high.low words)
    uint32_t serial_number{0};    ///< Device serial number
    
    uint16_t revisionMajor() const { 
        return static_cast<uint16_t>((revision_number >> 16) & 0xFFFF); 
    }
    uint16_t revisionMinor() const { 
        return static_cast<uint16_t>(revision_number & 0xFFFF); 
    }
    bool isValid() const { return vendor_id != 0; }
};

/**
 * @brief Mailbox Configuration from SII
 */
struct SIIMailboxConfig {
    // Bootstrap mailbox (used during firmware update)
    uint16_t bootstrap_rx_offset{0};  ///< Bootstrap RX mailbox physical address (MbxIn — Master->Slave, SM1)
    uint16_t bootstrap_rx_size{0};    ///< Bootstrap RX mailbox size in bytes          (MbxIn — Master->Slave, SM1)
    uint16_t bootstrap_tx_offset{0};  ///< Bootstrap TX mailbox physical address (MbxOut — Slave->Master, SM0)
    uint16_t bootstrap_tx_size{0};    ///< Bootstrap TX mailbox size in bytes          (MbxOut — Slave->Master, SM0)
    
    // Standard mailbox (normal operation)
    uint16_t std_rx_offset{0};        ///< Standard RX mailbox physical address (MbxIn — Master->Slave, SM1)
    uint16_t std_rx_size{0};          ///< Standard RX mailbox size in bytes          (MbxIn — Master->Slave, SM1)
    uint16_t std_tx_offset{0};        ///< Standard TX mailbox physical address (MbxOut — Slave->Master, SM0)
    uint16_t std_tx_size{0};          ///< Standard TX mailbox size in bytes          (MbxOut — Slave->Master, SM0)
    
    // Supported protocols
    uint16_t protocols{0};            ///< Mailbox protocol flags
    
    bool supportsAoE() const { return (protocols & MBX_PROTO_AOE) != 0; }
    bool supportsEoE() const { return (protocols & MBX_PROTO_EOE) != 0; }
    bool supportsCoE() const { return (protocols & MBX_PROTO_COE) != 0; }
    bool supportsFoE() const { return (protocols & MBX_PROTO_FOE) != 0; }
    bool supportsSoE() const { return (protocols & MBX_PROTO_SOE) != 0; }
    bool supportsVoE() const { return (protocols & MBX_PROTO_VOE) != 0; }
    bool hasMailbox() const { return std_rx_size > 0 && std_tx_size > 0; }
    bool hasBootstrapMailbox() const { return bootstrap_rx_size > 0 && bootstrap_tx_size > 0; }
};

/**
 * @brief General Device Information from CAT_GENERAL (category 30)
 */
struct SIIGeneralInfo {
    uint8_t group_idx{0};           ///< Group string index
    uint8_t image_idx{0};           ///< Image string index
    uint8_t order_idx{0};           ///< Order number string index
    uint8_t name_idx{0};            ///< Device name string index
    uint8_t reserved{0};            ///< Reserved
    uint8_t coe_details{0};         ///< CoE details flags
    uint8_t foe_details{0};         ///< FoE details flags
    uint8_t eoe_details{0};         ///< EoE details flags
    uint8_t soe_channels{0};        ///< SoE channels (reserved)
    uint8_t ds402_channels{0};      ///< DS402 channels (reserved)
    uint8_t sys_man_class{0};       ///< System Manager Class
    uint8_t flags{0};               ///< General flags
    int16_t current_ebus{0};        ///< E-Bus current consumption (mA, negative = provides power)
    uint8_t group_idx2{0};          ///< Group string index (duplicate for compatibility)
    uint8_t reserved2{0};           ///< Reserved
    uint16_t phys_port{0};          ///< Physical port configuration
    uint16_t phys_mem_addr{0};      ///< Physical memory address
    uint8_t reserved3[12]{0};       ///< Reserved bytes
    
    // CoE details flags
    bool coeEnableSdo() const { return (coe_details & 0x01) != 0; }
    bool coeEnableSdoInfo() const { return (coe_details & 0x02) != 0; }
    bool coeEnablePdoAssign() const { return (coe_details & 0x04) != 0; }
    bool coeEnablePdoConfig() const { return (coe_details & 0x08) != 0; }
    bool coeEnableUploadStartup() const { return (coe_details & 0x10) != 0; }
    bool coeEnableSdoComplete() const { return (coe_details & 0x20) != 0; }
    
    // General flags
    bool enableNotLRW() const { return (flags & 0x01) != 0; }
    bool enableSafeOp() const { return (flags & 0x02) != 0; }
    bool enableLRW() const { return (flags & 0x04) != 0; }
};

/**
 * @brief Sync Manager Configuration Entry from CAT_SYNC_MANAGER (category 41)
 */
struct SIISyncManager {
    uint16_t phys_start_address{0};  ///< Physical start address in ESC memory
    uint16_t length{0};              ///< Length in bytes
    EtherCAT::SyncManager::SMControlReg  control_register{};  ///< Control register
    EtherCAT::SyncManager::SMStatusReg   status_register{};   ///< Status register (usually 0)
    EtherCAT::SyncManager::SMActivateReg enable{};            ///< Enable flags
    uint8_t sm_type{0};              ///< Sync Manager type (SyncManagerType enum)
    
    bool isEnabled() const { return enable.enable; }
    bool isFixedContent() const { return (std::bit_cast<uint8_t>(enable) & 0x02) != 0; }
    bool isVirtualSM() const { return (std::bit_cast<uint8_t>(enable) & 0x04) != 0; }
    bool opOnly() const { return (std::bit_cast<uint8_t>(enable) & 0x08) != 0; }
    
    SyncManagerType getType() const { return static_cast<SyncManagerType>(sm_type); }
    
    const char* getTypeName() const {
        switch (sm_type) {
            case SM_TYPE_UNUSED: return "Unused";
            case SM_TYPE_MBX_WRITE: return "Mailbox Write";
            case SM_TYPE_MBX_READ: return "Mailbox Read";
            case SM_TYPE_PROCESS_OUT: return "Process Output (RxPDO)";
            case SM_TYPE_PROCESS_IN: return "Process Input (TxPDO)";
            default: return "Unknown";
        }
    }
};

/**
 * @brief FMMU Configuration Entry from CAT_FMMU (category 40)
 */
struct SIIFMMU {
    uint8_t fmmu_type{0};  ///< FMMU type (FMMUType enum)
    
    FMMUType getType() const { return static_cast<FMMUType>(fmmu_type); }
    
    const char* getTypeName() const {
        switch (fmmu_type) {
            case FMMU_TYPE_UNUSED: return "Unused";
            case FMMU_TYPE_OUTPUT: return "Outputs";
            case FMMU_TYPE_INPUT: return "Inputs";
            case FMMU_TYPE_MBX_SYNC: return "Mailbox Sync";
            default: return "Unknown";
        }
    }
};

/**
 * @brief PDO Entry (object mapping within a PDO)
 */
struct SIIPDOEntry {
    uint16_t index{0};      ///< Object dictionary index
    uint8_t subindex{0};    ///< Object dictionary subindex
    uint8_t name_idx{0};    ///< String index for entry name (0 = no name)
    uint8_t data_type{0};   ///< Data type (ETG.1000 types)
    uint8_t bit_length{0};  ///< Size in bits
    uint16_t flags{0};      ///< Additional flags
};

/**
 * @brief PDO Description from CAT_TXPDO/CAT_RXPDO (categories 50/51)
 */
struct SIIPDO {
    uint16_t pdo_index{0};           ///< PDO index (0x1600-0x17FF RxPDO, 0x1A00-0x1BFF TxPDO)
    uint8_t n_entries{0};            ///< Number of entries
    uint8_t sync_manager{0};         ///< Assigned Sync Manager
    uint8_t dc_sync{0};              ///< DC Synchronization (0 = free run, 1 = DC Sync 0, 2 = DC Sync 1)
    uint8_t name_idx{0};             ///< String index for PDO name
    uint16_t flags{0};               ///< PDO flags
    std::vector<SIIPDOEntry> entries; ///< PDO entries (object mappings)
    
    bool isTxPDO() const { return (pdo_index >= 0x1A00 && pdo_index <= 0x1BFF); }
    bool isRxPDO() const { return (pdo_index >= 0x1600 && pdo_index <= 0x17FF); }
    bool isMandatory() const { return (flags & 0x0001) != 0; }
    bool isDefault() const { return (flags & 0x0002) != 0; }
    bool isFixed() const { return (flags & 0x0010) != 0; }
    
    size_t totalBits() const {
        size_t bits = 0;
        for (const auto& e : entries) {
            bits += e.bit_length;
        }
        return bits;
    }
    
    size_t totalBytes() const { return (totalBits() + 7) / 8; }
};

/**
 * @brief Distributed Clock Configuration from CAT_DC (category 60)
 */
struct SIIDCConfig {
    uint32_t cycle_time_0{0};     ///< Cycle time SYNC0 in ns
    uint32_t shift_time_0{0};     ///< Shift time SYNC0 in ns
    uint32_t shift_time_1{0};     ///< Shift time SYNC1 in ns
    int16_t cycle_time_1_factor{0}; ///< Cycle time factor for SYNC1
    uint16_t assign_activate{0};   ///< DC AssignActivate value
    int16_t cycle_time_0_factor{0}; ///< Cycle time factor for SYNC0
    uint8_t name_idx{0};          ///< String index for name
    uint8_t desc_idx{0};          ///< String index for description
    uint8_t reserved[4]{0};       ///< Reserved
    
    bool sync0Enabled() const { return (assign_activate & 0x0100) != 0; }
    bool sync1Enabled() const { return (assign_activate & 0x0200) != 0; }
};

// ============================================================================
// String Storage
// ============================================================================

/**
 * @brief String storage for SII strings
 * 
 * The SII contains a string category with indexed strings. This structure
 * provides efficient storage and lookup of these strings.
 */
class SIIStrings {
public:
    static constexpr size_t MAX_STRINGS = ECAT_SII_MAX_STRINGS;
    static constexpr size_t MAX_STRING_BUFFER = ECAT_SII_MAX_STRING_BUFFER;
    
    SIIStrings() {
        clear();
    }
    
    void clear() {
        m_count = 0;
        m_buffer_offset = 0;
        memset(m_buffer, 0, sizeof(m_buffer));
        memset(m_offsets, 0, sizeof(m_offsets));
        memset(m_lengths, 0, sizeof(m_lengths));
    }
    
    /**
     * @brief Add a string to storage
     * @param str String data
     * @param len String length (not including null terminator)
     * @return String index (1-based), or 0 on failure
     */
    uint8_t addString(const char* str, size_t len) {
        if (m_count >= MAX_STRINGS) return 0;
        if (m_buffer_offset + len + 1 > MAX_STRING_BUFFER) return 0;
        
        m_offsets[m_count] = static_cast<uint16_t>(m_buffer_offset);
        m_lengths[m_count] = static_cast<uint8_t>(len);
        
        if (str && len > 0) {
            memcpy(m_buffer + m_buffer_offset, str, len);
        }
        m_buffer[m_buffer_offset + len] = '\0';
        m_buffer_offset += len + 1;
        
        m_count++;
        return m_count;  // 1-based index
    }
    
    /**
     * @brief Get string by index (1-based as per SII convention)
     * @param index String index (1-based, 0 returns empty string)
     * @return Pointer to null-terminated string
     */
    const char* getString(uint8_t index) const {
        if (index == 0 || index > m_count) return "";
        return m_buffer + m_offsets[index - 1];
    }
    
    size_t count() const { return m_count; }
    
private:
    char m_buffer[MAX_STRING_BUFFER];
    uint16_t m_offsets[MAX_STRINGS];
    uint8_t m_lengths[MAX_STRINGS];
    size_t m_count{0};
    size_t m_buffer_offset{0};
};

// ============================================================================
// Complete SII Data Structure
// ============================================================================

/**
 * @brief Complete parsed SII data for a slave
 */
struct SIIData {
    // Basic configuration
    uint16_t pdi_control{0};           ///< PDI Control register value
    uint16_t pdi_config{0};            ///< PDI Configuration
    uint16_t sync_impulse_len{0};      ///< Sync impulse length (x10ns)
    uint16_t pdi_config2{0};           ///< Extended PDI configuration
    uint16_t alias_address{0};         ///< Configured station alias
    uint16_t checksum{0};              ///< Configuration area CRC
    bool checksum_ok{false};            ///< CRC-8 (words 0..6) matched low byte of `checksum`

    // Device identity
    SIIIdentity identity;
    
    // Mailbox configuration
    SIIMailboxConfig mailbox;
    
    // General device info
    SIIGeneralInfo general;
    bool has_general{false};
    
    // Strings
    SIIStrings strings;
    
    // Sync Managers (up to 8 in standard ESC)
    std::array<SIISyncManager, 8> sync_managers;
    size_t sm_count{0};
    
    // FMMUs (up to 8 in standard ESC)
    std::array<SIIFMMU, 8> fmmus;
    size_t fmmu_count{0};
    
    // PDOs
    std::vector<SIIPDO> tx_pdos;
    std::vector<SIIPDO> rx_pdos;
    
    // DC configuration
    std::vector<SIIDCConfig> dc_configs;
    
    // Size information
    uint16_t version{0};
    uint16_t eeprom_size_kbits{0};   ///< EEPROM size in Kbits (from SII_SIZE_INFO word 0x002E)
    uint16_t eeprom_size_words{0};   ///< EEPROM size in 16-bit words (calculated from kbits)
    
    // Parsing state
    bool valid{false};
    bool parse_complete{false};
    
    /**
     * @brief Clear all data
     */
    void clear() {
        pdi_control = 0;
        pdi_config = 0;
        sync_impulse_len = 0;
        pdi_config2 = 0;
        alias_address = 0;
        checksum = 0;
        identity = SIIIdentity{};
        mailbox = SIIMailboxConfig{};
        general = SIIGeneralInfo{};
        has_general = false;
        strings.clear();
        for (auto& sm : sync_managers) sm = SIISyncManager{};
        sm_count = 0;
        for (auto& fmmu : fmmus) fmmu = SIIFMMU{};
        fmmu_count = 0;
        tx_pdos.clear();
        rx_pdos.clear();
        dc_configs.clear();
        version = 0;
        eeprom_size_kbits = 0;
        eeprom_size_words = 0;
        valid = false;
        parse_complete = false;
        checksum_ok = false;
    }
    
    /**
     * @brief Get device name from strings
     */
    const char* deviceName() const {
        return strings.getString(general.name_idx);
    }
    
    /**
     * @brief Get group name from strings
     */
    const char* groupName() const {
        return strings.getString(general.group_idx);
    }
    
    /**
     * @brief Get order code from strings
     */
    const char* orderCode() const {
        return strings.getString(general.order_idx);
    }
    
    /**
     * @brief Find process output Sync Manager (RxPDO)
     * @return Pointer to SM or nullptr
     */
    const SIISyncManager* findProcessOutputSM() const {
        for (size_t i = 0; i < sm_count; i++) {
            if (sync_managers[i].sm_type == SM_TYPE_PROCESS_OUT) {
                return &sync_managers[i];
            }
        }
        return nullptr;
    }
    
    /**
     * @brief Find process input Sync Manager (TxPDO)
     * @return Pointer to SM or nullptr
     */
    const SIISyncManager* findProcessInputSM() const {
        for (size_t i = 0; i < sm_count; i++) {
            if (sync_managers[i].sm_type == SM_TYPE_PROCESS_IN) {
                return &sync_managers[i];
            }
        }
        return nullptr;
    }
    
    /**
     * @brief Get total RxPDO size in bytes
     */
    size_t totalRxPDOBytes() const {
        size_t total = 0;
        for (const auto& pdo : rx_pdos) {
            total += pdo.totalBytes();
        }
        return total;
    }
    
    /**
     * @brief Get total TxPDO size in bytes
     */
    size_t totalTxPDOBytes() const {
        size_t total = 0;
        for (const auto& pdo : tx_pdos) {
            total += pdo.totalBytes();
        }
        return total;
    }
};

// ============================================================================
// Category Header for Parsing
// ============================================================================

/**
 * @brief SII Category Header (4 bytes)
 */
struct SIICategoryHeader {
    uint16_t type;      ///< Category type (SIICategoryType)
    uint16_t size;      ///< Size in words (not including header)
};

// ============================================================================
// Forward Declarations
// ============================================================================

class SIIReader;
class SIIParser;

// ============================================================================
// Utility Functions
// ============================================================================

/**
 * @brief Get human-readable name for category type
 */
const char* getCategoryTypeName(uint16_t type);

/**
 * @brief Get human-readable name for mailbox protocol
 */
const char* getMailboxProtocolName(uint16_t protocol);

/**
 * @brief Format vendor ID as human-readable string
 */
const char* formatVendorId(uint32_t vendor_id);

} // namespace SII
} // namespace EtherCAT
