// SPDX-License-Identifier: MIT
/**
 * @file SIILogger.cpp
 * @brief SII EEPROM logging and diagnostic printing functions
 *
 * @details
 * Extracted from SIIReader.cpp.  These functions are stateless — they
 * operate only on the data structures passed to them and have no
 * dependency on the SIIReader or SIIParser class instance state.
 */

#include "tether/sii/SIILogger.hpp"

#include "tether/sii/SIIParser.hpp"   // SIIData, SIIIdentity, SIIMailboxConfig, CAT_*, MBX_PROTO_*, SII_* constants
#include "tether/sii/SIIReader.hpp"    // SIIReader (used by debugSIIMailboxDerivation)
#include "tether/ethercat/Master.hpp"
#include "tether/platform/Platform.hpp"

#include <cinttypes>
#include <cstring>
#include <string>

namespace EtherCAT {
namespace SII {

// ============================================================================
// Utility Functions
// ============================================================================

const char* getCategoryTypeName(uint16_t type) {
    switch (type) {
        case CAT_NOP: return "NOP";
        case CAT_STRINGS: return "Strings";
        case CAT_DATA_TYPES: return "DataTypes";
        case CAT_GENERAL: return "General";
        case CAT_FMMU: return "FMMU";
        case CAT_SYNC_MANAGER: return "SyncManager";
        case CAT_FMMU_EX: return "FMMU_EX";
        case CAT_SYNC_UNIT: return "SyncUnit";
        case CAT_TXPDO: return "TxPDO";
        case CAT_RXPDO: return "RxPDO";
        case CAT_DC: return "DC";
        case CAT_END: return "End";
        default: return "Unknown";
    }
}

const char* getMailboxProtocolName(uint16_t protocol) {
    static char buf[64];
    buf[0] = '\0';
    size_t buf_len = 0;
    
    if (protocol & MBX_PROTO_AOE) {
        strncat(buf, "AoE ", sizeof(buf) - buf_len - 1);
        buf_len = strlen(buf);
    }
    if (protocol & MBX_PROTO_EOE) {
        strncat(buf, "EoE ", sizeof(buf) - buf_len - 1);
        buf_len = strlen(buf);
    }
    if (protocol & MBX_PROTO_COE) {
        strncat(buf, "CoE ", sizeof(buf) - buf_len - 1);
        buf_len = strlen(buf);
    }
    if (protocol & MBX_PROTO_FOE) {
        strncat(buf, "FoE ", sizeof(buf) - buf_len - 1);
        buf_len = strlen(buf);
    }
    if (protocol & MBX_PROTO_SOE) {
        strncat(buf, "SoE ", sizeof(buf) - buf_len - 1);
        buf_len = strlen(buf);
    }
    if (protocol & MBX_PROTO_VOE) {
        strncat(buf, "VoE ", sizeof(buf) - buf_len - 1);
        buf_len = strlen(buf);
    }
    
    if (buf[0] == '\0') {
        return "None";
    }
    
    // Remove trailing space
    if (buf_len > 0 && buf[buf_len-1] == ' ') {
        buf[buf_len-1] = '\0';
    }
    
    return buf;
}

// ============================================================================
// Logging Functions
// ============================================================================

void logSIIIdentity(const SIIIdentity& identity, const char* tag) {
    TETHER_LOGI(tag, "SII Identity:\n  Vendor ID:    0x{:08x}\n  Product Code: 0x{:08x}\n  Revision:     {}.{} (0x{:08x})\n  Serial:       0x{:08x}",
             identity.vendor_id, identity.product_code,
             identity.revisionMajor(), identity.revisionMinor(), identity.revision_number,
             identity.serial_number);
} 

void logSIIMailbox(const SIIMailboxConfig& mailbox, const char* tag) {
    TETHER_LOGI(tag, "SII Mailbox Configuration:");

    if (mailbox.hasMailbox()) {
        TETHER_LOGI(tag, "  Standard Mailbox:\n    RX (MbxIn — Master->Slave, SM0): addr=0x{:04X} size={}\n    TX (MbxOut — Slave->Master, SM1): addr=0x{:04X} size={}",
                 mailbox.std_rx_offset, mailbox.std_rx_size, mailbox.std_tx_offset, mailbox.std_tx_size);
    } else {
        TETHER_LOGI(tag, "  Standard Mailbox: Not configured");
    }

    if (mailbox.hasBootstrapMailbox()) {
        TETHER_LOGI(tag, "  Bootstrap Mailbox:\n    RX (MbxIn — Master->Slave, SM0): addr=0x{:04X} size={}\n    TX (MbxOut — Slave->Master, SM1): addr=0x{:04X} size={}",
                 mailbox.bootstrap_rx_offset, mailbox.bootstrap_rx_size, mailbox.bootstrap_tx_offset, mailbox.bootstrap_tx_size);
    } else {
        TETHER_LOGI(tag, "  Bootstrap Mailbox: Not configured");
    }

    TETHER_LOGI(tag, "  Protocols: 0x{:04X} ({})",
             mailbox.protocols, getMailboxProtocolName(mailbox.protocols));
}

void logSIISyncManagers(const SIIData& data, const char* tag) {
    TETHER_LOGI(tag, "SII Sync Managers ({} configured):", data.sm_count);
    
    for (size_t i = 0; i < data.sm_count; i++) {
        const auto& sm = data.sync_managers[i];
        TETHER_LOGI(tag, "  SM{}: {}\n    Address: 0x{:04X}  Length: {} bytes", i, sm.getTypeName(), sm.phys_start_address, sm.length);
        TETHER_LOGI(tag, "    Control: 0x{:02X}  Enable: 0x{:02X} {}",
                 std::bit_cast<uint8_t>(sm.control_register), std::bit_cast<uint8_t>(sm.enable),
                 sm.isEnabled() ? "[ENABLED]" : "[DISABLED]");
    }
}

void logSIIPDOs(const SIIData& data, const char* tag) {
    if (!data.tx_pdos.empty()) {
        TETHER_LOGI(tag, "SII TxPDOs (Slave→Master):");
        for (const auto& pdo : data.tx_pdos) {
            const char* name = data.strings.getString(pdo.name_idx);
            TETHER_LOGI(tag, "  PDO 0x{:04X} '{}' SM{} ({} entries, {} bits)",
                     pdo.pdo_index, name, pdo.sync_manager,
                     pdo.entries.size(), pdo.totalBits());
            
            for (const auto& entry : pdo.entries) {
                const char* entry_name = data.strings.getString(entry.name_idx);
                TETHER_LOGI(tag, "    0x{:04X}:{:02X} '{}' {} bits",
                         entry.index, entry.subindex, entry_name, entry.bit_length);
            }
        }
    }
    
    if (!data.rx_pdos.empty()) {
        TETHER_LOGI(tag, "SII RxPDOs (Master→Slave):");
        for (const auto& pdo : data.rx_pdos) {
            const char* name = data.strings.getString(pdo.name_idx);
            TETHER_LOGI(tag, "  PDO 0x{:04X} '{}' SM{} ({} entries, {} bits)",
                     pdo.pdo_index, name, pdo.sync_manager,
                     pdo.entries.size(), pdo.totalBits());
            
            for (const auto& entry : pdo.entries) {
                const char* entry_name = data.strings.getString(entry.name_idx);
                TETHER_LOGI(tag, "    0x{:04X}:{:02X} '{}' {} bits",
                         entry.index, entry.subindex, entry_name, entry.bit_length);
            }
        }
    }
}

void logSIIData(const SIIData& data, const char* tag) {
    TETHER_LOGI(tag, "\n╔══════════════════════════════════════════════════════════════╗\n║           SII EEPROM Data                                    ║\n╚══════════════════════════════════════════════════════════════╝\n");
    
    // Device info
    if (data.has_general) {
        TETHER_LOGI(tag, "Device Information:\n  Name:  {}\n  Group: {}\n  Order: {}\n", data.deviceName(), data.groupName(), data.orderCode());
    }
    
    // Identity
    logSIIIdentity(data.identity, tag);
    TETHER_LOGI(tag, "");
    
    // Mailbox
    logSIIMailbox(data.mailbox, tag);
    TETHER_LOGI(tag, "");
    
    // Sync Managers
    logSIISyncManagers(data, tag);
    TETHER_LOGI(tag, "");
    
    // FMMUs
    if (data.fmmu_count > 0) {
        TETHER_LOGI(tag, "SII FMMUs ({} configured):", data.fmmu_count);
        for (size_t i = 0; i < data.fmmu_count; i++) {
            TETHER_LOGI(tag, "  FMMU{}: {}", i, data.fmmus[i].getTypeName());
        }
        TETHER_LOGI(tag, "");
    }
    
    // PDOs
    logSIIPDOs(data, tag);
    
    // DC
    if (!data.dc_configs.empty()) {
        TETHER_LOGI(tag, "SII Distributed Clocks ({} configured):", data.dc_configs.size());
        for (size_t i = 0; i < data.dc_configs.size(); i++) {
            const auto& dc = data.dc_configs[i];
            TETHER_LOGI(tag, "  DC{}: cycle0={}ns shift0={}ns sync0={} sync1={}",
                     i, dc.cycle_time_0, dc.shift_time_0,
                     dc.sync0Enabled() ? "on" : "off", dc.sync1Enabled() ? "on" : "off");
        }
    }
}

void debugSIIMailboxDerivation(Master& master, uint16_t slave_index, const char* tag) {
    TETHER_LOGI(tag, "\n╔══════════════════════════════════════════════════════════════╗\n║  SII Mailbox Derivation Debug (Slave {})                      ║\n╚══════════════════════════════════════════════════════════════╝\n", (unsigned)slave_index);
    
    SIIReader reader(master);
    
    // Helper to log a word read with byte-level detail
    auto log_word_read = [&](uint16_t word_addr, const char* field_name, uint16_t& out_value) -> bool {
        if (!reader.readWord(slave_index, word_addr, out_value)) {
            TETHER_LOGE(tag, "  ❌ Failed to read word 0x{:04X} ({})", (unsigned)word_addr, field_name);
            return false;
        }
        
        // Log word read
        TETHER_LOGI(tag, "  📖 Read word 0x{:04X} ({}): 0x{:04X} ({})",
                 (unsigned)word_addr, field_name, (unsigned)out_value, (unsigned)out_value);
        
        // Log byte breakdown
        uint8_t lo_byte = out_value & 0xFF;
        uint8_t hi_byte = (out_value >> 8) & 0xFF;
        TETHER_LOGI(tag, "     Byte breakdown: [0x{:02X}, 0x{:02X}] (little-endian: lo=0x{:02X} hi=0x{:02X})",
                 (unsigned)lo_byte, (unsigned)hi_byte, (unsigned)lo_byte, (unsigned)hi_byte);
        
        return true;
    };
    
    TETHER_LOGI(tag, "\n📋 STEP 1: Reading Bootstrap Mailbox Configuration (words 0x0014-0x0017)");
    
    uint16_t bootstrap_rx_offset = 0, bootstrap_rx_size = 0;
    uint16_t bootstrap_tx_offset = 0, bootstrap_tx_size = 0;
    
    log_word_read(SII_BOOTSTRAP_RX_MBX_OFFSET, "Bootstrap RX Offset", bootstrap_rx_offset);
    log_word_read(SII_BOOTSTRAP_RX_MBX_SIZE, "Bootstrap RX Size", bootstrap_rx_size);
    log_word_read(SII_BOOTSTRAP_TX_MBX_OFFSET, "Bootstrap TX Offset", bootstrap_tx_offset);
    log_word_read(SII_BOOTSTRAP_TX_MBX_SIZE, "Bootstrap TX Size", bootstrap_tx_size);
    
    TETHER_LOGI(tag, "\n📋 STEP 2: Reading Standard Mailbox Configuration (words 0x0018-0x001C)");
    
    uint16_t std_rx_offset = 0, std_rx_size = 0;
    uint16_t std_tx_offset = 0, std_tx_size = 0;
    uint16_t protocols = 0;
    
    log_word_read(SII_STD_RX_MBX_OFFSET, "Standard RX Offset", std_rx_offset);
    log_word_read(SII_STD_RX_MBX_SIZE, "Standard RX Size", std_rx_size);
    log_word_read(SII_STD_TX_MBX_OFFSET, "Standard TX Offset", std_tx_offset);
    log_word_read(SII_STD_TX_MBX_SIZE, "Standard TX Size", std_tx_size);
    log_word_read(SII_MAILBOX_PROTOCOLS, "Mailbox Protocols", protocols);
    
    TETHER_LOGI(tag, "\n📋 STEP 3: Field Assignments");
    TETHER_LOGI(tag, "  Bootstrap Mailbox:");
    TETHER_LOGI(tag, "    bootstrap_rx_offset  = 0x{:04X} (word 0x0014) → RX mailbox address (MbxIn, Master→Slave, SM0)",
             (unsigned)bootstrap_rx_offset);
    TETHER_LOGI(tag, "    bootstrap_rx_size    = 0x{:04X} (word 0x0015) → RX mailbox size in bytes",
             (unsigned)bootstrap_rx_size);
    TETHER_LOGI(tag, "    bootstrap_tx_offset  = 0x{:04X} (word 0x0016) → TX mailbox address (MbxOut, Slave→Master, SM1)",
             (unsigned)bootstrap_tx_offset);
    TETHER_LOGI(tag, "    bootstrap_tx_size    = 0x{:04X} (word 0x0017) → TX mailbox size in bytes",
             (unsigned)bootstrap_tx_size);
    
    TETHER_LOGI(tag, "  Standard Mailbox:");
    TETHER_LOGI(tag, "    std_rx_offset        = 0x{:04X} (word 0x0018) → RX mailbox address (MbxIn, Master→Slave, SM0)",
             (unsigned)std_rx_offset);
    TETHER_LOGI(tag, "    std_rx_size          = 0x{:04X} (word 0x0019) → RX mailbox size in bytes",
             (unsigned)std_rx_size);
    TETHER_LOGI(tag, "    std_tx_offset        = 0x{:04X} (word 0x001A) → TX mailbox address (MbxOut, Slave→Master, SM1)",
             (unsigned)std_tx_offset);
    TETHER_LOGI(tag, "    std_tx_size          = 0x{:04X} (word 0x001B) → TX mailbox size in bytes",
             (unsigned)std_tx_size);
    TETHER_LOGI(tag, "    protocols            = 0x{:04X} (word 0x001C) → Supported mailbox protocols",
             (unsigned)protocols);
    
    TETHER_LOGI(tag, "\n📋 STEP 4: Protocol Flag Decoding");
    TETHER_LOGI(tag, "  Protocol value: 0x{:04X} ({})", (unsigned)protocols, getMailboxProtocolName(protocols));
    TETHER_LOGI(tag, "  Bit breakdown:");
    TETHER_LOGI(tag, "    Bit 0 (0x0001): AoE (ADS over EtherCAT)  = {}", (protocols & MBX_PROTO_AOE) ? "✓ Supported" : "✗ Not supported");
    TETHER_LOGI(tag, "    Bit 1 (0x0002): EoE (Ethernet over EtherCAT) = {}", (protocols & MBX_PROTO_EOE) ? "✓ Supported" : "✗ Not supported");
    TETHER_LOGI(tag, "    Bit 2 (0x0004): CoE (CANopen over EtherCAT) = {}", (protocols & MBX_PROTO_COE) ? "✓ Supported" : "✗ Not supported");
    TETHER_LOGI(tag, "    Bit 3 (0x0008): FoE (File over EtherCAT)   = {}", (protocols & MBX_PROTO_FOE) ? "✓ Supported" : "✗ Not supported");
    TETHER_LOGI(tag, "    Bit 4 (0x0010): SoE (Servo over EtherCAT)  = {}", (protocols & MBX_PROTO_SOE) ? "✓ Supported" : "✗ Not supported");
    TETHER_LOGI(tag, "    Bit 5 (0x0020): VoE (Vendor over EtherCAT) = {}", (protocols & MBX_PROTO_VOE) ? "✓ Supported" : "✗ Not supported");
    
    TETHER_LOGI(tag, "\n📋 STEP 5: Sync Manager Mapping (EtherCAT Convention)");
    TETHER_LOGI(tag, "  Per EtherCAT spec ETG.2010:");
    TETHER_LOGI(tag, "    SM0 = Receive mailbox (MbxIn) = Master→Slave direction");
    TETHER_LOGI(tag, "    SM1 = Send mailbox (MbxOut) = Slave→Master direction");
    TETHER_LOGI(tag, "  SII terminology mapping:");
    TETHER_LOGI(tag, "    std_rx  (SII word 0x0018/0x0019) → SM0 (Receive/MbxIn/Master→Slave)");
    TETHER_LOGI(tag, "    std_tx  (SII word 0x001A/0x001B) → SM1 (Send/MbxOut/Slave→Master)");
    
    TETHER_LOGI(tag, "\n📋 STEP 6: Final Mailbox Configuration");
    TETHER_LOGI(tag, "  SM0 (Receive/MbxIn/Master→Slave):");
    TETHER_LOGI(tag, "    Address = 0x{:04X} (from std_rx_offset at word 0x0018)", (unsigned)std_rx_offset);
    TETHER_LOGI(tag, "    Size    = {} bytes (from std_rx_size at word 0x0019)", (unsigned)std_rx_size);
    
    TETHER_LOGI(tag, "  SM1 (Send/MbxOut/Slave→Master):");
    TETHER_LOGI(tag, "    Address = 0x{:04X} (from std_tx_offset at word 0x001A)", (unsigned)std_tx_offset);
    TETHER_LOGI(tag, "    Size    = {} bytes (from std_tx_size at word 0x001B)", (unsigned)std_tx_size);
    
    TETHER_LOGI(tag, "  Supported Protocols: 0x{:04X} ({})", (unsigned)protocols, getMailboxProtocolName(protocols));
    
    TETHER_LOGI(tag, "\n📋 STEP 7: Validation Checks");
    bool has_mailbox = (std_rx_size > 0 && std_tx_size > 0);
    TETHER_LOGI(tag, "  Has valid mailbox: {}", has_mailbox ? "✓ Yes" : "✗ No");
    
    if (has_mailbox) {
        if (std_rx_size == 0 || std_tx_size == 0) {
            TETHER_LOGW(tag, "  ⚠ WARNING: Zero-size mailbox detected");
        }
        if (std_rx_size < 32 || std_tx_size < 32) {
            TETHER_LOGW(tag, "  ⚠ WARNING: Unusually small mailbox (< 32 bytes)");
        }
        if (std_rx_offset == std_tx_offset) {
            TETHER_LOGW(tag, "  ⚠ WARNING: RX and TX mailboxes have same address");
        }
        if (std_rx_offset >= std_tx_offset) {
            TETHER_LOGW(tag, "  ⚠ WARNING: Non-standard address ordering (SM1 >= SM0)");
        }
    }
    
    TETHER_LOGI(tag, "\n╔══════════════════════════════════════════════════════════════╗\n║  End of SII Mailbox Derivation Debug                          ║\n╚══════════════════════════════════════════════════════════════╝\n");
}

void logSIISummary(const SIIData& data, std::string_view log_prefix, const char* tag) {
    TETHER_LOGI(tag, "{}: {} (Vendor=0x{:08x} Product=0x{:08x}) "
             "SM:{} RxPDO:{}/{}B TxPDO:{}/{}B {}",
             std::string(log_prefix).c_str(),
             data.deviceName(),
             data.identity.vendor_id,
             data.identity.product_code,
             data.sm_count,
             data.rx_pdos.size(), data.totalRxPDOBytes(),
             data.tx_pdos.size(), data.totalTxPDOBytes(),
             getMailboxProtocolName(data.mailbox.protocols));
}

} // namespace SII
} // namespace EtherCAT
