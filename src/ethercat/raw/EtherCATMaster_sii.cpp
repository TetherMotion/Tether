/**
 * @file EtherCATMaster_sii.cpp
 * @brief EtherCATMaster — SII/EEPROM access, mailbox auto-configuration and discovery summary
 */

#include "tether/ethercat/EtherCATMaster.hpp"
#include "tether/ethercat/EtherCATSlave.hpp"
#include "tether/ethercat/EtherCATDC.hpp"
#include "tether/ethercat/EtherCATPDO.hpp"
#include "tether/ethercat/EtherCATSDO.hpp"
#include "tether/ethercat/EtherCATFoE.hpp"
#include "tether/ethercat/EtherCATVoE.hpp"
#include "tether/ethercat/EtherCATEoE.hpp"
#include "tether/ethercat/EtherCATFaultDetection.hpp"
#include "tether/ethercat/EtherCATRealtimeLoop.hpp"
#include "tether/ethercat/SyncManagerValidation.hpp"
#include "tether/sii/SIIParser.hpp"
#include "tether/fmmu/FMMUConfiguration.hpp"
#include "raw/internal.hpp"
#include "tether/platform/Platform.hpp"

#include <thread>
#include <chrono>
#include <algorithm>
#include <cstdio>
#include <cstring>
#include <cerrno>
#include "sii/SIIReader.hpp"
#include <inttypes.h>

namespace EtherCAT {

static const char* TAG = "ethercat";

// Global debug flag for ethercat-statemachine (shared with EtherCATSlave)
extern bool g_debug_statemachine;

// Global debug flags for tx/rx packet logging (shared with EtherCATSlave)
extern bool g_debug_tx_packets;
extern bool g_debug_rx_packets;

// Global debug flags for PDO logging (shared with PDOManager)
extern bool g_debug_rx_pdo;
extern bool g_debug_tx_pdo;

// ============================================================================
// SII / EEPROM — delegate to existing Raw:: functions for now
// ============================================================================

bool EtherCATMaster::siiReadString(uint16_t slave_index, uint16_t string_number,
                                    char* out, size_t out_cap)
{
    return Raw::sii_read_string(*this, slave_index, string_number, out, out_cap);
}

bool EtherCATMaster::configureMailboxFromSii(uint16_t slave_index,
                                              uint16_t* wr_addr, uint16_t* wr_len,
                                              uint16_t* rd_addr, uint16_t* rd_len,
                                              uint16_t* mbx_proto)
{
    return Raw::configure_mailbox_from_sii(*this, slave_index,
                                           wr_addr, wr_len, rd_addr, rd_len,
                                           mbx_proto);
}

bool EtherCATMaster::autoConfigureMailbox(SlaveAddress slave_address, Tether::Platform::LogLevel log_level)
{
    const char* local_tag = "autoMbox";
    uint16_t slave_index = 0;
    if (!resolvePhysicalSlaveIndex(slave_address, slave_index)) {
        return false;
    }

    uint16_t adp = adpForSlaveIndex(slave_index);
    
    // Log start with requested verbosity
    if (log_level >= Tether::Platform::LogLevel::Debug) {
        TETHER_LOGD(local_tag, "======================================================================\n  AUTO-CONFIGURING MAILBOX FOR SLAVE %u (ADP=0x%04X)\n======================================================================",
                    (unsigned)slave_index, adp);
    } else {
        TETHER_LOGI(local_tag, "Auto-configuring mailbox for slave %u...", (unsigned)slave_index);
    }
    
    // Step 1: Read mailbox configuration from SII
    uint16_t wr_addr = 0, wr_len = 0, rd_addr = 0, rd_len = 0, proto = 0;
    
    if (log_level >= Tether::Platform::LogLevel::Debug) {
        TETHER_LOGD(local_tag, "[1/3] Reading mailbox configuration from SII EEPROM...");
    }
    
    bool sii_ok = configureMailboxFromSii(adp, &wr_addr, &wr_len, &rd_addr, &rd_len, &proto);
    
    if (!sii_ok) {
        TETHER_LOGE(local_tag, "Failed to read SII mailbox configuration for slave %u", 
                    (unsigned)slave_index);
        // Note: configureMailboxFromSii already sets defaults on failure, so we can continue
    }
    
    // Step 2: Apply mailbox override to master    
    if (log_level >= Tether::Platform::LogLevel::Debug) {
        TETHER_LOGD(local_tag, "[2/3] Applying mailbox configuration to master override...\n      Send    (S→M, SM0): addr=0x%04X len=%u\n      Receive (M→S, SM1): addr=0x%04X len=%u\n      Protocols: 0x%04X",
                   rd_addr, (unsigned)rd_len, wr_addr, (unsigned)wr_len, proto);
        
        // Decode protocols for verbose logging
        std::string proto_str;
        if (proto & static_cast<uint16_t>(SII::MBX_PROTO_AOE)) proto_str += "AoE ";
        if (proto & static_cast<uint16_t>(SII::MBX_PROTO_EOE)) proto_str += "EoE ";
        if (proto & static_cast<uint16_t>(SII::MBX_PROTO_COE)) proto_str += "CoE ";
        if (proto & static_cast<uint16_t>(SII::MBX_PROTO_FOE)) proto_str += "FoE ";
        if (proto & static_cast<uint16_t>(SII::MBX_PROTO_SOE)) proto_str += "SoE ";
        if (proto & static_cast<uint16_t>(SII::MBX_PROTO_VOE)) proto_str += "VoE ";
        if (proto_str.empty()) proto_str = "(none)";
        
        TETHER_LOGD(local_tag, "      Supported: %s", proto_str.c_str());
    }
    
    setMailboxOverride(slave_address, wr_addr, wr_len, rd_addr, rd_len, proto);
    
    if (log_level >= Tether::Platform::LogLevel::Debug) {
        TETHER_LOGD(local_tag, "      ✓ Master mailbox override configured");
    }
    
    // Step 3: Configure SDO subsystem mailbox
    if (log_level >= Tether::Platform::LogLevel::Debug) {
        TETHER_LOGD(local_tag, "[3/3] Configuring SDO subsystem mailbox...");
    }
    
    sdo_manager_->configureSlaveMailbox(slave_index, wr_addr, wr_len, rd_addr, rd_len);
    
    if (log_level >= Tether::Platform::LogLevel::Debug) {
        TETHER_LOGD(local_tag, "      ✓ SDO subsystem mailbox configured");
    }
    
    // Step 4: Verify configuration was applied
    if (log_level >= Tether::Platform::LogLevel::Debug) {
        TETHER_LOGD(local_tag, "[Verification] Checking SDO subsystem mailbox configuration...");
        
        uint16_t verify_wr = 0, verify_wr_len = 0, verify_rd = 0, verify_rd_len = 0;
        bool verify_ok = sdo_manager_->getSlaveMailbox(slave_index, 
                                                        &verify_wr, &verify_wr_len, 
                                                        &verify_rd, &verify_rd_len);
        
        if (verify_ok) {
            bool match = (verify_wr == wr_addr && verify_wr_len == wr_len && 
                         verify_rd == rd_addr && verify_rd_len == rd_len);
            
            if (match) {
                TETHER_LOGD(local_tag, "      ✓ SDO subsystem mailbox verified: Receive(SM0)=0x%04X/%u Send(SM1)=0x%04X/%u",
                           verify_wr, (unsigned)verify_wr_len, verify_rd, (unsigned)verify_rd_len);
            } else {
                TETHER_LOGW(local_tag, "      ⚠ MISMATCH! SDO subsystem has Receive(SM0)=0x%04X/%u Send(SM1)=0x%04X/%u\n      Expected: Receive(SM0)=0x%04X/%u Send(SM1)=0x%04X/%u",
                           verify_wr, (unsigned)verify_wr_len, verify_rd, (unsigned)verify_rd_len,
                           wr_addr, (unsigned)wr_len, rd_addr, (unsigned)rd_len);
            }
        } else {
            TETHER_LOGE(local_tag, "      ✗ CRITICAL: SDO subsystem has NO mailbox configuration!\n      This indicates configureSlaveMailbox() failed.");
            return false;
        }
    }
    
    // Final success message
    if (log_level >= Tether::Platform::LogLevel::Debug) {
        TETHER_LOGD(local_tag, "======================================================================\n  ✓ MAILBOX AUTO-CONFIGURATION COMPLETE FOR SLAVE %u\n======================================================================",
                    (unsigned)slave_index);
    } else {
        TETHER_LOGI(local_tag, "✓ Mailbox auto-configured for slave %u: Receive(SM0)=0x%04X/%u Send(SM1)=0x%04X/%u",
                    (unsigned)slave_index, wr_addr, (unsigned)wr_len, rd_addr, (unsigned)rd_len);
    }
    
    return true;
}
void EtherCATMaster::logDiscoveredSlavesSummary(const char* tag)
{
    const uint16_t n = getDiscoveredSlaveCount();
    TETHER_LOGI(tag, "Discovered %u slave(s)", n);

    for (uint16_t i = 0; i < n; ++i) {
        EtherCAT::SII::SIIData sii_data;
        if (EtherCAT::SII::readSII(*this, i, sii_data)) {
            // Detailed summary handled by SII module
            EtherCAT::SII::logSIISummary(sii_data, i, tag);
        }
        else {
            // Fallback: try reading identity only
            EtherCAT::SII::SIIIdentity id;
            if (EtherCAT::SII::readSIIIdentity(*this, i, id)) {
                char name_buf[64] = {0};
                if (!siiReadString(i, 1, name_buf, sizeof(name_buf)) &&
                    !siiReadString(i, 2, name_buf, sizeof(name_buf)) &&
                    !siiReadString(i, 3, name_buf, sizeof(name_buf))) {
                    strncpy(name_buf, "<unknown>", sizeof(name_buf));
                }

                TETHER_LOGI(tag, "Slave %u @ ADP=0x%04X Vendor=0x%08" PRIx32 " Product=0x%08" PRIx32 " Name='%s'",
                         i, adpForSlaveIndex(i), id.vendor_id, id.product_code, name_buf);
            }
            else {
                TETHER_LOGW(tag, "Slave %u @ ADP=0x%04X: unable to read SII/identity", i, adpForSlaveIndex(i));
            }
        }
    }
}

} // namespace EtherCAT
