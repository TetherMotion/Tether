/**
 * @file Master_sii.cpp
 * @brief Master — SII/EEPROM access, mailbox auto-configuration and discovery summary
 */

#include "tether/ethercat/Master.hpp"
#include "tether/ethercat/Slave.hpp"
#include "tether/ethercat/DC.hpp"
#include "tether/ethercat/PDOManager.hpp"
#include "tether/ethercat/SDOManager.hpp"
#include "tether/ethercat/CoEManager.hpp"
#include "tether/ethercat/FoE.hpp"
#include "tether/ethercat/VoE.hpp"
#include "tether/ethercat/EoE.hpp"
#include "tether/ethercat/FaultDetection.hpp"
#include "tether/ethercat/RealtimeLoop.hpp"
#include "tether/ethercat/SyncManagerValidation.hpp"
#include "tether/sii/SIIParser.hpp"
#include "tether/fmmu/FMMUConfiguration.hpp"
#include "raw/internal.hpp"
#include "tether/platform/Platform.hpp"

#include <thread>
#include <chrono>
#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cerrno>
#include "sii/SIIReader.hpp"
#include <inttypes.h>

namespace EtherCAT {

static const char* TAG = "ethercat";

// ============================================================================
// SII / EEPROM — delegate to existing Raw:: functions for now
// ============================================================================

bool Master::siiReadString(uint16_t slave_index, uint16_t string_number,
                                    char* out, size_t out_cap)
{
    return Raw::sii_read_string(*this, slave_index, string_number, out, out_cap);
}

bool Master::configureMailboxFromSii(uint16_t slave_index,
                                              uint16_t* wr_addr, uint16_t* wr_len,
                                              uint16_t* rd_addr, uint16_t* rd_len,
                                              uint16_t* mbx_proto)
{
    return Raw::configure_mailbox_from_sii(*this, slave_index,
                                           wr_addr, wr_len, rd_addr, rd_len,
                                           mbx_proto);
}

bool Master::autoConfigureMailbox(SlaveAddress slave_address, Tether::Platform::LogLevel log_level)
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
    
    sdoManager(slave_index).configureMailbox(wr_addr, wr_len, rd_addr, rd_len);
    
    if (log_level >= Tether::Platform::LogLevel::Debug) {
        TETHER_LOGD(local_tag, "      ✓ SDO subsystem mailbox configured");
    }
    
    // Step 4: Write mailbox SM registers to slave ESC
    if (slave_index < PDO::kMaxPDOSlaves) {
        auto* slave_configs = pdo_->slaveConfigs();
        slave_configs[slave_index].sm[0] = PDO::SyncManagerConfig::mailbox_write(wr_addr, wr_len);
        slave_configs[slave_index].sm[1] = PDO::SyncManagerConfig::mailbox_read(rd_addr, rd_len);

        if (pdo_->configureSlavesSMs(slave_index)) {
            uint8_t sm0_ctrl = 0, sm1_ctrl = 0;
            bool sm0_ok = readRegister(SlaveAddress(slave_index), static_cast<uint16_t>(Raw::EC_REG_SM0 + 0x04), sm0_ctrl, 200);
            bool sm1_ok = readRegister(SlaveAddress(slave_index), static_cast<uint16_t>(Raw::EC_REG_SM1 + 0x04), sm1_ctrl, 200);

            if (sm0_ok && sm1_ok) {
                if (sm0_ctrl == 0x26 && sm1_ctrl == 0x22) {
                    if (log_level >= Tether::Platform::LogLevel::Debug) {
                        TETHER_LOGD(local_tag, "[4/4] SM registers verified: SM0=0x%02X SM1=0x%02X", sm0_ctrl, sm1_ctrl);
                    }
                } else {
                    TETHER_LOGW(local_tag, "SM0=0x%02X SM1=0x%02X: expected 0x26/0x22 after writing mailbox SM config", sm0_ctrl, sm1_ctrl);
                }
            } else {
                TETHER_LOGW(local_tag, "Failed to read back SM0/SM1 control registers after configuration");
            }
        } else {
            TETHER_LOGE(local_tag, "Failed to write mailbox SM registers to slave %u", (unsigned)slave_index);
            return false;
        }
    }

    // Step 5: Verify configuration was applied
    if (log_level >= Tether::Platform::LogLevel::Debug) {
        TETHER_LOGD(local_tag, "[Verification] Checking SDO subsystem mailbox configuration...");
        
        uint16_t verify_wr = 0, verify_wr_len = 0, verify_rd = 0, verify_rd_len = 0;
        bool verify_ok = sdoManager(slave_index).getMailbox(
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
void Master::logDiscoveredSlavesSummary(const char* tag)
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

bool Master::verifySlaveIdentity(uint16_t slave_index,
                                   const Identity::SlaveIdentity& expected,
                                   bool exit_on_error,
                                   const char* tag)
{
    EtherCAT::SII::SIIIdentity id;
    if (!EtherCAT::SII::readSIIIdentity(*this, slave_index, id)) {
        TETHER_LOGE(tag, "Slave %u: Failed to read SII identity", slave_index);
        if (exit_on_error) {
            stop();
            std::exit(1);
        }
        return false;
    }

    bool ok = true;

    if (expected.vendor_id.has_value() && id.vendor_id != expected.vendor_id.value()) {
        TETHER_LOGE(tag, "Slave %u Vendor ID mismatch: expected 0x%08X, got 0x%08X",
                    slave_index, expected.vendor_id.value(), id.vendor_id);
        ok = false;
    }

    if (expected.product_code.has_value() && id.product_code != expected.product_code.value()) {
        TETHER_LOGE(tag, "Slave %u Product Code mismatch: expected 0x%08X, got 0x%08X",
                    slave_index, expected.product_code.value(), id.product_code);
        ok = false;
    }

    if (expected.revision_number.has_value() && id.revision_number != expected.revision_number.value()) {
        TETHER_LOGE(tag, "Slave %u Revision Number mismatch: expected 0x%08X, got 0x%08X",
                    slave_index, expected.revision_number.value(), id.revision_number);
        ok = false;
    }

    if (expected.serial_number.has_value() && id.serial_number != expected.serial_number.value()) {
        TETHER_LOGE(tag, "Slave %u Serial Number mismatch: expected 0x%08X, got 0x%08X",
                    slave_index, expected.serial_number.value(), id.serial_number);
        ok = false;
    }

    if (!ok && exit_on_error) {
        stop();
        std::exit(1);
    }

    return ok;
}

} // namespace EtherCAT
