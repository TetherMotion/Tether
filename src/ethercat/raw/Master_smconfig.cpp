/**
 * @file Master_smconfig.cpp
 * @brief Master — process-data sync-manager configuration from SII.
 *
 * TU split out of Master_slave.cpp.
 */

#include "tether/ethercat/Master.hpp"
#include "tether/ethercat/Slave.hpp"
#include "tether/ethercat/PDOManager.hpp"
#include "tether/ethercat/LogicalAddressManager.hpp"
#include "tether/ethercat/CoEManager.hpp"
#include "tether/ethercat/DebugFlags.hpp"
#include "tether/sii/SIIParser.hpp"
#include "tether/fmmu/FMMUConfiguration.hpp"
#include "raw/internal.hpp"
#include "raw/SlaveRegistry.hpp"
#include "tether/platform/Platform.hpp"
#include <cstring>
#include <format>
#include "tether/ethercat/SyncManager.hpp"

namespace EtherCAT {

static const char* TAG = "ethercat";

bool Master::configureProcessDataSyncManagersFromSii(SlaveAddress slave_address)
{
    uint16_t slave_index = 0;
    if (!resolvePhysicalSlaveIndex(slave_address, slave_index)) {
        return false;
    }

    if (slave_index >= PDO::kMaxPDOSlaves) {
        TETHER_LOGE(TAG, "configureProcessDataSyncManagersFromSii: invalid slave index {}", slave_index);
        return false;
    }

    // Read full SII data (includes SM category with SM2/SM3)
    EtherCAT::SII::SIIData sii;
    bool sii_valid = EtherCAT::SII::readSII(*this, slave_index, sii);
    if (!sii_valid) {
        TETHER_LOGW(TAG, "configureProcessDataSyncManagersFromSii: SII read failed for {}, using fallback", slaveLogPrefix(slave_index).c_str());
    }

    auto& pdo = pdoForSlave(slave_index);
    auto* slave_configs = pdo.slaveConfigs();
    bool configured_any = false;

    for (size_t i = 2; sii_valid && i < 4 && i < sii.sm_count; i++) {
        const auto& src = sii.sync_managers[i];
        auto& dst = slave_configs[slave_index].sm[i];

        if (src.phys_start_address == 0 && src.length == 0) {
            TETHER_LOGD(TAG, "SM{}: SII has no data (addr=0 len=0), skipping", i);
            continue;
        }

        dst.phys_start_addr = src.phys_start_address;
        dst.length = src.length;
        dst.control = src.control_register;
        dst.enable = src.isEnabled();
        dst.type = static_cast<PDO::SyncManagerType>(src.sm_type);

        TETHER_LOGI(TAG, "SM{} from SII: Addr=0x{:04X} Len={} Ctrl=0x{:02X} Type={} Enable={}",
                 i, dst.phys_start_addr, dst.length, std::bit_cast<uint8_t>(dst.control),
                 src.getTypeName(), dst.enable ? "yes" : "no");
        configured_any = true;
    }

    if (!configured_any) {
        TETHER_LOGW(TAG, "SII has no SM2/SM3 data for {}, trying HW registers", slaveLogPrefix(slave_index).c_str());

        for (uint8_t sm = 2; sm < 4; sm++) {
            uint16_t base = static_cast<uint16_t>(0x0800 + sm * 8);
            uint8_t buf[8] = {0};
            if (readRegister(SlaveAddress(slave_index), base, buf, sizeof(buf), 200)) {
                uint16_t addr = static_cast<uint16_t>(buf[0] | (buf[1] << 8));
                uint16_t len  = static_cast<uint16_t>(buf[2] | (buf[3] << 8));
                uint8_t  ctrl = buf[4];
                uint8_t  act  = buf[6];

                if (addr != 0) {
                    auto& dst = slave_configs[slave_index].sm[sm];
                    dst.phys_start_addr = addr;
                    dst.length = len;
                    dst.control = std::bit_cast<EtherCAT::SyncManager::SMControlReg>(ctrl);
                    dst.enable = (act & 0x01) != 0;
                    dst.type = (sm == 2) ? PDO::SyncManagerType::ProcessOutput
                                         : PDO::SyncManagerType::ProcessInput;
                    TETHER_LOGI(TAG, "SM{} from HW regs: Addr=0x{:04X} Len={} Ctrl=0x{:02X} Act=0x{:02X}",
                             sm, addr, len, ctrl, act);
                    // configured_any = true; // Not used
                }
            }
        }
    }

    if (slave_configs[slave_index].sm[2].phys_start_addr == 0) {
        auto& sm2 = slave_configs[slave_index].sm[2];
        sm2 = PDO::SyncManagerConfig::process_output(0x1800, 0);
        TETHER_LOGW(TAG, "SM2 (RxPDO) using DEFAULT: Addr=0x{:04X} Ctrl=0x{:02X}", sm2.phys_start_addr, std::bit_cast<uint8_t>(sm2.control));
    }
    if (slave_configs[slave_index].sm[3].phys_start_addr == 0) {
        auto& sm3 = slave_configs[slave_index].sm[3];
        sm3 = PDO::SyncManagerConfig::process_input(0x1C00, 0);
        TETHER_LOGW(TAG, "SM3 (TxPDO) using DEFAULT: Addr=0x{:04X} Ctrl=0x{:02X}", sm3.phys_start_addr, std::bit_cast<uint8_t>(sm3.control));
    }

    pdo.finalizeMapping(slave_index);

    // Phase 1: Write SM2/SM3 registers with activation DISABLED.
    // The ESC must see valid Sync Manager state before FMMUs reference them,
    // but we must NOT enable the SMs yet — doing so starts the SM watchdog
    // on strict ESCs (e.g. HMS Anybus). If the watchdog trips before FMMUs
    // are configured, subsequent register accesses may be rejected.
    for (uint8_t sm = 2; sm < 4; sm++) {
        const auto& cfg = slave_configs[slave_index].sm[sm];
        if (cfg.type != PDO::SyncManagerType::Unused && cfg.phys_start_addr != 0) {
            uint16_t base = static_cast<uint16_t>(0x0800 + sm * 8);

            uint8_t disable = 0x00;
            writeRegister(SlaveAddress(slave_index), static_cast<uint16_t>(base + 6), &disable, 1, 200);

            uint16_t addr_le = Raw::host_to_le16(cfg.phys_start_addr);
            writeRegister(SlaveAddress(slave_index), base, &addr_le, 2, 200);

            uint16_t len_le = Raw::host_to_le16(cfg.length);
            writeRegister(SlaveAddress(slave_index), static_cast<uint16_t>(base + 2), &len_le, 2, 200);

            // Keep the ESI watchdog-enable bit: firmware-driven ESCs validate
            // the SM control byte at PRE_OP->SAFE_OP and reject a cleared
            // watchdog bit with AL 0x0017.  Latched SM watchdogs at the
            // SAFE_OP->OP retry are handled by Slave::transitionToOp().
            uint8_t ctrl_byte = std::bit_cast<uint8_t>(cfg.control);
            writeRegister(SlaveAddress(slave_index), static_cast<uint16_t>(base + 4), &ctrl_byte, 1, 200);

            TETHER_LOGI(TAG, "Wrote SM{} to {}: Addr=0x{:04X} Len={} Ctrl=0x{:02X} Act=0x00 (disabled)",
                     sm, slaveLogPrefix(slave_index).c_str(), cfg.phys_start_addr, cfg.length, ctrl_byte);
        }
    }

    // Build/rebuild the logical address map from all configured slaves.
    // Use discovered slave count since this is called per-slave during discovery
    // before pdo_->slaveCount() has been set.
    auto& lam = logicalAddressManagerForSlave(slave_index);
    lam.buildAddressMap(pdo.slaveConfigs(),
                        getDiscoveredSlaveCount());

    // Phase 2: Configure FMMUs while SMs are still disabled.
    auto& fmmu_mgr = slave(slave_index).fmmuManager();
    const bool fmmu_dbg = debug_flags_.fmmu && debug_flags_.fmmuFilt.allows(slave_index);
    if (sii_valid && lam.hasSlavePDOs(slave_index)) {
        uint32_t rx_log = lam.getRxPDOLogicalAddr(slave_index);
        uint16_t rx_len = lam.getRxPDOLength(slave_index);
        uint32_t tx_log = lam.getTxPDOLogicalAddr(slave_index);
        uint16_t tx_len = lam.getTxPDOLength(slave_index);

        fmmu_mgr.configureManual(
            slave_configs[slave_index].sm[2].phys_start_addr, rx_len, rx_log,
            slave_configs[slave_index].sm[3].phys_start_addr, tx_len, tx_log,
            fmmu_dbg);
        if (!fmmu_mgr.writeToSlave(fmmu_dbg)) {
            TETHER_LOGE(TAG, "{}: FMMU write (manual) failed", slaveLogPrefix(slave_index).c_str());
            return false;
        }
    } else if (sii_valid) {
        fmmu_mgr.configureFromSii(&sii, &slave_configs[slave_index], 0, fmmu_dbg);
        if (!fmmu_mgr.writeToSlave(fmmu_dbg)) {
            TETHER_LOGE(TAG, "{}: FMMU write (from SII) failed", slaveLogPrefix(slave_index).c_str());
            return false;
        }
    } else {
        TETHER_LOGW(TAG, "{}: SII unavailable — FMMU not configured", slaveLogPrefix(slave_index).c_str());
    }

    // Phase 3: Enable SM2/SM3 NOW that FMMUs are configured.
    // This starts the SM watchdog — must happen as late as possible.
    for (uint8_t sm = 2; sm < 4; sm++) {
        const auto& cfg = slave_configs[slave_index].sm[sm];
        if (cfg.type != PDO::SyncManagerType::Unused && cfg.phys_start_addr != 0 && cfg.enable) {
            uint16_t base = static_cast<uint16_t>(0x0800 + sm * 8);
            uint8_t activate = 0x01;
            writeRegister(SlaveAddress(slave_index), static_cast<uint16_t>(base + 6), &activate, 1, 200);
            TETHER_LOGI(TAG, "Enabled SM{} on {}: Act=0x{:01X}",
                     sm, slaveLogPrefix(slave_index).c_str(), activate);
        }
    }

    slave_configs[slave_index].configured = true;
    return true;
}
} // namespace EtherCAT

