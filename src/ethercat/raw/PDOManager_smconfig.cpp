/**
 * @file PDOManager_smconfig.cpp
 * @brief PDOManager — sync-manager register configuration.
 *
 * TU split out of PDOManager.cpp.
 */

#include "tether/ethercat/PDOManager.hpp"
#include "raw/PDOManagerInternal.hpp"
#include "tether/ethercat/LogicalAddressManager.hpp"
#include "tether/utils/ColoredBitsetFormatter.hpp"

#include <algorithm>
#include <bit>
#include <cstring>
#include <format>
#include <vector>

namespace EtherCAT {

// ============================================================================
// SM Configuration helpers (formerly in sync_manager.cpp)
// ============================================================================

bool PDOManager::writeSMConfig(uint16_t adp, uint8_t sm_index,
                               const PDO::SyncManagerConfig& config,
                               uint16_t slave_index)
{
    const uint16_t base = sm_base_address(sm_index);

    // Control — keep the ESI watchdog-enable bit verbatim: firmware-driven
    // ESCs validate the SM control byte at the PRE_OP->SAFE_OP transition
    // and reject a cleared watchdog bit with AL 0x0017 "Invalid Sync
    // Manager configuration".  A latched SM watchdog at OP is handled by
    // the disarm fallback in Slave::transitionToOp().
    uint8_t ctrl_byte = std::bit_cast<uint8_t>(config.control);

    // Disable the SM first — the ESC rejects phys-addr/length writes on
    // an active channel (mailbox SMs may already be running from the
    // EEPROM bootstrap).
    uint8_t disable = 0x00;
    if (!transport_.writeRegister(adp, static_cast<uint16_t>(base + SM_OFF_ACTIVATE),
                                  &disable, sizeof(disable), 200)) {
        TETHER_LOGW(TAG, "SM{}: failed to disable", sm_index);
    }

    // Then write the configurable SM registers (phys addr + length +
    // control, bytes 0-4) in a single datagram — replaces the previous
    // per-field round-trips.  Byte 5 (status) is read-only and byte 7
    // (PDI control) must NOT be written: zeroing it breaks the SM
    // programming on firmware-driven ESCs (AL 0x0017 "Invalid Sync
    // Manager configuration" on the SAFE_OP request).  Activation
    // happens separately below so the channel comes up with a
    // fully-programmed block.
    uint8_t block[5] = {};
    const uint16_t addr_le = host_to_le16(config.phys_start_addr);
    const uint16_t len_le  = host_to_le16(config.length);
    std::memcpy(block + 0, &addr_le, sizeof(addr_le));
    std::memcpy(block + 2, &len_le, sizeof(len_le));
    block[4] = ctrl_byte;
    if (!transport_.writeRegister(adp, base, block, sizeof(block), 200)) {
        TETHER_LOGE(TAG, "SM{}: failed to write register block "
                         "(addr=0x{:04x} len={} ctrl=0x{:02x})",
                    sm_index, config.phys_start_addr, config.length, ctrl_byte);
        return false;
    }

    {
        const bool is_mbx = (config.type == PDO::SyncManagerType::MailboxWrite ||
                             config.type == PDO::SyncManagerType::MailboxRead);
        const bool dbg = is_mbx ? mailboxCfgDebug(slave_index)
                                : pdoSmDebug(slave_index);
        if (dbg) {
            TETHER_LOGI(TAG, "{}: SM{}: configured addr=0x{:04x} len={} ctrl=0x{:02x} act=0x{:02x}",
                        slavePrefix(slave_index).c_str(), sm_index, config.phys_start_addr, config.length, ctrl_byte,
                        config.enable ? SM_ACT_ENABLE : 0x00);
        }
    }

    if ((rxPDODebug() && config.type == PDO::SyncManagerType::ProcessOutput) ||
        (txPDODebug() && config.type == PDO::SyncManagerType::ProcessInput)) {
        const char* sm_type_str = "unknown";
        switch (config.type) {
            case PDO::SyncManagerType::Unused:        sm_type_str = "unused"; break;
            case PDO::SyncManagerType::MailboxWrite:  sm_type_str = "mailbox-write"; break;
            case PDO::SyncManagerType::MailboxRead:   sm_type_str = "mailbox-read"; break;
            case PDO::SyncManagerType::ProcessOutput: sm_type_str = "process-output (RxPDO)"; break;
            case PDO::SyncManagerType::ProcessInput:  sm_type_str = "process-input (TxPDO)"; break;
        }
        const char* mode_str = (config.control.mode == static_cast<uint8_t>(EtherCAT::SyncManager::SMMode::Mailbox)) ? "mailbox" :
                               (config.control.mode == static_cast<uint8_t>(EtherCAT::SyncManager::SMMode::Buffered)) ? "buffered" : "unknown";
        const char* dir_str  = config.control.direction ? "write (master→slave)" : "read (slave→master)";
        TETHER_LOGI(TAG, "  [PDO-DEBUG] SM{} detail: type={} mode={} dir={} enable={}",
                    sm_index, sm_type_str, mode_str, dir_str,
                    config.enable ? "yes" : "no");
        if (config.control.ecat_irq) TETHER_LOGI(TAG, "    IRQ eCAT enabled");
        if (config.control.pdi_irq) TETHER_LOGI(TAG, "    IRQ PDI enabled");
        if (config.control.watchdog) TETHER_LOGI(TAG, "    Watchdog enabled");
    }

    return true;
}

bool PDOManager::readSMStatus(uint16_t adp, uint8_t sm_index, uint8_t& status) {
    const uint16_t addr = static_cast<uint16_t>(sm_base_address(sm_index) + SM_OFF_STATUS);
    return transport_.readRegister(adp, addr, &status, sizeof(status), 200);
}

// ============================================================================
// SM Configuration public API
// ============================================================================

bool PDOManager::configureSlavesSMs(uint16_t slave_index) {
    if (slave_index >= PDO::kMaxPDOSlaves) {
        TETHER_LOGE(TAG, "Invalid slave index {}", slave_index);
        return false;
    }
    PDO::SlaveConfig& cfg = slave_configs_[slave_index];
    const uint16_t adp = transport_.adpForSlaveIndex(slave_index);

    if (pdoSmDebug(slave_index) || mailboxCfgDebug(slave_index)) {
        TETHER_LOGI(TAG, "Configuring SMs for {} (adp=0x{:04x})", slavePrefix(slave_index).c_str(), adp);
    }

    if (rxPDODebug(slave_index) || txPDODebug(slave_index)) {
        TETHER_LOGI(TAG, "  [PDO-DEBUG] {} config: vendor=0x{:08x} product=0x{:08x}",
                    slavePrefix(slave_index).c_str(), cfg.vendor_id, cfg.product_code);
        for (int sm = 0; sm < 4; sm++) {
            const char* sm_type_str = "unused";
            switch (cfg.sm[sm].type) {
                case PDO::SyncManagerType::MailboxWrite:  sm_type_str = "mailbox-write"; break;
                case PDO::SyncManagerType::MailboxRead:   sm_type_str = "mailbox-read"; break;
                case PDO::SyncManagerType::ProcessOutput: sm_type_str = "process-output (RxPDO)"; break;
                case PDO::SyncManagerType::ProcessInput:  sm_type_str = "process-input (TxPDO)"; break;
                default: break;
            }
            if (cfg.sm[sm].type != PDO::SyncManagerType::Unused) {
                TETHER_LOGI(TAG, "  [PDO-DEBUG]   SM{}: addr=0x{:04x} len={} ctrl=0x{:02x} type={} enable={}",
                            sm, cfg.sm[sm].phys_start_addr, cfg.sm[sm].length,
                            std::bit_cast<uint8_t>(cfg.sm[sm].control), sm_type_str,
                            cfg.sm[sm].enable ? "yes" : "no");
            } else {
                TETHER_LOGI(TAG, "  [PDO-DEBUG]   SM{}: unused", sm);
            }
        }
    }

    for (int sm = 0; sm < 4; sm++) {
        if (cfg.sm[sm].type != PDO::SyncManagerType::Unused) {
            if (!writeSMConfig(adp, static_cast<uint8_t>(sm), cfg.sm[sm], slave_index)) {
                TETHER_LOGE(TAG, "Failed to configure SM{} for {}", sm, slavePrefix(slave_index).c_str());
                return false;
            }
        }
    }

    // Activate all enabled SMs in a single frame — one 1-byte APWR per
    // SM with pre-registered waiters, instead of one round-trip each.
    // Falls back to sequential activate writes when the transport can't
    // pre-register waiters or the batched send/wait fails (activation
    // writes are idempotent, so a partial batched send is safe to redo).
    {
        uint8_t act_vals[4];
        MultiDatagramSpec specs[4];
        RxDatagram resps[4];
        size_t slots[4];
        size_t n = 0;
        bool batch_ok = true;
        for (int sm = 0; sm < 4; ++sm) {
            if (cfg.sm[sm].type == PDO::SyncManagerType::Unused) continue;
            act_vals[sm] = cfg.sm[sm].enable ? SM_ACT_ENABLE : 0x00;
            const uint8_t idx = transport_.allocIdx();
            const size_t slot = transport_.preRegisterResponseWaiter(
                idx, resps[n].data, sizeof(resps[n].data));
            if (slot == IPDOTransport::kPreRegInvalid) { batch_ok = false; break; }
            slots[n] = slot;
            specs[n] = {Command::APWR, idx, adp,
                        static_cast<uint16_t>(
                            sm_base_address(static_cast<uint8_t>(sm)) +
                            SM_OFF_ACTIVATE),
                        &act_vals[sm], 1, true};
            ++n;
        }
        if (batch_ok && n > 0) {
            if (transport_.sendMultiDatagram(specs, n) == 0) {
                TETHER_LOGW(TAG, "SM activate frame send failed for {} — "
                                 "falling back to sequential writes",
                            slavePrefix(slave_index).c_str());
                batch_ok = false;
            } else {
                for (size_t i = 0; i < n; ++i) {
                    if (!transport_.waitForPreRegistered(slots[i], 200, resps[i])) {
                        TETHER_LOGW(TAG, "SM{}: activate not confirmed for {} — "
                                         "falling back to sequential writes",
                                    i, slavePrefix(slave_index).c_str());
                        batch_ok = false;
                    }
                }
            }
        }
        if (!batch_ok) {
            // Release any pre-registered slots before their response
            // buffers (resps[]) go out of scope.
            for (size_t i = 0; i < n; ++i)
                transport_.cancelPreRegistered(slots[i]);
            // Fallback: sequential activate writes.
            for (int sm = 0; sm < 4; ++sm) {
                if (cfg.sm[sm].type == PDO::SyncManagerType::Unused) continue;
                act_vals[sm] = cfg.sm[sm].enable ? SM_ACT_ENABLE : 0x00;
                if (!transport_.writeRegister(
                        adp,
                        static_cast<uint16_t>(
                            sm_base_address(static_cast<uint8_t>(sm)) +
                            SM_OFF_ACTIVATE),
                        &act_vals[sm], 1, 200)) {
                    TETHER_LOGE(TAG, "SM{}: failed to activate for {}",
                                sm, slavePrefix(slave_index).c_str());
                    return false;
                }
            }
        }
    }

    cfg.configured = true;
    return true;
}

uint16_t PDOManager::configureAllSlaveSMs(uint16_t slave_count) {
    uint16_t configured = 0;
    for (uint16_t i = 0; i < slave_count && i < PDO::kMaxPDOSlaves; i++) {
        if (configureSlavesSMs(i)) configured++;
    }
    slave_count_ = slave_count;
    TETHER_LOGI(TAG, "Configured {}/{} slaves", configured, slave_count);
    return configured;
}

} // namespace EtherCAT
