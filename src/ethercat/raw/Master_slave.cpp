/**
 * @file Master_slave.cpp
 * @brief Master — Slave management, AL state, watchdog and mailbox fallback
 */

#include "tether/ethercat/Master.hpp"
#include "tether/ethercat/Slave.hpp"
#include "tether/ethercat/DC.hpp"
#include "tether/ethercat/PDOManager.hpp"
#include "tether/ethercat/LogicalAddressManager.hpp"
#include "tether/ethercat/SDOManager.hpp"
#include "tether/ethercat/CoEManager.hpp"
#include "tether/ethercat/FoE.hpp"
#include "tether/ethercat/VoE.hpp"
#include "tether/ethercat/EoE.hpp"
#include "tether/ethercat/FaultDetection.hpp"
#include "tether/ethercat/RealtimeLoop.hpp"
#include "tether/ethercat/SyncManagerValidation.hpp"
#include "tether/ethercat/DebugFlags.hpp"
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
#include <bit>
#include <vector>
#include "sii/SIIReader.hpp"
#include <inttypes.h>

namespace EtherCAT {

static const char* TAG = "ethercat";

// ============================================================================
// Helper: determine whether SM0 is configured as write in SII (DEPRECATED/UNUSED).
// EtherCAT standard mandates: SM0=MbxIn(master→slave/write), SM1=MbxOut(slave→master/read).
// Some device SII EEPROMs incorrectly specify reversed directions, but the master
// must ignore those errors and always configure according to the standard.
// This function is retained for diagnostic purposes only.
// ============================================================================
static bool siiMailboxSM0IsWrite(Master& master, uint16_t slave_index)
{
    EtherCAT::SII::SIIData sii;
    if (!EtherCAT::SII::readSII(master, slave_index, sii) || sii.sm_count < 2)
        return false;
    return sii.sync_managers[0].control_register.direction;
}
// ============================================================================
// Slave Management
// ============================================================================

void Master::initSlaves(uint16_t count)
{
    slaves_.clear();
    slaves_.reserve(count);
    {
        std::lock_guard<std::mutex> lock(sii_cache_mutex_);
        sii_word_caches_.clear();
        sii_word_caches_.resize(count);
    }
    for (uint16_t i = 0; i < count; ++i) {
        auto s = std::make_unique<Slave>(*this, i);
        // Initialize SII cache for each slave
        s->siiCache().init(siiReader(), i);
        slaves_.push_back(std::move(s));
    }
    // Bulk-prefetch the first 256 words of SII EEPROM for each slave.
    // This turns all subsequent SII reads (mailbox config, identity,
    // categories) into cache hits, eliminating repeated EEPSTAT polling.
    for (uint16_t i = 0; i < count; ++i) {
        (void)sii_reader_->prefetchWords(i, 0, 256);
    }
    // Resize filters to current slave count and push per-slave flags.
    debug_flags_.resizeFilters(count);
    updateDebugFlags();
}

bool Master::drainSlaveMailbox(uint16_t slave_index, unsigned int max_drain)
{
    const char* local_tag = "mbox_drain";

    uint16_t mbx_wr_addr = 0, mbx_wr_len = 0;
    uint16_t mbx_rd_addr = 0, mbx_rd_len = 0;
    if (!sdoManager(slave_index).getMailbox(&mbx_wr_addr, &mbx_wr_len,
                                            &mbx_rd_addr, &mbx_rd_len)) {
        TETHER_LOGW(local_tag, "Slave %u: cannot drain mailbox, no mailbox configured", slave_index);
        return false;
    }

    if (mbx_rd_len == 0) {
        TETHER_LOGW(local_tag, "Slave %u: cannot drain mailbox, read length is zero", slave_index);
        return false;
    }

    // Some ESCs only clear WRITE_BUF_FULL when the master reads the entire
    // configured mailbox length in one datagram. Allocate a buffer that matches
    // the SM length instead of capping at 256 bytes.
    std::vector<uint8_t> drain_buf(mbx_rd_len, 0);

    bool drained_any = false;
    for (unsigned int i = 0; i < max_drain; ++i) {
        uint8_t sm1_status = 0;
        if (!readRegister(SlaveAddress(slave_index), Raw::sm_status_address(1), sm1_status, 100)) {
            TETHER_LOGW(local_tag, "Slave %u: failed to read SM1 status while draining", slave_index);
            return false;
        }

        // SM1 (slave→master mailbox) signals unread data via the mailbox-full
        // flag (bit 3, 0x08) per ETG.1000.4. Bit 7 (WRITE_BUFFER_FULL) is only
        // meaningful for 3-buffer PDO SMs and must not be treated as "mailbox
        // full" — doing so causes premature reads (WKC=0) on slaves that
        // transiently set bit 7 before writing the response.
        if ((sm1_status & Raw::EC_SM_STATUS_MBXFULL) == 0) {
            if (drained_any) {
                TETHER_LOGI(local_tag, "Slave %u: SM1 drained successfully", slave_index);
            }
            return true;
        }

        if (!drained_any) {
            TETHER_LOGW(local_tag, "Slave %u: SM1 full at startup (status=0x%02X). Draining stale mailbox data...",
                        slave_index, sm1_status);
        }

        if (!readRegister(SlaveAddress(slave_index), mbx_rd_addr,
                          drain_buf.data(), static_cast<uint16_t>(drain_buf.size()), 200)) {
            TETHER_LOGW(local_tag, "Slave %u: SM1 drain read failed, attempting SM1 activate reset", slave_index);
            if (resetSlaveMailboxSM1(slave_index)) {
                // After a successful reset the buffer should be empty; re-check
                // before continuing so we don't loop on stale status.
                uint8_t sm1_status = 0;
                if (readRegister(SlaveAddress(slave_index), Raw::sm_status_address(1), sm1_status, 100) &&
                    (sm1_status & Raw::EC_SM_STATUS_MBXFULL) == 0) {
                    TETHER_LOGI(local_tag, "Slave %u: SM1 empty after reset", slave_index);
                    return true;
                }
            }
            return false;
        }
        TETHER_LOGW(local_tag, "Slave %u: drained stale mailbox data #%u (len=%u)",
                    slave_index, i + 1, static_cast<unsigned>(drain_buf.size()));
        drained_any = true;
    }

    // After max_drain reads, SM1 is still reporting full. The slave may be
    // continuously refilling the buffer or the ESC is not acknowledging the
    // reads; do not pretend the drain succeeded.
    TETHER_LOGE(local_tag, "Slave %u: SM1 still full after %u drain attempts", slave_index, max_drain);
    return false;
}

bool Master::resetSlaveMailboxSM1(uint16_t slave_index)
{
    const char* local_tag = "mbox_reset";
    const uint16_t sm1_activate_addr = static_cast<uint16_t>(Raw::EC_REG_SM1 + 0x06u);
    const uint16_t sm1_status_addr = Raw::sm_status_address(1);

    TETHER_LOGW(local_tag, "Slave %u: cycling SM1 activate register (0x%04X) to clear stuck WRITE_BUF_FULL",
                slave_index, sm1_activate_addr);

    uint8_t disable = 0x00;
    uint8_t enable = 0x01;
    if (!writeRegister(SlaveAddress(slave_index), RegisterAddress(sm1_activate_addr), &disable, sizeof(disable), 200)) {
        TETHER_LOGE(local_tag, "Slave %u: failed to disable SM1", slave_index);
        return false;
    }
    if (!writeRegister(SlaveAddress(slave_index), RegisterAddress(sm1_activate_addr), &enable, sizeof(enable), 200)) {
        TETHER_LOGE(local_tag, "Slave %u: failed to re-enable SM1", slave_index);
        return false;
    }

    uint8_t sm1_status = 0;
    if (!readRegister(SlaveAddress(slave_index), RegisterAddress(sm1_status_addr), sm1_status, 100)) {
        TETHER_LOGE(local_tag, "Slave %u: failed to read SM1 status after reset", slave_index);
        return false;
    }

    if ((sm1_status & Raw::EC_SM_STATUS_MBXFULL) != 0) {
        TETHER_LOGE(local_tag, "Slave %u: SM1 still full after activate reset (status=0x%02X)",
                    slave_index, sm1_status);
        return false;
    }

    TETHER_LOGI(local_tag, "Slave %u: SM1 reset successful", slave_index);
    return true;
}

bool Master::resetSlaveMailboxSM0(uint16_t slave_index)
{
    const char* local_tag = "mbox_reset";
    const uint16_t sm0_activate_addr = static_cast<uint16_t>(Raw::EC_REG_SM0 + 0x06u);
    const uint16_t sm0_status_addr = Raw::sm_status_address(0);

    TETHER_LOGW(local_tag, "Slave %u: cycling SM0 activate register (0x%04X) to clear stuck mailbox-full",
                slave_index, sm0_activate_addr);

    uint8_t disable = 0x00;
    uint8_t enable = 0x01;
    if (!writeRegister(SlaveAddress(slave_index), RegisterAddress(sm0_activate_addr), &disable, sizeof(disable), 200)) {
        TETHER_LOGE(local_tag, "Slave %u: failed to disable SM0", slave_index);
        return false;
    }
    if (!writeRegister(SlaveAddress(slave_index), RegisterAddress(sm0_activate_addr), &enable, sizeof(enable), 200)) {
        TETHER_LOGE(local_tag, "Slave %u: failed to re-enable SM0", slave_index);
        return false;
    }

    uint8_t sm0_status = 0;
    if (!readRegister(SlaveAddress(slave_index), RegisterAddress(sm0_status_addr), sm0_status, 100)) {
        TETHER_LOGE(local_tag, "Slave %u: failed to read SM0 status after reset", slave_index);
        return false;
    }

    if ((sm0_status & Raw::EC_SM_STATUS_MBXFULL) != 0) {
        TETHER_LOGE(local_tag, "Slave %u: SM0 still full after activate reset (status=0x%02X)",
                    slave_index, sm0_status);
        return false;
    }

    TETHER_LOGI(local_tag, "Slave %u: SM0 reset successful", slave_index);
    return true;
}

void Master::updateDebugFlags()
{
    const uint16_t count = static_cast<uint16_t>(slaves_.size());
    std::vector<EtherCATSlaveDebugFlags> per_slave_flags(count);
    for (uint16_t i = 0; i < count; ++i) {
        per_slave_flags[i] = debug_flags_.computeForSlave(i);
        if (slaves_[i]) {
            slaves_[i]->updateDebugFlags(per_slave_flags[i]);
        }
    }
    {
        std::lock_guard<std::mutex> lock(sdo_managers_mutex_);
        for (uint16_t i = 0; i < count; ++i) {
            if (i < sdo_managers_.size() && sdo_managers_[i]) {
                sdo_managers_[i]->updateDebugFlags(per_slave_flags[i]);
            }
        }
    }
    if (pdo_) {
        pdo_->setDebugFlags(&debug_flags_);
    }
}

Slave& Master::slave(uint16_t slave_index)
{
    if (slave_index < slaves_.size()) {
        return *slaves_[slave_index];
    }
    // Return sentinel for invalid index
    if (!non_existing_slave_) {
        non_existing_slave_ = std::make_unique<NonExistingSlave>(*this, slave_index);
    }
    // Update the index so the error message is correct
    non_existing_slave_ = std::make_unique<NonExistingSlave>(*this, slave_index);
    return *non_existing_slave_;
}

SII::SIIReader& Master::siiReader()
{
    if (!sii_reader_) {
        sii_reader_ = std::make_unique<SII::SIIReader>(*this);
    }
    return *sii_reader_;
}

bool Master::getSIICachedWord(uint16_t slave_index, uint16_t word_addr, uint16_t& out) const
{
    std::lock_guard<std::mutex> lock(sii_cache_mutex_);
    if (slave_index >= sii_word_caches_.size()) return false;
    const auto& cache = sii_word_caches_[slave_index];
    auto it = cache.find(word_addr);
    if (it == cache.end()) return false;
    out = it->second;
    return true;
}

void Master::setSIICachedWord(uint16_t slave_index, uint16_t word_addr, uint16_t value)
{
    std::lock_guard<std::mutex> lock(sii_cache_mutex_);
    if (slave_index >= sii_word_caches_.size()) return;
    sii_word_caches_[slave_index][word_addr] = value;
}

void Master::clearSIICache(uint16_t slave_index)
{
    std::lock_guard<std::mutex> lock(sii_cache_mutex_);
    if (slave_index >= sii_word_caches_.size()) return;
    sii_word_caches_[slave_index].clear();
}

bool Master::resolvePhysicalSlaveIndex(SlaveAddress slave_address, uint16_t& slave_index_out)
{
    if (!slave_address.isPhysical()) {
        TETHER_LOGE(TAG, "Operation requires a physical slave address");
        return false;
    }

    slave_index_out = slave_address.slavePosition();
    return true;
}

// ============================================================================
// Frame handling
// ============================================================================

void Master::handleRxFrame(const uint8_t* frame, size_t length)
{
    parseEtherCATFrame(frame, length);
}

// ============================================================================
// Discovery
// ============================================================================

uint16_t Master::getDiscoveredSlaveCount() const
{
    return discovered_slave_count_.load(std::memory_order_acquire);
}

// ============================================================================
// AL state management
// ============================================================================

bool Master::requestSlaveApplicationLayerState(SlaveAddress slave_address, uint8_t state_code)
{
    uint16_t slave_index = 0;
    if (slave_address.isPhysical()) slave_index = slave_address.slavePosition();

    if (debug_flags_.stateMachine && debug_flags_.stateMachineFilt.allows(slave_index)) {
        uint8_t current_state_code = 0;
        readSlaveApplicationLayerState(slave_address, current_state_code);
        const char* current_state_name = getECStateName(current_state_code);
        const char* target_state_name = getECStateName(state_code);
        
        TETHER_LOGI(TAG, "╔══════════════════════════════════════════════════════════════╗");
        TETHER_LOGI(TAG, "║  AL State Request: Slave %u                                  ║", slave_address.slavePosition());
        TETHER_LOGI(TAG, "╠══════════════════════════════════════════════════════════════╣");
        TETHER_LOGI(TAG, "║  Current State: %s (0x%02X)", current_state_name, current_state_code);
        TETHER_LOGI(TAG, "║  Target State:  %s (0x%02X)", target_state_name, state_code);
        TETHER_LOGI(TAG, "║  Action:        Writing AL_CONTROL register");
        TETHER_LOGI(TAG, "╚══════════════════════════════════════════════════════════════╝");
    }
    
    bool result = writeRegister(slave_address, RegisterAddress(Raw::EC_REG_AL_CONTROL), static_cast<uint16_t>(state_code));
    
    if (debug_flags_.stateMachine && debug_flags_.stateMachineFilt.allows(slave_index)) {
        TETHER_LOGI(TAG, "╔══════════════════════════════════════════════════════════════╗");
        TETHER_LOGI(TAG, "║  AL State Request Result: %s                                 ║", result ? "SUCCESS" : "FAILED");
        TETHER_LOGI(TAG, "╚══════════════════════════════════════════════════════════════╝");
    }
    
    return result;
}

bool Master::readSlaveApplicationLayerState(SlaveAddress slave_address, uint8_t& state_code)
{
    uint16_t application_layer_status = 0;
    if (!readRegister(slave_address, RegisterAddress(Raw::EC_REG_AL_STATUS), application_layer_status, 200)) {
        return false;
    }

    state_code = static_cast<uint8_t>(Raw::le16_to_host(application_layer_status) & 0x0F);
    return true;
}

bool Master::transitionSlaveToPreOperational(SlaveAddress slave_address)
{
    return setPreopAndConfirm(slave_address.slavePosition());
}

bool Master::configureProcessDataSyncManagersFromSii(SlaveAddress slave_address)
{
    uint16_t slave_index = 0;
    if (!resolvePhysicalSlaveIndex(slave_address, slave_index)) {
        return false;
    }

    if (slave_index >= PDO::kMaxPDOSlaves) {
        TETHER_LOGE(TAG, "configureProcessDataSyncManagersFromSii: invalid slave index %u", slave_index);
        return false;
    }

    // Read full SII data (includes SM category with SM2/SM3)
    EtherCAT::SII::SIIData sii;
    bool sii_valid = EtherCAT::SII::readSII(*this, slave_index, sii);
    if (!sii_valid) {
        TETHER_LOGW(TAG, "configureProcessDataSyncManagersFromSii: SII read failed for slave %u, using fallback", slave_index);
    }

    auto* slave_configs = pdo_->slaveConfigs();
    bool configured_any = false;

    for (size_t i = 2; sii_valid && i < 4 && i < sii.sm_count; i++) {
        const auto& src = sii.sync_managers[i];
        auto& dst = slave_configs[slave_index].sm[i];

        if (src.phys_start_address == 0 && src.length == 0) {
            TETHER_LOGD(TAG, "SM%zu: SII has no data (addr=0 len=0), skipping", i);
            continue;
        }

        dst.phys_start_addr = src.phys_start_address;
        dst.length = src.length;
        dst.control = src.control_register;
        dst.enable = src.isEnabled();
        dst.type = static_cast<PDO::SyncManagerType>(src.sm_type);

        TETHER_LOGI(TAG, "SM%zu from SII: Addr=0x%04X Len=%u Ctrl=0x%02X Type=%s Enable=%s",
                 i, dst.phys_start_addr, dst.length, std::bit_cast<uint8_t>(dst.control),
                 src.getTypeName(), dst.enable ? "yes" : "no");
        configured_any = true;
    }

    if (!configured_any) {
        TETHER_LOGW(TAG, "SII has no SM2/SM3 data for slave %u, trying HW registers", slave_index);

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
                    TETHER_LOGI(TAG, "SM%u from HW regs: Addr=0x%04X Len=%u Ctrl=0x%02X Act=0x%02X",
                             sm, addr, len, ctrl, act);
                    // configured_any = true; // Not used
                }
            }
        }
    }

    if (slave_configs[slave_index].sm[2].phys_start_addr == 0) {
        auto& sm2 = slave_configs[slave_index].sm[2];
        sm2 = PDO::SyncManagerConfig::process_output(0x1800, 0);
        TETHER_LOGW(TAG, "SM2 (RxPDO) using DEFAULT: Addr=0x%04X Ctrl=0x%02X", sm2.phys_start_addr, std::bit_cast<uint8_t>(sm2.control));
    }
    if (slave_configs[slave_index].sm[3].phys_start_addr == 0) {
        auto& sm3 = slave_configs[slave_index].sm[3];
        sm3 = PDO::SyncManagerConfig::process_input(0x1C00, 0);
        TETHER_LOGW(TAG, "SM3 (TxPDO) using DEFAULT: Addr=0x%04X Ctrl=0x%02X", sm3.phys_start_addr, std::bit_cast<uint8_t>(sm3.control));
    }

    pdo_->finalizeMapping(slave_index);

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

            uint8_t ctrl_byte = std::bit_cast<uint8_t>(cfg.control);
            writeRegister(SlaveAddress(slave_index), static_cast<uint16_t>(base + 4), &ctrl_byte, 1, 200);

            TETHER_LOGI(TAG, "Wrote SM%u to slave %u: Addr=0x%04X Len=%u Ctrl=0x%02X Act=0x00 (disabled)",
                     sm, slave_index, cfg.phys_start_addr, cfg.length, ctrl_byte);
        }
    }

    // Build/rebuild the logical address map from all configured slaves.
    // Use discovered slave count since this is called per-slave during discovery
    // before pdo_->slaveCount() has been set.
    logical_addr_mgr_->buildAddressMap(pdo_->slaveConfigs(),
                                        getDiscoveredSlaveCount());

    // Phase 2: Configure FMMUs while SMs are still disabled.
    auto& fmmu_mgr = slave(slave_index).fmmuManager();
    if (sii_valid && logical_addr_mgr_->hasSlavePDOs(slave_index)) {
        uint32_t rx_log = logical_addr_mgr_->getRxPDOLogicalAddr(slave_index);
        uint16_t rx_len = logical_addr_mgr_->getRxPDOLength(slave_index);
        uint32_t tx_log = logical_addr_mgr_->getTxPDOLogicalAddr(slave_index);
        uint16_t tx_len = logical_addr_mgr_->getTxPDOLength(slave_index);

        fmmu_mgr.configureManual(
            slave_configs[slave_index].sm[2].phys_start_addr, rx_len,
            slave_configs[slave_index].sm[3].phys_start_addr, tx_len,
            rx_log);
        if (!fmmu_mgr.writeToSlave(debug_flags_.fmmu && debug_flags_.fmmuFilt.allows(slave_index))) {
            TETHER_LOGE(TAG, "Slave %u: FMMU write (manual) failed", slave_index);
            return false;
        }
    } else if (sii_valid) {
        fmmu_mgr.configureFromSii(&sii, &slave_configs[slave_index], 0);
        if (!fmmu_mgr.writeToSlave(debug_flags_.fmmu && debug_flags_.fmmuFilt.allows(slave_index))) {
            TETHER_LOGE(TAG, "Slave %u: FMMU write (from SII) failed", slave_index);
            return false;
        }
    } else {
        TETHER_LOGW(TAG, "Slave %u: SII unavailable — FMMU not configured", slave_index);
    }

    // Phase 3: Enable SM2/SM3 NOW that FMMUs are configured.
    // This starts the SM watchdog — must happen as late as possible.
    for (uint8_t sm = 2; sm < 4; sm++) {
        const auto& cfg = slave_configs[slave_index].sm[sm];
        if (cfg.type != PDO::SyncManagerType::Unused && cfg.phys_start_addr != 0 && cfg.enable) {
            uint16_t base = static_cast<uint16_t>(0x0800 + sm * 8);
            uint8_t activate = 0x01;
            writeRegister(SlaveAddress(slave_index), static_cast<uint16_t>(base + 6), &activate, 1, 200);
            TETHER_LOGI(TAG, "Enabled SM%u on slave %u: Act=0x%01X",
                     sm, slave_index, activate);
        }
    }

    slave_configs[slave_index].configured = true;
    return true;
}
// ============================================================================
// Watchdog configuration
// ============================================================================

bool Master::configureWatchdogs(SlaveAddress slave_address,
                                         uint16_t pdi_timeout_100us,
                                         uint16_t pdata_timeout_100us)
{
    if (!writeRegister(slave_address, Raw::EC_REG_WD_DIV, static_cast<uint16_t>(0x09C2))) return false;
    if (!writeRegister(slave_address, Raw::EC_REG_WD_TIME_PDI, pdi_timeout_100us)) return false;
    if (!writeRegister(slave_address, Raw::EC_REG_WD_TIME_PDATA, pdata_timeout_100us)) return false;
    return true;
}

bool Master::disableWatchdogs(SlaveAddress slave_address)
{
    return configureWatchdogs(slave_address, 0, 0);
}

bool Master::readWatchdogStatus(SlaveAddress slave_address,
                                         uint8_t& wd_status,
                                         uint8_t& pdi_cnt,
                                         uint8_t& pdata_cnt)
{
    if (!readRegister(slave_address, Raw::EC_REG_WD_STATUS, wd_status, 200)) return false;
    if (!readRegister(slave_address, Raw::EC_REG_WD_CNT_PDI, pdi_cnt, 200)) return false;
    if (!readRegister(slave_address, Raw::EC_REG_WD_CNT_PDATA, pdata_cnt, 200)) return false;
    return true;
}

} // namespace EtherCAT
