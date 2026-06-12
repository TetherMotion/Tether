/**
 * @file FMMUConfiguration.cpp
 * @brief FMMU Configuration Implementation — FMMUManager class + backward-compat free functions
 * 
 * @see FMMUConfiguration.hpp for detailed documentation
 */

#include "fmmu/FMMUConfiguration.hpp"
#include "tether/platform/EspCompat.hpp"
#include "sii/SIIParser.hpp"
#include "EtherCATPDO.hpp"

// raw/internal.hpp is only needed by the backward-compat RawFMMUTransport
// (guarded by TETHER_COMPILE_MASTER below).
#ifdef TETHER_COMPILE_MASTER
#include "tether/ethercat/EtherCATMaster.hpp"
#endif

#include <cstdio>
#include <cstring>

namespace EtherCAT {

// Global FMMU debug flag
bool g_debug_fmmu = false;

namespace fmmu {

static const char* TAG = "fmmu";

// Helper: dump a byte buffer as space-separated hex
static void hexDump(const uint8_t* data, size_t len, char* out, size_t out_len) {
    size_t pos = 0;
    for (size_t i = 0; i < len && pos + 3 < out_len; i++) {
        pos += static_cast<size_t>(std::snprintf(out + pos, out_len - pos, "%02X ", data[i]));
    }
    if (pos > 0) out[pos - 1] = '\0';
}

// ============================================================================
// Utility Functions
// ============================================================================

const char* getFMMUTypeName(FMMUType type) {
    switch (type) {
        case FMMUType::Unused:   return "Unused";
        case FMMUType::Output:   return "Output";
        case FMMUType::Input:    return "Input";
        case FMMUType::MboxSync: return "MboxSync";
        default:                 return "Unknown";
    }
}

// ============================================================================
// FMMUManager — Construction
// ============================================================================

FMMUManager::FMMUManager(IFMMUTransport& transport)
    : transport_(transport)
{
}

// ============================================================================
// FMMUManager — Initialization
// ============================================================================

void FMMUManager::init() {
    TETHER_LOGI(TAG, "Initializing FMMU module");

    for (size_t i = 0; i < kMaxFMMUSlaves; i++) {
        configs_[i].clear();
        configs_[i].slave_index = static_cast<uint16_t>(i);
    }

    global_logical_addr_ = 0;
    initialized_ = true;
}

SlaveFMMUConfig* FMMUManager::getConfig(uint16_t slave_index) {
    if (slave_index >= kMaxFMMUSlaves) return nullptr;
    return &configs_[slave_index];
}

// ============================================================================
// FMMUManager — Configuration from SII
// ============================================================================

bool FMMUManager::configureFromSii(uint16_t slave_index,
                                    const SII::SIIData* sii,
                                    const PDO::SlaveConfig* sm_config,
                                    uint32_t base_logical_addr) {
    if (slave_index >= kMaxFMMUSlaves) {
        TETHER_LOGE(TAG, "Slave index %u exceeds maximum (%zu)", slave_index, kMaxFMMUSlaves);
        return false;
    }

    if (!initialized_) {
        init();
    }

    SlaveFMMUConfig* cfg = &configs_[slave_index];
    cfg->clear();
    cfg->slave_index = slave_index;
    cfg->next_logical_addr = base_logical_addr;

    size_t sii_fmmu_count = sii ? sii->fmmu_count : 0;

    TETHER_LOGI(TAG, "Slave %u: Configuring FMMUs from SII (%zu FMMU hints)",
             slave_index, sii_fmmu_count);

    uint16_t sm2_addr = 0x1800;
    uint16_t sm2_len = 0;
    uint16_t sm3_addr = 0x1C00;
    uint16_t sm3_len = 0;

    if (sm_config != nullptr) {
        if (sm_config->sm[2].phys_start_addr != 0) {
            sm2_addr = sm_config->sm[2].phys_start_addr;
            sm2_len = sm_config->sm[2].length;
        }
        if (sm_config->sm[3].phys_start_addr != 0) {
            sm3_addr = sm_config->sm[3].phys_start_addr;
            sm3_len = sm_config->sm[3].length;
        }
        if (sm2_len == 0 && sm_config->rxpdo_size > 0) {
            sm2_len = sm_config->rxpdo_size;
        }
        if (sm3_len == 0 && sm_config->txpdo_size > 0) {
            sm3_len = sm_config->txpdo_size;
        }
    }

    TETHER_LOGI(TAG, "  SM2 (Output): addr=0x%04X len=%u\n  SM3 (Input):  addr=0x%04X len=%u", sm2_addr, sm2_len, sm3_addr, sm3_len);

    bool has_output = false;
    bool has_input = false;

    if (sii != nullptr && sii_fmmu_count > 0) {
        for (size_t i = 0; i < sii_fmmu_count && i < kMaxFMMUs; i++) {
            const auto& sii_fmmu = sii->fmmus[i];
            uint8_t fmmu_type = sii_fmmu.fmmu_type;

            switch (fmmu_type) {
                case SII::FMMU_TYPE_OUTPUT:
                    if (!has_output && sm2_len > 0) {
                        cfg->addOutput(sm2_addr, sm2_len, 2);
                        has_output = true;
                        TETHER_LOGI(TAG, "  FMMU%zu: Output -> SM2 (log=0x%08lX phy=0x%04X len=%u)",
                                 cfg->fmmu_count - 1,
                                 (unsigned long)(cfg->next_logical_addr - sm2_len),
                                 sm2_addr, sm2_len);
                    }
                    break;
                case SII::FMMU_TYPE_INPUT:
                    if (!has_input && sm3_len > 0) {
                        cfg->addInput(sm3_addr, sm3_len, 3);
                        has_input = true;
                        TETHER_LOGI(TAG, "  FMMU%zu: Input -> SM3 (log=0x%08lX phy=0x%04X len=%u)",
                                 cfg->fmmu_count - 1,
                                 (unsigned long)(cfg->next_logical_addr - sm3_len),
                                 sm3_addr, sm3_len);
                    }
                    break;
                case SII::FMMU_TYPE_MBX_SYNC:
                    TETHER_LOGD(TAG, "  FMMU%zu: MboxSync (skipped)", i);
                    break;
                case SII::FMMU_TYPE_UNUSED:
                default:
                    break;
            }
        }
    }

    if (!has_output && sm2_len > 0) {
        cfg->addOutput(sm2_addr, sm2_len, 2);
        TETHER_LOGI(TAG, "  FMMU%zu: Output -> SM2 (DEFAULT, log=0x%08lX phy=0x%04X len=%u)",
                 cfg->fmmu_count - 1,
                 (unsigned long)(cfg->next_logical_addr - sm2_len),
                 sm2_addr, sm2_len);
    }

    if (!has_input && sm3_len > 0) {
        cfg->addInput(sm3_addr, sm3_len, 3);
        TETHER_LOGI(TAG, "  FMMU%zu: Input -> SM3 (DEFAULT, log=0x%08lX phy=0x%04X len=%u)",
                 cfg->fmmu_count - 1,
                 (unsigned long)(cfg->next_logical_addr - sm3_len),
                 sm3_addr, sm3_len);
    }

    global_logical_addr_ = cfg->next_logical_addr;

    TETHER_LOGI(TAG, "Slave %u: %zu FMMUs configured, next_log=0x%08lX",
             slave_index, cfg->fmmu_count, (unsigned long)cfg->next_logical_addr);

    return cfg->fmmu_count > 0;
}

// ============================================================================
// FMMUManager — Manual Configuration
// ============================================================================

bool FMMUManager::configureManual(uint16_t slave_index,
                                   uint16_t output_phys, uint16_t output_len,
                                   uint16_t input_phys, uint16_t input_len,
                                   uint32_t base_logical_addr) {
    if (slave_index >= kMaxFMMUSlaves) return false;

    if (!initialized_) init();

    SlaveFMMUConfig* cfg = &configs_[slave_index];
    cfg->clear();
    cfg->slave_index = slave_index;
    cfg->next_logical_addr = base_logical_addr;

    TETHER_LOGI(TAG, "Manual FMMU config for slave %u: out_phy=0x%04X out_len=%u, in_phy=0x%04X in_len=%u, base=0x%08lX",
             slave_index, output_phys, output_len, input_phys, input_len, (unsigned long)base_logical_addr);

    if (output_len > 0) {
        cfg->addOutput(output_phys, output_len, 2);
        TETHER_LOGI(TAG, "  FMMU0 Output: log=0x%08lX phy=0x%04X len=%u",
                 (unsigned long)cfg->fmmus[0].logical_start_addr,
                 cfg->fmmus[0].physical_start_addr,
                 cfg->fmmus[0].length);
    }

    if (input_len > 0) {
        cfg->addInput(input_phys, input_len, 3);
        TETHER_LOGI(TAG, "  FMMU1 Input: log=0x%08lX phy=0x%04X len=%u",
                 (unsigned long)cfg->fmmus[cfg->fmmu_count - 1].logical_start_addr,
                 cfg->fmmus[cfg->fmmu_count - 1].physical_start_addr,
                 cfg->fmmus[cfg->fmmu_count - 1].length);
    }

    global_logical_addr_ = cfg->next_logical_addr;

    return true;
}

// ============================================================================
// FMMUManager — Hardware Register Access
// ============================================================================

bool FMMUManager::writeToSlave(uint16_t slave_index) {
    if (slave_index >= kMaxFMMUSlaves) {
        TETHER_LOGE(TAG, "Invalid slave index %u", slave_index);
        return false;
    }

    SlaveFMMUConfig* cfg = &configs_[slave_index];
    if (cfg->fmmu_count == 0) {
        TETHER_LOGW(TAG, "Slave %u: No FMMUs to configure", slave_index);
        return true;
    }

    uint16_t adp = transport_.adpForSlaveIndex(slave_index);

    TETHER_LOGI(TAG, "Writing %zu FMMUs to slave %u...", cfg->fmmu_count, slave_index);

    bool all_ok = true;

    for (size_t i = 0; i < cfg->fmmu_count; i++) {
        const FMMUConfig& fmmu = cfg->fmmus[i];

        FMMURegBlock regs;
        std::memset(&regs, 0, sizeof(regs));

        regs.logical_start_le = fmmu.logical_start_addr;
        regs.length_le = fmmu.length;
        regs.logical_start_bit = fmmu.logical_start_bit;
        regs.logical_end_bit = fmmu.logical_end_bit;
        regs.physical_start_le = fmmu.physical_start_addr;
        regs.physical_start_bit = fmmu.physical_start_bit;
        regs.type = fmmu.type;
        regs.activate = fmmu.activate;

        uint16_t reg_addr = kFMMURegBase + (static_cast<uint16_t>(i) * kFMMURegSize);

        TETHER_LOGI(TAG, "  FMMU%zu @ 0x%04X: log=0x%08lX->phy=0x%04X len=%u type=0x%02X act=0x%02X",
                 i, reg_addr,
                 (unsigned long)fmmu.logical_start_addr,
                 fmmu.physical_start_addr,
                 fmmu.length,
                 fmmu.type,
                 fmmu.activate);

        if (g_debug_fmmu) {
            char hex_buf[128];
            hexDump(reinterpret_cast<const uint8_t*>(&regs), sizeof(regs), hex_buf, sizeof(hex_buf));
            TETHER_LOGI(TAG, "  [FMMU-DEBUG] apwr adp=0x%04X ado=0x%04X len=%zu bytes: %s",
                     adp, reg_addr, sizeof(regs), hex_buf);
        }

        if (!transport_.apwr(adp, reg_addr, &regs, sizeof(regs), 100)) {
            TETHER_LOGE(TAG, "  FMMU%zu: 16-byte write failed!", i);

            // Fallback probe 1: try a 1-byte write to the activate register.
            // Some ESCs reject 16-byte APWR but accept smaller writes.
            uint8_t act_byte = regs.activate;
            if (transport_.apwr(adp, static_cast<uint16_t>(reg_addr + 0x0C), &act_byte, 1, 100)) {
                TETHER_LOGE(TAG, "  FMMU%zu:  1-byte activate write SUCCEEDED — 16-byte APWR rejected by ESC", i);
            } else {
                // Fallback probe 2: APRD the activate byte to check reachability.
                uint8_t probe = 0xFF;
                if (transport_.aprd(adp, static_cast<uint16_t>(reg_addr + 0x0C), &probe, 1, 100)) {
                    if (probe == 0xFF) {
                        TETHER_LOGE(TAG, "  FMMU%zu:  APRD returned frame but WKC=0 (probe still 0xFF) — slave ignores 0x%04X", i, reg_addr);
                    } else {
                        TETHER_LOGE(TAG, "  FMMU%zu:  APRD succeeded (activate=0x%02X) — 16-byte APWR had WKC=0 or was rejected", i, probe);
                    }
                } else {
                    TETHER_LOGE(TAG, "  FMMU%zu:  Both 1-byte APWR and APRD failed — transport timeout or slave unreachable", i);
                }
            }
            all_ok = false;
        }
    }

    FMMURegBlock zero_regs;
    std::memset(&zero_regs, 0, sizeof(zero_regs));

    for (size_t i = cfg->fmmu_count; i < kMaxFMMUs; i++) {
        uint16_t reg_addr = kFMMURegBase + (static_cast<uint16_t>(i) * kFMMURegSize);
        if (!transport_.apwr(adp, reg_addr + 0x0C, &zero_regs.activate, 1, 50)) {
            TETHER_LOGD(TAG, "  FMMU%zu: Could not clear (may not exist)", i);
        }
    }

    cfg->configured = all_ok;

    if (all_ok) {
        TETHER_LOGI(TAG, "Slave %u: All %zu FMMUs written successfully", slave_index, cfg->fmmu_count);
    }

    return all_ok;
}

size_t FMMUManager::readFromSlave(uint16_t slave_index,
                                   FMMUConfig* out_configs, size_t max_fmmus) {
    if (out_configs == nullptr || max_fmmus == 0) return 0;

    uint16_t adp = transport_.adpForSlaveIndex(slave_index);
    size_t count = 0;

    for (size_t i = 0; i < max_fmmus && i < kMaxFMMUs; i++) {
        uint16_t reg_addr = kFMMURegBase + (static_cast<uint16_t>(i) * kFMMURegSize);

        FMMURegBlock regs;
        if (!transport_.aprd(adp, reg_addr, &regs, sizeof(regs), 100)) {
            break;
        }

        FMMUConfig& cfg = out_configs[i];
        cfg.logical_start_addr = regs.logical_start_le;
        cfg.length = regs.length_le;
        cfg.logical_start_bit = regs.logical_start_bit;
        cfg.logical_end_bit = regs.logical_end_bit;
        cfg.physical_start_addr = regs.physical_start_le;
        cfg.physical_start_bit = regs.physical_start_bit;
        cfg.type = regs.type;
        cfg.activate = regs.activate;

        if (regs.activate & FMMUActivate::Enable) {
            if (regs.type & FMMURegType::Write) {
                cfg.sii_type = FMMUType::Output;
            } else if (regs.type & FMMURegType::Read) {
                cfg.sii_type = FMMUType::Input;
            }
        }

        count++;
    }

    return count;
}

bool FMMUManager::verify(uint16_t slave_index) {
    if (slave_index >= kMaxFMMUSlaves) return false;

    SlaveFMMUConfig* cfg = &configs_[slave_index];
    if (cfg->fmmu_count == 0) return true;

    FMMUConfig hw_configs[kMaxFMMUs];
    size_t hw_count = readFromSlave(slave_index, hw_configs, kMaxFMMUs);

    if (hw_count < cfg->fmmu_count) {
        TETHER_LOGE(TAG, "Slave %u: Could only read %zu/%zu FMMUs",
                 slave_index, hw_count, cfg->fmmu_count);
        return false;
    }

    TETHER_LOGI(TAG, "=== FMMU HARDWARE READBACK (Slave %u) ===", slave_index);
    for (size_t i = 0; i < hw_count; i++) {
        const FMMUConfig& actual = hw_configs[i];
        TETHER_LOGI(TAG, "  FMMU%zu HW: log=0x%08lX phy=0x%04X len=%u type=0x%02X act=0x%02X",
                 i,
                 (unsigned long)actual.logical_start_addr,
                 actual.physical_start_addr,
                 actual.length,
                 actual.type,
                 actual.activate);
    }

    bool match = true;
    for (size_t i = 0; i < cfg->fmmu_count; i++) {
        const FMMUConfig& expected = cfg->fmmus[i];
        const FMMUConfig& actual = hw_configs[i];

        bool ok = (expected.logical_start_addr == actual.logical_start_addr) &&
                  (expected.length == actual.length) &&
                  (expected.physical_start_addr == actual.physical_start_addr) &&
                  (expected.type == actual.type) &&
                  ((expected.activate & FMMUActivate::Enable) == (actual.activate & FMMUActivate::Enable));

        if (!ok) {
            TETHER_LOGE(TAG, "Slave %u FMMU%zu mismatch:", slave_index, i);
            match = false;
        } else {
            TETHER_LOGI(TAG, "  FMMU%zu: VERIFIED OK", i);
        }
    }

    return match;
}

bool FMMUManager::disableAll(uint16_t slave_index) {
    uint16_t adp = transport_.adpForSlaveIndex(slave_index);
    uint8_t zero = 0;

    for (size_t i = 0; i < kMaxFMMUs; i++) {
        uint16_t reg_addr = kFMMURegBase + (static_cast<uint16_t>(i) * kFMMURegSize) + 0x0C;
        (void)transport_.apwr(adp, reg_addr, &zero, 1, 50);
    }

    if (slave_index < kMaxFMMUSlaves) {
        configs_[slave_index].configured = false;
    }

    return true;
}

// ============================================================================
// FMMUManager — Address Query Functions
// ============================================================================

uint32_t FMMUManager::getOutputLogicalAddr(uint16_t slave_index) const {
    if (slave_index >= kMaxFMMUSlaves) return 0;
    const FMMUConfig* fmmu = configs_[slave_index].getOutputFMMU();
    return fmmu ? fmmu->logical_start_addr : 0;
}

uint32_t FMMUManager::getInputLogicalAddr(uint16_t slave_index) const {
    if (slave_index >= kMaxFMMUSlaves) return 0;
    const FMMUConfig* fmmu = configs_[slave_index].getInputFMMU();
    return fmmu ? fmmu->logical_start_addr : 0;
}

// ============================================================================
// FMMUManager — Diagnostics
// ============================================================================

void FMMUManager::logConfig(uint16_t slave_index, const char* tag) const {
    if (slave_index >= kMaxFMMUSlaves) {
        TETHER_LOGE(tag, "Invalid slave index %u", slave_index);
        return;
    }
    const SlaveFMMUConfig* cfg = &configs_[slave_index];
    TETHER_LOGI(tag, "FMMU Config - Slave %u: %zu FMMUs, configured=%s, next_log=0x%08lX",
             slave_index, cfg->fmmu_count, cfg->configured ? "yes" : "no",
             (unsigned long)cfg->next_logical_addr);
    for (size_t i = 0; i < cfg->fmmu_count; i++) {
        const FMMUConfig& f = cfg->fmmus[i];
        TETHER_LOGI(tag, "  FMMU%zu [%s]: log=0x%08lX phy=0x%04X len=%u type=0x%02X act=0x%02X SM%u",
                 i, getFMMUTypeName(f.sii_type),
                 (unsigned long)f.logical_start_addr,
                 f.physical_start_addr, f.length,
                 f.type, f.activate, f.associated_sm);
    }
}

void FMMUManager::logHardware(uint16_t slave_index, const char* tag) {
    FMMUConfig hw_configs[kMaxFMMUs];
    size_t count = readFromSlave(slave_index, hw_configs, kMaxFMMUs);
    TETHER_LOGI(tag, "FMMU Hardware State - Slave %u (%zu FMMUs read)", slave_index, count);
    for (size_t i = 0; i < count; i++) {
        const FMMUConfig& f = hw_configs[i];
        if (f.activate == 0 && f.length == 0) continue;
        TETHER_LOGI(tag, "  FMMU%zu: log=0x%08lX phy=0x%04X len=%u type=0x%02X act=0x%02X [%s]",
                 i, (unsigned long)f.logical_start_addr, f.physical_start_addr,
                 f.length, f.type, f.activate,
                 (f.activate & FMMUActivate::Enable) ? "ENABLED" : "disabled");
    }
}

// ============================================================================
// Backward-compatible free functions
//
// These delegate to a module-local FMMUManager.  The transport implementation
// depends on whether the full EtherCAT master is linked.
// ============================================================================

#ifdef TETHER_COMPILE_MASTER

/// Concrete transport delegating to the active EtherCATMaster.
class RawFMMUTransport : public IFMMUTransport {
public:
    void setMaster(EtherCAT::EtherCATMaster* m) { master_ = m; }
    
    bool apwr(uint16_t adp, uint16_t ado, const void* data, uint16_t len, unsigned int timeout_ms) override {
        if (!master_) return false;
        return master_->writeRegister(EtherCATMaster::slaveAddressFromADP(adp), ado, data, len, timeout_ms);
    }
    bool aprd(uint16_t adp, uint16_t ado, void* out, uint16_t len, unsigned int timeout_ms) override {
        if (!master_) return false;
        return master_->readRegister(EtherCATMaster::slaveAddressFromADP(adp), ado, out, len, timeout_ms);
    }
    uint16_t adpForSlaveIndex(uint16_t slave_index) override {
        return EtherCAT::EtherCATMaster::adpForSlaveIndex(slave_index);
    }
private:
    EtherCAT::EtherCATMaster* master_ = nullptr;
};

static RawFMMUTransport s_raw_transport;

#else // !TETHER_COMPILE_MASTER

/// Null transport for common-only builds (no real I/O available).
class NullFMMUTransport : public IFMMUTransport {
public:
    bool apwr(uint16_t, uint16_t, const void*, uint16_t, unsigned int) override { return false; }
    bool aprd(uint16_t, uint16_t, void*, uint16_t, unsigned int) override { return false; }
    uint16_t adpForSlaveIndex(uint16_t slave_index) override {
        return static_cast<uint16_t>(0u - slave_index);
    }
};

static NullFMMUTransport s_raw_transport;

#endif // TETHER_COMPILE_MASTER

static FMMUManager s_default_manager(s_raw_transport);

// ---- backward-compat free-function wrappers ----

SlaveFMMUConfig* fmmu_get_slave_configs() {
    return s_default_manager.getSlaveConfigs();
}

SlaveFMMUConfig* fmmu_get_config(uint16_t slave_index) {
    return s_default_manager.getConfig(slave_index);
}

void fmmu_init() {
    s_default_manager.init();
}

bool fmmu_configure_from_sii(uint16_t slave_index,
                              const SII::SIIData* sii,
                              const PDO::SlaveConfig* sm_config,
                              uint32_t base_logical_addr) {
    return s_default_manager.configureFromSii(slave_index, sii, sm_config, base_logical_addr);
}

bool fmmu_configure_manual(uint16_t slave_index,
                           uint16_t output_phys, uint16_t output_len,
                           uint16_t input_phys, uint16_t input_len,
                           uint32_t base_logical_addr) {
    return s_default_manager.configureManual(slave_index, output_phys, output_len,
                                              input_phys, input_len, base_logical_addr);
}

bool fmmu_write_to_slave(const uint8_t /*src_mac*/[6], uint16_t slave_index) {
    return s_default_manager.writeToSlave(slave_index);
}

size_t fmmu_read_from_slave(const uint8_t /*src_mac*/[6], uint16_t slave_index,
                            FMMUConfig* out_configs, size_t max_fmmus) {
    return s_default_manager.readFromSlave(slave_index, out_configs, max_fmmus);
}

bool fmmu_verify(const uint8_t /*src_mac*/[6], uint16_t slave_index) {
    return s_default_manager.verify(slave_index);
}

bool fmmu_disable_all(const uint8_t /*src_mac*/[6], uint16_t slave_index) {
    return s_default_manager.disableAll(slave_index);
}

void fmmu_log_config(uint16_t slave_index, const char* tag) {
    s_default_manager.logConfig(slave_index, tag);
}

void fmmu_log_hardware(const uint8_t /*src_mac*/[6], uint16_t slave_index, const char* tag) {
    s_default_manager.logHardware(slave_index, tag);
}

uint32_t fmmu_get_output_logical_addr(uint16_t slave_index) {
    return s_default_manager.getOutputLogicalAddr(slave_index);
}

uint32_t fmmu_get_input_logical_addr(uint16_t slave_index) {
    return s_default_manager.getInputLogicalAddr(slave_index);
}

uint32_t fmmu_get_total_logical_size() {
    return s_default_manager.getTotalLogicalSize();
}

void fmmu_set_master(EtherCAT::EtherCATMaster* master) {
#ifdef TETHER_COMPILE_MASTER
    s_raw_transport.setMaster(master);
#else
    (void)master;
#endif
}

} // namespace fmmu
} // namespace EtherCAT
