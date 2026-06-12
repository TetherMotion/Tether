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

#include <cstdio>
#include <cstring>

namespace EtherCAT {

// Global FMMU debug flag (still used for verbose register-dump logging)
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
// FMMUManager — Configuration from SII
// ============================================================================

bool FMMUManager::configureFromSii(const SII::SIIData* sii,
                                    const PDO::SlaveConfig* sm_config,
                                    uint32_t base_logical_addr) {
    config_.clear();
    config_.next_logical_addr = base_logical_addr;

    size_t sii_fmmu_count = sii ? sii->fmmu_count : 0;

    TETHER_LOGI(TAG, "Configuring FMMUs from SII (%zu FMMU hints)", sii_fmmu_count);

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
                        config_.addOutput(sm2_addr, sm2_len, 2);
                        has_output = true;
                        TETHER_LOGI(TAG, "  FMMU%zu: Output -> SM2 (log=0x%08lX phy=0x%04X len=%u)",
                                 config_.fmmu_count - 1,
                                 (unsigned long)(config_.next_logical_addr - sm2_len),
                                 sm2_addr, sm2_len);
                    }
                    break;
                case SII::FMMU_TYPE_INPUT:
                    if (!has_input && sm3_len > 0) {
                        config_.addInput(sm3_addr, sm3_len, 3);
                        has_input = true;
                        TETHER_LOGI(TAG, "  FMMU%zu: Input -> SM3 (log=0x%08lX phy=0x%04X len=%u)",
                                 config_.fmmu_count - 1,
                                 (unsigned long)(config_.next_logical_addr - sm3_len),
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
        config_.addOutput(sm2_addr, sm2_len, 2);
        TETHER_LOGI(TAG, "  FMMU%zu: Output -> SM2 (DEFAULT, log=0x%08lX phy=0x%04X len=%u)",
                 config_.fmmu_count - 1,
                 (unsigned long)(config_.next_logical_addr - sm2_len),
                 sm2_addr, sm2_len);
    }

    if (!has_input && sm3_len > 0) {
        config_.addInput(sm3_addr, sm3_len, 3);
        TETHER_LOGI(TAG, "  FMMU%zu: Input -> SM3 (DEFAULT, log=0x%08lX phy=0x%04X len=%u)",
                 config_.fmmu_count - 1,
                 (unsigned long)(config_.next_logical_addr - sm3_len),
                 sm3_addr, sm3_len);
    }

    TETHER_LOGI(TAG, "%zu FMMUs configured, next_log=0x%08lX",
             config_.fmmu_count, (unsigned long)config_.next_logical_addr);

    return config_.fmmu_count > 0;
}

// ============================================================================
// FMMUManager — Manual Configuration
// ============================================================================

bool FMMUManager::configureManual(uint16_t output_phys, uint16_t output_len,
                                   uint16_t input_phys, uint16_t input_len,
                                   uint32_t base_logical_addr) {
    config_.clear();
    config_.next_logical_addr = base_logical_addr;

    TETHER_LOGI(TAG, "Manual FMMU config: out_phy=0x%04X out_len=%u, in_phy=0x%04X in_len=%u, base=0x%08lX",
             output_phys, output_len, input_phys, input_len, (unsigned long)base_logical_addr);

    if (output_len > 0) {
        config_.addOutput(output_phys, output_len, 2);
        TETHER_LOGI(TAG, "  FMMU0 Output: log=0x%08lX phy=0x%04X len=%u",
                 (unsigned long)config_.fmmus[0].logical_start_addr,
                 config_.fmmus[0].physical_start_addr,
                 config_.fmmus[0].length);
    }

    if (input_len > 0) {
        config_.addInput(input_phys, input_len, 3);
        TETHER_LOGI(TAG, "  FMMU1 Input: log=0x%08lX phy=0x%04X len=%u",
                 (unsigned long)config_.fmmus[config_.fmmu_count - 1].logical_start_addr,
                 config_.fmmus[config_.fmmu_count - 1].physical_start_addr,
                 config_.fmmus[config_.fmmu_count - 1].length);
    }

    return true;
}

// ============================================================================
// FMMUManager — Hardware Register Access
// ============================================================================

bool FMMUManager::writeToSlave() {
    if (config_.fmmu_count == 0) {
        TETHER_LOGW(TAG, "No FMMUs to configure");
        return true;
    }

    TETHER_LOGI(TAG, "Writing %zu FMMUs to slave...", config_.fmmu_count);

    bool all_ok = true;

    for (size_t i = 0; i < config_.fmmu_count; i++) {
        const FMMUConfig& fmmu = config_.fmmus[i];

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
            TETHER_LOGI(TAG, "  [FMMU-DEBUG] apwr ado=0x%04X len=%zu bytes: %s",
                     reg_addr, sizeof(regs), hex_buf);
        }

        if (!transport_.apwr(reg_addr, &regs, sizeof(regs), 100)) {
            TETHER_LOGE(TAG, "  FMMU%zu: 16-byte write failed!", i);

            // Fallback probe 1: try a 1-byte write to the activate register.
            uint8_t act_byte = regs.activate;
            if (transport_.apwr(static_cast<uint16_t>(reg_addr + 0x0C), &act_byte, 1, 100)) {
                TETHER_LOGE(TAG, "  FMMU%zu:  1-byte activate write SUCCEEDED — 16-byte APWR rejected by ESC", i);
            } else {
                // Fallback probe 2: APRD the activate byte to check reachability.
                uint8_t probe = 0xFF;
                if (transport_.aprd(static_cast<uint16_t>(reg_addr + 0x0C), &probe, 1, 100)) {
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

    for (size_t i = config_.fmmu_count; i < kMaxFMMUs; i++) {
        uint16_t reg_addr = kFMMURegBase + (static_cast<uint16_t>(i) * kFMMURegSize);
        (void)transport_.apwr(static_cast<uint16_t>(reg_addr + 0x0C), &zero_regs.activate, 1, 50);
    }

    config_.configured = all_ok;

    if (all_ok) {
        TETHER_LOGI(TAG, "All %zu FMMUs written successfully", config_.fmmu_count);
    }

    return all_ok;
}

size_t FMMUManager::readFromSlave(FMMUConfig* out_configs, size_t max_fmmus) {
    if (out_configs == nullptr || max_fmmus == 0) return 0;

    size_t count = 0;

    for (size_t i = 0; i < max_fmmus && i < kMaxFMMUs; i++) {
        uint16_t reg_addr = kFMMURegBase + (static_cast<uint16_t>(i) * kFMMURegSize);

        FMMURegBlock regs;
        if (!transport_.aprd(reg_addr, &regs, sizeof(regs), 100)) {
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

bool FMMUManager::verify() {
    if (config_.fmmu_count == 0) return true;

    FMMUConfig hw_configs[kMaxFMMUs];
    size_t hw_count = readFromSlave(hw_configs, kMaxFMMUs);

    if (hw_count < config_.fmmu_count) {
        TETHER_LOGE(TAG, "Could only read %zu/%zu FMMUs",
                 hw_count, config_.fmmu_count);
        return false;
    }

    TETHER_LOGI(TAG, "=== FMMU HARDWARE READBACK ===");
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
    for (size_t i = 0; i < config_.fmmu_count; i++) {
        const FMMUConfig& expected = config_.fmmus[i];
        const FMMUConfig& actual = hw_configs[i];

        bool ok = (expected.logical_start_addr == actual.logical_start_addr) &&
                  (expected.length == actual.length) &&
                  (expected.physical_start_addr == actual.physical_start_addr) &&
                  (expected.type == actual.type) &&
                  ((expected.activate & FMMUActivate::Enable) == (actual.activate & FMMUActivate::Enable));

        if (!ok) {
            TETHER_LOGE(TAG, "FMMU%zu mismatch:", i);
            match = false;
        } else {
            TETHER_LOGI(TAG, "  FMMU%zu: VERIFIED OK", i);
        }
    }

    return match;
}

bool FMMUManager::verifyFromSlave() {
    // Always read from slave — never rely on cached config_.fmmu_count
    FMMUConfig hw_configs[kMaxFMMUs];
    size_t hw_count = readFromSlave(hw_configs, kMaxFMMUs);

    if (hw_count == 0) {
        TETHER_LOGE(TAG, "verifyFromSlave: failed to read any FMMU registers from slave");
        return false;
    }

    // Count how many FMMUs are actually enabled on the slave hardware
    size_t enabled_count = 0;
    for (size_t i = 0; i < hw_count; i++) {
        if (hw_configs[i].isEnabled()) {
            enabled_count++;
        }
    }

    // If slave has no enabled FMMUs, that's acceptable
    if (enabled_count == 0) {
        return true;
    }

    // Slave has enabled FMMUs — they must match our expected configuration
    if (hw_count < config_.fmmu_count) {
        TETHER_LOGE(TAG, "verifyFromSlave: could only read %zu/%zu FMMUs",
                 hw_count, config_.fmmu_count);
        return false;
    }

    bool match = true;
    for (size_t i = 0; i < config_.fmmu_count; i++) {
        const FMMUConfig& expected = config_.fmmus[i];
        const FMMUConfig& actual   = hw_configs[i];

        bool ok = (expected.logical_start_addr == actual.logical_start_addr) &&
                  (expected.length == actual.length) &&
                  (expected.physical_start_addr == actual.physical_start_addr) &&
                  (expected.type == actual.type) &&
                  ((expected.activate & FMMUActivate::Enable) == (actual.activate & FMMUActivate::Enable));

        if (!ok) {
            TETHER_LOGE(TAG, "verifyFromSlave: FMMU%zu mismatch", i);
            match = false;
        }
    }

    return match;
}

bool FMMUManager::disableAll() {
    uint8_t zero = 0;

    for (size_t i = 0; i < kMaxFMMUs; i++) {
        uint16_t reg_addr = kFMMURegBase + (static_cast<uint16_t>(i) * kFMMURegSize) + 0x0C;
        (void)transport_.apwr(reg_addr, &zero, 1, 50);
    }

    config_.configured = false;

    return true;
}

// ============================================================================
// FMMUManager — Address Query Functions
// ============================================================================

uint32_t FMMUManager::getOutputLogicalAddr() const {
    const FMMUConfig* fmmu = config_.getOutputFMMU();
    return fmmu ? fmmu->logical_start_addr : 0;
}

uint32_t FMMUManager::getInputLogicalAddr() const {
    const FMMUConfig* fmmu = config_.getInputFMMU();
    return fmmu ? fmmu->logical_start_addr : 0;
}

// ============================================================================
// FMMUManager — Diagnostics
// ============================================================================

void FMMUManager::logConfig(const char* tag) const {
    TETHER_LOGI(tag, "FMMU Config: %zu FMMUs, configured=%s, next_log=0x%08lX",
             config_.fmmu_count, config_.configured ? "yes" : "no",
             (unsigned long)config_.next_logical_addr);
    for (size_t i = 0; i < config_.fmmu_count; i++) {
        const FMMUConfig& f = config_.fmmus[i];
        TETHER_LOGI(tag, "  FMMU%zu [%s]: log=0x%08lX phy=0x%04X len=%u type=0x%02X act=0x%02X SM%u",
                 i, getFMMUTypeName(f.sii_type),
                 (unsigned long)f.logical_start_addr,
                 f.physical_start_addr, f.length,
                 f.type, f.activate, f.associated_sm);
    }
}

void FMMUManager::logHardware(const char* tag) {
    FMMUConfig hw_configs[kMaxFMMUs];
    size_t count = readFromSlave(hw_configs, kMaxFMMUs);
    TETHER_LOGI(tag, "FMMU Hardware State (%zu FMMUs read)", count);
    for (size_t i = 0; i < count; i++) {
        const FMMUConfig& f = hw_configs[i];
        if (f.activate == 0 && f.length == 0) continue;
        TETHER_LOGI(tag, "  FMMU%zu: log=0x%08lX phy=0x%04X len=%u type=0x%02X act=0x%02X [%s]",
                 i, (unsigned long)f.logical_start_addr, f.physical_start_addr,
                 f.length, f.type, f.activate,
                 (f.activate & FMMUActivate::Enable) ? "ENABLED" : "disabled");
    }
}

} // namespace fmmu
} // namespace EtherCAT
