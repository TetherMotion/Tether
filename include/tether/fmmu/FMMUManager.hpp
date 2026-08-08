/**
 * @file FMMUManager.hpp
 * @brief FMMU configuration manager and transport interface
 *
 * Extracted from FMMUConfiguration.hpp. Contains:
 * - SlaveFMMUConfig complete configuration set
 * - IFMMUTransport interface
 * - FMMUManager class
 */

#pragma once

#include <array>
#include <cstdint>
#include <vector>

#include "tether/fmmu/FMMUTypes.hpp"
#include "tether/ethercat/PDOMappingConfig.hpp"

namespace EtherCAT {
class Master;
namespace SII {
struct SIIData;
struct SIIFMMU;
}
namespace PDO {
struct SlaveConfig;
}
}

namespace EtherCAT {
namespace fmmu {

struct SlaveFMMUConfig {
    uint16_t slave_index{0};
    std::array<FMMUConfig, kMaxFMMUs> fmmus{};
    size_t fmmu_count{0};
    uint32_t next_logical_addr{0};
    bool configured{false};

    /// Per-PDO logical address entries (tracks multiple PDOs per FMMU region).
    std::array<FMMUPDOEntry, kMaxPDOsPerFMMU> pdo_entries{};
    size_t pdo_entry_count{0};

    bool addOutput(uint16_t phys_addr, uint16_t length, uint8_t sm_index = 2) {
        if (fmmu_count >= kMaxFMMUs || length == 0) return false;
        fmmus[fmmu_count] = FMMUConfig::output(next_logical_addr, phys_addr, length, sm_index);
        next_logical_addr += length;
        fmmu_count++;
        return true;
    }

    bool addInput(uint16_t phys_addr, uint16_t length, uint8_t sm_index = 3) {
        if (fmmu_count >= kMaxFMMUs || length == 0) return false;
        fmmus[fmmu_count] = FMMUConfig::input(next_logical_addr, phys_addr, length, sm_index);
        next_logical_addr += length;
        fmmu_count++;
        return true;
    }

    /**
     * @brief Add an output (RxPDO) FMMU with multiple PDO mappings.
     *
     * Creates one FMMU covering the entire SM physical range.  Each PDO's
     * logical address is allocated contiguously within the FMMU region.
     *
     * @param phys_addr  Physical start address of the SM
     * @param sm_index   Sync manager index (default 2)
     * @param pdos       Array of PDO mapping regions
     * @param count      Number of PDO mappings
     * @return true on success, false if FMMU slots or PDO entry slots exhausted
     */
    bool addOutputWithPDOs(uint16_t phys_addr, uint8_t sm_index,
                           const PDO::PDOMappingRegion* pdos, size_t count) {
        if (fmmu_count >= kMaxFMMUs || count == 0) return false;
        uint16_t total_len = 0;
        for (size_t i = 0; i < count; i++) total_len += pdos[i].size_bytes;
        if (total_len == 0) return false;

        fmmus[fmmu_count] = FMMUConfig::output(next_logical_addr, phys_addr, total_len, sm_index);
        uint32_t base_log = next_logical_addr;
        uint16_t phys_offset = 0;
        for (size_t i = 0; i < count && pdo_entry_count < kMaxPDOsPerFMMU; i++) {
            pdo_entries[pdo_entry_count].pdo_index = pdos[i].pdo_index;
            pdo_entries[pdo_entry_count].logical_addr = base_log + phys_offset;
            pdo_entries[pdo_entry_count].physical_offset = phys_offset;
            pdo_entries[pdo_entry_count].size_bytes = pdos[i].size_bytes;
            pdo_entries[pdo_entry_count].sm_index = sm_index;
            phys_offset += pdos[i].size_bytes;
            pdo_entry_count++;
        }
        next_logical_addr += total_len;
        fmmu_count++;
        return true;
    }

    /**
     * @brief Add an input (TxPDO) FMMU with multiple PDO mappings.
     *
     * @param phys_addr  Physical start address of the SM
     * @param sm_index   Sync manager index (default 3)
     * @param pdos       Array of PDO mapping regions
     * @param count      Number of PDO mappings
     * @return true on success, false if FMMU slots or PDO entry slots exhausted
     */
    bool addInputWithPDOs(uint16_t phys_addr, uint8_t sm_index,
                          const PDO::PDOMappingRegion* pdos, size_t count) {
        if (fmmu_count >= kMaxFMMUs || count == 0) return false;
        uint16_t total_len = 0;
        for (size_t i = 0; i < count; i++) total_len += pdos[i].size_bytes;
        if (total_len == 0) return false;

        fmmus[fmmu_count] = FMMUConfig::input(next_logical_addr, phys_addr, total_len, sm_index);
        uint32_t base_log = next_logical_addr;
        uint16_t phys_offset = 0;
        for (size_t i = 0; i < count && pdo_entry_count < kMaxPDOsPerFMMU; i++) {
            pdo_entries[pdo_entry_count].pdo_index = pdos[i].pdo_index;
            pdo_entries[pdo_entry_count].logical_addr = base_log + phys_offset;
            pdo_entries[pdo_entry_count].physical_offset = phys_offset;
            pdo_entries[pdo_entry_count].size_bytes = pdos[i].size_bytes;
            pdo_entries[pdo_entry_count].sm_index = sm_index;
            phys_offset += pdos[i].size_bytes;
            pdo_entry_count++;
        }
        next_logical_addr += total_len;
        fmmu_count++;
        return true;
    }

    const FMMUConfig* findByType(FMMUType type) const {
        for (size_t i = 0; i < fmmu_count; i++) {
            if (fmmus[i].sii_type == type) return &fmmus[i];
        }
        return nullptr;
    }

    const FMMUConfig* getOutputFMMU() const { return findByType(FMMUType::Output); }
    const FMMUConfig* getInputFMMU() const { return findByType(FMMUType::Input); }

    /// Find a PDO entry by its OD index.
    const FMMUPDOEntry* findPDOByIndex(uint16_t pdo_index) const {
        for (size_t i = 0; i < pdo_entry_count; i++) {
            if (pdo_entries[i].pdo_index == pdo_index) return &pdo_entries[i];
        }
        return nullptr;
    }

    void clearPDOEntries() {
        for (auto& e : pdo_entries) e = FMMUPDOEntry{};
        pdo_entry_count = 0;
    }

    void clear() {
        for (auto& f : fmmus) f = FMMUConfig{};
        fmmu_count = 0;
        next_logical_addr = 0;
        configured = false;
        clearPDOEntries();
    }
};

class IFMMUTransport {
public:
    virtual ~IFMMUTransport() = default;
    virtual bool apwr(uint16_t ado, const void* data,
                      uint16_t len, unsigned int timeout_ms) = 0;
    virtual bool aprd(uint16_t ado, void* out,
                      uint16_t len, unsigned int timeout_ms) = 0;
};

class FMMUManager {
public:
    explicit FMMUManager(IFMMUTransport& transport);

    SlaveFMMUConfig& config() { return config_; }
    const SlaveFMMUConfig& config() const { return config_; }

    bool configureFromSii(const SII::SIIData* sii,
                          const PDO::SlaveConfig* sm_config,
                          uint32_t base_logical_addr = 0);

    bool configureManual(uint16_t output_phys, uint16_t output_len,
                         uint16_t input_phys, uint16_t input_len,
                         uint32_t base_logical_addr = 0);

    /**
     * @brief Configure FMMUs from multiple PDO sync manager configs.
     *
     * Creates one FMMU per SM (output/input), with per-PDO logical address
     * tracking.  Each PDO mapping within an SM gets its own logical address
     * offset within the FMMU region.
     *
     * @param sm_configs       Vector of multi-PDO sync manager configs
     * @param base_logical_addr Base logical address for this slave's FMMU region
     * @return true if at least one FMMU was configured
     */
    bool configureFromMultiPDO(const std::vector<PDO::MultiPDOSyncManagerConfig>& sm_configs,
                               uint32_t base_logical_addr = 0);

    bool writeToSlave(bool fmmu_debug = false);
    size_t readFromSlave(FMMUConfig* out_configs, size_t max_fmmus);
    bool verify();
    bool verifyFromSlave();
    bool disableAll();

    uint32_t getOutputLogicalAddr() const;
    uint32_t getInputLogicalAddr() const;
    uint32_t getTotalLogicalSize() const { return config_.next_logical_addr; }

    // ----- Per-PDO address queries (multi-PDO mode) -----

    /// Get the logical address of a specific PDO by its OD index.
    /// Returns 0 if the PDO is not found.
    uint32_t getPDOLogicalAddr(uint16_t pdo_index) const;

    /// Get the size in bytes of a specific PDO by its OD index.
    /// Returns 0 if the PDO is not found.
    uint16_t getPDOSize(uint16_t pdo_index) const;

    /// Find a PDO entry by its OD index.
    /// Returns nullptr if not found.
    const FMMUPDOEntry* findPDOEntry(uint16_t pdo_index) const;

    void logConfig(const char* tag) const;
    void logHardware(const char* tag);

private:
    IFMMUTransport& transport_;
    SlaveFMMUConfig config_;
};

} // namespace fmmu
} // namespace EtherCAT
