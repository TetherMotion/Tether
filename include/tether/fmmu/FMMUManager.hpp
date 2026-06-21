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

#include "tether/fmmu/FMMUTypes.hpp"

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

    const FMMUConfig* findByType(FMMUType type) const {
        for (size_t i = 0; i < fmmu_count; i++) {
            if (fmmus[i].sii_type == type) return &fmmus[i];
        }
        return nullptr;
    }

    const FMMUConfig* getOutputFMMU() const { return findByType(FMMUType::Output); }
    const FMMUConfig* getInputFMMU() const { return findByType(FMMUType::Input); }

    void clear() {
        for (auto& f : fmmus) f = FMMUConfig{};
        fmmu_count = 0;
        next_logical_addr = 0;
        configured = false;
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

    bool writeToSlave(bool fmmu_debug = false);
    size_t readFromSlave(FMMUConfig* out_configs, size_t max_fmmus);
    bool verify();
    bool verifyFromSlave();
    bool disableAll();

    uint32_t getOutputLogicalAddr() const;
    uint32_t getInputLogicalAddr() const;
    uint32_t getTotalLogicalSize() const { return config_.next_logical_addr; }

    void logConfig(const char* tag) const;
    void logHardware(const char* tag);

private:
    IFMMUTransport& transport_;
    SlaveFMMUConfig config_;
};

} // namespace fmmu
} // namespace EtherCAT
