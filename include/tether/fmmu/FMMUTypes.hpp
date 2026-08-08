/**
 * @file FMMUTypes.hpp
 * @brief FMMU type definitions and configuration structures
 *
 * Extracted from FMMUConfiguration.hpp. Contains:
 * - FMMU constants, FMMUType enum
 * - FMMURegType, FMMUActivate namespaces
 * - FMMUConfig struct with bitstruct-backed type/activate fields
 * - FMMURegBlock wire format
 */

#pragma once

#include <cstdint>
#include <cstddef>

#include "tether/platform/EspCompat.hpp"
#include "tether/ethercat/FMMURegisters.hpp"

namespace EtherCAT {
namespace fmmu {

constexpr size_t kMaxFMMUs = 8;
constexpr uint16_t kFMMURegBase = 0x0600;
constexpr size_t kFMMURegSize = 16;

/// Maximum number of PDO mappings tracked per FMMU region.
/// Matches PDO::kMaxPDOsPerSM — well above any real-world slave.
constexpr size_t kMaxPDOsPerFMMU = 16;

enum class FMMUType : uint8_t {
    Unused      = 0x00,
    Output      = 0x01,
    Input       = 0x02,
    MboxSync    = 0x03,
};

const char* getFMMUTypeName(FMMUType type);

namespace FMMURegType {
    constexpr uint8_t Read          = 0x01;
    constexpr uint8_t Write         = 0x02;
    constexpr uint8_t ReadWrite     = 0x03;
}

namespace FMMUActivate {
    constexpr uint8_t Enable        = 0x01;
    constexpr uint8_t Disable       = 0x00;
}

struct FMMUConfig {
    uint32_t logical_start_addr{0};
    uint16_t length{0};
    uint8_t logical_start_bit{0};
    uint8_t logical_end_bit{7};
    uint16_t physical_start_addr{0};
    uint8_t physical_start_bit{0};
    EtherCAT::FMMU::FMMUTypeReg type{};
    EtherCAT::FMMU::FMMUActivateReg activate{};
    uint8_t associated_sm{0xFF};
    FMMUType sii_type{FMMUType::Unused};

    bool isEnabled() const {
        return activate.enable != 0 && length > 0;
    }

    bool isOutput() const {
        return type.write_enable != 0;
    }

    bool isInput() const {
        return type.read_enable != 0;
    }

    static FMMUConfig output(uint32_t logical_addr, uint16_t phys_addr,
                             uint16_t len, uint8_t sm_index = 2) {
        FMMUConfig cfg;
        cfg.logical_start_addr = logical_addr;
        cfg.physical_start_addr = phys_addr;
        cfg.length = len;
        cfg.logical_start_bit = 0;
        cfg.logical_end_bit = 7;
        cfg.physical_start_bit = 0;
        cfg.type.write_enable = 1;
        cfg.activate.enable = 1;
        cfg.associated_sm = sm_index;
        cfg.sii_type = FMMUType::Output;
        return cfg;
    }

    static FMMUConfig input(uint32_t logical_addr, uint16_t phys_addr,
                            uint16_t len, uint8_t sm_index = 3) {
        FMMUConfig cfg;
        cfg.logical_start_addr = logical_addr;
        cfg.physical_start_addr = phys_addr;
        cfg.length = len;
        cfg.logical_start_bit = 0;
        cfg.logical_end_bit = 7;
        cfg.physical_start_bit = 0;
        cfg.type.read_enable = 1;
        cfg.activate.enable = 1;
        cfg.associated_sm = sm_index;
        cfg.sii_type = FMMUType::Input;
        return cfg;
    }
};

/**
 * @brief Per-PDO logical address entry within an FMMU region.
 *
 * When multiple PDOs are assigned to a single sync manager, one FMMU maps
 * the entire SM physical range.  This struct tracks each PDO's logical
 * address offset and physical offset within that FMMU region, enabling
 * per-PDO data access during LRW exchanges.
 */
struct FMMUPDOEntry {
    uint16_t pdo_index{0};          ///< OD index of the PDO mapping object
    uint32_t logical_addr{0};       ///< Logical address of this PDO within the FMMU region
    uint16_t physical_offset{0};    ///< Byte offset from the SM physical start address
    uint16_t size_bytes{0};         ///< Size of this PDO in bytes
    uint8_t  sm_index{0xFF};        ///< Associated sync manager index

    bool isValid() const { return pdo_index != 0 && size_bytes > 0; }
};

struct __attribute__((packed)) FMMURegBlock {
    uint32_t logical_start_le;
    uint16_t length_le;
    uint8_t logical_start_bit;
    uint8_t logical_end_bit;
    uint16_t physical_start_le;
    uint8_t physical_start_bit;
    uint8_t type;
    uint8_t activate;
    uint8_t reserved[3];
};
static_assert(sizeof(FMMURegBlock) == 16, "FMMURegBlock must be 16 bytes");

} // namespace fmmu
} // namespace EtherCAT
