/**
 * @file RP20Module.hpp
 * @brief RP20 EtherCAT module descriptor table, lookup, and data access helpers
 *
 * Defines a compile-time descriptor for each of the 10 RP20 module types,
 * a lookup function by ModuleIdent, and typed read/write helpers for
 * accessing PDO channel data.
 */

#pragma once

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <array>
#include <optional>
#include <span>

#include "tether/drives/RP20/RP20Registers.hpp"
#include "tether/drives/RP20/RP20PDO.hpp"
#include "tether/drives/RP20/RP20ModuleConfig.hpp"

namespace EtherCAT {
namespace Drives {
namespace RP20Module {

namespace Reg = ::EtherCAT::Drives::Registers::RP20;
namespace Pdo = ::EtherCAT::Drives::RP20_pdo;
namespace Cfg = ::EtherCAT::Drives::RP20Config;

// ---------------------------------------------------------------------------
// Module type enum
// ---------------------------------------------------------------------------
enum class ModuleType : uint8_t {
    Unknown     = 0,
    DI_16       = 0x04,  // RP20-1600DT
    DO_16_PNP   = 0x08,  // RP20-0016DTP
    DR_8        = 0x09,  // RP20-0008DR
    DO_16_NPN   = 0x0A,  // RP20-0016DTN
    Multi_DIO_8 = 0x0C,  // RP20-0808DTP
    AI_4        = 0x10,  // RP20-0400IV
    RD_4        = 0x11,  // RP20-0400RD
    TC_4        = 0x12,  // RP20-0400TC
    AO_4        = 0x20,  // RP20-0004IV
    Mixed_AIO   = 0x30,  // RP20-0202IV
};

// ---------------------------------------------------------------------------
// Module descriptor
// ---------------------------------------------------------------------------
struct ModuleDescriptor {
    uint32_t module_ident;
    ModuleType type;
    const char* name;
    const char* module_class;

    bool has_txpdo;
    const Pdo::PDODescriptor* txpdo;

    bool has_rxpdo;
    const Pdo::PDODescriptor* rxpdo;

    const Cfg::CoEInitCmd* init_cmds;
    size_t init_cmd_count;

    const Reg::RegisterList* registers;
};

// ---------------------------------------------------------------------------
// Descriptor table — all 10 RP20 modules
// ---------------------------------------------------------------------------
static constexpr ModuleDescriptor kModule_0808DTP = {
    .module_ident = 0x0000000C,
    .type = ModuleType::Multi_DIO_8,
    .name = "RP20-0808DTP",
    .module_class = "Multi-DIO module",
    .has_txpdo = true,
    .txpdo = &Pdo::TxPDO_0808DTP,
    .has_rxpdo = true,
    .rxpdo = &Pdo::RxPDO_0808DTP,
    .init_cmds = Cfg::InitCmds_0808DTP.data(),
    .init_cmd_count = Cfg::InitCmds_0808DTP.size(),
    .registers = &Reg::DI::kRegisterList,
};

static constexpr ModuleDescriptor kModule_1600DT = {
    .module_ident = 0x00000004,
    .type = ModuleType::DI_16,
    .name = "RP20-1600DT",
    .module_class = "DI module",
    .has_txpdo = true,
    .txpdo = &Pdo::TxPDO_1600DT,
    .has_rxpdo = false,
    .rxpdo = nullptr,
    .init_cmds = Cfg::InitCmds_1600DT.data(),
    .init_cmd_count = Cfg::InitCmds_1600DT.size(),
    .registers = &Reg::DI::kRegisterList,
};

static constexpr ModuleDescriptor kModule_0016DTP = {
    .module_ident = 0x00000008,
    .type = ModuleType::DO_16_PNP,
    .name = "RP20-0016DTP",
    .module_class = "DO module",
    .has_txpdo = false,
    .txpdo = nullptr,
    .has_rxpdo = true,
    .rxpdo = &Pdo::RxPDO_0016DTP,
    .init_cmds = Cfg::InitCmds_0016DTP.data(),
    .init_cmd_count = Cfg::InitCmds_0016DTP.size(),
    .registers = &Reg::DO::kRegisterList,
};

static constexpr ModuleDescriptor kModule_0016DTN = {
    .module_ident = 0x0000000A,
    .type = ModuleType::DO_16_NPN,
    .name = "RP20-0016DTN",
    .module_class = "DO module",
    .has_txpdo = false,
    .txpdo = nullptr,
    .has_rxpdo = true,
    .rxpdo = &Pdo::RxPDO_0016DTN,
    .init_cmds = Cfg::InitCmds_0016DTN.data(),
    .init_cmd_count = Cfg::InitCmds_0016DTN.size(),
    .registers = &Reg::DO::kRegisterList,
};

static constexpr ModuleDescriptor kModule_0400IV = {
    .module_ident = 0x00000010,
    .type = ModuleType::AI_4,
    .name = "RP20-0400IV",
    .module_class = "Analog input module",
    .has_txpdo = true,
    .txpdo = &Pdo::TxPDO_0400IV,
    .has_rxpdo = false,
    .rxpdo = nullptr,
    .init_cmds = Cfg::InitCmds_0400IV.data(),
    .init_cmd_count = Cfg::InitCmds_0400IV.size(),
    .registers = &Reg::AI::kRegisterList,
};

static constexpr ModuleDescriptor kModule_0004IV = {
    .module_ident = 0x00000020,
    .type = ModuleType::AO_4,
    .name = "RP20-0004IV",
    .module_class = "AO module",
    .has_txpdo = false,
    .txpdo = nullptr,
    .has_rxpdo = true,
    .rxpdo = &Pdo::RxPDO_0004IV,
    .init_cmds = Cfg::InitCmds_0004IV.data(),
    .init_cmd_count = Cfg::InitCmds_0004IV.size(),
    .registers = &Reg::AO::kRegisterList,
};

static constexpr ModuleDescriptor kModule_0400RD = {
    .module_ident = 0x00000011,
    .type = ModuleType::RD_4,
    .name = "RP20-0400RD",
    .module_class = "RTD module",
    .has_txpdo = true,
    .txpdo = &Pdo::TxPDO_0400RD,
    .has_rxpdo = false,
    .rxpdo = nullptr,
    .init_cmds = Cfg::InitCmds_0400RD.data(),
    .init_cmd_count = Cfg::InitCmds_0400RD.size(),
    .registers = &Reg::RD::kRegisterList,
};

static constexpr ModuleDescriptor kModule_0400TC = {
    .module_ident = 0x00000012,
    .type = ModuleType::TC_4,
    .name = "RP20-0400TC",
    .module_class = "Thermocouple module",
    .has_txpdo = true,
    .txpdo = &Pdo::TxPDO_0400TC,
    .has_rxpdo = false,
    .rxpdo = nullptr,
    .init_cmds = Cfg::InitCmds_0400TC.data(),
    .init_cmd_count = Cfg::InitCmds_0400TC.size(),
    .registers = &Reg::TC::kRegisterList,
};

static constexpr ModuleDescriptor kModule_0008DR = {
    .module_ident = 0x00000009,
    .type = ModuleType::DR_8,
    .name = "RP20-0008DR",
    .module_class = "DO module",
    .has_txpdo = false,
    .txpdo = nullptr,
    .has_rxpdo = true,
    .rxpdo = &Pdo::RxPDO_0008DR,
    .init_cmds = Cfg::InitCmds_0008DR.data(),
    .init_cmd_count = Cfg::InitCmds_0008DR.size(),
    .registers = &Reg::DR::kRegisterList,
};

static constexpr ModuleDescriptor kModule_0202IV = {
    .module_ident = 0x00000030,
    .type = ModuleType::Mixed_AIO,
    .name = "RP20-0202IV",
    .module_class = "Multi-AIO module",
    .has_txpdo = true,
    .txpdo = &Pdo::TxPDO_0202IV,
    .has_rxpdo = true,
    .rxpdo = &Pdo::RxPDO_0202IV,
    .init_cmds = Cfg::InitCmds_0202IV.data(),
    .init_cmd_count = Cfg::InitCmds_0202IV.size(),
    .registers = &Reg::AI::kRegisterList,
};

// ---------------------------------------------------------------------------
// Lookup table
// ---------------------------------------------------------------------------
static constexpr std::array<const ModuleDescriptor*, 10> kAllModules = {
    &kModule_0808DTP,
    &kModule_1600DT,
    &kModule_0016DTP,
    &kModule_0016DTN,
    &kModule_0400IV,
    &kModule_0004IV,
    &kModule_0400RD,
    &kModule_0400TC,
    &kModule_0008DR,
    &kModule_0202IV,
};

// ---------------------------------------------------------------------------
// Lookup by ModuleIdent
// ---------------------------------------------------------------------------
inline constexpr const ModuleDescriptor* findByIdent(uint32_t ident) noexcept {
    for (const auto* mod : kAllModules) {
        if (mod->module_ident == ident) {
            return mod;
        }
    }
    return nullptr;
}

inline constexpr const ModuleDescriptor* findByType(ModuleType type) noexcept {
    for (const auto* mod : kAllModules) {
        if (mod->type == type) {
            return mod;
        }
    }
    return nullptr;
}

// ---------------------------------------------------------------------------
// Slot-dependent index helpers
// ---------------------------------------------------------------------------
inline constexpr uint16_t slotIndex(uint16_t base, uint8_t slot) {
    return static_cast<uint16_t>(base + slot * Reg::kSlotIndexIncrement);
}

inline constexpr uint16_t slotPDOIndex(uint16_t base, uint8_t slot) {
    return static_cast<uint16_t>(base + slot * Reg::kSlotPDOIncrement);
}

inline constexpr uint16_t configIndexForSlot(uint8_t slot) {
    return slotIndex(Reg::kConfigBaseIndex, slot);
}

inline constexpr uint16_t diagnosisIndexForSlot(uint8_t slot) {
    return slotIndex(Reg::kDiagnosisBaseIndex, slot);
}

// ---------------------------------------------------------------------------
// Typed data access helpers for PDO buffers
// ---------------------------------------------------------------------------

// Read a single bit from a digital PDO field
inline bool readBit(std::span<const uint8_t> pdo_buffer,
                    const Pdo::PDOField& field, uint8_t bit) {
    if (field.offset >= pdo_buffer.size()) return false;
    return (pdo_buffer[field.offset] >> bit) & 1u;
}

// Write a single bit to a digital PDO field
inline void writeBit(std::span<uint8_t> pdo_buffer,
                     const Pdo::PDOField& field, uint8_t bit, bool value) {
    if (field.offset >= pdo_buffer.size()) return;
    if (value) {
        pdo_buffer[field.offset] |= static_cast<uint8_t>(1u << bit);
    } else {
        pdo_buffer[field.offset] &= static_cast<uint8_t>(~(1u << bit));
    }
}

// Read a 16-bit signed value from an analog PDO field
inline int16_t readI16(std::span<const uint8_t> pdo_buffer,
                       const Pdo::PDOField& field) {
    if (field.offset + 1 >= pdo_buffer.size()) return 0;
    int16_t val;
    std::memcpy(&val, &pdo_buffer[field.offset], sizeof(int16_t));
    return val;
}

// Write a 16-bit signed value to an analog PDO field
inline void writeI16(std::span<uint8_t> pdo_buffer,
                     const Pdo::PDOField& field, int16_t value) {
    if (field.offset + 1 >= pdo_buffer.size()) return;
    std::memcpy(&pdo_buffer[field.offset], &value, sizeof(int16_t));
}

// Read an 8-bit unsigned value from a PDO field
inline uint8_t readU8(std::span<const uint8_t> pdo_buffer,
                      const Pdo::PDOField& field) {
    if (field.offset >= pdo_buffer.size()) return 0;
    return pdo_buffer[field.offset];
}

// Write an 8-bit unsigned value to a PDO field
inline void writeU8(std::span<uint8_t> pdo_buffer,
                    const Pdo::PDOField& field, uint8_t value) {
    if (field.offset >= pdo_buffer.size()) return;
    pdo_buffer[field.offset] = value;
}

// Get a PDO field by channel index from a descriptor
inline const Pdo::PDOField* getFieldByChannel(
    const Pdo::PDODescriptor& pdo, size_t channel) {
    if (channel >= pdo.field_count) return nullptr;
    return &pdo.fields[channel];
}

} // namespace RP20Module
} // namespace Drives
} // namespace EtherCAT
