/**
 * @file RP20PDO.hpp
 * @brief Compile-time PDO layout descriptors for all RP20 modules
 *
 * Describes RxPDO and TxPDO layouts for the 10 RP20 module types
 * defined in RP20_ECT_1.1.0.7.xml.
 */

#pragma once

#include <cstddef>
#include <cstdint>
#include <array>

#include "tether/drives/RP20/Registers/DigitalInput.hpp"
#include "tether/drives/RP20/Registers/DigitalOutput.hpp"
#include "tether/drives/RP20/Registers/AnalogInput.hpp"
#include "tether/drives/RP20/Registers/AnalogOutput.hpp"
#include "tether/drives/RP20/Registers/RTDInput.hpp"
#include "tether/drives/RP20/Registers/ThermocoupleInput.hpp"
#include "tether/drives/RP20/Registers/RelayOutput.hpp"

namespace EtherCAT {
namespace Drives {
namespace RP20_pdo {

namespace Reg = ::EtherCAT::Drives::Registers::RP20;

struct PDOField {
    const ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry* entry;
    uint16_t offset;
    uint8_t size;
    const char* description;
};

struct PDODescriptor {
    uint16_t index;
    uint16_t size;
    const PDOField* fields;
    size_t field_count;
};

static constexpr PDODescriptor makePDO(uint16_t idx, uint16_t sz,
                                        const PDOField* flds, size_t count) {
    return { idx, sz, flds, count };
}

// ===========================================================================
// RP20-0808DTP (ModuleIdent 0x0C) — 8-ch DI + 8-ch DO (PNP)
// ===========================================================================

// TxPDO 0x1A00: 1 byte (8-bit DI)
static constexpr std::array<PDOField, 1> TxPDO_0808DTP_Fields = {{
    { &Reg::DI::InputCH0, 0u, 1, "DI CH0-8bit" },
}};
static constexpr PDODescriptor TxPDO_0808DTP =
    makePDO(0x1A00u, 1u, TxPDO_0808DTP_Fields.data(), TxPDO_0808DTP_Fields.size());

// RxPDO 0x1600: 1 byte (8-bit DO)
static constexpr std::array<PDOField, 1> RxPDO_0808DTP_Fields = {{
    { &Reg::DO::OutputCH0, 0u, 1, "DO CH0-8bit" },
}};
static constexpr PDODescriptor RxPDO_0808DTP =
    makePDO(0x1600u, 1u, RxPDO_0808DTP_Fields.data(), RxPDO_0808DTP_Fields.size());

// ===========================================================================
// RP20-1600DT (ModuleIdent 0x04) — 16-ch DI
// ===========================================================================

// TxPDO 0x1A00: 2 bytes (2 × 8-bit DI)
static constexpr std::array<PDOField, 2> TxPDO_1600DT_Fields = {{
    { &Reg::DI::InputCH0, 0u, 1, "DI CH0-8bit" },
    { &Reg::DI::InputCH1, 1u, 1, "DI CH1-8bit" },
}};
static constexpr PDODescriptor TxPDO_1600DT =
    makePDO(0x1A00u, 2u, TxPDO_1600DT_Fields.data(), TxPDO_1600DT_Fields.size());

// ===========================================================================
// RP20-0016DTP (ModuleIdent 0x08) — 16-ch DO (PNP)
// ===========================================================================

// RxPDO 0x1600: 2 bytes (2 × 8-bit DO)
static constexpr std::array<PDOField, 2> RxPDO_0016DTP_Fields = {{
    { &Reg::DO::OutputCH0, 0u, 1, "DO CH0-8bit" },
    { &Reg::DO::OutputCH1, 1u, 1, "DO CH1-8bit" },
}};
static constexpr PDODescriptor RxPDO_0016DTP =
    makePDO(0x1600u, 2u, RxPDO_0016DTP_Fields.data(), RxPDO_0016DTP_Fields.size());

// ===========================================================================
// RP20-0016DTN (ModuleIdent 0x0A) — 16-ch DO (NPN)
// ===========================================================================

// RxPDO 0x1600: 2 bytes (identical layout to DTP)
static constexpr std::array<PDOField, 2> RxPDO_0016DTN_Fields = {{
    { &Reg::DO::OutputCH0, 0u, 1, "DO CH0-8bit" },
    { &Reg::DO::OutputCH1, 1u, 1, "DO CH1-8bit" },
}};
static constexpr PDODescriptor RxPDO_0016DTN =
    makePDO(0x1600u, 2u, RxPDO_0016DTN_Fields.data(), RxPDO_0016DTN_Fields.size());

// ===========================================================================
// RP20-0400IV (ModuleIdent 0x10) — 4-ch AI
// ===========================================================================

// TxPDO 0x1A00: 8 bytes (4 × 16-bit INT)
static constexpr std::array<PDOField, 4> TxPDO_0400IV_Fields = {{
    { &Reg::AI::InputCH0, 0u, 2, "AI CH0" },
    { &Reg::AI::InputCH1, 2u, 2, "AI CH1" },
    { &Reg::AI::InputCH2, 4u, 2, "AI CH2" },
    { &Reg::AI::InputCH3, 6u, 2, "AI CH3" },
}};
static constexpr PDODescriptor TxPDO_0400IV =
    makePDO(0x1A00u, 8u, TxPDO_0400IV_Fields.data(), TxPDO_0400IV_Fields.size());

// ===========================================================================
// RP20-0004IV (ModuleIdent 0x20) — 4-ch AO
// ===========================================================================

// RxPDO 0x1600: 8 bytes (4 × 16-bit INT)
static constexpr std::array<PDOField, 4> RxPDO_0004IV_Fields = {{
    { &Reg::AO::OutputCH0, 0u, 2, "AO CH0" },
    { &Reg::AO::OutputCH1, 2u, 2, "AO CH1" },
    { &Reg::AO::OutputCH2, 4u, 2, "AO CH2" },
    { &Reg::AO::OutputCH3, 6u, 2, "AO CH3" },
}};
static constexpr PDODescriptor RxPDO_0004IV =
    makePDO(0x1600u, 8u, RxPDO_0004IV_Fields.data(), RxPDO_0004IV_Fields.size());

// ===========================================================================
// RP20-0400RD (ModuleIdent 0x11) — 4-ch RTD
// ===========================================================================

// TxPDO 0x1A00: 8 bytes (4 × 16-bit INT)
static constexpr std::array<PDOField, 4> TxPDO_0400RD_Fields = {{
    { &Reg::RD::InputCH0, 0u, 2, "RTD CH0" },
    { &Reg::RD::InputCH1, 2u, 2, "RTD CH1" },
    { &Reg::RD::InputCH2, 4u, 2, "RTD CH2" },
    { &Reg::RD::InputCH3, 6u, 2, "RTD CH3" },
}};
static constexpr PDODescriptor TxPDO_0400RD =
    makePDO(0x1A00u, 8u, TxPDO_0400RD_Fields.data(), TxPDO_0400RD_Fields.size());

// ===========================================================================
// RP20-0400TC (ModuleIdent 0x12) — 4-ch Thermocouple
// ===========================================================================

// TxPDO 0x1A00: 8 bytes (4 × 16-bit INT)
static constexpr std::array<PDOField, 4> TxPDO_0400TC_Fields = {{
    { &Reg::TC::InputCH0, 0u, 2, "TC CH0" },
    { &Reg::TC::InputCH1, 2u, 2, "TC CH1" },
    { &Reg::TC::InputCH2, 4u, 2, "TC CH2" },
    { &Reg::TC::InputCH3, 6u, 2, "TC CH3" },
}};
static constexpr PDODescriptor TxPDO_0400TC =
    makePDO(0x1A00u, 8u, TxPDO_0400TC_Fields.data(), TxPDO_0400TC_Fields.size());

// ===========================================================================
// RP20-0008DR (ModuleIdent 0x09) — 8-ch Relay Output
// ===========================================================================

// RxPDO 0x1600: 1 byte (8-bit relay)
static constexpr std::array<PDOField, 1> RxPDO_0008DR_Fields = {{
    { &Reg::DR::OutputCH0, 0u, 1, "DR CH0-8bit" },
}};
static constexpr PDODescriptor RxPDO_0008DR =
    makePDO(0x1600u, 1u, RxPDO_0008DR_Fields.data(), RxPDO_0008DR_Fields.size());

// ===========================================================================
// RP20-0202IV (ModuleIdent 0x30) — 2-ch AI + 2-ch AO
// ===========================================================================

// TxPDO 0x1A00: 4 bytes (2 × 16-bit INT) — AI data at 0x6000
static constexpr std::array<PDOField, 2> TxPDO_0202IV_Fields = {{
    { &Reg::DI::InputCH0, 0u, 2, "AI CH0" },  // 0x6000:1 in mixed mode
    { &Reg::DI::InputCH1, 2u, 2, "AI CH1" },  // 0x6000:2 in mixed mode
}};
static constexpr PDODescriptor TxPDO_0202IV =
    makePDO(0x1A00u, 4u, TxPDO_0202IV_Fields.data(), TxPDO_0202IV_Fields.size());

// RxPDO 0x1600: 4 bytes (2 × 16-bit INT) — AO data at 0x7000
static constexpr std::array<PDOField, 2> RxPDO_0202IV_Fields = {{
    { &Reg::AO::OutputCH0, 0u, 2, "AO CH0" },
    { &Reg::AO::OutputCH1, 2u, 2, "AO CH1" },
}};
static constexpr PDODescriptor RxPDO_0202IV =
    makePDO(0x1600u, 4u, RxPDO_0202IV_Fields.data(), RxPDO_0202IV_Fields.size());

} // namespace RP20_pdo
} // namespace Drives
} // namespace EtherCAT
