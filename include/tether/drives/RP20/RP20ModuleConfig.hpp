/**
 * @file RP20ModuleConfig.hpp
 * @brief CoE mailbox initialization commands for all RP20 modules
 *
 * Each module's InitCmd entries from RP20_ECT_1.1.0.7.xml are encoded as
 * constexpr arrays. All commands use Transition "PS" (PRE-OP to SAFE-OP).
 * Indices are slot-dependent (DependOnSlot="true"); the base index 0x8000
 * is stored and the slot offset is applied at runtime.
 */

#pragma once

#include <cstddef>
#include <cstdint>
#include <array>

namespace EtherCAT {
namespace Drives {
namespace RP20Config {

struct CoEInitCmd {
    uint16_t index;      // OD index (base, slot offset applied at runtime)
    uint8_t  subindex;   // OD subindex
    uint16_t data_size;  // Size of data in bytes
    uint32_t data;       // Raw data (little-endian, up to 4 bytes)
    const char* comment;
};

static constexpr uint16_t kConfigBaseIndex = 0x8000;

// ===========================================================================
// RP20-0808DTP (0x0C) — 4 init commands
// ===========================================================================
static constexpr std::array<CoEInitCmd, 4> InitCmds_0808DTP = {{
    { kConfigBaseIndex, 0x01, 1, 0x00, "DI module bit0-3 filter time" },
    { kConfigBaseIndex, 0x02, 1, 0x00, "DI module bit4-7 filter time" },
    { kConfigBaseIndex, 0x03, 1, 0x00, "DO module CH0 stopmode after EtherCAT lost link" },
    { kConfigBaseIndex, 0x04, 1, 0x00, "DO module CH0 stopvalue after EtherCAT lost link" },
}};

// ===========================================================================
// RP20-1600DT (0x04) — 4 init commands
// ===========================================================================
static constexpr std::array<CoEInitCmd, 4> InitCmds_1600DT = {{
    { kConfigBaseIndex, 0x01, 1, 0x00, "DI module CH0 bit0-3 filter time" },
    { kConfigBaseIndex, 0x02, 1, 0x00, "DI module CH0 bit4-7 filter time" },
    { kConfigBaseIndex, 0x03, 1, 0x00, "DI module CH1 bit0-3 filter time" },
    { kConfigBaseIndex, 0x04, 1, 0x00, "DI module CH1 bit4-7 filter time" },
}};

// ===========================================================================
// RP20-0016DTP (0x08) — 4 init commands
// ===========================================================================
static constexpr std::array<CoEInitCmd, 4> InitCmds_0016DTP = {{
    { kConfigBaseIndex, 0x01, 1, 0x00, "DO module CH0 Stopmode After EtherCAT Lost Link" },
    { kConfigBaseIndex, 0x02, 1, 0x00, "DO module CH0 Output Value After EtherCAT Lost Link" },
    { kConfigBaseIndex, 0x03, 1, 0x00, "DO module CH1 Stopmode After EtherCAT Lost Link" },
    { kConfigBaseIndex, 0x04, 1, 0x00, "DO module CH1 Output Value After EtherCAT Lost Link" },
}};

// ===========================================================================
// RP20-0016DTN (0x0A) — 4 init commands (identical to DTP)
// ===========================================================================
static constexpr std::array<CoEInitCmd, 4> InitCmds_0016DTN = {{
    { kConfigBaseIndex, 0x01, 1, 0x00, "DO module CH0 Stopmode After EtherCAT Lost Link" },
    { kConfigBaseIndex, 0x02, 1, 0x00, "DO module CH0 Output Value After EtherCAT Lost Link" },
    { kConfigBaseIndex, 0x03, 1, 0x00, "DO module CH1 Stopmode After EtherCAT Lost Link" },
    { kConfigBaseIndex, 0x04, 1, 0x00, "DO module CH1 Output Value After EtherCAT Lost Link" },
}};

// ===========================================================================
// RP20-0400IV (0x10) — 8 init commands
// ===========================================================================
static constexpr std::array<CoEInitCmd, 8> InitCmds_0400IV = {{
    { kConfigBaseIndex, 0x01, 1, 0x00, "CH0 Signal Form" },
    { kConfigBaseIndex, 0x02, 1, 0x00, "CH1 Signal Form" },
    { kConfigBaseIndex, 0x03, 1, 0x00, "CH2 Signal Form" },
    { kConfigBaseIndex, 0x04, 1, 0x00, "CH3 Signal Form" },
    { kConfigBaseIndex, 0x05, 1, 0x01, "CH0 Filtering Mode" },
    { kConfigBaseIndex, 0x06, 1, 0x01, "CH1 Filtering Mode" },
    { kConfigBaseIndex, 0x07, 1, 0x01, "CH2 Filtering Mode" },
    { kConfigBaseIndex, 0x08, 1, 0x01, "CH3 Filtering Mode" },
}};

// ===========================================================================
// RP20-0004IV (0x20) — 12 init commands
// ===========================================================================
static constexpr std::array<CoEInitCmd, 12> InitCmds_0004IV = {{
    { kConfigBaseIndex, 0x01, 1, 0x00, "AO CH0 Signal Form" },
    { kConfigBaseIndex, 0x02, 1, 0x00, "AO CH1 Signal Form" },
    { kConfigBaseIndex, 0x03, 1, 0x00, "AO CH2 Signal Form" },
    { kConfigBaseIndex, 0x04, 1, 0x00, "AO CH3 Signal Form" },
    { kConfigBaseIndex, 0x05, 1, 0x00, "AO CH0 Stopmode After EtherCAT Lost Link" },
    { kConfigBaseIndex, 0x06, 1, 0x00, "AO CH1 Stopmode After EtherCAT Lost Link" },
    { kConfigBaseIndex, 0x07, 1, 0x00, "AO CH2 Stopmode After EtherCAT Lost Link" },
    { kConfigBaseIndex, 0x08, 1, 0x00, "AO CH3 Stopmode After EtherCAT Lost Link" },
    { kConfigBaseIndex, 0x09, 2, 0x0000, "AO CH0 Stopvalue After EtherCAT Lost Link" },
    { kConfigBaseIndex, 0x0A, 2, 0x0000, "AO CH1 Stopvalue After EtherCAT Lost Link" },
    { kConfigBaseIndex, 0x0B, 2, 0x0000, "AO CH2 Stopvalue After EtherCAT Lost Link" },
    { kConfigBaseIndex, 0x0C, 2, 0x0000, "AO CH3 Stopvalue After EtherCAT Lost Link" },
}};

// ===========================================================================
// RP20-0400RD (0x11) — 8 init commands
// ===========================================================================
static constexpr std::array<CoEInitCmd, 8> InitCmds_0400RD = {{
    { kConfigBaseIndex, 0x01, 1, 0x00, "CH0 Signal Form" },
    { kConfigBaseIndex, 0x02, 1, 0x00, "CH1 Signal Form" },
    { kConfigBaseIndex, 0x03, 1, 0x00, "CH2 Signal Form" },
    { kConfigBaseIndex, 0x04, 1, 0x00, "CH3 Signal Form" },
    { kConfigBaseIndex, 0x05, 1, 0x01, "CH0 Filtering Mode" },
    { kConfigBaseIndex, 0x06, 1, 0x01, "CH1 Filtering Mode" },
    { kConfigBaseIndex, 0x07, 1, 0x01, "CH2 Filtering Mode" },
    { kConfigBaseIndex, 0x08, 1, 0x01, "CH3 Filtering Mode" },
}};

// ===========================================================================
// RP20-0400TC (0x12) — 12 init commands
// ===========================================================================
static constexpr std::array<CoEInitCmd, 12> InitCmds_0400TC = {{
    { kConfigBaseIndex, 0x01, 1, 0x00, "CH0 Signal Form" },
    { kConfigBaseIndex, 0x02, 1, 0x00, "CH1 Signal Form" },
    { kConfigBaseIndex, 0x03, 1, 0x00, "CH2 Signal Form" },
    { kConfigBaseIndex, 0x04, 1, 0x00, "CH3 Signal Form" },
    { kConfigBaseIndex, 0x05, 1, 0x01, "CH0 Filtering Mode" },
    { kConfigBaseIndex, 0x06, 1, 0x01, "CH1 Filtering Mode" },
    { kConfigBaseIndex, 0x07, 1, 0x01, "CH2 Filtering Mode" },
    { kConfigBaseIndex, 0x08, 1, 0x01, "CH3 Filtering Mode" },
    { kConfigBaseIndex, 0x09, 1, 0x00, "CH0 Cold Junction Compensation Mode" },
    { kConfigBaseIndex, 0x0A, 1, 0x00, "CH1 Cold Junction Compensation Mode" },
    { kConfigBaseIndex, 0x0B, 1, 0x00, "CH2 Cold Junction Compensation Mode" },
    { kConfigBaseIndex, 0x0C, 1, 0x00, "CH3 Cold Junction Compensation Mode" },
}};

// ===========================================================================
// RP20-0008DR (0x09) — 2 init commands
// ===========================================================================
static constexpr std::array<CoEInitCmd, 2> InitCmds_0008DR = {{
    { kConfigBaseIndex, 0x01, 1, 0x00, "CH0 Stopmode After EtherCAT Lost Link" },
    { kConfigBaseIndex, 0x02, 1, 0x00, "CH0 Stopvalue After EtherCAT Lost Link" },
}};

// ===========================================================================
// RP20-0202IV (0x30) — 10 init commands
// ===========================================================================
static constexpr std::array<CoEInitCmd, 10> InitCmds_0202IV = {{
    { kConfigBaseIndex, 0x01, 1, 0x00, "AI CH0 Signal Form" },
    { kConfigBaseIndex, 0x02, 1, 0x00, "AI CH1 Signal Form" },
    { kConfigBaseIndex, 0x03, 1, 0x01, "AI CH0 Filtering Mode" },
    { kConfigBaseIndex, 0x04, 1, 0x01, "AI CH1 Filtering Mode" },
    { kConfigBaseIndex, 0x05, 1, 0x00, "AO CH0 Signal Form" },
    { kConfigBaseIndex, 0x06, 1, 0x00, "AO CH1 Signal Form" },
    { kConfigBaseIndex, 0x07, 1, 0x00, "AO CH0 Stopmode After EtherCAT Lost Link" },
    { kConfigBaseIndex, 0x08, 1, 0x00, "AO CH1 Stopmode After EtherCAT Lost Link" },
    { kConfigBaseIndex, 0x09, 2, 0x0000, "AO CH0 Stopvalue After EtherCAT Lost Link" },
    { kConfigBaseIndex, 0x0A, 2, 0x0000, "AO CH1 Stopvalue After EtherCAT Lost Link" },
}};

} // namespace RP20Config
} // namespace Drives
} // namespace EtherCAT
