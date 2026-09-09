/**
 * @file CiA402BitLabels.hpp
 * @brief ColoredBitsetFormatter label tables for CiA 402 statusword/controlword
 *
 * Provides ready-to-use `ColoredBitsetFormatter` instances for the CiA 402
 * drive statusword (0x6041) and controlword (0x6040).
 *
 * Bit definitions follow IEC 61800-7-201 / CiA 402.  The Quick stop bit
 * (controlword bit 2, statusword bit 5) is **active-low** — the bit is
 * clear when quick stop is active — so those labels use `bitInv()`.
 *
 * Bits 12-13 of the statusword are mode-specific (e.g. HomingAttained /
 * SetpointAck on bit 12, HomingError / FollowingError on bit 13).  They
 * are included with generic names since the formatter does not know the
 * active operating mode.
 *
 * Usage:
 * @code
 *   #include "tether/profiles/cia402/CiA402BitLabels.hpp"
 *
 *   // Full colored ON/OFF for all bits
 *   std::string s = CiA402::kStatuswordFormatter.format(statusword);
 *
 *   // Compact active-only list
 *   std::string a = CiA402::kControlwordFormatter.formatActive(controlword);
 * @endcode
 */

#pragma once

#include "tether/utils/ColoredBitsetFormatter.hpp"
#include "tether/profiles/cia301/CiA402Defs.hpp"

namespace CiA402 {

using ::EtherCAT::Utils::BitLabel;
using ::EtherCAT::Utils::ColoredBitsetFormatter;

// ---------------------------------------------------------------------------
// Statusword (0x6041) label table
// ---------------------------------------------------------------------------

/// CiA 402 statusword bit labels (0x6041).
/// Quick stop (bit 5) is active-low: bit clear = quick stop active.
/// Bits 12-13 are mode-specific (generic names used here).
inline BitLabel kStatuswordLabels[] = {
    BitLabel::bit(   "RdyToSwOn",  StatuswordBits::ReadyToSwitchOn),
    BitLabel::bit(   "SwitchedOn", StatuswordBits::SwitchedOn),
    BitLabel::bit(   "OpEnabled",  StatuswordBits::OperationEnabled),
    BitLabel::bit(   "Fault",      StatuswordBits::Fault),
    BitLabel::bit(   "Voltage",    StatuswordBits::VoltageEnabled),
    BitLabel::bitInv("QuickStop",  StatuswordBits::QuickStop),
    BitLabel::bit(   "SwOnDis",    StatuswordBits::SwitchOnDisabled),
    BitLabel::bit(   "Warning",    StatuswordBits::Warning),
    BitLabel::bit(   "Remote",     StatuswordBits::Remote),
    BitLabel::bit(   "Target",     StatuswordBits::TargetReached),
    BitLabel::bit(   "Limit",      StatuswordBits::InternalLimitActive),
    BitLabel::bit(   "ModeBit12",  StatuswordBits::HomingAttained),
    BitLabel::bit(   "ModeBit13",  StatuswordBits::FollowingError),
};

/// Formatter for the CiA 402 statusword.
inline const ColoredBitsetFormatter kStatuswordFormatter{kStatuswordLabels};

// ---------------------------------------------------------------------------
// Controlword (0x6040) label table
// ---------------------------------------------------------------------------

/// CiA 402 controlword bit labels (0x6040).
/// Quick stop (bit 2) is active-low: bit clear = quick stop command.
/// Bits 4-6 are mode-specific (generic names used here).
inline BitLabel kControlwordLabels[] = {
    BitLabel::bit(   "SwOn",       ControlwordBits::SwitchOn),
    BitLabel::bit(   "EnableV",    ControlwordBits::EnableVoltage),
    BitLabel::bitInv("QuickStop",  ControlwordBits::QuickStop),
    BitLabel::bit(   "EnableOp",   ControlwordBits::EnableOperation),
    BitLabel::bit(   "ModeBit4",   ControlwordBits::NewSetPoint),
    BitLabel::bit(   "ModeBit5",   ControlwordBits::ChangeSetImmediately),
    BitLabel::bit(   "ModeBit6",   ControlwordBits::AbsoluteRelative),
    BitLabel::bit(   "FaultRst",  ControlwordBits::FaultReset),
    BitLabel::bit(   "Halt",       ControlwordBits::Halt),
};

/// Formatter for the CiA 402 controlword.
inline const ColoredBitsetFormatter kControlwordFormatter{kControlwordLabels};

} // namespace CiA402
