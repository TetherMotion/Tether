/**
 * @file FSoEPDODecoder.hpp
 * @brief Synapticon SOMANET — FSoE PDO decoding and logging helpers
 *
 * Reusable presentation layer for the Synapticon FSoE safety PDOs
 * (RxPDO 0x1700 / TxPDO 0x1B00).  Decodes the device-specific PDO structs
 * defined in tether/drives/Synapticon/SynapticonPDO.hpp into named fields
 * and logs them with ANSI color coding via ColoredBitsetFormatter.
 *
 * This code is used by the `--debug fsoe-frame`, `--debug fsoe-raw`, and
 * `--debug fsoe-wire` debug flags.  It is device-specific (it knows about
 * the Synapticon PDO layout and the zero-active / one-active bit
 * conventions) and therefore lives in the Synapticon FSoE driver rather
 * than in the generic Tether FSoE core.
 *
 * Naming conventions used by the decoders:
 *
 *   Master → Slave (RxPDO 0x1700):
 *     STO, SS1, SS2, SOS, SLS1-4, SBC use **zero-active** encoding
 *       (bit = 0 → active / safe, bit = 1 → inactive / unsafe).
 *     ErrorAck, RestartAck, ResetPosition use **one-active** encoding
 *       (bit = 1 → active).
 *     The display inverts zero-active bits so that "ON" (green) always
 *     means the safety function is active (safe).
 *
 *   Slave → Master (TxPDO 0x1B00):
 *     All flags use **one-active** encoding
 *       (bit = 1 → active, bit = 0 → inactive).  No inversion needed.
 *
 * All safety flags are shown uniformly with ANSI color coding via
 * ColoredBitsetFormatter — no opinionated pre-selection of which flags
 * are "important".  Raw hex bytes are shown AFTER the decoded meaning.
 */

#pragma once

#include <cstdint>
#include <cstdio>
#include <cstring>
#include <span>
#include <string>

#include "tether/drives/Synapticon/SynapticonPDO.hpp"
#include "tether/fsoe/FSoEDefs.hpp"
#include "tether/fsoe/FSoEHelpers.hpp"
#include "tether/utils/ColoredBitsetFormatter.hpp"
#include "logging/Logger.hpp"

namespace EtherCAT {
namespace Drives {
namespace Synapticon {
namespace FSoEDebug {

// ============================================================================
// FSoE protocol enum name helpers
// ============================================================================
//
// These are generic FSoE protocol concepts (not device-specific) and are
// provided by the FSoE driver in tether/fsoe/FSoEHelpers.hpp:
//   FSoE::fsoeStateName()   — connection state (Reset, Session, Data, ...)
//   FSoE::fsoeErrorName()   — error code (CRCError, WatchdogError, ...)
//   FSoE::fsoeCommandName() — command byte (ProcessData, Reset, ...)
//
// This header only adds the Synapticon-specific PDO struct decoding on top.

using Utils::BitLabel;
using Utils::ColoredBitsetFormatter;

// ============================================================================
// Label tables
// ============================================================================
//
// Data-driven label definitions for each flag field in the FSoE PDOs.
// All labels in a table are treated uniformly by ColoredBitsetFormatter —
// no hardcoded "primary" vs "secondary" pre-selection.  The caller picks
// which table to use and how to render it (format() for colored ON/OFF,
// formatActive() for a compact active-only list).

/// Safety flags in the master→slave RxPDO 0x1700 (safety_flags field).
/// STO/SS1/SS2/SOS/SLS/SBC are zero-active; ErrorAck/RestartAck/ResetPos
/// are one-active.
inline BitLabel kRxSafetyLabels[] = {
    BitLabel::bitInv("STO",        SynapticonPDO::SOMANET_RxPDO_1700::kSTO),
    BitLabel::bitInv("SS1",        SynapticonPDO::SOMANET_RxPDO_1700::kSS1),
    BitLabel::bitInv("SS2",        SynapticonPDO::SOMANET_RxPDO_1700::kSS2),
    BitLabel::bitInv("SOS",        SynapticonPDO::SOMANET_RxPDO_1700::kSOS),
    BitLabel::bitInv("SLS1",       SynapticonPDO::SOMANET_RxPDO_1700::kSLS_Instance1),
    BitLabel::bitInv("SLS2",       SynapticonPDO::SOMANET_RxPDO_1700::kSLS_Instance2),
    BitLabel::bitInv("SLS3",       SynapticonPDO::SOMANET_RxPDO_1700::kSLS_Instance3),
    BitLabel::bitInv("SLS4",       SynapticonPDO::SOMANET_RxPDO_1700::kSLS_Instance4),
    BitLabel::bitInv("SBC",        SynapticonPDO::SOMANET_RxPDO_1700::kSBCCommand),
    BitLabel::bit(   "ErrorAck",   SynapticonPDO::SOMANET_RxPDO_1700::kErrorAck),
    BitLabel::bit(   "RestartAck", SynapticonPDO::SOMANET_RxPDO_1700::kRestartAck),
    BitLabel::bit(   "ResetPos",   SynapticonPDO::SOMANET_RxPDO_1700::kResetPosition),
};

/// Safe outputs in the master→slave RxPDO 0x1700 (safe_outputs field).
/// One-active: bit=1 → output ON.
inline BitLabel kRxSafeOutputLabels[] = {
    BitLabel::bit("OUT1", SynapticonPDO::SOMANET_RxPDO_1700::kSafeOutput1),
    BitLabel::bit("OUT2", SynapticonPDO::SOMANET_RxPDO_1700::kSafeOutput2),
};

/// Safety state flags in the slave→master TxPDO 0x1B00 (safety_state_flags
/// field).  All one-active.
inline BitLabel kTxSafetyStateLabels[] = {
    BitLabel::bit("STO",  SynapticonPDO::SOMANET_TxPDO_1B00::kSTOState),
    BitLabel::bit("SS1",  SynapticonPDO::SOMANET_TxPDO_1B00::kSS1State),
    BitLabel::bit("SS2",  SynapticonPDO::SOMANET_TxPDO_1B00::kSS2State),
    BitLabel::bit("SOS",  SynapticonPDO::SOMANET_TxPDO_1B00::kSOSState),
    BitLabel::bit("ERR",  SynapticonPDO::SOMANET_TxPDO_1B00::kErrorState),
    BitLabel::bit("SLS1", SynapticonPDO::SOMANET_TxPDO_1B00::kSLSInstance1),
    BitLabel::bit("SLS2", SynapticonPDO::SOMANET_TxPDO_1B00::kSLSInstance2),
    BitLabel::bit("SLS3", SynapticonPDO::SOMANET_TxPDO_1B00::kSLSInstance3),
    BitLabel::bit("SLS4", SynapticonPDO::SOMANET_TxPDO_1B00::kSLSInstance4),
};

/// Diagnostic flags in the slave→master TxPDO 0x1B00 (diagnostic_flags
/// field).  All one-active.
inline BitLabel kTxDiagnosticLabels[] = {
    BitLabel::bit("SBC",         SynapticonPDO::SOMANET_TxPDO_1B00::kSBCState),
    BitLabel::bit("RestartAckReq", SynapticonPDO::SOMANET_TxPDO_1B00::kRestartAckReq),
    BitLabel::bit("TempWarn",    SynapticonPDO::SOMANET_TxPDO_1B00::kTemperatureWarning),
    BitLabel::bit("SafePosValid", SynapticonPDO::SOMANET_TxPDO_1B00::kSafePositionValid),
    BitLabel::bit("SafeSpdValid", SynapticonPDO::SOMANET_TxPDO_1B00::kSafeSpeedValid),
    BitLabel::bit("In1",         SynapticonPDO::SOMANET_TxPDO_1B00::kSafeInput1),
    BitLabel::bit("In2",         SynapticonPDO::SOMANET_TxPDO_1B00::kSafeInput2),
    BitLabel::bit("In3",         SynapticonPDO::SOMANET_TxPDO_1B00::kSafeInput3),
    BitLabel::bit("In4",         SynapticonPDO::SOMANET_TxPDO_1B00::kSafeInput4),
    BitLabel::bit("OutMon1",     SynapticonPDO::SOMANET_TxPDO_1B00::kSafeOutputMonitor1),
    BitLabel::bit("OutMon2",     SynapticonPDO::SOMANET_TxPDO_1B00::kSafeOutputMonitor2),
    BitLabel::bit("AnalogDiag",  SynapticonPDO::SOMANET_TxPDO_1B00::kAnalogDiagActive),
    BitLabel::bit("AnalogValid", SynapticonPDO::SOMANET_TxPDO_1B00::kAnalogValueValid),
};

// ============================================================================
// Safety-flag indicator label sets (packed 32-bit word)
// ============================================================================
//
// The per-frame indicator packs the two 16-bit TxPDO flag fields into one
// 32-bit word:  flags = safety_state_flags | (diagnostic_flags << 16).
// Two label sets are provided for the slave→master direction:
//
//   kTxIndicatorLabels         — real SOMANET slave: all status bits are
//                                one-active (bit=1 → active/reported).
//   kTxMirroredIndicatorLabels — verbatim-mirroring slave (e.g. the
//                                SafeMotion emulator): the function-state
//                                bits are verbatim echoes of the zero-active
//                                command bits, so they are displayed bitInv.
//
// The master→slave direction always uses kRxSafetyLabels.

/// Slave→master indicator labels for a real SOMANET slave — all one-active.
inline BitLabel kTxIndicatorLabels[] = {
    // safety_state_flags (low 16 bits)
    BitLabel::bit("STO",  SynapticonPDO::SOMANET_TxPDO_1B00::kSTOState),
    BitLabel::bit("SS1",  SynapticonPDO::SOMANET_TxPDO_1B00::kSS1State),
    BitLabel::bit("SS2",  SynapticonPDO::SOMANET_TxPDO_1B00::kSS2State),
    BitLabel::bit("SOS",  SynapticonPDO::SOMANET_TxPDO_1B00::kSOSState),
    BitLabel::bit("SLS1", SynapticonPDO::SOMANET_TxPDO_1B00::kSLSInstance1),
    BitLabel::bit("SLS2", SynapticonPDO::SOMANET_TxPDO_1B00::kSLSInstance2),
    BitLabel::bit("SLS3", SynapticonPDO::SOMANET_TxPDO_1B00::kSLSInstance3),
    BitLabel::bit("SLS4", SynapticonPDO::SOMANET_TxPDO_1B00::kSLSInstance4),
    BitLabel::bit("Err",  SynapticonPDO::SOMANET_TxPDO_1B00::kErrorState),
    // diagnostic_flags (high 16 bits)
    BitLabel::bit("SBC",          SynapticonPDO::SOMANET_TxPDO_1B00::kSBCState << 16),
    BitLabel::bit("RestartAckReq", SynapticonPDO::SOMANET_TxPDO_1B00::kRestartAckReq << 16),
    BitLabel::bit("TempWarn",     SynapticonPDO::SOMANET_TxPDO_1B00::kTemperatureWarning << 16),
    BitLabel::bit("SafePosValid", SynapticonPDO::SOMANET_TxPDO_1B00::kSafePositionValid << 16),
    BitLabel::bit("SafeSpdValid", SynapticonPDO::SOMANET_TxPDO_1B00::kSafeSpeedValid << 16),
    BitLabel::bit("In1",      SynapticonPDO::SOMANET_TxPDO_1B00::kSafeInput1 << 16),
    BitLabel::bit("In2",      SynapticonPDO::SOMANET_TxPDO_1B00::kSafeInput2 << 16),
    BitLabel::bit("In3",      SynapticonPDO::SOMANET_TxPDO_1B00::kSafeInput3 << 16),
    BitLabel::bit("In4",      SynapticonPDO::SOMANET_TxPDO_1B00::kSafeInput4 << 16),
    BitLabel::bit("OutMon1",  SynapticonPDO::SOMANET_TxPDO_1B00::kSafeOutputMonitor1 << 16),
    BitLabel::bit("OutMon2",  SynapticonPDO::SOMANET_TxPDO_1B00::kSafeOutputMonitor2 << 16),
    BitLabel::bit("AnalogDiag",  SynapticonPDO::SOMANET_TxPDO_1B00::kAnalogDiagActive << 16),
    BitLabel::bit("AnalogValid", SynapticonPDO::SOMANET_TxPDO_1B00::kAnalogValueValid << 16),
};

/// Slave→master indicator labels for a verbatim-mirroring slave (the
/// SafeMotion emulator, which echoes the command bits via
/// SafeMotion::Codec::kStatusBitMirrors).  The echoed function-state bits
/// keep the command's zero-active convention — 0 means the function is
/// active — so they are displayed bitInv.  Genuine slave status bits are
/// one-active.
inline BitLabel kTxMirroredIndicatorLabels[] = {
    // Function-request echoes — zero-active (verbatim mirror of cmd bits)
    BitLabel::bitInv("STO",  SynapticonPDO::SOMANET_TxPDO_1B00::kSTOState),
    BitLabel::bitInv("SS1",  SynapticonPDO::SOMANET_TxPDO_1B00::kSS1State),
    BitLabel::bitInv("SS2",  SynapticonPDO::SOMANET_TxPDO_1B00::kSS2State),
    BitLabel::bitInv("SOS",  SynapticonPDO::SOMANET_TxPDO_1B00::kSOSState),
    BitLabel::bitInv("SLS1", SynapticonPDO::SOMANET_TxPDO_1B00::kSLSInstance1),
    BitLabel::bitInv("SLS2", SynapticonPDO::SOMANET_TxPDO_1B00::kSLSInstance2),
    BitLabel::bitInv("SLS3", SynapticonPDO::SOMANET_TxPDO_1B00::kSLSInstance3),
    BitLabel::bitInv("SLS4", SynapticonPDO::SOMANET_TxPDO_1B00::kSLSInstance4),
    BitLabel::bitInv("SBC",  SynapticonPDO::SOMANET_TxPDO_1B00::kSBCState << 16),
    // Genuine slave status — one-active
    BitLabel::bit("Err",           SynapticonPDO::SOMANET_TxPDO_1B00::kErrorState),
    BitLabel::bit("RestartAckReq", SynapticonPDO::SOMANET_TxPDO_1B00::kRestartAckReq << 16),
    BitLabel::bit("TempWarn",      SynapticonPDO::SOMANET_TxPDO_1B00::kTemperatureWarning << 16),
    BitLabel::bit("SafePosValid",  SynapticonPDO::SOMANET_TxPDO_1B00::kSafePositionValid << 16),
    BitLabel::bit("SafeSpdValid",  SynapticonPDO::SOMANET_TxPDO_1B00::kSafeSpeedValid << 16),
    BitLabel::bit("In1",      SynapticonPDO::SOMANET_TxPDO_1B00::kSafeInput1 << 16),
    BitLabel::bit("In2",      SynapticonPDO::SOMANET_TxPDO_1B00::kSafeInput2 << 16),
    BitLabel::bit("In3",      SynapticonPDO::SOMANET_TxPDO_1B00::kSafeInput3 << 16),
    BitLabel::bit("In4",      SynapticonPDO::SOMANET_TxPDO_1B00::kSafeInput4 << 16),
    BitLabel::bit("OutMon1",  SynapticonPDO::SOMANET_TxPDO_1B00::kSafeOutputMonitor1 << 16),
    BitLabel::bit("OutMon2",  SynapticonPDO::SOMANET_TxPDO_1B00::kSafeOutputMonitor2 << 16),
    BitLabel::bit("AnalogDiag",  SynapticonPDO::SOMANET_TxPDO_1B00::kAnalogDiagActive << 16),
    BitLabel::bit("AnalogValid", SynapticonPDO::SOMANET_TxPDO_1B00::kAnalogValueValid << 16),
};

// ============================================================================
// Indicator helpers
// ============================================================================

/// Extract the packed flags word for the indicator from a raw FSoE frame.
/// The frame layout (both directions) starts with the FSoE command byte;
/// the safe data follows with CRC words interleaved:
///   command frame (11 B): [0]=cmd [1-2]=safety_flags [3-4]=crc0 ...
///   status frame (31 B):  [0]=cmd [1-2]=safety_state [3-4]=crc0
///                         [5-6]=diagnostic_flags ...
/// @param data            Raw FSoE frame bytes.
/// @param slave_to_master true for the slave→master (TxPDO 0x1B00) layout.
inline uint32_t indicatorFlags(const uint8_t* data, bool slave_to_master) {
    if (slave_to_master) {
        const auto* f =
            reinterpret_cast<const SynapticonPDO::SOMANET_TxPDO_1B00*>(data);
        return f->safety_state_flags |
               (static_cast<uint32_t>(f->diagnostic_flags) << 16);
    }
    const auto* f = reinterpret_cast<const SynapticonPDO::SOMANET_RxPDO_1700*>(data);
    return f->safety_flags;
}

/// "Known" flag-bit coverage for the opt-in unknown-bits report: the FULL
/// PDO flag definitions, so defined bits the indicator doesn't display are
/// not reported as unknown.
inline uint32_t indicatorKnownMask(bool slave_to_master) {
    if (slave_to_master) {
        return ColoredBitsetFormatter::labelCoverage(kTxSafetyStateLabels) |
               (ColoredBitsetFormatter::labelCoverage(kTxDiagnosticLabels) << 16);
    }
    return ColoredBitsetFormatter::labelCoverage(kRxSafetyLabels);
}

/// Format the safety-flag indicator as a "[NAME=ON NAME=OFF ...]" string.
/// Unknown set bits (outside the PDO flag definitions) are appended as
/// "Unknown:i,j,...".  Returns an empty string for non-process-data
/// frames — Session/Connection/Parameter/Reset carry protocol or
/// parameter payloads in the safe-data area, not safety flags.
///
/// @param data            Raw FSoE frame bytes.
/// @param slave_to_master Frame direction (selects the PDO layout).
/// @param tx_labels       Label set for the slave→master direction —
///                        kTxIndicatorLabels for a real slave,
///                        kTxMirroredIndicatorLabels for a verbatim-
///                        mirroring (emulated) slave.
/// @param color           Emit ANSI colors via ColoredBitsetFormatter
///                        (pass isatty(STDOUT_FILENO) for stream output).
inline std::string formatSafetyIndicator(const uint8_t* data,
                                         bool slave_to_master,
                                         std::span<const BitLabel> tx_labels,
                                         bool color) {
    if (data[0] != FSoE::Command::ProcessData &&
        data[0] != FSoE::Command::FailSafeData) {
        return {};
    }
    const uint32_t flags = indicatorFlags(data, slave_to_master);
    const uint32_t known = indicatorKnownMask(slave_to_master);
    const std::span<const BitLabel> labels =
        slave_to_master ? tx_labels
                        : std::span<const BitLabel>{kRxSafetyLabels};

    if (color) {
        const ColoredBitsetFormatter fmt{labels};
        return "[" + fmt.format(flags, " ", known) + "]";
    }
    std::string s = "[";
    bool first = true;
    for (const auto& label : labels) {
        if (!first) s += ' ';
        first = false;
        s += label.name;
        s += label.isActive(flags) ? "=ON" : "=OFF";
    }
    if (const std::string unk =
            ColoredBitsetFormatter::formatUnknownBits(flags, known);
        !unk.empty()) {
        s += ' ';
        s += unk;
    }
    s += "]";
    return s;
}

// ============================================================================
// Low-level formatting helpers
// ============================================================================

/// Format raw hex bytes from a buffer into a string.
inline void formatHex(char* buf, size_t bufsize, const uint8_t* data, size_t len) {
    size_t pos = 0;
    for (size_t b = 0; b < len && pos + 3 < bufsize; b++) {
        pos += static_cast<size_t>(snprintf(buf + pos, bufsize - pos, "%02X ", data[b]));
    }
    if (pos > 0 && pos < bufsize) buf[pos - 1] = '\0';  // trim trailing space
}

// ============================================================================
// Full PDO struct decoders (--debug fsoe-frame)
// ============================================================================
//
// Decodes the device-specific Synapticon FSoE PDO structs into named fields,
// showing the FSoE protocol data as the drive sees it (not raw hex).
//
// All safety flags are shown uniformly with ANSI color coding via
// ColoredBitsetFormatter, then CRCs / safe data values, and finally the
// raw hex bytes.

/// Decode the master→slave FSoE frame from the Synapticon RxPDO 0x1700 struct.
///
/// IMPORTANT: In the master→slave direction, STO/SS1/SS2/SOS/SLS/SBC use
/// **zero-active** encoding (bit=0 → active/safe, bit=1 → inactive/unsafe).
/// ErrorAck, RestartAck, ResetPosition use one-active encoding (bit=1 → active).
/// The display inverts zero-active bits so that "ON" (green) always means
/// the safety function is active (safe).
inline void dumpRxPDO(const char* tag, const SynapticonPDO::SOMANET_RxPDO_1700& rx) {
    ColoredBitsetFormatter safety_fmt{kRxSafetyLabels};
    ColoredBitsetFormatter out_fmt{kRxSafeOutputLabels};

    TETHER_LOGI(tag, "[fsoe-frame] TX→slave RxPDO 0x1700 (11 bytes):  "
                     "{}  cmd={}  conn_id=0x{:04X}",
                safety_fmt.format(rx.safety_flags),
                FSoE::fsoeCommandName(rx.fsoe_command), rx.fsoe_connection_id);

    TETHER_LOGI(tag, "  safety_flags=0x{:04X}  crc0=0x{:04X}  crc1=0x{:04X}",
                rx.safety_flags, rx.fsoe_crc_0, rx.fsoe_crc_1);

    TETHER_LOGI(tag, "  safe_outputs=0x{:02X}  {}",
                rx.safe_outputs, out_fmt.format(rx.safe_outputs));

    // --- Raw hex LAST ---
    char hex[64];
    formatHex(hex, sizeof(hex), reinterpret_cast<const uint8_t*>(&rx), sizeof(rx));
    TETHER_LOGI(tag, "  raw: {}", hex);
}

/// Decode the slave→master FSoE frame from the Synapticon TxPDO 0x1B00 struct.
///
/// In the slave→master direction, all flags use **one-active** encoding
/// (bit=1 → active, bit=0 → inactive).  No inversion needed.
inline void dumpTxPDO(const char* tag, const SynapticonPDO::SOMANET_TxPDO_1B00& tx) {
    ColoredBitsetFormatter sflags_fmt{kTxSafetyStateLabels};
    ColoredBitsetFormatter dflags_fmt{kTxDiagnosticLabels};

    TETHER_LOGI(tag, "[fsoe-frame] RX←slave TxPDO 0x1B00 (31 bytes):  "
                     "{}  cmd={}  conn_id=0x{:04X}",
                sflags_fmt.format(tx.safety_state_flags),
                FSoE::fsoeCommandName(tx.fsoe_command), tx.fsoe_connection_id);

    TETHER_LOGI(tag, "  safety_state=0x{:04X}  {}",
                tx.safety_state_flags, sflags_fmt.formatActive(tx.safety_state_flags));

    TETHER_LOGI(tag, "  diag=0x{:04X}  {}",
                tx.diagnostic_flags, dflags_fmt.format(tx.diagnostic_flags));

    TETHER_LOGI(tag,
        "  crc0=0x{:04X} crc1=0x{:04X} crc2=0x{:04X} crc3=0x{:04X} "
        "crc4=0x{:04X} crc5=0x{:04X} crc6=0x{:04X}",
        tx.fsoe_crc_0, tx.fsoe_crc_1, tx.fsoe_crc_2, tx.fsoe_crc_3,
        tx.fsoe_crc_4, tx.fsoe_crc_5, tx.fsoe_crc_6);
    TETHER_LOGI(tag,
        "  safe_pos=0x{:04X}  safe_pos_dup=0x{:04X}  "
        "safe_vel=0x{:04X}  safe_vel_dup=0x{:04X}  safe_analog=0x{:04X}",
        tx.safe_position_actual, tx.safe_position_actual_dup,
        tx.safe_velocity_actual, tx.safe_velocity_actual_dup,
        tx.safe_analog_value);

    // --- Raw hex LAST ---
    char hex[96];
    formatHex(hex, sizeof(hex), reinterpret_cast<const uint8_t*>(&tx), sizeof(tx));
    TETHER_LOGI(tag, "  raw: {}", hex);
}

// ============================================================================
// Compact frame dumpers (--debug fsoe-frame)
// ============================================================================
//
// Single-line decoded interpretation showing the safety bits, FSoE command,
// and safe motion data (position/velocity).  No raw hex or CRC details —
// use --debug fsoe-raw for the full verbose dump.

/// Compact one-line decode of the slave→master TxPDO (0x1B00).
/// Shows all safety state flags + diagnostic flags + command + safe_pos + safe_vel.
inline void dumpTxPDOFrame(const char* tag,
                           const SynapticonPDO::SOMANET_TxPDO_1B00& tx) {
    ColoredBitsetFormatter sflags_fmt{kTxSafetyStateLabels};
    ColoredBitsetFormatter dflags_fmt{kTxDiagnosticLabels};
    TETHER_LOGI(tag,
        "[fsoe-frame] RX←slave TxPDO 0x1B00 (31 bytes):  "
        "{}  {}  cmd={}  conn_id=0x{:04X}  "
        "safe_pos=0x{:04X}  safe_vel=0x{:04X}",
        sflags_fmt.format(tx.safety_state_flags),
        dflags_fmt.format(tx.diagnostic_flags),
        FSoE::fsoeCommandName(tx.fsoe_command), tx.fsoe_connection_id,
        tx.safe_position_actual, tx.safe_velocity_actual);
}

/// Compact one-line decode of the master→slave RxPDO (0x1700).
/// Shows all safety flags + safe outputs + command (no safe data in this direction).
inline void dumpRxPDOFrame(const char* tag,
                           const SynapticonPDO::SOMANET_RxPDO_1700& rx) {
    ColoredBitsetFormatter safety_fmt{kRxSafetyLabels};
    ColoredBitsetFormatter out_fmt{kRxSafeOutputLabels};
    TETHER_LOGI(tag,
        "[fsoe-frame] TX→slave RxPDO 0x1700 (11 bytes):  "
        "{}  {}  cmd={}  conn_id=0x{:04X}",
        safety_fmt.format(rx.safety_flags),
        out_fmt.format(rx.safe_outputs),
        FSoE::fsoeCommandName(rx.fsoe_command), rx.fsoe_connection_id);
}

// ============================================================================
// Compact summary dumpers (--debug fsoe-raw)
// ============================================================================
//
// Single-line summaries that show the safety flags (with color), then the
// FSoE command, then the raw hex bytes.  Used for the "on change" raw frame
// dumps.

/// One-line summary of the slave→master TxPDO (0x1B00).
/// All flags use one-active encoding (bit=1 → active/safe).
inline void dumpTxPDOSummary(const char* tag,
                             const SynapticonPDO::SOMANET_TxPDO_1B00& tx) {
    ColoredBitsetFormatter sflags_fmt{kTxSafetyStateLabels};
    ColoredBitsetFormatter dflags_fmt{kTxDiagnosticLabels};
    char hex[128];
    formatHex(hex, sizeof(hex), reinterpret_cast<const uint8_t*>(&tx), sizeof(tx));
    TETHER_LOGI(tag, "[TxPDO-FSoE slave→master] changed: {}  {}  cmd={}  | {}",
                sflags_fmt.format(tx.safety_state_flags, " ",
                    ColoredBitsetFormatter::labelCoverage(kTxSafetyStateLabels)),
                dflags_fmt.format(tx.diagnostic_flags, " ",
                    ColoredBitsetFormatter::labelCoverage(kTxDiagnosticLabels)),
                FSoE::fsoeCommandName(tx.fsoe_command), hex);
}

/// One-line summary of the master→slave RxPDO (0x1700).
/// Safety flags use zero-active encoding (bit=0 → active/safe).
inline void dumpRxPDOSummary(const char* tag,
                             const SynapticonPDO::SOMANET_RxPDO_1700& rx) {
    ColoredBitsetFormatter safety_fmt{kRxSafetyLabels};
    char hex[128];
    formatHex(hex, sizeof(hex), reinterpret_cast<const uint8_t*>(&rx), sizeof(rx));
    TETHER_LOGI(tag, "[RxPDO-FSoE master→slave] changed: {}  cmd={}  | {}",
                safety_fmt.format(rx.safety_flags, " ",
                    ColoredBitsetFormatter::labelCoverage(kRxSafetyLabels)),
                FSoE::fsoeCommandName(rx.fsoe_command), hex);
}

// ============================================================================
// Per-cycle wire dump (--debug fsoe-wire)
// ============================================================================
//
// Shows both directions in a single log line (meaning first), followed by
// the full PDO buffer hex (FSoE + motion) for each direction.

/// Per-cycle wire dump showing both directions' safety flags + command, then
/// the full combined PDO buffer hex for each direction.
///
/// @param tag           Log tag.
/// @param tx_buffer     Pointer to the slave→master FSoE PDO (0x1B00) within
///                      the combined SM3 buffer.
/// @param rx_buffer     Pointer to the master→slave FSoE PDO (0x1700) within
///                      the combined SM2 buffer.
/// @param sm2_total_len Total SM2 buffer length (FSoE + motion PDOs), for the
///                      full RxPDO hex dump.
/// @param sm3_total_len Total SM3 buffer length (FSoE + motion PDOs), for the
///                      full TxPDO hex dump.
/// @param cycle_count   Cycle counter to print (caller-managed).
inline void dumpWire(const char* tag,
                     const uint8_t* tx_buffer,
                     const uint8_t* rx_buffer,
                     size_t sm2_total_len,
                     size_t sm3_total_len,
                     uint32_t cycle_count) {
    using Tx = SynapticonPDO::SOMANET_TxPDO_1B00;
    using Rx = SynapticonPDO::SOMANET_RxPDO_1700;

    const auto* tx_pdo = reinterpret_cast<const Tx*>(tx_buffer);
    const auto* rx_pdo = reinterpret_cast<const Rx*>(rx_buffer);

    ColoredBitsetFormatter tx_sflags_fmt{kTxSafetyStateLabels};
    ColoredBitsetFormatter tx_dflags_fmt{kTxDiagnosticLabels};
    ColoredBitsetFormatter rx_safety_fmt{kRxSafetyLabels};

    TETHER_LOGI(tag, "cycle {}:  RX←slave {}  {}  cmd={}  |  TX→slave {}  cmd={}",
                cycle_count,
                tx_sflags_fmt.format(tx_pdo->safety_state_flags),
                tx_dflags_fmt.format(tx_pdo->diagnostic_flags),
                FSoE::fsoeCommandName(tx_pdo->fsoe_command),
                rx_safety_fmt.format(rx_pdo->safety_flags),
                FSoE::fsoeCommandName(rx_pdo->fsoe_command));

    // --- Raw hex LAST ---
    char hex[256];
    size_t pos;

    // TxPDO (slave-to-master) -- full PDO buffer (FSoE + motion)
    pos = 0;
    for (size_t b = 0; b < sm3_total_len && pos + 3 < sizeof(hex); b++) {
        pos += static_cast<size_t>(snprintf(hex + pos, sizeof(hex) - pos, "%02X ", tx_buffer[b]));
    }
    TETHER_LOGI(tag, "  [TxPDO full {}B] {}", sm3_total_len, hex);

    // RxPDO (master-to-slave) -- full PDO buffer (FSoE + motion)
    pos = 0;
    for (size_t b = 0; b < sm2_total_len && pos + 3 < sizeof(hex); b++) {
        pos += static_cast<size_t>(snprintf(hex + pos, sizeof(hex) - pos, "%02X ", rx_buffer[b]));
    }
    TETHER_LOGI(tag, "  [RxPDO full {}B] {}", sm2_total_len, hex);
}

} // namespace FSoEDebug
} // namespace Synapticon
} // namespace Drives
} // namespace EtherCAT
