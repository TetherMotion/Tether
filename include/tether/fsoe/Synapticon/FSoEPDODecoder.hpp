/**
 * @file FSoEPDODecoder.hpp
 * @brief Synapticon SOMANET — FSoE PDO decoding and logging helpers
 *
 * Reusable presentation layer for the Synapticon FSoE safety PDOs
 * (RxPDO 0x1700 / TxPDO 0x1B00).  Decodes the device-specific PDO structs
 * defined in tether/drives/Synapticon/SynapticonPDO.hpp into named fields
 * and logs them with ANSI color coding for the most safety-critical
 * signals (STO, SBC).
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
 * The most important safety signals — STO (Safe Torque Off) and SBC (Safe
 * Brake Control) — are shown FIRST with ANSI color coding:
 *   green = safe (bit set / enabled)
 *   red   = unsafe (bit clear / disabled)
 * Raw hex bytes are shown AFTER the decoded meaning.
 */

#pragma once

#include <cstdint>
#include <cstdio>
#include <cstring>

#include "tether/drives/Synapticon/SynapticonPDO.hpp"
#include "tether/fsoe/FSoEDefs.hpp"
#include "tether/fsoe/FSoEHelpers.hpp"
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

// ============================================================================
// ANSI color codes for terminal output
// ============================================================================

static constexpr const char* kAnsGreen  = "\033[32m";
static constexpr const char* kAnsRed    = "\033[31m";
static constexpr const char* kAnsYellow = "\033[33m";
static constexpr const char* kAnsBold   = "\033[1m";
static constexpr const char* kAnsReset  = "\033[0m";

// ============================================================================
// Low-level formatting helpers
// ============================================================================

/// Format a safety bit as green (set=safe) or red (clear=unsafe).
/// Returns a string like "STO=ON" or "STO=OFF" with ANSI color prefix.
inline void formatSafetyBit(char* buf, size_t bufsize, bool set, const char* name) {
    if (set) {
        snprintf(buf, bufsize, "%s%s=%sON%s", kAnsGreen, name, kAnsBold, kAnsReset);
    } else {
        snprintf(buf, bufsize, "%s%s=%sOFF%s", kAnsRed, name, kAnsBold, kAnsReset);
    }
}

/// Append a flag name to buf if the bit is set.
/// @param bufpos  current write position in buf (updated by caller via strlen).
inline void appendFlag(char* buf, size_t bufpos, size_t bufsize,
                       bool set, const char* name) {
    if (!set) return;
    // Prepend a space if buf is non-empty (not the first flag)
    if (bufpos > 0 && bufpos + 1 < bufsize) {
        buf[bufpos++] = ' ';
        buf[bufpos] = '\0';
    }
    size_t len = strlen(name);
    if (bufpos + len < bufsize) {
        memcpy(buf + bufpos, name, len);
        bufpos += len;
        buf[bufpos] = '\0';
    }
}

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
// STO and SBC are shown FIRST with ANSI color coding, then the remaining
// safety flags, then CRCs / safe data values, and finally the raw hex bytes.

/// Decode the master→slave FSoE frame from the Synapticon RxPDO 0x1700 struct.
///
/// IMPORTANT: In the master→slave direction, STO/SS1/SS2/SOS/SLS/SBC use
/// **zero-active** encoding (bit=0 → active/safe, bit=1 → inactive/unsafe).
/// ErrorAck, RestartAck, ResetPosition use one-active encoding (bit=1 → active).
/// The display inverts zero-active bits so that "ON" (green) always means
/// the safety function is active (safe).
inline void dumpRxPDO(const char* tag, const Synapticon_pdo::SOMANET_RxPDO_1700& rx) {
    using Rx = Synapticon_pdo::SOMANET_RxPDO_1700;

    // --- STO and SBC are what we care about most — show them FIRST ---
    // Zero-active: bit=0 → active (safe), bit=1 → inactive (unsafe)
    const bool sto_active = (rx.safety_flags & Rx::kSTO) == 0;
    const bool sbc_active = (rx.safety_flags & Rx::kSBCCommand) == 0;

    char sto_str[64], sbc_str[64];
    formatSafetyBit(sto_str, sizeof(sto_str), sto_active, "STO");
    formatSafetyBit(sbc_str, sizeof(sbc_str), sbc_active, "SBC");

    TETHER_LOGI(tag, "[fsoe-frame] TX→slave RxPDO 0x1700 (11 bytes):  "
                     "{}  {}  cmd={}  conn_id=0x{:04X}",
                sto_str, sbc_str,
                FSoE::fsoeCommandName(rx.fsoe_command), rx.fsoe_connection_id);

    // --- Other safety flags (secondary) ---
    // Zero-active bits: SS1, SS2, SOS, SLS1-4 (bit=0 → active)
    // One-active bits: ErrorAck, RestartAck, ResetPosition (bit=1 → active)
    char flags[128] = {};
    size_t pos = 0;
    appendFlag(flags, pos, sizeof(flags), (rx.safety_flags & Rx::kSS1) == 0, "SS1");
    pos = strlen(flags);
    appendFlag(flags, pos, sizeof(flags), (rx.safety_flags & Rx::kSS2) == 0, "SS2");
    pos = strlen(flags);
    appendFlag(flags, pos, sizeof(flags), (rx.safety_flags & Rx::kSOS) == 0, "SOS");
    pos = strlen(flags);
    appendFlag(flags, pos, sizeof(flags), (rx.safety_flags & Rx::kSLS_Instance1) == 0, "SLS1");
    pos = strlen(flags);
    appendFlag(flags, pos, sizeof(flags), (rx.safety_flags & Rx::kSLS_Instance2) == 0, "SLS2");
    pos = strlen(flags);
    appendFlag(flags, pos, sizeof(flags), (rx.safety_flags & Rx::kSLS_Instance3) == 0, "SLS3");
    pos = strlen(flags);
    appendFlag(flags, pos, sizeof(flags), (rx.safety_flags & Rx::kSLS_Instance4) == 0, "SLS4");
    pos = strlen(flags);
    appendFlag(flags, pos, sizeof(flags), rx.safety_flags & Rx::kErrorAck, "ErrorAck");
    pos = strlen(flags);
    appendFlag(flags, pos, sizeof(flags), rx.safety_flags & Rx::kRestartAck, "RestartAck");
    pos = strlen(flags);
    appendFlag(flags, pos, sizeof(flags), rx.safety_flags & Rx::kResetPosition, "ResetPos");
    TETHER_LOGI(tag, "  other_flags=0x{:04X} [{}]  crc0=0x{:04X}  crc1=0x{:04X}",
                rx.safety_flags, flags[0] ? flags : "(none)",
                rx.fsoe_crc_0, rx.fsoe_crc_1);

    // --- Safe outputs ---
    char outs[32] = {};
    pos = 0;
    appendFlag(outs, pos, sizeof(outs), rx.safe_outputs & Rx::kSafeOutput1, "OUT1");
    pos = strlen(outs);
    appendFlag(outs, pos, sizeof(outs), rx.safe_outputs & Rx::kSafeOutput2, "OUT2");
    TETHER_LOGI(tag, "  safe_outputs=0x{:02X} [{}]", rx.safe_outputs,
                outs[0] ? outs : "(none)");

    // --- Raw hex LAST ---
    char hex[64];
    formatHex(hex, sizeof(hex), reinterpret_cast<const uint8_t*>(&rx), sizeof(Rx));
    TETHER_LOGI(tag, "  raw: {}", hex);
}

/// Decode the slave→master FSoE frame from the Synapticon TxPDO 0x1B00 struct.
///
/// In the slave→master direction, all flags use **one-active** encoding
/// (bit=1 → active, bit=0 → inactive).  No inversion needed.
inline void dumpTxPDO(const char* tag, const Synapticon_pdo::SOMANET_TxPDO_1B00& tx) {
    using Tx = Synapticon_pdo::SOMANET_TxPDO_1B00;

    // --- STO and SBC are what we care about most — show them FIRST ---
    // One-active: bit=1 → active (safe), bit=0 → inactive (unsafe)
    // STO state is in safety_state_flags bit 0
    // SBC state is in diagnostic_flags bit 1
    const bool sto_active = (tx.safety_state_flags & Tx::kSTOState) != 0;
    const bool sbc_active = (tx.diagnostic_flags & Tx::kSBCState) != 0;

    char sto_str[64], sbc_str[64];
    formatSafetyBit(sto_str, sizeof(sto_str), sto_active, "STO");
    formatSafetyBit(sbc_str, sizeof(sbc_str), sbc_active, "SBC");

    TETHER_LOGI(tag, "[fsoe-frame] RX←slave TxPDO 0x1B00 (31 bytes):  "
                     "{}  {}  cmd={}  conn_id=0x{:04X}",
                sto_str, sbc_str,
                FSoE::fsoeCommandName(tx.fsoe_command), tx.fsoe_connection_id);

    // --- Safety state flags (secondary) ---
    char sflags[128] = {};
    size_t pos = 0;
    appendFlag(sflags, pos, sizeof(sflags), tx.safety_state_flags & Tx::kSOSState, "SOS");
    pos = strlen(sflags);
    appendFlag(sflags, pos, sizeof(sflags), tx.safety_state_flags & Tx::kSS1State, "SS1");
    pos = strlen(sflags);
    appendFlag(sflags, pos, sizeof(sflags), tx.safety_state_flags & Tx::kSS2State, "SS2");
    pos = strlen(sflags);
    appendFlag(sflags, pos, sizeof(sflags), tx.safety_state_flags & Tx::kErrorState, "ERR");
    pos = strlen(sflags);
    appendFlag(sflags, pos, sizeof(sflags), tx.safety_state_flags & Tx::kSLSInstance1, "SLS1");
    pos = strlen(sflags);
    appendFlag(sflags, pos, sizeof(sflags), tx.safety_state_flags & Tx::kSLSInstance2, "SLS2");
    pos = strlen(sflags);
    appendFlag(sflags, pos, sizeof(sflags), tx.safety_state_flags & Tx::kSLSInstance3, "SLS3");
    pos = strlen(sflags);
    appendFlag(sflags, pos, sizeof(sflags), tx.safety_state_flags & Tx::kSLSInstance4, "SLS4");
    TETHER_LOGI(tag, "  safety_state=0x{:04X} [{}]", tx.safety_state_flags,
                sflags[0] ? sflags : "(none)");

    // --- Diagnostic flags (secondary, excluding SBC which was shown above) ---
    char dflags[160] = {};
    pos = 0;
    appendFlag(dflags, pos, sizeof(dflags), tx.diagnostic_flags & Tx::kRestartAckReq, "RestartAckReq");
    pos = strlen(dflags);
    appendFlag(dflags, pos, sizeof(dflags), tx.diagnostic_flags & Tx::kTemperatureWarning, "TempWarn");
    pos = strlen(dflags);
    appendFlag(dflags, pos, sizeof(dflags), tx.diagnostic_flags & Tx::kSafePositionValid, "SafePosValid");
    pos = strlen(dflags);
    appendFlag(dflags, pos, sizeof(dflags), tx.diagnostic_flags & Tx::kSafeSpeedValid, "SafeSpdValid");
    pos = strlen(dflags);
    appendFlag(dflags, pos, sizeof(dflags), tx.diagnostic_flags & Tx::kSafeInput1, "In1");
    pos = strlen(dflags);
    appendFlag(dflags, pos, sizeof(dflags), tx.diagnostic_flags & Tx::kSafeInput2, "In2");
    pos = strlen(dflags);
    appendFlag(dflags, pos, sizeof(dflags), tx.diagnostic_flags & Tx::kSafeInput3, "In3");
    pos = strlen(dflags);
    appendFlag(dflags, pos, sizeof(dflags), tx.diagnostic_flags & Tx::kSafeInput4, "In4");
    pos = strlen(dflags);
    appendFlag(dflags, pos, sizeof(dflags), tx.diagnostic_flags & Tx::kSafeOutputMonitor1, "OutMon1");
    pos = strlen(dflags);
    appendFlag(dflags, pos, sizeof(dflags), tx.diagnostic_flags & Tx::kSafeOutputMonitor2, "OutMon2");
    pos = strlen(dflags);
    appendFlag(dflags, pos, sizeof(dflags), tx.diagnostic_flags & Tx::kAnalogDiagActive, "AnalogDiag");
    pos = strlen(dflags);
    appendFlag(dflags, pos, sizeof(dflags), tx.diagnostic_flags & Tx::kAnalogValueValid, "AnalogValid");
    TETHER_LOGI(tag, "  diag=0x{:04X} [{}]", tx.diagnostic_flags,
                dflags[0] ? dflags : "(none)");

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
    formatHex(hex, sizeof(hex), reinterpret_cast<const uint8_t*>(&tx), sizeof(Tx));
    TETHER_LOGI(tag, "  raw: {}", hex);
}

// ============================================================================
// Compact summary dumpers (--debug fsoe-raw)
// ============================================================================
//
// Single-line summaries that show the STO/SBC meaning FIRST (with color),
// then the FSoE command, then the raw hex bytes.  Used for the "on change"
// raw frame dumps.

/// One-line summary of the slave→master TxPDO (0x1B00).
/// STO/SBC use one-active encoding (bit=1 → active/safe).
inline void dumpTxPDOSummary(const char* tag,
                             const Synapticon_pdo::SOMANET_TxPDO_1B00& tx) {
    using Tx = Synapticon_pdo::SOMANET_TxPDO_1B00;
    const bool sto = (tx.safety_state_flags & Tx::kSTOState) != 0;
    const bool sbc = (tx.diagnostic_flags & Tx::kSBCState) != 0;
    char sto_str[64], sbc_str[64];
    formatSafetyBit(sto_str, sizeof(sto_str), sto, "STO");
    formatSafetyBit(sbc_str, sizeof(sbc_str), sbc, "SBC");
    char hex[128];
    formatHex(hex, sizeof(hex), reinterpret_cast<const uint8_t*>(&tx), sizeof(Tx));
    TETHER_LOGI(tag, "[TxPDO-FSoE slave→master] changed: {}  {}  cmd={}  | {}",
                sto_str, sbc_str, FSoE::fsoeCommandName(tx.fsoe_command), hex);
}

/// One-line summary of the master→slave RxPDO (0x1700).
/// STO/SBC use zero-active encoding (bit=0 → active/safe).
inline void dumpRxPDOSummary(const char* tag,
                             const Synapticon_pdo::SOMANET_RxPDO_1700& rx) {
    using Rx = Synapticon_pdo::SOMANET_RxPDO_1700;
    const bool sto = (rx.safety_flags & Rx::kSTO) == 0;
    const bool sbc = (rx.safety_flags & Rx::kSBCCommand) == 0;
    char sto_str[64], sbc_str[64];
    formatSafetyBit(sto_str, sizeof(sto_str), sto, "STO");
    formatSafetyBit(sbc_str, sizeof(sbc_str), sbc, "SBC");
    char hex[128];
    formatHex(hex, sizeof(hex), reinterpret_cast<const uint8_t*>(&rx), sizeof(Rx));
    TETHER_LOGI(tag, "[RxPDO-FSoE master→slave] changed: {}  {}  cmd={}  | {}",
                sto_str, sbc_str, FSoE::fsoeCommandName(rx.fsoe_command), hex);
}

// ============================================================================
// Per-cycle wire dump (--debug fsoe-wire)
// ============================================================================
//
// Shows both directions in a single log line (meaning first), followed by
// the full PDO buffer hex (FSoE + motion) for each direction.

/// Per-cycle wire dump showing both directions' STO/SBC + command, then the
/// full combined PDO buffer hex for each direction.
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
    using Tx = Synapticon_pdo::SOMANET_TxPDO_1B00;
    using Rx = Synapticon_pdo::SOMANET_RxPDO_1700;

    const auto* tx_pdo = reinterpret_cast<const Tx*>(tx_buffer);
    const auto* rx_pdo = reinterpret_cast<const Rx*>(rx_buffer);

    // Slave feedback (TxPDO): one-active encoding
    //   STO state in safety_state_flags bit 0 (bit=1 → active/safe)
    //   SBC state in diagnostic_flags bit 1 (bit=1 → active/safe)
    const bool tx_sto = (tx_pdo->safety_state_flags & Tx::kSTOState) != 0;
    const bool tx_sbc = (tx_pdo->diagnostic_flags & Tx::kSBCState) != 0;

    // Master command (RxPDO): zero-active encoding
    //   STO in safety_flags bit 0 (bit=0 → active/safe)
    //   SBC command in safety_flags bit 13 (bit=0 → active/safe)
    const bool rx_sto = (rx_pdo->safety_flags & Rx::kSTO) == 0;
    const bool rx_sbc = (rx_pdo->safety_flags & Rx::kSBCCommand) == 0;

    // Format with color: green=safe(ON), red=unsafe(OFF)
    char tx_sto_str[64], tx_sbc_str[64], rx_sto_str[64], rx_sbc_str[64];
    formatSafetyBit(tx_sto_str, sizeof(tx_sto_str), tx_sto, "STO");
    formatSafetyBit(tx_sbc_str, sizeof(tx_sbc_str), tx_sbc, "SBC");
    formatSafetyBit(rx_sto_str, sizeof(rx_sto_str), rx_sto, "STO");
    formatSafetyBit(rx_sbc_str, sizeof(rx_sbc_str), rx_sbc, "SBC");

    TETHER_LOGI(tag, "cycle {}:  RX←slave {}  {}  cmd={}  |  TX→slave {}  {}  cmd={}",
                cycle_count,
                tx_sto_str, tx_sbc_str, FSoE::fsoeCommandName(tx_pdo->fsoe_command),
                rx_sto_str, rx_sbc_str, FSoE::fsoeCommandName(rx_pdo->fsoe_command));

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
