/**
 * @file synapticon_cst_fsoe.cpp
 * @brief Synapticon SOMANET drive — CST mode + FSoE safe-motion example
 *
 * Interfaces to a Synapticon SOMANET drive (Vendor 0x22D2, CiA 402 firmware
 * v5.1.x), puts it into Cyclic Sync Torque (CST) mode, maps the SOMANET
 * PDOs for that mode (RxPDO 0x1600 / TxPDO 0x1A00), sends a sinusoidal
 * torque command (default 0.5 Nm peak-to-peak at 0.5 Hz), and runs the FSoE
 * safe-motion protocol alongside the cyclic data exchange.
 *
 * On startup the slave is automatically reset to INIT if it is currently
 * in a higher ESM state (e.g. left over from a previous run that didn't
 * shut down cleanly).  This ensures a clean starting point for mailbox
 * configuration and PDO mapping.
 *
 * FSoE frames are exchanged each cycle with the REAL drive via the FSoE
 * safety PDOs (RxPDO 0x1700 / TxPDO 0x1B00), which are mapped alongside
 * the CiA 402 motion PDOs using the multi-PDO-per-sync-manager API.  The
 * FSoE master state machine (MainInstance) builds the command frame into
 * the RxPDO 0x1700 buffer and processes the drive's safety status frame
 * from the TxPDO 0x1B00 buffer each cycle.  The drive's safety firmware
 * handles the slave side of the FSoE protocol.
 *
 * The safety layer gates torque output: if the FSoE connection is not
 * operational or motion is not allowed, target_torque is forced to 0
 * regardless of the commanded value.
 *
 * Rich diagnostics are printed each cycle:
 *   - Drive statusword, mode display, actual torque/position
 *   - FSoE connection state, error codes, frame/watchdog statistics
 *   - Safe-motion status (STO, SS1, SS2, SOS, motion-allowed)
 *
 * Usage (Linux, requires root or CAP_NET_RAW):
 *   ./synapticon_cst_fsoe                       # auto-select iface, slave 0, 10 s
 *   ./synapticon_cst_fsoe -i enx34298f762c4e    # specify interface
 *   ./synapticon_cst_fsoe -s 1 -d 30            # slave 1, 30 s
 *   ./synapticon_cst_fsoe --no-fsoe             # CST only, no FSoE
 *   ./synapticon_cst_fsoe --dc-sync             # enable DC synchronization
 *   ./synapticon_cst_fsoe --connection-id 0x0006 --safety-address 0x0006 --watchdog-ms 15
 *   ./synapticon_cst_fsoe --debug fsoe          # high-level FSoE protocol trace
 *   ./synapticon_cst_fsoe --debug fsoe-frame    # decoded FSoE PDO struct fields (on change)
 *   ./synapticon_cst_fsoe --debug fsoe-raw      # FSoE protocol trace + raw frame hex dumps (on change)
 *   ./synapticon_cst_fsoe --debug fsoe-wire     # every-cycle PDO wire dumps (firehose)
 *   ./synapticon_cst_fsoe --debug fsoe-sequence # per-cycle frame accept/reject + state change summary
 *   ./synapticon_cst_fsoe --debug fsoe-crc      # CRC parameters used for TX build and RX check
 *   ./synapticon_cst_fsoe --debug rx-pdo,tx-pdo # EtherCAT PDO data logging
 *   ./synapticon_cst_fsoe --debug coe-reads,coe-writes  # SDO access logging
 *   ./synapticon_cst_fsoe --debug al-state,pdo-sm       # state machine + SM config
 *   ./synapticon_cst_fsoe --debug help         # list all available debug flags
 *   ./synapticon_cst_fsoe --torque-nm 10 --freq-hz 1.0  # 10 Nm P-P at 1 Hz
 *   ./synapticon_cst_fsoe --rated-torque-mnm 4200       # override rated torque
 *   ./synapticon_cst_fsoe --sto 0 --sbc 0               # raw STO bit=0, SBC bit=0
 *   ./synapticon_cst_fsoe --sto 1 --sbc 1               # raw STO bit=1, SBC bit=1
 */

#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <future>
#include <iostream>
#include <memory>
#include <string>

#include "DS402ExampleSupport.hpp"
#include "common/ExampleHelpers.hpp"
#include "tether/drives/Synapticon.hpp"
#include "tether/drives/Synapticon/SynapticonPDO.hpp"
#include "tether/drives/Synapticon/SafetyDiagnostics.hpp"
#include "tether/ethercat/ALResetController.hpp"
#include "tether/ethercat/FaultDetection.hpp"
#include "tether/ethercat/Slave.hpp"
#include "tether/fsoe/FSoEDefs.hpp"
#include "tether/fsoe/Synapticon/SafeMotionFSoE.hpp"
#include "tether/platform/EspCompat.hpp"
#include "tether/profiles/cia301/CiA402Defs.hpp"
#include "tether/profiles/cia402/DS402Master.hpp"
#include "tether/ethercat/CoEManager.hpp"

#include <argparse/argparse.hpp>

namespace {

constexpr const char* TAG = "synapticon_cst_fsoe";

// ============================================================================
// Mailbox settings — from SOMANET_CiA_402_v5.1.9.xml (ESI)
// ============================================================================
//
// All three SOMANET devices in the ESI (SOMANET Node ProductCode 0x0201,
// SOMANET Circulo 0x0301, and the third device) share identical mailbox
// sync-manager definitions:
//
//   <Sm DefaultSize="1024" StartAddress="#x1000" ControlByte="#x26" ...>MBoxOut</Sm>
//   <Sm DefaultSize="1024" StartAddress="#x1400" ControlByte="#x22" ...>MBoxIn</Sm>
//
// The ControlByte values are the authoritative direction indicators:
//   0x26 = MAILBOX | DIR_WRITE | WATCHDOG  → master writes → SM0 (M→S)
//   0x22 = MAILBOX | DIR_READ  | WATCHDOG  → master reads  → SM1 (S→M)
//
// So despite the ESI "MBoxOut"/"MBoxIn" labels (master-perspective naming),
// the hardware mapping is:
//   SM0 (write, master→slave): addr=0x1000, size=1024
//   SM1 (read,  slave→master): addr=0x1400, size=1024
//
// Mailbox protocols (from <Mailbox DataLinkLayer="1">):
//   <CoE SdoInfo="1" PdoAssign="1" PdoConfig="1" .../>
//   <FoE/>
//   → CoE (0x0004) | FoE (0x0008) = 0x000C
//
// Mailbox timeouts (from <Info><Mailbox><Timeout>):
//   RequestTimeout  = 100 ms
//   ResponseTimeout = 6000 ms
//
// These constants are now provided by the Synapticon driver header
// (tether/drives/Synapticon.hpp) and are re-exported here for readability.

// SM0 — master→slave write mailbox (ESI "MBoxOut", ControlByte 0x26)
constexpr uint16_t kMailboxWriteAddr = EtherCAT::Drives::Synapticon::kMailboxWriteAddr;
// NOTE: SOMANET_CiA_402_v5.1.9.xml ESI advertises 1024-byte mailbox buffers,
// but the firmware (v5.1.7 and observed on current Synapticon drives) only
// accepts 512 bytes.  Configuring 1024 causes the slave to reject the
// mailbox configuration in PRE_OP with AL_STATUS_CODE 0x0016
// ("Invalid mailbox configuration (PRE_OP)").  Use 512 — the value the
// drive actually accepts.
constexpr uint16_t kMailboxWriteSize = 512;

// SM1 — slave→master read mailbox (ESI "MBoxIn", ControlByte 0x22)
constexpr uint16_t kMailboxReadAddr = EtherCAT::Drives::Synapticon::kMailboxReadAddr;
constexpr uint16_t kMailboxReadSize = 512;

// CoE | FoE
constexpr uint16_t kMailboxProtocols = EtherCAT::Drives::Synapticon::kMailboxProtocols;

// SDO response timeout from ESI ResponseTimeout (6000 ms)
constexpr uint32_t kSdoTimeoutMs = EtherCAT::Drives::Synapticon::kSdoTimeoutMs;

// SOMANET PDO types — from SynapticonPDO.hpp (extracted from ESI)
//
// CiA 402 motion PDOs (CST mode):
//   RxPDO 0x1600: controlword, modes_of_operation, target_torque,
//                 target_position, target_velocity, torque_offset,
//                 tuning_command  (19 bytes)
//   TxPDO 0x1A00: statusword, modes_of_operation_display, position_actual,
//                 velocity_actual, torque_actual  (13 bytes)
//
// FSoE safety PDOs:
//   RxPDO 0x1700: FSoE command frame (master→slave, 11 bytes)
//   TxPDO 0x1B00: FSoE status frame (slave→master, 31 bytes)
//
// Both sets are mapped simultaneously using the multi-PDO-per-sync-manager
// API.  ALL PDOs (including FSoE) are written explicitly to 0x1C12/0x1C13.
// FSoE PDOs come FIRST in the assignment order, then motion PDOs.
//   SM2 (Rx, master→slave): [0x1700 (11B)][0x1600 (19B)][0x1601][0x1602] = 46 bytes
//   SM3 (Tx, slave→master): [0x1B00 (31B)][0x1A00 (13B)][0x1A01][0x1A02][0x1A03] = 78 bytes
//
// FSoE PDOs come FIRST in the assignment order.  This is critical because
// the Synapticon Circulo EtherCAT chip has a bug where the last word in the
// SM buffer is zeroed.  If the FSoE PDO were last, the ConnectionID (the
// final word of the FSoE frame) would be zeroed and the slave would reject
// every frame.  By placing motion PDOs last, the zeroed word falls on
// motion data, not the FSoE ConnectionID.
// See: https://doc.synapticon.com/circulo_safe_motion/smm/ecat_fsoe_issues.htm
using RxPDO = EtherCAT::Drives::Synapticon_pdo::SOMANET_RxPDO_1600;
using TxPDO = EtherCAT::Drives::Synapticon_pdo::SOMANET_TxPDO_1A00;
using FSoERxPDO = EtherCAT::Drives::Synapticon_pdo::SOMANET_RxPDO_1700;
using FSoETxPDO = EtherCAT::Drives::Synapticon_pdo::SOMANET_TxPDO_1B00;

// PDO offsets within the combined PDO buffer.
// FSoE PDOs come FIRST (offset 0), motion PDOs follow after the FSoE PDO.
//   SM2: 0x1700 (11B) first, then 0x1600 (19B) at offset 11
//   SM3: 0x1B00 (31B) first, then 0x1A00 (13B) at offset 31
constexpr size_t kFSoERxPDOOffset   = 0;                    // FSoE first
constexpr size_t kMotionRxPDOOffset = sizeof(FSoERxPDO);    // 11 bytes
constexpr size_t kFSoETxPDOOffset   = 0;                    // FSoE first
constexpr size_t kMotionTxPDOOffset = sizeof(FSoETxPDO);    // 31 bytes

// Total SM lengths (full combined: FSoE + all motion PDOs).
constexpr size_t kSM2TotalLen = EtherCAT::Drives::Synapticon_pdo::kSM2CombinedSize;   // 46
constexpr size_t kSM3TotalLen = EtherCAT::Drives::Synapticon_pdo::kSM3CombinedSize;   // 78

using FSoEMain = EtherCAT::Drives::Synapticon::SafeMotion::MainInstance;

// ============================================================================
// Decoded name helpers
// ============================================================================

const char* fsoeStateName(uint8_t state) {
    switch (state) {
        case FSoE::ConnectionState::Reset:      return "RESET";
        case FSoE::ConnectionState::Session:    return "SESSION";
        case FSoE::ConnectionState::Connection: return "CONNECTION";
        case FSoE::ConnectionState::Parameter:  return "PARAMETER";
        case FSoE::ConnectionState::Data:       return "DATA";
        case FSoE::ConnectionState::FailSafe:   return "FAILSAFE";
        case FSoE::ConnectionState::Error:      return "ERROR";
        default:                                return "UNKNOWN";
    }
}

const char* fsoeErrorName(uint16_t code) {
    switch (code) {
        case FSoE::ErrorCode::NoError:           return "NoError";
        case FSoE::ErrorCode::CommandError:      return "CommandError";
        case FSoE::ErrorCode::CRCError:          return "CRCError";
        case FSoE::ErrorCode::WatchdogError:     return "WatchdogError";
        case FSoE::ErrorCode::SequenceError:     return "SequenceError";
        case FSoE::ErrorCode::ConnectionIDError: return "ConnectionIDError";
        case FSoE::ErrorCode::DataLengthError:   return "DataLengthError";
        case FSoE::ErrorCode::ParameterError:    return "ParameterError";
        case FSoE::ErrorCode::ApplicationError:  return "ApplicationError";
        case FSoE::ErrorCode::TimeoutError:      return "TimeoutError";
        case FSoE::ErrorCode::UnexpectedData:    return "UnexpectedData";
        case FSoE::ErrorCode::SessionError:      return "SessionError";
        case FSoE::ErrorCode::MasterTimeout:     return "MasterTimeout";
        case FSoE::ErrorCode::SlaveTimeout:      return "SlaveTimeout";
        case FSoE::ErrorCode::StartupError:      return "StartupError";
        case FSoE::ErrorCode::CommChannelError:  return "CommChannelError";
        default:                                 return "Unknown";
    }
}

const char* fsoeCommandName(uint8_t cmd) {
    switch (cmd) {
        case FSoE::Command::ProcessData:    return "ProcessData(0x36)";
        case FSoE::Command::Reset:          return "Reset(0x2A)";
        case FSoE::Command::Session:        return "Session(0x4E)";
        case FSoE::Command::Connection:     return "Connection(0x64)";
        case FSoE::Command::Parameter:      return "Parameter(0x52)";
        case FSoE::Command::FailSafeData:   return "FailSafeData(0x08)";
        default:                            return "Unknown";
    }
}

void hexDump(const char* tag, const char* label, const uint8_t* data, size_t len) {
    constexpr size_t kBytesPerLine = 16;
    char hex[kBytesPerLine * 3 + 1];
    for (size_t i = 0; i < len; i += kBytesPerLine) {
        size_t pos = 0;
        for (size_t j = i; j < i + kBytesPerLine && j < len; j++) {
            pos += snprintf(hex + pos, sizeof(hex) - pos, "%02X ", data[j]);
        }
        TETHER_LOGI(tag, "  %s [%3zu/%3zu]: %s", label, i, len, hex);
    }
}

// ============================================================================
// FSoE frame decoder (--debug fsoe-frame)
// ============================================================================
//
// Decodes the device-specific Synapticon FSoE PDO structs into named fields,
// showing the FSoE protocol data as the drive sees it (not raw hex).
//
// The most important safety signals — STO (Safe Torque Off) and SBC (Safe
// Brake Control) — are shown FIRST with ANSI color coding:
//   green = safe (bit set / enabled)
//   red   = unsafe (bit clear / disabled)
// Raw hex bytes are shown AFTER the decoded meaning.

// ANSI color codes for terminal output
static constexpr const char* kAnsGreen  = "\033[32m";
static constexpr const char* kAnsRed    = "\033[31m";
static constexpr const char* kAnsYellow = "\033[33m";
static constexpr const char* kAnsBold   = "\033[1m";
static constexpr const char* kAnsReset  = "\033[0m";

/// Format a safety bit as green (set=safe) or red (clear=unsafe).
/// Returns a string like "ON" or "OFF" with ANSI color prefix.
static void formatSafetyBit(char* buf, size_t bufsize, bool set, const char* name) {
    if (set) {
        snprintf(buf, bufsize, "%s%s=%sON%s", kAnsGreen, name, kAnsBold, kAnsReset);
    } else {
        snprintf(buf, bufsize, "%s%s=%sOFF%s", kAnsRed, name, kAnsBold, kAnsReset);
    }
}

/// Append a flag name to buf if the bit is set.
static void appendFlag(char* buf, size_t bufpos, size_t bufsize,
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
static void formatHex(char* buf, size_t bufsize, const uint8_t* data, size_t len) {
    size_t pos = 0;
    for (size_t b = 0; b < len && pos + 3 < bufsize; b++) {
        pos += static_cast<size_t>(snprintf(buf + pos, bufsize - pos, "%02X ", data[b]));
    }
    if (pos > 0 && pos < bufsize) buf[pos - 1] = '\0';  // trim trailing space
}

/// Decode the master→slave FSoE frame from the Synapticon RxPDO 0x1700 struct.
/// Shows STO/SBC command status FIRST (green=safe, red=unsafe), then raw hex.
///
/// IMPORTANT: In the master→slave direction, STO/SS1/SS2/SOS/SLS/SBC use
/// **zero-active** encoding (bit=0 → active/safe, bit=1 → inactive/unsafe).
/// ErrorAck, RestartAck, ResetPosition use one-active encoding (bit=1 → active).
/// The display inverts zero-active bits so that "ON" (green) always means
/// the safety function is active (safe).
void dumpFSoERxPDO(const char* tag, const FSoERxPDO& rx) {
    // --- STO and SBC are what we care about most — show them FIRST ---
    // Zero-active: bit=0 → active (safe), bit=1 → inactive (unsafe)
    const bool sto_active = (rx.safety_flags & FSoERxPDO::kSTO) == 0;
    const bool sbc_active = (rx.safety_flags & FSoERxPDO::kSBCCommand) == 0;

    char sto_str[64], sbc_str[64];
    formatSafetyBit(sto_str, sizeof(sto_str), sto_active, "STO");
    formatSafetyBit(sbc_str, sizeof(sbc_str), sbc_active, "SBC");

    TETHER_LOGI(tag, "[fsoe-frame] TX→slave RxPDO 0x1700 (11 bytes):  "
                     "%s  %s  cmd=%s  conn_id=0x%04X",
                sto_str, sbc_str,
                fsoeCommandName(rx.fsoe_command), rx.fsoe_connection_id);

    // --- Other safety flags (secondary) ---
    // Zero-active bits: SS1, SS2, SOS, SLS1-4 (bit=0 → active)
    // One-active bits: ErrorAck, RestartAck, ResetPosition (bit=1 → active)
    char flags[128] = {};
    size_t pos = 0;
    appendFlag(flags, pos, sizeof(flags), (rx.safety_flags & FSoERxPDO::kSS1) == 0, "SS1");
    pos = strlen(flags);
    appendFlag(flags, pos, sizeof(flags), (rx.safety_flags & FSoERxPDO::kSS2) == 0, "SS2");
    pos = strlen(flags);
    appendFlag(flags, pos, sizeof(flags), (rx.safety_flags & FSoERxPDO::kSOS) == 0, "SOS");
    pos = strlen(flags);
    appendFlag(flags, pos, sizeof(flags), (rx.safety_flags & FSoERxPDO::kSLS_Instance1) == 0, "SLS1");
    pos = strlen(flags);
    appendFlag(flags, pos, sizeof(flags), (rx.safety_flags & FSoERxPDO::kSLS_Instance2) == 0, "SLS2");
    pos = strlen(flags);
    appendFlag(flags, pos, sizeof(flags), (rx.safety_flags & FSoERxPDO::kSLS_Instance3) == 0, "SLS3");
    pos = strlen(flags);
    appendFlag(flags, pos, sizeof(flags), (rx.safety_flags & FSoERxPDO::kSLS_Instance4) == 0, "SLS4");
    pos = strlen(flags);
    appendFlag(flags, pos, sizeof(flags), rx.safety_flags & FSoERxPDO::kErrorAck, "ErrorAck");
    pos = strlen(flags);
    appendFlag(flags, pos, sizeof(flags), rx.safety_flags & FSoERxPDO::kRestartAck, "RestartAck");
    pos = strlen(flags);
    appendFlag(flags, pos, sizeof(flags), rx.safety_flags & FSoERxPDO::kResetPosition, "ResetPos");
    TETHER_LOGI(tag, "  other_flags=0x%04X [%s]  crc0=0x%04X  crc1=0x%04X",
                rx.safety_flags, flags[0] ? flags : "(none)",
                rx.fsoe_crc_0, rx.fsoe_crc_1);

    // --- Safe outputs ---
    char outs[32] = {};
    pos = 0;
    appendFlag(outs, pos, sizeof(outs), rx.safe_outputs & FSoERxPDO::kSafeOutput1, "OUT1");
    pos = strlen(outs);
    appendFlag(outs, pos, sizeof(outs), rx.safe_outputs & FSoERxPDO::kSafeOutput2, "OUT2");
    TETHER_LOGI(tag, "  safe_outputs=0x%02X [%s]", rx.safe_outputs,
                outs[0] ? outs : "(none)");

    // --- Raw hex LAST ---
    char hex[64];
    formatHex(hex, sizeof(hex), reinterpret_cast<const uint8_t*>(&rx), sizeof(FSoERxPDO));
    TETHER_LOGI(tag, "  raw: %s", hex);
}

/// Decode the slave→master FSoE frame from the Synapticon TxPDO 0x1B00 struct.
/// Shows STO/SBC feedback status FIRST (green=safe, red=unsafe), then raw hex.
///
/// In the slave→master direction, all flags use **one-active** encoding
/// (bit=1 → active, bit=0 → inactive).  No inversion needed.
void dumpFSoETxPDO(const char* tag, const FSoETxPDO& tx) {
    // --- STO and SBC are what we care about most — show them FIRST ---
    // One-active: bit=1 → active (safe), bit=0 → inactive (unsafe)
    // STO state is in safety_state_flags bit 0
    // SBC state is in diagnostic_flags bit 1
    const bool sto_active = (tx.safety_state_flags & FSoETxPDO::kSTOState) != 0;
    const bool sbc_active = (tx.diagnostic_flags & FSoETxPDO::kSBCState) != 0;

    char sto_str[64], sbc_str[64];
    formatSafetyBit(sto_str, sizeof(sto_str), sto_active, "STO");
    formatSafetyBit(sbc_str, sizeof(sbc_str), sbc_active, "SBC");

    TETHER_LOGI(tag, "[fsoe-frame] RX←slave TxPDO 0x1B00 (31 bytes):  "
                     "%s  %s  cmd=%s  conn_id=0x%04X",
                sto_str, sbc_str,
                fsoeCommandName(tx.fsoe_command), tx.fsoe_connection_id);

    // --- Safety state flags (secondary) ---
    char sflags[128] = {};
    size_t pos = 0;
    appendFlag(sflags, pos, sizeof(sflags), tx.safety_state_flags & FSoETxPDO::kSOSState, "SOS");
    pos = strlen(sflags);
    appendFlag(sflags, pos, sizeof(sflags), tx.safety_state_flags & FSoETxPDO::kSS1State, "SS1");
    pos = strlen(sflags);
    appendFlag(sflags, pos, sizeof(sflags), tx.safety_state_flags & FSoETxPDO::kSS2State, "SS2");
    pos = strlen(sflags);
    appendFlag(sflags, pos, sizeof(sflags), tx.safety_state_flags & FSoETxPDO::kErrorState, "ERR");
    pos = strlen(sflags);
    appendFlag(sflags, pos, sizeof(sflags), tx.safety_state_flags & FSoETxPDO::kSLSInstance1, "SLS1");
    pos = strlen(sflags);
    appendFlag(sflags, pos, sizeof(sflags), tx.safety_state_flags & FSoETxPDO::kSLSInstance2, "SLS2");
    pos = strlen(sflags);
    appendFlag(sflags, pos, sizeof(sflags), tx.safety_state_flags & FSoETxPDO::kSLSInstance3, "SLS3");
    pos = strlen(sflags);
    appendFlag(sflags, pos, sizeof(sflags), tx.safety_state_flags & FSoETxPDO::kSLSInstance4, "SLS4");
    TETHER_LOGI(tag, "  safety_state=0x%04X [%s]", tx.safety_state_flags,
                sflags[0] ? sflags : "(none)");

    // --- Diagnostic flags (secondary, excluding SBC which was shown above) ---
    char dflags[160] = {};
    pos = 0;
    appendFlag(dflags, pos, sizeof(dflags), tx.diagnostic_flags & FSoETxPDO::kRestartAckReq, "RestartAckReq");
    pos = strlen(dflags);
    appendFlag(dflags, pos, sizeof(dflags), tx.diagnostic_flags & FSoETxPDO::kTemperatureWarning, "TempWarn");
    pos = strlen(dflags);
    appendFlag(dflags, pos, sizeof(dflags), tx.diagnostic_flags & FSoETxPDO::kSafePositionValid, "SafePosValid");
    pos = strlen(dflags);
    appendFlag(dflags, pos, sizeof(dflags), tx.diagnostic_flags & FSoETxPDO::kSafeSpeedValid, "SafeSpdValid");
    pos = strlen(dflags);
    appendFlag(dflags, pos, sizeof(dflags), tx.diagnostic_flags & FSoETxPDO::kSafeInput1, "In1");
    pos = strlen(dflags);
    appendFlag(dflags, pos, sizeof(dflags), tx.diagnostic_flags & FSoETxPDO::kSafeInput2, "In2");
    pos = strlen(dflags);
    appendFlag(dflags, pos, sizeof(dflags), tx.diagnostic_flags & FSoETxPDO::kSafeInput3, "In3");
    pos = strlen(dflags);
    appendFlag(dflags, pos, sizeof(dflags), tx.diagnostic_flags & FSoETxPDO::kSafeInput4, "In4");
    pos = strlen(dflags);
    appendFlag(dflags, pos, sizeof(dflags), tx.diagnostic_flags & FSoETxPDO::kSafeOutputMonitor1, "OutMon1");
    pos = strlen(dflags);
    appendFlag(dflags, pos, sizeof(dflags), tx.diagnostic_flags & FSoETxPDO::kSafeOutputMonitor2, "OutMon2");
    pos = strlen(dflags);
    appendFlag(dflags, pos, sizeof(dflags), tx.diagnostic_flags & FSoETxPDO::kAnalogDiagActive, "AnalogDiag");
    pos = strlen(dflags);
    appendFlag(dflags, pos, sizeof(dflags), tx.diagnostic_flags & FSoETxPDO::kAnalogValueValid, "AnalogValid");
    TETHER_LOGI(tag, "  diag=0x%04X [%s]", tx.diagnostic_flags,
                dflags[0] ? dflags : "(none)");

    TETHER_LOGI(tag,
        "  crc0=0x%04X crc1=0x%04X crc2=0x%04X crc3=0x%04X "
        "crc4=0x%04X crc5=0x%04X crc6=0x%04X",
        tx.fsoe_crc_0, tx.fsoe_crc_1, tx.fsoe_crc_2, tx.fsoe_crc_3,
        tx.fsoe_crc_4, tx.fsoe_crc_5, tx.fsoe_crc_6);
    TETHER_LOGI(tag,
        "  safe_pos=0x%04X  safe_pos_dup=0x%04X  "
        "safe_vel=0x%04X  safe_vel_dup=0x%04X  safe_analog=0x%04X",
        tx.safe_position_actual, tx.safe_position_actual_dup,
        tx.safe_velocity_actual, tx.safe_velocity_actual_dup,
        tx.safe_analog_value);

    // --- Raw hex LAST ---
    char hex[96];
    formatHex(hex, sizeof(hex), reinterpret_cast<const uint8_t*>(&tx), sizeof(FSoETxPDO));
    TETHER_LOGI(tag, "  raw: %s", hex);
}

// ============================================================================
// Sine-wave torque motion controller for CST mode
// ============================================================================
//
// Generates a sinusoidal torque command at the specified frequency and
// peak-to-peak amplitude.  The torque is converted from Nm to CiA 402
// per-mille units (0.1% of rated torque) using the motor's rated torque
// (object 0x6076, in mNm):
//
//   target_torque_permille = (torque_Nm * 1'000'000) / rated_torque_mNm
//
// The safety layer (FSoE) gates the actual torque output at the drive
// level — if the FSoE connection is not in the Data state, the drive's
// safety firmware inhibits torque regardless of the commanded value.

template <typename PDO>
class SineTorqueController final : public EtherCAT::DS402Master::IDriveMotionController {
public:
    /// @param amplitude_nm   Peak torque amplitude in Nm (half of peak-to-peak)
    /// @param frequency_hz   Sine frequency in Hz
    /// @param rated_torque_mnm  Motor rated torque in mNm (from object 0x6076)
    SineTorqueController(double amplitude_nm, double frequency_hz,
                         uint32_t rated_torque_mnm)
        : amplitude_nm_(amplitude_nm)
        , frequency_hz_(frequency_hz)
        , rated_torque_mnm_(rated_torque_mnm)
    {
        // Pre-compute the per-mille scaling factor:
        //   permille = Nm * 1e6 / mNm
        if (rated_torque_mnm > 0) {
            nm_to_permille_ = 1'000'000.0 / static_cast<double>(rated_torque_mnm);
        }
    }

    bool start(EtherCAT::CiA402Drive& drive) override {
        return drive.setOperatingMode(CiA402::OperatingMode::CyclicSyncTorque);
    }

    void stop(EtherCAT::CiA402Drive&) override {}

    bool update(EtherCAT::CiA402Drive& drive, double dt_seconds) override {
        // Motion PDO is at offset kMotionRxPDOOffset (FSoE comes first, motion second)
        auto* rx = reinterpret_cast<PDO*>(
            static_cast<uint8_t*>(drive.getRxPDOBuffer()) + kMotionRxPDOOffset);
        if (rx == nullptr) return false;

        // Advance the phase accumulator
        elapsed_s_ += dt_seconds;

        // Compute the sine torque command in Nm, then convert to per-mille
        const double torque_nm =
            amplitude_nm_ * std::sin(2.0 * M_PI * frequency_hz_ * elapsed_s_);
        const double torque_permille = torque_nm * nm_to_permille_;

        // Clamp to INT16 range (-32768..32767 per-mille = ±3276.8% rated)
        constexpr double kMaxPermille = 32767.0;
        constexpr double kMinPermille = -32768.0;
        const double clamped = (torque_permille > kMaxPermille) ? kMaxPermille :
                               (torque_permille < kMinPermille) ? kMinPermille :
                               torque_permille;

        rx->controlword = static_cast<uint16_t>(CiA402::ControlWord::ENABLE_OPERATION);
        rx->modes_of_operation = CiA402::OperatingMode::CyclicSyncTorque;

        if constexpr (requires(PDO& pdo) { pdo.target_torque; }) {
            rx->target_torque = static_cast<int16_t>(clamped);
        }
        if constexpr (requires(PDO& pdo) { pdo.target_velocity; }) {
            rx->target_velocity = 0;
        }
        if constexpr (requires(PDO& pdo) { pdo.target_position; }) {
            rx->target_position = 0;
        }

        last_torque_nm_ = torque_nm;
        return true;
    }

    /// Returns the most recently commanded torque in Nm (for diagnostics)
    double lastTorqueNm() const { return last_torque_nm_; }

private:
    double   amplitude_nm_      = 0.0;
    double   frequency_hz_      = 0.0;
    uint32_t rated_torque_mnm_  = 0;
    double   nm_to_permille_    = 0.0;  // per-mille per Nm
    double   elapsed_s_         = 0.0;
    double   last_torque_nm_    = 0.0;
};

// ============================================================================
// FSoE + drive diagnostics cyclic task
// ============================================================================

class FSoEDiagnosticsTask final : public EtherCAT::DS402Master::ICyclicTask {
public:
    FSoEDiagnosticsTask(uint16_t slave_index,
                        FSoEMain& fsoe_main,
                        uint32_t interval_ms)
        : slave_index_(slave_index)
        , fsoe_main_(fsoe_main)
        , interval_ms_(interval_ms)
    {
    }

    bool update(EtherCAT::DS402Master& master, double dt_seconds) override {
        elapsed_ms_ += static_cast<uint64_t>(dt_seconds * 1000.0);
        if (elapsed_ms_ - last_print_ms_ < interval_ms_) return true;
        last_print_ms_ = elapsed_ms_;

        auto* drive = master.driveBySlaveIndex(slave_index_);
        if (drive == nullptr) return true;

        // Motion PDO is at offset kMotionTxPDOOffset (FSoE comes first, motion second)
        auto* tx = reinterpret_cast<const TxPDO*>(
            static_cast<const uint8_t*>(drive->getTxPDOBuffer()) + kMotionTxPDOOffset);
        // Also read the commanded target_torque from the RxPDO for comparison
        auto* rx = reinterpret_cast<const RxPDO*>(
            static_cast<const uint8_t*>(drive->getRxPDOBuffer()) + kMotionRxPDOOffset);
        if (tx) {
            TETHER_LOGI(TAG,
                "--- Drive @ %llu ms ---",
                static_cast<unsigned long long>(elapsed_ms_));
            TETHER_LOGI(TAG,
                "  statusword=0x%04X mode_display=%d "
                "target_torque=%d torque_actual=%d "
                "position_actual=%lld",
                tx->statusword,
                static_cast<int>(tx->modes_of_operation_display),
                rx ? static_cast<int>(rx->target_torque) : 0,
                static_cast<int>(tx->torque_actual),
                static_cast<long long>(tx->position_actual));
        }

        auto& conn = fsoe_main_.rawConnection();
        const auto status = conn.getStatus();
        const auto stats = conn.getStats();

        TETHER_LOGI(TAG,
            "--- FSoE @ %llu ms ---",
            static_cast<unsigned long long>(elapsed_ms_));
        TETHER_LOGI(TAG,
            "  state=%s(%u) operational=%d fail_safe=%d data_valid=%d",
            fsoeStateName(status.state), status.state,
            status.isOperational() ? 1 : 0,
            status.isFailSafe() ? 1 : 0,
            status.data_valid ? 1 : 0);
        TETHER_LOGI(TAG,
            "  session_id=0x%04X rx_seq=%u watchdog=%u ms",
            status.session_id, status.sequence_number,
            status.watchdog_counter);
        if (status.hasError()) {
            TETHER_LOGW(TAG,
                "  ERROR: 0x%04X (%s)",
                status.error_code, fsoeErrorName(status.error_code));
        }
        TETHER_LOGI(TAG,
            "  frames: tx=%u rx=%u | crc_err=%u seq_err=%u watchdog_evt=%u "
            "reset_evt=%u timeout_evt=%u dup=%u invalid=%u",
            stats.frames_sent, stats.frames_received,
            stats.crc_errors, stats.sequence_errors, stats.watchdog_events,
            stats.reset_events, stats.timeout_events,
            stats.duplicate_frames, stats.invalid_frames);
        TETHER_LOGI(TAG,
            "  recovery: attempts=%u successful=%u",
            stats.recovery_attempts, stats.successful_recoveries);

        if (fsoe_main_.hasStatus()) {
            const auto& sm = fsoe_main_.status();
            TETHER_LOGI(TAG,
                "  safe-motion: motion_allowed=%d sto=%d ss1=%d ss2=%d "
                "sos=%d error=%d",
                sm.motionAllowed() ? 1 : 0,
                sm.sto_active ? 1 : 0,
                sm.ss1_active ? 1 : 0,
                sm.ss2_active ? 1 : 0,
                sm.sos_active ? 1 : 0,
                sm.error_active ? 1 : 0);
        }

        return true;
    }

private:
    uint16_t slave_index_;
    FSoEMain& fsoe_main_;
    uint32_t interval_ms_;
    uint64_t elapsed_ms_ = 0;
    uint64_t last_print_ms_ = 0;
};

// ============================================================================
// Safety state confirmation monitor
// ============================================================================
//
// Monitors the drive's FSoE status feedback to confirm that commanded STO/SBC
// state changes have been acknowledged by the drive.  When the operator uses
// --sto / --sbc overrides, this task logs:
//   - When the command is first applied (commanded state)
//   - When the drive confirms the state (status matches command)
//   - A warning if the drive does NOT confirm within a timeout
//
// The monitor tracks the expected state from the Command struct and compares
// it against the Status struct each cycle once FSoE reaches the Data state.

class SafetyStateMonitor final : public EtherCAT::DS402Master::ICyclicTask {
public:
    SafetyStateMonitor(FSoEMain& fsoe_main,
                       bool track_sto, bool track_sbc,
                       bool expected_sto, bool expected_sbc,
                       uint32_t confirm_timeout_ms = 3000)
        : fsoe_main_(fsoe_main)
        , track_sto_(track_sto)
        , track_sbc_(track_sbc)
        , expected_sto_(expected_sto)
        , expected_sbc_(expected_sbc)
        , confirm_timeout_ms_(confirm_timeout_ms)
    {
    }

    bool update(EtherCAT::DS402Master& /*master*/, double dt_seconds) override {
        if (!track_sto_ && !track_sbc_) return true;

        elapsed_ms_ += static_cast<uint64_t>(dt_seconds * 1000.0);

        // Only check once FSoE has reached Data state and we have status.
        if (!fsoe_main_.hasStatus()) return true;
        const auto& conn_status = fsoe_main_.rawConnection().getStatus();
        if (!conn_status.isOperational()) return true;

        // Record the timestamp when we first see Data state.
        if (data_state_entered_ms_ == 0) {
            data_state_entered_ms_ = elapsed_ms_;
        }

        const auto& sm = fsoe_main_.status();

        // --- STO confirmation ---
        if (track_sto_ && !sto_confirmed_) {
            // STO is "active" in the status when torque is inhibited.
            // We expect sto_active == expected_sto_.
            if (sm.sto_active == expected_sto_) {
                sto_confirmed_ = true;
                TETHER_LOGI(TAG,
                    "[safety-confirm] STO %s confirmed by drive after %llu ms "
                    "(sto_active=%d)",
                    expected_sto_ ? "ON (torque inhibited)" : "OFF (torque allowed)",
                    static_cast<unsigned long long>(
                        elapsed_ms_ - data_state_entered_ms_),
                    sm.sto_active ? 1 : 0);
            } else if (!sto_warned_ &&
                       (elapsed_ms_ - data_state_entered_ms_) > confirm_timeout_ms_) {
                sto_warned_ = true;
                TETHER_LOGW(TAG,
                    "[safety-confirm] STO NOT confirmed after %u ms! "
                    "Commanded STO=%s but drive reports sto_active=%d "
                    "(expected %d). The drive may be ignoring the command "
                    "bit or using a different polarity/position.",
                    confirm_timeout_ms_,
                    expected_sto_ ? "ON" : "OFF",
                    sm.sto_active ? 1 : 0,
                    expected_sto_ ? 1 : 0);
            }
        }

        // --- SBC confirmation ---
        if (track_sbc_ && !sbc_confirmed_) {
            // brake_engaged in status reflects the SBC state.
            if (sm.brake_engaged == expected_sbc_) {
                sbc_confirmed_ = true;
                TETHER_LOGI(TAG,
                    "[safety-confirm] SBC %s confirmed by drive after %llu ms "
                    "(brake_engaged=%d)",
                    expected_sbc_ ? "ENGAGED" : "RELEASED",
                    static_cast<unsigned long long>(
                        elapsed_ms_ - data_state_entered_ms_),
                    sm.brake_engaged ? 1 : 0);
            } else if (!sbc_warned_ &&
                       (elapsed_ms_ - data_state_entered_ms_) > confirm_timeout_ms_) {
                sbc_warned_ = true;
                TETHER_LOGW(TAG,
                    "[safety-confirm] SBC NOT confirmed after %u ms! "
                    "Commanded SBC=%s but drive reports brake_engaged=%d "
                    "(expected %d). The brake may not be responding or the "
                    "drive uses a different bit polarity/position.",
                    confirm_timeout_ms_,
                    expected_sbc_ ? "ENGAGED" : "RELEASED",
                    sm.brake_engaged ? 1 : 0,
                    expected_sbc_ ? 1 : 0);
            }
        }

        return true;
    }

private:
    FSoEMain& fsoe_main_;
    bool track_sto_;
    bool track_sbc_;
    bool expected_sto_;
    bool expected_sbc_;
    uint32_t confirm_timeout_ms_;
    uint64_t elapsed_ms_ = 0;
    uint64_t data_state_entered_ms_ = 0;
    bool sto_confirmed_ = false;
    bool sbc_confirmed_ = false;
    bool sto_warned_ = false;
    bool sbc_warned_ = false;
};

// ============================================================================
// FSoE PDO exchange cyclic task (real drive via PDOs)
// ============================================================================
//
// Exchanges FSoE frames with the real Synapticon drive via the FSoE safety
// PDOs (RxPDO 0x1700 / TxPDO 0x1B00) that are mapped alongside the CiA 402
// motion PDOs in the combined PDO buffer.  The FSoE PDOs are at a fixed
// offset within the buffer (after the motion PDO data).

class FSoEPDOExchangeTask final : public EtherCAT::DS402Master::ICyclicTask {
public:
    FSoEPDOExchangeTask(uint16_t slave_index,
                        FSoEMain& main_instance,
                        size_t rx_pdo_offset,
                        size_t tx_pdo_offset,
                        bool debug_raw = false,
                        bool debug_frame = false,
                        bool debug_wire = false)
        : slave_index_(slave_index)
        , main_instance_(main_instance)
        , rx_pdo_offset_(rx_pdo_offset)
        , tx_pdo_offset_(tx_pdo_offset)
        , debug_raw_(debug_raw)
        , debug_frame_(debug_frame)
        , debug_wire_(debug_wire)
    {}

    bool update(EtherCAT::DS402Master& master, double dt_seconds) override {
        if (!main_instance_.featureEnabled()) {
            return true;
        }

        elapsed_time_ms_ += static_cast<uint64_t>(dt_seconds * 1000.0);

        auto* drive = master.driveBySlaveIndex(slave_index_);
        if (drive == nullptr) {
            return true;  // don't stop the process
        }

        // Access the FSoE PDO region within the combined PDO buffer.
        // The FSoE PDO (0x1700/0x1B00) comes FIRST at offset 0; the motion
        // PDO (0x1600/0x1A00) follows after the FSoE PDO.
        uint8_t* rx_buffer = static_cast<uint8_t*>(drive->getRxPDOBuffer()) + rx_pdo_offset_;
        const uint8_t* tx_buffer = static_cast<const uint8_t*>(drive->getTxPDOBuffer()) + tx_pdo_offset_;

        // --debug fsoe-wire: dump every cycle, unconditionally.
        // This is the "firehose" mode for seeing the raw PDO wire bytes
        // on every single cycle, even when nothing changes.
        if (debug_wire_) {
            dumpWire(tx_buffer, rx_buffer);
        }

        // --debug fsoe-raw / fsoe-frame: dump only when the FSoE frame
        // content changes.  Compares the current TxPDO (slave-to-master) and
        // RxPDO (master-to-slave) FSoE regions against the last seen copies.
        const bool tx_changed = (debug_raw_ || debug_frame_) &&
            std::memcmp(tx_buffer, last_tx_.data(), sizeof(FSoETxPDO)) != 0;
        if (tx_changed) {
            std::memcpy(last_tx_.data(), tx_buffer, sizeof(FSoETxPDO));
        }

        // --debug fsoe-raw: STO/SBC meaning first, then hex dump on change
        if (debug_raw_ && tx_changed) {
            const auto* tx_pdo = reinterpret_cast<const FSoETxPDO*>(tx_buffer);
            const bool tx_sto = (tx_pdo->safety_state_flags & FSoETxPDO::kSTOState) != 0;
            const bool tx_sbc = (tx_pdo->diagnostic_flags & FSoETxPDO::kSBCState) != 0;
            char sto_str[64], sbc_str[64];
            formatSafetyBit(sto_str, sizeof(sto_str), tx_sto, "STO");
            formatSafetyBit(sbc_str, sizeof(sbc_str), tx_sbc, "SBC");
            char hex[128];
            formatHex(hex, sizeof(hex), tx_buffer, sizeof(FSoETxPDO));
            TETHER_LOGI("fsoe-cyclic", "[TxPDO-FSoE slave→master] changed: %s  %s  cmd=%s  | %s",
                        sto_str, sbc_str, fsoeCommandName(tx_pdo->fsoe_command), hex);
        }

        // --debug fsoe-frame: decoded struct dump on change
        if (debug_frame_ && tx_changed) {
            const auto* tx_pdo = reinterpret_cast<const FSoETxPDO*>(tx_buffer);
            dumpFSoETxPDO(TAG, *tx_pdo);
        }

        const bool ok = main_instance_.exchangeViaPDO(
            rx_buffer, sizeof(FSoERxPDO),
            tx_buffer, sizeof(FSoETxPDO),
            elapsed_time_ms_);

        // RxPDO (master-to-slave) is checked AFTER exchangeViaPDO -- it
        // contains what the master just built for this cycle.
        const bool rx_changed = (debug_raw_ || debug_frame_) &&
            std::memcmp(rx_buffer, last_rx_.data(), sizeof(FSoERxPDO)) != 0;
        if (rx_changed) {
            std::memcpy(last_rx_.data(), rx_buffer, sizeof(FSoERxPDO));
        }

        if (debug_frame_ && rx_changed) {
            const auto* rx_pdo = reinterpret_cast<const FSoERxPDO*>(rx_buffer);
            dumpFSoERxPDO(TAG, *rx_pdo);
        }

        if (debug_raw_ && rx_changed) {
            const auto* rx_pdo = reinterpret_cast<const FSoERxPDO*>(rx_buffer);
            // Zero-active: bit=0 → active (safe)
            const bool rx_sto = (rx_pdo->safety_flags & FSoERxPDO::kSTO) == 0;
            const bool rx_sbc = (rx_pdo->safety_flags & FSoERxPDO::kSBCCommand) == 0;
            char sto_str[64], sbc_str[64];
            formatSafetyBit(sto_str, sizeof(sto_str), rx_sto, "STO");
            formatSafetyBit(sbc_str, sizeof(sbc_str), rx_sbc, "SBC");
            char hex[128];
            formatHex(hex, sizeof(hex), rx_buffer, sizeof(FSoERxPDO));
            TETHER_LOGI("fsoe-cyclic", "[RxPDO-FSoE master→slave] changed: %s  %s  cmd=%s  | %s",
                        sto_str, sbc_str, fsoeCommandName(rx_pdo->fsoe_command), hex);
        }

        return ok;
    }

private:
    uint16_t slave_index_;
    FSoEMain& main_instance_;
    size_t rx_pdo_offset_;
    size_t tx_pdo_offset_;
    bool debug_raw_ = false;
    bool debug_frame_ = false;
    bool debug_wire_ = false;
    uint64_t elapsed_time_ms_ = 0;
    uint32_t cycle_count_ = 0;
    // Last-seen FSoE PDO content for change detection
    std::array<uint8_t, sizeof(FSoETxPDO)> last_tx_{};
    std::array<uint8_t, sizeof(FSoERxPDO)> last_rx_{};

    void dumpWire(const uint8_t* tx_buffer, const uint8_t* rx_buffer) {
        // --- Show MEANING first: STO/SBC from both directions ---
        const auto* tx_pdo = reinterpret_cast<const FSoETxPDO*>(tx_buffer);
        const auto* rx_pdo = reinterpret_cast<const FSoERxPDO*>(rx_buffer);

        // Slave feedback (TxPDO): one-active encoding
        //   STO state in safety_state_flags bit 0 (bit=1 → active/safe)
        //   SBC state in diagnostic_flags bit 1 (bit=1 → active/safe)
        const bool tx_sto = (tx_pdo->safety_state_flags & FSoETxPDO::kSTOState) != 0;
        const bool tx_sbc = (tx_pdo->diagnostic_flags & FSoETxPDO::kSBCState) != 0;

        // Master command (RxPDO): zero-active encoding
        //   STO in safety_flags bit 0 (bit=0 → active/safe)
        //   SBC command in safety_flags bit 13 (bit=0 → active/safe)
        const bool rx_sto = (rx_pdo->safety_flags & FSoERxPDO::kSTO) == 0;
        const bool rx_sbc = (rx_pdo->safety_flags & FSoERxPDO::kSBCCommand) == 0;

        // Format with color: green=safe(ON), red=unsafe(OFF)
        char tx_sto_str[64], tx_sbc_str[64], rx_sto_str[64], rx_sbc_str[64];
        formatSafetyBit(tx_sto_str, sizeof(tx_sto_str), tx_sto, "STO");
        formatSafetyBit(tx_sbc_str, sizeof(tx_sbc_str), tx_sbc, "SBC");
        formatSafetyBit(rx_sto_str, sizeof(rx_sto_str), rx_sto, "STO");
        formatSafetyBit(rx_sbc_str, sizeof(rx_sbc_str), rx_sbc, "SBC");

        TETHER_LOGI("fsoe-wire", "cycle %u:  RX←slave %s  %s  cmd=%s  |  TX→slave %s  %s  cmd=%s",
                    cycle_count_,
                    tx_sto_str, tx_sbc_str, fsoeCommandName(tx_pdo->fsoe_command),
                    rx_sto_str, rx_sbc_str, fsoeCommandName(rx_pdo->fsoe_command));

        // --- Raw hex LAST ---
        char hex[256];
        size_t pos;

        // TxPDO (slave-to-master) -- full PDO buffer (FSoE + CST)
        pos = 0;
        for (size_t b = 0; b < kSM3TotalLen && pos + 3 < sizeof(hex); b++) {
            pos += static_cast<size_t>(snprintf(hex + pos, sizeof(hex) - pos, "%02X ", tx_buffer[b]));
        }
        TETHER_LOGI("fsoe-wire", "  [TxPDO full %zuB] %s", kSM3TotalLen, hex);

        // RxPDO (master-to-slave) -- full PDO buffer (FSoE + CST)
        pos = 0;
        for (size_t b = 0; b < kSM2TotalLen && pos + 3 < sizeof(hex); b++) {
            pos += static_cast<size_t>(snprintf(hex + pos, sizeof(hex) - pos, "%02X ", rx_buffer[b]));
        }
        TETHER_LOGI("fsoe-wire", "  [RxPDO full %zuB] %s", kSM2TotalLen, hex);

        cycle_count_++;
    }
};

// ============================================================================
// Default PDO / SM / FMMU diagnostic dump
// ============================================================================
//
// Reads the drive's default PDO assignment (0x1C12/0x1C13), SyncManager
// registers (0x0810–0x081F), and FMMU registers (0x0600–0x063F) via SDO
// and register access.  This shows the drive's power-on defaults before
// the master overwrites them — useful for understanding what the drive
// expects for FSoE + motion PDO configuration.
//
// Call this right after PRE_OP transition, before any PDO configuration.

static void dumpDefaultPdoConfig(EtherCAT::Master& ec_master,
                                  uint16_t slave_idx,
                                  EtherCAT::CoE::CoEManager& sdo) {
    constexpr uint32_t kSdoTimeoutMs = 1000;
    const auto slave_addr = EtherCAT::Master::slaveAddressFromADP(
        EtherCAT::Master::adpForSlaveIndex(slave_idx));

    TETHER_LOGI(TAG, "===== Default PDO/SM/FMMU dump (slave %u) =====", slave_idx);

    // --- 0x1C12 (SM2 RxPDO assignment) ---
    {
        auto cnt = sdo.readU8(0x1C12, 0, {.timeout_ms = kSdoTimeoutMs});
        if (cnt.has_value()) {
            TETHER_LOGI(TAG, "  0x1C12 (SM2 RxPDO assign): %u entries", cnt.value());
            for (uint8_t i = 1; i <= cnt.value() && i <= 16; i++) {
                auto entry = sdo.readU16(0x1C12, i, {.timeout_ms = kSdoTimeoutMs});
                if (entry.has_value())
                    TETHER_LOGI(TAG, "    0x1C12:%u = 0x%04X", i, entry.value());
                else
                    TETHER_LOGW(TAG, "    0x1C12:%u FAILED", i);
            }
        } else {
            TETHER_LOGW(TAG, "  0x1C12:0 FAILED (no default RxPDO assignment)");
        }
    }

    // --- 0x1C13 (SM3 TxPDO assignment) ---
    {
        auto cnt = sdo.readU8(0x1C13, 0, {.timeout_ms = kSdoTimeoutMs});
        if (cnt.has_value()) {
            TETHER_LOGI(TAG, "  0x1C13 (SM3 TxPDO assign): %u entries", cnt.value());
            for (uint8_t i = 1; i <= cnt.value() && i <= 16; i++) {
                auto entry = sdo.readU16(0x1C13, i, {.timeout_ms = kSdoTimeoutMs});
                if (entry.has_value())
                    TETHER_LOGI(TAG, "    0x1C13:%u = 0x%04X", i, entry.value());
                else
                    TETHER_LOGW(TAG, "    0x1C13:%u FAILED", i);
            }
        } else {
            TETHER_LOGW(TAG, "  0x1C13:0 FAILED (no default TxPDO assignment)");
        }
    }

    // --- SyncManager registers ---
    // SM0: 0x0800-0x0807, SM1: 0x0808-0x080F, SM2: 0x0810-0x0817, SM3: 0x0818-0x081F
    const char* sm_names[] = {"SM0 (MBoxOut)", "SM1 (MBoxIn)", "SM2 (Outputs)", "SM3 (Inputs)"};
    for (int sm = 0; sm < 4; sm++) {
        uint16_t base = 0x0800 + static_cast<uint16_t>(sm) * 8;
        uint16_t start_addr = 0;
        uint16_t length = 0;
        uint8_t control = 0;
        uint8_t status = 0;
        uint8_t activate = 0;
        uint8_t pdi_control = 0;

        ec_master.readRegister(slave_addr, base,     &start_addr, 2, 200);
        ec_master.readRegister(slave_addr, base + 2, &length, 2, 200);
        ec_master.readRegister(slave_addr, base + 4, &control, 1, 200);
        ec_master.readRegister(slave_addr, base + 5, &status, 1, 200);
        ec_master.readRegister(slave_addr, base + 6, &activate, 1, 200);
        ec_master.readRegister(slave_addr, base + 7, &pdi_control, 1, 200);

        TETHER_LOGI(TAG, "  %s: start=0x%04X len=%u ctrl=0x%02X status=0x%02X act=0x%02X pdi=0x%02X",
                    sm_names[sm], start_addr, length, control, status, activate, pdi_control);
    }

    // --- FMMU registers ---
    // FMMU0: 0x0600-0x060F, FMMU1: 0x0610-0x061F, FMMU2: 0x0620-0x062F, FMMU3: 0x0630-0x063F
    for (int fmmu = 0; fmmu < 4; fmmu++) {
        uint16_t base = 0x0600 + static_cast<uint16_t>(fmmu) * 16;
        uint32_t log_addr = 0;
        uint16_t length = 0;
        uint16_t phys_addr = 0;
        uint8_t fmmu_type = 0;  // bit0=read, bit1=write, bit3=enable

        ec_master.readRegister(slave_addr, base,     &log_addr, 4, 200);
        ec_master.readRegister(slave_addr, base + 4, &length, 2, 200);
        ec_master.readRegister(slave_addr, base + 6, &phys_addr, 2, 200);
        ec_master.readRegister(slave_addr, base + 8, &fmmu_type, 1, 200);

        bool enabled = fmmu_type & 0x08;
        bool read_access = fmmu_type & 0x01;
        bool write_access = fmmu_type & 0x02;

        if (enabled || log_addr != 0 || length != 0) {
            const char* dir = (read_access && write_access) ? "RW" :
                              read_access ? "R" : write_access ? "W" : "--";
            TETHER_LOGI(TAG, "  FMMU%d: log=0x%08X phys=0x%04X len=%u type=0x%02X [%s %s]",
                        fmmu, log_addr, phys_addr, length, fmmu_type,
                        enabled ? "EN" : "DIS", dir);
        } else {
            TETHER_LOGI(TAG, "  FMMU%d: (unused)", fmmu);
        }
    }

    // --- Read individual PDO mapping objects for the assigned PDOs ---
    // For each PDO index in 0x1C12/0x1C13, read 0x1A00:0 etc. to get the
    // mapping entry count and total size.
    auto dump_pdo_mapping = [&](uint16_t pdo_idx, const char* label) {
        auto cnt = sdo.readU8(pdo_idx, 0, {.timeout_ms = kSdoTimeoutMs});
        if (!cnt.has_value()) {
            TETHER_LOGW(TAG, "  %s (0x%04X): FAILED to read count", label, pdo_idx);
            return;
        }
        uint16_t total_bits = 0;
        TETHER_LOGI(TAG, "  %s (0x%04X): %u entries", label, pdo_idx, cnt.value());
        for (uint8_t i = 1; i <= cnt.value() && i <= 32; i++) {
            auto entry = sdo.readU32(pdo_idx, i, {.timeout_ms = kSdoTimeoutMs});
            if (entry.has_value()) {
                uint32_t v = entry.value();
                uint16_t idx = (v >> 16) & 0xFFFF;
                uint8_t sub = (v >> 8) & 0xFF;
                uint8_t bits = v & 0xFF;
                total_bits += bits;
                TETHER_LOGI(TAG, "    %s:%u = 0x%08X (idx=0x%04X sub=%u bits=%u)",
                            label, i, v, idx, sub, bits);
            } else {
                TETHER_LOGW(TAG, "    %s:%u FAILED", label, i);
            }
        }
        TETHER_LOGI(TAG, "    → total: %u bits = %u bytes", total_bits, total_bits / 8);
    };

    // Read mappings for all relevant PDOs
    dump_pdo_mapping(0x1600, "RxPDO_1600");
    dump_pdo_mapping(0x1601, "RxPDO_1601");
    dump_pdo_mapping(0x1602, "RxPDO_1602");
    dump_pdo_mapping(0x1700, "FSoE_RxPDO_1700");
    dump_pdo_mapping(0x1A00, "TxPDO_1A00");
    dump_pdo_mapping(0x1A01, "TxPDO_1A01");
    dump_pdo_mapping(0x1A02, "TxPDO_1A02");
    dump_pdo_mapping(0x1A03, "TxPDO_1A03");
    dump_pdo_mapping(0x1B00, "FSoE_TxPDO_1B00");

    TETHER_LOGI(TAG, "===== End default PDO/SM/FMMU dump =====");
}

// ============================================================================
// Main
// ============================================================================

struct Args {
    std::string interface;
    int slave_index = 0;
    double duration = 10.0;
    bool enable_fsoe = true;
    bool enable_dc_sync = false;
    uint16_t connection_id = 0x0006;  ///< Must match drive's Device Safety Address (0xF980:1)
    uint16_t safety_address = 0x0006; ///< FSoE slave safety address (0x2620:3)
    uint16_t watchdog_ms = EtherCAT::Drives::Synapticon::SafeMotion::Timing::kMinimumWatchdogTimeMs;
    uint32_t diag_interval_ms = 1000;
    std::string debug;
    double torque_pp_nm = 0.5;       ///< Peak-to-peak torque amplitude in Nm
    double freq_hz = 0.5;            ///< Sine wave frequency in Hz
    uint32_t rated_torque_mnm = 0;   ///< Motor rated torque in mNm (0 = auto-detect from 0x6076)
    ///< STO override: -1 = not set (use motionEnabled default), 0 = force STO off, 1 = force STO on
    int sto_override = -1;
    ///< SBC override: -1 = not set (use motionEnabled default), 0 = force brake released, 1 = force brake engaged
    int sbc_override = -1;
};

bool parseArgs(int argc, char** argv, Args& out) {
    argparse::ArgumentParser program("synapticon_cst_fsoe");
    program.add_argument("-i", "--interface")
        .default_value(std::string(""))
        .help("Network interface (e.g. eth0, enx34298f762c4e). "
              "If omitted, auto-selects the sole physical Ethernet interface.");
    program.add_argument("-s", "--slave")
        .scan<'i', int>()
        .default_value(0)
        .help("Slave index on the bus (0-based)");
    program.add_argument("-d", "--duration")
        .scan<'g', double>()
        .default_value(10.0)
        .help("Duration in seconds");
    program.add_argument("--no-fsoe")
        .default_value(false)
        .implicit_value(true)
        .help("Disable FSoE safe-motion (CST-only mode)");
    program.add_argument("--dc-sync")
        .default_value(false)
        .implicit_value(true)
        .help("Enable EtherCAT distributed-clock synchronization (off by default)");
    program.add_argument("--connection-id")
        .scan<'x', unsigned int>()
        .default_value(static_cast<unsigned int>(0x0006))
        .help("FSoE connection ID (hex, default 0x0006 — must match drive's "
              "Device Safety Address 0xF980:1)");
    program.add_argument("--safety-address")
        .scan<'x', unsigned int>()
        .default_value(static_cast<unsigned int>(0x0006))
        .help("FSoE slave safety address (hex, default 0x0006 — from drive's "
              "0x2620:3)");
    program.add_argument("--watchdog-ms")
        .scan<'i', int>()
        .default_value(static_cast<int>(
            EtherCAT::Drives::Synapticon::SafeMotion::Timing::kMinimumWatchdogTimeMs))
        .help("FSoE watchdog timeout in ms");
    program.add_argument("--diag-interval-ms")
        .scan<'i', int>()
        .default_value(1000)
        .help("Diagnostics print interval in ms");
    program.add_argument("--debug")
        .default_value(std::string(""))
        .help("Comma-separated debug flags. Use '--debug help' for a list.");
    program.add_argument("--torque-nm")
        .scan<'g', double>()
        .default_value(0.5)
        .help("Peak-to-peak sine torque amplitude in Nm (default 0.5 = ±0.25 Nm)");
    program.add_argument("--freq-hz")
        .scan<'g', double>()
        .default_value(0.5)
        .help("Sine torque frequency in Hz (default 0.5)");
    program.add_argument("--rated-torque-mnm")
        .scan<'i', int>()
        .default_value(static_cast<int>(0))
        .help("Motor rated torque in mNm (0 = auto-detect from object 0x6076)");
    program.add_argument("--sto")
        .scan<'i', int>()
        .default_value(-1)
        .help("Set raw STO bit value in the FSoE PDO: 0 = bit cleared, 1 = bit set. "
              "Default (-1) uses motionEnabled() default. "
              "NOTE: the codec uses 0-active encoding, so bit=0 means STO active "
              "(torque off) and bit=1 means STO inactive (torque allowed).");
    program.add_argument("--sbc")
        .scan<'i', int>()
        .default_value(-1)
        .help("Set raw SBC bit value in the FSoE PDO: 0 = bit cleared, 1 = bit set. "
              "Default (-1) uses motionEnabled() default. "
              "NOTE: the codec uses 0-active encoding, so bit=0 means SBC active "
              "(brake engaged) and bit=1 means SBC inactive (brake released).");

    try {
        program.parse_args(argc, argv);
    } catch (const std::runtime_error& err) {
        std::cerr << err.what() << "\n" << program;
        return false;
    }

    out.interface = Tether::Examples::resolveInterface(program.get<std::string>("--interface"), TAG);
    out.slave_index = program.get<int>("--slave");
    out.duration = program.get<double>("--duration");
    out.enable_fsoe = !program.get<bool>("--no-fsoe");
    out.enable_dc_sync = program.get<bool>("--dc-sync");
    out.connection_id = static_cast<uint16_t>(program.get<unsigned int>("--connection-id"));
    out.safety_address = static_cast<uint16_t>(program.get<unsigned int>("--safety-address"));
    out.watchdog_ms = static_cast<uint16_t>(program.get<int>("--watchdog-ms"));
    out.diag_interval_ms = static_cast<uint32_t>(program.get<int>("--diag-interval-ms"));
    out.debug = program.get<std::string>("--debug");
    out.torque_pp_nm = program.get<double>("--torque-nm");
    out.freq_hz = program.get<double>("--freq-hz");
    out.rated_torque_mnm = static_cast<uint32_t>(program.get<int>("--rated-torque-mnm"));
    out.sto_override = program.get<int>("--sto");
    out.sbc_override = program.get<int>("--sbc");
    return true;
}

// ----------------------------------------------------------------------------
// Pre-activation safety check (currently disabled — see call site in main)
// ----------------------------------------------------------------------------
// Reads 0x2611 (Safety Module input diagnostics) and 0x2620:2 ("Safe fieldbus"
// FSoE active indicator) from the drive via SDO, then decides whether to
// proceed with activation.
//
// When FSoE is enabled, the drive *starts* in safe state (STO active by
// default) and the FSoE master brings it out of safe state once the safety
// protocol reaches the Data state.  Aborting here would prevent the FSoE
// connection from ever establishing, so we only abort on safe state when
// FSoE is disabled (--no-fsoe) — in that case there is no mechanism to
// clear the safe state and enabling the drive would be futile.
//
// Returns 0 on success (proceed with activation), non-zero on failure
// (caller should abort and shut down).
int preActivationSafetyCheck(EtherCAT::DS402Master& master,
                             Tether::Examples::HostMasterSession& session,
                             uint16_t slave_idx,
                             bool enable_fsoe) {
    auto& slave = master.ethercatMaster().slave(slave_idx);
    const auto safety = EtherCAT::Drives::Synapticon::readSafetyModuleState(slave);

    TETHER_LOGI(TAG,
        "Safety module diagnostics (0x2611): input1=%u input2=%u -> %s",
        static_cast<unsigned>(safety.input1),
        static_cast<unsigned>(safety.input2),
        safety.stateSummary());

    // 0x2620:2 "Safe fieldbus" reports whether the FSoE connection is
    // active on the drive.  Logged up front so the operator can see the
    // FSoE state regardless of the safety verdict below.
    TETHER_LOGI(TAG,
        "FSoE active indicator (0x2620:2 \"Safe fieldbus\"): raw=%u -> %s",
        static_cast<unsigned>(safety.safe_fieldbus),
        safety.fsoeStateSummary());

    if (!safety.ok) {
        if (enable_fsoe) {
            // SDO read failed, but FSoE is enabled — the safety module
            // might be in a state where SDO access is temporarily
            // unavailable.  Continue anyway; the FSoE protocol will
            // handle the safety state via PDOs.
            TETHER_LOGW(TAG,
                "Failed to read safety module diagnostics (0x2611) via SDO — "
                "continuing with FSoE enabled (PDO-based safety handling)");
        } else {
            TETHER_LOGE(TAG,
                "Failed to read safety module diagnostics (0x2611) via SDO — "
                "cannot verify safety state, aborting activation");
            Tether::Examples::stopHostMasterSession(master, session);
            return 2;
        }
    }

    if (safety.isInSafeState()) {
        if (enable_fsoe) {
            // FSoE is enabled — the drive is expected to start in safe
            // state.  The FSoE master will bring it out of safe state
            // once the safety protocol reaches the Data state.  Log the
            // state and continue; do NOT abort.
            TETHER_LOGI(TAG,
                "Drive is in SAFE STATE (safety function active, motion "
                "inhibited) — FSoE is %s (0x2620:2=%u) — continuing; "
                "the FSoE master will clear the safe state once the "
                "safety protocol reaches the Data state",
                safety.fsoeStateSummary(),
                static_cast<unsigned>(safety.safe_fieldbus));
        } else {
            // FSoE is disabled — there is no mechanism to clear the safe
            // state, so enabling the drive would be futile.  Abort.
            TETHER_LOGE(TAG,
                "Drive is in SAFE STATE (safety function active, motion "
                "inhibited) and FSoE is disabled (--no-fsoe) — there is "
                "no mechanism to clear the safe state, refusing to "
                "activate drive, triggering shutdown");
            Tether::Examples::stopHostMasterSession(master, session);
            return 2;
        }
    } else {
        TETHER_LOGI(TAG,
            "Safety check passed: safety module reports motion allowed");
    }

    return 0;
}

} // namespace

int main(int argc, char** argv) {
    Args args;
    if (!parseArgs(argc, argv, args)) return 1;

    if (args.debug == "help") {
        std::cout << "Available --debug flags (comma-separated):\n";
        for (const auto& info : EtherCAT::debug::allDebugFlags()) {
            std::cout << "  " << info.name << "\n      " << info.description << "\n";
        }
        std::cout << "\nFilter syntax:\n"
                  << "  --debug flagname:(slaves:0,2,5),otherflag:(slaves:1-3)\n"
                  << "  (default: pass-all for every flag)\n";
        return 0;
    }

    if (args.slave_index < 0 || args.slave_index > 65535) {
        std::cerr << "Invalid slave index\n";
        return 1;
    }
    const uint16_t slave_idx = static_cast<uint16_t>(args.slave_index);

    Tether::Platform::ensureRealtimeKernelOrExit();

    TETHER_LOGI(TAG,
        "synapticon_cst_fsoe — interface=%s slave=%u duration=%.1f fsoe=%s dc_sync=%s debug='%s' "
        "torque_pp=%.3fNm freq=%.3fHz rated_torque_mnm=%u "
        "conn_id=0x%04X safety_addr=0x%04X "
        "sto_override=%s sbc_override=%s",
        args.interface.c_str(), slave_idx, args.duration,
        args.enable_fsoe ? "on" : "off",
        args.enable_dc_sync ? "on" : "off",
        args.debug.c_str(),
        args.torque_pp_nm, args.freq_hz, args.rated_torque_mnm,
        args.connection_id, args.safety_address,
        args.sto_override < 0 ? "default" : std::to_string(args.sto_override).c_str(),
        args.sbc_override < 0 ? "default" : std::to_string(args.sbc_override).c_str());

    // --- Start EtherCAT master ---
    EtherCAT::DS402Master master;
    Tether::Examples::HostMasterSession session;
    if (!Tether::Examples::startHostMasterSession(args.interface, master, session, TAG)) {
        return 2;
    }

    // --- Configure mailbox with SOMANET ESI values ---
    // The SOMANET_CiA_402_v5.1.9.xml ESI defines the mailbox sync managers
    // with 1024-byte buffers at 0x1000 (SM0, M→S write) and 0x1400 (SM1,
    // S→M read), supporting CoE + FoE.  In practice the drive firmware only
    // accepts 512-byte buffers, so kMailboxWriteSize/kMailboxReadSize are
    // overridden to 512 above.  These values are used instead of relying on
    // SII EEPROM auto-configuration so the correct mailbox geometry is
    // always used for SOMANET drives.
    {
        if (!master.ethercatMaster().discoverSlaves()) {
            TETHER_LOGW(TAG, "No slaves discovered during pre-config scan");
        }
        if (!master.waitForDriveCount(
                static_cast<uint16_t>(slave_idx + 1), 2000)) {
            TETHER_LOGE(TAG, "Timed out waiting for slave %u", slave_idx);
            Tether::Examples::stopHostMasterSession(master, session);
            return 2;
        }

        // --- Apply EtherCAT framework debug flags ---
        // The --debug string may contain both FSoE-specific flags (fsoe,
        // fsoe-frame, etc.) and EtherCAT framework flags (rx-pdo, coe-reads,
        // al-state, etc.).  The FSoE flags are parsed separately below via
        // string search; the framework flags are applied here via the shared
        // ExampleHelpers parser, which supports per-slave filter syntax.
        {
            const auto debug_flags =
                Tether::Examples::parseDebugFlags(args.debug);
            Tether::Examples::applyDebugFlags(
                debug_flags, master.ethercatMaster(), TAG);
        }

        // --- Slave identity verification is intentionally skipped ---
        // The Synapticon drive does not support APWR to the EEPCTL register,
        // so SII/EEPROM reads are impossible via the standard register
        // interface.  This example targets SOMANET drives only and relies on
        // the hardcoded ESI values from tether/drives/Synapticon.hpp for all
        // mailbox/PDO configuration — no SII read or vendor/product
        // verification is performed.

        // --- Reset slave to INIT if currently in a higher state ---
        // If the slave is already in PRE_OP, SAFE_OP, or OP (e.g. from a
        // previous run that didn't shut down cleanly), bring it back to INIT
        // before reconfiguring the mailbox and PDO mapping.  This ensures a
        // clean starting point regardless of the slave's current state.
        {
            uint8_t current_state = 0;
            if (master.ethercatMaster().readSlaveApplicationLayerState(
                    slave_idx, current_state)) {
                TETHER_LOGI(TAG,
                    "Slave %u current AL state: 0x%02X (%s)",
                    slave_idx, current_state,
                    EtherCAT::getECStateName(static_cast<EtherCAT::ECState>(current_state)));

                if (current_state != static_cast<uint8_t>(EtherCAT::ECState::Init)) {
                    TETHER_LOGI(TAG,
                        "Slave %u is not in INIT (0x%02X) — resetting to INIT "
                        "before configuration",
                        slave_idx, current_state);

                    EtherCAT::ALResetController reset_ctrl(master.ethercatMaster());
                    reset_ctrl.setProgressCallback(
                        [](uint16_t si, int iter, int max_iter,
                           uint16_t al, uint16_t code, bool reached) {
                            if (!reached) {
                                const char* state_name =
                                    EtherCAT::al_status_get_state_name(al);
                                const bool has_err = EtherCAT::al_status_has_error(al);
                                TETHER_LOGI(TAG,
                                    "  Slave %u reset iter %d/%d: "
                                    "AL_STATUS=0x%04X (state=%s, error=%s)",
                                    si, iter, max_iter, al,
                                    state_name, has_err ? "true" : "false");
                                if (code != 0) {
                                    TETHER_LOGI(TAG,
                                        "    AL_STATUS_CODE=0x%04X (%s)",
                                        code, EtherCAT::getALStatusCodeName(code));
                                }
                            }
                        });

                    const auto result = reset_ctrl.resetSlave(
                        slave_idx, static_cast<uint8_t>(EtherCAT::ECState::Init));

                    if (result.success) {
                        TETHER_LOGI(TAG,
                            "Slave %u reset to INIT OK (%s, %d iterations)",
                            slave_idx, result.message.c_str(),
                            result.iterations_used);
                    } else {
                        TETHER_LOGE(TAG,
                            "Slave %u reset to INIT FAILED (%s, %d iterations, "
                            "final AL_STATUS=0x%04X, AL_STATUS_CODE=0x%04X)",
                            slave_idx, result.message.c_str(),
                            result.iterations_used,
                            result.final_al_status,
                            result.final_al_status_code);
                        Tether::Examples::stopHostMasterSession(master, session);
                        return 2;
                    }

                    // Give the slave a moment to settle after the reset.
                    Tether::Platform::Clock::instance().delayMilliseconds(100);
                }
            } else {
                TETHER_LOGW(TAG,
                    "Could not read AL state for slave %u — continuing anyway",
                    slave_idx);
            }
        }

        auto& slave = master.ethercatMaster().slave(slave_idx);
        // configureMailbox(mbox_out, mbox_in, protocols):
        //   mbox_in  → SM0 (mailbox_write, master→slave)
        //   mbox_out → SM1 (mailbox_read,  slave→master)
        const auto mb_err = slave.configureMailbox(
            {.address = kMailboxReadAddr,  .length = kMailboxReadSize},  // mbox_out → SM1
            {.address = kMailboxWriteAddr, .length = kMailboxWriteSize}, // mbox_in  → SM0
            kMailboxProtocols);
        if (mb_err != EtherCAT::SlaveError::Ok) {
            TETHER_LOGE(TAG, "Failed to configure mailbox: %s",
                        EtherCAT::slaveErrorToString(mb_err));
            Tether::Examples::stopHostMasterSession(master, session);
            return 2;
        }
        TETHER_LOGI(TAG,
            "Mailbox configured from SOMANET ESI: "
            "SM0(M->S)=0x%04X/%u SM1(S->M)=0x%04X/%u proto=0x%04X",
            kMailboxWriteAddr, kMailboxWriteSize,
            kMailboxReadAddr, kMailboxReadSize, kMailboxProtocols);

        // Transition to PRE_OP before any SDO exchange.  Mailbox communication
        // (CoE/SDO) is only valid in PRE_OP or higher — the slave's PDI does
        // not service the mailbox in INIT, leaving SM0 full and SM1 empty.
        const auto pre_err = slave.transitionToPreOp();
        if (pre_err != EtherCAT::SlaveError::Ok) {
            TETHER_LOGE(TAG, "Failed to transition to PRE_OP: %s",
                        EtherCAT::slaveErrorToString(pre_err));
            Tether::Examples::stopHostMasterSession(master, session);
            return 2;
        }
        TETHER_LOGI(TAG, "Slave %u transitioned to PRE_OP", slave_idx);
    }

    // --- Dump default PDO/SM/FMMU configuration ---
    // Read the drive's power-on defaults before we overwrite anything.
    // This shows what the drive expects for FSoE + motion PDO layout.
    {
        auto& sdo = master.ethercatMaster().sdoManager(slave_idx);
        dumpDefaultPdoConfig(master.ethercatMaster(), slave_idx, sdo);
    }

    // --- Read ETG.5000/5001 modular device profile objects (0xFxxx) ---
    // These objects describe the modular framework state and FSoE configuration.
    // Must be read in PRE_OP (mailbox SDOs don't work in OP).
    {
        auto& sdo = master.ethercatMaster().sdoManager(slave_idx);
        TETHER_LOGI(TAG, "===== ETG.5000/5001 objects (0xFxxx) =====");

        auto read_u8 = [&](uint16_t idx, uint8_t sub, const char* name) -> void {
            auto res = sdo.readU8(idx, sub, {.timeout_ms = kSdoTimeoutMs});
            if (res.has_value())
                TETHER_LOGI(TAG, "  0x%04X:%u (%s) = 0x%02X", idx, sub, name, res.value());
            else
                TETHER_LOGW(TAG, "  0x%04X:%u (%s) FAILED", idx, sub, name);
        };
        auto read_u16 = [&](uint16_t idx, uint8_t sub, const char* name) -> void {
            auto res = sdo.readU16(idx, sub, {.timeout_ms = kSdoTimeoutMs});
            if (res.has_value())
                TETHER_LOGI(TAG, "  0x%04X:%u (%s) = 0x%04X", idx, sub, name, res.value());
            else
                TETHER_LOGW(TAG, "  0x%04X:%u (%s) FAILED", idx, sub, name);
        };
        auto read_u32 = [&](uint16_t idx, uint8_t sub, const char* name) -> void {
            auto res = sdo.readU32(idx, sub, {.timeout_ms = kSdoTimeoutMs});
            if (res.has_value())
                TETHER_LOGI(TAG, "  0x%04X:%u (%s) = 0x%08X", idx, sub, name, res.value());
            else
                TETHER_LOGW(TAG, "  0x%04X:%u (%s) FAILED", idx, sub, name);
        };

        // ETG.5000 Modular Device Profile objects
        TETHER_LOGI(TAG, "  -- Module Ident Lists --");
        read_u8 (0xF002, 0, "Module Ident List count");
        read_u32(0xF002, 1, "Module Ident[1]");
        read_u32(0xF002, 2, "Module Ident[2]");

        TETHER_LOGI(TAG, "  -- Configured Module Ident (0xF030) --");
        read_u8 (0xF030, 0, "0xF030 count");
        read_u32(0xF030, 1, "0xF030:1 (Module ident pos 1)");
        read_u32(0xF030, 2, "0xF030:2 (0x22D20001=no-param, 0x22D20002=with-param)");

        TETHER_LOGI(TAG, "  -- Detected Module Ident (0xF050) --");
        read_u8 (0xF050, 0, "0xF050 count");
        read_u32(0xF050, 1, "0xF050:1 (Detected Module ident pos 1)");
        read_u32(0xF050, 2, "0xF050:2 (Detected Module ident pos 2)");

        // ETG.5001 FSoE objects
        TETHER_LOGI(TAG, "  -- Device Safety Address (0xF980) --");
        read_u8 (0xF980, 0, "0xF980 count");
        read_u16(0xF980, 1, "Device Safety Address");

        // Additional 0xFxxx objects that may exist
        TETHER_LOGI(TAG, "  -- Other 0xFxxx objects --");
        read_u8 (0xF010, 0, "0xF010 count (Modular Device Profile)");
        read_u16(0xF010, 1, "0xF010:1 (Profile type)");
        read_u16(0xF010, 2, "0xF010:2 (Profile instance)");

        TETHER_LOGI(TAG, "===== End ETG.5000/5001 objects =====");
    }

    // --- Pre-activation safety check (disabled) ---
    // The Synapticon drive does not expose 0x2611 / 0x2620:2 via SDO on this
    // firmware, causing "Object does not exist" aborts.  The FSoE protocol
    // handles safety state via PDOs, so this SDO-based pre-check is not
    // required.  Re-enable by uncommenting the call below.
    // {
    //     const int safety_rc = preActivationSafetyCheck(
    //         master, session, slave_idx, args.enable_fsoe);
    //     if (safety_rc != 0) return safety_rc;
    // }

    // --- Read FSoE safety address (0xF980:1) for verification ---
    // Object 0xF980:1 contains the safety address configured on the drive.
    // For Synapticon SOMANET drives, the FSoE connection ID must match this
    // value (default 0x0006).  Reading it before starting the FSoE handshake
    // lets us warn the operator if --connection-id doesn't match the drive's
    // configured safety address, which would cause ConnectionIDError failures.
    //
    // When FSoE is disabled (--no-fsoe), this read is skipped.
    if (args.enable_fsoe) {
        auto& slave = master.ethercatMaster().slave(slave_idx);
        uint16_t drive_safety_address = 0;
        const auto addr_err =
            EtherCAT::Drives::Synapticon::readFSoESafetyAddress(
                slave, drive_safety_address);
        if (addr_err == EtherCAT::SlaveError::Ok) {
            TETHER_LOGI(TAG,
                "FSoE safety address (0xF980:1): 0x%04X — "
                "--connection-id=0x%04X --safety-address=0x%04X",
                drive_safety_address,
                args.connection_id,
                args.safety_address);
            if (drive_safety_address != args.connection_id) {
                TETHER_LOGW(TAG,
                    "WARNING: --connection-id (0x%04X) does NOT match the "
                    "drive's Device Safety Address 0xF980:1 (0x%04X).  "
                    "This will likely cause FSoE ConnectionIDError.  "
                    "Use --connection-id 0x%04X to match the drive.",
                    args.connection_id, drive_safety_address,
                    drive_safety_address);
            }
        } else {
            TETHER_LOGW(TAG,
                "Failed to read FSoE safety address (0xF980:1) via SDO "
                "(err=%u) — falling back to --connection-id=0x%04X",
                static_cast<unsigned>(addr_err),
                args.connection_id);
        }
    }

    // --- Configure drive: CST mode + FSoE, combined PDO mapping ---
    //
    // When FSoE is enabled, both the CiA 402 motion PDOs (0x1600/0x1A00) and
    // the FSoE safety PDOs (0x1700/0x1B00) must be mapped simultaneously.
    // This requires the multi-PDO-per-sync-manager API
    // (CiA402Drive::transitionToOp(const Slave::MultiPDOAssignment&)).
    //
    // PDO buffer layout (combined):
    //   SM2 (Rx): [0x1600 (12B)][0x1700 (11B)] = 23 bytes
    //   SM3 (Tx): [0x1A00 (12B)][0x1B00 (31B)] = 43 bytes
    //
    // When FSoE is disabled (--no-fsoe), only the motion PDOs are mapped
    // using the standard single-PDO configuration path.
    int rc = 0;

    // Discover slaves and initialize distributed clocks (common to both paths)
    if (!master.ethercatMaster().discoverSlaves()) {
        TETHER_LOGW(TAG, "No slaves discovered");
    }
    const uint16_t minimum_drive_count = static_cast<uint16_t>(slave_idx + 1);
    if (!master.waitForDriveCount(minimum_drive_count, 2000)) {
        TETHER_LOGE(TAG, "Timed out waiting for %u drive(s)", minimum_drive_count);
        Tether::Examples::stopHostMasterSession(master, session);
        return 3;
    }

    {
        EtherCAT::DC::DCConfig dc_config = EtherCAT::DC::DCConfig::defaults();
        if (!master.initializeDistributedClocks(dc_config)) {
            TETHER_LOGE(TAG, "Failed to initialize distributed clocks");
            Tether::Examples::stopHostMasterSession(master, session);
            return 3;
        }
        if (!master.startDistributedClocks()) {
            TETHER_LOGE(TAG, "Failed to start distributed clocks");
            Tether::Examples::stopHostMasterSession(master, session);
            return 3;
        }
    }

    // Get-or-create the drive and configure it for OP transition.
    //
    // ensureDrive() is used instead of driveBySlaveIndex() because the
    // multi-PDO FSoE path below bypasses configureDrive() (which would
    // otherwise create the drive).  Without this, driveBySlaveIndex()
    // would return nullptr — no CiA402Drive object exists yet and the
    // slave role defaults to NonDS402.
    auto& drive = master.ensureDrive(slave_idx);
    drive.setSDOTimeout(kSdoTimeoutMs);

    // Operating mode (CST) is set via the PDO buffer in the cyclic callback
    // (SineTorqueController::update writes rx->modes_of_operation).  The
    // Synapticon drive does not expose 0x6060 via SDO, so we skip the SDO
    // write entirely and rely on PDO-based mode setting.
    //
    // The PDO offset for modes_of_operation is configured below, after PDO
    // buffer sizes are known (FSoE vs. non-FSoE paths differ).

    if (args.enable_fsoe) {
        // Combined FSoE + motion PDO mapping via multi-PDO assignment.
        //
        // Uses the ESI layout with ALL PDOs, FSoE first:
        //   SM2: 0x1700 + 0x1600 + 0x1601 + 0x1602 = 46 bytes
        //   SM3: 0x1B00 + 0x1A00 + 0x1A01 + 0x1A02 + 0x1A03 = 78 bytes
        //
        // ALL PDOs (including FSoE) are written explicitly to 0x1C12/0x1C13.
        // FSoE PDOs come FIRST (critical for the Synapticon ESC bug — see
        // comment above).
        const auto assignment =
            EtherCAT::Drives::Synapticon_pdo::makeCombinedPDOAssignment();

        TETHER_LOGI(TAG,
            "Transitioning to OP with combined FSoE+motion PDO mapping: "
            "SM2=%u bytes (FSoE %uB + motion %uB), "
            "SM3=%u bytes (FSoE %uB + motion %uB)",
            static_cast<uint16_t>(kSM2TotalLen),
            static_cast<uint16_t>(sizeof(FSoERxPDO)),
            EtherCAT::Drives::Synapticon_pdo::kSM2TotalSize,
            static_cast<uint16_t>(kSM3TotalLen),
            static_cast<uint16_t>(sizeof(FSoETxPDO)),
            EtherCAT::Drives::Synapticon_pdo::kSM3TotalSize);

        // Note: Safety parameters (0x2620, 0x2641, etc.) are configured on the
        // drive via OBLAC Drives and cannot be written via SDO (error 0x08000021
        // "Local control error").  The FSoE module reads them from the drive's
        // safety parameter store.  The drive is already configured with:
        //   0x2620:2 (Safe fieldbus) = 0xFF (FSoE active)
        //   0x2620:3 (Safe address)  = 0x0006 (connection ID)
        //   0xF980:1 (Device Safety Address) = 0x0006

        if (!drive.transitionToOp(assignment)) {
            TETHER_LOGE(TAG, "Failed to transition to OP with combined PDO assignment");
            master.stopDistributedClocks();
            Tether::Examples::stopHostMasterSession(master, session);
            return 3;
        }
        TETHER_LOGI(TAG, "Slave %u transitioned to OP with combined CST+FSoE PDOs", slave_idx);

        // --- Comprehensive safety diagnostics ---
        // Query all safety-related objects via SDO before starting the PDO
        // exchange loop.  This catches configuration issues (FSoE not active,
        // parameter validation missing, connection ID mismatch) and reports
        // any active safety faults with human-readable descriptions.
        {
            auto& slave_ref = master.ethercatMaster().slave(slave_idx);
            auto diag_report = EtherCAT::Drives::Synapticon::runFullSafetyDiagnostics(
                slave_ref);

            // Log actionable warnings based on the diagnostic report
            if (diag_report.hasFault()) {
                TETHER_LOGE(TAG,
                    "SAFETY FAULT detected before starting PDO loop: '%s' — "
                    "FSoE communication may fail.  Check OBLAC Drives parameter "
                    "validation and safety configuration.",
                    diag_report.error_report);
            }
            if (diag_report.general_safety_ok && !diag_report.fsoeActive()) {
                TETHER_LOGE(TAG,
                    "FSoE is NOT active on the drive (0x2620:2 = 0).  "
                    "The safety module will not process FSoE PDO data.  "
                    "Enable FSoE in OBLAC Drives configuration.");
            }
            if (diag_report.module_ident_ok &&
                diag_report.configured_ident_pos2 == EtherCAT::Drives::Synapticon::kModuleIdentNoParam) {
                TETHER_LOGW(TAG,
                    "Module ident = 0x22D20001 (no parameter changes via master).  "
                    "Safety parameters must be validated in OBLAC Drives.  "
                    "If not validated, the safety module will report 'SmmFIO25' "
                    "(black channel fault) and refuse to communicate via FSoE.");
            }
            if (diag_report.connIdMismatch()) {
                TETHER_LOGE(TAG,
                    "Connection ID mismatch: 0xF980 (0x%04X) != 0x2620:3 (0x%04X).  "
                    "The master --connection-id must match the device safety address.",
                    diag_report.device_safety_address,
                    diag_report.safe_address);
            }
        }

        // --- PDO write / SDO readback diagnostic ---
        // Write a known pattern to the FSoE RxPDO region (0x1700), let the
        // cyclic task send it for a few cycles, then read back the individual
        // mapped objects via SDO to verify the data lands at the correct
        // offsets in the slave's object dictionary.
        TETHER_LOGI(TAG, "=== PDO write / SDO readback diagnostic ===");

        // Enable PDO exchange so the data actually gets sent
        master.ethercatMaster().pdo().resetStats();
        master.ethercatMaster().dc().setPDOEnabled(true);

        // Read the slave's configured station address (register 0x0010)
        // and set it in the PDOManager so exchangePhysical() uses the
        // correct FPWR/FPRD address.  Without this, exchangePhysical()
        // falls back to adp=0x0000 (physical position) which fails with
        // WKC=0 because the slave's configured address is different.
        {
            uint16_t cfg_addr = 0;
            if (master.ethercatMaster().readRegister(
                    EtherCAT::Master::slaveAddressFromADP(
                        EtherCAT::Master::adpForSlaveIndex(slave_idx)),
                    0x0010, cfg_addr, 200)) {
                TETHER_LOGI(TAG, "Slave %u configured station address (reg 0x0010) = 0x%04X",
                    slave_idx, cfg_addr);
                // Set the configured address in the PDO mapping so
                // exchangePhysical() uses FPWR/FPRD with the correct adp.
                master.ethercatMaster().pdo().mapping().set_slave_configured_address(
                    slave_idx, cfg_addr);
                // Also set it in the SlaveConfig so exchangePhysical()'s
                // cfg->configured_address check uses the right value.
                auto* sc = master.ethercatMaster().pdo().slaveConfigs();
                if (sc && slave_idx < EtherCAT::PDO::kMaxPDOSlaves) {
                    sc[slave_idx].configured_address = cfg_addr;
                }
            } else {
                TETHER_LOGW(TAG, "Failed to read configured station address (reg 0x0010)");
            }
        }

        // First, verify that SDO readback of PDO-mapped objects works at all.
        // Write a known value to the Controlword (0x6040) via PDO, then read
        // it back via SDO.  If this doesn't work, SDO readback of PDO-mapped
        // objects isn't supported on this slave.
        {
            uint8_t* rx_buf = static_cast<uint8_t*>(drive.getRxPDOBuffer());
            // Controlword is at offset kMotionRxPDOOffset in the combined PDO
            // (FSoE PDO comes first, motion PDO second)
            rx_buf[kMotionRxPDOOffset + 0] = 0x0F;  // Controlword low byte
            rx_buf[kMotionRxPDOOffset + 1] = 0x00;  // Controlword high byte
            TETHER_LOGI(TAG, "Wrote Controlword=0x000F to RxPDO offset %zu",
                kMotionRxPDOOffset);
        }

        // Wait for the cyclic task to send the PDO data a few times
        Tether::Platform::Clock::instance().delayMilliseconds(100);

        // Read back Controlword via SDO
        auto& sdo = master.ethercatMaster().sdoManager(slave_idx);
        {
            auto cw = sdo.readU16(0x6040, 0, {.timeout_ms = kSdoTimeoutMs});
            if (cw.has_value())
                TETHER_LOGI(TAG, "  SDO read 0x6040:0 (Controlword) = 0x%04X (expect 0x000F if PDO readback works)",
                    cw.value());
            else
                TETHER_LOGW(TAG, "  SDO read 0x6040:0 (Controlword) FAILED");
        }

        // Now write a known FSoE Reset frame to the FSoE PDO region:
        //   byte 0:     FSoE Command = 0x2A (Reset)
        //   bytes 1-2:  safety_flags = 0x0000
        //   bytes 3-4:  CRC_0 = 0x0000 (CRC of {0,0} = 0x0000)
        //   bytes 5-6:  reserved + safe_outputs = 0x0000
        //   bytes 7-8:  CRC_1 = 0x0000
        //   bytes 9-10: ConnectionID = 0x0006
        {
            uint8_t* rx_buf = static_cast<uint8_t*>(drive.getRxPDOBuffer());
            // FSoE PDO is at offset kFSoERxPDOOffset (FSoE comes first, motion second)
            uint8_t* fsoe_buf = rx_buf + kFSoERxPDOOffset;

            // Fill FSoE region with known pattern
            fsoe_buf[0] = 0x2A;  // FSoE Command (Reset)
            fsoe_buf[1] = 0x00;  // safety_flags low
            fsoe_buf[2] = 0x00;  // safety_flags high
            fsoe_buf[3] = 0x00;  // CRC_0 low
            fsoe_buf[4] = 0x00;  // CRC_0 high
            fsoe_buf[5] = 0x00;  // reserved
            fsoe_buf[6] = 0x00;  // safe_outputs
            fsoe_buf[7] = 0x00;  // CRC_1 low
            fsoe_buf[8] = 0x00;  // CRC_1 high
            fsoe_buf[9] = 0x06;  // ConnectionID low
            fsoe_buf[10] = 0x00; // ConnectionID high

            TETHER_LOGI(TAG, "Wrote FSoE pattern to RxPDO offset %zu: "
                "2A 00 00 00 00 00 00 00 00 06 00", kFSoERxPDOOffset);
        }

        // Wait for the cyclic task to send the PDO data a few times
        Tether::Platform::Clock::instance().delayMilliseconds(200);

        // Read back the FSoE objects via SDO
        auto read_u8 = [&](uint16_t idx, uint8_t sub, const char* name) -> void {
            auto res = sdo.readU8(idx, sub, {.timeout_ms = kSdoTimeoutMs});
            if (res.has_value())
                TETHER_LOGI(TAG, "  SDO read 0x%04X:%u (%s) = 0x%02X", idx, sub, name, res.value());
            else
                TETHER_LOGW(TAG, "  SDO read 0x%04X:%u (%s) FAILED", idx, sub, name);
        };
        auto read_u16 = [&](uint16_t idx, uint8_t sub, const char* name) -> void {
            auto res = sdo.readU16(idx, sub, {.timeout_ms = kSdoTimeoutMs});
            if (res.has_value())
                TETHER_LOGI(TAG, "  SDO read 0x%04X:%u (%s) = 0x%04X", idx, sub, name, res.value());
            else
                TETHER_LOGW(TAG, "  SDO read 0x%04X:%u (%s) FAILED", idx, sub, name);
        };
        auto read_u32 = [&](uint16_t idx, uint8_t sub, const char* name) -> void {
            auto res = sdo.readU32(idx, sub, {.timeout_ms = kSdoTimeoutMs});
            if (res.has_value())
                TETHER_LOGI(TAG, "  SDO read 0x%04X:%u (%s) = 0x%08X", idx, sub, name, res.value());
            else
                TETHER_LOGW(TAG, "  SDO read 0x%04X:%u (%s) FAILED", idx, sub, name);
        };

        TETHER_LOGI(TAG, "Reading back FSoE objects via SDO:");
        read_u8 (0x6770, 1, "FSoE Command (expect 0x2A)");
        read_u16(0x6770, 2, "FSoE ConnectionID (expect 0x0006)");
        read_u16(0x6770, 3, "FSoE CRC_0 (expect 0x0000)");
        read_u16(0x6770, 4, "FSoE CRC_1 (expect 0x0000)");
        read_u16(0x6640, 0, "STO (expect 0x0000)");
        read_u16(0x26F0, 1, "Safe output 1 (expect 0x0000)");

        // Also read back the TxPDO FSoE objects to see what the slave sends
        TETHER_LOGI(TAG, "Reading slave TxPDO FSoE objects via SDO:");
        read_u8 (0x6760, 1, "FSoE Command (slave->master)");
        read_u16(0x6760, 2, "FSoE ConnectionID (slave->master)");

        // Check PDO mapping objects — these tell us if the slave has
        // actually configured the FSoE PDO entries
        TETHER_LOGI(TAG, "Checking PDO mapping objects:");
        read_u8 (0x1700, 0, "RxPDO 0x1700 mapping count (expect 18)");
        read_u8 (0x1B00, 0, "TxPDO 0x1B00 mapping count (expect 18)");

        // Full PDO mapping readout and module identification are not needed
        // for normal operation — commented out to reduce SDO traffic and
        // log noise.  Uncomment for low-level PDO layout debugging.
#if 0
        // Read the full PDO mapping to understand the actual byte layout
        TETHER_LOGI(TAG, "Reading full RxPDO 0x1700 mapping:");
        {
            auto cnt = sdo.readU8(0x1700, 0, {.timeout_ms = kSdoTimeoutMs});
            if (cnt.has_value()) {
                for (uint8_t i = 1; i <= cnt.value() && i <= 30; i++) {
                    auto entry = sdo.readU32(0x1700, i, {.timeout_ms = kSdoTimeoutMs});
                    if (entry.has_value()) {
                        uint32_t v = entry.value();
                        uint16_t idx = (v >> 16) & 0xFFFF;
                        uint8_t sub = (v >> 8) & 0xFF;
                        uint8_t bits = v & 0xFF;
                        TETHER_LOGI(TAG, "  0x1700:%u = 0x%08X (idx=0x%04X sub=%u bits=%u)",
                            i, v, idx, sub, bits);
                    } else {
                        TETHER_LOGW(TAG, "  0x1700:%u FAILED", i);
                    }
                }
            }
        }
        TETHER_LOGI(TAG, "Reading full TxPDO 0x1B00 mapping:");
        {
            auto cnt = sdo.readU8(0x1B00, 0, {.timeout_ms = kSdoTimeoutMs});
            if (cnt.has_value()) {
                for (uint8_t i = 1; i <= cnt.value() && i <= 50; i++) {
                    auto entry = sdo.readU32(0x1B00, i, {.timeout_ms = kSdoTimeoutMs});
                    if (entry.has_value()) {
                        uint32_t v = entry.value();
                        uint16_t idx = (v >> 16) & 0xFFFF;
                        uint8_t sub = (v >> 8) & 0xFF;
                        uint8_t bits = v & 0xFF;
                        TETHER_LOGI(TAG, "  0x1B00:%u = 0x%08X (idx=0x%04X sub=%u bits=%u)",
                            i, v, idx, sub, bits);
                    } else {
                        TETHER_LOGW(TAG, "  0x1B00:%u FAILED", i);
                    }
                }
            }
        }

        // Check module identification (ETG.5000 modular device profile)
        TETHER_LOGI(TAG, "Checking module identification:");
        read_u16(0xF002, 0, "ModuleIdentList count");
        if (true) {
            auto cnt = sdo.readU16(0xF002, 0, {.timeout_ms = kSdoTimeoutMs});
            if (cnt.has_value()) {
                for (uint16_t i = 1; i <= cnt.value() && i <= 4; i++) {
                    char label[64];
                    snprintf(label, sizeof(label), "ModuleIdent[%u]", i);
                    read_u16(0xF002, i, label);
                }
            }
        }
#endif

        // Check 0x2620 (General safety object) subindices
        TETHER_LOGI(TAG, "Checking safety general object (0x2620):");
        read_u8 (0x2620, 0, "0x2620 count");
        read_u8 (0x2620, 1, "0x2620:1 (unknown)");
        read_u8 (0x2620, 2, "Safe fieldbus (expect 255=active)");
        read_u16(0x2620, 3, "Safe address");
        read_u8 (0x2620, 4, "0x2620:4 (unknown)");

        // Check 0xF980 (Device Safety Address)
        TETHER_LOGI(TAG, "Checking Device Safety Address (0xF980):");
        read_u8 (0xF980, 0, "0xF980 count");
        read_u16(0xF980, 1, "Device Safety Address");

        // Check 0xF030 (Configured module ident list) and 0xF050 (Detected Module Ident List)
        // 0xF030:2 = 0x22D20001 → No parameter changes via master
        // 0xF030:2 = 0x22D20002 → With parameter changes via master
        TETHER_LOGI(TAG, "Checking Module ID configuration (0xF030/0xF050):");
        read_u8 (0xF030, 0, "0xF030 count");
        read_u32(0xF030, 1, "0xF030:1 (Module ident pos 1)");
        read_u32(0xF030, 2, "0xF030:2 (Module ident pos 2 — 0x22D20001=no-param, 0x22D20002=with-param)");
        read_u8 (0xF050, 0, "0xF050 count");
        read_u32(0xF050, 1, "0xF050:1 (Detected Module ident pos 1)");
        read_u32(0xF050, 2, "0xF050:2 (Detected Module ident pos 2)");

        // Check 0x2610 (Manufacturing parameters)
        TETHER_LOGI(TAG, "Checking manufacturing parameters (0x2610):");
        read_u8 (0x2610, 0, "0x2610 count");

        // Check 0x2621 (Safety digital IO)
        TETHER_LOGI(TAG, "Checking safety digital IO (0x2621):");
        read_u8 (0x2621, 0, "0x2621 count");

        // Read Safety statusword (0x6621) — this tells us the FSoE module's
        // internal state and might explain why it's not responding
        TETHER_LOGI(TAG, "Reading safety statusword (0x6621):");
        read_u8(0x6621, 0, "Safety statusword count");
        read_u8(0x6621, 1, "Safety status byte 1 (STO)");
        read_u8(0x6621, 2, "Safety status byte 2 (SBC)");

        // Read error report (0x203F) — may contain safety-related errors
        TETHER_LOGI(TAG, "Reading error report (0x203F):");
        read_u8(0x203F, 0, "Error report count");
        {
            uint8_t err_buf[16] = {};
            size_t err_len = 0;
            if (sdo.readSync(0x203F, 1, err_buf, sizeof(err_buf), kSdoTimeoutMs, &err_len)) {
                char err_str[17];
                size_t copy_len = err_len < 8 ? err_len : 8;
                for (size_t i = 0; i < copy_len; i++) {
                    err_str[i] = (err_buf[i] >= 0x20 && err_buf[i] < 0x7F) ? static_cast<char>(err_buf[i]) : '.';
                }
                err_str[copy_len] = '\0';
                TETHER_LOGI(TAG, "  SDO read 0x203F:1 (Error report) = '%s' (len=%zu, hex: %02X %02X %02X %02X %02X %02X %02X %02X)",
                    err_str, err_len,
                    err_buf[0], err_buf[1], err_buf[2], err_buf[3],
                    err_buf[4], err_buf[5], err_buf[6], err_buf[7]);
            } else {
                TETHER_LOGW(TAG, "  SDO read 0x203F:1 (Error report) FAILED");
            }
        }

        TETHER_LOGI(TAG, "=== End PDO/SDO diagnostic ===");

        // Note: PDO exchange stays enabled — the FSoE cyclic task needs it
    } else {
        // FSoE disabled — motion-only single-PDO configuration
        Tether::Examples::SingleDriveExampleConfig config;
        config.drive.slave_index = slave_idx;
        config.drive.rxpdo_index = EtherCAT::Drives::Synapticon_pdo::RxPDO_1600.index;
        config.drive.txpdo_index = EtherCAT::Drives::Synapticon_pdo::TxPDO_1A00.index;
        config.drive.rxpdo_size = EtherCAT::Drives::Synapticon_pdo::RxPDO_1600.size;
        config.drive.txpdo_size = EtherCAT::Drives::Synapticon_pdo::TxPDO_1A00.size;
        // Operating mode is set via PDO, not SDO — skip configureDrive's SDO write
        config.drive.operating_mode = 0;
        config.drive.sdo_timeout_ms = kSdoTimeoutMs;
        config.drive.auto_configure_mailbox = false;
        config.drive.transition_to_operational = true;

        if (!master.configureDrive(config.drive)) {
            TETHER_LOGE(TAG, "Failed to configure slave %u", slave_idx);
            master.stopDistributedClocks();
            Tether::Examples::stopHostMasterSession(master, session);
            return 3;
        }
    }

    // --- Configure PDO-based operating mode offset ---
    // The modes_of_operation (0x6060) field is in the motion RxPDO (0x1600)
    // at offset 2 (after uint16_t controlword).  The motion PDO is at
    // kMotionRxPDOOffset in the combined buffer (after FSoE PDO).
    {
        const size_t motion_rx_offset =
            args.enable_fsoe ? kMotionRxPDOOffset : 0;
        const size_t opmode_offset = motion_rx_offset +
            offsetof(EtherCAT::Drives::Synapticon_pdo::SOMANET_RxPDO_1600,
                    modes_of_operation);
        drive.setOpmodePDOOffset(static_cast<int>(opmode_offset));
        TETHER_LOGI(TAG,
            "Operating mode PDO offset: %zu (motion_rx=%zu, opmode=%zu)",
            opmode_offset, motion_rx_offset, opmode_offset);
    }

    // --- Read motor rated torque (0x6076) for Nm→per-mille conversion ---
    // CiA 402 target_torque (0x6071) is in per-mille (0.1%) of rated torque.
    // Object 0x6076 (Motor Rated Torque) is in mNm.  We read it here so the
    // sine torque controller can convert Nm commands to per-mille units.
    // If --rated-torque-mnm was given non-zero, use that instead of SDO read.
    uint32_t rated_torque_mnm = args.rated_torque_mnm;
    if (rated_torque_mnm == 0) {
        auto& sdo_mgr = master.ethercatMaster().sdoManager(slave_idx);
        auto rated = sdo_mgr.readU32(0x6076, 0, {.timeout_ms = kSdoTimeoutMs});
        if (rated.has_value() && rated.value() > 0) {
            rated_torque_mnm = rated.value();
            TETHER_LOGI(TAG,
                "Motor rated torque (0x6076) = %u mNm (%.3f Nm)",
                rated_torque_mnm, rated_torque_mnm / 1000.0);
        } else {
            TETHER_LOGE(TAG,
                "Failed to read motor rated torque (0x6076) via SDO — "
                "cannot convert Nm to per-mille.  Use --rated-torque-mnm to "
                "specify it manually.");
            master.stopDistributedClocks();
            Tether::Examples::stopHostMasterSession(master, session);
            return 3;
        }
    } else {
        TETHER_LOGI(TAG,
            "Using --rated-torque-mnm = %u mNm (%.3f Nm)",
            rated_torque_mnm, rated_torque_mnm / 1000.0);
    }

    // --- Add sine torque motion controller ---
    // Amplitude is half the peak-to-peak value (±torque_pp_nm/2).
    const double amplitude_nm = args.torque_pp_nm / 2.0;
    TETHER_LOGI(TAG,
        "Sine torque: %.3f Nm P-P (%.3f Nm amplitude) at %.3f Hz, "
        "rated torque %u mNm -> %.1f per-mille peak",
        args.torque_pp_nm, amplitude_nm, args.freq_hz,
        rated_torque_mnm,
        amplitude_nm * 1'000'000.0 / static_cast<double>(rated_torque_mnm));

    if (!master.addMotionController(
            slave_idx,
            std::make_unique<SineTorqueController<RxPDO>>(
                amplitude_nm, args.freq_hz, rated_torque_mnm))) {
        TETHER_LOGE(TAG, "Failed to add sine torque controller");
        rc = 4;
        Tether::Examples::shutdownSingleDrive(master, slave_idx);
        Tether::Examples::stopHostMasterSession(master, session);
        return rc;
    }

    // --- Set up FSoE safe-motion (real drive via PDOs) ---
    std::unique_ptr<FSoEMain> fsoe_main;

    // --- Thread synchronization for FSoE → CiA 402 enable ---
    //
    // The FSoE state machine runs in the realtime motion loop thread, while
    // the CiA 402 enable sequence is driven from this (main) thread.  We use
    // a std::promise<bool> as a one-shot signaling mechanism:
    //   true  → FSoE reached Data state (safe to enable the drive)
    //   false → FSoE entered Error state (abort enable)
    //
    // The promise is fulfilled from the state change callback (realtime
    // thread).  The main thread blocks on the shared_future with a timeout.
    // An atomic<bool> guard ensures set_value() is called exactly once,
    // even if the state machine transitions through multiple terminal states.
    auto fsoe_ready_promise = std::make_unique<std::promise<bool>>();
    std::shared_future<bool> fsoe_ready_future = fsoe_ready_promise->get_future();
    std::atomic<bool> fsoe_signaled{false};

    auto signal_fsoe = [&fsoe_ready_promise, &fsoe_signaled](bool success) {
        if (!fsoe_signaled.exchange(true)) {
            fsoe_ready_promise->set_value(success);
        }
    };

    if (args.enable_fsoe) {
        // FSoE debug flags are now part of the Tether debug framework and
        // support per-slave filtering.  They are applied via applyDebugFlags()
        // above (same call as the EtherCAT framework flags).  Query them here
        // for the target slave.
        const auto& dbg = master.ethercatMaster().debugFlags();
        const bool debug_fsoe         = dbg.isEnabled("fsoe",         slave_idx);
        const bool debug_fsoe_frame   = dbg.isEnabled("fsoe-frame",   slave_idx);
        const bool debug_fsoe_raw     = dbg.isEnabled("fsoe-raw",     slave_idx);
        const bool debug_fsoe_wire    = dbg.isEnabled("fsoe-wire",    slave_idx);
        const bool debug_fsoe_sequence= dbg.isEnabled("fsoe-sequence",slave_idx);
        const bool debug_fsoe_crc     = dbg.isEnabled("fsoe-crc",     slave_idx);

        EtherCAT::Drives::Synapticon::SafeMotion::MainConfig main_config;
        main_config.feature_enabled = true;
        main_config.slave_address = slave_idx;
        main_config.safety_address = args.safety_address;  // FSoE slave safety address
        main_config.connection_id = args.connection_id;    // FSoE connection ID
        main_config.master_address = 0x0001;
        main_config.watchdog_time_ms = args.watchdog_ms;

        fsoe_main = std::make_unique<FSoEMain>(main_config);

        if (!fsoe_main->initialize()) {
            TETHER_LOGE(TAG, "FSoE initialization failed");
            rc = 5;
            master.clearCyclicTasks();
            (void)master.removeMotionController(slave_idx);
            Tether::Examples::shutdownSingleDrive(master, slave_idx);
            Tether::Examples::stopHostMasterSession(master, session);
            return rc;
        }

        fsoe_main->requestMotionEnabled();

        // Apply STO/SBC overrides from command-line flags.
        // The flags control RAW bit values in the PDO (0=bit cleared, 1=bit set).
        // The codec uses setZeroActive for STO and SBC, which inverts:
        //   setZeroActive(true)  → bit=0
        //   setZeroActive(false) → bit=1
        // So to get raw bit=0, we set the Command field to true (active),
        // and to get raw bit=1, we set it to false (inactive).
        if (args.sto_override >= 0 || args.sbc_override >= 0) {
            auto cmd = fsoe_main->command();
            if (args.sto_override >= 0) {
                // setZeroActive: bit=0 when active=true, bit=1 when active=false
                cmd.sto = (args.sto_override == 0);  // 0→true(bit=0), 1→false(bit=1)
                TETHER_LOGI(TAG,
                    "[safety-command] Setting raw STO bit=%d (cmd.sto=%s → 0-active: %s)",
                    args.sto_override,
                    cmd.sto ? "true" : "false",
                    args.sto_override == 0 ? "STO active (torque off)" : "STO inactive (torque allowed)");
            }
            if (args.sbc_override >= 0) {
                // setZeroActive: bit=0 when active=true, bit=1 when active=false
                cmd.brake_engage = (args.sbc_override == 0);  // 0→true(bit=0), 1→false(bit=1)
                TETHER_LOGI(TAG,
                    "[safety-command] Setting raw SBC bit=%d (cmd.brake_engage=%s → 0-active: %s)",
                    args.sbc_override,
                    cmd.brake_engage ? "true" : "false",
                    args.sbc_override == 0 ? "SBC active (brake engaged)" : "SBC inactive (brake released)");
            }
            fsoe_main->setCommand(cmd);
        }

        // Install FSoE callbacks for real-time state tracking.
        // The state change and error callbacks also signal the main thread
        // via the promise/future pair declared above the FSoE block.
        fsoe_main->rawConnection().setStateChangeCallback(
            [&signal_fsoe](uint8_t old_s, uint8_t new_s) {
                TETHER_LOGI(TAG,
                    "[FSoE] state: %s -> %s",
                    fsoeStateName(old_s), fsoeStateName(new_s));
                if (new_s == FSoE::ConnectionState::Data) {
                    signal_fsoe(true);
                } else if (new_s == FSoE::ConnectionState::Error) {
                    signal_fsoe(false);
                }
            });
        fsoe_main->rawConnection().setErrorCallback(
            [&signal_fsoe](uint16_t code, const FSoE::FSoEErrorDetail& detail) {
                if (detail.message[0] != '\0') {
                    TETHER_LOGE(TAG,
                        "[FSoE] error: 0x%04X (%s): %s",
                        code, fsoeErrorName(code), detail.message);
                } else {
                    TETHER_LOGE(TAG,
                        "[FSoE] error: 0x%04X (%s)",
                        code, fsoeErrorName(code));
                }
                // Signal failure on critical errors (non-zero error code).
                // Non-critical errors (NoError = 0x0000) are just diagnostic
                // events and don't prevent the handshake from completing.
                if (code != FSoE::ErrorCode::NoError) {
                    signal_fsoe(false);
                }
            });
        fsoe_main->rawConnection().setFailSafeCallback(
            []() {
                TETHER_LOGW(TAG, "[FSoE] fail-safe activated");
            });

        // High-level protocol trace (--debug fsoe or --debug fsoe-raw).
        // The trace callback emits human-readable descriptions of every
        // protocol decision: what the master is sending and why, what it
        // received from the slave and how it interpreted it.
        if (debug_fsoe || debug_fsoe_raw) {
            fsoe_main->rawConnection().setTraceCallback(
                [](const char* message) {
                    TETHER_LOGI(TAG, "[fsoe] %s", message);
                });
        }

        // Per-cycle sequence trace (--debug fsoe-sequence).
        // Emits one line per exchangeViaPDO() call summarizing:
        //   cycle N: state_before -> state_after  cmd=0xXX  accepted/rejected  reason
        if (debug_fsoe_sequence) {
            fsoe_main->rawConnection().setSequenceTraceCallback(
                [](const FSoE::SequenceTraceInfo& info) {
                    const char* arrow = info.state_changed ? " -> " : " == ";
                    TETHER_LOGI(TAG,
                        "[fsoe-seq] cycle %u: %s%s%s  cmd=0x%02X  %s%s  reason=%s",
                        info.cycle,
                        fsoeStateName(info.state_before),
                        arrow,
                        fsoeStateName(info.state_after),
                        info.rx_cmd,
                        info.frame_accepted ? "ACCEPTED" : "REJECTED",
                        info.tx_rebuilt ? " (tx rebuilt)" : "",
                        info.reason);
                });
        }

        // CRC parameter trace (--debug fsoe-crc).
        // Emits the exact CRC inputs and outputs for every frame built (TX)
        // and checked (RX): start_crc, seq_expected, seq_used, CRC0, and
        // whether the seq±1 fallback was used.  This is the low-level
        // diagnostic view for debugging CRC/seq synchronization issues.
        if (debug_fsoe_crc) {
            fsoe_main->rawConnection().setCrcTraceCallback(
                [](const FSoE::CrcTraceInfo& info) {
                    // Format SafeData bytes as hex string.
                    char data_hex[64] = {};
                    size_t pos = 0;
                    for (size_t i = 0; i < info.data_len && pos < sizeof(data_hex) - 4; i++) {
                        pos += static_cast<size_t>(snprintf(
                            data_hex + pos, sizeof(data_hex) - pos, "%02X ", info.data[i]));
                    }
                    if (pos == 0) {
                        snprintf(data_hex, sizeof(data_hex), "(none)");
                    }

                    if (info.direction ==
                        FSoE::CrcTraceInfo::Direction::TX) {
                        // TX: master building a frame.
                        // CRC inputs (in byte processing order):
                        //   start_crc(Lo,Hi), conn_id(Lo,Hi),
                        //   seq(Lo,Hi), command, data[0..n-1]
                        TETHER_LOGI(TAG,
                            "[fsoe-crc] TX %s cmd=0x%02X: "
                            "start_crc=0x%04X seq=%u -> "
                            "CRC0=0x%04X | "
                            "CRC inputs: oldCRC=0x%04X conn_id=0x%04X "
                            "seq=%u cmd=0x%02X data[%zu]={%s}",
                            fsoeStateName(info.state),
                            info.command,
                            info.start_crc, info.seq_used,
                            info.crc0,
                            info.start_crc, info.conn_id,
                            info.seq_used, info.command,
                            info.data_len, data_hex);
                    } else {
                        // RX: master checking a frame.
                        // Compute the expected CRC0 from the master's
                        // parameters to show the mismatch directly.
                        uint16_t expected_crc0 = 0;
                        if (info.data_len > 0) {
                            uint8_t tmpbuf[32];
                            FSoE::CRC::buildFSoEFrame(
                                tmpbuf, info.command,
                                info.data, info.data_len,
                                info.conn_id,
                                info.start_crc,
                                info.crc_ok ? info.seq_used : info.seq_expected,
                                &expected_crc0);
                        }

                        if (info.crc_ok) {
                            const char* fb = "";
                            char fb_buf[64] = {};
                            if (info.fallback_used) {
                                snprintf(fb_buf, sizeof(fb_buf),
                                    " [FALLBACK seq%+d: expected=%u]",
                                    info.fallback_delta, info.seq_expected);
                                fb = fb_buf;
                            }
                            TETHER_LOGI(TAG,
                                "[fsoe-crc] RX %s cmd=0x%02X: "
                                "start_crc=0x%04X seq=%u -> "
                                "CRC0=0x%04X OK (expected=0x%04X) | "
                                "CRC inputs: oldCRC=0x%04X conn_id=0x%04X "
                                "seq=%u cmd=0x%02X data[%zu]={%s}%s",
                                fsoeStateName(info.state),
                                info.command,
                                info.start_crc, info.seq_used,
                                info.crc0, expected_crc0,
                                info.start_crc, info.conn_id,
                                info.seq_used, info.command,
                                info.data_len, data_hex, fb);
                        } else {
                            TETHER_LOGE(TAG,
                                "[fsoe-crc] RX %s cmd=0x%02X: "
                                "start_crc=0x%04X seq=%u -> "
                                "CRC FAIL: received=0x%04X expected=0x%04X | "
                                "CRC inputs: oldCRC=0x%04X conn_id=0x%04X "
                                "seq=%u cmd=0x%02X data[%zu]={%s}",
                                fsoeStateName(info.state),
                                info.command,
                                info.start_crc, info.seq_expected,
                                info.crc0, expected_crc0,
                                info.start_crc, info.conn_id,
                                info.seq_expected, info.command,
                                info.data_len, data_hex);
                        }
                    }
                });
        }

        // Raw frame hex dumps (--debug fsoe-raw).
        // Frame event listeners are invoked from inside the FSoE state
        // machine for every master-to-slave (tx) and slave-to-master (rx)
        // frame.  Each listener receives an immutable shared_ptr<const
        // std::vector<uint8_t>> copy of the frame bytes.
        // Only prints when the frame content changes from the previous cycle.
        if (debug_fsoe_raw) {
            auto last_tx = std::make_shared<std::vector<uint8_t>>();
            auto last_rx = std::make_shared<std::vector<uint8_t>>();
            fsoe_main->rawConnection().txFrameEvents().addListener(
                [last_tx](std::shared_ptr<const std::vector<uint8_t>> data) {
                    if (*data == *last_tx) return;  // skip unchanged
                    *last_tx = *data;
                    const uint8_t cmd = (!data->empty()) ? (*data)[0] : 0;
                    TETHER_LOGI(TAG, "[fsoe-raw] TX (master->slave) len=%zu cmd=%s",
                                data->size(), fsoeCommandName(cmd));
                    hexDump(TAG, "TX (master->slave)", data->data(), data->size());
                });
            fsoe_main->rawConnection().rxFrameEvents().addListener(
                [last_rx](std::shared_ptr<const std::vector<uint8_t>> data) {
                    if (*data == *last_rx) return;  // skip unchanged
                    *last_rx = *data;
                    const uint8_t cmd = (!data->empty()) ? (*data)[0] : 0;
                    TETHER_LOGI(TAG, "[fsoe-raw] RX (slave->master) len=%zu cmd=%s",
                                data->size(), fsoeCommandName(cmd));
                    hexDump(TAG, "RX (slave->master)", data->data(), data->size());
                });
        }

        // Add FSoE PDO exchange as a cyclic task.
        // This exchanges FSoE frames with the REAL drive via the FSoE safety
        // PDOs (0x1700/0x1B00) mapped in the combined PDO buffer.  The FSoE
        // PDOs are at a fixed offset after the motion PDO data.
        if (!master.addCyclicTask(
                std::make_unique<FSoEPDOExchangeTask>(
                    slave_idx, *fsoe_main,
                    kFSoERxPDOOffset, kFSoETxPDOOffset,
                    debug_fsoe_raw, debug_fsoe_frame, debug_fsoe_wire))) {
            TETHER_LOGE(TAG, "Failed to add FSoE PDO exchange task");
            rc = 6;
            master.clearCyclicTasks();
            (void)master.removeMotionController(slave_idx);
            Tether::Examples::shutdownSingleDrive(master, slave_idx);
            Tether::Examples::stopHostMasterSession(master, session);
            return rc;
        }

        // Add diagnostics task
        if (!master.addCyclicTask(
                std::make_unique<FSoEDiagnosticsTask>(
                    slave_idx, *fsoe_main, args.diag_interval_ms))) {
            TETHER_LOGW(TAG, "Failed to add FSoE diagnostics task (non-fatal)");
        }

        // Add safety state confirmation monitor (only if overrides are set)
        if (args.sto_override >= 0 || args.sbc_override >= 0) {
            const auto& cmd = fsoe_main->command();
            // The monitor expects the drive's status to match the Command's
            // semantic fields (sto_active should match cmd.sto, brake_engaged
            // should match cmd.brake_engage).
            if (!master.addCyclicTask(
                    std::make_unique<SafetyStateMonitor>(
                        *fsoe_main,
                        args.sto_override >= 0, args.sbc_override >= 0,
                        args.sto_override >= 0 ? cmd.sto : false,
                        args.sbc_override >= 0 ? cmd.brake_engage : false))) {
                TETHER_LOGW(TAG, "Failed to add safety state monitor (non-fatal)");
            }
        }

        TETHER_LOGI(TAG,
            "FSoE enabled: conn_id=0x%04X safety_addr=0x%04X watchdog=%u ms debug=%s%s%s%s%s%s",
            args.connection_id, args.safety_address, args.watchdog_ms,
            debug_fsoe ? "fsoe" : "off",
            debug_fsoe_frame ? "+frame" : "",
            debug_fsoe_raw ? "+raw" : "",
            debug_fsoe_wire ? "+wire" : "",
            debug_fsoe_sequence ? "+seq" : "",
            debug_fsoe_crc ? "+crc" : "");
    } else {
        TETHER_LOGI(TAG, "FSoE disabled (--no-fsoe)");
    }

    // --- Start realtime motion loop ---
    // This must happen BEFORE enabling the drive so that the FSoE cyclic task
    // can exchange frames and progress toward the Data state.  Once FSoE
    // reaches Data, the drive's safe state is cleared and the CiA 402 enable
    // sequence can succeed.
    EtherCAT::Master::RealtimeMotionLoopConfig loop_config;
    loop_config.cycle_period_us = 1000;
    loop_config.sync_interval_cycles = 10;
    loop_config.enable_dc_synchronization = args.enable_dc_sync;
    if (!master.startRealtimeMotionControlLoop(loop_config)) {
        TETHER_LOGE(TAG, "Failed to start realtime motion loop");
        rc = 7;
        master.clearCyclicTasks();
        (void)master.removeMotionController(slave_idx);
        Tether::Examples::shutdownSingleDrive(master, slave_idx);
        Tether::Examples::stopHostMasterSession(master, session);
        return rc;
    }

    // --- Wait for FSoE to reach Data state (if enabled) ---
    // The drive starts in SAFE STATE when FSoE safety is active.  The FSoE
    // master must reach the Data state to clear the safe state and allow the
    // CiA 402 enable sequence to proceed.
    //
    // The FSoE state machine runs in the realtime motion loop thread.  We
    // block here on the std::shared_future that is fulfilled by the state
    // change callback (see above) when Data state is reached or an Error
    // occurs.  This is the idiomatic C++ one-shot cross-thread signaling
    // pattern: no polling, no sleep loops, no shared mutable state beyond
    // the promise/future pair.
    //
    // If FSoE does NOT reach Data state (handshake fails, slave sends Reset,
    // CRC error, watchdog timeout, etc.), we do NOT attempt to enable the
    // drive — the safe state is still active and the enable would fail or
    // be unsafe.  Instead, we skip the run loop and proceed directly to
    // clean shutdown.
    bool fsoe_data_reached = false;
    if (fsoe_main) {
        TETHER_LOGI(TAG, "Waiting for FSoE Data state (timeout 5 s)...");
        using namespace std::chrono_literals;
        const auto fsoe_status = fsoe_ready_future.wait_for(5s);
        if (fsoe_status == std::future_status::timeout) {
            TETHER_LOGE(TAG,
                "FSoE did not reach Data state within 5 s — "
                "current state: %s.  Aborting drive enable.",
                fsoeStateName(fsoe_main->rawConnection().getState()));
            rc = 9;
        } else if (!fsoe_ready_future.get()) {
            TETHER_LOGE(TAG,
                "FSoE entered Error state before reaching Data — "
                "aborting drive enable.");
            rc = 9;
        } else {
            TETHER_LOGI(TAG,
                "FSoE Data state reached — enabling CiA 402 drive.");
            fsoe_data_reached = true;
        }
    } else {
        // FSoE not enabled — drive enable can proceed directly.
        fsoe_data_reached = true;
    }

    // --- Enable the drive (only if FSoE is operational or not enabled) ---
    if (fsoe_data_reached) {
        if (!master.enableDrive(slave_idx, 5000)) {
            TETHER_LOGE(TAG, "Failed to enable slave %u", slave_idx);
            rc = 8;
        } else {
            TETHER_LOGI(TAG, "Slave %u drive enabled", slave_idx);
        }
    } else {
        // FSoE failed — skip drive enable, set error code.
        rc = 9;
    }

    // --- Run loop ---
    // If the drive was enabled, run the CST loop for the requested duration.
    // If FSoE is enabled, monitor it during the run: if the slave enters
    // FailSafe or Error (e.g. sends a Reset due to a CRC error), shut down
    // cleanly immediately rather than continuing to run in an unsafe state.
    if (rc == 0) {
        TETHER_LOGI(TAG,
            "CST mode active, sine torque %.3f Nm P-P at %.3f Hz for %.1f s",
            args.torque_pp_nm, args.freq_hz, args.duration);

        const auto run_start_ms = Tether::Platform::Clock::instance().getMilliseconds();
        const uint32_t run_duration_ms =
            static_cast<uint32_t>(args.duration * 1000.0);

        while (true) {
            const auto elapsed_ms =
                Tether::Platform::Clock::instance().getMilliseconds() - run_start_ms;
            if (elapsed_ms >= run_duration_ms) break;

            // If FSoE is enabled, check that it's still operational.
            // If the slave enters FailSafe or Error, stop immediately.
            if (fsoe_main) {
                const auto status = fsoe_main->rawConnection().getStatus();
                if (status.isFailSafe() || status.hasError()) {
                    TETHER_LOGE(TAG,
                        "FSoE entered %s state during run (code=0x%04X) — "
                        "shutting down cleanly",
                        status.isFailSafe() ? "FailSafe" : "Error",
                        status.error_code);
                    rc = 10;
                    break;
                }
            }

            Tether::Platform::Clock::instance().delayMilliseconds(50);
        }
    }

    // --- Stop and clean up ---
    master.stopMotionControlLoop();
    master.clearCyclicTasks();
    (void)master.removeMotionController(slave_idx);

    // Final FSoE diagnostics dump
    if (fsoe_main) {
        TETHER_LOGI(TAG, "=== Final FSoE Diagnostics ===");
        TETHER_LOGI(TAG, "%s", fsoe_main->rawConnection().getDiagnostics().c_str());
    }

    // PDO transfer statistics — reveals WKC errors (slave not ack'ing frames)
    {
        const auto& pdo = master.ethercatMaster().pdo();
        const auto& st = pdo.getStats();
        TETHER_LOGI(TAG, "=== PDO Transfer Statistics ===");
        TETHER_LOGI(TAG, "  [FMMU] total_cycles=%llu rx_sent=%llu tx_recv=%llu",
            (unsigned long long)st.total_cycles,
            (unsigned long long)st.rxpdo_frames_sent,
            (unsigned long long)st.txpdo_frames_recv);
        TETHER_LOGI(TAG, "  [FMMU] rx_errors=%u tx_errors=%u wkc_errors=%u",
            st.rxpdo_errors, st.txpdo_errors, st.wkc_errors);
        const auto& ps = pdo.getPhysicalStats();
        TETHER_LOGI(TAG, "  [PHYS] fpwr_ok=%u fpwr_wkc_err=%u fprd_ok=%u fprd_wkc_err=%u",
            ps.fpwr_success, ps.fpwr_wkc_errors, ps.fprd_success, ps.fprd_wkc_errors);
        TETHER_LOGI(TAG, "  [PHYS] send_err=%u timeout_err=%u",
            ps.send_errors, ps.timeout_errors);
    }

    Tether::Examples::shutdownSingleDrive(master, slave_idx);
    Tether::Examples::stopHostMasterSession(master, session);
    return rc;
}
