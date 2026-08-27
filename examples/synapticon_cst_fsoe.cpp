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
 *   ./synapticon_cst_fsoe --connection-id 0x4321 --watchdog-ms 15
 *   ./synapticon_cst_fsoe --debug fsoe          # high-level FSoE protocol trace
 *   ./synapticon_cst_fsoe --debug fsoe-frame    # decoded FSoE PDO struct fields (on change)
 *   ./synapticon_cst_fsoe --debug fsoe-raw      # FSoE protocol trace + raw frame hex dumps (on change)
 *   ./synapticon_cst_fsoe --debug fsoe-wire     # every-cycle PDO wire dumps (firehose)
 *   ./synapticon_cst_fsoe --debug fsoe-sequence # per-cycle frame accept/reject + state change summary
 *   ./synapticon_cst_fsoe --debug fsoe-crc      # CRC parameters used for TX build and RX check
 *   ./synapticon_cst_fsoe --torque-nm 10 --freq-hz 1.0  # 10 Nm P-P at 1 Hz
 *   ./synapticon_cst_fsoe --rated-torque-mnm 4200       # override rated torque
 */

#include <array>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
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
#include "tether/sii/SIIReader.hpp"

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
constexpr uint16_t kMailboxWriteSize = EtherCAT::Drives::Synapticon::kMailboxWriteSize;

// SM1 — slave→master read mailbox (ESI "MBoxIn", ControlByte 0x22)
constexpr uint16_t kMailboxReadAddr = EtherCAT::Drives::Synapticon::kMailboxReadAddr;
constexpr uint16_t kMailboxReadSize = EtherCAT::Drives::Synapticon::kMailboxReadSize;

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
// API.  The PDO buffer layout is:
//   SM2 (Rx, master→slave): [0x1600 (19B)][0x1700 (11B)] = 30 bytes
//   SM3 (Tx, slave→master): [0x1A00 (13B)][0x1B00 (31B)] = 44 bytes
using RxPDO = EtherCAT::Drives::Synapticon_pdo::SOMANET_RxPDO_1600;
using TxPDO = EtherCAT::Drives::Synapticon_pdo::SOMANET_TxPDO_1A00;
using FSoERxPDO = EtherCAT::Drives::Synapticon_pdo::SOMANET_RxPDO_1700;
using FSoETxPDO = EtherCAT::Drives::Synapticon_pdo::SOMANET_TxPDO_1B00;

// FSoE PDO offsets within the combined PDO buffer.
// The FSoE PDO (0x1700/0x1B00) comes FIRST; the motion PDO (0x1600/0x1A00)
// follows at the offset equal to the FSoE PDO size.
// This ordering is critical: the Synapticon Circulo EtherCAT chip has a bug
// where the last word in the SM buffer is zeroed.  If the FSoE PDO were last,
// the ConnectionID (the final word of the FSoE frame) would be zeroed and the
// slave would reject every frame.  By placing the motion PDO last, the zeroed
// word falls on motion data, not the FSoE ConnectionID.
// See: https://doc.synapticon.com/circulo_safe_motion/smm/ecat_fsoe_issues.htm
constexpr size_t kFSoERxPDOOffset = 0;                    // FSoE first
constexpr size_t kMotionRxPDOOffset = sizeof(FSoERxPDO);  // 11 bytes
constexpr size_t kFSoETxPDOOffset = 0;                    // FSoE first
constexpr size_t kMotionTxPDOOffset = sizeof(FSoETxPDO);  // 31 bytes

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

/// Decode the master→slave FSoE frame from the Synapticon RxPDO 0x1700 struct.
void dumpFSoERxPDO(const char* tag, const FSoERxPDO& rx) {
    TETHER_LOGI(tag, "[fsoe-frame] TX→slave RxPDO 0x1700 (11 bytes):");
    TETHER_LOGI(tag, "  cmd=%s  conn_id=0x%04X",
                fsoeCommandName(rx.fsoe_command), rx.fsoe_connection_id);

    // Safety flags (master→slave commands)
    char flags[128] = {};
    size_t pos = 0;
    appendFlag(flags, pos, sizeof(flags), rx.safety_flags & FSoERxPDO::kSTO, "STO");
    pos = strlen(flags);
    appendFlag(flags, pos, sizeof(flags), rx.safety_flags & FSoERxPDO::kSS1, "SS1");
    pos = strlen(flags);
    appendFlag(flags, pos, sizeof(flags), rx.safety_flags & FSoERxPDO::kSS2, "SS2");
    pos = strlen(flags);
    appendFlag(flags, pos, sizeof(flags), rx.safety_flags & FSoERxPDO::kSOS, "SOS");
    pos = strlen(flags);
    appendFlag(flags, pos, sizeof(flags), rx.safety_flags & FSoERxPDO::kSBCCommand, "SBC");
    pos = strlen(flags);
    appendFlag(flags, pos, sizeof(flags), rx.safety_flags & FSoERxPDO::kSLS_Instance1, "SLS1");
    pos = strlen(flags);
    appendFlag(flags, pos, sizeof(flags), rx.safety_flags & FSoERxPDO::kSLS_Instance2, "SLS2");
    pos = strlen(flags);
    appendFlag(flags, pos, sizeof(flags), rx.safety_flags & FSoERxPDO::kSLS_Instance3, "SLS3");
    pos = strlen(flags);
    appendFlag(flags, pos, sizeof(flags), rx.safety_flags & FSoERxPDO::kSLS_Instance4, "SLS4");
    pos = strlen(flags);
    appendFlag(flags, pos, sizeof(flags), rx.safety_flags & FSoERxPDO::kResetPosition, "ResetPos");
    pos = strlen(flags);
    appendFlag(flags, pos, sizeof(flags), rx.safety_flags & FSoERxPDO::kErrorAck, "ErrorAck");
    pos = strlen(flags);
    appendFlag(flags, pos, sizeof(flags), rx.safety_flags & FSoERxPDO::kRestartAck, "RestartAck");
    TETHER_LOGI(tag, "  safety_flags=0x%04X [%s]", rx.safety_flags,
                flags[0] ? flags : "(none)");

    TETHER_LOGI(tag, "  crc0=0x%04X  crc1=0x%04X",
                rx.fsoe_crc_0, rx.fsoe_crc_1);

    // Safe outputs
    char outs[32] = {};
    pos = 0;
    appendFlag(outs, pos, sizeof(outs), rx.safe_outputs & FSoERxPDO::kSafeOutput1, "OUT1");
    pos = strlen(outs);
    appendFlag(outs, pos, sizeof(outs), rx.safe_outputs & FSoERxPDO::kSafeOutput2, "OUT2");
    TETHER_LOGI(tag, "  safe_outputs=0x%02X [%s]", rx.safe_outputs,
                outs[0] ? outs : "(none)");
}

/// Decode the slave→master FSoE frame from the Synapticon TxPDO 0x1B00 struct.
void dumpFSoETxPDO(const char* tag, const FSoETxPDO& tx) {
    TETHER_LOGI(tag, "[fsoe-frame] RX←slave TxPDO 0x1B00 (31 bytes):");
    TETHER_LOGI(tag, "  cmd=%s  conn_id=0x%04X",
                fsoeCommandName(tx.fsoe_command), tx.fsoe_connection_id);

    // Safety state flags (slave→master status)
    char sflags[128] = {};
    size_t pos = 0;
    appendFlag(sflags, pos, sizeof(sflags), tx.safety_state_flags & FSoETxPDO::kSTOState, "STO");
    pos = strlen(sflags);
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

    // Diagnostic flags
    char dflags[160] = {};
    pos = 0;
    appendFlag(dflags, pos, sizeof(dflags), tx.diagnostic_flags & FSoETxPDO::kRestartAckReq, "RestartAckReq");
    pos = strlen(dflags);
    appendFlag(dflags, pos, sizeof(dflags), tx.diagnostic_flags & FSoETxPDO::kSBCState, "SBC");
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
        // Motion PDO is at offset kMotionRxPDOOffset (FSoE PDO comes first)
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

        // Motion PDO is at offset kMotionTxPDOOffset (FSoE PDO comes first)
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
        // The motion PDO (0x1600/0x1A00) occupies the first bytes; the FSoE
        // PDO (0x1700/0x1B00) follows at the configured offset.
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

        // --debug fsoe-raw: hex dump on change
        if (debug_raw_ && tx_changed) {
            char hex[128];
            size_t pos = 0;
            for (size_t b = 0; b < sizeof(FSoETxPDO) && pos + 3 < sizeof(hex); b++) {
                pos += static_cast<size_t>(snprintf(hex + pos, sizeof(hex) - pos, "%02X ", tx_buffer[b]));
            }
            TETHER_LOGI("fsoe-cyclic", "[TxPDO-FSoE slave→master] changed: %s", hex);
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
            char hex[128];
            size_t pos = 0;
            for (size_t b = 0; b < sizeof(FSoERxPDO) && pos + 3 < sizeof(hex); b++) {
                pos += static_cast<size_t>(snprintf(hex + pos, sizeof(hex) - pos, "%02X ", rx_buffer[b]));
            }
            TETHER_LOGI("fsoe-cyclic", "[RxPDO-FSoE master→slave] changed: %s", hex);
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
        char hex[128];
        size_t pos;

        // TxPDO (slave-to-master) -- FSoE region
        pos = 0;
        for (size_t b = 0; b < sizeof(FSoETxPDO) && pos + 3 < sizeof(hex); b++) {
            pos += static_cast<size_t>(snprintf(hex + pos, sizeof(hex) - pos, "%02X ", tx_buffer[b]));
        }
        TETHER_LOGI("fsoe-wire", "[TxPDO] cycle %u: %s", cycle_count_, hex);

        // RxPDO (master-to-slave) -- FSoE region
        pos = 0;
        for (size_t b = 0; b < sizeof(FSoERxPDO) && pos + 3 < sizeof(hex); b++) {
            pos += static_cast<size_t>(snprintf(hex + pos, sizeof(hex) - pos, "%02X ", rx_buffer[b]));
        }
        TETHER_LOGI("fsoe-wire", "[RxPDO] cycle %u: %s", cycle_count_, hex);

        cycle_count_++;
    }
};

// ============================================================================
// Main
// ============================================================================

struct Args {
    std::string interface;
    int slave_index = 0;
    double duration = 10.0;
    bool enable_fsoe = true;
    bool enable_dc_sync = false;
    uint16_t connection_id = 0x1234;
    uint16_t watchdog_ms = EtherCAT::Drives::Synapticon::SafeMotion::Timing::kMinimumWatchdogTimeMs;
    uint32_t diag_interval_ms = 1000;
    std::string debug;
    double torque_pp_nm = 0.5;       ///< Peak-to-peak torque amplitude in Nm
    double freq_hz = 0.5;            ///< Sine wave frequency in Hz
    uint32_t rated_torque_mnm = 0;   ///< Motor rated torque in mNm (0 = auto-detect from 0x6076)
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
        .default_value(static_cast<unsigned int>(0x1234))
        .help("FSoE connection ID (hex)");
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
        .help("Comma-separated debug flags: 'fsoe' for high-level protocol trace, "
              "'fsoe-frame' for decoded PDO struct fields (on change), "
              "'fsoe-raw' for raw frame hex dumps (on change), "
              "'fsoe-wire' for every-cycle PDO wire dumps, "
              "'fsoe-sequence' for per-cycle frame accept/reject + state change summary, "
              "'fsoe-crc' for CRC parameters used in TX build and RX check");
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
    out.watchdog_ms = static_cast<uint16_t>(program.get<int>("--watchdog-ms"));
    out.diag_interval_ms = static_cast<uint32_t>(program.get<int>("--diag-interval-ms"));
    out.debug = program.get<std::string>("--debug");
    out.torque_pp_nm = program.get<double>("--torque-nm");
    out.freq_hz = program.get<double>("--freq-hz");
    out.rated_torque_mnm = static_cast<uint32_t>(program.get<int>("--rated-torque-mnm"));
    return true;
}

} // namespace

int main(int argc, char** argv) {
    Args args;
    if (!parseArgs(argc, argv, args)) return 1;

    if (args.slave_index < 0 || args.slave_index > 65535) {
        std::cerr << "Invalid slave index\n";
        return 1;
    }
    const uint16_t slave_idx = static_cast<uint16_t>(args.slave_index);

    Tether::Platform::ensureRealtimeKernelOrExit();

    TETHER_LOGI(TAG,
        "synapticon_cst_fsoe — interface=%s slave=%u duration=%.1f fsoe=%s dc_sync=%s debug='%s' "
        "torque_pp=%.3fNm freq=%.3fHz rated_torque_mnm=%u",
        args.interface.c_str(), slave_idx, args.duration,
        args.enable_fsoe ? "on" : "off",
        args.enable_dc_sync ? "on" : "off",
        args.debug.c_str(),
        args.torque_pp_nm, args.freq_hz, args.rated_torque_mnm);

    // --- Start EtherCAT master ---
    EtherCAT::DS402Master master;
    Tether::Examples::HostMasterSession session;
    if (!Tether::Examples::startHostMasterSession(args.interface, master, session, TAG)) {
        return 2;
    }

    // --- Configure mailbox with SOMANET ESI values ---
    // The SOMANET_CiA_402_v5.1.9.xml ESI defines the mailbox sync managers
    // with 1024-byte buffers at 0x1000 (SM0, M→S write) and 0x1400 (SM1,
    // S→M read), supporting CoE + FoE.  These values are provided by the
    // Synapticon driver header (tether/drives/Synapticon.hpp) and are used
    // here instead of relying on SII EEPROM auto-configuration so the
    // correct mailbox geometry is always used for SOMANET drives.
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

        // --- Verify slave identity (manufacturer + device) before init ---
        // The slave's SII EEPROM identity (vendor/product) is read directly
        // via the ESC — no mailbox or AL state required — so it works even in
        // INIT.  A vendor mismatch is fatal (this example only targets
        // Synapticon SOMANET drives).  An unknown product code is a non-fatal
        // warning: the drive may still be a compatible SOMANET variant not yet
        // listed in the known-product-code table.
        //
        // SII read failure is NON-FATAL: some ESCs (e.g. Synapticon ESC211)
        // do not support APWR to the EEPROM control register, making SII
        // reads impossible via the standard register interface.  In that case
        // we warn and continue — the mailbox/PDO configuration uses hardcoded
        // ESI values, not SII, so the drive can still be operated.
        {
            EtherCAT::SII::SIIIdentity sii_id;
            if (!EtherCAT::SII::readSIIIdentity(
                    master.ethercatMaster(), slave_idx, sii_id)) {
                TETHER_LOGW(TAG,
                    "Slave %u: SII identity read failed — cannot verify "
                    "manufacturer/device via EEPROM.  This is expected on "
                    "ESCs that do not support APWR to EEPCTL (e.g. "
                    "Synapticon ESC211).  Continuing without identity "
                    "verification; mailbox/PDO config uses hardcoded ESI "
                    "values.  Use --debug eeprom for low-level EEPROM "
                    "register diagnostics.",
                    slave_idx);
                // Non-fatal: continue without identity verification.
            } else {
                TETHER_LOGI(TAG,
                    "Slave %u identity: vendor=0x%08X product=0x%08X "
                    "revision=0x%08X serial=0x%08X",
                    slave_idx,
                    sii_id.vendor_id, sii_id.product_code,
                    sii_id.revision_number, sii_id.serial_number);

                if (sii_id.vendor_id != EtherCAT::Drives::Synapticon::kVendorId) {
                    TETHER_LOGE(TAG,
                        "Slave %u: VENDOR MISMATCH — expected 0x%08X "
                        "(Synapticon), got 0x%08X.  This example only supports "
                        "Synapticon SOMANET drives, aborting.",
                        slave_idx,
                        EtherCAT::Drives::Synapticon::kVendorId,
                        sii_id.vendor_id);
                    Tether::Examples::stopHostMasterSession(master, session);
                    return 2;
                }

                if (!EtherCAT::Drives::Synapticon::isKnownProductCode(
                        sii_id.product_code)) {
                    TETHER_LOGW(TAG,
                        "Slave %u: UNKNOWN product code 0x%08X (vendor matches "
                        "Synapticon).  Not in known-product list "
                        "{0x0201 Node, 0x0301 Circulo, 0x0302 Circulo 7 "
                        "SafeMotion} — continuing, but PDO/safety layout may not "
                        "match this device.",
                        slave_idx, sii_id.product_code);
                } else {
                    TETHER_LOGI(TAG,
                        "Slave %u: product code 0x%08X is a known SOMANET device",
                        slave_idx, sii_id.product_code);
                }
            }
        }

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

    // --- Pre-activation safety check: read 0x2611 Safety Module input diagnostics ---
    // Object 0x2611 (per Synapticon documentation) reports the state of the
    // safety module: 0 = safe state (safety function active, torque inhibited),
    // 1 = not safe state (motion allowed).
    //
    // When FSoE is enabled, the drive *starts* in safe state (STO active by
    // default) and the FSoE master brings it out of safe state once the safety
    // protocol reaches the Data state.  Aborting here would prevent the FSoE
    // connection from ever establishing, so we only abort on safe state when
    // FSoE is disabled (--no-fsoe) — in that case there is no mechanism to
    // clear the safe state and enabling the drive would be futile.
    {
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
            if (args.enable_fsoe) {
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
            if (args.enable_fsoe) {
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
    }

    // --- Read FSoE safety address (0xF980:1) and use as connection ID ---
    // Object 0xF980:1 contains the safety address configured on the drive.
    // The master must use this value as the FSoE connection ID so that
    // master and slave agree on the connection identifier.  Reading it
    // before starting the FSoE handshake avoids ConnectionIDError
    // failures caused by a mismatch between the --connection-id default
    // and the drive's configured value.
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
                "FSoE safety address (0xF980:1): 0x%04X — using as "
                "slave_safety_addr (NOT connection_id; "
                "--connection-id=0x%04X is kept)",
                drive_safety_address,
                args.connection_id);
            // Use the drive's safety address as the slave_safety_addr
            // (already set to 0x0006 in main_config below), but do NOT
            // override the connection_id — they are separate FSoE concepts.
            // The connection_id comes from --connection-id (default 0x1234).
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
    //   SM2 (Rx, master→slave): [0x1600 (19B)][0x1700 (11B)] = 30 bytes
    //   SM3 (Tx, slave→master): [0x1A00 (13B)][0x1B00 (31B)] = 44 bytes
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

    // Set operating mode via SDO while in PRE_OP (before PDO mapping)
    if (!drive.setOperatingMode(CiA402::OperatingMode::CyclicSyncTorque)) {
        TETHER_LOGE(TAG, "Failed to set operating mode to CST");
        master.stopDistributedClocks();
        Tether::Examples::stopHostMasterSession(master, session);
        return 3;
    }

    if (args.enable_fsoe) {
        // Combined FSoE + motion PDO mapping via multi-PDO assignment.
        // IMPORTANT: FSoE PDOs (0x1700/0x1B00) are placed FIRST, motion PDOs
        // (0x1600/0x1A00) SECOND.  This is a workaround for a Synapticon
        // Circulo EtherCAT chip bug that zeros the last word of the SM
        // buffer.  If the FSoE PDO were last, the ConnectionID (final word
        // of the FSoE frame) would be zeroed and the slave would reject
        // every frame.
        const auto assignment =
            EtherCAT::Drives::Synapticon_pdo::makePDOAssignment(
                {EtherCAT::Drives::Synapticon_pdo::RxPDO_1700.index,
                 EtherCAT::Drives::Synapticon_pdo::RxPDO_1600.index},
                {EtherCAT::Drives::Synapticon_pdo::TxPDO_1B00.index,
                 EtherCAT::Drives::Synapticon_pdo::TxPDO_1A00.index});

        TETHER_LOGI(TAG,
            "Transitioning to OP with combined FSoE+CST PDO mapping: "
            "SM2=RxPDO 0x1700+0x1600 (%u+%u=%u bytes), "
            "SM3=TxPDO 0x1B00+0x1A00 (%u+%u=%u bytes)",
            EtherCAT::Drives::Synapticon_pdo::RxPDO_1700.size,
            EtherCAT::Drives::Synapticon_pdo::RxPDO_1600.size,
            static_cast<uint16_t>(EtherCAT::Drives::Synapticon_pdo::RxPDO_1700.size +
                                  EtherCAT::Drives::Synapticon_pdo::RxPDO_1600.size),
            EtherCAT::Drives::Synapticon_pdo::TxPDO_1B00.size,
            EtherCAT::Drives::Synapticon_pdo::TxPDO_1A00.size,
            static_cast<uint16_t>(EtherCAT::Drives::Synapticon_pdo::TxPDO_1B00.size +
                                  EtherCAT::Drives::Synapticon_pdo::TxPDO_1A00.size));

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
            // FSoE PDO is at offset 0 (FSoE comes first, motion second)
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
        // Operating mode already set above; set to 0 to skip redundant SDO write
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

    if (args.enable_fsoe) {
        // Debug flags:
        //   fsoe        — high-level protocol trace ("TX Reset(0x2A): ...",
        //                 "RX Session(0x4E): slave accepted session, ...")
        //   fsoe-frame  — decode device-specific FSoE PDO structs into named
        //                 fields (cmd, conn_id, CRCs, safety flags, safe I/O)
        //   fsoe-raw    — protocol trace + raw frame hex dumps via the
        //                 txFrameEvents/rxFrameEvents listeners
        //   fsoe-master — deprecated alias for fsoe-raw (backward compat)
        //   fsoe-sequence — per-cycle frame accept/reject + state change summary
        //   fsoe-crc     — CRC parameters used for TX build and RX check
        //                 (start_crc, seq, CRC0, fallback info)
        const bool debug_fsoe =
            (args.debug.find("fsoe") != std::string::npos);
        const bool debug_fsoe_frame =
            (args.debug.find("fsoe-frame") != std::string::npos);
        const bool debug_fsoe_raw =
            (args.debug.find("fsoe-raw") != std::string::npos ||
             args.debug.find("fsoe-master") != std::string::npos);
        const bool debug_fsoe_wire =
            (args.debug.find("fsoe-wire") != std::string::npos);
        const bool debug_fsoe_sequence =
            (args.debug.find("fsoe-sequence") != std::string::npos);
        const bool debug_fsoe_crc =
            (args.debug.find("fsoe-crc") != std::string::npos);

        EtherCAT::Drives::Synapticon::SafeMotion::MainConfig main_config;
        main_config.feature_enabled = true;
        main_config.slave_address = slave_idx;
        main_config.safety_address = 0x0006;  // FSoE slave safety address
        main_config.connection_id = args.connection_id;  // FSoE connection ID
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

        // Install FSoE callbacks for real-time state tracking
        fsoe_main->rawConnection().setStateChangeCallback(
            [](uint8_t old_s, uint8_t new_s) {
                TETHER_LOGI(TAG,
                    "[FSoE] state: %s -> %s",
                    fsoeStateName(old_s), fsoeStateName(new_s));
            });
        fsoe_main->rawConnection().setErrorCallback(
            [](uint16_t code, const FSoE::FSoEErrorDetail& detail) {
                if (detail.message[0] != '\0') {
                    TETHER_LOGE(TAG,
                        "[FSoE] error: 0x%04X (%s): %s",
                        code, fsoeErrorName(code), detail.message);
                } else {
                    TETHER_LOGE(TAG,
                        "[FSoE] error: 0x%04X (%s)",
                        code, fsoeErrorName(code));
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

        TETHER_LOGI(TAG,
            "FSoE enabled: conn_id=0x%04X watchdog=%u ms debug=%s%s%s%s%s%s",
            args.connection_id, args.watchdog_ms,
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
    // CiA 402 enable sequence to proceed.  We poll the FSoE connection state
    // for up to 5 seconds.
    //
    // If FSoE does NOT reach Data state (handshake fails, slave sends Reset,
    // CRC error, watchdog timeout, etc.), we do NOT attempt to enable the
    // drive — the safe state is still active and the enable would fail or
    // be unsafe.  Instead, we skip the run loop and proceed directly to
    // clean shutdown.
    bool fsoe_data_reached = false;
    if (fsoe_main) {
        constexpr uint32_t kFsoEStartupTimeoutMs = 5000;
        constexpr uint32_t kFsoEPollIntervalMs = 50;
        TETHER_LOGI(TAG,
            "Waiting up to %u ms for FSoE to reach Data state...",
            kFsoEStartupTimeoutMs);

        uint32_t waited_ms = 0;
        while (waited_ms < kFsoEStartupTimeoutMs) {
            const auto status = fsoe_main->rawConnection().getStatus();
            if (status.isOperational()) {
                fsoe_data_reached = true;
                break;
            }
            if (status.isFailSafe() || status.hasError()) {
                TETHER_LOGE(TAG,
                    "FSoE entered %s state (code=0x%04X) during startup — "
                    "aborting drive enable",
                    status.isFailSafe() ? "FailSafe" : "Error",
                    status.error_code);
                break;
            }
            Tether::Platform::Clock::instance().delayMilliseconds(kFsoEPollIntervalMs);
            waited_ms += kFsoEPollIntervalMs;
        }

        if (fsoe_data_reached) {
            TETHER_LOGI(TAG,
                "FSoE reached Data state after %u ms — proceeding with drive enable",
                waited_ms);
        } else {
            const auto status = fsoe_main->rawConnection().getStatus();
            TETHER_LOGE(TAG,
                "FSoE did not reach Data state (state=%s, error=0x%04X) — "
                "drive enable SKIPPED, proceeding to shutdown",
                fsoeStateName(status.state),
                status.error_code);
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

    Tether::Examples::shutdownSingleDrive(master, slave_idx);
    Tether::Examples::stopHostMasterSession(master, session);
    return rc;
}
