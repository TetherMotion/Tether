/**
 * @file synapticon_cst_fsoe.cpp
 * @brief Synapticon SOMANET drive — CST mode + FSoE safe-motion example
 *
 * Interfaces to a Synapticon SOMANET drive (Vendor 0x22D2, CiA 402 firmware
 * v5.1.x), puts it into Cyclic Sync Torque (CST) mode, maps the SOMANET
 * PDOs for that mode (RxPDO 0x1600 / TxPDO 0x1A00), sends 0 torque, and
 * runs the FSoE safe-motion protocol alongside the cyclic data exchange.
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
 *   ./synapticon_cst_fsoe                       # eth0, slave 0, 10 s
 *   ./synapticon_cst_fsoe -i enx34298f762c4e    # specify interface
 *   ./synapticon_cst_fsoe -s 1 -d 30            # slave 1, 30 s
 *   ./synapticon_cst_fsoe --no-fsoe             # CST only, no FSoE
 *   ./synapticon_cst_fsoe --dc-sync             # enable DC synchronization
 *   ./synapticon_cst_fsoe --connection-id 0x4321 --watchdog-ms 15
 *   ./synapticon_cst_fsoe --debug fsoe-master   # verbose per-cycle FSoE frame logging
 */

#include <array>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <iostream>
#include <memory>
#include <string>

#include "DS402ExampleSupport.hpp"
#include "tether/drives/Synapticon.hpp"
#include "tether/drives/Synapticon/SynapticonPDO.hpp"
#include "tether/ethercat/ALResetController.hpp"
#include "tether/ethercat/FaultDetection.hpp"
#include "tether/ethercat/Slave.hpp"
#include "tether/fsoe/FSoEDefs.hpp"
#include "tether/fsoe/Synapticon/SafeMotionFSoE.hpp"
#include "tether/platform/EspCompat.hpp"
#include "tether/profiles/cia301/CiA402Defs.hpp"
#include "tether/profiles/cia402/DS402Master.hpp"

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
// The motion PDO (0x1600/0x1A00) comes first; the FSoE PDO (0x1700/0x1B00)
// follows at the offset equal to the motion PDO size.
constexpr size_t kFSoERxPDOOffset = sizeof(RxPDO);   // 19 bytes
constexpr size_t kFSoETxPDOOffset = sizeof(TxPDO);   // 13 bytes

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
// Zero-torque motion controller for CST mode
// ============================================================================

template <typename PDO>
class ZeroTorqueController final : public EtherCAT::DS402Master::IDriveMotionController {
public:
    bool start(EtherCAT::CiA402Drive& drive) override {
        return drive.setOperatingMode(CiA402::OperatingMode::CyclicSyncTorque);
    }

    void stop(EtherCAT::CiA402Drive&) override {}

    bool update(EtherCAT::CiA402Drive& drive, double /*dt_seconds*/) override {
        auto* rx = drive.rxPDO<PDO>();
        if (rx == nullptr) return false;

        rx->controlword = static_cast<uint16_t>(CiA402::ControlWord::ENABLE_OPERATION);
        rx->modes_of_operation = CiA402::OperatingMode::CyclicSyncTorque;

        if constexpr (requires(PDO& pdo) { pdo.target_torque; }) {
            rx->target_torque = 0;
        }
        if constexpr (requires(PDO& pdo) { pdo.target_velocity; }) {
            rx->target_velocity = 0;
        }
        if constexpr (requires(PDO& pdo) { pdo.target_position; }) {
            rx->target_position = 0;
        }

        return true;
    }
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

        auto* tx = drive->txPDO<TxPDO>();
        if (tx) {
            TETHER_LOGI(TAG,
                "--- Drive @ %llu ms ---",
                static_cast<unsigned long long>(elapsed_ms_));
            TETHER_LOGI(TAG,
                "  statusword=0x%04X mode_display=%d torque_actual=%d "
                "position_actual=%lld",
                tx->statusword,
                static_cast<int>(tx->modes_of_operation_display),
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
                        size_t tx_pdo_offset)
        : slave_index_(slave_index)
        , main_instance_(main_instance)
        , rx_pdo_offset_(rx_pdo_offset)
        , tx_pdo_offset_(tx_pdo_offset)
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

        const bool ok = main_instance_.exchangeViaPDO(
            rx_buffer, sizeof(FSoERxPDO),
            tx_buffer, sizeof(FSoETxPDO),
            elapsed_time_ms_);

        return ok;
    }

private:
    uint16_t slave_index_;
    FSoEMain& main_instance_;
    size_t rx_pdo_offset_;
    size_t tx_pdo_offset_;
    uint64_t elapsed_time_ms_ = 0;
};

// ============================================================================
// Main
// ============================================================================

struct Args {
    std::string interface = "eth0";
    int slave_index = 0;
    double duration = 10.0;
    bool enable_fsoe = true;
    bool enable_dc_sync = false;
    uint16_t connection_id = 0x1234;
    uint16_t watchdog_ms = EtherCAT::Drives::Synapticon::SafeMotion::Timing::kMinimumWatchdogTimeMs;
    uint32_t diag_interval_ms = 1000;
    std::string debug;
};

bool parseArgs(int argc, char** argv, Args& out) {
    argparse::ArgumentParser program("synapticon_cst_fsoe");
    program.add_argument("-i", "--interface")
        .default_value(std::string("eth0"))
        .help("Network interface (e.g. eth0, enx34298f762c4e)");
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
        .help("Comma-separated debug flags: 'fsoe-master' for per-cycle FSoE frame logging");

    try {
        program.parse_args(argc, argv);
    } catch (const std::runtime_error& err) {
        std::cerr << err.what() << "\n" << program;
        return false;
    }

    out.interface = program.get<std::string>("--interface");
    out.slave_index = program.get<int>("--slave");
    out.duration = program.get<double>("--duration");
    out.enable_fsoe = !program.get<bool>("--no-fsoe");
    out.enable_dc_sync = program.get<bool>("--dc-sync");
    out.connection_id = static_cast<uint16_t>(program.get<unsigned int>("--connection-id"));
    out.watchdog_ms = static_cast<uint16_t>(program.get<int>("--watchdog-ms"));
    out.diag_interval_ms = static_cast<uint32_t>(program.get<int>("--diag-interval-ms"));
    out.debug = program.get<std::string>("--debug");
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
        "synapticon_cst_fsoe — interface=%s slave=%u duration=%.1f fsoe=%s dc_sync=%s debug='%s'",
        args.interface.c_str(), slave_idx, args.duration,
        args.enable_fsoe ? "on" : "off",
        args.enable_dc_sync ? "on" : "off",
        args.debug.c_str());

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
            TETHER_LOGE(TAG,
                "Failed to read safety module diagnostics (0x2611) via SDO — "
                "cannot verify safety state, aborting activation");
            Tether::Examples::stopHostMasterSession(master, session);
            return 2;
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
                "connection ID (overrides --connection-id=0x%04X)",
                drive_safety_address,
                args.connection_id);
            args.connection_id = drive_safety_address;
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
        // Combined motion + FSoE PDO mapping via multi-PDO assignment
        const auto assignment =
            EtherCAT::Drives::Synapticon_pdo::makePDOAssignment(
                {EtherCAT::Drives::Synapticon_pdo::RxPDO_1600.index,
                 EtherCAT::Drives::Synapticon_pdo::RxPDO_1700.index},
                {EtherCAT::Drives::Synapticon_pdo::TxPDO_1A00.index,
                 EtherCAT::Drives::Synapticon_pdo::TxPDO_1B00.index});

        TETHER_LOGI(TAG,
            "Transitioning to OP with combined CST+FSoE PDO mapping: "
            "SM2=RxPDO 0x1600+0x1700 (%u+%u=%u bytes), "
            "SM3=TxPDO 0x1A00+0x1B00 (%u+%u=%u bytes)",
            EtherCAT::Drives::Synapticon_pdo::RxPDO_1600.size,
            EtherCAT::Drives::Synapticon_pdo::RxPDO_1700.size,
            static_cast<uint16_t>(EtherCAT::Drives::Synapticon_pdo::RxPDO_1600.size +
                                  EtherCAT::Drives::Synapticon_pdo::RxPDO_1700.size),
            EtherCAT::Drives::Synapticon_pdo::TxPDO_1A00.size,
            EtherCAT::Drives::Synapticon_pdo::TxPDO_1B00.size,
            static_cast<uint16_t>(EtherCAT::Drives::Synapticon_pdo::TxPDO_1A00.size +
                                  EtherCAT::Drives::Synapticon_pdo::TxPDO_1B00.size));

        if (!drive.transitionToOp(assignment)) {
            TETHER_LOGE(TAG, "Failed to transition to OP with combined PDO assignment");
            master.stopDistributedClocks();
            Tether::Examples::stopHostMasterSession(master, session);
            return 3;
        }
        TETHER_LOGI(TAG, "Slave %u transitioned to OP with combined CST+FSoE PDOs", slave_idx);
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

    // --- Add zero-torque motion controller ---
    if (!master.addMotionController(
            slave_idx,
            std::make_unique<ZeroTorqueController<RxPDO>>())) {
        TETHER_LOGE(TAG, "Failed to add zero-torque controller");
        rc = 4;
        Tether::Examples::shutdownSingleDrive(master, slave_idx);
        Tether::Examples::stopHostMasterSession(master, session);
        return rc;
    }

    // --- Set up FSoE safe-motion (real drive via PDOs) ---
    std::unique_ptr<FSoEMain> fsoe_main;

    if (args.enable_fsoe) {
        const bool debug_fsoe_master =
            (args.debug.find("fsoe-master") != std::string::npos);

        EtherCAT::Drives::Synapticon::SafeMotion::MainConfig main_config;
        main_config.feature_enabled = true;
        main_config.slave_address = slave_idx;
        main_config.safety_address = 0x0001;
        main_config.connection_id = args.connection_id;
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
            [](uint16_t code) {
                TETHER_LOGE(TAG,
                    "[FSoE] error: 0x%04X (%s)",
                    code, fsoeErrorName(code));
            });
        fsoe_main->rawConnection().setFailSafeCallback(
            []() {
                TETHER_LOGW(TAG, "[FSoE] fail-safe activated");
            });

        // Per-cycle FSoE frame trace logging (--debug fsoe-master).
        // Frame event listeners are invoked from inside the FSoE state
        // machine for every master→slave (tx) and slave→master (rx) frame.
        // Each listener receives an immutable shared_ptr<const
        // std::vector<uint8_t>> copy of the frame bytes.
        if (debug_fsoe_master) {
            fsoe_main->rawConnection().txFrameEvents().addListener(
                [](std::shared_ptr<const std::vector<uint8_t>> data) {
                    const uint8_t cmd = (!data->empty()) ? (*data)[0] : 0;
                    TETHER_LOGI(TAG, "[fsoe-master] TX (master->slave) len=%zu cmd=%s",
                                data->size(), fsoeCommandName(cmd));
                    hexDump(TAG, "TX (master->slave)", data->data(), data->size());
                });
            fsoe_main->rawConnection().rxFrameEvents().addListener(
                [](std::shared_ptr<const std::vector<uint8_t>> data) {
                    const uint8_t cmd = (!data->empty()) ? (*data)[0] : 0;
                    TETHER_LOGI(TAG, "[fsoe-master] RX (slave->master) len=%zu cmd=%s",
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
                    kFSoERxPDOOffset, kFSoETxPDOOffset))) {
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
            "FSoE enabled: conn_id=0x%04X watchdog=%u ms debug_master=%s",
            args.connection_id, args.watchdog_ms,
            debug_fsoe_master ? "on" : "off");
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
    if (fsoe_main) {
        constexpr uint32_t kFsoEStartupTimeoutMs = 5000;
        constexpr uint32_t kFsoEPollIntervalMs = 50;
        TETHER_LOGI(TAG,
            "Waiting up to %u ms for FSoE to reach Data state...",
            kFsoEStartupTimeoutMs);

        uint32_t waited_ms = 0;
        bool fsoe_data_reached = false;
        while (waited_ms < kFsoEStartupTimeoutMs) {
            const auto status = fsoe_main->rawConnection().getStatus();
            if (status.isOperational()) {
                fsoe_data_reached = true;
                break;
            }
            if (status.hasError()) {
                TETHER_LOGE(TAG,
                    "FSoE entered error state (code=0x%04X) during startup",
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
            TETHER_LOGW(TAG,
                "FSoE did not reach Data state within %u ms (state=%s) — "
                "attempting drive enable anyway",
                waited_ms,
                fsoeStateName(fsoe_main->rawConnection().getStatus().state));
        }
    }

    // --- Enable the drive ---
    // Now that FSoE has had a chance to reach Data state and clear the safe
    // state, attempt the CiA 402 enable sequence.  If this fails, we continue
    // running so the user can observe FSoE debug output (--debug fsoe-master).
    if (!master.enableDrive(slave_idx, 5000)) {
        TETHER_LOGE(TAG, "Failed to enable slave %u — continuing for diagnostics", slave_idx);
        rc = 8;
    } else {
        TETHER_LOGI(TAG, "Slave %u drive enabled", slave_idx);
    }

    TETHER_LOGI(TAG, "CST mode active, sending 0 torque for %.1f s", args.duration);

    Tether::Platform::Clock::instance().delayMilliseconds(
        static_cast<uint32_t>(args.duration * 1000.0));

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
