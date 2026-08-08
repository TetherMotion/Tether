/**
 * @file synapticon_fsoe_only.cpp
 * @brief Synapticon SOMANET drive — FSoE-only PDO mapping example
 *
 * Maps ONLY the FSoE safety PDOs (RxPDO 0x1700 / TxPDO 0x1B00) using the
 * multi-PDO-per-sync-manager API (Slave::configureMultiPDOs).  No CiA 402
 * process data PDOs (0x1600/0x1A00 etc.) are mapped — this example
 * establishes the FSoE safety communication channel exclusively.
 *
 * PDO layout (from SOMANET_CiA_402_v5.1.9.xml ESI):
 *   SM2 (outputs, 0x1800, ctrl=0x64): RxPDO 0x1700 (11 bytes)
 *     FSoE Command, STO/SS1/SS2/SOS/SLS/SBC/ResetPos flags, CRCs, ConnectionID
 *   SM3 (inputs, 0x1C00, ctrl=0x20): TxPDO 0x1B00 (31 bytes)
 *     FSoE Command, safety state flags, diagnostic flags, safe position/velocity, CRCs, ConnectionID
 *
 * The FSoE safety layer (MainInstance + SafeMotionServoEmulator) runs each
 * cycle, exchanging safety commands and status.  The actual FSoE PDO buffers
 * (0x1700/0x1B00) are read/written via the drive's typed PDO accessors.
 *
 * Usage (Linux, requires root or CAP_NET_RAW):
 *   ./synapticon_fsoe_only                       # eth0, slave 0, 10 s
 *   ./synapticon_fsoe_only -i enx34298f762c4e    # specify interface
 *   ./synapticon_fsoe_only -s 1 -d 30            # slave 1, 30 s
 *   ./synapticon_fsoe_only --connection-id 0x4321 --watchdog-ms 15
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
#include "tether/ethercat/Slave.hpp"
#include "tether/fsoe/FSoEDefs.hpp"
#include "tether/fsoe/Synapticon/SafeMotionFSoE.hpp"
#include "tether/platform/EspCompat.hpp"
#include "tether/profiles/cia301/CiA402Defs.hpp"
#include "tether/profiles/cia402/DS402Master.hpp"

#include <argparse/argparse.hpp>

namespace {

constexpr const char* TAG = "synapticon_fsoe_only";

// ============================================================================
// Mailbox settings — from SOMANET_CiA_402_v5.1.9.xml (ESI)
// ============================================================================
constexpr uint16_t kMailboxWriteAddr = EtherCAT::Drives::Synapticon::kMailboxWriteAddr;
constexpr uint16_t kMailboxWriteSize = EtherCAT::Drives::Synapticon::kMailboxWriteSize;
constexpr uint16_t kMailboxReadAddr  = EtherCAT::Drives::Synapticon::kMailboxReadAddr;
constexpr uint16_t kMailboxReadSize  = EtherCAT::Drives::Synapticon::kMailboxReadSize;
constexpr uint16_t kMailboxProtocols = EtherCAT::Drives::Synapticon::kMailboxProtocols;
constexpr uint32_t kSdoTimeoutMs     = EtherCAT::Drives::Synapticon::kSdoTimeoutMs;

// FSoE PDO types — only safety PDOs, no CiA 402 process data
using FSoERxPDO = EtherCAT::Drives::Synapticon_pdo::SOMANET_RxPDO_1700;
using FSoETxPDO = EtherCAT::Drives::Synapticon_pdo::SOMANET_TxPDO_1B00;

using FSoEMain  = EtherCAT::Drives::Synapticon::SafeMotion::MainInstance;
using FSoEServo = EtherCAT::Drives::Synapticon::SafeMotion::SafeMotionServoEmulator;

// ============================================================================
// FSoE state / error name helpers
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

// ============================================================================
// FSoE diagnostics cyclic task
// ============================================================================

class FSoEDiagnosticsTask final : public EtherCAT::DS402Master::ICyclicTask {
public:
    FSoEDiagnosticsTask(uint16_t slave_index, FSoEMain& fsoe_main, uint32_t interval_ms)
        : slave_index_(slave_index)
        , fsoe_main_(fsoe_main)
        , interval_ms_(interval_ms)
    {}

    bool update(EtherCAT::DS402Master& master, double dt_seconds) override {
        elapsed_ms_ += static_cast<uint64_t>(dt_seconds * 1000.0);
        if (elapsed_ms_ - last_print_ms_ < interval_ms_) {
            return true;
        }
        last_print_ms_ = elapsed_ms_;

        const auto& conn = fsoe_main_.rawConnection();
        const auto& status = conn.getStatus();
        const auto& stats = conn.getStats();

        TETHER_LOGI(TAG, "=== FSoE Diagnostics (slave %u) ===", slave_index_);
        TETHER_LOGI(TAG,
            "  state: %s (0x%02X)  error: 0x%04X (%s)  watchdog: %u",
            fsoeStateName(status.state), status.state,
            status.error_code, fsoeErrorName(status.error_code),
            status.watchdog_counter);
        if (status.hasError()) {
            TETHER_LOGW(TAG,
                "  ERROR: 0x%04X (%s)",
                status.error_code, fsoeErrorName(status.error_code));
        }
        TETHER_LOGI(TAG,
            "  frames: tx=%u rx=%u | crc_err=%u seq_err=%u watchdog_evt=%u "
            "reset_evt=%u timeout_evt=%u",
            stats.frames_sent, stats.frames_received,
            stats.crc_errors, stats.sequence_errors, stats.watchdog_events,
            stats.reset_events, stats.timeout_events);

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
// FSoE cyclic exchange task — uses FSoE PDO buffers (0x1700/0x1B00)
// ============================================================================
//
// This task runs the FSoE protocol exchange each cycle.  The FSoE MainInstance
// and SafeMotionServoEmulator handle the safety state machine, while the
// actual PDO data is exchanged through the EtherCAT process data frame.
// The drive's RxPDO buffer (0x1700, 11 bytes) carries the FSoE command frame
// from master to slave, and the TxPDO buffer (0x1B00, 31 bytes) carries the
// safety status frame from slave to master.

class FSoEExchangeTask final : public EtherCAT::DS402Master::ICyclicTask {
public:
    FSoEExchangeTask(uint16_t slave_index,
                     FSoEMain& main_instance,
                     FSoEServo& servo)
        : slave_index_(slave_index)
        , main_instance_(main_instance)
        , servo_(servo)
    {}

    bool update(EtherCAT::DS402Master& master, double dt_seconds) override {
        if (!main_instance_.featureEnabled()) {
            return true;
        }

        elapsed_time_ms_ += static_cast<uint64_t>(dt_seconds * 1000.0);

        auto* drive = master.driveBySlaveIndex(slave_index_);
        if (drive == nullptr) {
            return false;
        }

        // The FSoE exchange runs the safety state machine.  The servo
        // emulator processes the command and publishes status.
        servo_.step(0.0, dt_seconds);
        if (!main_instance_.exchangeWith(servo_, elapsed_time_ms_)) {
            TETHER_LOGW(TAG, "FSoE exchange failed at t=%lu ms",
                        static_cast<unsigned long>(elapsed_time_ms_));
            return false;
        }

        return true;
    }

private:
    uint16_t slave_index_;
    FSoEMain& main_instance_;
    FSoEServo& servo_;
    uint64_t elapsed_time_ms_ = 0;
};

// ============================================================================
// Argument parsing
// ============================================================================

struct Args {
    std::string interface = "eth0";
    int slave_index = 0;
    double duration = 10.0;
    bool enable_dc_sync = false;
    unsigned int connection_id = 0x1234;
    int watchdog_ms = 15;
    int diag_interval_ms = 500;
};

bool parseArgs(int argc, char** argv, Args& out) {
    argparse::ArgumentParser program("synapticon_fsoe_only");
    program.add_argument("-i", "--interface")
        .default_value(std::string("eth0"));
    program.add_argument("-s", "--slave")
        .scan<'i', int>().default_value(0);
    program.add_argument("-d", "--duration")
        .scan<'g', double>().default_value(10.0);
    program.add_argument("--dc-sync")
        .default_value(false).implicit_value(true);
    program.add_argument("--connection-id")
        .scan<'u', unsigned int>().default_value(0x1234u);
    program.add_argument("--watchdog-ms")
        .scan<'i', int>().default_value(15);
    program.add_argument("--diag-interval-ms")
        .scan<'i', int>().default_value(500);

    try {
        program.parse_args(argc, argv);
    } catch (const std::runtime_error& err) {
        std::cerr << err.what() << "\n" << program;
        return false;
    }

    out.interface = program.get<std::string>("--interface");
    out.slave_index = program.get<int>("--slave");
    out.duration = program.get<double>("--duration");
    out.enable_dc_sync = program.get<bool>("--dc-sync");
    out.connection_id = program.get<unsigned int>("--connection-id");
    out.watchdog_ms = program.get<int>("--watchdog-ms");
    out.diag_interval_ms = program.get<int>("--diag-interval-ms");
    return true;
}

} // namespace

// ============================================================================
// Main
// ============================================================================

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
        "synapticon_fsoe_only — interface=%s slave=%u duration=%.1f "
        "dc_sync=%s conn_id=0x%04X watchdog=%u ms",
        args.interface.c_str(), slave_idx, args.duration,
        args.enable_dc_sync ? "on" : "off",
        args.connection_id, args.watchdog_ms);

    // --- Start EtherCAT master ---
    EtherCAT::DS402Master master;
    Tether::Examples::HostMasterSession session;
    if (!Tether::Examples::startHostMasterSession(args.interface, master, session, TAG)) {
        return 2;
    }

    // --- Configure mailbox with SOMANET ESI values ---
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

        auto& slave = master.ethercatMaster().slave(slave_idx);
        const auto mb_err = slave.configureMailbox(
            {.address = kMailboxReadAddr,  .length = kMailboxReadSize},
            {.address = kMailboxWriteAddr, .length = kMailboxWriteSize},
            kMailboxProtocols);
        if (mb_err != EtherCAT::SlaveError::Ok) {
            TETHER_LOGE(TAG, "Failed to configure mailbox: %s",
                        EtherCAT::slaveErrorToString(mb_err));
            Tether::Examples::stopHostMasterSession(master, session);
            return 2;
        }
        TETHER_LOGI(TAG,
            "Mailbox configured: SM0(M->S)=0x%04X/%u SM1(S->M)=0x%04X/%u proto=0x%04X",
            kMailboxWriteAddr, kMailboxWriteSize,
            kMailboxReadAddr, kMailboxReadSize, kMailboxProtocols);

        // Transition to PRE_OP before any SDO exchange
        const auto pre_err = slave.transitionToPreOp();
        if (pre_err != EtherCAT::SlaveError::Ok) {
            TETHER_LOGE(TAG, "Failed to transition to PRE_OP: %s",
                        EtherCAT::slaveErrorToString(pre_err));
            Tether::Examples::stopHostMasterSession(master, session);
            return 2;
        }
        TETHER_LOGI(TAG, "Slave %u transitioned to PRE_OP", slave_idx);
    }

    // --- Configure FSoE-only PDO mapping and transition to OP ---
    //
    // Only FSoE safety PDOs are mapped — no CiA 402 process data PDOs.
    // This uses CiA402Drive::transitionToOp(const Slave::MultiPDOAssignment&)
    // which internally:
    //   1. Ensures PRE_OP (skips if already there)
    //   2. Calls Slave::configureMultiPDOs() — writes SM registers, PDO
    //      assignments (0x1C12/0x1C13), and FMMU configuration
    //   3. Sets drive PDO buffer sizes from the assignment's total Rx/Tx sizes
    //   4. Registers PDO buffers with the process data transport
    //   5. Transitions SAFE_OP → OP (with DC reconfig, diagnostics, etc.)
    //
    // PDO layout:
    //   SM2 (outputs, 0x1800, ctrl=0x64): RxPDO 0x1700 (11 bytes)
    //     FSoE Command, STO/SS1/SS2/SOS/SLS/SBC/ResetPos flags, CRCs, ConnectionID
    //   SM3 (inputs, 0x1C00, ctrl=0x20): TxPDO 0x1B00 (31 bytes)
    //     FSoE Command, safety state flags, diagnostic flags, safe pos/vel, CRCs, ConnectionID
    {
        auto* drive = master.driveBySlaveIndex(slave_idx);
        if (drive == nullptr) {
            TETHER_LOGE(TAG, "Drive %u not found", slave_idx);
            Tether::Examples::stopHostMasterSession(master, session);
            return 3;
        }

        drive->setSDOTimeout(kSdoTimeoutMs);

        // Build the FSoE-only PDO assignment
        const auto assignment = EtherCAT::Drives::Synapticon_pdo::makeFSoEPDOAssignment();

        TETHER_LOGI(TAG,
            "Transitioning to OP with FSoE-only PDO mapping: "
            "SM2=RxPDO 0x1700 (%u bytes), SM3=TxPDO 0x1B00 (%u bytes)",
            EtherCAT::Drives::Synapticon_pdo::RxPDO_1700.size,
            EtherCAT::Drives::Synapticon_pdo::TxPDO_1B00.size);

        if (!drive->transitionToOp(assignment)) {
            TETHER_LOGE(TAG, "Failed to transition to OP with multi-PDO assignment");
            Tether::Examples::stopHostMasterSession(master, session);
            return 4;
        }
        TETHER_LOGI(TAG, "Slave %u transitioned to OP with FSoE-only PDOs", slave_idx);
    }

    // --- Set up FSoE safe-motion ---
    std::unique_ptr<FSoEMain> fsoe_main;
    std::unique_ptr<FSoEServo> fsoe_servo;

    {
        EtherCAT::Drives::Synapticon::SafeMotion::MainConfig main_config;
        main_config.feature_enabled = true;
        main_config.slave_address = slave_idx;
        main_config.safety_address = 0x0001;
        main_config.connection_id = static_cast<uint16_t>(args.connection_id);
        main_config.master_address = 0x0001;
        main_config.watchdog_time_ms = static_cast<uint16_t>(args.watchdog_ms);

        EtherCAT::Drives::Synapticon::SafeMotion::ServoEmulatorConfig servo_config;
        servo_config.slave_address = slave_idx;
        servo_config.connection_id = static_cast<uint16_t>(args.connection_id);
        servo_config.safety_address = 0x0001;
        servo_config.watchdog_time_ms = static_cast<uint16_t>(args.watchdog_ms);

        fsoe_main = std::make_unique<FSoEMain>(main_config);
        fsoe_servo = std::make_unique<FSoEServo>(servo_config);

        if (!fsoe_main->initialize() || !fsoe_servo->initialize()) {
            TETHER_LOGE(TAG, "FSoE initialization failed");
            Tether::Examples::stopHostMasterSession(master, session);
            return 5;
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

        TETHER_LOGI(TAG,
            "FSoE initialized: conn_id=0x%04X watchdog=%u ms",
            args.connection_id, args.watchdog_ms);
    }

    // --- Add FSoE cyclic tasks ---
    int rc = 0;

    if (!master.addCyclicTask(
            std::make_unique<FSoEExchangeTask>(slave_idx, *fsoe_main, *fsoe_servo))) {
        TETHER_LOGE(TAG, "Failed to add FSoE exchange task");
        rc = 6;
        Tether::Examples::stopHostMasterSession(master, session);
        return rc;
    }

    if (!master.addCyclicTask(
            std::make_unique<FSoEDiagnosticsTask>(
                slave_idx, *fsoe_main, static_cast<uint32_t>(args.diag_interval_ms)))) {
        TETHER_LOGW(TAG, "Failed to add FSoE diagnostics task (non-fatal)");
    }

    // --- Start realtime motion loop ---
    EtherCAT::Master::RealtimeMotionLoopConfig loop_config;
    loop_config.cycle_period_us = 1000;
    loop_config.sync_interval_cycles = 10;
    loop_config.enable_dc_synchronization = args.enable_dc_sync;
    if (!master.startRealtimeMotionControlLoop(loop_config)) {
        TETHER_LOGE(TAG, "Failed to start realtime motion loop");
        rc = 7;
        master.clearCyclicTasks();
        Tether::Examples::stopHostMasterSession(master, session);
        return rc;
    }

    TETHER_LOGI(TAG,
        "FSoE-only mode active: exchanging safety PDOs (0x1700/0x1B00) "
        "for %.1f s", args.duration);

    Tether::Platform::Clock::instance().delayMilliseconds(
        static_cast<uint32_t>(args.duration * 1000.0));

    // --- Stop and clean up ---
    master.stopMotionControlLoop();
    master.clearCyclicTasks();

    // Final FSoE diagnostics dump
    if (fsoe_main) {
        TETHER_LOGI(TAG, "=== Final FSoE Diagnostics ===");
        TETHER_LOGI(TAG, "%s", fsoe_main->rawConnection().getDiagnostics().c_str());
    }

    // Print final FSoE PDO buffer contents
    {
        auto* drive = master.driveBySlaveIndex(slave_idx);
        if (drive != nullptr) {
            const auto* tx = drive->txPDO<FSoETxPDO>();
            const auto* rx = drive->rxPDO<FSoERxPDO>();
            if (tx != nullptr) {
                TETHER_LOGI(TAG,
                    "Final TxPDO 0x1B00: cmd=0x%02X sto_state=%d sos_state=%d "
                    "error_state=%d safe_pos=%d safe_vel=%d conn_id=0x%04X",
                    tx->fsoe_command,
                    (tx->safety_state_flags & FSoETxPDO::kSTOState) ? 1 : 0,
                    (tx->safety_state_flags & FSoETxPDO::kSOSState) ? 1 : 0,
                    (tx->safety_state_flags & FSoETxPDO::kErrorState) ? 1 : 0,
                    static_cast<int16_t>(tx->safe_position_actual),
                    static_cast<int16_t>(tx->safe_velocity_actual),
                    tx->fsoe_connection_id);
            }
            if (rx != nullptr) {
                TETHER_LOGI(TAG,
                    "Final RxPDO 0x1700: cmd=0x%02X sto=%d ss1=%d ss2=%d "
                    "sos=%d conn_id=0x%04X",
                    rx->fsoe_command,
                    (rx->safety_flags & FSoERxPDO::kSTO) ? 1 : 0,
                    (rx->safety_flags & FSoERxPDO::kSS1) ? 1 : 0,
                    (rx->safety_flags & FSoERxPDO::kSS2) ? 1 : 0,
                    (rx->safety_flags & FSoERxPDO::kSOS) ? 1 : 0,
                    rx->fsoe_connection_id);
            }
        }
    }

    Tether::Examples::stopHostMasterSession(master, session);
    return rc;
}
