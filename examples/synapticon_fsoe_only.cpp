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
 * The FSoE master state machine (MainInstance) runs each cycle, building the
 * FSoE command frame into the RxPDO 0x1700 buffer and processing the drive's
 * safety status frame from the TxPDO 0x1B00 buffer.  The drive's safety
 * firmware handles the slave side of the FSoE protocol.
 *
 * Usage (Linux, requires root or CAP_NET_RAW):
 *   ./synapticon_fsoe_only                       # auto-select iface, slave 0, 10 s
 *   ./synapticon_fsoe_only -i enx34298f762c4e    # specify interface
 *   ./synapticon_fsoe_only -s 1 -d 30            # slave 1, 30 s
 *   ./synapticon_fsoe_only --connection-id 0x4321 --watchdog-ms 15
 *   ./synapticon_fsoe_only --debug fsoe          # high-level FSoE protocol trace
 *   ./synapticon_fsoe_only --debug fsoe-frame    # decoded FSoE PDO struct fields (on change)
 *   ./synapticon_fsoe_only --debug fsoe-raw      # protocol trace + raw frame hex dumps (on change)
 *   ./synapticon_fsoe_only --debug fsoe-wire     # every-cycle PDO wire dumps (firehose)
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
#include "tether/drives/Synapticon/SafetyDiagnostics.hpp"
#include "tether/drives/Synapticon/SynapticonPDO.hpp"
#include "tether/ethercat/Slave.hpp"
#include "tether/fsoe/FSoEDefs.hpp"
#include "tether/fsoe/FSoEHelpers.hpp"
#include "tether/fsoe/Synapticon/SafeMotionFSoE.hpp"
#include "tether/fsoe/Synapticon/FSoEPDODecoder.hpp"
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

// Reusable FSoE PDO decoding/logging helpers (from the Synapticon FSoE driver).
namespace fsoe_dbg = EtherCAT::Drives::Synapticon::FSoEDebug;

// ============================================================================
// General hex dump helper (used by the FSoE trace callback)
// ============================================================================

void hexDump(const char* tag, const char* label, const uint8_t* data, size_t len) {
    constexpr size_t kBytesPerLine = 16;
    char hex[kBytesPerLine * 3 + 1];
    for (size_t i = 0; i < len; i += kBytesPerLine) {
        size_t pos = 0;
        for (size_t j = i; j < i + kBytesPerLine && j < len; j++) {
            pos += snprintf(hex + pos, sizeof(hex) - pos, "%02X ", data[j]);
        }
        TETHER_LOGI(tag, "  {} [{:3}/{:3}]: {}", label, i, len, hex);
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

        TETHER_LOGI(TAG, "=== FSoE Diagnostics (slave {}) ===", slave_index_);
        TETHER_LOGI(TAG,
            "  state: {} (0x{:02X})  error: 0x{:04X} ({})  watchdog: {}",
            FSoE::fsoeStateName(status.state), status.state,
            status.error_code, FSoE::fsoeErrorName(status.error_code),
            status.watchdog_counter);
        if (status.hasError()) {
            TETHER_LOGW(TAG,
                "  ERROR: 0x{:04X} ({})",
                status.error_code, FSoE::fsoeErrorName(status.error_code));
        }
        TETHER_LOGI(TAG,
            "  frames: tx={} rx={} | crc_err={} seq_err={} watchdog_evt={} "
            "reset_evt={} timeout_evt={}",
            stats.frames_sent, stats.frames_received,
            stats.crc_errors, stats.sequence_errors, stats.watchdog_events,
            stats.reset_events, stats.timeout_events);

        if (fsoe_main_.hasStatus()) {
            const auto& sm = fsoe_main_.status();
            TETHER_LOGI(TAG,
                "  safe-motion: motion_allowed={} sto={} ss1={} ss2={} "
                "sos={} error={}",
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
// FSoE cyclic exchange task — real PDO communication via 0x1700/0x1B00
// ============================================================================
//
// This task runs the FSoE protocol exchange each cycle using the actual
// EtherCAT PDO buffers.  The FSoE MainInstance builds its master→slave frame
// into the RxPDO 0x1700 buffer (11 bytes), and processes the slave→master
// frame from the TxPDO 0x1B00 buffer (31 bytes).  The drive's safety
// firmware handles the slave side of the FSoE state machine.
//
// On exchange failure the task logs a warning but does NOT stop the process
// — this allows the operator to see the FSoE state machine diagnostics even
// when the drive is in safe state or not yet responding correctly.

class FSoEPDOExchangeTask final : public EtherCAT::DS402Master::ICyclicTask {
public:
    FSoEPDOExchangeTask(uint16_t slave_index,
                        FSoEMain& main_instance,
                        bool debug_raw,
                        bool debug_frame = false,
                        bool debug_wire = false)
        : slave_index_(slave_index)
        , main_instance_(main_instance)
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
            TETHER_LOGW(TAG, "Drive {} not found — skipping FSoE exchange",
                        slave_index_);
            return true;  // don't stop the process
        }

        auto* rx = drive->rxPDO<FSoERxPDO>();
        auto* tx = drive->txPDO<FSoETxPDO>();
        if (rx == nullptr || tx == nullptr) {
            TETHER_LOGW(TAG, "FSoE PDO buffers not available -- skipping exchange");
            return true;  // don't stop the process
        }

        const auto* rx_bytes = reinterpret_cast<const uint8_t*>(rx);
        const auto* tx_bytes = reinterpret_cast<const uint8_t*>(tx);

        // --debug fsoe-wire: dump every cycle, unconditionally.
        if (debug_wire_) {
            char hex[128];
            size_t pos;

            pos = 0;
            for (size_t b = 0; b < sizeof(FSoERxPDO) && pos + 3 < sizeof(hex); b++) {
                pos += static_cast<size_t>(snprintf(hex + pos, sizeof(hex) - pos, "%02X ", rx_bytes[b]));
            }
            TETHER_LOGI("fsoe-wire", "[RxPDO] cycle {}: {}", cycle_count_, hex);

            pos = 0;
            for (size_t b = 0; b < sizeof(FSoETxPDO) && pos + 3 < sizeof(hex); b++) {
                pos += static_cast<size_t>(snprintf(hex + pos, sizeof(hex) - pos, "%02X ", tx_bytes[b]));
            }
            TETHER_LOGI("fsoe-wire", "[TxPDO] cycle {}: {}", cycle_count_, hex);
            cycle_count_++;
        }

        // --debug fsoe-raw / fsoe-frame: only on frame content change
        const bool tx_changed = (debug_raw_ || debug_frame_) &&
            std::memcmp(tx_bytes, last_tx_.data(), sizeof(FSoETxPDO)) != 0;
        if (tx_changed) {
            std::memcpy(last_tx_.data(), tx_bytes, sizeof(FSoETxPDO));
        }

        // Decode the slave-to-master FSoE frame BEFORE exchangeViaPDO.
        if (debug_frame_ && tx_changed) {
            fsoe_dbg::dumpTxPDO(TAG, *tx);
        }

        // Exchange FSoE frames via the PDO buffers.
        const bool ok = main_instance_.exchangeViaPDO(
            reinterpret_cast<uint8_t*>(rx), sizeof(FSoERxPDO),
            reinterpret_cast<const uint8_t*>(tx), sizeof(FSoETxPDO),
            elapsed_time_ms_);

        const bool rx_changed = (debug_raw_ || debug_frame_) &&
            std::memcmp(rx_bytes, last_rx_.data(), sizeof(FSoERxPDO)) != 0;
        if (rx_changed) {
            std::memcpy(last_rx_.data(), rx_bytes, sizeof(FSoERxPDO));
        }

        // Decode the master-to-slave FSoE frame AFTER exchangeViaPDO.
        if (debug_frame_ && rx_changed) {
            fsoe_dbg::dumpRxPDO(TAG, *rx);
        }

        if (debug_raw_ && (tx_changed || rx_changed)) {
            const auto status = main_instance_.rawConnection().getStatus();
            TETHER_LOGI(TAG, "[fsoe-raw] t={} ms  state={}(0x{:02X})  ok={}",
                        static_cast<unsigned long>(elapsed_time_ms_),
                        FSoE::fsoeStateName(status.state), status.state,
                        ok ? 1 : 0);
            if (rx_changed) {
                hexDump(TAG, "TX->RxPDO 0x1700",
                        reinterpret_cast<const uint8_t*>(rx), sizeof(FSoERxPDO));
                TETHER_LOGI(TAG, "  TX cmd={} conn_id=0x{:04X} crc0=0x{:04X} crc1=0x{:04X}",
                            FSoE::fsoeCommandName(rx->fsoe_command),
                            rx->fsoe_connection_id,
                            rx->fsoe_crc_0, rx->fsoe_crc_1);
            }
            if (tx_changed) {
                hexDump(TAG, "RX<-TxPDO 0x1B00",
                        reinterpret_cast<const uint8_t*>(tx), sizeof(FSoETxPDO));
                TETHER_LOGI(TAG, "  RX cmd={} conn_id=0x{:04X} crc0=0x{:04X}",
                            FSoE::fsoeCommandName(tx->fsoe_command),
                            tx->fsoe_connection_id,
                            tx->fsoe_crc_0);
            }
        }

        if (!ok) {
            TETHER_LOGW(TAG, "FSoE exchange failed at t={} ms (state={}) -- continuing",
                        static_cast<unsigned long>(elapsed_time_ms_),
                        FSoE::fsoeStateName(main_instance_.rawConnection().getState()));
        }

        return true;
    }

private:
    uint16_t slave_index_;
    FSoEMain& main_instance_;
    bool debug_raw_;
    bool debug_frame_ = false;
    bool debug_wire_ = false;
    uint64_t elapsed_time_ms_ = 0;
    uint32_t cycle_count_ = 0;
    std::array<uint8_t, sizeof(FSoETxPDO)> last_tx_{};
    std::array<uint8_t, sizeof(FSoERxPDO)> last_rx_{};
};

// ============================================================================
// Argument parsing
// ============================================================================

struct Args {
    std::string interface;
    int slave_index = 0;
    double duration = 10.0;
    bool enable_dc_sync = false;
    unsigned int connection_id = 0x1234;
    int watchdog_ms = 15;
    int diag_interval_ms = 500;
    std::string debug;
};

bool parseArgs(int argc, char** argv, Args& out) {
    argparse::ArgumentParser program("synapticon_fsoe_only");
    program.add_argument("-i", "--interface")
        .default_value(std::string(""))
        .help("Network interface (e.g. eth0, enx34298f762c4e). "
              "If omitted, auto-selects the sole physical Ethernet interface.");
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
    program.add_argument("--debug")
        .default_value(std::string(""))
        .help("Comma-separated debug flags. Use '--debug help' for a list.");

    try {
        program.parse_args(argc, argv);
    } catch (const std::runtime_error& err) {
        std::cerr << err.what() << "\n" << program;
        return false;
    }

    out.interface = Tether::Examples::resolveInterface(program.get<std::string>("--interface"), TAG);
    out.slave_index = program.get<int>("--slave");
    out.duration = program.get<double>("--duration");
    out.enable_dc_sync = program.get<bool>("--dc-sync");
    out.connection_id = program.get<unsigned int>("--connection-id");
    out.watchdog_ms = program.get<int>("--watchdog-ms");
    out.diag_interval_ms = program.get<int>("--diag-interval-ms");
    out.debug = program.get<std::string>("--debug");
    return true;
}

} // namespace

// ============================================================================
// Main
// ============================================================================

int main(int argc, char** argv) {
    Args args;
    if (!parseArgs(argc, argv, args)) return 1;

    if (args.debug == "help") {
        std::cout << "Available --debug flags (comma-separated):\n"
                  << "  fsoe          High-level FSoE protocol trace\n"
                  << "  fsoe-frame    Decoded FSoE PDO struct fields (on change)\n"
                  << "  fsoe-raw      Protocol trace + raw frame hex dumps (on change)\n"
                  << "  fsoe-wire     Every-cycle PDO wire dumps (firehose)\n";
        return 0;
    }

    if (args.slave_index < 0 || args.slave_index > 65535) {
        std::cerr << "Invalid slave index\n";
        return 1;
    }
    const uint16_t slave_idx = static_cast<uint16_t>(args.slave_index);

    Tether::Platform::ensureRealtimeKernelOrExit();

    TETHER_LOGI(TAG,
        "synapticon_fsoe_only — interface={} slave={} duration={:.1f} "
        "dc_sync={} conn_id=0x{:04X} watchdog={} ms debug='{}'",
        args.interface.c_str(), slave_idx, args.duration,
        args.enable_dc_sync ? "on" : "off",
        args.connection_id, args.watchdog_ms,
        args.debug.c_str());

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
            TETHER_LOGE(TAG, "Timed out waiting for slave {}", slave_idx);
            Tether::Examples::stopHostMasterSession(master, session);
            return 2;
        }

        auto& slave = master.ethercatMaster().slave(slave_idx);
        const auto mb_err = slave.configureMailbox(
            {.address = kMailboxReadAddr,  .length = kMailboxReadSize},
            {.address = kMailboxWriteAddr, .length = kMailboxWriteSize},
            kMailboxProtocols);
        if (mb_err != EtherCAT::SlaveError::Ok) {
            TETHER_LOGE(TAG, "Failed to configure mailbox: {}",
                        EtherCAT::slaveErrorToString(mb_err));
            Tether::Examples::stopHostMasterSession(master, session);
            return 2;
        }
        TETHER_LOGI(TAG,
            "Mailbox configured: SM0(M->S)=0x{:04X}/{} SM1(S->M)=0x{:04X}/{} proto=0x{:04X}",
            kMailboxWriteAddr, kMailboxWriteSize,
            kMailboxReadAddr, kMailboxReadSize, kMailboxProtocols);

        // Transition to PRE_OP before any SDO exchange
        const auto pre_err = slave.transitionToPreOp();
        if (pre_err != EtherCAT::SlaveError::Ok) {
            TETHER_LOGE(TAG, "Failed to transition to PRE_OP: {}",
                        EtherCAT::slaveErrorToString(pre_err));
            Tether::Examples::stopHostMasterSession(master, session);
            return 2;
        }
        TETHER_LOGI(TAG, "Slave {} transitioned to PRE_OP", slave_idx);
    }

    // --- Pre-activation safety check: read 0x2611 Safety Module input diagnostics ---
    // Object 0x2611 reports the state of the safety module: 0 = safe state
    // (safety function active, torque inhibited), 1 = not safe state (motion
    // allowed).  We log this for diagnostics but do NOT abort if the drive
    // is in safe state — the FSoE connection itself will bring the drive out
    // of safe state once the safety protocol reaches the Data state.
    {
        auto& slave = master.ethercatMaster().slave(slave_idx);
        const auto safety = EtherCAT::Drives::Synapticon::readSafetyModuleState(slave);

        TETHER_LOGI(TAG,
            "Safety module diagnostics (0x2611): input1={} input2={} -> {}",
            static_cast<unsigned>(safety.input1),
            static_cast<unsigned>(safety.input2),
            safety.stateSummary());

        TETHER_LOGI(TAG,
            "FSoE active indicator (0x2620:2 \"Safe fieldbus\"): raw={} -> {}",
            static_cast<unsigned>(safety.safe_fieldbus),
            safety.fsoeStateSummary());

        if (!safety.ok) {
            TETHER_LOGW(TAG,
                "Failed to read safety module diagnostics (0x2611) via SDO — "
                "continuing anyway (FSoE will attempt to establish connection)");
        }

        // NOTE: The check that would abort activation when the drive is in
        // safe state is commented out.  For FSoE-only operation, the drive
        // starts in safe state (STO active) and the FSoE master brings it
        // out of safe state by going through the safety protocol.  Aborting
        // here would prevent the FSoE connection from ever establishing.
        //
        // if (safety.isInSafeState()) {
        //     TETHER_LOGE(TAG,
        //         "Drive is in SAFE STATE (safety function active, motion "
        //         "inhibited) — FSoE is {} (0x2620:2={}) — refusing to "
        //         "activate drive, triggering shutdown",
        //         safety.fsoeStateSummary(),
        //         static_cast<unsigned>(safety.safe_fieldbus));
        //     Tether::Examples::stopHostMasterSession(master, session);
        //     return 2;
        // }
    }

    // --- Read FSoE safety address (0xF980:1) and use as connection ID ---
    // Object 0xF980:1 contains the safety address configured on the drive.
    // The master must use this value as the FSoE connection ID so that
    // master and slave agree on the connection identifier.  Reading it
    // before starting the FSoE handshake avoids ConnectionIDError
    // failures caused by a mismatch between the --connection-id default
    // and the drive's configured value.
    {
        auto& slave = master.ethercatMaster().slave(slave_idx);
        uint16_t drive_safety_address = 0;
        const auto addr_err =
            EtherCAT::Drives::Synapticon::readFSoESafetyAddress(
                slave, drive_safety_address);
        if (addr_err == EtherCAT::SlaveError::Ok) {
            TETHER_LOGI(TAG,
                "FSoE safety address (0xF980:1): 0x{:04X} — using as "
                "connection ID (overrides --connection-id=0x{:04X})",
                drive_safety_address,
                args.connection_id);
            args.connection_id = drive_safety_address;
        } else {
            TETHER_LOGW(TAG,
                "Failed to read FSoE safety address (0xF980:1) via SDO "
                "(err={}) — falling back to --connection-id=0x{:04X}",
                static_cast<unsigned>(addr_err),
                args.connection_id);
        }
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
        // ensureDrive() creates the CiA402Drive object and marks the slave
        // as DS402-managed.  This is required because the multi-PDO path
        // below bypasses configureDrive() (which would otherwise create
        // the drive).  Without this, driveBySlaveIndex() would return
        // nullptr.
        auto& drive = master.ensureDrive(slave_idx);
        drive.setSDOTimeout(kSdoTimeoutMs);

        // Build the FSoE-only PDO assignment
        const auto assignment = EtherCAT::Drives::Synapticon_pdo::makeFSoEPDOAssignment();

        TETHER_LOGI(TAG,
            "Transitioning to OP with FSoE-only PDO mapping: "
            "SM2=RxPDO 0x1700 ({} bytes), SM3=TxPDO 0x1B00 ({} bytes)",
            EtherCAT::Drives::Synapticon_pdo::RxPDO_1700.size,
            EtherCAT::Drives::Synapticon_pdo::TxPDO_1B00.size);

        if (!drive.transitionToOp(assignment)) {
            TETHER_LOGE(TAG, "Failed to transition to OP with multi-PDO assignment");
            Tether::Examples::stopHostMasterSession(master, session);
            return 4;
        }
        TETHER_LOGI(TAG, "Slave {} transitioned to OP with FSoE-only PDOs", slave_idx);
    }

    // --- Set up FSoE safe-motion (master side only — no emulator) ---
    std::unique_ptr<FSoEMain> fsoe_main;

    // Parse --debug flags:
    //   fsoe        — high-level protocol trace
    //   fsoe-frame  — decode device-specific FSoE PDO structs into named fields
    //   fsoe-raw    — protocol trace + raw frame hex dumps
    //   fsoe-master — deprecated alias for fsoe-raw
    const bool debug_fsoe = (args.debug.find("fsoe") != std::string::npos);
    const bool debug_fsoe_frame =
        (args.debug.find("fsoe-frame") != std::string::npos);
    const bool debug_fsoe_raw =
        (args.debug.find("fsoe-raw") != std::string::npos ||
         args.debug.find("fsoe-master") != std::string::npos);
    const bool debug_fsoe_wire =
        (args.debug.find("fsoe-wire") != std::string::npos);

    {
        EtherCAT::Drives::Synapticon::SafeMotion::MainConfig main_config;
        main_config.feature_enabled = true;
        main_config.slave_address = slave_idx;
        main_config.safety_address = 0x0006;
        main_config.connection_id = static_cast<uint16_t>(args.connection_id);
        main_config.master_address = 0x0001;
        main_config.watchdog_time_ms = static_cast<uint16_t>(args.watchdog_ms);

        fsoe_main = std::make_unique<FSoEMain>(main_config);

        if (!fsoe_main->initialize()) {
            TETHER_LOGE(TAG, "FSoE initialization failed");
            Tether::Examples::stopHostMasterSession(master, session);
            return 5;
        }

        fsoe_main->requestMotionEnabled();

        // Install FSoE callbacks for real-time state tracking
        fsoe_main->rawConnection().setStateChangeCallback(
            [](uint8_t old_s, uint8_t new_s) {
                TETHER_LOGI(TAG,
                    "[FSoE] state: {} -> {}",
                    FSoE::fsoeStateName(old_s), FSoE::fsoeStateName(new_s));
            });
        fsoe_main->rawConnection().setErrorCallback(
            [](uint16_t code, const FSoE::FSoEErrorDetail& detail) {
                if (detail.message[0] != '\0') {
                    TETHER_LOGE(TAG,
                        "[FSoE] error: 0x{:04X} ({}): {}",
                        code, FSoE::fsoeErrorName(code), detail.message);
                } else {
                    TETHER_LOGE(TAG,
                        "[FSoE] error: 0x{:04X} ({})",
                        code, FSoE::fsoeErrorName(code));
                }
            });
        fsoe_main->rawConnection().setFailSafeCallback(
            []() {
                TETHER_LOGW(TAG, "[FSoE] fail-safe activated");
            });

        // High-level protocol trace (--debug fsoe or --debug fsoe-raw).
        if (debug_fsoe || debug_fsoe_raw) {
            fsoe_main->rawConnection().setTraceCallback(
                [](const char* message) {
                    TETHER_LOGI(TAG, "[fsoe] {}", message);
                });
        }

        TETHER_LOGI(TAG,
            "FSoE initialized: conn_id=0x{:04X} watchdog={} ms debug={}{}{}",
            args.connection_id, args.watchdog_ms,
            debug_fsoe ? "fsoe" : "off",
            debug_fsoe_frame ? "+frame" : "",
            debug_fsoe_raw ? "+raw" : "",
            debug_fsoe_wire ? "+wire" : "");
    }

    // --- Add FSoE cyclic tasks ---
    int rc = 0;

    if (!master.addCyclicTask(
            std::make_unique<FSoEPDOExchangeTask>(slave_idx, *fsoe_main,
                                                  debug_fsoe_raw, debug_fsoe_frame,
                                                  debug_fsoe_wire))) {
        TETHER_LOGE(TAG, "Failed to add FSoE PDO exchange task");
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
        "for {:.1f} s", args.duration);

    Tether::Platform::Clock::instance().delayMilliseconds(
        static_cast<uint32_t>(args.duration * 1000.0));

    // --- Stop and clean up ---
    master.stopMotionControlLoop();
    master.clearCyclicTasks();

    // Final FSoE diagnostics dump
    if (fsoe_main) {
        TETHER_LOGI(TAG, "=== Final FSoE Diagnostics ===");
        TETHER_LOGI(TAG, "{}", fsoe_main->rawConnection().getDiagnostics().c_str());
    }

    // Print final FSoE PDO buffer contents
    {
        auto* drive = master.driveBySlaveIndex(slave_idx);
        if (drive != nullptr) {
            const auto* tx = drive->txPDO<FSoETxPDO>();
            const auto* rx = drive->rxPDO<FSoERxPDO>();
            if (tx != nullptr) {
                TETHER_LOGI(TAG,
                    "Final TxPDO 0x1B00: cmd=0x{:02X} sto_state={} sos_state={} "
                    "error_state={} safe_pos={} safe_vel={} conn_id=0x{:04X}",
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
                    "Final RxPDO 0x1700: cmd=0x{:02X} sto={} ss1={} ss2={} "
                    "sos={} conn_id=0x{:04X}",
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
