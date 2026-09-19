/**
 * @file synapticon_cst.cpp
 * @brief Synapticon SOMANET drive  -  CST mode example (no FSoE)
 *
 * Interfaces to a Synapticon SOMANET drive (Vendor 0x22D2, CiA 402 firmware
 * v5.1.x), puts it into Cyclic Sync Torque (CST) mode, maps the SOMANET
 * motion PDOs (RxPDO 0x1600 / TxPDO 0x1A00), and sends a sinusoidal torque
 * command (default 0.2 Nm peak-to-peak at 0.5 Hz).
 *
 * No FSoE safe-motion protocol is run.  If the drive's safety module is
 * holding STO, torque output is gated at the drive level regardless of the
 * commanded value  -  use synapticon_cst_fsoe to clear the safe state via
 * FSoE.
 *
 * On startup the slave is automatically reset to INIT if it is currently
 * in a higher ESM state (e.g. left over from a previous run that didn't
 * shut down cleanly).  This ensures a clean starting point for mailbox
 * configuration and PDO mapping.
 *
 * Usage (Linux, requires root or CAP_NET_RAW):
 *   ./synapticon_cst                            # auto-select iface, slave 0, 10 s
 *   ./synapticon_cst -i enx34298f762c4e         # specify interface
 *   ./synapticon_cst -s 1 -d 30                 # slave 1, 30 s
 *   ./synapticon_cst --dc-sync                  # enable DC synchronization
 *   ./synapticon_cst --debug rx-pdo,tx-pdo      # EtherCAT PDO data logging
 *   ./synapticon_cst --debug coe-reads,coe-writes  # SDO access logging
 *   ./synapticon_cst --debug al-state,pdo-sm    # state machine + SM config
 *   ./synapticon_cst --debug help               # list all available debug flags
 *   ./synapticon_cst --torque-nm 10 --freq-hz 1.0     # 10 Nm P-P at 1 Hz
 *   ./synapticon_cst --rated-torque-mnm 4200          # override rated torque
 */

#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <iostream>
#include <memory>
#include <string>

#include "DS402ExampleSupport.hpp"
#include "common/ExampleHelpers.hpp"
#include "tether/drives/Synapticon.hpp"
#include "tether/drives/Synapticon/SynapticonPDO.hpp"
#include "tether/drives/Synapticon/SafetyDiagnostics.hpp"
#include "tether/drives/Synapticon/BrakeControl.hpp"
#include "tether/utils/SignalHandler.hpp"
#include "tether/ethercat/ALResetController.hpp"
#include "tether/ethercat/FaultDetection.hpp"
#include "tether/ethercat/Slave.hpp"
#include "tether/profiles/cia402/CiA402BitLabels.hpp"
#include "tether/platform/EspCompat.hpp"
#include "tether/profiles/cia301/CiA402Defs.hpp"
#include "tether/profiles/cia402/DS402Master.hpp"
#include "tether/ethercat/CoEManager.hpp"

#include <argparse/argparse.hpp>

namespace {

constexpr const char* TAG = "synapticon_cst";

// Format a failed CoE transaction for a one-line log message:
//   abort   -> "Object does not exist (0x06020000)"
//   other   -> "Timeout" / "Transport error" / ...
// coeErrorStr() already decodes the slave's SDO abort code, so the caller
// only needs this to append the raw code for aborts.
static std::string coeErrorDetail(const EtherCAT::CoE::CoEError& e) {
    std::string s = EtherCAT::CoE::coeErrorStr(e);
    if (e.code == EtherCAT::CoE::CoEErrorCode::Aborted && e.abort_code != 0) {
        char buf[20];
        std::snprintf(buf, sizeof(buf), " (0x%08X)", e.abort_code);
        s += buf;
    }
    return s;
}

// ============================================================================
// Mailbox settings  -  from SOMANET_CiA_402_v5.1.9.xml (ESI)
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

// SM0  -  master→slave write mailbox (ESI "MBoxOut", ControlByte 0x26)
constexpr uint16_t kMailboxWriteAddr = EtherCAT::Drives::Synapticon::kMailboxWriteAddr;
// NOTE: The SOMANET ESI advertises 1024-byte mailbox buffers, but the
// firmware only accepts 512 bytes.  The driver header now defines
// kMailboxWriteSize/kMailboxReadSize as 512 to match the firmware.
constexpr uint16_t kMailboxWriteSize = EtherCAT::Drives::Synapticon::kMailboxWriteSize;

// SM1  -  slave→master read mailbox (ESI "MBoxIn", ControlByte 0x22)
constexpr uint16_t kMailboxReadAddr = EtherCAT::Drives::Synapticon::kMailboxReadAddr;
constexpr uint16_t kMailboxReadSize = EtherCAT::Drives::Synapticon::kMailboxReadSize;

// CoE | FoE
constexpr uint16_t kMailboxProtocols = EtherCAT::Drives::Synapticon::kMailboxProtocols;

// SDO response timeout from ESI ResponseTimeout (6000 ms)
constexpr uint32_t kSdoTimeoutMs = EtherCAT::Drives::Synapticon::kSdoTimeoutMs;

// SOMANET PDO types  -  from SynapticonPDO.hpp (extracted from ESI)
//
// CiA 402 motion PDOs (CST mode):
//   RxPDO 0x1600: controlword, modes_of_operation, target_torque,
//                 target_position, target_velocity, torque_offset,
//                 tuning_command  (19 bytes)
//   TxPDO 0x1A00: statusword, modes_of_operation_display, position_actual,
//                 velocity_actual, torque_actual  (13 bytes)
using RxPDO = EtherCAT::Drives::SynapticonPDO::SOMANET_RxPDO_1600;
using TxPDO = EtherCAT::Drives::SynapticonPDO::SOMANET_TxPDO_1A00;

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
// NOTE: without FSoE there is no mechanism to clear the drive's safe state.
// If the safety module holds STO, the drive inhibits torque regardless of
// the commanded value.

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
        return drive.setOperatingMode(static_cast<int8_t>(CiA402::OperatingMode::CyclicSyncTorque));
    }

    void stop(EtherCAT::CiA402Drive&) override {}

    bool update(EtherCAT::CiA402Drive& drive, double dt_seconds) override {
        // Motion PDO is the only PDO mapped  -  it starts at offset 0.
        auto* rx = static_cast<PDO*>(drive.getRxPDOBuffer());
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

        rx->controlword = static_cast<uint16_t>(CiA402::ControlWord::EnableOperation);
        rx->modes_of_operation = static_cast<int8_t>(CiA402::OperatingMode::CyclicSyncTorque);

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
// Drive diagnostics cyclic task
// ============================================================================
//
// Periodically prints the drive's statusword, operating-mode display, and
// actual torque/position read back from the TxPDO.

class DriveDiagnosticsTask final : public EtherCAT::DS402Master::ICyclicTask {
public:
    DriveDiagnosticsTask(uint16_t slave_index, uint32_t interval_ms)
        : slave_index_(slave_index)
        , interval_ms_(interval_ms)
    {
    }

    bool update(EtherCAT::DS402Master& master, double dt_seconds) override {
        elapsed_ms_ += static_cast<uint64_t>(dt_seconds * 1000.0);
        if (elapsed_ms_ - last_print_ms_ < interval_ms_) return true;
        last_print_ms_ = elapsed_ms_;

        auto* drive = master.driveBySlaveIndex(slave_index_);
        if (drive == nullptr) return true;

        auto* tx = static_cast<const TxPDO*>(drive->getTxPDOBuffer());
        auto* rx = static_cast<const RxPDO*>(drive->getRxPDOBuffer());
        if (tx) {
            TETHER_LOGI(TAG,
                "--- Drive @ {} ms ---",
                static_cast<unsigned long long>(elapsed_ms_));
            TETHER_LOGI(TAG,
                "  statusword=0x{:04X} [{}] mode_display={} "
                "target_torque={} torque_actual={} "
                "position_actual={}",
                tx->statusword,
                CiA402::kStatuswordFormatter.formatActive(tx->statusword),
                static_cast<int>(tx->modes_of_operation_display),
                rx ? static_cast<int>(rx->target_torque) : 0,
                static_cast<int>(tx->torque_actual),
                static_cast<long long>(tx->position_actual));
        }

        return true;
    }

private:
    uint16_t slave_index_;
    uint32_t interval_ms_;
    uint64_t elapsed_ms_ = 0;
    uint64_t last_print_ms_ = 0;
};

// ============================================================================
// Default PDO / SM / FMMU diagnostic dump
// ============================================================================
//
// Reads the drive's default PDO assignment (0x1C12/0x1C13), SyncManager
// registers (0x0810–0x081F), and FMMU registers (0x0600–0x063F) via SDO
// and register access.  This shows the drive's power-on defaults before
// the master overwrites them  -  useful for understanding what the drive
// expects for the PDO configuration.
//
// Call this right after PRE_OP transition, before any PDO configuration.

static void dumpDefaultPdoConfig(EtherCAT::Master& ec_master,
                                  uint16_t slave_idx,
                                  EtherCAT::CoE::CoEManager& sdo) {
    constexpr uint32_t kDumpSdoTimeoutMs = 1000;

    if (!ec_master.debugFlags().isEnabled("pdo-configuration", slave_idx)) {
        return;
    }

    TETHER_LOGI(TAG, "===== Default PDO/SM/FMMU dump (slave {}) =====", slave_idx);

    // --- 0x1C12 (SM2 RxPDO assignment) ---
    {
        auto cnt = sdo.readU8(0x1C12, 0, {.timeout_ms = kDumpSdoTimeoutMs});
        if (cnt.has_value()) {
            TETHER_LOGI(TAG, "  0x1C12 (SM2 RxPDO assign): {} entries", cnt.value());
            for (uint8_t i = 1; i <= cnt.value() && i <= 16; i++) {
                auto entry = sdo.readU16(0x1C12, i, {.timeout_ms = kDumpSdoTimeoutMs});
                if (entry.has_value())
                    TETHER_LOGI(TAG, "    0x1C12:{} = 0x{:04X}", i, entry.value());
                else
                    TETHER_LOGW(TAG, "    0x1C12:{} FAILED ({})", i, coeErrorDetail(entry.error()));
            }
        } else {
            TETHER_LOGW(TAG, "  0x1C12:0 FAILED ({}) — no default RxPDO assignment", coeErrorDetail(cnt.error()));
        }
    }

    // --- 0x1C13 (SM3 TxPDO assignment) ---
    {
        auto cnt = sdo.readU8(0x1C13, 0, {.timeout_ms = kDumpSdoTimeoutMs});
        if (cnt.has_value()) {
            TETHER_LOGI(TAG, "  0x1C13 (SM3 TxPDO assign): {} entries", cnt.value());
            for (uint8_t i = 1; i <= cnt.value() && i <= 16; i++) {
                auto entry = sdo.readU16(0x1C13, i, {.timeout_ms = kDumpSdoTimeoutMs});
                if (entry.has_value())
                    TETHER_LOGI(TAG, "    0x1C13:{} = 0x{:04X}", i, entry.value());
                else
                    TETHER_LOGW(TAG, "    0x1C13:{} FAILED ({})", i, coeErrorDetail(entry.error()));
            }
        } else {
            TETHER_LOGW(TAG, "  0x1C13:0 FAILED ({}) — no default TxPDO assignment", coeErrorDetail(cnt.error()));
        }
    }

    // --- SyncManager registers ---
    // SM0: 0x0800-0x0807, SM1: 0x0808-0x080F, SM2: 0x0810-0x0817, SM3: 0x0818-0x081F
    const auto slave_addr = EtherCAT::Master::slaveAddressFromADP(
        EtherCAT::Master::adpForSlaveIndex(slave_idx));
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

        TETHER_LOGI(TAG, "  {}: start=0x{:04X} len={} ctrl=0x{:02X} status=0x{:02X} act=0x{:02X} pdi=0x{:02X}",
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
            TETHER_LOGI(TAG, "  FMMU{}: log=0x{:08X} phys=0x{:04X} len={} type=0x{:02X} [{} {}]",
                        fmmu, log_addr, phys_addr, length, fmmu_type,
                        enabled ? "EN" : "DIS", dir);
        } else {
            TETHER_LOGI(TAG, "  FMMU{}: (unused)", fmmu);
        }
    }

    // --- Read individual PDO mapping objects for the assigned PDOs ---
    // For each PDO index in 0x1C12/0x1C13, read 0x1A00:0 etc. to get the
    // mapping entry count and total size.
    auto dump_pdo_mapping = [&](uint16_t pdo_idx, const char* label) {
        auto cnt = sdo.readU8(pdo_idx, 0, {.timeout_ms = kDumpSdoTimeoutMs});
        if (!cnt.has_value()) {
            TETHER_LOGW(TAG, "  {} (0x{:04X}): FAILED to read count ({})", label, pdo_idx, coeErrorDetail(cnt.error()));
            return;
        }
        uint16_t total_bits = 0;
        TETHER_LOGI(TAG, "  {} (0x{:04X}): {} entries", label, pdo_idx, cnt.value());
        for (uint8_t i = 1; i <= cnt.value() && i <= 32; i++) {
            auto entry = sdo.readU32(pdo_idx, i, {.timeout_ms = kDumpSdoTimeoutMs});
            if (entry.has_value()) {
                uint32_t v = entry.value();
                uint16_t idx = (v >> 16) & 0xFFFF;
                uint8_t sub = (v >> 8) & 0xFF;
                uint8_t bits = v & 0xFF;
                total_bits += bits;
                TETHER_LOGI(TAG, "    {}:{} = 0x{:08X} (idx=0x{:04X} sub={} bits={})",
                            label, i, v, idx, sub, bits);
            } else {
                TETHER_LOGW(TAG, "    {}:{} FAILED ({})", label, i, coeErrorDetail(entry.error()));
            }
        }
        TETHER_LOGI(TAG, "    → total: {} bits = {} bytes", total_bits, total_bits / 8);
    };

    // Read mappings for all relevant PDOs
    dump_pdo_mapping(0x1600, "RxPDO_1600");
    dump_pdo_mapping(0x1601, "RxPDO_1601");
    dump_pdo_mapping(0x1602, "RxPDO_1602");
    dump_pdo_mapping(0x1A00, "TxPDO_1A00");
    dump_pdo_mapping(0x1A01, "TxPDO_1A01");
    dump_pdo_mapping(0x1A02, "TxPDO_1A02");
    dump_pdo_mapping(0x1A03, "TxPDO_1A03");

    TETHER_LOGI(TAG, "===== End default PDO/SM/FMMU dump =====");
}

// ============================================================================
// Main
// ============================================================================

struct Args {
    std::string interface;
    int slave_index = 0;
    double duration = 10.0;
    bool enable_dc_sync = false;
    uint32_t diag_interval_ms = 1000;
    std::string debug;
    double torque_pp_nm = 0.2;       ///< Peak-to-peak torque amplitude in Nm
    double freq_hz = 0.5;            ///< Sine wave frequency in Hz
    uint32_t rated_torque_mnm = 0;   ///< Motor rated torque in mNm (0 = auto-detect from 0x6076)
    /// Phase-1 stop strategy for the controlled shutdown at exit.
    EtherCAT::CiA402Drive::ShutdownStopMode stop_mode =
        EtherCAT::CiA402Drive::ShutdownStopMode::FixedTimeVelocity;
    uint32_t stop_time_ms = 500;     ///< Ramp duration for fixed-time stop
    double   decel_limit = 0.0;      ///< |dv/dt| limit for decel-limit stop (units/s)
    uint32_t standstill_ms = 25;     ///< Zero-velocity hold before disabling
    /// When true, release (disengage) the brake after the drive is
    /// disabled.  Default keeps the brake engaged once stopped — the drive
    /// auto-engages it on disable, and phase 3 only ensures that state.
    bool     release_brake = false;
    Tether::Examples::VlanConfig vlan;
};

bool parseArgs(int argc, char** argv, Args& out) {
    argparse::ArgumentParser program("synapticon_cst", "1.0", argparse::default_arguments::help);
    Tether::Examples::addInterfaceArg(program);
    program.add_argument("-s", "--slave")
        .scan<'i', int>()
        .default_value(0)
        .help("Slave index on the bus (0-based)");
    program.add_argument("-d", "--duration")
        .scan<'g', double>()
        .default_value(10.0)
        .help("Duration in seconds");
    program.add_argument("--dc-sync")
        .default_value(false)
        .implicit_value(true)
        .help("Enable EtherCAT distributed-clock synchronization (off by default)");
    program.add_argument("--diag-interval-ms")
        .scan<'i', int>()
        .default_value(1000)
        .help("Diagnostics print interval in ms");
    program.add_argument("--debug")
        .default_value(std::string(""))
        .help("Comma-separated debug flags. Use '--debug help' for a list.");
    program.add_argument("--torque-nm")
        .scan<'g', double>()
        .default_value(0.2)
        .help("Peak-to-peak sine torque amplitude in Nm (default 0.2 = ±0.1 Nm)");
    program.add_argument("--freq-hz")
        .scan<'g', double>()
        .default_value(0.5)
        .help("Sine torque frequency in Hz (default 0.5)");
    program.add_argument("--rated-torque-mnm")
        .scan<'i', int>()
        .default_value(static_cast<int>(0))
        .help("Motor rated torque in mNm (0 = auto-detect from object 0x6076)");
    program.add_argument("--stop-mode")
        .choices("fixed-time", "instant", "decel-limit", "none")
        .default_value(std::string("fixed-time"))
        .help("Controlled-shutdown stop strategy: 'fixed-time' = linear CSV ramp "
              "to 0 over --stop-time-ms; 'instant' = CSV zero-velocity step; "
              "'decel-limit' = CSV ramp at --decel-limit units/s; "
              "'none' = disable immediately (default fixed-time)");
    program.add_argument("--stop-time-ms")
        .scan<'i', int>()
        .default_value(500)
        .help("Fixed-time stop ramp duration in ms (default 500)");
    program.add_argument("--decel-limit")
        .scan<'g', double>()
        .default_value(0.0)
        .help("Deceleration limit for --stop-mode=decel-limit in velocity "
              "units/s (must be > 0)");
    program.add_argument("--standstill-ms")
        .scan<'i', int>()
        .default_value(25)
        .help("Zero-velocity standstill hold before disabling, ms (default 25)");
    program.add_argument("--release-brake")
        .default_value(false)
        .implicit_value(true)
        .help("Release (disengage) the holding brake after disabling — by "
              "default phase 3 ensures the brake stays ENGAGED once stopped");

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
    out.diag_interval_ms = static_cast<uint32_t>(program.get<int>("--diag-interval-ms"));
    out.debug = program.get<std::string>("--debug");
    out.torque_pp_nm = program.get<double>("--torque-nm");
    out.freq_hz = program.get<double>("--freq-hz");
    out.rated_torque_mnm = static_cast<uint32_t>(program.get<int>("--rated-torque-mnm"));

    const std::string stop_mode = program.get<std::string>("--stop-mode");
    using StopMode = EtherCAT::CiA402Drive::ShutdownStopMode;
    if (stop_mode == "instant")           out.stop_mode = StopMode::InstantHardStop;
    else if (stop_mode == "decel-limit")  out.stop_mode = StopMode::AccelerationLimited;
    else if (stop_mode == "none")         out.stop_mode = StopMode::None;
    else                                  out.stop_mode = StopMode::FixedTimeVelocity;
    out.stop_time_ms =
        static_cast<uint32_t>(program.get<int>("--stop-time-ms"));
    out.decel_limit   = program.get<double>("--decel-limit");
    out.standstill_ms =
        static_cast<uint32_t>(program.get<int>("--standstill-ms"));
    out.release_brake = program.get<bool>("--release-brake");
    if (out.stop_mode == StopMode::AccelerationLimited && out.decel_limit <= 0.0) {
        std::cerr << "--stop-mode=decel-limit requires --decel-limit > 0\n";
        return false;
    }
    if (!Tether::Examples::parseVlanArgs(
            program.get<std::string>("--rx-vlan"),
            program.get<std::string>("--tx-vlan"),
            out.vlan, TAG)) {
        return false;
    }
    Tether::Examples::logVlanConfig(out.vlan, TAG);
    return true;
}

// ----------------------------------------------------------------------------
// Pre-activation safety check (currently disabled  -  see call site in main)
// ----------------------------------------------------------------------------
// Reads 0x2611 (Safety Module input diagnostics) and 0x2620:2 ("Safe fieldbus"
// FSoE active indicator) from the drive via SDO, then decides whether to
// proceed with activation.
//
// Without FSoE there is no mechanism to bring the drive out of a safe state,
// so if the safety module reports a safe state we abort and shut down.
//
// Returns 0 on success (proceed with activation), non-zero on failure
// (caller should abort and shut down).
int preActivationSafetyCheck(EtherCAT::DS402Master& master,
                             Tether::Examples::HostMasterSession& session,
                             uint16_t slave_idx) {
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
        TETHER_LOGE(TAG,
            "Failed to read safety module diagnostics (0x2611) via SDO  —  "
            "cannot verify safety state, aborting activation");
        Tether::Examples::stopHostMasterSession(master, session);
        return 2;
    }

    if (safety.isInSafeState()) {
        // There is no mechanism to clear the safe state without FSoE, so
        // enabling the drive would be futile.  Abort.
        TETHER_LOGE(TAG,
            "Drive is in SAFE STATE (safety function active, motion "
            "inhibited)  —  there is no mechanism to clear the safe state "
            "without FSoE, refusing to activate drive, triggering shutdown");
        Tether::Examples::stopHostMasterSession(master, session);
        return 2;
    }

    TETHER_LOGI(TAG,
        "Safety check passed: safety module reports motion allowed");
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
        "synapticon_cst  —  interface={} slave={} duration={:.1f} dc_sync={} debug='{}' "
        "torque_pp={:.3f}Nm freq={:.3f}Hz rated_torque_mnm={}",
        args.interface.c_str(), slave_idx, args.duration,
        args.enable_dc_sync ? "on" : "off",
        args.debug.c_str(),
        args.torque_pp_nm, args.freq_hz, args.rated_torque_mnm);

    // --- Start EtherCAT master ---
    EtherCAT::DS402Master master;
    Tether::Examples::HostMasterSession session;
    if (!Tether::Examples::startHostMasterSession(args.interface, master, session, TAG, args.vlan)) {
        return 2;
    }

    // Ctrl-C / SIGTERM: request cancellation on the master (wakes SDO/mailbox
    // waiters) and set the stop flag polled by the run loop below, so the
    // program reaches the controlled-shutdown path instead of dying with the
    // drive still enabled.
    Tether::Utils::SignalHandler sig;
    sig.setCancelCallback([&master] { master.ethercatMaster().requestCancel(); });

    // --- Configure mailbox with SOMANET ESI values ---
    // The SOMANET_CiA_402_v5.1.9.xml ESI defines the mailbox sync managers
    // with 1024-byte buffers at 0x1000 (SM0, M→S write) and 0x1400 (SM1,
    // S→M read), supporting CoE + FoE.  In practice the drive firmware only
    // accepts 512-byte buffers, so kMailboxWriteSize/kMailboxReadSize are
    // overridden to 512 above.  These values are used instead of relying on
    // SII EEPROM auto-configuration so the correct mailbox geometry is
    // always used for SOMANET drives.
    {
        // --- Verify the target slave is a Synapticon SOMANET drive ---
        // The bus may carry other EtherCAT devices ahead of the drive (e.g.
        // the ESC211 safety controller), so -s must point at the SOMANET.
        // Configuring the wrong slave manifests as AL 0x001E/0x0025
        // "Invalid input/output configuration" when the device rejects the
        // SOMANET PDO assignment.
        //
        // A vendor-ID mismatch is fatal.  A product-code mismatch is only a
        // warning because Synapticon ships several product codes on the same
        // CiA 402 firmware family.  If the SII cannot be read at all (the
        // SOMANET historically rejects APWR to EEPCTL), the check degrades
        // to a warning so the run can continue on drives with unreadable
        // EEPROMs.
        constexpr uint32_t kSynapticonVendorId   = 0x000022D2;
        constexpr uint32_t kSomanetProductCode  = 0x00000302;

        const auto discovered =
            master.ethercatMaster().discovery().discover(
                {EtherCAT::DiscoveryOption::VendorId,
                 EtherCAT::DiscoveryOption::ProductCode,
                 EtherCAT::DiscoveryOption::DeviceNames});
        if (discovered.empty()) {
            TETHER_LOGE(TAG, "No slaves discovered");
            Tether::Examples::stopHostMasterSession(master, session);
            return 2;
        }
        for (const auto& s : discovered) {
            const std::string name = s.device_name.value_or("<unreadable>");
            TETHER_LOGI(TAG,
                "  slave {}: vendor=0x{:08X} product=0x{:08X} name='{}'",
                s.index,
                s.vendor_id.value_or(0),
                s.product_code.value_or(0),
                name.c_str());
        }
        if (slave_idx >= discovered.size()) {
            TETHER_LOGE(TAG,
                "Slave index {} out of range  -  only {} slave(s) on the bus",
                slave_idx, discovered.size());
            Tether::Examples::stopHostMasterSession(master, session);
            return 2;
        }
        if (!master.waitForDriveCount(
                static_cast<uint16_t>(slave_idx + 1), 2000)) {
            TETHER_LOGE(TAG, "Timed out waiting for slave {}", slave_idx);
            Tether::Examples::stopHostMasterSession(master, session);
            return 2;
        }

        const auto& target = discovered[slave_idx];
        const std::string target_name_str =
            target.device_name.value_or("<unreadable>");
        const char* target_name = target_name_str.c_str();
        if (target.vendor_id.has_value() &&
            !target.hasVendorId(kSynapticonVendorId)) {
            TETHER_LOGE(TAG,
                "Slave {} is not a Synapticon drive: vendor=0x{:08X} "
                "(expected 0x{:08X}) product=0x{:08X} name='{}'.  "
                "Pass -s with the correct bus position (see list above).",
                slave_idx, *target.vendor_id, kSynapticonVendorId,
                target.product_code.value_or(0), target_name);
            Tether::Examples::stopHostMasterSession(master, session);
            return 2;
        }
        if (!target.vendor_id.has_value()) {
            TETHER_LOGW(TAG,
                "Slave {}: could not read vendor ID from SII (name='{}')  -  "
                "cannot verify this is a Synapticon drive, continuing anyway",
                slave_idx, target_name);
        } else if (!target.hasProductCode(kSomanetProductCode)) {
            TETHER_LOGW(TAG,
                "Slave {}: Synapticon vendor OK but product=0x{:08X} "
                "(expected 0x{:08X}, name='{}')  -  continuing anyway",
                slave_idx, target.product_code.value_or(0),
                kSomanetProductCode, target_name);
        } else {
            TETHER_LOGI(TAG,
                "Slave {}: verified Synapticon drive '{}' "
                "(vendor=0x{:08X} product=0x{:08X})",
                slave_idx, target_name,
                *target.vendor_id, *target.product_code);
        }

        // --- Apply EtherCAT framework debug flags ---
        // The --debug string is parsed by the shared ExampleHelpers parser,
        // which supports per-slave filter syntax.
        {
            const auto debug_flags =
                Tether::Examples::parseDebugFlags(args.debug);
            Tether::Examples::applyDebugFlags(
                debug_flags, master.ethercatMaster(), TAG);
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
                    "Slave {} current AL state: 0x{:02X} ({})",
                    slave_idx, current_state,
                    EtherCAT::getECStateName(static_cast<EtherCAT::ECState>(current_state)));

                if (current_state != static_cast<uint8_t>(EtherCAT::ECState::Init)) {
                    TETHER_LOGI(TAG,
                        "Slave {} is not in INIT (0x{:02X})  —  resetting to INIT "
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
                                    "  Slave {} reset iter {}/{}: "
                                    "AL_STATUS=0x{:04X} (state={}, error={})",
                                    si, iter, max_iter, al,
                                    state_name, has_err ? "true" : "false");
                                if (code != 0) {
                                    TETHER_LOGI(TAG,
                                        "    AL_STATUS_CODE=0x{:04X} ({})",
                                        code, EtherCAT::getALStatusCodeName(code));
                                }
                            }
                        });

                    const auto result = reset_ctrl.resetSlave(
                        slave_idx, static_cast<uint8_t>(EtherCAT::ECState::Init));

                    if (result.success) {
                        TETHER_LOGI(TAG,
                            "Slave {} reset to INIT OK ({}, {} iterations)",
                            slave_idx, result.message.c_str(),
                            result.iterations_used);
                    } else {
                        TETHER_LOGE(TAG,
                            "Slave {} reset to INIT FAILED ({}, {} iterations, "
                            "final AL_STATUS=0x{:04X}, AL_STATUS_CODE=0x{:04X})",
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
                    "Could not read AL state for slave {}  —  continuing anyway",
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
            TETHER_LOGE(TAG, "Failed to configure mailbox: {}",
                        EtherCAT::slaveErrorToString(mb_err));
            Tether::Examples::stopHostMasterSession(master, session);
            return 2;
        }
        TETHER_LOGI(TAG,
            "Mailbox configured from SOMANET ESI: "
            "SM0(M->S)=0x{:04X}/{} SM1(S->M)=0x{:04X}/{} proto=0x{:04X}",
            kMailboxWriteAddr, kMailboxWriteSize,
            kMailboxReadAddr, kMailboxReadSize, kMailboxProtocols);

        // Transition to PRE_OP before any SDO exchange.  Mailbox communication
        // (CoE/SDO) is only valid in PRE_OP or higher  -  the slave's PDI does
        // not service the mailbox in INIT, leaving SM0 full and SM1 empty.
        const auto pre_err = slave.transitionToPreOp();
        if (pre_err != EtherCAT::SlaveError::Ok) {
            TETHER_LOGE(TAG, "Failed to transition to PRE_OP: {}",
                        EtherCAT::slaveErrorToString(pre_err));
            Tether::Examples::stopHostMasterSession(master, session);
            return 2;
        }
        TETHER_LOGI(TAG, "Slave {} transitioned to PRE_OP", slave_idx);
    }

    // --- Dump default PDO/SM/FMMU configuration ---
    // Read the drive's power-on defaults before we overwrite anything.
    {
        auto& sdo = master.ethercatMaster().sdoManager(slave_idx);
        dumpDefaultPdoConfig(master.ethercatMaster(), slave_idx, sdo);
    }

    // --- Pre-activation safety check (disabled) ---
    // The Synapticon drive does not expose 0x2611 / 0x2620:2 via SDO on this
    // firmware, causing "Object does not exist" aborts.  Without FSoE the
    // drive's safety state cannot be cleared anyway, so the operator must
    // ensure the drive is not held in a safe state (e.g. STO wiring).
    // Re-enable by uncommenting the call below.
    // {
    //     const int safety_rc = preActivationSafetyCheck(master, session, slave_idx);
    //     if (safety_rc != 0) return safety_rc;
    // }

    // --- Configure drive: CST mode, single-PDO motion mapping ---
    int rc = 0;

    // Discover slaves and initialize distributed clocks
    if (master.ethercatMaster().discovery().discover(EtherCAT::DiscoveryOptions()).empty()) {
        TETHER_LOGW(TAG, "No slaves discovered");
    }
    const uint16_t minimum_drive_count = static_cast<uint16_t>(slave_idx + 1);
    if (!master.waitForDriveCount(minimum_drive_count, 2000)) {
        TETHER_LOGE(TAG, "Timed out waiting for {} drive(s)", minimum_drive_count);
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
    // multi-PDO path below bypasses configureDrive() (which would otherwise
    // create the drive).  Without this, driveBySlaveIndex() would return
    // nullptr  -  no CiA402Drive object exists yet.
    auto& drive = master.ensureDrive(slave_idx);
    drive.setSDOTimeout(kSdoTimeoutMs);

    // Operating mode (CST) is set via the PDO buffer in the cyclic callback
    // (SineTorqueController::update writes rx->modes_of_operation).  The
    // Synapticon drive does not expose 0x6060 via SDO, so we skip the SDO
    // write entirely and rely on PDO-based mode setting.

    // Motion-only PDO configuration via the multi-PDO API.  This writes the
    // PDO assignment (0x1C12/0x1C13) and the SM2/SM3 registers explicitly
    // using the ESI physical addresses (0x1800/0x1C00).  The single-PDO
    // configureDrive() path cannot be used here: it derives SM2/SM3 from the
    // SII, and this drive's SII reports SM2 at 0x1400  -  overlapping mailbox
    // SM1 (0x1400-0x15FF)  -  which makes the slave reject PRE_OP->SAFE_OP
    // with AL 0x001E "Invalid input configuration".
    //
    // We assign the full standard ESI motion set (0x1600+0x1601+0x1602 /
    // 0x1A00+0x1A01+0x1A02+0x1A03): the SOMANET firmware rejects partial
    // assignments such as {0x1600}/{0x1A00} with AL 0x001E.
    {
        const auto assignment =
            EtherCAT::Drives::SynapticonPDO::makeStandardPDOAssignment();

        TETHER_LOGI(TAG,
            "Transitioning to OP with standard motion PDOs: "
            "SM2={} bytes (0x1600+0x1601+0x1602), "
            "SM3={} bytes (0x1A00+0x1A01+0x1A02+0x1A03)",
            35, 47);

        if (!drive.transitionToOp(assignment)) {
            TETHER_LOGE(TAG, "Failed to transition to OP with CST PDO assignment");
            master.stopDistributedClocks();
            Tether::Examples::stopHostMasterSession(master, session);
            return 3;
        }
        TETHER_LOGI(TAG, "Slave {} transitioned to OP with CST PDOs", slave_idx);
    }

    // --- Configure PDO-based operating mode offset ---
    // The modes_of_operation (0x6060) field is in the motion RxPDO (0x1600)
    // at offset 2 (after uint16_t controlword).  The motion PDO is the only
    // PDO mapped, so it starts at buffer offset 0.
    {
        // Controlword (0x6040) is at the start of the RxPDO
        drive.setControlwordPDOOffset(0);

        // Statusword (0x6041) is at the start of the TxPDO
        drive.setStatuswordPDOOffset(0);

        const size_t opmode_offset =
            offsetof(EtherCAT::Drives::SynapticonPDO::SOMANET_RxPDO_1600,
                    modes_of_operation);
        drive.setOpmodePDOOffset(static_cast<int>(opmode_offset));

        // Velocity field offsets for controlledShutdown()'s CSV stop phase —
        // computed from the PDO structs via member pointers, no hand-counted
        // offsets.  Both PDOs are first in their SM assignment, so the struct
        // offsets equal the buffer offsets.
        drive.setVelocityPDOFields(
            &EtherCAT::Drives::SynapticonPDO::SOMANET_RxPDO_1600::target_velocity,
            &EtherCAT::Drives::SynapticonPDO::SOMANET_TxPDO_1A00::velocity_actual);

        TETHER_LOGI(TAG,
            "PDO offsets: controlword=0 statusword=0 opmode={} "
            "target_velocity={} velocity_actual={}",
            opmode_offset,
            drive.targetVelocityPDOOffset(),
            drive.actualVelocityPDOOffset());
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
                "Motor rated torque (0x6076) = {} mNm ({:.3f} Nm)",
                rated_torque_mnm, rated_torque_mnm / 1000.0);
        } else {
            TETHER_LOGE(TAG,
                "Failed to read motor rated torque (0x6076) via SDO  —  "
                "cannot convert Nm to per-mille.  Use --rated-torque-mnm to "
                "specify it manually.");
            master.stopDistributedClocks();
            Tether::Examples::stopHostMasterSession(master, session);
            return 3;
        }
    } else {
        TETHER_LOGI(TAG,
            "Using --rated-torque-mnm = {} mNm ({:.3f} Nm)",
            rated_torque_mnm, rated_torque_mnm / 1000.0);
    }

    // --- Add sine torque motion controller ---
    // Amplitude is half the peak-to-peak value (±torque_pp_nm/2).
    const double amplitude_nm = args.torque_pp_nm / 2.0;
    TETHER_LOGI(TAG,
        "Sine torque: {:.3f} Nm P-P ({:.3f} Nm amplitude) at {:.3f} Hz, "
        "rated torque {} mNm -> {:.1f} per-mille peak",
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

    // --- Add drive diagnostics task ---
    if (!master.addCyclicTask(
            std::make_unique<DriveDiagnosticsTask>(
                slave_idx, args.diag_interval_ms))) {
        TETHER_LOGW(TAG, "Failed to add drive diagnostics task (non-fatal)");
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
        (void)master.removeMotionController(slave_idx);
        Tether::Examples::shutdownSingleDrive(master, slave_idx);
        Tether::Examples::stopHostMasterSession(master, session);
        return rc;
    }

    // --- Enable the drive ---
    if (!master.enableDrive(slave_idx, 5000)) {
        TETHER_LOGE(TAG, "Failed to enable slave {}", slave_idx);
        rc = 8;
    } else {
        TETHER_LOGI(TAG, "Slave {} drive enabled", slave_idx);
    }

    // --- Run loop ---
    if (rc == 0) {
        TETHER_LOGI(TAG,
            "CST mode active, sine torque {:.3f} Nm P-P at {:.3f} Hz for {:.1f} s",
            args.torque_pp_nm, args.freq_hz, args.duration);

        const auto run_start_ms = Tether::Platform::Clock::instance().getMilliseconds();
        const uint32_t run_duration_ms =
            static_cast<uint32_t>(args.duration * 1000.0);

        while (true) {
            const auto elapsed_ms =
                Tether::Platform::Clock::instance().getMilliseconds() - run_start_ms;
            if (elapsed_ms >= run_duration_ms) break;
            if (sig.stop_requested()) {
                TETHER_LOGI(TAG, "Ctrl-C received  -  stopping early");
                break;
            }
            Tether::Platform::Clock::instance().delayMilliseconds(50);
        }
    }

    // --- Controlled shutdown (ordering is load-bearing) ---
    //   1. Remove the motion controller so it stops overwriting the RxPDO
    //      buffer every cycle.
    //   2. Clear the Ctrl-C cancel flag so SDO/mailbox access (e.g. the
    //      brake release) is not short-circuited.
    //   3. controlledShutdown() runs the CSV stop -> disable -> brake
    //      release phases through the still-live PDO exchange.
    //   4. Only then stop the realtime loop.
    (void)master.removeMotionController(slave_idx);
    master.ethercatMaster().clearCancel();
    {
        EtherCAT::CiA402Drive::ControlledShutdownConfig scfg;
        scfg.stop_mode    = args.stop_mode;
        scfg.stop_time_ms = args.stop_time_ms;
        scfg.standstill_ms = args.standstill_ms;
        scfg.decel_limit  = args.decel_limit;
        // Phase 3: the SOMANET auto-engages its holding brake when the drive
        // leaves Operation Enabled, so the default action only ensures the
        // engaged state (no extra brake click).  --release-brake opts into
        // disengaging it again, e.g. for manual shaft movement.
        scfg.brake_action = [&master, slave_idx, &args] {
            auto& sdo = master.ethercatMaster().sdoManager(slave_idx);
            return args.release_brake
                ? EtherCAT::Drives::Synapticon::BrakeControl::disengageBrake(sdo)
                : EtherCAT::Drives::Synapticon::BrakeControl::engageBrake(sdo);
        };
        if (!drive.controlledShutdown(scfg)) {
            TETHER_LOGW(TAG, "Controlled shutdown reported errors");
        }
    }
    master.stopMotionControlLoop();
    master.clearCyclicTasks();

    // PDO transfer statistics  -  reveals WKC errors (slave not ack'ing frames)
    {
        const auto& pdo = master.ethercatMaster().pdo();
        const auto& st = pdo.getStats();
        TETHER_LOGI(TAG, "=== PDO Transfer Statistics ===");
        TETHER_LOGI(TAG, "  [FMMU] total_cycles={} rx_sent={} tx_recv={}",
            (unsigned long long)st.total_cycles,
            (unsigned long long)st.rxpdo_frames_sent,
            (unsigned long long)st.txpdo_frames_recv);
        TETHER_LOGI(TAG, "  [FMMU] rx_errors={} tx_errors={} wkc_errors={}",
            st.rxpdo_errors, st.txpdo_errors, st.wkc_errors);
        const auto& ps = pdo.getPhysicalStats();
        TETHER_LOGI(TAG, "  [PHYS] fpwr_ok={} fpwr_wkc_err={} fprd_ok={} fprd_wkc_err={}",
            ps.fpwr_success, ps.fpwr_wkc_errors, ps.fprd_success, ps.fprd_wkc_errors);
        TETHER_LOGI(TAG, "  [PHYS] send_err={} timeout_err={}",
            ps.send_errors, ps.timeout_errors);
    }

    Tether::Examples::shutdownSingleDrive(master, slave_idx);
    Tether::Examples::stopHostMasterSession(master, session);
    return rc;
}
