/**
 * @file synapticon_csp_2motors.cpp
 * @brief Two Synapticon SOMANET drives, torque-limited CSP sine motion
 *        without FSoE — pure Tether, no ethercat_manager layer.
 *
 * All EtherCAT AL state transitions are issued group-wide via
 * EtherCAT::SlaveGroup + SynapticonDriveInitializer statics: INIT,
 * PRE_OP, SAFE_OP and OP each hit both drives with ONE EtherCAT packet,
 * and every AL poll reads both slaves in a single frame.  Only the
 * inherently per-slave steps run sequentially: ESC mailbox config, PDO
 * buffer registration, and the CiA 402 enable sequence (PDO controlword
 * state machine, not an AL transition).
 *
 * Motion: both motors run the identical sine position demand in CSP mode
 * starting and stopping together; motor 2 is phase-shifted by
 * --phase-deg (default 90).  Both use --max-torque-pm for 0x6072.
 *
 * No homing routine is run — the sine is centered on each drive's
 * position_actual captured right after enabling (homing method 35
 * equivalent).
 *
 * Usage:
 *   ./synapticon_csp_2motors -i enx00e04c680004            # slaves 0+1, 11 s
 *   ./synapticon_csp_2motors -i eth0 -s 0 --slave2 1 -d 20
 *   ./synapticon_csp_2motors -i eth0 --amplitude-deg 45 --freq-hz 0.25
 */

#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <iostream>
#include <memory>
#include <print>
#include <string>
#include <vector>

#include "common/ExampleHelpers.hpp"
#include "common/EtherCATHostSetup.hpp"
#include "tether/drives/Synapticon.hpp"
#include "tether/drives/Synapticon/BrakeControl.hpp"
#include "tether/drives/Synapticon/SynapticonDriveInitializer.hpp"
#include "tether/drives/Synapticon/SynapticonPDO.hpp"
#include "tether/ethercat/Master.hpp"
#include "tether/ethercat/SlaveGroup.hpp"
#include "tether/platform/Platform.hpp"
#include "tether/profiles/cia301/CiA402Defs.hpp"
#include "tether/profiles/cia402/60xx-Parameters.hpp"
#include "tether/profiles/cia402/CiA402Drive.hpp"
#include "tether/profiles/cia402/CiA402StateUtils.hpp"
#include "tether/profiles/cia402/DS402Master.hpp"
#include "tether/utils/SignalHandler.hpp"

#include <argparse/argparse.hpp>

namespace {

constexpr const char* TAG = "synapticon_csp_2motors";

namespace Syn = EtherCAT::Drives::Synapticon;
namespace SPdo = EtherCAT::Drives::SynapticonPDO;
namespace P60 = CiA402::Parameters60xx;
using Init = Syn::SynapticonDriveInitializer;

using RxPDO = SPdo::SOMANET_RxPDO_1600;
using TxPDO = SPdo::SOMANET_TxPDO_1A00;

// ----------------------------------------------------------------------------
// CSP sine position controller — identical demand on both drives, per-drive
// phase shift.  The first 500 ms ramp the amplitude in so a 90°-shifted
// motor does not get a setpoint step at t=0.
// ----------------------------------------------------------------------------
class SinePositionController final
    : public EtherCAT::DS402Master::IDriveMotionController {
public:
    SinePositionController(int32_t center, int32_t amplitude_inc,
                           double freq_hz, double phase_rad)
        : center_(center), amplitude_inc_(amplitude_inc),
          freq_hz_(freq_hz), phase_rad_(phase_rad) {}

    bool start(EtherCAT::CiA402Drive&) override { return true; }
    void stop(EtherCAT::CiA402Drive&) override {}

    bool update(EtherCAT::CiA402Drive& drive, double dt_seconds) override {
        auto* rx = static_cast<RxPDO*>(drive.getRxPDOBuffer());
        if (!rx) return false;

        elapsed_s_ += dt_seconds;
        const double ramp = elapsed_s_ < 0.5 ? elapsed_s_ / 0.5 : 1.0;
        const double w = 2.0 * M_PI * freq_hz_ * elapsed_s_ + phase_rad_;

        rx->controlword =
            static_cast<uint16_t>(CiA402::ControlWord::EnableOperation);
        rx->modes_of_operation =
            static_cast<int8_t>(CiA402::OperatingMode::CyclicSyncPosition);
        rx->target_position = center_ + static_cast<int32_t>(
            std::lround(amplitude_inc_ * ramp * std::sin(w)));
        return true;
    }

private:
    int32_t center_ = 0;
    int32_t amplitude_inc_ = 0;
    double  freq_hz_ = 0.0;
    double  phase_rad_ = 0.0;
    double  elapsed_s_ = 0.0;
};

// ----------------------------------------------------------------------------
// Periodic status line: both drives on one line, decoded straight from the
// live TxPDO buffers (runs on the RT loop thread).
// ----------------------------------------------------------------------------
class DriveStatusTask final : public EtherCAT::ICyclicTask {
public:
    DriveStatusTask(uint16_t slave0, uint16_t slave1)
        : slaves_{slave0, slave1} {}

    bool update(EtherCAT::DS402Master& master, double dt_seconds) override {
        elapsed_ms_ += static_cast<uint64_t>(dt_seconds * 1000.0);
        if (elapsed_ms_ - last_print_ms_ < 1000) return true;
        last_print_ms_ = elapsed_ms_;

        const char* names[] = {"m1", "m2"};
        std::string line;
        for (int i = 0; i < 2; ++i) {
            auto* d = master.driveBySlaveIndex(slaves_[i]);
            auto* tx = d ? static_cast<const TxPDO*>(d->getTxPDOBuffer())
                       : nullptr;
            if (!tx) continue;
            if (!line.empty()) line += " | ";
            line += std::format("{}: sw=0x{:04X} pos={} vel={} tq={}",
                                names[i], tx->statusword,
                                static_cast<int32_t>(tx->position_actual),
                                static_cast<int32_t>(tx->velocity_actual),
                                static_cast<int16_t>(tx->torque_actual));
        }
        TETHER_LOGI(TAG, "--- Drives @ {} ms ---  {}",
                    static_cast<unsigned long long>(elapsed_ms_), line);
        return true;
    }

private:
    uint16_t slaves_[2];
    uint64_t elapsed_ms_ = 0;
    uint64_t last_print_ms_ = 0;
};

}  // namespace

int main(int argc, char* argv[])
{
    argparse::ArgumentParser program("synapticon_csp_2motors", "1.0");
    Tether::Examples::addInterfaceArg(program);
    Tether::Examples::addSlaveArg(program);
    program.add_argument("--slave2")
        .help("Bus index of motor 2 (default: --slave + 1)")
        .default_value(-1)
        .scan<'i', int>();
    program.add_argument("-d", "--duration")
        .help("Run duration in seconds — both motors start and stop "
              "together (default: 11.0)")
        .default_value(11.0)
        .scan<'g', double>();
    program.add_argument("--amplitude-deg")
        .help("Sine amplitude in degrees around the enabled position "
              "(default 90 = ±90°)")
        .default_value(90.0)
        .scan<'g', double>();
    program.add_argument("--freq-hz")
        .help("Sine frequency in Hz (default 0.5)")
        .default_value(0.5)
        .scan<'g', double>();
    program.add_argument("--phase-deg")
        .help("Phase offset of motor 2's sine relative to motor 1, in "
              "degrees (default 90)")
        .default_value(90.0)
        .scan<'g', double>();
    program.add_argument("--max-torque-pm")
        .help("CSP torque limit for both motors in per-mille of rated "
              "torque, written to 0x6072 Max torque in PRE_OP "
              "(default 300 = 30%)")
        .default_value(300)
        .scan<'i', int>();
    program.add_argument("--debug")
        .default_value(std::string(""))
        .help("Comma-separated Tether debug flags. '--debug help' lists them.");

    try {
        program.parse_args(argc, argv);
    } catch (const std::exception& e) {
        std::print(stderr, "{}\n{}\n", e.what(), program.help().str());
        return 1;
    }

    const std::string iface = Tether::Examples::resolveInterface(
        program.get<std::string>("--interface"), TAG);
    const auto slave_indices_opt =
        Tether::Examples::resolveSlaveIndices(program, {"--slave", "--slave2"});
    if (!slave_indices_opt) return 1;
    const std::vector<uint16_t> slave_indices = *slave_indices_opt;
    const double duration_sec = program.get<double>("--duration");
    const double amplitude_deg = program.get<double>("--amplitude-deg");
    const double freq_hz = program.get<double>("--freq-hz");
    const double phase_deg = program.get<double>("--phase-deg");
    const int max_torque_pm = program.get<int>("--max-torque-pm");
    const std::string debug_str = program.get<std::string>("--debug");

    if (debug_str == "help") {
        std::cout << "Available --debug flags (comma-separated):\n";
        for (const auto& info : EtherCAT::debug::allDebugFlags()) {
            std::cout << "  " << info.name << "\n      "
                      << info.description << "\n";
        }
        return 0;
    }

    Tether::Examples::EncapsulationConfig encap;
    if (!Tether::Examples::parseEncapsulationArg(
            program.get<std::string>("--encapsulation"), encap, TAG)) {
        return 1;
    }

    Tether::Platform::ensureRealtimeKernelOrExit();

    if (amplitude_deg <= 0.0 || amplitude_deg > 360.0) {
        std::print(stderr, "--amplitude-deg must be in (0, 360]\n");
        return 1;
    }
    if (max_torque_pm < 0 || max_torque_pm > 1000) {
        std::print(stderr, "--max-torque-pm must be in [0, 1000]\n");
        return 1;
    }

    TETHER_LOGI(TAG,
        "synapticon_csp_2motors — interface={} slaves={}+{} "
        "duration={:.1f}s amplitude=±{:.1f}° freq={:.3f}Hz phase={:.1f}° "
        "max_torque={}‰",
        iface, slave_indices[0], slave_indices[1], duration_sec,
        amplitude_deg, freq_hz, phase_deg, max_torque_pm);

    // ---- Host Ethernet + master bring-up ----
    EtherCAT::DS402Master master;
    Tether::Examples::HostEtherNetSession session;
    Tether::Utils::SignalHandler sig;  // Ctrl-C → master.requestCancel()
    if (!Tether::Examples::bringUpHost(session, master.ethercatMaster(),
                                     iface, encap, TAG, &sig)) {
        return 5;
    }
    // Stops DCs + master + host session on every exit path from here on.
    Tether::Examples::HostSessionGuard host_guard(session, [&master] {
        master.stopDistributedClocks();
        master.stop();
    });

    // ---- Discover + verify both motors are SOMANET drives ----
    if (!Syn::discoverDrives(
            master.ethercatMaster().discovery(), slave_indices, TAG)) {
        return 3;
    }
    const uint16_t max_idx = std::max(slave_indices[0], slave_indices[1]);
    if (!master.waitForDriveCount(max_idx + 1, 2000)) {
        TETHER_LOGE(TAG, "Timed out waiting for slave {}", max_idx);
        return 4;
    }
    {
        const auto flags = Tether::Examples::parseDebugFlags(debug_str);
        Tether::Examples::applyDebugFlags(flags, master.ethercatMaster(), TAG);
    }

    if (!master.startDistributedClocks(EtherCAT::DC::DCConfig::defaults())) {
        TETHER_LOGE(TAG, "Failed to start distributed clocks");
        return 5;
    }

    // ---- Synchronised group bring-up ----
    // Every AL transition below is a single EtherCAT packet covering both
    // drives; AL polls read both in one frame.
    EtherCAT::SlaveGroup drives(master.ethercatMaster(),
                                {slave_indices[0], slave_indices[1]});
    Init init[2] = {
        {master, slave_indices[0], TAG}, {master, slave_indices[1], TAG}};

    // INIT + mailbox + PRE_OP for the whole group; also reads each drive's
    // firmware version (0x100A) and applies the mailbox-in-SAFE_OP policy
    // (SOMANET >= 5.6 stops servicing the mailbox in SAFE_OP).
    if (!Init::initGroupToPreOp(init, drives)) {
        return 6;
    }
    TETHER_LOGI(TAG, "Both drives in PRE_OP, mailbox ready");

    // ---- PRE_OP SDO configuration ----
    // All mailbox traffic must complete before SAFE_OP on this firmware.
    const EtherCAT::CoE::CoETransactionOptions sdo_opts{.timeout_ms = 5000};

    if (!CiA402::setOperatingModeVerified(drives,
            CiA402::OperatingMode::CyclicSyncPosition,
            sdo_opts, /*settle_ms=*/50, TAG)) {
        return 6;
    }
    TETHER_LOGI(TAG, "CSP mode (0x6060) set to 8 on both drives");

    if (!drives.writeEntryAll(P60::MaxTorque,
            static_cast<uint64_t>(max_torque_pm), sdo_opts, TAG)) {
        return 6;
    }

    // Encoder resolution (0x608F) for the degrees→increments conversion —
    // read now, while the mailbox is still serviced.
    uint32_t encoder_res[2] = {Syn::kDefaultEncoderResolution,
                               Syn::kDefaultEncoderResolution};
    for (size_t m = 0; m < 2; ++m) {
        auto res = master.ethercatMaster().sdoManager(slave_indices[m])
                       .readU32(0x608F, 1, sdo_opts);
        if (res && *res > 0) encoder_res[m] = *res;
    }

    // Velocity field offsets for the controlled-shutdown CSV ramp.
    for (auto& ini : init) {
        ini.drive().setVelocityPDOFields(&RxPDO::target_velocity,
                                         &TxPDO::velocity_actual);
    }

    // ---- Group SAFE_OP — one packet for both drives ----
    // PDO config happens in PRE_OP per slave; the SAFE_OP request itself is
    // a single packet so both slaves validate SM2/SM3 at the same instant.
    // The keep-alive starts the moment SAFE_OP is reached — it keeps the
    // process-data watchdogs fed during the post-SAFE_OP per-slave prep.
    std::unique_ptr<EtherCAT::PDOKeepAlive> pdo_keep_alive;
    const auto pdo_assignment = SPdo::makeStandardPDOAssignment();
    if (!Init::bringGroupToSafeOp(init, pdo_assignment, drives, [&] {
            pdo_keep_alive = master.ethercatMaster().pdo().startKeepAlive();
        })) {
        return 6;
    }
    TETHER_LOGI(TAG, "Both drives reached SAFE_OP simultaneously");

    // ---- Group OP — one packet for both slaves ----
    if (!Init::requestGroupOp(drives, TAG)) {
        return 6;
    }
    TETHER_LOGI(TAG, "Both slaves reached OP simultaneously");

    // Hand the cyclic exchange from the keep-alive to the realtime motion
    // loop (running both at once would race on the PDOManager).
    pdo_keep_alive->stop();
    EtherCAT::Master::RealtimeMotionLoopConfig loop_config;
    loop_config.cycle_period_us = 1000;
    if (!master.startRealtimeMotionControlLoop(loop_config)) {
        TETHER_LOGE(TAG, "Failed to start realtime motion loop");
        return 7;
    }

    int rc = 0;

    // ---- Register the motion controllers BEFORE enabling: they write
    // controlword=EnableOperation + mode=CSP to the PDO every cycle, so a
    // valid demand is on the wire throughout the enable sequence (which
    // uses SDO controlword writes precisely because the PDO CW is owned by
    // the cyclic controllers).  The sine center is the current position.
    int32_t sine_center[2] = {0, 0};
    int32_t amplitude_inc[2] = {0, 0};
    for (int m = 0; m < 2; ++m) {
        auto* drv = master.driveBySlaveIndex(slave_indices[m]);
        auto* tx = static_cast<const TxPDO*>(drv->getTxPDOBuffer());
        if (tx) sine_center[m] = tx->position_actual;
        amplitude_inc[m] = Syn::degreesToIncrements(amplitude_deg,
                                                  encoder_res[m]);
    }
    // Both motors run the more conservative of the two amplitudes (encoder
    // resolutions may differ between drives).
    amplitude_inc[0] = amplitude_inc[1] =
        std::min(amplitude_inc[0], amplitude_inc[1]);
    const double phase_rad = phase_deg * M_PI / 180.0;

    master.addMotionController(slave_indices[0],
        std::make_unique<SinePositionController>(
            sine_center[0], amplitude_inc[0], freq_hz, 0.0));
    master.addMotionController(slave_indices[1],
        std::make_unique<SinePositionController>(
            sine_center[1], amplitude_inc[1], freq_hz, phase_rad));
    master.addCyclicTask(std::make_unique<DriveStatusTask>(
        slave_indices[0], slave_indices[1]));

    // ---- Enable both drives.  Retry with fault reset between attempts:
    // these drives can fall back to Switch-On-Disabled during the enable
    // sequence after a previously latched fault — the interface path
    // recovered the same way (reset + retry, up to 3 attempts).
    for (int m = 0; m < 2; ++m) {
        auto* drv = master.driveBySlaveIndex(slave_indices[m]);
        bool enabled = false;
        for (int attempt = 0; attempt < 3 && !enabled; ++attempt) {
            drv->resetFault();
            enabled = master.enableDrive(slave_indices[m], 5000);
        }
        if (!enabled) {
            TETHER_LOGE(TAG, "Slave {}: drive enable failed",
                        slave_indices[m]);
            rc = 8;
            break;
        }
        TETHER_LOGI(TAG, "Motor {} (slave {}): enabled — sine ±{:.1f}° = "
                    "{} inc around {} inc",
                    m + 1, slave_indices[m], amplitude_deg, amplitude_inc[m],
                    sine_center[m]);
    }

    if (rc == 0) {
        TETHER_LOGI(TAG,
            "CSP active on both drives, identical sine ±{:.1f}° at {:.3f} "
            "Hz for {:.1f} s (motor 2 phase-shifted +{:.1f}°)",
            amplitude_deg, freq_hz, duration_sec, phase_deg);

        // ---- Run loop ----
        auto& clock = Tether::Platform::Clock::instance();
        const int64_t run_start_ms = clock.getMilliseconds();
        const int64_t run_duration_ms =
            static_cast<int64_t>(duration_sec * 1000.0);
        while (true) {
            if (clock.getMilliseconds() - run_start_ms >= run_duration_ms)
                break;
            if (sig.stop_requested()) {
                TETHER_LOGI(TAG, "Ctrl-C received — stopping early");
                break;
            }
            clock.delayMilliseconds(50);
        }
    }

    // ---- Controlled shutdown (ordering is load-bearing) ----
    // Remove the motion controllers first so they stop overwriting the
    // RxPDO buffers every cycle; then ramp to standstill and engage the
    // brake while the PDO exchange is still live.
    for (const auto idx : slave_indices) {
        (void)master.removeMotionController(idx);
    }
    master.ethercatMaster().clearCancel();
    for (const auto idx : slave_indices) {
        auto* drive = master.driveBySlaveIndex(idx);
        if (!drive) continue;
        EtherCAT::CiA402Drive::ControlledShutdownConfig scfg;
        scfg.brake_action = [&master, idx] {
            return Syn::BrakeControl::engageBrake(
                master.ethercatMaster().sdoManager(idx));
        };
        if (!drive->controlledShutdown(scfg)) {
            TETHER_LOGW(TAG, "Slave {}: controlled shutdown reported errors",
                        idx);
        }
    }
    master.stopMotionControlLoop();
    master.clearCyclicTasks();

    master.ethercatMaster().pdo().dumpStats(TAG);
    return rc;
}
