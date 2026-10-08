/**
 * @file synapticon_cst_dynamic_brake.cpp
 * @brief Single-motor SOMANET example: CSP sine with periodic PDO-mapped
 *        brake toggling — pure Tether, no ethercat_manager layer.
 *
 * ========================================================================
 * HOW THE DYNAMIC BRAKE CONTROL WORKS
 * ========================================================================
 *
 * The SOMANET brake is a spring-loaded, normally-engaged holding brake on
 * the motor shaft. Releasing it requires driving a PWM voltage onto the
 * motor's "4th phase" (phase D) output stage — there is no dedicated
 * brake pin, the brake solenoid shares the inverter. The SOMANET brake
 * object 0x2004 controls this:
 *
 *   0x2004:1  Pull voltage (mV)      – needed to retract the pin
 *   0x2004:2  Hold voltage (mV)      – keeps it retracted, less heating
 *   0x2004:4  Release strategy       – 0=Manual output voltage,
 *                                      1=Clutch (firmware-managed),
 *                                      2=Pin brake
 *   0x2004:7  Brake status/command   – status: 1=engaged, 2=released
 *                                      (NOT PDO-mappable on this firmware)
 *   0x2004:10 Output voltage (mV)    – RxPDO-MAPPABLE; the voltage that is
 *                                      physically applied to phase D
 *
 * The mechanism abused here is documented in the SOMANET object
 * dictionary entry for 0x2004:10:
 *
 *   "Voltage which is applied on the 4th phase when the Release strategy
 *    is set to manual mode (0) AND Physical output bit 0 as well as the
 *    corresponding Bit mask in object 0x60FE is set to 1."
 *
 * So the brake pin is driven by TWO PDO-visible controls:
 *
 *   a) 0x60FE:1 "Physical outputs" bit 0 + 0x60FE:2 "Bit mask" bit 0
 *      — the ON/OFF switch that gates the phase-D output stage. These
 *      are already mapped in the standard RxPDO 0x1601 (8 bytes),
 *      sitting at offset 19 in the SM2 image. No remapping needed.
 *
 *   b) 0x2004:10 "Output voltage" (u16, mV) — the LEVEL that is applied
 *      when the gate is open. Not mapped by default, so this example
 *      remaps RxPDO 0x1602 in PRE_OP to carry this single 16-bit word
 *      (offset 27 in the SM2 image).
 *
 * With strategy = Manual, the firmware no longer runs its automatic
 * brake state machine; instead the output stage literally puts the
 * 0x2004:10 millivolts onto phase D whenever 0x60FE bit 0 is set.
 *
 *   Engage  : physical_outputs bit0 = 0  (and voltage word = 0)
 *             -> no voltage on phase D -> spring clamps the brake.
 *   Release : physical_outputs bit0 = 1 + voltage word = PULL (24 V)
 *             -> solenoid retracts the pin. Then the word drops to
 *             HOLD (7 V) to keep it retracted without overheating.
 *
 * The result: brake engage/release is just two cyclic PDO writes per
 * cycle — bit 0 of the 0x1601 image plus a 16-bit mV word — with zero
 * mailbox traffic in OP. A background SDO poll of the read-only status
 * object 0x2004:7 confirms the physical transitions (1 <-> 2).
 *
 * ========================================================================
 * MOTION SEQUENCE (all via cyclic PDO data)
 * ========================================================================
 *
 *   1. CSP sine around the enabled position for --cycle-ms (default
 *      1500 ms)
 *   2. Switch modes_of_operation byte to CSV with target velocity 0 and
 *      wait for near-standstill (--zero-vel threshold)
 *   3. Drop the output bit -> brake engages (verified: torque rises to
 *      the limit while velocity stays ~0)
 *   4. Raise the output bit with pull voltage -> pin retracts
 *   5. Switch back to CSP, re-center on the current position, restart
 *      the sine
 *
 * SM2 layout stays the standard 35 bytes: 0x1600 (19 B, CiA 402 motion)
 * + 0x1601 (8 B, physical outputs — the brake gate) + 0x1602 (2 B brake
 * voltage word + 6 B padding).
 *
 * Example:
 *   runec ./synapticon_cst_dynamic_brake -i enx00e04c680004
 *   runec ./synapticon_cst_dynamic_brake -i enx00e04c680004 \
 *       --cycle-ms 3000 --zero-vel 500 --amplitude-deg 90
 */
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <cstdio>
#include <iostream>
#include <memory>
#include <print>
#include <string>
#include <vector>
#include "common/ExampleHelpers.hpp"
#include "common/EtherCATHostSetup.hpp"
#include "tether/drives/Synapticon.hpp"
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

namespace Syn  = EtherCAT::Drives::Synapticon;
namespace SPdo = EtherCAT::Drives::SynapticonPDO;
using Init       = Syn::SynapticonDriveInitializer;

using RxPDO = SPdo::SOMANET_RxPDO_1600;
using TxPDO = SPdo::SOMANET_TxPDO_1A00;

namespace P60 = CiA402::Parameters60xx;

static constexpr const char* TAG = "synapticon_cst_dynamic_brake";

/// Offset of RxPDO 0x1601 (0x60FE physical outputs + bit mask) inside the
/// 35-byte SM2 image — bit 0 of both words drives the brake output pin.
static constexpr size_t kBrakeOutputOffset = sizeof(SPdo::SOMANET_RxPDO_1600);
/// Byte offset of the remapped 0x2004:10 Output voltage word inside the
/// SM2 image: after RxPDO 0x1600 (19 B) and RxPDO 0x1601 (8 B).
static constexpr size_t kBrakeByteOffset =
    sizeof(SPdo::SOMANET_RxPDO_1600) + sizeof(SPdo::SOMANET_RxPDO_1601);
static constexpr size_t kRxPDOImageSize = 35;
static_assert(kBrakeByteOffset + 2 <= kRxPDOImageSize);

// ----------------------------------------------------------------------------
// Cyclic controller: CSP sine with periodic CSV-decel + PDO brake toggle.
// ----------------------------------------------------------------------------
class DynamicBrakeController final : public EtherCAT::DS402Master::IDriveMotionController {
public:
    DynamicBrakeController(int32_t center, int32_t amplitude_inc,
                           double freq_hz, uint64_t cycle_ms,
                           int32_t zero_vel_inc, uint64_t brake_hold_ms,
                           uint16_t pull_mv, uint16_t hold_mv)
        : center_(center), amplitude_inc_(amplitude_inc), freq_hz_(freq_hz),
          cycle_ms_(cycle_ms), zero_vel_inc_(zero_vel_inc),
          brake_hold_ms_(brake_hold_ms),
          pull_mv_(pull_mv), hold_mv_(hold_mv) {}

    bool start(EtherCAT::CiA402Drive&) override { return true; }
    void stop(EtherCAT::CiA402Drive&) override {}

    const char* phaseName() const {
        switch (phase_) {
        case Phase::Run:      return "RUN";
        case Phase::Decel:    return "DECEL";
        case Phase::Release:  return "BRAKE-OFF";
        case Phase::Engage:   return "BRAKE-ON";
        }
        return "?";
    }

    bool update(EtherCAT::CiA402Drive& drive, double dt_seconds) override {
        auto* rx = static_cast<RxPDO*>(drive.getRxPDOBuffer());
        auto* tx = static_cast<const TxPDO*>(drive.getTxPDOBuffer());
        if (!rx || !tx) return false;
        uint8_t* buf = static_cast<uint8_t*>(drive.getRxPDOBuffer());
        // 0x60FE:1 physical outputs / 0x60FE:2 bit mask — bit 0 selects the
        // brake output pin.  Mask bit must stay set for the pin to follow.
        auto* io = reinterpret_cast<SPdo::SOMANET_RxPDO_1601*>(
            buf + kBrakeOutputOffset);
        io->bit_mask = 1;
        // 0x2004:10 Output voltage (mV) — the solenoid level.
        uint16_t* brake = reinterpret_cast<uint16_t*>(buf + kBrakeByteOffset);

        const uint64_t dt_ms =
            static_cast<uint64_t>(dt_seconds * 1000.0 + 0.5);
        phase_ms_ += dt_ms;

        rx->controlword =
            static_cast<uint16_t>(CiA402::ControlWord::EnableOperation);

        switch (phase_) {
        case Phase::Run:
            // Brake released while moving: output bit set; pull voltage to
            // retract the pin at the start of each run window, then the
            // lower hold voltage to limit solenoid heating.
            io->physical_outputs = 1;
            *brake = (elapsed_s_ < 0.15) ? pull_mv_ : hold_mv_;
            elapsed_s_ += dt_seconds;
            rx->modes_of_operation =
                static_cast<int8_t>(CiA402::OperatingMode::CyclicSyncPosition);
            {
                const double ramp = elapsed_s_ < 0.5 ? elapsed_s_ / 0.5 : 1.0;
                const double w = 2.0 * M_PI * freq_hz_ * elapsed_s_;
                rx->target_position = center_ + static_cast<int32_t>(
                    std::lround(amplitude_inc_ * ramp * std::sin(w)));
            }
            if (phase_ms_ >= cycle_ms_) {
                phase_ = Phase::Decel;
                phase_ms_ = 0;
                TETHER_LOGI(TAG, "Switching to CSV, decelerating...");
            }
            break;

        case Phase::Decel:
            io->physical_outputs = 1;
            *brake = hold_mv_;
            rx->modes_of_operation =
                static_cast<int8_t>(CiA402::OperatingMode::CyclicSyncVelocity);
            rx->target_velocity = 0;
            if (std::abs(static_cast<int32_t>(tx->velocity_actual)) <=
                    zero_vel_inc_ ||
                phase_ms_ >= decel_timeout_ms_) {
                phase_ = Phase::Engage;
                phase_ms_ = 0;
                TETHER_LOGI(TAG, "Standstill reached (vel={}) — engaging brake",
                            static_cast<int32_t>(tx->velocity_actual));
            }
            break;

        case Phase::Engage:
            io->physical_outputs = 0;
            *brake = 0;
            if (phase_ms_ >= brake_hold_ms_) {
                phase_ = Phase::Release;
                phase_ms_ = 0;
                TETHER_LOGI(TAG, "Releasing brake");
            }
            break;

        case Phase::Release:
            io->physical_outputs = 1;
            *brake = pull_mv_;
            if (phase_ms_ >= brake_hold_ms_) {
                // Resume motion: back to CSP, re-center on the current
                // position, restart the sine from zero.  Hold voltage is
                // written by the Run phase on the next cycle.
                center_ = tx->position_actual;
                elapsed_s_ = 0.0;
                rx->target_position = center_;
                rx->modes_of_operation = static_cast<int8_t>(
                    CiA402::OperatingMode::CyclicSyncPosition);
                phase_ = Phase::Run;
                phase_ms_ = 0;
                TETHER_LOGI(TAG, "Resuming CSP sine around {} inc", center_);
            }
            break;
        }
        return true;
    }

private:
    enum class Phase { Run, Decel, Release, Engage };

    Phase    phase_ = Phase::Run;
    int32_t  center_ = 0;
    uint16_t pull_mv_ = 24000;
    uint16_t hold_mv_ = 7000;
    int32_t  amplitude_inc_ = 0;
    double   freq_hz_ = 0.0;
    uint64_t cycle_ms_ = 3000;
    int32_t  zero_vel_inc_ = 500;
    uint64_t brake_hold_ms_ = 400;
    uint64_t decel_timeout_ms_ = 5000;
    uint64_t phase_ms_ = 0;
    double   elapsed_s_ = 0.0;
};

// ----------------------------------------------------------------------------
// Periodic status line (runs on the RT loop thread).
// ----------------------------------------------------------------------------
class DriveStatusTask final : public EtherCAT::ICyclicTask {
public:
    DriveStatusTask(uint16_t slave, const DynamicBrakeController* ctl)
        : slave_(slave), ctl_(ctl) {}

    bool update(EtherCAT::DS402Master& master, double dt_seconds) override {
        elapsed_ms_ += static_cast<uint64_t>(dt_seconds * 1000.0);
        if (elapsed_ms_ - last_print_ms_ < 500) return true;
        last_print_ms_ = elapsed_ms_;

        auto* d = master.driveBySlaveIndex(slave_);
        auto* tx = d ? static_cast<const TxPDO*>(d->getTxPDOBuffer())
                     : nullptr;
        if (!tx) return true;
        uint16_t brake = 0;
        uint8_t out_bit = 0;
        if (d) {
            const auto* buf =
                static_cast<const uint8_t*>(d->getRxPDOBuffer());
            brake = *reinterpret_cast<const uint16_t*>(buf +
                                                       kBrakeByteOffset);
            const auto* io = reinterpret_cast<const SPdo::SOMANET_RxPDO_1601*>(
                buf + kBrakeOutputOffset);
            out_bit = io->physical_outputs & 1;
        }
        TETHER_LOGI(TAG,
            "@{} ms [{}] sw=0x{:04X} pos={} vel={} tq={} out={} brake={}mV",
            static_cast<unsigned long long>(elapsed_ms_), ctl_->phaseName(),
            tx->statusword, static_cast<int32_t>(tx->position_actual),
            static_cast<int32_t>(tx->velocity_actual),
            static_cast<int16_t>(tx->torque_actual), out_bit, brake);
        return true;
    }

private:
    uint16_t slave_;
    const DynamicBrakeController* ctl_;
    uint64_t elapsed_ms_ = 0;
    uint64_t last_print_ms_ = 0;
};

// ============================================================================
int main(int argc, char* argv[])
{
    argparse::ArgumentParser program("synapticon_cst_dynamic_brake", "1.0");
    Tether::Examples::addInterfaceArg(program);
    Tether::Examples::addSlaveArg(program);
    program.add_argument("-d", "--duration")
        .help("Run duration in seconds (default: 15.0)")
        .default_value(15.0)
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
    program.add_argument("--cycle-ms")
        .help("Sine-run duration between brake cycles in ms (default 1500)")
        .default_value(1500)
        .scan<'i', int>();
    program.add_argument("--zero-vel")
        .help("Near-standstill threshold for velocity_actual (default 500)")
        .default_value(500)
        .scan<'i', int>();
    program.add_argument("--brake-hold-ms")
        .help("Brake off/on hold time in ms (default 400)")
        .default_value(400)
        .scan<'i', int>();
    program.add_argument("--max-torque-pm")
        .help("Torque limit in per-mille of rated torque, written to "
              "0x6072 Max torque in PRE_OP (default 300 = 30%)")
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
        Tether::Examples::resolveSlaveIndices(program, {"--slave"});
    if (!slave_indices_opt) return 1;
    const uint16_t slave_idx = (*slave_indices_opt)[0];
    const double duration_sec = program.get<double>("--duration");
    const double amplitude_deg = program.get<double>("--amplitude-deg");
    const double freq_hz = program.get<double>("--freq-hz");
    const int cycle_ms = program.get<int>("--cycle-ms");
    const int zero_vel = program.get<int>("--zero-vel");
    const int brake_hold_ms = program.get<int>("--brake-hold-ms");
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
        "synapticon_cst_dynamic_brake — interface={} slave={} "
        "duration={:.1f}s amplitude=±{:.1f}° freq={:.3f}Hz "
        "cycle={}ms zero_vel={} brake_hold={}ms max_torque={}‰",
        iface, slave_idx, duration_sec, amplitude_deg, freq_hz,
        cycle_ms, zero_vel, brake_hold_ms, max_torque_pm);

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

    // ---- Discover + verify the drive is a SOMANET ----
    const std::vector<uint16_t> slave_vec{slave_idx};
    if (!Syn::discoverDrives(
            master.ethercatMaster().discovery(), slave_vec, TAG)) {
        return 3;
    }
    if (!master.waitForDriveCount(slave_idx + 1, 2000)) {
        TETHER_LOGE(TAG, "Timed out waiting for slave {}", slave_idx);
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

    // ---- Group bring-up (single slave — same single-packet AL path) ----
    EtherCAT::SlaveGroup drive_grp(master.ethercatMaster(), {slave_idx});
    Init init[1] = {{master, slave_idx, TAG}};

    if (!Init::initGroupToPreOp(init, drive_grp)) {
        return 6;
    }
    TETHER_LOGI(TAG, "Drive in PRE_OP, mailbox ready");

    // ---- PRE_OP SDO configuration ----
    // All mailbox traffic must complete before SAFE_OP on this firmware.
    const EtherCAT::CoE::CoETransactionOptions sdo_opts{.timeout_ms = 5000};
    auto& sdo = master.ethercatMaster().sdoManager(slave_idx);

    if (!CiA402::setOperatingModeVerified(drive_grp,
            CiA402::OperatingMode::CyclicSyncPosition,
            sdo_opts, /*settle_ms=*/50, TAG)) {
        return 6;
    }
    TETHER_LOGI(TAG, "CSP mode (0x6060) set to 8");

    if (!drive_grp.writeEntryAll(P60::MaxTorque,
            static_cast<uint64_t>(max_torque_pm), sdo_opts, TAG)) {
        return 6;
    }

    // Encoder resolution (0x608F) for the degrees→increments conversion.
    uint32_t encoder_res = Syn::kDefaultEncoderResolution;
    if (auto res = sdo.readU32(0x608F, 1, sdo_opts); res && *res > 0) {
        encoder_res = *res;
    }

    // ---- Manual brake control over PDO ----
    // 0x2004:7 is not PDO-mappable on this firmware — the cyclic control
    // point is 0x2004:10 "Output voltage", honored when the release
    // strategy 0x2004:4 is Manual (0).  Read pull/hold voltages so the
    // toggle uses the drive's configured solenoid levels.
    uint16_t brake_pull_mv = 24000;
    uint16_t brake_hold_mv = 7000;
    if (auto v = sdo.readU32(0x2004, 1, sdo_opts); v && *v > 0 &&
        *v <= 60000) {
        brake_pull_mv = static_cast<uint16_t>(*v);
    }
    if (auto v = sdo.readU32(0x2004, 2, sdo_opts); v && *v > 0 &&
        *v <= 60000) {
        brake_hold_mv = static_cast<uint16_t>(*v);
    }
    if (!sdo.writeU8(0x2004, 4, 0, sdo_opts).has_value()) {
        TETHER_LOGE(TAG, "Failed to set 0x2004:4 release strategy = Manual");
        return 6;
    }
    TETHER_LOGI(TAG, "Brake: manual output-voltage mode, pull={} mV "
                     "hold={} mV", brake_pull_mv, brake_hold_mv);

    // ---- Remap RxPDO 0x1602 to carry 0x2004:10 Output voltage ----
    // ETG.1000 mapping sequence: disable (sub0=0), write entries, enable.
    // 0x20040A10 = index 0x2004, subindex 10, 16 bits.
    {
        bool ok = sdo.writeU8(0x1602, 0, 0, sdo_opts).has_value()
               && sdo.writeU32(0x1602, 1, 0x20040A10, sdo_opts).has_value()
               && sdo.writeU8(0x1602, 0, 1, sdo_opts).has_value();
        if (!ok) {
            TETHER_LOGE(TAG, "Failed to remap 0x1602 -> 0x2004:10 "
                             "(brake output voltage) via SDO");
            return 6;
        }
        auto cnt = sdo.readU8(0x1602, 0, sdo_opts);
        auto ent = sdo.readU32(0x1602, 1, sdo_opts);
        TETHER_LOGI(TAG, "RxPDO 0x1602 remapped: entry 1 = 0x2004:10 "
                         "brake voltage (16 bit) at SM2 offset {} "
                         "[readback: count={} entry0=0x{:08X}]",
                    kBrakeByteOffset,
                    cnt.has_value() ? *cnt : 0xFF,
                    ent.has_value() ? *ent : 0xFFFFFFFFu);
    }

    // Velocity field offsets for the controlled-shutdown CSV ramp.
    init[0].drive().setVelocityPDOFields(&RxPDO::target_velocity,
                                         &TxPDO::velocity_actual);

    // ---- SAFE_OP — single packet ----
    std::unique_ptr<EtherCAT::PDOKeepAlive> pdo_keep_alive;
    const auto pdo_assignment = SPdo::makeStandardPDOAssignment();
    if (!Init::bringGroupToSafeOp(init, pdo_assignment, drive_grp, [&] {
            pdo_keep_alive = master.ethercatMaster().pdo().startKeepAlive();
        })) {
        return 6;
    }
    TETHER_LOGI(TAG, "Drive reached SAFE_OP");

    // ---- OP ----
    if (!Init::requestGroupOp(drive_grp, TAG)) {
        return 6;
    }
    TETHER_LOGI(TAG, "Drive reached OP");

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

    // ---- Register the motion controller BEFORE enabling: it writes
    // controlword=EnableOperation + mode=CSP to the PDO every cycle, so a
    // valid demand is on the wire throughout the enable sequence.
    auto* drv = master.driveBySlaveIndex(slave_idx);
    auto* tx = static_cast<const TxPDO*>(drv->getTxPDOBuffer());
    const int32_t sine_center = tx ? tx->position_actual : 0;
    const int32_t amplitude_inc =
        Syn::degreesToIncrements(amplitude_deg, encoder_res);

    auto controller = std::make_unique<DynamicBrakeController>(
        sine_center, amplitude_inc, freq_hz,
        static_cast<uint64_t>(cycle_ms), zero_vel,
        static_cast<uint64_t>(brake_hold_ms),
        brake_pull_mv, brake_hold_mv);
    auto* ctl_ptr = controller.get();
    master.addMotionController(slave_idx, std::move(controller));
    master.addCyclicTask(std::make_unique<DriveStatusTask>(slave_idx,
                                                         ctl_ptr));

    // ---- Enable (with fault-reset retries — drives can latch a fault
    // ---- from an aborted previous run).
    bool enabled = false;
    for (int attempt = 0; attempt < 3 && !enabled; ++attempt) {
        drv->resetFault();
        enabled = master.enableDrive(slave_idx, 5000);
    }
    if (!enabled) {
        TETHER_LOGE(TAG, "Slave {}: drive enable failed", slave_idx);
        rc = 8;
    } else {
        TETHER_LOGI(TAG, "Drive enabled — sine ±{:.1f}° = {} inc around "
                    "{} inc; brake toggle every {} ms",
                    amplitude_deg, amplitude_inc, sine_center, cycle_ms);
    }

    if (rc == 0) {
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
            // Ground truth: mailbox is serviced in OP — read the actual
            // brake status to confirm the PDO voltage toggles it.
            auto bs = sdo.readU8(0x2004, 7, sdo_opts);
            auto ov = sdo.readU16(0x2004, 10, sdo_opts);
            auto st = sdo.readU8(0x2004, 4, sdo_opts);
            TETHER_LOGI(TAG, "  [strategy 0x2004:4 = {} "
                             "brake-status 0x2004:7 = {} "
                             "out-voltage 0x2004:10 = {} mV]",
                        st.has_value() ? static_cast<int>(*st) : -1,
                        bs.has_value() ? static_cast<int>(*bs) : -1,
                        ov.has_value() ? static_cast<int>(*ov) : -1);
            clock.delayMilliseconds(500);
        }
    }

    // ---- Controlled shutdown (ordering is load-bearing) ----
    (void)master.removeMotionController(slave_idx);
    master.ethercatMaster().clearCancel();
    {
        EtherCAT::CiA402Drive::ControlledShutdownConfig scfg;
        // All brake interaction goes through the PDO-mapped command byte —
        // write Engage into 0x2004:7 and let the cyclic exchange deliver it.
        scfg.brake_action = [drv] {
            auto* buf = static_cast<uint8_t*>(drv->getRxPDOBuffer());
            if (!buf) return false;
            auto* io = reinterpret_cast<SPdo::SOMANET_RxPDO_1601*>(
                buf + kBrakeOutputOffset);
            io->bit_mask = 1;
            io->physical_outputs = 0;   // drop the pin -> brake engages
            return true;
        };
        if (drv && !drv->controlledShutdown(scfg)) {
            TETHER_LOGW(TAG, "Slave {}: controlled shutdown reported errors",
                        slave_idx);
        }
    }
    master.stopMotionControlLoop();
    master.clearCyclicTasks();

    master.ethercatMaster().pdo().dumpStats(TAG);
    return rc;
}
