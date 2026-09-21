#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cctype>
#include <cstdio>
#include <atomic>
#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include <algorithm>

#include "DS402ExampleSupport.hpp"
#include "tether/control/SineMotionController.hpp"
#include "tether/drives/AS715N.hpp"
#include "tether/drives/AS715N/AS715NDriveInitializer.hpp"
#include "tether/drives/AS715N/AS715NPDO.hpp"
#include "tether/ethercat/CoEManager.hpp"
#include "tether/ethercat/CoETypes.hpp"
#include "tether/platform/EspCompat.hpp"
#include "tether/profiles/cia301/CiA402Defs.hpp"

namespace {

constexpr const char* TAG = "as715n_sine";
constexpr double kTwoPi = 6.28318530717958647692;
constexpr double kFrequencyHz = 0.25;

// Peak amplitudes for each cyclic mode (drive-specific units)
constexpr double kPositionAmplitude = 30000.0;   // encoder counts
constexpr double kVelocityAmplitude = 30000.0;   // counts/s
constexpr double kTorqueAmplitude   = 1000.0;    // 0.1% of rated torque

using CyclicTarget = EtherCAT::DS402Master::CyclicTarget;

/// Wraps a motion controller and writes the full Rx/Tx PDO contents to a
/// CSV file once per control cycle.  Uses a 1 MiB output buffer so the
/// realtime loop is not blocked by the host filesystem.  This example
/// targets the AS715N 0x1704 / 0x1B04 mapping, but the wrapper is generic.
template<typename RxPDO, typename TxPDO>
class CsvLogController : public EtherCAT::DS402Master::IDriveMotionController {
public:
    CsvLogController(std::unique_ptr<EtherCAT::DS402Master::IDriveMotionController> inner,
                     std::string csv_path)
        : inner_(std::move(inner))
        , csv_path_(std::move(csv_path))
    {
    }

    ~CsvLogController()
    {
        if (file_) {
            std::fflush(file_);
            std::fclose(file_);
        }
    }

    bool start(EtherCAT::CiA402Drive& drive) override
    {
        file_ = std::fopen(csv_path_.c_str(), "w");
        if (!file_) {
            TETHER_LOGE(TAG, "Failed to open CSV file '{}'; continuing without trace",
                        csv_path_);
        } else {
            std::setvbuf(file_, nullptr, _IOFBF, 1 << 20);  // 1 MiB buffer
            start_time_ = std::chrono::steady_clock::now();
            std::fprintf(file_,
                         "t,"
                         "rx_controlword,rx_target_position,rx_target_velocity,rx_target_torque,"
                         "rx_modes_of_operation,rx_touch_probe_function,rx_max_profile_velocity,"
                         "rx_positive_torque_limit,rx_negative_torque_limit,"
                         "tx_error_code,tx_statusword,tx_position_actual,tx_torque_actual,"
                         "tx_modes_of_operation_display,tx_position_deviation,tx_touch_probe_status,"
                         "tx_touch_probe_pos1,tx_touch_probe_pos2,tx_speed_feedback\n");
        }
        return inner_->start(drive);
    }

    void stop(EtherCAT::CiA402Drive& drive) override
    {
        inner_->stop(drive);
        if (file_) {
            std::fflush(file_);
            std::fclose(file_);
            file_ = nullptr;
        }
    }

    bool update(EtherCAT::CiA402Drive& drive, double dt_seconds) override
    {
        bool ok = inner_->update(drive, dt_seconds);
        if (file_ && start_time_ != std::chrono::steady_clock::time_point{}) {
            const double t = std::chrono::duration<double>(
                std::chrono::steady_clock::now() - start_time_).count();
            const auto* rx = drive.rxPDO<RxPDO>();
            const auto* tx = drive.txPDO<TxPDO>();
            if (rx && tx) {
                std::fprintf(file_,
                             "%.6f,%u,%ld,%ld,%d,%d,%u,%lu,%u,%u,"
                             "%u,%u,%ld,%d,%d,%ld,%u,%ld,%ld,%ld\n",
                             t,
                             static_cast<unsigned>(rx->controlword),
                             static_cast<long>(rx->target_position),
                             static_cast<long>(rx->target_velocity),
                             static_cast<int>(rx->target_torque),
                             static_cast<int>(rx->modes_of_operation),
                             static_cast<unsigned>(rx->touch_probe_function),
                             static_cast<unsigned long>(rx->max_profile_velocity),
                             static_cast<unsigned>(rx->positive_torque_limit),
                             static_cast<unsigned>(rx->negative_torque_limit),
                             static_cast<unsigned>(tx->error_code),
                             static_cast<unsigned>(tx->statusword),
                             static_cast<long>(tx->position_actual),
                             static_cast<int>(tx->torque_actual),
                             static_cast<int>(tx->modes_of_operation_display),
                             static_cast<long>(tx->position_deviation),
                             static_cast<unsigned>(tx->touch_probe_status),
                             static_cast<long>(tx->touch_probe_pos1),
                             static_cast<long>(tx->touch_probe_pos2),
                             static_cast<long>(tx->speed_feedback));
            }
        }
        return ok;
    }

private:
    std::unique_ptr<EtherCAT::DS402Master::IDriveMotionController> inner_;
    std::string csv_path_;
    std::FILE* file_ = nullptr;
    std::chrono::steady_clock::time_point start_time_;
};

CyclicTarget modeToTarget(const std::string& mode)
{
    std::string m = mode;
    std::transform(m.begin(), m.end(), m.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    if (m == "csp") return CyclicTarget::Position;
    if (m == "csv") return CyclicTarget::Velocity;
    if (m == "cst") return CyclicTarget::Torque;
    TETHER_LOGW(TAG, "Unknown mode '{}'; defaulting to CSV", mode);
    return CyclicTarget::Velocity;
}

/// Polls the AS715N fault registers via CoE once per second while the motion
/// loop runs.  Prints a report only when the statusword fault bit is set (or
/// the SDO read itself fails), de-duplicated so a persistent fault is reported
/// once per value change rather than once per second.  Silent while healthy.
void faultMonitorLoop(EtherCAT::DS402Master& master,
                      uint16_t slave_index,
                      const std::atomic<bool>& stop)
{
    auto& sdo = master.ethercatMaster().sdoManager(slave_index);
    const EtherCAT::CoE::CoETransactionOptions options{.timeout_ms = 1000};
    uint32_t last_reported = 0xFFFFFFFFu;  // (sw<<16)|mfr_ext of last report
    bool fault_active = false;
    while (!stop.load(std::memory_order_relaxed)) {
        auto sw = sdo.readU16(0x6041, 0x00, options);
        if (!sw.has_value()) {
            TETHER_LOGW(TAG, "Fault monitor: statusword SDO read failed");
        } else if (*sw & 0x0008) {
            using EtherCAT::Drives::AS715NFaultHandler;
            const auto mfr =
                AS715NFaultHandler::readManufacturerFaultExtended(sdo, slave_index);
            const uint16_t cia =
                AS715NFaultHandler::readCiA402Error(sdo, slave_index);
            const uint32_t key = (static_cast<uint32_t>(*sw) << 16)
                               | mfr.external_code;
            if (key != last_reported) {
                last_reported = key;
                const auto err = EtherCAT::Drives::AS715NError::parse(
                    mfr.external_code);
                TETHER_LOGE(TAG,
                            "DRIVE FAULT: statusword=0x{:04X} "
                            "0x203F ext=0x{:04X} int=0x{:04X} ({} \"{}\") "
                            "0x603F=0x{:04X}",
                            *sw, mfr.external_code, mfr.internal_code, err.name,
                            err.description ? err.description : "(none)", cia);
            }
            fault_active = true;
        } else if (fault_active) {
            fault_active = false;
            last_reported = 0xFFFFFFFFu;
            TETHER_LOGI(TAG, "Fault monitor: fault cleared (statusword=0x{:04X})",
                        *sw);
        }
        for (int i = 0; i < 10 && !stop.load(std::memory_order_relaxed); ++i) {
            Tether::Platform::Clock::instance().delayMilliseconds(100);
        }
    }
}

int runSineMotion(EtherCAT::DS402Master& master,
                  uint16_t slave_index,
                  const Tether::Examples::MotionNativeArgs& args,
                  CyclicTarget target)
{
    tether::control::SineMotionController::Config config =
        tether::control::SineMotionController::Config::getDefault();
    config.frequency = args.frequency_hz;

    switch (target) {
        case CyclicTarget::Position:
            config.amplitude = args.position_amplitude;
            break;
        case CyclicTarget::Velocity:
            config.amplitude = args.velocity_amplitude / (kTwoPi * args.frequency_hz);
            break;
        case CyclicTarget::Torque:
            config.amplitude = args.torque_amplitude;
            break;
    }

    // Start the CoE fault monitor before homing/motion so drive faults are
    // reported during the whole run, not only during cyclic motion.
    std::atomic<bool> monitor_stop{false};
    std::thread fault_monitor(faultMonitorLoop, std::ref(master),
                              slave_index, std::cref(monitor_stop));

    // Deadline-driven fast loop (CyclicExecutive): the exchange runs via the
    // reserved-slot cyclic datapath — kernel ring backend when available —
    // and DC sync is emitted by the executive's own dedicated thread, so no
    // separate startDistributedClocks() loop is needed.  Split placement
    // overlaps the wire round-trip with the phases between send and collect.
    EtherCAT::Master::CyclicLoopConfig loop_config;
    loop_config.cycle_period_us = 1000;
    loop_config.sync_interval_cycles = 10;
    loop_config.enable_dc_synchronization = true;
    loop_config.exchange_placement =
        EtherCAT::Master::ExchangePlacement::Split;
    loop_config.cpu_isolation.enabled = true;   // runtime opt-in; logs and
                                                // degrades when unavailable
    // For CSP, set the current position as home before moving.
    if (target == CyclicTarget::Position) {
        auto* drive = master.driveBySlaveIndex(slave_index);
        if (!drive || !drive->homeToCurrentPosition()) {
            TETHER_LOGE(TAG, "Failed to set current position as home");
            monitor_stop.store(true);
            fault_monitor.join();
            return 5;
        }
    }

    // RxPDO 0x1704 carries torque limits and max profile velocity in the
    // cyclic frame.  The motion controller only writes controlword, mode and
    // the active setpoint, so these fields must be initialised once here —
    // otherwise the drive clamps output torque (and profile speed) to zero.
    if (auto* drive = master.driveBySlaveIndex(slave_index)) {
        if (auto* rx = drive->rxPDO<EtherCAT::Drives::AS715N_pdo::AS715N_RxPDO_1704>()) {
            rx->positive_torque_limit = 1000;  // 100.0% of rated torque
            rx->negative_torque_limit = 1000;  // 100.0% of rated torque
            rx->max_profile_velocity  = 100000;  // counts/s
        }
    }

    using RxPDO = EtherCAT::Drives::AS715N_pdo::AS715N_RxPDO_1704;
    using TxPDO = EtherCAT::Drives::AS715N_pdo::AS715N_TxPDO_1B04;
    auto sine = std::make_unique<tether::control::SineMotionController>(config);
    auto inner = std::make_unique<EtherCAT::DS402Master::GenericDriveMotionController<RxPDO>>(
        target, std::move(sine), 1.0);

    std::unique_ptr<EtherCAT::DS402Master::IDriveMotionController> controller;
    if (!args.csv_path.empty()) {
        controller = std::make_unique<CsvLogController<RxPDO, TxPDO>>(
            std::move(inner), args.csv_path);
    } else {
        controller = std::move(inner);
    }

    if (!master.addMotionController(slave_index, std::move(controller))) {
        TETHER_LOGE(TAG, "Failed to add {} sine motion controller",
                    target == CyclicTarget::Position ? "CSP" :
                    (target == CyclicTarget::Velocity ? "CSV" : "CST"));
        return 4;
    }

    // Register the controller before starting the realtime loop.  The loop
    // invokes updateMotionControllers() immediately and motion_controllers_
    // is not safe to modify concurrently with that update.
    if (!master.startCyclicLoop(loop_config)) {
        TETHER_LOGE(TAG, "Failed to start cyclic loop");
        monitor_stop.store(true);
        fault_monitor.join();
        (void)master.removeMotionController(slave_index);
        return 3;
    }

    Tether::Platform::Clock::instance().delayMilliseconds(
        static_cast<uint32_t>(args.duration * 1000.0));
    master.stopCyclicLoop();
    monitor_stop.store(true);
    fault_monitor.join();
    (void)master.removeMotionController(slave_index);
    return 0;
}

/// Reads the drive's actual 0x1B04 TxPDO mapping via SDO and checks that the
/// statusword (0x6041) sits at byte offset 2 — the position the wire logger
/// and CiA402Drive decode it from.  The mapping is never rewritten by
/// configureMultiPDOs(), so this verifies the drive's factory layout.
bool verifyTxPDOMapping(EtherCAT::DS402Master& master, uint16_t slave_index)
{
    auto& sdo = master.ethercatMaster().sdoManager(slave_index);
    const EtherCAT::CoE::CoETransactionOptions options{.timeout_ms = 1000};

    const auto count = sdo.readU8(0x1B04, 0x00, options);
    if (!count.has_value() || *count == 0) {
        TETHER_LOGE(TAG, "Failed to read 0x1B04 mapping count");
        return false;
    }

    uint32_t byte_offset = 0;
    bool statusword_ok = false;
    for (uint8_t sub = 1; sub <= *count; ++sub) {
        const auto e = sdo.readU32(0x1B04, sub, options);
        if (!e.has_value()) {
            TETHER_LOGE(TAG, "Failed to read 0x1B04:{} mapping entry", sub);
            return false;
        }
        const uint16_t idx  = static_cast<uint16_t>(*e >> 16);
        const uint8_t  subi = static_cast<uint8_t>((*e >> 8) & 0xFF);
        const uint8_t  bits = static_cast<uint8_t>(*e & 0xFF);
        TETHER_LOGI(TAG, "0x1B04[{}]: 0x{:04X}:{:02X} {} bits @ byte {}",
                    sub, idx, subi, bits, byte_offset);
        if (idx == 0x6041) {
            statusword_ok = (byte_offset == 2 && bits == 16);
        }
        byte_offset += bits / 8;
    }

    if (!statusword_ok) {
        TETHER_LOGE(TAG, "0x6041 NOT at TxPDO byte 2 — decoded statusword "
                         "values are unreliable!");
        return false;
    }
    TETHER_LOGI(TAG, "TxPDO 0x1B04 mapping verified: statusword at byte 2");
    return true;
}

bool configureDrive(EtherCAT::DS402Master& master, uint16_t slave_index)
{
    EtherCAT::Drives::AS715N::AS715NDriveInitializer init(master, slave_index, TAG);

    // Use RxPDO 0x1704/TxPDO 0x1B04 because 0x1704 carries TargetTorque,
    // TargetPosition and TargetVelocity, so one mapping serves CSP, CSV and CST.
    const auto assignment = EtherCAT::Drives::AS715N_pdo::makePDOAssignment(
        EtherCAT::Drives::AS715N_pdo::RxPDO_1704,
        EtherCAT::Drives::AS715N_pdo::TxPDO_1B04);

    if (!init.init(assignment)) {
        TETHER_LOGE(TAG, "AS715N drive initialization failed");
        return false;
    }

    if (!verifyTxPDOMapping(master, slave_index)) {
        TETHER_LOGE(TAG, "TxPDO 0x1B04 mapping verification failed");
        return false;
    }

    // The AS715N does not clear faults via the CiA402 controlword reset bit —
    // the A6-EC manual requires S-ON cleared first, then a fault reset via
    // F31.00 (0x2031:01), with a dedicated sequence for DC sync errors.
    {
        using EtherCAT::Drives::AS715NError;
        using EtherCAT::Drives::AS715NFaultHandler;
        auto& sdo = master.ethercatMaster().sdoManager(slave_index);
        uint16_t mfr_error = 0, cia402_error = 0;
        if (AS715NFaultHandler::checkFault(sdo, slave_index, &mfr_error, &cia402_error)) {
            auto cw = sdo.readU16(0x6040, 0x00, {.timeout_ms = 3000});
            if (cw.has_value() && (*cw & 0x0001u)) {
                (void)sdo.writeU16(0x6040, 0x00,
                                   static_cast<uint16_t>(*cw & ~0x0001u),
                                   {.timeout_ms = 3000});
                Tether::Platform::Clock::instance().delayMilliseconds(50);
            }
            const auto err = AS715NError::parse(mfr_error);
            const bool cleared = err.isDCSyncError()
                ? AS715NFaultHandler::handleNoSyncError(sdo, slave_index, 3)
                : AS715NFaultHandler::resetFault(sdo, slave_index);
            if (!cleared) {
                TETHER_LOGE(TAG, "Fault reset failed (mfr=0x{:04X} cia=0x{:04X})",
                            mfr_error, cia402_error);
                return false;
            }
            TETHER_LOGI(TAG, "Fault cleared — proceeding with enable");
        }
    }

    if (!init.enableDrive()) {
        TETHER_LOGE(TAG, "Failed to enable drive");
        return false;
    }

    return true;
}

void readAndPrint2006_08(EtherCAT::DS402Master& master, uint16_t slave_index)
{
    auto& coe = master.ethercatMaster().sdoManager(slave_index);
    EtherCAT::CoE::CoETransactionOptions options;
    options.timeout_ms = 1000;
    options.max_retries = 3;

    uint8_t buf[8] = {0};
    size_t len = 0;
    if (!coe.sdoUploadWithRetry(0x2006u, 0x08u, buf, sizeof(buf), &len, options)) {
        TETHER_LOGE(TAG, "Failed to read 0x2006.08");
        return;
    }

    if (len == 0) {
        TETHER_LOGI(TAG, "0x2006.08 = <empty>");
    } else if (len == 1) {
        TETHER_LOGI(TAG, "0x2006.08 = 0x{:02X} ({})", buf[0], buf[0]);
    } else if (len == 2) {
        const uint16_t v = static_cast<uint16_t>(buf[0] | (buf[1] << 8));
        TETHER_LOGI(TAG, "0x2006.08 = 0x{:04X} ({})", v, v);
    } else if (len <= 4) {
        const uint32_t v = static_cast<uint32_t>(buf[0] | (buf[1] << 8) | (buf[2] << 16) | (buf[3] << 24));
        TETHER_LOGI(TAG, "0x2006.08 = 0x{:08X} ({})", v, v);
    } else {
        TETHER_LOGI(TAG, "0x2006.08 = {} bytes: {:02X}{:02X}{:02X}{:02X}...",
                    len, buf[0], buf[1], buf[2], buf[3]);
    }
}

} // namespace

int main(int argc, char** argv)
{
    Tether::Examples::MotionNativeArgs args;
    if (!Tether::Examples::parseMotionNativeArgs(argc, argv, "as715n_sine_motion_native", args)) {
        return 1;
    }

    Tether::Platform::ensureRealtimeKernelOrExit();

    const auto target = modeToTarget(args.mode);

    EtherCAT::DS402Master master;
    Tether::Examples::HostMasterSession session;
    if (!Tether::Examples::startHostMasterSession(args.interface, master, session, TAG, args.vlan)) {
        return 2;
    }

    if (master.ethercatMaster().discovery().discover(EtherCAT::DiscoveryOptions()).empty()) {
        TETHER_LOGW(TAG, "No slaves discovered");
    }

    const uint16_t slave_index = static_cast<uint16_t>(args.slave_index);
    const uint16_t minimum_drive_count = static_cast<uint16_t>(slave_index + 1);
    if (!master.waitForDriveCount(minimum_drive_count, 2000)) {
        TETHER_LOGE(TAG, "Timed out waiting for {} drive(s)", minimum_drive_count);
        Tether::Examples::stopHostMasterSession(master, session);
        return 2;
    }

    // initializeDistributedClocks() arms the slaves' sync units; the cyclic
    // loop's dedicated DC task then emits the sync frames — the legacy
    // startDistributedClocks() realtime loop would compete with the cyclic
    // exchange on the wire and is therefore not started.
    {
        EtherCAT::DC::DCConfig dc_config = EtherCAT::DC::DCConfig::defaults();
        if (!master.initializeDistributedClocks(dc_config)) {
            TETHER_LOGE(TAG, "Failed to initialize distributed clocks");
            Tether::Examples::stopHostMasterSession(master, session);
            return 2;
        }
    }

    int rc = 0;
    if (!configureDrive(master, slave_index)) {
        rc = 3;
    } else {
        readAndPrint2006_08(master, slave_index);
        rc = runSineMotion(master, slave_index, args, target);
        Tether::Examples::shutdownSingleDrive(master, slave_index);
    }

    Tether::Examples::stopHostMasterSession(master, session);
    return rc;
}
