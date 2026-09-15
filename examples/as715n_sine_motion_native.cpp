#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cctype>
#include <string>
#include <algorithm>

#include "DS402ExampleSupport.hpp"
#include "tether/control/SineMotionController.hpp"
#include "tether/drives/AS715N/AS715NDriveInitializer.hpp"
#include "tether/drives/AS715N/AS715NPDO.hpp"
#include "tether/ethercat/CoEManager.hpp"
#include "tether/ethercat/CoETypes.hpp"
#include "tether/platform/EspCompat.hpp"
#include "tether/profiles/cia301/CiA402Defs.hpp"

namespace {

constexpr const char* TAG = "as715n_sine";
constexpr uint16_t kSlaveIndex = 0;
constexpr double kTwoPi = 6.28318530717958647692;
constexpr double kFrequencyHz = 0.25;

// Peak amplitudes for each cyclic mode (drive-specific units)
constexpr double kPositionAmplitude = 30000.0;   // encoder counts
constexpr double kVelocityAmplitude = 30000.0;   // counts/s
constexpr double kTorqueAmplitude   = 1000.0;    // 0.1% of rated torque

using CyclicTarget = EtherCAT::DS402Master::CyclicTarget;

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

int runSineMotion(EtherCAT::DS402Master& master, CyclicTarget target, double duration_seconds)
{
    tether::control::SineMotionController::Config config =
        tether::control::SineMotionController::Config::getDefault();
    config.frequency = kFrequencyHz;

    switch (target) {
        case CyclicTarget::Position:
            config.amplitude = kPositionAmplitude;
            break;
        case CyclicTarget::Velocity:
            config.amplitude = kVelocityAmplitude / (kTwoPi * kFrequencyHz);
            break;
        case CyclicTarget::Torque:
            config.amplitude = kTorqueAmplitude;
            break;
    }

    EtherCAT::Master::RealtimeMotionLoopConfig loop_config;
    loop_config.cycle_period_us = 1000;
    loop_config.sync_interval_cycles = 10;
    loop_config.enable_dc_synchronization = true;
    // For CSP, set the current position as home before moving.
    if (target == CyclicTarget::Position) {
        auto* drive = master.driveBySlaveIndex(kSlaveIndex);
        if (!drive || !drive->homeToCurrentPosition()) {
            TETHER_LOGE(TAG, "Failed to set current position as home");
            return 5;
        }
    }

    if (!master.addMotionController<EtherCAT::Drives::AS715N_pdo::AS715N_RxPDO_1704>(
            kSlaveIndex,
            target,
            std::make_unique<tether::control::SineMotionController>(config))) {
        TETHER_LOGE(TAG, "Failed to add {} sine motion controller",
                    target == CyclicTarget::Position ? "CSP" :
                    (target == CyclicTarget::Velocity ? "CSV" : "CST"));
        return 4;
    }

    // Register the controller before starting the realtime loop.  The loop
    // invokes updateMotionControllers() immediately and motion_controllers_
    // is not safe to modify concurrently with that update.
    if (!master.startRealtimeMotionControlLoop(loop_config)) {
        TETHER_LOGE(TAG, "Failed to start realtime motion control loop");
        (void)master.removeMotionController(kSlaveIndex);
        return 3;
    }

    Tether::Platform::Clock::instance().delayMilliseconds(
        static_cast<uint32_t>(duration_seconds * 1000.0));
    master.stopMotionControlLoop();
    (void)master.removeMotionController(kSlaveIndex);
    return 0;
}

bool configureDrive(EtherCAT::DS402Master& master)
{
    EtherCAT::Drives::AS715N::AS715NDriveInitializer init(master, kSlaveIndex, TAG);

    // Use RxPDO 0x1704/TxPDO 0x1B04 because 0x1704 carries TargetTorque,
    // TargetPosition and TargetVelocity, so one mapping serves CSP, CSV and CST.
    const auto assignment = EtherCAT::Drives::AS715N_pdo::makePDOAssignment(
        EtherCAT::Drives::AS715N_pdo::RxPDO_1704,
        EtherCAT::Drives::AS715N_pdo::TxPDO_1B04);

    if (!init.init(assignment)) {
        TETHER_LOGE(TAG, "AS715N drive initialization failed");
        return false;
    }

    if (!init.enableDrive()) {
        TETHER_LOGE(TAG, "Failed to enable drive");
        return false;
    }

    return true;
}

void readAndPrint2006_08(EtherCAT::DS402Master& master)
{
    auto& coe = master.ethercatMaster().sdoManager(kSlaveIndex);
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

    const uint16_t minimum_drive_count = static_cast<uint16_t>(kSlaveIndex + 1);
    if (!master.waitForDriveCount(minimum_drive_count, 2000)) {
        TETHER_LOGE(TAG, "Timed out waiting for {} drive(s)", minimum_drive_count);
        Tether::Examples::stopHostMasterSession(master, session);
        return 2;
    }

    {
        EtherCAT::DC::DCConfig dc_config = EtherCAT::DC::DCConfig::defaults();
        if (!master.initializeDistributedClocks(dc_config)) {
            TETHER_LOGE(TAG, "Failed to initialize distributed clocks");
            Tether::Examples::stopHostMasterSession(master, session);
            return 2;
        }
        if (!master.startDistributedClocks()) {
            TETHER_LOGE(TAG, "Failed to start distributed clocks");
            Tether::Examples::stopHostMasterSession(master, session);
            return 2;
        }
    }

    int rc = 0;
    if (!configureDrive(master)) {
        rc = 3;
    } else {
        readAndPrint2006_08(master);
        rc = runSineMotion(master, target, args.duration);
        Tether::Examples::shutdownSingleDrive(master, kSlaveIndex);
    }

    Tether::Examples::stopHostMasterSession(master, session);
    return rc;
}
