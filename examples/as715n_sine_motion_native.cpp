#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <string>

#include "DS402ExampleSupport.hpp"
#include "tether/control/SineMotionController.hpp"
#include "tether/drives/AS715N/AS715NDriveInitializer.hpp"
#include "tether/drives/AS715N/AS715NPDO.hpp"
#include "tether/platform/EspCompat.hpp"
#include "tether/profiles/cia301/CiA402Defs.hpp"

namespace {

constexpr const char* TAG = "as715n_sine";
constexpr uint16_t kSlaveIndex = 0;
constexpr double kTwoPi = 6.28318530717958647692;

int runSineMotion(EtherCAT::DS402Master& master, double duration_seconds)
{
    constexpr double kAmplitudeCountsPerSecond = 30000.0;
    constexpr double kFrequencyHz = 0.25;
    tether::control::SineMotionController::Config config = tether::control::SineMotionController::Config::getDefault();
    config.frequency = kFrequencyHz;
    config.amplitude = kAmplitudeCountsPerSecond / (kTwoPi * kFrequencyHz);

    if (!master.addMotionController<EtherCAT::Drives::AS715N_pdo::AS715N_RxPDO_1705>(
            kSlaveIndex,
            EtherCAT::DS402Master::CyclicTarget::Velocity,
            std::make_unique<tether::control::SineMotionController>(config))) {
        return 2;
    }

    EtherCAT::Master::RealtimeMotionLoopConfig loop_config;
    loop_config.cycle_period_us = 1000;
    loop_config.sync_interval_cycles = 10;
    loop_config.enable_dc_synchronization = true;
    if (!master.startRealtimeMotionControlLoop(loop_config)) {
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

    if (!init.init()) {
        TETHER_LOGE(TAG, "AS715N drive initialization failed");
        return false;
    }

    if (!init.drive().setOperatingMode(CiA402::OperatingMode::CyclicSyncVelocity)) {
        TETHER_LOGE(TAG, "Failed to set Cyclic Sync Velocity mode");
        return false;
    }

    if (!init.enableDrive()) {
        TETHER_LOGE(TAG, "Failed to enable drive");
        return false;
    }

    return true;
}

} // namespace

int main(int argc, char** argv)
{
    Tether::Examples::MotionNativeArgs args;
    if (!Tether::Examples::parseMotionNativeArgs(argc, argv, "as715n_sine_motion_native", args)) {
        return 1;
    }

    Tether::Platform::ensureRealtimeKernelOrExit();

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
        rc = runSineMotion(master, args.duration);
        Tether::Examples::shutdownSingleDrive(master, kSlaveIndex);
    }

    Tether::Examples::stopHostMasterSession(master, session);
    return rc;
}
