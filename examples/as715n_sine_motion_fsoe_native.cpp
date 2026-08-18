#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <memory>
#include <string>

#include "DS402ExampleSupport.hpp"
#include "tether/control/SineMotionController.hpp"
#include "tether/drives/AS715N/AS715NPDO.hpp"
#include "tether/fsoe/Synapticon/SafeMotionFSoE.hpp"
#include "tether/platform/EspCompat.hpp"
#include "tether/profiles/cia301/CiA402Defs.hpp"

#include <argparse/argparse.hpp>

namespace {

constexpr const char* TAG = "as715n_sine_fsoe";
constexpr uint16_t kSlaveIndex = 0;
constexpr double kTwoPi = 6.28318530717958647692;

using FSoEMain = EtherCAT::Drives::Synapticon::SafeMotion::MainInstance;
using FSoEServo = EtherCAT::Drives::Synapticon::SafeMotion::SafeMotionServoEmulator;
using FSoELoopFeature =
    EtherCAT::Drives::Synapticon::SafeMotion::MainLoopFeature<
        EtherCAT::Drives::AS715N_pdo::AS715N_RxPDO_1705>;

int runSineMotion(EtherCAT::DS402Master& master,
                  double duration_seconds,
                  bool enable_fsoe)
{
    constexpr double kAmplitudeCountsPerSecond = 30000.0;
    constexpr double kFrequencyHz = 0.25;
    tether::control::SineMotionController::Config config =
        tether::control::SineMotionController::Config::getDefault();
    config.frequency = kFrequencyHz;
    config.amplitude = kAmplitudeCountsPerSecond / (kTwoPi * kFrequencyHz);

    if (!master.addMotionController<EtherCAT::Drives::AS715N_pdo::AS715N_RxPDO_1705>(
            kSlaveIndex,
            EtherCAT::DS402Master::CyclicTarget::Velocity,
            std::make_unique<tether::control::SineMotionController>(config))) {
        return 2;
    }

    std::unique_ptr<FSoEMain> fsoe_main;
    std::unique_ptr<FSoEServo> fsoe_servo;
    if (enable_fsoe) {
        EtherCAT::Drives::Synapticon::SafeMotion::MainConfig main_config;
        main_config.feature_enabled = true;
        main_config.connection_id = 0x4321;
        main_config.watchdog_time_ms =
            EtherCAT::Drives::Synapticon::SafeMotion::Timing::kMinimumWatchdogTimeMs;

        EtherCAT::Drives::Synapticon::SafeMotion::ServoEmulatorConfig servo_config;
        servo_config.connection_id = main_config.connection_id;
        servo_config.watchdog_time_ms = main_config.watchdog_time_ms;

        fsoe_main = std::make_unique<FSoEMain>(main_config);
        fsoe_servo = std::make_unique<FSoEServo>(servo_config);
        if (!fsoe_main->initialize() || !fsoe_servo->initialize()) {
            (void)master.removeMotionController(kSlaveIndex);
            return 3;
        }

        fsoe_main->requestMotionEnabled();
        if (!master.addCyclicTask(
                std::make_unique<FSoELoopFeature>(kSlaveIndex, *fsoe_main, *fsoe_servo))) {
            (void)master.removeMotionController(kSlaveIndex);
            return 4;
        }
    }

    EtherCAT::Master::RealtimeMotionLoopConfig loop_config;
    loop_config.cycle_period_us = 1000;
    loop_config.sync_interval_cycles = 10;
    loop_config.enable_dc_synchronization = true;
    if (!master.startRealtimeMotionControlLoop(loop_config)) {
        master.clearCyclicTasks();
        (void)master.removeMotionController(kSlaveIndex);
        return 5;
    }

    Tether::Platform::Clock::instance().delayMilliseconds(
        static_cast<uint32_t>(duration_seconds * 1000.0));
    master.stopMotionControlLoop();
    master.clearCyclicTasks();
    (void)master.removeMotionController(kSlaveIndex);
    return 0;
}

bool configureDrive(EtherCAT::DS402Master& master)
{
    Tether::Examples::SingleDriveExampleConfig config;
    config.drive.slave_index = kSlaveIndex;
    config.drive.rxpdo_index = EtherCAT::Drives::AS715N_pdo::RxPDO_1705.index;
    config.drive.txpdo_index = EtherCAT::Drives::AS715N_pdo::TxPDO_1B04.index;
    config.drive.rxpdo_size = EtherCAT::Drives::AS715N_pdo::RxPDO_1705.size;
    config.drive.txpdo_size = EtherCAT::Drives::AS715N_pdo::TxPDO_1B04.size;
    config.drive.operating_mode = CiA402::OperatingMode::CyclicSyncVelocity;
    return Tether::Examples::configureAndEnableSingleDrive(master, config, TAG);
}

} // namespace

int main(int argc, char** argv)
{
    argparse::ArgumentParser program("as715n_sine_motion_fsoe_native");
    program.add_argument("-i", "--interface").default_value(std::string("eth0"));
    program.add_argument("-d", "--duration").scan<'g', double>().default_value(10.0);
    program.add_argument("--enable-fsoe").default_value(false).implicit_value(true);

    try {
        program.parse_args(argc, argv);
    } catch (const std::runtime_error& err) {
        std::cerr << err.what() << '\n' << program;
        return 1;
    }

    Tether::Platform::ensureRealtimeKernelOrExit();

    EtherCAT::DS402Master master;
    Tether::Examples::HostMasterSession session;
    if (!Tether::Examples::startHostMasterSession(
            program.get<std::string>("--interface"), master, session, TAG)) {
        return 2;
    }

    int rc = 0;
    if (!configureDrive(master)) {
        rc = 3;
    } else {
        rc = runSineMotion(master,
                           program.get<double>("--duration"),
                           program.get<bool>("--enable-fsoe"));
        Tether::Examples::shutdownSingleDrive(master, kSlaveIndex);
    }

    Tether::Examples::stopHostMasterSession(master, session);
    return rc;
}