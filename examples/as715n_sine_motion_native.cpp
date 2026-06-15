#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <string>

#include "DS402ExampleSupport.hpp"
#include "tether/drives/AS715N/AS715NPDO.hpp"
#include "tether/platform/EspCompat.hpp"
#include "tether/profiles/cia301/CiA402Defs.hpp"

#include <argparse/argparse.hpp>

namespace {

constexpr const char* TAG = "as715n_sine";
constexpr uint16_t kSlaveIndex = 0;
constexpr double kTwoPi = 6.28318530717958647692;

int runSineMotion(EtherCAT::DS402Master& master, double duration_seconds)
{
    constexpr double kAmplitudeCountsPerSecond = 30000.0;
    constexpr double kFrequencyHz = 0.25;
    Control::SineMotionController::Config config = Control::SineMotionController::Config::getDefault();
    config.frequency = kFrequencyHz;
    config.amplitude = kAmplitudeCountsPerSecond / (kTwoPi * kFrequencyHz);

    if (!master.addSineMotionController<EtherCAT::Drives::AS715N_pdo::AS715N_RxPDO_1705>(
            kSlaveIndex,
            EtherCAT::DS402Master::CyclicTarget::Velocity,
            config)) {
        return 2;
    }

    EtherCAT::EtherCATMaster::RealtimeMotionLoopConfig loop_config;
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
    argparse::ArgumentParser program("as715n_sine_motion_native");
    program.add_argument("-i", "--interface").default_value(std::string("eth0"));
    program.add_argument("-d", "--duration").scan<'g', double>().default_value(10.0);

    try {
        program.parse_args(argc, argv);
    } catch (const std::runtime_error& err) {
        std::cerr << err.what() << '\n' << program;
        return 1;
    }

    EtherCAT::DS402Master master;
    Tether::Examples::HostMasterSession session;
    if (!Tether::Examples::startHostMasterSession(program.get<std::string>("--interface"), master, session, TAG)) {
        return 2;
    }

    int rc = 0;
    if (!configureDrive(master)) {
        rc = 3;
    } else {
        rc = runSineMotion(master, program.get<double>("--duration"));
        Tether::Examples::shutdownSingleDrive(master, kSlaveIndex);
    }

    Tether::Examples::stopHostMasterSession(master, session);
    return rc;
}