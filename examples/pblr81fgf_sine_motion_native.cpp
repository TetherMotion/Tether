#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <string>

#include "DS402ExampleSupport.hpp"
#include "tether/drives/PBLR81FGF/PBLR81FGFPDO.hpp"
#include "tether/platform/EspCompat.hpp"
#include "tether/profiles/cia301/CiA402Defs.hpp"

#include <argparse/argparse.hpp>

namespace {

constexpr const char* TAG = "pblr_sine";
constexpr uint16_t kSlaveIndex = 0;
constexpr double kCountsPerRevolution = 131072.0;

int runSineMotion(EtherCAT::DS402Master& master, double duration_seconds)
{
    constexpr double kAmplitudeDegrees = 90.0;
    constexpr double kFrequencyHz = 0.2;
    Control::SineMotionController::Config config = Control::SineMotionController::Config::getDefault();
    config.frequency = kFrequencyHz;
    config.amplitude = kAmplitudeDegrees;

    if (!master.addSineMotionController<EtherCAT::Drives::PBLR81FGF::PBLR81FGF_RxPDO_1600>(
            kSlaveIndex,
            EtherCAT::DS402Master::CyclicTarget::Position,
            config,
            kCountsPerRevolution / 360.0)) {
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
    config.drive.rxpdo_index = EtherCAT::Drives::PBLR81FGF::RxPDO_1600.index;
    config.drive.txpdo_index = EtherCAT::Drives::PBLR81FGF::TxPDO_1A00.index;
    config.drive.rxpdo_size = EtherCAT::Drives::PBLR81FGF::RxPDO_1600.size;
    config.drive.txpdo_size = EtherCAT::Drives::PBLR81FGF::TxPDO_1A00.size;
    config.drive.operating_mode = CiA402::OperatingMode::CyclicSyncPosition;
    return Tether::Examples::configureAndEnableSingleDrive(master, config, TAG);
}

} // namespace

int main(int argc, char** argv)
{
    argparse::ArgumentParser program("pblr81fgf_sine_motion_native");
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