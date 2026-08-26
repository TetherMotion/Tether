#include <array>
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

constexpr const char* TAG = "pblr_step";
constexpr uint16_t kSlaveIndex = 0;
constexpr double kCountsPerRevolution = 131072.0;

class StepDriveMotionController final : public EtherCAT::DS402Master::IDriveMotionController {
public:
    explicit StepDriveMotionController(double dwell_seconds)
        : dwell_seconds_(dwell_seconds)
    {
    }

    bool start(EtherCAT::CiA402Drive& drive) override {
        initialized_ = false;
        segment_index_ = 1;
        phase_time_ = 0.0;
        return drive.setOperatingMode(CiA402::OperatingMode::CyclicSyncPosition);
    }

    void stop(EtherCAT::CiA402Drive&) override {}

    bool update(EtherCAT::CiA402Drive& drive, double dt_seconds) override {
        auto* rx = drive.rxPDO<EtherCAT::Drives::PBLR81FGF::PBLR81FGF_RxPDO_1600>();
        const auto* tx = drive.txPDO<EtherCAT::Drives::PBLR81FGF::PBLR81FGF_TxPDO_1A00>();
        if (rx == nullptr || tx == nullptr) {
            return false;
        }

        if (!initialized_) {
            const int32_t origin = tx->position_actual;
            const double counts_per_degree = kCountsPerRevolution / 360.0;
            targets_ = {
                origin,
                origin + static_cast<int32_t>(std::lround(45.0 * counts_per_degree)),
                origin + static_cast<int32_t>(std::lround(90.0 * counts_per_degree)),
                origin + static_cast<int32_t>(std::lround(180.0 * counts_per_degree)),
                origin,
            };
            current_target_ = origin;
            initialized_ = true;
        }

        constexpr double kRampSeconds = 0.5;
        if (segment_index_ < targets_.size()) {
            phase_time_ += dt_seconds;
            const int32_t start_target = targets_[segment_index_ - 1];
            const int32_t next_target = targets_[segment_index_];

            if (phase_time_ < kRampSeconds) {
                const double alpha = phase_time_ / kRampSeconds;
                current_target_ = static_cast<int32_t>(std::lround(
                    (1.0 - alpha) * start_target + alpha * next_target));
            } else if (phase_time_ < (kRampSeconds + dwell_seconds_)) {
                current_target_ = next_target;
            } else {
                phase_time_ = 0.0;
                current_target_ = next_target;
                ++segment_index_;
            }
        }

        rx->controlword = static_cast<uint16_t>(CiA402::ControlWord::ENABLE_OPERATION);
        rx->modes_of_operation = CiA402::OperatingMode::CyclicSyncPosition;
        rx->target_position = current_target_;
        rx->target_velocity = 0;
        return true;
    }

private:
    double dwell_seconds_{0.5};
    bool initialized_{false};
    size_t segment_index_{1};
    double phase_time_{0.0};
    int32_t current_target_{0};
    std::array<int32_t, 5> targets_{};
};

int runStepMotion(EtherCAT::DS402Master& master, double dwell_seconds)
{
    if (!master.addMotionController(kSlaveIndex, std::make_unique<StepDriveMotionController>(dwell_seconds))) {
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
        static_cast<uint32_t>((dwell_seconds + 0.5) * 4.0 * 1000.0));
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
    argparse::ArgumentParser program("pblr81fgf_step_motion_native");
    program.add_argument("-i", "--interface")
        .default_value(std::string(""))
        .help("Network interface (e.g. eth0, enp3s0). "
              "If omitted, auto-selects the sole physical Ethernet interface.");
    program.add_argument("--dwell").scan<'g', double>().default_value(0.5);

    try {
        program.parse_args(argc, argv);
    } catch (const std::runtime_error& err) {
        std::cerr << err.what() << '\n' << program;
        return 1;
    }

    Tether::Platform::ensureRealtimeKernelOrExit();

    EtherCAT::DS402Master master;
    Tether::Examples::HostMasterSession session;
    if (!Tether::Examples::startHostMasterSession(Tether::Examples::resolveInterface(program.get<std::string>("--interface"), TAG), master, session, TAG)) {
        return 2;
    }

    int rc = 0;
    if (!configureDrive(master)) {
        rc = 3;
    } else {
        rc = runStepMotion(master, program.get<double>("--dwell"));
        Tether::Examples::shutdownSingleDrive(master, kSlaveIndex);
    }

    Tether::Examples::stopHostMasterSession(master, session);
    return rc;
}