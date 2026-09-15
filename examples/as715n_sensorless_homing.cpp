#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <limits>
#include <string>

#include "DS402ExampleSupport.hpp"
#include "tether/control/PIDControllers.hpp"
#include "tether/drives/AS715N/AS715NPDO.hpp"
#include "tether/platform/EspCompat.hpp"
#include "tether/platform/Platform.hpp"
#include "tether/profiles/cia301/CiA402Defs.hpp"

namespace {

constexpr const char* TAG = "as715n_homing";

struct SensorlessHomingArgs {
    std::string interface;
    int slave = 0;
    double target_velocity = 2000.0;   // counts/s, magnitude
    int direction = 1;                 // +1 or -1
    double max_torque_percent = 1.0;   // % of rated
    double kp = 0.05;
    double ki = 0.005;
    double stall_velocity = 100.0;     // counts/s, below = not moving
    double stall_time = 1.0;           // seconds below threshold before homing
    double max_runtime = 60.0;         // seconds
    uint32_t homing_timeout_ms = 10000;
    Tether::Examples::VlanConfig vlan;
};

inline bool parseSensorlessHomingArgs(int argc, char** argv,
                                      const char* program_name,
                                      SensorlessHomingArgs& out)
{
    argparse::ArgumentParser program(program_name, "1.0", argparse::default_arguments::help);
    Tether::Examples::addInterfaceArg(program);
    Tether::Examples::addSlaveArg(program, 0);

    program.add_argument("--target-velocity")
        .scan<'g', double>()
        .default_value(2000.0)
        .help("target velocity setpoint magnitude in counts/s");
    program.add_argument("--direction")
        .scan<'i', int>()
        .default_value(1)
        .help("direction: +1 or -1");
    program.add_argument("--max-torque")
        .scan<'g', double>()
        .default_value(1.0)
        .help("maximum torque in percent of rated (e.g. 1.0 = 1 pct)");
    program.add_argument("--kp")
        .scan<'g', double>()
        .default_value(0.05)
        .help("velocity loop proportional gain");
    program.add_argument("--ki")
        .scan<'g', double>()
        .default_value(0.005)
        .help("velocity loop integral gain");
    program.add_argument("--stall-velocity")
        .scan<'g', double>()
        .default_value(100.0)
        .help("speed magnitude below which the drive is considered stalled (counts/s)");
    program.add_argument("--stall-time")
        .scan<'g', double>()
        .default_value(1.0)
        .help("time the speed must stay below stall-velocity before homing (s)");
    program.add_argument("--max-runtime")
        .scan<'g', double>()
        .default_value(60.0)
        .help("maximum time to wait for a stall before aborting (s)");
    program.add_argument("--homing-timeout")
        .scan<'i', int>()
        .default_value(10000)
        .help("timeout for the drive's current-position homing routine (ms)");

    try {
        program.parse_args(argc, argv);
    } catch (const std::runtime_error& err) {
        std::cerr << err.what() << '\n' << program;
        return false;
    }

    out.interface = Tether::Examples::resolveInterface(
        program.get<std::string>("--interface"), program_name);
    if (out.interface.empty()) {
        return false;
    }
    out.slave = program.get<int>("--slave");
    out.target_velocity = program.get<double>("--target-velocity");
    out.direction = program.get<int>("--direction");
    if (out.direction != 1 && out.direction != -1) {
        std::cerr << "--direction must be +1 or -1\n";
        return false;
    }
    out.max_torque_percent = program.get<double>("--max-torque");
    if (out.max_torque_percent <= 0.0) {
        std::cerr << "--max-torque must be positive\n";
        return false;
    }
    out.kp = program.get<double>("--kp");
    out.ki = program.get<double>("--ki");
    out.stall_velocity = program.get<double>("--stall-velocity");
    out.stall_time = program.get<double>("--stall-time");
    out.max_runtime = program.get<double>("--max-runtime");
    out.homing_timeout_ms = static_cast<uint32_t>(program.get<int>("--homing-timeout"));
    if (!Tether::Examples::parseVlanArgs(
            program.get<std::string>("--rx-vlan"),
            program.get<std::string>("--tx-vlan"),
            out.vlan, program_name)) {
        return false;
    }
    Tether::Examples::logVlanConfig(out.vlan, program_name);
    return true;
}

class SensorlessHomingController : public EtherCAT::DS402Master::IDriveMotionController {
public:
    SensorlessHomingController(const SensorlessHomingArgs& args)
        : args_(args)
        , reference_velocity_(args.direction * args.target_velocity)
        , max_permille_(std::min(1000.0, args.max_torque_percent * 10.0))
    {
    }

    bool start(EtherCAT::CiA402Drive& drive) override
    {
        if (drive.rxPDO<EtherCAT::Drives::AS715N_pdo::AS715N_RxPDO_1702>() == nullptr) {
            TETHER_LOGE(TAG, "RxPDO 0x1702 not mapped for slave {}", args_.slave);
            return false;
        }
        if (drive.txPDO<EtherCAT::Drives::AS715N_pdo::AS715N_TxPDO_1B04>() == nullptr) {
            TETHER_LOGE(TAG, "TxPDO 0x1B04 not mapped for slave {}", args_.slave);
            return false;
        }

        if (!drive.setModeCST()) {
            TETHER_LOGE(TAG, "Failed to set slave {} to CST mode", args_.slave);
            return false;
        }

        pi_.setGains(args_.kp, args_.ki);
        pi_.setIntegralLimits(-max_permille_, max_permille_);
        pi_.setSaturationLimits({-max_permille_, max_permille_, -max_permille_, max_permille_,
                                 -std::numeric_limits<double>::max(),
                                 std::numeric_limits<double>::max(),
                                 std::numeric_limits<double>::max()});
        pi_.setAntiWindup(tether::control::AntiWindupMethod::Clamping, 0.0);
        pi_.reset();

        TETHER_LOGI(TAG,
                    "Sensorless homing started on slave {}: reference={} counts/s, max_torque={}%",
                    args_.slave, reference_velocity_, args_.max_torque_percent);
        return true;
    }

    void stop(EtherCAT::CiA402Drive&) override
    {
    }

    bool update(EtherCAT::CiA402Drive& drive, double dt_seconds) override
    {
        auto* rx = drive.rxPDO<EtherCAT::Drives::AS715N_pdo::AS715N_RxPDO_1702>();
        auto* tx = drive.txPDO<EtherCAT::Drives::AS715N_pdo::AS715N_TxPDO_1B04>();
        if (rx == nullptr || tx == nullptr) {
            return false;
        }

        rx->controlword = static_cast<uint16_t>(CiA402::ControlWord::ENABLE_OPERATION);
        rx->modes_of_operation = CiA402::OperatingMode::CyclicSyncTorque;
        rx->max_profile_velocity = 100000u;
        rx->target_position = 0;
        rx->target_velocity = 0;
        rx->touch_probe_function = 0;

        if (homed_.load(std::memory_order_acquire)) {
            rx->target_torque = 0;
            return true;
        }

        const double speed = static_cast<double>(tx->speed_feedback);
        if (!moved_ && std::abs(speed) > args_.stall_velocity) {
            moved_ = true;
            TETHER_LOGI(TAG, "Drive started moving: speed={} counts/s", speed);
        }

        if (moved_) {
            if (std::abs(speed) <= args_.stall_velocity) {
                stall_timer_ += dt_seconds;
                if (stall_timer_ >= args_.stall_time) {
                    TETHER_LOGI(TAG, "Stall detected (speed={} counts/s, time={} s); homing", speed, stall_timer_);
                    homed_.store(true, std::memory_order_release);
                    rx->target_torque = 0;
                    return true;
                }
            } else {
                stall_timer_ = 0.0;
            }
        }

        tether::control::ControllerInput input;
        input.reference = reference_velocity_;
        input.measured = speed;
        input.dt = dt_seconds;
        input.enable = true;

        const auto output = pi_.compute(input);
        rx->target_torque = static_cast<int16_t>(std::llround(std::clamp(
            output.control, -max_permille_, max_permille_)));
        return true;
    }

    bool homed() const { return homed_.load(std::memory_order_acquire); }

private:
    SensorlessHomingArgs args_;
    double reference_velocity_;
    double max_permille_;
    tether::control::PIController pi_;
    std::atomic<bool> homed_{false};
    bool moved_{false};
    double stall_timer_{0.0};
};

bool configureAndEnableDrive(EtherCAT::DS402Master& master, const SensorlessHomingArgs& args)
{
    Tether::Examples::SingleDriveExampleConfig config;
    config.drive.slave_index = static_cast<uint16_t>(args.slave);
    config.drive.rxpdo_index = EtherCAT::Drives::AS715N_pdo::RxPDO_1702.index;
    config.drive.txpdo_index = EtherCAT::Drives::AS715N_pdo::TxPDO_1B04.index;
    config.drive.rxpdo_size = EtherCAT::Drives::AS715N_pdo::RxPDO_1702.size;
    config.drive.txpdo_size = EtherCAT::Drives::AS715N_pdo::TxPDO_1B04.size;
    config.drive.operating_mode = CiA402::OperatingMode::CyclicSyncTorque;
    return Tether::Examples::configureAndEnableSingleDrive(master, config, TAG);
}

} // namespace

int main(int argc, char** argv)
{
    SensorlessHomingArgs args;
    if (!parseSensorlessHomingArgs(argc, argv, "as715n_sensorless_homing", args)) {
        return 1;
    }

    Tether::Platform::ensureRealtimeKernelOrExit();

    EtherCAT::DS402Master master;
    Tether::Examples::HostMasterSession session;
    if (!Tether::Examples::startHostMasterSession(args.interface, master, session, TAG, args.vlan)) {
        return 2;
    }

    int rc = 0;
    if (!configureAndEnableDrive(master, args)) {
        rc = 3;
    } else {
        auto* controller = new SensorlessHomingController(args);
        if (!master.addMotionController(static_cast<uint16_t>(args.slave),
                                        std::unique_ptr<EtherCAT::DS402Master::IDriveMotionController>(controller))) {
            TETHER_LOGE(TAG, "Failed to add sensorless homing controller");
            delete controller;
            rc = 4;
        } else {
            EtherCAT::Master::RealtimeMotionLoopConfig loop_config;
            loop_config.cycle_period_us = 1000;
            loop_config.sync_interval_cycles = 10;
            loop_config.enable_dc_synchronization = true;
            if (!master.startRealtimeMotionControlLoop(loop_config)) {
                TETHER_LOGE(TAG, "Failed to start realtime motion loop");
                (void)master.removeMotionController(static_cast<uint16_t>(args.slave));
                rc = 5;
            } else {
                const auto start_ms = Tether::Platform::Clock::instance().getMilliseconds();
                const auto timeout_ms = static_cast<uint32_t>(args.max_runtime * 1000.0);
                while (!controller->homed() &&
                       Tether::Platform::Clock::instance().getMilliseconds() - start_ms < timeout_ms) {
                    Tether::Platform::Clock::instance().delayMilliseconds(10);
                }

                master.stopMotionControlLoop();

                if (!controller->homed()) {
                    TETHER_LOGE(TAG, "Timeout waiting for stall");
                    rc = 6;
                } else {
                    (void)master.removeMotionController(static_cast<uint16_t>(args.slave));
                    auto* drive = master.driveBySlaveIndex(static_cast<uint16_t>(args.slave));
                    if (drive == nullptr || !drive->homeToCurrentPosition(0)) {
                        TETHER_LOGE(TAG, "Failed to set current position as home");
                        rc = 7;
                    } else {
                        TETHER_LOGI(TAG, "Sensorless homing complete");
                    }
                }
            }
        }

        Tether::Examples::shutdownSingleDrive(master, static_cast<uint16_t>(args.slave));
    }

    Tether::Examples::stopHostMasterSession(master, session);
    return rc;
}
