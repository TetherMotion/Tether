#include <cstdint>
#include <cstdlib>
#include <memory>
#include <string>

#include <algorithm>

#include "DS402ExampleSupport.hpp"
#include "tether/drives/AS715N/AS715NDriveInitializer.hpp"
#include "tether/drives/AS715N/AS715NPDO.hpp"
#include "tether/platform/EspCompat.hpp"
#include "tether/platform/Platform.hpp"
#include "tether/profiles/cia301/CiA402Defs.hpp"
#include "tether/profiles/cia402/SensorlessTorqueHomingController.hpp"

namespace {

constexpr const char* TAG = "as715n_homing";

using HomingController = EtherCAT::SensorlessTorqueHomingController<
    EtherCAT::Drives::AS715N_pdo::AS715N_RxPDO_1702,
    EtherCAT::Drives::AS715N_pdo::AS715N_TxPDO_1B04>;

struct SensorlessHomingArgs {
    std::string interface;
    int slave = 0;
    double target_velocity = 30000.0;
    int direction = 1;
    double max_torque_percent = 10.0;
    bool use_csv_mode = true;
    double kp = 0.05;
    double ki = 0.005;
    std::string stall_detection = "position";
    double stall_velocity = 100.0;
    double stall_window = 0.1;
    double stall_position_counts = 20.0;
    double stall_torque_permille = -1.0;
    double stall_time = 1.0;
    int home_avg_samples = 250;
    double max_runtime = 60.0;
    uint32_t homing_timeout_ms = 10000;
    int fine_homing_passes = 1;
    double backoff_distance = 5000.0;
    double fine_velocity = 0.0;
    double phase_timeout = 30.0;
    std::string pass_aggregation = "mean";
    bool apply_drive_homing = false;
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
        .default_value(30000.0)
        .help("target velocity setpoint magnitude in counts/s");
    program.add_argument("--direction")
        .scan<'i', int>()
        .default_value(1)
        .help("direction: +1 or -1");
    program.add_argument("--max-torque")
        .scan<'g', double>()
        .default_value(10.0)
        .help("maximum torque in percent of rated (e.g. 10.0 = 10 pct)");
    program.add_argument("--csv")
        .default_value(true)
        .implicit_value(true)
        .help("use drive-internal velocity loop (CSV mode) with torque limits "
              "0x60E0/0x60E1 instead of the host-side CST velocity PI "
              "(default: true, pass 0/false to disable)");
    program.add_argument("--kp")
        .scan<'g', double>()
        .default_value(0.05)
        .help("velocity loop proportional gain");
    program.add_argument("--ki")
        .scan<'g', double>()
        .default_value(0.005)
        .help("velocity loop integral gain");
    program.add_argument("--stall-detection")
        .default_value(std::string("position"))
        .help("stall detection method: 'position' (default), 'speed' or 'torque'");
    program.add_argument("--stall-velocity")
        .scan<'g', double>()
        .default_value(100.0)
        .help("speed magnitude below which the drive is considered stalled (counts/s)");
    program.add_argument("--stall-window")
        .scan<'g', double>()
        .default_value(0.1)
        .help("position-delta window for 'position' stall detection (s)");
    program.add_argument("--stall-position-counts")
        .scan<'g', double>()
        .default_value(20.0)
        .help("max position change within --stall-window that still counts as stalled");
    program.add_argument("--stall-torque")
        .scan<'g', double>()
        .default_value(-1.0)
        .help("torque_actual magnitude (permille) that counts as stalled in "
              "'torque' mode; <0 = 80%% of --max-torque");
    program.add_argument("--home-avg-samples")
        .scan<'i', int>()
        .default_value(250)
        .help("number of position samples averaged for each recorded pass position");
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
    program.add_argument("--fine-passes")
        .scan<'i', int>()
        .default_value(1)
        .help("number of fine homing passes (1-64)");
    program.add_argument("--backoff-distance")
        .scan<'g', double>()
        .default_value(5000.0)
        .help("counts to back off between fine passes");
    program.add_argument("--fine-velocity")
        .scan<'g', double>()
        .default_value(0.0)
        .help("velocity for back-off and re-approach passes; 0 means use --target-velocity");
    program.add_argument("--phase-timeout")
        .scan<'g', double>()
        .default_value(30.0)
        .help("max seconds per approach/back-off phase before failing; 0 disables");
    program.add_argument("--pass-aggregation")
        .default_value(std::string("mean"))
        .help("'mean' or 'sum' of recorded pass positions");
    program.add_argument("--apply-drive-homing")
        .default_value(false)
        .implicit_value(true)
        .help("opt-in: call CiA402Drive::homeToCurrentPosition(0) after stalling");

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
    out.use_csv_mode = program.get<bool>("--csv");
    out.kp = program.get<double>("--kp");
    out.ki = program.get<double>("--ki");
    out.stall_detection = program.get<std::string>("--stall-detection");
    if (out.stall_detection != "position" && out.stall_detection != "speed" &&
        out.stall_detection != "torque") {
        std::cerr << "--stall-detection must be 'position', 'speed' or 'torque'\n";
        return false;
    }
    out.stall_velocity = program.get<double>("--stall-velocity");
    out.stall_window = program.get<double>("--stall-window");
    if (out.stall_window <= 0.0) {
        std::cerr << "--stall-window must be positive\n";
        return false;
    }
    out.stall_position_counts = program.get<double>("--stall-position-counts");
    out.stall_torque_permille = program.get<double>("--stall-torque");
    out.stall_time = program.get<double>("--stall-time");
    out.home_avg_samples = program.get<int>("--home-avg-samples");
    if (out.home_avg_samples < 1) {
        std::cerr << "--home-avg-samples must be >= 1\n";
        return false;
    }
    out.max_runtime = program.get<double>("--max-runtime");
    out.homing_timeout_ms = static_cast<uint32_t>(program.get<int>("--homing-timeout"));
    out.fine_homing_passes = program.get<int>("--fine-passes");
    if (out.fine_homing_passes < 1 ||
        out.fine_homing_passes > HomingController::kMaxFineHomingPasses) {
        std::cerr << "--fine-passes must be in [1, "
                  << HomingController::kMaxFineHomingPasses << "]\n";
        return false;
    }
    out.backoff_distance = program.get<double>("--backoff-distance");
    out.fine_velocity = program.get<double>("--fine-velocity");
    out.phase_timeout = program.get<double>("--phase-timeout");
    out.pass_aggregation = program.get<std::string>("--pass-aggregation");
    if (out.pass_aggregation != "mean" && out.pass_aggregation != "sum") {
        std::cerr << "--pass-aggregation must be 'mean' or 'sum'\n";
        return false;
    }
    out.apply_drive_homing = program.get<bool>("--apply-drive-homing");
    if (!Tether::Examples::parseVlanArgs(
            program.get<std::string>("--rx-vlan"),
            program.get<std::string>("--tx-vlan"),
            out.vlan, program_name)) {
        return false;
    }
    Tether::Examples::logVlanConfig(out.vlan, program_name);
    return true;
}

bool configureAndEnableDrive(EtherCAT::DS402Master& master, const SensorlessHomingArgs& args)
{
    const uint16_t slave_idx = static_cast<uint16_t>(args.slave);
    const auto assignment = EtherCAT::Drives::AS715N_pdo::makePDOAssignment(
        EtherCAT::Drives::AS715N_pdo::RxPDO_1702,
        EtherCAT::Drives::AS715N_pdo::TxPDO_1B04);

    EtherCAT::Drives::AS715N::AS715NDriveInitializer init(master, slave_idx, TAG);
    if (!init.init(assignment)) {
        TETHER_LOGE(TAG, "AS715N drive initialization failed");
        return false;
    }

    const int8_t op_mode = args.use_csv_mode
        ? CiA402::OperatingMode::CyclicSyncVelocity
        : CiA402::OperatingMode::CyclicSyncTorque;
    if (!init.drive().setOperatingMode(op_mode)) {
        TETHER_LOGE(TAG, "Failed to set operating mode {}", static_cast<int>(op_mode));
        return false;
    }

    if (args.use_csv_mode) {
        // CSV: the drive closes the velocity loop internally.  The torque
        // the drive may apply while approaching the stop is limited via
        // the CiA402 positive/negative torque limit objects.
        const uint16_t permille = static_cast<uint16_t>(
            std::min(1000.0, args.max_torque_percent * 10.0));
        auto& sdo = init.sdo();
        if (!sdo.writeU16(0x60E0, 0x00, permille, {.timeout_ms = 3000}).has_value() ||
            !sdo.writeU16(0x60E1, 0x00, permille, {.timeout_ms = 3000}).has_value()) {
            TETHER_LOGE(TAG, "Failed to write torque limits 0x60E0/0x60E1");
            return false;
        }
        TETHER_LOGI(TAG, "Torque limits 0x60E0/0x60E1 set to {} permille", permille);
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

    if (master.ethercatMaster().discovery().discover(EtherCAT::DiscoveryOptions()).empty()) {
        TETHER_LOGW(TAG, "No slaves discovered");
    }

    const uint16_t minimum_drive_count = static_cast<uint16_t>(args.slave + 1);
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
    if (!configureAndEnableDrive(master, args)) {
        rc = 3;
    } else {
        HomingController::Config ctrl_cfg;
        ctrl_cfg.target_velocity = args.target_velocity;
        ctrl_cfg.direction = args.direction;
        ctrl_cfg.max_torque_percent = args.max_torque_percent;
        ctrl_cfg.use_csv_mode = args.use_csv_mode;
        ctrl_cfg.kp = args.kp;
        ctrl_cfg.ki = args.ki;
        if (args.stall_detection == "speed") {
            ctrl_cfg.stall_detection = HomingController::StallDetection::Speed;
        } else if (args.stall_detection == "torque") {
            ctrl_cfg.stall_detection = HomingController::StallDetection::Torque;
        } else {
            ctrl_cfg.stall_detection = HomingController::StallDetection::Position;
        }
        ctrl_cfg.stall_velocity = args.stall_velocity;
        ctrl_cfg.stall_window = args.stall_window;
        ctrl_cfg.stall_position_counts = args.stall_position_counts;
        ctrl_cfg.stall_torque_permille = args.stall_torque_permille;
        ctrl_cfg.stall_time = args.stall_time;
        ctrl_cfg.home_position_avg_samples =
            static_cast<uint32_t>(args.home_avg_samples);
        ctrl_cfg.fine_homing_passes = args.fine_homing_passes;
        ctrl_cfg.backoff_distance = args.backoff_distance;
        ctrl_cfg.fine_velocity = args.fine_velocity;
        ctrl_cfg.phase_timeout = args.phase_timeout;
        ctrl_cfg.pass_aggregation = (args.pass_aggregation == "sum")
            ? HomingController::PassAggregation::Sum
            : HomingController::PassAggregation::Mean;

        auto controller = std::make_unique<HomingController>(ctrl_cfg);
        auto* raw = controller.get();
        if (!master.addMotionController(static_cast<uint16_t>(args.slave), std::move(controller))) {
            TETHER_LOGE(TAG, "Failed to add sensorless homing controller");
            rc = 4;
        } else {
            EtherCAT::Master::RealtimeMotionLoopConfig loop_config;
            loop_config.cycle_period_us = 1000;
            loop_config.sync_interval_cycles = 10;
            loop_config.enable_dc_synchronization = true;
            if (!master.startRealtimeMotionControlLoop(loop_config)) {
                TETHER_LOGE(TAG, "Failed to start realtime motion loop");
                rc = 5;
            } else {
                const auto start_ms = Tether::Platform::Clock::instance().getMilliseconds();
                const auto timeout_ms = static_cast<uint32_t>(args.max_runtime * 1000.0);
                while (!raw->isHomed() && !raw->hasFailed() &&
                       Tether::Platform::Clock::instance().getMilliseconds() - start_ms < timeout_ms) {
                    Tether::Platform::Clock::instance().delayMilliseconds(10);
                }

                master.stopMotionControlLoop();

                if (raw->hasFailed()) {
                    TETHER_LOGE(TAG, "Controller failed: {}", raw->failureMessage());
                    rc = 6;
                } else if (!raw->isHomed()) {
                    TETHER_LOGE(TAG, "Timeout waiting for stall");
                    rc = 7;
                } else {
                    // removeMotionController() destroys the controller object,
                    // so read the results into locals before removing it.
                    const bool has_home_position = raw->hasHomePosition();
                    const int64_t home_position = raw->homePosition();
                    master.removeMotionController(static_cast<uint16_t>(args.slave));
                    if (has_home_position) {
                        TETHER_LOGI(TAG,
                                    "Virtual homing switch active at {}",
                                    home_position);
                    }
                    if (args.apply_drive_homing) {
                        auto* drive = master.driveBySlaveIndex(static_cast<uint16_t>(args.slave));
                        if (drive == nullptr ||
                            !drive->homeToCurrentPosition(0, args.homing_timeout_ms)) {
                            TETHER_LOGE(TAG, "Failed to set current position as home");
                            rc = 8;
                        } else {
                            TETHER_LOGI(TAG, "Sensorless homing complete; drive count set to 0");
                        }
                    } else {
                        TETHER_LOGI(TAG, "Sensorless homing complete; drive count not modified");
                    }
                }
            }
        }

        Tether::Examples::shutdownSingleDrive(master, static_cast<uint16_t>(args.slave));
    }

    Tether::Examples::stopHostMasterSession(master, session);
    return rc;
}
