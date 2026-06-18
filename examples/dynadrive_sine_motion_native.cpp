#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <string>

#include "DS402ExampleSupport.hpp"
#include "tether/drives/DynaDrive.hpp"
#include "tether/drives/DynaDrive/DynaDrivePDO.hpp"
#include "tether/platform/EspCompat.hpp"

namespace {

constexpr const char* TAG = "dynadrive_sine";
constexpr uint16_t kSlaveIndex = 6;
constexpr double kTwoPi = 6.28318530717958647692;

// ============================================================================
// Custom DynaDrive Motion Controller
// ============================================================================

class DynaDriveSineMotionController final : public EtherCAT::DS402Master::IDriveMotionController {
public:
    explicit DynaDriveSineMotionController(const Control::SineMotionController::Config& config)
        : controller_(config)
    {
    }

    bool start(EtherCAT::CiA402Drive& drive) override {
        controller_.start();
        auto* rx = drive.rxPDO<EtherCAT::Drives::DynaDrive_pdo::DynaDrive_RxPDO>();
        if (!rx) return false;
        rx->modeOfOperation_ = static_cast<uint16_t>(EtherCAT::Drives::Registers::DynaDrive::ModesOfOperation::Options::JointPosition);
        return true;
    }

    void stop(EtherCAT::CiA402Drive&) override {
        controller_.stopImmediate();
    }

    bool update(EtherCAT::CiA402Drive& drive, double dt_seconds) override {
        auto* rx = drive.rxPDO<EtherCAT::Drives::DynaDrive_pdo::DynaDrive_RxPDO>();
        if (!rx) return false;

        controller_.update(dt_seconds);

        rx->controlword_     = 0;
        rx->modeOfOperation_ = static_cast<uint16_t>(EtherCAT::Drives::Registers::DynaDrive::ModesOfOperation::Options::JointPosition);
        rx->desiredPosition_ = controller_.getPosition();
        rx->desiredVelocity_ = 0.0f;
        rx->desiredJointTorque_ = 0.0f;
        rx->desiredMotorCurrent_ = 0.0f;

        rx->controlParameterA_ = 10.0f;
        rx->controlParameterB_ = 0.1f;
        rx->controlParameterC_ = 1.0f;
        rx->controlParameterD_ = 0.0f;

        return true;
    }

private:
    Control::SineMotionController controller_;
};

// ============================================================================
// Run Sine Motion
// ============================================================================

int runSineMotion(EtherCAT::DS402Master& master, double duration_seconds)
{
    constexpr double kAmplitudeDegrees = 45.0;
    constexpr double kFrequencyHz = 0.25;
    Control::SineMotionController::Config config = Control::SineMotionController::Config::getDefault();
    config.frequency = kFrequencyHz;
    config.amplitude = kAmplitudeDegrees * (kTwoPi / 360.0);

    if (!master.addMotionController(
            kSlaveIndex,
            std::make_unique<DynaDriveSineMotionController>(config))) {
        return 2;
    }

    EtherCAT::Master::RealtimeMotionLoopConfig loop_config;
    loop_config.cycle_period_us = 1000;
    loop_config.sync_interval_cycles = 10;
    loop_config.enable_dc_synchronization = false;
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

// ============================================================================
// Configure Drive
// ============================================================================

bool configureDrive(EtherCAT::DS402Master& master)
{
    if (!master.ethercatMaster().discoverSlaves()) {
        TETHER_LOGW(TAG, "No slaves discovered");
    }

    const uint16_t minimum_drive_count = static_cast<uint16_t>(kSlaveIndex + 1);
    if (!master.waitForDriveCount(minimum_drive_count, 2000)) {
        TETHER_LOGE(TAG, "Timed out waiting for %u drive(s)", minimum_drive_count);
        return false;
    }

    master.setSlaveAsDynaDrive(kSlaveIndex);

    EtherCAT::DS402Master::DriveConfiguration config;
    config.slave_index  = kSlaveIndex;
    config.rxpdo_index = EtherCAT::Drives::DynaDrive_pdo::RxPDO_1603.index;
    config.txpdo_index = EtherCAT::Drives::DynaDrive_pdo::TxPDO_1A04.index;
    config.rxpdo_size  = EtherCAT::Drives::DynaDrive_pdo::RxPDO_1603.size;
    config.txpdo_size  = EtherCAT::Drives::DynaDrive_pdo::TxPDO_1A04.size;
    config.operating_mode = 0;
    config.auto_configure_mailbox = false;
    config.sdo_timeout_ms = 3000;

    if (!master.configureDrive(config)) {
        TETHER_LOGE(TAG, "Failed to configure slave %u", kSlaveIndex);
        return false;
    }

    if (!master.enableDrive(kSlaveIndex, 10000)) {
        TETHER_LOGE(TAG, "Failed to enable slave %u", kSlaveIndex);
        return false;
    }

    return true;
}

} // namespace

int main(int argc, char** argv)
{
    Tether::Examples::MotionNativeArgs args;
    if (!Tether::Examples::parseMotionNativeArgs(argc, argv, "dynadrive_sine_motion_native", args)) {
        return 1;
    }

    EtherCAT::DS402Master master;
    Tether::Examples::HostMasterSession session;
    if (!Tether::Examples::startHostMasterSession(args.interface, master, session, TAG)) {
        return 2;
    }

    int rc = 0;
    if (!configureDrive(master)) {
        rc = 3;
    } else {
        rc = runSineMotion(master, args.duration);
        (void)master.disableDrive(kSlaveIndex);
    }

    Tether::Examples::stopHostMasterSession(master, session);
    return rc;
}
