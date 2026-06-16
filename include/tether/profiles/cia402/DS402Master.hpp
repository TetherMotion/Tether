#pragma once

#include <cstdint>
#include <memory>
#include <utility>
#include <vector>

#include "tether/control/SineMotionController.hpp"
#include "tether/ethercat/DC.hpp"
#include "tether/ethercat/Master.hpp"
#include "tether/profiles/cia402/CiA402Drive.hpp"

namespace EtherCAT {

class DS402Master {
public:
    enum class SlaveRole : uint8_t {
        NonDS402,
        DS402,
        DynaDrive,
    };

    enum class CyclicTarget : uint8_t {
        Position,
        Velocity,
        Torque,
    };

    class IDriveMotionController {
    public:
        virtual ~IDriveMotionController() = default;
        virtual bool start(CiA402Drive& drive) = 0;
        virtual void stop(CiA402Drive& drive) = 0;
        virtual bool update(CiA402Drive& drive, double dt_seconds) = 0;
    };

    class ICyclicTask {
    public:
        virtual ~ICyclicTask() = default;
        virtual bool update(DS402Master& master, double dt_seconds) = 0;
    };

    template<typename RxPDO>
    class SineDriveMotionController final : public IDriveMotionController {
    public:
        SineDriveMotionController(CyclicTarget target,
                                  const Control::SineMotionController::Config& config,
                                  double scale)
            : target_(target)
            , controller_(config)
            , scale_(scale)
        {
        }

        bool start(CiA402Drive& drive) override {
            controller_.start();
            return drive.setOperatingMode(modeForTarget());
        }

        void stop(CiA402Drive&) override {
            controller_.stopImmediate();
        }

        bool update(CiA402Drive& drive, double dt_seconds) override {
            auto* rx = drive.rxPDO<RxPDO>();
            if (rx == nullptr) {
                return false;
            }

            controller_.update(dt_seconds);
            rx->controlword = static_cast<uint16_t>(ControlWord::ENABLE_OPERATION);
            rx->modes_of_operation = modeForTarget();

            switch (target_) {
                case CyclicTarget::Position:
                    if constexpr (requires(RxPDO& pdo) { pdo.target_position; }) {
                        rx->target_position = controller_.getPositionScaled(scale_);
                    } else {
                        return false;
                    }
                    if constexpr (requires(RxPDO& pdo) { pdo.target_velocity; }) {
                        rx->target_velocity = 0;
                    }
                    break;
                case CyclicTarget::Velocity:
                    if constexpr (requires(RxPDO& pdo) { pdo.target_velocity; }) {
                        rx->target_velocity = controller_.getVelocityScaled(scale_);
                    } else {
                        return false;
                    }
                    break;
                case CyclicTarget::Torque:
                    if constexpr (requires(RxPDO& pdo) { pdo.target_torque; }) {
                        rx->target_torque = static_cast<int16_t>(controller_.getPosition() * scale_);
                    } else {
                        return false;
                    }
                    break;
            }

            return true;
        }

    private:
        int8_t modeForTarget() const {
            switch (target_) {
                case CyclicTarget::Position:
                    return CiA402::OperatingMode::CyclicSyncPosition;
                case CyclicTarget::Velocity:
                    return CiA402::OperatingMode::CyclicSyncVelocity;
                case CyclicTarget::Torque:
                    return CiA402::OperatingMode::CyclicSyncTorque;
            }

            return 0;
        }

        CyclicTarget target_;
        Control::SineMotionController controller_;
        double scale_{1.0};
    };

    struct DriveConfiguration {
        uint16_t slave_index{0};
        uint16_t rxpdo_index{0};
        uint16_t txpdo_index{0};
        uint16_t rxpdo_size{0};
        uint16_t txpdo_size{0};
        int8_t operating_mode{0};
        uint32_t sdo_timeout_ms{3000};
        bool auto_configure_mailbox{true};
        bool transition_to_operational{true};
    };

    DS402Master();
    explicit DS402Master(const Master::Config& config);

    void start(const NetworkInterface& iface, const uint8_t src_mac[6]);

    void stop();
    bool isRunning() const;

    Master& ethercatMaster() { return ethercat_master_; }
    const Master& ethercatMaster() const { return ethercat_master_; }

    uint16_t discoveredDriveCount() const;
    bool waitForDriveCount(uint16_t minimum_drive_count, uint32_t timeout_ms);

    bool initializeDistributedClocks(const DC::DCConfig& config, uint16_t slave_count = 0);
    bool startDistributedClocks();
    void stopDistributedClocks();

    bool configureDrive(const DriveConfiguration& config);
    bool configureDrives(const std::vector<DriveConfiguration>& configs);

    void setSlaveRole(uint16_t slave_index, SlaveRole role);
    void setSlaveAsDS402(uint16_t slave_index);
    void setSlaveAsNonDS402(uint16_t slave_index);
    void setSlaveAsDynaDrive(uint16_t slave_index);
    SlaveRole slaveRole(uint16_t slave_index) const;
    bool isDS402Slave(uint16_t slave_index) const;
    bool isManagedDrive(uint16_t slave_index) const;

    bool addMotionController(uint16_t slave_index, std::unique_ptr<IDriveMotionController> controller);
    template<typename RxPDO>
    bool addSineMotionController(uint16_t slave_index,
                                 CyclicTarget target,
                                 const Control::SineMotionController::Config& config,
                                 double scale = 1.0)
    {
        return addMotionController(
            slave_index,
            std::make_unique<SineDriveMotionController<RxPDO>>(target, config, scale));
    }
    bool removeMotionController(uint16_t slave_index);
    void clearMotionControllers();
    bool addCyclicTask(std::unique_ptr<ICyclicTask> task);
    void clearCyclicTasks();
    bool updateMotionControllers(double dt_seconds);

    bool startRealtimeMotionControlLoop();
    bool startRealtimeMotionControlLoop(
        const Master::RealtimeMotionLoopConfig& config);
    bool startPollingMotionControlLoop();
    bool startPollingMotionControlLoop(
        const Master::PollingMotionLoopConfig& config);
    void stopMotionControlLoop();

    size_t driveCount() const { return drives_.size(); }
    CiA402Drive* driveAt(size_t index);
    const CiA402Drive* driveAt(size_t index) const;
    CiA402Drive* driveBySlaveIndex(uint16_t slave_index);
    const CiA402Drive* driveBySlaveIndex(uint16_t slave_index) const;

    bool enableDrive(uint16_t slave_index, uint32_t timeout_ms = 5000);
    bool disableDrive(uint16_t slave_index);
    bool enableAllDrives(uint32_t timeout_ms = 5000);
    bool disableAllDrives();

private:
    void ensureSlaveRoleCapacity(uint16_t slave_index);
    CiA402Drive& ensureDrive(uint16_t slave_index);

    Master ethercat_master_;
    std::vector<std::unique_ptr<CiA402Drive>> drives_;
    std::vector<SlaveRole> slave_roles_;
    std::vector<std::pair<uint16_t, std::unique_ptr<IDriveMotionController>>> motion_controllers_;
    std::vector<std::unique_ptr<ICyclicTask>> cyclic_tasks_;
};

} // namespace EtherCAT