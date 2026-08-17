#pragma once

#include <cstdint>
#include <memory>
#include <utility>
#include <vector>

#include "tether/common/ISetpointSource.hpp"
#include "tether/ethercat/DC.hpp"
#include "tether/ethercat/Master.hpp"
#include "tether/ethercat/CyclicTaskScheduler.hpp"
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

    using ICyclicTask = EtherCAT::ICyclicTask;

    template<typename RxPDO>
    class GenericDriveMotionController final : public IDriveMotionController {
    public:
        GenericDriveMotionController(CyclicTarget target,
                                     std::unique_ptr<tether::common::ISetpointSource> source,
                                     double scale)
            : target_(target)
            , controller_(std::move(source))
            , scale_(scale)
        {
        }

        bool start(CiA402Drive& drive) override {
            controller_->start();
            return drive.setOperatingMode(modeForTarget());
        }

        void stop(CiA402Drive&) override {
            controller_->stopImmediate();
        }

        bool update(CiA402Drive& drive, double dt_seconds) override {
            auto* rx = drive.rxPDO<RxPDO>();
            if (rx == nullptr) {
                return false;
            }

            controller_->update(dt_seconds);
            rx->controlword = static_cast<uint16_t>(ControlWord::ENABLE_OPERATION);
            rx->modes_of_operation = modeForTarget();

            switch (target_) {
                case CyclicTarget::Position:
                    if constexpr (requires(RxPDO& pdo) { pdo.target_position; }) {
                        rx->target_position = static_cast<int32_t>(controller_->getPosition() * scale_);
                    } else {
                        return false;
                    }
                    if constexpr (requires(RxPDO& pdo) { pdo.target_velocity; }) {
                        rx->target_velocity = 0;
                    }
                    break;
                case CyclicTarget::Velocity:
                    if constexpr (requires(RxPDO& pdo) { pdo.target_velocity; }) {
                        rx->target_velocity = static_cast<int32_t>(controller_->getVelocity() * scale_);
                    } else {
                        return false;
                    }
                    break;
                case CyclicTarget::Torque:
                    if constexpr (requires(RxPDO& pdo) { pdo.target_torque; }) {
                        rx->target_torque = static_cast<int16_t>(controller_->getPosition() * scale_);
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
        std::unique_ptr<tether::common::ISetpointSource> controller_;
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
    bool addMotionController(uint16_t slave_index,
                             CyclicTarget target,
                             std::unique_ptr<tether::common::ISetpointSource> source,
                             double scale = 1.0)
    {
        return addMotionController(
            slave_index,
            std::make_unique<GenericDriveMotionController<RxPDO>>(target, std::move(source), scale));
    }
    bool removeMotionController(uint16_t slave_index);
    void clearMotionControllers();
    bool addCyclicTask(std::unique_ptr<ICyclicTask> task);
    bool addCyclicTask(std::unique_ptr<ICyclicTask> task, TaskPhase phase, uint8_t priority = 128);
    void clearCyclicTasks();
    CyclicTaskScheduler& cyclicTaskScheduler() { return cyclic_task_scheduler_; }
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

    /// Get-or-create the CiA402Drive for @p slave_index.
    ///
    /// Ensures the slave is marked as DS402-managed and that a CiA402Drive
    /// object exists in the internal drive table.  Idempotent: repeated calls
    /// with the same slave index return the same drive.  Use this instead of
    /// driveBySlaveIndex() when a drive must be created without going through
    /// configureDrive() (e.g. when using the multi-PDO-per-sync-manager
    /// transitionToOp() overload directly).
    CiA402Drive& ensureDrive(uint16_t slave_index);

    bool enableDrive(uint16_t slave_index, uint32_t timeout_ms = 5000);
    bool disableDrive(uint16_t slave_index);
    bool enableAllDrives(uint32_t timeout_ms = 5000);
    bool disableAllDrives();

private:
    void ensureSlaveRoleCapacity(uint16_t slave_index);

    Master ethercat_master_;
    std::vector<std::unique_ptr<CiA402Drive>> drives_;
    std::vector<SlaveRole> slave_roles_;
    std::vector<std::pair<uint16_t, std::unique_ptr<IDriveMotionController>>> motion_controllers_;
    CyclicTaskScheduler cyclic_task_scheduler_;
    std::vector<std::unique_ptr<ICyclicTask>> owned_tasks_;  // Keep ownership for backward compat
};

} // namespace EtherCAT