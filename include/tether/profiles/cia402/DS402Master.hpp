#pragma once

#include <atomic>
#include <cstdint>
#include <memory>
#include <mutex>
#include <utility>
#include <vector>

#include "tether/common/ISetpointSource.hpp"
#include "tether/ethercat/DC.hpp"
#include "tether/ethercat/Master.hpp"
#include "tether/ethercat/CyclicTaskScheduler.hpp"
#include "tether/ethercat/SlaveSupervisor.hpp"
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

        /// Set the controlword the cyclic update asserts every cycle.
        /// Defaults to ENABLE_OPERATION; the motion loop re-writes it each
        /// cycle so one-shot SDO/PDO writes cannot leave a stale value (e.g.
        /// a bare homing-start bit with the enable bits cleared) on the wire.
        void setDesiredControlword(uint16_t cw) {
            desired_controlword_.store(cw, std::memory_order_relaxed);
        }
        uint16_t desiredControlword() const {
            return desired_controlword_.load(std::memory_order_relaxed);
        }

        bool update(CiA402Drive& drive, double dt_seconds) override {
            auto* rx = drive.rxPDO<RxPDO>();
            if (rx == nullptr) {
                return false;
            }

            controller_->update(dt_seconds);
            rx->controlword = desired_controlword_.load(std::memory_order_relaxed);
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
                    return static_cast<int8_t>(CiA402::OperatingMode::CyclicSyncPosition);
                case CyclicTarget::Velocity:
                    return static_cast<int8_t>(CiA402::OperatingMode::CyclicSyncVelocity);
                case CyclicTarget::Torque:
                    return static_cast<int8_t>(CiA402::OperatingMode::CyclicSyncTorque);
            }

            return 0;
        }

        CyclicTarget target_;
        std::unique_ptr<tether::common::ISetpointSource> controller_;
        double scale_{1.0};
        std::atomic<uint16_t> desired_controlword_{
            static_cast<uint16_t>(ControlWord::EnableOperation)};
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
    /// Applies any controller/task mutations still queued from a previous
    /// loop run before the members destruct.
    ~DS402Master();

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

    /// Register a motion controller for a managed drive.
    ///
    /// The controller's start() runs synchronously so failures are reported
    /// to the caller.  While a realtime loop is running the registration is
    /// deferred: the controller joins the loop at the next cycle boundary,
    /// applied on the loop thread — safe to call at any time.
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
    /// Remove a controller.  While a realtime loop is running the removal
    /// (and the controller's stop()) is deferred to the loop thread's next
    /// pass; with no loop running it happens synchronously.  Returns false
    /// if no controller is registered or pending for the slave.
    bool removeMotionController(uint16_t slave_index);
    /// Remove all controllers.  Deferred to the loop thread while running.
    void clearMotionControllers();
    bool addCyclicTask(std::unique_ptr<ICyclicTask> task);
    bool addCyclicTask(std::unique_ptr<ICyclicTask> task, TaskPhase phase, uint8_t priority = 128);
    void clearCyclicTasks();
    CyclicTaskScheduler& cyclicTaskScheduler() { return cyclic_task_scheduler_; }
    bool updateMotionControllers(double dt_seconds);

    // ========================================================================
    // Slave Recovery / Supervision
    // ========================================================================

    /**
     * @brief Access the underlying Master's SlaveSupervisor.
     *
     * The supervisor is always present on the Master but is disabled by
     * default.  Use this to configure and enable automatic slave recovery.
     *
     * @code
     *   auto& sup = ds402.slaveSupervisor();
     *   RecoveryConfig cfg;
     *   cfg.enabled = true;
     *   cfg.max_attempts = 3;
     *   sup.configure(cfg);
     *   sup.setRecoveryHandler(
     *       std::make_unique<DS402RecoveryHandler>(ds402, configs));
     *   sup.start();
     * @endcode
     */
    SlaveSupervisor& slaveSupervisor();

    /**
     * @brief Built-in recovery handler that re-configures and re-enables a
     *        DS402 drive from scratch.
     *
     * Stores a copy of the DriveConfiguration for each slave and replays the
     * full configureDrive() + enableDrive() sequence when
     * reinitializeSlave() is called.
     */
    class DS402RecoveryHandler : public ISlaveRecoveryHandler {
    public:
        DS402RecoveryHandler(DS402Master& master,
                             const std::vector<DriveConfiguration>& configs);

        bool reinitializeSlave(uint16_t slave_index) override;

    private:
        DS402Master& master_;
        std::vector<DriveConfiguration> configs_;
    };

    /**
     * @brief Convenience: enable slave recovery with a DS402 re-init handler.
     *
     * Configures the supervisor with the given RecoveryConfig, installs a
     * DS402RecoveryHandler using the provided drive configurations, and
     * starts supervision.
     *
     * @param config       Recovery configuration (enabled is forced to true)
     * @param configs      Drive configurations for re-initialization
     * @return true if supervision started successfully
     */
    bool enableSlaveRecovery(const RecoveryConfig& config,
                              const std::vector<DriveConfiguration>& configs);

    /// Disable and stop slave recovery supervision.
    void disableSlaveRecovery();

    bool startRealtimeMotionControlLoop();
    bool startRealtimeMotionControlLoop(
        const Master::RealtimeMotionLoopConfig& config);
    bool startPollingMotionControlLoop();
    bool startPollingMotionControlLoop(
        const Master::PollingMotionLoopConfig& config);
    void stopMotionControlLoop();

    /**
     * @brief Start the deadline-driven cyclic executive.
     *
     * Replaces startDistributedClocks() + startRealtimeMotionControlLoop()
     * with a single low-latency loop: one thread sleeps directly on an
     * absolute deadline (clock_nanosleep TIMER_ABSTIME) and runs the LRW
     * exchange via the reserved-index fast path; DC sync runs on a
     * dedicated fault-isolated thread by default (config.exec.dc_placement).
     *
     * Motion controllers run in the loop's MotionControl phase unless
     * config.motion_in_loop is false (external motion source).
     */
    bool startCyclicLoop(const Master::CyclicLoopConfig& config);
    /// RAII variant: returns a guard that owns the running loop and stops it
    /// on destruction.  A disengaged guard means startup failed.
    [[nodiscard]] Master::CyclicLoopGuard startCyclicLoopScoped(
        const Master::CyclicLoopConfig& config);
    void stopCyclicLoop();

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

    /// True while a realtime loop thread may be iterating controllers/tasks.
    bool realtimeLoopRunning() const;
    /// Stop and erase a controller by slave index.  Caller thread only —
    /// must not run concurrently with updateMotionControllers().
    bool eraseController(uint16_t slave_index);
    /// Apply queued controller/task mutations.  Called at the top of
    /// updateMotionControllers() (the loop thread) and by the stop/clear
    /// paths once no loop is running.
    void drainControllerOps();

    struct PendingControllerOp {
        enum class Kind : uint8_t { Add, Remove, Clear } kind;
        uint16_t slave_index{0};
        std::unique_ptr<IDriveMotionController> controller;
    };

    Master ethercat_master_;
    std::vector<std::unique_ptr<CiA402Drive>> drives_;
    std::vector<SlaveRole> slave_roles_;
    // Iterated by updateMotionControllers().  While a loop runs it is only
    // mutated by drainControllerOps() on the loop thread; when idle it is
    // only touched by the application thread — never concurrently.
    std::vector<std::pair<uint16_t, std::unique_ptr<IDriveMotionController>>> motion_controllers_;
    CyclicTaskScheduler cyclic_task_scheduler_;
    std::vector<std::unique_ptr<ICyclicTask>> owned_tasks_;  // Keep ownership for backward compat
    // Cross-thread mutation plumbing: application-thread add/remove/clear
    // calls enqueue ops here; the loop thread applies them in order.
    std::mutex motion_ctl_mutex_;
    std::vector<PendingControllerOp> pending_controller_ops_;
    std::vector<std::unique_ptr<ICyclicTask>> retired_tasks_;
    std::atomic<bool> controller_ops_pending_{false};
    std::atomic<bool> tasks_retired_{false};
};

} // namespace EtherCAT