#include "tether/profiles/cia402/DS402Master.hpp"

#include "tether/platform/Platform.hpp"

#include <algorithm>

namespace EtherCAT {

DS402Master::DS402Master()
{
    // See member comments: fixed capacity keeps concurrent loop-thread
    // iteration free of reallocation hazards.
    drives_.reserve(PDO::kMaxPDOSlaves);
    slave_roles_.resize(PDO::kMaxPDOSlaves, SlaveRole::NonDS402);
}

DS402Master::DS402Master(const Master::Config& config)
    : ethercat_master_(config)
{
    drives_.reserve(PDO::kMaxPDOSlaves);
    slave_roles_.resize(PDO::kMaxPDOSlaves, SlaveRole::NonDS402);
}

DS402Master::~DS402Master()
{
    drainControllerOps();
}

void DS402Master::start(const NetworkInterface& iface, const uint8_t src_mac[6])
{
    ethercat_master_.start(iface, src_mac);
}

void DS402Master::stop()
{
    ethercat_master_.stop();
}

bool DS402Master::isRunning() const
{
    return ethercat_master_.isRunning();
}

uint16_t DS402Master::discoveredDriveCount() const
{
    return ethercat_master_.getDiscoveredSlaveCount();
}

bool DS402Master::waitForDriveCount(uint16_t minimum_drive_count, uint32_t timeout_ms)
{
    auto& clock = Tether::Platform::Clock::instance();
    const uint64_t deadline_us = clock.getMicroseconds() + static_cast<uint64_t>(timeout_ms) * 1000ULL;

    while (clock.getMicroseconds() <= deadline_us) {
        if (discoveredDriveCount() >= minimum_drive_count) {
            return true;
        }
        clock.delayMilliseconds(10);
    }

    return discoveredDriveCount() >= minimum_drive_count;
}

bool DS402Master::initializeDistributedClocks(const DC::DCConfig& config, uint16_t slave_count)
{
    const uint16_t configured_slave_count = slave_count == 0 ? discoveredDriveCount() : slave_count;
    if (configured_slave_count == 0) {
        return false;
    }

    return ethercat_master_.dc().init(config, configured_slave_count);
}

bool DS402Master::startDistributedClocks()
{
    return ethercat_master_.dc().start();
}

void DS402Master::stopDistributedClocks()
{
    ethercat_master_.dc().stop();
}

bool DS402Master::configureDrive(const DriveConfiguration& config)
{
    auto& drive = ensureDrive(config.slave_index);
    drive.setSDOTimeout(config.sdo_timeout_ms);

    if (slaveRole(config.slave_index) != SlaveRole::DynaDrive) {
        if (config.auto_configure_mailbox &&
            !ethercat_master_.autoConfigureMailbox(config.slave_index, Tether::Platform::LogLevel::Info)) {
            return false;
        }
    }

    if (!drive.assignFixedPDOs(config.rxpdo_index, config.txpdo_index,
                               config.rxpdo_size, config.txpdo_size)) {
        return false;
    }

    if (!ethercat_master_.configureProcessDataSyncManagersFromSii(config.slave_index)) {
        return false;
    }

    if (!drive.registerPDOBuffers()) {
        return false;
    }

    if (config.transition_to_operational && !drive.transitionToOp()) {
        return false;
    }

    if (slaveRole(config.slave_index) != SlaveRole::DynaDrive) {
        if (config.operating_mode != 0 && !drive.setOperatingMode(config.operating_mode)) {
            return false;
        }
    }

    return true;
}

bool DS402Master::configureDrives(const std::vector<DriveConfiguration>& configs)
{
    for (const auto& config : configs) {
        if (!configureDrive(config)) {
            return false;
        }
    }

    return true;
}

void DS402Master::setSlaveRole(uint16_t slave_index, SlaveRole role)
{
    ensureSlaveRoleCapacity(slave_index);
    slave_roles_[slave_index] = role;

    if (role == SlaveRole::NonDS402) {
        if (realtimeLoopRunning()) {
            // The loop thread iterates drives_ via driveBySlaveIndex() —
            // defer the erase to drainControllerOps() on that thread.
            std::lock_guard<std::mutex> lock(motion_ctl_mutex_);
            pending_controller_ops_.push_back(
                {PendingControllerOp::Kind::EraseDrive, slave_index, {}});
            controller_ops_pending_.store(true, std::memory_order_release);
        } else {
            (void)removeMotionController(slave_index);
            drives_.erase(
                std::remove_if(drives_.begin(), drives_.end(),
                               [slave_index](const std::unique_ptr<CiA402Drive>& drive) {
                                   return drive && drive->slaveIndex() == slave_index;
                               }),
                drives_.end());
        }
    }
}

void DS402Master::setSlaveAsDS402(uint16_t slave_index)
{
    setSlaveRole(slave_index, SlaveRole::DS402);
}

void DS402Master::setSlaveAsNonDS402(uint16_t slave_index)
{
    setSlaveRole(slave_index, SlaveRole::NonDS402);
}

void DS402Master::setSlaveAsDynaDrive(uint16_t slave_index)
{
    setSlaveRole(slave_index, SlaveRole::DynaDrive);
}

DS402Master::SlaveRole DS402Master::slaveRole(uint16_t slave_index) const
{
    if (slave_index >= slave_roles_.size()) {
        return SlaveRole::NonDS402;
    }

    return slave_roles_[slave_index];
}

bool DS402Master::isDS402Slave(uint16_t slave_index) const
{
    return slaveRole(slave_index) == SlaveRole::DS402;
}

bool DS402Master::isManagedDrive(uint16_t slave_index) const
{
    const auto role = slaveRole(slave_index);
    return role == SlaveRole::DS402 || role == SlaveRole::DynaDrive;
}

bool DS402Master::realtimeLoopRunning() const
{
    return ethercat_master_.isMotionControlLoopRunning() ||
           ethercat_master_.isCyclicLoopRunning();
}

CiA402Drive* DS402Master::findDriveRaw(uint16_t slave_index)
{
    // Same scan as driveBySlaveIndex() but WITHOUT the role gate — the
    // role flips synchronously on a deferred EraseDrive while the drive
    // object lives until the drain; teardown must still reach it.
    for (auto& drive : drives_) {
        if (drive && drive->slaveIndex() == slave_index) {
            return drive.get();
        }
    }
    return nullptr;
}

bool DS402Master::eraseController(uint16_t slave_index)
{
    auto it = std::find_if(
        motion_controllers_.begin(), motion_controllers_.end(),
        [slave_index](const auto& entry) { return entry.first == slave_index; });
    if (it == motion_controllers_.end()) {
        return false;
    }

    if (auto* drive = findDriveRaw(slave_index)) {
        it->second->stop(*drive);
    }
    motion_controllers_.erase(it);
    return true;
}

void DS402Master::drainControllerOps()
{
    std::vector<PendingControllerOp> ops;
    std::vector<std::unique_ptr<ICyclicTask>> retired;
    {
        std::lock_guard<std::mutex> lock(motion_ctl_mutex_);
        ops.swap(pending_controller_ops_);
        retired.swap(retired_tasks_);
        controller_ops_pending_.store(false, std::memory_order_release);
        tasks_retired_.store(false, std::memory_order_release);
    }

    // Applied in enqueue order on this (the loop's, or a post-stop
    // application) thread — never concurrently with the iteration below.
    for (auto& op : ops) {
        switch (op.kind) {
        case PendingControllerOp::Kind::Add:
            eraseController(op.slave_index);  // replace semantics
            motion_controllers_.emplace_back(op.slave_index,
                                             std::move(op.controller));
            break;
        case PendingControllerOp::Kind::Remove:
            eraseController(op.slave_index);
            break;
        case PendingControllerOp::Kind::Clear:
            for (auto& entry : motion_controllers_) {
                if (entry.second) {
                    if (auto* drive = findDriveRaw(entry.first)) {
                        entry.second->stop(*drive);
                    }
                }
            }
            motion_controllers_.clear();
            break;
        case PendingControllerOp::Kind::EraseDrive:
            // Controller first, then the drive object — the controller's
            // stop() may still touch the drive's PDO buffers.
            eraseController(op.slave_index);
            drives_.erase(
                std::remove_if(drives_.begin(), drives_.end(),
                    [idx = op.slave_index](const std::unique_ptr<CiA402Drive>& drive) {
                        return drive && drive->slaveIndex() == idx;
                    }),
                drives_.end());
            break;
        }
    }
    // `retired` destroys here: cyclic tasks removed via clearCyclicTasks()
    // while a loop ran are only destructed once the scheduler's schedule
    // snapshot is guaranteed rebuilt (its schedule_dirty_ flag) — i.e.
    // this drain precedes the next executeAll().
}

bool DS402Master::addMotionController(uint16_t slave_index,
                                      std::unique_ptr<IDriveMotionController> controller)
{
    if (!controller || !isManagedDrive(slave_index)) {
        return false;
    }

    auto* drive = driveBySlaveIndex(slave_index);
    if (drive == nullptr || !controller->start(*drive)) {
        return false;
    }

    if (realtimeLoopRunning()) {
        // The loop thread owns motion_controllers_ — hand the controller
        // over; it joins at the next cycle boundary.
        std::lock_guard<std::mutex> lock(motion_ctl_mutex_);
        pending_controller_ops_.push_back(
            {PendingControllerOp::Kind::Add, slave_index,
             std::move(controller)});
        controller_ops_pending_.store(true, std::memory_order_release);
        return true;
    }

    eraseController(slave_index);  // replace an existing controller
    motion_controllers_.emplace_back(slave_index, std::move(controller));
    return true;
}

bool DS402Master::removeMotionController(uint16_t slave_index)
{
    if (realtimeLoopRunning()) {
        std::lock_guard<std::mutex> lock(motion_ctl_mutex_);
        // Registered (still active until the drain) or a pending Add?
        bool known = std::any_of(
            motion_controllers_.begin(), motion_controllers_.end(),
            [slave_index](const auto& e) { return e.first == slave_index; });
        if (!known) {
            for (auto it = pending_controller_ops_.rbegin();
                 it != pending_controller_ops_.rend(); ++it) {
                if (it->kind == PendingControllerOp::Kind::Clear) break;
                if (it->slave_index != slave_index) continue;
                known = (it->kind == PendingControllerOp::Kind::Add);
                break;
            }
        }
        if (!known) {
            return false;
        }
        pending_controller_ops_.push_back(
            {PendingControllerOp::Kind::Remove, slave_index, {}});
        controller_ops_pending_.store(true, std::memory_order_release);
        return true;
    }

    return eraseController(slave_index);
}

void DS402Master::clearMotionControllers()
{
    if (realtimeLoopRunning()) {
        std::lock_guard<std::mutex> lock(motion_ctl_mutex_);
        pending_controller_ops_.push_back(
            {PendingControllerOp::Kind::Clear, 0, {}});
        controller_ops_pending_.store(true, std::memory_order_release);
        return;
    }

    drainControllerOps();  // apply anything stranded by a prior run
    for (auto& entry : motion_controllers_) {
        if (entry.second) {
            if (auto* drive = findDriveRaw(entry.first)) {
                entry.second->stop(*drive);
            }
        }
    }
    motion_controllers_.clear();
}

bool DS402Master::addCyclicTask(std::unique_ptr<ICyclicTask> task)
{
    if (!task) {
        return false;
    }

    ICyclicTask* raw = task.get();
    owned_tasks_.push_back(std::move(task));
    cyclic_task_scheduler_.addTask(raw, TaskPhase::MotionControl, 128);
    return true;
}

bool DS402Master::addCyclicTask(std::unique_ptr<ICyclicTask> task, TaskPhase phase, uint8_t priority)
{
    if (!task) {
        return false;
    }

    ICyclicTask* raw = task.get();
    owned_tasks_.push_back(std::move(task));
    cyclic_task_scheduler_.addTask(raw, phase, priority);
    return true;
}

void DS402Master::clearCyclicTasks()
{
    cyclic_task_scheduler_.clear();
    if (realtimeLoopRunning()) {
        // Destruction deferred: the scheduler's current schedule snapshot
        // may still hold these pointers until the next executeAll() rebuild.
        std::lock_guard<std::mutex> lock(motion_ctl_mutex_);
        for (auto& t : owned_tasks_) {
            retired_tasks_.push_back(std::move(t));
        }
        owned_tasks_.clear();
        tasks_retired_.store(true, std::memory_order_release);
        return;
    }
    owned_tasks_.clear();
}

bool DS402Master::updateMotionControllers(double dt_seconds)
{
    // Apply queued add/remove/clear on this thread — the only thread that
    // iterates motion_controllers_ while a loop is running.
    if (controller_ops_pending_.load(std::memory_order_acquire) ||
        tasks_retired_.load(std::memory_order_acquire)) {
        drainControllerOps();
    }

    // Check if any slave is suspended by the supervisor.
    // Suspended slaves must not have their PDO data passed to motion
    // controllers — the slave is being re-initialized and its PDO buffers
    // are not valid.
    auto& supervisor = ethercat_master_.slaveSupervisor();

    for (auto& entry : motion_controllers_) {
        // Skip slaves that are suspended (in recovery)
        if (supervisor.isSlaveSuspended(entry.first)) {
            continue;
        }
        // A queued EraseDrive flips the role synchronously while the
        // controller erasure waits for this drain — the controller is
        // already dead weight; skip it rather than failing the cycle.
        if (!isManagedDrive(entry.first)) {
            continue;
        }
        auto* drive = driveBySlaveIndex(entry.first);
        if (drive == nullptr || !entry.second || !entry.second->update(*drive, dt_seconds)) {
            return false;
        }
    }

    return cyclic_task_scheduler_.executeAll(*this, dt_seconds);
}

bool DS402Master::startRealtimeMotionControlLoop()
{
    return startRealtimeMotionControlLoop(Master::RealtimeMotionLoopConfig{});
}

bool DS402Master::startRealtimeMotionControlLoop(const Master::RealtimeMotionLoopConfig& config)
{
    ethercat_master_.setMotionControlCallback(
        [this](double dt_seconds) { return updateMotionControllers(dt_seconds); });
    return ethercat_master_.startRealtimeMotionControlLoop(config);
}

bool DS402Master::startPollingMotionControlLoop()
{
    return startPollingMotionControlLoop(Master::PollingMotionLoopConfig{});
}

bool DS402Master::startPollingMotionControlLoop(const Master::PollingMotionLoopConfig& config)
{
    ethercat_master_.setMotionControlCallback(
        [this](double dt_seconds) { return updateMotionControllers(dt_seconds); });
    return ethercat_master_.startPollingMotionControlLoop(config);
}

void DS402Master::stopMotionControlLoop()
{
    ethercat_master_.stopMotionControlLoop();
    drainControllerOps();  // no more updates — apply leftovers on this thread
}

bool DS402Master::startCyclicLoop(const Master::CyclicLoopConfig& config)
{
    ethercat_master_.setMotionControlCallback(
        [this](double dt_seconds) { return updateMotionControllers(dt_seconds); });
    return ethercat_master_.startCyclicLoop(config);
}

Master::CyclicLoopGuard DS402Master::startCyclicLoopScoped(
    const Master::CyclicLoopConfig& config)
{
    ethercat_master_.setMotionControlCallback(
        [this](double dt_seconds) { return updateMotionControllers(dt_seconds); });
    return ethercat_master_.startCyclicLoopScoped(config);
}

void DS402Master::stopCyclicLoop()
{
    ethercat_master_.stopCyclicLoop();
    drainControllerOps();  // no more updates — apply leftovers on this thread
}

CiA402Drive* DS402Master::driveAt(size_t index)
{
    if (index >= drives_.size()) {
        return nullptr;
    }

    return drives_[index].get();
}

const CiA402Drive* DS402Master::driveAt(size_t index) const
{
    if (index >= drives_.size()) {
        return nullptr;
    }

    return drives_[index].get();
}

bool DS402Master::enableDrive(uint16_t slave_index, uint32_t timeout_ms)
{
    auto* drive = driveBySlaveIndex(slave_index);
    if (!drive) return false;
    if (slaveRole(slave_index) == SlaveRole::DynaDrive) {
        return drive->enableDynaDrive(timeout_ms);
    }
    return drive->enable(timeout_ms);
}

bool DS402Master::disableDrive(uint16_t slave_index)
{
    auto* drive = driveBySlaveIndex(slave_index);
    if (!drive) return false;
    if (slaveRole(slave_index) == SlaveRole::DynaDrive) {
        return drive->disableDynaDrive();
    }
    return drive->disable();
}

bool DS402Master::enableAllDrives(uint32_t timeout_ms)
{
    for (auto& drive : drives_) {
        if (!drive) continue;
        if (slaveRole(drive->slaveIndex()) == SlaveRole::DynaDrive) {
            if (!drive->enableDynaDrive(timeout_ms)) return false;
        } else {
            if (!drive->enable(timeout_ms)) return false;
        }
    }

    return true;
}

bool DS402Master::disableAllDrives()
{
    for (auto& drive : drives_) {
        if (!drive) continue;
        if (slaveRole(drive->slaveIndex()) == SlaveRole::DynaDrive) {
            if (!drive->disableDynaDrive()) return false;
        } else {
            if (!drive->disable()) return false;
        }
    }

    return true;
}

CiA402Drive* DS402Master::driveBySlaveIndex(uint16_t slave_index)
{
    if (!isManagedDrive(slave_index)) {
        return nullptr;
    }

    for (auto& drive : drives_) {
        if (drive && drive->slaveIndex() == slave_index) {
            return drive.get();
        }
    }

    return nullptr;
}

const CiA402Drive* DS402Master::driveBySlaveIndex(uint16_t slave_index) const
{
    if (!isManagedDrive(slave_index)) {
        return nullptr;
    }

    for (const auto& drive : drives_) {
        if (drive && drive->slaveIndex() == slave_index) {
            return drive.get();
        }
    }

    return nullptr;
}

CiA402Drive& DS402Master::ensureDrive(uint16_t slave_index)
{
    if (auto* existing_drive = driveBySlaveIndex(slave_index)) {
        return *existing_drive;
    }

    if (slaveRole(slave_index) == SlaveRole::NonDS402) {
        setSlaveAsDS402(slave_index);
    }

    drives_.push_back(std::make_unique<CiA402Drive>(ethercat_master_, slave_index));
    return *drives_.back();
}

void DS402Master::ensureSlaveRoleCapacity(uint16_t slave_index)
{
    if (slave_roles_.size() <= slave_index) {
        slave_roles_.resize(static_cast<size_t>(slave_index) + 1, SlaveRole::NonDS402);
    }
}

// ============================================================================
// Slave Recovery / Supervision
// ============================================================================

SlaveSupervisor& DS402Master::slaveSupervisor()
{
    return ethercat_master_.slaveSupervisor();
}

DS402Master::DS402RecoveryHandler::DS402RecoveryHandler(
    DS402Master& master,
    const std::vector<DriveConfiguration>& configs)
    : master_(master)
    , configs_(configs)
{
}

bool DS402Master::DS402RecoveryHandler::reinitializeSlave(uint16_t slave_index)
{
    // Find the configuration for this slave
    const DriveConfiguration* cfg = nullptr;
    for (const auto& c : configs_) {
        if (c.slave_index == slave_index) {
            cfg = &c;
            break;
        }
    }
    if (cfg == nullptr) {
        TETHER_LOGE("DS402Recovery", "{}: No configuration stored for recovery",
                    master_.ethercatMaster().slaveLogPrefix(slave_index).c_str());
        return false;
    }

    // Reset the drive's PDO registration state and remove stale PDO
    // mapping entries.  After a forced INIT, the old PDO entries in the
    // PDOManager are invalid and must be removed before re-registering.
    auto* drive = master_.driveBySlaveIndex(slave_index);
    if (drive != nullptr) {
        drive->resetPDORegistration();
    }
    master_.ethercatMaster().pdoForSlave(slave_index).mapping().remove_entries_for_slave(slave_index);

    // Re-configure the drive (this re-does mailbox, PDO, SM, and OP transition)
    DriveConfiguration reinit_cfg = *cfg;
    reinit_cfg.transition_to_operational = true;
    if (!master_.configureDrive(reinit_cfg)) {
        TETHER_LOGE("DS402Recovery", "{}: configureDrive() failed during recovery",
                    master_.ethercatMaster().slaveLogPrefix(slave_index).c_str());
        return false;
    }

    // Re-enable the drive (CiA 402 fault reset + enable)
    if (!master_.enableDrive(slave_index)) {
        TETHER_LOGE("DS402Recovery", "{}: enableDrive() failed during recovery",
                    master_.ethercatMaster().slaveLogPrefix(slave_index).c_str());
        return false;
    }

    TETHER_LOGI("DS402Recovery", "{}: Re-initialized and enabled successfully",
                master_.ethercatMaster().slaveLogPrefix(slave_index).c_str());
    return true;
}

bool DS402Master::enableSlaveRecovery(const RecoveryConfig& config,
                                       const std::vector<DriveConfiguration>& configs)
{
    auto& sup = slaveSupervisor();

    RecoveryConfig cfg = config;
    cfg.enabled = true;
    sup.configure(cfg);
    sup.setRecoveryHandler(
        std::make_unique<DS402RecoveryHandler>(*this, configs));

    return sup.start();
}

void DS402Master::disableSlaveRecovery()
{
    auto& sup = slaveSupervisor();
    sup.stop();
}

} // namespace EtherCAT