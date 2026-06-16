#include "tether/profiles/cia402/DS402Master.hpp"

#include "tether/platform/Platform.hpp"

#include <algorithm>

namespace EtherCAT {

DS402Master::DS402Master() = default;

DS402Master::DS402Master(const Master::Config& config)
    : ethercat_master_(config)
{
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
        (void)removeMotionController(slave_index);
        drives_.erase(
            std::remove_if(drives_.begin(), drives_.end(),
                           [slave_index](const std::unique_ptr<CiA402Drive>& drive) {
                               return drive && drive->slaveIndex() == slave_index;
                           }),
            drives_.end());
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

    removeMotionController(slave_index);
    motion_controllers_.emplace_back(slave_index, std::move(controller));
    return true;
}

bool DS402Master::removeMotionController(uint16_t slave_index)
{
    auto it = std::find_if(
        motion_controllers_.begin(), motion_controllers_.end(),
        [slave_index](const auto& entry) { return entry.first == slave_index; });
    if (it == motion_controllers_.end()) {
        return false;
    }

    if (auto* drive = driveBySlaveIndex(slave_index)) {
        it->second->stop(*drive);
    }
    motion_controllers_.erase(it);
    return true;
}

void DS402Master::clearMotionControllers()
{
    for (auto& entry : motion_controllers_) {
        if (entry.second) {
            if (auto* drive = driveBySlaveIndex(entry.first)) {
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

    cyclic_tasks_.push_back(std::move(task));
    return true;
}

void DS402Master::clearCyclicTasks()
{
    cyclic_tasks_.clear();
}

bool DS402Master::updateMotionControllers(double dt_seconds)
{
    for (auto& entry : motion_controllers_) {
        auto* drive = driveBySlaveIndex(entry.first);
        if (drive == nullptr || !entry.second || !entry.second->update(*drive, dt_seconds)) {
            return false;
        }
    }

    for (auto& task : cyclic_tasks_) {
        if (task && !task->update(*this, dt_seconds)) {
            return false;
        }
    }

    return true;
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

} // namespace EtherCAT