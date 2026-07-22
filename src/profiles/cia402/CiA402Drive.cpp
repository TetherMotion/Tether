/**
 * @file CiA402Drive.cpp
 * @brief CiA 402 Drive Controller - Constructor and DriveManager
 * 
 * This is the main entry file for CiA402Drive. The implementation is split across:
 * - CiA402Drive.cpp (this file): Constructor and DriveManager
 * - CiA402DriveStateMachine.cpp: State machine, enable/disable, homing
 * - CiA402DrivePDO.cpp: PDO mapping, buffer handling, cyclic update
 */

#include "profiles/cia402/CiA402Drive.hpp"
#include "tether/ethercat/Master.hpp"
#include "tether/ethercat/SDOManager.hpp"
#include "tether/ethercat/PDOManager.hpp"
#include "tether/ethercat/DC.hpp"
#include "tether/platform/EspCompat.hpp"

#include <cstring>
#include <memory>

static const char* TAG = "CiA402Drive";

namespace EtherCAT {

// ============================================================================
// CiA402Drive Constructor
// ============================================================================

CiA402Drive::CiA402Drive(Master& master, uint16_t slave_index)
    : m_slave_index(slave_index)
    , m_master(&master)
{
    std::memset(m_rxpdo_buffer, 0, sizeof(m_rxpdo_buffer));
    std::memset(m_txpdo_buffer, 0, sizeof(m_txpdo_buffer));
}

// ============================================================================
// DriveManager Implementation
// ============================================================================
// Note: no explicit destructor needed — std::unique_ptr in m_drives handles
// cleanup automatically and is exception-safe.

size_t DriveManager::initializeDrives(Master& master, size_t slave_count) {
    m_drive_count = 0;

    for (size_t i = 0; i < slave_count && m_drive_count < kMaxManagedDrives; i++) {
        // Check if slave is a CiA 402 drive by reading device type
        // For simplicity, assume all slaves are drives for now
        // In real implementation, use SlaveIdentifier first

        m_drives[m_drive_count] = std::make_unique<CiA402Drive>(master, static_cast<uint16_t>(i));
        m_drive_count++;
    }
    
    TETHER_LOGI(TAG, "DriveManager: Initialized %zu drives", m_drive_count);
    return m_drive_count;
}

CiA402Drive* DriveManager::getDrive(size_t index) {
    if (index < m_drive_count) {
        return m_drives[index].get();
    }
    return nullptr;
}

CiA402Drive* DriveManager::getDriveBySlaveIndex(uint16_t slave_index) {
    for (size_t i = 0; i < m_drive_count; i++) {
        if (m_drives[i] && m_drives[i]->slaveIndex() == slave_index) {
            return m_drives[i].get();
        }
    }
    return nullptr;
}

bool DriveManager::transitionAllToOp() {
    bool all_success = true;
    for (size_t i = 0; i < m_drive_count; i++) {
        if (m_drives[i] && !m_drives[i]->transitionToOp()) {
            all_success = false;
        }
    }
    return all_success;
}

bool DriveManager::enableAll() {
    bool all_success = true;
    for (size_t i = 0; i < m_drive_count; i++) {
        if (m_drives[i] && !m_drives[i]->enable()) {
            all_success = false;
        }
    }
    return all_success;
}

bool DriveManager::disableAll() {
    bool all_success = true;
    for (size_t i = 0; i < m_drive_count; i++) {
        if (m_drives[i] && !m_drives[i]->disable()) {
            all_success = false;
        }
    }
    return all_success;
}

} // namespace EtherCAT
