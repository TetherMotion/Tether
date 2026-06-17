/**
 * @file CiA402DrivePDO.cpp
 * @brief CiA 402 Drive — Fixed PDO assignment and buffer registration
 *
 * Implements assignFixedPDOs() and registerPDOBuffers().
 */

#include "profiles/cia402/CiA402Drive.hpp"
#include "tether/ethercat/Master.hpp"
#include "SDOManager.hpp"
#include "PDOManager.hpp"
#include "tether/platform/EspCompat.hpp"

#include <cstring>
#include <cinttypes>

static const char* TAG = "CiA402PDO";

namespace EtherCAT {

// ============================================================================
// assignFixedPDOs
// ============================================================================

bool CiA402Drive::assignFixedPDOs(uint16_t rxpdo_index, uint16_t txpdo_index,
                                   uint16_t rxpdo_size, uint16_t txpdo_size) {
    // Store sizes first — they are used for SM/FMMU even if SDO fails.
    m_rxpdo_index = rxpdo_index;
    m_txpdo_index = txpdo_index;
    m_rxpdo_size  = rxpdo_size;
    m_txpdo_size  = txpdo_size;
    m_pdo_configured = true;

    TETHER_LOGI(TAG, "Slave %u: assignFixedPDOs RxPDO=0x%04X/%u bytes, TxPDO=0x%04X/%u bytes",
             m_slave_index, rxpdo_index, rxpdo_size, txpdo_index, txpdo_size);

    uint8_t zero = 0;
    uint8_t one  = 1;
    bool sdo_ok = true;

    // SM2 (RxPDO) assignment — best effort
    if (!m_master->sdoManager(m_slave_index).writeU8( CiA301::SyncManager2PDOAssign, 0, zero, {.timeout_ms = m_sdo_timeout_ms}).has_value()) {
        TETHER_LOGW(TAG, "Slave %u: Failed to clear SM2 PDO count (may be fixed)", m_slave_index);
    }
    if (!m_master->sdoManager(m_slave_index).writeU16( CiA301::SyncManager2PDOAssign, 1, rxpdo_index, {.timeout_ms = m_sdo_timeout_ms}).has_value()) {
        TETHER_LOGW(TAG, "Slave %u: Failed to assign RxPDO 0x%04X to SM2", m_slave_index, rxpdo_index);
        sdo_ok = false;
    }
    if (!m_master->sdoManager(m_slave_index).writeU8( CiA301::SyncManager2PDOAssign, 0, one, {.timeout_ms = m_sdo_timeout_ms}).has_value()) {
        TETHER_LOGW(TAG, "Slave %u: Failed to set SM2 PDO count", m_slave_index);
    }

    // SM3 (TxPDO) assignment — best effort
    if (!m_master->sdoManager(m_slave_index).writeU8( CiA301::SyncManager3PDOAssign, 0, zero, {.timeout_ms = m_sdo_timeout_ms}).has_value()) {
        TETHER_LOGW(TAG, "Slave %u: Failed to clear SM3 PDO count (may be fixed)", m_slave_index);
    }
    if (!m_master->sdoManager(m_slave_index).writeU16( CiA301::SyncManager3PDOAssign, 1, txpdo_index, {.timeout_ms = m_sdo_timeout_ms}).has_value()) {
        TETHER_LOGW(TAG, "Slave %u: Failed to assign TxPDO 0x%04X to SM3", m_slave_index, txpdo_index);
        sdo_ok = false;
    }
    if (!m_master->sdoManager(m_slave_index).writeU8( CiA301::SyncManager3PDOAssign, 0, one, {.timeout_ms = m_sdo_timeout_ms}).has_value()) {
        TETHER_LOGW(TAG, "Slave %u: Failed to set SM3 PDO count", m_slave_index);
    }

    if (!sdo_ok) {
        TETHER_LOGW(TAG, "Slave %u: PDO SDO assignment had failures; using configured sizes anyway", m_slave_index);
    }

    return true;
}

// ============================================================================
// registerPDOBuffers
// ============================================================================

bool CiA402Drive::registerPDOBuffers() {
    if (m_pdo_registered) {
        TETHER_LOGW(TAG, "Slave %u: PDO buffers already registered", m_slave_index);
        return true;
    }

    PDO::PDOMapping* mapping = &m_master->pdo().mapping();

    if (m_rxpdo_size > 0) {
        m_rxpdo_entry_index = mapping->add_rxpdo(
            m_slave_index,
            m_rxpdo_buffer,
            m_rxpdo_size,
            m_rxpdo_index,
            PDO::PDOAddressMode::Position
        );
        if (m_rxpdo_entry_index < 0) {
            TETHER_LOGE(TAG, "Slave %u: Failed to register RxPDO buffer!", m_slave_index);
            return false;
        }
        TETHER_LOGI(TAG, "Slave %u: Registered RxPDO %u bytes (entry %d)",
                 m_slave_index, m_rxpdo_size, m_rxpdo_entry_index);
    }

    if (m_txpdo_size > 0) {
        m_txpdo_entry_index = mapping->add_txpdo(
            m_slave_index,
            m_txpdo_buffer,
            m_txpdo_size,
            m_txpdo_index,
            PDO::PDOAddressMode::Position
        );
        if (m_txpdo_entry_index < 0) {
            TETHER_LOGE(TAG, "Slave %u: Failed to register TxPDO buffer!", m_slave_index);
            return false;
        }
        TETHER_LOGI(TAG, "Slave %u: Registered TxPDO %u bytes (entry %d)",
                 m_slave_index, m_txpdo_size, m_txpdo_entry_index);
    }

    m_pdo_registered = true;
    return true;
}

} // namespace EtherCAT
