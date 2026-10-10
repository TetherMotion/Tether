/**
 * @file PDOMapping.cpp
 * @brief PDOMapping implementation — the PDO entry table value type.
 *
 * Split out of PDOManager.cpp so the mapping data structure stands on its
 * own; PDOManager keeps the exchange/configuration machinery.
 */

#include "tether/ethercat/PDOManager.hpp"
#include "tether/platform/Platform.hpp"

#include <cstring>

namespace EtherCAT {

static const char* TAG = "ec_pdo_mgr";

namespace PDO {

int PDOMapping::add_rxpdo(uint16_t slave_index, uint16_t size,
                          uint16_t pdo_index, PDOAddressMode mode) {
    if (m_entry_count >= kMaxPDOEntries) {
        TETHER_LOGE(TAG,
            "Tether internal PDO entry limit reached ({} entries). This is a Tether limit, "
            "not a slave limit. Increase ECAT_PDO_MAX_ENTRIES in EtherCATConfig.hpp.",
            kMaxPDOEntries);
        return -1;
    }
    if (size == 0 || size > kMaxPDOSize) {
        TETHER_LOGE(TAG,
            "Invalid PDO size ({}). {}",
            size,
            (size > kMaxPDOSize)
                ? "This is a Tether limit, not a slave limit. "
                  "Increase ECAT_PDO_MAX_BUFFER_SIZE in EtherCATConfig.hpp."
                : "Size is zero.");
        return -1;
    }
    PDOEntry& e   = m_entries[m_entry_count];
    e.slave_index  = slave_index;
    e.direction    = PDODirection::RxPDO;
    e.address_mode = mode;
    std::memset(e.storage, 0, sizeof(e.storage));
    e.data_size    = size;
    e.pdo_index    = pdo_index;
    e.enabled      = true;
    e.error_count  = 0;
    e.success_count= 0;
    e.physical_offset = 0;
    if (slave_index < kMaxPDOSlaves)
        e.configured_address = m_slave_configured_addrs[slave_index];
    if (m_log_adds) {
        TETHER_LOGI(TAG, "Added RxPDO: slave={} size={} pdo=0x{:04x} mode={}",
                    slave_index, size, pdo_index, static_cast<int>(mode));
    }
    return static_cast<int>(m_entry_count++);
}

int PDOMapping::add_txpdo(uint16_t slave_index, uint16_t size,
                          uint16_t pdo_index, PDOAddressMode mode) {
    if (m_entry_count >= kMaxPDOEntries) {
        TETHER_LOGE(TAG,
            "Tether internal PDO entry limit reached ({} entries). This is a Tether limit, "
            "not a slave limit. Increase ECAT_PDO_MAX_ENTRIES in EtherCATConfig.hpp.",
            kMaxPDOEntries);
        return -1;
    }
    if (size == 0 || size > kMaxPDOSize) {
        TETHER_LOGE(TAG,
            "Invalid PDO size ({}). {}",
            size,
            (size > kMaxPDOSize)
                ? "This is a Tether limit, not a slave limit. "
                  "Increase ECAT_PDO_MAX_BUFFER_SIZE in EtherCATConfig.hpp."
                : "Size is zero.");
        return -1;
    }
    PDOEntry& e   = m_entries[m_entry_count];
    e.slave_index  = slave_index;
    e.direction    = PDODirection::TxPDO;
    e.address_mode = mode;
    std::memset(e.storage, 0, sizeof(e.storage));
    e.data_size    = size;
    e.pdo_index    = pdo_index;
    e.enabled      = true;
    e.error_count  = 0;
    e.success_count= 0;
    e.physical_offset = 0;
    if (slave_index < kMaxPDOSlaves)
        e.configured_address = m_slave_configured_addrs[slave_index];
    if (m_log_adds) {
        TETHER_LOGI(TAG, "Added TxPDO: slave={} size={} pdo=0x{:04x} mode={}",
                slave_index, size, pdo_index, static_cast<int>(mode));
    }
    return static_cast<int>(m_entry_count++);
}

int PDOMapping::add_broadcast_rxpdo(uint16_t size, uint16_t physical_offset) {
    if (m_entry_count >= kMaxPDOEntries) return -1;
    PDOEntry& e = m_entries[m_entry_count];
    e.slave_index     = 0xFFFF;
    e.direction       = PDODirection::RxPDO;
    e.address_mode    = PDOAddressMode::Broadcast;
    e.physical_offset = physical_offset;
    std::memset(e.storage, 0, sizeof(e.storage));
    e.data_size       = size;
    e.enabled         = true;
    e.error_count     = 0;
    e.success_count   = 0;
    TETHER_LOGI(TAG, "Added broadcast RxPDO: offset=0x{:04x} size={}", physical_offset, size);
    return static_cast<int>(m_entry_count++);
}

int PDOMapping::add_broadcast_txpdo(uint16_t size, uint16_t physical_offset) {
    if (m_entry_count >= kMaxPDOEntries) return -1;
    PDOEntry& e = m_entries[m_entry_count];
    e.slave_index     = 0xFFFF;
    e.direction       = PDODirection::TxPDO;
    e.address_mode    = PDOAddressMode::Broadcast;
    e.physical_offset = physical_offset;
    std::memset(e.storage, 0, sizeof(e.storage));
    e.data_size       = size;
    e.enabled         = true;
    e.error_count     = 0;
    e.success_count   = 0;
    TETHER_LOGI(TAG, "Added broadcast TxPDO: offset=0x{:04x} size={}", physical_offset, size);
    return static_cast<int>(m_entry_count++);
}

void PDOMapping::set_slave_configured_address(uint16_t slave_index, uint16_t configured_addr) {
    if (slave_index < kMaxPDOSlaves) {
        m_slave_configured_addrs[slave_index] = configured_addr;
        for (size_t i = 0; i < m_entry_count; i++) {
            if (m_entries[i].slave_index == slave_index)
                m_entries[i].configured_address = configured_addr;
        }
    }
}

const PDOEntry* PDOMapping::get_entry(size_t index) const {
    return (index < m_entry_count) ? &m_entries[index] : nullptr;
}

PDOEntry* PDOMapping::get_entry_mut(size_t index) {
    return (index < m_entry_count) ? &m_entries[index] : nullptr;
}

void PDOMapping::clear() {
    m_entry_count = 0;
    ++m_epoch;   // invalidate outstanding entryHandle()s
    std::memset(m_entries, 0, sizeof(m_entries));
}

void PDOMapping::remove_entries_for_slave(uint16_t slave_index) {
    size_t write = 0;
    for (size_t read = 0; read < m_entry_count; ++read) {
        if (m_entries[read].slave_index != slave_index) {
            if (write != read) {
                m_entries[write] = m_entries[read];
            }
            ++write;
        }
    }
    // zero out the vacated tail so stale pointers don't dangle
    for (size_t i = write; i < m_entry_count; ++i) {
        std::memset(&m_entries[i], 0, sizeof(PDOEntry));
    }
    m_entry_count = write;
    ++m_epoch;   // indices may have compacted — invalidate handles
}

size_t PDOMapping::total_rxpdo_bytes() const {
    size_t total = 0;
    for (size_t i = 0; i < m_entry_count; i++)
        if (m_entries[i].direction == PDODirection::RxPDO && m_entries[i].enabled)
            total += m_entries[i].data_size;
    return total;
}

size_t PDOMapping::total_txpdo_bytes() const {
    size_t total = 0;
    for (size_t i = 0; i < m_entry_count; i++)
        if (m_entries[i].direction == PDODirection::TxPDO && m_entries[i].enabled)
            total += m_entries[i].data_size;
    return total;
}

} // namespace PDO

} // namespace EtherCAT
