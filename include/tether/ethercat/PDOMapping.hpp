/**
 * @file PDOMapping.hpp
 * @brief PDOMapping — the PDO entry table value type.
 *
 * Split out of PDOManager.hpp.
 */

#pragma once

#include "tether/ethercat/PDOTypes.hpp"
#include "tether/ethercat/ProcessImage.hpp"

namespace EtherCAT {
namespace PDO {

// ============================================================================
// PDO Mapping Manager (value type – no transport dependency)
// ============================================================================

class PDOMapping {
public:
    /// Register an RxPDO (master→slave).  Storage is manager-owned — the
    /// caller receives an entry index and accesses the bytes through
    /// entryDataMut()/entryDataAs<T>() or an epoch-checked entryHandle().
    int  add_rxpdo(uint16_t slave_index, uint16_t size,
                   uint16_t pdo_index = 0x1600,
                   PDOAddressMode mode = PDOAddressMode::Position);

    int  add_txpdo(uint16_t slave_index, uint16_t size,
                   uint16_t pdo_index = 0x1A00,
                   PDOAddressMode mode = PDOAddressMode::Position);

    int  add_broadcast_rxpdo(uint16_t size, uint16_t physical_offset);
    int  add_broadcast_txpdo(uint16_t size, uint16_t physical_offset);

    void set_slave_configured_address(uint16_t slave_index, uint16_t configured_addr);

    /// Enable/disable the "Added RxPDO/TxPDO" registration logs
    /// (set by PDOManager from the pdo-configuration debug flag).
    void setAddLogging(bool enabled) { m_log_adds = enabled; }

    size_t         entry_count() const { return m_entry_count; }
    const PDOEntry* get_entry(size_t index) const;
    PDOEntry*       get_entry_mut(size_t index);
    void            clear();
    void            remove_entries_for_slave(uint16_t slave_index);

    // ---- Buffered-path data access (Q1) ---------------------------------
    /// Epoch-checked handle for a buffered entry.  offset stays -1 — the
    /// handle resolves to entry storage, not a process-image offset.
    /// The epoch changes on clear()/remove_entries_for_slave(), making
    /// stale handles fail resolve() instead of pointing at recycled slots.
    EntryHandle entryHandle(size_t index) const {
        EntryHandle h;
        h.index  = static_cast<uint32_t>(index);
        h.offset = -1;
        h.epoch  = m_epoch;
        return h;
    }
    /// Direct index access — the fast path for RT code that caches the
    /// pointer once after registration.  nullptr on out-of-range.
    /// Taking a storage pointer marks the entry `storage_bound`: image-mode
    /// exchanges keep bridging it so the cached pointer stays live.
    uint8_t* entryDataMut(size_t index) {
        if (index >= m_entry_count) return nullptr;
        m_entries[index].storage_bound = true;
        return m_entries[index].storage;
    }
    const uint8_t* entryData(size_t index) const {
        if (index >= m_entry_count) return nullptr;
        m_entries[index].storage_bound = true;
        return m_entries[index].storage;
    }
    /// Typed access — nullptr when sizeof(T) exceeds the registered size.
    template<typename T> T* entryDataAs(size_t index) {
        return (index < m_entry_count && sizeof(T) <= m_entries[index].data_size)
                   ? reinterpret_cast<T*>(entryDataMut(index)) : nullptr;
    }
    template<typename T> const T* entryDataAs(size_t index) const {
        return (index < m_entry_count && sizeof(T) <= m_entries[index].data_size)
                   ? reinterpret_cast<const T*>(entryData(index)) : nullptr;
    }
    /// Epoch-checked resolve — nullptr when the handle is stale or the
    /// entry is out of range.
    uint8_t* resolveMut(const EntryHandle& h) {
        return (h.epoch == m_epoch) ? entryDataMut(h.index) : nullptr;
    }
    const uint8_t* resolve(const EntryHandle& h) const {
        return (h.epoch == m_epoch) ? entryData(h.index) : nullptr;
    }
    uint32_t epoch() const { return m_epoch; }

    size_t total_rxpdo_bytes() const;
    size_t total_txpdo_bytes() const;

private:
    PDOEntry m_entries[kMaxPDOEntries];
    size_t   m_entry_count = 0;
    uint16_t m_slave_configured_addrs[kMaxPDOSlaves] = {0};
    uint32_t m_epoch = 0;   ///< bumped on clear()/remove_entries_for_slave()
    bool     m_log_adds = false;  ///< gate for the add_rxpdo/add_txpdo logs
};

} // namespace PDO
} // namespace EtherCAT
