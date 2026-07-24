/**
 * @file LogicalAddressManager.cpp
 * @brief LogicalAddressManager implementation
 */

#include "tether/ethercat/LogicalAddressManager.hpp"
#include "tether/ethercat/Types.hpp"

#include <cstring>

namespace EtherCAT {

static const char* TAG = "ec_logaddr";
static constexpr uint32_t kBaseLogicalAddress = 0x10000;

// ============================================================================
// Constructor / Lifecycle
// ============================================================================

LogicalAddressManager::LogicalAddressManager(IPDOTransport& transport)
    : transport_(transport)
{
    std::memset(addr_map_, 0, sizeof(addr_map_));
}

bool LogicalAddressManager::init() {
    if (initialized_) return true;
    std::memset(addr_map_, 0, sizeof(addr_map_));
    slave_count_ = 0;
    total_rxpdo_bytes_ = 0;
    total_txpdo_bytes_ = 0;
    stats_ = Stats{};
    initialized_ = true;
    TETHER_LOGI(TAG, "Logical address manager initialized");
    return true;
}

void LogicalAddressManager::deinit() {
    std::memset(addr_map_, 0, sizeof(addr_map_));
    slave_count_ = 0;
    total_rxpdo_bytes_ = 0;
    total_txpdo_bytes_ = 0;
    initialized_ = false;
    TETHER_LOGI(TAG, "Logical address manager deinitialized");
}

// ============================================================================
// buildAddressMap
// ============================================================================

bool LogicalAddressManager::buildAddressMap(const PDO::SlaveConfig* configs,
                                             uint16_t slave_count) {
    if (!configs || slave_count == 0) {
        TETHER_LOGW(TAG, "buildAddressMap: no configs or zero slave count");
        return false;
    }
    if (slave_count > PDO::kMaxPDOSlaves) {
        TETHER_LOGE(TAG,
            "buildAddressMap: slave_count %u exceeds Tether internal max %zu. "
            "This is a Tether limit, not a slave limit. "
            "Increase ECAT_PDO_MAX_SLAVES in TetherConfig.hpp.",
            slave_count, PDO::kMaxPDOSlaves);
        return false;
    }

    if (!initialized_) init();

    std::memset(addr_map_, 0, sizeof(addr_map_));
    slave_count_ = slave_count;
    total_rxpdo_bytes_ = 0;
    total_txpdo_bytes_ = 0;

    // Pass 1: compute total RxPDO size
    for (uint16_t i = 0; i < slave_count; i++) {
        const auto& cfg = configs[i];
        if (cfg.sm[2].type == PDO::SyncManagerType::ProcessOutput && cfg.rxpdo_size > 0) {
            total_rxpdo_bytes_ += cfg.rxpdo_size;
        }
        if (cfg.sm[3].type == PDO::SyncManagerType::ProcessInput && cfg.txpdo_size > 0) {
            total_txpdo_bytes_ += cfg.txpdo_size;
        }
    }

    // Pass 2: assign logical addresses
    uint32_t rxpdo_offset = kBaseLogicalAddress;
    uint32_t txpdo_offset = kBaseLogicalAddress + total_rxpdo_bytes_;

    for (uint16_t i = 0; i < slave_count; i++) {
        const auto& cfg = configs[i];
        bool has_rxpdo = (cfg.sm[2].type == PDO::SyncManagerType::ProcessOutput && cfg.rxpdo_size > 0);
        bool has_txpdo = (cfg.sm[3].type == PDO::SyncManagerType::ProcessInput && cfg.txpdo_size > 0);

        if (!has_rxpdo && !has_txpdo) continue;

        auto& entry = addr_map_[i];
        entry.active = true;

        if (has_rxpdo) {
            entry.rxpdo_logical_addr = rxpdo_offset;
            entry.rxpdo_length = cfg.rxpdo_size;
            rxpdo_offset += cfg.rxpdo_size;
        }

        if (has_txpdo) {
            entry.txpdo_logical_addr = txpdo_offset;
            entry.txpdo_length = cfg.txpdo_size;
            txpdo_offset += cfg.txpdo_size;
        }

        TETHER_LOGI(TAG, "Slave %u: RxPDO log=0x%08lX len=%u  TxPDO log=0x%08lX len=%u",
                    i,
                    static_cast<unsigned long>(entry.rxpdo_logical_addr), entry.rxpdo_length,
                    static_cast<unsigned long>(entry.txpdo_logical_addr), entry.txpdo_length);
    }

    TETHER_LOGI(TAG, "Address map built: %u slaves, RxPDO=%u bytes, TxPDO=%u bytes, total=%u",
                slave_count, total_rxpdo_bytes_, total_txpdo_bytes_,
                total_rxpdo_bytes_ + total_txpdo_bytes_);

    return true;
}

// ============================================================================
// FMMU configuration queries
// ============================================================================

uint32_t LogicalAddressManager::getRxPDOLogicalAddr(uint16_t slave_index) const {
    if (slave_index >= slave_count_) return 0;
    return addr_map_[slave_index].rxpdo_logical_addr;
}

uint16_t LogicalAddressManager::getRxPDOLength(uint16_t slave_index) const {
    if (slave_index >= slave_count_) return 0;
    return addr_map_[slave_index].rxpdo_length;
}

uint32_t LogicalAddressManager::getTxPDOLogicalAddr(uint16_t slave_index) const {
    if (slave_index >= slave_count_) return 0;
    return addr_map_[slave_index].txpdo_logical_addr;
}

uint16_t LogicalAddressManager::getTxPDOLength(uint16_t slave_index) const {
    if (slave_index >= slave_count_) return 0;
    return addr_map_[slave_index].txpdo_length;
}

bool LogicalAddressManager::hasSlavePDOs(uint16_t slave_index) const {
    if (slave_index >= slave_count_) return false;
    return addr_map_[slave_index].active;
}

// ============================================================================
// Statistics
// ============================================================================

LogicalAddressManager::Stats LogicalAddressManager::getStats() const { return stats_; }
void LogicalAddressManager::resetStats() { stats_ = Stats{}; }

// ============================================================================
// exchangeAllLRW
// ============================================================================

bool LogicalAddressManager::exchangeAllLRW(const PDO::PDOMapping& mapping) {
    if (!initialized_ || slave_count_ == 0) {
        TETHER_LOGW(TAG, "exchangeAllLRW: not initialized or no slaves");
        stats_.send_errors++;
        return false;
    }

    const uint32_t total_data = total_rxpdo_bytes_ + total_txpdo_bytes_;
    if (total_data == 0) return true;
    if (total_data > PDO::kMaxPDOSize * PDO::kMaxPDOSlaves) {
        TETHER_LOGE(TAG,
            "exchangeAllLRW: total data %u exceeds Tether internal buffer capacity "
            "(max=%zu = %zu bytes/slave * %zu slaves). This is a Tether limit, not a slave limit. "
            "Increase ECAT_PDO_MAX_BUFFER_SIZE or ECAT_PDO_MAX_SLAVES in TetherConfig.hpp.",
            total_data, PDO::kMaxPDOSize * PDO::kMaxPDOSlaves,
            PDO::kMaxPDOSize, PDO::kMaxPDOSlaves);
        stats_.send_errors++;
        return false;
    }

    // Build payload: RxPDO data + TxPDO space
    // Max payload = all slaves * max PDO size
    static constexpr size_t kMaxLRWPayload = PDO::kMaxPDOSize * PDO::kMaxPDOSlaves;
    uint8_t payload[kMaxLRWPayload];
    std::memset(payload, 0, total_data);

    // Fill RxPDO (write) portion from app buffers
    for (size_t i = 0; i < mapping.entry_count(); i++) {
        const PDO::PDOEntry* e = mapping.get_entry(i);
        if (!e || !e->enabled || e->direction != PDO::PDODirection::RxPDO) continue;
        if (e->slave_index >= slave_count_) continue;
        if (!addr_map_[e->slave_index].active) continue;

        const auto& addr = addr_map_[e->slave_index];
        const uint32_t offset = addr.rxpdo_logical_addr - kBaseLogicalAddress;
        if (e->app_buffer && e->data_size > 0 &&
            offset + e->data_size <= total_rxpdo_bytes_) {
            std::memcpy(payload + offset, e->app_buffer, e->data_size);
        }
    }

    // Send LRW datagram
    const uint8_t idx = transport_.allocIdx();
    const uint32_t logical_addr = kBaseLogicalAddress; // RxPDO region starts at base
    const uint16_t adp = static_cast<uint16_t>(logical_addr & 0xFFFF);
    const uint16_t ado = static_cast<uint16_t>((logical_addr >> 16) & 0xFFFF);

    if (!transport_.sendSingleDatagram(Command::LRW, idx, adp, ado,
                                        payload, static_cast<uint16_t>(total_data),
                                        true)) {
        TETHER_LOGE(TAG, "exchangeAllLRW: send failed");
        stats_.send_errors++;
        return false;
    }

    // Wait for response
    RxDatagram resp;
    if (!transport_.waitForResponseIdx(idx, 10, resp)) {
        TETHER_LOGE(TAG, "exchangeAllLRW: response timeout");
        stats_.timeout_errors++;
        return false;
    }

    if (resp.wkc == 0) {
        TETHER_LOGW(TAG, "exchangeAllLRW: WKC=0");
        stats_.wkc_errors++;
        return false;
    }

    // Extract TxPDO (read) data from response into app buffers
    if (resp.datalen >= total_data) {
        const uint8_t* rx_data = resp.data + total_rxpdo_bytes_;
        for (size_t i = 0; i < mapping.entry_count(); i++) {
            const PDO::PDOEntry* e = mapping.get_entry(i);
            if (!e || !e->enabled || e->direction != PDO::PDODirection::TxPDO) continue;
            if (e->slave_index >= slave_count_) continue;
            if (!addr_map_[e->slave_index].active) continue;

            const auto& addr = addr_map_[e->slave_index];
            const uint32_t offset = addr.txpdo_logical_addr - kBaseLogicalAddress - total_rxpdo_bytes_;
            if (e->app_buffer && e->data_size > 0 &&
                offset + e->data_size <= total_txpdo_bytes_) {
                std::memcpy(e->app_buffer, rx_data + offset, e->data_size);
            }
        }
    }

    stats_.success++;
    return true;
}

// ============================================================================
// exchangeLRWForSlaves
// ============================================================================

bool LogicalAddressManager::exchangeLRWForSlaves(const PDO::PDOMapping& mapping,
                                                   uint32_t slave_mask) {
    if (!initialized_ || slave_count_ == 0) {
        TETHER_LOGW(TAG, "exchangeLRWForSlaves: not initialized or no slaves");
        stats_.send_errors++;
        return false;
    }
    if (slave_mask == 0) return true;

    // Pass 1: compute compacted sizes for included slaves
    uint32_t compact_rxpdo = 0;
    uint32_t compact_txpdo = 0;

    for (uint16_t s = 0; s < slave_count_; s++) {
        if (!(slave_mask & (1u << s))) continue;
        if (!addr_map_[s].active) continue;
        compact_rxpdo += addr_map_[s].rxpdo_length;
        compact_txpdo += addr_map_[s].txpdo_length;
    }

    const uint32_t total_data = compact_rxpdo + compact_txpdo;
    if (total_data == 0) return true;

    static constexpr size_t kMaxLRWPayload = PDO::kMaxPDOSize * PDO::kMaxPDOSlaves;
    uint8_t payload[kMaxLRWPayload];
    std::memset(payload, 0, total_data);

    // Pass 2: fill RxPDO data and build compact offsets
    uint32_t rxpdo_off = 0;
    uint32_t txpdo_off = compact_rxpdo;

    for (size_t i = 0; i < mapping.entry_count(); i++) {
        const PDO::PDOEntry* e = mapping.get_entry(i);
        if (!e || !e->enabled) continue;
        if (!(slave_mask & (1u << e->slave_index))) continue;
        if (e->slave_index >= slave_count_) continue;
        if (!addr_map_[e->slave_index].active) continue;

        if (e->direction == PDO::PDODirection::RxPDO) {
            if (e->app_buffer && e->data_size > 0 &&
                rxpdo_off + e->data_size <= compact_rxpdo) {
                std::memcpy(payload + rxpdo_off, e->app_buffer, e->data_size);
            }
            rxpdo_off += e->data_size;
        }
    }

    // Send LRW datagram
    const uint8_t idx = transport_.allocIdx();
    const uint32_t logical_addr = kBaseLogicalAddress;
    const uint16_t adp = static_cast<uint16_t>(logical_addr & 0xFFFF);
    const uint16_t ado = static_cast<uint16_t>((logical_addr >> 16) & 0xFFFF);

    if (!transport_.sendSingleDatagram(Command::LRW, idx, adp, ado,
                                        payload, static_cast<uint16_t>(total_data),
                                        true)) {
        TETHER_LOGE(TAG, "exchangeLRWForSlaves: send failed (mask=0x%08lX)",
                    static_cast<unsigned long>(slave_mask));
        stats_.send_errors++;
        return false;
    }

    RxDatagram resp;
    if (!transport_.waitForResponseIdx(idx, 10, resp)) {
        TETHER_LOGE(TAG, "exchangeLRWForSlaves: response timeout");
        stats_.timeout_errors++;
        return false;
    }

    if (resp.wkc == 0) {
        TETHER_LOGW(TAG, "exchangeLRWForSlaves: WKC=0");
        stats_.wkc_errors++;
        return false;
    }

    // Extract TxPDO data
    if (resp.datalen >= total_data) {
        const uint8_t* rx_data = resp.data + compact_rxpdo;
        uint32_t tx_off = 0;
        for (size_t i = 0; i < mapping.entry_count(); i++) {
            const PDO::PDOEntry* e = mapping.get_entry(i);
            if (!e || !e->enabled || e->direction != PDO::PDODirection::TxPDO) continue;
            if (!(slave_mask & (1u << e->slave_index))) continue;
            if (e->slave_index >= slave_count_) continue;
            if (!addr_map_[e->slave_index].active) continue;

            if (e->app_buffer && e->data_size > 0 &&
                tx_off + e->data_size <= compact_txpdo) {
                std::memcpy(e->app_buffer, rx_data + tx_off, e->data_size);
            }
            tx_off += e->data_size;
        }
    }

    stats_.success++;
    return true;
}

} // namespace EtherCAT
