/**
 * @file LogicalAddressManager.cpp
 * @brief LogicalAddressManager implementation
 */

#include "tether/ethercat/LogicalAddressManager.hpp"
#include "tether/ethercat/ProcessImage.hpp"
#include "tether/ethercat/CyclicChannel.hpp"
#include "tether/ethercat/Types.hpp"
#include "raw/TxFailureDiagnostics.hpp"
#include "raw/LogicalAddressManagerInternal.hpp"

#include <algorithm>
#include <array>
#include <bit>
#include <chrono>
#include <cstring>
#include <ctime>

namespace EtherCAT {

static const char* TAG = "ec_logaddr";

// ============================================================================
// Constructor / Lifecycle
// ============================================================================

LogicalAddressManager::LogicalAddressManager(IPDOTransport& transport)
    : transport_(transport)
    , tx_diag_(std::make_unique<TxFailureDiagnostics>(
          [this] { return transport_.txFailureDiagnostics(); },
          [this] { return transport_.lastSendErrno(); },
          [this]() -> std::string {
              // Ring probe: APRD of AL_STATUS (0x0130) on the first slave —
              // distinguishes "host can't see replies" from "ring is
              // broken".  Runs on the monitor thread; async transport ops
              // are legal concurrently with the cyclic path.
              uint16_t al_status = 0;
              const bool alive = transport_.readRegister(0, 0x0130,
                                                         &al_status, 2, 20);
              return std::format("{} (AL_STATUS=0x{:04X})",
                      alive ? "ALIVE" : "FAILED — ring likely broken",
                      al_status);
          },
          TAG))
{
    std::memset(addr_map_, 0, sizeof(addr_map_));
    expected_wkc_.fill(kWkcUnknown);
}

LogicalAddressManager::~LogicalAddressManager() = default;

bool LogicalAddressManager::init() {
    if (initialized_) return true;
    std::memset(addr_map_, 0, sizeof(addr_map_));
    slave_log_base_.fill(kUnassigned);
    slave_log_size_.fill(0);
    next_free_log_ = 0;
    slave_count_ = 0;
    total_rxpdo_bytes_ = 0;
    total_txpdo_bytes_ = 0;
    stats_ = Stats{};
    initialized_ = true;
    tx_diag_->start();   // non-RT monitor: all exchange error logging lives there
    TETHER_LOGI(TAG, "Logical address manager initialized");
    return true;
}

void LogicalAddressManager::deinit() {
    tx_diag_->stop();
    std::memset(addr_map_, 0, sizeof(addr_map_));
    slave_log_base_.fill(kUnassigned);
    slave_log_size_.fill(0);
    next_free_log_ = 0;
    slave_count_ = 0;
    total_rxpdo_bytes_ = 0;
    total_txpdo_bytes_ = 0;
    initialized_ = false;
    const auto* df = debug_flags_.load(std::memory_order_relaxed);
    if (df && df->shutdown) {
        TETHER_LOGI(TAG, "Logical address manager deinitialized");
    }
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
            "buildAddressMap: slave_count {} exceeds Tether internal max {}. "
            "This is a Tether limit, not a slave limit. "
            "Increase ECAT_PDO_MAX_SLAVES in EtherCATConfig.hpp.",
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

    // Pass 2: assign logical addresses — each slave's [RxPDO][TxPDO]
    // region is laid out contiguously.  The window base is STICKY: once a
    // slave has been assigned a window its FMMU has been (or is being)
    // programmed against it, so rebuilds must append new slaves at the
    // end rather than re-pack the image — otherwise an earlier slave's
    // already-programmed FMMU points at the wrong logical region.
    for (uint16_t i = 0; i < slave_count; i++) {
        const auto& cfg = configs[i];
        bool has_rxpdo = (cfg.sm[2].type == PDO::SyncManagerType::ProcessOutput && cfg.rxpdo_size > 0);
        bool has_txpdo = (cfg.sm[3].type == PDO::SyncManagerType::ProcessInput && cfg.txpdo_size > 0);

        if (!has_rxpdo && !has_txpdo) continue;

        const uint32_t window = (has_rxpdo ? cfg.rxpdo_size : 0)
                              + (has_txpdo ? cfg.txpdo_size : 0);
        if (slave_log_base_[i] == kUnassigned ||
            slave_log_size_[i] != window) {
            // New slave, or a re-configured slave whose window size
            // changed — allocate a fresh window at the end (its FMMU is
            // reprogrammed against the new base in the same call).
            slave_log_base_[i] = next_free_log_;
            slave_log_size_[i] = window;
            next_free_log_    += window;
        }
        uint32_t offset = base_logical_addr_ + slave_log_base_[i];

        auto& entry = addr_map_[i];
        entry.active = true;

        if (has_rxpdo) {
            entry.rxpdo_logical_addr = offset;
            entry.rxpdo_length = cfg.rxpdo_size;
            offset += cfg.rxpdo_size;
        }

        if (has_txpdo) {
            entry.txpdo_logical_addr = offset;
            entry.txpdo_length = cfg.txpdo_size;
            offset += cfg.txpdo_size;
        }

        TETHER_LOGI(TAG, "{}: RxPDO log=0x{:08X} len={}  TxPDO log=0x{:08X} len={}",
                    slavePrefix(i).c_str(),
                    static_cast<unsigned long>(entry.rxpdo_logical_addr), entry.rxpdo_length,
                    static_cast<unsigned long>(entry.txpdo_logical_addr), entry.txpdo_length);
    }

    TETHER_LOGI(TAG, "Address map built: {} slaves, RxPDO={} bytes, TxPDO={} bytes, total={}",
                slave_count, total_rxpdo_bytes_, total_txpdo_bytes_,
                total_rxpdo_bytes_ + total_txpdo_bytes_);

    ensureCyclicPayload(next_free_log_);
    return true;
}

// ============================================================================
// buildAddressMapFromMultiPDO
// ============================================================================

bool LogicalAddressManager::buildAddressMapFromMultiPDO(
    const std::vector<PDO::MultiPDOSyncManagerConfig>* sm_configs,
    uint16_t slave_count) {

    if (!sm_configs || slave_count == 0) {
        TETHER_LOGW(TAG, "buildAddressMapFromMultiPDO: no configs or zero slave count");
        return false;
    }
    if (slave_count > PDO::kMaxPDOSlaves) {
        TETHER_LOGE(TAG,
            "buildAddressMapFromMultiPDO: slave_count {} exceeds max {}",
            slave_count, PDO::kMaxPDOSlaves);
        return false;
    }

    if (!initialized_) init();

    std::memset(addr_map_, 0, sizeof(addr_map_));
    slave_count_ = slave_count;
    total_rxpdo_bytes_ = 0;
    total_txpdo_bytes_ = 0;

    // Pass 1: compute total RxPDO and TxPDO sizes across all slaves
    for (uint16_t s = 0; s < slave_count; s++) {
        const auto& configs = sm_configs[s];
        for (const auto& sm_cfg : configs) {
            if (sm_cfg.type == PDO::SyncManagerType::ProcessOutput) {
                total_rxpdo_bytes_ += sm_cfg.totalLength();
            } else if (sm_cfg.type == PDO::SyncManagerType::ProcessInput) {
                total_txpdo_bytes_ += sm_cfg.totalLength();
            }
        }
    }

    // Pass 2: assign logical addresses and per-PDO entries — each slave's
    // SM regions are laid out contiguously in config order, matching the
    // order FMMUManager::configureFromMultiPDO packs them on the slave.
    // The window base is STICKY (see buildAddressMap): slaves configured
    // earlier keep the window their FMMU was programmed with; later
    // slaves are appended at the end.
    for (uint16_t s = 0; s < slave_count; s++) {
        const auto& configs = sm_configs[s];
        auto& entry = addr_map_[s];
        bool has_any = false;

        uint32_t window = 0;
        for (const auto& sm_cfg : configs) {
            if (sm_cfg.pdo_mappings.empty()) continue;
            if (sm_cfg.type == PDO::SyncManagerType::ProcessOutput ||
                sm_cfg.type == PDO::SyncManagerType::ProcessInput) {
                window += sm_cfg.totalLength();
            }
        }
        if (window > 0 &&
            (slave_log_base_[s] == kUnassigned ||
             slave_log_size_[s] != window)) {
            slave_log_base_[s] = next_free_log_;
            slave_log_size_[s] = window;
            next_free_log_    += window;
        }
        uint32_t offset = base_logical_addr_ + slave_log_base_[s];
        if (slave_log_base_[s] == kUnassigned) offset = base_logical_addr_;

        for (const auto& sm_cfg : configs) {
            if (sm_cfg.pdo_mappings.empty()) continue;

            bool is_output = (sm_cfg.type == PDO::SyncManagerType::ProcessOutput);
            bool is_input = (sm_cfg.type == PDO::SyncManagerType::ProcessInput);
            if (!is_output && !is_input) continue;

            has_any = true;
            uint16_t sm_total = sm_cfg.totalLength();
            uint32_t base_addr = offset;
            uint16_t pdo_offset = 0;

            if (is_output) {
                entry.rxpdo_logical_addr = base_addr;
                entry.rxpdo_length = sm_total;
            } else {
                entry.txpdo_logical_addr = base_addr;
                entry.txpdo_length = sm_total;
            }

            // Record per-PDO entries
            for (const auto& pdo : sm_cfg.pdo_mappings) {
                if (entry.pdo_entry_count >= SlaveLogicalAddr::kMaxPDOEntries) break;
                auto& pe = entry.pdo_entries[entry.pdo_entry_count];
                pe.pdo_index = pdo.pdo_index;
                pe.logical_addr = base_addr + pdo_offset;
                pe.length = pdo.size_bytes;
                pe.sm_index = sm_cfg.sm_index;
                pe.is_output = is_output;
                pdo_offset += pdo.size_bytes;
                entry.pdo_entry_count++;

                TETHER_LOGI(TAG, "{} PDO 0x{:04X}: log=0x{:08X} len={} SM{} ({})",
                            slavePrefix(s).c_str(), pdo.pdo_index, (unsigned long)pe.logical_addr,
                            pe.length, pe.sm_index, is_output ? "RxPDO" : "TxPDO");
            }

            offset += sm_total;
        }

        entry.active = has_any;
    }

    TETHER_LOGI(TAG, "Multi-PDO address map: {} slaves, RxPDO={}, TxPDO={}, total={}, {} PDO entries",
                slave_count, total_rxpdo_bytes_, total_txpdo_bytes_,
                total_rxpdo_bytes_ + total_txpdo_bytes_,
                [&] {
                    size_t total = 0;
                    for (uint16_t s = 0; s < slave_count; s++)
                        total += addr_map_[s].pdo_entry_count;
                    return total;
                }());

    ensureCyclicPayload(next_free_log_);
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
// Per-PDO address queries (multi-PDO mode)
// ============================================================================

uint32_t LogicalAddressManager::getPDOLogicalAddr(uint16_t slave_index, uint16_t pdo_index) const {
    if (slave_index >= slave_count_) return 0;
    const auto* pe = addr_map_[slave_index].findPDO(pdo_index);
    return pe ? pe->logical_addr : 0;
}

uint16_t LogicalAddressManager::getPDOLength(uint16_t slave_index, uint16_t pdo_index) const {
    if (slave_index >= slave_count_) return 0;
    const auto* pe = addr_map_[slave_index].findPDO(pdo_index);
    return pe ? pe->length : 0;
}

std::vector<LogicalAddressManager::PDOLogicalAddrEntry>
LogicalAddressManager::getSlavePDOLogicalAddrs(uint16_t slave_index) const {
    std::vector<PDOLogicalAddrEntry> result;
    if (slave_index >= slave_count_) return result;
    const auto& entry = addr_map_[slave_index];
    result.reserve(entry.pdo_entry_count);
    for (size_t i = 0; i < entry.pdo_entry_count; i++) {
        result.push_back(entry.pdo_entries[i]);
    }
    return result;
}

// ============================================================================
// Statistics
// ============================================================================

LogicalAddressManager::Stats LogicalAddressManager::getStats() const {
    Stats s = stats_;
    s.tx_diag_spawns = tx_diag_ ? tx_diag_->spawns() : 0;
    return s;
}
void LogicalAddressManager::resetStats() { stats_ = Stats{}; }

// ============================================================================
// exchangeAllLRW / exchangeLRWSlice
// ============================================================================

bool LogicalAddressManager::exchangeAllLRW(const PDO::PDOMapping& mapping) {
    // Early-out on zero LIVE bytes, not zero extent: with sticky windows a
    // slave reconfigured to no PDOs leaves its old window allocated as
    // dead space (next_free_log_ > 0), but there is nothing to exchange.
    if (total_rxpdo_bytes_ + total_txpdo_bytes_ == 0) return true;
    return exchangeLRWImpl(mapping, 0,
                           next_free_log_,
                           /*enforce_slice_limit=*/false);
}

void LogicalAddressManager::ensureCyclicPayload(uint32_t size) {
    if (size == 0 || cyclic_payload_size_ >= size) return;
    cyclic_payload_ = std::make_unique<uint8_t[]>(size);
    cyclic_payload_size_ = size;
}

// ============================================================================
// exchangeAllLRWCyclic — whole-image LRW over the reserved-slot fast path
// ============================================================================

size_t LogicalAddressManager::computeImageOffsets(
        const PDO::PDOMapping& mapping, int32_t* out, size_t cap) const {
    const size_t n = std::min(mapping.entry_count(), cap);
    for (size_t i = 0; i < n; ++i) out[i] = -1;
    if (!initialized_ || slave_count_ == 0) return n;

    std::array<uint32_t, PDO::kMaxPDOSlaves> rx_running{};
    std::array<uint32_t, PDO::kMaxPDOSlaves> tx_running{};
    for (size_t i = 0; i < mapping.entry_count(); i++) {
        const PDO::PDOEntry* e = mapping.get_entry(i);
        if (!e || !e->enabled) continue;
        if (e->slave_index >= slave_count_) continue;
        if (!addr_map_[e->slave_index].active) continue;
        const auto& addr = addr_map_[e->slave_index];
        uint32_t off;
        if (e->direction == PDO::PDODirection::RxPDO) {
            off = addr.rxpdo_logical_addr - base_logical_addr_
                + rx_running[e->slave_index];
            rx_running[e->slave_index] += e->data_size;
        } else {
            off = addr.txpdo_logical_addr - base_logical_addr_
                + tx_running[e->slave_index];
            tx_running[e->slave_index] += e->data_size;
        }
        // storage_bound entries keep the offset OUT of the image table —
        // the application holds a pointer into entry storage, so the
        // exchange bridges them (gather/scatter) even in image modes.
        if (i < n && e->data_size > 0 && !e->image_exclude &&
            !e->storage_bound) {
            out[i] = static_cast<int32_t>(off);
        }
    }

    // Forced-buffered: entries sharing a byte with a neighbour (bit-packed
    // PDOs — read-modify-write on a shared byte would race).
    for (size_t i = 0; i < n; ++i) {
        if (out[i] < 0) continue;
        const PDO::PDOEntry* ei = mapping.get_entry(i);
        const uint32_t a0 = static_cast<uint32_t>(out[i]);
        const uint32_t a1 = a0 + ei->data_size;
        for (size_t j = i + 1; j < n; ++j) {
            if (out[j] < 0) continue;
            const PDO::PDOEntry* ej = mapping.get_entry(j);
            const uint32_t b0 = static_cast<uint32_t>(out[j]);
            const uint32_t b1 = b0 + ej->data_size;
            if (a0 < b1 && b0 < a1) { out[i] = -1; out[j] = -1; }
        }
    }
    return n;
}

bool LogicalAddressManager::exchangeAllLRWCyclic(const PDO::PDOMapping& mapping,
                                                 uint32_t rx_timeout_ns,
                                                 ProcessImage* image) {
    if (!cyclicSend(mapping, image, rx_timeout_ns)) return false;
    return cyclicCollect(mapping, image);
}

} // namespace EtherCAT
