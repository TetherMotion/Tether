/**
 * @file LogicalAddressManager.cpp
 * @brief LogicalAddressManager implementation
 */

#include "tether/ethercat/LogicalAddressManager.hpp"
#include "tether/ethercat/ProcessImage.hpp"
#include "tether/ethercat/CyclicChannel.hpp"
#include "tether/ethercat/Types.hpp"

#include <algorithm>
#include <array>
#include <cstring>
#include <ctime>

namespace EtherCAT {

static const char* TAG = "ec_logaddr";

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

    // Pass 2: assign logical addresses
    uint32_t rxpdo_offset = base_logical_addr_;
    uint32_t txpdo_offset = base_logical_addr_ + total_rxpdo_bytes_;

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

        TETHER_LOGI(TAG, "{}: RxPDO log=0x{:08X} len={}  TxPDO log=0x{:08X} len={}",
                    slavePrefix(i).c_str(),
                    static_cast<unsigned long>(entry.rxpdo_logical_addr), entry.rxpdo_length,
                    static_cast<unsigned long>(entry.txpdo_logical_addr), entry.txpdo_length);
    }

    TETHER_LOGI(TAG, "Address map built: {} slaves, RxPDO={} bytes, TxPDO={} bytes, total={}",
                slave_count, total_rxpdo_bytes_, total_txpdo_bytes_,
                total_rxpdo_bytes_ + total_txpdo_bytes_);

    ensureCyclicPayload(total_rxpdo_bytes_ + total_txpdo_bytes_);
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

    // Pass 2: assign logical addresses and per-PDO entries
    uint32_t rxpdo_offset = base_logical_addr_;
    uint32_t txpdo_offset = base_logical_addr_ + total_rxpdo_bytes_;

    for (uint16_t s = 0; s < slave_count; s++) {
        const auto& configs = sm_configs[s];
        auto& entry = addr_map_[s];
        bool has_any = false;

        for (const auto& sm_cfg : configs) {
            if (sm_cfg.pdo_mappings.empty()) continue;

            bool is_output = (sm_cfg.type == PDO::SyncManagerType::ProcessOutput);
            bool is_input = (sm_cfg.type == PDO::SyncManagerType::ProcessInput);
            if (!is_output && !is_input) continue;

            has_any = true;
            uint16_t sm_total = sm_cfg.totalLength();
            uint32_t base_addr = is_output ? rxpdo_offset : txpdo_offset;
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

                TETHER_LOGI(TAG, "Slave {} PDO 0x{:04X}: log=0x{:08X} len={} SM{} ({})",
                            s, pdo.pdo_index, (unsigned long)pe.logical_addr,
                            pe.length, pe.sm_index, is_output ? "RxPDO" : "TxPDO");
            }

            if (is_output) {
                rxpdo_offset += sm_total;
            } else {
                txpdo_offset += sm_total;
            }
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

    ensureCyclicPayload(total_rxpdo_bytes_ + total_txpdo_bytes_);
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

LogicalAddressManager::Stats LogicalAddressManager::getStats() const { return stats_; }
void LogicalAddressManager::resetStats() { stats_ = Stats{}; }

// ============================================================================
// exchangeAllLRW / exchangeLRWSlice
// ============================================================================

bool LogicalAddressManager::exchangeAllLRW(const PDO::PDOMapping& mapping) {
    return exchangeLRWImpl(mapping, 0,
                           total_rxpdo_bytes_ + total_txpdo_bytes_,
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
        if (i < n && e->data_size > 0 && !e->image_exclude) {
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

static uint64_t monoNowNs() {
    timespec ts{};
    ::clock_gettime(CLOCK_MONOTONIC, &ts);
    return static_cast<uint64_t>(ts.tv_sec) * 1'000'000'000ull
         + static_cast<uint64_t>(ts.tv_nsec);
}

bool LogicalAddressManager::exchangeAllLRWCyclic(const PDO::PDOMapping& mapping,
                                                 uint32_t rx_timeout_ns,
                                                 ProcessImage* image) {
    if (!cyclicSend(mapping, image, rx_timeout_ns)) return false;
    return cyclicCollect(mapping, image);
}

// ============================================================================
// cyclicSend — gather outputs + emit all LRW slices
// ============================================================================

bool LogicalAddressManager::cyclicSend(const PDO::PDOMapping& mapping,
                                       ProcessImage* image,
                                       uint32_t rx_timeout_ns) {
    if (!transport_.supportsCyclicFastPath()) {
        // Legacy path stays atomic — no split support.
        return exchangeAllLRW(mapping);
    }
    if (!initialized_ || slave_count_ == 0) {
        stats_.send_errors++;
        return false;
    }
    const uint32_t total_data = total_rxpdo_bytes_ + total_txpdo_bytes_;
    if (total_data == 0) return true;
    if (total_data > cyclic_payload_size_) {
        ensureCyclicPayload(total_data);
        if (total_data > cyclic_payload_size_) {
            stats_.send_errors++;
            return false;
        }
    }

    // Slice count: one LRW datagram per maxSliceLength()-sized chunk,
    // sent on cyclic slots 0..N-1.  Wire time is the real constraint —
    // at 100 Mbit/s a max-size frame costs ~120 µs on the wire.
    const uint32_t max_slice = maxSliceLength();
    uint32_t nslices = max_slice ? (total_data + max_slice - 1) / max_slice
                                 : kMaxCyclicSlices + 1;
    if (nslices > kMaxCyclicSlices) {
        TETHER_LOGE(TAG, "cyclic image {}B needs {} slices > {} slots — "
                         "cannot exchange (raise slot count or shrink "
                         "the image)", total_data, nslices,
                    kMaxCyclicSlices);
        stats_.send_errors++;
        return false;
    }
    cyclic_slice_count_ = static_cast<uint8_t>(nslices);

    const bool img_active = image && image->configured() &&
                            image->mode() != ImageMode::Buffered;

    // ---- Choose the TX payload ------------------------------------------
    uint8_t* payload         = cyclic_payload_.get();
    uint8_t* rotating_frame  = nullptr;
    bool     rotating_send   = false;

    if (img_active && image->mode() == ImageMode::Rotating) {
        rotating_frame = image->rotatingFrameBase();
        if (!rotating_frame) {
            rotating_frame = transport_.acquireCyclicTxFrame();
            image->attachTxFrame(rotating_frame);
        }
        if (rotating_frame && nslices == 1) {
            payload        = rotating_frame + kCyclicFramePayloadOff;
            rotating_send  = true;
        } else if (rotating_frame) {
            // Rotating cannot span multiple frames — stage instead.
            TETHER_LOGW(TAG, "Rotating mode with {} slices — staging "
                             "payload instead (image > one frame)",
                        nslices);
            payload = const_cast<uint8_t*>(image->acquireSendImage());
            rotating_frame = nullptr;
        }
    } else if (img_active) {
        payload = const_cast<uint8_t*>(image->acquireSendImage());
    }

    if (!payload) {
        payload = cyclic_payload_.get();
        rotating_send = false;
    }

    if (!img_active) {
        std::memset(payload, 0, total_data);
    }

    // Gather RxPDO bytes from app buffers for entries that are NOT image
    // mapped (all entries on the legacy path; only -1-offset entries in
    // image modes).
    std::array<uint32_t, PDO::kMaxPDOSlaves> rx_running{};
    for (size_t i = 0; i < mapping.entry_count(); i++) {
        const PDO::PDOEntry* e = mapping.get_entry(i);
        if (!e || !e->enabled || e->direction != PDO::PDODirection::RxPDO) continue;
        if (e->slave_index >= slave_count_) continue;
        if (!addr_map_[e->slave_index].active) continue;

        const auto& addr = addr_map_[e->slave_index];
        const uint32_t entry_off = addr.rxpdo_logical_addr - base_logical_addr_
                                 + rx_running[e->slave_index];
        rx_running[e->slave_index] += e->data_size;
        if (!e->app_buffer || e->data_size == 0) continue;
        if (entry_off + e->data_size > total_data) break;  // layout guard
        if (img_active && image->entryOffset(i) >= 0) continue; // in-image

        std::memcpy(payload + entry_off, e->app_buffer, e->data_size);
    }

    // ---- Emit slices -----------------------------------------------------
    // Collect the response deadline once — the collect half shares it
    // regardless of when it runs (split-phase overlap).
    cyclic_deadline_ns_ = monoNowNs() + rx_timeout_ns;
    cyclic_pending_count_ = 0;
    pending_image_ = image;

    for (uint32_t s = 0; s < nslices; ++s) {
        const uint32_t off = s * max_slice;
        const uint32_t len = std::min(max_slice, total_data - off);
        const uint32_t logical_addr = base_logical_addr_ + off;
        const uint16_t adp = static_cast<uint16_t>(logical_addr & 0xFFFF);
        const uint16_t ado = static_cast<uint16_t>((logical_addr >> 16)
                                                 & 0xFFFF);
        const uint8_t slot = static_cast<uint8_t>(s);

        cyclic_pending_[s].token = transport_.cyclicSlotToken(slot);
        cyclic_pending_[s].off   = off;
        cyclic_pending_[s].len   = len;

        bool sent;
        if (rotating_send) {
            transport_.composeCyclicHeader(rotating_frame, Command::LRW,
                                           slot, adp, ado,
                                           static_cast<uint16_t>(len),
                                           true);
            *reinterpret_cast<uint16_t*>(
                rotating_frame + kCyclicFramePayloadOff + len) = 0;
            image->detachTxFrame();
            sent = transport_.sendCyclicFrame(
                kCyclicFramePayloadOff + len + sizeof(uint16_t));
            image->attachTxFrame(transport_.acquireCyclicTxFrame());
            rotating_send = false;   // single-slice path only
        } else {
            sent = transport_.sendCyclicDatagram(
                Command::LRW, slot, adp, ado, payload + off,
                static_cast<uint16_t>(len), true);
        }
        if (!sent) {
            stats_.send_errors++;
            cyclic_pending_count_ = 0;
            pending_image_ = nullptr;
            return false;
        }
        ++cyclic_pending_count_;
    }
    return true;
}

// ============================================================================
// cyclicCollect — wait slices, verify WKC, publish + scatter
// ============================================================================

bool LogicalAddressManager::cyclicCollect(const PDO::PDOMapping& mapping,
                                          ProcessImage* image) {
    if (!transport_.supportsCyclicFastPath()) {
        return true;   // cyclicSend already ran the atomic legacy exchange
    }
    const uint32_t total_data = total_rxpdo_bytes_ + total_txpdo_bytes_;
    const uint8_t nslices = cyclic_pending_count_;
    if (nslices == 0) return total_data == 0;
    image = pending_image_;
    const bool img_active = image && image->configured() &&
                            image->mode() != ImageMode::Buffered;

    bool ok = true;
    bool wkc_learn = !expected_wkc_valid_;
    std::array<const CyclicSlotView*, kMaxCyclicSlices> resps{};
    std::array<CyclicSlotView, kMaxCyclicSlices> views{};

    for (uint8_t s = 0; s < nslices; ++s) {
        const auto& pend = cyclic_pending_[s];
        const uint64_t now = monoNowNs();
        const uint32_t remaining = now < cyclic_deadline_ns_
            ? static_cast<uint32_t>(cyclic_deadline_ns_ - now) : 0;

        if (!transport_.waitCyclicSlotView(s, pend.token, remaining,
                                           views[s])) {
            stats_.timeout_errors++;
            ok = false;
            break;
        }
        const CyclicSlotView& resp = views[s];
        if (resp.wkc == 0 ||
            (!wkc_learn && strict_wkc_ && resp.wkc != expected_wkc_[s])) {
            stats_.wkc_errors++;
            ok = false;
            // keep draining remaining slices so slots don't go stale
            continue;
        }
        if (wkc_learn) expected_wkc_[s] = resp.wkc;
        resps[s] = &views[s];
    }
    cyclic_pending_count_ = 0;
    pending_image_ = nullptr;
    if (wkc_learn && ok) expected_wkc_valid_ = true;
    if (!ok) return false;

    // ---- Publish the input image --------------------------------------
    if (img_active) {
        if (nslices == 1 && resps[0] && resps[0]->payload) {
            const CyclicSlotView& resp = *resps[0];
            if (resp.channel) {
                image->publishInputView(resp.payload, resp.datalen,
                                        resp.cookie, resp.channel);
            } else {
                image->publishInputCopy(resp.payload, resp.datalen);
            }
        } else {
            // Multi-slice: stage each slice at its offset, one publish.
            uint8_t* bank = image->inputWriteBank();
            if (bank) {
                for (uint8_t s = 0; s < nslices; ++s) {
                    if (!resps[s] || !resps[s]->payload) continue;
                    const uint32_t off = cyclic_pending_[s].off;
                    const uint32_t n = std::min<uint32_t>(resps[s]->datalen,
                                                          total_data - off);
                    std::memcpy(bank + off, resps[s]->payload, n);
                }
                image->commitInput();
            }
        }
    }

    // ---- Scatter TxPDO data into app buffers (non-image entries) ------
    std::array<uint32_t, PDO::kMaxPDOSlaves> tx_running{};
    for (size_t i = 0; i < mapping.entry_count(); i++) {
        const PDO::PDOEntry* e = mapping.get_entry(i);
        if (!e || !e->enabled || e->direction != PDO::PDODirection::TxPDO) continue;
        if (e->slave_index >= slave_count_) continue;
        if (!addr_map_[e->slave_index].active) continue;

        const auto& addr = addr_map_[e->slave_index];
        const uint32_t entry_off = addr.txpdo_logical_addr - base_logical_addr_
                                 + tx_running[e->slave_index];
        tx_running[e->slave_index] += e->data_size;
        if (!e->app_buffer || e->data_size == 0) continue;
        if (img_active && image->entryOffset(i) >= 0) continue;

        // Copy from whichever slice(s) cover the entry range.
        uint32_t done = 0;
        for (uint8_t s = 0; s < nslices && done < e->data_size; ++s) {
            if (!resps[s] || !resps[s]->payload) continue;
            const uint32_t s0 = cyclic_pending_[s].off;
            const uint32_t s1 = s0 + resps[s]->datalen;
            const uint32_t e0 = entry_off, e1 = entry_off + e->data_size;
            if (e0 >= s1 || s0 >= e1) continue;
            const uint32_t lo = std::max(e0, s0);
            const uint32_t hi = std::min(e1, s1);
            std::memcpy(static_cast<uint8_t*>(e->app_buffer) + (lo - e0),
                        resps[s]->payload + (lo - s0), hi - lo);
            done += hi - lo;
        }
    }

    stats_.success++;
    return true;
}

bool LogicalAddressManager::exchangeLRWSlice(const PDO::PDOMapping& mapping,
                                             uint32_t offset, uint32_t length) {
    return exchangeLRWImpl(mapping, offset, length,
                           /*enforce_slice_limit=*/true);
}

uint32_t LogicalAddressManager::maxSliceLength() const {
    // One LRW datagram = frame payload minus the per-datagram wire overhead
    // (datagram header: cmd+idx+adp+ado+len/flags+irq = 10 B, plus WKC = 2 B).
    static constexpr uint32_t kDatagramOverhead = 12;
    const size_t frame = transport_.maxEtherCATPayloadPerFrame();
    return frame > kDatagramOverhead
         ? static_cast<uint32_t>(frame - kDatagramOverhead)
         : 0u;
}

std::vector<LogicalAddressManager::EntrySlice>
LogicalAddressManager::describeEntries(const PDO::PDOMapping& mapping) const {
    std::vector<EntrySlice> out;
    if (!initialized_ || slave_count_ == 0) return out;
    out.reserve(mapping.entry_count());

    std::array<uint32_t, PDO::kMaxPDOSlaves> rx_running{};
    std::array<uint32_t, PDO::kMaxPDOSlaves> tx_running{};

    for (size_t i = 0; i < mapping.entry_count(); i++) {
        const PDO::PDOEntry* e = mapping.get_entry(i);
        if (!e || !e->enabled) continue;
        if (e->slave_index >= slave_count_) continue;
        if (!addr_map_[e->slave_index].active) continue;

        const auto& addr = addr_map_[e->slave_index];
        EntrySlice s;
        s.entry_index = i;
        s.slave_index = e->slave_index;
        s.pdo_index   = e->pdo_index;
        s.direction   = e->direction;
        s.length      = e->data_size;
        if (e->direction == PDO::PDODirection::RxPDO) {
            s.offset = addr.rxpdo_logical_addr - base_logical_addr_
                     + rx_running[e->slave_index];
            rx_running[e->slave_index] += e->data_size;
        } else {
            s.offset = addr.txpdo_logical_addr - base_logical_addr_
                     + tx_running[e->slave_index];
            tx_running[e->slave_index] += e->data_size;
        }
        out.push_back(s);
    }
    return out;
}

bool LogicalAddressManager::exchangeLRWImpl(const PDO::PDOMapping& mapping,
                                            uint32_t offset, uint32_t length,
                                            bool enforce_slice_limit) {
    if (!initialized_ || slave_count_ == 0) {
        TETHER_LOGW(TAG, "exchangeLRW: not initialized or no slaves");
        stats_.send_errors++;
        return false;
    }

    static constexpr size_t kMaxLRWPayload = PDO::kMaxPDOSize * PDO::kMaxPDOSlaves;
    const uint32_t total_data = total_rxpdo_bytes_ + total_txpdo_bytes_;
    if (length == 0) return true;

    if (total_data > kMaxLRWPayload) {
        TETHER_LOGE(TAG,
            "exchangeLRW: total data {} exceeds Tether internal buffer capacity "
            "(max={} = {} bytes/slave * {} slaves). This is a Tether limit, not a slave limit. "
            "Increase ECAT_PDO_MAX_BUFFER_SIZE or ECAT_PDO_MAX_SLAVES in EtherCATConfig.hpp.",
            total_data, kMaxLRWPayload,
            PDO::kMaxPDOSize, PDO::kMaxPDOSlaves);
        stats_.send_errors++;
        return false;
    }
    if (static_cast<uint64_t>(offset) + length > total_data) {
        TETHER_LOGE(TAG, "exchangeLRW: slice [{}, {}) out of range (total={})",
                    offset, offset + length, total_data);
        stats_.send_errors++;
        return false;
    }
    if (enforce_slice_limit && length > maxSliceLength()) {
        TETHER_LOGE(TAG,
            "exchangeLRW: slice length {} exceeds one datagram ({}); "
            "split the exchange into several exchangeLRWSlice() calls",
            length, maxSliceLength());
        stats_.send_errors++;
        return false;
    }

    // Build payload covering only [offset, offset+length).
    uint8_t payload[kMaxLRWPayload];
    std::memset(payload, 0, length);
    const uint32_t slice_end = offset + length;

    // Fill RxPDO (write) portion from app buffers of entries intersecting the
    // slice.  Per-slave running offset places multiple PDO entries on the same
    // slave at consecutive positions within the slave's region.
    std::array<uint32_t, PDO::kMaxPDOSlaves> rx_running{};
    for (size_t i = 0; i < mapping.entry_count(); i++) {
        const PDO::PDOEntry* e = mapping.get_entry(i);
        if (!e || !e->enabled || e->direction != PDO::PDODirection::RxPDO) continue;
        if (e->slave_index >= slave_count_) continue;
        if (!addr_map_[e->slave_index].active) continue;

        const auto& addr = addr_map_[e->slave_index];
        const uint32_t entry_off = addr.rxpdo_logical_addr - base_logical_addr_
                                 + rx_running[e->slave_index];
        rx_running[e->slave_index] += e->data_size;
        if (!e->app_buffer || e->data_size == 0) continue;

        const uint32_t lo = std::max(entry_off, offset);
        const uint32_t hi = std::min(entry_off + e->data_size, slice_end);
        if (lo >= hi) continue;
        std::memcpy(payload + (lo - offset),
                    static_cast<const uint8_t*>(e->app_buffer) + (lo - entry_off),
                    hi - lo);
    }

    // Send LRW datagram for the slice (logical address = base + offset).
    const uint8_t idx = transport_.allocIdx();
    const uint32_t logical_addr = base_logical_addr_ + offset;
    const uint16_t adp = static_cast<uint16_t>(logical_addr & 0xFFFF);
    const uint16_t ado = static_cast<uint16_t>((logical_addr >> 16) & 0xFFFF);

    // Pre-register the response waiter BEFORE sending: the response can
    // return faster than we can register, and an unregistered response is
    // dropped as "unrouted" followed by a spurious timeout.
    RxDatagram resp;
    const size_t slot = transport_.preRegisterResponseWaiter(
        idx, resp.data, sizeof(resp.data));
    const bool have_slot = (slot != IPDOTransport::kPreRegInvalid);

    if (!transport_.sendSingleDatagram(Command::LRW, idx, adp, ado,
                                        payload, static_cast<uint16_t>(length),
                                        true)) {
        if (have_slot) transport_.waitForPreRegistered(slot, 0, resp);
        if (!transport_.isCancelRequested()) {
            TETHER_LOGE(TAG, "exchangeLRW: send failed");
        }
        stats_.send_errors++;
        return false;
    }

    // Wait for response
    const bool got_resp = have_slot
        ? transport_.waitForPreRegistered(slot, 10, resp)
        : transport_.waitForResponseIdx(idx, 10, resp);
    if (!got_resp) {
        TETHER_LOGE(TAG, "exchangeLRW: response timeout");
        stats_.timeout_errors++;
        return false;
    }

    if (resp.wkc == 0) {
        TETHER_LOGW(TAG, "exchangeLRW: WKC=0");
        stats_.wkc_errors++;
        return false;
    }

    // Extract TxPDO (read) data from the response for entries intersecting
    // the slice.  Entry offsets are absolute within the process image, which
    // matches the LRW response payload (Rx region then Tx region).
    if (resp.datalen >= length) {
        const uint8_t* rx_data = resp.data;
        std::array<uint32_t, PDO::kMaxPDOSlaves> tx_running{};
        for (size_t i = 0; i < mapping.entry_count(); i++) {
            const PDO::PDOEntry* e = mapping.get_entry(i);
            if (!e || !e->enabled || e->direction != PDO::PDODirection::TxPDO) continue;
            if (e->slave_index >= slave_count_) continue;
            if (!addr_map_[e->slave_index].active) continue;

            const auto& addr = addr_map_[e->slave_index];
            const uint32_t entry_off = addr.txpdo_logical_addr - base_logical_addr_
                                     + tx_running[e->slave_index];
            tx_running[e->slave_index] += e->data_size;
            if (!e->app_buffer || e->data_size == 0) continue;

            const uint32_t lo = std::max(entry_off, offset);
            const uint32_t hi = std::min(entry_off + e->data_size, slice_end);
            if (lo >= hi) continue;
            std::memcpy(static_cast<uint8_t*>(e->app_buffer) + (lo - entry_off),
                        rx_data + (lo - offset), hi - lo);
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
    const uint32_t logical_addr = base_logical_addr_;
    const uint16_t adp = static_cast<uint16_t>(logical_addr & 0xFFFF);
    const uint16_t ado = static_cast<uint16_t>((logical_addr >> 16) & 0xFFFF);

    // Pre-register the response waiter BEFORE sending (see exchangeLRWImpl).
    RxDatagram resp;
    const size_t slot = transport_.preRegisterResponseWaiter(
        idx, resp.data, sizeof(resp.data));
    const bool have_slot = (slot != IPDOTransport::kPreRegInvalid);

    if (!transport_.sendSingleDatagram(Command::LRW, idx, adp, ado,
                                        payload, static_cast<uint16_t>(total_data),
                                        true)) {
        if (have_slot) transport_.waitForPreRegistered(slot, 0, resp);
        TETHER_LOGE(TAG, "exchangeLRWForSlaves: send failed (mask=0x{:08X})",
                    static_cast<unsigned long>(slave_mask));
        stats_.send_errors++;
        return false;
    }

    const bool got_resp = have_slot
        ? transport_.waitForPreRegistered(slot, 10, resp)
        : transport_.waitForResponseIdx(idx, 10, resp);
    if (!got_resp) {
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
