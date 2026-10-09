/**
 * @file LogicalAddressManager_polled.cpp
 * @brief LogicalAddressManager — polled LRW exchange (impl + per-slave).
 *
 * TU split out of LogicalAddressManager.cpp.
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

bool LogicalAddressManager::exchangeLRWImpl(const PDO::PDOMapping& mapping,
                                            uint32_t offset, uint32_t length,
                                            bool enforce_slice_limit) {
    if (!initialized_ || slave_count_ == 0) {
        TETHER_LOGW(TAG, "exchangeLRW: not initialized or no slaves");
        stats_.send_errors++;
        return false;
    }
    stallCheck();

    static constexpr size_t kMaxLRWPayload = PDO::kMaxPDOSize * PDO::kMaxPDOSlaves;
    const uint32_t total_data = next_free_log_;
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
        if (e->data_size == 0) continue;

        const uint32_t lo = std::max(entry_off, offset);
        const uint32_t hi = std::min(entry_off + e->data_size, slice_end);
        if (lo >= hi) continue;
        std::memcpy(payload + (lo - offset),
                    e->storage + (lo - entry_off),
                    hi - lo);
    }

    // Claim a response slot for the LRW datagram (logical address = base +
    // offset).  Only the slot we transmit on is pruned — busy slots hold
    // live async waiters and are never stolen.
    RxDatagram resp;
    uint8_t idx = 0;
    const size_t slot = claimExchangeWaiter(idx, resp);
    if (slot == IPDOTransport::kPreRegBusy) {
        tx_diag_->noteNoResponseSlot();   // logged rate-limited by the monitor
        stats_.send_errors++;
        return false;
    }
    const bool have_slot = (slot != IPDOTransport::kPreRegInvalid);
    const uint32_t logical_addr = base_logical_addr_ + offset;
    const uint16_t adp = static_cast<uint16_t>(logical_addr & 0xFFFF);
    const uint16_t ado = static_cast<uint16_t>((logical_addr >> 16) & 0xFFFF);

    if (!transport_.sendSingleDatagram(Command::LRW, idx, adp, ado,
                                        payload, static_cast<uint16_t>(length),
                                        true)) {
        if (have_slot) transport_.waitForPreRegistered(slot, 0, resp);
        if (!transport_.isCancelRequested()) {
            tx_diag_->noteSendFailure();   // logged rate-limited by the monitor
        }
        stats_.send_errors++;
        return false;
    }
    tx_diag_->noteSendOk();

    // Wait for response
    const bool got_resp = have_slot
        ? transport_.waitForPreRegistered(slot, response_timeout_ms_, resp)
        : transport_.waitForResponseIdx(idx, response_timeout_ms_, resp);
    if (!got_resp) {
        stats_.timeout_errors++;
        onExchangeTimeout("exchangeLRW");
        return false;
    }

    if (resp.wkc == 0) {
        tx_diag_->noteWkcZero("exchangeLRW");   // logged rate-limited by monitor
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
            if (e->data_size == 0) continue;

            const uint32_t lo = std::max(entry_off, offset);
            const uint32_t hi = std::min(entry_off + e->data_size, slice_end);
            if (lo >= hi) continue;
            std::memcpy(e->storage + (lo - entry_off),
                        rx_data + (lo - offset), hi - lo);
        }
    }

    onExchangeSuccess();
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
    stallCheck();
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
            if (e->data_size > 0 &&
                rxpdo_off + e->data_size <= compact_rxpdo) {
                std::memcpy(payload + rxpdo_off, e->storage, e->data_size);
            }
            rxpdo_off += e->data_size;
        }
    }

    // Claim a response slot for the LRW datagram — same selective prune
    // as exchangeLRWImpl: only the transmitted-on slot is touched.
    RxDatagram resp;
    uint8_t idx = 0;
    const size_t slot = claimExchangeWaiter(idx, resp);
    if (slot == IPDOTransport::kPreRegBusy) {
        tx_diag_->noteNoResponseSlot();   // logged rate-limited by the monitor
        stats_.send_errors++;
        return false;
    }
    const bool have_slot = (slot != IPDOTransport::kPreRegInvalid);
    const uint32_t logical_addr = base_logical_addr_;
    const uint16_t adp = static_cast<uint16_t>(logical_addr & 0xFFFF);
    const uint16_t ado = static_cast<uint16_t>((logical_addr >> 16) & 0xFFFF);

    if (!transport_.sendSingleDatagram(Command::LRW, idx, adp, ado,
                                        payload, static_cast<uint16_t>(total_data),
                                        true)) {
        if (have_slot) transport_.waitForPreRegistered(slot, 0, resp);
        if (!transport_.isCancelRequested()) {
            tx_diag_->noteSendFailure(static_cast<uint32_t>(slave_mask));
        }
        stats_.send_errors++;
        return false;
    }
    tx_diag_->noteSendOk();

    const bool got_resp = have_slot
        ? transport_.waitForPreRegistered(slot, response_timeout_ms_, resp)
        : transport_.waitForResponseIdx(idx, response_timeout_ms_, resp);
    if (!got_resp) {
        stats_.timeout_errors++;
        onExchangeTimeout("exchangeLRWForSlaves");
        return false;
    }

    if (resp.wkc == 0) {
        tx_diag_->noteWkcZero("exchangeLRWForSlaves");
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

            if (e->data_size > 0 &&
                tx_off + e->data_size <= compact_txpdo) {
                std::memcpy(e->storage, rx_data + tx_off, e->data_size);
            }
            tx_off += e->data_size;
        }
    }

    onExchangeSuccess();
    return true;
}

} // namespace EtherCAT

