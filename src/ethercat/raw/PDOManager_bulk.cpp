/**
 * @file PDOManager_bulk.cpp
 * @brief PDOManager — bulk send/receive/exchange across all entries.
 *
 * TU split out of PDOManager.cpp.
 */

#include "tether/ethercat/PDOManager.hpp"
#include "raw/PDOManagerInternal.hpp"
#include "tether/ethercat/LogicalAddressManager.hpp"
#include "tether/utils/ColoredBitsetFormatter.hpp"

#include <algorithm>
#include <bit>
#include <cstring>
#include <format>
#include <vector>

namespace EtherCAT {

bool PDOManager::sendAll() {
    // LRW path: atomic exchange, can't be split
    if (logical_addr_mgr_ && logical_addr_mgr_->isInitialized()) {
        split_state_.lrw_mode = true;

        // A process image larger than one Ethernet frame cannot be sent
        // as a single LRW datagram.  Exchange it in contiguous logical-
        // address slices, split at the RxPDO/TxPDO boundary (partial
        // reads — see LogicalAddressManager::exchangeLRWSlice()).
        const uint32_t max_len = logical_addr_mgr_->maxSliceLength();
        const uint32_t total   = logical_addr_mgr_->totalLogicalSize();
        if (max_len != 0 && total > max_len) {
            const uint32_t rx_total = logical_addr_mgr_->totalRxPDOBytes();
            bool ok = true;
            for (uint32_t off = 0; off < total; ) {
                uint32_t len = std::min(max_len, total - off);
                if (off < rx_total && off + len > rx_total) len = rx_total - off;
                // exchangeLRWSlice() mirrors the per-slave exchange
                // counters for intersected entries.
                ok = exchangeLRWSlice(off, len) && ok;
                off += len;
            }
            split_state_.send_phase_ok = ok;
            return ok;
        }

        split_state_.send_phase_ok = logical_addr_mgr_->exchangeAllLRW(mapping_);
        if (split_state_.send_phase_ok) {
            for (size_t i = 0; i < mapping_.entry_count(); i++) {
                const PDO::PDOEntry* e = mapping_.get_entry(i);
                if (!e || !e->enabled || e->slave_index >= PDO::kMaxPDOSlaves) continue;
                if (e->direction == PDO::PDODirection::RxPDO) {
                    slave_configs_[e->slave_index].pdo_request_count++;
                } else {
                    slave_configs_[e->slave_index].pdo_reply_count++;
                }
            }
        }
        return split_state_.send_phase_ok;
    }

    split_state_.lrw_mode = false;
    split_state_.send_phase_ok = true;
    split_state_.rx_confirmed_idxs.clear();
    split_state_.rx_confirmed_entry_idxs.clear();
    split_state_.rx_confirmed_slots.clear();
    split_state_.rx_confirmed_responses.clear();
    stats_.total_cycles++;

    if (rxPDODebug() || txPDODebug()) {
        TETHER_LOGI(TAG, "[PDO-DEBUG] === Cycle {} ===", static_cast<unsigned long long>(stats_.total_cycles));
    }

    // Phase 1: Batch all RxPDO writes into one frame
    if (rxPDODebug()) {
        TETHER_LOGI(TAG, "  [RxPDO-DEBUG] --- Send phase (batched) ---");
    }

    std::vector<MultiDatagramSpec> rx_specs;
    std::vector<size_t> rx_entry_idx;
    std::vector<size_t> rx_confirmed_spec_map;
    size_t rx_skipped = 0;

    for (size_t i = 0; i < mapping_.entry_count(); i++) {
        const PDO::PDOEntry* e = mapping_.get_entry(i);
        if (!e || !e->enabled || e->direction != PDO::PDODirection::RxPDO)
            continue;

        Command cmd;
        uint16_t adp;
        uint8_t idx;
        bool roundtrip;

        switch (e->address_mode) {
            case PDO::PDOAddressMode::Position:
                cmd = Command::APWR;
                adp = transport_.adpForSlaveIndex(e->slave_index);
                idx = IPDOTransport::kFireAndForgetIdx;
                roundtrip = false;
                break;
            case PDO::PDOAddressMode::ConfiguredAddress:
                cmd = Command::FPWR;
                adp = e->configured_address;
                idx = transport_.allocIdx();
                roundtrip = true;
                break;
            case PDO::PDOAddressMode::Broadcast:
                cmd = Command::BWR;
                adp = 0;
                idx = transport_.allocIdx();
                roundtrip = true;
                break;
            default:
                rx_skipped++;
                continue;
        }

        rx_specs.push_back({cmd, idx, adp, e->physical_offset,
                           e->storage, e->data_size, roundtrip});
        rx_entry_idx.push_back(i);
        if (roundtrip) {
            split_state_.rx_confirmed_idxs.push_back(idx);
            rx_confirmed_spec_map.push_back(rx_specs.size() - 1);
        }
    }

    if (!rx_specs.empty()) {
        // Pre-register response waiter slots for confirmed entries BEFORE
        // sending to avoid the send-then-register race: multiple datagrams
        // share one frame, so all responses arrive in one frame on the RX
        // thread.  If we register slots only after the send returns, later
        // responses arrive with no pending slot and are dropped ("unrouted").
        split_state_.rx_confirmed_responses.resize(
            split_state_.rx_confirmed_idxs.size());
        split_state_.rx_confirmed_slots.resize(
            split_state_.rx_confirmed_idxs.size(),
            IPDOTransport::kPreRegInvalid);
        for (size_t j = 0; j < split_state_.rx_confirmed_idxs.size(); j++) {
            auto& resp_buf = split_state_.rx_confirmed_responses[j];
            split_state_.rx_confirmed_slots[j] = transport_.preRegisterResponseWaiter(
                split_state_.rx_confirmed_idxs[j],
                resp_buf.data, sizeof(resp_buf.data));
        }

        size_t frames_sent = transport_.sendMultiDatagram(rx_specs.data(), rx_specs.size());
        if (frames_sent == 0) {
            for (size_t i : rx_entry_idx) {
                mapping_.get_entry_mut(i)->error_count++;
                stats_.rxpdo_errors++;
            }
            split_state_.send_phase_ok = false;
        } else {
            // Handle fire-and-forget entries (assume success if send succeeded)
            for (size_t j = 0; j < rx_specs.size(); j++) {
                if (rx_specs[j].idx == IPDOTransport::kFireAndForgetIdx) {
                    PDO::PDOEntry* e = mapping_.get_entry_mut(rx_entry_idx[j]);
                    e->success_count++;
                    stats_.rxpdo_frames_sent++;
                    if (e->slave_index < PDO::kMaxPDOSlaves)
                        slave_configs_[e->slave_index].pdo_request_count++;
                    // Callback mode: fire TxSent callback for fire-and-forget entries
                    if (mode_ == PDOMode::Callback && callback_config_.fire_on_tx_sent &&
                        rx_entry_idx[j] < callbacks_.size() && callbacks_[rx_entry_idx[j]].tx_sent) {
                        callbacks_[rx_entry_idx[j]].tx_sent(e->slave_index, stats_.total_cycles,
                            static_cast<uint64_t>(Tether::Platform::Clock::instance().getMicroseconds()) * 1000ULL);
                    }
                }
            }
            // Store confirmed entry indices for receiveAll() to wait on
            for (size_t j = 0; j < split_state_.rx_confirmed_idxs.size(); j++) {
                split_state_.rx_confirmed_entry_idxs.push_back(
                    rx_entry_idx[rx_confirmed_spec_map[j]]);
            }
        }
    }

    // If all enabled RxPDO entries were skipped (e.g. Logical addressing
    // is not implemented), mark the send phase as failed.
    if (rx_specs.empty() && rx_skipped > 0) {
        split_state_.send_phase_ok = false;
        for (size_t i = 0; i < mapping_.entry_count(); i++) {
            PDO::PDOEntry* e = mapping_.get_entry_mut(i);
            if (!e || !e->enabled || e->direction != PDO::PDODirection::RxPDO)
                continue;
            e->error_count++;
            stats_.rxpdo_errors++;
        }
    }

    // Debug gate checkpoint: first successful RxPDO send
    if (debug_gate_ && !first_rxpdo_emitted_ && split_state_.send_phase_ok) {
        first_rxpdo_emitted_ = true;
        debug_gate_->notifyCheckpoint("first-rxpdo");
    }

    return split_state_.send_phase_ok;
}

bool PDOManager::receiveAll() {
    // LRW path: already done in sendAll(), nothing to receive
    if (split_state_.lrw_mode) {
        return split_state_.send_phase_ok;
    }

    bool all_ok = split_state_.send_phase_ok;

    // Wait for confirmed RxPDO responses from sendAll()
    for (size_t j = 0; j < split_state_.rx_confirmed_idxs.size(); j++) {
        size_t entry_i = split_state_.rx_confirmed_entry_idxs[j];
        RxDatagram resp;
        bool got;
        if (j < split_state_.rx_confirmed_slots.size() &&
            split_state_.rx_confirmed_slots[j] != IPDOTransport::kPreRegInvalid) {
            got = transport_.waitForPreRegistered(
                split_state_.rx_confirmed_slots[j], 5, resp);
        } else {
            got = transport_.waitForResponseIdx(
                split_state_.rx_confirmed_idxs[j], 5, resp);
        }
        if (got && resp.wkc > 0) {
            PDO::PDOEntry* e = mapping_.get_entry_mut(entry_i);
            e->success_count++;
            stats_.rxpdo_frames_sent++;
            if (e->slave_index < PDO::kMaxPDOSlaves)
                slave_configs_[e->slave_index].pdo_request_count++;
            // Callback mode: fire TxSent callback for confirmed entries
            if (mode_ == PDOMode::Callback && callback_config_.fire_on_tx_sent &&
                entry_i < callbacks_.size() && callbacks_[entry_i].tx_sent) {
                callbacks_[entry_i].tx_sent(e->slave_index, stats_.total_cycles,
                    static_cast<uint64_t>(Tether::Platform::Clock::instance().getMicroseconds()) * 1000ULL);
            }
        } else {
            PDO::PDOEntry* e = mapping_.get_entry_mut(entry_i);
            e->error_count++;
            stats_.rxpdo_errors++;
            all_ok = false;
        }
    }
    split_state_.rx_confirmed_idxs.clear();
    split_state_.rx_confirmed_entry_idxs.clear();
    split_state_.rx_confirmed_slots.clear();
    split_state_.rx_confirmed_responses.clear();

    // Phase 2: Batch all TxPDO reads into one frame
    if (txPDODebug()) {
        TETHER_LOGI(TAG, "  [TxPDO-DEBUG] --- Receive phase (batched) ---");
    }

    std::vector<MultiDatagramSpec> tx_specs;
    std::vector<size_t> tx_entry_idx;
    std::vector<uint8_t> tx_idxs;
    std::vector<size_t> tx_spec_map;
    size_t tx_skipped = 0;

    for (size_t i = 0; i < mapping_.entry_count(); i++) {
        const PDO::PDOEntry* e = mapping_.get_entry(i);
        if (!e || !e->enabled || e->direction != PDO::PDODirection::TxPDO)
            continue;

        Command cmd;
        uint16_t adp;
        uint8_t idx;
        bool roundtrip;

        switch (e->address_mode) {
            case PDO::PDOAddressMode::Position:
                cmd = Command::APRD;
                adp = transport_.adpForSlaveIndex(e->slave_index);
                idx = IPDOTransport::kFireAndForgetIdx;
                roundtrip = true;  // APRD always roundtrip
                break;
            case PDO::PDOAddressMode::ConfiguredAddress:
                cmd = Command::FPRD;
                adp = e->configured_address;
                idx = transport_.allocIdx();
                roundtrip = true;
                break;
            case PDO::PDOAddressMode::Broadcast:
                cmd = Command::BRD;
                adp = 0;
                idx = transport_.allocIdx();
                roundtrip = true;
                break;
            default:
                tx_skipped++;
                continue;
        }

        tx_specs.push_back({cmd, idx, adp, e->physical_offset,
                           nullptr, e->data_size, roundtrip});
        tx_entry_idx.push_back(i);
        if (idx != IPDOTransport::kFireAndForgetIdx) {
            tx_idxs.push_back(idx);
            tx_spec_map.push_back(tx_specs.size() - 1);
        }
    }

    if (!tx_specs.empty()) {
        // Pre-register response waiter slots for confirmed TxPDO entries
        // BEFORE sending to avoid the send-then-register race.
        std::vector<size_t> tx_slots(tx_idxs.size(), IPDOTransport::kPreRegInvalid);
        std::vector<RxDatagram> tx_responses(tx_idxs.size());
        for (size_t j = 0; j < tx_idxs.size(); j++) {
            tx_slots[j] = transport_.preRegisterResponseWaiter(
                tx_idxs[j], tx_responses[j].data, sizeof(tx_responses[j].data));
        }

        size_t frames_sent = transport_.sendMultiDatagram(tx_specs.data(), tx_specs.size());
        if (frames_sent == 0) {
            // Cancel any pre-registered slots
            for (size_t j = 0; j < tx_slots.size(); j++) {
                if (tx_slots[j] != IPDOTransport::kPreRegInvalid)
                    transport_.waitForPreRegistered(tx_slots[j], 0, tx_responses[j]);
            }
            for (size_t i : tx_entry_idx) {
                mapping_.get_entry_mut(i)->error_count++;
                stats_.txpdo_errors++;
            }
            all_ok = false;
        } else {
            // Handle fire-and-forget entries (position mode - assume success)
            for (size_t j = 0; j < tx_specs.size(); j++) {
                if (tx_specs[j].idx == IPDOTransport::kFireAndForgetIdx) {
                    PDO::PDOEntry* e = mapping_.get_entry_mut(tx_entry_idx[j]);
                    e->success_count++;
                    stats_.txpdo_frames_recv++;
                    if (e->slave_index < PDO::kMaxPDOSlaves)
                        slave_configs_[e->slave_index].pdo_reply_count++;
                    // Callback mode: fire RxReceived callback for fire-and-forget entries
                    // (data not actually received for fire-and-forget, but callback signals cycle completion)
                    if (mode_ == PDOMode::Callback && callback_config_.fire_on_rx_received &&
                        tx_entry_idx[j] < callbacks_.size() && callbacks_[tx_entry_idx[j]].rx_received) {
                        callbacks_[tx_entry_idx[j]].rx_received(e->slave_index, e->storage, e->data_size,
                            stats_.total_cycles,
                            static_cast<uint64_t>(Tether::Platform::Clock::instance().getMicroseconds()) * 1000ULL);
                    }
                }
            }
            // Wait for confirmed responses and copy data
            for (size_t j = 0; j < tx_idxs.size(); j++) {
                size_t entry_i = tx_entry_idx[tx_spec_map[j]];
                PDO::PDOEntry* e = mapping_.get_entry_mut(entry_i);
                RxDatagram resp;
                bool got;
                if (tx_slots[j] != IPDOTransport::kPreRegInvalid) {
                    got = transport_.waitForPreRegistered(tx_slots[j], 5, resp);
                } else {
                    got = transport_.waitForResponseIdx(tx_idxs[j], 5, resp);
                }
                if (got &&
                    resp.wkc > 0 && resp.datalen >= e->data_size) {
                    std::memcpy(e->storage, resp.data, e->data_size);
                    e->success_count++;
                    stats_.txpdo_frames_recv++;
                    if (e->slave_index < PDO::kMaxPDOSlaves)
                        slave_configs_[e->slave_index].pdo_reply_count++;
                    // Callback mode: fire RxReceived callback for confirmed entries
                    if (mode_ == PDOMode::Callback && callback_config_.fire_on_rx_received &&
                        entry_i < callbacks_.size() && callbacks_[entry_i].rx_received) {
                        callbacks_[entry_i].rx_received(e->slave_index, e->storage, e->data_size,
                            stats_.total_cycles,
                            static_cast<uint64_t>(Tether::Platform::Clock::instance().getMicroseconds()) * 1000ULL);
                    }
                } else {
                    e->error_count++;
                    stats_.txpdo_errors++;
                    all_ok = false;
                }
            }
        }
    }

    // If all enabled TxPDO entries were skipped (e.g. Logical addressing
    // is not implemented), mark the receive phase as failed.
    if (tx_specs.empty() && tx_skipped > 0) {
        all_ok = false;
        for (size_t i = 0; i < mapping_.entry_count(); i++) {
            PDO::PDOEntry* e = mapping_.get_entry_mut(i);
            if (!e || !e->enabled || e->direction != PDO::PDODirection::TxPDO)
                continue;
            e->error_count++;
            stats_.txpdo_errors++;
        }
    }

    if (rxPDODebug() || txPDODebug()) {
        TETHER_LOGI(TAG, "  [PDO-DEBUG] Cycle result: {}", all_ok ? "OK" : "ERRORS");
    }

    // Debug gate checkpoint: first successful TxPDO receive
    if (debug_gate_ && !first_txpdo_emitted_ && all_ok) {
        first_txpdo_emitted_ = true;
        debug_gate_->notifyCheckpoint("first-txpdo");
    }

    return all_ok;
}

bool PDOManager::exchangeAll() {
    if (!sendAll()) {
        // Even if send phase fails, still try to receive
        // (receiveAll handles the split_state_ correctly)
    }
    return receiveAll();
}

} // namespace EtherCAT

