/**
 * @file PDOManager_transfer.cpp
 * @brief PDOManager — single-entry PDO transfer (position/configured/broadcast).
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

// ============================================================================
// Transfer helpers (formerly in pdo_transfer.cpp)
// ============================================================================

bool PDOManager::sendRxPDOPosition(const PDO::PDOEntry& entry) {
    const uint16_t adp = transport_.adpForSlaveIndex(entry.slave_index);
    bool do_confirmed = ((transfer_stats_.rxpdo_debug_count % 100) == 0);

    if (do_confirmed) {
        const uint8_t idx = transport_.allocIdx();
        bool ok = transport_.sendSingleDatagram(
            Command::APWR, idx, adp, entry.physical_offset,
            entry.storage, entry.data_size, true);
        if (ok) {
            RxDatagram resp;
            bool got = transport_.waitForResponseIdx(idx, 10, resp);
            if (got && resp.wkc > 0) {
                transfer_stats_.rxpdo_confirmed_ok++;
            } else {
                transfer_stats_.rxpdo_confirmed_fail++;
            }
        } else {
            transfer_stats_.rxpdo_confirmed_fail++;
        }
    } else {
        if (!transport_.sendSingleDatagram(
                Command::APWR, IPDOTransport::kFireAndForgetIdx,
                adp, entry.physical_offset,
                entry.storage, entry.data_size, false)) {
            transfer_stats_.rxpdo_debug_count++;
            return false;
        }
    }
    transfer_stats_.rxpdo_debug_count++;
    return true;
}

bool PDOManager::recvTxPDOPosition(PDO::PDOEntry& entry) {
    const uint16_t adp = transport_.adpForSlaveIndex(entry.slave_index);
    transfer_stats_.txpdo_debug_count++;

    // Send APRD and use fire-and-forget — responses are best-effort.
    if (!transport_.sendSingleDatagram(
            Command::APRD, IPDOTransport::kFireAndForgetIdx,
            adp, entry.physical_offset, nullptr, entry.data_size, true)) {
        return false;
    }
    return true;
}

bool PDOManager::sendRxPDOConfigured(const PDO::PDOEntry& entry) {
    const uint8_t idx = transport_.allocIdx();
    if (!transport_.sendSingleDatagram(
            Command::FPWR, idx, entry.configured_address,
            entry.physical_offset, entry.storage, entry.data_size, true)) {
        return false;
    }
    RxDatagram resp;
    return transport_.waitForResponseIdx(idx, 5, resp) && resp.wkc > 0;
}

bool PDOManager::recvTxPDOConfigured(PDO::PDOEntry& entry) {
    const uint8_t idx = transport_.allocIdx();
    if (!transport_.sendSingleDatagram(
            Command::FPRD, idx, entry.configured_address,
            entry.physical_offset, nullptr, entry.data_size, true)) {
        return false;
    }
    RxDatagram resp;
    if (!transport_.waitForResponseIdx(idx, 5, resp)) return false;
    if (resp.wkc == 0 || resp.datalen < entry.data_size) return false;
    std::memcpy(entry.storage, resp.data, entry.data_size);
    return true;
}

bool PDOManager::sendRxPDOBroadcast(const PDO::PDOEntry& entry, uint16_t /*expected_wkc*/) {
    const uint8_t idx = transport_.allocIdx();
    if (!transport_.sendSingleDatagram(
            Command::BWR, idx, 0, entry.physical_offset,
            entry.storage, entry.data_size, true)) {
        return false;
    }
    RxDatagram resp;
    if (!transport_.waitForResponseIdx(idx, 5, resp)) return false;
    return resp.wkc > 0;
}

bool PDOManager::recvTxPDOBroadcast(PDO::PDOEntry& entry, uint16_t /*expected_wkc*/) {
    const uint8_t idx = transport_.allocIdx();
    if (!transport_.sendSingleDatagram(
            Command::BRD, idx, 0, entry.physical_offset,
            nullptr, entry.data_size, true)) {
        return false;
    }
    RxDatagram resp;
    if (!transport_.waitForResponseIdx(idx, 5, resp)) return false;
    if (resp.datalen < entry.data_size) return false;
    std::memcpy(entry.storage, resp.data, entry.data_size);
    return resp.wkc > 0;
}

// ============================================================================
// PDO Transfer public API (formerly pdo_api.cpp)
// ============================================================================

bool PDOManager::sendRxPDO(size_t entry_index) {
    const PDO::PDOEntry* entry = mapping_.get_entry(entry_index);
    if (!entry || !entry->enabled || entry->direction != PDO::PDODirection::RxPDO)
        return false;

    if (rxPDODebug(entry->slave_index)) {
        TETHER_LOGI(TAG, "[RxPDO-DEBUG] Sending entry {}: slave={} offset=0x{:04x} size={} mode={}",
                    entry_index, entry->slave_index, entry->physical_offset, entry->data_size,
                    entry->address_mode == PDO::PDOAddressMode::Position ? "position" :
                    entry->address_mode == PDO::PDOAddressMode::ConfiguredAddress ? "configured" :
                    entry->address_mode == PDO::PDOAddressMode::Broadcast ? "broadcast" :
                    entry->address_mode == PDO::PDOAddressMode::Logical ? "logical" : "unknown");
        if (entry->data_size <= 32) {
            char hex[128] = {0};
            size_t pos = 0;
            const uint8_t* buf = entry->storage;
            for (uint16_t b = 0; b < entry->data_size && pos + 3 < sizeof(hex); b++) {
                pos += static_cast<size_t>(std::snprintf(hex + pos, sizeof(hex) - pos, "%02X ", buf[b]));
            }
            TETHER_LOGI(TAG, "  [RxPDO-DEBUG] Data: {}", hex);
        }
    }

    bool success = false;
    switch (entry->address_mode) {
        case PDO::PDOAddressMode::Position:
            success = sendRxPDOPosition(*entry);
            break;
        case PDO::PDOAddressMode::ConfiguredAddress:
            success = sendRxPDOConfigured(*entry);
            break;
        case PDO::PDOAddressMode::Broadcast:
            success = sendRxPDOBroadcast(*entry, 1);
            break;
        case PDO::PDOAddressMode::Logical:
            TETHER_LOGW(TAG, "Logical addressing not yet implemented");
            break;
    }

    if (rxPDODebug(entry->slave_index)) {
        TETHER_LOGI(TAG, "  [RxPDO-DEBUG] Result: {} (success_count={} error_count={})",
                    success ? "OK" : "FAIL",
                    static_cast<unsigned>(entry->success_count + (success ? 1 : 0)),
                    static_cast<unsigned>(entry->error_count + (success ? 0 : 1)));
    }

    if (success) {
        const_cast<PDO::PDOEntry*>(entry)->success_count++;
        stats_.rxpdo_frames_sent++;
        if (entry->slave_index < PDO::kMaxPDOSlaves) {
            slave_configs_[entry->slave_index].pdo_request_count++;
        }
    } else {
        const_cast<PDO::PDOEntry*>(entry)->error_count++;
        stats_.rxpdo_errors++;
    }
    return success;
}

bool PDOManager::receiveTxPDO(size_t entry_index) {
    PDO::PDOEntry* entry = mapping_.get_entry_mut(entry_index);
    if (!entry || !entry->enabled || entry->direction != PDO::PDODirection::TxPDO)
        return false;

    if (txPDODebug(entry->slave_index)) {
        TETHER_LOGI(TAG, "[TxPDO-DEBUG] Receiving entry {}: slave={} offset=0x{:04x} size={} mode={}",
                    entry_index, entry->slave_index, entry->physical_offset, entry->data_size,
                    entry->address_mode == PDO::PDOAddressMode::Position ? "position" :
                    entry->address_mode == PDO::PDOAddressMode::ConfiguredAddress ? "configured" :
                    entry->address_mode == PDO::PDOAddressMode::Broadcast ? "broadcast" :
                    entry->address_mode == PDO::PDOAddressMode::Logical ? "logical" : "unknown");
    }

    bool success = false;
    switch (entry->address_mode) {
        case PDO::PDOAddressMode::Position:
            success = recvTxPDOPosition(*entry);
            break;
        case PDO::PDOAddressMode::ConfiguredAddress:
            success = recvTxPDOConfigured(*entry);
            break;
        case PDO::PDOAddressMode::Broadcast:
            success = recvTxPDOBroadcast(*entry, 1);
            break;
        case PDO::PDOAddressMode::Logical:
            TETHER_LOGW(TAG, "Logical addressing not yet implemented");
            break;
    }

    if (txPDODebug(entry->slave_index)) {
        TETHER_LOGI(TAG, "  [TxPDO-DEBUG] Result: {} (success_count={} error_count={})",
                    success ? "OK" : "FAIL",
                    static_cast<unsigned>(entry->success_count + (success ? 1 : 0)),
                    static_cast<unsigned>(entry->error_count + (success ? 0 : 1)));
        if (success && entry->data_size <= 32) {
            char hex[128] = {0};
            size_t pos = 0;
            const uint8_t* buf = entry->storage;
            for (uint16_t b = 0; b < entry->data_size && pos + 3 < sizeof(hex); b++) {
                pos += static_cast<size_t>(std::snprintf(hex + pos, sizeof(hex) - pos, "%02X ", buf[b]));
            }
            TETHER_LOGI(TAG, "  [TxPDO-DEBUG] Data: {}", hex);
        }
    }

    if (success) {
        entry->success_count++;
        stats_.txpdo_frames_recv++;
        if (entry->slave_index < PDO::kMaxPDOSlaves) {
            slave_configs_[entry->slave_index].pdo_reply_count++;
        }
    } else {
        entry->error_count++;
        stats_.txpdo_errors++;
    }
    return success;
}

} // namespace EtherCAT

