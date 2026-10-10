/**
 * @file PDOManager_mapping.cpp
 * @brief PDOManager — configured-address resolution and mapping finalization.
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
// Mapping Finalization
// ============================================================================

bool PDOManager::ensureConfiguredAddress(uint16_t slave_index) {
    if (slave_index >= PDO::kMaxPDOSlaves) return false;
    PDO::SlaveConfig& cfg = slave_configs_[slave_index];
    if (cfg.configured_address_known) return true;

    // Single APRD of the Configured Station Address register (0x0010).
    // The slave's configured station address is assigned during INIT (from
    // SII/EEPROM or by the master).  Without it, FPWR/FPRD transfers would
    // fall back to the auto-increment position address, which addressed
    // commands don't respond to after INIT.
    constexpr uint16_t kRegConfiguredStationAddress = 0x0010;
    const uint16_t adp = transport_.adpForSlaveIndex(slave_index);
    const uint8_t idx = transport_.allocIdx();
    if (!transport_.sendSingleDatagram(
            Command::APRD, idx, adp, kRegConfiguredStationAddress,
            nullptr, 2, /*roundtrip=*/true)) {
        TETHER_LOGW(TAG,
            "{}: failed to read configured station address (reg 0x0010) — "
            "FPWR/FPRD will use auto-increment fallback",
            slavePrefix(slave_index).c_str());
        return false;
    }
    RxDatagram resp;
    if (!transport_.waitForResponseIdx(idx, 200, resp) ||
        resp.wkc == 0 || resp.datalen < 2) {
        TETHER_LOGW(TAG,
            "{}: no response reading configured station address "
            "(reg 0x0010) — FPWR/FPRD will use auto-increment fallback",
            slavePrefix(slave_index).c_str());
        return false;
    }
    uint16_t cfg_addr;
    std::memcpy(&cfg_addr, resp.data, 2);
    if (cfg_addr == 0) {
        // Station address 0 is the unassigned default — with several slaves
        // sharing it, every FPWR/FPRD to that address is answered by all of
        // them (outputs collide, inputs mirror).  Assign a unique station
        // address via APWR (position addressing always works, even in INIT).
        const uint16_t assigned =
            static_cast<uint16_t>(0x1000 + slave_index);
        const uint8_t widx = transport_.allocIdx();
        if (transport_.sendSingleDatagram(
                Command::APWR, widx, adp, kRegConfiguredStationAddress,
                reinterpret_cast<const uint8_t*>(&assigned), 2,
                /*roundtrip=*/true)) {
            RxDatagram wresp;
            if (transport_.waitForResponseIdx(widx, 200, wresp) &&
                wresp.wkc > 0) {
                cfg_addr = assigned;
                TETHER_LOGI(TAG,
                    "{}: station address was 0 — assigned 0x{:04X}",
                    slavePrefix(slave_index).c_str(), cfg_addr);
            }
        }
    }
    cfg.configured_address       = cfg_addr;
    cfg.configured_address_known = true;
    mapping_.set_slave_configured_address(slave_index, cfg_addr);
    if (pdoCfgDebug(slave_index)) {
        TETHER_LOGI(TAG, "{}: configured station address (reg 0x0010) = 0x{:04X}",
                    slavePrefix(slave_index).c_str(), cfg_addr);
    }
    return true;
}

bool PDOManager::finalizeMapping(uint16_t slave_index) {
    if (slave_index >= PDO::kMaxPDOSlaves) {
        TETHER_LOGE(TAG, "Invalid slave index {}", slave_index);
        return false;
    }
    // Make sure FPWR/FPRD transfers address the slave by its configured
    // station address — resolve it from reg 0x0010 when nobody set it.
    ensureConfiguredAddress(slave_index);

    PDO::SlaveConfig& cfg = slave_configs_[slave_index];
    const uint16_t sm2_addr = cfg.sm[2].phys_start_addr;
    const uint16_t sm3_addr = cfg.sm[3].phys_start_addr;

    if (pdoCfgDebug(slave_index)) {
        TETHER_LOGI(TAG, "Finalizing PDO mapping for {} (SM2=0x{:04X} SM3=0x{:04X})",
                    slavePrefix(slave_index).c_str(), sm2_addr, sm3_addr);
    }

    uint16_t total_rxpdo_size = 0;
    uint16_t total_txpdo_size = 0;
    size_t rxpdo_count = 0;
    size_t txpdo_count = 0;

    for (size_t i = 0; i < mapping_.entry_count(); i++) {
        PDO::PDOEntry* entry = mapping_.get_entry_mut(i);
        if (!entry || entry->slave_index != slave_index) continue;

        if (entry->direction == PDO::PDODirection::RxPDO) {
            entry->physical_offset = sm2_addr + total_rxpdo_size;
            total_rxpdo_size += entry->data_size;
            rxpdo_count++;
            if (rxPDODebug(slave_index)) {
                TETHER_LOGI(TAG, "  [RxPDO-DEBUG] Entry {}: slave={} offset=0x{:04x} size={} buf={:p} pdo=0x{:04x}",
                            i, slave_index, entry->physical_offset, entry->data_size,
                            static_cast<const void*>(entry->storage), entry->pdo_index);
            }
        } else {
            entry->physical_offset = sm3_addr + total_txpdo_size;
            total_txpdo_size += entry->data_size;
            txpdo_count++;
            if (txPDODebug(slave_index)) {
                TETHER_LOGI(TAG, "  [TxPDO-DEBUG] Entry {}: slave={} offset=0x{:04x} size={} buf={:p} pdo=0x{:04x}",
                            i, slave_index, entry->physical_offset, entry->data_size,
                            static_cast<const void*>(entry->storage), entry->pdo_index);
            }
        }
    }

    if (rxPDODebug(slave_index) || txPDODebug(slave_index)) {
        TETHER_LOGI(TAG, "  [PDO-DEBUG] Summary for {}: RxPDO entries={} total={} bytes, TxPDO entries={} total={} bytes",
                    slavePrefix(slave_index).c_str(), rxpdo_count, total_rxpdo_size, txpdo_count, total_txpdo_size);
    }

    if (total_rxpdo_size > 0 && cfg.sm[2].type != PDO::SyncManagerType::Unused) {
        cfg.sm[2].length = total_rxpdo_size;
        cfg.rxpdo_size   = total_rxpdo_size;
    }
    if (total_txpdo_size > 0 && cfg.sm[3].type != PDO::SyncManagerType::Unused) {
        cfg.sm[3].length = total_txpdo_size;
        cfg.txpdo_size   = total_txpdo_size;
    }
    return true;
}

} // namespace EtherCAT

