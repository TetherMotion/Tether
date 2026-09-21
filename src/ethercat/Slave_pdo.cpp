/**
 * @file Slave_pdo.cpp
 * @brief Slave — PDO registration: SII/ESI auto-configuration, custom PDO mapping, multi-PDO sync-manager assignment
 */

#include "tether/ethercat/Slave.hpp"
#include "tether/ethercat/Master.hpp"
#include "tether/ethercat/SDOManager.hpp"
#include "tether/ethercat/CoEManager.hpp"
#include "tether/ethercat/PDOManager.hpp"
#include "tether/ethercat/LogicalAddressManager.hpp"
#include "tether/ethercat/Types.hpp"
#include "tether/ethercat/SyncManager.hpp"
#include "tether/ethercat/DebugFlags.hpp"
#include "tether/ethercat/ESITypes.hpp"
#include "tether/sii/SIIReader.hpp"
#include "tether/ethercat/FaultDetection.hpp"
#include "tether/platform/Platform.hpp"

#include <cstdio>
#include <cstring>
#include <bit>

#include "SlaveESIHelpers.hpp"

namespace EtherCAT {

static const char* TAG = "Slave";

// ============================================================================
// PDO auto-configuration from SII
// ============================================================================

SlaveError Slave::registerPDOsFromSII(SIIPDOConfig& out_config) {
#if TETHER_ENABLE_SII
    SII::SIIData sii;
    if (readSII(sii) != SlaveError::Ok) {
        TETHER_LOGE(TAG, "{}: Failed to read SII for PDO auto-config", logPrefix().c_str());
        return SlaveError::SIIReadError;
    }

    // Find best-matching PDOs
    const SII::SIIPDO* rxpdo = nullptr;
    const SII::SIIPDO* txpdo = nullptr;

    for (const auto& pdo : sii.rx_pdos) {
        if (pdo.sync_manager == 2 || pdo.isDefault()) {
            rxpdo = &pdo;
            break;
        }
    }
    for (const auto& pdo : sii.tx_pdos) {
        if (pdo.sync_manager == 3 || pdo.isDefault()) {
            txpdo = &pdo;
            break;
        }
    }

    if (!rxpdo && !txpdo) {
        TETHER_LOGE(TAG, "{}: No PDO data found in SII", logPrefix().c_str());
        return SlaveError::PDOConfigFailed;
    }

    // Remove any existing entries for this slave to avoid duplicates
    PDO::PDOMapping& mapping = master_->pdoForSlave(index_).mapping();
    mapping.remove_entries_for_slave(index_);

    // Allocate buffers and register entries
    out_config = SIIPDOConfig{};  // clear

    if (rxpdo) {
        uint16_t size = static_cast<uint16_t>(rxpdo->totalBytes());
        int idx = mapping.add_rxpdo(index_, size,
                                    rxpdo->pdo_index, PDO::PDOAddressMode::Position);
        if (idx < 0) {
            TETHER_LOGE(TAG, "{}: Failed to register RxPDO mapping entry", logPrefix().c_str());
            return SlaveError::PDOMappingFailed;
        }
        out_config.rxpdo_index = rxpdo->pdo_index;
        out_config.rxpdo_size  = size;
        out_config.has_rxpdo   = true;
        TETHER_LOGI(TAG, "{}: Registered RxPDO 0x{:04X} ({} bytes) from SII", logPrefix().c_str(),
                    rxpdo->pdo_index, size);
    }

    if (txpdo) {
        uint16_t size = static_cast<uint16_t>(txpdo->totalBytes());
        int idx = mapping.add_txpdo(index_, size,
                                    txpdo->pdo_index, PDO::PDOAddressMode::Position);
        if (idx < 0) {
            TETHER_LOGE(TAG, "{}: Failed to register TxPDO mapping entry", logPrefix().c_str());
            return SlaveError::PDOMappingFailed;
        }
        out_config.txpdo_index = txpdo->pdo_index;
        out_config.txpdo_size  = size;
        out_config.has_txpdo   = true;
        TETHER_LOGI(TAG, "{}: Registered TxPDO 0x{:04X} ({} bytes) from SII", logPrefix().c_str(),
                    txpdo->pdo_index, size);
    }

    // Finalize so SlaveConfig rxpdo_size / txpdo_size are updated
    master_->pdoForSlave(index_).finalizeMapping(index_);

    return SlaveError::Ok;
#else
    (void)out_config;
    TETHER_LOGE(TAG, "{}: SII support is disabled, cannot register PDOs from SII", logPrefix().c_str());
    return SlaveError::SIIReadError;
#endif
}

SlaveError Slave::registerPDOsFromESI(const ESIFile& esi, SIIPDOConfig& out_config) {
    if (esi.empty()) {
        TETHER_LOGE(TAG, "{}: ESI file is empty — cannot register PDOs from ESI", logPrefix().c_str());
        return SlaveError::PDOConfigFailed;
    }

    auto id = readIdentityForESIMatch(*master_, index_);
    const ESI::DeviceInfo* dev = esi.findDevice(id.vendorId, id.productCode);
    if (!dev) {
        TETHER_LOGE(TAG, "{}: ESI file has no devices", logPrefix().c_str());
        return SlaveError::PDOConfigFailed;
    }

    // Find best-matching RxPDO (SM2) and TxPDO (SM3) from ESI
    const ESI::PDO* rxpdo = nullptr;
    const ESI::PDO* txpdo = nullptr;

    for (const auto& pdo : dev->rxPdos) {
        if (pdo.sm == 2 || pdo.sm == -1) {
            rxpdo = &pdo;
            break;
        }
    }
    for (const auto& pdo : dev->txPdos) {
        if (pdo.sm == 3 || pdo.sm == -1) {
            txpdo = &pdo;
            break;
        }
    }

    if (!rxpdo && !txpdo) {
        TETHER_LOGE(TAG, "{}: No PDO data found in ESI", logPrefix().c_str());
        return SlaveError::PDOConfigFailed;
    }

    // Compute total byte sizes from ESI entries
    auto pdoTotalBytes = [](const ESI::PDO& p) -> uint16_t {
        uint16_t bits = 0;
        for (const auto& e : p.entries) bits += e.bitLen;
        return static_cast<uint16_t>((bits + 7) / 8);
    };

    // Remove any existing entries for this slave to avoid duplicates
    PDO::PDOMapping& mapping = master_->pdoForSlave(index_).mapping();
    mapping.remove_entries_for_slave(index_);

    out_config = SIIPDOConfig{};

    if (rxpdo) {
        uint16_t size = pdoTotalBytes(*rxpdo);
        int idx = mapping.add_rxpdo(index_, size,
                                    rxpdo->index, PDO::PDOAddressMode::Position);
        if (idx < 0) {
            TETHER_LOGE(TAG, "{}: Failed to register RxPDO mapping entry from ESI", logPrefix().c_str());
            return SlaveError::PDOMappingFailed;
        }
        out_config.rxpdo_index = rxpdo->index;
        out_config.rxpdo_size  = size;
        out_config.has_rxpdo   = true;
        TETHER_LOGI(TAG, "{}: Registered RxPDO 0x{:04X} ({} bytes) from ESI", logPrefix().c_str(),
                    rxpdo->index, size);
    }

    if (txpdo) {
        uint16_t size = pdoTotalBytes(*txpdo);
        int idx = mapping.add_txpdo(index_, size,
                                    txpdo->index, PDO::PDOAddressMode::Position);
        if (idx < 0) {
            TETHER_LOGE(TAG, "{}: Failed to register TxPDO mapping entry from ESI", logPrefix().c_str());
            return SlaveError::PDOMappingFailed;
        }
        out_config.txpdo_index = txpdo->index;
        out_config.txpdo_size  = size;
        out_config.has_txpdo   = true;
        TETHER_LOGI(TAG, "{}: Registered TxPDO 0x{:04X} ({} bytes) from ESI", logPrefix().c_str(),
                    txpdo->index, size);
    }

    master_->pdoForSlave(index_).finalizeMapping(index_);

    return SlaveError::Ok;
}

SlaveError Slave::assignPDOs(const SIIPDOConfig& config) {
    if (!config.has_rxpdo && !config.has_txpdo) {
        TETHER_LOGE(TAG, "{}: assignPDOs called with empty config (zero PDOs available), skipping", logPrefix().c_str());
        return SlaveError::Ok;
    }

    auto& sdo = master_->sdoManager(index_);
    uint8_t zero = 0;
    uint8_t one  = 1;
    bool sdo_ok = true;

    if (config.has_rxpdo) {
        if (!sdo.writeU8(CiA301::SyncManager2PDOAssign, 0, zero).has_value()) {
            TETHER_LOGW(TAG, "{}: Failed to clear SM2 PDO count (may be fixed)", logPrefix().c_str());
        }
        if (!sdo.writeU16(CiA301::SyncManager2PDOAssign, 1, config.rxpdo_index).has_value()) {
            TETHER_LOGW(TAG, "{}: Failed to assign RxPDO 0x{:04X} to SM2", logPrefix().c_str(), config.rxpdo_index);
            sdo_ok = false;
        }
        if (!sdo.writeU8(CiA301::SyncManager2PDOAssign, 0, one).has_value()) {
            TETHER_LOGW(TAG, "{}: Failed to set SM2 PDO count", logPrefix().c_str());
        }
        TETHER_LOGI(TAG, "{}: Assigned RxPDO 0x{:04X} to SM2 (0x1C12)", logPrefix().c_str(), config.rxpdo_index);
    }

    if (config.has_txpdo) {
        if (!sdo.writeU8(CiA301::SyncManager3PDOAssign, 0, zero).has_value()) {
            TETHER_LOGW(TAG, "{}: Failed to clear SM3 PDO count (may be fixed)", logPrefix().c_str());
        }
        if (!sdo.writeU16(CiA301::SyncManager3PDOAssign, 1, config.txpdo_index).has_value()) {
            TETHER_LOGW(TAG, "{}: Failed to assign TxPDO 0x{:04X} to SM3", logPrefix().c_str(), config.txpdo_index);
            sdo_ok = false;
        }
        if (!sdo.writeU8(CiA301::SyncManager3PDOAssign, 0, one).has_value()) {
            TETHER_LOGW(TAG, "{}: Failed to set SM3 PDO count", logPrefix().c_str());
        }
        TETHER_LOGI(TAG, "{}: Assigned TxPDO 0x{:04X} to SM3 (0x1C13)", logPrefix().c_str(), config.txpdo_index);
    }

    if (!sdo_ok) {
        TETHER_LOGW(TAG, "{}: PDO assignment had SDO failures; continuing anyway", logPrefix().c_str());
    }

    return SlaveError::Ok;
}

SlaveError Slave::registerFixedPDOs(const SIIPDOConfig& config) {
    if (!config.has_rxpdo && !config.has_txpdo) {
        TETHER_LOGW(TAG, "{}: registerFixedPDOs called with empty config, skipping", logPrefix().c_str());
        return SlaveError::Ok;
    }

    // Remove any existing entries for this slave to avoid duplicates
    PDO::PDOMapping& mapping = master_->pdoForSlave(index_).mapping();
    mapping.remove_entries_for_slave(index_);

    if (config.has_rxpdo) {
        int idx = mapping.add_rxpdo(index_, config.rxpdo_size,
                                    config.rxpdo_index, PDO::PDOAddressMode::Position);
        if (idx < 0) {
            TETHER_LOGE(TAG, "{}: Failed to register fixed RxPDO 0x{:04X} mapping entry", logPrefix().c_str(), config.rxpdo_index);
            return SlaveError::PDOMappingFailed;
        }
        TETHER_LOGI(TAG, "{}: Registered fixed RxPDO 0x{:04X} ({} bytes)", logPrefix().c_str(),
                    config.rxpdo_index, config.rxpdo_size);
    }

    if (config.has_txpdo) {
        int idx = mapping.add_txpdo(index_, config.txpdo_size,
                                    config.txpdo_index, PDO::PDOAddressMode::Position);
        if (idx < 0) {
            TETHER_LOGE(TAG, "{}: Failed to register fixed TxPDO 0x{:04X} mapping entry", logPrefix().c_str(), config.txpdo_index);
            return SlaveError::PDOMappingFailed;
        }
        TETHER_LOGI(TAG, "{}: Registered fixed TxPDO 0x{:04X} ({} bytes)", logPrefix().c_str(),
                    config.txpdo_index, config.txpdo_size);
    }

    master_->pdoForSlave(index_).finalizeMapping(index_);
    return SlaveError::Ok;
}

// ============================================================================
// Custom PDO mapping
// ============================================================================

SlaveError Slave::configureCustomRxPDO(
    uint16_t pdo_index,
    std::initializer_list<CustomPDOMappingEntry> entries)
{
    return configureCustomTxPDO(pdo_index,
                                std::span<const CustomPDOMappingEntry>(entries),
                                PDO::PDODirection::RxPDO);
}

SlaveError Slave::configureCustomTxPDO(
    uint16_t pdo_index,
    std::initializer_list<CustomPDOMappingEntry> entries)
{
    return configureCustomTxPDO(pdo_index,
                                std::span<const CustomPDOMappingEntry>(entries),
                                PDO::PDODirection::TxPDO);
}

SlaveError Slave::configureCustomTxPDO(
    uint16_t pdo_index,
    std::span<const CustomPDOMappingEntry> entries,
    PDO::PDODirection direction)
{
    if (entries.size() == 0) {
        TETHER_LOGE(TAG, "configureCustomPDO: empty entry list for PDO 0x{:04X}", pdo_index);
        return SlaveError::PDOMappingFailed;
    }
    if (entries.size() > 255) {
        TETHER_LOGE(TAG, "configureCustomPDO: too many entries ({}) for PDO 0x{:04X}",
                    entries.size(), pdo_index);
        return SlaveError::PDOMappingFailed;
    }

    // Build field layout and compute total size
    std::vector<CustomPDOFieldLayout> fields;
    fields.reserve(entries.size());
    uint16_t offset = 0;
    bool any_unresolved = false;

    for (const auto& e : entries) {
        uint8_t sz = e.resolvedSize();
        if (sz == 0) {
            TETHER_LOGE(TAG, "configureCustomPDO: cannot infer size for entry 0x{:04X}:0x{:02X} ({}) — specify explicit byte size",
                        e.entry->index, e.entry->subindex, e.entry->name ? e.entry->name : "?");
            any_unresolved = true;
        }
        fields.push_back({e.entry, offset, sz});
        offset += sz;
    }
    if (any_unresolved) {
        return SlaveError::PDOMappingFailed;
    }

    uint16_t total_size = offset;
    const char* dir_str = (direction == PDO::PDODirection::RxPDO) ? "RxPDO" : "TxPDO";
    TETHER_LOGI(TAG, "{}: Configuring custom {} 0x{:04X}: {} entries, {} bytes",
                logPrefix().c_str(), dir_str, pdo_index, entries.size(), total_size);

    // Write SDO mapping to slave
    auto err = sdoWriteU8(pdo_index, 0x00, 0);  // clear count
    if (err != SlaveError::Ok) {
        TETHER_LOGE(TAG, "Failed to clear PDO mapping count for 0x{:04X}", pdo_index);
        return err;
    }

    uint8_t sub = 1;
    const bool dbg_pdo = (direction == PDO::PDODirection::RxPDO)
                             ? slave_debug_flags_.rxPDO
                             : slave_debug_flags_.txPDO;
    for (const auto& e : entries) {
        uint8_t sz = e.resolvedSize();
        uint32_t val = encodePDOMappingValue(e.entry, sz);
        err = sdoWriteU32(pdo_index, sub, val);
        if (err != SlaveError::Ok) {
            TETHER_LOGE(TAG, "Failed to write PDO mapping entry {} for 0x{:04X}", sub, pdo_index);
            return err;
        }
        if (dbg_pdo) {
            TETHER_LOGI(TAG, "  {} 0x{:04X} sub {}: 0x{:08X} (idx=0x{:04X} sub=0x{:02X} {} bytes)",
                        dir_str, pdo_index, sub, val, e.entry->index, e.entry->subindex, sz);
        }
        ++sub;
    }

    err = sdoWriteU8(pdo_index, 0x00, static_cast<uint8_t>(entries.size()));
    if (err != SlaveError::Ok) {
        TETHER_LOGE(TAG, "Failed to set PDO mapping count for 0x{:04X}", pdo_index);
        return err;
    }

    storeCustomPDOInfo(pdo_index, direction, total_size, std::move(fields));

    return SlaveError::Ok;
}

void Slave::storeCustomPDOInfo(
    uint16_t pdo_index,
    PDO::PDODirection direction,
    uint16_t total_size,
    std::vector<CustomPDOFieldLayout>&& fields)
{
    // Remove any existing entry with the same pdo_index
    for (auto it = custom_pdo_infos_.begin(); it != custom_pdo_infos_.end(); ++it) {
        if (it->pdo_index == pdo_index) {
            custom_pdo_infos_.erase(it);
            break;
        }
    }
    CustomPDOInfo info;
    info.pdo_index = pdo_index;
    info.direction = direction;
    info.total_size = total_size;
    info.fields = std::move(fields);
    info.mapping_entry_index = -1;
    custom_pdo_infos_.push_back(std::move(info));
}

SlaveError Slave::registerExistingRxPDO(uint16_t pdo_index) {
    return registerExistingPDO(pdo_index, PDO::PDODirection::RxPDO);
}

SlaveError Slave::registerExistingTxPDO(uint16_t pdo_index) {
    return registerExistingPDO(pdo_index, PDO::PDODirection::TxPDO);
}

SlaveError Slave::registerExistingPDO(uint16_t pdo_index,
                                      PDO::PDODirection direction) {
    const char* dir_str = (direction == PDO::PDODirection::RxPDO) ? "RxPDO" : "TxPDO";
    auto& sdo = master_->sdoManager(index_);

    // Read the slave's own mapping: subindex 0 = entry count, subindex n
    // holds 0xIIIISSBB (object index : subindex : bit length).
    auto count_r = sdo.readU8(pdo_index, 0x00);
    if (!count_r.has_value()) {
        TETHER_LOGE(TAG, "{}: Failed to read {} 0x{:04X} mapping count",
                    logPrefix().c_str(), dir_str, pdo_index);
        return SlaveError::SDOError;
    }
    const uint8_t count = *count_r;
    if (count == 0) {
        TETHER_LOGE(TAG, "{}: {} 0x{:04X} reports an empty mapping",
                    logPrefix().c_str(), dir_str, pdo_index);
        return SlaveError::PDOMappingFailed;
    }

    // Read all mapping entries first so an uninitialized mapping (device
    // reports a count but every entry is 0x00000000) can be reported as
    // such instead of failing the byte-alignment check.
    std::vector<uint32_t> raw_entries;
    raw_entries.reserve(count);
    bool any_nonzero = false;
    for (uint8_t sub = 1; sub <= count; ++sub) {
        auto entry_r = sdo.readU32(pdo_index, sub);
        if (!entry_r.has_value()) {
            TETHER_LOGE(TAG, "{}: Failed to read {} 0x{:04X} mapping entry {}",
                        logPrefix().c_str(), dir_str, pdo_index, sub);
            return SlaveError::SDOError;
        }
        raw_entries.push_back(*entry_r);
        any_nonzero |= (*entry_r != 0);
    }
    if (!any_nonzero) {
        TETHER_LOGE(TAG, "{}: {} 0x{:04X} mapping is uninitialized/empty — "
                    "all {} entries are 0x0000:00, 0 bits",
                    logPrefix().c_str(), dir_str, pdo_index, count);
        return SlaveError::PDOMappingFailed;
    }

    CustomPDOInfo info;
    info.pdo_index = pdo_index;
    info.direction = direction;
    info.mapping_entry_index = -1;
    info.owned_entries.reserve(count);
    info.fields.reserve(count);

    uint32_t bit_offset = 0;
    for (uint8_t sub = 1; sub <= count; ++sub) {
        const uint32_t v = raw_entries[sub - 1];
        const uint16_t obj_idx = static_cast<uint16_t>(v >> 16);
        const uint8_t  obj_sub = static_cast<uint8_t>((v >> 8) & 0xFF);
        const uint8_t  bits    = static_cast<uint8_t>(v & 0xFF);

        if (v == 0) {
            TETHER_LOGE(TAG, "{}: {} 0x{:04X} entry {} is all-zeros "
                        "(0x0000:00, 0 bits) — uninitialized entry",
                        logPrefix().c_str(), dir_str, pdo_index, sub);
            return SlaveError::PDOMappingFailed;
        }

        if (bits == 0 || (bits % 8) != 0 || (bit_offset % 8) != 0) {
            TETHER_LOGE(TAG, "{}: {} 0x{:04X} entry {} (0x{:04X}:{:02X}, {} bits) "
                        "is not byte-aligned (bit offset {}) — unsupported",
                        logPrefix().c_str(), dir_str, pdo_index, sub,
                        obj_idx, obj_sub, bits, bit_offset);
            return SlaveError::PDOMappingFailed;
        }

        info.owned_entries.push_back(ObjectDictionary::ObjectDictionaryEntry{
            .index = obj_idx,
            .subindex = obj_sub,
            .name = nullptr,
            .data_type = ObjectDictionary::ObjectDictionaryDataType::OctetString,
            .default_value = 0,
            .unit = ObjectDictionary::Unit_None,
            .options_enum = nullptr,
            .min_value = 0,
            .max_value = 0,
            .modification_mode = ObjectDictionary::ModificationMode::ReadOnly,
            .effective_time = ObjectDictionary::EffectiveTime::Immediately,
            .comment = nullptr,
        });
        info.fields.push_back(CustomPDOFieldLayout{
            &info.owned_entries.back(),
            static_cast<uint16_t>(bit_offset / 8),
            static_cast<uint8_t>(bits / 8),
        });
        bit_offset += bits;
    }
    info.total_size = static_cast<uint16_t>(bit_offset / 8);

    TETHER_LOGI(TAG, "{}: Registered existing {} 0x{:04X}: {} entries, {} bytes "
                "(device mapping read, not rewritten)",
                logPrefix().c_str(), dir_str, pdo_index, count, info.total_size);

    // Replace any existing info with the same PDO index.  owned_entries
    // moves into the list without moving its heap buffer, so the
    // fields[].entry pointers stay valid.
    for (auto it = custom_pdo_infos_.begin(); it != custom_pdo_infos_.end(); ++it) {
        if (it->pdo_index == pdo_index) {
            custom_pdo_infos_.erase(it);
            break;
        }
    }
    custom_pdo_infos_.push_back(std::move(info));
    return SlaveError::Ok;
}

void Slave::clearCustomPDOs() {
    custom_pdo_infos_.clear();
}

SlaveError Slave::applyCustomPDOs() {
    if (custom_pdo_infos_.empty()) {
        TETHER_LOGW(TAG, "{}: applyCustomPDOs called with no custom PDOs configured", logPrefix().c_str());
        return SlaveError::Ok;
    }

    // Remove existing PDO mapping entries for this slave
    PDO::PDOMapping& mapping = master_->pdoForSlave(index_).mapping();
    mapping.remove_entries_for_slave(index_);

    // Register each custom PDO — storage lives in the mapping entries.
    std::vector<uint16_t> rx_indices, tx_indices;
    for (auto& info : custom_pdo_infos_) {
        int idx;
        if (info.direction == PDO::PDODirection::RxPDO) {
            idx = mapping.add_rxpdo(index_, info.total_size,
                                    info.pdo_index, PDO::PDOAddressMode::Position);
            rx_indices.push_back(info.pdo_index);
        } else {
            idx = mapping.add_txpdo(index_, info.total_size,
                                    info.pdo_index, PDO::PDOAddressMode::Position);
            tx_indices.push_back(info.pdo_index);
        }
        if (idx < 0) {
            TETHER_LOGE(TAG, "{}: Failed to register custom PDO 0x{:04X}", logPrefix().c_str(), info.pdo_index);
            return SlaveError::PDOMappingFailed;
        }
        info.mapping_entry_index = idx;
        TETHER_LOGI(TAG, "{}: Registered custom PDO 0x{:04X} ({} bytes, entry {})",
                    logPrefix().c_str(), info.pdo_index, info.total_size, idx);
    }

    // Write PDO assignment SDOs (0x1C12 for Rx, 0x1C13 for Tx)
    auto& sdo = master_->sdoManager(index_);
    bool sdo_ok = true;

    if (!rx_indices.empty()) {
        if (!sdo.writeU8(CiA301::SyncManager2PDOAssign, 0, 0).has_value()) {
            TETHER_LOGW(TAG, "{}: Failed to clear SM2 PDO count", logPrefix().c_str());
        }
        for (size_t i = 0; i < rx_indices.size(); i++) {
            if (!sdo.writeU16(CiA301::SyncManager2PDOAssign, static_cast<uint8_t>(i + 1),
                              rx_indices[i]).has_value()) {
                TETHER_LOGE(TAG, "{}: Failed to assign RxPDO 0x{:04X} to SM2", logPrefix().c_str(), rx_indices[i]);
                sdo_ok = false;
            }
        }
        if (!sdo.writeU8(CiA301::SyncManager2PDOAssign, 0, static_cast<uint8_t>(rx_indices.size())).has_value()) {
            TETHER_LOGW(TAG, "{}: Failed to set SM2 PDO count", logPrefix().c_str());
        }
        TETHER_LOGI(TAG, "{}: Assigned {} RxPDO(s) to SM2 (0x1C12)", logPrefix().c_str(), rx_indices.size());
    }

    if (!tx_indices.empty()) {
        if (!sdo.writeU8(CiA301::SyncManager3PDOAssign, 0, 0).has_value()) {
            TETHER_LOGW(TAG, "{}: Failed to clear SM3 PDO count", logPrefix().c_str());
        }
        for (size_t i = 0; i < tx_indices.size(); i++) {
            if (!sdo.writeU16(CiA301::SyncManager3PDOAssign, static_cast<uint8_t>(i + 1),
                              tx_indices[i]).has_value()) {
                TETHER_LOGE(TAG, "{}: Failed to assign TxPDO 0x{:04X} to SM3", logPrefix().c_str(), tx_indices[i]);
                sdo_ok = false;
            }
        }
        if (!sdo.writeU8(CiA301::SyncManager3PDOAssign, 0, static_cast<uint8_t>(tx_indices.size())).has_value()) {
            TETHER_LOGW(TAG, "{}: Failed to set SM3 PDO count", logPrefix().c_str());
        }
        TETHER_LOGI(TAG, "{}: Assigned {} TxPDO(s) to SM3 (0x1C13)", logPrefix().c_str(), tx_indices.size());
    }

    if (!sdo_ok) {
        TETHER_LOGW(TAG, "{}: Custom PDO assignment had SDO failures; continuing anyway", logPrefix().c_str());
    }

    // Finalize mapping to update SM lengths
    master_->pdoForSlave(index_).finalizeMapping(index_);

    return SlaveError::Ok;
}

// ============================================================================
// Multi-PDO sync manager configuration
// ============================================================================

SlaveError Slave::configureMultiPDOs(const MultiPDOAssignment& config) {
    if (config.sm_configs.empty()) {
        TETHER_LOGW(TAG, "{}: configureMultiPDOs called with empty config", logPrefix().c_str());
        return SlaveError::Ok;
    }

    auto& pdo = master_->pdoForSlave(index_);
    auto* cfgs = pdo.slaveConfigs();
    if (index_ >= PDO::kMaxPDOSlaves) {
        TETHER_LOGE(TAG, "{}: index exceeds max PDO slaves ({})", logPrefix().c_str(), PDO::kMaxPDOSlaves);
        return SlaveError::PDOConfigFailed;
    }

    TETHER_LOGI(TAG, "{}: Configuring multi-PDO SMs ({} SM configs)", logPrefix().c_str(), config.sm_configs.size());

    const bool dbg_pdo_cfg = slave_debug_flags_.pdoConfiguration;

    // Read the slave's configured station address (register 0x0010) so
    // FPWR/FPRD in exchangePhysical() uses the correct address.
    // The slave's configured station address is assigned during INIT (from
    // SII/EEPROM or by the master).  Without this, exchangePhysical() falls
    // back to the auto-increment position address, which FPWR/FPRD don't
    // respond to after INIT.
    {
        uint16_t cfg_addr = 0;
        const auto slave_addr = EtherCAT::Master::slaveAddressFromADP(
            EtherCAT::Master::adpForSlaveIndex(index_));
        if (master_->readRegister(slave_addr, 0x0010, &cfg_addr, 2, 200)) {
            cfgs[index_].configured_address = cfg_addr;
            // Also set it in the PDO mapping entries
            pdo.mapping().set_slave_configured_address(index_, cfg_addr);
            TETHER_LOGI(TAG, "{}: Configured station address (reg 0x0010) = 0x{:04X}",
                        logPrefix().c_str(), cfg_addr);
        } else {
            TETHER_LOGW(TAG, "{}: Failed to read configured station address (reg 0x0010), "
                        "FPWR/FPRD will use auto-increment fallback", logPrefix().c_str());
        }
    }

    // Build MultiPDOSyncManagerConfig vector for FMMU and LogicalAddressManager
    std::vector<PDO::MultiPDOSyncManagerConfig> multi_configs;
    std::vector<uint16_t> rx_pdo_indices, tx_pdo_indices;
    uint8_t output_sm_index = 2, input_sm_index = 3;

    for (const auto& sm_cfg : config.sm_configs) {
        if (sm_cfg.sm_index <= 1) {
            TETHER_LOGW(TAG, "{}: SM{} is a mailbox SM — assigning PDOs is unusual",
                        logPrefix().c_str(), sm_cfg.sm_index);
        }

        PDO::MultiPDOSyncManagerConfig mc;
        mc.sm_index = sm_cfg.sm_index;
        mc.phys_start_addr = sm_cfg.phys_start_addr;
        mc.control = std::bit_cast<EtherCAT::SyncManager::SMControlReg>(sm_cfg.control_byte);
        mc.enable = true;

        // Determine SM type from control byte direction bit
        bool is_write = (sm_cfg.control_byte & 0x04) != 0;
        mc.type = is_write ? PDO::SyncManagerType::ProcessOutput : PDO::SyncManagerType::ProcessInput;

        for (const auto& pdo_region : sm_cfg.pdo_mappings) {
            mc.addPDOMapping(pdo_region.pdo_index, pdo_region.size_bytes, pdo_region.fixed);
            if (is_write) {
                rx_pdo_indices.push_back(pdo_region.pdo_index);
                output_sm_index = sm_cfg.sm_index;
            } else {
                tx_pdo_indices.push_back(pdo_region.pdo_index);
                input_sm_index = sm_cfg.sm_index;
            }
        }

        multi_configs.push_back(std::move(mc));
    }

    if (dbg_pdo_cfg) {
        TETHER_LOGI(TAG, "{}: [pdo-cfg] Multi-PDO configuration plan ({} SMs):",
                    logPrefix().c_str(), multi_configs.size());
        for (const auto& mc : multi_configs) {
            const char* dir = (mc.type == PDO::SyncManagerType::ProcessOutput)
                              ? "RxPDO (master->slave)" : "TxPDO (slave->master)";
            TETHER_LOGI(TAG, "{}: [pdo-cfg]   SM{}: phys=0x{:04X} len={} ctrl=0x{:02X} {}, {} PDO(s):",
                        logPrefix().c_str(), mc.sm_index, mc.phys_start_addr,
                        mc.totalLength(), std::bit_cast<uint8_t>(mc.control),
                        dir, mc.pdo_mappings.size());
            uint16_t offset = 0;
            for (const auto& p : mc.pdo_mappings) {
                TETHER_LOGI(TAG, "{}: [pdo-cfg]     0x{:04X}: off={} size={} bytes{}",
                            logPrefix().c_str(), p.pdo_index, offset, p.size_bytes,
                            p.fixed ? " [FIXED — not written to 0x1C1n]" : "");
                offset += p.size_bytes;
            }
        }
    }

    // Step 1: Update SlaveConfig SM entries
    //
    // SlaveConfig SM length = TOTAL (all PDOs, including FSoE).
    // This is used by PDOManager::exchangePhysical() for FPWR/FPRD size
    // and must cover the full buffer so the master can write/read all PDO data.
    //
    // The SM REGISTER (0x0810+2/0x0818+2) is also written with the TOTAL
    // length in Step 2 — the drive accepts the full ESI PDO assignment.
    //
    // The FMMU (configured later) also uses the TOTAL length so the master's
    // process data image covers the full buffer.
    for (const auto& mc : multi_configs) {
        if (mc.sm_index >= 4) continue;  // SlaveConfig only has sm[4]
        auto& sm = cfgs[index_].sm[mc.sm_index];
        sm.phys_start_addr = mc.phys_start_addr;
        sm.length = mc.totalLength();
        sm.control = mc.control;
        sm.enable = false;  // Will be enabled after FMMU config
        sm.type = mc.type;
    }

    // Update rxpdo/txpdo sizes for LogicalAddressManager compatibility
    // Use totalLength (including fixed PDOs) for FMMU/logical address mapping
    for (const auto& mc : multi_configs) {
        if (mc.type == PDO::SyncManagerType::ProcessOutput) {
            cfgs[index_].rxpdo_size = mc.totalLength();
            cfgs[index_].rxpdo_sm = mc.sm_index;
        } else if (mc.type == PDO::SyncManagerType::ProcessInput) {
            cfgs[index_].txpdo_size = mc.totalLength();
            cfgs[index_].txpdo_sm = mc.sm_index;
        }
    }

    // Step 2: Write SM registers (disabled) — follows the same pattern as
    // Master::configureProcessDataSyncManagersFromSii
    const uint16_t adp = pdo.transport().adpForSlaveIndex(index_);
    for (const auto& mc : multi_configs) {
        if (mc.pdo_mappings.empty()) continue;

        uint16_t base = static_cast<uint16_t>(0x0800 + mc.sm_index * 8);

        // Disable SM first
        uint8_t disable = 0x00;
        master_->writeRegister(EtherCAT::SlaveAddress(index_),
                              static_cast<uint16_t>(base + 6), &disable, 1, 200);

        // Physical address
        uint16_t addr_le = mc.phys_start_addr;
        master_->writeRegister(EtherCAT::SlaveAddress(index_), base, &addr_le, 2, 200);

        // Length — total (all PDOs, including FSoE)
        uint16_t len_le = mc.totalLength();
        master_->writeRegister(EtherCAT::SlaveAddress(index_),
                              static_cast<uint16_t>(base + 2), &len_le, 2, 200);

        // Control
        uint8_t ctrl_byte = std::bit_cast<uint8_t>(mc.control);
        master_->writeRegister(EtherCAT::SlaveAddress(index_),
                              static_cast<uint16_t>(base + 4), &ctrl_byte, 1, 200);

        TETHER_LOGI(TAG, "{}: Wrote SM{} (disabled): addr=0x{:04X} len={} (total) ctrl=0x{:02X}",
                    logPrefix().c_str(), mc.sm_index, mc.phys_start_addr, mc.totalLength(), ctrl_byte);

        if (dbg_pdo_cfg) {
            TETHER_LOGI(TAG, "{}: [pdo-cfg]   SM{} register writes: "
                        "0x{:04X}=0x{:04X} (start_addr), 0x{:04X}=0x{:04X} (total_len={}), "
                        "0x{:04X}=0x{:02X} (control), 0x{:04X}=0x00 (disable)",
                        logPrefix().c_str(), mc.sm_index,
                        base, mc.phys_start_addr,
                        static_cast<uint16_t>(base + 2), mc.totalLength(), mc.totalLength(),
                        static_cast<uint16_t>(base + 4), ctrl_byte,
                        static_cast<uint16_t>(base + 6));
        }
    }

    // Step 3: Write PDO assignments to OD (0x1C10+n) via SyncManagerAccessor
    // ALL PDOs (including FSoE) are written explicitly — the `fixed` flag
    // is always false.  The drive accepts the full ESI PDO assignment.
    for (const auto& mc : multi_configs) {
        if (mc.pdo_mappings.empty()) continue;

        std::vector<uint16_t> pdo_indices;
        pdo_indices.reserve(mc.pdo_mappings.size());
        for (const auto& p : mc.pdo_mappings) {
            pdo_indices.push_back(p.pdo_index);
        }

        if (dbg_pdo_cfg) {
            const uint16_t od_idx = static_cast<uint16_t>(0x1C10 + mc.sm_index);
            TETHER_LOGI(TAG, "{}: [pdo-cfg] SM{} PDO assignment (0x{:04X}): {} PDO(s)",
                        logPrefix().c_str(), mc.sm_index, od_idx,
                        pdo_indices.size());
            for (size_t i = 0; i < pdo_indices.size(); ++i) {
                TETHER_LOGI(TAG, "{}: [pdo-cfg]   0x{:04X} subindex {} = 0x{:04X}",
                            logPrefix().c_str(), od_idx, i + 1, pdo_indices[i]);
            }
        }

        auto sm_accessor = this->sm(mc.sm_index);
        SlaveError err = sm_accessor.writePDOAssignments(pdo_indices);
        if (err != SlaveError::Ok) {
            TETHER_LOGE(TAG, "{}: Failed to write PDO assignments for SM{}",
                        logPrefix().c_str(), mc.sm_index);
            // Don't return error — some slaves may not support PDO assignment writes
        } else {
            TETHER_LOGI(TAG, "{}: Wrote {} PDO assignment(s) to SM{} (0x1C1{:X})",
                        logPrefix().c_str(), pdo_indices.size(), mc.sm_index, mc.sm_index);
        }
    }

    // Step 4: Configure FMMUs via FMMUManager::configureFromMultiPDO
    // Get the base logical address from the LogicalAddressManager if available
    uint32_t base_log = 0;
    auto* lam = pdo.logicalAddressManager();
    if (lam && lam->isInitialized()) {
        // Use the existing address map — our SM lengths are already set
        lam->buildAddressMap(cfgs, master_->getDiscoveredSlaveCount());
        if (lam->hasSlavePDOs(index_)) {
            base_log = lam->getRxPDOLogicalAddr(index_);
        }
    }

    if (!fmmu_mgr_.configureFromMultiPDO(multi_configs, base_log, slave_debug_flags_.fmmu)) {
        TETHER_LOGE(TAG, "{}: FMMU configuration from multi-PDO failed", logPrefix().c_str());
        return SlaveError::PDOConfigFailed;
    }

    if (!fmmu_mgr_.writeToSlave(slave_debug_flags_.fmmu)) {
        TETHER_LOGE(TAG, "{}: FMMU write failed", logPrefix().c_str());
        return SlaveError::PDOConfigFailed;
    }

    // Step 5: Enable SMs now that FMMUs are configured
    for (const auto& mc : multi_configs) {
        if (mc.pdo_mappings.empty()) continue;

        uint16_t base = static_cast<uint16_t>(0x0800 + mc.sm_index * 8);
        uint8_t activate = 0x01;
        master_->writeRegister(EtherCAT::SlaveAddress(index_),
                              static_cast<uint16_t>(base + 6), &activate, 1, 200);
        TETHER_LOGI(TAG, "{}: Enabled SM{}", logPrefix().c_str(), mc.sm_index);

        // Update SlaveConfig to reflect enabled state
        if (mc.sm_index < 4) {
            cfgs[index_].sm[mc.sm_index].enable = true;
        }
    }

    cfgs[index_].configured = true;
    pdo_configured_ = true;

    // Step 6 (pdo-configuration only): Read back 0x1C12/0x1C13 and SM
    // registers to verify the slave accepted the configuration.
    if (dbg_pdo_cfg) {
        for (const auto& mc : multi_configs) {
            if (mc.pdo_mappings.empty()) continue;

            // Read back PDO assignment
            auto sm_accessor = this->sm(mc.sm_index);
            std::vector<uint16_t> readback;
            SlaveError rb_err = sm_accessor.readAllPDOAssignments(readback);
            if (rb_err != SlaveError::Ok) {
                TETHER_LOGI(TAG, "{}: [pdo-cfg] SM{} 0x1C1{:X} readback: FAILED (err={})",
                            logPrefix().c_str(), mc.sm_index, mc.sm_index,
                            static_cast<int>(rb_err));
            } else {
                TETHER_LOGI(TAG, "{}: [pdo-cfg] SM{} 0x1C1{:X} readback: {} entries",
                            logPrefix().c_str(), mc.sm_index, mc.sm_index,
                            readback.size());
                for (size_t i = 0; i < readback.size(); ++i) {
                    TETHER_LOGI(TAG, "{}: [pdo-cfg]   readback sub{} = 0x{:04X}",
                                logPrefix().c_str(), i + 1, readback[i]);
                }
            }

            // Read back SM registers
            uint16_t base = static_cast<uint16_t>(0x0800 + mc.sm_index * 8);
            uint16_t rb_addr = 0, rb_len = 0;
            uint8_t rb_ctrl = 0, rb_act = 0;
            master_->readRegister(EtherCAT::SlaveAddress(index_), base, &rb_addr, 2, 200);
            master_->readRegister(EtherCAT::SlaveAddress(index_),
                                 static_cast<uint16_t>(base + 2), &rb_len, 2, 200);
            master_->readRegister(EtherCAT::SlaveAddress(index_),
                                 static_cast<uint16_t>(base + 4), &rb_ctrl, 1, 200);
            master_->readRegister(EtherCAT::SlaveAddress(index_),
                                 static_cast<uint16_t>(base + 6), &rb_act, 1, 200);
            TETHER_LOGI(TAG, "{}: [pdo-cfg] SM{} register readback: "
                        "addr=0x{:04X} len={} ctrl=0x{:02X} act=0x{:02X} "
                        "(expected: addr=0x{:04X} len={} ctrl=0x{:02X} act=0x01)",
                        logPrefix().c_str(), mc.sm_index,
                        rb_addr, rb_len, rb_ctrl, rb_act,
                        mc.phys_start_addr, mc.totalLength(),
                        std::bit_cast<uint8_t>(mc.control));
        }
    }

    TETHER_LOGI(TAG, "{}: Multi-PDO configuration complete ({} SMs, {} RxPDOs, {} TxPDOs)",
                logPrefix().c_str(), multi_configs.size(), rx_pdo_indices.size(), tx_pdo_indices.size());

    return SlaveError::Ok;
}

const uint8_t* Slave::customPDOData(uint16_t pdo_index) const {
    // Data lives in mapping entry storage — resolve by (slave, pdo) so the
    // result stays correct if remove_entries_for_slave() compacted indices.
    const auto& mapping = master_->pdoForSlave(index_).mapping();
    for (size_t i = 0; i < mapping.entry_count(); ++i) {
        const auto* e = mapping.get_entry(i);
        if (e && e->slave_index == index_ && e->pdo_index == pdo_index) {
            return e->storage;
        }
    }
    return nullptr;
}

uint16_t Slave::customPDOSize(uint16_t pdo_index) const {
    for (const auto& info : custom_pdo_infos_) {
        if (info.pdo_index == pdo_index) {
            return info.total_size;
        }
    }
    return 0;
}

const uint8_t* Slave::customPDOFieldRaw(uint16_t pdo_index, size_t field_index) const {
    for (const auto& info : custom_pdo_infos_) {
        if (info.pdo_index == pdo_index) {
            if (field_index >= info.fields.size()) return nullptr;
            const uint8_t* base = customPDOData(pdo_index);
            return base ? base + info.fields[field_index].offset : nullptr;
        }
    }
    return nullptr;
}

const uint8_t* Slave::customPDOField(
    uint16_t pdo_index,
    const ObjectDictionary::ObjectDictionaryEntry* entry) const {
    for (const auto& info : custom_pdo_infos_) {
        if (info.pdo_index == pdo_index) {
            for (const auto& f : info.fields) {
                if (f.entry == entry) {
                    const uint8_t* base = customPDOData(pdo_index);
                    return base ? base + f.offset : nullptr;
                }
            }
        }
    }
    return nullptr;
}

} // namespace EtherCAT
