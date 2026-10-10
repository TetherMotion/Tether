/**
 * @file Slave_pdo_register.cpp
 * @brief Slave — PDO registration/auto-configuration from SII/ESI.
 *
 * TU split out of Slave_pdo.cpp.
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
#include <chrono>
#include <thread>

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
        if (slave_debug_flags_.pdoConfiguration) {
            TETHER_LOGI(TAG, "{}: Registered RxPDO 0x{:04X} ({} bytes) from SII", logPrefix().c_str(),
                        rxpdo->pdo_index, size);
        }
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
        if (slave_debug_flags_.pdoConfiguration) {
            TETHER_LOGI(TAG, "{}: Registered TxPDO 0x{:04X} ({} bytes) from SII", logPrefix().c_str(),
                        txpdo->pdo_index, size);
        }
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
        if (slave_debug_flags_.pdoConfiguration) {
            TETHER_LOGI(TAG, "{}: Registered RxPDO 0x{:04X} ({} bytes) from ESI", logPrefix().c_str(),
                        rxpdo->index, size);
        }
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
        if (slave_debug_flags_.pdoConfiguration) {
            TETHER_LOGI(TAG, "{}: Registered TxPDO 0x{:04X} ({} bytes) from ESI", logPrefix().c_str(),
                        txpdo->index, size);
        }
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

} // namespace EtherCAT
