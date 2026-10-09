/**
 * @file Slave.cpp
 * @brief Slave and NonExistingSlave implementation
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
#include "SlaveESIHelpers.hpp"
#include "raw/RawConstants.hpp"
#include "tether/platform/Platform.hpp"

#include <cstdio>
#include <cstring>
#include <bit>

namespace EtherCAT {

static const char* TAG = "Slave";

// ============================================================================
// Slave
// ============================================================================

Slave::Slave(Master& master, uint16_t index)
    : master_(&master), index_(index)
{
}

Slave::~Slave()
{
    // Clear the back-pointer to the owning master so nothing can accidentally
    // use it while this child object is being torn down.
    master_ = nullptr;
}

uint16_t Slave::adp() const {
    return Master::adpForSlaveIndex(index_);
}

void Slave::setName(std::string name) {
    master_->setSlaveName(index_, std::move(name));
}

std::string_view Slave::name() const {
    return master_->slaveName(index_);
}

std::string Slave::logPrefix() const {
    return master_->slaveLogPrefix(index_);
}

bool Slave::apwr(uint16_t ado, const void* data, uint16_t len, unsigned int timeout_ms) {
    return master_->writeRegister(SlaveAddress(index_), ado, data, len, timeout_ms);
}

bool Slave::aprd(uint16_t ado, void* out, uint16_t len, unsigned int timeout_ms) {
    return master_->readRegister(SlaveAddress(index_), ado, out, len, timeout_ms);
}

bool Slave::apwrBatch(const uint16_t* ados, const void* const* datas,
                      const uint16_t* lens, size_t count,
                      unsigned int timeout_ms) {
    std::vector<SlaveAddress> addrs(count, SlaveAddress(index_));
    auto batch = master_->writeRegistersBatch(addrs.data(), ados, datas,
                                              lens, count);
    if (batch.count() != count) return false;
    std::vector<BatchReadResult> results;
    batch.waitAll(timeout_ms, results);
    for (const auto& r : results) {
        if (!r.success) return false;
    }
    return true;
}

// -- PDO SM configuration ----------------------------------------------------

SlaveError Slave::configurePDOSyncManagers() {
    if (!master_->configureProcessDataSyncManagersFromSii(index_)) {
        TETHER_LOGE( TAG,
            "{}: Failed to configure PDO sync-managers from SII", logPrefix().c_str());
        return SlaveError::PDOConfigFailed;
    }
    pdo_configured_ = true;
    TETHER_LOGI( TAG,
        "{}: PDO sync-managers configured from SII", logPrefix().c_str());

    if (slave_debug_flags_.pdoSm) {
        EtherCAT::debugPDOSyncManagerConfiguration(*master_, index_, TAG);
    }

    return SlaveError::Ok;
}

SlaveError Slave::configurePDOSyncManagers(
    uint16_t sm2_addr, uint16_t sm2_len, uint8_t sm2_ctrl,
    uint16_t sm3_addr, uint16_t sm3_len, uint8_t sm3_ctrl)
{
    auto& pdo = master_->pdoForSlave(index_);
    auto* cfgs = pdo.slaveConfigs();
    if (index_ >= PDO::kMaxPDOSlaves) {
        TETHER_LOGE( TAG,
            "{}: Index exceeds Tether internal max PDO slaves ({}). "
            "This is a Tether limit, not a slave limit. "
            "Increase ECAT_PDO_MAX_SLAVES in EtherCATConfig.hpp.",
            logPrefix().c_str(), PDO::kMaxPDOSlaves);
        return SlaveError::PDOConfigFailed;
    }
    cfgs[index_].sm[2].phys_start_addr = sm2_addr;
    cfgs[index_].sm[2].length = sm2_len;
    cfgs[index_].sm[2].control = std::bit_cast<EtherCAT::SyncManager::SMControlReg>(sm2_ctrl);
    cfgs[index_].sm[2].enable = 1;
    cfgs[index_].sm[2].type = PDO::SyncManagerType::ProcessOutput;

    cfgs[index_].sm[3].phys_start_addr = sm3_addr;
    cfgs[index_].sm[3].length = sm3_len;
    cfgs[index_].sm[3].control = std::bit_cast<EtherCAT::SyncManager::SMControlReg>(sm3_ctrl);
    cfgs[index_].sm[3].enable = 1;
    cfgs[index_].sm[3].type = PDO::SyncManagerType::ProcessInput;

    if (!pdo.configureSlavesSMs(index_)) {
        TETHER_LOGE( TAG,
            "{}: Failed to write PDO SM registers", logPrefix().c_str());
        return SlaveError::PDOConfigFailed;
    }

    pdo_configured_ = true;
    return SlaveError::Ok;
}

SlaveError Slave::configurePDOSyncManagers(const ESIFile& esi) {
    if (esi.empty()) {
        TETHER_LOGE(TAG, "{}: ESI file is empty — cannot configure PDO SMs from ESI", logPrefix().c_str());
        return SlaveError::PDOConfigFailed;
    }

    auto id = readIdentityForESIMatch(*master_, index_);
    const ESI::DeviceInfo* dev = esi.findDevice(id.vendorId, id.productCode);
    if (!dev) {
        TETHER_LOGE(TAG, "{}: ESI file has no devices", logPrefix().c_str());
        return SlaveError::PDOConfigFailed;
    }

    // Find Outputs (SM2) and Inputs (SM3) sync managers
    const ESI::SyncManagerEntry* smOut = findSmByName(*dev, "Outputs");
    const ESI::SyncManagerEntry* smIn  = findSmByName(*dev, "Inputs");

    if (!smOut || !smIn) {
        TETHER_LOGE(TAG, "{}: ESI missing Outputs/Inputs sync managers", logPrefix().c_str());
        return SlaveError::PDOConfigFailed;
    }

    uint8_t sm2_ctrl = std::bit_cast<uint8_t>(smOut->control);
    uint8_t sm3_ctrl = std::bit_cast<uint8_t>(smIn->control);

    TETHER_LOGI(TAG, "{}: Configuring PDO SMs from ESI (SM2=0x{:04X}/{} ctrl=0x{:02X}, SM3=0x{:04X}/{} ctrl=0x{:02X})",
                logPrefix().c_str(), smOut->startAddress, smOut->defaultSize, sm2_ctrl,
                smIn->startAddress, smIn->defaultSize, sm3_ctrl);

    return configurePDOSyncManagers(
        smOut->startAddress, smOut->defaultSize, sm2_ctrl,
        smIn->startAddress, smIn->defaultSize, sm3_ctrl);
}

void Slave::assumePDOAlreadyConfigured() {
    pdo_configured_ = true;
    TETHER_LOGI( TAG,
        "{}: Assuming PDO sync-managers already configured", logPrefix().c_str());
}

// -- Watchdog ----------------------------------------------------------------

SlaveError Slave::configureWatchdogs(uint16_t pdi_timeout_100us,
                                              uint16_t pdata_timeout_100us) {
    if (!master_->configureWatchdogs(index_, pdi_timeout_100us, pdata_timeout_100us)) {
        return SlaveError::TransportError;
    }
    return SlaveError::Ok;
}

SlaveError Slave::disableWatchdogs() {
    if (!master_->disableWatchdogs(index_)) {
        return SlaveError::TransportError;
    }
    return SlaveError::Ok;
}

SlaveError Slave::readWatchdogStatus(uint8_t& wd_status,
                                              uint8_t& pdi_cnt,
                                              uint8_t& pdata_cnt) {
    if (!master_->readWatchdogStatus(index_, wd_status, pdi_cnt, pdata_cnt)) {
        return SlaveError::TransportError;
    }
    return SlaveError::Ok;
}

// -- SII convenience ---------------------------------------------------------

SlaveError Slave::readSII(SII::SIIData& data) {
#if TETHER_ENABLE_SII
    if (!sii().isInitialised()) {
        TETHER_LOGW( TAG,
            "{}: SII manager not initialised — using direct read", logPrefix().c_str());
        if (!SII::readSII(*master_, index_, data)) {
            return SlaveError::SIIReadError;
        }
        return SlaveError::Ok;
    }
    if (!sii().parseFull(data)) {
        return SlaveError::SIIReadError;
    }
    return SlaveError::Ok;
#else
    TETHER_LOGE(TAG, "{}: SII support is disabled", logPrefix().c_str());
    return SlaveError::SIIReadError;
#endif
}

void Slave::logSIISummary(const char* tag) {
#if TETHER_ENABLE_SII
    SII::SIIData data;
    if (readSII(data) == SlaveError::Ok) {
        SII::logSIISummary(data, logPrefix(), tag);
    } else {
        TETHER_LOGW( tag,
            "{}: Failed to read SII for summary", logPrefix().c_str());
    }
#else
    (void)tag;
    TETHER_LOGE(TAG, "{}: SII support is disabled", logPrefix().c_str());
#endif
}

// ============================================================================

SyncManagerAccessor Slave::sm(uint8_t smIndex) {
    return SyncManagerAccessor(*this, smIndex);
}

} // namespace EtherCAT
