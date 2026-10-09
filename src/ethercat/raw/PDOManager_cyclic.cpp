/**
 * @file PDOManager_cyclic.cpp
 * @brief PDOManager — cyclic integration, slave windows and PDO slices.
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

bool PDOManager::exchangeConfiguredSlaveWindows() {
    if (!logical_addr_mgr_ || !logical_addr_mgr_->isInitialized()) {
        return exchangeAll();
    }
    bool ok = true;
    for (uint16_t s = 0; s < PDO::kMaxPDOSlaves; ++s) {
        uint32_t offset = 0, length = 0;
        if (!logical_addr_mgr_->getSlaveLogicalWindow(s, offset, length)) {
            continue;
        }
        ok = exchangeLRWSlice(offset, length) && ok;
    }
    return ok;
}

bool PDOManager::exchangeLRWSlice(uint32_t offset, uint32_t length) {
    if (!logical_addr_mgr_ || !logical_addr_mgr_->isInitialized()) {
        TETHER_LOGW(TAG, "exchangeLRWSlice: no logical address manager");
        return false;
    }
    const bool ok = logical_addr_mgr_->exchangeLRWSlice(mapping_, offset, length);
    if (ok) {
        // Mirror sendAll()'s per-slave exchange counters so the OP-transition
        // "PDO exchange happened" check sees the traffic (req/reply > 0).
        const uint32_t end = offset + length;
        for (const auto& s : logical_addr_mgr_->describeEntries(mapping_)) {
            if (s.slave_index >= PDO::kMaxPDOSlaves) continue;
            if (s.offset >= end || s.offset + s.length <= offset) continue;
            if (s.direction == PDO::PDODirection::RxPDO)
                slave_configs_[s.slave_index].pdo_request_count++;
            else
                slave_configs_[s.slave_index].pdo_reply_count++;
        }
    }
    return ok;
}

bool PDOManager::exchangeAllLRWCyclic(uint32_t rx_timeout_ns,
                                      ProcessImage* image) {
    if (!logical_addr_mgr_ || !logical_addr_mgr_->isInitialized()) {
        return exchangeAll();
    }
    const bool ok = logical_addr_mgr_->exchangeAllLRWCyclic(
        mapping_, rx_timeout_ns, image);
    if (ok) {
        // Mirror the per-slave counters like exchangeLRWSlice() does.
        for (const auto& s : logical_addr_mgr_->describeEntries(mapping_)) {
            if (s.slave_index >= PDO::kMaxPDOSlaves) continue;
            if (s.direction == PDO::PDODirection::RxPDO)
                slave_configs_[s.slave_index].pdo_request_count++;
            else
                slave_configs_[s.slave_index].pdo_reply_count++;
        }
    }
    return ok;
}

bool PDOManager::cyclicSend(ProcessImage* image, uint32_t rx_timeout_ns) {
    if (!logical_addr_mgr_ || !logical_addr_mgr_->isInitialized()) {
        return exchangeAll();
    }
    return logical_addr_mgr_->cyclicSend(mapping_, image, rx_timeout_ns);
}

bool PDOManager::cyclicCollect(ProcessImage* image) {
    if (!logical_addr_mgr_ || !logical_addr_mgr_->isInitialized()) {
        return true;
    }
    const bool ok = logical_addr_mgr_->cyclicCollect(mapping_, image);
    if (ok) {
        for (const auto& s : logical_addr_mgr_->describeEntries(mapping_)) {
            if (s.slave_index >= PDO::kMaxPDOSlaves) continue;
            if (s.direction == PDO::PDODirection::RxPDO)
                slave_configs_[s.slave_index].pdo_request_count++;
            else
                slave_configs_[s.slave_index].pdo_reply_count++;
        }
    }
    return ok;
}

bool PDOManager::cyclicExchangePending() const {
    return logical_addr_mgr_ && logical_addr_mgr_->cyclicExchangePending();
}

uint32_t PDOManager::definePDOSlice(const PDOSliceSpec& spec) {
    if (!logical_addr_mgr_ || !logical_addr_mgr_->isInitialized())
        return 0xFFFFFFFFu;
    return logical_addr_mgr_->definePDOSlice(mapping_, spec);
}

bool PDOManager::clearPDOSlices() {
    return logical_addr_mgr_ && logical_addr_mgr_->clearPDOSlices();
}

size_t PDOManager::pdoSliceCount() const {
    return logical_addr_mgr_ ? logical_addr_mgr_->pdoSliceCount() : 0;
}

bool PDOManager::exchangePDOSlice(const PDOSliceSpec& spec) {
    if (!logical_addr_mgr_ || !logical_addr_mgr_->isInitialized())
        return false;
    return logical_addr_mgr_->exchangePDOSlice(mapping_, spec);
}

void PDOManager::setImageExchangeDecimation(uint32_t every_n) {
    if (logical_addr_mgr_)
        logical_addr_mgr_->setImageExchangeDecimation(every_n);
}

uint8_t PDOManager::cyclicSliceCount() const {
    return logical_addr_mgr_ ? logical_addr_mgr_->cyclicSliceCount() : 0;
}

void PDOManager::setCyclicStrictWkc(bool strict) {
    if (logical_addr_mgr_) logical_addr_mgr_->setStrictWkc(strict);
}

bool PDOManager::configureProcessImage(ProcessImage& image,
                                       ImageMode mode,
                                       const char* shm_name) {
    if (!logical_addr_mgr_ || !logical_addr_mgr_->isInitialized()) {
        return false;
    }
    int32_t offsets[ProcessImage::kMaxEntries];
    const size_t n = logical_addr_mgr_->computeImageOffsets(
        mapping_, offsets, ProcessImage::kMaxEntries);
    ProcessImage::Config cfg;
    cfg.mode          = mode;
    cfg.rx_bytes      = logical_addr_mgr_->totalRxPDOBytes();
    cfg.tx_bytes      = logical_addr_mgr_->totalTxPDOBytes();
    cfg.image_bytes   = logical_addr_mgr_->totalLogicalSize();
    cfg.entry_offsets = offsets;
    cfg.entry_count   = n;
    cfg.shm_name      = shm_name;
    return image.configure(cfg);
}

uint32_t PDOManager::maxLogicalSliceLength() const {
    if (!logical_addr_mgr_ || !logical_addr_mgr_->isInitialized()) return 0;
    return logical_addr_mgr_->maxSliceLength();
}

std::vector<PDO::LogicalEntrySlice> PDOManager::describeLogicalEntries() const {
    if (!logical_addr_mgr_ || !logical_addr_mgr_->isInitialized()) return {};
    return logical_addr_mgr_->describeEntries(mapping_);
}

} // namespace EtherCAT

