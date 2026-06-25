/**
 * @file PDORegionManager.cpp
 * @brief PDORegionManager implementation
 */

#include "tether/ethercat/PDORegionManager.hpp"
#include "tether/ethercat/CyclicTaskScheduler.hpp"
#include <cstring>
#include <algorithm>

namespace EtherCAT {

namespace {

/// Adapter tasks for scheduler registration
class PrepareExchangeTask final : public ICyclicTask {
public:
    explicit PrepareExchangeTask(PDORegionManager& mgr) : mgr_(mgr) {}
    bool update(DS402Master&, double) override {
        mgr_.prepareExchange();
        return true;
    }
private:
    PDORegionManager& mgr_;
};

class ProcessExchangeTask final : public ICyclicTask {
public:
    explicit ProcessExchangeTask(PDORegionManager& mgr) : mgr_(mgr) {}
    bool update(DS402Master&, double) override {
        mgr_.processExchange();
        return true;
    }
private:
    PDORegionManager& mgr_;
};

} // anonymous namespace

PDORegionManager::PDORegionManager(PDOManager& pdo_manager)
    : pdo_manager_(pdo_manager)
{
}

PDORegionManager::RegionEntry* PDORegionManager::findRegion(int region_id) {
    for (auto& r : regions_) {
        if (r.id == region_id) return &r;
    }
    return nullptr;
}

const PDORegionManager::RegionEntry* PDORegionManager::findRegion(int region_id) const {
    for (const auto& r : regions_) {
        if (r.id == region_id) return &r;
    }
    return nullptr;
}

int PDORegionManager::registerRegion(uint16_t slave_index, PDO::PDODirection direction,
                                     uint16_t offset, uint16_t size) {
    if (size == 0) return -1;

    std::lock_guard<std::mutex> lock(mutex_);

    // Find the matching PDO entry in the mapping
    auto& mapping = pdo_manager_.mapping();
    PDO::PDOEntry* matched_entry = nullptr;

    for (size_t i = 0; i < mapping.entry_count(); ++i) {
        PDO::PDOEntry* entry = mapping.get_entry_mut(i);
        if (!entry || !entry->enabled) continue;
        if (entry->slave_index != slave_index) continue;
        if (entry->direction != direction) continue;
        if (offset + size > entry->data_size) continue;

        matched_entry = entry;
        break;
    }

    if (!matched_entry) return -1;

    int id = next_id_.fetch_add(1, std::memory_order_relaxed);
    if (id == 0) {
        id = next_id_.fetch_add(1, std::memory_order_relaxed);
    }

    RegionEntry re;
    re.id = id;
    re.region.slave_index = slave_index;
    re.region.direction = direction;
    re.region.offset = offset;
    re.region.size = size;
    re.pdo_entry = matched_entry;
    re.tx_queue_buffer.resize(size, 0);
    re.rx_buffer.resize(size, 0);

    regions_.push_back(std::move(re));
    return id;
}

void PDORegionManager::setProvider(int region_id, IPDORegionProvider* provider) {
    std::lock_guard<std::mutex> lock(mutex_);
    auto* re = findRegion(region_id);
    if (re) {
        re->provider = provider;
    }
}

bool PDORegionManager::pushTxData(int region_id, std::span<const uint8_t> data) {
    std::lock_guard<std::mutex> lock(mutex_);
    auto* re = findRegion(region_id);
    if (!re || re->region.direction != PDO::PDODirection::RxPDO) return false;
    if (data.size() != re->region.size) return false;

    std::memcpy(re->tx_queue_buffer.data(), data.data(), re->region.size);
    re->tx_pending = true;
    return true;
}

bool PDORegionManager::popRxData(int region_id, std::span<uint8_t> data) {
    std::lock_guard<std::mutex> lock(mutex_);
    auto* re = findRegion(region_id);
    if (!re || re->region.direction != PDO::PDODirection::TxPDO) return false;
    if (!re->has_rx_data) return false;
    if (data.size() != re->region.size) return false;

    std::memcpy(data.data(), re->rx_buffer.data(), re->region.size);
    re->has_rx_data = false;
    return true;
}

uint8_t* PDORegionManager::getTxBuffer(int region_id) {
    std::lock_guard<std::mutex> lock(mutex_);
    auto* re = findRegion(region_id);
    if (!re || !re->pdo_entry || !re->pdo_entry->app_buffer) return nullptr;
    if (re->region.direction != PDO::PDODirection::RxPDO) return nullptr;
    return static_cast<uint8_t*>(re->pdo_entry->app_buffer) + re->region.offset;
}

const uint8_t* PDORegionManager::getRxBuffer(int region_id) const {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto* re = findRegion(region_id);
    if (!re || !re->pdo_entry || !re->pdo_entry->app_buffer) return nullptr;
    if (re->region.direction != PDO::PDODirection::TxPDO) return nullptr;
    return static_cast<const uint8_t*>(re->pdo_entry->app_buffer) + re->region.offset;
}

void PDORegionManager::prepareExchange() {
    // Called from RT thread before sendAll(). No lock needed — RT context is single-threaded.
    // However, pushTxData may have been called from another thread, so we need
    // to handle the queue buffer carefully. The tx_pending flag is set under mutex
    // and read here. In practice, the queue mode is designed for the user thread
    // to push before the RT cycle.

    for (auto& re : regions_) {
        if (re.region.direction != PDO::PDODirection::RxPDO) continue;
        if (!re.pdo_entry || !re.pdo_entry->app_buffer) continue;

        uint8_t* dest = static_cast<uint8_t*>(re.pdo_entry->app_buffer) + re.region.offset;

        if (re.provider) {
            // Mode A: callback
            re.provider->fillTxRegion(re.region, dest);
        } else if (re.tx_pending) {
            // Mode B: queue
            std::memcpy(dest, re.tx_queue_buffer.data(), re.region.size);
            re.tx_pending = false;
        }
        // Mode C: raw buffer — user already wrote directly, nothing to do
    }
}

void PDORegionManager::processExchange() {
    // Called from RT thread after receiveAll().

    for (auto& re : regions_) {
        if (re.region.direction != PDO::PDODirection::TxPDO) continue;
        if (!re.pdo_entry || !re.pdo_entry->app_buffer) continue;

        const uint8_t* src = static_cast<const uint8_t*>(re.pdo_entry->app_buffer) + re.region.offset;

        if (re.provider) {
            // Mode A: callback
            re.provider->consumeRxRegion(re.region, src);
        } else {
            // Mode B: queue — copy into rx_buffer for later popRxData
            std::memcpy(re.rx_buffer.data(), src, re.region.size);
            re.has_rx_data = true;
        }
        // Mode C: raw buffer — user reads directly, nothing to do
    }
}

void PDORegionManager::registerWithScheduler(CyclicTaskScheduler& scheduler) {
    // The scheduler owns the task objects via raw pointers (non-owning).
    // We allocate them with new and they live until the scheduler is cleared
    // or the tasks are removed. In practice, they live for the lifetime of the
    // scheduler. We don't store the handles because the scheduler is typically
    // cleared on shutdown.
    scheduler.addTask(new PrepareExchangeTask(*this), TaskPhase::PreExchange, 0);
    scheduler.addTask(new ProcessExchangeTask(*this), TaskPhase::PostExchange, 0);
}

size_t PDORegionManager::regionCount() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return regions_.size();
}

bool PDORegionManager::getRegion(int region_id, PDORegion& out) const {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto* re = findRegion(region_id);
    if (!re) return false;
    out = re->region;
    return true;
}

} // namespace EtherCAT
