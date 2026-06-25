#pragma once

/**
 * @file PDORegionManager.hpp
 * @brief Manages named memory regions within PDO exchange buffers
 *
 * Allows subsystems (FSoE, safety, custom protocols) to reserve regions
 * within PDO entry app buffers and fill/consume them via three modes:
 *  (a) Callback-based: IPDORegionProvider::fillTxRegion / consumeRxRegion
 *  (b) Queue-based: pushTxData / popRxData
 *  (c) Raw buffer: getTxBuffer / getRxBuffer for direct memcpy
 *
 * The manager hooks into the RT cycle via CyclicTaskScheduler:
 *  - PreExchange: fill Tx regions from providers/queues/buffers
 *  - PostExchange: consume Rx regions into providers/queues/buffers
 */

#include <cstdint>
#include <cstddef>
#include <mutex>
#include <vector>
#include <span>
#include <atomic>

#include "tether/ethercat/PDOManager.hpp"

namespace EtherCAT {

class CyclicTaskScheduler;

/// A reserved region within a PDO entry's app buffer
struct PDORegion {
    uint16_t slave_index = 0;
    PDO::PDODirection direction = PDO::PDODirection::TxPDO;
    uint16_t offset = 0;   ///< Byte offset within the PDO entry's app buffer
    uint16_t size = 0;     ///< Region size in bytes
};

/// Provider interface for callback mode
class IPDORegionProvider {
public:
    virtual ~IPDORegionProvider() = default;
    /// Called before sendAll(): fill the Tx region with data to send
    virtual void fillTxRegion(const PDORegion& region, uint8_t* buffer) = 0;
    /// Called after receiveAll(): consume Rx data from the region
    virtual void consumeRxRegion(const PDORegion& region, const uint8_t* buffer) = 0;
};

class PDORegionManager {
public:
    explicit PDORegionManager(PDOManager& pdo_manager);
    ~PDORegionManager() = default;

    PDORegionManager(const PDORegionManager&) = delete;
    PDORegionManager& operator=(const PDORegionManager&) = delete;

    /// Reserve a region within an existing PDO entry's app buffer.
    /// Finds the PDO entry for the given slave + direction, then reserves
    /// bytes at the specified offset.
    /// Returns a region ID (>0) on success, or -1 on failure.
    int registerRegion(uint16_t slave_index, PDO::PDODirection direction,
                       uint16_t offset, uint16_t size);

    // --- Mode A: Callback-based ---
    void setProvider(int region_id, IPDORegionProvider* provider);

    // --- Mode B: Queue-based ---
    /// Push Tx data to be sent next cycle. Overwrites any pending data.
    bool pushTxData(int region_id, std::span<const uint8_t> data);
    /// Pop Rx data received last cycle. Returns false if no data available.
    bool popRxData(int region_id, std::span<uint8_t> data);

    // --- Mode C: Raw buffer ---
    /// Get a direct pointer to the region's Tx backing buffer for memcpy.
    /// The buffer is the PDO entry's app_buffer + offset.
    uint8_t* getTxBuffer(int region_id);
    /// Get a direct pointer to the region's Rx backing buffer for reading.
    const uint8_t* getRxBuffer(int region_id) const;

    // --- Cycle hooks ---
    /// Called before PDOManager::sendAll(). Fills Tx regions.
    void prepareExchange();
    /// Called after PDOManager::receiveAll(). Consumes Rx regions.
    void processExchange();

    /// Register this manager as cyclic tasks (PreExchange + PostExchange)
    /// with the given scheduler. The scheduler must outlive this manager.
    void registerWithScheduler(CyclicTaskScheduler& scheduler);

    /// Number of registered regions.
    size_t regionCount() const;

    /// Get region info by ID (for diagnostics).
    bool getRegion(int region_id, PDORegion& out) const;

private:
    struct RegionEntry {
        int id = 0;
        PDORegion region;
        PDO::PDOEntry* pdo_entry = nullptr;  // Cached pointer into PDOManager mapping
        // Mode A
        IPDORegionProvider* provider = nullptr;
        // Mode B
        std::vector<uint8_t> tx_queue_buffer;
        std::vector<uint8_t> rx_buffer;
        bool tx_pending = false;
        bool has_rx_data = false;
    };

    RegionEntry* findRegion(int region_id);
    const RegionEntry* findRegion(int region_id) const;

    PDOManager& pdo_manager_;
    std::vector<RegionEntry> regions_;
    mutable std::mutex mutex_;
    std::atomic<int> next_id_{1};
};

} // namespace EtherCAT
