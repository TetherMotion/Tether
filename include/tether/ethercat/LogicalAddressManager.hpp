/**
 * @file LogicalAddressManager.hpp
 * @brief Logical Address Manager for multi-slave PDO exchange via LRW
 *
 * Allocates a contiguous logical address space across all slaves' PDOs,
 * builds LRW datagrams, and provides FMMU configuration data.
 *
 * ## Logical Address Layout
 *
 *   RxPDO Region (write: master -> slaves):
 *     slave0_RxPDO | slave1_RxPDO | ... | slaveN_RxPDO
 *
 *   TxPDO Region (read: slaves -> master):
 *     slave0_TxPDO | slave1_TxPDO | ... | slaveN_TxPDO
 *
 * A single LRW datagram covers both regions.  Each slave's FMMU is
 * configured with the appropriate logical address range and type
 * (Write for RxPDO, Read for TxPDO).
 */

#pragma once

#include <cstdint>
#include <functional>
#include <format>
#include <string>
#include <vector>

#include "tether/platform/EspCompat.hpp"
#include "tether/ethercat/PDOManager.hpp"
#include "tether/ethercat/PDOMappingConfig.hpp"

namespace EtherCAT {

class IPDOTransport;

class LogicalAddressManager {
public:
    explicit LogicalAddressManager(IPDOTransport& transport);

    LogicalAddressManager(const LogicalAddressManager&)            = delete;
    LogicalAddressManager& operator=(const LogicalAddressManager&) = delete;

    bool init();
    void deinit();
    bool isInitialized() const { return initialized_; }

    /**
     * @brief Build the logical address map from configured slave PDO sizes.
     *
     * Must be called after all slaves have their PDO sizes finalized
     * (i.e. after finalizeMapping has been called for each slave).
     *
     * @param configs  SlaveConfig array from PDOManager
     * @param slave_count  Number of slaves
     * @return true on success
     */
    bool buildAddressMap(const PDO::SlaveConfig* configs, uint16_t slave_count);

    /**
     * @brief Build the logical address map from multi-PDO sync manager configs.
     *
     * Allocates per-PDO logical addresses within each slave's FMMU region.
     * Each PDO mapping gets its own logical address offset, enabling per-PDO
     * data access during LRW exchanges.
     *
     * @param sm_configs  Array of vectors, one per slave, of multi-PDO SM configs
     * @param slave_count  Number of slaves
     * @return true on success
     */
    bool buildAddressMapFromMultiPDO(
        const std::vector<PDO::MultiPDOSyncManagerConfig>* sm_configs,
        uint16_t slave_count);

    // ----- FMMU configuration queries -----

    uint32_t getRxPDOLogicalAddr(uint16_t slave_index) const;
    uint16_t getRxPDOLength(uint16_t slave_index) const;
    uint32_t getTxPDOLogicalAddr(uint16_t slave_index) const;
    uint16_t getTxPDOLength(uint16_t slave_index) const;

    bool hasSlavePDOs(uint16_t slave_index) const;

    // ----- Per-PDO address queries (multi-PDO mode) -----

    /// Per-PDO logical address entry within a slave's FMMU region.
    struct PDOLogicalAddrEntry {
        uint16_t pdo_index{0};
        uint32_t logical_addr{0};
        uint16_t length{0};
        uint8_t  sm_index{0xFF};
        bool     is_output{false};  ///< true = RxPDO (output), false = TxPDO (input)
    };

    /// Get the logical address of a specific PDO on a specific slave.
    /// Returns 0 if the PDO is not found.
    uint32_t getPDOLogicalAddr(uint16_t slave_index, uint16_t pdo_index) const;

    /// Get the length of a specific PDO on a specific slave.
    /// Returns 0 if the PDO is not found.
    uint16_t getPDOLength(uint16_t slave_index, uint16_t pdo_index) const;

    /// Get all per-PDO logical address entries for a slave.
    std::vector<PDOLogicalAddrEntry> getSlavePDOLogicalAddrs(uint16_t slave_index) const;

    uint32_t totalRxPDOBytes() const { return total_rxpdo_bytes_; }
    uint32_t totalTxPDOBytes() const { return total_txpdo_bytes_; }
    uint32_t totalLogicalSize()  const { return total_rxpdo_bytes_ + total_txpdo_bytes_; }

    // ----- LRW Exchange -----

    /**
     * @brief Build and send a single LRW datagram covering all slaves.
     *
     * Concatenates all RxPDO app buffers into the write portion and
     * all TxPDO space into the read portion.  After receiving the
     * response, copies TxPDO data back into each entry's app_buffer.
     *
     * @param mapping  PDOMapping with all PDO entries
     * @return true if the LRW exchange succeeded (sent + response received)
     */
    bool exchangeAllLRW(const PDO::PDOMapping& mapping);

    /**
     * @brief Build and send an LRW datagram for specific slaves only.
     *
     * @param mapping     PDOMapping with all PDO entries
     * @param slave_mask  Bitmask: bit N = include slave N
     * @return true on success
     */
    bool exchangeLRWForSlaves(const PDO::PDOMapping& mapping, uint32_t slave_mask);

    // ----- Statistics -----

    struct Stats {
        uint32_t success{0};
        uint32_t wkc_errors{0};
        uint32_t send_errors{0};
        uint32_t timeout_errors{0};
    };
    Stats getStats() const;
    void  resetStats();

    // ----- Log Prefix (set by Master from per-slave name) -----

    /// @brief Set a function that returns the log prefix for a given slave index.
    void setPrefixProvider(std::function<std::string(uint16_t)> provider) {
        prefix_provider_ = std::move(provider);
    }

    // ----- Base Logical Address -----

    /// @brief Set the base logical address for this manager's address space.
    ///
    /// Default is 0x10000.  When using multiple independent PDOManager /
    /// LogicalAddressManager instances (e.g. one per slave), each manager
    /// should use a non-overlapping base address so that LRW datagrams
    /// from different managers don't conflict on the same slave's FMMU.
    ///
    /// Must be called before buildAddressMap() / buildAddressMapFromMultiPDO().
    void setBaseLogicalAddress(uint32_t base) { base_logical_addr_ = base; }
    uint32_t getBaseLogicalAddress() const { return base_logical_addr_; }

private:
    IPDOTransport& transport_;

    struct SlaveLogicalAddr {
        uint32_t rxpdo_logical_addr{0};
        uint16_t rxpdo_length{0};
        uint32_t txpdo_logical_addr{0};
        uint16_t txpdo_length{0};
        bool     active{false};

        /// Per-PDO logical address entries (multi-PDO mode).
        /// Fixed-size array to avoid heap allocation in real-time paths.
        static constexpr size_t kMaxPDOEntries = 32;  ///< 16 RxPDO + 16 TxPDO max
        PDOLogicalAddrEntry pdo_entries[kMaxPDOEntries]{};
        size_t pdo_entry_count{0};

        const PDOLogicalAddrEntry* findPDO(uint16_t pdo_index) const {
            for (size_t i = 0; i < pdo_entry_count; i++) {
                if (pdo_entries[i].pdo_index == pdo_index) return &pdo_entries[i];
            }
            return nullptr;
        }
    };

    SlaveLogicalAddr addr_map_[PDO::kMaxPDOSlaves];
    uint16_t slave_count_{0};
    uint32_t total_rxpdo_bytes_{0};
    uint32_t total_txpdo_bytes_{0};
    uint32_t base_logical_addr_{0x10000};  ///< Base logical address (default 0x10000)
    Stats    stats_{};
    bool     initialized_{false};
    std::function<std::string(uint16_t)> prefix_provider_;

    /// Build the log prefix for a slave (uses prefix_provider_ if set, else default)
    std::string slavePrefix(uint16_t idx) const {
        if (prefix_provider_) return prefix_provider_(idx);
        return std::format("Slave {}", idx);
    }
};

} // namespace EtherCAT
