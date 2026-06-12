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

#include "tether/platform/EspCompat.hpp"
#include "tether/ethercat/EtherCATPDO.hpp"

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

    // ----- FMMU configuration queries -----

    uint32_t getRxPDOLogicalAddr(uint16_t slave_index) const;
    uint16_t getRxPDOLength(uint16_t slave_index) const;
    uint32_t getTxPDOLogicalAddr(uint16_t slave_index) const;
    uint16_t getTxPDOLength(uint16_t slave_index) const;

    bool hasSlavePDOs(uint16_t slave_index) const;

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

private:
    IPDOTransport& transport_;

    struct SlaveLogicalAddr {
        uint32_t rxpdo_logical_addr{0};
        uint16_t rxpdo_length{0};
        uint32_t txpdo_logical_addr{0};
        uint16_t txpdo_length{0};
        bool     active{false};
    };

    SlaveLogicalAddr addr_map_[PDO::kMaxPDOSlaves];
    uint16_t slave_count_{0};
    uint32_t total_rxpdo_bytes_{0};
    uint32_t total_txpdo_bytes_{0};
    Stats    stats_{};
    bool     initialized_{false};
};

} // namespace EtherCAT
