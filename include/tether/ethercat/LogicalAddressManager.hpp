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

#include <array>
#include <atomic>
#include <cstdint>
#include <functional>
#include <format>
#include <memory>
#include <string>
#include <vector>

#include "tether/ethercat/DebugFlags.hpp"

#include "tether/platform/EspCompat.hpp"
#include "tether/ethercat/PDOManager.hpp"
#include "tether/ethercat/PDOMappingConfig.hpp"

namespace EtherCAT {

class IPDOTransport;
class ProcessImage;    // defined in ProcessImage.hpp

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
     * response, copies TxPDO data back into each entry's manager-owned storage.
     *
     * @param mapping  PDOMapping with all PDO entries
     * @return true if the LRW exchange succeeded (sent + response received)
     */
    bool exchangeAllLRW(const PDO::PDOMapping& mapping);

    /**
     * @brief Whole-image LRW exchange over the transport's cyclic fast path.
     *
     * Same semantics as exchangeAllLRW() but uses the reserved-index slot
     * bank: the datagram is sent on a fixed reserved index and the response
     * is collected from a fixed deposit slot — no TransactionRouter
     * round-trip, no condition variable, and the payload is staged in a
     * persistent buffer instead of a large stack array.
     *
     * Falls back to exchangeAllLRW() when the transport does not implement
     * the fast path.
     *
     * @param mapping        PDOMapping with all PDO entries
     * @param rx_timeout_ns  Response wait budget in ns (default 200 µs).
     *                       Must fit inside the caller's cycle budget.
     * @return true on success
     */
    bool exchangeAllLRWCyclic(const PDO::PDOMapping& mapping,
                              uint32_t rx_timeout_ns = 200'000,
                              ProcessImage* image = nullptr);

    /**
     * @brief Split-phase halves of exchangeAllLRWCyclic().
     *
     * cyclicSend() gathers outputs and emits all LRW slice datagrams,
     * recording the per-slot sequence tokens.  cyclicCollect() waits for
     * the responses against a deadline anchored at send time, validates
     * WKC, publishes the input image and scatters buffered entries.
     *
     * Images larger than one frame are sliced automatically across cyclic
     * slots 0..N (one LRW datagram each — see the wire-time note in
     * docs/CyclicRealtimeTransport.md).
     *
     * Between cyclicSend() and cyclicCollect() no other cyclic exchange
     * may be started; cyclicExchangePending() reports the open half.
     */
    bool cyclicSend(const PDO::PDOMapping& mapping, ProcessImage* image,
                    uint32_t rx_timeout_ns);
    bool cyclicCollect(const PDO::PDOMapping& mapping, ProcessImage* image);
    bool cyclicExchangePending() const { return cyclic_pending_count_ > 0; }

    /// Slices the current image occupies (1 = single-frame exchange).
    uint8_t cyclicSliceCount() const { return cyclic_slice_count_; }

    /// Strict WKC verify (default on): after the first successful exchange
    /// the per-slice response WKC is learned; later mismatches count as
    /// wkc_errors.  Disable when slaves legitimately vary their WKC.
    void setStrictWkc(bool strict) { strict_wkc_ = strict; }

    /// Sentinel per-slice expected WKC — "not derivable, learn from the
    /// first successful response" (Q6/Q7).
    static constexpr uint16_t kWkcUnknown = 0xFFFF;

    /**
     * @brief Derive the expected per-slice WKC from the slave set (Q6).
     *
     * LRW semantics: each slave increments the WKC by 1 when it writes
     * output (RxPDO) bytes in the datagram's logical range and by 2 when
     * it reads input (TxPDO) bytes.  For each slice this walks the
     * mapping's logical placement and counts distinct contributing slaves
     * per direction.  Slices that derive to 0 keep kWkcUnknown and fall
     * back to learn-on-first-response — the strict check then applies
     * from the second cycle on.
     *
     * Called automatically from cyclicSend() when a mapping is known;
     * safe to call again after a slave-set change.
     */
    void deriveExpectedWkc(const PDO::PDOMapping& mapping);

    /// Forget all derived/learned expectations — the next successful
    /// responses relearn them (Q7).
    void resetExpectedWkc() {
        expected_wkc_.fill(kWkcUnknown);
    }

    /// Override one slice's expected WKC (0 = no slaves respond; use
    /// kWkcUnknown to return that slice to learn mode).
    void setExpectedWkc(uint8_t slice, uint16_t wkc) {
        if (slice < kMaxCyclicSlices) expected_wkc_[slice] = wkc;
    }

    /// Current per-slice expectations (kWkcUnknown = still learning).
    uint16_t expectedWkc(uint8_t slice) const {
        return slice < kMaxCyclicSlices ? expected_wkc_[slice] : 0;
    }

    /**
     * @brief Compute each mapping entry's byte offset in the LRW process
     *        image for ProcessImage::configure().
     *
     * Entries sharing a byte with a neighbour (bit-packed PDOs) or flagged
     * @c PDOEntry::image_exclude (FSoE-managed) are marked -1 — they stay
     *        on the buffered-storage path in image modes.
     *
     * @return number of entries written (== mapping.entry_count(), clamped
     *         to @p cap)
     */
    size_t computeImageOffsets(const PDO::PDOMapping& mapping,
                               int32_t* out, size_t cap) const;

    /**
     * @brief Build and send an LRW datagram for specific slaves only.
     *
     * @param mapping     PDOMapping with all PDO entries
     * @param slave_mask  Bitmask: bit N = include slave N
     * @return true on success
     */
    bool exchangeLRWForSlaves(const PDO::PDOMapping& mapping, uint32_t slave_mask);

    // ----- Partial LRW exchange (logical-address slices) -----

    /// Logical placement of one enabled PDO entry (see PDO::LogicalEntrySlice).
    using EntrySlice = PDO::LogicalEntrySlice;

    /// Maximum slice length that fits in a single LRW datagram (one frame).
    uint32_t maxSliceLength() const;

    /**
     * @brief Exchange one contiguous slice of the logical process image.
     *
     * Sends a single LRW datagram covering [offset, offset+length) relative
     * to the base logical address.  RxPDO app buffers of mapping entries that
     * intersect the slice are written and TxPDO app buffers are updated;
     * entries outside the slice are left untouched.  Call it several times
     * with different slices to exchange a process image larger than one
     * Ethernet frame as several datagrams (partial reads — e.g. a FSoE region
     * and a separate RSAP/debug region) instead of splitting a single frame.
     *
     * @param mapping  PDOMapping with all PDO entries
     * @param offset   Byte offset from the base logical address
     * @param length   Number of bytes (must be <= maxSliceLength())
     * @return true on success
     */
    bool exchangeLRWSlice(const PDO::PDOMapping& mapping,
                          uint32_t offset, uint32_t length);

    /// @brief Describe the logical placement of every enabled PDO entry.
    ///
    /// Lets a caller compose slices from the PDO layout (e.g. group the FSoE
    /// PDOs apart from the RSAP/debug PDOs) without knowing the internal
    /// address map.
    std::vector<EntrySlice> describeEntries(const PDO::PDOMapping& mapping) const;

    // ----- Statistics -----

    struct Stats {
        uint32_t success{0};
        uint32_t wkc_errors{0};
        uint32_t send_errors{0};
        uint32_t timeout_errors{0};
        /// Responses whose echoed send-generation didn't match the pending
        /// send — a stale deposit surviving a timed-out cycle.
        uint32_t stale_responses{0};
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

    /// @brief Wire the manager to the master' s debug flags.
    void setDebugFlags(const EtherCATMasterDebugFlags* flags) {
        debug_flags_.store(flags, std::memory_order_relaxed);
    }

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
    std::atomic<const EtherCATMasterDebugFlags*> debug_flags_{nullptr};

    /// Shared implementation for exchangeAllLRW() (whole image) and
    /// exchangeLRWSlice() (partial).  @p enforce_slice_limit rejects slices
    /// larger than one datagram; the whole-image path leaves it off so it
    /// keeps the transport's own frame-size diagnostic.
    bool exchangeLRWImpl(const PDO::PDOMapping& mapping,
                         uint32_t offset, uint32_t length,
                         bool enforce_slice_limit);

    /// Persistent LRW payload for the cyclic fast path — allocated at
    /// buildAddressMap() time (or lazily on first cyclic exchange), sized to
    /// the total process image.  Avoids the ~64 KiB stack buffer the router
    /// path uses per call.
    std::unique_ptr<uint8_t[]> cyclic_payload_;
    uint32_t cyclic_payload_size_{0};
    void ensureCyclicPayload(uint32_t size);

    // ---- Cyclic slice / split-phase state (cyclic thread only) ----
    static constexpr size_t kMaxCyclicSlices = IPDOTransport::kNumCyclicSlots;
    struct PendingSlice { uint64_t token; uint32_t off; uint32_t len; uint8_t gen; };
    std::array<PendingSlice, kMaxCyclicSlices> cyclic_pending_{};
    uint8_t  cyclic_pending_count_{0};   ///< slices awaiting collect
    uint8_t  cyclic_slice_count_{1};     ///< slices needed for the image
    uint64_t cyclic_deadline_ns_{0};     ///< collect deadline (mono ns)
    ProcessImage* pending_image_{nullptr};

    /// Expected-WKC per slice — kWkcUnknown = learn from first success
    /// (derived from the slave set by cyclicSend when a mapping is known).
    std::array<uint16_t, kMaxCyclicSlices> expected_wkc_{};
    bool strict_wkc_{true};

    /// Build the log prefix for a slave (uses prefix_provider_ if set, else default)
    std::string slavePrefix(uint16_t idx) const {
        if (prefix_provider_) return prefix_provider_(idx);
        return std::format("Slave {}", idx);
    }
};

} // namespace EtherCAT
