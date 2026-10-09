/**
 * @file LogicalAddressManager.hpp
 * @brief Logical Address Manager for multi-slave PDO exchange via LRW
 *
 * Allocates a contiguous logical address space across all slaves' PDOs,
 * builds LRW datagrams, and provides FMMU configuration data.
 *
 * ## Logical Address Layout
 *
 * Each slave owns one contiguous logical window holding its output region
 * followed by its input region, appended in configuration order:
 *
 *   slave0: [RxPDO][TxPDO] | slave1: [RxPDO][TxPDO] | ... | slaveN: [RxPDO][TxPDO]
 *
 * Windows are sticky — once assigned they are never relocated, because a
 * slave's FMMUs are programmed against the addresses in effect at its
 * configuration time.  Re-configured or late-joining slaves get a fresh
 * window appended at the end (see docs/PDOAddressSpace.md).
 *
 * A single LRW datagram can cover the whole image.  Each slave's FMMU is
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
#include "tether/ethercat/Types.hpp"

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

    /// Logical window of one slave relative to base_logical_addr_ —
    /// i.e. the [offset, offset+length) slice that covers all of that
    /// slave's PDO regions and can be exchanged with a single datagram.
    /// Returns false when the slave has no assigned window.
    bool getSlaveLogicalWindow(uint16_t slave_index,
                               uint32_t& offset, uint32_t& length) const {
        if (slave_index >= PDO::kMaxPDOSlaves ||
            slave_log_base_[slave_index] == kUnassigned ||
            slave_log_size_[slave_index] == 0) {
            return false;
        }
        offset = slave_log_base_[slave_index];
        length = slave_log_size_[slave_index];
        return true;
    }

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

    // ----- Rotating index pool, LRW counter trailer, DC timepoint -----

    /**
     * @brief Select the slave whose DC System Time register (0x0910) is
     *        read by an APRD datagram riding the same frame as the cyclic
     *        LRW exchange (dedicated wire index 0xFF).
     *
     * @param position  Auto-increment position of the slave on the ring
     *                  (0 = first slave), or -1 to disable (default).
     */
    void setCyclicDcTimeSlave(int32_t position) { dc_slave_pos_ = position; }
    int32_t cyclicDcTimeSlave() const { return dc_slave_pos_; }

    /**
     * @brief The 64-bit LRW counter verified by the last successful
     *        cyclic collect.
     *
     * Each cyclic send increments a counter and appends it as a trailing
     * LRW datagram covering unmapped logical space (base + image size);
     * the slaves pass those bytes through verbatim, so the echo returning
     * the same value proves the frame — and its PDO data — belongs to
     * THIS send and not to a late echo of a previous cycle.
     */
    uint64_t lastLrwCounter() const { return lrw_counter_ok_; }

    /// DC System Time read in the last collect (nanoseconds since
    /// 2000-01-01).  Meaningful only when dcTimeValid() is true.
    uint64_t lastDcTimeNs() const { return dc_time_ns_; }
    /// True when the last collect verified a fresh DC System Time.
    bool dcTimeValid() const { return dc_time_valid_; }
    /// True while responses are in flight — image slices OR user PDO
    /// slices (a decimated image can leave only slice runs pending).
    bool cyclicExchangePending() const {
        if (cyclic_pending_count_ > 0) return true;
        for (const auto& s : slices_) if (s.pending) return true;
        return false;
    }

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
     * @brief Per-image-slice health — expected/last WKC, last collect
     *        outcome, and a consecutive-failure streak.
     *
     * Written by the cyclic thread during cyclicCollect(); readers get a
     * snapshot.  This is the protocol-level liveness signal: Timeout means
     * the datagram did not circulate back, WkcError means it did but its
     * working counter is wrong — neither is derived from packet-drop
     * counters, which cannot distinguish intentional BPF filtering from
     * real loss.
     */
    CyclicSliceHealth sliceHealth(uint8_t slice) const {
        return slice < kMaxCyclicSlices
                   ? CyclicSliceHealth::unpack(__atomic_load_n(
                         &slice_health_[slice], __ATOMIC_ACQUIRE))
                   : CyclicSliceHealth{};
    }

    /// Health of every user-defined PDO-slice run, in (slice, run) order.
    /// Empty when no slices are defined.
    std::vector<CyclicSliceHealth> pdoSliceHealth() const {
        std::vector<CyclicSliceHealth> out;
        for (const auto& s : slices_)
            for (uint8_t r = 0; r < s.run_count; ++r)
                out.push_back(CyclicSliceHealth::unpack(__atomic_load_n(
                    &s.runs[r].health_packed, __ATOMIC_ACQUIRE)));
        return out;
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

    // ----- User-defined PDO slices (dedicated wire idx, own pipeline) -----

    /// Runs a slice resolves to at plan time — one LRW datagram each.
    static constexpr size_t kMaxSliceRuns = 8;

    /// Slice configuration type — defined at namespace scope in
    /// PDOManager.hpp (LAM is forward-declared there, so the spec cannot
    /// live as a nested type here).
    using PDOSliceSpec = ::EtherCAT::PDOSliceSpec;

    /**
     * @brief Define a custom PDO slice exchanged alongside the cyclic
     *        image exchange.
     *
     * Each contiguous run the spec resolves to claims one dedicated slice
     * slot (wire idx 0xE0+slot) — the kernel demux steers its responses
     * into that slot, so a slice exchange never touches the cyclic
     * image's slots.  Runs are re-planned automatically when the PDO
     * mapping epoch changes (recovery re-registration).
     *
     * @param mapping  The mapping the entry indices refer to
     * @param spec     Entries/ranges, decimation, optional callback
     * @return slice handle (index), or UINT32_MAX on failure — spec
     *         resolving to zero runs, a run > maxSliceLength(), or the
     *         slice-slot pool (16) exhausted.
     */
    uint32_t definePDOSlice(const PDO::PDOMapping& mapping,
                            const PDOSliceSpec& spec);
    /// Drop all defined slices (pending responses are abandoned).
    bool clearPDOSlices();
    size_t pdoSliceCount() const { return slices_.size(); }

    /**
     * @brief Exchange a spec's bytes once, on the blocking async path.
     *
     * The "one-off" counterpart of definePDOSlice(): resolves the spec to
     * runs and exchanges each via exchangeLRWSlice() — no reserved index,
     * no fast path, TransactionRouter round-trip per run.  For ad-hoc
     * reads/writes (commissioning, diagnostics), not the cyclic loop.
     */
    bool exchangePDOSlice(const PDO::PDOMapping& mapping,
                          const PDOSliceSpec& spec);

    /**
     * @brief Run the whole-image exchange every Nth cycle (default 1).
     *
     * Pair with a fast PDO slice: e.g. every_n=10 gives a full exchange
     * at 1/10 rate while the slice's hot bytes run every cycle.
     */
    void setImageExchangeDecimation(uint32_t every_n) {
        image_every_n_ = every_n ? every_n : 1;
    }
    uint32_t imageExchangeDecimation() const { return image_every_n_; }

    // ----- Statistics -----

    struct Stats {
        uint32_t success{0};
        uint32_t wkc_errors{0};
        uint32_t send_errors{0};
        uint32_t timeout_errors{0};
        /// Responses whose echoed send-generation didn't match the pending
        /// send — a stale deposit surviving a timed-out cycle.
        uint32_t stale_responses{0};
        /// Wire RTT of replies that arrived in-generation, measured from
        /// slice emit to the deposit's stamp_ns (kernel RX stamp where
        /// available).  min/avg/max in ns; rtt_samples counts arrivals.
        uint64_t rtt_ns_sum{0};
        uint32_t rtt_ns_min{0};
        uint32_t rtt_ns_max{0};
        uint32_t rtt_samples{0};
        /// Host stalls detected in the polled LRW path (gap between
        /// exchangeLRW calls exceeded stall detection threshold).
        uint32_t stall_events{0};
        /// Frames pulled off the wire by post-stall/timeout drains.
        uint32_t drained_frames{0};
        /// Currently consecutive LRW timeouts (resets on success).
        uint32_t consecutive_timeouts{0};
        /// Cyclic counter-trailer datagrams whose echo did not match the
        /// send — a late/stale frame, consumed and rejected.
        uint32_t counter_mismatches{0};
        /// Cyclic DC-timepoint datagrams that timed out or failed WKC.
        uint32_t dc_timeouts{0};
    };
    Stats getStats() const;
    void  resetStats();

    // ----- Stall self-heal (polled LRW path) -----

    /// Response wait budget per LRW datagram, ms (default 10).
    void setLrwResponseTimeoutMs(uint32_t ms) { response_timeout_ms_ = ms ? ms : 1; }
    uint32_t lrwResponseTimeoutMs() const { return response_timeout_ms_; }

    /**
     * @brief Gap between exchangeLRW calls that counts as a host stall,
     *        in microseconds (default 5000; 0 disables).
     *
     * On a detected stall the manager purges pending response waiters and
     * drains the wire backlog before sending, so a stale echo cannot
     * satisfy a new request on a reused idx — then resumes exchanging.
     */
    void setStallDetectionGapUs(uint32_t us) { stall_detect_us_ = us; }
    uint32_t stallDetectionGapUs() const { return stall_detect_us_; }

    /// Consecutive-timeout count that triggers a ring probe + warning
    /// (default 64; 0 disables).
    void setEscalateAfterTimeouts(uint32_t n) { escalate_after_timeouts_ = n; }

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
    // Stall self-heal state (polled LRW path).
    uint32_t response_timeout_ms_{10};
    uint32_t stall_detect_us_{5000};
    uint32_t escalate_after_timeouts_{64};
    int64_t  last_call_ns_{0};
    int64_t  last_timeout_log_ns_{0};
    uint32_t timeout_log_suppressed_{0};
    bool     escalate_logged_{false};

    /// Detect a host stall since the last exchange call; drain the wire
    /// backlog when one fired.  Pending waiters are NOT purged — backlog
    /// frames are routed normally so late replies still reach them.
    void stallCheck();
    /**
     * @brief Claim the response slot this polled exchange will send on.
     *
     * Drains the wire backlog first (stale echoes drop unrouted while
     * frames for other live waiters are delivered), then walks allocIdx()
     * until a slot is claimed.  A busy slot belongs to a live waiter and
     * is skipped — the exchange only ever prunes the slot it transmits on.
     *
     * @param idx_out  datagram index to transmit on (also set when the
     *                 transport lacks pre-registration).
     * @param resp     response buffer bound to the claimed slot.
     * @return claimed slot handle, IPDOTransport::kPreRegInvalid when the
     *         transport does not support pre-registration (caller falls
     *         back to waitForResponseIdx on idx_out), or
     *         IPDOTransport::kPreRegBusy when every candidate slot was
     *         owned by a live waiter.
     */
    size_t claimExchangeWaiter(uint8_t& idx_out, RxDatagram& resp);
    /// Timeout bookkeeping: rate-limited log (4 Hz), drain the wire,
    /// count the streak, ring-probe after escalate_after_timeouts_
    /// consecutive failures.  `what` is the caller's function name.
    void onExchangeTimeout(const char* what);
    /// Success bookkeeping: reset the consecutive-timeout streak.
    void onExchangeSuccess();

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

    // Sticky per-slave logical window assignment.  A slave's FMMUs are
    // programmed once during its PDO configuration using the addresses in
    // effect at that time; if a later slave's configuration rebuilt the
    // map and moved an earlier slave's window, that slave's FMMU would
    // silently point at the wrong logical region (watchdog trips, PDO
    // data never exchanged).  Windows are therefore assigned append-only
    // in configuration order and never relocated until deinit().
    static constexpr uint32_t kUnassigned = 0xFFFFFFFFu;
    std::array<uint32_t, PDO::kMaxPDOSlaves> slave_log_base_{};
    std::array<uint32_t, PDO::kMaxPDOSlaves> slave_log_size_{};
    uint32_t next_free_log_{0};  ///< next free logical offset (rel. to base)
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
    /// One in-flight image slice.  `pos` is the ROTATING pool position
    /// the datagram was sent on — slices no longer own fixed slots, so
    /// a late response deposits into a mailbox nobody else will re-arm
    /// until the pool wraps (82 sends), and the generation/counter
    /// checks reject it if it outlives that window.
    struct PendingSlice { uint64_t token; uint32_t off; uint32_t len;
                          uint8_t gen; uint8_t pos; };
    std::array<PendingSlice, kMaxCyclicSlices> cyclic_pending_{};
    uint8_t  cyclic_pending_count_{0};   ///< slices awaiting collect
    uint8_t  cyclic_slice_count_{1};     ///< slices needed for the image
    uint64_t cyclic_deadline_ns_{0};     ///< collect deadline (mono ns)
    uint64_t cyclic_send_ns_{0};         ///< slice emit time (mono ns)
    ProcessImage* pending_image_{nullptr};

    // ---- Rotating pool / counter trailer / DC timepoint ------------
    uint8_t  pool_pos_next_{0};   ///< next free rotating-pool position
    uint64_t lrw_counter_{0};     ///< counter stamped into the last send
    uint64_t lrw_counter_ok_{0};  ///< counter verified by the last collect
    /// Bookkeeping for the trailing counter datagram (one per send —
    /// a dedicated pool position, payload = the counter bytes at the
    /// first unmapped logical address, echoed verbatim).
    uint8_t  cnt_pos_{0};
    uint64_t cnt_token_{0};
    uint8_t  cnt_gen_{0};
    uint64_t cnt_value_{0};       ///< counter bytes as sent (LE64)
    bool     cnt_pending_{false};
    /// Bookkeeping for the DC-timepoint datagram (fixed idx 0xFF =
    /// pool position kCyclicDcPoolPos).
    uint64_t dc_token_{0};
    uint8_t  dc_gen_{0};
    bool     dc_pending_{false};
    int32_t  dc_slave_pos_{-1};   ///< APRD position; <0 = disabled
    uint64_t dc_time_ns_{0};      ///< last verified DC System Time
    bool     dc_time_valid_{false};

    /// Draw the next rotating-pool position (wraps 0..kNumCyclicSlots-1).
    uint8_t allocPoolPos() {
        const uint8_t p = pool_pos_next_;
        pool_pos_next_ = static_cast<uint8_t>(
            (p + 1) % kNumCyclicSlots);
        return p;
    }

    /// Expected-WKC per slice — kWkcUnknown = learn from first success
    /// (derived from the slave set by cyclicSend when a mapping is known).
    std::array<uint16_t, kMaxCyclicSlices> expected_wkc_{};
    /// Last collect outcome per image slice — packed CyclicSliceHealth
    /// words (single atomic load/store, never torn for readers).
    std::array<uint64_t, kMaxCyclicSlices> slice_health_{};
    bool strict_wkc_{true};

    // ---- User PDO slices (cyclic thread only) ---------------------------
    struct PDOSliceRun {
        uint32_t off{0};                    ///< image-space offset
        uint32_t len{0};                    ///< bytes (<= maxSliceLength)
        uint8_t  slot{0};                   ///< slice slot (idx 0xE0+slot)
        uint64_t token{0};                  ///< pending seq token
        uint8_t  gen{0};                    ///< send generation expected
        uint8_t  sent{0};                   ///< datagram emitted this cycle
        uint16_t expected_wkc{kWkcUnknown}; ///< derived/learned WKC
        uint64_t health_packed{0};          ///< packed CyclicSliceHealth
    };
    struct PDOSlice {
        std::array<PDOSliceRun, kMaxSliceRuns> runs{};
        uint8_t  run_count{0};
        /// Spec for replanning on mapping-epoch change (config data only —
        /// never touched on the RT path except during a replan).
        PDOSliceSpec spec;
        uint32_t every_n{1};
        uint32_t cycle_mod{0};              ///< decimation phase counter
        uint64_t deadline_ns{0};            ///< collect deadline this emit
        bool     pending{false};            ///< runs in flight
    };
    std::vector<PDOSlice> slices_;
    /// Mapping epoch the runs were planned on.  UINT32_MAX = "never
    /// planned" — a define/clear stamps it so the next emitSlices()
    /// replans even when the mapping's epoch is still 0 (add_*pdo()
    /// doesn't bump the epoch, only clear()/remove_* do).
    uint32_t slice_epoch_{0xFFFFFFFFu};
    uint64_t exchange_cycle_{0};  ///< cyclicSend invocation counter
    uint32_t image_every_n_{1};   ///< whole-image decimation
    /// Staging for non-image-mode slice payloads (gather per run).
    std::unique_ptr<uint8_t[]> slice_payload_;
    uint32_t slice_payload_size_{0};

    /// Re-plan every slice's runs against the current mapping — assigns
    /// slice slots in define-order and derives per-run expected WKC.
    /// Runs on epoch change or first send after define.
    bool replanSlices(const PDO::PDOMapping& mapping);
    /// Resolve a spec to contiguous runs (no slot assignment).
    size_t resolveSpecRuns(const PDO::PDOMapping& mapping,
                           const PDOSliceSpec& spec,
                           std::array<std::pair<uint32_t,uint32_t>,
                                      kMaxSliceRuns>& out) const;
    /// cyclicSend() helper — emit due slices onto their slice slots.
    void emitSlices(const PDO::PDOMapping& mapping, ProcessImage* image,
                    uint32_t rx_timeout_ns);
    /// cyclicCollect() helper — wait/scatter/publish pending slices.
    /// Shares the image input bank with the full-image publish so later
    /// (fresher) slice data lands on top.  Returns false on any miss.
    bool collectSlices(const PDO::PDOMapping& mapping, ProcessImage* image);

    /// Build the log prefix for a slave (uses prefix_provider_ if set, else default)
    std::string slavePrefix(uint16_t idx) const {
        if (prefix_provider_) return prefix_provider_(idx);
        return std::format("Slave {}", idx);
    }
};

} // namespace EtherCAT
