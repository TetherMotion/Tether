/**
 * @file PDOManager.hpp
 * @brief EtherCAT PDO mapping/transfer manager (instance-based, no globals).
 *
 * Split out of PDOManager.hpp.
 */

#pragma once

#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <format>
#include <functional>
#include <memory>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "tether/platform/EspCompat.hpp"
#include "tether/ethercat/CyclicChannel.hpp"
#include "tether/ethercat/DebugFlags.hpp"
#include "tether/ethercat/EtherCATConfig.hpp"
#include "tether/ethercat/IPDOTransport.hpp"
#include "tether/ethercat/PDOKeepAlive.hpp"
#include "tether/ethercat/PDOMapping.hpp"
#include "tether/ethercat/PDOModes.hpp"
#include "tether/ethercat/PDOTypes.hpp"
#include "tether/ethercat/ProcessImage.hpp"
#include "tether/ethercat/SMRegisters.hpp"
#include "tether/ethercat/Types.hpp"

#include <atomic_queue/atomic_queue.h>

#ifdef ESP_PLATFORM
#include "esp_eth_driver.h"
#endif

namespace EtherCAT {

class PDOManager;
class LogicalAddressManager;

namespace PDO {

// ============================================================================
// Backward-compatible free functions (delegate to PDOManager)
// ============================================================================

bool       pdo_init(PDOManager& mgr);
void       pdo_deinit(PDOManager& mgr);
PDOMapping& pdo_get_mapping(PDOManager& mgr);
SlaveConfig* pdo_get_slave_configs(PDOManager& mgr);
bool       pdo_configure_slave_sms(PDOManager& mgr, uint16_t slave_index);
uint16_t   pdo_configure_all_slave_sms(PDOManager& mgr, uint16_t slave_count);
bool       pdo_exchange_all(PDOManager& mgr);
bool       pdo_exchange_lrw(PDOManager& mgr, uint16_t slave_count);
void       pdo_get_lrw_stats(PDOManager& mgr,
                             uint32_t* success, uint32_t* wkc_errors,
                             uint32_t* send_errors, uint32_t* timeout_errors);
void       pdo_set_separate_mode(PDOManager& mgr, bool separate);
bool       pdo_get_separate_mode(PDOManager& mgr);
bool       pdo_exchange_separate(PDOManager& mgr, uint16_t slave_count);
void       pdo_get_separate_stats(PDOManager& mgr,
                                  uint32_t* lwr_success, uint32_t* lwr_wkc_errors,
                                  uint32_t* lrd_success, uint32_t* lrd_wkc_errors);
void       pdo_set_physical_mode(PDOManager& mgr, bool physical);
bool       pdo_get_physical_mode(PDOManager& mgr);
bool       pdo_exchange_physical(PDOManager& mgr, uint16_t slave_count);
void       pdo_get_physical_stats(PDOManager& mgr,
                                  uint32_t* fpwr_success, uint32_t* fpwr_wkc_errors,
                                  uint32_t* fprd_success, uint32_t* fprd_wkc_errors);
bool       pdo_send_rxpdo(PDOManager& mgr, size_t entry_index);
bool       pdo_receive_txpdo(PDOManager& mgr, size_t entry_index);
bool       pdo_finalize_mapping(PDOManager& mgr, uint16_t slave_index);
PDOStats   pdo_get_stats(PDOManager& mgr);
void       pdo_reset_stats(PDOManager& mgr);

} // namespace PDO

// ============================================================================
// PDOManager — instance-based, owns all PDO/SM state (no globals)
// ============================================================================

/**
 * @brief Instance-based PDO manager.
 *
 * Each PDOManager owns its own SlaveConfig array, PDOMapping, PDOStats,
 * and mode/transfer counters.  Multiple independent instances can co-exist.
 * Network I/O is performed through the injected IPDOTransport.
 */
class PDOManager {
public:
    explicit PDOManager(IPDOTransport& transport);
    ~PDOManager();

    PDOManager(const PDOManager&)            = delete;
    PDOManager& operator=(const PDOManager&) = delete;

    // ----- Lifecycle -----
    bool init();
    void deinit();
    bool isInitialized() const;

    // ----- Debug flags -----
    void setDebugFlags(const EtherCATMasterDebugFlags* flags) { debug_flags_.store(flags, std::memory_order_relaxed); }

    // ----- Debug gate (for conditional debugging checkpoints) -----
    void setDebugGate(DebugGate* gate) { debug_gate_ = gate; }

    // ----- Configuration Access -----
    PDO::PDOMapping&       mapping();
    const PDO::PDOMapping& mapping() const;

    PDO::SlaveConfig*       slaveConfigs();
    const PDO::SlaveConfig* slaveConfigs() const;

    size_t slaveCount() const;
    void   setSlaveCount(size_t count);

    // ----- Statistics -----
    PDO::PDOStats  getStats() const;
    void           resetStats();
    PDO::PDOStats& statsRef();

    /// Log the transfer statistics block (FMMU/LRW cycle counters plus
    /// physical FPWR/FPRD counters) — reveals WKC errors where a slave
    /// stopped acknowledging frames.
    void dumpStats(const char* tag) const;

    // ----- Per-slave PDO counter accessors -----
    bool     hasSlavePDOEntries(uint16_t slave_index) const;
    uint32_t getSlavePDORequestCount(uint16_t slave_index) const;
    uint32_t getSlavePDOReplyCount(uint16_t slave_index) const;

    // ----- SM Configuration -----
    bool     configureSlavesSMs(uint16_t slave_index);
    uint16_t configureAllSlaveSMs(uint16_t slave_count);

    // ----- Mapping Finalization -----

    /**
     * @brief Ensure the slave's configured station address (reg 0x0010) is
     *        known to the PDO mapping.
     *
     * Entries registered through add_rxpdo()/add_txpdo() take their
     * `configured_address` (used by FPWR/FPRD transfers) from the mapping's
     * per-slave table.  When that table is still empty for @p slave_index
     * this issues a single APRD of register 0x0010 and installs the result
     * in both the SlaveConfig and the mapping.  Called automatically by
     * finalizeMapping(), so callers never need to read 0x0010 themselves.
     *
     * @return true when the address is known (cached or freshly read).
     */
    bool ensureConfiguredAddress(uint16_t slave_index);

    bool finalizeMapping(uint16_t slave_index);

    // ----- PDO Transfer (single entry) -----
    bool sendRxPDO(size_t entry_index);
    bool receiveTxPDO(size_t entry_index);
    bool exchangeAll();

    /**
     * @brief Start a background keep-alive exchange (see PDOKeepAlive).
     *
     * Call before configuring additional slaves so the slaves that are
     * already in OP keep receiving their process data (the keep-alive
     * exchanges the mapping as it exists — the "old" image — until the
     * new slave's entries are registered and it joins the image).
     *
     * @param period  Exchange period (default 1 ms).
     * @return RAII guard; destroying it stops the keep-alive.
     */
    std::unique_ptr<PDOKeepAlive> startKeepAlive(
        std::chrono::milliseconds period = std::chrono::milliseconds{1});

    /**
     * @brief Exchange each configured slave's logical window — one LRW
     *        datagram per slave.
     *
     * Lightweight keep-alive exchange used by PDOKeepAlive: for every
     * slave that already has an assigned logical window (i.e. finished
     * its PDO/FMMU configuration) a single LRW datagram covering exactly
     * that window is emitted.  Slaves still being initialized have no
     * window yet and are skipped — they join automatically once their
     * configuration assigns one.
     *
     * Falls back to a full exchangeAll() when no logical address
     * manager is configured.
     */
    bool exchangeConfiguredSlaveWindows();

    // ----- Split send/receive (Mode 1: direct, user-driven) -----
    // sendAll() sends all RxPDO datagrams and returns immediately.
    // receiveAll() waits for all TxPDO responses and copies data into buffers.
    // exchangeAll() = sendAll() + receiveAll() (backward compat).
    // For LRW mode, sendAll() does the full atomic exchange; receiveAll() is a no-op.
    bool sendAll();
    bool receiveAll();

    // ----- Partial LRW exchange (logical-address slices) -----
    // Requires a logical address manager (setLogicalAddressManager()).
    // Allows a process image larger than one Ethernet frame to be exchanged
    // as several datagrams (partial reads) instead of splitting a frame —
    // e.g. a FSoE region and a separate RSAP/debug region.  These bypass the
    // sendAll()/receiveAll() split state (they are self-contained atomic
    // LRW exchanges).
    bool exchangeLRWSlice(uint32_t offset, uint32_t length);
    /// Maximum slice length that fits one LRW datagram (0 if unavailable).
    uint32_t maxLogicalSliceLength() const;
    /// Logical placement of every enabled PDO entry (empty if unavailable).
    std::vector<PDO::LogicalEntrySlice> describeLogicalEntries() const;

    /**
     * @brief Whole-image LRW exchange over the reserved-slot cyclic fast path.
     *
     * For use inside CyclicExecutive's exchange step.  Bypasses the
     * TransactionRouter entirely when the transport supports the fast path;
     * falls back to exchangeAllLRW() semantics otherwise.  Keeps the
     * per-slave PDO counters that the OP-transition check reads.
     *
     * @param rx_timeout_ns  Response wait budget in nanoseconds.
     */
    bool exchangeAllLRWCyclic(uint32_t rx_timeout_ns = 200'000,
                              ProcessImage* image = nullptr);

    /**
     * @brief Split-phase halves of exchangeAllLRWCyclic()
     *        (ExchangePlacement != Atomic).
     *
     * cyclicSend() emits the LRW slice datagram(s); cyclicCollect()
     * waits/publishes/scatters against the deadline anchored at send
     * time.  Collect mirrors the per-slave PDO counters.  Without a
     * logical address manager cyclicSend falls back to the atomic
     * exchangeAll() and cyclicCollect() is a no-op.
     */
    bool cyclicSend(ProcessImage* image, uint32_t rx_timeout_ns = 200'000);
    bool cyclicCollect(ProcessImage* image);
    bool cyclicExchangePending() const;

    // ---- User-defined PDO slices --------------------------------------
    // Declarative custom slices exchanged on dedicated wire indices —
    // see PDOSliceSpec / LogicalAddressManager::definePDOSlice().  All
    // no-ops returning failure when no logical address manager exists.
    uint32_t definePDOSlice(const PDOSliceSpec& spec);
    bool     clearPDOSlices();
    size_t   pdoSliceCount() const;
    /// Blocking one-off slice exchange on the async path (no fast path).
    bool     exchangePDOSlice(const PDOSliceSpec& spec);
    /// Run the whole-image exchange every Nth cycle (default 1).
    void     setImageExchangeDecimation(uint32_t every_n);

    /// Slices the current image occupies on the wire (1 = single frame).
    uint8_t cyclicSliceCount() const;

    /// Strict per-slice WKC verify — see LogicalAddressManager::setStrictWkc.
    void setCyclicStrictWkc(bool strict);

    /**
     * @brief Configure @p image for the current PDO mapping in @p mode.
     *
     * Computes each enabled entry's byte offset in the LRW process image;
     * entries sharing a byte with a neighbour or flagged
     * `PDOEntry::image_exclude` stay buffered (offset -1).
     * Requires an initialized LogicalAddressManager.
     *
     * @param shm_name  Optional POSIX shm export — the image's regions
     *        live in a shared segment a process-external motion source
     *        can ProcessImage::attachShared() to.  Forces Direct-mode
     *        semantics.
     */
    bool configureProcessImage(ProcessImage& image, ImageMode mode,
                               const char* shm_name = nullptr);

    // ----- Callback mode (Mode 3) -----
    // Per-entry callbacks fire during sendAll()/receiveAll() on the calling thread.
    // User must ensure callbacks are RT-safe if called from a realtime context.
    void configureCallbackMode(const CallbackModeConfig& config = {});
    void setTxSentCallback(size_t entry_index, PDOTxSentCallback callback);
    void setRxReceivedCallback(size_t entry_index, PDORxReceivedCallback callback);
    PDOMode getMode() const { return mode_; }

    // ----- Queue mode (Mode 2) -----
    // Audio-driver model: user pushes TX data and pulls RX data via lock-free queues.
    // An internal RT loop calls queueCycle() each cycle.
    void configureQueueMode(const QueueModeConfig& config = {});
    bool enqueueTx(size_t entry_index, std::shared_ptr<PDOFrame> frame);
    bool tryDequeueRx(size_t entry_index, std::shared_ptr<PDOFrame>& frame);
    bool tryPollEvent(std::shared_ptr<PDOEvent>& event);
    void setUnderrunCallback(UnderrunCallback callback);

    // Called by the RT loop each cycle (Mode 2).
    // Pops from TX queues, applies underrun policy, calls sendAll()+receiveAll(),
    // pushes received data to RX queues and events to the event queue.
    bool queueCycle();

    // ----- PDO Exchange modes -----
    bool exchangeLRW(uint16_t slave_count);
    bool exchangeSeparate(uint16_t slave_count);
    bool exchangePhysical(uint16_t slave_count);

    // ----- Mode settings -----
    void setSeparateMode(bool separate);
    bool getSeparateMode() const;
    void setPhysicalMode(bool physical);
    bool getPhysicalMode() const;

    // ----- Detailed per-mode statistics -----

    struct LRWStats {
        uint32_t lrw_success{0};
        uint32_t lrw_wkc_errors{0};
        uint32_t lrw_send_errors{0};
        uint32_t lrw_timeout_errors{0};
    };
    LRWStats getLRWStats() const;

    struct SeparateStats {
        uint32_t lwr_success{0};
        uint32_t lwr_wkc_errors{0};
        uint32_t lrd_success{0};
        uint32_t lrd_wkc_errors{0};
        uint32_t send_errors{0};
        uint32_t timeout_errors{0};
    };
    SeparateStats getSeparateStats() const;

    struct PhysicalStats {
        uint32_t fpwr_success{0};
        uint32_t fpwr_wkc_errors{0};
        uint32_t fprd_success{0};
        uint32_t fprd_wkc_errors{0};
        uint32_t send_errors{0};
        uint32_t timeout_errors{0};
    };
    PhysicalStats getPhysicalStats() const;

    struct TransferStats {
        uint32_t rxpdo_debug_count{0};
        uint32_t rxpdo_confirmed_ok{0};
        uint32_t rxpdo_confirmed_fail{0};
        uint32_t txpdo_debug_count{0};
    };
    TransferStats getTransferStats() const;

    IPDOTransport& transport();

    void setLogicalAddressManager(LogicalAddressManager* mgr) { logical_addr_mgr_ = mgr; }
    LogicalAddressManager* logicalAddressManager() const { return logical_addr_mgr_; }

    // ----- Log Prefix (set by Master from per-slave name) -----

    /// @brief Set a function that returns the log prefix for a given slave index.
    void setPrefixProvider(std::function<std::string(uint16_t)> provider) {
        prefix_provider_ = std::move(provider);
    }

private:
    /// Build the log prefix for a slave (uses prefix_provider_ if set, else default)
    std::string slavePrefix(uint16_t idx) const {
        if (prefix_provider_) return prefix_provider_(idx);
        return std::format("Slave {}", idx);
    }

    IPDOTransport& transport_;
    std::function<std::string(uint16_t)> prefix_provider_;

    // Owned state (formerly globals)
    PDO::SlaveConfig slave_configs_[PDO::kMaxPDOSlaves];
    PDO::PDOMapping  mapping_;
    PDO::PDOStats    stats_{};
    bool             initialized_ = false;
    size_t           slave_count_ = 0;

    std::atomic<const EtherCATMasterDebugFlags*> debug_flags_{nullptr};
    DebugGate* debug_gate_ = nullptr;
    bool first_rxpdo_emitted_ = false;
    bool first_txpdo_emitted_ = false;

    // Mode flags
    bool use_separate_commands_ = false;
    bool use_physical_mode_     = false;

    LogicalAddressManager* logical_addr_mgr_ = nullptr;

    // Per-mode stats
    LRWStats      lrw_stats_;
    SeparateStats separate_stats_;
    PhysicalStats physical_stats_;
    TransferStats transfer_stats_;

    // ----- Private helpers -----
    bool rxPDODebug(uint16_t slave_index = 0xFFFF) const {
        const auto* df = debug_flags_.load(std::memory_order_relaxed);
        if (!df) return false;
        if (slave_index == 0xFFFF) return df->rxPDO;
        return df->rxPDO && df->rxPDOFilt.allows(slave_index);
    }
    bool txPDODebug(uint16_t slave_index = 0xFFFF) const {
        const auto* df = debug_flags_.load(std::memory_order_relaxed);
        if (!df) return false;
        if (slave_index == 0xFFFF) return df->txPDO;
        return df->txPDO && df->txPDOFilt.allows(slave_index);
    }

    bool writeSMConfig(uint16_t adp, uint8_t sm_index,
                       const PDO::SyncManagerConfig& config,
                       uint16_t slave_index = 0xFFFF);
    bool readSMStatus(uint16_t adp, uint8_t sm_index, uint8_t& status);

    // Transfer helpers
    bool sendRxPDOPosition(const PDO::PDOEntry& entry);
    bool recvTxPDOPosition(PDO::PDOEntry& entry);
    bool sendRxPDOConfigured(const PDO::PDOEntry& entry);
    bool recvTxPDOConfigured(PDO::PDOEntry& entry);
    bool sendRxPDOBroadcast(const PDO::PDOEntry& entry, uint16_t expected_wkc);
    bool recvTxPDOBroadcast(PDO::PDOEntry& entry, uint16_t expected_wkc);

    // Split send/receive internal state (used by sendAll/receiveAll)
    struct SplitState {
        bool send_phase_ok = true;
        bool lrw_mode = false;  // true if last sendAll() used LRW (receiveAll is no-op)
        std::vector<uint8_t> rx_confirmed_idxs;
        std::vector<size_t>  rx_confirmed_entry_idxs;
        // Pre-registered slot handles and response buffers for race-free
        // multi-datagram exchanges.  Populated by sendAll() before the send,
        // consumed by receiveAll().
        std::vector<size_t>  rx_confirmed_slots;
        std::vector<RxDatagram> rx_confirmed_responses;
    };
    SplitState split_state_;

    // Mode state
    PDOMode mode_ = PDOMode::Direct;

    // Callback mode storage (Mode 3)
    CallbackModeConfig callback_config_;
    struct CallbackEntry {
        PDOTxSentCallback tx_sent;
        PDORxReceivedCallback rx_received;
    };
    std::vector<CallbackEntry> callbacks_;  // Indexed by PDO entry index

    // Queue mode storage (Mode 2)
    // Uses AtomicQueueB2 (runtime-size, state-based) for shared_ptr elements.
    // Queues are heap-allocated via unique_ptr because AtomicQueueB2 is non-copyable
    // and requires a size parameter at construction.
    using FrameQueue = atomic_queue::AtomicQueueB2<std::shared_ptr<PDOFrame>>;
    using EventQueue = atomic_queue::AtomicQueueB2<std::shared_ptr<PDOEvent>>;

    QueueModeConfig queue_config_;
    std::vector<std::unique_ptr<FrameQueue>> tx_queues_;   // Per-entry TX queues (user → RT)
    std::vector<std::unique_ptr<FrameQueue>> rx_queues_;   // Per-entry RX queues (RT → user)
    std::unique_ptr<EventQueue> event_queue_;              // Global event queue
    std::vector<std::shared_ptr<PDOFrame>> last_tx_frames_; // For RepeatLastFrame underrun policy
    UnderrunCallback underrun_callback_;
};

} // namespace EtherCAT
