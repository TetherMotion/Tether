/**
 * @file PDOManager.hpp
 * @brief EtherCAT Process Data Object (PDO) mapping and transfer system
 *
 * @details
 * This module provides a flexible system for mapping user-defined data structures
 * to EtherCAT Process Data Objects (PDOs).  PDOs are the primary mechanism for
 * real-time data exchange between the EtherCAT master and slaves.
 *
 * ## Architecture (refactored – no global state)
 *
 * All mutable state lives inside **PDOManager** instances.  Network I/O is
 * abstracted behind the **IPDOTransport** interface so that unit tests can
 * inject a mock without linking the full EtherCAT stack.
 *
 * Backward-compatible free functions in `namespace PDO` are retained but
 * now take a `PDOManager&` as their first parameter.
 */

#pragma once

#include <cstdint>
#include <cstddef>
#include <cstring>
#include <ctime>
#include <atomic>
#include <functional>
#include <bit>
#include <memory>
#include <vector>

#include "tether/platform/EspCompat.hpp"
#include "tether/ethercat/CyclicChannel.hpp"
#include "tether/ethercat/ProcessImage.hpp"
#include "tether/ethercat/EtherCATConfig.hpp"
#include "tether/ethercat/DebugFlags.hpp"
#include "tether/ethercat/Types.hpp"
#include "tether/ethercat/SMRegisters.hpp"
#include "tether/ethercat/PDOModes.hpp"

#include <atomic_queue/atomic_queue.h>

#ifdef ESP_PLATFORM
#include "esp_eth_driver.h"
#endif

namespace EtherCAT {

/// Monotonic nanoseconds for inline transport helpers (no Platform dep).
inline uint64_t monoNowNsFallback() {
    timespec ts{};
    ::clock_gettime(CLOCK_MONOTONIC, &ts);
    return static_cast<uint64_t>(ts.tv_sec) * 1'000'000'000ull
         + static_cast<uint64_t>(ts.tv_nsec);
}

// Forward declarations
class IPDOTransport;
class PDOManager;
class LogicalAddressManager;

namespace PDO {

// ============================================================================
// Constants and Limits
// ============================================================================

constexpr size_t kMaxPDOEntries = ECAT_PDO_MAX_ENTRIES;
constexpr size_t kMaxPDOSize    = ECAT_PDO_MAX_BUFFER_SIZE;
constexpr size_t kMaxPDOSlaves  = ECAT_PDO_MAX_SLAVES;

// ============================================================================
// Sync Manager Configuration
// ============================================================================

enum class SyncManagerType : uint8_t {
    Unused        = 0,
    MailboxWrite  = 1,
    MailboxRead   = 2,
    ProcessOutput = 3,
    ProcessInput  = 4
};

enum SyncManagerControl : uint8_t {
    SM_CTRL_MODE_MASK     = 0x03,
    SM_CTRL_MODE_BUFFERED = 0x00,
    SM_CTRL_MODE_MAILBOX  = 0x02,
    SM_CTRL_MODE_3PDO     = 0x03,
    SM_CTRL_DIR_READ      = 0x00,
    SM_CTRL_DIR_WRITE     = 0x04,
    SM_CTRL_IRQ_ECAT      = 0x08,
    SM_CTRL_IRQ_PDI       = 0x10,
    SM_CTRL_WATCHDOG      = 0x20,
    SM_CTRL_REPEAT_REQ    = 0x40,
};

struct SyncManagerConfig {
    uint16_t                      phys_start_addr;
    uint16_t                      length;
    EtherCAT::SyncManager::SMControlReg control;
    bool                          enable;
    SyncManagerType               type;

    static SyncManagerConfig mailbox_write(uint16_t addr, uint16_t len) {
        return { addr, len,
                 std::bit_cast<EtherCAT::SyncManager::SMControlReg>(
                     static_cast<uint8_t>(SM_CTRL_MODE_MAILBOX | SM_CTRL_DIR_WRITE | SM_CTRL_WATCHDOG)),
                 true, SyncManagerType::MailboxWrite };
    }
    static SyncManagerConfig mailbox_read(uint16_t addr, uint16_t len) {
        return { addr, len,
                 std::bit_cast<EtherCAT::SyncManager::SMControlReg>(
                     static_cast<uint8_t>(SM_CTRL_MODE_MAILBOX | SM_CTRL_DIR_READ | SM_CTRL_WATCHDOG)),
                 true, SyncManagerType::MailboxRead };
    }
    static SyncManagerConfig process_output(uint16_t addr, uint16_t len) {
        return { addr, len,
                 std::bit_cast<EtherCAT::SyncManager::SMControlReg>(
                     static_cast<uint8_t>(SM_CTRL_MODE_BUFFERED | SM_CTRL_DIR_WRITE | SM_CTRL_REPEAT_REQ)),
                 true, SyncManagerType::ProcessOutput };
    }
    static SyncManagerConfig process_input(uint16_t addr, uint16_t len) {
        return { addr, len,
                 std::bit_cast<EtherCAT::SyncManager::SMControlReg>(
                     static_cast<uint8_t>(SM_CTRL_MODE_BUFFERED | SM_CTRL_DIR_READ | SM_CTRL_REPEAT_REQ)),
                 true, SyncManagerType::ProcessInput };
    }
};

// ============================================================================
// PDO Addressing Modes
// ============================================================================

enum class PDOAddressMode : uint8_t {
    Broadcast         = 0,
    ConfiguredAddress  = 1,
    Position           = 2,
    Logical            = 3
};

// ============================================================================
// PDO Entry Definition
// ============================================================================

enum class PDODirection : uint8_t {
    TxPDO = 0,  ///< Slave→Master
    RxPDO = 1   ///< Master→Slave
};

struct PDOEntry {
    uint16_t       slave_index;
    PDODirection   direction;
    PDOAddressMode address_mode;

    uint16_t configured_address;
    uint32_t logical_address;
    uint16_t physical_offset;

    /**
     * @brief Manager-owned buffered-path storage (Q1: replaces the former
     *        caller-supplied `void* app_buffer`).
     *
     * The exchange gathers/scatters directly in this array.  Applications
     * reach it through PDOMapping::entryData()/entryDataMut()/
     * entryDataAs<T>() or an epoch-checked entryHandle() + resolve().
     * Fixed inline storage keeps PDOEntry trivially copyable and makes the
     * buffered path allocation-free after registration.
     */
    /// `mutable`: the exchange writes wire data here through const
    /// PDOMapping& — storage is I/O state, not mapping metadata (same
    /// const-escape the old app_buffer pointee had).
    alignas(8) mutable uint8_t storage[kMaxPDOSize];
    uint16_t data_size;

    uint16_t pdo_index;

    bool     enabled;
    /**
     * @brief Force this entry onto the buffered (storage) path even when
     *        a ProcessImage mode is active.  Set for FSoE-managed PDOs —
     *        safe frames are staged and CRC'd by the FSoE layer and must
     *        not be written in place by the application.
     */
    bool     image_exclude{false};
    uint32_t error_count;
    uint32_t success_count;
};

/// Logical placement of one enabled PDO entry within the process image,
/// using the layout of the logical LRW exchange.  Lets a caller compose
/// partial-read slices (e.g. group the FSoE PDOs apart from the RSAP/debug
/// PDOs) without knowing the internal address map.
struct LogicalEntrySlice {
    size_t       entry_index{0};   ///< index into PDOMapping
    uint16_t     slave_index{0};
    uint16_t     pdo_index{0};
    PDODirection direction{PDODirection::RxPDO};
    uint32_t     offset{0};        ///< byte offset from the base logical address
    uint16_t     length{0};
};

// ============================================================================
// PDO Mapping Manager (value type – no transport dependency)
// ============================================================================

class PDOMapping {
public:
    /// Register an RxPDO (master→slave).  Storage is manager-owned — the
    /// caller receives an entry index and accesses the bytes through
    /// entryDataMut()/entryDataAs<T>() or an epoch-checked entryHandle().
    int  add_rxpdo(uint16_t slave_index, uint16_t size,
                   uint16_t pdo_index = 0x1600,
                   PDOAddressMode mode = PDOAddressMode::Position);

    int  add_txpdo(uint16_t slave_index, uint16_t size,
                   uint16_t pdo_index = 0x1A00,
                   PDOAddressMode mode = PDOAddressMode::Position);

    int  add_broadcast_rxpdo(uint16_t size, uint16_t physical_offset);
    int  add_broadcast_txpdo(uint16_t size, uint16_t physical_offset);

    void set_slave_configured_address(uint16_t slave_index, uint16_t configured_addr);

    size_t         entry_count() const { return m_entry_count; }
    const PDOEntry* get_entry(size_t index) const;
    PDOEntry*       get_entry_mut(size_t index);
    void            clear();
    void            remove_entries_for_slave(uint16_t slave_index);

    // ---- Buffered-path data access (Q1) ---------------------------------
    /// Epoch-checked handle for a buffered entry.  offset stays -1 — the
    /// handle resolves to entry storage, not a process-image offset.
    /// The epoch changes on clear()/remove_entries_for_slave(), making
    /// stale handles fail resolve() instead of pointing at recycled slots.
    EntryHandle entryHandle(size_t index) const {
        EntryHandle h;
        h.index  = static_cast<uint32_t>(index);
        h.offset = -1;
        h.epoch  = m_epoch;
        return h;
    }
    /// Direct index access — the fast path for RT code that caches the
    /// pointer once after registration.  nullptr on out-of-range.
    uint8_t* entryDataMut(size_t index) {
        return index < m_entry_count ? m_entries[index].storage : nullptr;
    }
    const uint8_t* entryData(size_t index) const {
        return index < m_entry_count ? m_entries[index].storage : nullptr;
    }
    /// Typed access — nullptr when sizeof(T) exceeds the registered size.
    template<typename T> T* entryDataAs(size_t index) {
        return (index < m_entry_count && sizeof(T) <= m_entries[index].data_size)
                   ? reinterpret_cast<T*>(m_entries[index].storage) : nullptr;
    }
    template<typename T> const T* entryDataAs(size_t index) const {
        return (index < m_entry_count && sizeof(T) <= m_entries[index].data_size)
                   ? reinterpret_cast<const T*>(m_entries[index].storage)
                   : nullptr;
    }
    /// Epoch-checked resolve — nullptr when the handle is stale or the
    /// entry is out of range.
    uint8_t* resolveMut(const EntryHandle& h) {
        return (h.epoch == m_epoch) ? entryDataMut(h.index) : nullptr;
    }
    const uint8_t* resolve(const EntryHandle& h) const {
        return (h.epoch == m_epoch) ? entryData(h.index) : nullptr;
    }
    uint32_t epoch() const { return m_epoch; }

    size_t total_rxpdo_bytes() const;
    size_t total_txpdo_bytes() const;

private:
    PDOEntry m_entries[kMaxPDOEntries];
    size_t   m_entry_count = 0;
    uint16_t m_slave_configured_addrs[kMaxPDOSlaves] = {0};
    uint32_t m_epoch = 0;   ///< bumped on clear()/remove_entries_for_slave()
};

// ============================================================================
// Slave Configuration Structure
// ============================================================================

struct SlaveConfig {
    uint16_t slave_index;
    uint16_t configured_address;
    uint32_t vendor_id;
    uint32_t product_code;

    SyncManagerConfig sm[4];

    uint16_t rxpdo_size;
    uint16_t txpdo_size;
    uint16_t rxpdo_sm;
    uint16_t txpdo_sm;

    uint16_t mbx_write_offset;
    uint16_t mbx_write_size;
    uint16_t mbx_read_offset;
    uint16_t mbx_read_size;
    uint8_t  mbx_protocols;

    bool     configured;
    bool     operational;

    uint32_t pdo_request_count = 0;   // successful RxPDO sends (master -> slave)
    uint32_t pdo_reply_count   = 0;   // successful TxPDO receives (slave -> master)
};

// ============================================================================
// PDO Exchange Statistics
// ============================================================================

struct PDOStats {
    uint64_t total_cycles;
    uint64_t rxpdo_frames_sent;
    uint64_t txpdo_frames_recv;
    uint32_t rxpdo_errors;
    uint32_t txpdo_errors;
    uint32_t wkc_errors;
    uint32_t last_rxpdo_time_us;
    uint32_t last_txpdo_time_us;
    uint32_t max_cycle_time_us;
};

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
// IPDOTransport — abstract transport interface for PDO I/O
// ============================================================================

/**
 * @brief Transport interface for PDO operations.
 *
 * Abstracts the low-level EtherCAT I/O primitives needed by PDOManager.
 * Production builds use a concrete implementation that delegates to
 * Raw:: functions; unit tests inject a mock.
 */
class IPDOTransport {
public:
    virtual ~IPDOTransport() = default;

    /// Fire-and-forget index constant
    static constexpr uint8_t kFireAndForgetIdx = 0xFE;

    // ------------------------------------------------------------------
    // Cyclic fast path: reserved index range + fixed response slots.
    //
    // Datagrams sent with idx in [kCyclicSlotBase, kCyclicSlotBase +
    // kNumCyclicSlots) bypass TransactionRouter entirely: the RX parser
    // deposits them into fixed preallocated slots that the cyclic thread
    // polls on a sequence counter — no mutex, no condition variable, and
    // no RxDatagram copy on the hot path.  allocIdx() never returns an
    // index in this range.
    // ------------------------------------------------------------------
    static constexpr uint8_t kCyclicSlotBase = ::EtherCAT::kCyclicSlotBaseIdx;
    static constexpr size_t  kNumCyclicSlots = ::EtherCAT::kNumCyclicSlots;

    /// @return true if the transport implements the cyclic slot fast path.
    virtual bool supportsCyclicFastPath() const { return false; }

    /**
     * @brief Read the current sequence token of a cyclic slot.
     *
     * Call BEFORE sendCyclicDatagram().  The token distinguishes the
     * response belonging to the upcoming send from stale deposits.
     */
    virtual uint64_t cyclicSlotToken(uint8_t slot) {
        (void)slot; return 0;
    }

    /**
     * @brief Send a single datagram on a reserved cyclic slot index.
     * @param slot   Slot number in [0, kNumCyclicSlots)
     * @return true if the frame was handed to the wire.
     */
    virtual bool sendCyclicDatagram(Command cmd, uint8_t slot,
                                    uint16_t adp, uint16_t ado,
                                    const void* data, uint16_t datalen,
                                    bool roundtrip) {
        (void)cmd; (void)slot; (void)adp; (void)ado;
        (void)data; (void)datalen; (void)roundtrip;
        return false;
    }

    /**
     * @brief Wait until the response for @p token arrives on @p slot.
     *
     * On Linux this blocks in ppoll() on the socket + a deposit eventfd —
     * the calling thread sleeps until either the response lands or the
     * deadline passes.  Without a receive path it falls back to a bounded
     * sequence-counter spin.
     *
     * @param timeout_ns  Maximum wait in nanoseconds (sub-ms scale).
     * @return true and fills @p out on success; false on timeout/cancel.
     */
    virtual bool waitCyclicSlot(uint8_t slot, uint64_t token,
                                uint32_t timeout_ns, RxDatagram& out) {
        (void)slot; (void)token; (void)timeout_ns; (void)out;
        return false;
    }

    /**
     * @brief View-returning variant of waitCyclicSlot().
     *
     * `out.payload` points into the slot's inline buffer (software path)
     * or into channel-owned ring/bank memory (channel path — `out.channel`
     * / `out.cookie` then identify the held region).
     */
    virtual bool waitCyclicSlotView(uint8_t slot, uint64_t token,
                                    uint32_t timeout_ns,
                                    CyclicSlotView& out) {
        (void)slot; (void)token; (void)timeout_ns; (void)out;
        return false;
    }

    /**
     * @brief Wait until every slot in @p slot_mask has a deposit newer
     *        than its token — one wait for a whole multi-slice exchange.
     *
     * Transports with a real wake path override this so a sliced collect
     * pays ONE sleep instead of one per slice (Q3).  The default
     * implementation degenerates to a per-slot waitCyclicSlotView() loop
     * sharing the same deadline — correct on every transport, just
     * syscall-heavier.
     *
     * @param slot_mask  Bitmask over slots [0, kNumCyclicSlots)
     * @param tokens     Per-slot seq tokens, indexed by slot number
     * @param timeout_ns Shared deadline budget across the whole mask
     * @param views      Output array, indexed by slot number — filled
     *                   only for slots that arrived
     * @return The subset of @p slot_mask whose responses arrived before
     *         the deadline — full success iff the return == slot_mask.
     */
    virtual uint32_t waitCyclicSlotMask(uint32_t slot_mask,
                                        const uint64_t* tokens,
                                        uint32_t timeout_ns,
                                        CyclicSlotView* views) {
        // Per-slot fallback: walk the mask, sharing one CLOCK_MONOTONIC
        // deadline across slots.
        uint32_t arrived = 0;
        const uint64_t deadline = monoNowNsFallback() + timeout_ns;
        for (uint8_t s = 0; s < kNumCyclicSlots; ++s) {
            if (!(slot_mask & (1u << s))) continue;
            const uint64_t now = monoNowNsFallback();
            const uint32_t remain = now < deadline
                ? static_cast<uint32_t>(deadline - now) : 0;
            if (!waitCyclicSlotView(s, tokens[s], remain, views[s]))
                break;   // deadline shared — later slots are worse off
            arrived |= 1u << s;
        }
        return arrived;
    }

    /**
     * @brief Acquire the channel's next TX frame buffer (Rotating image
     *        mode).  nullptr when no channel exists.
     */
    virtual uint8_t* acquireCyclicTxFrame() { return nullptr; }

    /**
     * @brief Compose [eth][ecat][dg-hdr] into an acquired frame buffer
     *        (payload region follows at offset 26).
     */
    virtual void composeCyclicHeader(uint8_t* frame, Command cmd,
                                     uint8_t slot, uint16_t adp,
                                     uint16_t ado, uint16_t datalen,
                                     bool roundtrip) {
        (void)frame; (void)cmd; (void)slot; (void)adp; (void)ado;
        (void)datalen; (void)roundtrip;
    }

    /// Commit the frame composed into the last acquireCyclicTxFrame() buffer.
    virtual bool sendCyclicFrame(uint32_t frame_len) {
        (void)frame_len; return false;
    }

    /// @return the cyclic channel, or nullptr on the software path.
    virtual ICyclicChannel* cyclicChannel() { return nullptr; }

    virtual bool writeRegister(uint16_t adp, uint16_t ado,
                               const void* data, uint16_t len,
                               unsigned int timeout_ms) = 0;

    virtual bool readRegister(uint16_t adp, uint16_t ado,
                              void* data, uint16_t len,
                              unsigned int timeout_ms) = 0;

    virtual bool sendSingleDatagram(Command cmd, uint8_t idx,
                                    uint16_t adp, uint16_t ado,
                                    const void* data, uint16_t datalen,
                                    bool roundtrip) = 0;

    virtual size_t sendMultiDatagram(const MultiDatagramSpec* specs, size_t count) = 0;

    virtual bool waitForResponseIdx(uint8_t idx, unsigned int timeout_ms,
                                    RxDatagram& out) = 0;

    /// Pre-register a response waiter slot for @p idx BEFORE sending the frame.
    /// This avoids the send-then-register race when multiple datagrams share
    /// one frame.  Returns a slot handle; a value of kPreRegInvalid means
    /// the transport does not support pre-registration.
    virtual size_t preRegisterResponseWaiter(uint8_t idx,
                                             uint8_t* buffer, size_t buffer_size) {
        (void)idx; (void)buffer; (void)buffer_size;
        return static_cast<size_t>(-1);
    }

    /// Wait for a previously pre-registered response.  Fills @p out on success.
    virtual bool waitForPreRegistered(size_t slot, unsigned int timeout_ms,
                                      RxDatagram& out) {
        (void)slot; (void)timeout_ms; (void)out;
        return false;
    }

    /// Sentinel returned by preRegisterResponseWaiter when unsupported.
    static constexpr size_t kPreRegInvalid = static_cast<size_t>(-1);

    virtual uint8_t  allocIdx() = 0;
    virtual uint16_t adpForSlaveIndex(uint16_t slave_index) = 0;

    /// @return true if cancellation has been requested (e.g. during shutdown).
    /// Used by callers to suppress error logging when failures are expected.
    virtual bool isCancelRequested() const { return false; }

    /// Maximum EtherCAT payload bytes that fit in one Ethernet frame,
    /// including per-datagram overhead.  Used to size partial LRW slices.
    virtual size_t maxEtherCATPayloadPerFrame() const { return 1498; }
};

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

    // ----- Per-slave PDO counter accessors -----
    bool     hasSlavePDOEntries(uint16_t slave_index) const;
    uint32_t getSlavePDORequestCount(uint16_t slave_index) const;
    uint32_t getSlavePDOReplyCount(uint16_t slave_index) const;

    // ----- SM Configuration -----
    bool     configureSlavesSMs(uint16_t slave_index);
    uint16_t configureAllSlaveSMs(uint16_t slave_count);

    // ----- Mapping Finalization -----
    bool finalizeMapping(uint16_t slave_index);

    // ----- PDO Transfer (single entry) -----
    bool sendRxPDO(size_t entry_index);
    bool receiveTxPDO(size_t entry_index);
    bool exchangeAll();

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

private:
    IPDOTransport& transport_;

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
