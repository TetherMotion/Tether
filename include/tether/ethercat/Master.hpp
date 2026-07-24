#pragma once

/**
 * @file Master.hpp
 * @brief EtherCAT Master — class-based API for multi-instance support
 *
 * @details
 * Master encapsulates all state required to run an independent
 * EtherCAT master instance.  Multiple masters can coexist in the same
 * process, each driving a separate Ethernet interface.
 *
 * ## Quick start
 * @code
 *   EtherCAT::Master master;
 *   master.start(networkIface, srcMac);
 *
 *   // Wait for slaves
 *   while (master.getDiscoveredSlaveCount() == 0) { }
 *
 *   // Use sub-managers
 *   master.pdo().init();
 *   master.dc().init(dcConfig);
 *   master.dc().start();
 * @endcode
 */

#include <atomic>
#include <cstdint>
#include <cstring>
#include <deque>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <type_traits>
#include <vector>
#include <unordered_map>
#include <unordered_set>

#include "logging/DeduplicatingLogger.hpp"

#include "tether/ethercat/DebugFlags.hpp"
#include "tether/ethercat/DebugGate.hpp"
#include "tether/ethercat/SlaveIdentity.hpp"
#include "tether/ethercat/TetherConfig.hpp"
#include "tether/ethercat/Types.hpp"
#include "tether/ethercat/TransactionRouter.hpp"
#include "tether/platform/MessageQueue.hpp"
#include "tether/platform/EspCompat.hpp"

namespace EtherCAT {

struct PhysicalAddress {
    constexpr explicit PhysicalAddress(unsigned int slave_position_in)
        : slave_position(static_cast<uint16_t>(slave_position_in)) {}

    constexpr uint16_t raw() const { return static_cast<uint16_t>(0u - slave_position); }
    constexpr uint16_t slavePosition() const { return slave_position; }

private:
    uint16_t slave_position;
};

struct LogicalAddress {
    constexpr explicit LogicalAddress(unsigned int configured_address_in)
        : configured_address(static_cast<uint16_t>(configured_address_in)) {}

    constexpr uint16_t raw() const { return configured_address; }

private:
    uint16_t configured_address;
};

class SlaveAddress {
public:
    enum class Kind : uint8_t {
        Physical,
        Logical,
    };

    constexpr SlaveAddress(unsigned int slave_position_in)
        : kind_(Kind::Physical), value_(static_cast<uint16_t>(slave_position_in)) {}

    constexpr SlaveAddress(PhysicalAddress physical_address)
        : kind_(Kind::Physical), value_(physical_address.slavePosition()) {}

    constexpr SlaveAddress(LogicalAddress logical_address)
        : kind_(Kind::Logical), value_(logical_address.raw()) {}

    constexpr bool isPhysical() const { return kind_ == Kind::Physical; }
    constexpr bool isLogical() const { return kind_ == Kind::Logical; }
    constexpr uint16_t raw() const {
        return isPhysical() ? static_cast<uint16_t>(0u - value_) : value_;
    }
    constexpr uint16_t slavePosition() const { return value_; }
    constexpr Kind kind() const { return kind_; }

private:
    Kind kind_;
    uint16_t value_;
};

struct RegisterAddress {
    constexpr explicit RegisterAddress(uint16_t value_in) : value(value_in) {}
    constexpr uint16_t raw() const { return value; }

private:
    uint16_t value;
};

// Forward declarations for sub-managers
class IPDOTransport;
class PDOManager;
class LogicalAddressManager;
class DCManager;
class FoEManager;
class VoEManager;
class EoEManager;
class FaultDetector;
class IFaultTransport;
class SlaveStatusPoller;
class Slave;
class NonExistingSlave;

namespace SII {
    class SIIReader;
}

// Forward declarations for CoE/SDO namespace
namespace SDO {
    class ISDOTransport;
}
namespace CoE {
    class CoEManager;
}
namespace Raw {
    class CoeSDOChannel;
}

class IMotionControlLoop;

// ============================================================================
// Master
// ============================================================================

/**
 * @brief Owns all state for one EtherCAT master instance.
 *
 * Each instance manages its own network interface, packet routing,
 * slave discovery, and sub-manager objects (PDO, SDO, DC, …).
 * The class is **non-copyable** but can be moved.
 */
class Master {
public:
    /// Configuration knobs passed at construction time.
    struct Config {
        uint32_t rx_queue_depth     = 64;
        uint32_t txpdo_queue_depth  = 8;
        bool enable_mailbox_fallback = false; ///< Opt-in: force default mailbox on InvalidMailboxConfig

        /// Tuning knobs for `setPreopAndConfirm` (INIT -> PRE_OP transition).
        /// Defaults preserve the original production timing. Tests that simulate
        /// slaves which never reach PRE_OP can shrink these to avoid spending
        /// ~13s of wall time per attempt in `std::this_thread::sleep_for`.
        uint16_t preop_max_attempts   = 3;   ///< Outer retry attempts for the PRE_OP transition
        uint16_t preop_inner_tries    = 200; ///< AL_STATUS polls per attempt
        uint16_t preop_inner_sleep_ms = 20;  ///< Delay between AL_STATUS polls
        uint16_t preop_backoff_ms     = 200; ///< Base backoff before retry (scaled by attempt index)

#if TETHER_ENABLE_UDP_ENCAPSULATION
        /// EtherCAT-over-UDP encapsulation settings (opt-in, default: disabled).
        /// When enabled, frames are encapsulated as Ethernet/IPv4/UDP(port 34980)
        /// instead of using EtherType 0x88A4 directly.  This allows communicating
        /// with devices that use UDP encapsulation (e.g. some ESC-based slaves,
        /// tunneling over IP networks).
        ///
        /// @note This struct only exists when TETHER_ENABLE_UDP_ENCAPSULATION is
        ///       enabled at compile time.  When disabled, all UDP encapsulation
        ///       code is compiled out for zero overhead.
        struct UdpEncapsulation {
            bool     enabled          = false;          ///< Master switch (default: off)
            uint32_t source_ip        = 0;              ///< Source IPv4 (host byte order), 0 = 0.0.0.0
            uint32_t destination_ip   = 0xFFFFFFFF;     ///< Destination IPv4 (host byte order), default broadcast
            uint16_t source_port      = 0x88A4;         ///< UDP source port (default 34980)
            uint16_t destination_port = 0x88A4;         ///< UDP destination port (default 34980)
        } udp_encapsulation;
#endif
    };

    /** Enable or disable the mailbox fallback at runtime. */
    void setEnableMailboxFallback(bool enabled) { config_.enable_mailbox_fallback = enabled; }
    bool isMailboxFallbackEnabled() const { return config_.enable_mailbox_fallback; }

    /** Override the PRE_OP retry timing (useful for tests that emulate slaves). */
    void setPreopRetryConfig(uint16_t max_attempts, uint16_t inner_tries,
                             uint16_t inner_sleep_ms, uint16_t backoff_ms) {
        config_.preop_max_attempts   = max_attempts;
        config_.preop_inner_tries    = inner_tries;
        config_.preop_inner_sleep_ms = inner_sleep_ms;
        config_.preop_backoff_ms     = backoff_ms;
    }

    /** @return true if EtherCAT-over-UDP encapsulation is enabled. */
#if TETHER_ENABLE_UDP_ENCAPSULATION
    bool isUdpEncapsulationEnabled() const { return config_.udp_encapsulation.enabled; }
#else
    bool isUdpEncapsulationEnabled() const { return false; }
#endif

    /** Set a callback invoked when the master attempts the mailbox fallback for a slave. */
    void setMailboxFallbackCallback(std::function<void(uint16_t)> cb) { mailbox_fallback_cb_ = std::move(cb); }

    /** Test helper: force conservative mailbox defaults for a slave (returns true if applied) */
    bool forceMailboxDefaults(SlaveAddress slave_address);

    /**
     * @brief Set an explicit mailbox override for a slave using values from XML/ESI.
     *
     * These values will be applied during discovery *instead* of attempting to
     * read the mailbox configuration from the device EEPROM. This is useful
     * when you want to enforce vendor-supplied ESI values without reading SII.
     */
    void setMailboxOverride(SlaveAddress slave_address, uint16_t wr_addr, uint16_t wr_len,
                             uint16_t rd_addr, uint16_t rd_len, uint16_t proto);

    Master();
    explicit Master(const Config& config);
    ~Master();

    Master(const Master&)            = delete;
    Master& operator=(const Master&) = delete;
    Master(Master&&)                 = delete;
    Master& operator=(Master&&)      = delete;

    // ---- Lifecycle ---------------------------------------------------------

    /** Start master task using hardware-independent NetworkInterface. */
    void start(const NetworkInterface& iface, const uint8_t src_mac[6]);

    /** Stop the master task and shut down all sub-systems. */
    void stop();

    /** @return true while the master task is running. */
    bool isRunning() const;

    /** Request immediate cancellation of all blocking operations. */
    void requestCancel();

    /** @return true if cancellation has been requested. */
    bool isCancelRequested() const;

    /** Clear the cancellation flag (useful for re-starting). */
    void clearCancel();

    using MotionControlCallback = std::function<bool(double dt_seconds)>;

    struct RealtimeMotionLoopConfig {
        uint32_t cycle_period_us{1000};
        uint32_t sync_interval_cycles{10};
        bool enable_dc_synchronization{false};
    };

    struct PollingMotionLoopConfig {
        uint32_t cycle_period_us{1000};
        uint32_t sync_interval_cycles{10};
        bool enable_dc_synchronization{false};
        bool request_realtime_priority{true};
    };

    void setMotionControlCallback(MotionControlCallback callback);
    bool startRealtimeMotionControlLoop();
    bool startRealtimeMotionControlLoop(const RealtimeMotionLoopConfig& config);
    bool startPollingMotionControlLoop();
    bool startPollingMotionControlLoop(const PollingMotionLoopConfig& config);
    void stopMotionControlLoop();
    bool isMotionControlLoopRunning() const;

    // ---- Queue-mode RT loop (Mode 2) ---------------------------------------
    // Starts an internal RT loop that calls pdo.queueCycle() each cycle.
    // Requires PDOManager to be configured in Queue mode.
    bool startQueueModeLoop();
    bool startQueueModeLoop(const RealtimeMotionLoopConfig& config);
    void stopQueueModeLoop();
    bool isQueueModeLoopRunning() const;

    // ---- Frame handling ----------------------------------------------------

    /** Route a received Ethernet frame to the internal parser. */
    void handleRxFrame(const uint8_t* frame, size_t length);

    // ---- Discovery ---------------------------------------------------------

    uint16_t getDiscoveredSlaveCount() const;

    /**
     * @brief Discover slaves on the bus and initialise internal state.
     *
     * This is normally called by the application after start().
     */
    bool discoverSlaves();

    // ---- Slave access -------------------------------------------------------

    /**
     * @brief Access a slave by index.
     *
     * Returns a reference to the Slave at the given bus position.
     * If `slave_index >= getDiscoveredSlaveCount()`, returns a reference to
     * a NonExistingSlave sentinel that logs CRITICAL errors on every call.
     *
     * @param slave_index  Zero-based slave position on the bus
     * @return Reference to the slave (or NonExistingSlave if out of range)
     *
     * @code
     *   master.slave(0).configureMailbox();
     *   master.slave(0).transitionToPreOp();
     *   master.slave(0).transitionTo(SlaveState::SAFE_OP);
     * @endcode
     */
    Slave& slave(uint16_t slave_index);

    /**
     * @brief Initialise the slave vector after discovery.
     *
     * Called automatically when the master discovers slaves.  Can also be
     * called manually for testing.
     *
     * @param count  Number of slaves on the bus
     */
    void initSlaves(uint16_t count);

    /**
     * @brief Access the SII reader (lazily created).
     */
    SII::SIIReader& siiReader();

    // ---- SII EEPROM cache --------------------------------------------------

    /**
     * @brief Check the per-slave SII word cache.
     *
     * All SIIReader instances transparently hit this cache so ephemeral
     * readers (created by readSII(), readSIIIdentity(), etc.) do not
     * re-read already-fetched EEPROM words.
     *
     * @param slave_index  Slave index
     * @param word_addr    EEPROM word address
     * @param[out] out     Cached value (only valid on true return)
     * @return true if the word is cached
     */
    bool getSIICachedWord(uint16_t slave_index, uint16_t word_addr, uint16_t& out) const;

    /**
     * @brief Store a word in the per-slave SII word cache.
     */
    void setSIICachedWord(uint16_t slave_index, uint16_t word_addr, uint16_t value);

    /**
     * @brief Clear the SII word cache for a given slave (or all slaves).
     */
    void clearSIICache(uint16_t slave_index);

    // ---- AL state management -----------------------------------------------

    bool requestSlaveApplicationLayerState(SlaveAddress slave_address, uint8_t state_code);
    bool readSlaveApplicationLayerState(SlaveAddress slave_address, uint8_t& state_code);
    bool transitionSlaveToPreOperational(SlaveAddress slave_address);

    /**
     * @brief Configure SM2/SM3 (process data SMs) from SII EEPROM data
     * 
     * Reads the Sync Manager category from the slave's SII EEPROM and
     * populates g_slave_configs[slave_index].sm[2] and sm[3] with the
     * correct physical addresses, control bytes, and types. Then writes
     * them to the slave's SM registers.
     * 
     * This MUST be called after PDO mapping (SDO writes) and before
     * requesting SAFE_OP, because the slave validates SM2/SM3 configuration
     * during the PRE_OP → SAFE_OP transition.
     * 
     * @param slave_index Slave index (0-based)
     * @return true if SM2/SM3 were configured and written successfully
     */
    bool configureProcessDataSyncManagersFromSii(SlaveAddress slave_address);

    // ---- Transport primitives ----------------------------------------------

    bool sendRawFrame(const void* buf, size_t len);

    bool sendDatagram(Command cmd, uint8_t idx,
                      SlaveAddress slave_address, RegisterAddress register_address,
                      const void* data, uint16_t datalen,
                      bool roundtrip);

    bool sendSingleDatagram(Command cmd, uint8_t idx,
                            uint16_t adp, uint16_t ado,
                            const void* data, uint16_t datalen,
                            bool roundtrip);

    /// Pack multiple datagrams into one or more Ethernet frames and send immediately.
    /// Auto-splits across frames if total exceeds 1514 bytes.
    /// @param specs Array of datagram specifications
    /// @param count Number of specs
    /// @return Number of frames sent, or 0 on failure
    size_t sendMultiDatagram(const MultiDatagramSpec* specs, size_t count);

    bool writeRegister(SlaveAddress slave_address, RegisterAddress register_address,
                       const void* data, uint16_t len,
                       unsigned int timeout_ms = 200);

    bool writeRegister(SlaveAddress slave_address, uint16_t register_address,
                       const void* data, uint16_t len,
                       unsigned int timeout_ms = 200) {
        return writeRegister(slave_address, RegisterAddress(register_address), data, len, timeout_ms);
    }

    bool writeRegister(SlaveAddress slave_address, RegisterAddress register_address,
                       uint16_t value);

    bool writeRegister(SlaveAddress slave_address, uint16_t register_address,
                       uint16_t value) {
        return writeRegister(slave_address, RegisterAddress(register_address), value);
    }

    template<typename T>
    bool writeRegister(SlaveAddress slave_address, RegisterAddress register_address,
                       const T& value, unsigned int timeout_ms = 200) {
        static_assert(std::is_trivially_copyable_v<T>, "Register writes require trivially copyable types");
        return writeRegister(slave_address, register_address, &value, static_cast<uint16_t>(sizeof(T)), timeout_ms);
    }

    template<typename T>
    bool writeRegister(SlaveAddress slave_address, uint16_t register_address,
                       const T& value, unsigned int timeout_ms = 200) {
        return writeRegister(slave_address, RegisterAddress(register_address), value, timeout_ms);
    }

    bool readRegister(SlaveAddress slave_address, RegisterAddress register_address,
                      void* out, uint16_t len,
                      unsigned int timeout_ms = 200);

    bool readRegister(SlaveAddress slave_address, uint16_t register_address,
                      void* out, uint16_t len,
                      unsigned int timeout_ms = 200) {
        return readRegister(slave_address, RegisterAddress(register_address), out, len, timeout_ms);
    }

    template<typename T>
    bool readRegister(SlaveAddress slave_address, RegisterAddress register_address,
                      T& out, unsigned int timeout_ms = 200) {
        static_assert(std::is_trivially_copyable_v<T>, "Register reads require trivially copyable types");
        return readRegister(slave_address, register_address, &out, static_cast<uint16_t>(sizeof(T)), timeout_ms);
    }

    template<typename T>
    bool readRegister(SlaveAddress slave_address, uint16_t register_address,
                      T& out, unsigned int timeout_ms = 200) {
        return readRegister(slave_address, RegisterAddress(register_address), out, timeout_ms);
    }

    bool waitForResponseIdx(uint8_t idx, unsigned int timeout_ms,
                            RxDatagram& out);

    bool waitForResponseAdo(uint16_t ado, Command cmd,
                            unsigned int timeout_ms,
                            RxDatagram& out);

    // ---- Batch (multi-datagram) APIs ---------------------------------------

    /**
     * @brief Async handle for a batch read or write transaction.
     *
     * Returned by readRegistersBatch() / writeRegistersBatch().
     * Call getResult() to retrieve individual results or waitAll() for all.
     * Unclaimed slots are cleaned up on destruction.
     */
    class BatchTransaction {
    public:
        BatchTransaction() = default;
        BatchTransaction(TransactionRouter* router,
                         std::vector<uint8_t> idxs,
                         std::vector<size_t> slots,
                         std::vector<RxDatagram> responses);

        BatchTransaction(BatchTransaction&&) = default;
        BatchTransaction& operator=(BatchTransaction&&) = default;
        BatchTransaction(const BatchTransaction&) = delete;
        BatchTransaction& operator=(const BatchTransaction&) = delete;

        ~BatchTransaction();

        /// Get result for datagram at index i (blocks up to timeout_ms)
        BatchReadResult getResult(size_t i, uint32_t timeout_ms);

        /// Wait for all results (blocks up to timeout_ms per datagram)
        bool waitAll(uint32_t timeout_ms, std::vector<BatchReadResult>& out);

        /// Cancel any pending waits
        void cancel();

        /// Number of datagrams in this transaction
        size_t count() const { return idxs_.size(); }

    private:
        TransactionRouter* router_{nullptr};
        std::vector<uint8_t> idxs_;
        std::vector<size_t> slots_;
        std::vector<RxDatagram> responses_;
        bool cancelled_{false};
    };

    /**
     * @brief Read multiple physical addresses in one frame (async).
     *
     * Packs N APRD/FPRD datagrams into one Ethernet frame (auto-splits if
     * exceeding 1514 bytes) and sends immediately.  Returns a BatchTransaction
     * for async result retrieval.
     *
     * @param slave_addresses  Array of slave addresses (physical or logical)
     * @param register_addresses Array of register addresses
     * @param lengths          Array of expected read lengths
     * @param count            Number of reads
     * @return Batch transaction handle
     */
    BatchTransaction readRegistersBatch(
        const SlaveAddress* slave_addresses,
        const uint16_t* register_addresses,
        const uint16_t* lengths,
        size_t count);

    /**
     * @brief Write multiple physical addresses in one frame (async).
     *
     * Packs N APWR/FPWR datagrams into one Ethernet frame (auto-splits if
     * exceeding 1514 bytes) and sends immediately.  Returns a BatchTransaction
     * for async result retrieval.
     *
     * @param slave_addresses  Array of slave addresses
     * @param register_addresses Array of register addresses
     * @param data              Array of data pointers
     * @param lengths           Array of write lengths
     * @param count             Number of writes
     * @return Batch transaction handle
     */
    BatchTransaction writeRegistersBatch(
        const SlaveAddress* slave_addresses,
        const uint16_t* register_addresses,
        const void* const* data,
        const uint16_t* lengths,
        size_t count);

    /**
     * @brief Return the Working Counter of the last register read/write.
     *
     * Useful for callers that need to distinguish WKC==0 (slave did not
     * respond) from other transport failures.
     */
    uint16_t lastWkc() const { return last_wkc_; }

    // ---- Index allocation --------------------------------------------------

    static constexpr uint8_t kFireAndForgetIdx = 0xFE;

    uint8_t allocIdx();
    void    resetIdx();

    // ---- Watchdog ----------------------------------------------------------

    bool configureWatchdogs(SlaveAddress slave_address,
                            uint16_t pdi_timeout_100us,
                            uint16_t pdata_timeout_100us);
    bool disableWatchdogs(SlaveAddress slave_address);
    bool readWatchdogStatus(SlaveAddress slave_address,
                            uint8_t& wd_status,
                            uint8_t& pdi_cnt,
                            uint8_t& pdata_cnt);

    // ---- Source MAC --------------------------------------------------------

    const uint8_t* getSrcMac() const;

    // ---- SII / EEPROM ------------------------------------------------------

    bool siiReadString(uint16_t slave_index, uint16_t string_number,
                       char* out, size_t out_cap);

    /**
     * @brief Log a concise discovered-slave summary for each slave
     * 
     * This convenience helper reads SII/identity information for every
     * discovered slave and prints a short diagnostic summary. It is
     * useful for examples and diagnostics.
     */
    void logDiscoveredSlavesSummary(const char* tag = "EtherCAT");

    bool configureMailboxFromSii(uint16_t slave_index,
                                 uint16_t* out_wr_addr, uint16_t* out_wr_len,
                                 uint16_t* out_rd_addr, uint16_t* out_rd_len,
                                 uint16_t* out_mbx_proto);

    /**
     * @brief Automatically configure mailbox from SII for a slave
     * 
     * This is a high-level convenience method that:
     * 1. Reads mailbox configuration from SII EEPROM
     * 2. Applies it to the master's mailbox override
     * 3. Configures the SDO subsystem with the mailbox parameters
     * 4. Falls back to sane defaults if SII read fails
     * 
     * @param slave_index Slave index (0-based)
     * @param log_level Log level for diagnostics (Debug, Info, etc.)
     *                  Use LogLevel::Debug for verbose output
     * @return true if mailbox was configured successfully, false on critical failure
     * 
     * @note This should be called AFTER slave discovery but BEFORE attempting
     *       any SDO operations. Typical usage is right after the master starts
     *       and slaves are discovered.
     * 
     * Example:
     * @code
     *   master.start(iface, mac);
     *   // Wait for discovery...
     *   master.autoConfigureMailbox(0, Tether::Platform::LogLevel::Debug); // Verbose logging
     * @endcode
     */
    bool autoConfigureMailbox(SlaveAddress slave_address, Tether::Platform::LogLevel log_level = Tether::Platform::LogLevel::Info);

    /**
     * @brief Drain any stale data from the slave-to-master mailbox (SM1).
     *
     * Reads the SM1 status register and, if it indicates unread data, reads the
     * configured SM1 address to clear the ESC buffer toggle.  Repeats until SM1
     * is no longer full or the drain limit is reached.
     *
     * This should be called once after mailbox auto-configuration (or manual
     * configuration) and before the first SDO exchange.  It is also safe to call
     * as a diagnostic recovery step.
     *
     * @param slave_index    Slave index (0-based)
     * @param max_drain      Maximum back-to-back reads to perform (default 16)
     * @return true if the drain completed (SM1 empty or successfully drained);
     *         false if mailbox is not configured or SM1 status could not be read
     */
    bool drainSlaveMailbox(uint16_t slave_index, unsigned int max_drain = 16);

    /**
     * @brief Hardware-reset the slave-to-master mailbox (SM1) by cycling its
     *        activate register.
     *
     * This is a last-resort recovery for an ESC that reports SM1 full but
     * rejects master read datagrams (WKC == 0). Disabling and re-enabling SM1
     * flushes the internal buffer state and clears stuck full flags.
     *
     * @param slave_index    Slave index (0-based)
     * @return true if SM1 status reads back clear after the reset
     */
    bool resetSlaveMailboxSM1(uint16_t slave_index);

    /**
     * @brief Verify a slave's SII identity against expected values.
     *
     * Reads the slave's identity from SII EEPROM and compares each
     * present field in @p expected.  Nullopt fields are ignored.
     *
     * @param slave_index    Slave index (0-based)
     * @param expected       Expected identity values
     * @param exit_on_error  If true, stop the master and call std::exit(1) on mismatch
     * @param tag            ESP-style log tag
     * @return true if all checked fields match, false otherwise
     */
    bool verifySlaveIdentity(uint16_t slave_index,
                             const Identity::SlaveIdentity& expected,
                             bool exit_on_error = false,
                             const char* tag = "EtherCAT");

    // ---- Debug flags -------------------------------------------------------

    /** @brief Access the master's debug flags (read/write). */
    EtherCATMasterDebugFlags& debugFlags() { return debug_flags_; }
    const EtherCATMasterDebugFlags& debugFlags() const { return debug_flags_; }

    /**
     * @brief Recompute and push per-slave debug flags to all slaves and CoE managers.
     *
     * Call this after modifying the master debug flags, or after slave
     * discovery changes the slave count.
     */
    void updateDebugFlags();

    /** @brief Convenience: is a named debug flag enabled for a slave? */
    bool isDebugEnabled(const std::string& name, uint16_t slave_index) const {
        return debug_flags_.isEnabled(name, slave_index);
    }

    /** @brief Access the master's debug gate (for conditional debugging). */
    DebugGate& debugGate() { return *debug_gate_; }
    const DebugGate& debugGate() const { return *debug_gate_; }

    // ---- CoE / SDO low-level -----------------------------------------------

    bool coeSdoUpload(uint16_t adp, uint8_t* inout_mbx_cnt,
                      uint16_t mbx_wr_addr, uint16_t mbx_wr_len,
                      uint16_t mbx_rd_addr, uint16_t mbx_rd_len,
                      uint16_t index, uint8_t sub,
                      uint8_t* out, size_t out_cap, size_t* out_len,
                      bool diag_enabled = false,
                      unsigned int poll_interval_ms = 5,
                      unsigned int transaction_timeout_ms = 1000);

    bool coeSdoDownload(uint16_t adp, uint8_t* inout_mbx_cnt,
                        uint16_t mbx_wr_addr, uint16_t mbx_wr_len,
                        uint16_t mbx_rd_addr, uint16_t mbx_rd_len,
                        uint16_t index, uint8_t sub,
                        const uint8_t* data, size_t data_len,
                        bool diag_enabled = false,
                        unsigned int poll_interval_ms = 5,
                        unsigned int transaction_timeout_ms = 1000);

    /// @brief Return the CoE SDO abort code reported by the slave on the most
    /// recent coeSdoUpload/coeSdoDownload call. 0 means no abort (success or
    /// a non-abort failure such as a transport/timeout error). Read this
    /// immediately after a call returns false to distinguish a definitive
    /// slave rejection (e.g. 0x06070010 length mismatch) from a transport
    /// issue.
    uint32_t lastCoeSdoAbortCode() const;

    // ---- Utilities (stateless) ---------------------------------------------

    static constexpr PhysicalAddress physicalAddressForSlaveIndex(uint16_t slave_index) {
        return PhysicalAddress(slave_index);
    }

    static uint16_t adpForSlaveIndex(uint16_t slave_index) {
        return physicalAddressForSlaveIndex(slave_index).raw();
    }

    static constexpr SlaveAddress slaveAddressFromADP(uint16_t adp) {
        return SlaveAddress((adp == 0x0000) ? 0 : static_cast<uint16_t>(0u - adp));
    }

    static const char* getECStateName(uint8_t state);

    static bool resolvePhysicalSlaveIndex(SlaveAddress slave_address, uint16_t& slave_index_out);

    // ---- Sub-managers ------------------------------------------------------

    PDOManager&     pdo();
    LogicalAddressManager& logicalAddressManager();
    ::EtherCAT::CoE::CoEManager& sdoManager(uint16_t slave_index);
    DCManager&      dc();
    FoEManager&     foe();
    VoEManager&     voe();
    EoEManager&     eoe();
    FaultDetector&     faults();
    SlaveStatusPoller&  statusPoller();
private:
    // Mailbox override storage — guarded by a mutex and sized to kMaxPDOSlaves
    struct MailboxOverrideInternal {
        bool enabled{false};
        uint16_t wr_addr{0};
        uint16_t wr_len{0};
        uint16_t rd_addr{0};
        uint16_t rd_len{0};
        uint16_t proto{0};
    };

    std::mutex m_mailbox_override_mutex_;
    std::vector<MailboxOverrideInternal> m_mailbox_overrides_;

public:
    // Public destructor/cleanup follows...
    ConditionalPacketRouter& packetRouter();

    // ---- Internal queue access (used by sub-managers) -----------------------

    Tether::Platform::MessageQueue<RxDatagram>* rxQueue();
    Tether::Platform::MessageQueue<RxDatagram>* txpdoRxQueue();

    // ---- Test hooks --------------------------------------------------------

    using AprdTestCb = std::function<bool(uint16_t adp, uint16_t ado,
                                          void* out, uint16_t len,
                                          unsigned int timeout_ms)>;
    using ApwrTestCb = std::function<bool(uint16_t adp, uint16_t ado,
                                          const void* data, uint16_t len,
                                          unsigned int timeout_ms)>;

    void setAprdTestCallback(AprdTestCb cb);
    void setApwrTestCallback(ApwrTestCb cb);
    void pushAprdResponse(bool success, uint16_t adp, uint16_t ado,
                          const void* data, uint16_t len);
    void clearAprdResponses();

    const NetworkInterface* networkInterface() const;

    /**
     * @brief Test helper: check if a one-time fault diagnosis was issued for a slave
     * 
     * Used by unit tests to verify that `fault_diagnose()` was called when
     * AL_STATUS error conditions were observed.
     */
    bool wasFaultDiagnosed(uint16_t slave_index) const;

    // ---- Statistics --------------------------------------------------------

#if TETHER_ENABLE_ETHERCAT_STATS
    struct Stats {
        uint32_t tx_retry_count  = 0;
        uint32_t tx_fail_count   = 0;
        uint32_t rx_frame_count  = 0;
        uint32_t rx_queue_sent   = 0;
        uint32_t rx_flushed      = 0;
        uint32_t flush_calls     = 0;
    };
    Stats getStats() const;
#endif

private:
    // ---- Internal helpers --------------------------------------------------
    bool setPreopAndConfirm(uint16_t slave_index);
    void ensureRxQueues();
    void flushRxQueue();
    void parseEtherCATFrame(const uint8_t* frame, size_t length);

    // ---- EtherCAT-over-UDP encapsulation ----------------------------------
#if TETHER_ENABLE_UDP_ENCAPSULATION
    /// Compute the IPv4 header checksum (ones-complement sum over 20-byte header).
    static uint16_t computeIpChecksum(const uint8_t* ip_header);
    /// Transform a direct Ethernet/EtherCAT frame into Ethernet/IPv4/UDP/EtherCAT.
    /// @param in_frame  Pointer to the original frame (Ethernet + EtherCAT, EtherType 0x88A4).
    /// @param in_len    Length of the original frame.
    /// @param out_buf   Output buffer (must be at least in_len + kUdpEncapOverhead bytes).
    /// @param out_cap   Capacity of out_buf.
    /// @param[out] out_len  Actual length of the encapsulated frame.
    /// @return true on success, false if the frame is too short or output buffer too small.
    bool encapsulateFrame(const uint8_t* in_frame, size_t in_len,
                          uint8_t* out_buf, size_t out_cap, size_t* out_len) const;
    /// Send a frame via iface_.send(), applying UDP encapsulation if enabled.
    bool sendWithEncapsulation(const uint8_t* frame, size_t len);
    /// @return max EtherCAT payload bytes per frame, accounting for UDP overhead.
    size_t maxEtherCATPayloadPerFrame() const;
#else
    /// Without UDP encapsulation, sendWithEncapsulation is a trivial passthrough.
    bool sendWithEncapsulation(const uint8_t* frame, size_t len) {
        return iface_.send ? iface_.send(frame, len) : false;
    }
#endif

    size_t    preRegisterResponseWaiter(uint8_t idx, uint8_t* buffer,
                                        size_t buffer_size);
    WaitResult waitForPreRegistered(size_t slot, uint32_t timeout_ms);

    // ---- Data members ------------------------------------------------------

    Config config_;

    // Network
    NetworkInterface iface_{};
    uint8_t src_mac_[6] = {};

    // Queues
    std::unique_ptr<Tether::Platform::MessageQueue<RxDatagram>> rx_queue_;
    std::unique_ptr<Tether::Platform::MessageQueue<RxDatagram>> txpdo_rx_queue_;

    // Packet router (TransactionRouter — race-free, idx-indexed)
    TransactionRouter packet_router_;

    // Index allocator
    std::atomic<uint8_t> next_idx_{0};

#if TETHER_ENABLE_UDP_ENCAPSULATION
    // IP identification counter for UDP encapsulation
    mutable std::atomic<uint16_t> ip_id_counter_{0};
#endif

    // Master state
    std::atomic<bool>     running_{false};
    std::atomic<bool>     cancel_requested_{false};
    std::atomic<uint16_t> discovered_slave_count_{0};
    MotionControlCallback motion_control_callback_;
    std::unique_ptr<IMotionControlLoop> motion_control_loop_;

    // Last working counter from a real bus transaction
    std::atomic<uint16_t> last_wkc_{0};

    // Test hooks
    AprdTestCb aprd_cb_;
    ApwrTestCb apwr_cb_;
    struct AprdResponse {
        bool     success{true};
        uint16_t adp{0};
        uint16_t ado{0};
        std::vector<uint8_t> data;
    };
    std::deque<AprdResponse> aprd_responses_;

    // Optional test callback invoked when mailbox fallback is triggered
    std::function<void(uint16_t)> mailbox_fallback_cb_;


    // Statistics
#if TETHER_ENABLE_ETHERCAT_STATS
    std::atomic<uint32_t> tx_retry_count_{0};
    std::atomic<uint32_t> tx_fail_count_{0};
    std::atomic<uint32_t> rx_frame_count_{0};
    std::atomic<uint32_t> rx_queue_sent_{0};
    std::atomic<uint32_t> total_flushed_{0};
    std::atomic<uint32_t> flush_calls_{0};
#endif

    // Log dedup / rate limiting
    Tether::Logging::DeduplicatingLogger send_fail_log_{
        "NetworkInterface::send failed after retries",
        Tether::Logging::DedupLogConfig{2, 10'000, true, Tether::Platform::LogLevel::Info}
    };

    // RX-path unrouted/dropped-packet log throttling counters (per-instance;
    // formerly function-local statics shared across all Master instances).
    uint32_t unrouted_log_count_ = 0;
    uint32_t rx_drop_log_count_  = 0;

    // Diagnostics: one-time per-slave fault diagnostic tracker
    mutable std::mutex m_diag_mutex_;
    std::unordered_set<uint16_t> m_diagnosed_slaves_;

    // Instance-based SDO manager (new approach)
    class MasterSDOTransport;
    std::unique_ptr<::EtherCAT::SDO::ISDOTransport> sdo_transport_;
    std::vector<std::unique_ptr<::EtherCAT::CoE::CoEManager>> sdo_managers_;
    mutable std::mutex sdo_managers_mutex_;

    // CoE SDO mailbox channel (refactored from free functions)
    std::unique_ptr<::EtherCAT::Raw::CoeSDOChannel> coe_sdo_channel_;

    // Sub-managers (legacy wrappers)
    std::unique_ptr<IPDOTransport> pdo_transport_;
    std::unique_ptr<PDOManager>    pdo_;
    std::unique_ptr<LogicalAddressManager> logical_addr_mgr_;
    std::unique_ptr<DCManager>     dc_;
    std::unique_ptr<FoEManager>    foe_;
    std::unique_ptr<VoEManager>    voe_;
    std::unique_ptr<EoEManager>    eoe_;
    std::unique_ptr<IFaultTransport>    fault_transport_;
    std::unique_ptr<FaultDetector>      faults_;
    std::unique_ptr<SlaveStatusPoller>  status_poller_;

    // Per-slave state machines
    std::vector<std::unique_ptr<Slave>> slaves_;
    std::unique_ptr<NonExistingSlave> non_existing_slave_;

    // Debug flags (master-level with per-slave filtering)
    EtherCATMasterDebugFlags debug_flags_;

    // Debug gate (conditional debug activation)
    std::unique_ptr<DebugGate> debug_gate_;

    // SII reader (lazily created)
    std::unique_ptr<SII::SIIReader> sii_reader_;

    // Per-slave EEPROM word cache (indexed by slave_index -> word_addr -> value)
    // Protected by sii_cache_mutex_ since the discovery thread writes via
    // setSIICachedWord while client threads read via getSIICachedWord.
    std::vector<std::unordered_map<uint16_t, uint16_t>> sii_word_caches_;
    mutable std::mutex sii_cache_mutex_;
};

// ============================================================================
// PDOManager
// ============================================================================

} // namespace EtherCAT
