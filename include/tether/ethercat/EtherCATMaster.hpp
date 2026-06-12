#pragma once

/**
 * @file EtherCATMaster.hpp
 * @brief EtherCAT Master — class-based API for multi-instance support
 *
 * @details
 * EtherCATMaster encapsulates all state required to run an independent
 * EtherCAT master instance.  Multiple masters can coexist in the same
 * process, each driving a separate Ethernet interface.
 *
 * ## Quick start
 * @code
 *   EtherCAT::EtherCATMaster master;
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
#include <type_traits>
#include <vector>
#include <unordered_set>

#include "logging/DeduplicatingLogger.hpp"

#include "tether/ethercat/EtherCATConfig.hpp"
#include "tether/ethercat/EtherCATTypes.hpp"
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
class DCManager;
class FoEManager;
class VoEManager;
class EoEManager;
class FaultDetector;
class IFaultTransport;
class EtherCATSlave;
class NonExistingSlave;

namespace SII {
    class SIIReader;
}

// Forward declarations for SDO namespace
namespace SDO {
    class ISDOTransport;
    class SDOManager;
}

class IMotionControlLoop;

// ============================================================================
// EtherCATMaster
// ============================================================================

/**
 * @brief Owns all state for one EtherCAT master instance.
 *
 * Each instance manages its own network interface, packet routing,
 * slave discovery, and sub-manager objects (PDO, SDO, DC, …).
 * The class is **non-copyable** but can be moved.
 */
class EtherCATMaster {
public:
    /// Configuration knobs passed at construction time.
    struct Config {
        uint32_t rx_queue_depth     = 64;
        uint32_t txpdo_queue_depth  = 8;
        bool enable_mailbox_fallback = false; ///< Opt-in: force default mailbox on InvalidMailboxConfig
    };

    /** Enable or disable the mailbox fallback at runtime. */
    void setEnableMailboxFallback(bool enabled) { config_.enable_mailbox_fallback = enabled; }
    bool isMailboxFallbackEnabled() const { return config_.enable_mailbox_fallback; }

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

    EtherCATMaster();
    explicit EtherCATMaster(const Config& config);
    ~EtherCATMaster();

    EtherCATMaster(const EtherCATMaster&)            = delete;
    EtherCATMaster& operator=(const EtherCATMaster&) = delete;
    EtherCATMaster(EtherCATMaster&&)                 = delete;
    EtherCATMaster& operator=(EtherCATMaster&&)      = delete;

    // ---- Lifecycle ---------------------------------------------------------

    /** Start master task using hardware-independent NetworkInterface. */
    void start(const NetworkInterface& iface, const uint8_t src_mac[6]);

    /** Stop the master task and shut down all sub-systems. */
    void stop();

    /** @return true while the master task is running. */
    bool isRunning() const;

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
     * Returns a reference to the EtherCATSlave at the given bus position.
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
    EtherCATSlave& slave(uint16_t slave_index);

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

    // ---- CoE / SDO low-level -----------------------------------------------

    bool coeSdoUpload(uint16_t adp, uint8_t* inout_mbx_cnt,
                      uint16_t mbx_wr_addr, uint16_t mbx_wr_len,
                      uint16_t mbx_rd_addr, uint16_t mbx_rd_len,
                      uint16_t index, uint8_t sub,
                      uint8_t* out, size_t out_cap, size_t* out_len,
                      bool diag_enabled = false);

    bool coeSdoDownload(uint16_t adp, uint8_t* inout_mbx_cnt,
                        uint16_t mbx_wr_addr, uint16_t mbx_wr_len,
                        uint16_t mbx_rd_addr, uint16_t mbx_rd_len,
                        uint16_t index, uint8_t sub,
                        const uint8_t* data, size_t data_len,
                        bool diag_enabled = false);

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
    ::EtherCAT::SDO::SDOManager& sdoManager();
    DCManager&      dc();
    FoEManager&     foe();
    VoEManager&     voe();
    EoEManager&     eoe();
    FaultDetector&  faults();
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
     * @brief Find a running EtherCATMaster by its NetworkInterface pointer.
     *
     * This helper is used by host-side transport helpers (examples) to locate
     * the master instance that was started with a given NetworkInterface.
     * Returns nullptr if no matching instance is registered.
     */
    static EtherCATMaster* findByNetworkInterface(const NetworkInterface* iface);

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

    size_t    preRegisterResponseWaiter(uint8_t idx, uint8_t* buffer,
                                        size_t buffer_size);
    WaitResult waitForPreRegistered(size_t slot, uint32_t timeout_ms);

    // ---- Data members ------------------------------------------------------

    Config config_;

    // Network
    NetworkInterface iface_{};
    const NetworkInterface* iface_ptr_{nullptr}; ///< Original pointer for identity comparison
    uint8_t src_mac_[6] = {};

    // Queues
    std::unique_ptr<Tether::Platform::MessageQueue<RxDatagram>> rx_queue_;
    std::unique_ptr<Tether::Platform::MessageQueue<RxDatagram>> txpdo_rx_queue_;

    // Packet router (TransactionRouter — race-free, idx-indexed)
    TransactionRouter packet_router_;

    // Index allocator
    std::atomic<uint8_t> next_idx_{0};

    // Master state
    std::atomic<bool>     running_{false};
    std::atomic<uint16_t> discovered_slave_count_{0};
    MotionControlCallback motion_control_callback_;
    std::unique_ptr<IMotionControlLoop> motion_control_loop_;

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
    uint32_t rx_frame_count_ = 0;
    uint32_t rx_queue_sent_  = 0;
    uint32_t total_flushed_  = 0;
    uint32_t flush_calls_    = 0;
#endif

    // Log dedup / rate limiting
    Tether::Logging::DeduplicatingLogger send_fail_log_{
        "NetworkInterface::send failed after retries",
        Tether::Logging::DedupLogConfig{2, 10'000, true, Tether::Platform::LogLevel::Info}
    };

    // Diagnostics: one-time per-slave fault diagnostic tracker
    mutable std::mutex m_diag_mutex_;
    std::unordered_set<uint16_t> m_diagnosed_slaves_;

    // Instance-based SDO manager (new approach)
    class MasterSDOTransport;
    std::unique_ptr<::EtherCAT::SDO::ISDOTransport> sdo_transport_;
    std::unique_ptr<::EtherCAT::SDO::SDOManager>    sdo_manager_;

    // Sub-managers (legacy wrappers)
    std::unique_ptr<IPDOTransport> pdo_transport_;
    std::unique_ptr<PDOManager>    pdo_;
    std::unique_ptr<DCManager>     dc_;
    std::unique_ptr<FoEManager>    foe_;
    std::unique_ptr<VoEManager>    voe_;
    std::unique_ptr<EoEManager>    eoe_;
    std::unique_ptr<IFaultTransport> fault_transport_;
    std::unique_ptr<FaultDetector> faults_;

    // Per-slave state machines
    std::vector<std::unique_ptr<EtherCATSlave>> slaves_;
    std::unique_ptr<NonExistingSlave> non_existing_slave_;

    // SII reader (lazily created)
    std::unique_ptr<SII::SIIReader> sii_reader_;
};

// ============================================================================
// PDOManager
// ============================================================================

} // namespace EtherCAT
