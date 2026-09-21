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
#include <string>
#include <string_view>
#include <type_traits>
#include <array>
#include <vector>
#include <unordered_map>
#include <unordered_set>

#include "logging/DeduplicatingLogger.hpp"

#include "tether/ethercat/DebugFlags.hpp"
#include "tether/ethercat/DebugGate.hpp"
#include "tether/ethercat/CyclicChannel.hpp"
#include "tether/ethercat/CyclicExecutive.hpp"
#include "tether/ethercat/AsyncCyclicLoop.hpp"
#include "tether/ethercat/ProcessImage.hpp"
#include "tether/ethercat/EtherCATTransport.hpp"
#include "tether/ethercat/SlaveIdentity.hpp"
#include "tether/ethercat/EtherCATConfig.hpp"
#if TETHER_ENABLE_SII
#include "tether/sii/SIIManager.hpp"
#endif
#include "tether/ethercat/SlaveDiscoveryManager.hpp"
#include "tether/ethercat/DC.hpp"
#include "tether/ethercat/Types.hpp"
#include "tether/ethercat/TransactionRouter.hpp"
#include "tether/ethercat/ESIFile.hpp"
#include "tether/platform/MessageQueue.hpp"
#include "tether/platform/EspCompat.hpp"

namespace EtherCAT {

class SlaveDiscoveryManager;  // forward declaration (see SlaveDiscoveryManager.hpp)

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
class SlaveSupervisor;
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
class CyclicDatapath;
class SlaveRegistry;

class Master;

/// Factory for the private Master::MasterSDOTransport adapter
/// (defined in src/ethercat/raw/MasterTransports.cpp).  Friended below so
/// it can name the private nested class.
std::unique_ptr<SDO::ISDOTransport> makeMasterSDOTransport(Master& master);

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

        /// Maximum Ethernet frame size excluding FCS (default 1514).
        /// Raise for jumbo links to carry larger LRW slices per frame —
        /// a single datagram still caps at 2047 B (11-bit length field),
        /// but jumbo frames also pack more datagrams per frame.
        /// Requires NIC MTU and slave support.  Clamped to [1514, 9014].
        uint32_t max_frame_size = 1514;

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
        using UdpEncapsulation = EtherCAT::UdpEncapsulationConfig;
        UdpEncapsulation udp_encapsulation;
#endif
    };

    /** Enable or disable the mailbox fallback at runtime. */
    void setEnableMailboxFallback(bool enabled) { config_.enable_mailbox_fallback = enabled; }
    bool isMailboxFallbackEnabled() const { return config_.enable_mailbox_fallback; }

    /**
     * @brief Configure EtherCAT-over-UDP encapsulation (ETG.1000.3).
     *
     * Safe to call before start() — the transport reads the live config
     * through a pointer captured at construction.  No-op when built with
     * TETHER_ENABLE_UDP_ENCAPSULATION=0; verify with
     * isUdpEncapsulationEnabled().
     */
    void setUdpEncapsulation(const UdpEncapsulationConfig& cfg) {
#if TETHER_ENABLE_UDP_ENCAPSULATION
        config_.udp_encapsulation = cfg;
#else
        (void)cfg;
#endif
    }

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

    // ---- Cyclic executive (deadline-driven fast loop) ---------------------
    //
    // Single cyclic thread that sleeps directly on an absolute deadline
    // (clock_nanosleep TIMER_ABSTIME on Linux — no timer-thread -> event ->
    // worker hop) and drives the phase pipeline:
    //   [inline DC] -> PreExchange -> LRW exchange -> PostExchange -> Motion
    //
    // Uses the reserved-index cyclic fast path for the PDO exchange when the
    // transport supports it (responses deposited into fixed slots — no
    // TransactionRouter round-trip).
    /// Which CPUs to claim for the RT threads (opt-in runtime isolation).
    struct CpuIsolationConfig {
        bool enabled      = false;
        int  cyclic_cpu   = -1;    ///< >=0: pin cyclic thread to this CPU
        int  dc_cpu       = -1;    ///< >=0: pin DC thread to this CPU
        bool prefer_isolated = true;  ///< prefer /sys-isolated CPUs
        bool avoid_cpu0      = true;  ///< CPU0 handles most default IRQs
        /// Q8 (root, opt-in): create a cgroup2 cpuset partition covering
        /// the claimed CPUs — real isolation without isolcpus=.  Best-
        /// effort: failure degrades to the plain affinity pin + a warning.
        bool create_cpuset = false;
        /// Q9 (root, opt-in): steer /proc/irq/*/smp_affinity_list off the
        /// claimed CPUs; original masks restored on releaseAll().
        bool steer_irqs    = false;
    };

    /// Granular memory locking — each flag is an independent opt-out.
    /// Locking is best-effort: failures are logged, never fatal.
    struct MemoryLockConfig {
        bool lock_all_process = false;  ///< mlockall(CURRENT|FUTURE) — most
                                        ///< thorough, least selective
        bool lock_image       = true;   ///< ProcessImage buffers
        bool lock_slots       = true;   ///< cyclic slot bank + TX staging
        bool prefault_stack   = true;   ///< pre-touch cyclic thread stack
        uint32_t stack_prefault_bytes = 0;  ///< 0 → conservative default
    };

    /// Where in the phase pipeline the exchange's collect half runs when
    /// split_exchange is set.  Atomic keeps send+collect in one step.
    enum class ExchangePlacement : uint8_t {
        Atomic,   ///< send+wait in the Exchange phase (default)
        Split,    ///< send at Exchange; collect at PostExchange — the wire
                  ///< round-trip overlaps the work in between
        SplitLate ///< send at Exchange; collect at Diagnostics — maximum
                  ///< overlap; only valid when nothing in-loop reads the
                  ///< fresh inputs (external motion source use case)
    };

    struct CyclicLoopConfig {
        uint32_t cycle_period_us{1000};
        bool     enable_dc_synchronization{false};
        uint32_t sync_interval_cycles{10};
        /// Run the registered MotionControlCallback in the MotionControl
        /// phase of the cyclic thread.  If false, motion is left to an
        /// external source/thread (external-motion use case).
        bool     motion_in_loop{true};
        /// Threading/timing options — priority, affinity, sleep mode,
        /// DC placement (dedicated thread / inline / disabled).
        CyclicExecutive::Config exec{};

        /// Wire datapath: Auto = packet ring when supported, socket
        /// sendmsg/recv bank otherwise (loud log on fallback).
        /// Unavailable platforms/capabilities degrade to the software
        /// deposit path — correctness is never affected.
        CyclicWireMode wire_mode{CyclicWireMode::Auto};

        /// Process-image exposure mode (see ProcessImage.hpp).
        /// Buffered keeps the per-entry storage[] exchange.
        ImageMode image_mode{ImageMode::Buffered};

        /// Exchange phase split — Split* overlaps the wire round-trip
        /// with intermediate phases (see ExchangePlacement).
        ExchangePlacement exchange_placement{ExchangePlacement::Atomic};

        /// RX spin window (ns) inside the channel's own rxPoll() — the
        /// kernel-ring busy-poll.  0 disables.
        uint32_t rx_spin_ns{0};

        /// Slot-wait spin window (ns): before blocking in ppoll, the
        /// cyclic slot wait busy-polls the slot seq + channel rxPending()
        /// (pure memory reads — ring DMA writes are visible without a
        /// syscall).  Shares the response deadline budget.  0 disables.
        uint32_t slot_spin_ns{0};

        /// How the cyclic slot wait behaves when no fd wake-up path
        /// exists (non-Linux / no eventfd / no wire fd): a tight seq
        /// re-check loop (Spin — lowest wake latency, burns CPU) or a
        /// sched_yield() per iteration (Yield — default; a dead link
        /// can't wedge the scheduler).
        enum class SlotWaitFallback : uint8_t { Yield, Spin };
        SlotWaitFallback slot_wait_fallback{SlotWaitFallback::Yield};

        /// Split-exchange collect phase (Split only): the user picks
        /// where the collect half lands in the phase pipeline — any
        /// phase after Exchange is valid (PostExchange default;
        /// MotionControl/Diagnostics for deeper overlap).  SplitLate
        /// still means Diagnostics.  PreExchange/Exchange are rejected
        /// with a warning and clamped to PostExchange.
        TaskPhase collect_phase{TaskPhase::PostExchange};

        /// Runtime CPU claims for the cyclic/DC threads (opt-in).
        CpuIsolationConfig cpu_isolation{};

        /// Memory locking sections — each flag independently opt-out-able.
        MemoryLockConfig memory_lock{};

        /// Optional shared-memory export of the process image for a
        /// process-external motion source (shm_open name, e.g.
        /// "tether-img").  Empty = in-process only.
        std::string shm_image_name{};

        /// When true (default), a response WKC that differs from the
        /// learned expected value is counted as an error — catches partial
        /// slave dropout, which wkc==0 alone cannot see.
        bool strict_wkc{true};

        /// Optional DC configuration.  When set, startCyclicLoop()
        /// initializes distributed clocks itself (dc().init) unless DC is
        /// already initialized, and implies enable_dc_synchronization —
        /// the executive's dedicated DC task then emits sync frames every
        /// sync_interval_cycles.  Setting this removes the
        /// "initializeDistributedClocks() before start" ordering trap.
        /// NOTE: startDistributedClocks()/dc().start() must NOT be used
        /// with the cyclic loop — that legacy realtime loop owns a second
        /// PDO exchange and is stopped on start.
        std::optional<DC::DCConfig> dc_config{};

        /// Conservative low-latency preset: Split exchange placement (the
        /// wire round-trip overlaps the phases between send and collect),
        /// Auto wire mode (kernel ring when available), runtime CPU
        /// isolation opt-in, default granular memory locking.  Everything
        /// degrades gracefully on hosts without the capabilities.
        static CyclicLoopConfig lowLatency(uint32_t cycle_period_us = 1000) {
            CyclicLoopConfig c;
            c.cycle_period_us     = cycle_period_us;
            c.exchange_placement  = ExchangePlacement::Split;
            c.cpu_isolation.enabled = true;
            return c;
        }
    };

    /**
     * @brief RAII handle for a running cyclic loop.
     *
     * Returned by startCyclicLoopScoped(): engaged when the loop started,
     * empty on failure.  The destructor calls stopCyclicLoop() — the loop
     * cannot be left running by an early return or exception.  Move-only.
     */
    class CyclicLoopGuard {
    public:
        CyclicLoopGuard() = default;
        ~CyclicLoopGuard() { stop(); }
        CyclicLoopGuard(const CyclicLoopGuard&) = delete;
        CyclicLoopGuard& operator=(const CyclicLoopGuard&) = delete;
        CyclicLoopGuard(CyclicLoopGuard&& o) noexcept
            : master_(o.master_) { o.master_ = nullptr; }
        CyclicLoopGuard& operator=(CyclicLoopGuard&& o) noexcept {
            if (this != &o) {
                stop();
                master_ = o.master_;
                o.master_ = nullptr;
            }
            return *this;
        }
        /// true while the guard owns a running loop.
        explicit operator bool() const { return master_ != nullptr; }
        /// Stop early; safe to call explicitly (destructor then no-ops).
        void stop() {
            if (master_) { master_->stopCyclicLoop(); master_ = nullptr; }
        }
    private:
        friend class Master;
        explicit CyclicLoopGuard(Master* m) : master_(m) {}
        Master* master_ = nullptr;
    };

    /**
     * @brief Async RT loop — send-on-change for externally-clocked RxPDO
     *        producers.
     *
     * Unlike the cyclic loop there is no free-running exchange deadline:
     * the producer calls processImage().commitOutputs() then
     * triggerSend(), and the loop wires that edge to cyclicSend().  TxPDO
     * collection is either per-send (CollectMode::OnSend) or on its own
     * tick (CollectMode::Periodic).  Mutually exclusive with the cyclic
     * loop — starting one stops the other.
     */
    struct AsyncLoopConfig {
        /// TxPDO collection policy (see AsyncCyclicLoop::CollectMode).
        AsyncCyclicLoop::CollectMode collect_mode{
            AsyncCyclicLoop::CollectMode::OnSend};
        /// Periodic collect tick (µs) — CollectMode::Periodic only.
        uint32_t collect_period_us{1000};
        /// Trigger rate limiter (ns): closer triggers coalesce into one
        /// trailing send carrying the latest committed image.  0 = a send
        /// per trigger.
        uint32_t min_send_interval_ns{0};
        /// OnSend only: collect at least this often while no triggers
        /// arrive — a wire keep-alive for slave watchdogs.  0 = off.
        uint32_t max_idle_ns{0};
        /// Wire wait budget per send (ns).
        uint32_t rx_timeout_ns{200'000};

        /// DC synchronisation as an independent RT thread (Q23): emits
        /// sendSyncFrame() every dc_interval_us on its own deadline,
        /// decoupled from the trigger/send path.  Requires the master to
        /// be DC-enabled.
        bool     enable_dc_synchronization{false};
        uint32_t dc_interval_us{10'000};   ///< 10 ms default DC tick

        /// Threading/timing options — priority, affinity, sched class,
        /// stack prefault, timer slack.
        AsyncCyclicLoop::Config exec{};

        /// Same datapath knobs as CyclicLoopConfig.
        CyclicWireMode wire_mode{CyclicWireMode::Auto};
        ImageMode      image_mode{ImageMode::Buffered};
        uint32_t       rx_spin_ns{0};
        uint32_t       slot_spin_ns{0};
        CyclicLoopConfig::SlotWaitFallback slot_wait_fallback{
            CyclicLoopConfig::SlotWaitFallback::Yield};
        std::string    shm_image_name{};
        bool           strict_wkc{true};

        CpuIsolationConfig cpu_isolation{};
        MemoryLockConfig   memory_lock{};
    };

    /// Process image — valid once the cyclic loop is running with an
    /// image_mode other than Buffered.  Safe to cache the reference;
    /// epoch() changes on re-configuration.
    ProcessImage&       processImage();
    const ProcessImage& processImage() const;
    bool startCyclicLoop() { return startCyclicLoop(CyclicLoopConfig{}); }
    bool startCyclicLoop(const CyclicLoopConfig& config);
    /**
     * @brief RAII variant — returns an engaged CyclicLoopGuard on success
     *        (loop stops when it goes out of scope), empty guard on failure.
     */
    [[nodiscard]] CyclicLoopGuard startCyclicLoopScoped(
        const CyclicLoopConfig& config) {
        return startCyclicLoop(config) ? CyclicLoopGuard(this)
                                       : CyclicLoopGuard{};
    }
    void stopCyclicLoop();
    bool isCyclicLoopRunning() const;
    CyclicExecutive::Stats getCyclicLoopStats() const;

    /**
     * @brief Quiesce the wire exchange for mid-loop mapping mutation.
     *
     * Slave recovery re-registers PDO entries while a cyclic or async
     * loop iterates the mapping.  This suspends the exchange/collect
     * tasks and blocks until the loop thread has passed a quiesce point
     * (no send or collect in flight), or the timeout elapses.  Returns
     * true when the exchange is safely suspended — or immediately when
     * no loop is running.  On timeout the suspension is released and
     * false is returned (the caller may still proceed — the mapping
     * epoch guard degrades a collision to a skipped cycle).
     *
     * NOT reentrant — every suspend must be paired with exactly one
     * resumeCyclicExchange().  Suspension does not survive a loop
     * restart (start clears the flag).
     */
    bool suspendCyclicExchange(uint32_t timeout_us = 20'000);
    void resumeCyclicExchange();
    /// True while the exchange is suspended by suspendCyclicExchange().
    bool cyclicExchangeSuspended() const;

    /// Async send-on-change loop — see AsyncLoopConfig.  Explicitly
    /// separate from startCyclicLoop: the user chooses the loop model.
    bool startAsyncLoop() { return startAsyncLoop(AsyncLoopConfig{}); }
    bool startAsyncLoop(const AsyncLoopConfig& config);
    void stopAsyncLoop();
    bool isAsyncLoopRunning() const;
    AsyncCyclicLoop::Stats getAsyncLoopStats() const;

    // ---- Frame handling ----------------------------------------------------

    /** Route a received Ethernet frame to the internal parser. */
    void handleRxFrame(const uint8_t* frame, size_t length);

    // ---- Discovery ---------------------------------------------------------

    /**
     * @brief Access the slave discovery manager.
     *
     * The manager is owned by the master and created lazily on first access.
     * It provides synchronous and asynchronous discovery of slaves on the
     * bus, with selectable options for what information to read per slave.
     *
     * @code
     *   // Discover everything (blocking)
     *   auto slaves = master.discovery().discover();
     *
     *   // Discover only vendor and product IDs
     *   auto slaves = master.discovery().discover({
     *       DiscoveryOption::VendorId, DiscoveryOption::ProductCode});
     *
     *   // Discover one slave
     *   auto slave = master.discovery().discoverOne(3);
     *
     *   // Async discovery
     *   auto future = master.discovery().discoverAsync();
     *   auto slaves = future.get();
     * @endcode
     *
     * @return Reference to the master's SlaveDiscoveryManager
     */
    SlaveDiscoveryManager& discovery();

    /** @brief Number of slaves discovered by the last BRD scan. */
    uint16_t getDiscoveredSlaveCount() const;

    /**
     * @brief Fast BRD scan returning the number of slaves on the bus.
     *
     * This only sends a broadcast-read AL_STATUS datagram and counts the
     * working counter.  It does not read any slave attributes (vendor/product
     * ID, SII, etc.) and does not create Slave objects, so it is much faster
     * than discovery().discover() or discoverSlaves().
     */
    uint16_t discoverSlaveCount();

    /** @brief Quick check: at least one slave is present on the bus. */
    bool hasAnySlaves();

    /** @brief Quick check: at least @p n slaves are present on the bus. */
    bool hasAtLeastNSlaves(uint16_t n);

    /** @brief Quick check: exactly @p n slaves are present on the bus. */
    bool hasExactlyNSlaves(uint16_t n);

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

    // ---- Slave names --------------------------------------------------------

    /**
     * @brief Assign a human-readable name to a slave.
     *
     * The name is used in log messages instead of the bare index.
     * If the index is out of range, a warning is logged and the call is ignored.
     *
     * @param idx   Slave index (0-based)
     * @param name  Human-readable name (e.g. "X-Axis", "Extruder-1")
     */
    void setSlaveName(uint16_t idx, std::string name);

    /**
     * @brief Get the human-readable name assigned to a slave.
     * @return The name, or an empty string_view if no name is set.
     */
    std::string_view slaveName(uint16_t idx) const;

    /**
     * @brief Build a log prefix for a slave.
     *
     * Returns "Slave <name> (#<index>)" if a name is set,
     * or "Slave <index>" if no name is set.
     */
    std::string slaveLogPrefix(uint16_t idx) const;

    /**
     * @brief Initialise the slave vector after discovery.
     *
     * Called automatically when the master discovers slaves.  Can also be
     * called manually for testing.
     *
     * @param count  Number of slaves on the bus
     */
    void initSlaves(uint16_t count);

    // ---- SII EEPROM access -------------------------------------------------

#if TETHER_ENABLE_SII
    /**
     * @brief Access the per-slave SII manager for a discovered slave.
     *
     * The SII manager is created during `initSlaves()` and is owned by the
     * `Slave` object. Accessing it before discovery is safe: it returns a
     * manager that will fail reads until the slave has been discovered.
     *
     * @code
     *   #if TETHER_ENABLE_SII
     *   uint16_t vendor = 0;
     *   if (master.slave(0).sii().readWord(0x0008, vendor)) { ... }
     *   #endif
     * @endcode
     */
    SII::SIIManager& sii(uint16_t slave_index);

    /** @brief Default timeout (ms) used for per-slave SII bus reads. */
    uint32_t siiTimeoutMs() const { return sii_timeout_ms_; }

    /** @brief Set the default timeout for per-slave SII bus reads. */
    void setSiiTimeoutMs(uint32_t timeout_ms) { sii_timeout_ms_ = timeout_ms; }
#endif

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

    /**
     * @brief Configure SM2/SM3 (process data SMs) from an ESI file
     *
     * Same as configureProcessDataSyncManagersFromSii() but reads
     * sync-manager settings from the ESI XML file instead of SII EEPROM.
     * The slave's SII identity is used to match the correct device entry
     * in the ESI file; if no match is found, the first device is used.
     *
     * @param esi           ESI file with parsed device info
     * @param slave_address Slave address
     * @return true if SM2/SM3 were configured and written successfully
     */
    bool configureProcessDataSyncManagersFromESI(const ESIFile& esi, SlaveAddress slave_address);

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
     * @brief Automatically configure mailbox from an ESI file for a slave
     *
     * Same as autoConfigureMailbox(SlaveAddress, LogLevel) but reads
     * mailbox configuration from the ESI XML file instead of SII EEPROM.
     * The slave's SII identity is used to match the correct device entry
     * in the ESI file; if no match is found, the first device is used.
     *
     * @param esi           ESI file with parsed device info
     * @param slave_address Slave address (physical or logical)
     * @param log_level     Log level for diagnostics
     * @return true if mailbox was configured successfully
     */
    bool autoConfigureMailbox(const ESIFile& esi, SlaveAddress slave_address,
                              Tether::Platform::LogLevel log_level = Tether::Platform::LogLevel::Info);

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
     * @brief Hardware-reset the master-to-slave mailbox (SM0) by cycling its
     *        activate register.
     *
     * This clears a stuck SM0 that was left full by a failed SDO request (the
     * master wrote the request but the slave never read it — e.g. because the
     * slave was not in PRE_OP).  Disabling and re-enabling SM0 flushes the
     * internal write buffer and clears the mailbox-full flag, allowing the
     * next SDO request to be written.
     *
     * @param slave_index    Slave index (0-based)
     * @return true if SM0 status reads back clear after the reset
     */
    bool resetSlaveMailboxSM0(uint16_t slave_index);

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

    /**
     * @brief Enable/disable automatic mailbox-counter resync on the raw SDO path.
     *
     * When a slave rejects an SDO request with the ETG.1000.6 "syntax error
     * in mailbox message" counter-mismatch error, the raw layer probes the
     * 1..7 counter space until the slave accepts a request, then continues
     * the transaction transparently.  Slaves that reset their mailbox
     * counters on INIT->PRE_OP never emit this error and are unaffected;
     * slaves that retain counter state across restarts (e.g. Synapticon
     * SOMANET) require it to recover without a power cycle.
     *
     * Default: enabled.  Disable to restore the previous behaviour (fail
     * the transaction on counter mismatch; the SM0/SM1 reset fallbacks in
     * waitSm0NotFull()/drainStale() still apply).
     */
    void setMailboxCounterResyncEnabled(bool en) {
        mbx_counter_resync_enabled_.store(en, std::memory_order_relaxed);
    }
    bool mailboxCounterResyncEnabled() const {
        return mbx_counter_resync_enabled_.load(std::memory_order_relaxed);
    }

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
    SlaveSupervisor&    slaveSupervisor();

    // ---- Per-slave PDO manager routing -----------------------------------
    //
    // When using multiple PDOManager / LogicalAddressManager instances
    // (e.g. one per slave or group of slaves), these functions return
    // the manager that covers a given slave.  By default all slaves
    // are covered by the default pdo() / logicalAddressManager().
    //
    // Use createPdoGroup() to create an additional PDOManager + LAM
    // pair with its own logical address space and assign specific
    // slaves to it.

    /// Returns the PDOManager that covers the given slave.
    /// By default this is the default pdo(); after createPdoGroup() it
    /// may be a group-specific manager.
    PDOManager&     pdoForSlave(uint16_t slave_index);

    /// Returns the LogicalAddressManager that covers the given slave.
    /// By default this is the default logicalAddressManager(); after
    /// createPdoGroup() it may be a group-specific LAM.
    LogicalAddressManager& logicalAddressManagerForSlave(uint16_t slave_index);

    /// Create a new PDOManager + LogicalAddressManager pair with its
    /// own logical address space, and assign the given slaves to it.
    ///
    /// @param slave_indices  Slaves to assign to this group
    /// @param base_logical_addr  Base logical address for this group's
    ///                           LRW datagrams (must not overlap with
    ///                           other groups; default 0x10000)
    /// @return Reference to the new PDOManager
    PDOManager&     createPdoGroup(const std::vector<uint16_t>& slave_indices,
                                   uint32_t base_logical_addr = 0x10000);

    /// Access the list of additional PDO groups (for iteration by the
    /// DC real-time loop or custom exchange loops).
    struct PdoGroup {
        std::unique_ptr<IPDOTransport>         transport;
        std::unique_ptr<PDOManager>            pdo;
        std::unique_ptr<LogicalAddressManager> lam;
    };
    const std::vector<PdoGroup>& pdoGroups() const { return pdo_groups_; }
    std::vector<PdoGroup>&       pdoGroups()       { return pdo_groups_; }
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

    // ---- Pre-registration API (used by MasterPDOTransport) -----------------
    // Pre-register a response waiter slot before sending to avoid the
    // send-then-register race when multiple datagrams share one frame.
    size_t    preRegisterResponseWaiter(uint8_t idx, uint8_t* buffer,
                                        size_t buffer_size);
    WaitResult waitForPreRegistered(size_t slot, uint32_t timeout_ms);

    // ---- Cyclic fast path (used by MasterPDOTransport) --------------------
    // Reserved idx range [kCyclicSlotBase, +kNumCyclicSlots): responses are
    // deposited into fixed slots by the RX parser and consumed by the cyclic
    // thread without touching the TransactionRouter.
    bool     supportsCyclicFastPath() const { return static_cast<bool>(iface_.send); }
    uint64_t cyclicSlotToken(uint8_t slot) const;
    /// Generation bit of the last datagram sent on @p slot — the echo
    /// distinguishes a fresh response from a stale deposit that outlived
    /// its cycle's timeout (stale-deposit ABA guard).
    uint8_t  cyclicSlotGen(uint8_t slot) const;
    bool     sendCyclicDatagram(Command cmd, uint8_t slot,
                                uint16_t adp, uint16_t ado,
                                const void* data, uint16_t datalen,
                                bool roundtrip);
    /**
     * @brief Commit a frame already composed into a txAcquire() buffer
     *        (Rotating image mode — the LAM wrote the header at
     *        frame_base, the application wrote the payload at
     *        frame_base+kCyclicFramePayloadOff).
     */
    bool     sendCyclicFrame(uint32_t frame_len);
    /// Wire-offset of the first datagram's payload in a cyclic frame
    /// (eth 14 + ecat-hdr 2 + dg-hdr 10).
    static constexpr uint32_t kCyclicFramePayloadOff = 26;
    /// Acquire the channel's next TX frame (Rotating mode).
    uint8_t* acquireCyclicTxFrame();

    bool     waitCyclicSlot(uint8_t slot, uint64_t token,
                            uint32_t timeout_ns, RxDatagram& out);
    /// View-returning variant — payload points into the slot's inline
    /// buffer (copy path) or into channel ring/bank memory (view path).
    bool     waitCyclicSlotView(uint8_t slot, uint64_t token,
                                uint32_t timeout_ns, CyclicSlotView& out);
    /**
     * @brief Multi-slot wait — single-wake variant for sliced exchanges.
     *
     * Waits until every slot in @p slot_mask has a deposit newer than its
     * token (one ppoll registration for the whole mask — a per-slot loop
     * pays a syscall per slice).
     *
     * @param slot_mask  Bitmask over slots [0, kNumCyclicSlots)
     * @param tokens     Per-slot seq tokens, indexed by slot number
     * @param timeout_ns Shared deadline budget across the whole mask
     * @param views      Output array, indexed by slot number — filled
     *                   only for arrived slots
     * @return The subset of @p slot_mask that arrived before the
     *         deadline; success iff the return == slot_mask.
     */
    uint32_t waitCyclicSlotMask(uint32_t slot_mask, const uint64_t* tokens,
                                uint32_t timeout_ns, CyclicSlotView* views);
    /// Compose [eth][ecat][dg-hdr] into an acquired frame buffer
    /// (Rotating image mode).
    void     composeCyclicHeader(uint8_t* frame, Command cmd, uint8_t slot,
                                 uint16_t adp, uint16_t ado, uint16_t datalen,
                                 bool roundtrip);
    /// Channel accessor for the process image / transport adapter.
    ICyclicChannel* cyclicChannel() const;

    // ---- Frame capacity ----------------------------------------------------

#if TETHER_ENABLE_UDP_ENCAPSULATION
    /// @return max EtherCAT payload bytes per frame, accounting for UDP overhead.
    size_t maxEtherCATPayloadPerFrame() const;
#else
    /// @return max EtherCAT payload bytes per frame.
    size_t maxEtherCATPayloadPerFrame() const {
        if (!transport_) return 1498;
        return transport_->maxEtherCATPayloadPerFrame();
    }
#endif

private:
    friend class SlaveDiscoveryManager;
    /// Test seam — lets tests inject a cyclic channel and drive the
    /// private dispatch path without a live AF_PACKET socket.
    friend struct MasterCyclicTestAccess;
    /// The cyclic datapath reaches back for iface_/router/cancel state.
    friend class CyclicDatapath;

    // ---- Internal helpers --------------------------------------------------
    bool setPreopAndConfirm(uint16_t slave_index);

    /**
     * @brief Perform a BRD scan to discover slaves and initialise internal state.
     *
     * Called by SlaveDiscoveryManager::discover(). Not part of the public API.
     * Performs the broadcast read scan, calls initSlaves(), and initialises
     * fault detection, status poller, and supervisor.
     *
     * @return true if at least one slave was discovered
     */
    bool discoverSlaves();

    void ensureRxQueues();
    void flushRxQueue();
    void parseEtherCATFrame(const uint8_t* frame, size_t length);
    void depositCyclicSlot(uint8_t slot_idx, Command cmd,
                           uint16_t adp, uint16_t ado,
                           const uint8_t* payload, uint16_t datalen,
                           uint16_t wkc, uint8_t gen);
    /// View-mode deposit: payload points into channel-owned memory held by
    /// `cookie`.  The previous held cookie (if any) is released.
    /// `stamp_ns` carries the frame's kernel timestamp (0 → deposit-time).
    /// `gen` is the echoed send-generation bit (lenFlags res-bit 13).
    void publishCyclicSlotView(uint8_t slot_idx, Command cmd,
                               uint16_t adp, uint16_t ado,
                               const uint8_t* payload, uint16_t datalen,
                               uint16_t wkc, uint32_t cookie,
                               uint64_t stamp_ns = 0, uint8_t gen = 0);
    /// Route a frame received on the cyclic channel: pure-cyclic frames
    /// publish slot views, mixed/async frames go to the parser.
    void dispatchChannelFrame(const CyclicFrameView& view);

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
#else
    /// Without UDP encapsulation, sendWithEncapsulation delegates to transport_.
    bool sendWithEncapsulation(const uint8_t* frame, size_t len) {
        if (!transport_) return iface_.send ? iface_.send(frame, len) : false;
        return transport_->send(frame, len);
    }
#endif

    // ---- Data members ------------------------------------------------------

    Config config_;

    // Network
    NetworkInterface iface_{};
    uint8_t src_mac_[6] = {};

    // Frame transport (UDP encapsulation + raw sending)
    std::unique_ptr<EtherCATTransport> transport_;

    // Queues
    std::unique_ptr<Tether::Platform::MessageQueue<RxDatagram>> rx_queue_;
    std::unique_ptr<Tether::Platform::MessageQueue<RxDatagram>> txpdo_rx_queue_;

    // Packet router (TransactionRouter — race-free, idx-indexed)
    TransactionRouter packet_router_;

    // Index allocator
    std::atomic<uint8_t> next_idx_{0};

    // ---- Cyclic fast path ----
    // All cyclic wire state (response slots, channel, process image, send
    // generation, exchange suspension) lives on CyclicDatapath — see
    // src/ethercat/raw/CyclicDatapath.hpp.  Master keeps forwarders so the
    // public/test surface is unchanged.
    std::unique_ptr<CyclicDatapath> datapath_;

    std::unique_ptr<CyclicExecutive> cyclic_loop_;
    /// Async send-on-change loop — mutually exclusive with cyclic_loop_.
    std::unique_ptr<AsyncCyclicLoop> async_loop_;

    /// CPU claims held while the cyclic loop runs (CpuIsolationConfig);
    /// -1 = no claim.  Released by stopCyclicLoop() / ~Master().
    int cyclic_cpu_claim_ = -1;
    int dc_cpu_claim_     = -1;
    /// CPU claim held while the async loop runs; -1 = none.
    int async_cpu_claim_  = -1;

    /// Shared datapath bring-up/teardown used by startCyclicLoop() and
    /// startAsyncLoop(): cyclic channel, process image, strict-WKC, and
    /// the lockable image/slot sections.
    void setupCyclicDatapath(CyclicWireMode wire_mode, ImageMode image_mode,
                             const std::string& shm_name, uint32_t rx_spin_ns,
                             uint32_t slot_spin_ns,
                             CyclicLoopConfig::SlotWaitFallback slot_fallback,
                             bool strict_wkc, const MemoryLockConfig& memlock);
    void teardownCyclicDatapath();

#if TETHER_ENABLE_UDP_ENCAPSULATION
    // IP identification counter for UDP encapsulation
    mutable std::atomic<uint16_t> ip_id_counter_{0};
#endif

    // Master state
    std::atomic<bool>     running_{false};
    std::atomic<bool>     cancel_requested_{false};
    std::atomic<bool>     cancel_warn_logged_{false};  // log cancellation only once
    // (discovered slave count lives on slaves_ — SlaveRegistry::discovered_count)
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
    friend std::unique_ptr<::EtherCAT::SDO::ISDOTransport>
        makeMasterSDOTransport(Master&);
    std::unique_ptr<::EtherCAT::SDO::ISDOTransport> sdo_transport_;
    std::vector<std::unique_ptr<::EtherCAT::CoE::CoEManager>> sdo_managers_;
    mutable std::mutex sdo_managers_mutex_;

    // CoE SDO mailbox channel (refactored from free functions)
    std::unique_ptr<::EtherCAT::Raw::CoeSDOChannel> coe_sdo_channel_;

    // Slave discovery manager (created lazily by discovery())
    std::unique_ptr<SlaveDiscoveryManager> discovery_manager_;

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
    std::unique_ptr<SlaveSupervisor>    slave_supervisor_;

    // Additional PDO groups (for multi-PDOManager setups).
    // Each group owns its own PDOManager + LAM + transport, and covers
    // a set of slaves.  The default group (pdo_ / logical_addr_mgr_)
    // covers all slaves not assigned to a specific group.
    std::vector<PdoGroup>          pdo_groups_;
    std::array<int, ECAT_PDO_MAX_SLAVES> pdo_group_idx_for_slave_;  ///< -1 = default group, else index into pdo_groups_

    // Per-slave state machines, names, invalid-index sentinel, and the
    // discovered-slave count — owned by SlaveRegistry
    // (src/ethercat/raw/SlaveRegistry.hpp).
    std::unique_ptr<SlaveRegistry> slaves_;

    // Debug flags (master-level with per-slave filtering)
    EtherCATMasterDebugFlags debug_flags_;

    // Automatic mailbox-counter resync on the raw SDO path (default on).
    std::atomic<bool> mbx_counter_resync_enabled_{true};

    // Debug gate (conditional debug activation)
    std::unique_ptr<DebugGate> debug_gate_;

    // Default timeout for per-slave SII bus operations (ms).
    uint32_t sii_timeout_ms_ = 500;

    // Mutex protecting the underlying network send. The TransactionRouter
    // is already per-slot concurrent, but the raw Ethernet/UDP send path is
    // not guaranteed to be thread-safe, so all sends are serialised here.
    // The mutex is held only for the actual transmit; callers wait on their
    // own slot condition variable in parallel after releasing it.
    mutable std::mutex send_mutex_;
};

// ============================================================================
// PDOManager
// ============================================================================

} // namespace EtherCAT
