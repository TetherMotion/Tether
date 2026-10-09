/**
 * @file MasterConfig.hpp
 * @brief Master construction and RT-loop configuration structs.
 *
 * Split out of Master.hpp.  Master re-exports these under its old nested
 * names via using-declarations, so `Master::Config` etc. keep working.
 */

#pragma once

#include <cstdint>
#include <functional>
#include <optional>
#include <string>

#include "tether/ethercat/AsyncCyclicLoop.hpp"
#include "tether/ethercat/CyclicChannel.hpp"
#include "tether/ethercat/CyclicExecutive.hpp"
#include "tether/ethercat/DC.hpp"
#include "tether/ethercat/EtherCATTransport.hpp"
#include "tether/ethercat/ProcessImage.hpp"
#include "tether/ethercat/Types.hpp"

namespace EtherCAT {

struct MasterConfig {
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

    /// Wire encapsulation the cyclic datapath must honour — see
    /// setWireEncap().  Read once by CyclicDatapath::setup().
    EtherCAT::WireEncap wire_encap;

    /// Raw AF_PACKET fd of the wire socket, used to create the cyclic
    /// channel when iface_ cannot expose one — under VLAN routing
    /// iface_ is the router stub and this is the only fd available.
    /// Also the async-socket mirror-filter target.  -1 = use
    /// iface_.native_handle (direct EtherCAT path).
    int wire_fd = -1;

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

struct MemoryLockConfig {
    bool lock_all_process = false;  ///< mlockall(CURRENT|FUTURE) — most
                                    ///< thorough, least selective
    bool lock_image       = true;   ///< ProcessImage buffers
    bool lock_slots       = true;   ///< cyclic slot bank + TX staging
    bool prefault_stack   = true;   ///< pre-touch cyclic thread stack
    uint32_t stack_prefault_bytes = 0;  ///< 0 → conservative default
};

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

    /// Wire round-trip budget (ns) shared by a send's collect
    /// deadline — a reply must arrive within this of the send or the
    /// cycle counts wire_loss.  Must exceed the actual bus round-trip
    /// (probe it — e.g. a VLAN hop can push RTT well past 500 µs).
    /// 0 = auto: 80% of cycle_period_us.
    uint32_t rx_budget_ns{0};

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

} // namespace EtherCAT
