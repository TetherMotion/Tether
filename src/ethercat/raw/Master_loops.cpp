/**
 * @file Master_loops.cpp
 * @brief Master — motion-control loop lifecycle: legacy realtime/polling/
 *        queue loops, the CyclicExecutive fast loop, the AsyncCyclicLoop,
 *        and the exchange-suspension handshake used by slave recovery.
 *        (Datapath bring-up/teardown lives in CyclicDatapath.cpp.)
 */

#include "tether/ethercat/Master.hpp"
#include "tether/ethercat/Slave.hpp"
#include "tether/ethercat/DC.hpp"
#include "tether/ethercat/PDOManager.hpp"
#include "tether/ethercat/LogicalAddressManager.hpp"
#include "tether/ethercat/RealtimeLoop.hpp"
#include "tether/ethercat/CyclicChannel.hpp"
#include "tether/ethercat/DebugFlags.hpp"
#include "tether/ethercat/EtherCATConfig.hpp"
#include "raw/internal.hpp"
#include "raw/MotionLoops.hpp"
#include "raw/CyclicDatapath.hpp"
#include "tether/platform/Platform.hpp"
#include "tether/platform/RtMemory.hpp"
#include "tether/platform/CpuIsolation.hpp"

#include <thread>
#include <chrono>
#include <algorithm>
#include <cstring>
#include <inttypes.h>
#ifdef __linux__
#include <sys/eventfd.h>
#include <sys/socket.h>
#include <netpacket/packet.h>
#include <unistd.h>
#endif

namespace EtherCAT {

static const char* TAG = "ethercat";

void Master::setMotionControlCallback(MotionControlCallback callback)
{
    motion_control_callback_ = std::move(callback);
}

bool Master::startRealtimeMotionControlLoop()
{
    return startRealtimeMotionControlLoop(RealtimeMotionLoopConfig{});
}

bool Master::startRealtimeMotionControlLoop(const RealtimeMotionLoopConfig& config)
{
    if (!motion_control_callback_) {
        TETHER_LOGE(TAG, "No motion control callback configured");
        return false;
    }

    stopMotionControlLoop();
    clearCancel();
    auto wrapped_callback = [this](double dt) -> bool {
        if (cancel_requested_.load(std::memory_order_acquire)) {
            return false;
        }
        return motion_control_callback_ ? motion_control_callback_(dt) : true;
    };
    motion_control_loop_ = makeRealtimeMotionControlLoop(std::move(wrapped_callback), config, dc_.get());
    motion_control_loop_->setShutdownDebug(debug_flags_.shutdown);
    return motion_control_loop_->start();
}

bool Master::startPollingMotionControlLoop()
{
    return startPollingMotionControlLoop(PollingMotionLoopConfig{});
}

bool Master::startPollingMotionControlLoop(const PollingMotionLoopConfig& config)
{
    if (!motion_control_callback_) {
        TETHER_LOGE(TAG, "No motion control callback configured");
        return false;
    }

    stopMotionControlLoop();
    clearCancel();
    auto wrapped_callback = [this](double dt) -> bool {
        if (cancel_requested_.load(std::memory_order_acquire)) {
            return false;
        }
        return motion_control_callback_ ? motion_control_callback_(dt) : true;
    };
    motion_control_loop_ = makePollingMotionControlLoop(std::move(wrapped_callback), config, dc_.get());
    motion_control_loop_->setShutdownDebug(debug_flags_.shutdown);
    return motion_control_loop_->start();
}

void Master::stopMotionControlLoop()
{
    if (motion_control_loop_) {
        motion_control_loop_->stop();
        motion_control_loop_.reset();
    }
}

bool Master::isMotionControlLoopRunning() const
{
    return motion_control_loop_ && motion_control_loop_->isRunning();
}

// ============================================================================
// Cyclic executive — deadline-driven fast loop
// ============================================================================

bool Master::startCyclicLoop(const CyclicLoopConfig& config)
{
    stopAsyncLoop();   // mutually exclusive — the user picks the loop model
    stopCyclicLoop();
    stopMotionControlLoop();
    // A legacy DC realtime loop (dc().start()) drives its own PDO exchange —
    // it must not run alongside the cyclic exchange or two LRW streams share
    // the wire.  DC sync frames are driven by the executive's own DC task.
    if (dc_ && dc_->getState() == DC::DCState::Running) dc_->stop();

    // Optional DC auto-init — an explicit dc_config initializes DC here so
    // the app cannot forget the initialize-before-start ordering.  When DC
    // is already initialized the existing setup is kept.
    if (config.dc_config && !(dc_ && dc_->isInitialized())) {
        const uint16_t n = getDiscoveredSlaveCount();
        if (n > 0 && dc().init(*config.dc_config, n)) {
            TETHER_LOGI(TAG, "cyclic loop: DC auto-initialized "
                             "({} slaves)", n);
        } else {
            TETHER_LOGW(TAG, "cyclic loop: dc_config set but DC init failed "
                             "(slaves={}) — sync disabled", n);
        }
    }

    const bool dc_sync_enabled = config.enable_dc_synchronization ||
                                 config.dc_config.has_value();

    // Loud validation — these combinations silently no-op otherwise.
    if (config.motion_in_loop && !motion_control_callback_) {
        TETHER_LOGW(TAG, "cyclic loop: motion_in_loop is set but no motion "
                         "control callback is registered — the MotionControl "
                         "phase will be empty");
    }
    if (config.exchange_placement == ExchangePlacement::SplitLate &&
        config.motion_in_loop) {
        TETHER_LOGW(TAG, "cyclic loop: SplitLate collects after the "
                         "MotionControl phase — in-loop motion reads "
                         "previous-cycle inputs (intended only for external "
                         "motion sources)");
    }
    if (dc_sync_enabled && !(dc_ && dc_->isInitialized())) {
        TETHER_LOGW(TAG, "cyclic loop: DC synchronization enabled but DC was "
                         "never initialized — the DC task will idle (set "
                         "dc_config or call dc().init() first)");
    }
    // Storage-bound entries (any buffered-path accessor taken) are bridged
    // by the exchange itself — image modes stay coherent for device-level
    // PDO accessors without any warning gate.

    clearCancel();
    // A suspension must not leak into a fresh loop — the suspender (slave
    // recovery) may have died between suspend and resume.
    datapath_->exchange_suspended_.store(false, std::memory_order_release);

    CyclicExecutive::Config exec_cfg = config.exec;
    exec_cfg.cycle_period_us    = config.cycle_period_us;
    exec_cfg.dc_interval_cycles = config.sync_interval_cycles;
    datapath_->rx_spin_ns_         = config.rx_spin_ns;
    datapath_->slot_spin_ns_       = config.slot_spin_ns;
    datapath_->slot_wait_fallback_ = config.slot_wait_fallback;

    // ---- Runtime CPU isolation (opt-in) --------------------------------
    // Claim CPUs for the RT threads before computing affinities.  Claims
    // are released by stopCyclicLoop() and at process exit.
    if (config.cpu_isolation.enabled) {
        auto& iso = Tether::Platform::CpuIsolation::instance();
        Tether::Platform::CpuIsolation::Spec spec;
        spec.prefer_isolated = config.cpu_isolation.prefer_isolated;
        spec.avoid_cpu0      = config.cpu_isolation.avoid_cpu0;
        spec.create_cpuset   = config.cpu_isolation.create_cpuset;
        spec.steer_irqs      = config.cpu_isolation.steer_irqs;

        spec.requested_cpu = config.cpu_isolation.cyclic_cpu;
        auto c = iso.claim(spec);
        if (c.valid()) {
            cyclic_cpu_claim_      = c.cpu;
            exec_cfg.cpu_affinity  = c.cpu;
        } else if (config.cpu_isolation.cyclic_cpu >= 0) {
            TETHER_LOGW(TAG, "CPU isolation: cyclic CPU {} claim denied — "
                             "running unpinned",
                        config.cpu_isolation.cyclic_cpu);
        }

        if (exec_cfg.dc_placement ==
            CyclicExecutive::DCPlacement::DedicatedThread) {
            spec.requested_cpu = config.cpu_isolation.dc_cpu;
            auto d = iso.claim(spec);
            if (d.valid()) {
                dc_cpu_claim_            = d.cpu;
                exec_cfg.dc_cpu_affinity = d.cpu;
            } else if (config.cpu_isolation.dc_cpu >= 0) {
                TETHER_LOGW(TAG, "CPU isolation: DC CPU {} claim denied",
                            config.cpu_isolation.dc_cpu);
            }
        }
    }

    // ---- Memory locking (each section independently opt-out-able) ------
    if (config.memory_lock.lock_all_process) {
        Tether::Platform::lockAllMemory();
    }
    if (exec_cfg.stack_prefault_bytes == 0 &&
        config.memory_lock.stack_prefault_bytes > 0) {
        exec_cfg.stack_prefault_bytes =
            config.memory_lock.stack_prefault_bytes;
    }
    if (!config.memory_lock.prefault_stack) {
        exec_cfg.stack_prefault_bytes = 0;
    }

    if (dc_sync_enabled &&
        exec_cfg.dc_placement == CyclicExecutive::DCPlacement::Disabled) {
        // Caller asked for DC sync but left placement at a disabled value —
        // default to the fault-isolated dedicated thread.
        exec_cfg.dc_placement = CyclicExecutive::DCPlacement::DedicatedThread;
    }
    if (!dc_sync_enabled) {
        exec_cfg.dc_placement = CyclicExecutive::DCPlacement::Disabled;
    }
    exec_cfg.jitter    = JitterConfig::defaults(config.cycle_period_us);
    exec_cfg.dc_jitter = JitterConfig::defaults(
        config.cycle_period_us * config.sync_interval_cycles);

    // Exchange: LRW process image via the reserved-slot fast path when the
    // transport supports it, falling back to the router path otherwise.
    // Split placements run the send half at Exchange and the collect half
    // in a later phase — the wire round-trip overlaps the work between.
    const bool split_exchange =
        config.exchange_placement != ExchangePlacement::Atomic;
    const uint32_t cyclic_rx_budget_ns =
        std::min<uint32_t>(200'000, config.cycle_period_us * 900);

    CyclicExecutive::TaskFn exchange_fn = [this, split_exchange,
                                           cyclic_rx_budget_ns]() -> bool {
        // Exchange suspension (slave recovery re-registers the mapping):
        // skip the cycle's wire work and bump the quiesce counter the
        // suspender waits on.  Runs at task entry — the single-threaded
        // executive guarantees no exchange is in flight at that point.
        if (datapath_->exchange_suspended_.load(std::memory_order_acquire)) {
            datapath_->exchange_quiesced_.fetch_add(1, std::memory_order_relaxed);
            return true;
        }
        bool ok = true;
        if (pdo_) {
            if (logical_addr_mgr_ && logical_addr_mgr_->isInitialized()) {
                if (split_exchange) {
                    ok = pdo_->cyclicSend(&datapath_->image_,
                                          cyclic_rx_budget_ns);
                } else {
                    ok = pdo_->exchangeAllLRWCyclic(cyclic_rx_budget_ns,
                                                    &datapath_->image_);
                }
            } else {
                ok = pdo_->exchangeAll();
            }
        }
        for (auto& g : pdo_groups_) {
            if (g.pdo) ok = g.pdo->exchangeAll() && ok;
        }
        return ok;
    };

    CyclicExecutive::TaskFn collect_fn;
    if (split_exchange) {
        collect_fn = [this]() -> bool {
            if (datapath_->exchange_suspended_.load(std::memory_order_acquire)) {
                datapath_->exchange_quiesced_.fetch_add(1, std::memory_order_relaxed);
                return true;
            }
            datapath_->collect_calls_.fetch_add(1, std::memory_order_relaxed);
            return !pdo_ || pdo_->cyclicCollect(&datapath_->image_);
        };
    }

    CyclicExecutive::TaskFn dc_fn;
    if (dc_sync_enabled) {
        // Gate on isInitialized(), not Running — under the cyclic executive
        // there is no legacy DC loop to flip the state; initialize() arms
        // the slaves' sync units and this task emits the sync frames.  No
        // reference slave (init ran but nothing DC-capable) → idle quietly.
        dc_fn = [this]() -> bool {
            if (!dc_ || !dc_->isInitialized()) return true;
            EtherCATDC* dc = dc_->get();
            if (!dc || dc->referenceSlave() < 0) return true;
            return dc->sendSyncFrame();
        };
    }

    CyclicExecutive::TimeFunc time_fn;
    if (dc_ && dc_->isInitialized()) {
        time_fn = [this]() -> uint64_t {
            EtherCATDC* dc = dc_->get();
            return dc ? dc->getMasterTimeNs() : 0;
        };
    }

    // ---- Cyclic channel + process image + lockable sections ------------
    setupCyclicDatapath(config.wire_mode, config.image_mode,
                        config.shm_image_name, config.rx_spin_ns,
                        config.slot_spin_ns, config.slot_wait_fallback,
                        config.strict_wkc, config.memory_lock);

    cyclic_loop_ = std::make_unique<CyclicExecutive>(
        std::move(exchange_fn), std::move(dc_fn), std::move(time_fn), exec_cfg);

    // Split-phase collect placement — resolved before task registration so
    // that a MotionControl collect registers BEFORE the motion callback:
    // in-phase order is registration order, and collect-then-motion is what
    // makes the motion update see this cycle's inputs.  Split honors the
    // user-supplied collect_phase (Q2); SplitLate pins Diagnostics.
    TaskPhase collect_phase = config.collect_phase;
    if (config.exchange_placement == ExchangePlacement::SplitLate) {
        collect_phase = TaskPhase::Diagnostics;
    }
    if (collect_phase == TaskPhase::PreExchange ||
        collect_phase == TaskPhase::Exchange) {
        TETHER_LOGW(TAG, "collect_phase must run after Exchange — "
                         "clamping to PostExchange");
        collect_phase = TaskPhase::PostExchange;
    }
    if (collect_fn && collect_phase == TaskPhase::MotionControl) {
        cyclic_loop_->addTask(TaskPhase::MotionControl, std::move(collect_fn));
    }

    if (config.motion_in_loop && motion_control_callback_) {
        const double dt = static_cast<double>(config.cycle_period_us) / 1e6;
        auto cb = motion_control_callback_;
        cyclic_loop_->addTask(TaskPhase::MotionControl,
            [this, cb, dt]() -> bool {
                if (cancel_requested_.load(std::memory_order_acquire)) {
                    return false;
                }
                return cb(dt);
            });
    }

    if (collect_fn && collect_phase != TaskPhase::MotionControl) {
        cyclic_loop_->addTask(collect_phase, std::move(collect_fn));
    }

    // The cyclic loop consumes the wire on its own deadline — a
    // triggerSend() here bumps an unconsumed counter; mark the consumer
    // so the producer gets a one-shot warning (Q27).
    datapath_->image_.setSendConsumer(ProcessImage::SendConsumer::Cyclic);
    if (!cyclic_loop_->start()) {
        datapath_->image_.setSendConsumer(ProcessImage::SendConsumer::None);
        return false;
    }
    return true;
}

void Master::stopCyclicLoop()
{
    // The shared datapath may belong to the async loop — only tear it
    // down when the cyclic loop owned it (its thread is dead by then).
    const bool owned_datapath = cyclic_loop_ != nullptr;
    if (cyclic_loop_) {
        cyclic_loop_->stop();
        cyclic_loop_.reset();
        datapath_->image_.setSendConsumer(ProcessImage::SendConsumer::None);
    }
    // Release runtime CPU claims before the threads' affinity becomes
    // meaningless — claims are also auto-released at process exit.
    if (cyclic_cpu_claim_ >= 0 || dc_cpu_claim_ >= 0) {
        auto& iso = Tether::Platform::CpuIsolation::instance();
        if (cyclic_cpu_claim_ >= 0) { iso.release(cyclic_cpu_claim_); cyclic_cpu_claim_ = -1; }
        if (dc_cpu_claim_ >= 0)     { iso.release(dc_cpu_claim_);     dc_cpu_claim_ = -1; }
    }
    if (owned_datapath) {
        teardownCyclicDatapath();
    }
}

bool Master::isCyclicLoopRunning() const
{
    return cyclic_loop_ && cyclic_loop_->isRunning();
}

CyclicExecutive::Stats Master::getCyclicLoopStats() const
{
    return cyclic_loop_ ? cyclic_loop_->getStats() : CyclicExecutive::Stats{};
}

bool Master::suspendCyclicExchange(uint32_t timeout_us)
{
    const bool any_loop = isCyclicLoopRunning() || isAsyncLoopRunning();
    if (!any_loop) return true;   // nothing to quiesce

    const uint32_t q0 =
        datapath_->exchange_quiesced_.load(std::memory_order_relaxed);
    datapath_->exchange_suspended_.store(true, std::memory_order_release);
    // Kick an idle async loop out of waitSend — in OnSend mode with no
    // producers triggering, no gated task would otherwise run and the
    // wait below would burn the whole timeout.  The woken loop hits the
    // suspension gate before touching the wire.  (CyclicExecutive needs
    // no kick: its deadline timer bounds every task-entry interval.)
    if (isAsyncLoopRunning()) datapath_->image_.triggerSend();

    // Wait for the loop thread to pass a gated task entry — proves no
    // send/collect is executing (single-threaded loop) and all future
    // exchange work is gated until resume.
    auto& clock = Tether::Platform::Clock::instance();
    const int64_t deadline_us =
        clock.getMicroseconds() + static_cast<int64_t>(timeout_us);
    while (datapath_->exchange_quiesced_.load(std::memory_order_relaxed) == q0) {
        if (!isCyclicLoopRunning() && !isAsyncLoopRunning()) {
            return true;   // loop died mid-suspend — trivially quiesced
        }
        if (clock.getMicroseconds() >= deadline_us) {
            datapath_->exchange_suspended_.store(false, std::memory_order_release);
            TETHER_LOGW(TAG, "cyclic exchange suspend timed out — "
                             "released (epoch guard still covers the "
                             "mutation)");
            return false;
        }
        clock.delayMicroseconds(50);
    }
    return true;
}

void Master::resumeCyclicExchange()
{
    datapath_->exchange_suspended_.store(false, std::memory_order_release);
}

// ============================================================================
// Async send-on-change loop — external-producer RxPDO source
// ============================================================================

bool Master::startAsyncLoop(const AsyncLoopConfig& config)
{
    // Mutually exclusive with the cyclic loop — the user picks the model.
    // Async first: its thread blocks on datapath_->image_'s send word, which
    // the shared datapath teardown reconfigures — the thread must be dead
    // before any teardown runs, or waitSend reads unmapped shm.
    stopAsyncLoop();
    stopCyclicLoop();
    stopMotionControlLoop();
    // A running legacy DC loop owns its own PDO exchange — it must not
    // share the wire with the async loop's sends.
    if (dc_ && dc_->getState() == DC::DCState::Running) dc_->stop();
    clearCancel();
    datapath_->exchange_suspended_.store(false, std::memory_order_release);

    AsyncCyclicLoop::Config ecfg = config.exec;
    ecfg.collect_mode         = config.collect_mode;
    ecfg.collect_period_us    = config.collect_period_us;
    ecfg.min_send_interval_ns = config.min_send_interval_ns;
    ecfg.max_idle_ns          = config.max_idle_ns;
    ecfg.dc_interval_us       = config.enable_dc_synchronization
                                ? config.dc_interval_us : 0;
    datapath_->rx_spin_ns_         = config.rx_spin_ns;
    datapath_->slot_spin_ns_       = config.slot_spin_ns;
    datapath_->slot_wait_fallback_ = config.slot_wait_fallback;

    // ---- Runtime CPU isolation (opt-in; async thread + optional DC) ---
    if (config.cpu_isolation.enabled) {
        auto& iso = Tether::Platform::CpuIsolation::instance();
        Tether::Platform::CpuIsolation::Spec spec;
        spec.prefer_isolated = config.cpu_isolation.prefer_isolated;
        spec.avoid_cpu0      = config.cpu_isolation.avoid_cpu0;
        spec.create_cpuset   = config.cpu_isolation.create_cpuset;
        spec.steer_irqs      = config.cpu_isolation.steer_irqs;
        spec.requested_cpu   = config.cpu_isolation.cyclic_cpu;
        auto c = iso.claim(spec);
        if (c.valid()) {
            async_cpu_claim_    = c.cpu;
            ecfg.cpu_affinity   = c.cpu;
        } else if (config.cpu_isolation.cyclic_cpu >= 0) {
            TETHER_LOGW(TAG, "CPU isolation: async CPU {} claim denied — "
                             "running unpinned",
                        config.cpu_isolation.cyclic_cpu);
        }
        if (ecfg.dc_interval_us > 0) {
            spec.requested_cpu = config.cpu_isolation.dc_cpu;
            auto d = iso.claim(spec);
            if (d.valid()) {
                dc_cpu_claim_          = d.cpu;
                ecfg.dc_cpu_affinity   = d.cpu;
            } else if (config.cpu_isolation.dc_cpu >= 0) {
                TETHER_LOGW(TAG, "CPU isolation: async DC CPU {} claim "
                                 "denied", config.cpu_isolation.dc_cpu);
            }
        }
    }

    // ---- Memory locking (each section independently opt-out-able) ------
    if (config.memory_lock.lock_all_process) {
        Tether::Platform::lockAllMemory();
    }
    if (ecfg.stack_prefault_bytes == 0 &&
        config.memory_lock.stack_prefault_bytes > 0) {
        ecfg.stack_prefault_bytes = config.memory_lock.stack_prefault_bytes;
    }
    if (!config.memory_lock.prefault_stack) {
        ecfg.stack_prefault_bytes = 0;
    }

    // ---- Cyclic channel + process image + lockable sections ------------
    setupCyclicDatapath(config.wire_mode, config.image_mode,
                        config.shm_image_name, config.rx_spin_ns,
                        config.slot_spin_ns, config.slot_wait_fallback,
                        config.strict_wkc, config.memory_lock);

    // Send: one RxPDO shot per trigger.  cyclicSend() falls back to the
    // atomic exchange when no logical address manager is configured.
    AsyncCyclicLoop::TaskFn send_fn =
        [this, rx_timeout = config.rx_timeout_ns]() -> bool {
            if (datapath_->exchange_suspended_.load(std::memory_order_acquire)) {
                datapath_->exchange_quiesced_.fetch_add(1, std::memory_order_relaxed);
                return true;
            }
            bool ok = true;
            if (pdo_) {
                if (logical_addr_mgr_ && logical_addr_mgr_->isInitialized()) {
                    ok = pdo_->cyclicSend(&datapath_->image_, rx_timeout);
                } else {
                    ok = pdo_->exchangeAll();
                }
            }
            for (auto& g : pdo_groups_) {
                if (g.pdo) ok = g.pdo->exchangeAll() && ok;
            }
            return ok;
        };
    AsyncCyclicLoop::TaskFn collect_fn = [this]() -> bool {
        if (datapath_->exchange_suspended_.load(std::memory_order_acquire)) {
            datapath_->exchange_quiesced_.fetch_add(1, std::memory_order_relaxed);
            return true;
        }
        return !pdo_ || pdo_->cyclicCollect(&datapath_->image_);
    };
    AsyncCyclicLoop::TaskFn dc_fn;
    if (ecfg.dc_interval_us > 0) {
        // Same isInitialized() gate as the cyclic path — the async DC
        // thread owns the sync cadence; no legacy DC loop is required.
        dc_fn = [this]() -> bool {
            if (!dc_ || !dc_->isInitialized()) return true;
            EtherCATDC* dc = dc_->get();
            return dc ? dc->sendSyncFrame() : true;
        };
    }

    async_loop_ = std::make_unique<AsyncCyclicLoop>(
        datapath_->image_, std::move(send_fn), std::move(collect_fn),
        std::move(dc_fn), AsyncCyclicLoop::TimeFunc{}, ecfg);
    // Mark the trigger word's consumer BEFORE start() so a triggerSend()
    // under the wrong loop model warns instead of silently dropping (Q27).
    datapath_->image_.setSendConsumer(ProcessImage::SendConsumer::Async);
    if (!async_loop_->start()) {
        datapath_->image_.setSendConsumer(ProcessImage::SendConsumer::None);
        return false;
    }
    return true;
}

void Master::stopAsyncLoop()
{
    // The shared datapath may belong to the cyclic loop — only tear it
    // down when the async loop owned it (its thread is dead by then).
    if (async_loop_) {
        async_loop_->stop();
        async_loop_.reset();
        datapath_->image_.setSendConsumer(ProcessImage::SendConsumer::None);
        teardownCyclicDatapath();
    }
    if (async_cpu_claim_ >= 0) {
        Tether::Platform::CpuIsolation::instance().release(async_cpu_claim_);
        async_cpu_claim_ = -1;
    }
    if (dc_cpu_claim_ >= 0) {
        Tether::Platform::CpuIsolation::instance().release(dc_cpu_claim_);
        dc_cpu_claim_ = -1;
    }
}

bool Master::isAsyncLoopRunning() const
{
    return async_loop_ && async_loop_->isRunning();
}

AsyncCyclicLoop::Stats Master::getAsyncLoopStats() const
{
    return async_loop_ ? async_loop_->getStats() : AsyncCyclicLoop::Stats{};
}

// Forwarders live in CyclicDatapath.cpp — setup/teardown own the
// channel + process image there.

// Queue-mode RT loop
// ============================================================================

bool Master::startQueueModeLoop()
{
    return startQueueModeLoop(RealtimeMotionLoopConfig{});
}

bool Master::startQueueModeLoop(const RealtimeMotionLoopConfig& config)
{
    if (pdo_->getMode() != PDOMode::Queue) {
        TETHER_LOGE(TAG, "PDOManager is not in Queue mode; call configureQueueMode() first");
        return false;
    }

    stopMotionControlLoop();  // Ensure no other loop is running
    clearCancel();
    motion_control_loop_ = makeQueueMotionControlLoop(pdo_.get(), config, dc_.get());
    motion_control_loop_->setShutdownDebug(debug_flags_.shutdown);
    return motion_control_loop_->start();
}

void Master::stopQueueModeLoop()
{
    stopMotionControlLoop();
}

bool Master::isQueueModeLoopRunning() const
{
    return motion_control_loop_ && motion_control_loop_->isRunning();
}

} // namespace EtherCAT
