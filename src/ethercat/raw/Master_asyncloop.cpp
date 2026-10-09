/**
 * @file Master_asyncloop.cpp
 * @brief Master — async send-on-change loop lifecycle.
 *
 * TU split out of Master_loops.cpp.
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
#include <format>
#include <inttypes.h>
#ifdef __linux__
#include <sys/eventfd.h>
#include <sys/socket.h>
#include <netpacket/packet.h>
#include <unistd.h>
#endif

namespace EtherCAT {

static const char* TAG = "ethercat";

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

} // namespace EtherCAT

