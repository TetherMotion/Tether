// SPDX-License-Identifier: MIT
/**
 * @file SlaveSync.hpp
 * @brief Abstract multi-slave state-transition synchronization for EtherCAT
 *
 * @details
 * Provides a generic framework for coordinating state transitions across
 * multiple EtherCAT slaves — bringing them through INIT → PRE_OP → SAFE_OP
 * → OP (or any other sequence) with proper synchronization between phases.
 *
 * The framework is **drive-agnostic**: it knows nothing about Synapticon,
 * ESC211, CiA 402, or any specific slave type.  Each slave provides an
 * `ISlaveTransitionHandler` implementation that encapsulates its
 * drive-specific transition logic (mailbox config, PDO mapping, SDO
 * writes, etc.).
 *
 * ## Core concepts
 *
 * - **ISlaveTransitionHandler** — polymorphic interface for a single
 *   slave's state-transition logic.  Implementations are provided by
 *   the application (e.g. wrapping drive-specific init sequences).
 *
 * - **SlaveSyncSlot** — per-slave synchronization state: the handler,
 *   ready/in_target/failed atomics, and a post-transition callback.
 *   Owned by the application; the coordinator holds non-owning pointers.
 *
 * - **SlaveSyncGate** — condition-variable gate for waiting until all
 *   registered slaves have reached a target state (or one has failed).
 *
 * - **SlaveSyncCoordinator** — orchestrates multi-slave transitions.
 *   Supports sequential transitions, inter-phase hooks (callbacks run
 *   between transition phases), and recovery integration.
 *
 * - **SyncRecoveryHandler** — implements `ISlaveRecoveryHandler` by
 *   dispatching to the coordinator's slot registry, so the
 *   `SlaveSupervisor` can recover any registered slave.
 *
 * ## Typical usage
 *
 * ```cpp
 * EtherCAT::SlaveSyncCoordinator coord;
 *
 * // Create per-slave handlers (application-specific)
 * MyEsc211Handler esc211_handler(master, 0, ...);
 * MySynapticonHandler syn_handler(master, 1, ...);
 *
 * // Create slots (application owns these)
 * EtherCAT::SlaveSyncSlot esc211_slot{
 *     .slave_index = 0,
 *     .handler = &esc211_handler,
 *     .name = "ESC211",
 *     .on_reached = [&]() { pdo.cacheEsc211(); },
 * };
 * EtherCAT::SlaveSyncSlot syn_slot{
 *     .slave_index = 1,
 *     .handler = &syn_handler,
 *     .name = "Synapticon",
 *     .on_reached = [&]() { pdo.cacheSyn(); },
 * };
 *
 * coord.registerSlot(esc211_slot);
 * coord.registerSlot(syn_slot);
 *
 * // Phase 1: all slaves to SAFE_OP (no PDO exchange)
 * coord.transitionAllTo(EtherCAT::ECState::SafeOp,
 *     {.exchange_thread_active = false});
 *
 * // Inter-phase hook: start PDO exchange thread
 * exchange.start();
 *
 * // Phase 2: all slaves to OP (exchange thread priming)
 * coord.transitionAllTo(EtherCAT::ECState::Op,
 *     {.exchange_thread_active = true});
 * ```
 *
 * @see SlaveSupervisor for automatic recovery integration.
 */

#pragma once

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <stop_token>
#include <string>
#include <vector>

#include "tether/ethercat/SlaveSupervisor.hpp"  // ISlaveRecoveryHandler
#include "tether/platform/EspCompat.hpp"

namespace EtherCAT {

// Forward declaration — defined in tether/profiles/cia402/CiA402StateUtils.hpp
enum class ECState : uint8_t;

namespace Util {

// ============================================================================
// Transition Context
// ============================================================================

/// Context passed to `ISlaveTransitionHandler::transitionTo()`.
///
/// Carries information about the environment in which the transition
/// is happening, so the handler can adapt its behavior (e.g. skip
/// manual PDO priming when the exchange thread is already running).
struct TransitionContext {
    /// Whether a PDO exchange thread is already cycling this slave's
    /// PDO group.  When true, the handler should skip manual
    /// `exchangeAll()` priming — the exchange thread is doing it.
    /// When false, the handler must do its own priming before
    /// requesting a state that requires PDO counters (e.g. OP).
    bool exchange_thread_active = false;

    /// Whether this transition is part of a recovery sequence
    /// (triggered by the SlaveSupervisor).  Recovery transitions
    /// typically start from INIT and go all the way to OP, with
    /// `exchange_thread_active = false` (the exchange thread skips
    /// the recovering slave's group because its ready flag is cleared).
    bool is_recovery = false;
};

// ============================================================================
// Transition Handler Interface
// ============================================================================

/**
 * @brief Polymorphic interface for a single slave's state-transition logic.
 *
 * Implementations are drive/application-specific.  They encapsulate all
 * the steps needed to move a slave from its current state to a target
 * EtherCAT state — mailbox configuration, PDO mapping, SDO writes,
 * DC reconfiguration, error acknowledgement, etc.
 *
 * The handler is called from a non-realtime context (the main thread
 * or the supervisor's recovery thread).  It must not be invoked from
 * a realtime loop callback.
 */
class ISlaveTransitionHandler {
public:
    virtual ~ISlaveTransitionHandler() = default;

    /**
     * @brief Transition this slave to the target EtherCAT state.
     *
     * The handler is responsible for all steps needed to reach the
     * target state from the slave's current state.  For example, a
     * transition to SafeOp typically involves: mailbox config,
     * PRE_OP transition, PDO mapping, sync-manager config, and the
     * SAFE_OP state request.
     *
     * @param target  The target EtherCAT state (PreOp, SafeOp, Op, etc.)
     * @param ctx     Context describing the transition environment.
     * @return true if the slave reached the target state,
     *         false on failure.
     */
    virtual bool transitionTo(ECState target, const TransitionContext& ctx) = 0;

    /**
     * @brief Full re-initialization from INIT to the slave's operational state.
     *
     * Called by the recovery handler after the supervisor has forced
     * the slave to INIT.  The handler should call `transitionTo()`
     * for each state in sequence (PRE_OP → SAFE_OP → OP), with
     * appropriate `TransitionContext` for each phase.
     *
     * @return true if the slave is back in its operational state,
     *         false on failure.
     */
    virtual bool reinitialize() = 0;

    /**
     * @brief The slave index this handler manages.
     */
    virtual uint16_t slaveIndex() const = 0;
};

// ============================================================================
// Per-Slave Synchronization Slot
// ============================================================================

/**
 * @brief Per-slave synchronization state.
 *
 * Owned by the application (typically as a stack variable or in a
 * `std::vector<std::unique_ptr<SlaveSyncSlot>>`).  The coordinator
 * holds non-owning pointers.
 *
 * The `ready` flag is checked by the exchange thread before calling
 * `exchangeAll()` on the slave's PDO group.  The `in_target_state`
 * and `failed` flags are checked by the gate when waiting for all
 * slaves to reach a target state.
 */
struct SlaveSyncSlot {
    /// The slave index this slot manages.
    uint16_t slave_index = 0;

    /// The transition handler (owned by the application).
    ISlaveTransitionHandler* handler = nullptr;

    /// Human-readable name for logging.
    std::string name;

    // --- Synchronization atomics ---

    /// Set after PDO mapping is stable and PDO buffers are cached.
    /// The exchange thread checks this before calling exchangeAll().
    std::atomic<bool> ready{false};

    /// Set after the slave reaches its target state.
    /// Checked by the gate's waitAll/waitOne methods.
    std::atomic<bool> in_target_state{false};

    /// Set if a transition fails permanently.
    std::atomic<bool> failed{false};

    // --- Callbacks ---

    /// Called after a successful transition to the target state
    /// (e.g. to cache PDO buffer pointers and set the ready flag).
    /// Called from the same thread that invoked the coordinator's
    /// transitionAllTo().
    std::function<void()> on_reached;

    // --- Convenience ---

    /// Reset all flags to their initial state (before re-initialization).
    void reset() {
        ready.store(false, std::memory_order_release);
        in_target_state.store(false, std::memory_order_release);
        failed.store(false, std::memory_order_release);
    }
};

// ============================================================================
// Synchronization Gate
// ============================================================================

/**
 * @brief Condition-variable gate for waiting on N slaves to reach a state.
 *
 * Each `SlaveSyncSlot` owns its own `in_target_state` and `failed`
 * atomics.  The gate iterates over a list of slot pointers to check
 * if all are in the target state or any has failed.
 *
 * Thread-safety: the gate's `notify()` can be called from any thread.
 * `waitAll()` and `waitOne()` must be called from the same thread
 * (they acquire the internal mutex).
 */
class SlaveSyncGate {
public:
    /// Notify the gate that a slot's state changed.
    void notify();

    /// Wait until all slots have reached their target state or one has failed.
    /// @param st    Stop token for cooperative cancellation.
    /// @param slots List of slots to check.
    /// @return true if all slots are in their target state,
    ///         false if one failed or the stop token was requested.
    bool waitAll(std::stop_token st, const std::vector<SlaveSyncSlot*>& slots) const;

    /// Wait until one specific slot has reached its target state or failed.
    /// @param st   Stop token for cooperative cancellation.
    /// @param slot The slot to wait on.
    /// @return true if the slot reached its target state,
    ///         false if it failed or the stop token was requested.
    bool waitOne(std::stop_token st, const SlaveSyncSlot& slot) const;

private:
    mutable std::mutex mtx_;
    mutable std::condition_variable cv_;
};

// ============================================================================
// Synchronization Coordinator
// ============================================================================

/**
 * @brief Orchestrates multi-slave state transitions.
 *
 * The coordinator manages a registry of `SlaveSyncSlot`s and provides
 * methods to transition all (or a subset) to a target EtherCAT state.
 * Transitions are performed sequentially to avoid SDO mutex contention.
 *
 * Between transition phases, the application can run inter-phase hooks
 * (e.g. starting the PDO exchange thread between SAFE_OP and OP) by
 * simply calling `transitionAllTo()` twice with different target states.
 *
 * ## Recovery integration
 *
 * The coordinator works with `SlaveSupervisor` via `SyncRecoveryHandler`,
 * which implements `ISlaveRecoveryHandler` by dispatching to the
 * coordinator's slot registry.  When a slave is recovered:
 *   1. The handler clears the slot's `ready` flag (exchange thread stops).
 *   2. The handler calls `reinitialize()` on the slot's transition handler.
 *   3. On success, the `on_reached` callback is invoked (re-caches PDO
 *      buffers, sets `ready` again).
 */
class SlaveSyncCoordinator {
public:
    /// Register a slot.  The coordinator does not take ownership.
    /// The slot must outlive the coordinator.
    void registerSlot(SlaveSyncSlot& slot);

    /// Unregister a slot by slave index.
    void unregisterSlot(uint16_t slave_index);

    /// Get all registered slots.
    const std::vector<SlaveSyncSlot*>& slots() const { return slots_; }

    /// Find a slot by slave index.  Returns nullptr if not found.
    SlaveSyncSlot* findSlot(uint16_t slave_index) const;

    /// Clear the ready flag for all slots (before re-initialization).
    void clearAllReady();

    /// Clear the ready flag for one slot.
    void clearReady(uint16_t slave_index);

    /**
     * @brief Transition all registered slaves to the target state sequentially.
     *
     * Each slave's handler is called in registration order.  If a
     * transition fails, the slot's `failed` flag is set and the
     * coordinator continues with the next slave (so all failures are
     * reported, not just the first).
     *
     * After a successful transition, the slot's `in_target_state` flag
     * is set and the `on_reached` callback is invoked (if
     * `call_on_reached` is true).
     *
     * @param target           The target EtherCAT state.
     * @param ctx              Transition context.
     * @param call_on_reached  If true, call each slot's on_reached callback
     *                         after a successful transition.
     * @return true if all slaves reached the target state,
     *         false if any failed.
     */
    bool transitionAllTo(ECState target, const TransitionContext& ctx,
                         bool call_on_reached = true);

    /**
     * @brief Transition one slave to the target state.
     *
     * @param slave_index      The slave to transition.
     * @param target           The target EtherCAT state.
     * @param ctx              Transition context.
     * @param call_on_reached  If true, call the slot's on_reached callback.
     * @return true on success, false on failure or if the slave is not
     *         registered.
     */
    bool transitionOne(uint16_t slave_index, ECState target,
                       const TransitionContext& ctx,
                       bool call_on_reached = true);

    /**
     * @brief Transition all slaves and wait for all to confirm via
     *        their in_target_state flag.
     *
     * This is a convenience method that calls `transitionAllTo()` and
     * then `gate.waitAll()`.  It's useful when the in_target_state
     * flag is set asynchronously (e.g. by a processing thread).
     *
     * @param target  The target EtherCAT state.
     * @param ctx     Transition context.
     * @param gate    The gate to wait on.
     * @param st      Stop token for cooperative cancellation.
     * @return true if all slaves reached the target state,
     *         false if any failed or the stop token was requested.
     */
    bool transitionAllAndWait(ECState target, const TransitionContext& ctx,
                              SlaveSyncGate& gate, std::stop_token st);

private:
    std::vector<SlaveSyncSlot*> slots_;
};

// ============================================================================
// Recovery Handler (integrates with SlaveSupervisor)
// ============================================================================

/**
 * @brief Generic recovery handler that dispatches to the coordinator's slots.
 *
 * Implements `ISlaveRecoveryHandler` so the `SlaveSupervisor` can
 * recover any slave registered with the coordinator.
 *
 * When `reinitializeSlave()` is called:
 *   1. The slot's `ready` flag is cleared (exchange thread stops
 *      cycling the group).
 *   2. The slot's flags are reset.
 *   3. The handler's `reinitialize()` is called (full INIT → OP).
 *   4. On success, the slot's `on_reached` callback is invoked
 *      (re-caches PDO buffers, sets `ready` again).
 */
class SyncRecoveryHandler : public ISlaveRecoveryHandler {
public:
    /// Construct with a reference to the coordinator.
    explicit SyncRecoveryHandler(SlaveSyncCoordinator& coord);

    /// @brief Re-initialize the slave from scratch.
    /// @param slave_index  Slave to re-initialize.
    /// @return true if the slave is back in its operational state,
    ///         false on failure or if the slave is not registered.
    bool reinitializeSlave(uint16_t slave_index) override;

private:
    SlaveSyncCoordinator& coord_;
};

} // namespace Util
} // namespace EtherCAT
