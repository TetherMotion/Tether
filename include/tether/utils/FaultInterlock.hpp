/**
 * @file FaultInterlock.hpp
 * @brief "One faults, all stop" coordination for a group of axes or drives
 *
 * A FaultInterlock latches the first fault reported by any member and marks
 * every *other* member inhibited.  Members typically poll their
 * `Member::inhibited` flag on their control cycle and apply whatever
 * safe-stop policy fits the drive (e.g. CiA-402 QuickStop then
 * DisableVoltage written into the RxPDO controlword).
 *
 * Design notes:
 * - First fault wins: `reportFault()` is idempotent, `faultSource()` keeps
 *   the member that latched the interlock.
 * - The faulting member itself is never marked inhibited — its drive has
 *   already removed output; the interlock exists to stop the *healthy*
 *   axes before a higher-level shutdown propagates.
 * - An optional per-member `on_inhibit` callback runs on the reporting
 *   thread once per inhibition.  Keep it realtime-cheap (set an atomic,
 *   push to a lock-free ring) — it is invoked on whatever thread called
 *   reportFault(), which is usually a motion/control thread.
 * - `release()` / `releaseAll()` clear the inhibit flags; `releaseAll()`
 *   also clears the latch so the group is armed for the next fault.
 *
 * The interlock is deliberately drive-agnostic: it owns no PDO, SDO or
 * controlword logic.  Policies live in the members' cyclic code.
 *
 * Example (CiA-402 axis group):
 * @code
 *   Tether::Utils::FaultInterlock interlock;
 *   auto member = interlock.add(slave_index);
 *   // in the axis's cyclic update:
 *   if (member->inhibited.load(std::memory_order_acquire))
 *       write_quickstop_then_disable_voltage();
 *   // in the axis's fault detection:
 *   interlock.reportFault(slave_index);
 *   // in a fault-reset handler:
 *   interlock.releaseAll();
 * @endcode
 */

#pragma once

#include <atomic>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <vector>

namespace Tether {
namespace Utils {

class FaultInterlock {
public:
    using MemberId = uint16_t;

    /// Invoked once when a member becomes inhibited, on the reportFault()
    /// caller's thread.  Must not block.
    using InhibitCallback = std::function<void(MemberId member,
                                               MemberId source)>;

    /// Registration state for one member of the group.  Members read
    /// `inhibited` / `source` directly — plain atomic loads, RT-safe.
    struct Member {
        MemberId id;
        InhibitCallback on_inhibit;
        std::atomic<bool> inhibited{false};
        std::atomic<MemberId> source{0};
    };

    /// Register a member.  Idempotent by id — re-adding an existing id
    /// returns the same Member (the callback is replaced if given).
    std::shared_ptr<Member> add(MemberId id, InhibitCallback cb = {})
    {
        std::lock_guard<std::mutex> lk(mutex_);
        for (auto& m : members_) {
            if (m->id == id) {
                if (cb) m->on_inhibit = cb;
                return m;
            }
        }
        auto m = std::make_shared<Member>();
        m->id = id;
        m->on_inhibit = cb;
        members_.push_back(m);
        return m;
    }

    bool remove(MemberId id)
    {
        std::lock_guard<std::mutex> lk(mutex_);
        for (auto it = members_.begin(); it != members_.end(); ++it) {
            if ((*it)->id == id) {
                members_.erase(it);
                return true;
            }
        }
        return false;
    }

    /// Report a fault on `source`.  Latches the first fault source and
    /// marks all other members inhibited (their on_inhibit callbacks run
    /// on this thread, outside the internal lock).  Returns true when this
    /// call won the latch (i.e. it is the first fault in the group).
    bool reportFault(MemberId source)
    {
        bool expected = false;
        const bool first =
            latched_.compare_exchange_strong(expected, true,
                                           std::memory_order_acq_rel);
        if (first)
            fault_source_.store(source, std::memory_order_release);

        std::vector<std::shared_ptr<Member>> members;
        {
            std::lock_guard<std::mutex> lk(mutex_);
            members = members_;
        }
        for (auto& m : members) {
            if (m->id == source)
                continue;
            m->source.store(source, std::memory_order_relaxed);
            const bool was_inhibited =
                m->inhibited.exchange(true, std::memory_order_acq_rel);
            if (!was_inhibited && m->on_inhibit)
                m->on_inhibit(m->id, source);
        }
        return first;
    }

    /// Manually inhibit one member (same effect as a peer fault).
    void inhibit(MemberId id, MemberId source)
    {
        std::vector<std::shared_ptr<Member>> members;
        {
            std::lock_guard<std::mutex> lk(mutex_);
            members = members_;
        }
        for (auto& m : members) {
            if (m->id != id)
                continue;
            m->source.store(source, std::memory_order_relaxed);
            const bool was_inhibited =
                m->inhibited.exchange(true, std::memory_order_acq_rel);
            if (!was_inhibited && m->on_inhibit)
                m->on_inhibit(m->id, source);
        }
    }

    /// Clear one member's inhibit flag.  Does not clear the latch.
    void release(MemberId id)
    {
        std::vector<std::shared_ptr<Member>> members;
        {
            std::lock_guard<std::mutex> lk(mutex_);
            members = members_;
        }
        for (auto& m : members) {
            if (m->id == id)
                m->inhibited.store(false, std::memory_order_release);
        }
    }

    /// Clear every member's inhibit flag and re-arm the fault latch.
    void releaseAll()
    {
        std::vector<std::shared_ptr<Member>> members;
        {
            std::lock_guard<std::mutex> lk(mutex_);
            members = members_;
        }
        for (auto& m : members)
            m->inhibited.store(false, std::memory_order_release);
        fault_source_.store(0, std::memory_order_release);
        latched_.store(false, std::memory_order_release);
    }

    bool isInhibited(MemberId id) const
    {
        std::lock_guard<std::mutex> lk(mutex_);
        for (const auto& m : members_) {
            if (m->id == id)
                return m->inhibited.load(std::memory_order_acquire);
        }
        return false;
    }

    /// True once any member has reported a fault (until releaseAll()).
    bool latched() const { return latched_.load(std::memory_order_acquire); }

    /// The member that first latched the interlock (0 when released).
    MemberId faultSource() const
    {
        return fault_source_.load(std::memory_order_acquire);
    }

    size_t size() const
    {
        std::lock_guard<std::mutex> lk(mutex_);
        return members_.size();
    }

private:
    mutable std::mutex mutex_;  // guards members_ only; flags are atomic
    std::vector<std::shared_ptr<Member>> members_;
    std::atomic<bool> latched_{false};
    std::atomic<MemberId> fault_source_{0};
};

} // namespace Utils
} // namespace Tether
