#pragma once

// ============================================================================
// EventSource<T> — generic, reusable multi-listener event source
// ----------------------------------------------------------------------------
//
// A minimal, header-only utility for the "register N listeners, notify them
// all when something happens" pattern.  Designed for hot paths where, in the
// common case, *no* listeners are registered: the emit() fast path is a
// single `empty()` check, and the per-event payload is only constructed when
// at least one listener exists.
//
// Key properties:
//   - Multiple listeners, each removable by the handle returned from
//     addListener().
//   - Listeners receive std::shared_ptr<const T> — one immutable copy shared
//     among all listeners, so they cannot accidentally modify the source
//     data and no per-listener copy is made.
//   - No-copy-if-no-listeners: emit() takes a factory callable that builds
//     the shared_ptr; the factory is invoked only when listeners exist.
//   - NOT thread-safe.  Callers must protect access with their own mutex if
//     registration and emission can race.  This keeps the hot path lock-free
//     when the owning object already holds a lock (e.g. FSoEMasterConnection
//     invokes emit() while holding its recursive_mutex_).
//   - Listener reentrancy: listeners are invoked synchronously inside
//     emit().  A listener that mutates the EventSource (add/remove/clear)
//     during dispatch would invalidate the iteration — this is guarded
//     against by copying the listener list before dispatch.
//
// Example:
//   EventSource<std::vector<uint8_t>> events;
//   auto h = events.addListener([](std::shared_ptr<const std::vector<uint8_t>> d) {
//       printf("got %zu bytes\n", d->size());
//   });
//   events.emit([&] {
//       return std::make_shared<const std::vector<uint8_t>>(buf, buf + n);
//   });
//   events.removeListener(h);
// ============================================================================

#include <cstddef>
#include <functional>
#include <memory>
#include <utility>
#include <vector>

namespace Tether::Utils {

template <typename T>
class EventSource {
public:
    using DataType = T;
    using DataPtr = std::shared_ptr<const T>;
    using Listener = std::function<void(DataPtr)>;
    using ListenerHandle = size_t;

    /// Register a listener.  Returns a handle for later removal, or 0 if
    /// the listener is null/empty (in which case nothing is registered).
    ListenerHandle addListener(Listener listener) {
        if (!listener) return 0;
        const ListenerHandle handle = next_handle_++;
        listeners_.emplace_back(handle, std::move(listener));
        return handle;
    }

    /// Remove the listener with the given handle.  Returns true if a
    /// listener was removed, false if the handle was not found.
    bool removeListener(ListenerHandle handle) {
        for (auto it = listeners_.begin(); it != listeners_.end(); ++it) {
            if (it->first == handle) {
                listeners_.erase(it);
                return true;
            }
        }
        return false;
    }

    /// Remove all listeners.
    void clear() noexcept { listeners_.clear(); }

    /// True when no listeners are registered (emit() is a no-op then).
    bool empty() const noexcept { return listeners_.empty(); }

    /// Number of registered listeners.
    std::size_t size() const noexcept { return listeners_.size(); }

    /// Notify all listeners.  @p factory is called only when at least one
    /// listener is registered; the returned shared_ptr is passed to every
    /// listener.  When no listeners exist, factory is not invoked — this is
    /// the no-copy fast path.
    template <typename Factory>
    void emit(Factory&& factory) {
        if (listeners_.empty()) return;
        DataPtr data = factory();
        // Copy the listener list so a listener that (un)registers others
        // during dispatch does not invalidate our iteration.
        std::vector<Listener> snapshot;
        snapshot.reserve(listeners_.size());
        for (const auto& entry : listeners_) {
            snapshot.push_back(entry.second);
        }
        for (const auto& fn : snapshot) {
            fn(data);
        }
    }

private:
    std::vector<std::pair<ListenerHandle, Listener>> listeners_;
    ListenerHandle next_handle_ = 1;
};

} // namespace Tether::Utils
