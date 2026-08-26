/**
 * @file SignalHandler.hpp
 * @brief RAII signal handler for cooperative cancellation (C++20)
 *
 * Uses std::stop_source / std::stop_token for cooperative cancellation,
 * integrating naturally with std::jthread.  On SIGINT/SIGTERM, calls
 * stop_source.request_stop() and optionally invokes a user callback
 * (e.g. EtherCAT::Master::requestCancel).
 *
 * Header-only — does not depend on any Tether library.
 *
 * Usage with std::jthread:
 *
 *   Tether::Utils::SignalHandler sig;   // installs handlers, owns stop_source
 *
 *   std::jthread pdo_thread([&](std::stop_token st) {
 *       while (!st.stop_requested()) { ... }
 *   });
 *
 *   sig.registerMaster(master);         // optional: cancel master on signal
 *
 *   // main loop:
 *   while (!sig.stop_requested()) { ... }
 *
 *   // On signal: stop_source is triggered, all jthreads checking their
 *   // stop_token will exit.  SignalHandler's destructor restores handlers.
 *
 * Sharing the stop_source with code that uses atomic<bool>:
 *
 *   Tether::Utils::SignalHandler sig;
 *   // sig.cancelled() returns true after a signal
 *   // sig.stopToken() returns the std::stop_token for passing to jthreads
 */

#pragma once

#include <atomic>
#include <csignal>
#include <functional>
#include <stop_token>

namespace EtherCAT { class Master; }

namespace Tether {
namespace Utils {

/// RAII signal handler that triggers a std::stop_source on SIGINT/SIGTERM.
///
/// On construction, installs signal handlers for SIGINT and SIGTERM.
/// On destruction, restores the previous handlers.
/// The stop_source can be queried via stop_requested() / stopToken(), and
/// is intended to be shared with std::jthread instances for cooperative
/// cancellation.
class SignalHandler {
public:
    /// Create with an internally-owned stop_source.
    explicit SignalHandler()
        : stop_source_(owned_source_) {
        install();
    }

    /// Create with an externally-owned stop_source (e.g. one shared with
    /// jthreads created before the SignalHandler).  On signal,
    /// externalSource.request_stop() is called.
    explicit SignalHandler(std::stop_source& externalSource)
        : stop_source_(externalSource) {
        install();
    }

    /// Backward-compatible constructor: sets the given atomic<bool> to true
    /// on signal.  If install_signals is true (default), installs SIGINT/
    /// SIGTERM handlers; if false, the caller is responsible for signal
    /// handling (the atomic is still set if a signal was already delivered
    /// via a previously installed handler — but typically install_signals
    /// should be true).
    explicit SignalHandler(std::atomic<bool>& cancel_flag,
                           bool install_signals = true)
        : stop_source_(owned_source_), legacy_cancel_flag_(&cancel_flag) {
        if (install_signals) install();
    }

    ~SignalHandler() {
        uninstall();
    }

    // Non-copyable, non-movable (manages global signal handler state)
    SignalHandler(const SignalHandler&) = delete;
    SignalHandler& operator=(const SignalHandler&) = delete;
    SignalHandler(SignalHandler&&) = delete;
    SignalHandler& operator=(SignalHandler&&) = delete;

    /// Returns true if stop was requested (signal received).
    bool stop_requested() const {
        return stop_source_.stop_requested();
    }

    /// Returns the stop_token for this stop_source.  Pass this to
    /// std::jthread lambdas or use it directly in polling loops.
    std::stop_token stopToken() const {
        return stop_source_.get_token();
    }

    /// Returns a reference to the stop_source, allowing manual request_stop().
    std::stop_source& stopSource() { return stop_source_; }

    /// Request stop manually (same effect as receiving a signal).
    void requestStop() { stop_source_.request_stop(); }

    /// Set a callback invoked when the signal fires (in addition to
    /// request_stop()).  Use this to call Master::requestCancel() etc.
    /// Must be set before the signal arrives; not thread-safe with respect
    /// to signal delivery.
    void setCancelCallback(std::function<void()> cb) {
        callback_ = std::move(cb);
    }

private:
    std::stop_source owned_source_{};
    std::stop_source& stop_source_;
    std::atomic<bool>* legacy_cancel_flag_{nullptr};

    // Global state — only one SignalHandler should be active at a time.
    static std::stop_source* g_stop_source;
    static std::atomic<bool>* g_legacy_cancel_flag;
    static std::function<void()>* g_callback;
    static struct sigaction g_old_int;
    static struct sigaction g_old_term;
    static bool g_installed;

    std::function<void()> callback_;

    static void handleSignal(int) {
        if (g_stop_source) g_stop_source->request_stop();
        if (g_legacy_cancel_flag) g_legacy_cancel_flag->store(true, std::memory_order_relaxed);
        if (g_callback && *g_callback) (*g_callback)();
    }

    void install() {
        g_stop_source = &stop_source_;
        g_legacy_cancel_flag = legacy_cancel_flag_;
        g_callback = &callback_;

        struct sigaction sa{};
        sa.sa_handler = &SignalHandler::handleSignal;
        sigemptyset(&sa.sa_mask);
        sa.sa_flags = 0;

        sigaction(SIGINT, &sa, &g_old_int);
        sigaction(SIGTERM, &sa, &g_old_term);
        g_installed = true;
    }

    void uninstall() {
        if (g_installed) {
            sigaction(SIGINT, &g_old_int, nullptr);
            sigaction(SIGTERM, &g_old_term, nullptr);
            g_installed = false;
            g_stop_source = nullptr;
            g_legacy_cancel_flag = nullptr;
            g_callback = nullptr;
        }
    }
};

// Static member definitions (zero-initialized at program start)
inline std::stop_source* SignalHandler::g_stop_source = nullptr;
inline std::atomic<bool>* SignalHandler::g_legacy_cancel_flag = nullptr;
inline std::function<void()>* SignalHandler::g_callback = nullptr;
inline struct sigaction SignalHandler::g_old_int{};
inline struct sigaction SignalHandler::g_old_term{};
inline bool SignalHandler::g_installed = false;

} // namespace Utils
} // namespace Tether
