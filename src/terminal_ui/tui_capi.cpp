/**
 * @file tui_capi.cpp
 * @brief C ABI implementation for tui_capi.h — see the header for the
 *        contract.  Each handle owns a UI thread running the matching
 *        PanelScreen; events arrive through a mutex-protected queue.
 */

#include "tether/tui_capi.h"

#include <atomic>
#include <deque>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "tether/terminal_ui/AgentMonitorScreen.hpp"
#include "tether/terminal_ui/ChatScreen.hpp"
#include "tether/ui/AgentMonitor.hpp"
#include "tether/ui/ChatSession.hpp"
#include "tether/utils/FramePump.hpp"

namespace {

/// In-process frame source: Rust/FFI callers push() events, the screen
/// thread drains them via poll()/drainErr() each tick.
struct QueuePump : Tether::Utils::IFramePump {
    mutable std::mutex m;
    std::deque<Tether::Utils::Frame> q;
    std::string err;
    bool closed = false;

    void push(const char* kind, const char* payload) {
        std::lock_guard lk(m);
        if (closed) return;
        q.push_back({kind ? kind : "", payload ? payload : ""});
    }

    void pushErr(const char* text) {
        if (!text || !*text) return;
        std::lock_guard lk(m);
        if (closed) return;
        err += text;
        if (err.empty() || err.back() != '\n') err += '\n';
    }

    std::vector<Tether::Utils::Frame> poll() override {
        std::lock_guard lk(m);
        std::vector<Tether::Utils::Frame> out(q.begin(), q.end());
        q.clear();
        return out;
    }

    std::string drainErr() override {
        std::lock_guard lk(m);
        std::string out;
        out.swap(err);
        return out;
    }

    bool alive() const override {
        std::lock_guard lk(m);
        return !closed;
    }

    void shutdown() {
        std::lock_guard lk(m);
        closed = true;
    }
};

struct MonitorImpl {
    QueuePump pump;
    Tether::UI::AgentMonitor mon;
    std::thread th;
    std::atomic<bool> cancel{false};

    ~MonitorImpl() { close(); }

    void close() {
        cancel.store(true);
        if (th.joinable()) th.join();
        pump.shutdown();
    }
};

struct ChatImpl {
    QueuePump pump;
    Tether::UI::ChatSession ses;
    std::thread th;
    std::atomic<bool> cancel{false};

    ~ChatImpl() { close(); }

    void close() {
        cancel.store(true);
        if (th.joinable()) th.join();
        pump.shutdown();
    }
};

} // namespace

extern "C" {

TetherAgentMonitor* tether_monitor_start(const char* title,
                                         TetherTuiAnswerFn on_answer,
                                         void* user) {
    auto* impl = new MonitorImpl();
    impl->mon.sendFrame =
        [on_answer, user](const std::string& kind, const std::string& payload) {
            if (kind == "ANSWER" && on_answer)
                on_answer(payload.c_str(), user);
        };
    std::string t = title ? title : "agent monitor";
    impl->th = std::thread([impl, t] {
        Tether::TUI::AgentMonitorScreen screen(t, impl->pump, impl->mon);
        screen.run(impl->cancel);
        // UI ended (user quit or close()): unblock any pending ASK with
        // the empty/default answer — same semantics as monitor-mode QUIT.
        impl->mon.sendFrame("ANSWER", "");
    });
    return reinterpret_cast<TetherAgentMonitor*>(impl);
}

void tether_monitor_push(TetherAgentMonitor* mon,
                         const char* kind, const char* payload) {
    if (mon) reinterpret_cast<MonitorImpl*>(mon)->pump.push(kind, payload);
}

void tether_monitor_stderr(TetherAgentMonitor* mon, const char* text) {
    if (mon) reinterpret_cast<MonitorImpl*>(mon)->pump.pushErr(text);
}

void tether_monitor_close(TetherAgentMonitor* mon) {
    if (!mon) return;
    auto* impl = reinterpret_cast<MonitorImpl*>(mon);
    impl->close();
    delete impl;
}

TetherAgentChat* tether_chat_start(const char* title,
                                   TetherTuiSendFn on_send, void* user) {
    auto* impl = new ChatImpl();
    impl->ses.sendFrame =
        [on_send, user](const std::string& kind, const std::string& payload) {
            if (kind == "SEND" && on_send)
                on_send(payload.c_str(), user);
            else if (kind == "CLOSED" && on_send)
                on_send(nullptr, user);   // session ended — see header
        };
    std::string t = title ? title : "agent chat";
    impl->th = std::thread([impl, t] {
        Tether::TUI::ChatScreen screen(t, impl->pump, impl->ses);
        screen.run(impl->cancel);
        // UI ended: notify the host so a blocked message loop exits.
        impl->ses.sendFrame("CLOSED", "");
    });
    return reinterpret_cast<TetherAgentChat*>(impl);
}

void tether_chat_push(TetherAgentChat* chat,
                      const char* kind, const char* payload) {
    if (chat) reinterpret_cast<ChatImpl*>(chat)->pump.push(kind, payload);
}

void tether_chat_close(TetherAgentChat* chat) {
    if (!chat) return;
    auto* impl = reinterpret_cast<ChatImpl*>(chat);
    impl->close();
    delete impl;
}

} // extern "C"
