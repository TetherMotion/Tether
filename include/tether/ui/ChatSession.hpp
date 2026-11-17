#pragma once

/**
 * @file ChatSession.hpp
 * @brief Toolkit-agnostic model for an agent-chat TUI — owns the
 *        transcript log, the input line, and busy/ready state.  Frames
 *        from the child (`READY`/`TOOL`/`TEXT`/`COST`/`AWAIT`/`TURN`/`ERR`)
 *        mutate the model; the host's transport sends `SEND`/`QUIT`
 *        through the sendFrame callback.  A terminal_ui screen
 *        (ChatScreen) renders it; other frontends can drive it too.
 *
 * No ncurses: key handling is semantic.
 */

#include <string>
#include <functional>

#include <logging/Logger.hpp>  // Tether::Platform::LogLevel

#include "tether/ui/TextLog.hpp"

namespace Tether {
namespace UI {

class ChatSession {
public:
    TextLog transcript;

    std::string input;
    bool busy = false;    // a turn is in flight
    bool ready = false;   // READY frame seen

    /// Outbound transport — host wires this to its frame sender.
    std::function<void(const std::string& kind, const std::string& payload)>
        sendFrame;

    void setWidth(int w) { transcript.setWidth(w); }

    /// One inbound frame from the child.
    void onFrame(const std::string& kind, const std::string& payload) {
        if (kind == "READY") {
            ready = true;
            push("[session " + payload + "] — ask about the code",
                 Platform::LogLevel::Debug);
        } else if (kind == "TOOL") {
            push("  [tool] " + payload, Platform::LogLevel::Debug);
        } else if (kind == "TEXT") {
            push(payload, Platform::LogLevel::Info);
        } else if (kind == "COST") {
            if (!payload.empty())
                push("  [cost] " + payload, Platform::LogLevel::Debug);
        } else if (kind == "ERR") {
            push("[error] " + payload, Platform::LogLevel::Error);
        } else if (kind == "AWAIT") {
            push("(agent paused — message to continue)",
                 Platform::LogLevel::Warn);
        } else if (kind == "TURN") {
            busy = false;
        }
    }

    /// Child exited — mark once.
    void onBackendExit() {
        if (!transcript.empty()
                && transcript.back().text != "[backend exited]")
            push("[backend exited]", Platform::LogLevel::Error);
    }

    /// Enter: echo + SEND (only while the backend is ready).
    void submitInput() {
        if (input.empty() || !ready) return;
        push("> " + input, Platform::LogLevel::Warn);
        if (sendFrame) sendFrame("SEND", input);
        input.clear();
        busy = true;
        transcript.scrollEnd();
    }

    void inputChar(char c) {
        if (c >= 32 && c < 127) input += c;
    }
    void inputBackspace() {
        if (!input.empty()) input.pop_back();
    }

private:
    void push(const std::string& t, Platform::LogLevel l) {
        transcript.add(t, l);
    }
};

} // namespace UI
} // namespace Tether
