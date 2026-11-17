#pragma once

/**
 * @file AgentMonitor.hpp
 * @brief Toolkit-agnostic model for an agent-run monitor TUI — owns the
 *        event log, live status state (step/elapsed/chars/activity/
 *        tokens/cost), streamed-content preview, and pending ASK
 *        questions (option menu or free text).  A terminal_ui screen
 *        (AgentMonitorScreen) renders it; any other frontend can drive
 *        the same model.
 *
 * Frame protocol (child → us): INFO/STATUS/STEP/TICK/TOOLFORM/CONTENT/
 * STEPDONE/TOOL/AGENTTEXT/ASK/BUILD/COST/DONE/ERR — see KICODEGEN's
 * report.rs for the producer side.  Outbound: ANSWER/QUIT via the
 * sendFrame callback the host wires to its transport.
 *
 * No ncurses: key handling is semantic (pressEnter/pressChar/...), the
 * screen translates raw key codes.
 */

#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <format>
#include <functional>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include <logging/Logger.hpp>  // Tether::Platform::LogLevel

#include "tether/ui/SelectMenu.hpp"
#include "tether/ui/Spinner.hpp"
#include "tether/ui/TextLog.hpp"

namespace Tether {
namespace UI {

namespace agent_monitor_detail {

inline std::vector<std::string> splitTab(const std::string& s) {
    std::vector<std::string> out;
    size_t pos = 0;
    for (;;) {
        auto t = s.find('\t', pos);
        if (t == std::string::npos) { out.push_back(s.substr(pos)); break; }
        out.push_back(s.substr(pos, t - pos));
        pos = t + 1;
    }
    return out;
}

inline std::vector<std::string> splitLines(const std::string& s) {
    std::vector<std::string> out;
    size_t pos = 0;
    for (;;) {
        auto t = s.find('\n', pos);
        if (t == std::string::npos) { out.push_back(s.substr(pos)); break; }
        out.push_back(s.substr(pos, t - pos));
        pos = t + 1;
    }
    return out;
}

inline std::string fmtK(long v) {
    if (v >= 1000) {
        char b[16];
        snprintf(b, sizeof(b), "%.1fk", v / 1000.0);
        return b;
    }
    return std::to_string(v);
}

} // namespace agent_monitor_detail

class AgentMonitor {
public:
    TextLog log;

    // ---- live status state (read by the renderer) ----
    int  step = 0;
    bool busy = false;          // a model request is in flight
    bool finished = false;      // DONE frame seen
    std::chrono::steady_clock::time_point call_start;
    long chars = 0;             // stream bytes received this call
    std::string forming;        // tool call currently forming
    long in_tok = 0, cached_tok = 0, out_tok = 0;
    std::string cost;           // "≈ $0.086" tail of the COST frame
    std::string activity;       // TICK label — forming tool / model text
    uint64_t tick = 0;          // frame counter for the spinner

    std::string input;          // free-text answer buffer
    bool awaiting = false;      // an ASK frame wants an ANSWER
    std::optional<SelectMenu> ask_menu;  // option questions

    /// Outbound transport — host wires this to its frame sender
    /// (e.g. FramedChild::sendFrame).  May be unset (answers dropped).
    std::function<void(const std::string& kind, const std::string& payload)>
        sendFrame;

    void setWidth(int w) { log.setWidth(w); }

    /// One inbound frame from the child.
    void onFrame(const std::string& kind, const std::string& payload) {
        using namespace agent_monitor_detail;
        if (kind == "INFO") {
            push(payload, Platform::LogLevel::Info);
        } else if (kind == "STATUS") {
            push(payload, Platform::LogLevel::Debug);
        } else if (kind == "STEP") {
            auto v = splitTab(payload);
            step = v.empty() ? 0 : atoi(v[0].c_str());
            busy = true;
            chars = 0;
            forming.clear();
            activity.clear();
            call_start = std::chrono::steady_clock::now();
            flushContent();
            push("── step " + std::to_string(step) + " — request (" +
                 (v.size() > 1 ? v[1] : "?") + " msgs, " +
                 (v.size() > 2 ? v[2] : "?") + " tools)",
                 Platform::LogLevel::Debug);
        } else if (kind == "TICK") {
            auto v = splitTab(payload);
            if (v.size() > 1) chars = atol(v[1].c_str());
            if (v.size() > 3 && !v[3].empty()) activity = v[3];
        } else if (kind == "TOOLFORM") {
            forming = payload;
        } else if (kind == "CONTENT") {
            chars += long(payload.size());
            content_buf_ += payload;
            if (content_buf_.size() > 4000)
                content_buf_.erase(0, content_buf_.size() - 4000);
        } else if (kind == "STEPDONE") {
            auto v = splitTab(payload);
            flushContent();
            busy = false;
            long ms = v.size() > 1 ? atol(v[1].c_str()) : 0;
            std::string fin = v.size() > 2 ? v[2] : "?";
            long pin = v.size() > 3 ? atol(v[3].c_str()) : 0;
            long cch = v.size() > 4 ? atol(v[4].c_str()) : 0;
            long out = v.size() > 5 ? atol(v[5].c_str()) : 0;
            long rea = v.size() > 6 ? atol(v[6].c_str()) : 0;
            std::string label = v.size() > 7 ? v[7] : "";
            in_tok += pin; cached_tok += cch; out_tok += out;
            push("step " + (v.empty() ? "?" : v[0]) + " done in " +
                 std::format("{:.1f}s", ms / 1000.0) + " [" + fin +
                 "]" + (label.empty() ? "" : " — " + label) +
                 " — in " + fmtK(pin) + " (" + fmtK(cch) +
                 " cached), out " + fmtK(out) +
                 (rea > 0 ? " (" + fmtK(rea) + " reasoning)" : ""),
                 Platform::LogLevel::Debug);
            activity.clear();
        } else if (kind == "TOOL") {
            auto v = splitTab(payload);
            std::string line = v.empty() ? "?" : v[0];
            if (v.size() > 1 && !v[1].empty()) line += " " + v[1];
            std::string diff;
            if (v.size() > 2 && !v[2].empty()) diff += " +" + v[2];
            if (v.size() > 3 && !v[3].empty()) diff += " -" + v[3];
            push("  " + line + diff, Platform::LogLevel::Info);
        } else if (kind == "AGENTTEXT") {
            flushContent();
            push(payload, Platform::LogLevel::Info);
        } else if (kind == "ASK") {
            auto v = splitLines(payload);
            flushContent();
            if (!v.empty()) push("? " + v[0], Platform::LogLevel::Warn);
            ask_menu.reset();
            if (v.size() > 1)
                ask_menu.emplace(std::vector<std::string>(
                    v.begin() + 1, v.end()));
            awaiting = true;
            input.clear();
            busy = false;
        } else if (kind == "BUILD") {
            auto v = splitTab(payload);
            bool ok = !v.empty() && v[0] == "1";
            push("build " + std::string(ok ? "✓ " : "✗ ") +
                 (v.size() > 1 ? v[1] : "") + " — " +
                 (v.size() > 2 ? v[2] : ""),
                 ok ? Platform::LogLevel::Info : Platform::LogLevel::Error);
        } else if (kind == "COST") {
            push("cost — " + payload, Platform::LogLevel::Info);
            auto d = payload.find("≈");
            cost = d == std::string::npos ? payload : payload.substr(d);
        } else if (kind == "DONE") {
            auto v = splitTab(payload);
            bool ok = !v.empty() && v[0] == "1";
            flushContent();
            push((v.size() > 1 ? v[1] : "done"),
                 ok ? Platform::LogLevel::Info : Platform::LogLevel::Error);
            finished = true;
            busy = false;
        } else if (kind == "ERR") {
            push("error: " + payload, Platform::LogLevel::Error);
        }
    }

    /// Drained child stderr → dim log lines (stray diagnostics).
    void onStderr(const std::string& err) {
        for (auto& l : agent_monitor_detail::splitLines(err))
            if (!l.empty()) push(l, Platform::LogLevel::Debug);
    }

    /// Child exited — mark once.
    void onBackendExit() {
        if (!log.empty() && log.back().text != "[backend exited]") {
            flushContent();
            push("[backend exited]", Platform::LogLevel::Error);
            busy = false;
        }
    }

    // ---- ASK interaction (semantic keys; the screen translates) ----

    /// While an option ASK is open the screen forwards raw keys here
    /// (SelectMenu speaks the same int codes ncurses delivers).
    /// Returns true while the menu is still open; when it completes the
    /// ANSWER frame is emitted and awaiting clears.
    bool pollAskMenu() {
        if (!awaiting || !ask_menu) return false;
        if (!ask_menu->done()) return true;
        std::string ans, shown;
        if (ask_menu->cancelled()) {
            shown = "(skipped)";
        } else if (ask_menu->selected() >= 0) {
            ans = std::to_string(ask_menu->selected() + 1);
            shown = ask_menu->answer();
        } else {
            ans = shown = ask_menu->customText();
        }
        push("answer> " + shown, Platform::LogLevel::Warn);
        if (sendFrame) sendFrame("ANSWER", ans);
        ask_menu.reset();
        awaiting = false;
        return false;
    }

    /// Enter on a free-text (or custom-answer) question.
    void submitInput() {
        if (!awaiting) return;
        push("answer> " + input, Platform::LogLevel::Warn);
        if (sendFrame) sendFrame("ANSWER", input);
        input.clear();
        awaiting = false;
    }

    /// Esc on a free-text question = skip.
    void skipInput() {
        if (!awaiting) return;
        push("answer> (skipped)", Platform::LogLevel::Warn);
        if (sendFrame) sendFrame("ANSWER", "");
        input.clear();
        awaiting = false;
    }

    void inputChar(char c) {
        if (awaiting && c >= 32 && c < 127) input += c;
    }
    void inputBackspace() {
        if (awaiting && !input.empty()) input.pop_back();
    }

    /// Menu rows currently shown above the status line (0 when closed).
    int menuRows() const {
        return (awaiting && ask_menu) ? ask_menu->itemCount() : 0;
    }

    // ---- rendering helpers ----

    /// The one-line live status: spinner/step/elapsed/chars/activity,
    /// or done/waiting/idle, plus cumulative tokens and cost.
    std::string statusLine() const {
        std::string s;
        if (busy) {
            auto el = std::chrono::duration_cast<std::chrono::seconds>(
                std::chrono::steady_clock::now() - call_start).count();
            s = std::string(spinnerFrame(tick)) +
                " step " + std::to_string(step) + " · " +
                formatDuration(uint64_t(el)) + " · ↓" +
                agent_monitor_detail::fmtK(chars);
            if (!activity.empty()) s += " · " + activity;
            else if (!forming.empty()) s += " · " + forming + "…";
        } else if (finished) {
            s = "● done";
        } else if (awaiting) {
            s = "● waiting for your answer";
        } else {
            s = "● idle";
        }
        if (in_tok || out_tok)
            s += "  |  in " + agent_monitor_detail::fmtK(in_tok) + " (" +
                 agent_monitor_detail::fmtK(cached_tok) + " cached) out " +
                 agent_monitor_detail::fmtK(out_tok);
        if (!cost.empty()) s += "  |  " + cost;
        return s;
    }

private:
    std::string content_buf_;

    void push(const std::string& t, Platform::LogLevel l) {
        log.add(t, l);
    }
    void flushContent() {
        if (!content_buf_.empty()) {
            push(content_buf_, Platform::LogLevel::Verbose);
            content_buf_.clear();
        }
    }
};

} // namespace UI
} // namespace Tether
