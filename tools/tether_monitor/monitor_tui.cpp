/**
 * @file monitor_tui.cpp
 * @brief tether-monitor — ncurses progress UI for agent CLI runs
 *        (generate/update/extract/test/review/generalize-style commands),
 *        built on tether_terminal_ui.
 *
 * Usage (spawned automatically by the agent CLI on a TTY, or standalone):
 *   tether-monitor <agent-exe> <agent-args...>
 *
 * It forks `<exe> --monitor <args...>` and speaks the framed event
 * protocol:  `KIND <len>\n<payload>`  both ways.
 *   out→ : ANSWER <text>, QUIT
 *   in→  : INFO/STATUS/STEP/TICK/TOOLFORM/CONTENT/STEPDONE/TOOL/
 *          AGENTTEXT/ASK/BUILD/DONE/ERR  (+ drained child stderr)
 *
 * Layout: event log fills the body (PgUp/PgDn scroll), the row above
 * the input line is a live status line (spinner, step, elapsed, chars,
 * tool in formation, cumulative tokens), the last row answers ASK
 * prompts — type + Enter, a bare number picks a numbered option.
 */
#include "tether/utils/FramedChild.hpp"
#include "tether/ui/SelectMenu.hpp"
#include "tether/ui/TextLog.hpp"
#include "tether/ui/Spinner.hpp"
#include "tether/terminal_ui/PanelScreen.hpp"
#include "tether/terminal_ui/StyledText.hpp"

#include <ncurses.h>

#include <atomic>
#include <chrono>
#include <cstdio>
#include <filesystem>
#include <optional>
#include <string>
#include <vector>

namespace TUI = Tether::TUI;
using Tether::Platform::LogLevel;

namespace {

std::vector<std::string> splitTab(const std::string& s) {
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

std::vector<std::string> splitLines(const std::string& s) {
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

std::string fmtK(long v) {
    if (v >= 1000) {
        char b[16];
        snprintf(b, sizeof(b), "%.1fk", v / 1000.0);
        return b;
    }
    return std::to_string(v);
}

struct AgentState {
    int  step = 0;
    bool busy = false;          // a model request is in flight
    bool finished = false;      // DONE frame seen
    std::chrono::steady_clock::time_point call_start;
    long chars = 0;             // stream bytes received this call
    std::string forming;        // tool call currently forming
    long in_tok = 0, cached_tok = 0, out_tok = 0;
    std::string cost;           // "≈ $0.086" tail of the COST frame
    std::string activity;       // TICK label — forming tool / model's text line
    std::string content_buf;    // streamed text not yet committed to log
};

} // namespace

int main(int argc, char** argv) {
    if (argc < 3) {
        fprintf(stderr,
            "usage: %s <agent-exe> <agent-args...>\n"
            "  e.g. %s ./agent generate model.slx --out gen\n",
            argv[0], argv[0]);
        return 2;
    }
    const std::string exe = argv[1];
    std::vector<std::string> args = {"--monitor"};
    for (int i = 2; i < argc; ++i) args.push_back(argv[i]);

    Tether::Utils::FramedChild child;
    Tether::Utils::FramedChildOpts copts;
    copts.capture_stderr = true;    // stray diagnostics → log pane
    if (!child.spawn(exe, args, copts)) {
        fprintf(stderr, "failed to spawn %s --monitor\n", exe.c_str());
        return 1;
    }

    Tether::UI::TextLog log;
    AgentState st;
    std::string input;
    bool awaiting = false;        // an ASK frame wants an ANSWER
    std::optional<Tether::UI::SelectMenu> ask_menu;  // option questions
    uint64_t tick = 0;

    auto push = [&](const std::string& t, LogLevel l) {
        log.setWidth(std::max(20, COLS - 4));
        log.add(t, l);
    };
    auto flush_content = [&] {
        if (!st.content_buf.empty()) {
            push(st.content_buf, LogLevel::Verbose);
            st.content_buf.clear();
        }
    };

    TUI::PanelScreen screen(std::filesystem::path(exe).filename().string() +
            " — " + args[1] + " " +
            (args.size() > 2 ? args.back() : ""), TUI::PanelScreenHooks{
        .onTick = [&] {
            ++tick;
            for (auto& f : child.poll()) {
                const auto& k = f.kind;
                const auto& p = f.payload;
                if (k == "INFO") {
                    push(p, LogLevel::Info);
                } else if (k == "STATUS") {
                    // free-text status override — fold into INFO
                    push(p, LogLevel::Debug);
                } else if (k == "STEP") {
                    auto v = splitTab(p);
                    st.step = v.empty() ? 0 : atoi(v[0].c_str());
                    st.busy = true;
                    st.chars = 0;
                    st.forming.clear();
                    st.activity.clear();
                    st.call_start = std::chrono::steady_clock::now();
                    flush_content();
                    push("── step " + std::to_string(st.step) +
                         " — request (" +
                         (v.size() > 1 ? v[1] : "?") + " msgs, " +
                         (v.size() > 2 ? v[2] : "?") + " tools)",
                         LogLevel::Debug);
                } else if (k == "TICK") {
                    auto v = splitTab(p);
                    if (v.size() > 1) st.chars = atol(v[1].c_str());
                    if (v.size() > 3 && !v[3].empty()) st.activity = v[3];
                } else if (k == "TOOLFORM") {
                    st.forming = p;
                } else if (k == "CONTENT") {
                    st.chars += long(p.size());
                    st.content_buf += p;
                    // keep the preview bounded
                    if (st.content_buf.size() > 4000)
                        st.content_buf.erase(0, st.content_buf.size() - 4000);
                } else if (k == "STEPDONE") {
                    auto v = splitTab(p);
                    flush_content();
                    st.busy = false;
                    long ms = v.size() > 1 ? atol(v[1].c_str()) : 0;
                    std::string fin = v.size() > 2 ? v[2] : "?";
                    long pin = v.size() > 3 ? atol(v[3].c_str()) : 0;
                    long cch = v.size() > 4 ? atol(v[4].c_str()) : 0;
                    long out = v.size() > 5 ? atol(v[5].c_str()) : 0;
                    long rea = v.size() > 6 ? atol(v[6].c_str()) : 0;
                    std::string label = v.size() > 7 ? v[7] : "";
                    st.in_tok += pin; st.cached_tok += cch; st.out_tok += out;
                    push("step " + (v.empty() ? "?" : v[0]) + " done in " +
                         std::format("%.1fs", ms / 1000.0) + " [" + fin +
                         "]" + (label.empty() ? "" : " — " + label) +
                         " — in " + fmtK(pin) + " (" + fmtK(cch) +
                         " cached), out " + fmtK(out) +
                         (rea > 0 ? " (" + fmtK(rea) + " reasoning)" : ""),
                         LogLevel::Debug);
                    st.activity.clear();
                } else if (k == "TOOL") {
                    auto v = splitTab(p);
                    std::string line = v.empty() ? "?" : v[0];
                    if (v.size() > 1 && !v[1].empty()) line += " " + v[1];
                    std::string diff;
                    if (v.size() > 2 && !v[2].empty()) diff += " +" + v[2];
                    if (v.size() > 3 && !v[3].empty()) diff += " -" + v[3];
                    push("  " + line + diff, LogLevel::Info);
                } else if (k == "AGENTTEXT") {
                    flush_content();
                    push(p, LogLevel::Info);
                } else if (k == "ASK") {
                    auto v = splitLines(p);
                    flush_content();
                    if (!v.empty()) push("? " + v[0], LogLevel::Warn);
                    ask_menu.reset();
                    if (v.size() > 1)
                        ask_menu.emplace(std::vector<std::string>(
                            v.begin() + 1, v.end()));
                    awaiting = true;
                    input.clear();
                    st.busy = false;
                } else if (k == "BUILD") {
                    auto v = splitTab(p);
                    bool ok = !v.empty() && v[0] == "1";
                    push("build " + std::string(ok ? "✓ " : "✗ ") +
                         (v.size() > 1 ? v[1] : "") + " — " +
                         (v.size() > 2 ? v[2] : ""),
                         ok ? LogLevel::Info : LogLevel::Error);
                } else if (k == "COST") {
                    push("cost — " + p, LogLevel::Info);
                    auto d = p.find("≈");
                    st.cost = d == std::string::npos ? p : p.substr(d);
                } else if (k == "DONE") {
                    auto v = splitTab(p);
                    bool ok = !v.empty() && v[0] == "1";
                    flush_content();
                    push((v.size() > 1 ? v[1] : "done"),
                         ok ? LogLevel::Info : LogLevel::Error);
                    st.finished = true;
                    st.busy = false;
                } else if (k == "ERR") {
                    push("error: " + p, LogLevel::Error);
                }
            }
            // Drained child stderr → dim log lines (stray diagnostics,
            // panics — never silently lost).
            std::string err = child.drainErr();
            if (!err.empty())
                for (auto& l : splitLines(err))
                    if (!l.empty()) push(l, LogLevel::Debug);
            if (!child.alive() && !log.empty()
                    && log.back().text != "[backend exited]") {
                flush_content();
                push("[backend exited]", LogLevel::Error);
                st.busy = false;
            }
        },
        .renderBody = [&](int top, int bottom) {
            // Body: [top, bottom-2-menu_h) log, then the option menu
            // block while one is open, bottom-2 status, bottom-1 input.
            int menu_h = (awaiting && ask_menu) ? ask_menu->itemCount() : 0;
            int h = bottom - top - 2 - menu_h;
            if (h > 0) {
                int row = top;
                for (const auto* lp : log.visible(h)) {
                    move(row, 0); clrtoeol();
                    short pair = TUI::PalNone;
                    switch (lp->level) {
                        case LogLevel::Error: pair = 2; break;
                        case LogLevel::Warn:  pair = 3; break;
                        case LogLevel::Debug:
                        case LogLevel::Verbose: pair = 8; break;
                        default: pair = TUI::PalNone; break;
                    }
                    if (pair != TUI::PalNone) attron(COLOR_PAIR(pair));
                    mvprintw(row, 1, "%s", lp->text.c_str());
                    if (pair != TUI::PalNone) attroff(COLOR_PAIR(pair));
                    ++row;
                }
            }

            // Option menu block — live › highlight, sits above the
            // status line while an ASK question is open.
            if (awaiting && ask_menu) {
                for (int i = 0; i < ask_menu->itemCount(); ++i) {
                    int my = bottom - 2 - menu_h + i;
                    if (my < top) continue;
                    move(my, 0); clrtoeol();
                    std::string lbl = ask_menu->rowLabel(i);
                    if (int(lbl.size()) > COLS - 6)
                        lbl = lbl.substr(0, size_t(COLS - 7)) + "…";
                    if (i == ask_menu->cursor()) {
                        attron(A_REVERSE | A_BOLD);
                        mvprintw(my, 1, " › %s", lbl.c_str());
                        attroff(A_REVERSE | A_BOLD);
                    } else if (ask_menu->isCustomRow(i)) {
                        attron(COLOR_PAIR(8));
                        mvprintw(my, 3, "%s", lbl.c_str());
                        attroff(COLOR_PAIR(8));
                    } else {
                        mvprintw(my, 3, "%s", lbl.c_str());
                    }
                }
            }

            // Status line.
            int sy = bottom - 2;
            if (sy >= top) {
                move(sy, 0); clrtoeol();
                attron(A_REVERSE);
                std::string s;
                if (st.busy) {
                    auto el = std::chrono::duration_cast<std::chrono::seconds>(
                        std::chrono::steady_clock::now() - st.call_start).count();
                    s = std::string(Tether::UI::spinnerFrame(tick)) +
                        " step " + std::to_string(st.step) + " · " +
                        Tether::UI::formatDuration(uint64_t(el)) + " · ↓" +
                        fmtK(st.chars);
                    if (!st.activity.empty()) s += " · " + st.activity;
                    else if (!st.forming.empty()) s += " · " + st.forming + "…";
                } else if (st.finished) {
                    s = "● done";
                } else if (awaiting) {
                    s = "● waiting for your answer";
                } else {
                    s = "● idle";
                }
                if (st.in_tok || st.out_tok)
                    s += "  |  in " + fmtK(st.in_tok) + " (" +
                         fmtK(st.cached_tok) + " cached) out " +
                         fmtK(st.out_tok);
                if (!st.cost.empty()) s += "  |  " + st.cost;
                mvprintw(sy, 0, "%s", s.c_str());
                for (int x = int(s.size()); x < COLS; ++x) mvaddch(sy, x, ' ');
                attroff(A_REVERSE);
            }

            // Input row.
            int y = bottom - 1;
            if (y >= top) {
                move(y, 0); clrtoeol();
                if (awaiting) {
                    if (ask_menu && !ask_menu->editingCustom()) {
                        attron(COLOR_PAIR(8));
                        mvprintw(y, 1,
                                 "(↑/↓ move · Enter select · Esc skip · digits jump)");
                        attroff(COLOR_PAIR(8));
                    } else {
                        attron(A_BOLD);
                        mvprintw(y, 0, "answer>");
                        attroff(A_BOLD);
                        const std::string& buf =
                            (ask_menu && ask_menu->editingCustom())
                                ? ask_menu->customInput().buffer() : input;
                        std::string view = buf;
                        int maxw = COLS - 10;
                        if (int(view.size()) > maxw)
                            view = "…" + view.substr(view.size() - size_t(maxw - 1));
                        mvprintw(y, 9, "%s", view.c_str());
                        move(y, 9 + int(view.size()));
                    }
                } else {
                    attron(COLOR_PAIR(8));
                    mvprintw(y, 1, "PgUp/PgDn: scroll   q: quit");
                    attroff(COLOR_PAIR(8));
                }
            }
        },
        .showLogView = false,
        .onKey = [&](int ch) -> bool {
            // An open option menu is modal — every key goes to it.
            if (awaiting && ask_menu) {
                ask_menu->handleKey(ch);
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
                push("answer> " + shown, LogLevel::Warn);
                child.sendFrame("ANSWER", ans);
                ask_menu.reset();
                awaiting = false;
                return true;
            }
            switch (ch) {
                case '\n': case '\r': case KEY_ENTER:
                    if (awaiting) {
                        push("answer> " + input, LogLevel::Warn);
                        child.sendFrame("ANSWER", input);
                        input.clear();
                        awaiting = false;
                    }
                    return true;
                case 27:
                    // Esc during a free-text question = skip, not quit.
                    if (awaiting) {
                        push("answer> (skipped)", LogLevel::Warn);
                        child.sendFrame("ANSWER", "");
                        input.clear();
                        awaiting = false;
                        return true;
                    }
                    return false;
                case KEY_BACKSPACE: case 127: case 8:
                    if (!input.empty()) input.pop_back();
                    return true;
                case KEY_PPAGE: log.scrollBy(10); return true;
                case KEY_NPAGE: log.scrollBy(-10); return true;
                case KEY_END:   log.scrollEnd(); return true;
                default:
                    if (awaiting && ch >= 32 && ch < 127) {
                        input += char(ch);
                        return true;
                    }
                    return false;
            }
        },
        // While a question is open the menu/input owns every key —
        // including digits, which the screen would otherwise treat as
        // view switching.
        .wantsKey = [&](int) -> bool { return awaiting; },
        .keyHints = "PgUp/PgDn: scroll   q: quit",
    });

    std::atomic<bool> cancel{false};
    screen.run(cancel);
    child.quit();
    return 0;
}
