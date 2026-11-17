/**
 * @file chat_tui.cpp
 * @brief Chat TUI for agent CLIs — a devin-CLI-style conversation with the
 *        generated code, built on tether_terminal_ui.
 *
 * Usage (spawned by the agent CLI's `chat` command, or standalone):
 *   tether-chat <agent-exe> <repo_root> <live_dir>
 *
 * It forks `<exe> chat --rpc --repo <repo> --out <live>` and talks the
 * framed stdio protocol:  `KIND <len>\n<payload>`  both ways.
 *   out→ : SEND <text>, QUIT          in→ : READY/TOOL/TEXT/COST/AWAIT/TURN/ERR
 *
 * Layout: transcript fills the body (scrollable via the screen's log
 * pane), the last body row is the input line — printable keys type,
 * Enter sends, PgUp/PgDn scroll the transcript, q/Ctrl-D at an empty
 * prompt quits.
 */
#include "tether/utils/FramedChild.hpp"
#include "tether/ui/TextLog.hpp"
#include "tether/terminal_ui/PanelScreen.hpp"
#include "tether/terminal_ui/StyledText.hpp"

#include <ncurses.h>
#undef OK
#undef ERR

#include <atomic>
#include <cstdio>
#include <filesystem>
#include <string>
#include <vector>

namespace TUI = Tether::TUI;

int main(int argc, char** argv) {
    if (argc < 4) {
        fprintf(stderr,
            "usage: %s <agent-exe> <repo_root> <live_dir>\n", argv[0]);
        return 2;
    }
    const std::string exe = argv[1], repo = argv[2], live = argv[3];

    Tether::Utils::FramedChild child;
    Tether::Utils::FramedChildOpts copts;
    copts.capture_stderr = false;   // stderr passes through — logs visible on exit
    if (!child.spawn(exe, {"chat", "--rpc", "--repo", repo, "--out", live},
                     copts)) {
        fprintf(stderr, "failed to spawn %s chat --rpc\n", exe.c_str());
        return 1;
    }

    // Transcript state ------------------------------------------------
    Tether::UI::TextLog transcript;
    std::string input;
    bool busy = false;        // a turn is in flight
    bool ready = false;       // READY frame seen

    auto push = [&](const std::string& t, Tether::Platform::LogLevel l) {
        transcript.setWidth(std::max(20, COLS - 4));
        transcript.add(t, l);
    };

    TUI::PanelScreen screen(std::filesystem::path(exe).filename().string() +
            " chat — " + live, TUI::PanelScreenHooks{
        .onTick = [&] {
            for (auto& f : child.poll()) {
                auto& kind = f.kind;
                auto& payload = f.payload;
                if (kind == "READY") { ready = true;
                    push("[session " + payload + "] — ask about the code",
                         Tether::Platform::LogLevel::Debug); }
                else if (kind == "TOOL")
                    push("  [tool] " + payload, Tether::Platform::LogLevel::Debug);
                else if (kind == "TEXT")
                    push(payload, Tether::Platform::LogLevel::Info);
                else if (kind == "COST")
                    if (!payload.empty())
                        push("  [cost] " + payload,
                             Tether::Platform::LogLevel::Debug);
                else if (kind == "ERR")
                    push("[error] " + payload, Tether::Platform::LogLevel::Error);
                else if (kind == "AWAIT")
                    push("(agent paused — message to continue)",
                         Tether::Platform::LogLevel::Warn);
                else if (kind == "TURN") busy = false;
            }
            if (!child.alive() && !transcript.empty()
                    && transcript.back().text != "[backend exited]")
                push("[backend exited]", Tether::Platform::LogLevel::Error);
        },
        .renderBody = [&](int top, int bottom) {
            // Transcript: rows [top, bottom-1); input on bottom-1.
            int h = bottom - top - 1;
            if (h < 1) return;
            int row = top;
            for (const auto* lp : transcript.visible(h)) {
                move(row, 0); clrtoeol();
                const auto& l = *lp;
                short pair = TUI::PalNone;
                switch (l.level) {
                    case Tether::Platform::LogLevel::Error: pair = 2; break;
                    case Tether::Platform::LogLevel::Warn:  pair = 3; break;
                    case Tether::Platform::LogLevel::Debug: pair = 8; break;
                    default: pair = TUI::PalNone; break;
                }
                if (pair != TUI::PalNone) attron(COLOR_PAIR(pair));
                mvprintw(row, 1, "%s", l.text.c_str());
                if (pair != TUI::PalNone) attroff(COLOR_PAIR(pair));
                ++row;
            }
            // Input row.
            int y = bottom - 1;
            move(y, 0); clrtoeol();
            attron(A_BOLD);
            mvprintw(y, 0, busy ? "…" : ">");
            attroff(A_BOLD);
            std::string view = input;
            int maxw = COLS - 3;
            if (int(view.size()) > maxw)
                view = "…" + view.substr(view.size() - size_t(maxw - 1));
            mvprintw(y, 2, "%s", view.c_str());
            if (!ready)
                mvprintw(y, 2, "(connecting to backend…)");
            // Cursor at end of input.
            move(y, 2 + int(view.size()));
        },
        .showLogView = false,
        .onKey = [&](int ch) -> bool {
            switch (ch) {
                case '\n': case '\r': case KEY_ENTER:
                    if (!input.empty() && ready) {
                        push("> " + input, Tether::Platform::LogLevel::Warn);
                        child.sendFrame("SEND", input);
                        input.clear();
                        busy = true;
                        transcript.scrollEnd();
                    }
                    return true;
                case KEY_BACKSPACE: case 127: case 8:
                    if (!input.empty()) input.pop_back();
                    return true;
                case KEY_PPAGE: transcript.scrollBy(10); return true;
                case KEY_NPAGE: transcript.scrollBy(-10); return true;
                case KEY_END:   transcript.scrollEnd(); return true;
                case 4: // Ctrl-D — quit when the prompt is empty
                    if (input.empty()) return false;
                    return true;
                default:
                    if (ch >= 32 && ch < 127) { input += char(ch); return true; }
                    return false;
            }
        },
        .keyHints = "type + Enter: send   PgUp/PgDn: scroll   q: quit",
    });

    std::atomic<bool> cancel{false};
    screen.run(cancel);
    child.quit();
    return 0;
}
