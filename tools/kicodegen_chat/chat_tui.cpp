/**
 * @file chat_tui.cpp
 * @brief Chat TUI for KICodegen — a devin-CLI-style conversation with the
 *        generated code, built on tether_terminal_ui.
 *
 * Usage (spawned by `kicodegen chat`, or standalone):
 *   tether-chat <kicodegen-exe> <repo_root> <live_dir>
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
#include "tether/terminal_ui/PanelScreen.hpp"
#include "tether/terminal_ui/LogPane.hpp"
#include "tether/terminal_ui/StyledText.hpp"

#include <ncurses.h>
#undef OK
#undef ERR

#include <atomic>
#include <cerrno>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <format>
#include <string>
#include <vector>

#include <fcntl.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>

namespace TUI = Tether::TUI;

namespace {

// ---------------------------------------------------------------------------
// Child process: `kicodegen chat --rpc` on pipes.
// ---------------------------------------------------------------------------

struct RpcChild {
    pid_t pid = -1;
    int   to_child = -1;   // our write end → child stdin
    int   from_child = -1; // our read end  ← child stdout
    std::string rx;        // unparsed frame bytes
    bool  dead = false;

    bool spawn(const std::string& exe, const std::string& repo,
               const std::string& live) {
        int pin[2], pout[2];
        if (pipe(pin) || pipe(pout)) return false;
        pid = fork();
        if (pid < 0) return false;
        if (pid == 0) {
            dup2(pin[0], STDIN_FILENO);
            dup2(pout[1], STDOUT_FILENO);
            // stderr passes through — keep logs visible on exit.
            for (int fd : {pin[0], pin[1], pout[0], pout[1]}) close(fd);
            execl(exe.c_str(), exe.c_str(), "chat", "--rpc",
                  "--repo", repo.c_str(), "--out", live.c_str(),
                  (char*)nullptr);
            _exit(127);
        }
        close(pin[0]);
        close(pout[1]);
        to_child = pin[1];
        from_child = pout[0];
        fcntl(from_child, F_SETFL, O_NONBLOCK);
        return true;
    }

    void sendFrame(const char* kind, const std::string& payload) {
        if (to_child < 0) return;
        std::string hdr = std::format("{} {}\n", kind, payload.size());
        (void)!write(to_child, hdr.data(), hdr.size());
        (void)!write(to_child, payload.data(), payload.size());
    }

    void quit() {
        if (to_child >= 0) {
            const char q[] = "QUIT\n";
            (void)!write(to_child, q, sizeof(q) - 1);
            close(to_child);
            to_child = -1;
        }
        if (pid > 0) {
            for (int i = 0; i < 20 && waitpid(pid, nullptr, WNOHANG) == 0; ++i)
                usleep(50 * 1000);
            if (waitpid(pid, nullptr, WNOHANG) == 0) kill(pid, SIGTERM);
            waitpid(pid, nullptr, 0);
            pid = -1;
        }
        if (from_child >= 0) { close(from_child); from_child = -1; }
    }

    ~RpcChild() { quit(); }

    /**Pump available bytes; complete frames → (kind, payload) pairs. */
    std::vector<std::pair<std::string, std::string>> poll() {
        std::vector<std::pair<std::string, std::string>> out;
        if (from_child < 0) return out;
        char buf[8192];
        for (;;) {
            ssize_t n = read(from_child, buf, sizeof(buf));
            if (n > 0) { rx.append(buf, size_t(n)); continue; }
            if (n == 0) { dead = true; break; }
            if (errno != EAGAIN && errno != EWOULDBLOCK) dead = true;
            break;
        }
        for (;;) {
            auto nl = rx.find('\n');
            if (nl == std::string::npos) break;
            std::string hdr = rx.substr(0, nl);
            auto sp = hdr.find(' ');
            if (sp == std::string::npos) { rx.erase(0, nl + 1); continue; }
            size_t len = 0;
            try { len = std::stoul(hdr.substr(sp + 1)); }
            catch (...) { rx.erase(0, nl + 1); continue; }
            if (rx.size() < nl + 1 + len) break; // wait for the payload
            out.emplace_back(hdr.substr(0, sp), rx.substr(nl + 1, len));
            rx.erase(0, nl + 1 + len);
        }
        return out;
    }
};

// ---------------------------------------------------------------------------
// Word-wrap a message into ≤ w columns, keeping it readable.
// ---------------------------------------------------------------------------

std::vector<std::string> wrap(const std::string& text, int w) {
    std::vector<std::string> lines;
    std::string cur;
    std::string word;
    auto flush_word = [&] {
        while (int(word.size()) > w) {
            if (!cur.empty()) { lines.push_back(cur); cur.clear(); }
            lines.push_back(word.substr(0, size_t(w)));
            word.erase(0, size_t(w));
        }
        if (!word.empty()) {
            if (!cur.empty() && int(cur.size() + word.size()) + 1 > w) {
                lines.push_back(cur);
                cur.clear();
            }
            if (!cur.empty()) cur += ' ';
            cur += word;
            word.clear();
        }
    };
    for (char c : text) {
        if (c == '\n') { flush_word(); lines.push_back(cur); cur.clear(); }
        else if (c == ' ') flush_word();
        else word += c;
    }
    flush_word();
    if (!cur.empty() || lines.empty()) lines.push_back(cur);
    return lines;
}

} // namespace

int main(int argc, char** argv) {
    if (argc < 4) {
        fprintf(stderr,
            "usage: %s <kicodegen-exe> <repo_root> <live_dir>\n", argv[0]);
        return 2;
    }
    const std::string exe = argv[1], repo = argv[2], live = argv[3];

    RpcChild child;
    if (!child.spawn(exe, repo, live)) {
        fprintf(stderr, "failed to spawn %s chat --rpc\n", exe.c_str());
        return 1;
    }

    // Transcript state ------------------------------------------------
    struct Line { std::string text; Tether::Platform::LogLevel lvl; };
    std::deque<Line> transcript;
    std::string input;
    bool busy = false;        // a turn is in flight
    bool ready = false;       // READY frame seen
    int  scroll = 0;          // lines above bottom (0 = following)

    auto push = [&](std::string t, Tether::Platform::LogLevel l) {
        for (auto& w : wrap(t, std::max(20, COLS - 4)))
            transcript.push_back({w, l});
        while (transcript.size() > 4000) transcript.pop_front();
    };

    TUI::PanelScreen screen("kicodegen chat — " + live, TUI::PanelScreenHooks{
        .onTick = [&] {
            for (auto& [kind, payload] : child.poll()) {
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
            if (child.dead && !transcript.empty() && transcript.back().text
                    != "[backend exited]")
                push("[backend exited]", Tether::Platform::LogLevel::Error);
        },
        .renderBody = [&](int top, int bottom) {
            // Transcript: rows [top, bottom-1); input on bottom-1.
            int h = bottom - top - 1;
            if (h < 1) return;
            // Build the visible slice as a flat vector (transcript is deque).
            std::vector<const Line*> vis;
            vis.reserve(transcript.size());
            for (auto& l : transcript) vis.push_back(&l);
            int end = int(vis.size()) - scroll;
            if (end < 0) end = 0;
            int start = std::max(0, end - h);
            int row = top;
            for (int i = start; i < end; ++i, ++row) {
                move(row, 0); clrtoeol();
                const auto& l = *vis[size_t(i)];
                short pair = TUI::PalNone;
                switch (l.lvl) {
                    case Tether::Platform::LogLevel::Error: pair = 2; break;
                    case Tether::Platform::LogLevel::Warn:  pair = 3; break;
                    case Tether::Platform::LogLevel::Debug: pair = 8; break;
                    default: pair = TUI::PalNone; break;
                }
                if (pair != TUI::PalNone) attron(COLOR_PAIR(pair));
                mvprintw(row, 1, "%s", l.text.c_str());
                if (pair != TUI::PalNone) attroff(COLOR_PAIR(pair));
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
                        scroll = 0;
                    }
                    return true;
                case KEY_BACKSPACE: case 127: case 8:
                    if (!input.empty()) input.pop_back();
                    return true;
                case KEY_PPAGE: scroll += 10; return true;
                case KEY_NPAGE: scroll = std::max(0, scroll - 10); return true;
                case KEY_END:   scroll = 0; return true;
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
