/**
 * @file PanelScreen.cpp
 * @brief Composed title + body + log + footer screen implementation
 *        with hotkey-switchable named views (tabs).
 */

#include "tether/terminal_ui/PanelScreen.hpp"

#include <algorithm>
#include <chrono>
#include <cstring>
#include <string>

#include <ncurses.h>

namespace Tether {
namespace TUI {

PanelScreen::PanelScreen(std::string title, PanelScreenHooks hooks)
    : title_(std::move(title)), hooks_(std::move(hooks)) {
    log_.attach();   // capture console logging while curses owns the screen
}

size_t PanelScreen::viewCount() const {
    // views.empty() → one implicit "Main" view driven by hooks_.renderBody.
    const size_t app_views = std::max<size_t>(hooks_.views.size(), 1);
    return app_views + (hooks_.showLogView ? 1 : 0);
}

void PanelScreen::selectView(size_t index) {
    const size_t n = viewCount();
    if (n == 0) { active_ = 0; return; }
    const size_t clamped = std::min(index, n - 1);
    if (clamped == active_) return;
    active_ = clamped;
    if (hooks_.onViewChange) hooks_.onViewChange(active_);
}

void PanelScreen::run(std::atomic<bool>& cancel, double durationSec) {
    const auto t0 = std::chrono::steady_clock::now();

    TermWindow* logWin     = nullptr;
    TermWindow* logFullWin = nullptr;

    auto layout = [&]() {
        session_.freeWindow(logWin);
        session_.freeWindow(logFullWin);
        const int footerY = session_.rows() - 1;
        const int logH = static_cast<int>(log_.maxLines());
        const int sepY = footerY - logH - 1;
        bodyRows_ = std::max(1, sepY - 1);
        logWin = session_.makeWindow(footerY - logH, 0, logH,
                                     session_.cols());
        logFullWin = session_.makeWindow(1, 0, bodyRows_,
                                         session_.cols());
        return footerY;
    };

    int footerY = layout();
    bool filter_editing = false;
    std::string filter_buf;

    auto viewName = [&](size_t i) -> const char* {
        if (i < hooks_.views.size()) return hooks_.views[i].name.c_str();
        if (hooks_.views.empty() && i == 0) return "Main";
        return "Log";
    };

    while (!cancel.load()) {
        const double elapsed = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - t0).count();
        if (durationSec > 0.0 && elapsed >= durationSec) break;

        if (hooks_.onTick) hooks_.onTick();

        erase();

        // ---- Header + view tabs ----------------------------------------
        attron(A_BOLD | (session_.colors() ? COLOR_PAIR(PalHeader) : 0));
        mvprintw(0, 0, " %s", title_.c_str());
        attroff(A_BOLD | (session_.colors() ? COLOR_PAIR(PalHeader) : 0));
        mvprintw(0, session_.cols() - 12, "t=%6.1f s", elapsed);

        const size_t nViews = viewCount();
        if (nViews > 1) {
            int col = static_cast<int>(title_.size()) + 3;
            for (size_t i = 0; i < nViews; ++i) {
                const char* vn = viewName(i);
                const int need = static_cast<int>(std::strlen(vn)) + 5;
                if (col + need >= session_.cols() - 13) break;
                const bool act = (i == active_);
                if (act) attron(A_BOLD | A_REVERSE |
                                (session_.colors() ? COLOR_PAIR(PalValue) : 0));
                else if (session_.colors()) attron(COLOR_PAIR(PalMuted));
                mvprintw(0, col, " %zu:%s ", i + 1, vn);
                if (act) attroff(A_BOLD | A_REVERSE |
                                 (session_.colors() ? COLOR_PAIR(PalValue) : 0));
                else if (session_.colors()) attroff(COLOR_PAIR(PalMuted));
                col += need;
            }
        }

        // ---- Footer -----------------------------------------------------
        if (session_.colors()) attron(COLOR_PAIR(PalHint));
        if (filter_editing) {
            mvprintw(footerY, 0,
                     " filter: %s_   (level>=warn  tag=x  !tag=y  text=z"
                     "  | Enter: apply  Esc: cancel  empty: clear)",
                     filter_buf.c_str());
        } else {
            const char* vh = "";
            if (active_ < hooks_.views.size() &&
                !hooks_.views[active_].keyHints.empty())
                vh = hooks_.views[active_].keyHints.c_str();
            else if (!hooks_.keyHints.empty())
                vh = hooks_.keyHints.c_str();
            mvprintw(footerY, 0,
                     " q: quit  Tab/1-9: views  PgUp/PgDn/End: log  f: filter%s%s",
                     vh[0] ? "  " : "", vh);
        }
        if (session_.colors()) attroff(COLOR_PAIR(PalHint));

        // ---- Separator above the log pane -------------------------------
        const int sepY = footerY - static_cast<int>(log_.maxLines()) - 1;
        const int hlineW = std::max(0, session_.cols() - 2);
        std::string hline;
        hline.reserve(static_cast<size_t>(hlineW) * 3);
        for (int i = 0; i < hlineW; ++i) hline += "\xE2\x94\x80"; // ─
        mvprintw(sepY, 1, "%s", hline.c_str());
        mvprintw(sepY, 0, "%s", "\xE2\x94\x9C");                  // ├

        // ---- Body (active view) -----------------------------------------
        const bool logViewActive =
            hooks_.showLogView && active_ == nViews - 1;
        if (logViewActive) {
            log_.render(logFullWin);
        } else {
            std::function<void(int,int)> rb;
            if (active_ < hooks_.views.size())
                rb = hooks_.views[active_].renderBody;
            if (!rb) rb = hooks_.renderBody;
            if (rb) rb(1, sepY);
        }

        // Push stdscr first, then the log subwindow, one physical update.
        wnoutrefresh(stdscr);
        if (logViewActive)
            wnoutrefresh(static_cast<WINDOW*>(logFullWin));
        log_.render(logWin);
        wnoutrefresh(static_cast<WINDOW*>(logWin));
        doupdate();

        // ---- Input ------------------------------------------------------
        const int key = session_.pollKey(50);
        // Modal filter editor: 'f' opens it, every key goes to the
        // expression buffer until Enter (apply) or Esc (cancel).
        if (filter_editing) {
            if (key == 27) { filter_editing = false; continue; }
            if (key == '\n' || key == '\r' || key == KEY_ENTER) {
                filter_editing = false;
                if (filter_buf.empty()) log_.clearViewFilter();
                else log_.setViewFilter(LogFilter::parse(filter_buf));
                log_.scrollToEnd();
                continue;
            }
            if (key == KEY_BACKSPACE || key == 127 || key == '\b') {
                if (!filter_buf.empty()) filter_buf.pop_back();
                continue;
            }
            if (key >= 32 && key <= 126) {
                filter_buf += static_cast<char>(key);
            }
            continue;
        }
        if (key == 'f') { filter_editing = true; filter_buf.clear(); continue; }

        // View switching: digits 1-9, F1-F12, Tab / Shift-Tab.
        if (key >= '1' && key <= '9') {
            selectView(static_cast<size_t>(key - '1'));
            continue;
        }
        if (key >= KEY_F(1) && key <= KEY_F(12)) {
            selectView(static_cast<size_t>(key - KEY_F(1)));
            continue;
        }
        if (key == '\t') { selectView((active_ + 1) % nViews); continue; }
        if (key == KEY_BTAB) {
            selectView((active_ + nViews - 1) % nViews);
            continue;
        }

        // Log scrollback: in the log view arrows/page keys scroll by a
        // page; elsewhere PgUp/PgDn scroll the strip by its height.
        const int pageStep = logViewActive
            ? std::max(1, bodyRows_ - 1)
            : static_cast<int>(log_.maxLines());
        if (key == KEY_PPAGE) { log_.scrollLines(-pageStep); continue; }
        if (key == KEY_NPAGE) { log_.scrollLines(pageStep); continue; }
        if (key == KEY_END)   { log_.scrollToEnd(); continue; }
        if (logViewActive) {
            if (key == KEY_UP)   { log_.scrollLines(-1); continue; }
            if (key == KEY_DOWN) { log_.scrollLines(1); continue; }
            if (key == KEY_HOME) {
                log_.scrollLines(-static_cast<int>(log_.size()));
                continue;
            }
        }

        if (key == 'q' || key == 'Q' || key == 27) {
            if (hooks_.quitGuard && hooks_.quitGuard()) {
                if (hooks_.onKey) hooks_.onKey(key);
                continue;
            }
            break;
        }
        if (key == KEY_RESIZE) { footerY = layout(); continue; }
        if (key < 0) continue;

        // Active view keys, then the global fallback.
        if (active_ < hooks_.views.size() &&
            hooks_.views[active_].onKey &&
            hooks_.views[active_].onKey(key))
            continue;
        if (hooks_.onKey) hooks_.onKey(key);
    }

    session_.freeWindow(logWin);
    session_.freeWindow(logFullWin);
}

} // namespace TUI
} // namespace Tether
