/**
 * @file PanelScreen.cpp
 * @brief Composed title + body + log + footer screen implementation
 */

#include "tether/terminal_ui/PanelScreen.hpp"

#include <algorithm>
#include <chrono>
#include <string>

#include <ncurses.h>

namespace Tether {
namespace TUI {

PanelScreen::PanelScreen(std::string title, PanelScreenHooks hooks)
    : title_(std::move(title)), hooks_(std::move(hooks)) {
    log_.attach();   // capture console logging while curses owns the screen
}

void PanelScreen::run(std::atomic<bool>& cancel, double durationSec) {
    const auto t0 = std::chrono::steady_clock::now();

    TermWindow* logWin = nullptr;

    auto layout = [&]() {
        session_.freeWindow(logWin);
        const int footerY = session_.rows() - 1;
        const int logH = static_cast<int>(log_.maxLines());
        logWin = session_.makeWindow(footerY - logH, 0, logH,
                                     session_.cols());
        return footerY;
    };

    int footerY = layout();

    while (!cancel.load()) {
        const double elapsed = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - t0).count();
        if (durationSec > 0.0 && elapsed >= durationSec) break;

        if (hooks_.onTick) hooks_.onTick();

        erase();

        // ---- Header ----------------------------------------------------
        attron(A_BOLD | (session_.colors() ? COLOR_PAIR(PalHeader) : 0));
        mvprintw(0, 0, " %s", title_.c_str());
        attroff(A_BOLD | (session_.colors() ? COLOR_PAIR(PalHeader) : 0));
        mvprintw(0, session_.cols() - 12, "t=%6.1f s", elapsed);

        // ---- Footer -----------------------------------------------------
        if (session_.colors()) attron(COLOR_PAIR(PalHint));
        mvprintw(footerY, 0, " q: quit  PgUp/PgDn/End: log%s%s",
                 hooks_.keyHints.empty() ? "" : "  ",
                 hooks_.keyHints.c_str());
        if (session_.colors()) attroff(COLOR_PAIR(PalHint));

        // ---- Separator above the log pane -------------------------------
        const int sepY = footerY - static_cast<int>(log_.maxLines()) - 1;
        const int hlineW = std::max(0, session_.cols() - 2);
        std::string hline;
        hline.reserve(static_cast<size_t>(hlineW) * 3);
        for (int i = 0; i < hlineW; ++i) hline += "\xE2\x94\x80"; // ─
        mvprintw(sepY, 1, "%s", hline.c_str());
        mvprintw(sepY, 0, "%s", "\xE2\x94\x9C");                  // ├

        // ---- Body -------------------------------------------------------
        // The app draws rows [1, sepY) directly on stdscr.
        if (hooks_.renderBody) hooks_.renderBody(1, sepY);

        // Push stdscr first, then the log subwindow, one physical update.
        wnoutrefresh(stdscr);
        log_.render(logWin);
        wnoutrefresh(static_cast<WINDOW*>(logWin));
        doupdate();

        // ---- Input ------------------------------------------------------
        const int key = session_.pollKey(50);
        if (key == KEY_PPAGE) {
            log_.scrollLines(-static_cast<int>(log_.maxLines()));
            continue;
        }
        if (key == KEY_NPAGE) {
            log_.scrollLines(static_cast<int>(log_.maxLines()));
            continue;
        }
        if (key == KEY_END) { log_.scrollToEnd(); continue; }
        if (key == 'q' || key == 'Q' || key == 27) {
            if (hooks_.quitGuard && hooks_.quitGuard()) {
                if (hooks_.onKey) hooks_.onKey(key);
                continue;
            }
            break;
        }
        if (key == KEY_RESIZE) { footerY = layout(); continue; }
        if (key < 0) continue;
        if (hooks_.onKey) hooks_.onKey(key);
    }

    session_.freeWindow(logWin);
}

} // namespace TUI
} // namespace Tether
