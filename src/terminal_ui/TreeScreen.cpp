/**
 * @file TreeScreen.cpp
 * @brief Composed tree + detail + log screen implementation
 */

#include "tether/terminal_ui/TreeScreen.hpp"

#include <chrono>

#include <ncurses.h>

namespace Tether {
namespace TUI {

TreeScreen::TreeScreen(std::string title, TreeNode root, TreeScreenHooks hooks)
    : title_(std::move(title)), hooks_(std::move(hooks)) {
    tree_.setRoot(std::move(root));
    log_.attach();   // capture console logging while curses owns the screen
}

void TreeScreen::run(std::atomic<bool>& cancel, double durationSec) {
    const auto t0 = std::chrono::steady_clock::now();

    TermWindow* treeWin   = nullptr;
    TermWindow* detailWin = nullptr;
    TermWindow* logWin    = nullptr;

    auto layout = [&]() {
        session_.freeWindow(treeWin);
        session_.freeWindow(detailWin);
        session_.freeWindow(logWin);

        const int H = session_.rows();
        const int W = session_.cols();
        const int logH    = static_cast<int>(log_.maxLines());
        const int footerY = H - 1;
        const int logY    = footerY - logH;
        const int top     = 1;              // header row 0
        const int bottom  = logY - 1;       // separator above log
        const int contentH = bottom - top;
        const int leftW = std::max(24, W * 2 / 5);

        treeWin   = session_.makeWindow(top, 0, contentH, leftW - 1);
        detailWin = session_.makeWindow(top, leftW, contentH, W - leftW);
        logWin    = session_.makeWindow(logY, 0, logH, W);
        return footerY;
    };

    int footerY = layout();

    while (!cancel.load()) {
        const double elapsed = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - t0).count();
        if (durationSec > 0.0 && elapsed >= durationSec) break;

        if (hooks_.onTick) hooks_.onTick();

        // ---- Header ----------------------------------------------------
        erase();
        attron(A_BOLD | (session_.colors() ? COLOR_PAIR(PalHeader) : 0));
        mvprintw(0, 0, " %s", title_.c_str());
        attroff(A_BOLD | (session_.colors() ? COLOR_PAIR(PalHeader) : 0));
        mvprintw(0, session_.cols() - 12, "t=%6.1f s", elapsed);

        // ---- Footer -----------------------------------------------------
        if (session_.colors()) attron(COLOR_PAIR(PalHint));
        mvprintw(footerY, 0,
                 " arrows: navigate  left/right: fold  q: quit%s%s",
                 hooks_.keyHints.empty() ? "" : "  ",
                 hooks_.keyHints.c_str());
        if (session_.colors()) attroff(COLOR_PAIR(PalHint));

        // ---- Panes -----------------------------------------------------
        const int leftW = std::max(24, session_.cols() * 2 / 5);
        for (int r = 1; r < footerY - static_cast<int>(log_.maxLines()); ++r) {
            mvaddch(r, leftW - 1, ACS_VLINE);
        }
        mvaddch(footerY - static_cast<int>(log_.maxLines()), 0, ACS_LTEE);
        mvaddch(footerY - static_cast<int>(log_.maxLines()), leftW - 1, ACS_PLUS);
        mvhline(footerY - static_cast<int>(log_.maxLines()), 1, ACS_HLINE,
                session_.cols() - 2);

        // Push the stdscr frame first, then each subwindow over its own region,
        // and update the physical screen once at the end.  The previous
        // wrefresh(stdscr) at the end was repainting stdscr over the panes.
        wnoutrefresh(stdscr);

        tree_.render(treeWin);
        wnoutrefresh(static_cast<WINDOW*>(treeWin));

        werase(static_cast<WINDOW*>(detailWin));
        if (const TreeNode* sel = tree_.selected()) {
            if (hooks_.renderDetail) {
                hooks_.renderDetail(detailWin, *sel);
            }
        }
        wnoutrefresh(static_cast<WINDOW*>(detailWin));

        log_.render(logWin, PalError);
        wnoutrefresh(static_cast<WINDOW*>(logWin));

        doupdate();

        // ---- Input ------------------------------------------------------
        const int key = session_.pollKey(50);
        if (key == 'q' || key == 'Q' || key == 27) break;
        if (key == KEY_RESIZE) { footerY = layout(); continue; }
        if (key < 0) continue;
        if (tree_.handleKey(key)) continue;
        if (hooks_.onKey) hooks_.onKey(key);
    }
}

} // namespace TUI
} // namespace Tether
