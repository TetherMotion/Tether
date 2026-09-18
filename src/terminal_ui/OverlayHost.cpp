/**
 * @file OverlayHost.cpp
 * @brief ncurses renderer + key router for UI::OverlayStack.
 */

#include <ncurses.h>

#include <algorithm>
#include <string>

#include "tether/terminal_ui/OverlayHost.hpp"
#include "tether/terminal_ui/TablePanel.hpp"

namespace Tether {
namespace TUI {

namespace {
void box(int top, int left, int h, int w, const std::string& title) {
    attron(A_BOLD);
    mvaddch(top, left, ACS_ULCORNER);
    mvaddch(top, left + w - 1, ACS_URCORNER);
    mvaddch(top + h - 1, left, ACS_LLCORNER);
    mvaddch(top + h - 1, left + w - 1, ACS_LRCORNER);
    for (int x = 1; x < w - 1; ++x) {
        mvaddch(top, left + x, ACS_HLINE);
        mvaddch(top + h - 1, left + x, ACS_HLINE);
    }
    for (int y = 1; y < h - 1; ++y) {
        mvaddch(top + y, left, ACS_VLINE);
        mvaddch(top + y, left + w - 1, ACS_VLINE);
    }
    if (!title.empty())
        mvprintw(top, left + 2, " %s ", title.c_str());
    attroff(A_BOLD);
}
} // namespace

bool OverlayHost::render(int top, int bottom) {
    UI::Overlay* ov = stack_.top();
    if (!ov) return false;

    const int total_h = bottom - top;
    const int w = std::clamp(COLS * 4 / 5, 30, COLS - 4);
    const int h = std::clamp(total_h * 3 / 4, 8, total_h - 2);
    const int ot = top + (total_h - h) / 2;
    const int ol = (COLS - w) / 2;

    // Shade the box interior.
    for (int y = ot + 1; y < ot + h - 1; ++y)
        mvhline(y, ol + 1, ' ', w - 2);

    box(ot, ol, h, w, ov->title());

    const int ct = ot + 1, cb = ot + h - 1;
    if (UI::TableModel* t = ov->table()) {
        TablePanel::render(*t, ct, cb, ol + 2, w - 4, /*focused=*/true);
    } else {
        int y = ct;
        for (const auto& line : ov->lines()) {
            if (y >= cb) break;
            mvprintw(y++, ol + 2, "%.*s", w - 4, line.c_str());
        }
    }
    return true;
}

bool OverlayHost::handleKey(int key) {
    UI::Overlay* ov = stack_.top();
    if (!ov) return false;
    if (key == 27) { stack_.pop(); return true; }
    // Table overlays get full navigation via their model.
    if (UI::TableModel* t = ov->table()) {
        switch (key) {
            case KEY_UP:    t->move(-1); return true;
            case KEY_DOWN:  t->move(+1); return true;
            case KEY_PPAGE: t->pageMove(-1, 8); return true;
            case KEY_NPAGE: t->pageMove(+1, 8); return true;
            case KEY_HOME:  t->selectFirst(); return true;
            case KEY_END:   t->selectLast(); return true;
            default: break;
        }
    }
    ov->handleKey(key);   // overlay may consume or ignore
    return true;          // never leak to the underlying panel
}

std::string OverlayHost::keyHints() const {
    UI::Overlay* ov = stack_.top();
    if (!ov) return {};
    std::string h = ov->keyHints();
    return h.empty() ? std::string{"Esc:close"} : h + " Esc:close";
}

} // namespace TUI
} // namespace Tether
