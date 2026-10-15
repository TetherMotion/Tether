/**
 * @file OverlayHost.cpp
 * @brief ncurses renderer + key router for UI::OverlayStack.
 */

#include "tether/terminal_ui/OverlayHost.hpp"

#include <algorithm>
#include <string>

#include <ncurses.h>
#include "tether/terminal_ui/Box.hpp"
#include "tether/terminal_ui/RowWriter.hpp"
#include "tether/terminal_ui/TablePanel.hpp"

namespace Tether {
namespace TUI {

bool OverlayHost::render(int top, int bottom) {
    UI::Overlay* ov = stack_.top();
    if (!ov) return false;

    const int total_h = bottom - top;
    const int w = std::clamp(COLS * 4 / 5, 30, COLS - 4);
    const int h = std::clamp(total_h * 3 / 4, 8, total_h - 2);
    const int ot = top + (total_h - h) / 2;
    const int ol = (COLS - w) / 2;

    // Frame + shaded interior (drawBoxInnerRow pads content with spaces).
    attron(A_BOLD);
    drawBoxTop(ot, ol, w, ov->title(), /*bold=*/false);
    for (int r = 1; r < h - 1; ++r)
        drawBoxInnerRow(ot + r, ol, w);
    drawBoxBottom(ot + h - 1, ol, w);
    attroff(A_BOLD);

    const int ct = ot + 1, cb = ot + h - 1;
    const auto btns = ov->buttons();
    const int content_bottom = btns.empty() ? cb : cb - 1;
    if (UI::TableModel* t = ov->table()) {
        TablePanel::render(*t, ct, content_bottom, ol + 2, w - 4,
                           /*focused=*/true);
    } else {
        RowWriter rw(stdscr, ct, ol + 2, content_bottom);
        for (const auto& line : ov->lines()) {
            if (!rw.fits()) break;
            rw.text("%.*s", w - 4, line.c_str());
        }
    }

    // Dialog button row — centered on the last interior row, focused
    // button reversed.
    if (!btns.empty()) {
        int bw = 0;
        for (const auto& b : btns)
            bw += static_cast<int>(b.label.size()) + 6;  // "[ x ]" + gap
        int bx = ol + (w - bw) / 2;
        for (size_t i = 0; i < btns.size(); ++i) {
            const bool focus = (i == ov->focusedButton());
            if (focus) attron(A_REVERSE | A_BOLD);
            mvprintw(cb - 1, bx, "[ %s ]", btns[i].label.c_str());
            if (focus) attroff(A_REVERSE | A_BOLD);
            bx += static_cast<int>(btns[i].label.size()) + 6;
        }
    }
    return true;
}

bool OverlayHost::handleKey(int key) {
    UI::Overlay* ov = stack_.top();
    if (!ov) return false;
    if (key == 27) {
        // The overlay may consume Esc (dialog: cancel result); the
        // default is pop.
        if (!ov->handleKey(key)) stack_.pop();
    } else {
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
    }
    // A dialog that fired its result asks to be closed.
    if (stack_.top() == ov && ov->wantsClose()) stack_.pop();
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
