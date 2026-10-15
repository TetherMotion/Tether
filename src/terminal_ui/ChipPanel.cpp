#include "tether/terminal_ui/ChipPanel.hpp"

#include <algorithm>

#include <ncurses.h>
#include "tether/terminal_ui/Box.hpp"
#include "tether/terminal_ui/Session.hpp"

namespace Tether {
namespace TUI {

short chipTonePair(UI::DisplayTone t) {
    using T = UI::DisplayTone;
    switch (t) {
        case T::Ok:      return PalValue;   // green
        case T::Missing: return PalError;   // red
        case T::Fault:   return PalError;   // red (+ bold)
        case T::Warn:    return PalHint;    // yellow
        case T::Info:    return PalInfo;    // blue
        case T::Muted:   return PalMuted;   // dim
    }
    return PalNone;
}

int chipPanelContentLines(const UI::DisplayPanel& p, int inner_w) {
    if (inner_w < 1) return 0;
    int lines = 0;
    for (const auto& row : p.rows) {
        ++lines;
        int cx = 0;
        for (const auto& c : row.cells) {
            const int len = std::min<int>(inner_w,
                                          static_cast<int>(c.text.size()));
            int need = len + (cx ? 1 : 0);
            if (cx && cx + need > inner_w) { ++lines; cx = 0; need = len; }
            cx += need;
        }
    }
    return lines;
}

int drawChipPanelBox(int x, int y, int w,
                     const UI::DisplayPanel& p,
                     int max_h, int min_lines) {
    if (max_h < 3 || w < 4) return 0;
    const int inner = w - 2;
    int cy = y;

    drawBoxTop(cy++, x, w, p.title);

    const int last = y + max_h - 1;   // bottom border line
    int lines = 0;
    for (const auto& row : p.rows) {
        if (cy >= last) break;
        int cx = 0;
        for (const auto& c : row.cells) {
            std::string t = c.text;
            if (static_cast<int>(t.size()) > inner)
                t.resize(static_cast<size_t>(inner));
            const int len = static_cast<int>(t.size());
            const int need = len + (cx ? 1 : 0);
            if (cx && cx + need > inner) {
                padBoxRowEnd(cy, x, w, cx);
                ++cy; ++lines; cx = 0;
                if (cy >= last) break;
            }
            if (cx) { mvprintw(cy, x + 1 + cx, " "); ++cx; }
            const short pair = chipTonePair(c.tone);
            attron(COLOR_PAIR(pair));
            if (c.tone == UI::DisplayTone::Fault)
                attron(A_BOLD);
            mvprintw(cy, x + 1 + cx, "%s", t.c_str());
            if (c.tone == UI::DisplayTone::Fault)
                attroff(A_BOLD);
            attroff(COLOR_PAIR(pair));
            cx += len;
        }
        if (cy >= last) break;
        padBoxRowEnd(cy, x, w, cx);
        ++cy; ++lines;
    }
    while (lines < min_lines && cy < last) {
        padBoxRowEnd(cy, x, w, 0);
        ++cy; ++lines;
    }

    drawBoxBottom(cy++, x, w);
    return cy - y;
}

int drawChipPanelGrid(int top, int bottom,
                      const std::vector<UI::DisplayPanel>& panels) {
    if (bottom - top < 3 || top >= LINES) {
        return 0;
    }
    const int ncols = (COLS >= 150) ? 4 : (COLS >= 92) ? 2 : 1;
    int y = top;
    for (size_t i = 0; i < panels.size() && y < bottom;
         i += static_cast<size_t>(ncols)) {
        const size_t cnt = std::min<size_t>(ncols, panels.size() - i);
        const int col_w = COLS / ncols;
        // Group height = tallest panel in this grid row (clipped).
        int grp_h = 0;
        for (size_t c = 0; c < cnt; ++c) {
            const int px = static_cast<int>(c) * col_w;
            const int w = (c == cnt - 1) ? COLS - px : col_w;
            grp_h = std::max(
                grp_h,
                chipPanelContentLines(panels[i + c], w - 2) + 2);
        }
        grp_h = std::min(grp_h, bottom - y);
        for (size_t c = 0; c < cnt; ++c) {
            const int px = static_cast<int>(c) * col_w;
            const int w = (c == cnt - 1) ? COLS - px : col_w;
            drawChipPanelBox(px, y, w, panels[i + c], grp_h, grp_h - 2);
        }
        y += grp_h;
    }
    return y - top;
}

} // namespace TUI
} // namespace Tether
