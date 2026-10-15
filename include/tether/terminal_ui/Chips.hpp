#pragma once

/**
 * @file Chips.hpp
 * @brief Inline ncurses renderer for UI::DisplayCell chip lists.
 *
 * `drawChips()` draws `[a b c]` at a cursor position with per-cell
 * tone colors — the inline counterpart of the boxed `drawChipPanelBox`.
 * Use it for status elements embedded inside a line (flag indicators,
 * tag lists, state summaries).  `UI::formatChips()` renders the same
 * cell list as text for stream/log output, so one model drives both.
 */

#include <ncurses.h>

#include <span>

#include "tether/terminal_ui/ChipPanel.hpp"   // chipTonePair
#include "tether/ui/ChipPanels.hpp"

namespace Tether {
namespace TUI {

/// Display width of a chip list as drawn by drawChips().
inline int chipsWidth(std::span<const UI::DisplayCell> cells,
                      bool brackets = true) {
    int w = brackets ? 2 : 0;
    for (const auto& c : cells) {
        w += static_cast<int>(c.text.size());
    }
    if (!cells.empty()) w += static_cast<int>(cells.size()) - 1;
    return w;
}

/**
 * @brief Draw `[c1 c2 c3]` at (y, col), colored per cell tone.
 *
 * Fault-tone cells render bold; every other tone maps through
 * `chipTonePair`.  Cells are separated by single spaces; pass
 * `brackets=false` to draw just the space-separated cell texts.
 *
 * @return the column after the last character drawn (for chaining
 *         more content on the same line).
 */
inline int wdrawChips(WINDOW* w, int y, int col,
                      std::span<const UI::DisplayCell> cells,
                      bool brackets = true) {
    if (brackets) mvwprintw(w, y, col++, "[");
    bool first = true;
    for (const auto& c : cells) {
        if (!first) mvwprintw(w, y, col++, " ");
        first = false;
        const short pair = chipTonePair(c.tone);
        const bool bold = (c.tone == UI::DisplayTone::Fault);
        wattron(w, COLOR_PAIR(pair));
        if (bold) wattron(w, A_BOLD);
        mvwprintw(w, y, col, "%s", c.text.c_str());
        if (bold) wattroff(w, A_BOLD);
        wattroff(w, COLOR_PAIR(pair));
        col += static_cast<int>(c.text.size());
    }
    if (brackets) mvwprintw(w, y, col++, "]");
    return col;
}

/// stdscr convenience — same as wdrawChips(stdscr, ...).
inline int drawChips(int y, int col,
                     std::span<const UI::DisplayCell> cells,
                     bool brackets = true) {
    return wdrawChips(stdscr, y, col, cells, brackets);
}

} // namespace TUI
} // namespace Tether
