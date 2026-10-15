#pragma once

/// Segmented status bar: left-anchored and right-anchored styled segments on
/// one row, without manual column arithmetic.  Covers the "t=12.3s
/// reads=4021 [STALE]" header rows that applications previously drew with
/// hand-computed mvprintw offsets.
///
/// Uses mvprintw only (consumers #undef ERR after <ncurses.h>).

#include <ncurses.h>
#include <span>
#include <string>

#include "tether/terminal_ui/StyledText.hpp"

namespace Tether {
namespace TUI {

/// One status-bar segment (alias of the generic styled Span).
using BarSeg = Span;

/// Draw a status bar on row `y` spanning [0, width): `left` segments drawn
/// from column 0 left-to-right (each separated by one space if the text does
/// not end with whitespace), `right` segments right-aligned to `width`.
/// Returns the first free column after the left group.
inline int drawStatusBar(int y, std::span<const BarSeg> left,
                         std::span<const BarSeg> right = {},
                         int width = COLS) {
    int col = 0;
    for (const auto& s : left) {
        AttrScope scope(s.attrs);
        mvprintw(y, col, "%s", s.text.c_str());
        col += static_cast<int>(utf8Cols(s.text));
        if (col < width &&
            (s.text.empty() || s.text.back() != ' ')) {
            mvprintw(y, col, " ");
            ++col;
        }
    }
    if (!right.empty()) {
        int rw = 0;
        for (const auto& s : right) rw += static_cast<int>(utf8Cols(s.text)) + 1;
        int rcol = width - rw;
        for (const auto& s : right) {
            AttrScope scope(s.attrs);
            mvprintw(y, rcol, "%s", s.text.c_str());
            rcol += static_cast<int>(utf8Cols(s.text)) + 1;
        }
    }
    return col;
}

} // namespace TUI
} // namespace Tether
