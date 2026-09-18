#pragma once

/**
 * @file TablePanel.hpp
 * @brief ncurses renderer for UI::TableModel (standard table panel).
 *
 * Draws a column header, the visible (filtered) rows with selection
 * highlight and per-row semantic colors, a scroll indicator, and a
 * status line showing "shown/total" counts and the active filter.
 *
 * Column layout: pass explicit character widths in `widths` (a 0 entry
 * or a short vector leaves the remaining columns auto-sized; the last
 * column always takes the remainder).
 */

#include <vector>

#include "tether/terminal_ui/Session.hpp"
#include "tether/ui/TableModel.hpp"

namespace Tether {
namespace TUI {

class TablePanel {
public:
    /// Render `model` into the screen region [top, bottom) starting at
    /// column `left`.  Returns the number of screen rows consumed
    /// (header + rows + status line).  `focused=false` draws the header
    /// dimmed and hides the selection highlight.
    static int render(UI::TableModel& model, int top, int bottom,
                      int left, int width, bool focused = true,
                      const std::vector<int>& widths = {});

private:
    static std::vector<int> columnOffsets(const UI::TableModel& model,
                                          int width,
                                          const std::vector<int>& widths);
};

/// Map a UI::RowColor to a Session palette index.
inline short rowPalette(int color) {
    switch (color) {
        case UI::RowOk:    return PalValue;
        case UI::RowWarn:  return PalHint;
        case UI::RowErr:   return PalError;
        case UI::RowMuted: return PalMuted;
        case UI::RowInfo:  return PalInfo;
        default:           return PalNone;
    }
}

} // namespace TUI
} // namespace Tether
