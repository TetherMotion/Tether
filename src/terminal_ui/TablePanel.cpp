/**
 * @file TablePanel.cpp
 * @brief ncurses renderer for UI::TableModel.
 */

#include <ncurses.h>

#include <algorithm>
#include <string>

#include "tether/terminal_ui/TablePanel.hpp"

namespace Tether {
namespace TUI {

namespace {

/// Truncate/pad to fit `width` columns.
std::string fitCell(const std::string& s, int width) {
    if (width <= 0) return {};
    if (static_cast<int>(s.size()) > width)
        return s.substr(0, static_cast<size_t>(width - 1)) + "~";
    return s + std::string(static_cast<size_t>(width - s.size()), ' ');
}

} // namespace

std::vector<int> TablePanel::columnOffsets(const UI::TableModel& model,
                                           int width,
                                           const std::vector<int>& widths) {
    const size_t n = model.columns().size();
    std::vector<int> offs(n, 0);
    if (n == 0 || width <= 0) return offs;

    // Resolve widths: explicit > auto (remaining space split evenly).
    std::vector<int> w(n, 0);
    int fixed = 0;
    size_t auto_count = 0;
    for (size_t i = 0; i < n; ++i) {
        if (i < widths.size() && widths[i] > 0) {
            w[i] = widths[i];
            fixed += w[i] + 1;
        } else {
            ++auto_count;
        }
    }
    const int rest = std::max(8, width - fixed);
    const int per = auto_count ? std::max(4, rest / static_cast<int>(auto_count) - 1) : 0;
    int x = 0;
    for (size_t i = 0; i < n; ++i) {
        offs[i] = x;
        const int cw = (w[i] > 0) ? w[i] : (i == n - 1 ? width - x : per);
        x += cw + 1;
    }
    return offs;
}

int TablePanel::render(UI::TableModel& model, int top, int bottom,
                       int left, int width, bool focused,
                       const std::vector<int>& widths) {
    if (top >= bottom || width <= 0) return 0;
    const auto& cols = model.columns();
    const auto offs = columnOffsets(model, width, widths);

    int y = top;

    // Header
    if (focused) attron(A_BOLD | COLOR_PAIR(PalHeader));
    else attron(COLOR_PAIR(PalMuted));
    for (size_t c = 0; c < cols.size(); ++c) {
        const int next = (c + 1 < cols.size()) ? offs[c + 1] : width;
        mvprintw(y, left + offs[c], "%s",
                 fitCell(cols[c], next - offs[c] - 1).c_str());
    }
    if (focused) attroff(A_BOLD | COLOR_PAIR(PalHeader));
    else attroff(COLOR_PAIR(PalMuted));
    ++y;

    // Rows
    const size_t n = model.visibleCount();
    const int avail = std::max(1, bottom - y - 1);   // reserve status line
    model.ensureVisible(avail);
    const size_t scroll = model.scrollOffset();
    const size_t sel = model.selectedIndex();
    for (size_t vi = scroll; vi < n && y < bottom - 1; ++vi, ++y) {
        const UI::Row* r = model.visibleRow(vi);
        if (!r) continue;
        const short pal = rowPalette(r->color);
        if (vi == sel && focused) attron(A_REVERSE);
        if (pal != PalNone) attron(COLOR_PAIR(pal));
        for (size_t c = 0; c < cols.size(); ++c) {
            static const std::string kEmpty;
            const std::string& cell =
                c < r->cells.size() ? r->cells[c] : kEmpty;
            const int next = (c + 1 < cols.size()) ? offs[c + 1] : width;
            mvprintw(y, left + offs[c], "%s",
                     fitCell(cell, next - offs[c]).c_str());
        }
        if (pal != PalNone) attroff(COLOR_PAIR(pal));
        if (vi == sel && focused) attroff(A_REVERSE);
    }

    // Status line: counts + active filter/search + scroll indicator.
    std::string status =
        std::to_string(n) + "/" + std::to_string(model.rowCount()) + " rows";
    if (scroll > 0) status += " scroll=" + std::to_string(scroll + 1);
    if (!model.search().empty()) status += "  /" + model.search();
    if (!model.filterExpr().empty()) status += "  f:" + model.filterExpr();
    attron(COLOR_PAIR(PalMuted));
    mvprintw(bottom - 1, left, "%s", fitCell(status, width).c_str());
    attroff(COLOR_PAIR(PalMuted));

    return (y - top) + 1;
}

} // namespace TUI
} // namespace Tether
