#pragma once

/**
 * @file Box.hpp
 * @brief UTF-8 box-drawing primitives + ready-made boxed content widgets.
 *
 * The primitives (`drawBoxTop`/`drawBoxRule`/`drawBoxBottom`/
 * `drawBoxInnerRow`/`drawBoxFrame`) deduplicate the `┌─┐│├─┤└─┘`
 * border code that panels otherwise copy around.  The content widgets
 * built on them cover the two recurring "status box" shapes:
 *
 *   - `drawKeyValueBox`  — a two-column "Object | Value" table box with
 *                          per-row value colors (system-state panels,
 *                          register summaries).
 *   - `drawBoxedLines`   — a titled box around verbatim text lines
 *                          (message lists, dumps).
 *
 * All functions draw with plain mvprintw on stdscr, clip to COLS,
 * and return the number of rows consumed so callers can chain widgets
 * vertically (`y += drawKeyValueBox(...)`).
 */

#include <ncurses.h>

#include <algorithm>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <vector>

#include "tether/terminal_ui/ChipPanel.hpp"   // chipTonePair
#include "tether/terminal_ui/Session.hpp"    // PalNone
#include "tether/terminal_ui/TextWrap.hpp"   // padToWidth
#include "tether/ui/ChipPanels.hpp"          // DisplayTone

namespace Tether {
namespace TUI {

// ---------------------------------------------------------------------------
// Primitives
// ---------------------------------------------------------------------------

/// `┌─ title ────┐` top border (bold).  `title` is clipped to fit.
inline void drawBoxTop(int y, int x, int w, std::string_view title = {},
                       bool bold = true) {
    if (w < 2) return;
    std::string top = "┌";
    int used = 0;
    if (!title.empty()) {
        std::string t = " " + std::string(title);
        if (static_cast<int>(t.size()) > w - 3) {
            t.resize(static_cast<size_t>(std::max(0, w - 3)));
        }
        top += t;
        used += static_cast<int>(t.size());
    }
    while (used < w - 2) { top += "─"; ++used; }
    top += "┐";
    if (bold) attron(A_BOLD);
    mvprintw(y, x, "%s", top.c_str());
    if (bold) attroff(A_BOLD);
}

/// `├───┤` interior horizontal rule.
inline void drawBoxRule(int y, int x, int w) {
    if (w < 2) return;
    std::string r = "├";
    for (int i = 0; i < w - 2; ++i) r += "─";
    r += "┤";
    mvprintw(y, x, "%s", r.c_str());
}

/// `└───┘` bottom border.
inline void drawBoxBottom(int y, int x, int w) {
    if (w < 2) return;
    std::string r = "└";
    for (int i = 0; i < w - 2; ++i) r += "─";
    r += "┘";
    mvprintw(y, x, "%s", r.c_str());
}

/// `│<content padded to w-2>│` — one inner row with side borders.
/// `content` is clipped/padded to exactly w-2 columns.
inline void drawBoxInnerRow(int y, int x, int w,
                            std::string_view content = {}) {
    if (w < 2) return;
    mvprintw(y, x, "│%s│",
             padToWidth(std::string(content),
                        static_cast<size_t>(w - 2)).c_str());
}

/// Finish an inner row after `filled` columns of content were already
/// drawn at x+1..x+filled: pads the rest with spaces and draws the
/// right border.
inline void padBoxRowEnd(int y, int x, int w, int filled) {
    if (w < 2) return;
    mvprintw(y, x, "│");
    for (int i = filled; i < w - 2; ++i) mvprintw(y, x + 1 + i, " ");
    mvprintw(y, x + w - 1, "│");
}

/// Full frame: top border (optional title), `h-2` blank inner rows,
/// bottom border.  Returns the rows consumed (== h, or 0 when h<2 or
/// w<4).
inline int drawBoxFrame(int y, int x, int w, int h,
                        std::string_view title = {}) {
    if (h < 2 || w < 4) return 0;
    drawBoxTop(y, x, w, title);
    for (int i = 1; i < h - 1; ++i) drawBoxInnerRow(y + i, x, w);
    drawBoxBottom(y + h - 1, x, w);
    return h;
}

// ---------------------------------------------------------------------------
// Content widgets
// ---------------------------------------------------------------------------

/// One row of a key|value box; `value_tone` colors only the value cell
/// (std::nullopt draws the value uncolored).
struct KvRow {
    std::string key;
    std::string value;
    std::optional<UI::DisplayTone> value_tone;
};

/**
 * @brief Two-column "Object | Value" table box on stdscr.
 *
 * ```
 * ┌ Object ────────┬ Value ─────────┐
 * │ System state   │ MONITORING     │
 * └────────────────┴────────────────┘
 * ```
 *
 * The key column auto-sizes to the widest key; the value column takes
 * the remainder.  `key_header`/`value_header` are drawn (bold) when
 * non-empty — pass an empty header to skip the header row + rule.
 * Returns the rows consumed.
 */
inline int drawKeyValueBox(int y, std::span<const KvRow> rows,
                           std::string_view key_header = " Object ",
                           std::string_view value_header = " Value ") {
    if (COLS < 8) return 0;
    const int w = COLS;
    size_t max_key = 0;
    for (const auto& r : rows) max_key = std::max(max_key, r.key.size());
    max_key = std::max(max_key, key_header.size());
    int kw = static_cast<int>(max_key) + 2;
    kw = std::clamp(kw, 3, w - 6);
    const int vw = w - kw - 3;   // borders: │ + kw + │ + vw + │ == w

    int cy = y;
    // ┌─┬─┐ with the headers embedded
    {
        std::string top = "┌";
        for (int i = 0; i < kw; ++i) top += "─";
        top += "┬";
        for (int i = 0; i < vw; ++i) top += "─";
        top += "┐";
        mvprintw(cy++, 0, "%s", top.c_str());
    }
    const bool has_header = !key_header.empty() || !value_header.empty();
    if (has_header) {
        attron(A_BOLD);
        mvprintw(cy, 0, "│%s│%s│",
                 padToWidth(std::string(key_header),
                            static_cast<size_t>(kw)).c_str(),
                 padToWidth(std::string(value_header),
                            static_cast<size_t>(vw)).c_str());
        attroff(A_BOLD);
        ++cy;
        std::string sep = "├";
        for (int i = 0; i < kw; ++i) sep += "─";
        sep += "┼";
        for (int i = 0; i < vw; ++i) sep += "─";
        sep += "┤";
        mvprintw(cy++, 0, "%s", sep.c_str());
    }
    for (const auto& r : rows) {
        mvprintw(cy, 0, "│%s│",
                 padToWidth(" " + r.key + " ",
                            static_cast<size_t>(kw)).c_str());
        const short pair = r.value_tone ? chipTonePair(*r.value_tone)
                                        : PalNone;
        if (r.value_tone) attron(COLOR_PAIR(pair));
        if (r.value_tone == UI::DisplayTone::Fault) attron(A_BOLD);
        mvprintw(cy, kw + 2, "%s│",
                 padToWidth(" " + r.value + " ",
                            static_cast<size_t>(vw)).c_str());
        if (r.value_tone == UI::DisplayTone::Fault) attroff(A_BOLD);
        if (r.value_tone) attroff(COLOR_PAIR(pair));
        ++cy;
    }
    // └─┴─┘
    {
        std::string bot = "└";
        for (int i = 0; i < kw; ++i) bot += "─";
        bot += "┴";
        for (int i = 0; i < vw; ++i) bot += "─";
        bot += "┘";
        mvprintw(cy++, 0, "%s", bot.c_str());
    }
    return cy - y;
}

/**
 * @brief Titled box around verbatim text lines.
 *
 * ```
 * ┌ title ─────────────────────────┐
 * │ line 1                         │
 * │ line 2                         │
 * └────────────────────────────────┘
 * ```
 *
 * At most `max_lines` lines are drawn.  Returns the rows consumed.
 */
inline int drawBoxedLines(int y, int max_lines,
                          std::span<const std::string> lines,
                          std::string_view title = {}) {
    if (COLS < 8 || max_lines < 0) return 0;
    const int w = COLS;
    int cy = y;
    drawBoxTop(cy++, 0, w, title, /*bold=*/false);
    if (!title.empty()) drawBoxRule(cy++, 0, w);
    int n = 0;
    for (const auto& line : lines) {
        if (n >= max_lines || cy >= LINES) break;
        drawBoxInnerRow(cy++, 0, w, " " + line + " ");
        ++n;
    }
    drawBoxBottom(cy++, 0, w);
    return cy - y;
}

} // namespace TUI
} // namespace Tether
