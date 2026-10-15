#pragma once

/// Sequential top-to-bottom writer for one ncurses window — replaces the
/// `int row = top; mvwprintw(win, row++, x, ...)` idiom and its hand-rolled
/// `row < h` clipping.
///
/// Two emit shapes cover the existing call-site patterns:
///
///   - whole-line emitters (`text`, `styled`, `spans`, `lamp`, `rule`)
///     write at the configured margin column and then advance the cursor;
///   - fixed-column emitters (`put`, `putSpans`, `putLamp`) write at an
///     absolute column on the CURRENT row without advancing — for
///     multi-column lines (`put(4, ...); put(16, ...); advance()`).
///
/// Everything clips at `bottom` (default: window height), so the
/// `row < h` guards scattered through render callbacks disappear; query
/// `remaining()`/`fits()` to stop emitting early.
///
/// stdscr usage: `RowWriter w(stdscr, top, 0, bottom)`.
///
/// `columnSplit` covers the other recurring console idiom — splitting a
/// width into N side-by-side columns that collapse to fewer columns when
/// the terminal is narrow.

#include <ncurses.h>
#include <cstdarg>
#include <cstdio>
#include <span>

#include "tether/terminal_ui/StyledText.hpp"
#include "tether/terminal_ui/Lamp.hpp"
#include "tether/terminal_ui/AnsiLine.hpp"   // utf8Cols

namespace Tether {
namespace TUI {

class RowWriter {
public:
    /// @param win     target window (stdscr allowed)
    /// @param top     first row to write
    /// @param x       left margin column for the line emitters
    /// @param bottom  exclusive lower bound; <0 = window height
    RowWriter(WINDOW* win, int top = 0, int x = 0, int bottom = -1)
        : win_(win), row_(top), x_(x),
          bottom_(bottom >= 0 ? bottom : (win ? getmaxy(win) : 0)) {}

    int  row()       const { return row_; }
    int  x()         const { return x_; }
    int  bottom()    const { return bottom_; }
    WINDOW* window() const { return win_; }
    /// Change the left margin used by the line emitters (indented
    /// sub-sections inside a detail pane).
    RowWriter& margin(int x) { x_ = x; return *this; }
    int  remaining() const { return bottom_ - row_ > 0 ? bottom_ - row_ : 0; }
    bool fits()      const { return row_ < bottom_; }
    explicit operator bool() const { return fits(); }

    /// Jump to an absolute row (clamped at bottom) — for resuming after
    /// a section that drew at computed coordinates (multi-column blocks).
    RowWriter& seek(int row) {
        row_ = row;
        if (row_ > bottom_) row_ = bottom_;
        return *this;
    }

    /// Move the cursor down `n` rows (clamped at bottom).  Use after
    /// same-row `put*` emits; `blank` reads better when the rows are
    /// visual spacing.
    RowWriter& advance(int n = 1) {
        row_ += n;
        if (row_ > bottom_) row_ = bottom_;
        return *this;
    }
    RowWriter& blank(int n = 1) { return advance(n); }

    // -- whole-line emitters (margin column, then advance) ------------------

    RowWriter& text(const char* fmt, ...) {
        va_list ap; va_start(ap, fmt);
        emitV(x_, 0, fmt, ap, true);
        va_end(ap);
        return *this;
    }

    RowWriter& styled(int attrs, const char* fmt, ...) {
        va_list ap; va_start(ap, fmt);
        emitV(x_, attrs, fmt, ap, true);
        va_end(ap);
        return *this;
    }

    RowWriter& spans(std::span<const Span> s) {
        if (fits()) wputSpans(win_, row_, x_, s);
        return advance();
    }

    RowWriter& lamp(LampState s, LampStyle st = kLampDots) {
        if (fits()) wdrawLamp(win_, row_, x_, s, st);
        return advance();
    }

    /// Horizontal rule across the window's width, then advance.
    RowWriter& rule(char ch = '-', int attrs = 0) {
        if (fits()) {
            WAttrScope scope(win_, attrs);
            mvwhline(win_, row_, x_, ch, getmaxx(win_) - x_);
        }
        return advance();
    }

    // -- same-row emitters (absolute column, NO advance) --------------------
    // For multi-column lines: put(...) fragments, then advance().

    /// Write at absolute `col` on the current row; returns columns written.
    int put(int col, int attrs, const char* fmt, ...) {
        va_list ap; va_start(ap, fmt);
        const int n = emitV(col, attrs, fmt, ap, false);
        va_end(ap);
        return n;
    }

    int putSpans(int col, std::span<const Span> s) {
        if (!fits()) return 0;
        return wputSpans(win_, row_, col, s);
    }

    RowWriter& putLamp(int col, LampState s, LampStyle st = kLampDots) {
        if (fits()) wdrawLamp(win_, row_, col, s, st);
        return *this;
    }

private:
    /// Shared printf path: write `fmt` at (row_, col) if it fits, then
    /// advance when `bump`.  Returns columns written.
    int emitV(int col, int attrs, const char* fmt, va_list ap, bool bump) {
        if (!fits()) return 0;
        char buf[1024];
        vsnprintf(buf, sizeof(buf), fmt, ap);
        WAttrScope scope(win_, attrs);
        mvwprintw(win_, row_, col, "%s", buf);
        if (bump) ++row_;
        return static_cast<int>(utf8Cols(buf));
    }

    WINDOW* win_;
    int row_, x_, bottom_;
};

// ---------------------------------------------------------------------------
// Column split — N side-by-side regions that collapse when narrow
// ---------------------------------------------------------------------------

/// Geometry of an even column split: `cols` regions of `width` cells with
/// `gap` columns between them; `x(i)` gives the left edge of column i.
struct ColumnGeom {
    int cols  = 1;
    int width = 0;
    int gap   = 1;
    int x(int i) const { return i * (width + gap); }
};

/// Split `total_w` into up to `desired` side-by-side columns.  Each column
/// gets `min(max_w, available/cols)` cells; columns are dropped while that
/// per-column width would fall below `min_w`.  Always returns >= 1 column.
///
/// Replaces the `two_col = COLS >= 70; col_w = min(54, COLS/2 - 1);
/// out_x = two_col ? col_w : 0` hand math — `columnSplit(COLS, 2, 34, 54)`
/// expresses the same layout.
inline ColumnGeom columnSplit(int total_w, int desired, int min_w,
                              int max_w, int gap = 1) {
    ColumnGeom g;
    g.gap = gap;
    for (int n = desired; n >= 1; --n) {
        const int avail = total_w - (n - 1) * gap;
        int w = avail / n;
        if (w > max_w) w = max_w;
        if (w >= min_w || n == 1) {
            g.cols = n;
            g.width = w > 0 ? w : 1;
            return g;
        }
    }
    return g;
}

} // namespace TUI
} // namespace Tether
