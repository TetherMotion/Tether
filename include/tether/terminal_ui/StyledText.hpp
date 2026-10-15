#pragma once

/// Styled inline text helpers: replaces the `attron(...); mvprintw(...);
/// attroff(...)` triplet and its manual column tracking with functions that
/// return the column width written, so multi-part status lines chain as
/// `col += putStyled(...)` / `col += putSpans(...)`.
///
/// NOTE: consumers `#undef ERR`/`OK` after <ncurses.h> (the macros collide
/// with Tether error enums), so these helpers must only use mvprintw-family
/// calls — mvaddnstr/mvaddch macros expand `wmove()==ERR` internally and
/// would not compile in those translation units.

#include <ncurses.h>
#include <cstdarg>
#include <cstdio>
#include <span>
#include <string>
#include <vector>

#include "tether/terminal_ui/AnsiLine.hpp"  // utf8Cols

namespace Tether {
namespace TUI {

/// COLOR_PAIR() yields `chtype`; Span::attrs is int, so braced
/// initializers need this constexpr cast to avoid narrowing errors.
constexpr int pairAttrs(short pair) {
    return static_cast<int>(COLOR_PAIR(pair));
}

/// RAII attribute scope: `attron(attrs)` on entry, `attroff(attrs)` on exit.
struct AttrScope {
    explicit AttrScope(int attrs) : attrs_(attrs) {
        if (attrs_) ::attron(static_cast<int>(attrs_));
    }
    ~AttrScope() {
        if (attrs_) ::attroff(static_cast<int>(attrs_));
    }
    AttrScope(const AttrScope&) = delete;
    AttrScope& operator=(const AttrScope&) = delete;

private:
    int attrs_;
};

/// Print formatted text at (y, x) under `attrs` (may combine COLOR_PAIR(n)
/// with A_BOLD etc; 0 = plain).  Returns the number of screen columns
/// written (UTF-8 aware), for `col +=` chaining.
inline int putStyled(int y, int x, int attrs, const char* fmt, ...) {
    char buf[1024];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    AttrScope scope(attrs);
    mvprintw(y, x, "%s", buf);
    return static_cast<int>(utf8Cols(buf));
}

/// Convenience for the common `COLOR_PAIR(pair)` case.
inline int putPair(int y, int x, short pair, int extra_attrs,
                   const char* fmt, ...) {
    char buf[1024];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    AttrScope scope((pair ? pairAttrs(pair) : 0) | extra_attrs);
    mvprintw(y, x, "%s", buf);
    return static_cast<int>(utf8Cols(buf));
}

/// A styled text run: `attrs` may combine pairAttrs(pair) with attribute
/// bits (0 = default colors).
struct Span {
    std::string text;
    int attrs = 0;
};

/// Draw a row of styled spans left to right; returns total columns written.
inline int putSpans(int y, int x, std::span<const Span> spans) {
    int col = x;
    for (const auto& s : spans) {
        AttrScope scope(s.attrs);
        mvprintw(y, col, "%s", s.text.c_str());
        col += static_cast<int>(utf8Cols(s.text));
    }
    return col - x;
}

/// Width of a span list without drawing (for right-aligning segments).
inline int spansWidth(std::span<const Span> spans) {
    int w = 0;
    for (const auto& s : spans) w += static_cast<int>(utf8Cols(s.text));
    return w;
}

} // namespace TUI
} // namespace Tether
