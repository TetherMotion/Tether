#pragma once

/// Styled inline text helpers: replaces the `attron(...); mvprintw(...);
/// attroff(...)` triplet and its manual column tracking with functions that
/// return the column width written, so multi-part status lines chain as
/// `col += putStyled(...)` / `col += putSpans(...)`.
///
/// NOTE: translation units that `#undef ERR`/`OK` after <ncurses.h> (the
/// macros collide with Tether error enums) can still include this header —
/// the helpers only use mvprintw-family calls, since the mvaddnstr/mvaddch
/// macros expand `wmove()==ERR` internally and would not compile there.

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

/// RAII attribute scope on a window: `wattron(w, attrs)` on entry,
/// `wattroff(w, attrs)` on exit.
struct WAttrScope {
    WAttrScope(WINDOW* w, int attrs) : win_(w), attrs_(attrs) {
        if (attrs_) ::wattron(win_, static_cast<int>(attrs_));
    }
    ~WAttrScope() {
        if (attrs_) ::wattroff(win_, static_cast<int>(attrs_));
    }
    WAttrScope(const WAttrScope&) = delete;
    WAttrScope& operator=(const WAttrScope&) = delete;

private:
    WINDOW* win_;
    int attrs_;
};

/// stdscr convenience scope — same as WAttrScope(stdscr, attrs).
struct AttrScope : WAttrScope {
    explicit AttrScope(int attrs) : WAttrScope(stdscr, attrs) {}
};

/// Print formatted text at (y, x) in window `w` under `attrs` (may
/// combine COLOR_PAIR(n) with A_BOLD etc; 0 = plain).  Returns the
/// number of screen columns written (UTF-8 aware), for `col +=`
/// chaining.
inline int wputStyled(WINDOW* w, int y, int x, int attrs,
                      const char* fmt, ...) {
    char buf[1024];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    WAttrScope scope(w, attrs);
    mvwprintw(w, y, x, "%s", buf);
    return static_cast<int>(utf8Cols(buf));
}

/// stdscr convenience — same as wputStyled(stdscr, ...).
inline int putStyled(int y, int x, int attrs, const char* fmt, ...) {
    char buf[1024];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    WAttrScope scope(stdscr, attrs);
    mvprintw(y, x, "%s", buf);
    return static_cast<int>(utf8Cols(buf));
}

/// Window variant of putPair — the common COLOR_PAIR(pair) case.
inline int wputPair(WINDOW* w, int y, int x, short pair, int extra_attrs,
                    const char* fmt, ...) {
    char buf[1024];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    WAttrScope scope(w, (pair ? pairAttrs(pair) : 0) | extra_attrs);
    mvwprintw(w, y, x, "%s", buf);
    return static_cast<int>(utf8Cols(buf));
}

/// stdscr convenience — same as wputPair(stdscr, ...).
inline int putPair(int y, int x, short pair, int extra_attrs,
                   const char* fmt, ...) {
    char buf[1024];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    WAttrScope scope(stdscr, (pair ? pairAttrs(pair) : 0) | extra_attrs);
    mvprintw(y, x, "%s", buf);
    return static_cast<int>(utf8Cols(buf));
}

/// A styled text run: `attrs` may combine pairAttrs(pair) with attribute
/// bits (0 = default colors).
struct Span {
    std::string text;
    int attrs = 0;
};

/// Draw a row of styled spans left to right in window `w`;
/// returns total columns written.
inline int wputSpans(WINDOW* w, int y, int x, std::span<const Span> spans) {
    int col = x;
    for (const auto& s : spans) {
        WAttrScope scope(w, s.attrs);
        mvwprintw(w, y, col, "%s", s.text.c_str());
        col += static_cast<int>(utf8Cols(s.text));
    }
    return col - x;
}

/// stdscr convenience — same as wputSpans(stdscr, ...).
inline int putSpans(int y, int x, std::span<const Span> spans) {
    return wputSpans(stdscr, y, x, spans);
}

/// Width of a span list without drawing (for right-aligning segments).
inline int spansWidth(std::span<const Span> spans) {
    int w = 0;
    for (const auto& s : spans) w += static_cast<int>(utf8Cols(s.text));
    return w;
}

} // namespace TUI
} // namespace Tether
