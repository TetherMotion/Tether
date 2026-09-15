/**
 * @file Session.hpp
 * @brief RAII ncurses session — the base every TUI screen runs inside
 *
 * Session owns the ncurses lifecycle: setlocale, initscr, cbreak/noecho,
 * hidden cursor, keypad mode, nodelay input, and the standard Tether color
 * palette.  Constructing it takes over the terminal; destroying it calls
 * endwin() and restores the previous screen.
 *
 * ncurses types never appear in this header — windows are passed around as
 * opaque TermWindow* so that including this file does not drag in the
 * ncurses macros (OK/ERR/timeout/...) that collide with Tether identifiers.
 *
 * Use Session::available() to decide between the interactive UI and a
 * plain-text fallback before constructing one.
 */

#pragma once

#include <cstdint>

namespace Tether {
namespace TUI {

/// Opaque ncurses WINDOW.  TUI headers never expose ncurses itself.
using TermWindow = void;

/// Standard color pairs (usable as attron(COLOR_PAIR(n)) in render code,
/// or via the palette() constants).  0 means "no color attribute".
enum Palette : short {
    PalNone     = 0,
    PalValue    = 1,   ///< green — live on/true values
    PalHeader   = 2,   ///< cyan  — titles, coupler nodes
    PalHint     = 3,   ///< yellow — key hints, footer
    PalError    = 4,   ///< red   — errors, captured log lines
    PalSelected = 5,   ///< magenta — emphasis inside detail panes
    PalMuted    = 6,   ///< gray/white — inactive or invalid items
    PalInfo     = 7,   ///< blue   — informational section roots
};

class Session {
public:
    /// Takes over the terminal.  Check available() first; constructing on a
    /// non-terminal produces garbage escape output.
    Session();
    /// Restores the terminal (endwin).
    ~Session();

    Session(const Session&) = delete;
    Session& operator=(const Session&) = delete;

    /// True when interactive mode can run: stdout is a TTY and TERM names a
    /// usable terminal type.  Safe to call without a Session.
    static bool available();

    /// Whether the terminal supports the color palette.
    bool colors() const { return colors_; }

    int rows() const;
    int cols() const;

    /// Poll one key, waiting at most timeout_ms (-1 = block, 0 = nonblock).
    /// Returns -1 on timeout, else the curses key code (KEY_* etc.).
    int pollKey(int timeout_ms);

    /// Create a subwindow.  Rows/cols follow curses convention (y, x order).
    TermWindow* makeWindow(int y, int x, int h, int w) const;
    /// Free a window created by makeWindow().
    void freeWindow(TermWindow* win) const;

private:
    bool colors_ = false;
};

} // namespace TUI
} // namespace Tether
