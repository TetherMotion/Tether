#pragma once

/**
 * @file AnsiLine.hpp
 * @brief UTF-8 measuring + ANSI SGR-to-curses line rendering helpers.
 *
 * `renderAnsiLine()` renders text that may contain embedded ANSI SGR
 * sequences (`ESC[...m`) into a curses window, mapping basic/bright/
 * 256-color foregrounds to the `Pal*` palette and honoring bold /
 * reset / default-foreground codes.  Unsupported sequences (background,
 * truecolor, non-SGR CSI finals) are approximated or dropped, and an
 * unterminated trailing escape drops the tail rather than printing it —
 * captured log lines and colored console output can be replayed safely.
 *
 * `utf8Cols`/`utf8Trunc` measure and clip UTF-8 text by display columns
 * without splitting multibyte codepoints (invalid bytes count as one
 * column each) — the same convention TableView/TreeView use.
 */

#include <string>

#include <ncurses.h>

namespace Tether {
namespace TUI {

/// Display-column width of a UTF-8 string (invalid bytes count 1 each).
int utf8Cols(const std::string& s);

/// Truncate a UTF-8 string to at most `max_cols` display columns
/// without splitting a multibyte codepoint.
std::string utf8Trunc(const std::string& s, int max_cols);

/// Map an ANSI SGR basic foreground index (0-7) to the palette pair
/// carrying that color (black renders as muted — invisible on black).
short ansiFgPair(int color_index);

/**
 * @brief Render one line into `win` at (`row`, `col`), interpreting
 *        embedded ANSI SGR sequences as curses colors.
 *
 * @param win       target window (stdscr or a pane window).
 * @param row, col  start position in window coordinates.
 * @param width     maximum display columns to occupy from `col`.
 * @param text      line text; may contain `ESC[...m` sequences.
 * @param base_pair palette pair used for unstyled text and SGR 0/39 —
 *                  e.g. a severity color, or PalNone for default.
 */
void renderAnsiLine(WINDOW* win, int row, int col, int width,
                    const std::string& text, short base_pair);

} // namespace TUI
} // namespace Tether
