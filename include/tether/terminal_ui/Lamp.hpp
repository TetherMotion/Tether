#pragma once

/// Tri-state indicator lamp: renders on/off/unknown as a styled glyph.
/// Two ready-made styles cover the common cases:
///   kLampDots  — "●" green-bold / "○" muted / "?" muted  (pin/flag maps)
///   kLampTags  — " ON"/" off"/"  ?"                      (columnar lists)

#include <ncurses.h>
#include "tether/terminal_ui/Session.hpp"      // Palette
#include "tether/terminal_ui/StyledText.hpp"   // putStyled
#include "tether/terminal_ui/AnsiLine.hpp"     // utf8Cols

namespace Tether {
namespace TUI {

enum class LampState : uint8_t { Off, On, Unknown };

struct LampStyle {
    const char* on;
    const char* off;
    const char* unknown;
    int on_attrs;
    int off_attrs;
    int unknown_attrs;
};

inline constexpr LampStyle kLampDots = {
    "●", "○", "?",
    COLOR_PAIR(PalValue) | A_BOLD,
    COLOR_PAIR(PalMuted),
    COLOR_PAIR(PalMuted),
};

inline constexpr LampStyle kLampTags = {
    " ON", " off", "  ?",
    COLOR_PAIR(PalValue),
    COLOR_PAIR(PalMuted),
    COLOR_PAIR(PalHint),
};

/// "1"/"0" bit columns (dense packed-bit displays like FSoE inputs).
inline constexpr LampStyle kLampDigits = {
    "1", "0", "?",
    COLOR_PAIR(PalValue) | A_BOLD,
    A_DIM,
    A_DIM,
};

/// "*"/"." bit columns (sparse terminal channel rows).
inline constexpr LampStyle kLampBits = {
    "*", ".", "?",
    COLOR_PAIR(PalValue) | A_BOLD,
    A_DIM,
    A_DIM,
};

/// "*"/"." bit columns where "on" is a fault/flag condition (red).
inline constexpr LampStyle kLampAlertBits = {
    "*", ".", "?",
    COLOR_PAIR(PalError) | A_BOLD,
    A_DIM,
    A_DIM,
};

/// "■"/"□" square bit columns (beckhoff_tui detail panes).
inline constexpr LampStyle kLampSquares = {
    "\xE2\x96\xA0", "\xE2\x96\xA1", "?",
    COLOR_PAIR(PalValue) | A_BOLD,
    A_DIM,
    A_DIM,
};

/// Draw a lamp at (y, x) in window `w`; returns the glyph's column width.
inline int wdrawLamp(WINDOW* w, int y, int x, LampState s,
                     const LampStyle& style = kLampDots) {
    const char* glyph;
    int attrs;
    switch (s) {
        case LampState::On:      glyph = style.on;      attrs = style.on_attrs;      break;
        case LampState::Off:     glyph = style.off;     attrs = style.off_attrs;     break;
        default:                 glyph = style.unknown; attrs = style.unknown_attrs; break;
    }
    return wputStyled(w, y, x, attrs, "%s", glyph);
}

/// stdscr convenience — same as wdrawLamp(stdscr, ...).
inline int drawLamp(int y, int x, LampState s,
                    const LampStyle& style = kLampDots) {
    return wdrawLamp(stdscr, y, x, s, style);
}

} // namespace TUI
} // namespace Tether
