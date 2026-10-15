#pragma once

/**
 * @file Gauge.hpp
 * @brief Inline ncurses utilization-gauge widgets for TUI views.
 *
 * `drawGauge`        — `value/limit` fill bar `[###....]` colored by
 *                      fill ratio (green < 70 %, yellow < 90 %, red
 *                      above / when `violated`), with a `>` overflow
 *                      marker past 100 %.  `ratio < 0` renders an empty
 *                      muted bar ("not applicable").
 * `drawRangeTrack`   — a `[-----#-----]` track over an allowed
 *                      [low, high] range with the current value as a
 *                      marker: green inside (yellow in the outer 10 %
 *                      of the span), red when outside or `violated`.
 *
 * Both are plain `stdscr` draws — views decide column layout; pass the
 * bar column and width.  Used by the ESC211 "Limits" view and suitable
 * for any value-versus-limit display.
 */

#include <ncurses.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <format>
#include <optional>

#include "tether/terminal_ui/Session.hpp"

namespace Tether {
namespace TUI {

/// Fill ratio -> palette pair for utilization bars: muted for n/a,
/// green/yellow/red thresholds, red when the row is flagged violated.
inline short gaugeRatioPair(double ratio, bool violated) {
    if (ratio < 0) return PalMuted;
    if (violated || ratio >= 0.9) return PalError;
    if (ratio >= 0.7) return PalHint;
    return PalValue;
}

/// Draw one utilization bar `[###....]` of width `w` at (y, x) in window
/// `win`.  `ratio` 0..1 fills left to right; >1 clamps and draws a red
/// `>` overflow marker after the bar; <0 renders an empty muted bar.
/// `violated` forces the bar red+bold regardless of the ratio.
inline void wdrawGauge(WINDOW* win, int y, int x, int w,
                       double ratio, bool violated) {
    if (w < 4) return;
    const int pair = gaugeRatioPair(ratio, violated);
    const int attrs = violated ? A_BOLD : 0;
    const int inner = w - 2;
    const int filled = ratio < 0
        ? 0
        : std::clamp(static_cast<int>(std::lround(ratio * inner)),
                     0, inner);
    wattron(win, COLOR_PAIR(pair) | attrs);
    mvwprintw(win, y, x, "[");
    for (int i = 0; i < filled; ++i) mvwprintw(win, y, x + 1 + i, "#");
    wattroff(win, COLOR_PAIR(pair) | attrs);
    wattron(win, COLOR_PAIR(PalMuted));
    for (int i = filled; i < inner; ++i) mvwprintw(win, y, x + 1 + i, ".");
    mvwprintw(win, y, x + w - 1, "]");
    wattroff(win, COLOR_PAIR(PalMuted));
    if (ratio > 1.0) {
        wattron(win, COLOR_PAIR(PalError) | A_BOLD);
        mvwprintw(win, y, x + w, ">");
        wattroff(win, COLOR_PAIR(PalError) | A_BOLD);
    }
}

/// stdscr convenience — same as wdrawGauge(stdscr, ...).
inline void drawGauge(int y, int x, int w, double ratio, bool violated) {
    wdrawGauge(stdscr, y, x, w, ratio, violated);
}

/// Draw a range track `[-----#-----]` of width `w` at (y, x) in window
/// `win`: the allowed span is [low, high] and `value` is drawn as a `#`
/// marker at its position — clamped to the track ends when outside.
/// The marker is green inside (yellow within the outer 10 % of the
/// span), red when outside or `violated`.  Without a value only the
/// empty track shows.  Falls back to printing "low/high" when the track
/// does not fit.
inline void wdrawRangeTrack(WINDOW* win, int y, int x, int w,
                            int64_t low, int64_t high,
                            std::optional<int64_t> value, bool violated) {
    if (w < 6 || high <= low) {
        mvwprintw(win, y, x, "%lld/%lld", static_cast<long long>(low),
                  static_cast<long long>(high));
        return;
    }
    const int inner = w - 2;
    wattron(win, COLOR_PAIR(PalMuted));
    mvwprintw(win, y, x, "[");
    for (int i = 0; i < inner; ++i) mvwprintw(win, y, x + 1 + i, "-");
    mvwprintw(win, y, x + w - 1, "]");
    wattroff(win, COLOR_PAIR(PalMuted));
    if (!value) return;
    const double frac = static_cast<double>(*value - low) /
                        static_cast<double>(high - low);
    const bool inside = (*value >= low && *value <= high);
    // Closeness to the nearer bound, 0..1 (1 = dead centre).
    const double margin_frac = inside
        ? 1.0 - 2.0 * std::fabs(frac - 0.5)
        : -1.0;
    const int pair = !inside || violated ? PalError
                     : margin_frac < 0.2 ? PalHint
                                         : PalValue;
    const int pos = std::clamp(static_cast<int>(std::lround(
                                   frac * (inner - 1))),
                               0, inner - 1);
    wattron(win, COLOR_PAIR(pair) | A_BOLD);
    mvwprintw(win, y, x + 1 + pos, "#");
    wattroff(win, COLOR_PAIR(pair) | A_BOLD);
}

/// stdscr convenience — same as wdrawRangeTrack(stdscr, ...).
inline void drawRangeTrack(int y, int x, int w, int64_t low, int64_t high,
                           std::optional<int64_t> value, bool violated) {
    wdrawRangeTrack(stdscr, y, x, w, low, high, value, violated);
}

} // namespace TUI
} // namespace Tether
