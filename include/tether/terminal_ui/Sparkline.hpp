#pragma once

/// ncurses renderer for the toolkit-agnostic sparkline model
/// (ui/Sparkline.hpp): `▁▂▃▄▅▆▇█` trend history in `width` columns.

#include <ncurses.h>
#include <cstddef>
#include <span>
#include <string>

#include "tether/ui/Sparkline.hpp"
#include "tether/terminal_ui/StyledText.hpp"

namespace Tether {
namespace TUI {

/// Draw `samples` (chronological) as a `width`-column sparkline in
/// window `w`, quantized onto [lo, hi].  Returns `width`.
inline int wdrawSparkline(WINDOW* w, int y, int x, int width,
                          std::span<const double> samples,
                          double lo, double hi, int attrs = 0) {
    const std::string s =
        UI::renderSparkline(samples, static_cast<size_t>(width), lo, hi);
    WAttrScope scope(w, attrs);
    mvwprintw(w, y, x, "%s", s.c_str());
    return width;
}

/// stdscr convenience — same as wdrawSparkline(stdscr, ...).
inline int drawSparkline(int y, int x, int width,
                         std::span<const double> samples,
                         double lo, double hi, int attrs = 0) {
    return wdrawSparkline(stdscr, y, x, width, samples, lo, hi, attrs);
}

/// Convenience: draw a SparklineHistory's samples into a window.
template <size_t Capacity>
inline int wdrawSparkline(WINDOW* w, int y, int x, int width,
                          const UI::SparklineHistory<Capacity>& history,
                          double lo, double hi, int attrs = 0) {
    const auto samples = history.samples();
    return wdrawSparkline(w, y, x, width, samples, lo, hi, attrs);
}

/// stdscr convenience: draw a SparklineHistory's samples directly.
template <size_t Capacity>
inline int drawSparkline(int y, int x, int width,
                         const UI::SparklineHistory<Capacity>& history,
                         double lo, double hi, int attrs = 0) {
    const auto samples = history.samples();
    return drawSparkline(y, x, width, samples, lo, hi, attrs);
}

} // namespace TUI
} // namespace Tether
