#pragma once

/**
 * @file ChipPanel.hpp
 * @brief ncurses renderer for the Tether::UI chip-panel model
 *        (`tether/ui/ChipPanels.hpp`).
 *
 * `drawChipPanelBox()` draws one titled box with flow-wrapped,
 * tone-colored chip text; `drawChipPanelGrid()` lays a panel list out in
 * an adaptive column grid (up to 4 columns on wide terminals) with all
 * panels in a grid row padded to equal height — the shape used for
 * status dashboards made of many small titled sections.
 *
 * Tone -> palette mapping (Tether::TUI `Pal*` pairs):
 *   Ok      -> PalValue (green)
 *   Missing -> PalError (red)
 *   Fault   -> PalError + bold
 *   Warn    -> PalHint  (yellow)
 *   Info    -> PalInfo  (blue)
 *   Muted   -> PalMuted (dim)
 *
 * Used by the ESC211 RSAP views (esc211_fsoe_monitor /
 * esc211_fsoe_blackchannel) and suitable for any dashboard-style view.
 */

#include <string>
#include <vector>

#include "tether/ui/ChipPanels.hpp"

namespace Tether {
namespace TUI {

/// Map a chip-panel DisplayTone to a TUI palette index (Pal*).
short chipTonePair(UI::DisplayTone tone);

/// Number of inner (wrapped) text lines a panel needs at width
/// `inner_w` — the box adds a top and bottom border line on top.
int chipPanelContentLines(const UI::DisplayPanel& panel, int inner_w);

/// Draw one boxed chip panel at (x, y) with width w.  Content is
/// flow-wrapped inside `w-2` inner columns, clipped at `max_h` total
/// rows and padded with empty lines to `min_lines` content rows so
/// panels in a grid row line up.  Returns the rows consumed (0 when
/// `max_h < 3` or `w < 4`).
int drawChipPanelBox(int x, int y, int w,
                     const UI::DisplayPanel& panel,
                     int max_h, int min_lines);

/// Lay out chip panels in an adaptive column grid inside [top, bottom):
/// 4 columns at COLS >= 150, 2 at COLS >= 92, else 1; the last column
/// stretches to the screen edge.  Panels in a grid row share the height
/// of the tallest one (clipped to the remaining rows).  Returns the
/// rows consumed.
int drawChipPanelGrid(int top, int bottom,
                      const std::vector<UI::DisplayPanel>& panels);

} // namespace TUI
} // namespace Tether
