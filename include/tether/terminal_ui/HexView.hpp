#pragma once

/// Hex byte dump with optional field highlighting: `00 11 22 ...` where
/// byte ranges (e.g. command byte, connection id, CRC) are tinted by
/// palette attribute.  More useful than a flat bytesToHex for protocol
/// frames whose field layout is known.

#include <ncurses.h>
#include <cstddef>
#include <cstdint>
#include <span>

#include "tether/terminal_ui/Session.hpp"     // Palette
#include "tether/terminal_ui/StyledText.hpp"  // putStyled, AttrScope

namespace Tether {
namespace TUI {

/// A highlighted byte range inside a dump line.
struct HexField {
    size_t off = 0;
    size_t len = 0;
    int attrs = COLOR_PAIR(PalHeader);
};

/// Draw `data` as space-separated hex bytes at (y, x), clipped to `max_cols`
/// screen columns (appends "…" when truncated — returns the displayed byte
/// count).  Bytes inside any `fields` range are drawn with that field's
/// attrs; first matching field wins.  Plain bytes use `plain_attrs` (0 =
/// default color).  Returns columns written.
inline int drawHexLine(int y, int x, int max_cols,
                       std::span<const uint8_t> data,
                       std::span<const HexField> fields = {},
                       int plain_attrs = 0) {
    auto fieldFor = [&](size_t i) -> int {
        for (const auto& f : fields)
            if (i >= f.off && i < f.off + f.len) return f.attrs;
        return plain_attrs;
    };

    int col = x;
    size_t shown = 0;
    for (; shown < data.size(); ++shown) {
        // each byte needs "hh" (+1 space unless last); reserve 1 for '…'
        const int need = (shown + 1 < data.size() ? 4 : 3);
        if (col - x + need > max_cols) break;
        AttrScope scope(fieldFor(shown));
        col += putStyled(y, col, 0, "%02X ", data[shown]);
    }
    if (shown < data.size() && col - x < max_cols)
        col += putStyled(y, col, COLOR_PAIR(PalMuted), "…");
    return col - x;
}

} // namespace TUI
} // namespace Tether
