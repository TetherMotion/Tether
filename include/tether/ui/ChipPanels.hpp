#pragma once

/**
 * @file ChipPanels.hpp
 * @brief Toolkit-agnostic "chip panel" display model + bitfield-to-chip
 *        builders for status dashboards.
 *
 * A `DisplayPanel` is a titled box of `DisplayRow`s; each row is a label
 * followed by space-separated `DisplayCell` "chips".  Every cell carries
 * a `DisplayTone` — a *semantic* color role the UI layer maps to its own
 * palette (Tether::TUI maps them to `Pal*` pairs):
 *
 *   Ok      — enabled / present / safe state            (green)
 *   Missing — expected but absent/inactive              (red)
 *   Fault   — active fault / violation / error          (red+bold)
 *   Warn    — transitional / needs attention            (yellow)
 *   Info    — informational value                       (cyan/blue)
 *   Muted   — inactive but acceptable / unknown         (dim)
 *
 * The tone assignment encodes object semantics.  `ChipSemantics` picks
 * the mapping for a bitfield value:
 *
 *   - `ExpectedSet`: every labeled bit *should* be set in a healthy
 *     configuration (valid flags, wired safety inputs, driven outputs).
 *     Set -> Ok chip, clear -> Missing chip (all labels shown so the
 *     missing elements are visible).
 *   - `ExpectedClear`: the healthy state is value==0 (fault/limit/
 *     discrepancy objects).  0 -> green "none"; every set bit -> Fault.
 *   - `Mode`: plain state flags — active -> Ok, inactive -> Muted.
 *
 * A missing value (std::nullopt — e.g. the object could not be read)
 * always renders a muted "n/a" chip.
 *
 * This header is ncurses-free; the ncurses box/grid renderer lives in
 * `tether/terminal_ui/ChipPanel.hpp`.  Producers that only need text
 * output may render rows themselves — `appendBitfieldChips()` keeps the
 * semantics consistent across TUIs, logs and test dumps.
 */

#include <cstdint>
#include <format>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <vector>

#include "tether/utils/ColoredBitsetFormatter.hpp" // EtherCAT::Utils::BitLabel

namespace Tether {
namespace UI {

/// Semantic color role for one display cell (see file docs above).
enum class DisplayTone { Ok, Missing, Fault, Warn, Info, Muted };

struct DisplayCell {
    std::string text;
    DisplayTone tone = DisplayTone::Muted;
};
struct DisplayRow {
    std::vector<DisplayCell> cells;
};
struct DisplayPanel {
    std::string             title;
    std::vector<DisplayRow> rows;
};

/// A row whose first cell is a muted label — the common shape of
/// chip-panel rows ("EStop In1 In2 …").
inline DisplayRow labeledRow(std::string_view label) {
    return DisplayRow{{{std::string(label), DisplayTone::Muted}}};
}

/// How `appendBitfieldChips` maps bit activity to cell tones.
enum class ChipSemantics {
    ExpectedSet,   ///< set bits -> Ok, cleared labeled bits -> Missing
    ExpectedClear, ///< value==0 -> Ok "none", set bits -> Fault
    Mode,          ///< active labels -> Ok, inactive -> Muted
};

/// True when any label's mask covers @p bit — used to suppress "bitN"
/// chips for labeled bits while still reporting *unknown* set bits.
inline bool bitIsLabeled(std::span<const EtherCAT::Utils::BitLabel> labels,
                         uint32_t bit) {
    const uint32_t m = 1u << bit;
    for (const auto& l : labels) {
        if (l.mask & m) return true;
    }
    return false;
}

/**
 * @brief Append tone-coded chips for one labeled bitfield to @p row.
 *
 * @param row        target row (chips appended after existing cells).
 * @param value      the bitfield, or std::nullopt when it could not be
 *                   read — renders a muted "n/a" chip.
 * @param labels     the bit labels for the object (BitLabel::isActive
 *                   decides per label; masks may cover several bits).
 * @param semantics  ExpectedSet / ExpectedClear / Mode (see above).
 *
 * Unlabeled set bits are always reported as `bitN` chips — Info tone for
 * ExpectedSet/Mode rows, Fault tone for ExpectedClear rows.
 */
inline void appendBitfieldChips(
    DisplayRow& row, std::optional<uint32_t> value,
    std::span<const EtherCAT::Utils::BitLabel> labels,
    ChipSemantics semantics) {
    if (!value) {
        row.cells.push_back({"n/a", DisplayTone::Muted});
        return;
    }
    const uint32_t v = *value;
    switch (semantics) {
        case ChipSemantics::ExpectedSet: {
            bool any = false;
            for (const auto& l : labels) {
                row.cells.push_back({l.name, l.isActive(v) ? DisplayTone::Ok
                                                         : DisplayTone::Missing});
                any = true;
            }
            for (uint32_t b = 0; b < 32; ++b) {
                if ((v & (1u << b)) && !bitIsLabeled(labels, b)) {
                    row.cells.push_back(
                        {std::format("bit{}", b), DisplayTone::Info});
                }
            }
            if (!any && v == 0) {
                row.cells.push_back({"none", DisplayTone::Missing});
            }
            break;
        }
        case ChipSemantics::ExpectedClear: {
            if (v == 0) {
                row.cells.push_back({"none", DisplayTone::Ok});
                break;
            }
            for (const auto& l : labels) {
                if (l.isActive(v)) {
                    row.cells.push_back({l.name, DisplayTone::Fault});
                }
            }
            for (uint32_t b = 0; b < 32; ++b) {
                if ((v & (1u << b)) && !bitIsLabeled(labels, b)) {
                    row.cells.push_back(
                        {std::format("bit{}", b), DisplayTone::Fault});
                }
            }
            break;
        }
        case ChipSemantics::Mode: {
            for (const auto& l : labels) {
                row.cells.push_back({l.name, l.isActive(v) ? DisplayTone::Ok
                                                         : DisplayTone::Muted});
            }
            break;
        }
    }
}

/// Append one scalar chip: `text` when the value was read, muted "n/a"
/// otherwise.
inline void appendScalarChip(DisplayRow& row,
                             std::optional<std::string> text,
                             DisplayTone tone = DisplayTone::Ok) {
    row.cells.push_back(text ? DisplayCell{std::move(*text), tone}
                             : DisplayCell{"n/a", DisplayTone::Muted});
}

// ---------------------------------------------------------------------------
// Text rendering (stream/log output — the ncurses inline renderer is
// TUI::drawChips in tether/terminal_ui/Chips.hpp)
// ---------------------------------------------------------------------------

/// ANSI SGR parameter for a tone ("0" resets).
inline const char* toneToAnsi(DisplayTone t) {
    switch (t) {
        case DisplayTone::Ok:      return "32";    // green
        case DisplayTone::Missing: return "31";    // red
        case DisplayTone::Fault:   return "1;31";  // bold red
        case DisplayTone::Warn:    return "33";    // yellow
        case DisplayTone::Info:    return "36";    // cyan
        case DisplayTone::Muted:   return "90";    // bright black
    }
    return "0";
}

/**
 * @brief Render a chip list as text: `[a b c]`.
 *
 * The same DisplayCell model that TUI::drawChips draws in ncurses, so a
 * status element looks identical in the TUI, the --stream output and
 * log dumps.  When `ansi` is true each cell is wrapped in the SGR code
 * for its tone (pass `isatty(STDOUT_FILENO)`).
 */
inline std::string formatChips(std::span<const DisplayCell> cells,
                               bool brackets = true, bool ansi = false) {
    std::string out;
    if (brackets) out += '[';
    bool first = true;
    for (const auto& c : cells) {
        if (!first) out += ' ';
        first = false;
        if (ansi) {
            out += "\x1b[";
            out += toneToAnsi(c.tone);
            out += 'm';
            out += c.text;
            out += "\x1b[0m";
        } else {
            out += c.text;
        }
    }
    if (brackets) out += ']';
    return out;
}

} // namespace UI
} // namespace Tether
