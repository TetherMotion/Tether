#pragma once

/**
 * @file ColoredBitsetFormatter.hpp
 * @brief Generic colored formatting of bitset flags with named labels
 *
 * Formats a bitset (e.g. a `uint16_t` safety_flags field) as a sequence of
 * colored "NAME=ON" / "NAME=OFF" labels using ANSI escape codes:
 *   green = active, red = inactive.
 *
 * The formatter is **data-driven**: the caller supplies a list of `BitLabel`
 * entries and the formatter renders them uniformly.  No opinionated
 * pre-selection of which flags are "important" — every label in the list
 * gets the same treatment.
 *
 * ## Single-bit labels (the common case)
 *
 * Use the `BitLabel::bit()` / `BitLabel::bitInv()` factory functions:
 * @code
 *   static BitLabel labels[] = {
 *       BitLabel::bit("STO", 1u << 0),       // one-active: bit=1 → active
 *       BitLabel::bitInv("SS1", 1u << 1),    // zero-active: bit=0 → active
 *   };
 *   ColoredBitsetFormatter fmt(labels);
 *   std::string s = fmt.format(value);       // "STO=ON  SS1=OFF" (colored)
 * @endcode
 *
 * ## Multi-bit labels
 *
 * A label can cover multiple bits.  The `ActiveWhen` enum controls when the
 * label is considered "active":
 *   - `AllSet`   — active when **all** bits in mask are set
 *   - `AllClear` — active when **all** bits in mask are clear
 *   - `AnySet`   — active when **any** bit in mask is set
 *   - `AnyClear` — active when **any** bit in mask is clear
 *
 * Use the `bits()` / `bitsInv()` / `anyBit()` / `anyBitInv()` factories:
 * @code
 *   static BitLabel labels[] = {
 *       BitLabel::anyBit("SafeStop", 0x000B),  // STO|SS1|SS2 → active if any set
 *       BitLabel::bits("SLS_All", 0xF000),     // SLS1-4 → active only if all set
 *   };
 * @endcode
 *
 * ## Visibility control (show only when ON or OFF)
 *
 * By default, `format()` shows every label as either "NAME=ON" or
 * "NAME=OFF".  Sometimes you want a label to appear **only** when it is
 * active (ON) or **only** when it is inactive (OFF).  Use the `ShowWhen`
 * enum and the `bitIfOn()` / `bitIfOff()` factory families:
 * @code
 *   static BitLabel labels[] = {
 *       BitLabel::bitIfOn("FAULT", 0x01),     // shown only when fault is active
 *       BitLabel::bitIfOff("READY", 0x02),    // shown only when not ready
 *   };
 *   ColoredBitsetFormatter fmt(labels);
 *   // value=0x01: "FAULT=ON"        (READY hidden because it's ON/active)
 *   // value=0x02: "READY=OFF"       (FAULT hidden because it's OFF/inactive)
 *   // value=0x00: "READY=OFF"      (FAULT hidden, READY shown as OFF)
 * @endcode
 *
 * `formatActive()` also respects `shouldShow()`: a label with
 * `ShowWhen::WhenInactive` will never appear in `formatActive()` output
 * (since `formatActive` only lists active labels, and `WhenInactive`
 * hides active labels).
 *
 * ## Custom labels (subclassing)
 *
 * `BitLabel` is polymorphic — subclass it and override `isActive()` and/or
 * `shouldShow()` for custom logic that doesn't fit the `ActiveWhen` /
 * `ShowWhen` model:
 * @code
 *   class ThresholdLabel : public BitLabel {
 *   public:
 *       explicit ThresholdLabel(const char* name, uint32_t threshold)
 *           : BitLabel(name, 0, ActiveWhen::AllSet), threshold_(threshold) {}
 *       bool isActive(uint32_t value) const override {
 *           return value >= threshold_;
 *       }
 *   private:
 *       uint32_t threshold_;
 *   };
 *
 *   ThresholdLabel custom("Hot", 80);
 *   const BitLabel* labels[] = {&custom};
 *   ColoredBitsetFormatter fmt(labels);
 * @endcode
 *
 * ## Two output modes
 *
 * - `format()`      — visible labels as "NAME=ON"/"NAME=OFF" (colored),
 *                     joined by a separator.  Labels hidden by
 *                     `shouldShow()` are skipped entirely.
 * - `formatActive()` — only active (and visible) labels, as a
 *                     space-separated list of names (no color).  Compact
 *                     summary of what's set.
 */

#include <cstdint>
#include <span>
#include <string>
#include <string_view>
#include <vector>
#include <format>

namespace EtherCAT {
namespace Utils {

/// When a (possibly multi-bit) label is considered "active".
enum class ActiveWhen {
    AllSet,   ///< (value & mask) == mask — all bits in mask are set
    AllClear, ///< (value & mask) == 0    — all bits in mask are clear
    AnySet,   ///< (value & mask) != 0    — any bit in mask is set
    AnyClear, ///< (value & mask) != mask — any bit in mask is clear
};

/// When a label should appear in `format()` / `formatActive()` output.
enum class ShowWhen {
    Always,       ///< Always show (both ON and OFF).  Default.
    WhenActive,   ///< Show only when the label is active (ON).
    WhenInactive, ///< Show only when the label is inactive (OFF).
};

/// A labeled bit or bit-group within a bitset.
///
/// This is a polymorphic base class.  The default `isActive()` uses the
/// `mask` and `active_when` fields.  The default `shouldShow()` uses the
/// `show_when` field together with `isActive()`.  Subclass and override
/// either method for custom logic.
///
/// `name` must point to a string literal (static storage duration) when
/// used with `ColoredBitsetFormatter`.
class BitLabel {
public:
    const char* name;        ///< Display name (e.g. "STO")
    uint32_t mask;           ///< Bit mask (single or multiple bits)
    ActiveWhen active_when;  ///< Condition for the label to be "active"
    ShowWhen show_when;      ///< Condition for the label to appear in output

    BitLabel() = default;
    BitLabel(const char* name, uint32_t mask, ActiveWhen active_when,
             ShowWhen show_when = ShowWhen::Always)
        : name(name), mask(mask), active_when(active_when),
          show_when(show_when) {}
    virtual ~BitLabel() = default;

    /// Check whether this label is active for the given bitset value.
    /// Override in subclasses for custom behavior.
    virtual bool isActive(uint32_t value) const {
        switch (active_when) {
            case ActiveWhen::AllSet:   return (value & mask) == mask;
            case ActiveWhen::AllClear: return (value & mask) == 0;
            case ActiveWhen::AnySet:   return (value & mask) != 0;
            case ActiveWhen::AnyClear: return (value & mask) != mask;
        }
        return false;
    }

    /// Check whether this label should appear in output for the given
    /// bitset value.  Override in subclasses for custom visibility logic.
    virtual bool shouldShow(uint32_t value) const {
        switch (show_when) {
            case ShowWhen::Always:       return true;
            case ShowWhen::WhenActive:   return isActive(value);
            case ShowWhen::WhenInactive: return !isActive(value);
        }
        return true;
    }

    // --- Single-bit factories (always shown) ---

    /// One-active single bit: active when the bit is set (bit=1 → ON).
    static BitLabel bit(const char* name, uint32_t mask) {
        return {name, mask, ActiveWhen::AllSet, ShowWhen::Always};
    }
    /// Zero-active single bit: active when the bit is clear (bit=0 → ON).
    static BitLabel bitInv(const char* name, uint32_t mask) {
        return {name, mask, ActiveWhen::AllClear, ShowWhen::Always};
    }

    // --- Single-bit factories (show only when ON) ---

    /// One-active single bit, shown only when active (ON).
    static BitLabel bitIfOn(const char* name, uint32_t mask) {
        return {name, mask, ActiveWhen::AllSet, ShowWhen::WhenActive};
    }
    /// Zero-active single bit, shown only when active (ON).
    static BitLabel bitInvIfOn(const char* name, uint32_t mask) {
        return {name, mask, ActiveWhen::AllClear, ShowWhen::WhenActive};
    }

    // --- Single-bit factories (show only when OFF) ---

    /// One-active single bit, shown only when inactive (OFF).
    static BitLabel bitIfOff(const char* name, uint32_t mask) {
        return {name, mask, ActiveWhen::AllSet, ShowWhen::WhenInactive};
    }
    /// Zero-active single bit, shown only when inactive (OFF).
    static BitLabel bitInvIfOff(const char* name, uint32_t mask) {
        return {name, mask, ActiveWhen::AllClear, ShowWhen::WhenInactive};
    }

    // --- Multi-bit factories (always shown) ---

    /// Multi-bit label: active when ALL bits in mask are set.
    static BitLabel bits(const char* name, uint32_t mask) {
        return {name, mask, ActiveWhen::AllSet, ShowWhen::Always};
    }
    /// Multi-bit label: active when ALL bits in mask are clear.
    static BitLabel bitsInv(const char* name, uint32_t mask) {
        return {name, mask, ActiveWhen::AllClear, ShowWhen::Always};
    }
    /// Multi-bit label: active when ANY bit in mask is set.
    static BitLabel anyBit(const char* name, uint32_t mask) {
        return {name, mask, ActiveWhen::AnySet, ShowWhen::Always};
    }
    /// Multi-bit label: active when ANY bit in mask is clear.
    static BitLabel anyBitInv(const char* name, uint32_t mask) {
        return {name, mask, ActiveWhen::AnyClear, ShowWhen::Always};
    }

    // --- Multi-bit factories (show only when ON) ---

    /// Multi-bit label (all-set), shown only when active (ON).
    static BitLabel bitsIfOn(const char* name, uint32_t mask) {
        return {name, mask, ActiveWhen::AllSet, ShowWhen::WhenActive};
    }
    /// Multi-bit label (all-clear), shown only when active (ON).
    static BitLabel bitsInvIfOn(const char* name, uint32_t mask) {
        return {name, mask, ActiveWhen::AllClear, ShowWhen::WhenActive};
    }
    /// Multi-bit label (any-set), shown only when active (ON).
    static BitLabel anyBitIfOn(const char* name, uint32_t mask) {
        return {name, mask, ActiveWhen::AnySet, ShowWhen::WhenActive};
    }
    /// Multi-bit label (any-clear), shown only when active (ON).
    static BitLabel anyBitInvIfOn(const char* name, uint32_t mask) {
        return {name, mask, ActiveWhen::AnyClear, ShowWhen::WhenActive};
    }

    // --- Multi-bit factories (show only when OFF) ---

    /// Multi-bit label (all-set), shown only when inactive (OFF).
    static BitLabel bitsIfOff(const char* name, uint32_t mask) {
        return {name, mask, ActiveWhen::AllSet, ShowWhen::WhenInactive};
    }
    /// Multi-bit label (all-clear), shown only when inactive (OFF).
    static BitLabel bitsInvIfOff(const char* name, uint32_t mask) {
        return {name, mask, ActiveWhen::AllClear, ShowWhen::WhenInactive};
    }
    /// Multi-bit label (any-set), shown only when inactive (OFF).
    static BitLabel anyBitIfOff(const char* name, uint32_t mask) {
        return {name, mask, ActiveWhen::AnySet, ShowWhen::WhenInactive};
    }
    /// Multi-bit label (any-clear), shown only when inactive (OFF).
    static BitLabel anyBitInvIfOff(const char* name, uint32_t mask) {
        return {name, mask, ActiveWhen::AnyClear, ShowWhen::WhenInactive};
    }
};

/// Format a bitset as a sequence of colored "NAME=ON"/"NAME=OFF" labels.
///
/// The formatter stores an array of `const BitLabel*` pointers.  The
/// pointed-to labels must outlive the formatter (typically `static` or
/// `inline` arrays, or heap-allocated objects).
///
/// For the simple case (no subclassing), pass a C array of `BitLabel`
/// values — the formatter takes their addresses internally.  For the
/// subclassing case, pass an array of `const BitLabel*` pointers or an
/// `initializer_list<const BitLabel*>`.
class ColoredBitsetFormatter {
public:
    /// Construct from a C array of `BitLabel` values (simple case).
    /// The array must outlive the formatter.
    ColoredBitsetFormatter(std::span<const BitLabel> labels) {
        ptrs_.reserve(labels.size());
        for (const auto& label : labels) {
            ptrs_.push_back(&label);
        }
    }

    /// Construct from a C array of `const BitLabel*` pointers (subclassing
    /// case).  The array and the pointed-to labels must outlive the formatter.
    ColoredBitsetFormatter(std::span<const BitLabel* const> labels) {
        ptrs_.reserve(labels.size());
        for (const auto* ptr : labels) {
            ptrs_.push_back(ptr);
        }
    }

    /// Default-construct an empty formatter (renders empty strings).
    ColoredBitsetFormatter() = default;

    // -- Query --

    /// Number of labels in this formatter.
    size_t size() const { return ptrs_.size(); }

    // -- Formatting --

    /// Format visible labels as "NAME=ON" or "NAME=OFF" with ANSI color,
    /// joined by `sep` (default: two spaces).  Labels hidden by
    /// `shouldShow()` are skipped entirely.
    std::string format(uint32_t value, std::string_view sep = "  ") const {
        std::string result;
        bool first = true;
        for (const auto* label : ptrs_) {
            if (!label->shouldShow(value)) continue;
            if (!first) result.append(sep);
            result += formatBit(label->isActive(value), label->name);
            first = false;
        }
        return result;
    }

    /// Format only the ACTIVE (and visible) labels as a space-separated
    /// list of names (no color).  Returns `empty_text` (default "(none)")
    /// if no labels are both active and visible.
    std::string formatActive(uint32_t value,
                             std::string_view empty_text = "(none)") const {
        std::string result;
        bool first = true;
        for (const auto* label : ptrs_) {
            if (!label->shouldShow(value)) continue;
            if (!label->isActive(value)) continue;
            if (!first) result += ' ';
            result += label->name;
            first = false;
        }
        return first ? std::string(empty_text) : result;
    }

    /// Format a single bit as colored "NAME=ON" or "NAME=OFF".
    /// Green (active) or red (inactive), bold, with ANSI reset.
    static std::string formatBit(bool active, std::string_view name) {
        if (active) {
            return std::format("\033[32m{}=\033[1mON\033[0m", name);
        }
        return std::format("\033[31m{}=\033[1mOFF\033[0m", name);
    }

private:
    std::vector<const BitLabel*> ptrs_;
};

} // namespace Utils
} // namespace EtherCAT
