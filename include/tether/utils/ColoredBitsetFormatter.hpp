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
 * ## Custom labels (subclassing)
 *
 * `BitLabel` is polymorphic — subclass it and override `isActive()` for
 * custom activation logic that doesn't fit the `ActiveWhen` model:
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
 * - `format()`      — all labels as "NAME=ON"/"NAME=OFF" (colored), joined
 *                     by a separator.  Shows every flag's state.
 * - `formatActive()` — only active labels, as a space-separated list of
 *                     names (no color).  Compact summary of what's set.
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

/// A labeled bit or bit-group within a bitset.
///
/// This is a polymorphic base class.  The default `isActive()` uses the
/// `mask` and `active_when` fields.  Subclass and override `isActive()`
/// for custom activation logic.
///
/// `name` must point to a string literal (static storage duration) when
/// used with `ColoredBitsetFormatter`.
class BitLabel {
public:
    const char* name;        ///< Display name (e.g. "STO")
    uint32_t mask;           ///< Bit mask (single or multiple bits)
    ActiveWhen active_when;  ///< Condition for the label to be "active"

    BitLabel() = default;
    BitLabel(const char* name, uint32_t mask, ActiveWhen active_when)
        : name(name), mask(mask), active_when(active_when) {}
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

    // --- Single-bit factories ---

    /// One-active single bit: active when the bit is set (bit=1 → ON).
    static BitLabel bit(const char* name, uint32_t mask) {
        return {name, mask, ActiveWhen::AllSet};
    }
    /// Zero-active single bit: active when the bit is clear (bit=0 → ON).
    static BitLabel bitInv(const char* name, uint32_t mask) {
        return {name, mask, ActiveWhen::AllClear};
    }

    // --- Multi-bit factories ---

    /// Multi-bit label: active when ALL bits in mask are set.
    static BitLabel bits(const char* name, uint32_t mask) {
        return {name, mask, ActiveWhen::AllSet};
    }
    /// Multi-bit label: active when ALL bits in mask are clear.
    static BitLabel bitsInv(const char* name, uint32_t mask) {
        return {name, mask, ActiveWhen::AllClear};
    }
    /// Multi-bit label: active when ANY bit in mask is set.
    static BitLabel anyBit(const char* name, uint32_t mask) {
        return {name, mask, ActiveWhen::AnySet};
    }
    /// Multi-bit label: active when ANY bit in mask is clear.
    static BitLabel anyBitInv(const char* name, uint32_t mask) {
        return {name, mask, ActiveWhen::AnyClear};
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

    /// Format ALL labels as "NAME=ON" or "NAME=OFF" with ANSI color,
    /// joined by `sep` (default: two spaces).
    std::string format(uint32_t value, std::string_view sep = "  ") const {
        std::string result;
        for (size_t i = 0; i < ptrs_.size(); ++i) {
            if (i > 0) result.append(sep);
            result += formatBit(ptrs_[i]->isActive(value), ptrs_[i]->name);
        }
        return result;
    }

    /// Format only the ACTIVE labels as a space-separated list of names
    /// (no color).  Returns `empty_text` (default "(none)") if no labels
    /// are active.
    std::string formatActive(uint32_t value,
                             std::string_view empty_text = "(none)") const {
        std::string result;
        bool first = true;
        for (const auto* label : ptrs_) {
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
