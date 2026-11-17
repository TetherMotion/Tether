#pragma once

/**
 * @file SelectMenu.hpp
 * @brief Toolkit-agnostic single-select question menu model.
 *
 * One shared state machine for "pick one of N options" prompts —
 * the same shape as a raw-mode arrow-key menu (› cursor, ↑/↓ + j/k,
 * digit jump-select, Enter to commit, Esc to skip) with an optional
 * trailing "custom answer…" row that switches into a LineInput for
 * free text.
 *
 * The model owns selection + outcome and renders nothing; a frontend
 * draws `itemCount()` rows (via `rowLabel()`/`cursor()`/
 * `isCustomRow()`) plus its own hint line.  While `editingCustom()`
 * is true the embedded LineInput owns the bottom row and all keys.
 *
 * Result contract (mirrors the wire protocol of numbered answers):
 * `selected()` is the 0-based option index, or -1 when a custom text
 * answer was committed (read it via `customText()`); `cancelled()`
 * means Esc/Ctrl-C — treat as "no answer / take the default".
 */

#include <string>
#include <utility>
#include <vector>

#include "tether/ui/LineInput.hpp"

namespace Tether {
namespace UI {

class SelectMenu {
public:
    /**
     * @param options      The choices, in display order.
     * @param customLabel  Label of the trailing free-text row; empty
     *                     disables the custom entry entirely.
     */
    explicit SelectMenu(std::vector<std::string> options,
                        std::string customLabel = "custom answer…")
        : options_(std::move(options)), custom_label_(std::move(customLabel)) {}

    int itemCount() const {
        return static_cast<int>(options_.size()) + (hasCustom() ? 1 : 0);
    }
    int cursor() const { return cursor_; }
    const std::vector<std::string>& options() const { return options_; }
    bool hasCustom() const { return !custom_label_.empty(); }
    bool isCustomRow(int i) const { return hasCustom() && i == itemCount() - 1; }

    /// Display text of row `i` — "N. label" for options, the custom
    /// label for the last row.
    std::string rowLabel(int i) const {
        if (isCustomRow(i)) return custom_label_;
        return std::to_string(i + 1) + ". " + options_[static_cast<size_t>(i)];
    }

    /// True while the custom-entry LineInput owns the prompt row.
    bool editingCustom() const { return editing_; }
    LineInput& customInput() { return custom_; }
    const LineInput& customInput() const { return custom_; }

    bool done() const { return committed_ || cancelled_; }
    bool cancelled() const { return cancelled_; }
    /// 0-based option index, or -1 for a custom text answer.
    int selected() const { return selected_; }
    const std::string& customText() const { return custom_.buffer(); }

    /// The committed answer: option text, or the custom string.
    std::string answer() const {
        if (selected_ >= 0 && selected_ < static_cast<int>(options_.size()))
            return options_[static_cast<size_t>(selected_)];
        return custom_.buffer();
    }

    /**
     * Feed a key.  Returns true when consumed (the menu is modal while
     * active — everything is swallowed except what falls through after
     * done()).
     */
    bool handleKey(int key) {
        if (done()) return false;
        if (editing_) {
            custom_.handleKey(key);
            if (custom_.committed()) {
                selected_ = -1;
                committed_ = true;
            } else if (custom_.cancelled()) {
                editing_ = false;
                custom_.reset();
            }
            return true;
        }
        switch (key) {
            case KEY_UP_: case 'k':
                if (cursor_ > 0) --cursor_;
                return true;
            case KEY_DOWN_: case 'j':
                if (cursor_ + 1 < itemCount()) ++cursor_;
                return true;
            case 27:   // Esc
            case 3:    // Ctrl-C
                cancelled_ = true;
                return true;
            case '\n': case '\r': case KEY_ENTER_:
                if (isCustomRow(cursor_)) {
                    editing_ = true;
                    custom_.reset();
                    custom_.on_commit = [this](const std::string&) {};
                } else {
                    selected_ = cursor_;
                    committed_ = true;
                }
                return true;
            default: break;
        }
        if (key >= '1' && key <= '9') {
            int n = key - '0';
            if (n <= static_cast<int>(options_.size())) {
                selected_ = n - 1;
                committed_ = true;
            }
            return true;
        }
        return true;
    }

private:
    // ncurses key codes, duplicated here so this header stays
    // toolkit-free (must not include <curses.h>).
    static constexpr int KEY_DOWN_  = 258;
    static constexpr int KEY_UP_    = 259;
    static constexpr int KEY_ENTER_ = 343;

    std::vector<std::string> options_;
    std::string custom_label_;
    int cursor_ = 0;
    int selected_ = -1;
    bool committed_ = false;
    bool cancelled_ = false;
    bool editing_ = false;
    LineInput custom_;
};

} // namespace UI
} // namespace Tether
