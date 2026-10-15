#pragma once

/**
 * @file Dialog.hpp
 * @brief Toolkit-agnostic modal dialog model.
 *
 * A `UI::Dialog` is an `Overlay` with a message and a row of buttons.
 * The frontend (OverlayHost today, a modal route in a web UI later)
 * renders the buttons via `buttons()`/`focusedButton()` and pops the
 * dialog when `wantsClose()` becomes true.
 *
 * Selection moves with Left/Right (or h/l/Tab); Enter fires the
 * focused button, a button's `hotkey` fires it directly, and Esc fires
 * the designated cancel button — all through `onResult(id)`.
 *
 * Ready-made factories:
 *   makeMessageBox(title, text, onClose)        — [ OK ]
 *   makeConfirmDialog(title, text, onYes, onNo) — [ Yes ] [ No ]
 */

#include <functional>
#include <memory>
#include <span>
#include <string>
#include <utility>
#include <vector>

#include "tether/ui/Overlay.hpp"

namespace Tether {
namespace UI {

/// Common result ids — applications may use their own.
inline constexpr int kDialogCancel = -1;
inline constexpr int kDialogOk = 0;
inline constexpr int kDialogYes = 1;
inline constexpr int kDialogNo = 2;

class Dialog : public Overlay {
public:
    /// @param cancel_index button fired by Esc (default: last button).
    ///        Use -1 for "Esc closes without a result".
    Dialog(std::string title, std::vector<std::string> lines,
           std::vector<DialogButton> buttons,
           std::function<void(int id)> onResult,
           int cancel_index = -1)
        : title_(std::move(title)),
          lines_(std::move(lines)),
          buttons_(std::move(buttons)),
          on_result_(std::move(onResult)),
          cancel_index_(cancel_index < 0 && !buttons_.empty()
                            ? static_cast<int>(buttons_.size()) - 1
                            : cancel_index) {}

    std::string title() const override { return title_; }
    std::vector<std::string> lines() override { return lines_; }

    std::span<const DialogButton> buttons() const override {
        return buttons_;
    }
    size_t focusedButton() const override { return sel_; }
    bool wantsClose() const override { return dismissed_; }

    std::string keyHints() const override {
        return "Left/Right: select  Enter/hotkey: confirm  Esc: cancel";
    }

    bool handleKey(int key) override {
        switch (key) {
            case 27:   // Esc — fire the cancel button, then close
                if (cancel_index_ >= 0 &&
                    cancel_index_ < static_cast<int>(buttons_.size()))
                    fire(static_cast<size_t>(cancel_index_));
                else
                    dismissed_ = true;
                return true;
            case '\t': case KEY_RIGHT_: case 'l':
                sel_ = buttons_.empty() ? 0 : (sel_ + 1) % buttons_.size();
                return true;
            case KEY_LEFT_: case 'h':
                sel_ = buttons_.empty()
                           ? 0
                           : (sel_ + buttons_.size() - 1) % buttons_.size();
                return true;
            case '\n': case '\r': case KEY_ENTER_:
                if (!buttons_.empty()) fire(sel_);
                return true;
            default: break;
        }
        for (size_t i = 0; i < buttons_.size(); ++i) {
            const int hk = buttons_[i].hotkey;
            if (!hk) continue;
            const bool alpha_match =
                (hk >= 'A' && hk <= 'Z' && key == hk + ('a' - 'A')) ||
                (hk >= 'a' && hk <= 'z' && key == hk - ('a' - 'A'));
            if (key == hk || alpha_match) {
                fire(i);
                return true;
            }
        }
        return true;   // dialogs are modal — swallow everything else
    }

private:
    // KEY_LEFT/KEY_RIGHT live in <curses.h> which this header must not
    // include — use the numeric ncurses values (260/261).
    static constexpr int KEY_LEFT_  = 260;
    static constexpr int KEY_RIGHT_ = 261;
    static constexpr int KEY_ENTER_ = 343;   // ncurses KEY_ENTER (keypad)

    void fire(size_t i) {
        if (dismissed_) return;
        dismissed_ = true;
        if (on_result_) on_result_(buttons_[i].id);
    }

    std::string title_;
    std::vector<std::string> lines_;
    std::vector<DialogButton> buttons_;
    std::function<void(int)> on_result_;
    int cancel_index_;
    size_t sel_ = 0;
    bool dismissed_ = false;
};

/// Single-button message box — [ OK ].  `text` is word-wrapped by the
/// caller if needed (pass pre-split lines via `TUI::wrapText` or a plain
/// string with '\n' separators, which are split here).
inline std::unique_ptr<Dialog>
makeMessageBox(std::string title, std::string text,
               std::function<void()> onClose = {}) {
    std::vector<std::string> lines;
    std::string line;
    for (char c : text) {
        if (c == '\n') { lines.push_back(line); line.clear(); }
        else line += c;
    }
    if (!line.empty() || lines.empty()) lines.push_back(line);
    return std::make_unique<Dialog>(
        std::move(title), std::move(lines),
        std::vector<DialogButton>{
            {"OK", kDialogOk, 'o'}},
        [cb = std::move(onClose)](int) { if (cb) cb(); });
}

/// Two-button confirm — [ Yes ] [ No ], Esc = No.
/// `onNo` is optional; both results close the dialog.
inline std::unique_ptr<Dialog>
makeConfirmDialog(std::string title, std::string text,
                  std::function<void()> onYes,
                  std::function<void()> onNo = {}) {
    std::vector<std::string> lines;
    std::string line;
    for (char c : text) {
        if (c == '\n') { lines.push_back(line); line.clear(); }
        else line += c;
    }
    if (!line.empty() || lines.empty()) lines.push_back(line);
    return std::make_unique<Dialog>(
        std::move(title), std::move(lines),
        std::vector<DialogButton>{
            {"Yes", kDialogYes, 'y'},
            {"No", kDialogNo, 'n'}},
        [yes = std::move(onYes), no = std::move(onNo)](int id) {
            if (id == kDialogYes) { if (yes) yes(); }
            else if (no) no();
        },
        /*cancel_index=*/1);   // Esc = No
}

} // namespace UI
} // namespace Tether
