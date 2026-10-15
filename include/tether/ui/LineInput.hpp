#pragma once

/**
 * @file LineInput.hpp
 * @brief Toolkit-agnostic single-line text-input model.
 *
 * One shared editing state machine for every "type a line, Enter to
 * commit, Esc to cancel" prompt in the codebase — previously
 * hand-rolled three times (TreeScreen's LinePrompt buffer,
 * EntityBrowserPanel's prompt_/prompt_buf_, PanelScreen's
 * filter_editing/filter_buf_).
 *
 * The model owns buffer + cursor and reports outcome through flags:
 * Enter sets `committed()`, Esc sets `cancelled()`; both make
 * `done()` true so the frontend can close the prompt.  `on_change`
 * fires on every edit for live type-ahead (e.g. quicksearch updating
 * a TableModel while typing).  Cursor movement (Left/Right/Home/End),
 * Delete and insert-at-cursor are supported — the earlier ad-hoc
 * editors only appended.
 *
 * Rendering stays with the frontend (an inline footer line, a boxed
 * prompt, …); `cursor()` gives the insertion point for the caret.
 */

#include <functional>
#include <string>
#include <utility>

namespace Tether {
namespace UI {

class LineInput {
public:
    /// Live type-ahead hook — fires after every buffer mutation.
    std::function<void(const std::string&)> on_change;
    /// Enter pressed — receives the final buffer.
    std::function<void(const std::string&)> on_commit;
    /// Esc pressed.
    std::function<void()> on_cancel;

    explicit LineInput(std::string initial = {})
        : buf_(std::move(initial)), cur_(buf_.size()) {}

    const std::string& buffer() const { return buf_; }
    size_t cursor() const { return cur_; }

    /// Replace the buffer programmatically (cursor moves to the end).
    void setBuffer(std::string s) {
        buf_ = std::move(s);
        cur_ = buf_.size();
        if (on_change) on_change(buf_);
    }

    /// True once Enter or Esc ended the editing session.
    bool done() const { return committed_ || cancelled_; }
    bool committed() const { return committed_; }
    bool cancelled() const { return cancelled_; }

    /// Re-arm after done() (e.g. reopening the prompt).
    void reset(std::string initial = {}) {
        buf_ = std::move(initial);
        cur_ = buf_.size();
        committed_ = cancelled_ = false;
    }

    /// Feed a key.  Returns true when the key belonged to the editor
    /// (editing keys, Enter, Esc — everything while active).
    bool handleKey(int key) {
        if (done()) return false;
        switch (key) {
            case 27:   // Esc
                cancelled_ = true;
                if (on_cancel) on_cancel();
                return true;
            case '\n': case '\r': case KEY_ENTER_:
                committed_ = true;
                if (on_commit) on_commit(buf_);
                return true;
            case KEY_LEFT_:
                if (cur_ > 0) --cur_;
                return true;
            case KEY_RIGHT_:
                if (cur_ < buf_.size()) ++cur_;
                return true;
            case KEY_HOME_:
                cur_ = 0;
                return true;
            case KEY_END_:
                cur_ = buf_.size();
                return true;
            case KEY_BACKSPACE_: case 127: case '\b':
                if (cur_ > 0) {
                    buf_.erase(cur_ - 1, 1);
                    --cur_;
                    if (on_change) on_change(buf_);
                }
                return true;
            case KEY_DC_:
                if (cur_ < buf_.size()) {
                    buf_.erase(cur_, 1);
                    if (on_change) on_change(buf_);
                }
                return true;
            default: break;
        }
        if (key >= 32 && key <= 126) {
            buf_.insert(cur_, 1, static_cast<char>(key));
            ++cur_;
            if (on_change) on_change(buf_);
            return true;
        }
        return true;   // modal editor — swallow everything else
    }

private:
    // ncurses key codes, duplicated here so this header stays
    // toolkit-free (must not include <curses.h>).
    static constexpr int KEY_LEFT_      = 260;
    static constexpr int KEY_RIGHT_     = 261;
    static constexpr int KEY_HOME_      = 262;
    static constexpr int KEY_END_       = 360;
    static constexpr int KEY_BACKSPACE_ = 263;
    static constexpr int KEY_DC_        = 330;
    static constexpr int KEY_ENTER_     = 343;

    std::string buf_;
    size_t cur_ = 0;
    bool committed_ = false;
    bool cancelled_ = false;
};

} // namespace UI
} // namespace Tether
