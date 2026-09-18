/**
 * @file LogPane.hpp
 * @brief Scrollable ring buffer of captured log lines for display inside a TUI
 *
 * While ncurses owns the screen, console logging would corrupt the layout.
 * attach() redirects the process logger into this pane's ring buffer;
 * release() restores the console handler.  render() draws the buffered
 * lines into a TermWindow.
 *
 * The pane keeps a deep scrollback (`capacity`, default 500 lines)
 * independent of the on-screen view height (`viewHeight`, default 4).
 * Lines are colored by severity (Error=red, Warn=yellow, Debug/Verbose=
 * gray, Info=plain).  The view follows the newest line unless the user
 * scrolled up (scrollLines()/scrollToEnd()).
 */

#pragma once

#include <cstddef>
#include <deque>
#include <mutex>
#include <string>

#include "logging/Logger.hpp"
#include "tether/terminal_ui/Session.hpp"

namespace Tether {
namespace TUI {

class LogPane {
public:
    /// @param viewHeight  On-screen height used by the screens for layout
    ///                    (maxLines() reports this for compatibility).
    /// @param capacity    Scrollback depth in lines.
    explicit LogPane(size_t viewHeight = 4, size_t capacity = 500);
    ~LogPane();

    LogPane(const LogPane&) = delete;
    LogPane& operator=(const LogPane&) = delete;

    /// Redirect the global logger into this pane.
    void attach();
    /// Restore the previous (console) log handler.  Called by ~LogPane().
    void release();

    /// Append a line directly (e.g. application status text, Info level).
    void addLine(std::string line);
    /// Append a line with an explicit severity for coloring.
    void addLine(std::string line, Platform::LogLevel level);

    /// Scroll the view.  Positive delta moves toward newer lines,
    /// negative toward older.  Any nonzero offset disables following
    /// until scrollToEnd() (or scrolling back to the bottom).
    void scrollLines(int delta);
    /// Jump to the newest line and resume following.
    void scrollToEnd();
    /// True while the view tracks the newest line.
    bool following() const { return scrollOffset_ == 0; }
    /// Lines above the newest currently hidden by scrolling.
    size_t scrollOffset() const { return scrollOffset_; }

    /// Draw buffered lines into `win` (cleared first), newest at the
    /// bottom (or the scrolled slice).  `colorPair` colors Info-level
    /// lines; Warn/Error/Debug/Verbose use their own severity colors.
    void render(TermWindow* win, short colorPair = PalNone);

    /// On-screen view height (kept under the old name — the screens use
    /// it to size the log window).
    size_t maxLines() const { return viewHeight_; }
    size_t capacity() const { return capacity_; }
    size_t size() const;

private:
    struct Entry {
        std::string        text;
        Platform::LogLevel level;
    };

    std::deque<Entry> lines_;
    mutable std::mutex mutex_;
    size_t            viewHeight_;
    size_t            capacity_;
    /// Lines hidden below the view bottom (0 = following newest).
    /// Only mutated by the UI thread; read without a lock is fine.
    size_t            scrollOffset_ = 0;
    bool              attached_ = false;
};

} // namespace TUI
} // namespace Tether
