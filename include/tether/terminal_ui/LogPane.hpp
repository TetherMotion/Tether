/**
 * @file LogPane.hpp
 * @brief Ring buffer of captured log lines for display inside a TUI
 *
 * While ncurses owns the screen, console logging would corrupt the layout.
 * attach() redirects the process logger into this pane's ring buffer;
 * release() restores the console handler.  render() draws the buffered
 * lines into a TermWindow.
 */

#pragma once

#include <cstddef>
#include <deque>
#include <mutex>
#include <string>

#include "tether/terminal_ui/Session.hpp"

namespace Tether {
namespace TUI {

class LogPane {
public:
    explicit LogPane(size_t maxLines = 4);
    ~LogPane();

    LogPane(const LogPane&) = delete;
    LogPane& operator=(const LogPane&) = delete;

    /// Redirect the global logger into this pane.
    void attach();
    /// Restore the previous (console) log handler.  Called by ~LogPane().
    void release();

    /// Append a line directly (e.g. application status text).
    void addLine(std::string line);

    /// Draw buffered lines into `win` (cleared first), newest at the bottom.
    void render(TermWindow* win, short colorPair = PalError);

    size_t maxLines() const { return maxLines_; }

private:
    std::deque<std::string> lines_;
    std::mutex              mutex_;
    size_t                  maxLines_;
    bool                    attached_ = false;
};

} // namespace TUI
} // namespace Tether
