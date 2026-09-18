/**
 * @file PanelScreen.hpp
 * @brief Composed TUI screen for live status dashboards: title bar,
 *        an application-rendered body, captured log lines, key-hint footer
 *
 * Layout (adapts to terminal size):
 *
 *   +--------------------------------------------------------------+
 *   | title                                            t= 12.3 s   |
 *   |--------------------------------------------------------------|
 *   | body (renderBody hook draws rows [1, sepY) on stdscr)        |
 *   |--------------------------------------------------------------|
 *   | log lines ...                                                |
 *   | q: quit   <keyHints>                                         |
 *   +--------------------------------------------------------------+
 *
 * Unlike TreeScreen there is no widget in the body — the app draws it
 * directly with mvprintw() into the reported row range each frame.
 * Session owns the ncurses lifecycle, palette and UTF-8 locale.
 */

#pragma once

#include <atomic>
#include <functional>
#include <string>

#include "tether/terminal_ui/LogPane.hpp"
#include "tether/terminal_ui/Session.hpp"

namespace Tether {
namespace TUI {

struct PanelScreenHooks {
    /// Called once per frame (~50 ms) before drawing — update model state.
    std::function<void()> onTick;

    /// Draw the body region each frame.  Receives the first and last
    /// usable stdscr rows ([top, bottom)); draw with mvprintw et al.
    std::function<void(int top, int bottom)> renderBody;

    /// Keys not consumed by the screen arrive here.
    /// Return true when handled.
    std::function<bool(int)> onKey;

    /// Optional modal guard: when set and it returns true, the quit keys
    /// (q/Q/Esc) are passed to onKey instead of ending run().
    std::function<bool()> quitGuard;

    /// Extra text appended to the footer key hints.
    std::string keyHints;
};

class PanelScreen {
public:
    PanelScreen(std::string title, PanelScreenHooks hooks);

    /// Run the draw/dispatch loop until 'q'/Esc, `cancel`, or duration.
    void run(std::atomic<bool>& cancel, double durationSec = 0.0);

    /// Access to the captured log pane (add application lines).
    LogPane& log() { return log_; }

    /// The underlying session (rows/cols, colors).
    Session& session() { return session_; }

private:
    Session         session_;
    LogPane         log_;
    std::string     title_;
    PanelScreenHooks hooks_;
};

} // namespace TUI
} // namespace Tether
