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
#include <vector>

#include "tether/terminal_ui/LogPane.hpp"
#include "tether/terminal_ui/Session.hpp"

namespace Tether {
namespace TUI {

/// One named view (tab) inside a PanelScreen.  Views are switched with
/// the digit keys 1-9, F1-F12, Tab/S-Tab, or the hooks' onViewChange.
struct PanelView {
    std::string name;   ///< Short tab label shown in the title bar.

    /// Draw the body region each frame while this view is active.
    /// Receives the usable stdscr rows ([top, bottom)).
    std::function<void(int top, int bottom)> renderBody;

    /// Keys not consumed by the screen arrive here while the view is
    /// active.  Return true when handled.
    std::function<bool(int)> onKey;

    /// Extra text appended to the footer key hints while active.
    std::string keyHints;
};

struct PanelScreenHooks {
    /// Called once per frame (~50 ms) before drawing — update model state.
    std::function<void()> onTick;

    /// Draw the body region each frame.  Receives the first and last
    /// usable stdscr rows ([top, bottom)); draw with mvprintw et al.
    /// Used as the implicit first view ("Main") when `views` is empty.
    std::function<void(int top, int bottom)> renderBody;

    /// Named views selectable via hotkeys (1-9, F1-F12, Tab/S-Tab).
    /// When non-empty, `renderBody`/`onKey`/`keyHints` above act as the
    /// fallback for the FIRST view only if that view leaves them unset.
    std::vector<PanelView> views;

    /// Append a built-in "Log" view that renders the captured log
    /// scrollback full-screen (arrows/PgUp/PgDn/End scroll it).
    bool showLogView = false;

    /// Called after the active view changes (index into the effective
    /// view list, log view last).
    std::function<void(size_t)> onViewChange;

    /// Keys not consumed by the screen or the active view arrive here.
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

    /// Switch the active view by index (effective list order:
    /// app views, then the log view when showLogView is set).
    void selectView(size_t index);

    /// Access to the captured log pane (add application lines).
    LogPane& log() { return log_; }

    /// The underlying session (rows/cols, colors).
    Session& session() { return session_; }

    /// Index of the currently active view.
    size_t activeView() const { return active_; }
    /// Total number of effective views (app views + optional log view).
    size_t viewCount() const;

private:
    Session         session_;
    LogPane         log_;
    std::string     title_;
    PanelScreenHooks hooks_;
    size_t          active_ = 0;
    /// Rows available to the log view in the last layout (for paging).
    int             bodyRows_ = 8;
};

} // namespace TUI
} // namespace Tether
