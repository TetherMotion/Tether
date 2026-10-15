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
#include <chrono>
#include <functional>
#include <string>
#include <vector>

#include "tether/terminal_ui/LogPane.hpp"
#include "tether/terminal_ui/Session.hpp"
#include "tether/ui/Overlay.hpp"

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

    /// Dynamic hint provider — evaluated every frame, takes precedence
    /// over the static `keyHints` (e.g. widgets whose hints change with
    /// state, like an open overlay or prompt).
    std::function<std::string()> keyHintsFn;

    /// Pre-dispatch claim: evaluated before the screen's own key
    /// handling (tab digits, ? overlay, log scroll).  When it returns
    /// true for a key, the key goes to `onKey` first; an unhandled
    /// key still falls through to the screen.  Use sparingly — e.g.
    /// a view with an active quicksearch claims digits so "FSoE3"
    /// isn't split by the tab shortcuts.  Returning true for '1'
    /// while idle would break navigation, so claims should be
    /// state-conditional.
    std::function<bool(int)> wantsKey;
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

    /// Push a UI::Overlay on top of the screen (e.g. a TableOverlay
    /// with details for the selected row).  While the stack is
    /// non-empty the overlay is modal: it renders over the body and
    /// consumes all keys (Esc pops).  '?' opens a built-in keys
    /// overlay for the active view.
    void pushOverlay(std::unique_ptr<UI::Overlay> overlay);
    /// Whether a modal overlay is currently shown.
    bool overlayOpen() const { return !overlays_.empty(); }

    /// Show a transient notice in the footer for `seconds` (action
    /// feedback like "filter cleared" / "applied").  Empty message
    /// clears it.  The notice replaces the key hints while visible.
    void flash(std::string msg, double seconds = 3.0);

private:
    Session         session_;
    LogPane         log_;
    std::string     title_;
    PanelScreenHooks hooks_;
    size_t          active_ = 0;
    /// Rows available to the log view in the last layout (for paging).
    int             bodyRows_ = 8;
    /// Transient footer notice (flash()) — message + expiry.
    std::string     flash_msg_;
    std::chrono::steady_clock::time_point flash_until_{};
    /// Modal overlay stack — '?' pushes a keys overlay; apps may push
    /// their own via pushOverlay().
    UI::OverlayStack overlays_;
};

} // namespace TUI
} // namespace Tether
