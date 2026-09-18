#pragma once

/**
 * @file EntityBrowserPanel.hpp
 * @brief Standard "Entities" view widget — navigable entity table plus a
 *        named-register browser for the selected entity, extensible via
 *        actions and overlay panels.
 *
 * Composes two TablePanel tables driven by a UI::EntityBrowser model:
 * entities on top, registers of the selected entity below.  All model
 * logic lives in tether/ui/ (ncurses-free) so a web UI can reuse it.
 *
 * Keys (handled by handleKey):
 *   Up/Down, PgUp/PgDn, Home/End   navigate the focused table
 *   Enter / r    read registers of the selected entity (focus moves down)
 *   e            toggle focus between the two tables
 *   Esc/Left     from registers back to the entity table
 *   /            quicksearch prompt on the focused table
 *   f            complex filter prompt on the focused table
 *   x            clear filter+search on the focused table
 *   a            toggle auto-read of registers on selection change
 *   R            re-read registers without changing focus
 *   <actions>    app-registered PanelAction hotkeys (see addAction)
 *
 * Custom actions: addAction({key, label, invoke}) registers a hotkey.
 * invoke() typically calls pushOverlay() to open a modal overlay panel
 * (UI::TextOverlay for text, UI::TableOverlay for a full filtered
 * table, or a custom UI::Overlay subclass).  Esc closes the top
 * overlay; while any overlay is open the underlying tables see no keys.
 *
 * The provider poll is throttled (default 1 s) — register reads only
 * happen on demand (Enter/r/R or auto-read), never implicitly per frame.
 */

#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "tether/terminal_ui/OverlayHost.hpp"
#include "tether/ui/Overlay.hpp"
#include "tether/ui/Panels.hpp"

namespace Tether {
namespace TUI {

class EntityBrowserPanel {
public:
    explicit EntityBrowserPanel(UI::EntityBrowser& browser);

    /// Minimum interval between provider polls (default 1000 ms).
    void setRefreshInterval(std::chrono::milliseconds ms) { refresh_ms_ = ms; }

    /// Register a hotkey action.  The label appears in the footer hints;
    /// invoke() may push an overlay or do anything else.
    void addAction(UI::PanelAction action) { actions_.push_back(std::move(action)); }

    /// Push a modal overlay (called from action callbacks).
    void pushOverlay(std::unique_ptr<UI::Overlay> overlay) {
        overlays_.push(std::move(overlay));
    }
    UI::OverlayStack& overlays() { return overlays_; }
    bool overlayOpen() const { return !overlays_.empty(); }

    /// Render into the screen region [top, bottom).
    void render(int top, int bottom);

    /// Returns true when the key was consumed.
    bool handleKey(int key);

    /// Footer hint string for PanelView::keyHints.
    std::string keyHints() const;

private:
    enum class Prompt { None, Search, Filter };
    UI::TableModel& focusedTable();
    void commitPrompt();

    UI::EntityBrowser& b_;
    UI::OverlayStack overlays_;
    OverlayHost overlay_host_{overlays_};
    std::vector<UI::PanelAction> actions_;
    Prompt prompt_ = Prompt::None;
    std::string prompt_buf_;
    std::chrono::milliseconds refresh_ms_{1000};
    std::chrono::steady_clock::time_point last_refresh_{};
    bool first_render_ = true;
};

/// Helper for the common "show full entity details" action — builds a
/// TextOverlay dumping every field of the currently selected entity.
std::unique_ptr<UI::Overlay> makeEntityInfoOverlay(UI::EntityBrowser& b);

} // namespace TUI
} // namespace Tether
