#pragma once

/**
 * @file EntityBrowserPanel.hpp
 * @brief Standard "Entities" view widget — navigable entity table plus a
 *        named-register browser for the selected entity.
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
 *
 * The provider poll is throttled (default 1 s) — register reads only
 * happen on demand (Enter/r/R or auto-read), never implicitly per frame.
 */

#include <chrono>
#include <string>

#include "tether/ui/Panels.hpp"

namespace Tether {
namespace TUI {

class EntityBrowserPanel {
public:
    explicit EntityBrowserPanel(UI::EntityBrowser& browser);

    /// Minimum interval between provider polls (default 1000 ms).
    void setRefreshInterval(std::chrono::milliseconds ms) { refresh_ms_ = ms; }

    /// Render into the screen region [top, bottom).
    void render(int top, int bottom);

    /// Returns true when the key was consumed.
    bool handleKey(int key);

    /// Footer hint string for PanelView::keyHints.
    const char* keyHints() const {
        return "e:focus Enter/r:read /:search f:filter x:clear a:auto";
    }

private:
    enum class Prompt { None, Search, Filter };
    UI::TableModel& focusedTable();
    void commitPrompt();

    UI::EntityBrowser& b_;
    Prompt prompt_ = Prompt::None;
    std::string prompt_buf_;
    std::chrono::milliseconds refresh_ms_{1000};
    std::chrono::steady_clock::time_point last_refresh_{};
    bool first_render_ = true;
};

} // namespace TUI
} // namespace Tether
