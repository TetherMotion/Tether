#pragma once

/**
 * @file OverlayHost.hpp
 * @brief ncurses renderer + key router for a UI::OverlayStack.
 *
 * Draws the top overlay as a centered bordered box over the panel:
 * a TableOverlay gets a full TablePanel (with its own status line);
 * other overlays get their text lines rendered verbatim.  Esc closes
 * the top overlay; everything else goes to Overlay::handleKey().
 *
 * Nested overlays work — the stack renders only the top one, but keys
 * always reach it, so an action inside an overlay can push another.
 */

#include "tether/ui/Overlay.hpp"

namespace Tether {
namespace TUI {

class OverlayHost {
public:
    explicit OverlayHost(UI::OverlayStack& stack) : stack_(stack) {}

    /// Render the top overlay centered within [top, bottom).  Call after
    /// the underlying panel has been drawn.  No-op when the stack is
    /// empty.  Returns true when an overlay was drawn.
    bool render(int top, int bottom);

    /// Route a key to the top overlay.  Esc pops the stack.  Returns
    /// true when the stack is non-empty (key consumed either way — the
    /// underlying panel must not see keys while an overlay is up).
    bool handleKey(int key);

    /// Footer hints of the top overlay (or "" when the stack is empty).
    std::string keyHints() const;

private:
    UI::OverlayStack& stack_;
};

} // namespace TUI
} // namespace Tether
