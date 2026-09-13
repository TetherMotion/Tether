/**
 * @file TreeScreen.hpp
 * @brief Composed TUI screen: title, navigable tree (left), detail pane
 *        (right), captured log lines, key-hint footer
 *
 * Layout (adapts to terminal size, recreates panes on KEY_RESIZE):
 *
 *   +--------------------------------------------------------------+
 *   | title                                            t= 12.3 s   |
 *   |-----------------------------------+--------------------------|
 *   | tree (navigable)                  | detail pane              |
 *   |  > EK1100                         |  (renderDetail hook)     |
 *   |    +- EL1014                      |                          |
 *   |-----------------------------------+--------------------------|
 *   | log lines ...                                                |
 *   | arrows: navigate   left/right: fold   q: quit   <keyHints>   |
 *   +--------------------------------------------------------------+
 *
 * The application provides hooks: renderDetail draws the right pane for
 * the selected node, onTick refreshes data each frame, onKey receives any
 * key the tree did not consume (e.g. Enter to step a demo).  'q', 'Q' and
 * Escape quit the loop; run() also returns when `cancel` is set or the
 * duration elapses.
 */

#pragma once

#include <atomic>
#include <functional>
#include <string>

#include "tether/terminal_ui/LogPane.hpp"
#include "tether/terminal_ui/Session.hpp"
#include "tether/terminal_ui/TreeView.hpp"

namespace Tether {
namespace TUI {

struct TreeScreenHooks {
    /// Draw the detail pane for the selected node (may be a coupler-level
    /// node or a leaf — check node.tag / node.children).
    std::function<void(TermWindow*, const TreeNode&)> renderDetail;

    /// Called once per frame (~50 ms) before drawing — update model state.
    std::function<void()> onTick;

    /// Keys not consumed by tree navigation arrive here.
    /// Return true when handled (suppresses the 'q' quit check? — no: the
    /// quit keys are checked first; this only sees unclaimed keys).
    std::function<bool(int)> onKey;

    /// Extra text appended to the footer key hints (e.g. "enter: next").
    std::string keyHints;
};

class TreeScreen {
public:
    /// `title` shows in the header bar; `root`'s children become the
    /// top-level tree nodes (all expanded by default via TreeNode).
    TreeScreen(std::string title, TreeNode root, TreeScreenHooks hooks);

    /// Run the draw/dispatch loop until 'q'/Esc, `cancel`, or duration.
    void run(std::atomic<bool>& cancel, double durationSec = 0.0);

    /// Currently selected node (may be null when the tree is empty).
    const TreeNode* selected() const { return tree_.selected(); }

    /// Access to the captured log pane (add application lines).
    LogPane& log() { return log_; }

private:
    Session      session_;
    TreeView     tree_;
    LogPane      log_;
    std::string  title_;
    TreeScreenHooks hooks_;
};

} // namespace TUI
} // namespace Tether
