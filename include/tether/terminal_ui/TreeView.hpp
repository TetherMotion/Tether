/**
 * @file TreeView.hpp
 * @brief Navigable, expandable tree widget for ncurses panes
 *
 * TreeView renders a TreeNode hierarchy into a TermWindow and handles
 * navigation keys: Up/Down move the selection, Right expands a collapsed
 * node (or descends into children), Left collapses (or ascends to the
 * parent), Home/End jump, PageUp/PageDown scroll.  Selection is shown with
 * A_REVERSE; per-node colors come from the Session palette.
 *
 * The node payload is `tag` (an int, e.g. a slave index) so render
 * callbacks can map the selected node back to application state.
 *
 * Usage:
 *   TUI::TreeNode root;   // invisible root — its children are level 1
 *   root.children.push_back({.label = "EK1100", .tag = 0});
 *   TUI::TreeView tree(root);
 *   tree.render(win);
 *   if (tree.handleKey(key)) { ... }
 *   const TUI::TreeNode* sel = tree.selected();
 */

#pragma once

#include <cstddef>
#include <string>
#include <vector>

#include "tether/terminal_ui/Session.hpp"

namespace Tether {
namespace TUI {

struct TreeNode {
    std::string          label;              ///< Text shown in the tree
    std::string          badge;              ///< Small right-aligned indicator
    std::string          detail;             ///< Free-form text for detail panes
    std::vector<TreeNode> children;          ///< Sub-nodes (empty = leaf)
    bool                 expanded = true;    ///< Draw children when true
    int                  tag      = -1;      ///< User payload (e.g. slave index)
    short                color    = PalNone; ///< Palette index for the label
};

class TreeView {
public:
    TreeView() = default;
    explicit TreeView(TreeNode root) { setRoot(std::move(root)); }

    /// Replace the whole tree (root itself is not drawn — its children are
    /// the top-level rows).  Selection resets to the first row.
    void setRoot(TreeNode root);

    /// Currently selected node, or nullptr when the tree is empty.
    const TreeNode* selected() const;

    /// Tag of the selected node, or -1.
    int selectedTag() const;

    /// Move the selection to the node carrying `tag` (no-op if absent).
    void selectByTag(int tag);

    /// Set the right-aligned badge of the node carrying `tag` (no-op if
    /// absent).  Cheap: only the badge string changes, no re-flattening.
    void setBadge(int tag, std::string badge);

    /// Handle one key; returns true when the key was consumed as tree
    /// navigation (arrows, Home/End, PageUp/PageDown, Space/Enter fold).
    bool handleKey(int key);

    /// Move the selection to the next node (in document order, wrapping
    /// around) whose label, badge, or detail contains `needle`
    /// (case-insensitive).  Ancestors of the match are expanded so the
    /// match becomes visible.  Returns false when nothing matches.
    bool searchNext(const std::string& needle);

    /// Draw the visible part of the tree into `win` (cleared first).
    void render(TermWindow* win);

    /// Number of currently visible (i.e. not collapsed-away) rows.
    size_t visibleCount() const { return flat_.size(); }

private:
    void rebuild();   // recompute the flattened visible-row list

    TreeNode                       root_;
    std::vector<const TreeNode*>   flat_;      ///< visible rows, top to bottom
    std::vector<int>               depth_;     ///< indent depth per row
    size_t                         cursor_ = 0;
    size_t                         scroll_ = 0;
};

} // namespace TUI
} // namespace Tether
