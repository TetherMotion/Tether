/**
 * @file TreeView.cpp
 * @brief Navigable tree widget implementation
 */

#include "tether/terminal_ui/TreeView.hpp"

#include <algorithm>
#include <cwchar>

#include <ncurses.h>

namespace Tether {
namespace TUI {

// Truncate a UTF-8 string to at most `maxCols` display columns without
// splitting a multibyte codepoint.  Our labels are Latin + box-drawing, all
// width-1, so counting codepoints is sufficient.
static std::string utf8Trunc(const std::string& s, int maxCols) {
    if (maxCols <= 0) return {};
    std::mbstate_t st{};
    int    cols = 0;
    size_t i    = 0;
    while (i < s.size() && cols < maxCols) {
        wchar_t      wc = 0;
        const size_t n  = std::mbrtowc(&wc, s.data() + i, s.size() - i, &st);
        if (n == (size_t)-1 || n == (size_t)-2) { ++i; ++cols; continue; }
        if (n == 0) { ++i; continue; }
        i += n;
        ++cols;
    }
    return s.substr(0, i);
}

// Display width in columns: our glyphs are all width-1, so this is just the
// codepoint count (invalid bytes count as one column each).
static int utf8Cols(const std::string& s) {
    std::mbstate_t st{};
    int    cols = 0;
    size_t i    = 0;
    while (i < s.size()) {
        wchar_t      wc = 0;
        const size_t n  = std::mbrtowc(&wc, s.data() + i, s.size() - i, &st);
        if (n == (size_t)-1 || n == (size_t)-2) { ++i; ++cols; continue; }
        if (n == 0) { ++i; continue; }
        i += n;
        ++cols;
    }
    return cols;
}

void TreeView::setRoot(TreeNode root) {
    root_   = std::move(root);
    cursor_ = 0;
    scroll_ = 0;
    rebuild();
}

const TreeNode* TreeView::selected() const {
    return cursor_ < flat_.size() ? flat_[cursor_] : nullptr;
}

int TreeView::selectedTag() const {
    const TreeNode* n = selected();
    return n ? n->tag : -1;
}

void TreeView::selectByTag(int tag) {
    for (size_t i = 0; i < flat_.size(); ++i) {
        if (flat_[i]->tag == tag) { cursor_ = i; return; }
    }
}

void TreeView::setBadge(int tag, std::string badge) {
    std::vector<TreeNode*> stack;
    for (auto& c : root_.children) stack.push_back(&c);
    while (!stack.empty()) {
        TreeNode* n = stack.back();
        stack.pop_back();
        if (n->tag == tag) { n->badge = std::move(badge); return; }
        for (auto& c : n->children) stack.push_back(&c);
    }
}

bool TreeView::handleKey(int key) {
    if (flat_.empty()) return false;

    auto* cur = const_cast<TreeNode*>(flat_[cursor_]);
    switch (key) {
    case KEY_UP:
        if (cursor_ > 0) --cursor_;
        return true;
    case KEY_DOWN:
        if (cursor_ + 1 < flat_.size()) ++cursor_;
        return true;
    case KEY_HOME:
        cursor_ = 0;
        return true;
    case KEY_END:
        cursor_ = flat_.size() - 1;
        return true;
    case KEY_PPAGE:
        cursor_ = cursor_ > 8 ? cursor_ - 8 : 0;
        return true;
    case KEY_NPAGE:
        cursor_ = std::min(cursor_ + 8, flat_.size() - 1);
        return true;
    case KEY_RIGHT:
        if (!cur->children.empty()) {
            if (cur->expanded) {
                // Descend into first child.
                if (cursor_ + 1 < flat_.size()) ++cursor_;
            } else {
                cur->expanded = true;
                rebuild();
            }
        }
        return true;
    case KEY_LEFT:
        if (!cur->children.empty() && cur->expanded) {
            cur->expanded = false;
            rebuild();
        } else {
            // Ascend to parent: find the enclosing node.
            const int myDepth = depth_[cursor_];
            for (size_t i = cursor_; i-- > 0;) {
                if (depth_[i] < myDepth) { cursor_ = i; break; }
            }
        }
        return true;
    default:
        return false;
    }
}

void TreeView::render(TermWindow* w) {
    WINDOW* win = static_cast<WINDOW*>(w);
    werase(win);

    int h = 0, width = 0;
    getmaxyx(win, h, width);
    if (h <= 0 || width <= 0) return;

    // Keep the cursor inside the scroll window.
    if (cursor_ < scroll_) scroll_ = cursor_;
    if (cursor_ >= scroll_ + static_cast<size_t>(h)) {
        scroll_ = cursor_ - static_cast<size_t>(h) + 1;
    }

    for (size_t i = scroll_; i < flat_.size(); ++i) {
        const int row = static_cast<int>(i - scroll_);
        if (row >= h) break;

        const TreeNode& n = *flat_[i];
        const int  indent = depth_[i] * 2;
        const bool sel = (i == cursor_);
        const char* mark = n.children.empty() ? " "
                         : (n.expanded ? "\xE2\x96\xBC"   // ▼
                                        : "\xE2\x96\xB6"); // ▶

        int x = indent;
        if (x < width - 1) mvwprintw(win, row, x, "%s", mark);
        x += 2;

        int attrs = sel ? A_REVERSE : A_NORMAL;
        if (n.color != PalNone) attrs |= COLOR_PAIR(n.color);
        if (!n.children.empty() && !sel) attrs |= A_BOLD;

        // Right-aligned badge (e.g. live bit states) — reserves its column
        // width so the label truncates without overlapping it.
        int badgeX = -1;
        if (!n.badge.empty()) {
            const int bw = utf8Cols(n.badge);
            badgeX = width - bw - 1;
            if (badgeX <= x + 1) badgeX = -1;   // too narrow — drop it
        }

        wattron(win, attrs);
        const int labelCols = badgeX > 0 ? badgeX - x - 1 : width - x - 1;
        mvwprintw(win, row, x, "%s", utf8Trunc(n.label, labelCols).c_str());
        if (badgeX > 0)
            mvwprintw(win, row, badgeX, "%s", n.badge.c_str());
        wattroff(win, attrs);
    }
}

void TreeView::rebuild() {
    flat_.clear();
    depth_.clear();

    // Iterative DFS of visible nodes.
    struct Frame { const TreeNode* node; int depth; };
    std::vector<Frame> stack;
    for (auto it = root_.children.rbegin(); it != root_.children.rend(); ++it) {
        stack.push_back({&*it, 0});
    }
    while (!stack.empty()) {
        const Frame f = stack.back();
        stack.pop_back();
        flat_.push_back(f.node);
        depth_.push_back(f.depth);
        if (f.node->expanded) {
            for (auto it = f.node->children.rbegin();
                 it != f.node->children.rend(); ++it) {
                stack.push_back({&*it, f.depth + 1});
            }
        }
    }

    if (cursor_ >= flat_.size() && !flat_.empty()) cursor_ = flat_.size() - 1;
}

} // namespace TUI
} // namespace Tether
