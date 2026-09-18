/**
 * @file TreeScreen.cpp
 * @brief Composed tree + detail + log screen implementation
 */

#include "tether/terminal_ui/TreeScreen.hpp"

#include <algorithm>
#include <chrono>

#include <ncurses.h>

#include "tether/terminal_ui/TextWrap.hpp"

namespace Tether {
namespace TUI {

TreeScreen::TreeScreen(std::string title, TreeNode root, TreeScreenHooks hooks)
    : title_(std::move(title)), hooks_(std::move(hooks)) {
    tree_.setRoot(std::move(root));
    log_.attach();   // capture console logging while curses owns the screen
}

// ---------------------------------------------------------------------------
// Line prompt (modal)
// ---------------------------------------------------------------------------

void TreeScreen::openLinePrompt(LinePromptSpec spec) {
    prompt_spec_ = std::move(spec);
    prompt_buf_  = prompt_spec_.initial;
    prompt_active_ = true;
}

void TreeScreen::handlePromptKey(int key) {
    switch (key) {
        case 27:  // Esc cancels
            prompt_active_ = false;
            if (prompt_spec_.on_cancel) prompt_spec_.on_cancel();
            return;
        case '\n':
        case '\r':
        case KEY_ENTER:
            prompt_active_ = false;
            if (prompt_spec_.on_commit) prompt_spec_.on_commit(prompt_buf_);
            return;
        case KEY_BACKSPACE:
        case 127:
        case '\b':
            if (!prompt_buf_.empty()) prompt_buf_.pop_back();
            return;
        default:
            if (key >= 32 && key <= 126)
                prompt_buf_ += static_cast<char>(key);
            return;
    }
}

void TreeScreen::renderPrompt(TermWindow* w) {
    WINDOW* win = static_cast<WINDOW*>(w);
    wattron(win, A_BOLD | COLOR_PAIR(PalHint));
    mvwprintw(win, 0, 1, "%s: %s_", prompt_spec_.label.c_str(),
              prompt_buf_.c_str());
    wattroff(win, A_BOLD | COLOR_PAIR(PalHint));
    wattron(win, A_DIM);
    int row = 1;
    for (const auto& hint : prompt_spec_.hint_lines) {
        mvwprintw(win, row++, 1, "%s", hint.c_str());
    }
    wattroff(win, A_DIM);
}

// ---------------------------------------------------------------------------
// Built-in detail renderer (word-wrap + u/d scroll)
// ---------------------------------------------------------------------------

void TreeScreen::renderWrappedDetail(TermWindow* w, const TreeNode& node) {
    WINDOW* win = static_cast<WINDOW*>(w);
    int h = 0, width = 0;
    getmaxyx(win, h, width);
    if (h <= 0 || width <= 2) return;

    if (&node != detail_last_sel_) {
        detail_last_sel_ = &node;
        detail_scroll_ = 0;
    }

    const auto lines = wrapText(node.detail, width - 2);
    const int maxScroll =
        std::max(0, static_cast<int>(lines.size()) - h);
    detail_scroll_ = std::clamp(detail_scroll_, 0, maxScroll);
    for (int r = 0; r < h; ++r) {
        const int idx = detail_scroll_ + r;
        if (idx >= static_cast<int>(lines.size())) break;
        if (idx == 0) wattron(win, A_BOLD);
        mvwprintw(win, r, 1, "%s", lines[idx].c_str());
        if (idx == 0) wattroff(win, A_BOLD);
    }
    if (maxScroll > 0 && width >= 12) {
        wattron(win, A_DIM);
        mvwprintw(win, h - 1, width - 10, " u/d %d/%d ",
                  detail_scroll_, maxScroll);
        wattroff(win, A_DIM);
    }
}

void TreeScreen::run(std::atomic<bool>& cancel, double durationSec) {
    const auto t0 = std::chrono::steady_clock::now();

    TermWindow* treeWin   = nullptr;
    TermWindow* detailWin = nullptr;
    TermWindow* logWin    = nullptr;

    auto layout = [&]() {
        session_.freeWindow(treeWin);
        session_.freeWindow(detailWin);
        session_.freeWindow(logWin);

        const int H = session_.rows();
        const int W = session_.cols();
        const int logH    = static_cast<int>(log_.maxLines());
        const int footerY = H - 1;
        const int logY    = footerY - logH;
        const int top     = 1;              // header row 0
        const int bottom  = logY - 1;       // separator above log
        const int contentH = bottom - top;
        const int leftW = std::max(24, W * 2 / 5);

        treeWin   = session_.makeWindow(top, 0, contentH, leftW - 1);
        detailWin = session_.makeWindow(top, leftW, contentH, W - leftW);
        logWin    = session_.makeWindow(logY, 0, logH, W);
        return footerY;
    };

    int footerY = layout();

    while (!cancel.load()) {
        const double elapsed = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - t0).count();
        if (durationSec > 0.0 && elapsed >= durationSec) break;

        if (hooks_.onTick) hooks_.onTick();

        // ---- Header ----------------------------------------------------
        erase();
        attron(A_BOLD | (session_.colors() ? COLOR_PAIR(PalHeader) : 0));
        mvprintw(0, 0, " %s", title_.c_str());
        attroff(A_BOLD | (session_.colors() ? COLOR_PAIR(PalHeader) : 0));
        mvprintw(0, session_.cols() - 12, "t=%6.1f s", elapsed);

        // ---- Footer -----------------------------------------------------
        if (session_.colors()) attron(COLOR_PAIR(PalHint));
        mvprintw(footerY, 0,
                 " arrows: navigate  left/right: fold  q: quit"
                 "  S-Up/S-Dn/End: log%s%s",
                 hooks_.keyHints.empty() ? "" : "  ",
                 hooks_.keyHints.c_str());
        if (session_.colors()) attroff(COLOR_PAIR(PalHint));

        // ---- Panes -----------------------------------------------------
        const int leftW = std::max(24, session_.cols() * 2 / 5);
        // Separator sits one row above the log window (logWin starts at
        // footerY - logH); without the -1 the log pane paints over it.
        const int sepY = footerY - static_cast<int>(log_.maxLines()) - 1;
        for (int r = 1; r < sepY; ++r) {
            mvprintw(r, leftW - 1, "%s", "\xE2\x94\x82"); // │
        }
        const int hlineW = std::max(0, session_.cols() - 2);
        std::string hline;
        hline.reserve(hlineW * 3);
        for (int i = 0; i < hlineW; ++i) hline += "\xE2\x94\x80"; // ─
        mvprintw(sepY, 1, "%s", hline.c_str());
        // Tees drawn after the hline so they aren't overwritten by it.
        mvprintw(sepY, 0, "%s", "\xE2\x94\x9C");            // ├
        mvprintw(sepY, leftW - 1, "%s", "\xE2\x94\xBC");    // ┼

        // Push the stdscr frame first, then each subwindow over its own region,
        // and update the physical screen once at the end.  The previous
        // wrefresh(stdscr) at the end was repainting stdscr over the panes.
        wnoutrefresh(stdscr);

        tree_.render(treeWin);
        wnoutrefresh(static_cast<WINDOW*>(treeWin));

        werase(static_cast<WINDOW*>(detailWin));
        if (prompt_active_) {
            renderPrompt(detailWin);
        } else if (const TreeNode* sel = tree_.selected()) {
            if (hooks_.renderDetail) {
                hooks_.renderDetail(detailWin, *sel);
            } else {
                renderWrappedDetail(detailWin, *sel);
            }
        }
        wnoutrefresh(static_cast<WINDOW*>(detailWin));

        log_.render(logWin);
        wnoutrefresh(static_cast<WINDOW*>(logWin));

        doupdate();

        // ---- Input ------------------------------------------------------
        const int key = session_.pollKey(50);
        if (key == KEY_RESIZE) { footerY = layout(); continue; }
        // An open line prompt is modal: every key goes to it — including
        // 'q'/'Q' (printable → appended) and Esc (cancels the prompt).
        if (prompt_active_) { handlePromptKey(key); continue; }
        // Shift-Up/Down scroll the captured log (PgUp/PgDn already
        // scroll the detail pane); End jumps back to the newest line.
        if (key == KEY_SR) { log_.scrollLines(-1); continue; }
        if (key == KEY_SF) { log_.scrollLines(1); continue; }
        if (key == KEY_END) { log_.scrollToEnd(); continue; }
        if (key == 'q' || key == 'Q' || key == 27) {
            if (hooks_.quitGuard && hooks_.quitGuard()) {
                if (hooks_.onKey) hooks_.onKey(key);
                continue;
            }
            break;
        }
        if (key < 0) continue;
        if (hooks_.preKey && hooks_.preKey(key)) continue;
        if (tree_.handleKey(key)) continue;
        // With the built-in detail renderer the pane scroll keys are
        // consumed here; a custom renderDetail keeps full control.
        if (!hooks_.renderDetail) {
            if (key == 'u' || key == KEY_PPAGE) { --detail_scroll_; continue; }
            if (key == 'd' || key == KEY_NPAGE) { ++detail_scroll_; continue; }
        }
        if (hooks_.onKey) hooks_.onKey(key);
    }
}

} // namespace TUI
} // namespace Tether
