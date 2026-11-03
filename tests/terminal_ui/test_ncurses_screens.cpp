/**
 * @file test_ncurses_screens.cpp
 * @brief Headless coverage for the composed screens — PanelScreen and
 *        TreeScreen — by running their real draw/dispatch loops for a
 *        few frames against a /dev/null-backed newterm() SCREEN.
 *
 * run(cancel, durationSec) exits by duration, and ungetch() injects a
 * queued key so 'q' exercises the quit path — hooks see real frames.
 */

#include <gtest/gtest.h>

#include <atomic>
#include <string>
#include <vector>

#include "tether/ui/Overlay.hpp"

#include <clocale>
#include <ncurses.h>

#include "tether/terminal_ui/Session.hpp"
#include "tether/terminal_ui/PanelScreen.hpp"
#include "tether/terminal_ui/TreeScreen.hpp"
#include "tether/terminal_ui/TreeView.hpp"

#include "NcursesTestEnv.hpp"

using namespace Tether::TUI;
using namespace NcursesTest;
namespace UI = Tether::UI;

namespace {

// ============================================================================
// PanelScreen — draw loop, views, key routing
// ============================================================================

TEST_F(NcursesWidgetTest, PanelScreenDrawsTitleAndBody) {
    bool ticked = false, rendered = false;
    PanelScreenHooks hooks;
    hooks.onTick = [&] { ticked = true; };
    hooks.renderBody = [&](int top, int bottom) {
        rendered = true;
        mvprintw(top, 0, "BODY-CONTENT");
    };
    PanelScreen screen("TEST-TITLE", hooks);
    std::atomic<bool> cancel{false};
    screen.run(cancel, 0.05);   // ~one frame then duration exit
    EXPECT_TRUE(ticked);
    EXPECT_TRUE(rendered);
    EXPECT_NE(rowText(stdscr, 0).find("TEST-TITLE"), std::string::npos);
    EXPECT_NE(rowText(stdscr, 1).find("BODY-CONTENT"), std::string::npos);
}

TEST_F(NcursesWidgetTest, PanelScreenQuitsOnQ) {
    int frames = 0;
    PanelScreenHooks hooks;
    hooks.onTick = [&] { ++frames; };
    PanelScreen screen("T", hooks);
    ungetch('q');               // inject quit key
    std::atomic<bool> cancel{false};
    screen.run(cancel, 10.0);   // long duration — must exit on 'q'
    EXPECT_LE(frames, 3);       // ran at most a couple frames
}

TEST_F(NcursesWidgetTest, PanelScreenQuitGuardRedirectsQ) {
    bool guarded = false;
    PanelScreenHooks hooks;
    bool q_seen = false;
    hooks.quitGuard = [&] { guarded = true; return true; };
    hooks.onKey = [&](int k) { if (k == 'q') q_seen = true; return true; };
    PanelScreen screen("T", hooks);
    // ungetch pushes to the *front* of the queue (LIFO): queue Esc first so
    // 'q' is delivered before it.  Both quit keys must hit the guard → onKey.
    ungetch(27);
    ungetch('q');
    std::atomic<bool> cancel{false};
    screen.run(cancel, 0.15);
    EXPECT_TRUE(guarded);
    EXPECT_TRUE(q_seen);
}

TEST_F(NcursesWidgetTest, PanelScreenMultipleViews) {
    bool v1 = false, v2 = false;
    PanelView a, b;
    a.name = "V1";
    a.renderBody = [&](int top, int) { mvprintw(top, 0, "V1BODY"); v1 = true; };
    b.name = "V2";
    b.renderBody = [&](int top, int) { mvprintw(top, 0, "V2BODY"); v2 = true; };
    PanelScreenHooks hooks;
    hooks.views = {a, b};
    PanelScreen screen("T", hooks);

    screen.selectView(1);       // activate second view
    ungetch('q');
    std::atomic<bool> cancel{false};
    screen.run(cancel, 0.1);
    EXPECT_TRUE(v2);
}

TEST_F(NcursesWidgetTest, PanelScreenTabSwitchesViews) {
    bool v1 = false, v2 = false;
    PanelView a, b;
    a.name = "A";
    a.renderBody = [&](int top, int) { mvprintw(top, 0, "ABODY"); v1 = true; };
    b.name = "B";
    b.renderBody = [&](int top, int) { mvprintw(top, 0, "BBODY"); v2 = true; };
    PanelScreenHooks hooks;
    hooks.views = {a, b};
    PanelScreen screen("T", hooks);
    ungetch('q');               // LIFO — queue 'q' first so Tab pops first
    ungetch('\t');              // Tab → next view
    std::atomic<bool> cancel{false};
    screen.run(cancel, 0.1);
    EXPECT_TRUE(v2);
}

TEST_F(NcursesWidgetTest, PanelScreenLogView) {
    PanelScreenHooks hooks;
    hooks.showLogView = true;
    hooks.renderBody = [](int top, int) { mvprintw(top, 0, "MAIN"); };
    PanelScreen screen("T", hooks);
    screen.log().addLine("log line here");
    ungetch('q');
    std::atomic<bool> cancel{false};
    screen.run(cancel, 0.1);
}

TEST_F(NcursesWidgetTest, PanelScreenFlashNotice) {
    PanelScreenHooks hooks;
    hooks.renderBody = [](int, int) {};
    PanelScreen screen("T", hooks);
    screen.flash("saved!");     // transient footer notice
    ungetch('q');
    std::atomic<bool> cancel{false};
    screen.run(cancel, 0.05);
    // The flash notice replaces the footer hints — footerY is the last row.
    bool found = false;
    for (int y = 0; y < LINES; ++y)
        if (rowText(stdscr, y).find("saved") != std::string::npos) found = true;
    EXPECT_TRUE(found);
}

TEST_F(NcursesWidgetTest, PanelScreenOverlayRenders) {
    PanelScreenHooks hooks;
    hooks.renderBody = [](int top, int) { mvprintw(top, 0, "BASE"); };
    PanelScreen screen("T", hooks);
    screen.pushOverlay(std::make_unique<UI::TextOverlay>(
        "OVL", std::vector<std::string>{"overlay text"}));
    ungetch('q');
    std::atomic<bool> cancel{false};
    screen.run(cancel, 0.05);
    bool found = false;
    for (int y = 0; y < 12; ++y)
        if (rowText(stdscr, y).find("overlay text") != std::string::npos)
            found = true;
    EXPECT_TRUE(found);
}

TEST_F(NcursesWidgetTest, PanelScreenCancelStopsImmediately) {
    int frames = 0;
    PanelScreenHooks hooks;
    hooks.onTick = [&] { ++frames; };
    PanelScreen screen("T", hooks);
    std::atomic<bool> cancel{true};
    screen.run(cancel, 10.0);
    EXPECT_EQ(frames, 0);
}

// ============================================================================
// TreeScreen — tree + detail + log panes in a live frame
// ============================================================================

TEST_F(NcursesWidgetTest, TreeScreenDrawsTreeAndDetail) {
    TreeNode root;
    root.label = "root";
    TreeNode child;
    child.label = "EK1100";
    child.tag = 0;
    root.children = {child};

    const TreeNode* seen = nullptr;
    TreeScreenHooks hooks;
    hooks.renderDetail = [&](TermWindow* win, const TreeNode& n) {
        seen = &n;
        mvwprintw(static_cast<WINDOW*>(win), 0, 0, "detail:%s", n.label.c_str());
    };
    TreeScreen screen("TREE-T", root, hooks);
    ungetch('q');
    std::atomic<bool> cancel{false};
    screen.run(cancel, 0.1);
    // The tree/detail panes are sub-windows — observe selection through the
    // renderDetail hook rather than stdscr text.
    ASSERT_NE(seen, nullptr);
    EXPECT_EQ(seen->label, "EK1100");
}

TEST_F(NcursesWidgetTest, TreeScreenNavigatesWithArrows) {
    TreeNode root;
    TreeNode c1; c1.label = "node1";
    TreeNode c2; c2.label = "node2";
    root.children = {c1, c2};

    const TreeNode* seen = nullptr;
    TreeScreenHooks hooks;
    hooks.renderDetail = [&](TermWindow*, const TreeNode& n) { seen = &n; };
    TreeScreen screen("T", root, hooks);
    ungetch('q');               // LIFO — 'q' queued first, arrow pops first
    ungetch(KEY_DOWN);          // select node2
    std::atomic<bool> cancel{false};
    screen.run(cancel, 0.1);
    ASSERT_NE(seen, nullptr);
    EXPECT_EQ(seen->label, "node2");
}

TEST_F(NcursesWidgetTest, TreeScreenOnKeyReceivesUnconsumed) {
    TreeNode root;
    TreeNode c; c.label = "n";
    root.children = {c};
    int got = 0;
    TreeScreenHooks hooks;
    hooks.onKey = [&](int k) { got = k; return true; };
    TreeScreen screen("T", root, hooks);
    ungetch('q');               // LIFO — 'x' must pop before 'q'
    ungetch('x');               // unconsumed → onKey
    std::atomic<bool> cancel{false};
    screen.run(cancel, 0.1);
    EXPECT_EQ(got, 'x');
}

TEST_F(NcursesWidgetTest, TreeScreenQuitGuard) {
    TreeNode root;
    TreeNode c; c.label = "n";
    root.children = {c};
    bool q_seen = false;
    TreeScreenHooks hooks;
    hooks.quitGuard = [] { return true; };
    hooks.onKey = [&](int k) { if (k == 'q') q_seen = true; return true; };
    TreeScreen screen("T", root, hooks);
    ungetch('q');
    std::atomic<bool> cancel{false};
    screen.run(cancel, 0.05);   // exits by duration — 'q' went to onKey
    EXPECT_TRUE(q_seen);
}

TEST_F(NcursesWidgetTest, TreeScreenLogPaneAccess) {
    TreeNode root;
    TreeNode c; c.label = "n";
    root.children = {c};
    TreeScreen screen("T", root, TreeScreenHooks{});
    screen.log().addLine("app log");
    ungetch('q');
    std::atomic<bool> cancel{false};
    screen.run(cancel, 0.05);
}

TEST_F(NcursesWidgetTest, TreeScreenFlashAndOverlay) {
    TreeNode root;
    TreeNode c; c.label = "n";
    root.children = {c};
    TreeScreen screen("T", root, TreeScreenHooks{});
    screen.flash("flashed");
    screen.pushOverlay(std::make_unique<UI::TextOverlay>(
        "o", std::vector<std::string>{"ovl"}));
    ungetch('q');
    std::atomic<bool> cancel{false};
    screen.run(cancel, 0.05);
}

TEST_F(NcursesWidgetTest, TreeScreenKeyHintsInFooter) {
    TreeNode root;
    TreeNode c; c.label = "n";
    root.children = {c};
    TreeScreenHooks hooks;
    hooks.keyHints = "custom: hint";
    TreeScreen screen("T", root, hooks);
    ungetch('q');
    std::atomic<bool> cancel{false};
    screen.run(cancel, 0.05);
    // Footer hints are drawn on stdscr at footerY (last row) — the custom
    // hint sits past col 60, so read the full width.
    bool found = false;
    for (int y = 0; y < LINES; ++y)
        if (rowText(stdscr, y, 0, COLS).find("custom: hint") != std::string::npos)
            found = true;
    EXPECT_TRUE(found);
}

} // namespace
