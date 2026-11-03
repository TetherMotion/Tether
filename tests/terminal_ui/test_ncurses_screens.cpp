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

// ungetch() pushes to the *front* of the input queue (LIFO): push the
// sequence reversed so the first listed key is delivered first.
static void pushKeys(std::initializer_list<int> keys) {
    for (auto it = keys.end(); it != keys.begin();) ungetch(*--it);
}

// ============================================================================
// Session — pollKeyDecoded escape-tail decoding (CSI / SS3 / lone ESC)
// ============================================================================

TEST_F(NcursesWidgetTest, SessionDecodesCsiArrows) {
    Session s;
    pushKeys({27, '[', 'B'});
    EXPECT_EQ(s.pollKeyDecoded(0), KEY_DOWN);
    pushKeys({27, '[', 'A'});
    EXPECT_EQ(s.pollKeyDecoded(0), KEY_UP);
    pushKeys({27, '[', 'C'});
    EXPECT_EQ(s.pollKeyDecoded(0), KEY_RIGHT);
    pushKeys({27, '[', 'D'});
    EXPECT_EQ(s.pollKeyDecoded(0), KEY_LEFT);
}

TEST_F(NcursesWidgetTest, SessionDecodesCsiSpecials) {
    Session s;
    pushKeys({27, '[', 'H'});
    EXPECT_EQ(s.pollKeyDecoded(0), KEY_HOME);
    pushKeys({27, '[', 'F'});
    EXPECT_EQ(s.pollKeyDecoded(0), KEY_END);
    pushKeys({27, '[', 'Z'});           // Shift-Tab
    EXPECT_EQ(s.pollKeyDecoded(0), KEY_BTAB);
}

TEST_F(NcursesWidgetTest, SessionDecodesCsiTildeParams) {
    Session s;
    pushKeys({27, '[', '2', '~'});
    EXPECT_EQ(s.pollKeyDecoded(0), KEY_IC);
    pushKeys({27, '[', '3', '~'});
    EXPECT_EQ(s.pollKeyDecoded(0), KEY_DC);
    pushKeys({27, '[', '5', '~'});
    EXPECT_EQ(s.pollKeyDecoded(0), KEY_PPAGE);
    pushKeys({27, '[', '6', '~'});
    EXPECT_EQ(s.pollKeyDecoded(0), KEY_NPAGE);
    pushKeys({27, '[', '1', '~'});
    EXPECT_EQ(s.pollKeyDecoded(0), KEY_HOME);
    pushKeys({27, '[', '4', '~'});
    EXPECT_EQ(s.pollKeyDecoded(0), KEY_END);
    pushKeys({27, '[', '1', '5', '~'});
    EXPECT_EQ(s.pollKeyDecoded(0), KEY_F(5));
    pushKeys({27, '[', '2', '4', '~'});
    EXPECT_EQ(s.pollKeyDecoded(0), KEY_F(12));
    pushKeys({27, '[', '9', '9', '~'}); // unknown param → swallowed
    EXPECT_EQ(s.pollKeyDecoded(0), -1);
}

TEST_F(NcursesWidgetTest, SessionDecodesSs3Keys) {
    Session s;
    pushKeys({27, 'O', 'A'});
    EXPECT_EQ(s.pollKeyDecoded(0), KEY_UP);
    pushKeys({27, 'O', 'B'});
    EXPECT_EQ(s.pollKeyDecoded(0), KEY_DOWN);
    pushKeys({27, 'O', 'H'});
    EXPECT_EQ(s.pollKeyDecoded(0), KEY_HOME);
    pushKeys({27, 'O', 'P'});
    EXPECT_EQ(s.pollKeyDecoded(0), KEY_F(1));
    pushKeys({27, 'O', 'X'});           // unknown SS3 final → swallowed
    EXPECT_EQ(s.pollKeyDecoded(0), -1);
}

TEST_F(NcursesWidgetTest, SessionDecodesLoneAndDoubleEsc) {
    Session s;
    pushKeys({27});                      // no tail → lone ESC (30ms wait)
    EXPECT_EQ(s.pollKeyDecoded(0), 27);
    pushKeys({27, 27});                  // ESC ESC → still quit
    EXPECT_EQ(s.pollKeyDecoded(0), 27);
}

TEST_F(NcursesWidgetTest, SessionSwallowsUnknownSequences) {
    Session s;
    pushKeys({27, '[', 'X'});            // unrecognized CSI final
    EXPECT_EQ(s.pollKeyDecoded(0), -1);
    pushKeys({27, 'x'});                 // non-CSI/SS3 tail byte
    EXPECT_EQ(s.pollKeyDecoded(0), -1);
}

TEST_F(NcursesWidgetTest, SessionPollKeyBasics) {
    Session s;
    pushKeys({'a'});
    EXPECT_EQ(s.pollKeyDecoded(0), 'a');  // plain byte passes through
    EXPECT_EQ(s.pollKey(0), ERR);         // empty queue → ERR
    EXPECT_EQ(s.pollKeyDecoded(0), -1);
}

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

TEST_F(NcursesWidgetTest, PanelScreenDigitSwitchesViews) {
    bool v2 = false;
    PanelView a, b;
    a.name = "A";
    a.renderBody = [](int, int) {};
    b.name = "B";
    b.renderBody = [&](int, int) { v2 = true; };
    PanelScreenHooks hooks;
    hooks.views = {a, b};
    PanelScreen screen("T", hooks);
    pushKeys({'2', 'q'});               // '2' → selectView(1), then quit
    std::atomic<bool> cancel{false};
    screen.run(cancel, 0.1);
    EXPECT_TRUE(v2);
}

TEST_F(NcursesWidgetTest, PanelScreenFKeySwitchesViews) {
    bool v2 = false;
    PanelView a, b;
    a.name = "A";
    a.renderBody = [](int, int) {};
    b.name = "B";
    b.renderBody = [&](int, int) { v2 = true; };
    PanelScreenHooks hooks;
    hooks.views = {a, b};
    PanelScreen screen("T", hooks);
    pushKeys({KEY_F(2), 'q'});          // F2 → selectView(1)
    std::atomic<bool> cancel{false};
    screen.run(cancel, 0.1);
    EXPECT_TRUE(v2);
}

TEST_F(NcursesWidgetTest, PanelScreenViewPreDispatchClaimsKeys) {
    int claimed = 0, global = 0;
    PanelView a;
    a.name = "A";
    a.renderBody = [](int, int) {};
    a.wantsKey = [](int k) { return k == 'z'; };
    a.onKey = [&](int k) {
        if (k == 'z') { claimed = k; return true; }
        return false;                   // unclaimed → hooks.onKey
    };
    PanelScreenHooks hooks;
    hooks.views = {a};
    hooks.onKey = [&](int k) { global = k; return true; };
    PanelScreen screen("T", hooks);
    pushKeys({'z', 'y', 'q'});          // 'z'→view, 'y'→view.onKey, 'q'→quit
    std::atomic<bool> cancel{false};
    screen.run(cancel, 0.1);
    EXPECT_EQ(claimed, 'z');            // claimed before screen dispatch
    EXPECT_EQ(global, 'y');             // unclaimed falls to view/global onKey
}

TEST_F(NcursesWidgetTest, PanelScreenKeysOverlayOpensAndCloses) {
    PanelScreenHooks hooks;
    hooks.renderBody = [](int, int) {};
    PanelScreen screen("T", hooks);
    pushKeys({'?'});                    // opens the keys help overlay
    std::atomic<bool> cancel{false};
    screen.run(cancel, 0.1);            // stays open → exits by duration
    bool found = false;
    for (int y = 0; y < LINES; ++y)
        if (rowText(stdscr, y, 0, COLS).find("switch view") != std::string::npos)
            found = true;
    EXPECT_TRUE(found);                 // overlay rendered its help lines
}

TEST_F(NcursesWidgetTest, PanelScreenKeysOverlayEscCloses) {
    PanelScreenHooks hooks;
    hooks.renderBody = [](int, int) {};
    PanelScreen screen("T", hooks);
    // '?' opens the overlay; Esc must be LAST in the queue — a queued byte
    // after ESC would be swallowed as an escape tail instead.
    pushKeys({'?', 27});
    std::atomic<bool> cancel{false};
    screen.run(cancel, 0.15);           // Esc pops the overlay → duration exit
    bool found = false;
    for (int y = 0; y < LINES; ++y)
        if (rowText(stdscr, y, 0, COLS).find("switch view") != std::string::npos)
            found = true;
    EXPECT_FALSE(found);                // overlay closed before the last frame
}

TEST_F(NcursesWidgetTest, PanelScreenFilterEditorCommits) {
    PanelScreenHooks hooks;
    hooks.renderBody = [](int, int) {};
    PanelScreen screen("T", hooks);
    pushKeys({'f', 'w', 'a', 'r', 'n', '\n', 'q'});
    std::atomic<bool> cancel{false};
    screen.run(cancel, 0.2);
    const auto& f = screen.log().viewFilter();
    EXPECT_TRUE(f.active());
    EXPECT_EQ(f.contains, "warn");      // bare word → substring filter
}

TEST_F(NcursesWidgetTest, PanelScreenFilterEditorEscCancels) {
    PanelScreenHooks hooks;
    hooks.renderBody = [](int, int) {};
    PanelScreen screen("T", hooks);
    pushKeys({'f', 'x', 27});           // Esc LAST — cancels without committing
    std::atomic<bool> cancel{false};
    screen.run(cancel, 0.15);
    EXPECT_FALSE(screen.log().viewFilter().active());
}

TEST_F(NcursesWidgetTest, PanelScreenFilterEditorFooterShowsBuffer) {
    PanelScreenHooks hooks;
    hooks.renderBody = [](int, int) {};
    PanelScreen screen("T", hooks);
    pushKeys({'f', 'w', 'a'});          // stays editing → duration exit
    std::atomic<bool> cancel{false};
    screen.run(cancel, 0.1);
    bool found = false;
    for (int y = 0; y < LINES; ++y)
        if (rowText(stdscr, y, 0, COLS).find("filter: wa") != std::string::npos)
            found = true;
    EXPECT_TRUE(found);
}

TEST_F(NcursesWidgetTest, PanelScreenLogScrollKeys) {
    PanelScreenHooks hooks;
    hooks.renderBody = [](int, int) {};
    PanelScreen screen("T", hooks);
    for (int i = 0; i < 30; ++i)
        screen.log().addLine("line " + std::to_string(i));
    pushKeys({KEY_PPAGE, KEY_NPAGE, KEY_END, KEY_PPAGE, 'q'});
    std::atomic<bool> cancel{false};
    screen.run(cancel, 0.15);
    // Exercising scroll paths; the screen must still quit cleanly on 'q'.
}

TEST_F(NcursesWidgetTest, PanelScreenLogViewArrowScroll) {
    PanelScreenHooks hooks;
    hooks.showLogView = true;           // adds the log view as the last tab
    hooks.renderBody = [](int, int) {};
    PanelScreen screen("T", hooks);
    for (int i = 0; i < 30; ++i)
        screen.log().addLine("line " + std::to_string(i));
    pushKeys({'1', KEY_UP, KEY_DOWN, KEY_HOME, 'q'});
    std::atomic<bool> cancel{false};
    screen.run(cancel, 0.15);
    // '1' selects the (only) view = log view → arrows/Home scroll it.
}

TEST_F(NcursesWidgetTest, PanelScreenResizeRelayouts) {
    PanelScreenHooks hooks;
    hooks.renderBody = [](int, int) {};
    PanelScreen screen("T", hooks);
    pushKeys({KEY_RESIZE, 'q'});        // re-runs layout() then quits
    std::atomic<bool> cancel{false};
    screen.run(cancel, 0.1);
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

TEST_F(NcursesWidgetTest, TreeScreenFoldAndToggleKeys) {
    TreeNode root;
    TreeNode c; c.label = "n";
    root.children = {c};
    TreeScreen screen("T", root, TreeScreenHooks{});
    pushKeys({KEY_LEFT, KEY_RIGHT, ' ', KEY_HOME, KEY_END, 'q'});
    std::atomic<bool> cancel{false};
    screen.run(cancel, 0.15);
    // fold/unfold/toggle/Home/End all consumed by the tree view.
}

TEST_F(NcursesWidgetTest, TreeScreenLogScrollKeys) {
    TreeNode root;
    TreeNode c; c.label = "n";
    root.children = {c};
    TreeScreen screen("T", root, TreeScreenHooks{});
    for (int i = 0; i < 30; ++i)
        screen.log().addLine("line " + std::to_string(i));
    pushKeys({KEY_SR, KEY_SF, KEY_END, 'q'});   // Shift-Up/Down + End
    std::atomic<bool> cancel{false};
    screen.run(cancel, 0.15);
}

TEST_F(NcursesWidgetTest, TreeScreenDetailScrollKeys) {
    TreeNode root;
    TreeNode c; c.label = "n";
    root.children = {c};
    // No custom renderDetail → 'u'/'d'/PgUp/PgDn scroll the detail pane.
    TreeScreen screen("T", root, TreeScreenHooks{});
    pushKeys({'u', 'd', KEY_PPAGE, KEY_NPAGE, 'q'});
    std::atomic<bool> cancel{false};
    screen.run(cancel, 0.15);
}

TEST_F(NcursesWidgetTest, TreeScreenPreKeyConsumes) {
    TreeNode root;
    TreeNode c; c.label = "n";
    root.children = {c};
    int pre = 0;
    TreeScreenHooks hooks;
    hooks.preKey = [&](int k) { pre = k; return true; };
    TreeScreen screen("T", root, hooks);
    pushKeys({'x', 'q'});   // 'x' claimed by preKey; 'q' quits (pre-key order)
    std::atomic<bool> cancel{false};
    screen.run(cancel, 0.1);
    EXPECT_EQ(pre, 'x');
}

TEST_F(NcursesWidgetTest, TreeScreenResizeRelayouts) {
    TreeNode root;
    TreeNode c; c.label = "n";
    root.children = {c};
    TreeScreen screen("T", root, TreeScreenHooks{});
    pushKeys({KEY_RESIZE, 'q'});
    std::atomic<bool> cancel{false};
    screen.run(cancel, 0.1);
}

TEST_F(NcursesWidgetTest, TreeScreenLinePromptCommitsOnEnter) {
    TreeNode root;
    TreeNode c; c.label = "n";
    root.children = {c};
    std::string committed;
    int changes = 0;
    TreeScreen screen("T", root, TreeScreenHooks{});
    screen.openLinePrompt({
        .label = "find",
        .on_commit = [&](const std::string& s) { committed = s; },
        .on_change = [&](const std::string&) { ++changes; },
    });
    pushKeys({'a', 'b', '\n', 'q'});    // pops a,b,Enter → commit, 'q' → quit
    std::atomic<bool> cancel{false};
    screen.run(cancel, 0.2);
    EXPECT_EQ(committed, "ab");
    EXPECT_EQ(changes, 2);              // on_change per typed char
}

TEST_F(NcursesWidgetTest, TreeScreenLinePromptEscCancels) {
    TreeNode root;
    TreeNode c; c.label = "n";
    root.children = {c};
    bool cancelled = false;
    TreeScreen screen("T", root, TreeScreenHooks{});
    screen.openLinePrompt({
        .label = "find",
        .on_cancel = [&] { cancelled = true; },
    });
    // Esc must be the only queued key — a byte after it would be eaten as
    // an escape tail by pollKeyDecoded before the prompt sees Esc.
    pushKeys({27});
    std::atomic<bool> cancel{false};
    screen.run(cancel, 0.15);           // prompt closes → duration exit
    EXPECT_TRUE(cancelled);
}

TEST_F(NcursesWidgetTest, TreeScreenLinePromptSwallowsQ) {
    TreeNode root;
    TreeNode c; c.label = "n";
    root.children = {c};
    std::string committed;
    TreeScreen screen("T", root, TreeScreenHooks{});
    screen.openLinePrompt({
        .label = "x",
        .initial = "a",
        .on_commit = [&](const std::string& s) { committed = s; },
    });
    // First 'q' is printable input for the modal prompt (buffer "aq"),
    // Enter commits it, the second 'q' — after the prompt closed — quits.
    pushKeys({'q', '\n', 'q'});
    std::atomic<bool> cancel{false};
    screen.run(cancel, 0.2);
    EXPECT_EQ(committed, "aq");
}

TEST_F(NcursesWidgetTest, TreeScreenLinePromptSuppressesDetail) {
    TreeNode root;
    TreeNode c; c.label = "n";
    root.children = {c};
    bool detail_called = false;
    TreeScreenHooks hooks;
    hooks.renderDetail = [&](TermWindow*, const TreeNode&) {
        detail_called = true;
    };
    TreeScreen screen("T", root, hooks);
    screen.openLinePrompt({.label = "x"});
    pushKeys({'x'});                    // prompt stays open → duration exit
    std::atomic<bool> cancel{false};
    screen.run(cancel, 0.1);
    // While the prompt is active the detail pane renders the prompt,
    // never the node detail.
    EXPECT_FALSE(detail_called);
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
