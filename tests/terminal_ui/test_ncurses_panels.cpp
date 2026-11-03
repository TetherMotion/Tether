/**
 * @file test_ncurses_panels.cpp
 * @brief Headless-ncurses coverage for the composed panels —
 *        TreeView, LogPane, TablePanel, OverlayHost,
 *        EntityBrowserPanel — rendered into newterm()-backed windows
 *        and verified by reading cells back.
 */

#include <gtest/gtest.h>

#include <string>
#include <vector>

#include "tether/ui/Panels.hpp"
#include "tether/ui/TableModel.hpp"
#include "tether/ui/Overlay.hpp"
#include "tether/platform/Platform.hpp"
#include "logging/Logger.hpp"

#include <clocale>
#include <ncurses.h>

#include "tether/terminal_ui/Session.hpp"
#include "tether/terminal_ui/TreeView.hpp"
#include "tether/terminal_ui/LogPane.hpp"
#include "tether/terminal_ui/TablePanel.hpp"
#include "tether/terminal_ui/OverlayHost.hpp"
#include "tether/terminal_ui/EntityBrowserPanel.hpp"
#include "tether/terminal_ui/AnsiLine.hpp"

#include "NcursesTestEnv.hpp"

using namespace Tether::TUI;
using namespace NcursesTest;
namespace UI = Tether::UI;
namespace P = Tether::Platform;

namespace {

TreeNode leaf(const std::string& label, int tag = -1) {
    TreeNode n;
    n.label = label;
    n.tag = tag;
    return n;
}

// ============================================================================
// TreeView — navigation, folding, badges, search
// ============================================================================

class TreeViewTest : public NcursesWidgetTest {
protected:
    TreeNode makeTree() {
        TreeNode root;
        root.label = "root";
        TreeNode a = leaf("coupler", 1);
        a.children = {leaf("term1", 2), leaf("term2", 3)};
        root.children = {a, leaf("drive", 4)};
        return root;
    }
};

TEST_F(TreeViewTest, RenderDrawsVisibleNodes) {
    W w;
    TreeView tv(makeTree());
    tv.render(w);
    EXPECT_NE(rowText(w, 0).find("coupler"), std::string::npos);
    EXPECT_NE(rowText(w, 1).find("term1"), std::string::npos);
    EXPECT_NE(rowText(w, 3).find("drive"), std::string::npos);
    EXPECT_EQ(tv.visibleCount(), 4u);
}

TEST_F(TreeViewTest, ArrowKeysMoveSelection) {
    W w;
    TreeView tv(makeTree());
    EXPECT_EQ(tv.selected()->tag, 1);   // first child selected
    EXPECT_TRUE(tv.handleKey(KEY_DOWN));
    EXPECT_EQ(tv.selected()->tag, 2);
    EXPECT_TRUE(tv.handleKey(KEY_DOWN));
    EXPECT_EQ(tv.selected()->tag, 3);
    EXPECT_TRUE(tv.handleKey(KEY_UP));
    EXPECT_EQ(tv.selected()->tag, 2);
}

TEST_F(TreeViewTest, RightExpandsLeftCollapses) {
    W w;
    auto tree = makeTree();
    tree.children[0].expanded = false;   // coupler collapsed
    TreeView tv(tree);
    EXPECT_EQ(tv.visibleCount(), 2u);    // coupler + drive
    EXPECT_TRUE(tv.handleKey(KEY_RIGHT)); // expand
    EXPECT_EQ(tv.visibleCount(), 4u);
    EXPECT_TRUE(tv.handleKey(KEY_LEFT));  // collapse
    EXPECT_EQ(tv.visibleCount(), 2u);
}

TEST_F(TreeViewTest, HomeEndJumpToEnds) {
    W w;
    TreeView tv(makeTree());
    tv.handleKey(KEY_END);
    EXPECT_EQ(tv.selected()->tag, 4);
    tv.handleKey(KEY_HOME);
    EXPECT_EQ(tv.selected()->tag, 1);
}

TEST_F(TreeViewTest, SelectByTagAndBadge) {
    W w;
    TreeView tv(makeTree());
    tv.selectByTag(4);
    EXPECT_EQ(tv.selected()->tag, 4);
    tv.setBadge(4, "ERR");
    tv.render(w);
    EXPECT_NE(rowText(w, 3).find("ERR"), std::string::npos);
}

TEST_F(TreeViewTest, SearchNextFindsAndExpands) {
    W w;
    auto tree = makeTree();
    tree.children[0].expanded = false;
    TreeView tv(tree);
    EXPECT_TRUE(tv.searchNext("term2"));
    EXPECT_EQ(tv.selected()->tag, 3);
    EXPECT_FALSE(tv.searchNext("nonexistent-node"));
}

TEST_F(TreeViewTest, SelectedRowHighlighted) {
    W w;
    TreeView tv(makeTree());
    tv.render(w);
    // Selected row should carry a highlight attribute on some cell
    bool found_attr = false;
    for (int x = 0; x < 20; ++x) {
        short pair = 0; attr_t a = 0;
        cellAttrs(w, 0, x, pair, a);
        if (pair != 0 || (a & (A_REVERSE | A_BOLD | A_STANDOUT))) found_attr = true;
    }
    EXPECT_TRUE(found_attr);
}

TEST_F(TreeViewTest, EmptyTreeRendersNothing) {
    W w;
    TreeView tv(TreeNode{});
    tv.render(w);
    EXPECT_EQ(tv.visibleCount(), 0u);
}

// ============================================================================
// LogPane — captured log lines, filter, scroll, render
// ============================================================================

TEST(LogPaneTest, AddAndRenderLines) {
    ASSERT_TRUE(Ctx::get().ok);
    W w;
    LogPane pane(4);
    pane.addLine("I tag1: first line");
    pane.addLine("W tag2: second line");
    pane.render(w);
    EXPECT_NE(rowText(w, 0).find("first"), std::string::npos);
    EXPECT_NE(rowText(w, 1).find("second"), std::string::npos);
}

TEST(LogPaneTest, CapacityDropsOldLines) {
    ASSERT_TRUE(Ctx::get().ok);
    W w;
    LogPane pane(4, 3);   // capacity 3
    pane.addLine("l1");
    pane.addLine("l2");
    pane.addLine("l3");
    pane.addLine("l4");   // pushes out l1 — pane renders oldest at top
    pane.render(w);
    EXPECT_EQ(rowText(w, 0), "l2");
    EXPECT_EQ(rowText(w, 2), "l4");
}

TEST(LogPaneTest, ScrollLinesAndFollow) {
    ASSERT_TRUE(Ctx::get().ok);
    W w;
    LogPane pane(2, 10);
    for (int i = 0; i < 6; ++i) pane.addLine("line " + std::to_string(i));
    EXPECT_TRUE(pane.following());
    pane.scrollLines(-3);   // scroll up → not following
    EXPECT_FALSE(pane.following());
    pane.render(w);
    pane.scrollToEnd();
    EXPECT_TRUE(pane.following());
}

TEST(LogPaneTest, LevelFilterHidesLines) {
    ASSERT_TRUE(Ctx::get().ok);
    W w;
    LogPane pane(4);
    pane.addLine("info line", P::LogLevel::Info, "tagA");
    pane.addLine("err line", P::LogLevel::Error, "tagB");
    pane.setMinLevel(P::LogLevel::Error);   // ingest cap — keeps Error only
    pane.addLine("dbg line", P::LogLevel::Debug, "tagC");  // dropped
    pane.addLine("err2", P::LogLevel::Error, "tagD");      // kept
    pane.render(w);
    EXPECT_NE(rowText(w, 0).find("info"), std::string::npos);   // ingested earlier
    EXPECT_EQ(rowText(w, 0).find("err"), std::string::npos);
    EXPECT_NE(rowText(w, 1).find("err"), std::string::npos);
}

TEST(LogPaneTest, TagIncludeExcludeFilter) {
    ASSERT_TRUE(Ctx::get().ok);
    W w;
    LogPane pane(4);
    pane.addLine("a", P::LogLevel::Info, "alpha");
    pane.addLine("b", P::LogLevel::Info, "beta");
    pane.excludeTag("beta");
    pane.render(w);
    EXPECT_NE(rowText(w, 0).find("a"), std::string::npos);
    EXPECT_EQ(rowText(w, 0).find("b"), std::string::npos);
    EXPECT_TRUE(pane.filtered());
    pane.clearViewFilter();
    EXPECT_FALSE(pane.filtered());
}

TEST(LogPaneTest, LogFilterParseAndMatch) {
    auto f = LogFilter::parse("tag=foo");
    EXPECT_TRUE(f.active());
    LogEntry e;
    e.text = "I foo: msg"; e.tag = "foo";
    EXPECT_TRUE(f.matches(e));
    auto notf = LogFilter::parse("-tag=foo");
    EXPECT_FALSE(notf.matches(e));
}

TEST(LogPaneTest, ViewFilterContains) {
    ASSERT_TRUE(Ctx::get().ok);
    W w;
    LogPane pane(4);
    pane.addLine("alpha marker");
    pane.addLine("beta other");
    LogFilter f;
    f.contains = "marker";
    pane.setViewFilter(f);
    pane.render(w);
    EXPECT_NE(rowText(w, 0).find("alpha"), std::string::npos);
    pane.clearViewFilter();
}

TEST(LogPaneTest, KnownTagsCollect) {
    ASSERT_TRUE(Ctx::get().ok);
    LogPane pane(4);
    pane.addLine("x", P::LogLevel::Info, "mod1");
    pane.addLine("y", P::LogLevel::Info, "mod2");
    auto tags = pane.knownTags();
    EXPECT_GE(tags.size(), 2u);
}

TEST(LogPaneTest, ViewLevelAndTagSetters) {
    ASSERT_TRUE(Ctx::get().ok);
    W w;
    LogPane pane(4);
    pane.setViewLevel(P::LogLevel::Warn);
    pane.setViewTag("sub");
    pane.includeTag("inc");
    pane.render(w);
}

TEST(LogPaneTest, AttachCapturesAndReleaseRestores) {
    ASSERT_TRUE(Ctx::get().ok);
    LogPane pane(4);
    pane.attach();
    TETHER_LOGI("logpane_test", "captured line");
    pane.release();
    W w;
    pane.render(w);
    EXPECT_NE(rowText(w, 0).find("captured"), std::string::npos);
}

// ============================================================================
// TablePanel — UI::TableModel rendering
// ============================================================================

class TablePanelTest : public NcursesWidgetTest {
protected:
    UI::TableModel makeModel() {
        UI::TableModel m;
        m.setColumns({"Name", "Value"});
        std::vector<UI::Row> rows;
        UI::Row r1; r1.cells = {"alpha", "1"}; r1.tag = 1;
        UI::Row r2; r2.cells = {"beta", "2"}; r2.tag = 2;
        rows = {r1, r2};
        m.setRows(std::move(rows));
        return m;
    }
};

TEST_F(TablePanelTest, RenderDrawsRows) {
    W w;
    auto m = makeModel();
    werase(stdscr);
    int used = TablePanel::render(m, 0, 10, 0, 60);
    EXPECT_GT(used, 0);
    // header row 0, data rows 1+
    EXPECT_NE(rowText(stdscr, 0).find("Name"), std::string::npos);
    EXPECT_NE(rowText(stdscr, 1).find("alpha"), std::string::npos);
    EXPECT_NE(rowText(stdscr, 2).find("beta"), std::string::npos);
}

TEST_F(TablePanelTest, SelectionHighlightsRow) {
    W w;
    auto m = makeModel();
    werase(stdscr);
    TablePanel::render(m, 0, 10, 0, 60);
    // selected row (row 1) should carry reverse/highlight attr somewhere
    bool found = false;
    for (int x = 0; x < 30; ++x) {
        short p = 0; attr_t a = 0;
        cellAttrs(stdscr, 1, x, p, a);
        if (a & (A_REVERSE | A_STANDOUT | A_BOLD)) found = true;
    }
    EXPECT_TRUE(found);
}

TEST_F(TablePanelTest, EmptyModelRendersHeaderOnly) {
    W w;
    UI::TableModel m;
    m.setColumns({"A", "B"});
    werase(stdscr);
    int used = TablePanel::render(m, 0, 10, 0, 60);
    EXPECT_GE(used, 0);
}

// ============================================================================
// OverlayHost — renders top overlay, routes keys
// ============================================================================

TEST_F(NcursesWidgetTest, OverlayHostRendersTextOverlay) {
    UI::OverlayStack stack;
    stack.push(std::make_unique<UI::TextOverlay>(
        "TITLE", std::vector<std::string>{"line one", "line two"}));
    OverlayHost host(stack);
    werase(stdscr);
    EXPECT_TRUE(host.render(0, 20));
    const auto screen = rowText(stdscr, 0) + rowText(stdscr, 1) +
                        rowText(stdscr, 2) + rowText(stdscr, 3) +
                        rowText(stdscr, 4) + rowText(stdscr, 5) +
                        rowText(stdscr, 6) + rowText(stdscr, 7);
    EXPECT_NE(screen.find("line one"), std::string::npos);
}

TEST_F(NcursesWidgetTest, OverlayHostEmptyStackDrawsNothing) {
    UI::OverlayStack stack;
    OverlayHost host(stack);
    werase(stdscr);
    EXPECT_FALSE(host.render(0, 20));
}

TEST_F(NcursesWidgetTest, OverlayHostEscPops) {
    UI::OverlayStack stack;
    stack.push(std::make_unique<UI::TextOverlay>("t", std::vector<std::string>{"x"}));
    OverlayHost host(stack);
    EXPECT_TRUE(host.handleKey(27));   // Esc
    EXPECT_TRUE(stack.empty());
}

TEST_F(NcursesWidgetTest, OverlayHostRoutesKeyToOverlay) {
    UI::OverlayStack stack;
    auto ov = std::make_unique<UI::TextOverlay>("t", std::vector<std::string>{"x"});
    bool got = false;
    ov->onKey = [&got](int) { got = true; return true; };
    stack.push(std::move(ov));
    OverlayHost host(stack);
    host.handleKey('k');
    EXPECT_TRUE(got);
}

// ============================================================================
// EntityBrowserPanel — entity list + register read flow
// ============================================================================

struct FakeProvider : UI::ISlaveEntityProvider {
    std::vector<UI::SlaveEntity> entities() override {
        return {{0, "EK1100", "OP", 0, false, false, ""},
                {1, "EL2008", "PRE-OP", 0, false, false, ""}};
    }
};

struct FakeReader : UI::IRegisterReader {
    UI::RegisterResult read(uint16_t idx, const UI::NamedRegister& r) override {
        UI::RegisterResult res;
        res.ok = true;
        res.value = "0x" + std::to_string(idx * 100 + r.index);
        return res;
    }
};

class EntityBrowserPanelTest : public NcursesWidgetTest {
protected:
    FakeProvider provider;
    FakeReader reader;
    UI::EntityBrowser browser;
    void SetUp() override {
        NcursesWidgetTest::SetUp();
        browser.setProvider(&provider);
        browser.setRegisterReader(&reader);
        browser.setRegisters(UI::standardRegisters());
        browser.refresh();
    }
};

TEST_F(EntityBrowserPanelTest, RenderDrawsEntities) {
    EntityBrowserPanel panel(browser);
    werase(stdscr);
    panel.render(0, 20);
    bool found = false;
    for (int y = 0; y < 20; ++y)
        if (rowText(stdscr, y).find("EK1100") != std::string::npos) found = true;
    EXPECT_TRUE(found);
}

TEST_F(EntityBrowserPanelTest, EnterReadsRegisters) {
    EntityBrowserPanel panel(browser);
    werase(stdscr);
    panel.render(0, 20);
    EXPECT_TRUE(panel.handleKey('\n'));  // Enter → read registers
    panel.render(0, 20);
}

TEST_F(EntityBrowserPanelTest, ArrowKeysNavigate) {
    EntityBrowserPanel panel(browser);
    werase(stdscr);
    panel.render(0, 20);
    panel.handleKey(KEY_DOWN);
    panel.handleKey(KEY_UP);
    panel.handleKey('j');
    panel.handleKey('k');
    panel.render(0, 20);
}

TEST_F(EntityBrowserPanelTest, UnhandledKeyReturnsFalse) {
    EntityBrowserPanel panel(browser);
    EXPECT_FALSE(panel.handleKey(99999));
}

} // namespace
