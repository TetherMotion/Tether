/**
 * @file test_ui_models.cpp
 * @brief Coverage for the toolkit-agnostic UI model layer
 *        (include/tether/ui/*): KeyMap, Dialog, Overlay models +
 *        OverlayStack, ChipPanels, EntityAdapters, TableModel +
 *        RecordFilter, EntityBrowser, LineInput, Sparkline.
 *        No ncurses needed — these are plain C++ models.
 */

#include <gtest/gtest.h>

#include <string>
#include <vector>

#include "tether/ui/KeyMap.hpp"
#include "tether/ui/Dialog.hpp"
#include "tether/ui/Overlay.hpp"
#include "tether/ui/ChipPanels.hpp"
#include "tether/ui/EntityAdapters.hpp"
#include "tether/ui/TableModel.hpp"
#include "tether/ui/Panels.hpp"
#include "tether/ui/LineInput.hpp"
#include "tether/ui/Sparkline.hpp"

using namespace Tether::UI;

namespace {

// ============================================================================
// KeyMap — dispatch, footer hints, help lines
// ============================================================================

TEST(KeyMapTest, DispatchInvokesFirstMatch) {
    KeyMap km;
    int fired = 0;
    km.add('a', "a: act", "do a", [&] { fired = 'a'; return true; });
    EXPECT_TRUE(km.handleKey('a'));
    EXPECT_EQ(fired, 'a');
    EXPECT_FALSE(km.handleKey('b'));        // no binding → unhandled
}

TEST(KeyMapTest, MultiKeyBindingAndFirstMatchWins) {
    KeyMap km;
    int fired = 0;
    km.add(KeyMap::Binding{{'x', 265}, "x: nav", "navigate",
                           [&] { return false; }});
    km.add('x', "", "second", [&] { fired = 2; return true; });
    // The FIRST binding listing the key wins — its invoke result is the
    // return value; a false means "not consumed" without trying later
    // bindings.
    EXPECT_FALSE(km.handleKey('x'));
    EXPECT_EQ(fired, 0);                    // second binding never reached
    EXPECT_FALSE(km.handleKey(265));        // same binding → same false
}

TEST(KeyMapTest, NullInvokeConsumes) {
    KeyMap km;
    km.add('z', "z", "", nullptr);          // no invoke → consumes
    EXPECT_TRUE(km.handleKey('z'));
}

TEST(KeyMapTest, KeyHintsJoinNonEmpty) {
    KeyMap km;
    km.add('a', "a: one", "", [] { return true; });
    km.add('b', "", "", [] { return true; });   // hint-less → skipped
    km.add('c', "c: two", "", [] { return true; });
    EXPECT_EQ(km.keyHints(), "a: one c: two");
}

TEST(KeyMapTest, HelpLinesUseHintOrKeyName) {
    KeyMap km;
    km.add('a', "a-hint", "does a", [] { return true; });
    km.add('b', "", "", [] { return true; });   // no hint, no desc → keyName
    auto lines = km.helpLines();
    ASSERT_EQ(lines.size(), 2u);
    EXPECT_NE(lines[0].find("a-hint"), std::string::npos);
    EXPECT_NE(lines[0].find("does a"), std::string::npos);
    EXPECT_FALSE(lines[1].empty());             // falls back to key name
}

// ============================================================================
// Dialog — focus navigation, buttons, hotkeys, Esc/cancel
// ============================================================================

TEST(DialogTest, EnterFiresFocusedButton) {
    int result = -1;
    Dialog d("t", {"body"},
             {{"Yes", kDialogYes, 'y'}, {"No", kDialogNo, 'n'}},
             [&](int id) { result = id; }, 0);
    EXPECT_EQ(d.focusedButton(), 0u);
    EXPECT_TRUE(d.handleKey('\n'));
    EXPECT_EQ(result, kDialogYes);          // Enter fires focused [Yes]
    EXPECT_TRUE(d.wantsClose());
}

TEST(DialogTest, ArrowsAndHLMoveFocus) {
    Dialog d("t", {"b"},
             {{"A", 0, 0}, {"B", 1, 0}, {"C", 2, 0}}, {}, 0);
    d.handleKey(261);                       // KEY_RIGHT → next
    EXPECT_EQ(d.focusedButton(), 1u);
    d.handleKey('\t');                      // Tab also advances
    EXPECT_EQ(d.focusedButton(), 2u);
    d.handleKey('l');                       // wraps to 0
    EXPECT_EQ(d.focusedButton(), 0u);
    d.handleKey('h');                       // wraps back to last
    EXPECT_EQ(d.focusedButton(), 2u);
    d.handleKey(260);                       // KEY_LEFT
    EXPECT_EQ(d.focusedButton(), 1u);
}

TEST(DialogTest, HotkeyIsCaseInsensitive) {
    int result = -1;
    Dialog d("t", {"b"}, {{"Yes", kDialogYes, 'y'}}, [&](int id) { result = id; },
             0);
    d.handleKey('Y');                       // uppercase matches 'y' hotkey
    EXPECT_EQ(result, kDialogYes);
}

TEST(DialogTest, EscFiresCancelIndex) {
    int result = -1;
    // cancel_index = last button
    Dialog d("t", {"b"},
             {{"Yes", kDialogYes, 'y'}, {"No", kDialogNo, 'n'}},
             [&](int id) { result = id; }, 1);
    d.handleKey(27);
    EXPECT_EQ(result, kDialogNo);           // Esc fired [No]
    EXPECT_TRUE(d.wantsClose());
}

TEST(DialogTest, EscFiresCoercedCancelIndex) {
    int result = -99;
    // cancel_index=-1 with buttons coerces to "last button" → Esc fires it.
    Dialog d("t", {"b"}, {{"OK", kDialogOk, 0}}, [&](int id) { result = id; },
             -1);
    EXPECT_TRUE(d.handleKey(27));
    EXPECT_EQ(result, kDialogOk);           // Esc fired the (only) button
    EXPECT_TRUE(d.wantsClose());
}

TEST(DialogTest, EscDismissesWithoutButtons) {
    Dialog d("t", {"b"}, {}, {}, -1);       // no buttons → plain dismiss
    EXPECT_TRUE(d.handleKey(27));
    EXPECT_TRUE(d.wantsClose());
}

TEST(DialogTest, FiresOnlyOnce) {
    int calls = 0;
    Dialog d("t", {"b"}, {{"OK", kDialogOk, 0}}, [&](int) { ++calls; }, 0);
    d.handleKey('\n');
    d.handleKey('\n');
    EXPECT_EQ(calls, 1);                    // dismissed guard
}

TEST(DialogTest, ModalSwallowsUnknownKeys) {
    Dialog d("t", {"b"}, {{"OK", kDialogOk, 0}}, {}, 0);
    EXPECT_TRUE(d.handleKey(999));          // still consumed
    EXPECT_FALSE(d.wantsClose());
}

TEST(DialogTest, MessageBoxFactory) {
    int closed = 0;
    auto d = makeMessageBox("Title", "line1\nline2", [&] { ++closed; });
    ASSERT_NE(d, nullptr);
    EXPECT_EQ(d->title(), "Title");
    auto btns = d->buttons();
    ASSERT_EQ(btns.size(), 1u);
    EXPECT_EQ(btns[0].id, kDialogOk);
    EXPECT_EQ(btns[0].hotkey, 'o');
    EXPECT_EQ(d->lines().size(), 2u);       // '\n' split
    d->handleKey('\n');
    EXPECT_EQ(closed, 1);                   // onClose fired on dismissal
}

TEST(DialogTest, ConfirmDialogFactory) {
    int yes = 0, no = 0;
    auto d = makeConfirmDialog("Sure?", "really?",
                               [&] { ++yes; }, [&] { ++no; });
    auto btns = d->buttons();
    ASSERT_EQ(btns.size(), 2u);
    EXPECT_EQ(btns[0].id, kDialogYes);
    EXPECT_EQ(btns[1].id, kDialogNo);
    d->handleKey('n');                      // [No] hotkey
    EXPECT_EQ(no, 1);
    EXPECT_EQ(yes, 0);
}

TEST(DialogTest, KeyHintsPresent) {
    Dialog d("t", {"b"}, {{"OK", 0, 0}}, {}, 0);
    EXPECT_NE(d.keyHints().find("Enter"), std::string::npos);
}

// ============================================================================
// Overlay models + OverlayStack
// ============================================================================

TEST(OverlayModelTest, TextOverlayExposesLinesAndOnKey) {
    bool key_hit = false;
    TextOverlay o("T", {"l1", "l2"});
    o.onKey = [&](int k) { key_hit = (k == 'x'); return key_hit; };
    EXPECT_EQ(o.title(), "T");
    ASSERT_EQ(o.lines().size(), 2u);
    EXPECT_EQ(o.lines()[0], "l1");
    EXPECT_TRUE(o.handleKey('x'));
    EXPECT_TRUE(key_hit);
    EXPECT_FALSE(o.handleKey('y'));         // onKey rejects → false
}

TEST(OverlayModelTest, TableOverlayExposesModel) {
    TableModel tm;
    tm.setColumns({"c1"});
    tm.setRows({Row{{"v1"}, {}, 7, 0}});
    TableOverlay o("TT", std::move(tm));
    ASSERT_NE(o.table(), nullptr);
    EXPECT_EQ(o.table()->rowCount(), 1u);
    EXPECT_TRUE(o.handleKey('j'));          // nav keys route into the model
    EXPECT_TRUE(o.handleKey('k'));
    EXPECT_FALSE(o.handleKey('z'));         // unknown → false
    EXPECT_FALSE(o.keyHints().empty());
}

TEST(OverlayModelTest, MenuOverlayNavAndInvoke) {
    int fired = -1;
    MenuOverlay m("Menu");
    m.addItem({"first", 10, [&] { fired = 10; }});
    m.addItem({"second", 20, {}});
    m.handleKey('j');                       // move to row 1
    m.handleKey('k');                       // back to row 0
    m.handleKey('\n');                      // fire selected (first)
    EXPECT_EQ(fired, 10);                   // item invoke ran
    EXPECT_TRUE(m.wantsClose());
}

TEST(OverlayModelTest, MenuOverlayFallsBackToOnResult) {
    int result = -1;
    MenuOverlay m("M", [&](int id) { result = id; });
    m.addItem({"only", 42, {}});            // no invoke → onResult(id)
    m.handleKey('\n');
    EXPECT_EQ(result, 42);
    EXPECT_TRUE(m.wantsClose());
}

TEST(OverlayModelTest, MenuOverlayGAndgHomeEnd) {
    MenuOverlay m("M");
    m.addItem({"a", 1, {}});
    m.addItem({"b", 2, {}});
    m.addItem({"c", 3, {}});
    m.handleKey('G');                       // last
    EXPECT_EQ(m.table()->selectedIndex(), 2u);
    m.handleKey('g');                       // first
    EXPECT_EQ(m.table()->selectedIndex(), 0u);
}

TEST(OverlayModelTest, StackPushPopTopClear) {
    OverlayStack st;
    EXPECT_TRUE(st.empty());
    st.push(std::make_unique<TextOverlay>("a", std::vector<std::string>{"x"}));
    st.push(std::make_unique<TextOverlay>("b", std::vector<std::string>{"y"}));
    EXPECT_EQ(st.size(), 2u);
    ASSERT_NE(st.top(), nullptr);
    EXPECT_EQ(st.top()->title(), "b");
    st.pop();
    EXPECT_EQ(st.top()->title(), "a");
    st.clear();
    EXPECT_TRUE(st.empty());
    EXPECT_EQ(st.top(), nullptr);
}

// ============================================================================
// ChipPanels — tone-coded chip rows + stream formatting
// ============================================================================

using ::EtherCAT::Utils::BitLabel;

TEST(ChipPanelTest, LabeledRowStartsWithLabelCell) {
    auto row = labeledRow("STATE");
    ASSERT_EQ(row.cells.size(), 1u);
    EXPECT_EQ(row.cells[0].text, "STATE");
    EXPECT_EQ(row.cells[0].tone, DisplayTone::Muted);
}

TEST(ChipPanelTest, ScalarChipValueAndMissing) {
    DisplayRow row;
    appendScalarChip(row, std::string{"0x42"});
    appendScalarChip(row, std::nullopt);
    ASSERT_EQ(row.cells.size(), 2u);
    EXPECT_EQ(row.cells[0].text, "0x42");
    EXPECT_EQ(row.cells[0].tone, DisplayTone::Ok);
    EXPECT_EQ(row.cells[1].text, "n/a");
    EXPECT_EQ(row.cells[1].tone, DisplayTone::Muted);
}

TEST(ChipPanelTest, BitfieldChipsNulloptIsNa) {
    DisplayRow row;
    appendBitfieldChips(row, std::nullopt, {}, ChipSemantics::ExpectedSet);
    ASSERT_EQ(row.cells.size(), 1u);
    EXPECT_EQ(row.cells[0].text, "n/a");
    EXPECT_EQ(row.cells[0].tone, DisplayTone::Muted);
}

TEST(ChipPanelTest, BitfieldExpectedSetTones) {
    BitLabel labels[] = {
        BitLabel("Ready", 1u << 0, ::EtherCAT::Utils::ActiveWhen::AllSet),
        BitLabel("Op",    1u << 1, ::EtherCAT::Utils::ActiveWhen::AllSet),
    };
    DisplayRow row;
    appendBitfieldChips(row, uint32_t{0b101}, labels,
                      ChipSemantics::ExpectedSet);
    // "Ready" set → Ok; "Op" clear → Missing; bit2 unlabeled-set → Info.
    ASSERT_EQ(row.cells.size(), 3u);
    EXPECT_EQ(row.cells[0].tone, DisplayTone::Ok);
    EXPECT_EQ(row.cells[1].tone, DisplayTone::Missing);
    EXPECT_EQ(row.cells[2].text, "bit2");
    EXPECT_EQ(row.cells[2].tone, DisplayTone::Info);
}

TEST(ChipPanelTest, BitfieldExpectedSetZeroNoLabels) {
    DisplayRow row;
    appendBitfieldChips(row, uint32_t{0}, {}, ChipSemantics::ExpectedSet);
    ASSERT_EQ(row.cells.size(), 1u);
    EXPECT_EQ(row.cells[0].text, "none");
    EXPECT_EQ(row.cells[0].tone, DisplayTone::Missing);
}

TEST(ChipPanelTest, BitfieldExpectedClear) {
    DisplayRow zero;
    appendBitfieldChips(zero, uint32_t{0}, {}, ChipSemantics::ExpectedClear);
    ASSERT_EQ(zero.cells.size(), 1u);
    EXPECT_EQ(zero.cells[0].text, "none");
    EXPECT_EQ(zero.cells[0].tone, DisplayTone::Ok);

    BitLabel labels[] = {
        BitLabel("Err", 1u << 0, ::EtherCAT::Utils::ActiveWhen::AllSet),
    };
    DisplayRow row;
    appendBitfieldChips(row, uint32_t{0b101}, labels,
                      ChipSemantics::ExpectedClear);
    // set labeled bit → Fault; unlabeled set bit2 → Fault.
    ASSERT_EQ(row.cells.size(), 2u);
    EXPECT_EQ(row.cells[0].tone, DisplayTone::Fault);
    EXPECT_EQ(row.cells[1].text, "bit2");
    EXPECT_EQ(row.cells[1].tone, DisplayTone::Fault);
}

TEST(ChipPanelTest, BitfieldMode) {
    BitLabel labels[] = {
        BitLabel("On",  1u << 0, ::EtherCAT::Utils::ActiveWhen::AllSet),
        BitLabel("Off", 1u << 1, ::EtherCAT::Utils::ActiveWhen::AllSet),
    };
    DisplayRow row;
    appendBitfieldChips(row, uint32_t{1}, labels, ChipSemantics::Mode);
    ASSERT_EQ(row.cells.size(), 2u);
    EXPECT_EQ(row.cells[0].tone, DisplayTone::Ok);      // active
    EXPECT_EQ(row.cells[1].tone, DisplayTone::Muted);   // inactive
}

TEST(ChipPanelTest, FormatChipsPlainAndAnsi) {
    std::vector<DisplayCell> cells{
        {"a", DisplayTone::Ok}, {"b", DisplayTone::Fault}};
    EXPECT_EQ(formatChips(cells), "[a b]");
    EXPECT_EQ(formatChips(cells, false), "a b");
    const std::string ansi = formatChips(cells, true, true);
    EXPECT_NE(ansi.find("\x1b[32m"), std::string::npos);   // Ok → green
    EXPECT_NE(ansi.find("\x1b[1;31m"), std::string::npos); // Fault
}

TEST(ChipPanelTest, ToneToAnsiCoversAll) {
    EXPECT_STREQ(toneToAnsi(DisplayTone::Ok), "32");
    EXPECT_STREQ(toneToAnsi(DisplayTone::Missing), "31");
    EXPECT_STREQ(toneToAnsi(DisplayTone::Fault), "1;31");
    EXPECT_STREQ(toneToAnsi(DisplayTone::Warn), "33");
    EXPECT_STREQ(toneToAnsi(DisplayTone::Info), "36");
    EXPECT_STREQ(toneToAnsi(DisplayTone::Muted), "90");
}

TEST(ChipPanelTest, BitIsLabeledChecksMasks) {
    BitLabel labels[] = {
        BitLabel("ab", 0b11, ::EtherCAT::Utils::ActiveWhen::AllSet)};
    EXPECT_TRUE(bitIsLabeled(labels, 0));
    EXPECT_TRUE(bitIsLabeled(labels, 1));
    EXPECT_FALSE(bitIsLabeled(labels, 2));
}

// ============================================================================
// TableModel — selection, scrolling, filtering, quicksearch
// ============================================================================

static Row row(std::string text, int tag) {
    return Row{{std::move(text)}, {}, tag, RowNone};
}

TEST(TableModelTest, ColumnsRowsAndSelection) {
    TableModel m;
    m.setColumns({"a", "b"});
    EXPECT_EQ(m.columns().size(), 2u);
    m.setRows({row("r0", 0), row("r1", 1), row("r2", 2)});
    EXPECT_EQ(m.rowCount(), 3u);
    EXPECT_EQ(m.visibleCount(), 3u);
    EXPECT_EQ(m.selectedIndex(), 0u);
    m.move(1);
    EXPECT_EQ(m.selectedIndex(), 1u);
    m.selectLast();
    EXPECT_EQ(m.selectedIndex(), 2u);
    m.selectFirst();
    EXPECT_EQ(m.selectedIndex(), 0u);
    m.selectByTag(2);
    EXPECT_EQ(m.selectedIndex(), 2u);
    EXPECT_EQ(m.selectedTag(), 2);
}

TEST(TableModelTest, SelectionSurvivesSetRowsByTag) {
    TableModel m;
    m.setRows({row("a", 10), row("b", 20), row("c", 30)});
    m.selectByTag(20);
    m.setRows({row("x", 10), row("y", 20)});      // tag 20 still exists
    EXPECT_EQ(m.selectedTag(), 20);               // re-anchored by tag
    m.setRows({row("z", 99)});                    // tag 20 gone → clamp
    EXPECT_EQ(m.selectedTag(), 99);
}

TEST(TableModelTest, MoveClampsAndPageMove) {
    TableModel m;
    m.setRows({row("a", 0), row("b", 1), row("c", 2)});
    m.move(-5);
    EXPECT_EQ(m.selectedIndex(), 0u);
    m.pageMove(1, 10);                            // +10 → clamped
    EXPECT_EQ(m.selectedIndex(), 2u);
    m.pageMove(-1, 10);
    EXPECT_EQ(m.selectedIndex(), 0u);
}

TEST(TableModelTest, EmptyModelHasNullSelection) {
    TableModel m;
    EXPECT_EQ(m.selected(), nullptr);
    EXPECT_EQ(m.selectedTag(), -1);
    m.move(1);                                    // no crash on empty
    EXPECT_EQ(m.selectedIndex(), 0u);
}

TEST(TableModelTest, EnsureVisibleScrollsWindow) {
    TableModel m;
    std::vector<Row> rows;
    for (int i = 0; i < 20; ++i) rows.push_back(row(std::to_string(i), i));
    m.setRows(std::move(rows));
    m.selectByTag(15);
    m.ensureVisible(5);                           // height 5 → scroll to fit
    EXPECT_GE(m.scrollOffset(), 11u);             // sel 15 visible in window
    m.selectFirst();
    m.ensureVisible(5);
    EXPECT_EQ(m.scrollOffset(), 0u);
}

TEST(TableModelTest, QuicksearchNarrowsVisible) {
    TableModel m;
    m.setRows({row("alpha", 1), row("beta", 2), row("alphabet", 3)});
    m.setSearch("ALPHA");                         // case-insensitive
    EXPECT_EQ(m.visibleCount(), 2u);
    EXPECT_EQ(m.visibleRow(0)->cells[0], "alpha");
    EXPECT_EQ(m.visibleRow(1)->cells[0], "alphabet");
    EXPECT_TRUE(m.hasFilter());
    m.setSearch("");
    EXPECT_EQ(m.visibleCount(), 3u);
}

TEST(TableModelTest, FilterExprFieldAndSubstring) {
    TableModel m;
    Row a = row("alpha", 1); a.fields.fields["state"] = "OP";
    Row b = row("beta", 2);  b.fields.fields["state"] = "PREOP";
    m.setRows({a, b});
    EXPECT_TRUE(m.setFilterExpr("state=OP"));     // containsCi on field
    EXPECT_EQ(m.visibleCount(), 2u);              // OP ⊂ PREOP (substring!)
    EXPECT_TRUE(m.setFilterExpr("beta"));
    EXPECT_EQ(m.visibleCount(), 1u);
    EXPECT_TRUE(m.setFilterExpr("state=PREOP !beta"));
    EXPECT_EQ(m.visibleCount(), 0u);
    m.clearFilter();
    EXPECT_EQ(m.visibleCount(), 2u);
    EXPECT_FALSE(m.hasFilter());
}

// ============================================================================
// RecordFilter — expression grammar
// ============================================================================

TEST(RecordFilterTest, FieldEqualsSubstringMatch) {
    auto f = RecordFilter::parse("state=OP");
    FieldSet fs{{{"state", "PREOP"}}};
    EXPECT_TRUE(f.matches(fs, "anything"));       // "OP" ⊂ "PREOP" ci
    EXPECT_TRUE(f.matches(fs, ""));
    FieldSet other{{{"state", "INIT"}}};
    EXPECT_FALSE(f.matches(other, "INIT"));
}

TEST(RecordFilterTest, NegatedFieldExcludes) {
    auto f = RecordFilter::parse("!state=OP");
    FieldSet a{{{"state", "OP"}}}, b{{{"state", "INIT"}}};
    EXPECT_FALSE(f.matches(a, ""));
    EXPECT_TRUE(f.matches(b, ""));
}

TEST(RecordFilterTest, NumericCompares) {
    auto gt = RecordFilter::parse("idx>5");
    FieldSet fs{{{"idx", "9"}}};
    EXPECT_TRUE(gt.matches(fs, ""));
    FieldSet low{{{"idx", "3"}}};
    EXPECT_FALSE(gt.matches(low, ""));
    auto lt = RecordFilter::parse("idx<5");
    EXPECT_TRUE(lt.matches(low, ""));
    // Hex field values parse too ("0x" prefix skipped).
    auto hx = RecordFilter::parse("code>0x10");
    FieldSet hex{{{"code", "0x20"}}};
    EXPECT_TRUE(hx.matches(hex, ""));
    // Non-numeric value → term just doesn't hit (field present).
    auto nn = RecordFilter::parse("code>5");
    FieldSet text{{{"code", "abc"}}};
    EXPECT_FALSE(nn.matches(text, ""));
}

TEST(RecordFilterTest, BareWordsMatchTextOrFields) {
    auto f = RecordFilter::parse("alpha");
    FieldSet fs{{{"name", "alphaville"}}};
    EXPECT_TRUE(f.matches(fs, "zzz"));            // hit via field value
    EXPECT_TRUE(f.matches({}, "the alpha row"));  // hit via haystack
    EXPECT_FALSE(f.matches({}, "nothing"));
}

TEST(RecordFilterTest, NegatedBareWordExcludes) {
    auto f = RecordFilter::parse("!spam");
    EXPECT_FALSE(f.matches({}, "this is spam"));
    EXPECT_TRUE(f.matches({}, "clean"));
}

TEST(RecordFilterTest, TextAliasesAndDescribe) {
    auto f1 = RecordFilter::parse("text=foo");
    auto f2 = RecordFilter::parse("grep=foo");
    auto f3 = RecordFilter::parse("contains=foo");
    EXPECT_TRUE(f1.matches({}, "xx foo yy"));
    EXPECT_TRUE(f2.matches({}, "xx foo yy"));
    EXPECT_TRUE(f3.matches({}, "xx foo yy"));
    auto d = RecordFilter::parse("state=OP !tag=x word").describe();
    EXPECT_NE(d.find("state=OP"), std::string::npos);
    EXPECT_NE(d.find("!tag=x"), std::string::npos);
    EXPECT_NE(d.find("word"), std::string::npos);
}

TEST(RecordFilterTest, EmptyIsPassiveAndClears) {
    RecordFilter f;
    EXPECT_FALSE(f.active());
    EXPECT_TRUE(f.matches({}, "anything"));
    auto parsed = RecordFilter::parse("x=1");
    EXPECT_TRUE(parsed.active());
    parsed.clear();
    EXPECT_FALSE(parsed.active());
    EXPECT_TRUE(parsed.describe().empty());
}

// ============================================================================
// EntityAdapters — composite provider + dispatching reader
// ============================================================================

struct FakeProvider : ISlaveEntityProvider {
    std::vector<SlaveEntity> list;
    std::vector<SlaveEntity> entities() override { return list; }
};

struct FakeReader : IRegisterReader {
    RegisterResult read(uint16_t, const NamedRegister&) override {
        return {true, "0xAB", {}};
    }
};

TEST(EntityAdaptersTest, CompositeAppendsExtraEntities) {
    FakeProvider inner;
    inner.list = {SlaveEntity{.index = 1, .name = "real"}};
    CompositeEntityProvider comp(inner, [] {
        return std::vector<SlaveEntity>{
            SlaveEntity{.index = 100, .name = "virt"}};
    });
    auto all = comp.entities();
    ASSERT_EQ(all.size(), 2u);
    EXPECT_EQ(all[0].index, 1);
    EXPECT_EQ(all[1].index, 100);
}

TEST(EntityAdaptersTest, CompositeWithNullExtra) {
    FakeProvider inner;
    inner.list = {SlaveEntity{.index = 1}};
    CompositeEntityProvider comp(inner, nullptr);
    EXPECT_EQ(comp.entities().size(), 1u);
}

TEST(EntityAdaptersTest, DispatchingReaderRoutesByPredicate) {
    FakeReader inner;
    DispatchingRegisterReader rd(
        inner, [](uint16_t idx) { return idx >= 100; },
        [](uint16_t idx, const NamedRegister&) {
            return RegisterResult{true, "virt:" + std::to_string(idx), {}};
        });
    NamedRegister reg{.index = 0x1000, .subindex = 1, .name = "r"};
    auto v = rd.read(150, reg);                   // virtual → handler
    EXPECT_TRUE(v.ok);
    EXPECT_NE(v.value.find("virt"), std::string::npos);
    auto r = rd.read(1, reg);                     // real → inner
    EXPECT_TRUE(r.ok);
    EXPECT_EQ(r.value, "0xAB");
}

TEST(EntityAdaptersTest, DispatchingReaderMissingHandler) {
    FakeReader inner;
    DispatchingRegisterReader rd(
        inner, [](uint16_t) { return true; }, nullptr);
    auto r = rd.read(1, {.index = 1});
    EXPECT_FALSE(r.ok);                           // "no handler"
}

// ============================================================================
// EntityBrowser — master/detail model
// ============================================================================

TEST(EntityBrowserTest, RefreshPopulatesEntityTable) {
    FakeProvider p;
    p.list = {
        {.index = 0, .name = "EK1100", .state = "OP"},
        {.index = 1, .name = "EL2008", .state = "SAFE-OP", .error = true},
    };
    EntityBrowser b;
    b.setProvider(&p);
    b.refresh();
    EXPECT_EQ(b.entities().rowCount(), 2u);
    const Row* r0 = b.entities().visibleRow(0);
    ASSERT_NE(r0, nullptr);
    EXPECT_EQ(r0->cells[1], "EK1100");
    EXPECT_EQ(r0->color, RowOk);                  // OP → green
    const Row* r1 = b.entities().visibleRow(1);
    EXPECT_EQ(r1->color, RowErr);                 // error → red
    EXPECT_EQ(r1->cells[4], "ERR");
}

TEST(EntityBrowserTest, SelectedEntityFollowsTag) {
    FakeProvider p;
    p.list = {{.index = 5, .name = "a"}, {.index = 7, .name = "b"}};
    EntityBrowser b;
    b.setProvider(&p);
    b.refresh();
    ASSERT_NE(b.selectedEntity(), nullptr);
    EXPECT_EQ(b.selectedEntity()->index, 5);
    b.entities().selectByTag(7);
    EXPECT_EQ(b.selectedEntity()->index, 7);
}

TEST(EntityBrowserTest, RefreshRegistersViaReader) {
    FakeProvider p;
    p.list = {{.index = 3, .name = "drv"}};
    FakeReader rd;
    EntityBrowser b;
    b.setProvider(&p);
    b.setRegisterReader(&rd);
    b.setRegisters({NamedRegister{.index = 0x1000, .subindex = 1,
                                  .name = "status"}});
    b.refresh();
    b.refreshRegisters();
    EXPECT_EQ(b.registers().rowCount(), 1u);
    const Row* r = b.registers().visibleRow(0);
    ASSERT_NE(r, nullptr);
    EXPECT_NE(r->cells[0].find("0x1000"), std::string::npos);
    EXPECT_EQ(r->cells[2], "0xAB");               // FakeReader's value
    EXPECT_NE(b.registerStatus().find("1 register"), std::string::npos);
    EXPECT_EQ(b.registerSlave(), 3);
}

TEST(EntityBrowserTest, RegisterStatusEdgeCases) {
    EntityBrowser b;
    b.refreshRegisters();                         // no provider → no entity
    EXPECT_EQ(b.registerStatus(), "no entity selected");
    FakeProvider p;
    p.list = {{.index = 1}};
    b.setProvider(&p);
    b.refresh();
    b.refreshRegisters();                         // entity but no reader
    EXPECT_EQ(b.registerStatus(), "no register reader");
}

TEST(EntityBrowserTest, StaleAndUnreadFlags) {
    FakeProvider p;
    p.list = {{.index = 1}, {.index = 2}};
    FakeReader rd;
    EntityBrowser b;
    b.setProvider(&p);
    b.setRegisterReader(&rd);
    b.setRegisters({{.index = 0x1000}});
    b.refresh();
    EXPECT_TRUE(b.registersUnread());             // never read for entity 1
    b.refreshRegisters();
    EXPECT_FALSE(b.registersUnread());
    EXPECT_FALSE(b.registersStale());
    b.entities().selectByTag(2);                  // switch entity → stale
    EXPECT_TRUE(b.registersStale());
    EXPECT_TRUE(b.registersUnread());
}

TEST(EntityBrowserTest, RegisterSelectorOverridesFixedList) {
    FakeProvider p;
    p.list = {{.index = 1}};
    FakeReader rd;
    EntityBrowser b;
    b.setProvider(&p);
    b.setRegisterReader(&rd);
    b.setRegisters({{.index = 0x1000, .name = "fixed"}});
    b.setRegisterSelector([](const SlaveEntity&) {
        return std::vector<NamedRegister>{
            {.index = 0x2000, .name = "selected"},
            {.index = 0x2001, .name = "selected2"}};
    });
    b.refresh();
    b.refreshRegisters();
    EXPECT_EQ(b.registers().rowCount(), 2u);      // selector wins
    EXPECT_EQ(b.registers().visibleRow(0)->cells[1], "selected");
}

TEST(EntityBrowserTest, FocusAndAutoReadAccessors) {
    EntityBrowser b;
    EXPECT_EQ(b.focus(), EntityBrowser::Focus::Entities);
    b.setFocus(EntityBrowser::Focus::Registers);
    EXPECT_EQ(b.focus(), EntityBrowser::Focus::Registers);
    EXPECT_FALSE(b.autoRead());                 // off by default
    b.setAutoRead(true);
    EXPECT_TRUE(b.autoRead());
}

// ============================================================================
// LineInput — single-line editing model
// ============================================================================

TEST(LineInputTest, TypeCursorAndCommit) {
    LineInput in;
    std::string committed;
    in.on_commit = [&](const std::string& s) { committed = s; };
    in.handleKey('a');
    in.handleKey('b');
    EXPECT_EQ(in.buffer(), "ab");
    EXPECT_EQ(in.cursor(), 2u);
    in.handleKey('\n');
    EXPECT_EQ(committed, "ab");
    EXPECT_TRUE(in.done());
    EXPECT_TRUE(in.committed());
    EXPECT_FALSE(in.handleKey('c'));              // done → editor closed
}

TEST(LineInputTest, CursorKeysAndMidlineEdit) {
    LineInput in("ac");
    in.handleKey(260);                            // KEY_LEFT_ → cur 1
    in.handleKey('b');                            // insert at 1 → "abc"
    EXPECT_EQ(in.buffer(), "abc");
    in.handleKey(262);                            // KEY_HOME_ → cur 0
    EXPECT_EQ(in.cursor(), 0u);
    in.handleKey(360);                            // KEY_END_ → cur end
    EXPECT_EQ(in.cursor(), 3u);
    in.handleKey(261);                            // KEY_RIGHT_ at end → no-op
    EXPECT_EQ(in.cursor(), 3u);
}

TEST(LineInputTest, BackspaceAndDelete) {
    LineInput in("abc");
    in.handleKey(263);                            // KEY_BACKSPACE_ → "ab"
    EXPECT_EQ(in.buffer(), "ab");
    in.handleKey(262);                            // home
    in.handleKey(330);                            // KEY_DC_ → del 'a' → "b"
    EXPECT_EQ(in.buffer(), "b");
    EXPECT_EQ(in.cursor(), 0u);
    in.handleKey(330);                            // del at cur 0 → ""
    EXPECT_TRUE(in.buffer().empty());
}

TEST(LineInputTest, EscCancelsAndResets) {
    LineInput in("init");
    bool cancelled = false;
    in.on_cancel = [&] { cancelled = true; };
    in.handleKey(27);
    EXPECT_TRUE(cancelled);
    EXPECT_TRUE(in.cancelled());
    in.reset("re");
    EXPECT_FALSE(in.done());
    EXPECT_EQ(in.buffer(), "re");
    EXPECT_EQ(in.cursor(), 2u);
}

TEST(LineInputTest, OnChangeFiresPerMutation) {
    LineInput in;
    std::vector<std::string> seen;
    in.on_change = [&](const std::string& s) { seen.push_back(s); };
    in.handleKey('x');
    in.handleKey('y');
    in.handleKey(263);                            // backspace
    ASSERT_EQ(seen.size(), 3u);
    EXPECT_EQ(seen[2], "x");
    in.setBuffer("prog");
    EXPECT_EQ(seen.back(), "prog");               // setBuffer notifies
    EXPECT_EQ(in.cursor(), 4u);
}

TEST(LineInputTest, ModalSwallowsEverything) {
    LineInput in;
    EXPECT_TRUE(in.handleKey(9998)); // unknown key → consumed
    EXPECT_TRUE(in.handleKey(9999));
}

// ============================================================================
// Sparkline — ring-buffer history + block rendering
// ============================================================================

TEST(SparklineTest, HistoryPushesChronologically) {
    SparklineHistory<8> h;
    h.push(1.0);
    h.push(2.0);
    h.push(3.0);
    auto s = h.samples();
    ASSERT_EQ(s.size(), 3u);
    EXPECT_DOUBLE_EQ(s[0], 1.0);
    EXPECT_DOUBLE_EQ(s[2], 3.0);
    EXPECT_DOUBLE_EQ(h.minValue(), 1.0);
    EXPECT_DOUBLE_EQ(h.maxValue(), 3.0);
}

TEST(SparklineTest, HistoryWrapsAround) {
    SparklineHistory<4> h;
    for (double i = 1; i <= 6; ++i) h.push(i);
    EXPECT_EQ(h.size(), 4u);
    auto s = h.samples();
    ASSERT_EQ(s.size(), 4u);
    EXPECT_DOUBLE_EQ(s.front(), 3.0);             // oldest kept
    EXPECT_DOUBLE_EQ(s.back(), 6.0);              // newest
    h.clear();
    EXPECT_EQ(h.size(), 0u);
    EXPECT_EQ(h.minValue(), 0);                   // empty → 0
    EXPECT_EQ(h.maxValue(), 0);
}

TEST(SparklineTest, RenderScalesToBlocks) {
    std::vector<double> samples{0.0, 0.5, 1.0};
    const std::string s = renderSparkline(samples, 5, 0.0, 1.0);
    EXPECT_EQ(s.size(), 15u);                     // 5 glyph cols × 3 bytes
    EXPECT_NE(s.find("█"), std::string::npos);    // 1.0 → top block
    EXPECT_NE(s.find("▁"), std::string::npos);    // 0.0 → bottom/pad
}

TEST(SparklineTest, RenderEdgeCases) {
    EXPECT_TRUE(renderSparkline(std::vector<double>{1.0}, 0, 0, 1).empty());
    // Empty samples still fill `width` columns with floor glyphs.
    const std::string padded = renderSparkline(std::vector<double>{}, 3, 0, 1);
    EXPECT_EQ(padded.size(), 9u);                 // 3 × 3-byte block
    const std::string flat =
        renderSparkline(std::vector<double>{5.0, 5.0}, 2, 5.0, 5.0);
    EXPECT_EQ(flat.size(), 6u);                   // lo==hi → span=1 guard
}

} // namespace
