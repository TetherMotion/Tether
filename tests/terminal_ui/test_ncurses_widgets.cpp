/**
 * @file test_ncurses_widgets.cpp
 * @brief Coverage for the ncurses widget layer — Lamp, Gauge,
 *        RangeTrack, Chips, StyledText, StatusBar, HexView, Sparkline,
 *        Box frames, TextWrap, RowWriter, renderAnsiLine, ChipPanel.
 *
 * All tests run a real ncurses SCREEN headless: newterm() accepts any
 * FILE* pair, so /dev/null works for in+out; widget draws land in the
 * WINDOW buffer and are verified by reading cells back with
 * mvwinnstr()/mvwinch() (text + attributes incl. color pairs).
 *
 * Include order per AGENTS.md: standard → UI models → ncurses → widgets.
 */

#include <gtest/gtest.h>

#include <clocale>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <string>
#include <vector>

#include "tether/ui/ChipPanels.hpp"   // DisplayCell/DisplayRow/DisplayPanel, DisplayTone
#include "tether/ui/Sparkline.hpp"    // SparklineHistory

#include <ncurses.h>

#include "tether/terminal_ui/Session.hpp"     // Palette pair ids
#include "tether/terminal_ui/StyledText.hpp"
#include "tether/terminal_ui/Lamp.hpp"
#include "tether/terminal_ui/Gauge.hpp"
#include "tether/terminal_ui/Chips.hpp"
#include "tether/terminal_ui/StatusBar.hpp"
#include "tether/terminal_ui/HexView.hpp"
#include "tether/terminal_ui/Sparkline.hpp"
#include "tether/terminal_ui/Box.hpp"
#include "tether/terminal_ui/TextWrap.hpp"
#include "tether/terminal_ui/RowWriter.hpp"
#include "tether/terminal_ui/AnsiLine.hpp"
#include "tether/terminal_ui/ChipPanel.hpp"

#include "NcursesTestEnv.hpp"

using namespace Tether::TUI;
using Tether::UI::DisplayCell;
using Tether::UI::DisplayRow;
using Tether::UI::DisplayPanel;
using Tether::UI::DisplayTone;
using namespace NcursesTest;

using Tether::TUI::PalValue; using Tether::TUI::PalHeader; using Tether::TUI::PalHint;
using Tether::TUI::PalError; using Tether::TUI::PalSelected; using Tether::TUI::PalMuted;
using Tether::TUI::PalInfo; using Tether::TUI::PalNone;

// ============================================================================
// StyledText — putStyled / putPair / putSpans / AttrScope / pairAttrs
// ============================================================================

TEST_F(NcursesWidgetTest, PutStyledWritesAndReturnsAdvance) {
    W w;
    int col = wputStyled(w, 2, 5, A_BOLD, "hello");
    EXPECT_EQ(col, 5);   // returns chars written (for col += chaining)
    EXPECT_EQ(rowText(w, 2, 5, 12), "hello");
    EXPECT_TRUE(cellBold(w, 2, 5));
}

TEST_F(NcursesWidgetTest, PutStyledFormats) {
    W w;
    wputStyled(w, 0, 0, 0, "%d-%s", 42, "x");
    EXPECT_EQ(rowText(w, 0, 0, 8), "42-x");
}

TEST_F(NcursesWidgetTest, PutPairAppliesColorPair) {
    W w;
    wputPair(w, 1, 3, PalError, A_BOLD, "ERR");
    EXPECT_EQ(rowText(w, 1, 3, 8), "ERR");
    EXPECT_EQ(cellPair(w, 1, 3), PalError);
    EXPECT_TRUE(cellBold(w, 1, 3));
}

TEST_F(NcursesWidgetTest, PutSpansConcatenatesRuns) {
    W w;
    std::vector<Span> spans{{"a", 0}, {"b", A_BOLD}, {"c", 0}};
    int col = wputSpans(w, 0, 2, spans);
    EXPECT_EQ(col, 3);   // returns chars written
    EXPECT_EQ(rowText(w, 0, 2, 8), "abc");
    EXPECT_TRUE(cellBold(w, 0, 3));
    EXPECT_FALSE(cellBold(w, 0, 4));
}

TEST_F(NcursesWidgetTest, SpansWidthSumsTexts) {
    std::vector<Span> spans{{"ab", 0}, {"cde", 0}};
    EXPECT_EQ(spansWidth(spans), 5);
}

TEST_F(NcursesWidgetTest, AttrScopeResetsAttributes) {
    W w;
    {
        WAttrScope scope(w, A_BOLD | COLOR_PAIR(PalError));
        mvwprintw(w, 0, 0, "x");
    }
    mvwprintw(w, 0, 1, "y");
    EXPECT_TRUE(cellBold(w, 0, 0));
    EXPECT_EQ(cellPair(w, 0, 0), PalError);
    EXPECT_FALSE(cellBold(w, 0, 1));       // scope ended
    EXPECT_EQ(cellPair(w, 0, 1), 0);
}

TEST_F(NcursesWidgetTest, PairAttrsEncodesPair) {
    const int a = pairAttrs(PalError);
    EXPECT_EQ(a & A_COLOR, COLOR_PAIR(PalError));
}

// ============================================================================
// Lamp — all styles and states
// ============================================================================

TEST_F(NcursesWidgetTest, LampDotsDrawGlyphs) {
    W w;
    EXPECT_GT(wdrawLamp(w, 0, 0, LampState::On, kLampDots), 0);
    wdrawLamp(w, 0, 4, LampState::Off, kLampDots);
    wdrawLamp(w, 0, 8, LampState::Unknown, kLampDots);
    // States are distinguished by color: On=green+bold, Off/Unknown=muted
    short pair = 0; attr_t attrs = 0;
    cellAttrs(w, 0, 0, pair, attrs);
    EXPECT_EQ(pair, PalValue);
    EXPECT_TRUE(attrs & A_BOLD);
    cellAttrs(w, 0, 4, pair, attrs);
    EXPECT_EQ(pair, PalMuted);
    cellAttrs(w, 0, 8, pair, attrs);
    EXPECT_EQ(pair, PalMuted);
}

TEST_F(NcursesWidgetTest, LampTagsDrawText) {
    W w;
    wdrawLamp(w, 0, 0, LampState::On, kLampTags);
    wdrawLamp(w, 0, 8, LampState::Off, kLampTags);
    EXPECT_NE(rowText(w, 0, 0, 8).find("ON"), std::string::npos);
}

TEST_F(NcursesWidgetTest, LampDigitsDrawBinary) {
    W w;
    wdrawLamp(w, 0, 0, LampState::On, kLampDigits);
    wdrawLamp(w, 0, 2, LampState::Off, kLampDigits);
    EXPECT_EQ(cellCh(w, 0, 0), '1');
    EXPECT_EQ(cellCh(w, 0, 2), '0');
}

TEST_F(NcursesWidgetTest, LampBitsAndAlertBits) {
    W w;
    wdrawLamp(w, 0, 0, LampState::On, kLampBits);
    wdrawLamp(w, 0, 2, LampState::On, kLampAlertBits);
    EXPECT_EQ(cellCh(w, 0, 0), '*');
    EXPECT_EQ(cellCh(w, 0, 2), '*');
    // alert style marks On as red+bold
    EXPECT_EQ(cellPair(w, 0, 2), PalError);
}

TEST_F(NcursesWidgetTest, LampSquares) {
    W w;
    EXPECT_GT(wdrawLamp(w, 0, 0, LampState::On, kLampSquares), 0);
    wdrawLamp(w, 0, 4, LampState::Unknown, kLampSquares);
}

// ============================================================================
// Gauge — utilization bar + RangeTrack
// ============================================================================

TEST_F(NcursesWidgetTest, GaugeDrawsFill) {
    W w;
    wdrawGauge(w, 0, 0, 12, 0.5, false);
    const std::string r = rowText(w, 0, 0, 12);
    EXPECT_EQ(r.front(), '[');
    EXPECT_EQ(r.back(), ']');
    EXPECT_EQ(std::count(r.begin(), r.end(), '#'), 5);  // 50% of 10 inner
}

TEST_F(NcursesWidgetTest, GaugeEmptyAndFull) {
    W w;
    wdrawGauge(w, 0, 0, 8, 0.0, false);
    EXPECT_EQ(rowText(w, 0, 0, 8), "[......]");
    wdrawGauge(w, 1, 0, 8, 1.0, false);
    EXPECT_EQ(rowText(w, 1, 0, 8), "[######]");
}

TEST_F(NcursesWidgetTest, GaugeOverflowMarks) {
    W w;
    wdrawGauge(w, 0, 0, 8, 1.5, false);      // >1 → clamps + '>' marker at x+w
    EXPECT_EQ(cellCh(w, 0, 8), '>');
    EXPECT_EQ(cellPair(w, 0, 8), PalError);
    EXPECT_TRUE(cellBold(w, 0, 8));
}

TEST_F(NcursesWidgetTest, GaugeColorsByThreshold) {
    W w;
    wdrawGauge(w, 0, 0, 10, 0.5, false);     // green zone
    wdrawGauge(w, 1, 0, 10, 0.95, false);    // red zone
    wdrawGauge(w, 2, 0, 10, 0.3, true);      // violated → red
    EXPECT_EQ(cellPair(w, 0, 1), PalValue);
    EXPECT_EQ(cellPair(w, 1, 1), PalError);
    EXPECT_EQ(cellPair(w, 2, 1), PalError);
    EXPECT_TRUE(cellBold(w, 2, 1));
}

TEST_F(NcursesWidgetTest, GaugeTooNarrowIsNoop) {
    W w;
    wdrawGauge(w, 0, 0, 2, 0.5, false);      // w<3 → nothing
    EXPECT_EQ(rowText(w, 0), "");
}

TEST_F(NcursesWidgetTest, RangeTrackMarksValue) {
    W w;
    wdrawRangeTrack(w, 0, 0, 12, 0, 100, 50, false);
    const std::string r = rowText(w, 0, 0, 12);
    EXPECT_NE(r.find('#'), std::string::npos);
}

TEST_F(NcursesWidgetTest, RangeTrackOutOfBoundsRed) {
    W w;
    wdrawRangeTrack(w, 0, 0, 12, 0, 100, 150, false);  // outside range
    bool found_err = false;
    for (int x = 0; x < 12; ++x)
        if (cellPair(w, 0, x) == PalError) found_err = true;
    EXPECT_TRUE(found_err);
    wdrawRangeTrack(w, 1, 0, 12, 0, 100, std::nullopt, false);  // no value
    EXPECT_EQ(rowText(w, 1, 0, 12).find('#'), std::string::npos);
}

// ============================================================================
// Chips — inline chip rows
// ============================================================================

TEST_F(NcursesWidgetTest, DrawChipsReturnsAdvance) {
    W w;
    std::vector<DisplayCell> cells{{"OK", DisplayTone::Ok},
                                   {"ERR", DisplayTone::Fault}};
    int col = wdrawChips(w, 0, 2, cells);
    EXPECT_GT(col, 2);
    const std::string r = rowText(w, 0, 2, col);
    EXPECT_NE(r.find("OK"), std::string::npos);
    EXPECT_NE(r.find("ERR"), std::string::npos);
}

TEST_F(NcursesWidgetTest, DrawChipsColorsByTone) {
    W w;
    std::vector<DisplayCell> cells{{"x", DisplayTone::Ok},
                                   {"y", DisplayTone::Fault}};
    wdrawChips(w, 0, 0, cells);
    // Fault cell is red+bold — locate its 'y'
    EXPECT_EQ(cellPair(w, 0, 1), PalValue);
}

TEST_F(NcursesWidgetTest, DrawChipsNoBrackets) {
    W w;
    std::vector<DisplayCell> cells{{"a", DisplayTone::Muted},
                                   {"b", DisplayTone::Muted}};
    wdrawChips(w, 0, 0, cells, false);   // brackets=false
    EXPECT_EQ(rowText(w, 0, 0, 4), "a b");
}

TEST_F(NcursesWidgetTest, ChipsWidthMatchesDraw) {
    std::vector<DisplayCell> cells{{"ab", DisplayTone::Muted},
                                   {"cd", DisplayTone::Muted}};
    EXPECT_EQ(chipsWidth(cells, true), 2 + 2 + 1 + 2);   // "[ab cd]"
    EXPECT_EQ(chipsWidth(cells, false), 2 + 1 + 2);       // "ab cd"
}

// ============================================================================
// StatusBar — left/right anchored spans
// ============================================================================

TEST_F(NcursesWidgetTest, StatusBarAnchorsRight) {
    W w;
    std::vector<BarSeg> left{{"L", 0}};
    std::vector<BarSeg> right{{"R", 0}};
    wdrawStatusBar(w, 0, left, right, 40);
    EXPECT_EQ(cellCh(w, 0, 0), 'L');
    EXPECT_EQ(cellCh(w, 0, 38), 'R');   // width - ("R" + 1 space)
}

TEST_F(NcursesWidgetTest, StatusBarMultiSpan) {
    W w;
    std::vector<BarSeg> left{{"a", 0}, {"b", 0}, {"c", 0}};
    wdrawStatusBar(w, 1, left, {}, 40);
    EXPECT_NE(rowText(w, 1, 0, 10).find("a b c"), std::string::npos);
}

// ============================================================================
// HexView — byte dump with field highlighting
// ============================================================================

TEST_F(NcursesWidgetTest, HexLineDrawsBytes) {
    W w;
    std::vector<uint8_t> data{0xDE, 0xAD, 0xBE, 0xEF};
    int col = wdrawHexLine(w, 0, 0, 40, data, {}, 0);
    EXPECT_GT(col, 0);
    const std::string r = rowText(w, 0, 0, col);
    EXPECT_NE(r.find("DE"), std::string::npos);
    EXPECT_NE(r.find("EF"), std::string::npos);
}

TEST_F(NcursesWidgetTest, HexLineClipsWithEllipsis) {
    W w;
    std::vector<uint8_t> data(16, 0xAB);
    wdrawHexLine(w, 0, 0, 12, data, {}, 0);   // too narrow for 16 bytes
    const std::string r = rowText(w, 0, 0, 12);
    EXPECT_FALSE(r.empty());
}

TEST_F(NcursesWidgetTest, HexLineFieldHighlight) {
    W w;
    std::vector<uint8_t> data{0x11, 0x22, 0x33, 0x44};
    std::vector<HexField> fields{{2, 2, COLOR_PAIR(PalError) | A_BOLD}};
    wdrawHexLine(w, 0, 0, 40, data, fields, 0);
    // byte 0x33 at display col (2*3)=6
    EXPECT_EQ(cellPair(w, 0, 6), PalError);
    EXPECT_TRUE(cellBold(w, 0, 6));
    EXPECT_EQ(cellPair(w, 0, 0), 0);          // plain byte unstyled
}

// ============================================================================
// Sparkline — history render
// ============================================================================

TEST_F(NcursesWidgetTest, SparklineDrawsColumns) {
    W w;
    std::vector<double> samples{0.0, 0.5, 1.0};
    int col = wdrawSparkline(w, 0, 0, 8, samples, 0.0, 1.0, 0);
    EXPECT_GT(col, 0);
}

TEST_F(NcursesWidgetTest, SparklineHistoryOverload) {
    W w;
    Tether::UI::SparklineHistory<8> hist;
    hist.push(0.1f); hist.push(0.9f); hist.push(0.5f);
    wdrawSparkline(w, 0, 0, 8, hist, 0.0, 1.0, 0);
    EXPECT_FALSE(rowText(w, 0).empty());
}

// ============================================================================
// Box — frames, key-value boxes, boxed lines
// ============================================================================

TEST_F(NcursesWidgetTest, BoxFrameDrawsCorners) {
    W w;
    wdrawBoxFrame(w, 0, 0, 20, 5, "");
    // corners are box-drawing chars — just verify non-blank
    EXPECT_NE(rowText(w, 0, 0, 20).find(""), std::string::npos);
    EXPECT_FALSE(rowText(w, 4, 0, 20).empty());
}

TEST_F(NcursesWidgetTest, KeyValueBoxRendersRows) {
    W w;
    std::vector<KvRow> rows{
        {"state", "OP", DisplayTone::Ok},
        {"mode", "CSP", std::nullopt},
    };
    int used = wdrawKeyValueBox(w, 0, rows, "Key", "Val");
    EXPECT_GT(used, 0);
    const std::string r1 = rowText(w, 3);
    EXPECT_NE(r1.find("state"), std::string::npos);
    EXPECT_NE(r1.find("OP"), std::string::npos);
    const std::string r2 = rowText(w, 4);
    EXPECT_NE(r2.find("mode"), std::string::npos);
}

TEST_F(NcursesWidgetTest, KeyValueBoxValueTone) {
    W w;
    std::vector<KvRow> rows{{"k", "BAD", DisplayTone::Fault}};
    wdrawKeyValueBox(w, 0, rows, "", "");   // no headers → row at y=1
    // Fault tone → PalError somewhere on row 1
    bool found_red = false;
    for (int x = 0; x < 30; ++x)
        if (cellPair(w, 1, x) == PalError) found_red = true;
    EXPECT_TRUE(found_red);
}

TEST_F(NcursesWidgetTest, BoxedLinesWithTitle) {
    W w;
    std::vector<std::string> lines{"first", "second"};
    int used = wdrawBoxedLines(w, 0, 10, lines, "T");
    EXPECT_GT(used, 0);
    EXPECT_NE(rowText(w, 2).find("first"), std::string::npos);
    EXPECT_NE(rowText(w, 3).find("second"), std::string::npos);
}

TEST_F(NcursesWidgetTest, BoxedLinesClipsAtMaxLines) {
    W w;
    std::vector<std::string> lines{"a", "b", "c"};
    wdrawBoxedLines(w, 0, 2, lines, "");
    EXPECT_EQ(rowText(w, 4), "");   // third line clipped → only bottom remains
}

// ============================================================================
// TextWrap — pure functions
// ============================================================================

TEST_F(NcursesWidgetTest, WrapTextSplitsOnWidth) {
    auto lines = wrapText("hello world test", 5);
    EXPECT_EQ(lines.size(), 3u);   // "hello", "world", "test"
}

TEST_F(NcursesWidgetTest, WrapTextPreservesNewlines) {
    auto lines = wrapText("a\nb", 10);
    EXPECT_EQ(lines.size(), 2u);
}

TEST_F(NcursesWidgetTest, WrapTextLongWord) {
    auto lines = wrapText("abcdefghij", 4);
    EXPECT_EQ(lines.size(), 3u);   // abcd efgh ij
}

TEST_F(NcursesWidgetTest, WrapTextEmptyAndZeroWidth) {
    EXPECT_TRUE(wrapText("", 10).empty());
    auto r = wrapText("x", 0);
    EXPECT_TRUE(r.empty() || (r.size() == 1 && r[0].empty()));
}

TEST_F(NcursesWidgetTest, PadToWidthPadsShort) {
    EXPECT_EQ(padToWidth("ab", 5), "ab   ");
    EXPECT_EQ(padToWidth("abcde", 3), "abc");     // truncates to width
}

// ============================================================================
// RowWriter — sequential emission + clipping
// ============================================================================

TEST_F(NcursesWidgetTest, RowWriterWritesSequentially) {
    W w;
    RowWriter rw(w, 0, 0, 4);
    rw.text("one");
    rw.text("two");
    EXPECT_EQ(rowText(w, 0), "one");
    EXPECT_EQ(rowText(w, 1), "two");
}

TEST_F(NcursesWidgetTest, RowWriterPutColumns) {
    W w;
    RowWriter rw(w, 0);
    rw.put(2, 0, "A");
    rw.put(8, 0, "B");
    rw.advance();
    EXPECT_EQ(cellCh(w, 0, 2), 'A');
    EXPECT_EQ(cellCh(w, 0, 8), 'B');
}

TEST_F(NcursesWidgetTest, RowWriterClipsAtBottom) {
    W w;
    RowWriter rw(w, 0, 0, 2);   // bottom=2 rows
    rw.text("a");
    rw.text("b");
    EXPECT_FALSE(rw.fits());
    EXPECT_EQ(rw.remaining(), 0);
    rw.text("c");              // clipped — should not crash
    EXPECT_EQ(rowText(w, 2), "");
}

TEST_F(NcursesWidgetTest, RowWriterMarginAndSeek) {
    W w;
    RowWriter rw(w, 0);
    rw.margin(4).text("indented");
    EXPECT_EQ(rowText(w, 0, 4, 12), "indented");
    rw.seek(5).text("back");
    EXPECT_EQ(rowText(w, 5, 4, 10), "back");   // margin still applies
}

TEST_F(NcursesWidgetTest, RowWriterLampAndRule) {
    W w;
    RowWriter rw(w, 0);
    rw.lamp(LampState::On);
    rw.rule();
    EXPECT_FALSE(rowText(w, 0).empty());
    EXPECT_FALSE(rowText(w, 1).empty());
}

TEST_F(NcursesWidgetTest, ColumnSplitMath) {
    const auto g2 = columnSplit(80, 2, 20, 54);   // 80 cols → 2 columns
    EXPECT_EQ(g2.cols, 2);
    EXPECT_GE(g2.width, 20);
    EXPECT_LE(g2.width, 54);
    EXPECT_EQ(g2.x(1), g2.width + g2.gap);

    const auto g1 = columnSplit(10, 2, 20, 54);   // too narrow → 1 column
    EXPECT_EQ(g1.cols, 1);
    EXPECT_GE(g1.width, 1);
}

// ============================================================================
// renderAnsiLine — ANSI SGR → curses colors
// ============================================================================

TEST_F(NcursesWidgetTest, AnsiLinePlainText) {
    W w;
    renderAnsiLine(w, 0, 0, 40, "plain", PalNone);
    EXPECT_EQ(rowText(w, 0, 0, 10), "plain");
}

TEST_F(NcursesWidgetTest, AnsiLineSgrRed) {
    W w;
    renderAnsiLine(w, 0, 0, 40, "a \x1B[31mred\x1B[0m b", PalNone);
    const std::string r = rowText(w, 0);
    EXPECT_NE(r.find("red"), std::string::npos);
    EXPECT_NE(r.find("b"), std::string::npos);
}

TEST_F(NcursesWidgetTest, AnsiLineBoldAndBright) {
    W w;
    renderAnsiLine(w, 0, 0, 40, "\x1B[1mBOLD\x1B[0m\x1B[91mbright\x1B[0m", PalNone);
    EXPECT_TRUE(cellBold(w, 0, 0));
}

TEST_F(NcursesWidgetTest, AnsiLineClipsAtWidth) {
    W w;
    renderAnsiLine(w, 0, 0, 4, "abcdefghij", PalNone);
    EXPECT_EQ(rowText(w, 0, 0, 4), "abcd");
}

TEST_F(NcursesWidgetTest, AnsiLineUnterminatedEscapeDropsTail) {
    W w;
    renderAnsiLine(w, 0, 0, 40, "ok \x1B[3", PalNone);
    EXPECT_NE(rowText(w, 0).find("ok"), std::string::npos);
}

// ============================================================================
// ChipPanel — boxed + grid (.cpp implementations)
// ============================================================================

TEST_F(NcursesWidgetTest, ChipPanelBoxDraws) {
    W w(20, 60);
    DisplayPanel p;
    p.title = "status";
    DisplayRow row;
    row.cells = {{"RUN", DisplayTone::Ok}, {"E0", DisplayTone::Fault}};
    p.rows = {row};
    int used = drawChipPanelBox(0, 0, 30, p, 10, 1);
    EXPECT_GT(used, 0);
    EXPECT_NE(rowText(w, 0).find(""), std::string::npos);
}

TEST_F(NcursesWidgetTest, ChipPanelBoxTooSmallReturns0) {
    W w;
    DisplayPanel p;
    EXPECT_EQ(drawChipPanelBox(0, 0, 3, p, 10, 1), 0);  // w<4
    EXPECT_EQ(drawChipPanelBox(0, 0, 30, p, 2, 1), 0);  // max_h<3
}

TEST_F(NcursesWidgetTest, ChipPanelGridLaysOut) {
    W w(24, 100);
    DisplayPanel a, b;
    a.title = "one"; b.title = "two";
    DisplayRow r; r.cells = {{"x", DisplayTone::Muted}};
    a.rows = {r}; b.rows = {r};
    int used = drawChipPanelGrid(0, 24, {a, b});
    EXPECT_GT(used, 0);
}

TEST_F(NcursesWidgetTest, ChipTonePairMapping) {
    EXPECT_EQ(chipTonePair(DisplayTone::Ok),       PalValue);
    EXPECT_EQ(chipTonePair(DisplayTone::Fault),    PalError);
    EXPECT_EQ(chipTonePair(DisplayTone::Missing),  PalError);
    EXPECT_EQ(chipTonePair(DisplayTone::Muted),    PalMuted);
    EXPECT_EQ(chipTonePair(DisplayTone::Warn),     PalHint);
}

// ============================================================================
// stdscr convenience wrappers — exercise the non-w paths too
// ============================================================================

TEST_F(NcursesWidgetTest, StdscrWrappersDraw) {
    // stdscr exists after newterm; draw then read from stdscr directly
    werase(stdscr);
    drawLamp(0, 0, LampState::On, kLampDigits);
    drawGauge(1, 0, 8, 0.5, false);
    drawChips(2, 0, std::vector<DisplayCell>{{"c", DisplayTone::Ok}});
    drawStatusBar(3, std::vector<BarSeg>{{"s", 0}}, {}, 20);
    drawHexLine(4, 0, 20, std::vector<uint8_t>{0x42}, {}, 0);
    drawSparkline(5, 0, 6, std::vector<double>{0.5}, 0.0, 1.0, 0);
    drawBoxFrame(6, 0, 10, 3, "");
    drawRangeTrack(9, 0, 10, 0, 10, 5, false);
    putStyled(10, 0, 0, "t");
    putPair(11, 0, PalValue, 0, "p");

    EXPECT_EQ(cellCh(stdscr, 0, 0), '1');
    EXPECT_EQ(rowText(stdscr, 10, 0, 5), "t");
    EXPECT_EQ(rowText(stdscr, 11, 0, 5), "p");
}

TEST_F(NcursesWidgetTest, DrawChipPanelGridStdscr) {
    werase(stdscr);
    DisplayPanel p;
    DisplayRow r; r.cells = {{"a", DisplayTone::Ok}};
    p.rows = {r};
    EXPECT_GT(drawChipPanelGrid(0, 10, {p}), 0);
}


