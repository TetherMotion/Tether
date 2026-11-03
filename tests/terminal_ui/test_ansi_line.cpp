/**
 * @file test_ansi_line.cpp
 * @brief Coverage for the pure UTF-8 measuring/clipping helpers and the
 *        ANSI SGR → palette-pair mapping in AnsiLine.cpp.
 */

#include <gtest/gtest.h>

#include <string>

#include "tether/terminal_ui/AnsiLine.hpp"
#include "tether/terminal_ui/Session.hpp"

using namespace Tether::TUI;

// ============================================================================
// utf8Cols — ASCII is locale-independent
// ============================================================================

TEST(Utf8ColsTest, EmptyStringIsZeroColumns) {
    EXPECT_EQ(utf8Cols(""), 0);
}

TEST(Utf8ColsTest, AsciiCountsBytes) {
    EXPECT_EQ(utf8Cols("abc"), 3);
    EXPECT_EQ(utf8Cols("hello, world"), 12);
}

TEST(Utf8ColsTest, MultiByteIsAtLeastOneColumnPerChar) {
    // Locale-dependent: 1 column per char in UTF-8 locales, one per byte
    // in the C locale.  Either way it must be a sane positive count.
    const int cols = utf8Cols("\xC3\xA9");  // "é"
    EXPECT_GE(cols, 1);
    EXPECT_LE(cols, 2);
}

TEST(Utf8ColsTest, InvalidSequenceStillAdvances) {
    EXPECT_GT(utf8Cols("\xFF"), 0);
}

// ============================================================================
// utf8Trunc
// ============================================================================

TEST(Utf8TruncTest, NonPositiveLimitYieldsEmpty) {
    EXPECT_EQ(utf8Trunc("abc", 0), "");
    EXPECT_EQ(utf8Trunc("abc", -3), "");
}

TEST(Utf8TruncTest, KeepsPrefixWithinLimit) {
    EXPECT_EQ(utf8Trunc("abcdef", 3), "abc");
    EXPECT_EQ(utf8Trunc("abc", 10), "abc");
}

TEST(Utf8TruncTest, EmptyInput) {
    EXPECT_EQ(utf8Trunc("", 5), "");
}

TEST(Utf8TruncTest, DoesNotSplitBeyondColumnBudget) {
    // Whatever the locale reports for the multibyte char, the result must
    // be a byte-prefix of the input and no longer than it.
    const std::string s = "ab\xC3\xA9z";
    const std::string cut = utf8Trunc(s, 2);
    EXPECT_EQ(s.compare(0, cut.size(), cut), 0);
    EXPECT_LE(cut.size(), s.size());
}

// ============================================================================
// ansiFgPair — SGR colour index → palette pair
// ============================================================================

TEST(AnsiFgPairTest, MapsStandardColours) {
    EXPECT_EQ(ansiFgPair(0), PalMuted);      // black muted
    EXPECT_EQ(ansiFgPair(1), PalError);      // red
    EXPECT_EQ(ansiFgPair(2), PalValue);      // green
    EXPECT_EQ(ansiFgPair(3), PalHint);       // yellow
    EXPECT_EQ(ansiFgPair(4), PalInfo);       // blue
    EXPECT_EQ(ansiFgPair(5), PalSelected);   // magenta
    EXPECT_EQ(ansiFgPair(6), PalHeader);     // cyan
    EXPECT_EQ(ansiFgPair(7), PalMuted);      // white
}

TEST(AnsiFgPairTest, OutOfRangeFallsBackToMuted) {
    EXPECT_EQ(ansiFgPair(8), PalMuted);
    EXPECT_EQ(ansiFgPair(-1), PalMuted);
    EXPECT_EQ(ansiFgPair(99), PalMuted);
}
