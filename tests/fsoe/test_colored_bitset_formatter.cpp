// test_colored_bitset_formatter.cpp
//
// Unit tests for the ColoredBitsetFormatter utility — verifies single-bit
// labels, multi-bit labels, zero-active/one-active encoding, the two
// output modes (format / formatActive), and custom subclassing.

#include <gtest/gtest.h>
#include <string>
#include <string_view>

#include "tether/utils/ColoredBitsetFormatter.hpp"

using EtherCAT::Utils::ActiveWhen;
using EtherCAT::Utils::BitLabel;
using EtherCAT::Utils::ColoredBitsetFormatter;
using EtherCAT::Utils::ShowWhen;

// ============================================================================
// ANSI escape sequences used by the formatter
// ============================================================================

static constexpr std::string_view kGreen = "\033[32m";
static constexpr std::string_view kRed   = "\033[31m";
static constexpr std::string_view kBold  = "\033[1m";
static constexpr std::string_view kReset = "\033[0m";

/// Strip ANSI escape codes from a string (for assertions that don't care
/// about color).
static std::string stripAnsi(std::string_view s) {
    std::string out;
    out.reserve(s.size());
    for (size_t i = 0; i < s.size(); ++i) {
        if (s[i] == '\033') {
            // Skip the escape sequence: \033 [ ... letter
            i += 2;  // skip \033 [
            while (i < s.size() && !std::isalpha(static_cast<unsigned char>(s[i])))
                ++i;
            // s[i] is the terminating letter; loop's ++i skips it
        } else {
            out += s[i];
        }
    }
    return out;
}

// ============================================================================
// BitLabel factory functions
// ============================================================================

TEST(BitLabelTest, BitFactorySetsAllSetActiveWhen) {
    const BitLabel label = BitLabel::bit("FOO", 0x0001);
    EXPECT_STREQ(label.name, "FOO");
    EXPECT_EQ(label.mask, 0x0001u);
    EXPECT_EQ(label.active_when, ActiveWhen::AllSet);
}

TEST(BitLabelTest, BitInvFactorySetsAllClearActiveWhen) {
    const BitLabel label = BitLabel::bitInv("FOO", 0x0001);
    EXPECT_EQ(label.active_when, ActiveWhen::AllClear);
}

TEST(BitLabelTest, BitsFactorySetsAllSetActiveWhen) {
    const BitLabel label = BitLabel::bits("GROUP", 0x000F);
    EXPECT_EQ(label.active_when, ActiveWhen::AllSet);
}

TEST(BitLabelTest, BitsInvFactorySetsAllClearActiveWhen) {
    const BitLabel label = BitLabel::bitsInv("GROUP", 0x000F);
    EXPECT_EQ(label.active_when, ActiveWhen::AllClear);
}

TEST(BitLabelTest, AnyBitFactorySetsAnySetActiveWhen) {
    const BitLabel label = BitLabel::anyBit("ANY", 0x000F);
    EXPECT_EQ(label.active_when, ActiveWhen::AnySet);
}

TEST(BitLabelTest, AnyBitInvFactorySetsAnyClearActiveWhen) {
    const BitLabel label = BitLabel::anyBitInv("ANY", 0x000F);
    EXPECT_EQ(label.active_when, ActiveWhen::AnyClear);
}

// ============================================================================
// BitLabel::isActive (virtual, default implementation)
// ============================================================================

TEST(IsActiveTest, SingleBitAllSet) {
    const auto label = BitLabel::bit("X", 0x0004);
    EXPECT_TRUE(label.isActive(0x0004));
    EXPECT_TRUE(label.isActive(0x00FF));
    EXPECT_FALSE(label.isActive(0x0000));
    EXPECT_FALSE(label.isActive(0x0003));
}

TEST(IsActiveTest, SingleBitAllClear) {
    const auto label = BitLabel::bitInv("X", 0x0004);
    EXPECT_TRUE(label.isActive(0x0000));
    EXPECT_TRUE(label.isActive(0x0003));
    EXPECT_FALSE(label.isActive(0x0004));
    EXPECT_FALSE(label.isActive(0x00FF));
}

TEST(IsActiveTest, MultiBitAllSet) {
    const auto label = BitLabel::bits("G", 0x00F0);
    EXPECT_TRUE(label.isActive(0x00F0));
    EXPECT_TRUE(label.isActive(0xFFFF));
    EXPECT_FALSE(label.isActive(0x0070));
    EXPECT_FALSE(label.isActive(0x0000));
}

TEST(IsActiveTest, MultiBitAllClear) {
    const auto label = BitLabel::bitsInv("G", 0x00F0);
    EXPECT_TRUE(label.isActive(0x0000));
    EXPECT_TRUE(label.isActive(0x000F));
    EXPECT_FALSE(label.isActive(0x00F0));
    EXPECT_FALSE(label.isActive(0x00F1));
}

TEST(IsActiveTest, MultiBitAnySet) {
    const auto label = BitLabel::anyBit("G", 0x00F0);
    EXPECT_TRUE(label.isActive(0x0010));
    EXPECT_TRUE(label.isActive(0x00F0));
    EXPECT_FALSE(label.isActive(0x0000));
    EXPECT_FALSE(label.isActive(0x000F));
}

TEST(IsActiveTest, MultiBitAnyClear) {
    const auto label = BitLabel::anyBitInv("G", 0x00F0);
    EXPECT_TRUE(label.isActive(0x0000));
    EXPECT_TRUE(label.isActive(0x00E0));
    EXPECT_FALSE(label.isActive(0x00F0));
    EXPECT_FALSE(label.isActive(0xFFFF));
}

// ============================================================================
// BitLabel subclassing (virtual dispatch)
// ============================================================================

/// A custom label that is active when the value is at or above a threshold.
class ThresholdLabel : public BitLabel {
public:
    explicit ThresholdLabel(const char* name, uint32_t threshold)
        : BitLabel(name, 0, ActiveWhen::AllSet), threshold_(threshold) {}

    bool isActive(uint32_t value) const override {
        return value >= threshold_;
    }

private:
    uint32_t threshold_;
};

/// A custom label that is active when a specific bit pattern matches exactly.
class PatternMatchLabel : public BitLabel {
public:
    explicit PatternMatchLabel(const char* name, uint32_t pattern)
        : BitLabel(name, 0, ActiveWhen::AllSet), pattern_(pattern) {}

    bool isActive(uint32_t value) const override {
        return (value & 0xFF) == pattern_;
    }

private:
    uint32_t pattern_;
};

TEST(SubclassTest, ThresholdLabelIsActive) {
    ThresholdLabel label("Hot", 80);
    EXPECT_FALSE(label.isActive(0));
    EXPECT_FALSE(label.isActive(79));
    EXPECT_TRUE(label.isActive(80));
    EXPECT_TRUE(label.isActive(100));
    EXPECT_TRUE(label.isActive(0xFFFF));
}

TEST(SubclassTest, ThresholdLabelInFormatter) {
    static ThresholdLabel hot("Hot", 80);
    static BitLabel cool = BitLabel::bit("Cool", 0x01);
    static const BitLabel* labels[] = {&hot, &cool};

    const ColoredBitsetFormatter fmt{std::span<const BitLabel* const>{labels}};
    // value=100: Hot active (>=80), Cool inactive (bit 0 not set)
    EXPECT_EQ(stripAnsi(fmt.format(100)), "Hot=ON  Cool=OFF");
    // value=0: Hot inactive, Cool inactive
    EXPECT_EQ(stripAnsi(fmt.format(0)), "Hot=OFF  Cool=OFF");
    // value=1: Hot inactive, Cool active
    EXPECT_EQ(stripAnsi(fmt.format(1)), "Hot=OFF  Cool=ON");
}

TEST(SubclassTest, ThresholdLabelFormatActive) {
    static ThresholdLabel hot("Hot", 80);
    static ThresholdLabel warm("Warm", 50);
    static const BitLabel* labels[] = {&hot, &warm};

    const ColoredBitsetFormatter fmt{std::span<const BitLabel* const>{labels}};
    EXPECT_EQ(fmt.formatActive(100), "Hot Warm");
    EXPECT_EQ(fmt.formatActive(60), "Warm");
    EXPECT_EQ(fmt.formatActive(30), "(none)");
}

TEST(SubclassTest, PatternMatchLabelInFormatter) {
    static PatternMatchLabel match("Match42", 0x42);
    static const BitLabel* labels[] = {&match};

    const ColoredBitsetFormatter fmt{labels};
    EXPECT_EQ(stripAnsi(fmt.format(0x0042)), "Match42=ON");
    EXPECT_EQ(stripAnsi(fmt.format(0x0142)), "Match42=ON");
    EXPECT_EQ(stripAnsi(fmt.format(0x0043)), "Match42=OFF");
}

TEST(SubclassTest, MixedBaseAndSubclassInFormatter) {
    static BitLabel sto = BitLabel::bit("STO", 0x01);
    static ThresholdLabel hot("Hot", 0x80);
    static const BitLabel* labels[] = {&sto, &hot};

    const ColoredBitsetFormatter fmt{std::span<const BitLabel* const>{labels}};
    // value=0x81: STO active (bit 0 set), Hot active (>= 0x80)
    EXPECT_EQ(stripAnsi(fmt.format(0x81)), "STO=ON  Hot=ON");
    // value=0x01: STO active, Hot inactive
    EXPECT_EQ(stripAnsi(fmt.format(0x01)), "STO=ON  Hot=OFF");
    // value=0x80: STO inactive, Hot active
    EXPECT_EQ(stripAnsi(fmt.format(0x80)), "STO=OFF  Hot=ON");
    EXPECT_EQ(fmt.formatActive(0x81), "STO Hot");
}

TEST(SubclassTest, PointerArrayConstructor) {
    static BitLabel a = BitLabel::bit("A", 0x01);
    static BitLabel b = BitLabel::bit("B", 0x02);
    static const BitLabel* labels[] = {&a, &b};
    const ColoredBitsetFormatter fmt{labels};
    EXPECT_EQ(stripAnsi(fmt.format(0x03)), "A=ON  B=ON");
    EXPECT_EQ(fmt.formatActive(0x01), "A");
}

// ============================================================================
// ColoredBitsetFormatter::formatBit
// ============================================================================

TEST(FormatBitTest, ActiveIsGreen) {
    const std::string s = ColoredBitsetFormatter::formatBit(true, "STO");
    EXPECT_NE(s.find(kGreen), std::string::npos);
    EXPECT_EQ(s.find(kRed), std::string::npos);
    EXPECT_NE(s.find("ON"), std::string::npos);
    EXPECT_EQ(s.find("OFF"), std::string::npos);
    EXPECT_EQ(stripAnsi(s), "STO=ON");
}

TEST(FormatBitTest, InactiveIsRed) {
    const std::string s = ColoredBitsetFormatter::formatBit(false, "STO");
    EXPECT_NE(s.find(kRed), std::string::npos);
    EXPECT_EQ(s.find(kGreen), std::string::npos);
    EXPECT_NE(s.find("OFF"), std::string::npos);
    EXPECT_EQ(stripAnsi(s), "STO=OFF");
}

TEST(FormatBitTest, ContainsBoldAndReset) {
    const std::string on = ColoredBitsetFormatter::formatBit(true, "X");
    const std::string off = ColoredBitsetFormatter::formatBit(false, "X");
    EXPECT_NE(on.find(kBold), std::string::npos);
    EXPECT_NE(on.find(kReset), std::string::npos);
    EXPECT_NE(off.find(kBold), std::string::npos);
    EXPECT_NE(off.find(kReset), std::string::npos);
}

// ============================================================================
// ColoredBitsetFormatter::format (all labels, colored ON/OFF)
// ============================================================================

TEST(FormatTest, SingleBitActive) {
    static BitLabel labels[] = {BitLabel::bit("A", 0x01)};
    const ColoredBitsetFormatter fmt{labels};
    EXPECT_EQ(stripAnsi(fmt.format(0x01)), "A=ON");
}

TEST(FormatTest, SingleBitInactive) {
    static BitLabel labels[] = {BitLabel::bit("A", 0x01)};
    const ColoredBitsetFormatter fmt{labels};
    EXPECT_EQ(stripAnsi(fmt.format(0x00)), "A=OFF");
}

TEST(FormatTest, MultipleBitsWithSeparator) {
    static BitLabel labels[] = {
        BitLabel::bit("A", 0x01),
        BitLabel::bit("B", 0x02),
        BitLabel::bit("C", 0x04),
    };
    const ColoredBitsetFormatter fmt{labels};
    // Default separator is two spaces
    EXPECT_EQ(stripAnsi(fmt.format(0x05)), "A=ON  B=OFF  C=ON");
}

TEST(FormatTest, CustomSeparator) {
    static BitLabel labels[] = {
        BitLabel::bit("A", 0x01),
        BitLabel::bit("B", 0x02),
    };
    const ColoredBitsetFormatter fmt{labels};
    EXPECT_EQ(stripAnsi(fmt.format(0x03, " | ")), "A=ON | B=ON");
}

TEST(FormatTest, ZeroActiveBitInverted) {
    static BitLabel labels[] = {
        BitLabel::bitInv("STO", 0x01),  // bit=0 → active
    };
    const ColoredBitsetFormatter fmt{labels};
    EXPECT_EQ(stripAnsi(fmt.format(0x00)), "STO=ON");
    EXPECT_EQ(stripAnsi(fmt.format(0x01)), "STO=OFF");
}

TEST(FormatTest, EmptyFormatter) {
    const ColoredBitsetFormatter fmt;
    EXPECT_TRUE(fmt.format(0xFFFF).empty());
}

TEST(FormatTest, MultiBitLabelAllSet) {
    static BitLabel labels[] = {
        BitLabel::bits("SLS", 0x0F00),  // active only if all 4 bits set
    };
    const ColoredBitsetFormatter fmt{labels};
    EXPECT_EQ(stripAnsi(fmt.format(0x0F00)), "SLS=ON");
    EXPECT_EQ(stripAnsi(fmt.format(0x0700)), "SLS=OFF");
}

TEST(FormatTest, MultiBitLabelAnySet) {
    static BitLabel labels[] = {
        BitLabel::anyBit("SafeStop", 0x000B),  // STO|SS1|SS2
    };
    const ColoredBitsetFormatter fmt{labels};
    EXPECT_EQ(stripAnsi(fmt.format(0x0001)), "SafeStop=ON");
    EXPECT_EQ(stripAnsi(fmt.format(0x000A)), "SafeStop=ON");
    EXPECT_EQ(stripAnsi(fmt.format(0x0000)), "SafeStop=OFF");
}

// ============================================================================
// ColoredBitsetFormatter::formatActive (active labels only, no color)
// ============================================================================

TEST(FormatActiveTest, OnlyActiveLabels) {
    static BitLabel labels[] = {
        BitLabel::bit("A", 0x01),
        BitLabel::bit("B", 0x02),
        BitLabel::bit("C", 0x04),
    };
    const ColoredBitsetFormatter fmt{labels};
    EXPECT_EQ(fmt.formatActive(0x05), "A C");
}

TEST(FormatActiveTest, NoneActiveReturnsEmptyText) {
    static BitLabel labels[] = {
        BitLabel::bit("A", 0x01),
    };
    const ColoredBitsetFormatter fmt{labels};
    EXPECT_EQ(fmt.formatActive(0x00), "(none)");
}

TEST(FormatActiveTest, CustomEmptyText) {
    static BitLabel labels[] = {
        BitLabel::bit("A", 0x01),
    };
    const ColoredBitsetFormatter fmt{labels};
    EXPECT_EQ(fmt.formatActive(0x00, "---"), "---");
}

TEST(FormatActiveTest, ZeroActiveBitInverted) {
    static BitLabel labels[] = {
        BitLabel::bitInv("STO", 0x01),
        BitLabel::bitInv("SS1", 0x02),
    };
    const ColoredBitsetFormatter fmt{labels};
    // Both bits clear → both active
    EXPECT_EQ(fmt.formatActive(0x00), "STO SS1");
    // STO clear (active), SS1 set (inactive)
    EXPECT_EQ(fmt.formatActive(0x02), "STO");
}

TEST(FormatActiveTest, NoAnsiCodes) {
    static BitLabel labels[] = {
        BitLabel::bit("A", 0x01),
    };
    const ColoredBitsetFormatter fmt{labels};
    const std::string s = fmt.formatActive(0x01);
    EXPECT_EQ(s.find('\033'), std::string::npos);
}

TEST(FormatActiveTest, EmptyFormatter) {
    const ColoredBitsetFormatter fmt;
    EXPECT_EQ(fmt.formatActive(0xFFFF), "(none)");
}

// ============================================================================
// ColoredBitsetFormatter::size
// ============================================================================

TEST(SizeTest, ReturnsLabelCount) {
    static BitLabel labels[] = {
        BitLabel::bit("A", 0x01),
        BitLabel::bit("B", 0x02),
        BitLabel::bit("C", 0x04),
    };
    const ColoredBitsetFormatter fmt{labels};
    EXPECT_EQ(fmt.size(), 3u);
}

TEST(SizeTest, EmptyFormatter) {
    const ColoredBitsetFormatter fmt;
    EXPECT_EQ(fmt.size(), 0u);
}

// ============================================================================
// Integration: mixed one-active and zero-active labels in one table
// ============================================================================

TEST(IntegrationTest, MixedEncodings) {
    static BitLabel labels[] = {
        BitLabel::bitInv("STO", 0x0001),       // zero-active
        BitLabel::bitInv("SS1", 0x0002),       // zero-active
        BitLabel::bit(   "ErrorAck", 0x0080),  // one-active
    };
    const ColoredBitsetFormatter fmt{labels};

    // All safety bits clear, ErrorAck set
    const uint32_t val = 0x0080;
    EXPECT_EQ(stripAnsi(fmt.format(val)), "STO=ON  SS1=ON  ErrorAck=ON");
    EXPECT_EQ(fmt.formatActive(val), "STO SS1 ErrorAck");

    // All safety bits set, ErrorAck clear
    const uint32_t val2 = 0x0003;
    EXPECT_EQ(stripAnsi(fmt.format(val2)), "STO=OFF  SS1=OFF  ErrorAck=OFF");
    EXPECT_EQ(fmt.formatActive(val2), "(none)");
}

// ============================================================================
// ShowWhen: visibility control (show only when ON / OFF)
// ============================================================================

TEST(ShowWhenTest, FactorySetsShowWhenActive) {
    const BitLabel label = BitLabel::bitIfOn("FAULT", 0x01);
    EXPECT_EQ(label.show_when, ShowWhen::WhenActive);
    EXPECT_EQ(label.active_when, ActiveWhen::AllSet);
}

TEST(ShowWhenTest, FactorySetsShowWhenInactive) {
    const BitLabel label = BitLabel::bitIfOff("READY", 0x02);
    EXPECT_EQ(label.show_when, ShowWhen::WhenInactive);
    EXPECT_EQ(label.active_when, ActiveWhen::AllSet);
}

TEST(ShowWhenTest, BitInvIfOnFactory) {
    const BitLabel label = BitLabel::bitInvIfOn("SAFE", 0x04);
    EXPECT_EQ(label.show_when, ShowWhen::WhenActive);
    EXPECT_EQ(label.active_when, ActiveWhen::AllClear);
}

TEST(ShowWhenTest, BitInvIfOffFactory) {
    const BitLabel label = BitLabel::bitInvIfOff("UNSAFE", 0x08);
    EXPECT_EQ(label.show_when, ShowWhen::WhenInactive);
    EXPECT_EQ(label.active_when, ActiveWhen::AllClear);
}

TEST(ShowWhenTest, DefaultShowWhenIsAlways) {
    const BitLabel label = BitLabel::bit("X", 0x01);
    EXPECT_EQ(label.show_when, ShowWhen::Always);
}

// --- shouldShow() default implementation ---

TEST(ShowWhenTest, ShouldShowAlwaysReturnsTrue) {
    const auto label = BitLabel::bit("X", 0x01);
    EXPECT_TRUE(label.shouldShow(0x01));  // active
    EXPECT_TRUE(label.shouldShow(0x00));  // inactive
}

TEST(ShowWhenTest, ShouldShowWhenActive) {
    const auto label = BitLabel::bitIfOn("X", 0x01);
    EXPECT_TRUE(label.shouldShow(0x01));   // active → show
    EXPECT_FALSE(label.shouldShow(0x00));  // inactive → hide
}

TEST(ShowWhenTest, ShouldShowWhenInactive) {
    const auto label = BitLabel::bitIfOff("X", 0x01);
    EXPECT_FALSE(label.shouldShow(0x01));  // active → hide
    EXPECT_TRUE(label.shouldShow(0x00));   // inactive → show
}

TEST(ShowWhenTest, ShouldShowWhenActiveZeroActive) {
    const auto label = BitLabel::bitInvIfOn("SAFE", 0x01);
    // zero-active: active when bit=0
    EXPECT_TRUE(label.shouldShow(0x00));   // active (bit clear) → show
    EXPECT_FALSE(label.shouldShow(0x01));  // inactive (bit set) → hide
}

TEST(ShowWhenTest, ShouldShowWhenInactiveZeroActive) {
    const auto label = BitLabel::bitInvIfOff("UNSAFE", 0x01);
    // zero-active: active when bit=0
    EXPECT_FALSE(label.shouldShow(0x00));  // active (bit clear) → hide
    EXPECT_TRUE(label.shouldShow(0x01));   // inactive (bit set) → show
}

// --- format() respects shouldShow() ---

TEST(ShowWhenFormatTest, BitIfOnHidesWhenInactive) {
    static BitLabel labels[] = {
        BitLabel::bitIfOn("FAULT", 0x01),
        BitLabel::bit("STO", 0x02),
    };
    const ColoredBitsetFormatter fmt{labels};

    // value=0x01: FAULT active (shown), STO inactive (shown)
    EXPECT_EQ(stripAnsi(fmt.format(0x01)), "FAULT=ON  STO=OFF");
    // value=0x02: FAULT inactive (hidden), STO active (shown)
    EXPECT_EQ(stripAnsi(fmt.format(0x02)), "STO=ON");
    // value=0x00: FAULT inactive (hidden), STO inactive (shown)
    EXPECT_EQ(stripAnsi(fmt.format(0x00)), "STO=OFF");
}

TEST(ShowWhenFormatTest, BitIfOffHidesWhenActive) {
    static BitLabel labels[] = {
        BitLabel::bitIfOff("READY", 0x01),
        BitLabel::bit("STO", 0x02),
    };
    const ColoredBitsetFormatter fmt{labels};

    // value=0x01: READY active (hidden), STO inactive (shown)
    EXPECT_EQ(stripAnsi(fmt.format(0x01)), "STO=OFF");
    // value=0x02: READY inactive (shown), STO active (shown)
    EXPECT_EQ(stripAnsi(fmt.format(0x02)), "READY=OFF  STO=ON");
    // value=0x00: READY inactive (shown), STO inactive (shown)
    EXPECT_EQ(stripAnsi(fmt.format(0x00)), "READY=OFF  STO=OFF");
}

TEST(ShowWhenFormatTest, BitInvIfOnHidesWhenInactive) {
    static BitLabel labels[] = {
        BitLabel::bitInvIfOn("SAFE", 0x01),  // zero-active, show when ON
        BitLabel::bit("X", 0x02),
    };
    const ColoredBitsetFormatter fmt{labels};

    // value=0x00: SAFE active (bit clear, shown as ON), X inactive (shown)
    EXPECT_EQ(stripAnsi(fmt.format(0x00)), "SAFE=ON  X=OFF");
    // value=0x01: SAFE inactive (bit set, hidden), X inactive (shown)
    EXPECT_EQ(stripAnsi(fmt.format(0x01)), "X=OFF");
    // value=0x03: SAFE inactive (hidden), X active (shown)
    EXPECT_EQ(stripAnsi(fmt.format(0x03)), "X=ON");
}

TEST(ShowWhenFormatTest, BitInvIfOffHidesWhenActive) {
    static BitLabel labels[] = {
        BitLabel::bitInvIfOff("UNSAFE", 0x01),  // zero-active, show when OFF
        BitLabel::bit("X", 0x02),
    };
    const ColoredBitsetFormatter fmt{labels};

    // value=0x00: UNSAFE active (bit clear, hidden), X inactive (shown)
    EXPECT_EQ(stripAnsi(fmt.format(0x00)), "X=OFF");
    // value=0x01: UNSAFE inactive (bit set, shown as OFF), X inactive (shown)
    EXPECT_EQ(stripAnsi(fmt.format(0x01)), "UNSAFE=OFF  X=OFF");
    // value=0x03: UNSAFE inactive (shown), X active (shown)
    EXPECT_EQ(stripAnsi(fmt.format(0x03)), "UNSAFE=OFF  X=ON");
}

TEST(ShowWhenFormatTest, AllLabelsHiddenReturnsEmptyString) {
    static BitLabel labels[] = {
        BitLabel::bitIfOn("A", 0x01),
        BitLabel::bitIfOn("B", 0x02),
    };
    const ColoredBitsetFormatter fmt{labels};
    // value=0x00: both inactive → both hidden
    EXPECT_EQ(fmt.format(0x00), "");
}

TEST(ShowWhenFormatTest, MixedVisibilityAndAlways) {
    static BitLabel labels[] = {
        BitLabel::bitIfOn("FAULT", 0x01),    // show only when ON
        BitLabel::bit("RUN", 0x02),          // always show
        BitLabel::bitIfOff("IDLE", 0x04),    // show only when OFF
    };
    const ColoredBitsetFormatter fmt{labels};

    // value=0x02: FAULT hidden (inactive), RUN shown (ON), IDLE shown (OFF)
    EXPECT_EQ(stripAnsi(fmt.format(0x02)), "RUN=ON  IDLE=OFF");
    // value=0x07: FAULT shown (ON), RUN shown (ON), IDLE hidden (active)
    EXPECT_EQ(stripAnsi(fmt.format(0x07)), "FAULT=ON  RUN=ON");
    // value=0x00: FAULT hidden, RUN shown (OFF), IDLE shown (OFF)
    EXPECT_EQ(stripAnsi(fmt.format(0x00)), "RUN=OFF  IDLE=OFF");
}

// --- formatActive() respects shouldShow() ---

TEST(ShowWhenFormatActiveTest, WhenActiveLabelsAppearInFormatActive) {
    static BitLabel labels[] = {
        BitLabel::bitIfOn("FAULT", 0x01),  // show when ON
        BitLabel::bit("RUN", 0x02),        // always show
    };
    const ColoredBitsetFormatter fmt{labels};

    // value=0x03: both active, both visible
    EXPECT_EQ(fmt.formatActive(0x03), "FAULT RUN");
    // value=0x02: RUN active, FAULT inactive (hidden anyway)
    EXPECT_EQ(fmt.formatActive(0x02), "RUN");
    // value=0x00: nothing active
    EXPECT_EQ(fmt.formatActive(0x00), "(none)");
}

TEST(ShowWhenFormatActiveTest, WhenInactiveLabelsNeverInFormatActive) {
    static BitLabel labels[] = {
        BitLabel::bitIfOff("IDLE", 0x01),  // show when OFF (inactive)
        BitLabel::bit("RUN", 0x02),        // always show
    };
    const ColoredBitsetFormatter fmt{labels};

    // value=0x02: RUN active, IDLE inactive
    // IDLE has ShowWhen::WhenInactive, so shouldShow(0x02) = !isActive(0x02) = true
    // But isActive(0x02) for IDLE is false, so formatActive skips it (not active)
    EXPECT_EQ(fmt.formatActive(0x02), "RUN");
    // value=0x03: both active, but IDLE has WhenInactive → shouldShow=false → skipped
    EXPECT_EQ(fmt.formatActive(0x03), "RUN");
    // value=0x00: nothing active
    EXPECT_EQ(fmt.formatActive(0x00), "(none)");
}

TEST(ShowWhenFormatActiveTest, AllHiddenReturnsEmptyText) {
    static BitLabel labels[] = {
        BitLabel::bitIfOff("IDLE", 0x01),  // never visible when active
    };
    const ColoredBitsetFormatter fmt{labels};
    // value=0x01: IDLE active but WhenInactive → shouldShow=false → skipped
    EXPECT_EQ(fmt.formatActive(0x01), "(none)");
}

// --- Multi-bit visibility factories ---

TEST(ShowWhenMultiBitTest, BitsIfOnFactory) {
    const BitLabel label = BitLabel::bitsIfOn("GROUP", 0x0F);
    EXPECT_EQ(label.active_when, ActiveWhen::AllSet);
    EXPECT_EQ(label.show_when, ShowWhen::WhenActive);
}

TEST(ShowWhenMultiBitTest, BitsInvIfOffFactory) {
    const BitLabel label = BitLabel::bitsInvIfOff("CLEAR", 0x0F);
    EXPECT_EQ(label.active_when, ActiveWhen::AllClear);
    EXPECT_EQ(label.show_when, ShowWhen::WhenInactive);
}

TEST(ShowWhenMultiBitTest, AnyBitIfOnInFormat) {
    static BitLabel labels[] = {
        BitLabel::anyBitIfOn("WARN", 0x03),  // active if any of bits 0,1 set; show when active
        BitLabel::bit("X", 0x04),            // always show
    };
    const ColoredBitsetFormatter fmt{labels};

    // value=0x01: WARN active (shown), X inactive (shown)
    EXPECT_EQ(stripAnsi(fmt.format(0x01)), "WARN=ON  X=OFF");
    // value=0x04: WARN inactive (hidden), X active (shown)
    EXPECT_EQ(stripAnsi(fmt.format(0x04)), "X=ON");
}

// --- Custom shouldShow() via subclassing ---

/// Custom label that is shown only on even cycles (simulated).
class EvenCycleLabel : public BitLabel {
public:
    explicit EvenCycleLabel(const char* name, uint32_t mask)
        : BitLabel(name, mask, ActiveWhen::AllSet) {}

    bool shouldShow(uint32_t value) const override {
        // Show only when bit 7 (parity bit) is set, regardless of activity
        return (value & 0x80) != 0;
    }
};

TEST(ShowWhenSubclassTest, CustomShouldShowInFormat) {
    static EvenCycleLabel custom("CUSTOM", 0x01);
    static BitLabel normal = BitLabel::bit("NORMAL", 0x02);
    static const BitLabel* labels[] = {&custom, &normal};
    const ColoredBitsetFormatter fmt{std::span<const BitLabel* const>{labels}};

    // value=0x80: bit 7 set → custom shown (OFF, since bit 0 not set), normal shown (OFF)
    EXPECT_EQ(stripAnsi(fmt.format(0x80)), "CUSTOM=OFF  NORMAL=OFF");
    // value=0x81: bit 7 set → custom shown (ON), normal shown (OFF)
    EXPECT_EQ(stripAnsi(fmt.format(0x81)), "CUSTOM=ON  NORMAL=OFF");
    // value=0x01: bit 7 clear → custom hidden, normal shown (OFF)
    EXPECT_EQ(stripAnsi(fmt.format(0x01)), "NORMAL=OFF");
    // value=0x03: bit 7 clear → custom hidden, normal shown (ON)
    EXPECT_EQ(stripAnsi(fmt.format(0x03)), "NORMAL=ON");
}

TEST(ShowWhenSubclassTest, CustomShouldShowInFormatActive) {
    static EvenCycleLabel custom("CUSTOM", 0x01);
    static const BitLabel* labels[] = {&custom};
    const ColoredBitsetFormatter fmt{std::span<const BitLabel* const>{labels}};

    // value=0x81: bit 7 set (visible), bit 0 set (active) → shown
    EXPECT_EQ(fmt.formatActive(0x81), "CUSTOM");
    // value=0x80: bit 7 set (visible), bit 0 clear (inactive) → not active
    EXPECT_EQ(fmt.formatActive(0x80), "(none)");
    // value=0x01: bit 7 clear (hidden) → skipped even though active
    EXPECT_EQ(fmt.formatActive(0x01), "(none)");
}

/// Custom label that overrides both isActive() and shouldShow().
class FaultOnlyLabel : public BitLabel {
public:
    explicit FaultOnlyLabel(const char* name, uint32_t mask)
        : BitLabel(name, mask, ActiveWhen::AllSet, ShowWhen::WhenActive) {}

    bool isActive(uint32_t value) const override {
        // Active when the bit is set AND bit 15 (valid bit) is set
        return (value & mask) != 0 && (value & 0x8000) != 0;
    }
};

TEST(ShowWhenSubclassTest, CustomIsActiveAndShouldShow) {
    static FaultOnlyLabel fault("FAULT", 0x01);
    static const BitLabel* labels[] = {&fault};
    const ColoredBitsetFormatter fmt{std::span<const BitLabel* const>{labels}};

    // value=0x8001: valid bit set, fault bit set → active, shouldShow=true (WhenActive)
    EXPECT_EQ(stripAnsi(fmt.format(0x8001)), "FAULT=ON");
    // value=0x0001: valid bit clear → inactive, shouldShow=false → hidden
    EXPECT_EQ(fmt.format(0x0001), "");
    // value=0x8000: valid bit set, fault bit clear → inactive, shouldShow=false → hidden
    EXPECT_EQ(fmt.format(0x8000), "");
    // formatActive: only shows when both visible and active
    EXPECT_EQ(fmt.formatActive(0x8001), "FAULT");
    EXPECT_EQ(fmt.formatActive(0x0001), "(none)");
    EXPECT_EQ(fmt.formatActive(0x8000), "(none)");
}
