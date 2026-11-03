/**
 * @file test_mailbox_endpoint_config.cpp
 * @brief Coverage for the mailbox size/address CLI parsers.
 */

#include <gtest/gtest.h>

#include "tether/ethercat/MailboxEndpointConfig.hpp"

using namespace EtherCAT;

// ============================================================================
// parseMailboxSize — short form
// ============================================================================

TEST(MailboxSizeParseTest, PlainNumberSetsBothSizes) {
    MailboxSizeConfig out{};
    ASSERT_TRUE(parseMailboxSize("256", out));
    EXPECT_EQ(out.inSize, 256);
    EXPECT_EQ(out.outSize, 256);
}

TEST(MailboxSizeParseTest, HexNumberParses) {
    MailboxSizeConfig out{};
    ASSERT_TRUE(parseMailboxSize("0x100", out));
    EXPECT_EQ(out.inSize, 256);
    EXPECT_EQ(out.outSize, 256);
}

TEST(MailboxSizeParseTest, OutOfRangeRejected) {
    MailboxSizeConfig out{};
    EXPECT_FALSE(parseMailboxSize("0", out));
    EXPECT_FALSE(parseMailboxSize("65536", out));
    EXPECT_FALSE(parseMailboxSize("-5", out));
}

TEST(MailboxSizeParseTest, GarbageRejected) {
    MailboxSizeConfig out{};
    EXPECT_FALSE(parseMailboxSize("abc", out));
    EXPECT_FALSE(parseMailboxSize("0x", out));
}

// ============================================================================
// parseMailboxSize — long form
// ============================================================================

TEST(MailboxSizeParseTest, LongFormDecimal) {
    MailboxSizeConfig out{};
    ASSERT_TRUE(parseMailboxSize("in:128,out:256", out));
    EXPECT_EQ(out.inSize, 128);
    EXPECT_EQ(out.outSize, 256);
}

TEST(MailboxSizeParseTest, LongFormHex) {
    MailboxSizeConfig out{};
    ASSERT_TRUE(parseMailboxSize("in:0x80,out:0x100", out));
    EXPECT_EQ(out.inSize, 128);
    EXPECT_EQ(out.outSize, 256);
}

TEST(MailboxSizeParseTest, LongFormMissingHalfRejected) {
    MailboxSizeConfig out{};
    EXPECT_FALSE(parseMailboxSize("in:128", out));
    EXPECT_FALSE(parseMailboxSize("out:256", out));
    EXPECT_FALSE(parseMailboxSize("in:128:256", out));
}

TEST(MailboxSizeParseTest, LongFormBadValueRejected) {
    MailboxSizeConfig out{};
    EXPECT_FALSE(parseMailboxSize("in:0,out:256", out));
    EXPECT_FALSE(parseMailboxSize("in:128,out:70000", out));
    EXPECT_FALSE(parseMailboxSize("in:x,out:256", out));
    EXPECT_FALSE(parseMailboxSize("in:128,out:y", out));
}

// ============================================================================
// parseMailboxAddress
// ============================================================================

TEST(MailboxAddressParseTest, HexPairParses) {
    MailboxAddressConfig out{};
    ASSERT_TRUE(parseMailboxAddress("in:0x1000,out:0x1200", out));
    EXPECT_EQ(out.inAddress, 0x1000);
    EXPECT_EQ(out.outAddress, 0x1200);
}

TEST(MailboxAddressParseTest, DecimalPairParses) {
    MailboxAddressConfig out{};
    ASSERT_TRUE(parseMailboxAddress("in:4096,out:4608", out));
    EXPECT_EQ(out.inAddress, 4096);
    EXPECT_EQ(out.outAddress, 4608);
}

TEST(MailboxAddressParseTest, EqualAddressesRejected) {
    MailboxAddressConfig out{};
    EXPECT_FALSE(parseMailboxAddress("in:0x1000,out:0x1000", out));
    EXPECT_FALSE(parseMailboxAddress("in:4096,out:4096", out));
}

TEST(MailboxAddressParseTest, MissingHalfRejected) {
    MailboxAddressConfig out{};
    EXPECT_FALSE(parseMailboxAddress("in:0x1000", out));
    EXPECT_FALSE(parseMailboxAddress("out:0x1200", out));
}

TEST(MailboxAddressParseTest, BadValuesRejected) {
    MailboxAddressConfig out{};
    EXPECT_FALSE(parseMailboxAddress("in:-1,out:0x1200", out));
    EXPECT_FALSE(parseMailboxAddress("in:0x1000,out:70000", out));
    EXPECT_FALSE(parseMailboxAddress("in:zz,out:0x1200", out));
}

TEST(MailboxAddressParseTest, ZeroIsAllowed) {
    MailboxAddressConfig out{};
    ASSERT_TRUE(parseMailboxAddress("in:0,out:0x1000", out));
    EXPECT_EQ(out.inAddress, 0);
}

// ============================================================================
// logMailboxConfig
// ============================================================================

TEST(MailboxConfigLogTest, LogsWithoutCrash) {
    MailboxSizeConfig size{};
    size.inSize = 128;
    size.outSize = 256;
    MailboxAddressConfig addr{};
    addr.inAddress = 0x1000;
    addr.outAddress = 0x1200;
    logMailboxConfig(size, addr, "test");
}
