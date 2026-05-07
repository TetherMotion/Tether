#include <gtest/gtest.h>
#include "sii/SIIParser.hpp"
#include "sii/SIIReader.hpp"
#include "tether/platform/Platform.hpp"

using namespace EtherCAT::SII;
using Tether::Platform::Logger;

TEST(SIIUtils, CategoryAndProtocolNames) {
    EXPECT_STREQ(getCategoryTypeName(CAT_STRINGS), "Strings");
    EXPECT_STREQ(getCategoryTypeName(CAT_GENERAL), "General");
    EXPECT_STREQ(getCategoryTypeName(0xDEAD), "Unknown");

    uint16_t p = MBX_PROTO_COE | MBX_PROTO_EOE | MBX_PROTO_AOE;
    const char* s = getMailboxProtocolName(p);
    // order is AoE EoE CoE FoE SoE VoE but only first three set
    std::string str(s);
    EXPECT_NE(str.find("AoE"), std::string::npos);
    EXPECT_NE(str.find("EoE"), std::string::npos);
    EXPECT_NE(str.find("CoE"), std::string::npos);

    EXPECT_STREQ(getMailboxProtocolName(0), "None");
}

TEST(SIILogHelpers, IdentityAndMailboxLogging) {
    // Capture logs
    Logger& logger = Logger::instance();
    logger.setLevel(Tether::Platform::LogLevel::Info); // ensure Info messages are delivered
    std::vector<std::string> captured;
    logger.setHandler([&](Tether::Platform::LogLevel, const char* tag, const char* msg) {
        (void)tag; captured.emplace_back(msg);
    });

    SIIIdentity id;
    id.vendor_id = 0x12345678;
    id.product_code = 0xC0FFEE;
    id.revision_number = (0x0002 << 16) | 0x0003; // 2.3
    id.serial_number = 0xDEADBEEF;

    logSIIIdentity(id, "sii_test");

    bool foundVendor = false, foundRevision = false, foundSerial = false;
    for (const auto& m : captured) {
        if (m.find("Vendor ID") != std::string::npos) foundVendor = true;
        if (m.find("Revision") != std::string::npos) foundRevision = true;
        if (m.find("Serial") != std::string::npos) foundSerial = true;
    }
    EXPECT_TRUE(foundVendor);
    EXPECT_TRUE(foundRevision);
    EXPECT_TRUE(foundSerial);

    // Mailbox logging
    captured.clear();
    SIIMailboxConfig mbx{};
    mbx.std_rx_offset = 0x1000; mbx.std_rx_size = 128;
    mbx.std_tx_offset = 0x1100; mbx.std_tx_size = 64;
    mbx.protocols = MBX_PROTO_COE | MBX_PROTO_FOE;

    logSIIMailbox(mbx, "mbx_test");
    bool foundStd = false, foundProto = false;
    for (const auto& m : captured) {
        if (m.find("Standard Mailbox") != std::string::npos) foundStd = true;
        if (m.find("CoE") != std::string::npos || m.find("FoE") != std::string::npos) foundProto = true;
    }
    EXPECT_TRUE(foundStd);
    EXPECT_TRUE(foundProto);

    // Clean up: remove handler so it doesn't dangle after test exits
    logger.setHandler(nullptr);
}
