#include <gtest/gtest.h>

#include "tether/ethercat/FaultDetection.hpp"

using namespace EtherCAT;

// Minimal stub transport for the legacy tests in this file
class StubTransport : public IFaultTransport {
public:
    bool readRegister(uint16_t, uint16_t, void*, uint16_t) override { return false; }
    bool writeRegister(uint16_t, uint16_t, const void*, uint16_t) override { return false; }
    uint64_t getTimestampMs() override { return 0; }
    void delayMs(uint32_t) override {}
};

TEST(FaultDetection_Header, ALStatusNames) {
    EXPECT_STREQ(getALStatusCodeName(ALStatusCode::InvalidMailboxConfig), "Invalid mailbox configuration");
    // Test numeric overload
    EXPECT_STREQ(getALStatusCodeName(static_cast<uint16_t>(ALStatusCode::InvalidMailboxConfig)), "Invalid mailbox configuration");
}

TEST(FaultDetection_Header, CiA402ErrorNames) {
    EXPECT_STREQ(getCiA402ErrorCodeName(CiA402ErrorCode::OverCurrent), "Over current");
    EXPECT_STREQ(getCiA402ErrorCodeName(static_cast<uint16_t>(CiA402ErrorCode::OverCurrent)), "Over current");
}

TEST(ManufacturerFault, ParseAndFormat) {
    auto f1 = ManufacturerFault::parse(741, 0, 0);
    EXPECT_EQ(f1.raw_code, 741u);
    EXPECT_STREQ(f1.description, "No Sync (Err74.1)");

    char buf[32] = {};
    size_t n = f1.format(buf, sizeof(buf));
    EXPECT_GT(n, 0u);
    EXPECT_STREQ(buf, "Err74.1");

    auto f2 = ManufacturerFault::parse(200, 0, 0);
    EXPECT_STREQ(f2.description, "Over current");
    size_t n2 = f2.format(buf, sizeof(buf));
    EXPECT_GT(n2, 0u);
    EXPECT_STREQ(buf, "Err20");
}

TEST(FaultDetection, InitShutdown) {
    StubTransport transport;
    FaultDetector fd(transport);
    EXPECT_TRUE(fd.init(4));
    // Subsequent init should be idempotent
    EXPECT_TRUE(fd.init(2));
    fd.shutdown();
}
