#include <gtest/gtest.h>

#include "tether/ethercat/Retry.hpp"
#include "tether/ethercat/SDOManager.hpp"
#include "tether/ethercat/Types.hpp"

using namespace EtherCAT;

TEST(Retry_Header, PolicyTimeouts) {
    auto p = Raw::RetryPolicy::standard();
    EXPECT_EQ(p.max_retries, 3u);
    EXPECT_EQ(p.getTimeoutForAttempt(0), p.initial_timeout_ms);
    EXPECT_EQ(p.getTimeoutForAttempt(1), static_cast<uint32_t>(p.initial_timeout_ms * p.backoff_multiplier));
    EXPECT_GE(p.getTimeoutForAttempt(100), 0u);

    auto none = Raw::RetryPolicy::none();
    EXPECT_EQ(none.max_retries, 0u);
    EXPECT_EQ(none.getTimeoutForAttempt(1), none.initial_timeout_ms);
}

TEST(StoredDatagram_Header, ValidityAndConstruction) {
    uint8_t data[4] = {1,2,3,4};
    Raw::StoredDatagram d(Raw::EtherCATCommand::APRD, 5, 0x1000, 0x2000, data, sizeof(data));
    EXPECT_TRUE(d.isValid());
    EXPECT_EQ(d.cmd, Raw::EtherCATCommand::APRD);
    EXPECT_EQ(d.idx, 5u);
}

TEST(SDO_Header, ConstantsAndEnums) {
    EXPECT_EQ(SDO::kMaxSDODataSize, 256u);
    EXPECT_EQ(SDO::kDefaultSDOTimeoutMs, 1000u);

    EXPECT_EQ(static_cast<uint32_t>(SDO::SDOAbortCode::Success), 0u);
    EXPECT_EQ(static_cast<uint8_t>(SDO::SDOOperation::Upload), 0u);
}

TEST(Types_Header, FrameHelpers) {
    FrameHeader fh;
    fh.set(100, 2);
    EXPECT_EQ(fh.length(), 100u);
    EXPECT_EQ(fh.type(), 2u);

    Datagram g;
    const char* payload = "hello";
    g.setData(payload, 5);
    EXPECT_EQ(g.header.dataLength(), 5u);
    EXPECT_EQ(g.totalSize(), sizeof(DatagramHeader) + 5 + sizeof(uint16_t));
}
