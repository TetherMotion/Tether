/**
 * @file raw_transport_tests.cpp
 * @brief Tests for EtherCATMaster transport hooks (pushAprdResponse,
 *        readRegister, writeRegister, setAprdTestCallback, setApwrTestCallback).
 *
 * All transport operations go through the EtherCATMaster instance directly;
 * no global bridge or free-function wrappers are used.
 */
#include <gtest/gtest.h>
#include <cstring>
#include "tether/ethercat/EtherCATMaster.hpp"

class MasterTransportTest : public ::testing::Test {
protected:
    void SetUp() override {
        EtherCAT::EtherCATMaster::Config cfg;
        cfg.rx_queue_depth  = 4;
        cfg.txpdo_queue_depth = 4;
        master_ = std::make_unique<EtherCAT::EtherCATMaster>(cfg);
    }
    void TearDown() override {
        master_.reset();
    }
    std::unique_ptr<EtherCAT::EtherCATMaster> master_;
};

TEST_F(MasterTransportTest, AprdPushAndConsume) {
    master_->clearAprdResponses();
    const uint8_t payload[4] = {1, 2, 3, 4};
    master_->pushAprdResponse(true, 0x1234, 0x5678, payload, sizeof(payload));

    uint8_t out[4] = {0};
    bool ok = master_->readRegister(0x1234, 0x5678, out, sizeof(out), 1);
    EXPECT_TRUE(ok);
    EXPECT_EQ(0, memcmp(out, payload, sizeof(payload)));
}

TEST_F(MasterTransportTest, AprdFallbackZeroWhenQueueNonEmpty) {
    master_->clearAprdResponses();
    const uint8_t payload[2] = {0x55, 0x66};
    master_->pushAprdResponse(true, 0x1111, 0x2222, payload, sizeof(payload));

    uint8_t out[4];
    memset(out, 0xFF, sizeof(out));
    // Request for a different ADO/ADP should return fallback zero and succeed
    bool ok = master_->readRegister(0x9999, 0x8888, out, sizeof(out), 1);
    EXPECT_TRUE(ok);
    for (size_t i = 0; i < sizeof(out); ++i)
        EXPECT_EQ(out[i], 0);
}

TEST_F(MasterTransportTest, ApwrSucceedsWithCallback) {
    // writeRegister requires either a NetworkInterface or a test callback
    master_->setApwrTestCallback(
        [](uint16_t /*adp*/, uint16_t /*ado*/, const void* /*data*/,
           uint16_t /*len*/, unsigned int /*timeout_ms*/) -> bool {
            return true;
        });
    bool ok = master_->writeRegister(0x1010, 0x0300, nullptr, 0, 1);
    EXPECT_TRUE(ok);
    master_->setApwrTestCallback(nullptr);
}

TEST_F(MasterTransportTest, AprdAndApwrCallbacksAreUsed) {
    // APRD callback that writes a 4-byte value
    master_->setAprdTestCallback(
        [](uint16_t /*adp*/, uint16_t /*ado*/, void* out, uint16_t len,
           unsigned int /*timeout_ms*/) -> bool {
            if (len >= 4) {
                uint8_t data[4] = {9, 8, 7, 6};
                memcpy(out, data, 4);
                return true;
            }
            return false;
        });

    master_->setApwrTestCallback(
        [](uint16_t /*adp*/, uint16_t /*ado*/, const void* /*data*/,
           uint16_t /*len*/, unsigned int /*timeout_ms*/) -> bool {
            return true;
        });

    uint8_t out[4] = {0};
    EXPECT_TRUE(master_->readRegister(0x2000, 0x3000, out, sizeof(out), 1));
    const uint8_t expected[4] = {9, 8, 7, 6};
    EXPECT_EQ(0, memcmp(out, expected, 4));

    EXPECT_TRUE(master_->writeRegister(0x2000, 0x3000, nullptr, 0, 1));

    // cleanup
    master_->setAprdTestCallback(nullptr);
    master_->setApwrTestCallback(nullptr);
}
