/**
 * @file test_coe_pending_futures.cpp
 * @brief Coverage for the legacy queueRequest/getResponse CoE API
 *        (src/ethercat/raw/CoEPendingFutures.cpp): queued upload/download,
 *        immediate-complete responses, waitForResponse, and pendingCount.
 */

#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include <cstring>

#include "tether/ethercat/CoEManager.hpp"

using namespace EtherCAT;
using ::testing::_;
using ::testing::Invoke;
using ::testing::NiceMock;

namespace {

uint64_t realMicros() {
    return std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
}

class MockSDOTransportRaw : public SDO::ISDOTransport {
public:
    MOCK_METHOD(bool, sdoUpload,
                (uint16_t slave_index, uint8_t* mbx_counter,
                 uint16_t mbx_wr_addr, uint16_t mbx_wr_len,
                 uint16_t mbx_rd_addr, uint16_t mbx_rd_len,
                 uint16_t index, uint8_t sub,
                 uint8_t* out, size_t out_cap, size_t* out_len,
                 bool diag_enabled, unsigned int poll_interval_ms,
                 unsigned int transaction_timeout_ms),
                (override));
    MOCK_METHOD(bool, sdoDownload,
                (uint16_t slave_index, uint8_t* mbx_counter,
                 uint16_t mbx_wr_addr, uint16_t mbx_wr_len,
                 uint16_t mbx_rd_addr, uint16_t mbx_rd_len,
                 uint16_t index, uint8_t sub,
                 const uint8_t* data, size_t data_len,
                 bool diag_enabled, unsigned int poll_interval_ms,
                 unsigned int transaction_timeout_ms),
                (override));
    MOCK_METHOD(uint64_t, getMicroseconds, (), (override));
};

auto uploadOk(const void* data, size_t len) {
    return [data, len](uint16_t, uint8_t*, uint16_t, uint16_t, uint16_t, uint16_t,
                       uint16_t, uint8_t, uint8_t* out, size_t, size_t* out_len,
                       bool, unsigned int, unsigned int) -> bool {
        memcpy(out, data, len);
        if (out_len) *out_len = len;
        return true;
    };
}

auto downloadOk() {
    return [](uint16_t, uint8_t*, uint16_t, uint16_t, uint16_t, uint16_t,
              uint16_t, uint8_t, const uint8_t*, size_t,
              bool, unsigned int, unsigned int) -> bool {
        return true;
    };
}

class CoEPendingFuturesTest : public ::testing::Test {
protected:
    void SetUp() override {
        ON_CALL(transport_, getMicroseconds()).WillByDefault(Invoke(&realMicros));
        ON_CALL(transport_, sdoUpload(_, _, _, _, _, _, _, _, _, _, _, _, _, _))
            .WillByDefault(Invoke(uploadOk("\x11\x22\x33\x44", 4)));
        ON_CALL(transport_, sdoDownload(_, _, _, _, _, _, _, _, _, _, _, _, _))
            .WillByDefault(Invoke(downloadOk()));
    }

    NiceMock<MockSDOTransportRaw> transport_;
};

// Immediate-complete path: an uninitialized manager fails fast and the
// response is retrievable synchronously.
TEST_F(CoEPendingFuturesTest, UnconfiguredUploadFailsImmediately) {
    CoE::CoEManager mgr(0, transport_);
    SDO::SDORequest req{};
    req.slave_index = 0;
    req.index = 0x6040;
    req.subindex = 0;
    req.operation = SDO::SDOOperation::Upload;

    const uint32_t id = mgr.queueRequest(req);
    ASSERT_NE(id, 0u);

    SDO::SDOResponse resp{};
    ASSERT_TRUE(mgr.getResponse(id, resp));
    EXPECT_EQ(resp.request_id, id);
    EXPECT_EQ(resp.index, 0x6040);
    EXPECT_EQ(resp.operation, SDO::SDOOperation::Upload);
    EXPECT_EQ(resp.status, SDO::SDOStatus::Failed);
    EXPECT_NE(resp.abort_code, SDOAbortCode::Success);
}

TEST_F(CoEPendingFuturesTest, UnconfiguredDownloadFailsImmediately) {
    CoE::CoEManager mgr(0, transport_);
    SDO::SDORequest req{};
    req.slave_index = 0;
    req.index = 0x6040;
    req.subindex = 0;
    req.operation = SDO::SDOOperation::Download;
    req.data[0] = 0xAA;
    req.data_size = 1;

    const uint32_t id = mgr.queueRequest(req);
    ASSERT_NE(id, 0u);

    SDO::SDOResponse resp{};
    ASSERT_TRUE(mgr.getResponse(id, resp));
    EXPECT_EQ(resp.status, SDO::SDOStatus::Failed);
    // Download responses echo the request payload
    EXPECT_EQ(resp.data[0], 0xAA);
    EXPECT_EQ(resp.data_size, 1u);
}

// Pending-future path: an initialized manager defers completion to the worker.
TEST_F(CoEPendingFuturesTest, QueuedUploadCompletesViaWorker) {
    CoE::CoEManager mgr(0, transport_);
    mgr.configureMailbox(0x1000, 128, 0x1400, 128);
    ASSERT_TRUE(mgr.init());

    SDO::SDORequest req{};
    req.slave_index = 0;
    req.index = 0x1018;
    req.subindex = 1;
    req.operation = SDO::SDOOperation::Upload;

    const uint32_t id = mgr.queueRequest(req);
    ASSERT_NE(id, 0u);
    EXPECT_GE(mgr.pendingCount(), 0u);

    SDO::SDOResponse resp{};
    ASSERT_TRUE(mgr.waitForResponse(id, resp, 5000));
    EXPECT_EQ(resp.status, SDO::SDOStatus::Complete);
    EXPECT_EQ(resp.abort_code, SDOAbortCode::Success);
    EXPECT_TRUE(resp.success());
    ASSERT_GE(resp.data_size, 4u);
    EXPECT_EQ(resp.data[0], 0x11);
    EXPECT_EQ(resp.data[3], 0x44);
    mgr.deinit();
}

TEST_F(CoEPendingFuturesTest, QueuedDownloadCompletesViaWorker) {
    CoE::CoEManager mgr(0, transport_);
    mgr.configureMailbox(0x1000, 128, 0x1400, 128);
    ASSERT_TRUE(mgr.init());

    SDO::SDORequest req{};
    req.slave_index = 0;
    req.index = 0x6040;
    req.subindex = 0;
    req.operation = SDO::SDOOperation::Download;
    const uint8_t cw[2] = {0x0F, 0x00};
    std::memcpy(req.data, cw, 2);
    req.data_size = 2;

    const uint32_t id = mgr.queueRequest(req);
    ASSERT_NE(id, 0u);

    SDO::SDOResponse resp{};
    ASSERT_TRUE(mgr.waitForResponse(id, resp, 5000));
    EXPECT_EQ(resp.status, SDO::SDOStatus::Complete);
    EXPECT_EQ(resp.operation, SDO::SDOOperation::Download);
    EXPECT_EQ(resp.data[0], 0x0F);
    mgr.deinit();
}

TEST_F(CoEPendingFuturesTest, TransportFailureYieldsAbortCode) {
    ON_CALL(transport_, sdoUpload(_, _, _, _, _, _, _, _, _, _, _, _, _, _))
        .WillByDefault(::testing::Return(false));

    CoE::CoEManager mgr(0, transport_);
    mgr.configureMailbox(0x1000, 128, 0x1400, 128);
    ASSERT_TRUE(mgr.init());

    SDO::SDORequest req{};
    req.slave_index = 0;
    req.index = 0x1018;
    req.subindex = 1;
    req.operation = SDO::SDOOperation::Upload;
    req.timeout_ms = 500;

    const uint32_t id = mgr.queueRequest(req);
    ASSERT_NE(id, 0u);

    SDO::SDOResponse resp{};
    ASSERT_TRUE(mgr.waitForResponse(id, resp, 5000));
    EXPECT_EQ(resp.status, SDO::SDOStatus::Failed);
    EXPECT_NE(resp.abort_code, SDOAbortCode::Success);
    mgr.deinit();
}

TEST_F(CoEPendingFuturesTest, WaitForResponseTimesOutOnUnknownId) {
    CoE::CoEManager mgr(0, transport_);
    mgr.configureMailbox(0x1000, 128, 0x1400, 128);
    ASSERT_TRUE(mgr.init());

    SDO::SDOResponse resp{};
    EXPECT_FALSE(mgr.waitForResponse(0xDEAD, resp, 30));
    mgr.deinit();
}

TEST_F(CoEPendingFuturesTest, PendingCountTracksInFlightRequests) {
    CoE::CoEManager mgr(0, transport_);
    mgr.configureMailbox(0x1000, 128, 0x1400, 128);
    ASSERT_TRUE(mgr.init());

    SDO::SDORequest req{};
    req.slave_index = 0;
    req.index = 0x1018;
    req.subindex = 1;
    req.operation = SDO::SDOOperation::Upload;

    const uint32_t id = mgr.queueRequest(req);
    ASSERT_NE(id, 0u);

    // Drain it
    SDO::SDOResponse resp{};
    ASSERT_TRUE(mgr.waitForResponse(id, resp, 5000));
    EXPECT_EQ(mgr.pendingCount(), 0u);
    mgr.deinit();
}

}  // namespace
