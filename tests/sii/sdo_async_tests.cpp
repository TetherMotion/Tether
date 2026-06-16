#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include <thread>
#include <chrono>

#include "tether/ethercat/SDOManager.hpp"

using namespace EtherCAT;
using namespace std::chrono_literals;
using ::testing::_;
using ::testing::Return;

namespace {

// Mock transport for testing without network
class MockSDOTransport : public SDO::ISDOTransport {
public:
    MOCK_METHOD(uint64_t, getMicroseconds, (), (override));
    MOCK_METHOD(bool, sdoUpload, 
        (uint16_t slave_index, uint8_t* mbx_counter,
         uint16_t mbx_wr_addr, uint16_t mbx_wr_len,
         uint16_t mbx_rd_addr, uint16_t mbx_rd_len,
         uint16_t index, uint8_t sub,
         uint8_t* out, size_t out_cap, size_t* out_len),
        (override));
    MOCK_METHOD(bool, sdoDownload,
        (uint16_t slave_index, uint8_t* mbx_counter,
         uint16_t mbx_wr_addr, uint16_t mbx_wr_len,
         uint16_t mbx_rd_addr, uint16_t mbx_rd_len,
         uint16_t index, uint8_t sub,
         const uint8_t* data, size_t data_len),
        (override));
};

} // anonymous namespace

TEST(SDO_Async, QueueRequestAndCompletion) {
    auto transport = std::make_shared<MockSDOTransport>();
    
    // Mock transport calls will fail (return false)
    ON_CALL(*transport, sdoUpload(_, _, _, _, _, _, _, _, _, _, _))
        .WillByDefault(Return(false));
    
    SDO::SDOManager mgr(*transport);
    
    // Initialize SDO manager
    ASSERT_TRUE(mgr.init());

    // Ensure no pending requests
    EXPECT_EQ(mgr.pendingCount(), 0u);

    // Prepare a simple upload request (mock transport will fail)
    SDO::SDORequest req = {};
    req.slave_index = 0;
    req.index = 0x1234;
    req.subindex = 0;
    req.operation = SDO::SDOOperation::Upload;
    req.data_size = 16;

    uint32_t id = mgr.queueRequest(req);
    EXPECT_NE(id, 0u);

    // Wait up to 500ms for processing task to complete and publish a response
    bool completed = false;
    for (int i = 0; i < 20; ++i) {
        if (mgr.isComplete(id)) { completed = true; break; }
        std::this_thread::sleep_for(25ms);
    }
    EXPECT_TRUE(completed);

    SDO::SDOResponse resp = {};
    EXPECT_TRUE(mgr.getResponse(id, resp));

    // Since transport returns false, request should have failed
    EXPECT_NE(resp.status, SDO::SDOStatus::Complete);

    // Verify pending count eventually returns to zero
    EXPECT_EQ(mgr.pendingCount(), 0u);

    mgr.deinit();
}

TEST(SDO_Async, SyncReadFailsSafely) {
    auto transport = std::make_shared<MockSDOTransport>();
    
    // Mock transport will fail
    ON_CALL(*transport, sdoUpload(_, _, _, _, _, _, _, _, _, _, _))
        .WillByDefault(Return(false));
    
    SDO::SDOManager mgr(*transport);
    ASSERT_TRUE(mgr.init());

    uint8_t out[8] = {};
    size_t actual = 0;
    // No configured mailbox => read should return false
    EXPECT_FALSE(mgr.readSync(0, 0x6040, 0, out, sizeof(out), 50, &actual));

    mgr.deinit();
}

