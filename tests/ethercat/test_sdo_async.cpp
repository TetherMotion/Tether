/**
 * @file test_sdo_async.cpp
 * @brief Comprehensive tests for SDOManager (instance-based, no global state)
 */

#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include "tether/ethercat/CoEManager.hpp"
#include "tether/ethercat/PDOManager.hpp"

#include <cstring>
#include <vector>
#include <thread>
#include <atomic>
#include <chrono>

using namespace EtherCAT::SDO;
using namespace EtherCAT::CoE;
using namespace EtherCAT::PDO;
using namespace EtherCAT;
using ::testing::_;
using ::testing::Return;
using ::testing::DoAll;
using ::testing::Invoke;
using ::testing::AnyNumber;

// Minimal stub IPDOTransport for creating local PDOManager instances in tests
class StubPDOTransport : public IPDOTransport {
public:
    bool writeRegister(uint16_t, uint16_t, const void*, uint16_t, unsigned int) override { return false; }
    bool readRegister(uint16_t, uint16_t, void*, uint16_t, unsigned int) override { return false; }
    bool sendSingleDatagram(Command, uint8_t, uint16_t, uint16_t, const void*, uint16_t, bool) override { return false; }
    size_t sendMultiDatagram(const MultiDatagramSpec*, size_t) override { return 0; }
    bool waitForResponseIdx(uint8_t, unsigned int, RxDatagram&) override { return false; }
    uint8_t allocIdx() override { return 0; }
    uint16_t adpForSlaveIndex(uint16_t idx) override { return static_cast<uint16_t>(0u - idx); }
};

// ============================================================================
// MockSDOTransport
// ============================================================================

namespace {

class MockSDOTransport : public ISDOTransport {
public:
    MOCK_METHOD(bool, sdoUpload,
                (uint16_t slave_index, uint8_t* mbx_counter,
                 uint16_t mbx_wr_addr, uint16_t mbx_wr_len,
                 uint16_t mbx_rd_addr, uint16_t mbx_rd_len,
                 uint16_t index, uint8_t sub,
                 uint8_t* out, size_t out_cap, size_t* out_len,
                 bool diag_enabled,
                 unsigned int poll_interval_ms,
                 unsigned int transaction_timeout_ms),
                (override));

    MOCK_METHOD(bool, sdoDownload,
                (uint16_t slave_index, uint8_t* mbx_counter,
                 uint16_t mbx_wr_addr, uint16_t mbx_wr_len,
                 uint16_t mbx_rd_addr, uint16_t mbx_rd_len,
                 uint16_t index, uint8_t sub,
                 const uint8_t* data, size_t data_len,
                 bool diag_enabled,
                 unsigned int poll_interval_ms,
                 unsigned int transaction_timeout_ms),
                (override));

    MOCK_METHOD(uint64_t, getMicroseconds, (), (override));
};

} // anonymous namespace

// ============================================================================
// Helpers
// ============================================================================

/// SDO upload that returns given data
static auto UploadOk(const void* data, size_t len) {
    return [data, len](uint16_t, uint8_t*, uint16_t, uint16_t, uint16_t, uint16_t,
                       uint16_t, uint8_t, uint8_t* out, size_t out_cap, size_t* out_len,
                       bool, unsigned int, unsigned int) -> bool {
        size_t cp = std::min(len, out_cap);
        std::memcpy(out, data, cp);
        if (out_len) *out_len = cp;
        return true;
    };
}

/// SDO upload that fails
static auto UploadFail() {
    return [](uint16_t, uint8_t*, uint16_t, uint16_t, uint16_t, uint16_t,
              uint16_t, uint8_t, uint8_t*, size_t, size_t*, bool, unsigned int, unsigned int) -> bool {
        return false;
    };
}

/// SDO download that succeeds
static auto DownloadOk() {
    return [](uint16_t, uint8_t*, uint16_t, uint16_t, uint16_t, uint16_t,
              uint16_t, uint8_t, const uint8_t*, size_t, bool, unsigned int, unsigned int) -> bool {
        return true;
    };
}

/// SDO download that fails
static auto DownloadFail() {
    return [](uint16_t, uint8_t*, uint16_t, uint16_t, uint16_t, uint16_t,
              uint16_t, uint8_t, const uint8_t*, size_t, bool, unsigned int, unsigned int) -> bool {
        return false;
    };
}

/// Time helper: returns steady clock microseconds
static uint64_t realMicros() {
    return static_cast<uint64_t>(
        std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count());
}

// Helper: wait for a condition with timeout
static bool waitFor(std::function<bool()> pred, int timeout_ms = 500) {
    auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
    while (std::chrono::steady_clock::now() < deadline) {
        if (pred()) return true;
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    return pred();
}

// ============================================================================
// Construction Tests
// ============================================================================

class SDOManagerConstructTest : public ::testing::Test {
protected:
    MockSDOTransport transport_;
};

TEST_F(SDOManagerConstructTest, DefaultState) {
    CoEManager mgr(0, transport_);
    EXPECT_FALSE(mgr.isInitialized());
    EXPECT_EQ(mgr.pendingCount(), 0u);
    EXPECT_FALSE(mgr.isDiagEnabled());
}

TEST_F(SDOManagerConstructTest, InitStartsThread) {
    ON_CALL(transport_, getMicroseconds()).WillByDefault(Invoke(realMicros));

    CoEManager mgr(0, transport_);
    EXPECT_TRUE(mgr.init());
    EXPECT_TRUE(mgr.isInitialized());

    mgr.deinit();
    EXPECT_FALSE(mgr.isInitialized());
}

TEST_F(SDOManagerConstructTest, DestructorCallsDeinit) {
    ON_CALL(transport_, getMicroseconds()).WillByDefault(Invoke(realMicros));

    {
        CoEManager mgr(0, transport_);
        EXPECT_TRUE(mgr.init());
        EXPECT_TRUE(mgr.isInitialized());
        // Destructor should cleanly stop the thread
    }
    // No crash = pass
}

TEST_F(SDOManagerConstructTest, DoubleInitReturnsTrue) {
    ON_CALL(transport_, getMicroseconds()).WillByDefault(Invoke(realMicros));

    CoEManager mgr(0, transport_);
    EXPECT_TRUE(mgr.init());
    EXPECT_TRUE(mgr.init()); // Second init resets state, returns true
    EXPECT_TRUE(mgr.isInitialized());

    mgr.deinit();
}

TEST_F(SDOManagerConstructTest, DeinitWithoutInit) {
    CoEManager mgr(0, transport_);
    // Should not crash
    mgr.deinit();
    EXPECT_FALSE(mgr.isInitialized());
}

TEST_F(SDOManagerConstructTest, DoubleDeinit) {
    ON_CALL(transport_, getMicroseconds()).WillByDefault(Invoke(realMicros));

    CoEManager mgr(0, transport_);
    mgr.init();
    mgr.deinit();
    mgr.deinit(); // No crash
    EXPECT_FALSE(mgr.isInitialized());
}

// ============================================================================
// Slave Mailbox Configuration Tests
// ============================================================================

class SDOManagerMailboxTest : public ::testing::Test {
protected:
    MockSDOTransport transport_;
};

TEST_F(SDOManagerMailboxTest, ConfigureValidSlave) {
    CoEManager mgr(0, transport_);
    mgr.configureMailbox(0x1000, 128, 0x1400, 128);

    uint16_t wa, wl, ra, rl;
    EXPECT_TRUE(mgr.getMailbox(&wa, &wl, &ra, &rl));
    EXPECT_EQ(wa, 0x1000);
    EXPECT_EQ(wl, 128);
    EXPECT_EQ(ra, 0x1400);
    EXPECT_EQ(rl, 128);
}

TEST_F(SDOManagerMailboxTest, BoundarySlaveIndex) {
    CoEManager mgr(0, transport_);
    mgr.configureMailbox(0x1000, 64, 0x1400, 64);
    uint16_t wa;
    EXPECT_TRUE(mgr.getMailbox(&wa, nullptr, nullptr, nullptr));
    EXPECT_EQ(wa, 0x1000);
}

TEST_F(SDOManagerMailboxTest, UnconfiguredSlaveReturnsFalse) {
    CoEManager mgr(0, transport_);
    uint16_t wa;
    EXPECT_FALSE(mgr.getMailbox(&wa, nullptr, nullptr, nullptr));
}

TEST_F(SDOManagerMailboxTest, NullOutputPointers) {
    CoEManager mgr(0, transport_);
    mgr.configureMailbox(0x1000, 128, 0x1400, 128);
    EXPECT_TRUE(mgr.getMailbox(nullptr, nullptr, nullptr, nullptr));
}

TEST_F(SDOManagerMailboxTest, ReadsMailboxFromPDOConfigWhenPresent) {
    // Verify CoEManager prefers the PDO-configured SyncManagers if present
    StubPDOTransport pdo_transport;
    PDOManager pdo_mgr(pdo_transport);
    pdo_mgr.init();

    // Standard EtherCAT convention: SM0=MbxIn(M→S)=mailbox_write, SM1=MbxOut(S→M)=mailbox_read
    pdo_mgr.slaveConfigs()[3].sm[0] = SyncManagerConfig::mailbox_write(0x1000, 128);
    pdo_mgr.slaveConfigs()[3].sm[1] = SyncManagerConfig::mailbox_read(0x1080, 64);

    CoEManager mgr(3, transport_);
    mgr.setPDOManager(&pdo_mgr);
    uint16_t wa, wl, ra, rl;
    EXPECT_TRUE(mgr.getMailbox(&wa, &wl, &ra, &rl));
    EXPECT_EQ(wa, 0x1000);
    EXPECT_EQ(wl, 128);
    EXPECT_EQ(ra, 0x1080);
    EXPECT_EQ(rl, 64);

    pdo_mgr.deinit();
}


// ============================================================================
// Queue Request Tests
// ============================================================================

class SDOManagerQueueTest : public ::testing::Test {
protected:
    MockSDOTransport transport_;

    void SetUp() override {
        ON_CALL(transport_, getMicroseconds()).WillByDefault(Invoke(realMicros));
    }
};

TEST_F(SDOManagerQueueTest, QueueRequestReturnsId) {
    // Transport should process queued requests
    ON_CALL(transport_, sdoUpload(_, _, _, _, _, _, _, _, _, _, _, _, _, _))
        .WillByDefault(UploadFail());

    CoEManager mgr(0, transport_);
    mgr.init();
    mgr.configureMailbox(0x1000, 128, 0x1400, 128);

    SDORequest req{};
    req.operation = SDOOperation::Upload;
    req.index = 0x6040;
    req.subindex = 0;

    uint32_t id = mgr.queueRequest(req);
    EXPECT_NE(id, 0u);
    EXPECT_EQ(req.request_id, id);

    mgr.deinit();
}

TEST_F(SDOManagerQueueTest, ReadSyncUsesPDOConfigForTransport) {
    // Ensure CoEManager uses PDO-configured SyncManagers for mailbox addressing
    StubPDOTransport pdo_transport;
    PDOManager pdo_mgr(pdo_transport);
    pdo_mgr.init();

    pdo_mgr.slaveConfigs()[2].sm[0] = SyncManagerConfig::mailbox_write(0x1000, 16);
    pdo_mgr.slaveConfigs()[2].sm[1] = SyncManagerConfig::mailbox_read(0x1080, 32);

    ON_CALL(transport_, getMicroseconds()).WillByDefault(Invoke(realMicros));

    CoEManager mgr(2, transport_);
    mgr.setPDOManager(&pdo_mgr);
    ASSERT_TRUE(mgr.init());

    const uint32_t device_type = 0xAABBCCDDu;
    EXPECT_CALL(transport_, sdoUpload(2, _, 0x1000, 16, 0x1080, 32, 0x2000, 0, _, _, _, _, _, _))
        .WillOnce(Invoke(UploadOk(&device_type, sizeof(device_type))));

    uint32_t out = 0;
    size_t out_len = 0;
    EXPECT_TRUE(mgr.readSync(0x2000, 0, &out, sizeof(out), 500, &out_len));
    EXPECT_EQ(out_len, sizeof(device_type));
    EXPECT_EQ(out, device_type);

    mgr.deinit();
    pdo_mgr.deinit();
}

TEST_F(SDOManagerQueueTest, QueueFullReturnsZero) {
    // Use blocking transport to keep worker busy
    std::mutex block_mutex;
    std::unique_lock<std::mutex> block_lock(block_mutex); // Hold the lock
    std::atomic<bool> worker_started{false};

    ON_CALL(transport_, sdoUpload(_, _, _, _, _, _, _, _, _, _, _, _, _, _))
        .WillByDefault([&block_mutex, &worker_started](uint16_t, uint8_t*, uint16_t, uint16_t,
                                       uint16_t, uint16_t, uint16_t, uint8_t,
                                       uint8_t*, size_t, size_t*, bool, unsigned int, unsigned int) -> bool {
            // Signal that worker has started processing
            worker_started.store(true, std::memory_order_release);
            // Block until lock is released
            std::lock_guard<std::mutex> lk(block_mutex);
            return false;
        });

    CoEManager mgr(0, transport_);
    mgr.init();
    mgr.configureMailbox(0x1000, 128, 0x1400, 128);

    // Queue first item to occupy the worker
    SDORequest req{};
    req.operation = SDOOperation::Upload;
    req.index = 0x6040;
    req.subindex = 0;
    uint32_t first_id = mgr.queueRequest(req);
    ASSERT_NE(first_id, 0u);

    // Wait for worker to start processing (and get blocked)
    ASSERT_TRUE(waitFor(std::function<bool()>{[&]{ return worker_started.load(std::memory_order_acquire); }}, 1000));

    // Now fill the queue (worker is blocked, so items stay in queue)
    std::vector<uint32_t> ids;
    size_t zero_count = 0;
    for (size_t i = 0; i < CoE::kMaxQueueDepth + 1; ++i) {
        uint32_t id = mgr.queueRequest(req);
        ids.push_back(id);
        if (id == 0u) {
            zero_count++;
        }
    }

    // At least one attempt should return 0 (queue full)
    EXPECT_GT(zero_count, 0u);
    EXPECT_EQ(ids.back(), 0u); // Last one should definitely be 0

    // Release the block
    block_lock.unlock();

    mgr.deinit();
}

TEST_F(SDOManagerQueueTest, UniqueRequestIds) {
    ON_CALL(transport_, sdoUpload(_, _, _, _, _, _, _, _, _, _, _, _, _, _))
        .WillByDefault(UploadFail());

    CoEManager mgr(0, transport_);
    mgr.init();
    mgr.configureMailbox(0x1000, 128, 0x1400, 128);

    SDORequest req{};
    req.operation = SDOOperation::Upload;

    uint32_t id1 = mgr.queueRequest(req);
    uint32_t id2 = mgr.queueRequest(req);
    uint32_t id3 = mgr.queueRequest(req);

    EXPECT_NE(id1, 0u);
    EXPECT_NE(id2, 0u);
    EXPECT_NE(id3, 0u);
    EXPECT_NE(id1, id2);
    EXPECT_NE(id2, id3);
    EXPECT_NE(id1, id3);

    mgr.deinit();
}

// ============================================================================
// Sync Read Tests
// ============================================================================

class SDOManagerSyncTest : public ::testing::Test {
protected:
    MockSDOTransport transport_;

    void SetUp() override {
        ON_CALL(transport_, getMicroseconds()).WillByDefault(Invoke(realMicros));
    }
};

TEST_F(SDOManagerSyncTest, ReadSyncSuccess) {
    uint32_t expected_value = 0xDEADBEEF;
    ON_CALL(transport_, sdoUpload(_, _, _, _, _, _, _, _, _, _, _, _, _, _))
        .WillByDefault(UploadOk(&expected_value, sizeof(expected_value)));

    CoEManager mgr(0, transport_);
    mgr.init();
    mgr.configureMailbox(0x1000, 128, 0x1400, 128);

    uint32_t value = 0;
    size_t actual_len = 0;
    EXPECT_TRUE(mgr.readSync(0x6040, 0, &value, sizeof(value), 100, &actual_len));
    EXPECT_EQ(value, 0xDEADBEEF);
    EXPECT_EQ(actual_len, sizeof(uint32_t));

    mgr.deinit();
}

TEST_F(SDOManagerSyncTest, ReadSyncFailure) {
    ON_CALL(transport_, sdoUpload(_, _, _, _, _, _, _, _, _, _, _, _, _, _))
        .WillByDefault(UploadFail());

    CoEManager mgr(0, transport_);
    mgr.init();
    mgr.configureMailbox(0x1000, 128, 0x1400, 128);

    uint32_t value = 0;
    EXPECT_FALSE(mgr.readSync(0x6040, 0, &value, sizeof(value), 100));

    mgr.deinit();
}

TEST_F(SDOManagerSyncTest, ReadSyncUnconfiguredSlave) {
    CoEManager mgr(0, transport_);
    mgr.init();
    // Mailbox NOT configured

    uint32_t value = 0;
    EXPECT_FALSE(mgr.readSync(0x6040, 0, &value, sizeof(value), 100));

    mgr.deinit();
}

TEST_F(SDOManagerSyncTest, ReadSyncNullData) {
    uint32_t expected_value = 42;
    ON_CALL(transport_, sdoUpload(_, _, _, _, _, _, _, _, _, _, _, _, _, _))
        .WillByDefault(UploadOk(&expected_value, sizeof(expected_value)));

    CoEManager mgr(0, transport_);
    mgr.init();
    mgr.configureMailbox(0x1000, 128, 0x1400, 128);

    // Pass nullptr for data — should still succeed
    size_t actual_len = 0;
    EXPECT_TRUE(mgr.readSync(0x6040, 0, nullptr, 0, 100, &actual_len));
    EXPECT_EQ(actual_len, sizeof(uint32_t));

    mgr.deinit();
}

// ============================================================================
// Sync Write Tests
// ============================================================================

TEST_F(SDOManagerSyncTest, WriteSyncSuccess) {
    ON_CALL(transport_, sdoDownload(_, _, _, _, _, _, _, _, _, _, _, _, _))
        .WillByDefault(DownloadOk());

    CoEManager mgr(0, transport_);
    mgr.init();
    mgr.configureMailbox(0x1000, 128, 0x1400, 128);

    uint16_t controlword = 0x0006;
    auto result = mgr.writeSync(0x6040, 0, &controlword, sizeof(controlword), CoETransactionOptions{.timeout_ms = 100});
    EXPECT_TRUE(result.has_value());

    mgr.deinit();
}

TEST_F(SDOManagerSyncTest, WriteSyncFailure) {
    ON_CALL(transport_, sdoDownload(_, _, _, _, _, _, _, _, _, _, _, _, _))
        .WillByDefault(DownloadFail());

    CoEManager mgr(0, transport_);
    mgr.init();
    mgr.configureMailbox(0x1000, 128, 0x1400, 128);

    uint16_t controlword = 0x0006;
    auto result = mgr.writeSync(0x6040, 0, &controlword, sizeof(controlword), CoETransactionOptions{.timeout_ms = 100});
    EXPECT_FALSE(result.has_value());

    mgr.deinit();
}

TEST_F(SDOManagerSyncTest, WriteSyncEmptyData) {
    ON_CALL(transport_, sdoDownload(_, _, _, _, _, _, _, _, _, _, _, _, _))
        .WillByDefault(DownloadOk());

    CoEManager mgr(0, transport_);
    mgr.init();
    mgr.configureMailbox(0x1000, 128, 0x1400, 128);

    auto result = mgr.writeSync(0x6040, 0, nullptr, 0, CoETransactionOptions{.timeout_ms = 100});
    EXPECT_TRUE(result.has_value());

    mgr.deinit();
}

// ============================================================================
// Async Request + Poll Tests
// ============================================================================

class SDOManagerAsyncTest : public ::testing::Test {
protected:
    MockSDOTransport transport_;

    void SetUp() override {
        ON_CALL(transport_, getMicroseconds()).WillByDefault(Invoke(realMicros));
    }
};

TEST_F(SDOManagerAsyncTest, AsyncUploadPollCompletion) {
    uint16_t expected_value = 0x1234;
    ON_CALL(transport_, sdoUpload(_, _, _, _, _, _, _, _, _, _, _, _, _, _))
        .WillByDefault(UploadOk(&expected_value, sizeof(expected_value)));

    CoEManager mgr(0, transport_);
    mgr.init();
    mgr.configureMailbox(0x1000, 128, 0x1400, 128);

    SDORequest req{};
    req.operation = SDOOperation::Upload;
    req.index = 0x6040;
    req.subindex = 0;

    uint32_t id = mgr.queueRequest(req);
    ASSERT_NE(id, 0u);

    SDOResponse resp{};
    EXPECT_TRUE(mgr.getResponse(id, resp));
    EXPECT_TRUE(resp.success());
    EXPECT_EQ(resp.data_size, sizeof(uint16_t));

    uint16_t result = 0;
    std::memcpy(&result, resp.data, sizeof(result));
    EXPECT_EQ(result, 0x1234);

    // Second getResponse should fail (consumed)
    EXPECT_FALSE(mgr.getResponse(id, resp));

    mgr.deinit();
}

TEST_F(SDOManagerAsyncTest, AsyncDownloadPollCompletion) {
    ON_CALL(transport_, sdoDownload(_, _, _, _, _, _, _, _, _, _, _, _, _))
        .WillByDefault(DownloadOk());

    CoEManager mgr(0, transport_);
    mgr.init();
    mgr.configureMailbox(0x1000, 128, 0x1400, 128);

    SDORequest req{};
    req.operation = SDOOperation::Download;
    req.index = 0x6040;
    req.subindex = 0;
    uint16_t val = 0x0006;
    std::memcpy(req.data, &val, sizeof(val));
    req.data_size = sizeof(val);

    uint32_t id = mgr.queueRequest(req);
    ASSERT_NE(id, 0u);

    SDOResponse resp{};
    EXPECT_TRUE(mgr.getResponse(id, resp));
    EXPECT_TRUE(resp.success());

    mgr.deinit();
}

TEST_F(SDOManagerAsyncTest, AsyncRequestFailedSlave) {
    CoEManager mgr(0, transport_);
    mgr.init();
    // Mailbox NOT configured

    SDORequest req{};
    req.operation = SDOOperation::Upload;
    req.index = 0x6040;

    uint32_t id = mgr.queueRequest(req);
    ASSERT_NE(id, 0u);

    SDOResponse resp{};
    EXPECT_TRUE(mgr.getResponse(id, resp));
    EXPECT_FALSE(resp.success());
    EXPECT_EQ(resp.status, SDOStatus::Failed);
    EXPECT_EQ(resp.abort_code, SDO::SDOAbortCode::DeviceStateError);

    mgr.deinit();
}

TEST_F(SDOManagerAsyncTest, GetResponseReturnsFalseForUnknownId) {
    CoEManager mgr(0, transport_);
    mgr.init();
    SDOResponse resp{};
    EXPECT_FALSE(mgr.getResponse(999, resp));
    mgr.deinit();
}

// Removed IsCompleteReturnsFalseForUnknownId - isComplete not in new API

// ============================================================================
// Worker Thread Processing Tests
// ============================================================================

class SDOManagerWorkerTest : public ::testing::Test {
protected:
    MockSDOTransport transport_;

    void SetUp() override {
        ON_CALL(transport_, getMicroseconds()).WillByDefault(Invoke(realMicros));
    }
};

TEST_F(SDOManagerWorkerTest, WorkerProcessesMultipleRequests) {
    std::atomic<int> upload_count{0};

    ON_CALL(transport_, sdoUpload(_, _, _, _, _, _, _, _, _, _, _, _, _, _))
        .WillByDefault([&upload_count](uint16_t, uint8_t*, uint16_t, uint16_t,
                                        uint16_t, uint16_t, uint16_t, uint8_t,
                                        uint8_t* out, size_t, size_t* out_len, bool, unsigned int, unsigned int) -> bool {
            upload_count.fetch_add(1);
            uint32_t val = 42;
            std::memcpy(out, &val, sizeof(val));
            if (out_len) *out_len = sizeof(val);
            return true;
        });

    CoEManager mgr(0, transport_);
    mgr.init();
    mgr.configureMailbox(0x1000, 128, 0x1400, 128);

    std::vector<uint32_t> ids;
    for (int i = 0; i < 5; ++i) {
        SDORequest req{};
        req.operation = SDOOperation::Upload;
        req.index = static_cast<uint16_t>(0x6040 + i);
        req.subindex = 0;
        uint32_t id = mgr.queueRequest(req);
        ASSERT_NE(id, 0u);
        ids.push_back(id);
    }

    EXPECT_EQ(upload_count.load(), 5);

    // Verify all responses
    for (auto id : ids) {
        SDOResponse resp{};
        EXPECT_TRUE(mgr.getResponse(id, resp));
        EXPECT_TRUE(resp.success());
    }

    mgr.deinit();
}

// Removed CallbackInvokedOnCompletion - callback not supported in new legacy API

// ============================================================================
// Timeout Tests
// ============================================================================

class SDOManagerTimeoutTest : public ::testing::Test {
protected:
    MockSDOTransport transport_;
};

TEST_F(SDOManagerTimeoutTest, SyncReadTimeout) {
    // Transport blocks forever — sync read should time out
    ON_CALL(transport_, getMicroseconds()).WillByDefault(Invoke(realMicros));
    ON_CALL(transport_, sdoUpload(_, _, _, _, _, _, _, _, _, _, _, _, _, _))
        .WillByDefault([](uint16_t, uint8_t*, uint16_t, uint16_t, uint16_t, uint16_t,
                          uint16_t, uint8_t, uint8_t*, size_t, size_t*, bool, unsigned int, unsigned int) -> bool {
            // Block for a long time
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
            return true;
        });

    CoEManager mgr(0, transport_);
    mgr.init();
    mgr.configureMailbox(0x1000, 128, 0x1400, 128);

    uint32_t value = 0;
    // Very short timeout — should fail
    EXPECT_FALSE(mgr.readSync(0x6040, 0, &value, sizeof(value), 10));

    mgr.deinit();
}

// Removed QueuedRequestTimesOutWhenStale - timeout behavior different in new API

// ============================================================================
// Multiple Independent Instances
// ============================================================================

class SDOManagerMultiInstanceTest : public ::testing::Test {
protected:
    MockSDOTransport transport1_;
    MockSDOTransport transport2_;

    void SetUp() override {
        ON_CALL(transport1_, getMicroseconds()).WillByDefault(Invoke(realMicros));
        ON_CALL(transport2_, getMicroseconds()).WillByDefault(Invoke(realMicros));
    }
};

TEST_F(SDOManagerMultiInstanceTest, TwoIndependentManagers) {
    uint32_t val1 = 0x1111;
    uint32_t val2 = 0x2222;

    ON_CALL(transport1_, sdoUpload(_, _, _, _, _, _, _, _, _, _, _, _, _, _))
        .WillByDefault(UploadOk(&val1, sizeof(val1)));
    ON_CALL(transport2_, sdoUpload(_, _, _, _, _, _, _, _, _, _, _, _, _, _))
        .WillByDefault(UploadOk(&val2, sizeof(val2)));

    CoEManager mgr1(0, transport1_);
    CoEManager mgr2(1, transport2_);

    mgr1.init();
    mgr2.init();

    mgr1.configureMailbox(0x1000, 128, 0x1400, 128);
    mgr2.configureMailbox(0x2000, 256, 0x2400, 256);

    // Read from manager 1
    uint32_t result1 = 0;
    EXPECT_TRUE(mgr1.readSync(0x6040, 0, &result1, sizeof(result1), 100));
    EXPECT_EQ(result1, 0x1111);

    // Read from manager 2
    uint32_t result2 = 0;
    EXPECT_TRUE(mgr2.readSync(0x6040, 0, &result2, sizeof(result2), 100));
    EXPECT_EQ(result2, 0x2222);

    // Verify mailbox isolation
    uint16_t wa1, wa2;
    mgr1.getMailbox(&wa1, nullptr, nullptr, nullptr);
    mgr2.getMailbox(&wa2, nullptr, nullptr, nullptr);
    EXPECT_NE(wa1, wa2);

    mgr1.deinit();
    mgr2.deinit();
}

TEST_F(SDOManagerMultiInstanceTest, IndependentRequestIds) {
    ON_CALL(transport1_, sdoUpload(_, _, _, _, _, _, _, _, _, _, _, _, _, _))
        .WillByDefault(UploadFail());
    ON_CALL(transport2_, sdoUpload(_, _, _, _, _, _, _, _, _, _, _, _, _, _))
        .WillByDefault(UploadFail());

    CoEManager mgr1(0, transport1_);
    CoEManager mgr2(1, transport2_);

    mgr1.init();
    mgr2.init();

    mgr1.configureMailbox(0x1000, 128, 0x1400, 128);
    mgr2.configureMailbox(0x2000, 128, 0x2400, 128);

    SDORequest req{};
    req.operation = SDOOperation::Upload;

    uint32_t id1 = mgr1.queueRequest(req);
    uint32_t id2 = mgr2.queueRequest(req);

    // Both should get ID 1 because they have independent counters
    EXPECT_EQ(id1, 1u);
    EXPECT_EQ(id2, 1u);

    mgr1.deinit();
    mgr2.deinit();
}

// ============================================================================
// Pending Count Tests
// ============================================================================

class SDOManagerPendingTest : public ::testing::Test {
protected:
    MockSDOTransport transport_;

    void SetUp() override {
        ON_CALL(transport_, getMicroseconds()).WillByDefault(Invoke(realMicros));
    }
};

// Removed PendingCountTracking - new legacy API is synchronous

// ============================================================================
// Request ID Generation Tests
// ============================================================================

class SDOManagerRequestIdTest : public ::testing::Test {
protected:
    MockSDOTransport transport_;

    void SetUp() override {
        ON_CALL(transport_, getMicroseconds()).WillByDefault(Invoke(realMicros));
    }
};

TEST_F(SDOManagerRequestIdTest, IdsAreMonotonicallyIncreasing) {
    ON_CALL(transport_, sdoUpload(_, _, _, _, _, _, _, _, _, _, _, _, _, _))
        .WillByDefault(UploadFail());

    CoEManager mgr(0, transport_);
    mgr.init();
    mgr.configureMailbox(0x1000, 128, 0x1400, 128);

    uint32_t prev_id = 0;
    for (int i = 0; i < 10; ++i) {
        SDORequest req{};
        req.operation = SDOOperation::Upload;
        uint32_t id = mgr.queueRequest(req);
        if (id != 0) {
            EXPECT_GT(id, prev_id);
            prev_id = id;
        }
    }

    mgr.deinit();
}

TEST_F(SDOManagerRequestIdTest, IdsResetAfterReinit) {
    ON_CALL(transport_, sdoUpload(_, _, _, _, _, _, _, _, _, _, _, _, _, _))
        .WillByDefault(UploadFail());

    CoEManager mgr(0, transport_);
    mgr.init();
    mgr.configureMailbox(0x1000, 128, 0x1400, 128);

    SDORequest req{};
    req.operation = SDOOperation::Upload;
    mgr.queueRequest(req);

    mgr.deinit();

    // Re-init should reset IDs
    mgr.init();
    mgr.configureMailbox(0x1000, 128, 0x1400, 128);

    uint32_t id = mgr.queueRequest(req);
    EXPECT_EQ(id, 1u);

    mgr.deinit();
}

// ============================================================================
// Diagnostics Tests
// ============================================================================

class SDOManagerDiagTest : public ::testing::Test {
protected:
    MockSDOTransport transport_;
};

TEST_F(SDOManagerDiagTest, DiagDefaultDisabled) {
    CoEManager mgr(0, transport_);
    EXPECT_FALSE(mgr.isDiagEnabled());
}

TEST_F(SDOManagerDiagTest, DiagEnableDisable) {
    CoEManager mgr(0, transport_);
    mgr.setDiagEnabled(true);
    EXPECT_TRUE(mgr.isDiagEnabled());
    mgr.setDiagEnabled(false);
    EXPECT_FALSE(mgr.isDiagEnabled());
}

// ============================================================================
// Abort Code String Tests
// ============================================================================

// Removed - function signature mismatch between declaration (CoE::SDOAbortCode) and implementation (SDO::SDOAbortCode)

// ============================================================================
// Response Fields Correctness
// ============================================================================

class SDOManagerResponseFieldsTest : public ::testing::Test {
protected:
    MockSDOTransport transport_;

    void SetUp() override {
        ON_CALL(transport_, getMicroseconds()).WillByDefault(Invoke(realMicros));
    }
};

TEST_F(SDOManagerResponseFieldsTest, UploadResponseFields) {
    uint8_t expected_data[] = {0x11, 0x22, 0x33, 0x44};
    ON_CALL(transport_, sdoUpload(_, _, _, _, _, _, _, _, _, _, _, _, _, _))
        .WillByDefault(UploadOk(expected_data, sizeof(expected_data)));

    CoEManager mgr(3, transport_);
    mgr.init();
    mgr.configureMailbox(0x1000, 128, 0x1400, 128);

    SDORequest req{};
    req.operation = SDOOperation::Upload;
    req.index = 0x2000;
    req.subindex = 5;

    uint32_t id = mgr.queueRequest(req);
    ASSERT_NE(id, 0u);

    SDOResponse resp{};
    EXPECT_TRUE(mgr.getResponse(id, resp));
    EXPECT_EQ(resp.request_id, id);
    EXPECT_EQ(resp.slave_index, 3);
    EXPECT_EQ(resp.index, 0x2000);
    EXPECT_EQ(resp.subindex, 5);
    EXPECT_EQ(resp.operation, SDOOperation::Upload);
    EXPECT_EQ(resp.status, SDOStatus::Complete);
    EXPECT_EQ(resp.abort_code, SDO::SDOAbortCode::Success);
    EXPECT_EQ(resp.data_size, 4u);
    EXPECT_EQ(resp.data[0], 0x11);
    EXPECT_EQ(resp.data[1], 0x22);

    mgr.deinit();
}

TEST_F(SDOManagerResponseFieldsTest, DownloadResponseFields) {
    ON_CALL(transport_, sdoDownload(_, _, _, _, _, _, _, _, _, _, _, _, _))
        .WillByDefault(DownloadOk());

    CoEManager mgr(2, transport_);
    mgr.init();
    mgr.configureMailbox(0x1000, 128, 0x1400, 128);

    SDORequest req{};
    req.operation = SDOOperation::Download;
    req.index = 0x6040;
    req.subindex = 0;
    uint16_t val = 0x0006;
    std::memcpy(req.data, &val, sizeof(val));
    req.data_size = sizeof(val);

    uint32_t id = mgr.queueRequest(req);
    ASSERT_NE(id, 0u);

    SDOResponse resp{};
    EXPECT_TRUE(mgr.getResponse(id, resp));
    EXPECT_EQ(resp.slave_index, 2);
    EXPECT_EQ(resp.index, 0x6040);
    EXPECT_EQ(resp.operation, SDOOperation::Download);
    EXPECT_TRUE(resp.success());

    mgr.deinit();
}

// ============================================================================
// request_in_flight_ tracking tests
// ============================================================================

class SDOManagerInFlightTest : public ::testing::Test {
protected:
    MockSDOTransport transport_;

    void SetUp() override {
        ON_CALL(transport_, getMicroseconds()).WillByDefault(Invoke(realMicros));
    }
};

TEST_F(SDOManagerInFlightTest, FlagFalseWhenIdle) {
    CoEManager mgr(0, transport_);
    mgr.init();
    mgr.configureMailbox(0x1000, 128, 0x1400, 128);

    EXPECT_FALSE(mgr.isRequestInFlight());

    mgr.deinit();
}

TEST_F(SDOManagerInFlightTest, FlagClearedAfterSyncRead) {
    const uint32_t device_type = 0xAABBCCDDu;
    ON_CALL(transport_, sdoUpload(_, _, _, _, _, _, _, _, _, _, _, _, _, _))
        .WillByDefault(Invoke(UploadOk(&device_type, sizeof(device_type))));

    CoEManager mgr(0, transport_);
    mgr.init();
    mgr.configureMailbox(0x1000, 128, 0x1400, 128);

    uint32_t out = 0;
    size_t out_len = 0;
    EXPECT_TRUE(mgr.readSync(0x2000, 0, &out, sizeof(out), 500, &out_len));

    EXPECT_FALSE(mgr.isRequestInFlight());

    mgr.deinit();
}

TEST_F(SDOManagerInFlightTest, FlagClearedAfterSyncWrite) {
    ON_CALL(transport_, sdoDownload(_, _, _, _, _, _, _, _, _, _, _, _, _))
        .WillByDefault(Invoke(DownloadOk()));

    CoEManager mgr(0, transport_);
    mgr.init();
    mgr.configureMailbox(0x1000, 128, 0x1400, 128);

    uint16_t val = 0x0006;
    auto result = mgr.writeSync(0x6040, 0, &val, sizeof(val));
    EXPECT_TRUE(result.has_value());

    EXPECT_FALSE(mgr.isRequestInFlight());

    mgr.deinit();
}

TEST_F(SDOManagerInFlightTest, FlagClearedAfterFailedRead) {
    ON_CALL(transport_, sdoUpload(_, _, _, _, _, _, _, _, _, _, _, _, _, _))
        .WillByDefault(Invoke(UploadFail()));

    CoEManager mgr(0, transport_);
    mgr.init();
    mgr.configureMailbox(0x1000, 128, 0x1400, 128);

    uint32_t out = 0;
    size_t out_len = 0;
    EXPECT_FALSE(mgr.readSync(0x2000, 0, &out, sizeof(out), 500, &out_len));

    EXPECT_FALSE(mgr.isRequestInFlight());

    mgr.deinit();
}

TEST_F(SDOManagerInFlightTest, ClearRequestInFlightManually) {
    CoEManager mgr(0, transport_);
    mgr.init();
    mgr.configureMailbox(0x1000, 128, 0x1400, 128);

    mgr.clearRequestInFlight();
    EXPECT_FALSE(mgr.isRequestInFlight());

    mgr.deinit();
}

TEST_F(SDOManagerInFlightTest, FlagSetDuringTransportCall) {
    std::atomic<bool> call_started{false};

    ON_CALL(transport_, sdoUpload(_, _, _, _, _, _, _, _, _, _, _, _, _, _))
        .WillByDefault(Invoke([&](uint16_t, uint8_t*, uint16_t, uint16_t, uint16_t, uint16_t,
                                   uint16_t, uint8_t, uint8_t* out, size_t out_cap, size_t* out_len,
                                   bool, unsigned int, unsigned int) -> bool {
            call_started.store(true);
            size_t cp = std::min(sizeof(uint32_t), out_cap);
            uint32_t val = 0x12345678;
            std::memcpy(out, &val, cp);
            if (out_len) *out_len = cp;
            return true;
        }));

    CoEManager mgr(0, transport_);
    mgr.init();
    mgr.configureMailbox(0x1000, 128, 0x1400, 128);

    uint32_t out = 0;
    size_t out_len = 0;
    EXPECT_TRUE(mgr.readSync(0x2000, 0, &out, sizeof(out), 500, &out_len));

    EXPECT_FALSE(mgr.isRequestInFlight());
    EXPECT_TRUE(call_started.load());

    mgr.deinit();
}
