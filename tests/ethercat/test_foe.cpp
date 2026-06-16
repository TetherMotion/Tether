/**
 * @file test_foe.cpp
 * @brief Comprehensive tests for FoEManager (instance-based, no global state)
 */

#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include "tether/ethercat/FoE.hpp"

#include <cstring>
#include <vector>
#include <thread>
#include <atomic>

using namespace EtherCAT::FoE;
using ::testing::_;
using ::testing::Return;

// ============================================================================
// MockFoETransport
// ============================================================================

class MockFoETransport : public IFoETransport {
public:
    MOCK_METHOD(bool, mailboxWrite,
                (uint16_t slave_index, uint16_t mbx_wr_addr, uint16_t mbx_wr_len,
                 const uint8_t* data, size_t data_len, uint8_t* mbx_counter),
                (override));

    MOCK_METHOD(bool, mailboxRead,
                (uint16_t slave_index, uint16_t mbx_rd_addr, uint16_t mbx_rd_len,
                 uint8_t* data, size_t data_cap, size_t* data_len, uint32_t timeout_ms),
                (override));

    MOCK_METHOD(bool, fileOpen,
                (const char* path, bool for_write, uint32_t* file_size),
                (override));

    MOCK_METHOD(int32_t, fileRead, (void* buffer, size_t size), (override));
    MOCK_METHOD(int32_t, fileWrite, (const void* buffer, size_t size), (override));
    MOCK_METHOD(void, fileClose, (), (override));
    MOCK_METHOD(uint32_t, getTimeMs, (), (override));
    MOCK_METHOD(void, delayMs, (uint32_t ms), (override));
};

// ============================================================================
// String Helper Tests (stateless, no manager needed)
// ============================================================================

TEST(FoE_Strings, AllOpcodesCovered) {
    EXPECT_STREQ(foe_opcode_string(FoEOpcode::RRQ),   "RRQ");
    EXPECT_STREQ(foe_opcode_string(FoEOpcode::WRQ),   "WRQ");
    EXPECT_STREQ(foe_opcode_string(FoEOpcode::DATA),  "DATA");
    EXPECT_STREQ(foe_opcode_string(FoEOpcode::ACK),   "ACK");
    EXPECT_STREQ(foe_opcode_string(FoEOpcode::ERROR), "ERROR");
    EXPECT_STREQ(foe_opcode_string(FoEOpcode::BUSY),  "BUSY");
}

TEST(FoE_Strings, UnknownOpcode) {
    EXPECT_STREQ(foe_opcode_string(static_cast<FoEOpcode>(0xFF)), "UNKNOWN");
    EXPECT_STREQ(foe_opcode_string(static_cast<FoEOpcode>(0)),    "UNKNOWN");
    EXPECT_STREQ(foe_opcode_string(static_cast<FoEOpcode>(99)),   "UNKNOWN");
}

TEST(FoE_Strings, AllStandardErrorsCovered) {
    EXPECT_STREQ(foe_error_string(FoEError::SUCCESS),          "Success");
    EXPECT_STREQ(foe_error_string(FoEError::NOT_FOUND),        "File not found");
    EXPECT_STREQ(foe_error_string(FoEError::ACCESS_DENIED),    "Access denied");
    EXPECT_STREQ(foe_error_string(FoEError::DISK_FULL),        "Disk full");
    EXPECT_STREQ(foe_error_string(FoEError::ILLEGAL_OP),       "Illegal operation");
    EXPECT_STREQ(foe_error_string(FoEError::PACKET_NUM),       "Wrong packet number");
    EXPECT_STREQ(foe_error_string(FoEError::ALREADY_EXISTS),   "File already exists");
    EXPECT_STREQ(foe_error_string(FoEError::NO_USER),          "No user logged in");
    EXPECT_STREQ(foe_error_string(FoEError::BOOTSTRAP_ONLY),   "Only in bootstrap state");
    EXPECT_STREQ(foe_error_string(FoEError::NOT_BOOTSTRAP),    "Not in bootstrap state");
    EXPECT_STREQ(foe_error_string(FoEError::NO_RIGHTS),        "Insufficient rights");
    EXPECT_STREQ(foe_error_string(FoEError::PROGRAM_ERROR),    "Programming error");
    EXPECT_STREQ(foe_error_string(FoEError::CHECKSUM_ERROR),   "Checksum error");
}

TEST(FoE_Strings, AllInternalErrorsCovered) {
    EXPECT_STREQ(foe_error_string(FoEError::TIMEOUT),          "Timeout");
    EXPECT_STREQ(foe_error_string(FoEError::MAILBOX_ERROR),    "Mailbox error");
    EXPECT_STREQ(foe_error_string(FoEError::LOCAL_FILE_ERROR), "Local file error");
    EXPECT_STREQ(foe_error_string(FoEError::INVALID_STATE),    "Invalid state");
    EXPECT_STREQ(foe_error_string(FoEError::BUFFER_OVERFLOW),  "Buffer overflow");
    EXPECT_STREQ(foe_error_string(FoEError::CANCELLED),        "Cancelled");
    EXPECT_STREQ(foe_error_string(FoEError::NOT_INITIALIZED),  "Not initialized");
}

TEST(FoE_Strings, UnknownError) {
    EXPECT_STREQ(foe_error_string(static_cast<FoEError>(0xDEAD)), "Unknown error");
}

// ============================================================================
// Construction Tests
// ============================================================================

class FoEManagerConstructTest : public ::testing::Test {
protected:
    MockFoETransport transport_;
};

TEST_F(FoEManagerConstructTest, DefaultState) {
    FoEManager mgr(transport_);
    EXPECT_FALSE(mgr.isInitialized());
    EXPECT_EQ(mgr.pendingCount(), 0u);
    EXPECT_EQ(mgr.activeTransferCount(), 0u);
}

TEST_F(FoEManagerConstructTest, InitSucceeds) {
    FoEManager mgr(transport_);
    EXPECT_TRUE(mgr.init());
    EXPECT_TRUE(mgr.isInitialized());
}

TEST_F(FoEManagerConstructTest, DoubleInitIsIdempotent) {
    FoEManager mgr(transport_);
    EXPECT_TRUE(mgr.init());
    EXPECT_TRUE(mgr.init());  // Second call should also succeed
    EXPECT_TRUE(mgr.isInitialized());
}

TEST_F(FoEManagerConstructTest, DestructorCallsDeinit) {
    {
        FoEManager mgr(transport_);
        mgr.init();
        EXPECT_TRUE(mgr.isInitialized());
    }
    // Should not crash — destructor calls deinit()
}

// ============================================================================
// Init/Deinit Lifecycle Tests
// ============================================================================

class FoEManagerLifecycleTest : public ::testing::Test {
protected:
    MockFoETransport transport_;
};

TEST_F(FoEManagerLifecycleTest, InitThenDeinit) {
    FoEManager mgr(transport_);
    EXPECT_TRUE(mgr.init());
    EXPECT_TRUE(mgr.isInitialized());

    mgr.deinit();
    EXPECT_FALSE(mgr.isInitialized());
}

TEST_F(FoEManagerLifecycleTest, DeinitWithoutInit) {
    FoEManager mgr(transport_);
    mgr.deinit();  // Should not crash
    EXPECT_FALSE(mgr.isInitialized());
}

TEST_F(FoEManagerLifecycleTest, DoubleDeinit) {
    FoEManager mgr(transport_);
    mgr.init();
    mgr.deinit();
    mgr.deinit();  // Second call should be safe
    EXPECT_FALSE(mgr.isInitialized());
}

TEST_F(FoEManagerLifecycleTest, ReinitAfterDeinit) {
    FoEManager mgr(transport_);
    EXPECT_TRUE(mgr.init());
    mgr.deinit();
    EXPECT_FALSE(mgr.isInitialized());

    // Re-initialize
    EXPECT_TRUE(mgr.init());
    EXPECT_TRUE(mgr.isInitialized());
}

TEST_F(FoEManagerLifecycleTest, StatsResetOnInit) {
    FoEManager mgr(transport_);
    mgr.init();

    FoEStats stats = mgr.getStats();
    EXPECT_EQ(stats.uploads_completed, 0u);
    EXPECT_EQ(stats.uploads_failed, 0u);
    EXPECT_EQ(stats.downloads_completed, 0u);
    EXPECT_EQ(stats.downloads_failed, 0u);
    EXPECT_EQ(stats.bytes_uploaded, 0u);
    EXPECT_EQ(stats.bytes_downloaded, 0u);
    EXPECT_EQ(stats.timeouts, 0u);
    EXPECT_EQ(stats.retries, 0u);
}

// ============================================================================
// Operations Before Init (should return NOT_INITIALIZED)
// ============================================================================

class FoEManagerNotInitTest : public ::testing::Test {
protected:
    MockFoETransport transport_;
    FoEManager mgr_{transport_};
};

TEST_F(FoEManagerNotInitTest, UploadFileBeforeInit) {
    FoETransferConfig cfg;
    cfg.filename = "test.bin";
    FoEResult r = mgr_.uploadFile("/tmp/test.bin", cfg);
    EXPECT_FALSE(r.success);
    EXPECT_EQ(r.error_code, FoEError::NOT_INITIALIZED);
}

TEST_F(FoEManagerNotInitTest, DownloadFileBeforeInit) {
    FoETransferConfig cfg;
    cfg.filename = "test.bin";
    FoEResult r = mgr_.downloadFile("/tmp/test.bin", cfg);
    EXPECT_FALSE(r.success);
    EXPECT_EQ(r.error_code, FoEError::NOT_INITIALIZED);
}

TEST_F(FoEManagerNotInitTest, UploadMemoryBeforeInit) {
    uint8_t data[16] = {};
    FoETransferConfig cfg;
    cfg.filename = "test.bin";
    FoEResult r = mgr_.uploadMemory(data, sizeof(data), cfg);
    EXPECT_FALSE(r.success);
    EXPECT_EQ(r.error_code, FoEError::NOT_INITIALIZED);
}

TEST_F(FoEManagerNotInitTest, DownloadMemoryBeforeInit) {
    uint8_t buf[16] = {};
    size_t received = 0;
    FoETransferConfig cfg;
    cfg.filename = "test.bin";
    FoEResult r = mgr_.downloadMemory(buf, sizeof(buf), &received, cfg);
    EXPECT_FALSE(r.success);
    EXPECT_EQ(r.error_code, FoEError::NOT_INITIALIZED);
}

TEST_F(FoEManagerNotInitTest, AsyncUploadBeforeInit) {
    FoETransferConfig cfg;
    FoETransferHandle handle;
    EXPECT_FALSE(mgr_.uploadFileAsync("/tmp/test.bin", cfg, &handle));
}

TEST_F(FoEManagerNotInitTest, AsyncDownloadBeforeInit) {
    FoETransferConfig cfg;
    FoETransferHandle handle;
    EXPECT_FALSE(mgr_.downloadFileAsync("/tmp/test.bin", cfg, &handle));
}

// ============================================================================
// Operations After Init (stubbed — return INVALID_STATE)
// ============================================================================

class FoEManagerInitializedTest : public ::testing::Test {
protected:
    MockFoETransport transport_;
    FoEManager mgr_{transport_};

    void SetUp() override {
        mgr_.init();
    }

    void TearDown() override {
        mgr_.deinit();
    }
};

TEST_F(FoEManagerInitializedTest, UploadFileReturnsInvalidState) {
    FoETransferConfig cfg;
    cfg.filename = "firmware.bin";
    FoEResult r = mgr_.uploadFile("/tmp/firmware.bin", cfg);
    EXPECT_FALSE(r.success);
    EXPECT_EQ(r.error_code, FoEError::INVALID_STATE);
}

TEST_F(FoEManagerInitializedTest, DownloadFileReturnsInvalidState) {
    FoETransferConfig cfg;
    cfg.filename = "log.txt";
    FoEResult r = mgr_.downloadFile("/tmp/log.txt", cfg);
    EXPECT_FALSE(r.success);
    EXPECT_EQ(r.error_code, FoEError::INVALID_STATE);
}

TEST_F(FoEManagerInitializedTest, UploadMemoryReturnsInvalidState) {
    uint8_t data[32] = {0xAA};
    FoETransferConfig cfg;
    cfg.filename = "data.bin";
    FoEResult r = mgr_.uploadMemory(data, sizeof(data), cfg);
    EXPECT_FALSE(r.success);
    EXPECT_EQ(r.error_code, FoEError::INVALID_STATE);
}

TEST_F(FoEManagerInitializedTest, DownloadMemoryReturnsInvalidState) {
    uint8_t buf[64] = {};
    size_t received = 0;
    FoETransferConfig cfg;
    cfg.filename = "data.bin";
    FoEResult r = mgr_.downloadMemory(buf, sizeof(buf), &received, cfg);
    EXPECT_FALSE(r.success);
    EXPECT_EQ(r.error_code, FoEError::INVALID_STATE);
}

// ============================================================================
// FoEResult Helper Tests
// ============================================================================

TEST(FoE_Result, FailureHelper) {
    FoEResult r = FoEResult::Failure(FoEError::TIMEOUT);
    EXPECT_FALSE(r.success);
    EXPECT_EQ(r.error_code, FoEError::TIMEOUT);
    EXPECT_EQ(r.bytes_transferred, 0u);
    EXPECT_EQ(r.blocks_transferred, 0u);
    EXPECT_EQ(r.duration_ms, 0u);
    EXPECT_EQ(r.error_text[0], '\0');
    EXPECT_FALSE(static_cast<bool>(r));
}

TEST(FoE_Result, BoolOperator) {
    FoEResult success{};
    success.success = true;
    EXPECT_TRUE(static_cast<bool>(success));

    FoEResult failure{};
    failure.success = false;
    EXPECT_FALSE(static_cast<bool>(failure));
}

// ============================================================================
// Statistics Tests
// ============================================================================

class FoEManagerStatsTest : public ::testing::Test {
protected:
    MockFoETransport transport_;
    FoEManager mgr_{transport_};

    void SetUp() override {
        mgr_.init();
    }
};

TEST_F(FoEManagerStatsTest, InitialStatsAreZero) {
    FoEStats stats = mgr_.getStats();
    EXPECT_EQ(stats.uploads_completed, 0u);
    EXPECT_EQ(stats.uploads_failed, 0u);
    EXPECT_EQ(stats.downloads_completed, 0u);
    EXPECT_EQ(stats.downloads_failed, 0u);
    EXPECT_EQ(stats.bytes_uploaded, 0u);
    EXPECT_EQ(stats.bytes_downloaded, 0u);
    EXPECT_EQ(stats.timeouts, 0u);
    EXPECT_EQ(stats.retries, 0u);
}

TEST_F(FoEManagerStatsTest, ResetStats) {
    // Stats start at zero; reset should keep them at zero
    mgr_.resetStats();
    FoEStats stats = mgr_.getStats();
    EXPECT_EQ(stats.uploads_completed, 0u);
    EXPECT_EQ(stats.downloads_completed, 0u);
}

TEST_F(FoEManagerStatsTest, GetStatsAfterDeinit) {
    mgr_.deinit();
    // getStats should still work (returns zeroed stats from deinit)
    FoEStats stats = mgr_.getStats();
    EXPECT_EQ(stats.uploads_completed, 0u);
}

// ============================================================================
// Queue Management Tests
// ============================================================================

class FoEManagerQueueTest : public ::testing::Test {
protected:
    MockFoETransport transport_;
    FoEManager mgr_{transport_};

    void SetUp() override {
        mgr_.init();
    }
};

TEST_F(FoEManagerQueueTest, InitialPendingCountIsZero) {
    EXPECT_EQ(mgr_.pendingCount(), 0u);
}

TEST_F(FoEManagerQueueTest, InitialActiveTransferCountIsZero) {
    EXPECT_EQ(mgr_.activeTransferCount(), 0u);
}

TEST_F(FoEManagerQueueTest, PendingCountAfterDeinit) {
    mgr_.deinit();
    EXPECT_EQ(mgr_.pendingCount(), 0u);
}

// ============================================================================
// Transfer Handle Tests
// ============================================================================

TEST(FoE_Handle, DefaultState) {
    FoETransferHandle handle;
    EXPECT_EQ(handle.id, 0u);
    EXPECT_FALSE(handle.complete.load());
    EXPECT_FALSE(handle.cancel.load());
    EXPECT_TRUE(handle.in_progress());
}

TEST(FoE_Handle, RequestCancel) {
    FoETransferHandle handle;
    EXPECT_FALSE(handle.cancel.load());
    handle.request_cancel();
    EXPECT_TRUE(handle.cancel.load());
}

TEST(FoE_Handle, CompleteTransfer) {
    FoETransferHandle handle;
    EXPECT_TRUE(handle.in_progress());
    handle.complete.store(true);
    EXPECT_FALSE(handle.in_progress());
}

// ============================================================================
// TransferConfig Tests
// ============================================================================

TEST(FoE_Config, DefaultValues) {
    FoETransferConfig cfg;
    EXPECT_EQ(cfg.slave_index, 0);
    EXPECT_EQ(cfg.filename, nullptr);
    EXPECT_EQ(cfg.password, static_cast<uint32_t>(ECAT_FOE_DEFAULT_PASSWORD));
    EXPECT_EQ(cfg.timeout_ms, static_cast<uint32_t>(ECAT_FOE_TIMEOUT_MS));
    EXPECT_EQ(cfg.progress_callback, nullptr);
}

// ============================================================================
// Cancel / WaitComplete Tests
// ============================================================================

class FoEManagerCancelTest : public ::testing::Test {
protected:
    MockFoETransport transport_;
    FoEManager mgr_{transport_};

    void SetUp() override {
        mgr_.init();
    }
};

TEST_F(FoEManagerCancelTest, CancelNullHandle) {
    mgr_.cancel(nullptr);  // Should not crash
}

TEST_F(FoEManagerCancelTest, CancelSetsFlag) {
    FoETransferHandle handle;
    EXPECT_FALSE(handle.cancel.load());
    mgr_.cancel(&handle);
    EXPECT_TRUE(handle.cancel.load());
}

TEST_F(FoEManagerCancelTest, WaitCompleteNullHandle) {
    EXPECT_FALSE(mgr_.waitComplete(nullptr));
}

TEST_F(FoEManagerCancelTest, WaitCompleteOnIncompleteHandle) {
    FoETransferHandle handle;
    EXPECT_FALSE(mgr_.waitComplete(&handle));
}

TEST_F(FoEManagerCancelTest, WaitCompleteOnCompleteHandle) {
    FoETransferHandle handle;
    handle.complete.store(true);
    EXPECT_TRUE(mgr_.waitComplete(&handle));
}

// ============================================================================
// Wire Format Structure Tests
// ============================================================================

TEST(FoE_WireFormat, FoEHeaderSize) {
    EXPECT_EQ(sizeof(FoEHeader), 6u);
}

TEST(FoE_WireFormat, FoEErrorResponseSize) {
    EXPECT_EQ(sizeof(FoEErrorResponse), 6u);
}

TEST(FoE_WireFormat, FoEHeaderLayout) {
    FoEHeader hdr{};
    hdr.opcode = static_cast<uint8_t>(FoEOpcode::DATA);
    hdr.reserved = 0;
    hdr.packet_no_le = 42;
    EXPECT_EQ(hdr.opcode, 3);
    EXPECT_EQ(hdr.packet_no_le, 42u);
}

// ============================================================================
// Constants Tests
// ============================================================================

TEST(FoE_Constants, MailboxType) {
    EXPECT_EQ(kFoEMailboxType, 0x04);
}

TEST(FoE_Constants, MaxFilenameIsPositive) {
    EXPECT_GT(kMaxFilename, 0u);
}

TEST(FoE_Constants, BufferSizeIsPositive) {
    EXPECT_GT(kBufferSize, 0u);
}

TEST(FoE_Constants, MaxTransfersIsPositive) {
    EXPECT_GT(kMaxTransfers, 0u);
}

// ============================================================================
// Multiple Independent Instances
// ============================================================================

TEST(FoE_MultiInstance, TwoInstancesAreIndependent) {
    MockFoETransport transport1;
    MockFoETransport transport2;

    FoEManager mgr1(transport1);
    FoEManager mgr2(transport2);

    // Init only mgr1
    EXPECT_TRUE(mgr1.init());
    EXPECT_TRUE(mgr1.isInitialized());
    EXPECT_FALSE(mgr2.isInitialized());

    // Init mgr2
    EXPECT_TRUE(mgr2.init());
    EXPECT_TRUE(mgr2.isInitialized());

    // Deinit mgr1 should not affect mgr2
    mgr1.deinit();
    EXPECT_FALSE(mgr1.isInitialized());
    EXPECT_TRUE(mgr2.isInitialized());

    mgr2.deinit();
}

TEST(FoE_MultiInstance, StatsAreIndependent) {
    MockFoETransport transport1;
    MockFoETransport transport2;

    FoEManager mgr1(transport1);
    FoEManager mgr2(transport2);

    mgr1.init();
    mgr2.init();

    // Each has its own stats
    FoEStats s1 = mgr1.getStats();
    FoEStats s2 = mgr2.getStats();

    EXPECT_EQ(s1.uploads_completed, 0u);
    EXPECT_EQ(s2.uploads_completed, 0u);

    // Reset on one doesn't affect the other
    mgr1.resetStats();
    s1 = mgr1.getStats();
    s2 = mgr2.getStats();
    EXPECT_EQ(s1.uploads_completed, 0u);
    EXPECT_EQ(s2.uploads_completed, 0u);
}

TEST(FoE_MultiInstance, ThreeInstancesLifecycle) {
    MockFoETransport t1, t2, t3;
    FoEManager m1(t1), m2(t2), m3(t3);

    EXPECT_TRUE(m1.init());
    EXPECT_TRUE(m2.init());
    EXPECT_TRUE(m3.init());

    EXPECT_TRUE(m1.isInitialized());
    EXPECT_TRUE(m2.isInitialized());
    EXPECT_TRUE(m3.isInitialized());

    m2.deinit();
    EXPECT_TRUE(m1.isInitialized());
    EXPECT_FALSE(m2.isInitialized());
    EXPECT_TRUE(m3.isInitialized());
}

// ============================================================================
// Free-Function API (backward compatibility)
// ============================================================================

TEST(FoE_FreeFunction, InitDeinit) {
    MockFoETransport transport;
    FoEManager mgr(transport);

    EXPECT_TRUE(foe_init(mgr));
    EXPECT_TRUE(foe_is_initialized(mgr));
    foe_deinit(mgr);
    EXPECT_FALSE(foe_is_initialized(mgr));
}

TEST(FoE_FreeFunction, GetResetStats) {
    MockFoETransport transport;
    FoEManager mgr(transport);
    mgr.init();

    FoEStats stats = foe_get_stats(mgr);
    EXPECT_EQ(stats.uploads_completed, 0u);

    foe_reset_stats(mgr);
    stats = foe_get_stats(mgr);
    EXPECT_EQ(stats.uploads_completed, 0u);
}

TEST(FoE_FreeFunction, UploadDownload) {
    MockFoETransport transport;
    FoEManager mgr(transport);
    mgr.init();

    FoETransferConfig cfg;
    cfg.filename = "test.bin";

    // Upload file (stubbed -> INVALID_STATE)
    FoEResult r = foe_upload_file(mgr, "/tmp/test.bin", cfg);
    EXPECT_FALSE(r.success);
    EXPECT_EQ(r.error_code, FoEError::INVALID_STATE);

    // Download file
    r = foe_download_file(mgr, "/tmp/test.bin", cfg);
    EXPECT_FALSE(r.success);
    EXPECT_EQ(r.error_code, FoEError::INVALID_STATE);

    // Upload memory
    uint8_t data[16] = {};
    r = foe_upload_memory(mgr, data, sizeof(data), cfg);
    EXPECT_FALSE(r.success);

    // Download memory
    uint8_t buf[16] = {};
    size_t received = 0;
    r = foe_download_memory(mgr, buf, sizeof(buf), &received, cfg);
    EXPECT_FALSE(r.success);
}

// ============================================================================
// Enum Value Tests (ensuring enum values match protocol spec)
// ============================================================================

TEST(FoE_Enums, OpcodeValues) {
    EXPECT_EQ(static_cast<uint8_t>(FoEOpcode::RRQ),   1);
    EXPECT_EQ(static_cast<uint8_t>(FoEOpcode::WRQ),   2);
    EXPECT_EQ(static_cast<uint8_t>(FoEOpcode::DATA),  3);
    EXPECT_EQ(static_cast<uint8_t>(FoEOpcode::ACK),   4);
    EXPECT_EQ(static_cast<uint8_t>(FoEOpcode::ERROR), 5);
    EXPECT_EQ(static_cast<uint8_t>(FoEOpcode::BUSY),  6);
}

TEST(FoE_Enums, ErrorCodeValues) {
    EXPECT_EQ(static_cast<uint32_t>(FoEError::SUCCESS),        0x0000u);
    EXPECT_EQ(static_cast<uint32_t>(FoEError::NOT_FOUND),      0x8001u);
    EXPECT_EQ(static_cast<uint32_t>(FoEError::ACCESS_DENIED),  0x8002u);
    EXPECT_EQ(static_cast<uint32_t>(FoEError::DISK_FULL),      0x8003u);
    EXPECT_EQ(static_cast<uint32_t>(FoEError::ILLEGAL_OP),     0x8004u);
    EXPECT_EQ(static_cast<uint32_t>(FoEError::PACKET_NUM),     0x8005u);
    EXPECT_EQ(static_cast<uint32_t>(FoEError::ALREADY_EXISTS), 0x8006u);
    EXPECT_EQ(static_cast<uint32_t>(FoEError::NO_USER),        0x8007u);
    EXPECT_EQ(static_cast<uint32_t>(FoEError::NOT_INITIALIZED),0xFF07u);
}

// ============================================================================
// FoEProgress struct defaults
// ============================================================================

TEST(FoE_Progress, DefaultInitialization) {
    FoEProgress p{};
    EXPECT_EQ(p.slave_index, 0);
    EXPECT_EQ(p.filename, nullptr);
    EXPECT_EQ(p.bytes_transferred, 0u);
    EXPECT_EQ(p.total_bytes, 0u);
    EXPECT_EQ(p.block_number, 0u);
    EXPECT_FALSE(p.is_upload);
    EXPECT_FLOAT_EQ(p.throughput_bps, 0.0f);
}

// ============================================================================
// FoEStats struct defaults
// ============================================================================

TEST(FoE_Stats, ZeroInitialized) {
    FoEStats s{};
    EXPECT_EQ(s.uploads_completed, 0u);
    EXPECT_EQ(s.uploads_failed, 0u);
    EXPECT_EQ(s.downloads_completed, 0u);
    EXPECT_EQ(s.downloads_failed, 0u);
    EXPECT_EQ(s.bytes_uploaded, 0u);
    EXPECT_EQ(s.bytes_downloaded, 0u);
    EXPECT_EQ(s.timeouts, 0u);
    EXPECT_EQ(s.retries, 0u);
}
