/**
 * @file test_foe_manager.cpp
 * @brief Unit tests for the FoE (File over EtherCAT) manager — opcode/error
 *        name tables, lifecycle, request queue, statistics and the stubbed
 *        transfer API surface.
 */

#include <gtest/gtest.h>

#include <cstring>

#include "tether/ethercat/FoE.hpp"

using namespace EtherCAT::FoE;

namespace {

class StubFoETransport : public IFoETransport {
public:
    bool mailboxWrite(uint16_t, uint16_t, uint16_t, const uint8_t*, size_t,
                      uint8_t*) override { return false; }
    bool mailboxRead(uint16_t, uint16_t, uint16_t, uint8_t*, size_t, size_t*,
                     uint32_t) override { return false; }
    bool fileOpen(const char*, bool, uint32_t*) override { return false; }
    int32_t fileRead(void*, size_t) override { return -1; }
    int32_t fileWrite(const void*, size_t) override { return -1; }
    void fileClose() override {}
    uint32_t getTimeMs() override { return 0; }
    void delayMs(uint32_t) override {}
};

}  // namespace

TEST(FoENames, OpcodeStrings) {
    EXPECT_STREQ(foe_opcode_string(FoEOpcode::RRQ), "RRQ");
    EXPECT_STREQ(foe_opcode_string(FoEOpcode::WRQ), "WRQ");
    EXPECT_STREQ(foe_opcode_string(FoEOpcode::DATA), "DATA");
    EXPECT_STREQ(foe_opcode_string(FoEOpcode::ACK), "ACK");
    EXPECT_STREQ(foe_opcode_string(FoEOpcode::ERROR), "ERROR");
    EXPECT_STREQ(foe_opcode_string(FoEOpcode::BUSY), "BUSY");
    EXPECT_STREQ(foe_opcode_string(static_cast<FoEOpcode>(0xEE)), "UNKNOWN");
}

TEST(FoENames, ErrorStrings) {
    EXPECT_STREQ(foe_error_string(FoEError::SUCCESS), "Success");
    EXPECT_STREQ(foe_error_string(FoEError::NOT_FOUND), "File not found");
    EXPECT_STREQ(foe_error_string(FoEError::ACCESS_DENIED), "Access denied");
    EXPECT_STREQ(foe_error_string(FoEError::DISK_FULL), "Disk full");
    EXPECT_STREQ(foe_error_string(FoEError::ILLEGAL_OP), "Illegal operation");
    EXPECT_STREQ(foe_error_string(FoEError::PACKET_NUM), "Wrong packet number");
    EXPECT_STREQ(foe_error_string(FoEError::ALREADY_EXISTS), "File already exists");
    EXPECT_STREQ(foe_error_string(FoEError::NO_USER), "No user logged in");
    EXPECT_STREQ(foe_error_string(FoEError::BOOTSTRAP_ONLY), "Only in bootstrap state");
    EXPECT_STREQ(foe_error_string(FoEError::NOT_BOOTSTRAP), "Not in bootstrap state");
    EXPECT_STREQ(foe_error_string(FoEError::NO_RIGHTS), "Insufficient rights");
    EXPECT_STREQ(foe_error_string(FoEError::PROGRAM_ERROR), "Programming error");
    EXPECT_STREQ(foe_error_string(FoEError::CHECKSUM_ERROR), "Checksum error");
    EXPECT_STREQ(foe_error_string(FoEError::TIMEOUT), "Timeout");
    EXPECT_STREQ(foe_error_string(FoEError::MAILBOX_ERROR), "Mailbox error");
    EXPECT_STREQ(foe_error_string(FoEError::LOCAL_FILE_ERROR), "Local file error");
    EXPECT_STREQ(foe_error_string(FoEError::INVALID_STATE), "Invalid state");
    EXPECT_STREQ(foe_error_string(FoEError::BUFFER_OVERFLOW), "Buffer overflow");
    EXPECT_STREQ(foe_error_string(FoEError::CANCELLED), "Cancelled");
    EXPECT_STREQ(foe_error_string(FoEError::NOT_INITIALIZED), "Not initialized");
    EXPECT_STREQ(foe_error_string(static_cast<FoEError>(0xEE)), "Unknown error");
}

TEST(FoEManagerTest, Lifecycle) {
    StubFoETransport transport;
    FoEManager mgr(transport);

    EXPECT_FALSE(mgr.isInitialized());
    EXPECT_TRUE(mgr.init());
    EXPECT_TRUE(mgr.isInitialized());
    // Second init is a no-op that still reports success.
    EXPECT_TRUE(mgr.init());
    EXPECT_TRUE(mgr.isInitialized());

    mgr.deinit();
    EXPECT_FALSE(mgr.isInitialized());
    // Deinit when not initialized is a no-op.
    mgr.deinit();
    EXPECT_FALSE(mgr.isInitialized());
}

TEST(FoEManagerTest, DestructorDeinitializes) {
    StubFoETransport transport;
    {
        FoEManager mgr(transport);
        ASSERT_TRUE(mgr.init());
        EXPECT_TRUE(mgr.isInitialized());
    }
    SUCCEED();
}

TEST(FoEManagerTest, StatsReset) {
    StubFoETransport transport;
    FoEManager mgr(transport);
    ASSERT_TRUE(mgr.init());

    const FoEStats stats = mgr.getStats();
    EXPECT_EQ(stats.uploads_completed, 0u);
    EXPECT_EQ(stats.bytes_uploaded, 0u);
    mgr.resetStats();
    EXPECT_EQ(mgr.getStats().uploads_completed, 0u);
    EXPECT_EQ(mgr.activeTransferCount(), 0u);
    EXPECT_EQ(mgr.pendingCount(), 0u);
}

TEST(FoEManagerTest, TransferApiRequiresInit) {
    StubFoETransport transport;
    FoEManager mgr(transport);

    FoETransferConfig cfg{};
    FoETransferHandle handle;

    // Before init every entry point reports NOT_INITIALIZED / false.
    EXPECT_EQ(mgr.uploadFile("x", cfg).error_code, FoEError::NOT_INITIALIZED);
    EXPECT_EQ(mgr.downloadFile("x", cfg).error_code, FoEError::NOT_INITIALIZED);
    EXPECT_EQ(mgr.uploadMemory(nullptr, 0, cfg).error_code,
              FoEError::NOT_INITIALIZED);
    size_t received = 0;
    EXPECT_EQ(mgr.downloadMemory(nullptr, 0, &received, cfg).error_code,
              FoEError::NOT_INITIALIZED);
    EXPECT_FALSE(mgr.uploadFileAsync("x", cfg, &handle));
    EXPECT_FALSE(mgr.downloadFileAsync("x", cfg, &handle));

    // After init the stubs report INVALID_STATE / false.
    ASSERT_TRUE(mgr.init());
    EXPECT_EQ(mgr.uploadFile("x", cfg).error_code, FoEError::INVALID_STATE);
    EXPECT_EQ(mgr.downloadFile("x", cfg).error_code, FoEError::INVALID_STATE);
    EXPECT_EQ(mgr.uploadMemory(nullptr, 0, cfg).error_code,
              FoEError::INVALID_STATE);
    EXPECT_EQ(mgr.downloadMemory(nullptr, 0, &received, cfg).error_code,
              FoEError::INVALID_STATE);
    EXPECT_FALSE(mgr.uploadFileAsync("x", cfg, &handle));
    EXPECT_FALSE(mgr.downloadFileAsync("x", cfg, &handle));
}

TEST(FoEManagerTest, HandleHelpers) {
    StubFoETransport transport;
    FoEManager mgr(transport);
    ASSERT_TRUE(mgr.init());

    EXPECT_FALSE(mgr.waitComplete(nullptr, 10));

    FoETransferHandle handle;
    EXPECT_FALSE(mgr.waitComplete(&handle, 10));
    handle.complete.store(true);
    EXPECT_TRUE(mgr.waitComplete(&handle, 10));

    mgr.cancel(nullptr);  // no-op
    EXPECT_FALSE(handle.cancel.load());
    mgr.cancel(&handle);
    EXPECT_TRUE(handle.cancel.load());
}
