/**
 * @file test_foe_manager.cpp
 * @brief Unit tests for the FoE (File over EtherCAT) manager — opcode/error
 *        name tables, lifecycle, request queue, statistics and the stubbed
 *        transfer API surface.
 */

#include <gtest/gtest.h>

#include <algorithm>
#include <cstdint>
#include <cstring>
#include <deque>
#include <vector>

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

    // After init the sync calls fail fast (no filename -> INVALID_STATE)
    // and the async calls are accepted onto the worker queue.
    ASSERT_TRUE(mgr.init());
    EXPECT_EQ(mgr.uploadFile("x", cfg).error_code, FoEError::INVALID_STATE);
    EXPECT_EQ(mgr.downloadFile("x", cfg).error_code, FoEError::INVALID_STATE);
    EXPECT_EQ(mgr.uploadMemory(nullptr, 0, cfg).error_code,
              FoEError::INVALID_STATE);
    EXPECT_EQ(mgr.downloadMemory(nullptr, 0, &received, cfg).error_code,
              FoEError::INVALID_STATE);
    EXPECT_TRUE(mgr.uploadFileAsync("x", cfg, &handle));
    EXPECT_TRUE(mgr.waitComplete(&handle, 2000));
    EXPECT_EQ(handle.result.error_code, FoEError::INVALID_STATE);
    EXPECT_TRUE(mgr.downloadFileAsync("x", cfg, &handle));
    EXPECT_TRUE(mgr.waitComplete(&handle, 2000));
    EXPECT_EQ(handle.result.error_code, FoEError::INVALID_STATE);
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

// ============================================================================
// Scripted slave transport — exercises the real FoE protocol exchange.
// ============================================================================

namespace {

// Emulates a FoE slave: WRQ -> ACK(0); each DATA -> ACK(block); RRQ ->
// ACK(0,size) then DATA blocks of `file` content, one per master ACK.
class ScriptedFoESlave : public IFoETransport {
public:
    std::vector<uint8_t> file;     // content the "slave" serves
    std::vector<uint8_t> uploaded; // bytes received via DATA blocks

    bool mailboxWrite(uint16_t, uint16_t, uint16_t, const uint8_t* data,
                      size_t len, uint8_t*) override {
        if (len < 1) return false;
        const uint8_t op = data[0];
        if (op == 2) {                    // WRQ -> ACK block 0
            pushMsg({4, 0, 0, 0, 0, 0});
        } else if (op == 3) {             // DATA -> ACK same block
            if (len > 6) uploaded.insert(uploaded.end(), data + 6, data + len);
            pushMsg({4, 0, data[2], data[3], data[4], data[5]});
        } else if (op == 1) {             // RRQ -> ACK(0,size) + first DATA
            const uint32_t sz = static_cast<uint32_t>(file.size());
            pushMsg({4, 0, 0, 0, 0, 0,
                     static_cast<uint8_t>(sz), static_cast<uint8_t>(sz >> 8),
                     static_cast<uint8_t>(sz >> 16), static_cast<uint8_t>(sz >> 24)});
            rxOffset_ = 0;
            rxBlock_ = 1;
            txDone_ = false;
            pumpData();
        } else if (op == 4) {             // master ACK -> next DATA block
            if (!txDone_) pumpData();
        }
        return true;
    }

    bool mailboxRead(uint16_t, uint16_t, uint16_t, uint8_t* data, size_t cap,
                     size_t* len, uint32_t) override {
        if (msgs_.empty()) return false;
        const auto& m = msgs_.front();
        if (m.size() > cap) return false;
        std::memcpy(data, m.data(), m.size());
        *len = m.size();
        msgs_.pop_front();
        return true;
    }

    bool fileOpen(const char*, bool, uint32_t* size) override {
        readPos_ = 0;
        if (size) *size = static_cast<uint32_t>(file.size());
        return true;
    }
    int32_t fileRead(void* buf, size_t n) override {
        const size_t avail = file.size() - readPos_;
        const size_t c = std::min(n, avail);
        std::memcpy(buf, file.data() + readPos_, c);
        readPos_ += c;
        return static_cast<int32_t>(c);
    }
    int32_t fileWrite(const void* buf, size_t n) override {
        const uint8_t* p = static_cast<const uint8_t*>(buf);
        uploaded.insert(uploaded.end(), p, p + n);
        return static_cast<int32_t>(n);
    }
    void fileClose() override {}
    uint32_t getTimeMs() override { return 0; }
    void delayMs(uint32_t) override {}

private:
    void pushMsg(std::vector<uint8_t> m) { msgs_.push_back(std::move(m)); }

    void pumpData() {
        std::vector<uint8_t> m = {3, 0,
            static_cast<uint8_t>(rxBlock_),
            static_cast<uint8_t>(rxBlock_ >> 8),
            static_cast<uint8_t>(rxBlock_ >> 16),
            static_cast<uint8_t>(rxBlock_ >> 24)};
        const size_t n = std::min<size_t>(496, file.size() - rxOffset_);
        m.insert(m.end(), file.begin() + rxOffset_,
                 file.begin() + rxOffset_ + n);
        rxOffset_ += n;
        ++rxBlock_;
        txDone_ = (n < 496);   // short/empty block terminates the transfer
        pushMsg(std::move(m));
    }

    std::deque<std::vector<uint8_t>> msgs_;
    size_t rxOffset_ = 0;
    uint32_t rxBlock_ = 1;
    bool txDone_ = false;
    size_t readPos_ = 0;
};

} // namespace

TEST(FoEManagerTest, UploadMemoryRoundTrip) {
    ScriptedFoESlave slave;
    const std::vector<uint8_t> payload = {1, 2, 3, 4, 5, 6, 7, 8};
    slave.file = payload;
    FoEManager mgr(slave);
    ASSERT_TRUE(mgr.init());

    FoETransferConfig cfg;
    cfg.filename = "fw.bin";
    cfg.slave_index = 1;
    cfg.timeout_ms = 1000;

    FoEResult r = mgr.uploadMemory(payload.data(), payload.size(), cfg);
    EXPECT_TRUE(r.success) << "err=" << static_cast<int>(r.error_code);
    EXPECT_EQ(r.bytes_transferred, payload.size());
    EXPECT_EQ(slave.uploaded, payload);
    EXPECT_EQ(mgr.getStats().uploads_completed, 1u);
    mgr.deinit();
}

TEST(FoEManagerTest, DownloadMemoryRoundTrip) {
    ScriptedFoESlave slave;
    std::vector<uint8_t> payload(600);
    for (size_t i = 0; i < payload.size(); ++i) payload[i] = static_cast<uint8_t>(i);
    slave.file = payload;
    FoEManager mgr(slave);
    ASSERT_TRUE(mgr.init());

    uint8_t buf[1024] = {};
    size_t received = 0;
    FoETransferConfig cfg;
    cfg.filename = "log.bin";
    cfg.slave_index = 1;
    cfg.timeout_ms = 1000;

    FoEResult r = mgr.downloadMemory(buf, sizeof(buf), &received, cfg);
    EXPECT_TRUE(r.success) << "err=" << static_cast<int>(r.error_code);
    EXPECT_EQ(received, payload.size());
    EXPECT_EQ(std::memcmp(buf, payload.data(), payload.size()), 0);
    EXPECT_EQ(mgr.getStats().downloads_completed, 1u);
    mgr.deinit();
}
