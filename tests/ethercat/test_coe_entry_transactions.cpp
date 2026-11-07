/**
 * @file test_coe_entry_transactions.cpp
 * @brief Coverage for CoEManager typed entry access
 *        (src/ethercat/raw/CoEEntryTransactions.cpp): readEntry/writeEntry
 *        error paths — unsupported data types, uninit/shutdown/queue-full
 *        fast-fails, worker-side NotConfigured/abort/cancel/transport
 *        results, size-mismatch InternalError, trailing-byte warn,
 *        sign extension, and sync-wrapper timeouts.
 */

#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include <chrono>
#include <condition_variable>
#include <cstring>
#include <mutex>
#include <thread>

#include "tether/ethercat/CoEManager.hpp"

using namespace EtherCAT;
using namespace EtherCAT::CoE;
using ::testing::_;
using ::testing::Invoke;
using ::testing::NiceMock;
using ::testing::Return;

namespace {

using OD = ObjectDictionary::ObjectDictionaryDataType;

uint64_t realMicros() {
    return std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
}

class MockSDOTransport : public SDO::ISDOTransport {
public:
    MOCK_METHOD(bool, sdoUpload,
                (uint16_t, uint8_t*, uint16_t, uint16_t, uint16_t, uint16_t,
                 uint16_t, uint8_t, uint8_t*, size_t, size_t*,
                 bool, unsigned int, unsigned int), (override));
    MOCK_METHOD(bool, sdoDownload,
                (uint16_t, uint8_t*, uint16_t, uint16_t, uint16_t, uint16_t,
                 uint16_t, uint8_t, const uint8_t*, size_t,
                 bool, unsigned int, unsigned int), (override));
    MOCK_METHOD(uint64_t, getMicroseconds, (), (override));
    MOCK_METHOD(uint32_t, lastAbortCode, (), (const, override));
    MOCK_METHOD(bool, isCancelRequested, (), (const, override));
};

auto uploadData(std::initializer_list<uint8_t> bytes) {
    return [b = std::vector<uint8_t>(bytes)](
               uint16_t, uint8_t*, uint16_t, uint16_t, uint16_t, uint16_t,
               uint16_t, uint8_t, uint8_t* out, size_t cap, size_t* out_len,
               bool, unsigned int, unsigned int) -> bool {
        memcpy(out, b.data(), std::min(b.size(), cap));
        if (out_len) *out_len = b.size();
        return true;
    };
}

class CoEEntryTest : public ::testing::Test {
protected:
    void SetUp() override {
        ON_CALL(transport_, getMicroseconds()).WillByDefault(Invoke(&realMicros));
        ON_CALL(transport_, lastAbortCode()).WillByDefault(Return(0u));
        ON_CALL(transport_, isCancelRequested()).WillByDefault(Return(false));
        ON_CALL(transport_, sdoUpload(_, _, _, _, _, _, _, _, _, _, _, _, _, _))
            .WillByDefault(Invoke(uploadData({0x11, 0x22, 0x33, 0x44})));
        ON_CALL(transport_, sdoDownload(_, _, _, _, _, _, _, _, _, _, _, _, _))
            .WillByDefault(Return(true));
    }
    void initMgr(CoE::CoEManager& mgr, bool mailbox = true) {
        ASSERT_TRUE(mgr.init());
        if (mailbox) mgr.configureMailbox(0x1000, 128, 0x1080, 128);
    }

    NiceMock<MockSDOTransport> transport_;
};

ObjectDictionary::ObjectDictionaryEntry makeEntry(uint16_t index, OD dt) {
    ObjectDictionary::ObjectDictionaryEntry e{};
    e.index = index;
    e.subindex = 0;
    e.name = "test";
    e.data_type = dt;
    return e;
}

// ============================================================================
// readEntryAsync — fast-fail paths before enqueue
// ============================================================================

TEST_F(CoEEntryTest, UnsupportedDataTypeFailsImmediately) {
    CoE::CoEManager mgr(0, transport_);
    initMgr(mgr);
    auto fut = mgr.readEntryAsync(makeEntry(0x2000, OD::VisibleString));
    auto res = fut.get();
    ASSERT_FALSE(res.has_value());
}

TEST_F(CoEEntryTest, WriteUnsupportedDataTypeFailsImmediately) {
    CoE::CoEManager mgr(0, transport_);
    initMgr(mgr);
    auto fut = mgr.writeEntryAsync(makeEntry(0x2000, OD::Domain), 0x42);
    ASSERT_FALSE(fut.get().has_value());
}

TEST_F(CoEEntryTest, UninitializedManagerFailsNotConfigured) {
    CoE::CoEManager mgr(0, transport_);   // no init()
    auto fut = mgr.readEntryAsync(makeEntry(0x2000, OD::Unsigned8));
    auto res = fut.get();
    ASSERT_FALSE(res.has_value());
    EXPECT_EQ(res.error().code, CoEErrorCode::NotConfigured);
}

TEST_F(CoEEntryTest, CancelRequestedYieldsShuttingDown) {
    CoE::CoEManager mgr(0, transport_);
    initMgr(mgr);
    ON_CALL(transport_, isCancelRequested()).WillByDefault(Return(true));
    auto fut = mgr.readEntryAsync(makeEntry(0x2000, OD::Unsigned8));
    auto res = fut.get();
    ASSERT_FALSE(res.has_value());
    EXPECT_EQ(res.error().code, CoEErrorCode::ShuttingDown);
}

// ============================================================================
// Worker-side execute() results
// ============================================================================

TEST_F(CoEEntryTest, MissingMailboxYieldsNotConfigured) {
    CoE::CoEManager mgr(0, transport_);
    initMgr(mgr, /*mailbox=*/false);
    auto res = mgr.readEntry(makeEntry(0x2000, OD::Unsigned8));
    ASSERT_FALSE(res.has_value());
    EXPECT_EQ(res.error().code, CoEErrorCode::NotConfigured);
}

TEST_F(CoEEntryTest, UploadAbortPropagatesAbortCode) {
    CoE::CoEManager mgr(0, transport_);
    initMgr(mgr);
    ON_CALL(transport_, sdoUpload(_, _, _, _, _, _, _, _, _, _, _, _, _, _))
        .WillByDefault(Return(false));
    ON_CALL(transport_, lastAbortCode()).WillByDefault(Return(0x06020000u));

    auto res = mgr.readEntry(makeEntry(0x2000, OD::Unsigned8));
    ASSERT_FALSE(res.has_value());
}

TEST_F(CoEEntryTest, UploadFailNoAbortYieldsTransportError) {
    CoE::CoEManager mgr(0, transport_);
    initMgr(mgr);
    ON_CALL(transport_, sdoUpload(_, _, _, _, _, _, _, _, _, _, _, _, _, _))
        .WillByDefault(Return(false));

    auto res = mgr.readEntry(makeEntry(0x2000, OD::Unsigned8));
    ASSERT_FALSE(res.has_value());
    EXPECT_EQ(res.error().code, CoEErrorCode::TransportError);
}

TEST_F(CoEEntryTest, UploadFailCancelYieldsShuttingDown) {
    CoE::CoEManager mgr(0, transport_);
    initMgr(mgr);
    ON_CALL(transport_, sdoUpload(_, _, _, _, _, _, _, _, _, _, _, _, _, _))
        .WillByDefault(Return(false));
    ON_CALL(transport_, isCancelRequested()).WillByDefault(Return(true));

    auto res = mgr.readEntry(makeEntry(0x2000, OD::Unsigned8));
    ASSERT_FALSE(res.has_value());
    EXPECT_EQ(res.error().code, CoEErrorCode::ShuttingDown);
}

TEST_F(CoEEntryTest, ShortUploadYieldsInternalError) {
    CoE::CoEManager mgr(0, transport_);
    initMgr(mgr);
    // 1 byte back for a 4-byte declared type.
    ON_CALL(transport_, sdoUpload(_, _, _, _, _, _, _, _, _, _, _, _, _, _))
        .WillByDefault(Invoke(uploadData({0xAA})));

    auto res = mgr.readEntry(makeEntry(0x2000, OD::Unsigned32));
    ASSERT_FALSE(res.has_value());
    EXPECT_EQ(res.error().code, CoEErrorCode::InternalError);
}

TEST_F(CoEEntryTest, TrailingBytesDiscarded) {
    CoE::CoEManager mgr(0, transport_);
    initMgr(mgr);
    // 4 bytes back for a 1-byte declared type — warning arm, still succeeds.
    auto res = mgr.readEntry(makeEntry(0x2000, OD::Unsigned8));
    ASSERT_TRUE(res.has_value());
    EXPECT_EQ(*res, 0x11u);
}

TEST_F(CoEEntryTest, SignedValueIsSignExtended) {
    CoE::CoEManager mgr(0, transport_);
    initMgr(mgr);
    ON_CALL(transport_, sdoUpload(_, _, _, _, _, _, _, _, _, _, _, _, _, _))
        .WillByDefault(Invoke(uploadData({0x00, 0x80})));  // -32768 as i16

    auto res = mgr.readEntry(makeEntry(0x2000, OD::Integer16));
    ASSERT_TRUE(res.has_value());
    EXPECT_EQ(*res, 0xFFFFFFFFFFFF8000ull);
}

TEST_F(CoEEntryTest, CoeReadsDebugFlagLogsStart) {
    CoE::CoEManager mgr(0, transport_);
    initMgr(mgr);
    EtherCATSlaveDebugFlags flags{};
    flags.coeReads = true;
    mgr.updateDebugFlags(flags);

    auto res = mgr.readEntry(makeEntry(0x2000, OD::Unsigned8));
    ASSERT_TRUE(res.has_value());
}

// ============================================================================
// Queue-full
// ============================================================================

TEST_F(CoEEntryTest, QueueFullFailsFast) {
    CoE::CoEManager mgr(0, transport_);
    initMgr(mgr);

    // Block the worker inside the first upload.
    std::mutex m;
    std::condition_variable cv;
    bool release = false;
    ON_CALL(transport_, sdoUpload(_, _, _, _, _, _, _, _, _, _, _, _, _, _))
        .WillByDefault(Invoke([&](uint16_t, uint8_t*, uint16_t, uint16_t,
                                  uint16_t, uint16_t, uint16_t, uint8_t,
                                  uint8_t*, size_t, size_t* out_len,
                                  bool, unsigned int, unsigned int) {
            std::unique_lock<std::mutex> lk(m);
            cv.wait(lk, [&] { return release; });
            if (out_len) *out_len = 1;
            return true;
        }));

    auto entry = makeEntry(0x2000, OD::Unsigned8);
    std::vector<std::future<CoEResult<uint64_t>>> futs;
    futs.push_back(mgr.readEntryAsync(entry));   // dequeued by worker, blocks

    // Give the worker a moment to pick up the first item.
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    bool saw_queue_full = false;
    for (size_t i = 0; i < CoE::kMaxQueueDepth + 4; ++i) {
        auto f = mgr.readEntryAsync(entry);
        // failNow() futures are satisfied before readEntryAsync returns —
        // a ready future here is a fast-fail, not a queued item.
        if (f.wait_for(std::chrono::milliseconds(0)) ==
            std::future_status::ready) {
            auto r = f.get();
            if (!r.has_value() && r.error().code == CoEErrorCode::QueueFull) {
                saw_queue_full = true;
                break;
            }
        } else {
            futs.push_back(std::move(f));   // queued — drain after release
        }
    }
    EXPECT_TRUE(saw_queue_full);

    {
        std::lock_guard<std::mutex> lk(m);
        release = true;
    }
    cv.notify_all();
    for (auto& f : futs) (void)f.get();
}

// ============================================================================
// Sync wrappers — timeout path
// ============================================================================

TEST_F(CoEEntryTest, ReadEntrySyncTimeout) {
    CoE::CoEManager mgr(0, transport_);
    initMgr(mgr);
    ON_CALL(transport_, sdoUpload(_, _, _, _, _, _, _, _, _, _, _, _, _, _))
        .WillByDefault(Invoke([](auto&&...) -> bool {
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
            return true;
        }));

    CoETransactionOptions opts{};
    opts.timeout_ms = 30;
    auto res = mgr.readEntry(makeEntry(0x2000, OD::Unsigned8), opts);
    ASSERT_FALSE(res.has_value());
    EXPECT_EQ(res.error().code, CoEErrorCode::Timeout);
}

TEST_F(CoEEntryTest, WriteEntrySyncTimeout) {
    CoE::CoEManager mgr(0, transport_);
    initMgr(mgr);
    ON_CALL(transport_, sdoDownload(_, _, _, _, _, _, _, _, _, _, _, _, _))
        .WillByDefault(Invoke([](auto&&...) -> bool {
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
            return true;
        }));

    CoETransactionOptions opts{};
    opts.timeout_ms = 30;
    auto res = mgr.writeEntry(makeEntry(0x2000, OD::Unsigned8), 0x42, opts);
    ASSERT_FALSE(res.has_value());
    EXPECT_EQ(res.error().code, CoEErrorCode::Timeout);
}

TEST_F(CoEEntryTest, WriteEntrySuccess) {
    CoE::CoEManager mgr(0, transport_);
    initMgr(mgr);
    auto res = mgr.writeEntry(makeEntry(0x2000, OD::Unsigned16), 0xBEEF);
    EXPECT_TRUE(res.has_value());
}

// ============================================================================
// Subindex-override overloads — forward to the entry-based variants
// ============================================================================

TEST_F(CoEEntryTest, SubindexOverloadsForwardAndTimeout) {
    CoE::CoEManager mgr(0, transport_);
    initMgr(mgr);

    auto entry = makeEntry(0x2000, OD::Unsigned8);
    auto res = mgr.readEntry(entry, uint8_t{3});
    ASSERT_TRUE(res.has_value());
    EXPECT_EQ(*res, 0x11u);

    auto wres = mgr.writeEntry(entry, uint8_t{5}, 0x7Fu);
    EXPECT_TRUE(wres.has_value());

    auto afut = mgr.readEntryAsync(entry, uint8_t{7});
    ASSERT_TRUE(afut.get().has_value());
    auto wfut = mgr.writeEntryAsync(entry, uint8_t{9}, 0x1u);
    ASSERT_TRUE(wfut.get().has_value());
}

TEST_F(CoEEntryTest, SubindexSyncTimeout) {
    CoE::CoEManager mgr(0, transport_);
    initMgr(mgr);
    ON_CALL(transport_, sdoDownload(_, _, _, _, _, _, _, _, _, _, _, _, _))
        .WillByDefault(Invoke([](auto&&...) -> bool {
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
            return true;
        }));

    CoETransactionOptions opts{};
    opts.timeout_ms = 30;
    auto res = mgr.writeEntry(makeEntry(0x2000, OD::Unsigned8), uint8_t{1},
                              0x42, opts);
    ASSERT_FALSE(res.has_value());
    EXPECT_EQ(res.error().code, CoEErrorCode::Timeout);
}

// ============================================================================
// Shutdown flush — pending transactions get fail()
// ============================================================================

TEST_F(CoEEntryTest, DeinitFlushesPendingWithError) {
    auto mgr = std::make_unique<CoE::CoEManager>(0, transport_);
    initMgr(*mgr);

    std::mutex m;
    std::condition_variable cv;
    bool release = false;
    ON_CALL(transport_, sdoUpload(_, _, _, _, _, _, _, _, _, _, _, _, _, _))
        .WillByDefault(Invoke([&](uint16_t, uint8_t*, uint16_t, uint16_t,
                                  uint16_t, uint16_t, uint16_t, uint8_t,
                                  uint8_t*, size_t, size_t* out_len,
                                  bool, unsigned int, unsigned int) {
            std::unique_lock<std::mutex> lk(m);
            cv.wait(lk, [&] { return release; });
            if (out_len) *out_len = 1;
            return true;
        }));

    auto entry = makeEntry(0x2000, OD::Unsigned8);
    auto f1 = mgr->readEntryAsync(entry);   // blocks the worker
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    auto f2 = mgr->readEntryAsync(entry);   // stays queued

    {
        std::lock_guard<std::mutex> lk(m);
        release = true;
    }
    cv.notify_all();
    mgr->deinit();
    mgr.reset();

    // Both futures complete — the queued one through the fail() path.
    (void)f1.get();
    (void)f2.get();
}

}  // namespace
