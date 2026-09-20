/**
 * @file test_process_image.cpp
 * @brief Unit tests for ProcessImage modes + LogicalAddressManager image-mode
 *        cyclic exchange (Direct / TripleBuffered / Rotating / Buffered).
 */

#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include <atomic>
#include <cstring>
#include <thread>

#include "tether/ethercat/LogicalAddressManager.hpp"
#include "tether/ethercat/PDOManager.hpp"
#include "tether/ethercat/ProcessImage.hpp"
#include "tether/ethercat/CyclicChannel.hpp"

using namespace EtherCAT;
using namespace EtherCAT::PDO;

// ============================================================================
// Fake cyclic channel — records rxHold/rxRelease, serves canned frames
// ============================================================================

class FakeCyclicChannel : public ICyclicChannel {
public:
    uint8_t tx_frame[1600] = {};
    int     tx_acquire_count = 0;
    int     committed_len    = -1;
    uint8_t sent_frame[1600] = {};
    uint32_t sent_len        = 0;
    std::atomic<int> holds{0};

    uint8_t* txAcquire() override { ++tx_acquire_count; return tx_frame; }
    size_t   txCapacity() const override { return sizeof(tx_frame); }
    bool     txCommitFrame(uint32_t len) override {
        committed_len = static_cast<int>(len);
        std::memcpy(sent_frame, tx_frame, std::min<size_t>(len, sizeof(sent_frame)));
        sent_len = len;
        return true;
    }
    int  rxPoll(CyclicFrameView*, int, uint32_t) override { return 0; }
    void rxHold(uint32_t)   override { holds.fetch_add(1); }
    void rxRelease(uint32_t) override { holds.fetch_sub(1); }
    int  fd() const override { return -1; }
    bool zeroCopy() const override { return true; }
    const char* backendName() const override { return "fake"; }
};

// ============================================================================
// Fake PDO transport — implements the cyclic slot API in software
// ============================================================================

class FakePDOTransport : public IPDOTransport {
public:
    uint8_t  next_idx = 0;
    uint8_t  last_sent_frame[1600] = {};
    uint16_t last_sent_len = 0;
    uint8_t  last_sent_idx = 0;
    bool     send_ok = true;
    uint8_t  fake_tx_frame[1600] = {};
    bool     tx_acquire_called = false;
    FakeCyclicChannel channel;

    // Response served by waitCyclicSlotView.
    CyclicSlotView resp_view{};
    uint8_t        resp_data[1600] = {};
    bool           resp_ok = true;
    int            wait_calls = 0;

    bool supportsCyclicFastPath() const override { return true; }
    uint64_t cyclicSlotToken(uint8_t) override { return 0; }

    bool sendCyclicDatagram(Command, uint8_t, uint16_t, uint16_t,
                            const void* data, uint16_t datalen,
                            bool) override {
        if (!send_ok) return false;
        std::memcpy(last_sent_frame, data, datalen);
        last_sent_len = datalen;
        return true;
    }

    bool waitCyclicSlotView(uint8_t, uint64_t, uint32_t,
                            CyclicSlotView& out) override {
        ++wait_calls;
        if (!resp_ok) return false;
        out = resp_view;
        return true;
    }

    uint8_t* acquireCyclicTxFrame() override {
        tx_acquire_called = true;
        return fake_tx_frame;
    }
    void composeCyclicHeader(uint8_t* frame, Command cmd, uint8_t slot,
                             uint16_t adp, uint16_t ado, uint16_t datalen,
                             bool) override {
        // Minimal but checkable header: mark cmd/idx/adp/ado/len.
        frame[0]  = 0xEE;
        frame[16] = static_cast<uint8_t>(cmd);
        frame[17] = static_cast<uint8_t>(kCyclicSlotBase + slot);
        std::memcpy(frame + 18, &adp, 2);
        std::memcpy(frame + 20, &ado, 2);
        std::memcpy(frame + 22, &datalen, 2);
    }
    bool sendCyclicFrame(uint32_t frame_len) override {
        std::memcpy(last_sent_frame, fake_tx_frame,
                    std::min<size_t>(frame_len, sizeof(last_sent_frame)));
        last_sent_len = static_cast<uint16_t>(frame_len);
        last_sent_idx = fake_tx_frame[17];
        return send_ok;
    }
    ICyclicChannel* cyclicChannel() override { return &channel; }

    bool writeRegister(uint16_t, uint16_t, const void*, uint16_t,
                       unsigned int) override { return true; }
    bool readRegister(uint16_t, uint16_t, void*, uint16_t,
                      unsigned int) override { return true; }
    bool sendSingleDatagram(Command, uint8_t, uint16_t, uint16_t,
                            const void*, uint16_t, bool) override {
        return true;
    }
    size_t sendMultiDatagram(const MultiDatagramSpec*, size_t) override {
        return 0;
    }
    bool waitForResponseIdx(uint8_t, unsigned int, RxDatagram&) override {
        return false;
    }
    uint8_t  allocIdx() override { return next_idx++; }
    uint16_t adpForSlaveIndex(uint16_t s) override { return s; }
};

// ============================================================================
// Fixture: one slave, 8B RxPDO + 8B TxPDO
// ============================================================================

class ProcessImageTest : public ::testing::Test {
protected:
    FakePDOTransport transport;
    LogicalAddressManager mgr{transport};
    PDOMapping mapping;
    uint8_t rx_app[8] = {};
    uint8_t tx_app[8] = {};

    void SetUp() override {
        mgr.init();
        SlaveConfig configs[kMaxPDOSlaves] = {};
        configs[0].configured = true;
        configs[0].sm[2] = SyncManagerConfig::process_output(0x1800, 8);
        configs[0].rxpdo_size = 8;
        configs[0].sm[3] = SyncManagerConfig::process_input(0x1C00, 8);
        configs[0].txpdo_size = 8;
        ASSERT_TRUE(mgr.buildAddressMap(configs, 1));

        mapping.add_rxpdo(0, rx_app, 8);
        mapping.add_txpdo(0, tx_app, 8);
    }
};

// ============================================================================
// computeImageOffsets
// ============================================================================

TEST_F(ProcessImageTest, ComputeImageOffsetsAssignsLayout) {
    int32_t offs[ProcessImage::kMaxEntries];
    const size_t n = mgr.computeImageOffsets(mapping, offs,
                                             ProcessImage::kMaxEntries);
    ASSERT_EQ(n, 2u);
    EXPECT_EQ(offs[0], 0);    // RxPDO at logical offset 0
    EXPECT_EQ(offs[1], 8);    // TxPDO follows the Rx region
}

TEST_F(ProcessImageTest, ImageExcludeStaysBuffered) {
    mapping.get_entry_mut(0)->image_exclude = true;
    int32_t offs[ProcessImage::kMaxEntries];
    mgr.computeImageOffsets(mapping, offs, ProcessImage::kMaxEntries);
    EXPECT_EQ(offs[0], -1);   // excluded → buffered
    EXPECT_EQ(offs[1], 8);    // unaffected neighbour
}

TEST_F(ProcessImageTest, DisabledEntriesGetNoOffset) {
    mapping.get_entry_mut(0)->enabled = false;
    int32_t offs[ProcessImage::kMaxEntries];
    mgr.computeImageOffsets(mapping, offs, ProcessImage::kMaxEntries);
    EXPECT_EQ(offs[0], -1);
    // TxPDO unaffected
    EXPECT_EQ(offs[1], 8);
}

// ============================================================================
// ProcessImage modes — direct unit coverage
// ============================================================================

TEST(ProcessImageModeTest, BufferedModeInactive) {
    ProcessImage img;
    ProcessImage::Config cfg;
    cfg.mode = ImageMode::Buffered;
    cfg.rx_bytes = 8; cfg.tx_bytes = 8;
    ASSERT_TRUE(img.configure(cfg));
    EXPECT_EQ(img.mode(), ImageMode::Buffered);
    EXPECT_EQ(img.outputWrite(), nullptr);
    EXPECT_EQ(img.inputRead(), nullptr);
}

TEST(ProcessImageModeTest, DirectModeInPlaceWrites) {
    ProcessImage img;
    int32_t offs[2] = {0, 8};
    ProcessImage::Config cfg;
    cfg.mode = ImageMode::Direct;
    cfg.rx_bytes = 8; cfg.tx_bytes = 8;
    cfg.entry_offsets = offs; cfg.entry_count = 2;
    ASSERT_TRUE(img.configure(cfg));

    uint8_t* w = img.outputWrite();
    ASSERT_NE(w, nullptr);
    const uint8_t* send = img.acquireSendImage();
    ASSERT_EQ(send, w);     // Direct: send buffer IS the write buffer
}

TEST(ProcessImageModeTest, TripleBufferedCoherentRotation) {
    ProcessImage img;
    ProcessImage::Config cfg;
    cfg.mode = ImageMode::TripleBuffered;
    cfg.rx_bytes = 8; cfg.tx_bytes = 8;
    ASSERT_TRUE(img.configure(cfg));

    // Write pattern A into the write buffer, commit.
    uint8_t* w0 = img.outputWrite();
    ASSERT_NE(w0, nullptr);
    w0[0] = 0xA0;
    img.commitOutputs();

    // The send image must carry pattern A.
    const uint8_t* s0 = img.acquireSendImage();
    ASSERT_NE(s0, nullptr);
    EXPECT_EQ(s0[0], 0xA0);

    // New write buffer is a *different* buffer — writing it must not
    // corrupt the in-flight send image.
    uint8_t* w1 = img.outputWrite();
    ASSERT_NE(w1, nullptr);
    EXPECT_NE(w1, s0);
    w1[0] = 0xB0;
    EXPECT_EQ(s0[0], 0xA0);    // send image undisturbed

    img.commitOutputs();
    const uint8_t* s1 = img.acquireSendImage();
    EXPECT_EQ(s1[0], 0xB0);
}

TEST(ProcessImageModeTest, TripleBufferedNoCommitKeepsLastSend) {
    ProcessImage img;
    ProcessImage::Config cfg;
    cfg.mode = ImageMode::TripleBuffered;
    cfg.rx_bytes = 8; cfg.tx_bytes = 8;
    ASSERT_TRUE(img.configure(cfg));

    img.outputWrite()[0] = 0x11;
    img.commitOutputs();
    const uint8_t* s0 = img.acquireSendImage();
    EXPECT_EQ(s0[0], 0x11);

    // Acquire again without a new commit — same image re-sent.
    const uint8_t* s1 = img.acquireSendImage();
    EXPECT_EQ(s1[0], 0x11);
}

TEST(ProcessImageModeTest, DoubleBufferedCarryForward) {
    ProcessImage img;
    ProcessImage::Config cfg;
    cfg.mode = ImageMode::DoubleBuffered;
    cfg.rx_bytes = 8; cfg.tx_bytes = 8;
    ASSERT_TRUE(img.configure(cfg));

    // Writer fills field A, commits.
    uint8_t* w0 = img.outputWrite();
    ASSERT_NE(w0, nullptr);
    w0[0] = 0xA0;
    img.commitOutputs();

    const uint8_t* s0 = img.acquireSendImage();
    ASSERT_NE(s0, nullptr);
    EXPECT_EQ(s0[0], 0xA0);

    // Carry-forward: the persistent write buffer keeps field A while the
    // writer only touches field B.
    uint8_t* w1 = img.outputWrite();
    EXPECT_EQ(w1, w0);              // stable pointer — no re-query needed
    EXPECT_EQ(w1[0], 0xA0);
    w1[3] = 0xB0;
    img.commitOutputs();

    const uint8_t* s1 = img.acquireSendImage();
    EXPECT_EQ(s1[0], 0xA0);         // carried forward
    EXPECT_EQ(s1[3], 0xB0);

    // No commit → resend the same snapshot.
    const uint8_t* s2 = img.acquireSendImage();
    EXPECT_EQ(s2[0], 0xA0);
    EXPECT_EQ(s2[3], 0xB0);
}

TEST(ProcessImageModeTest, DoubleBufferedBeginWriteBlocksCleanSnapshot) {
    ProcessImage img;
    ProcessImage::Config cfg;
    cfg.mode = ImageMode::DoubleBuffered;
    cfg.rx_bytes = 8; cfg.tx_bytes = 8;
    ASSERT_TRUE(img.configure(cfg));

    img.beginWrite();
    img.outputWrite()[0] = 0x77;
    // Mid-burst: seqlock marks writer-active; the send snapshot still
    // holds the old (zero) data and dirty was never set.
    const uint8_t* s = img.acquireSendImage();
    EXPECT_EQ(s[0], 0x00);          // not yet committed → resend old
    img.commitOutputs();
    const uint8_t* s2 = img.acquireSendImage();
    EXPECT_EQ(s2[0], 0x77);
}

TEST(ProcessImageModeTest, RotatingAttachesChannelFrame) {
    ProcessImage img;
    ProcessImage::Config cfg;
    cfg.mode = ImageMode::Rotating;
    cfg.rx_bytes = 8; cfg.tx_bytes = 8;
    ASSERT_TRUE(img.configure(cfg));

    // Before attach: no writable output.
    EXPECT_EQ(img.outputWrite(), nullptr);

    uint8_t frame[1600] = {};
    img.attachTxFrame(frame, 26);
    EXPECT_EQ(img.outputWrite(), frame + 26);
    EXPECT_EQ(img.rotatingFrameBase(), frame);
    // acquireSendImage returns null in Rotating (payload lives in frame).
    EXPECT_EQ(img.acquireSendImage(), nullptr);

    img.detachTxFrame();
    EXPECT_EQ(img.outputWrite(), nullptr);
    EXPECT_EQ(img.rotatingFrameBase(), nullptr);
}

TEST(ProcessImageModeTest, PublishInputCopyAndSeq) {
    ProcessImage img;
    ProcessImage::Config cfg;
    cfg.mode = ImageMode::Direct;
    cfg.rx_bytes = 8; cfg.tx_bytes = 8;
    ASSERT_TRUE(img.configure(cfg));

    EXPECT_EQ(img.inputSeq(), 0u);
    uint8_t payload[16];
    std::memset(payload, 0x5A, sizeof(payload));
    img.publishInputCopy(payload, 16);
    EXPECT_EQ(img.inputSeq(), 1u);
    const uint8_t* in = img.inputRead();
    ASSERT_NE(in, nullptr);
    EXPECT_EQ(in[15], 0x5A);

    // Second publish — copy is stable (owned bank), seq bumps.
    payload[15] = 0x77;
    img.publishInputCopy(payload, 16);
    EXPECT_EQ(img.inputSeq(), 2u);
    EXPECT_EQ(img.inputRead()[15], 0x77);
}

TEST(ProcessImageModeTest, PublishInputViewHoldsCookie) {
    ProcessImage img;
    FakeCyclicChannel ch;
    ProcessImage::Config cfg;
    cfg.mode = ImageMode::Direct;
    cfg.rx_bytes = 8; cfg.tx_bytes = 8;
    ASSERT_TRUE(img.configure(cfg));

    uint8_t payload[16] = {};
    payload[9] = 0x42;
    img.publishInputView(payload, 16, /*cookie=*/3, &ch);
    EXPECT_EQ(img.inputSeq(), 1u);
    EXPECT_EQ(img.inputRead(), payload);          // zero-copy view
    EXPECT_EQ(img.inputRead()[9], 0x42);
    EXPECT_EQ(ch.holds.load(), 1);                // cookie held

    // Re-publish releases the old cookie, holds the new one.
    img.publishInputView(payload, 16, 7, &ch);
    EXPECT_EQ(ch.holds.load(), 1);                // net: still 1 held

    img.configure({});                            // teardown releases
    EXPECT_EQ(ch.holds.load(), 0);
}

TEST(ProcessImageModeTest, EntryAccessorsRespectOffsets) {
    ProcessImage img;
    int32_t offs[2] = {0, 8};
    ProcessImage::Config cfg;
    cfg.mode = ImageMode::Direct;
    cfg.rx_bytes = 8; cfg.tx_bytes = 8;
    cfg.entry_offsets = offs; cfg.entry_count = 2;
    ASSERT_TRUE(img.configure(cfg));

    int32_t* out = img.outputPtr<int32_t>(0);
    ASSERT_NE(out, nullptr);
    *out = 0x11223344;
    EXPECT_EQ(img.outputWrite()[0], 0x44);        // little-endian

    EXPECT_EQ(img.outputPtr<int32_t>(7), nullptr);   // unmapped entry index
    // Entry 1 is a TxPDO (input) — writing it as an output must fail:
    // its offset (8) + 4B exceeds the rx region (8 bytes).
    EXPECT_EQ(img.outputPtr<int32_t>(1), nullptr);
    // And it IS readable as an input after a publish.
    uint8_t payload[16] = {};
    payload[8] = 0x5A;
    img.publishInputCopy(payload, 16);
    const int32_t* in = img.inputPtr<int32_t>(1);
    ASSERT_NE(in, nullptr);
    EXPECT_EQ(*in & 0xFF, 0x5A);
}

TEST(ProcessImageModeTest, EpochBumpsOnReconfigure) {
    ProcessImage img;
    ProcessImage::Config cfg;
    cfg.mode = ImageMode::Direct;
    cfg.rx_bytes = 4; cfg.tx_bytes = 4;
    ASSERT_TRUE(img.configure(cfg));
    const uint32_t e0 = img.epoch();
    ASSERT_TRUE(img.configure(cfg));
    EXPECT_GT(img.epoch(), e0);
}

// ============================================================================
// LAM cyclic exchange in image modes
// ============================================================================

TEST_F(ProcessImageTest, BufferedExchangeUsesAppBuffers) {
    ProcessImage img;   // unconfigured → legacy path
    rx_app[0] = 0xAA;
    // Response must carry the full image + nonzero WKC.
    transport.resp_view.payload = transport.resp_data;
    transport.resp_view.datalen = 16;
    transport.resp_view.wkc     = 1;
    ASSERT_TRUE(mgr.exchangeAllLRWCyclic(mapping, 200'000, &img));
    EXPECT_EQ(transport.last_sent_frame[0], 0xAA);   // gathered from app_buffer
}

TEST_F(ProcessImageTest, DirectModeSendsImageInPlace) {
    ProcessImage img;
    int32_t offs[ProcessImage::kMaxEntries];
    const size_t n = mgr.computeImageOffsets(mapping, offs,
                                             ProcessImage::kMaxEntries);
    ProcessImage::Config cfg;
    cfg.mode = ImageMode::Direct;
    cfg.rx_bytes = mgr.totalRxPDOBytes();
    cfg.tx_bytes = mgr.totalTxPDOBytes();
    cfg.entry_offsets = offs; cfg.entry_count = n;
    ASSERT_TRUE(img.configure(cfg));

    // App writes the output field directly into the image.
    img.outputWrite()[0] = 0xCC;
    img.outputWrite()[7] = 0xDD;

    transport.resp_view.payload = transport.resp_data;
    transport.resp_view.datalen = 16;
    transport.resp_view.wkc     = 1;
    transport.resp_data[8] = 0x99;                // TxPDO byte at offset 8

    ASSERT_TRUE(mgr.exchangeAllLRWCyclic(mapping, 200'000, &img));

    // Payload was the image itself — no app_buffer gather for mapped entries.
    EXPECT_EQ(transport.last_sent_frame[0], 0xCC);
    EXPECT_EQ(transport.last_sent_frame[7], 0xDD);
    EXPECT_EQ(transport.last_sent_len, 16);

    // Input image published via copy path (view.channel == nullptr).
    EXPECT_EQ(img.inputSeq(), 1u);
    ASSERT_NE(img.inputRead(), nullptr);
    EXPECT_EQ(img.inputRead()[8], 0x99);
    // tx_app untouched for the image-mapped entry.
    EXPECT_EQ(tx_app[0], 0);
}

TEST_F(ProcessImageTest, TripleBufferedExchangeCarriesCommittedData) {
    ProcessImage img;
    int32_t offs[ProcessImage::kMaxEntries];
    const size_t n = mgr.computeImageOffsets(mapping, offs,
                                             ProcessImage::kMaxEntries);
    ProcessImage::Config cfg;
    cfg.mode = ImageMode::TripleBuffered;
    cfg.rx_bytes = mgr.totalRxPDOBytes();
    cfg.tx_bytes = mgr.totalTxPDOBytes();
    cfg.entry_offsets = offs; cfg.entry_count = n;
    ASSERT_TRUE(img.configure(cfg));

    img.outputWrite()[0] = 0x33;
    img.commitOutputs();

    transport.resp_view.payload = transport.resp_data;
    transport.resp_view.datalen = 16;
    transport.resp_view.wkc     = 1;

    ASSERT_TRUE(mgr.exchangeAllLRWCyclic(mapping, 200'000, &img));
    EXPECT_EQ(transport.last_sent_frame[0], 0x33);
}

TEST_F(ProcessImageTest, RotatingModeSendsInPlaceFrame) {
    ProcessImage img;
    int32_t offs[ProcessImage::kMaxEntries];
    const size_t n = mgr.computeImageOffsets(mapping, offs,
                                             ProcessImage::kMaxEntries);
    ProcessImage::Config cfg;
    cfg.mode = ImageMode::Rotating;
    cfg.rx_bytes = mgr.totalRxPDOBytes();
    cfg.tx_bytes = mgr.totalTxPDOBytes();
    cfg.entry_offsets = offs; cfg.entry_count = n;
    ASSERT_TRUE(img.configure(cfg));

    transport.resp_view.payload = transport.resp_data;
    transport.resp_view.datalen = 16;
    transport.resp_view.wkc     = 1;

    // First exchange: no frame attached → exchange acquires + attaches.
    // The app wrote nothing, so the payload is whatever was in the fake
    // frame (zeros).  After the exchange a new frame is attached.
    ASSERT_TRUE(mgr.exchangeAllLRWCyclic(mapping, 200'000, &img));
    EXPECT_TRUE(transport.tx_acquire_called);
    // sendCyclicFrame captured a full frame: header at 0, payload at 26.
    EXPECT_EQ(transport.last_sent_idx, 0xF8);         // reserved slot idx
    EXPECT_EQ(transport.last_sent_frame[16], 0x0C);   // LRW command
    EXPECT_EQ(transport.last_sent_len, 26 + 16 + 2);

    // A frame is attached for the next cycle — app writes into it.
    uint8_t* w = img.outputWrite();
    ASSERT_NE(w, nullptr);
    w[0] = 0x77;
    transport.resp_view.wkc = 1;
    ASSERT_TRUE(mgr.exchangeAllLRWCyclic(mapping, 200'000, &img));
    EXPECT_EQ(transport.fake_tx_frame[26], 0x77);     // payload sent in place
}

TEST_F(ProcessImageTest, ForcedBufferedEntriesStillGathered) {
    // image_exclude on the RxPDO entry → it stays on app_buffer even in
    // Direct mode (FSoE-style staged PDOs).
    mapping.get_entry_mut(0)->image_exclude = true;

    ProcessImage img;
    int32_t offs[ProcessImage::kMaxEntries];
    const size_t n = mgr.computeImageOffsets(mapping, offs,
                                             ProcessImage::kMaxEntries);
    ProcessImage::Config cfg;
    cfg.mode = ImageMode::Direct;
    cfg.rx_bytes = mgr.totalRxPDOBytes();
    cfg.tx_bytes = mgr.totalTxPDOBytes();
    cfg.entry_offsets = offs; cfg.entry_count = n;
    ASSERT_TRUE(img.configure(cfg));

    rx_app[0] = 0x5B;   // app writes its buffer — not the image
    transport.resp_view.payload = transport.resp_data;
    transport.resp_view.datalen = 16;
    transport.resp_view.wkc     = 1;

    ASSERT_TRUE(mgr.exchangeAllLRWCyclic(mapping, 200'000, &img));
    EXPECT_EQ(transport.last_sent_frame[0], 0x5B);   // gathered despite image mode
}

TEST_F(ProcessImageTest, WkcZeroFails) {
    ProcessImage img;
    transport.resp_view.payload = transport.resp_data;
    transport.resp_view.datalen = 16;
    transport.resp_view.wkc     = 0;
    EXPECT_FALSE(mgr.exchangeAllLRWCyclic(mapping, 200'000, &img));
    EXPECT_EQ(mgr.getStats().wkc_errors, 1u);
}

TEST_F(ProcessImageTest, TimeoutFails) {
    transport.resp_ok = false;
    EXPECT_FALSE(mgr.exchangeAllLRWCyclic(mapping, 200'000, nullptr));
    EXPECT_EQ(mgr.getStats().timeout_errors, 1u);
}
