/**
 * @file test_process_image.cpp
 * @brief Unit tests for ProcessImage modes + LogicalAddressManager image-mode
 *        cyclic exchange (Direct / TripleBuffered / Rotating / Buffered).
 */

#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include <atomic>
#include <cstring>
#include <functional>
#include <thread>

#ifdef __linux__
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>
#endif

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

    // Per-slot send capture + per-slot responses (multi-slice tests).
    static constexpr int kSlots = 8;
    int            send_count = 0;
    uint8_t        sent_slot[kSlots]  = {};
    uint16_t       sent_adp[kSlots]   = {};
    uint16_t       sent_ado[kSlots]   = {};
    uint16_t       sent_len_[kSlots]  = {};
    uint8_t        sent_data[kSlots][1600] = {};
    bool           per_slot_resp = false;
    CyclicSlotView resp_slots[kSlots] = {};
    uint8_t        resp_slot_data[kSlots][1600] = {};
    size_t         fake_frame_payload = 0;   // 0 → default 1498
    // Invoked inside waitCyclicSlotView — mutation-injection seam for
    // the mapping-epoch guard.
    std::function<void()> on_wait;

    // Send-generation echo (stale-deposit ABA guard): the fake toggles
    // tx_gen per send like Master does; echo_tx_gen stamps it into served
    // views, and serve_stale_once makes the FIRST wait return the
    // previous generation — a deposit that outlived its own cycle.
    uint8_t tx_gen = 0;
    bool echo_tx_gen = false;
    bool serve_stale_once = false;
    bool stale_served = false;

    bool supportsCyclicFastPath() const override { return true; }
    uint64_t cyclicSlotToken(uint8_t slot) override { return slot; }
    // 0 unless echoing — transports without a stamped header disable the
    // check by reporting gen 0 (matching unmarked views).
    uint8_t cyclicSlotGen(uint8_t) override { return echo_tx_gen ? tx_gen : 0; }
    size_t maxEtherCATPayloadPerFrame() const override {
        return fake_frame_payload ? fake_frame_payload : 1498;
    }

    bool sendCyclicDatagram(Command, uint8_t slot, uint16_t adp,
                            uint16_t ado,
                            const void* data, uint16_t datalen,
                            bool) override {
        if (!send_ok) return false;
        tx_gen ^= 1;   // per-slot in reality; single-slot fixture is fine
        std::memcpy(last_sent_frame, data, datalen);
        last_sent_len = datalen;
        if (send_count < kSlots) {
            const int i = send_count++;
            sent_slot[i] = slot; sent_adp[i] = adp; sent_ado[i] = ado;
            sent_len_[i] = datalen;
            std::memcpy(sent_data[i], data, datalen);
        }
        return true;
    }

    bool waitCyclicSlotView(uint8_t slot, uint64_t, uint32_t,
                            CyclicSlotView& out) override {
        ++wait_calls;
        if (on_wait) on_wait();
        if (!resp_ok) return false;
        out = per_slot_resp ? resp_slots[slot] : resp_view;
        if (echo_tx_gen) {
            if (serve_stale_once && !stale_served) {
                stale_served = true;
                out.gen = tx_gen ^ 1;   // previous generation — stale
            } else {
                out.gen = tx_gen;
            }
        }
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
    uint8_t* rx_app = nullptr;   ///< views into entry storage (Q1)
    uint8_t* tx_app = nullptr;

    void SetUp() override {
        mgr.init();
        SlaveConfig configs[kMaxPDOSlaves] = {};
        configs[0].configured = true;
        configs[0].sm[2] = SyncManagerConfig::process_output(0x1800, 8);
        configs[0].rxpdo_size = 8;
        configs[0].sm[3] = SyncManagerConfig::process_input(0x1C00, 8);
        configs[0].txpdo_size = 8;
        ASSERT_TRUE(mgr.buildAddressMap(configs, 1));

        mapping.add_rxpdo(0, 8);
        mapping.add_txpdo(0, 8);
        // NOTE: no eager entryDataMut here — taking a storage pointer marks
        // the entry storage_bound (bridged in image modes).  Tests opt in
        // via bindStorage() so image-mode coverage exercises pure-image
        // entries by default.
    }

    /// Bind both entries to the buffered storage path — what device
    /// layers (CiA402Drive::registerPDOBuffers etc.) do.
    void bindStorage() {
        rx_app = mapping.entryDataMut(0);
        tx_app = mapping.entryDataMut(1);
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
    bindStorage();
    rx_app[0] = 0xAA;
    // Response must carry the full image + nonzero WKC.
    transport.resp_view.payload = transport.resp_data;
    transport.resp_view.datalen = 16;
    transport.resp_view.wkc     = 3;
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
    transport.resp_view.wkc     = 3;
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
    // Unbound entry stays image-mapped: no scatter into entry storage.
    EXPECT_EQ(mapping.get_entry(1)->storage[0], 0);
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
    transport.resp_view.wkc     = 3;

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
    transport.resp_view.wkc     = 3;

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
    transport.resp_view.wkc = 3;
    ASSERT_TRUE(mgr.exchangeAllLRWCyclic(mapping, 200'000, &img));
    EXPECT_EQ(transport.fake_tx_frame[26], 0x77);     // payload sent in place
}

TEST_F(ProcessImageTest, ForcedBufferedEntriesStillGathered) {
    // image_exclude on the RxPDO entry → it stays on app_buffer even in
    // Direct mode (FSoE-style staged PDOs).
    bindStorage();
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
    transport.resp_view.wkc     = 3;

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

// ============================================================================
// Mapping-epoch guard — mid-exchange mapping mutation (slave recovery)
// ============================================================================

TEST_F(ProcessImageTest, MappingMutationDuringCollectSkipsScatter) {
    // A slave-recovery re-registration mutating the mapping while the
    // response wait is in flight must not scatter into entries whose
    // logical offsets are now stale.
    bindStorage();
    transport.resp_view.payload = transport.resp_data;
    transport.resp_view.datalen = 16;
    transport.resp_view.wkc     = 3;
    for (int i = 0; i < 16; ++i) transport.resp_data[i] =
        static_cast<uint8_t>(0x40 + i);

    transport.on_wait = [this]() { mapping.remove_entries_for_slave(0); };
    EXPECT_TRUE(mgr.exchangeAllLRWCyclic(mapping, 200'000, nullptr));
    EXPECT_EQ(mapping.entry_count(), 0u);
    // Vacated slots were zeroed by the removal — the skipped scatter must
    // NOT have written response bytes into them.
    EXPECT_EQ(tx_app[0], 0);
    EXPECT_EQ(mgr.getStats().timeout_errors, 0u);
    EXPECT_EQ(mgr.getStats().success, 1u);
}

TEST_F(ProcessImageTest, StableMappingCollectsNormally) {
    // Control: no mutation → scatter happens (TxPDO storage updated).
    bindStorage();
    transport.resp_view.payload = transport.resp_data;
    transport.resp_view.datalen = 16;
    transport.resp_view.wkc     = 3;
    for (int i = 0; i < 16; ++i) transport.resp_data[i] =
        static_cast<uint8_t>(0x40 + i);

    EXPECT_TRUE(mgr.exchangeAllLRWCyclic(mapping, 200'000, nullptr));
    // TxPDO entry (entry 1, logical offset 8) collects bytes 8..15.
    EXPECT_EQ(tx_app[0], 0x48);
    EXPECT_EQ(tx_app[7], 0x4F);
    EXPECT_EQ(mgr.getStats().success, 1u);
}

// ============================================================================
// Storage-bound bridge — device accessors stay coherent in image modes
// ============================================================================

TEST_F(ProcessImageTest, StorageBoundBridgeKeepsDirectModeCoherent) {
    // Bind BEFORE configure (the normal device-registration order):
    // computeImageOffsets keeps bound entries out of the image table, and
    // the exchange bridges them — storage writes reach the wire, wire
    // responses reach storage.
    bindStorage();

    ProcessImage img;
    int32_t offs[ProcessImage::kMaxEntries];
    const size_t n = mgr.computeImageOffsets(mapping, offs,
                                             ProcessImage::kMaxEntries);
    EXPECT_EQ(offs[0], -1);   // bound → storage-owned, not image-addressed
    EXPECT_EQ(offs[1], -1);
    ProcessImage::Config cfg;
    cfg.mode = ImageMode::Direct;
    cfg.rx_bytes = mgr.totalRxPDOBytes();
    cfg.tx_bytes = mgr.totalTxPDOBytes();
    cfg.entry_offsets = offs; cfg.entry_count = n;
    ASSERT_TRUE(img.configure(cfg));

    rx_app[0] = 0xB7;                       // device-level write
    transport.resp_view.payload = transport.resp_data;
    transport.resp_view.datalen = 16;
    transport.resp_view.wkc     = 3;
    transport.resp_data[8] = 0x5A;          // TxPDO response byte

    ASSERT_TRUE(mgr.exchangeAllLRWCyclic(mapping, 200'000, &img));
    EXPECT_EQ(transport.last_sent_frame[0], 0xB7);  // storage → wire
    EXPECT_EQ(tx_app[0], 0x5A);                     // wire → storage
    EXPECT_EQ(img.inputSeq(), 1u);                  // image still published
}

TEST_F(ProcessImageTest, LateBoundEntryStillBridges) {
    // Accessor taken AFTER the image was configured: the offsets table
    // image-maps the entry, but storage_bound wins at exchange time —
    // late-binding (e.g. recovery re-registration) stays coherent.
    ProcessImage img;
    int32_t offs[ProcessImage::kMaxEntries];
    const size_t n = mgr.computeImageOffsets(mapping, offs,
                                             ProcessImage::kMaxEntries);
    ASSERT_EQ(offs[0], 0);    // unbound → image-mapped
    ProcessImage::Config cfg;
    cfg.mode = ImageMode::Direct;
    cfg.rx_bytes = mgr.totalRxPDOBytes();
    cfg.tx_bytes = mgr.totalTxPDOBytes();
    cfg.entry_offsets = offs; cfg.entry_count = n;
    ASSERT_TRUE(img.configure(cfg));

    bindStorage();          // device binds its buffers post-configure
    rx_app[0] = 0x9C;
    img.outputWrite()[0] = 0x11;            // image byte loses to storage
    transport.resp_view.payload = transport.resp_data;
    transport.resp_view.datalen = 16;
    transport.resp_view.wkc     = 3;
    transport.resp_data[8] = 0x42;

    ASSERT_TRUE(mgr.exchangeAllLRWCyclic(mapping, 200'000, &img));
    EXPECT_EQ(transport.last_sent_frame[0], 0x9C);
    EXPECT_EQ(tx_app[0], 0x42);
}

// ============================================================================
// Stale-deposit ABA — send-generation echo rejects old responses
// ============================================================================

TEST_F(ProcessImageTest, StaleDepositRejectedThenRealResponseCollected) {
    bindStorage();
    transport.echo_tx_gen = true;
    transport.serve_stale_once = true;   // first wait returns old gen
    transport.resp_view.payload = transport.resp_data;
    transport.resp_view.datalen = 16;
    transport.resp_view.wkc     = 3;
    for (int i = 0; i < 16; ++i) transport.resp_data[i] =
        static_cast<uint8_t>(0x40 + i);

    ASSERT_TRUE(mgr.exchangeAllLRWCyclic(mapping, 200'000, nullptr));
    EXPECT_EQ(mgr.getStats().stale_responses, 1u);
    EXPECT_EQ(mgr.getStats().timeout_errors, 0u);
    EXPECT_EQ(mgr.getStats().success, 1u);
    EXPECT_EQ(transport.wait_calls, 2);   // stale consumed + one re-wait
    EXPECT_EQ(tx_app[0], 0x48);           // fresh data scattered
}

TEST_F(ProcessImageTest, StaleDepositTimeoutCountsMiss) {
    // Stale arrives, no real response follows → honest miss, stale still
    // consumed so it can't alias the next cycle either.
    bindStorage();
    transport.echo_tx_gen = true;
    transport.serve_stale_once = true;
    transport.resp_view.payload = transport.resp_data;
    transport.resp_view.datalen = 16;
    transport.resp_view.wkc     = 3;
    transport.on_wait = [this]() {
        if (transport.stale_served) transport.resp_ok = false;
    };

    EXPECT_FALSE(mgr.exchangeAllLRWCyclic(mapping, 200'000, nullptr));
    EXPECT_EQ(mgr.getStats().stale_responses, 1u);
    EXPECT_EQ(mgr.getStats().timeout_errors, 1u);
}

// ============================================================================
// Multi-slice cyclic exchange (image > one frame)
// ============================================================================

TEST_F(ProcessImageTest, MultiSliceSplitsImageAcrossSlots) {
    // 16-byte image, 8-byte slices → 2 slots.
    transport.fake_frame_payload = 20;   // maxSlice = 20 - 12 = 8
    ProcessImage img;
    ProcessImage::Config cfg;
    cfg.mode = ImageMode::Direct;
    cfg.rx_bytes = 8; cfg.tx_bytes = 8;
    int32_t offs[2] = {0, 8};
    cfg.entry_offsets = offs; cfg.entry_count = 2;
    ASSERT_TRUE(img.configure(cfg));

    uint8_t* out = img.outputWrite();
    ASSERT_NE(out, nullptr);
    for (int i = 0; i < 8; ++i) out[i] = 0xA0 + i;

    transport.per_slot_resp = true;
    // Derived WKC per slice (Q6): slice 0 covers the RxPDO region (+1),
    // slice 1 the TxPDO region (+2).
    const uint16_t slice_wkc[2] = {1, 2};
    for (int s = 0; s < 2; ++s) {
        for (int i = 0; i < 8; ++i)
            transport.resp_slot_data[s][i] = 0x10 * s + i;
        transport.resp_slot_data[s][8] = 0xEE;
        transport.resp_slots[s].payload = transport.resp_slot_data[s];
        transport.resp_slots[s].datalen = 8;
        transport.resp_slots[s].wkc     = slice_wkc[s];
    }

    ASSERT_TRUE(mgr.exchangeAllLRWCyclic(mapping, 200'000, &img));
    EXPECT_EQ(mgr.cyclicSliceCount(), 2);
    ASSERT_EQ(transport.send_count, 2);
    EXPECT_EQ(transport.sent_slot[0], 0);
    EXPECT_EQ(transport.sent_slot[1], 1);
    EXPECT_EQ(transport.sent_len_[0], 8);
    EXPECT_EQ(transport.sent_len_[1], 8);
    // Slice 1's logical address = base + 8 → adp low word differs by 8.
    EXPECT_EQ(transport.sent_adp[1],
              static_cast<uint16_t>(transport.sent_adp[0] + 8));

    // Input image assembled across both slices, one publish.
    const uint8_t* in = img.inputRead();
    ASSERT_NE(in, nullptr);
    for (int i = 0; i < 8; ++i) EXPECT_EQ(in[i], i);
    for (int i = 0; i < 8; ++i) EXPECT_EQ(in[8 + i], 0x10 + i);
    EXPECT_EQ(img.inputSeq(), 1u);
}

TEST_F(ProcessImageTest, SplitPhaseSendDoesNotWaitThenCollect) {
    ProcessImage img;
    ProcessImage::Config cfg;
    cfg.mode = ImageMode::Direct;
    cfg.rx_bytes = 8; cfg.tx_bytes = 8;
    int32_t offs[2] = {0, 8};
    cfg.entry_offsets = offs; cfg.entry_count = 2;
    ASSERT_TRUE(img.configure(cfg));

    transport.resp_view.payload = transport.resp_data;
    transport.resp_view.datalen = 16;
    transport.resp_view.wkc     = 3;

    ASSERT_TRUE(mgr.cyclicSend(mapping, &img, 200'000));
    EXPECT_TRUE(mgr.cyclicExchangePending());
    EXPECT_EQ(transport.wait_calls, 0);   // send must not block
    EXPECT_EQ(transport.send_count, 1);

    ASSERT_TRUE(mgr.cyclicCollect(mapping, &img));
    EXPECT_FALSE(mgr.cyclicExchangePending());
    EXPECT_EQ(transport.wait_calls, 1);
    EXPECT_EQ(img.inputSeq(), 1u);
}

TEST_F(ProcessImageTest, ExpectedWkcLearnedThenEnforced) {
    ProcessImage img;
    ProcessImage::Config cfg;
    cfg.mode = ImageMode::Direct;
    cfg.rx_bytes = 8; cfg.tx_bytes = 8;
    int32_t offs[2] = {0, 8};
    cfg.entry_offsets = offs; cfg.entry_count = 2;
    ASSERT_TRUE(img.configure(cfg));
    mgr.setStrictWkc(true);

    // First exchange learns the expected WKC.
    transport.resp_view.payload = transport.resp_data;
    transport.resp_view.datalen = 16;
    transport.resp_view.wkc     = 3;
    ASSERT_TRUE(mgr.exchangeAllLRWCyclic(mapping, 200'000, &img));

    // A later exchange with a different WKC (partial slave dropout)
    // must fail — wkc==0 alone cannot catch this.
    transport.resp_view.wkc = 2;
    EXPECT_FALSE(mgr.exchangeAllLRWCyclic(mapping, 200'000, &img));
    EXPECT_GE(mgr.getStats().wkc_errors, 1u);
}

TEST_F(ProcessImageTest, StrictWkcOffAcceptsVaryingWkc) {
    ProcessImage img;
    ProcessImage::Config cfg;
    cfg.mode = ImageMode::Direct;
    cfg.rx_bytes = 8; cfg.tx_bytes = 8;
    int32_t offs[2] = {0, 8};
    cfg.entry_offsets = offs; cfg.entry_count = 2;
    ASSERT_TRUE(img.configure(cfg));
    mgr.setStrictWkc(false);

    transport.resp_view.payload = transport.resp_data;
    transport.resp_view.datalen = 16;
    transport.resp_view.wkc     = 3;
    ASSERT_TRUE(mgr.exchangeAllLRWCyclic(mapping, 200'000, &img));
    transport.resp_view.wkc = 7;
    EXPECT_TRUE(mgr.exchangeAllLRWCyclic(mapping, 200'000, &img));
    transport.resp_view.wkc = 0;
    EXPECT_FALSE(mgr.exchangeAllLRWCyclic(mapping, 200'000, &img));
}

TEST_F(ProcessImageTest, OversizedImageBeyondSlotCountFailsClean) {
    // 16-byte image, 2-byte slices → 8 slices needed but kNumCyclicSlots=6.
    transport.fake_frame_payload = 14;   // maxSlice = 2
    EXPECT_FALSE(mgr.exchangeAllLRWCyclic(mapping, 200'000, nullptr));
    EXPECT_GE(mgr.getStats().send_errors, 1u);
}

// ============================================================================
// commitInput / waitInput (multi-part publish + external cycle tick)
// ============================================================================

TEST_F(ProcessImageTest, CommitInputPublishesStagedBankOnce) {
    ProcessImage img;
    ProcessImage::Config cfg;
    cfg.mode = ImageMode::Direct;
    cfg.rx_bytes = 8; cfg.tx_bytes = 8;
    ASSERT_TRUE(img.configure(cfg));

    uint8_t* bank = img.inputWriteBank();
    ASSERT_NE(bank, nullptr);
    std::memset(bank, 0xCC, 16);
    bank[3] = 0x42;
    img.commitInput();

    const uint8_t* in = img.inputRead();
    ASSERT_NE(in, nullptr);
    EXPECT_EQ(in[3], 0x42);
    EXPECT_EQ(in[0], 0xCC);
    EXPECT_EQ(img.inputSeq(), 1u);
}

TEST_F(ProcessImageTest, WaitInputWakesOnPublish) {
    ProcessImage img;
    ProcessImage::Config cfg;
    cfg.mode = ImageMode::Direct;
    cfg.rx_bytes = 8; cfg.tx_bytes = 8;
    ASSERT_TRUE(img.configure(cfg));

    std::atomic<bool> woke{false};
    const uint64_t seq0 = img.inputSeq();
    std::thread waiter([&] {
        woke = img.waitInput(seq0, 2'000'000'000u);  // 5 s budget
    });
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    img.commitInput();
    waiter.join();
    EXPECT_TRUE(woke);
}

TEST_F(ProcessImageTest, WaitInputReturnsImmediatelyOnNewSeq) {
    ProcessImage img;
    ProcessImage::Config cfg;
    cfg.mode = ImageMode::Direct;
    cfg.rx_bytes = 8; cfg.tx_bytes = 8;
    ASSERT_TRUE(img.configure(cfg));
    img.commitInput();
    // seq already advanced past 0 — returns instantly.
    EXPECT_TRUE(img.waitInput(0, 1'000'000));
}

TEST_F(ProcessImageTest, WaitInputTimesOutCleanly) {
    ProcessImage img;
    ProcessImage::Config cfg;
    cfg.mode = ImageMode::Direct;
    cfg.rx_bytes = 8; cfg.tx_bytes = 8;
    ASSERT_TRUE(img.configure(cfg));
    const uint64_t seq = img.inputSeq();
    const auto t0 = std::chrono::steady_clock::now();
    EXPECT_FALSE(img.waitInput(seq, 5'000'000));  // 5 ms
    const auto el = std::chrono::steady_clock::now() - t0;
    EXPECT_LT(el, std::chrono::milliseconds(500));
}

// ============================================================================
// EntryHandle — epoch-checked entry access
// ============================================================================

TEST_F(ProcessImageTest, EntryHandleResolvesAndInvalidates) {
    ProcessImage img;
    ProcessImage::Config cfg;
    cfg.mode = ImageMode::Direct;
    cfg.rx_bytes = 8; cfg.tx_bytes = 8;
    int32_t offs[2] = {0, 8};
    cfg.entry_offsets = offs; cfg.entry_count = 2;
    ASSERT_TRUE(img.configure(cfg));

    EntryHandle h = img.entryHandle(0);
    ASSERT_TRUE(h.valid());
    ASSERT_TRUE(h.imageMapped());
    EXPECT_EQ(h.offset, 0);

    uint8_t* p = img.outputPtrRaw(h);
    ASSERT_NE(p, nullptr);
    p[0] = 0x77;
    EXPECT_EQ(img.outputWrite()[0], 0x77);

    // Reconfigure → epoch bumps → the stale handle refuses to resolve.
    ASSERT_TRUE(img.configure(cfg));
    EXPECT_EQ(img.outputPtrRaw(h), nullptr);

    // Fresh handle resolves again.
    EntryHandle h2 = img.entryHandle(0);
    EXPECT_NE(img.outputPtrRaw(h2), nullptr);
}

TEST_F(ProcessImageTest, EntryHandleOutOfRangeIsInvalid) {
    ProcessImage img;
    ProcessImage::Config cfg;
    cfg.mode = ImageMode::Direct;
    cfg.rx_bytes = 8; cfg.tx_bytes = 8;
    int32_t offs[1] = {0};
    cfg.entry_offsets = offs; cfg.entry_count = 1;
    ASSERT_TRUE(img.configure(cfg));
    EXPECT_FALSE(img.entryHandle(7).valid());
}

// ============================================================================
// shm export + attach (process-external motion source)
// ============================================================================

#ifdef __linux__
TEST_F(ProcessImageTest, ShmExportAndAttachShareRegions) {
    const char* name = "tether-test-img";
    ProcessImage::Config cfg;
    cfg.mode = ImageMode::Direct;
    cfg.rx_bytes = 8; cfg.tx_bytes = 8;
    cfg.shm_name = name;

    ProcessImage server;
    ASSERT_TRUE(server.configure(cfg));
    ASSERT_TRUE(server.shmBacked());

    ProcessImage client;
    ASSERT_TRUE(client.attachShared(name));
    ASSERT_TRUE(client.shmBacked());
    EXPECT_EQ(client.outputBytes(), 8u);
    EXPECT_EQ(client.inputBytes(), 8u);

    // Client writes outputs — server's acquireSendImage sees them.
    uint8_t* cout = client.outputWrite();
    ASSERT_NE(cout, nullptr);
    cout[2] = 0x5A;
    const uint8_t* sout = server.acquireSendImage();
    ASSERT_NE(sout, nullptr);
    EXPECT_EQ(sout[2], 0x5A);

    // Server publishes inputs — client's inputRead sees them.
    uint8_t* bank = server.inputWriteBank();
    bank[9] = 0xB7;
    server.commitInput();
    const uint8_t* cin = client.inputRead();
    ASSERT_NE(cin, nullptr);
    EXPECT_EQ(cin[9], 0xB7);

    // Cross-"process" wait: client waits on seq, server publish wakes it.
    std::atomic<bool> woke{false};
    const uint64_t seq0 = client.inputSeq();
    std::thread waiter([&] {
        woke = client.waitInput(seq0, 2'000'000'000u);
    });
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    server.commitInput();
    waiter.join();
    EXPECT_TRUE(woke);

    // commitOutputs on the attached side bumps the shm output counter.
    const uint32_t oseq = server.outputSeq();
    client.commitOutputs();
    EXPECT_EQ(server.outputSeq(), oseq + 1);
}

TEST_F(ProcessImageTest, AttachSharedBadNameFails) {
    ProcessImage img;
    EXPECT_FALSE(img.attachShared("tether-nonexistent-image-xyz"));
}

// ---- Q20: attach refuses a half-written/poisoned export -------------

TEST_F(ProcessImageTest, ShmHandshakeRejectsIncompleteExport) {
    const char* path = "/tether-test-shm-half";
    // Craft a segment with valid magic/version but no ready/checksum —
    // the exporter "died" mid-export.  Attach must refuse.
    int fd = ::shm_open(path, O_RDWR | O_CREAT | O_TRUNC, 0600);
    ASSERT_GE(fd, 0);
    ASSERT_EQ(::ftruncate(fd, ShmImageLayout::kHeaderPadded), 0);
    void* map = ::mmap(nullptr, ShmImageLayout::kHeaderPadded,
                       PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
    ASSERT_NE(map, MAP_FAILED);
    auto* hdr = static_cast<ShmImageHeader*>(map);
    std::memset(map, 0, ShmImageLayout::kHeaderPadded);
    hdr->magic = kShmMagic;
    hdr->version = kShmVersion;
    hdr->rx_bytes = 8; hdr->tx_bytes = 8;
    hdr->header_bytes = sizeof(ShmImageHeader);
    // ready stays 0 — incomplete export.

    ProcessImage img;
    EXPECT_FALSE(img.attachShared("tether-test-shm-half"));
    ::munmap(map, ShmImageLayout::kHeaderPadded);
    ::close(fd);
    ::shm_unlink(path);
}

TEST_F(ProcessImageTest, ShmHandshakeRejectsBadChecksum) {
    const char* path = "/tether-test-shm-badcksum";
    int fd = ::shm_open(path, O_RDWR | O_CREAT | O_TRUNC, 0600);
    ASSERT_GE(fd, 0);
    ASSERT_EQ(::ftruncate(fd, ShmImageLayout::kHeaderPadded), 0);
    void* map = ::mmap(nullptr, ShmImageLayout::kHeaderPadded,
                       PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
    ASSERT_NE(map, MAP_FAILED);
    auto* hdr = static_cast<ShmImageHeader*>(map);
    std::memset(map, 0, ShmImageLayout::kHeaderPadded);
    hdr->magic = kShmMagic;
    hdr->version = kShmVersion;
    hdr->rx_bytes = 8; hdr->tx_bytes = 8;
    hdr->header_bytes = sizeof(ShmImageHeader);
    hdr->checksum.store(0xDEADBEEF);       // garbage
    hdr->ready.store(1);                   // claims ready — checksum lies
    ProcessImage img;
    EXPECT_FALSE(img.attachShared("tether-test-shm-badcksum"));
    ::munmap(map, ShmImageLayout::kHeaderPadded);
    ::close(fd);
    ::shm_unlink(path);
}

// ---- Q4: exported entry table ----------------------------------------

TEST_F(ProcessImageTest, ShmEntryTableExportedToAttacher) {
    const char* name = "tether-test-entrytbl";
    ProcessImage::Config cfg;
    cfg.mode = ImageMode::Direct;
    cfg.rx_bytes = 16; cfg.tx_bytes = 8;
    cfg.shm_name = name;
    const int32_t offs[2] = {0, 8};        // two rx entries
    cfg.entry_offsets = offs;
    cfg.entry_count = 2;

    ProcessImage server;
    ASSERT_TRUE(server.configure(cfg));

    ShmEntryDesc rows[2]{};
    rows[0].index = 0; rows[0].slave_index = 2;
    rows[0].pdo_index = 0x1600; rows[0].direction = 1; // RxPDO
    rows[0].offset = 0; rows[0].size = 8;
    std::snprintf(rows[0].label, sizeof(rows[0].label), "rx:s2:0x1600");
    rows[1].index = 1; rows[1].slave_index = 2;
    rows[1].pdo_index = 0x1601; rows[1].direction = 1;
    rows[1].offset = 8; rows[1].size = 8;
    server.exportEntryTable(rows, 2);
    EXPECT_TRUE(server.shmFlags() & kShmFlagEntryTable);

    ProcessImage client;
    ASSERT_TRUE(client.attachShared(name));
    EXPECT_TRUE(client.shmFlags() & kShmFlagEntryTable);
    ASSERT_EQ(client.shmEntryCount(), 2u);
    const ShmEntryDesc* tbl = client.shmEntryTable();
    ASSERT_NE(tbl, nullptr);
    EXPECT_EQ(tbl[0].pdo_index, 0x1600u);
    EXPECT_EQ(tbl[1].offset, 8);
    EXPECT_STREQ(tbl[0].label, "rx:s2:0x1600");
    // And the offsets feed entryHandle() — typed access without hardcoding.
    EXPECT_EQ(client.entryOffset(0), 0);
    EXPECT_EQ(client.entryOffset(1), 8);
    ::shm_unlink("/tether-test-entrytbl");
}

// ---- Q5: optional region seqlocks ------------------------------------

TEST_F(ProcessImageTest, ShmSeqlockDetectsWriterMidBurst) {
    const char* name = "tether-test-seqlock";
    ProcessImage::Config cfg;
    cfg.mode = ImageMode::Direct;
    cfg.rx_bytes = 16; cfg.tx_bytes = 8;
    cfg.shm_name = name;
    cfg.shm_seqlock = true;

    ProcessImage server;
    ASSERT_TRUE(server.configure(cfg));
    EXPECT_TRUE(server.shmFlags() & kShmFlagSeqlock);

    ProcessImage client;
    ASSERT_TRUE(client.attachShared(name));
    EXPECT_TRUE(client.shmFlags() & kShmFlagSeqlock);

    // Quiescent read: begin/end agree.
    const uint32_t s0 = client.shmReadBegin(ProcessImage::ShmRegion::Output);
    EXPECT_TRUE(client.shmReadEnd(ProcessImage::ShmRegion::Output, s0));

    // Writer mid-burst → reader's stamp goes stale.
    server.shmWriteBegin(ProcessImage::ShmRegion::Output);
    EXPECT_FALSE(client.shmReadEnd(ProcessImage::ShmRegion::Output, s0));
    server.shmWriteEnd(ProcessImage::ShmRegion::Output);
    const uint32_t s1 = client.shmReadBegin(ProcessImage::ShmRegion::Output);
    EXPECT_TRUE(client.shmReadEnd(ProcessImage::ShmRegion::Output, s1));
    EXPECT_NE(s0, s1);                     // seq advanced by the write burst
    ::shm_unlink("/tether-test-seqlock");
}

TEST_F(ProcessImageTest, ShmSeqlockDisabledIsNoOp) {
    const char* name = "tether-test-seqlock-off";
    ProcessImage::Config cfg;
    cfg.mode = ImageMode::Direct;
    cfg.rx_bytes = 16; cfg.tx_bytes = 8;
    cfg.shm_name = name;                   // shm_seqlock unset → flag off
    ProcessImage server;
    ASSERT_TRUE(server.configure(cfg));
    EXPECT_FALSE(server.shmFlags() & kShmFlagSeqlock);
    // No-op API: reads always "consistent", writes free.
    EXPECT_TRUE(server.shmReadEnd(ProcessImage::ShmRegion::Output, 0));
    server.shmWriteBegin(ProcessImage::ShmRegion::Output);
    server.shmWriteEnd(ProcessImage::ShmRegion::Output);
    ::shm_unlink("/tether-test-seqlock-off");
}

// ---- Q26: cross-process producer stats --------------------------------

TEST_F(ProcessImageTest, ShmProducerStatsCrossProcess) {
    const char* name = "tether-test-prodstats";
    ProcessImage::Config cfg;
    cfg.mode = ImageMode::Direct;
    cfg.rx_bytes = 8; cfg.tx_bytes = 8;
    cfg.shm_name = name;
    ProcessImage server;
    ASSERT_TRUE(server.configure(cfg));

    ProcessImage client;
    ASSERT_TRUE(client.attachShared(name));
    const int slot = client.claimProducerSlot();
    ASSERT_GE(slot, 1);
    client.triggerSend(static_cast<uint32_t>(slot));
    client.triggerSend(static_cast<uint32_t>(slot));
    client.triggerSend();                  // anonymous
    // The exporter sees both buckets on the shared words.
    EXPECT_EQ(server.producerTriggerCount(static_cast<uint32_t>(slot)), 2u);
    EXPECT_EQ(server.producerTriggerCount(0), 1u);
    EXPECT_EQ(server.sendSeq(), 3u);
    ::shm_unlink("/tether-test-prodstats");
}
#endif
