/**
 * @file test_process_image.cpp
 * @brief Coverage for src/ethercat/ProcessImage.cpp — configure() per
 *        ImageMode, entry offsets/handles, output/input accessors,
 *        publish paths (view/copy/commit), producer send triggers,
 *        futex wait/wake, and shm export fallback.
 */

#include <gtest/gtest.h>

#include <atomic>
#include <cstdint>
#include <cstring>
#include <thread>
#include <vector>

#include "tether/ethercat/ProcessImage.hpp"

using namespace EtherCAT;

namespace {

// ============================================================================
// configure() per mode
// ============================================================================

TEST(ProcessImageCov, UnconfiguredIsEmpty) {
    ProcessImage img;
    EXPECT_FALSE(img.configured());
    EXPECT_EQ(img.imageBytes(), 0u);
    EXPECT_EQ(img.outputWrite(), nullptr);
    EXPECT_EQ(img.inputRead(), nullptr);
    EXPECT_FALSE(img.waitInput(0, 1'000'000));   // not configured → false
}

TEST(ProcessImageCov, ConfigureBufferedMode) {
    ProcessImage img;
    ProcessImage::Config cfg;
    cfg.mode = ImageMode::Buffered;
    cfg.rx_bytes = 32;
    cfg.tx_bytes = 16;
    EXPECT_TRUE(img.configure(cfg));
    EXPECT_TRUE(img.configured());
    EXPECT_EQ(img.mode(), ImageMode::Buffered);
    EXPECT_EQ(img.outputBytes(), 32u);
    EXPECT_EQ(img.inputBytes(), 16u);
    EXPECT_EQ(img.imageBytes(), 48u);
}

TEST(ProcessImageCov, ConfigureDirectMode) {
    ProcessImage img;
    ProcessImage::Config cfg;
    cfg.mode = ImageMode::Direct;
    cfg.rx_bytes = 64;
    cfg.tx_bytes = 32;
    ASSERT_TRUE(img.configure(cfg));
    EXPECT_EQ(img.mode(), ImageMode::Direct);
    uint8_t* out = img.outputWrite();
    ASSERT_NE(out, nullptr);
    out[0] = 0xAB;
    EXPECT_EQ(img.outputWrite()[0], 0xAB);
}

TEST(ProcessImageCov, ConfigureDoubleBuffered) {
    ProcessImage img;
    ProcessImage::Config cfg;
    cfg.mode = ImageMode::DoubleBuffered;
    cfg.rx_bytes = 16;
    cfg.tx_bytes = 16;
    ASSERT_TRUE(img.configure(cfg));
    EXPECT_EQ(img.mode(), ImageMode::DoubleBuffered);
    img.beginWrite();
    img.outputWrite()[0] = 0x5A;
    img.commitOutputs();
    uint8_t* cur = img.outputWrite();
    ASSERT_NE(cur, nullptr);
}

TEST(ProcessImageCov, ConfigureTripleBuffered) {
    ProcessImage img;
    ProcessImage::Config cfg;
    cfg.mode = ImageMode::TripleBuffered;
    cfg.rx_bytes = 16;
    cfg.tx_bytes = 16;
    ASSERT_TRUE(img.configure(cfg));
    EXPECT_EQ(img.mode(), ImageMode::TripleBuffered);
    img.beginWrite();
    img.outputWrite()[0] = 0x77;
    img.commitOutputs();
    SUCCEED();
}

TEST(ProcessImageCov, ConfigureRotating) {
    ProcessImage img;
    ProcessImage::Config cfg;
    cfg.mode = ImageMode::Rotating;
    cfg.rx_bytes = 16;
    cfg.tx_bytes = 16;
    ASSERT_TRUE(img.configure(cfg));
    EXPECT_EQ(img.mode(), ImageMode::Rotating);
    // Rotating owns nothing — payload comes from attachTxFrame
    std::vector<uint8_t> frame(64, 0);
    img.attachTxFrame(frame.data(), 26);
    EXPECT_EQ(img.rotatingFrameBase(), frame.data());
    img.detachTxFrame();
    EXPECT_EQ(img.rotatingFrameBase(), nullptr);
}

TEST(ProcessImageCov, ConfigureZeroSizeFails) {
    ProcessImage img;
    ProcessImage::Config cfg;
    cfg.mode = ImageMode::Direct;
    cfg.rx_bytes = 0;
    cfg.tx_bytes = 0;
    EXPECT_FALSE(img.configure(cfg));
}

TEST(ProcessImageCov, ReconfigureBumpsEpoch) {
    ProcessImage img;
    ProcessImage::Config cfg;
    cfg.mode = ImageMode::Direct;
    cfg.rx_bytes = 8;
    cfg.tx_bytes = 8;
    ASSERT_TRUE(img.configure(cfg));
    uint32_t e0 = img.epoch();
    ASSERT_TRUE(img.configure(cfg));
    EXPECT_NE(img.epoch(), e0);
}

// ============================================================================
// Entry offsets / handles / typed accessors
// ============================================================================

TEST(ProcessImageCov, EntryOffsetsAndHandles) {
    ProcessImage img;
    int32_t offs[3] = {0, 8, -1};
    ProcessImage::Config cfg;
    cfg.mode = ImageMode::Direct;
    cfg.rx_bytes = 16;
    cfg.tx_bytes = 16;
    cfg.entry_offsets = offs;
    cfg.entry_count = 3;
    ASSERT_TRUE(img.configure(cfg));

    EXPECT_EQ(img.entryOffset(0), 0);
    EXPECT_EQ(img.entryOffset(1), 8);
    EXPECT_EQ(img.entryOffset(2), -1);
    EXPECT_EQ(img.entryOffset(9), -1);        // out of range

    EntryHandle h = img.entryHandle(1);
    EXPECT_TRUE(h.imageMapped());
    EXPECT_EQ(h.offset, 8);
    EXPECT_EQ(h.index, 1u);
    EntryHandle hb = img.entryHandle(2);
    EXPECT_FALSE(hb.imageMapped());
}

TEST(ProcessImageCov, NullOffsetsAllBuffered) {
    ProcessImage img;
    ProcessImage::Config cfg;
    cfg.mode = ImageMode::Direct;
    cfg.rx_bytes = 8;
    cfg.tx_bytes = 8;
    cfg.entry_offsets = nullptr;
    cfg.entry_count = 2;
    ASSERT_TRUE(img.configure(cfg));
    EXPECT_EQ(img.entryOffset(0), -1);
    EXPECT_EQ(img.entryOffset(1), -1);
}

TEST(ProcessImageCov, EntryCountCapped) {
    ProcessImage img;
    ProcessImage::Config cfg;
    cfg.mode = ImageMode::Direct;
    cfg.rx_bytes = 8;
    cfg.tx_bytes = 8;
    cfg.entry_count = 100000;                 // > kMaxEntries
    ASSERT_TRUE(img.configure(cfg));
    EXPECT_EQ(img.entryOffset(50000), -1);
}

TEST(ProcessImageCov, TypedOutputPtr) {
    ProcessImage img;
    int32_t offs[3] = {0, 4, 12};
    ProcessImage::Config cfg;
    cfg.mode = ImageMode::Direct;
    cfg.rx_bytes = 16;
    cfg.tx_bytes = 16;
    cfg.entry_offsets = offs;
    cfg.entry_count = 3;
    ASSERT_TRUE(img.configure(cfg));

    uint32_t* p = img.outputPtr<uint32_t>(0);
    ASSERT_NE(p, nullptr);
    *p = 0xDEADBEEF;
    EXPECT_EQ(*reinterpret_cast<uint32_t*>(img.outputWrite()), 0xDEADBEEFu);

    // offset + sizeof(T) must fit in rx_bytes: 12+8=20 > 16 → nullptr
    EXPECT_EQ(img.outputPtr<uint64_t>(2), nullptr);
    // entry 1 at offset 4, u64 fits (4+8=12 ≤ 16)
    EXPECT_NE(img.outputPtr<uint64_t>(1), nullptr);
}

TEST(ProcessImageCov, TypedInputPtrBeforePublish) {
    ProcessImage img;
    ProcessImage::Config cfg;
    cfg.mode = ImageMode::Direct;
    cfg.rx_bytes = 8;
    cfg.tx_bytes = 8;
    ASSERT_TRUE(img.configure(cfg));
    // No input published yet → nullptr
    EXPECT_EQ(img.inputPtr<uint8_t>(0), nullptr);
}

// ============================================================================
// Publish paths
// ============================================================================

TEST(ProcessImageCov, PublishInputCopyAndRead) {
    ProcessImage img;
    ProcessImage::Config cfg;
    cfg.mode = ImageMode::Direct;
    cfg.rx_bytes = 8;
    cfg.tx_bytes = 8;
    ASSERT_TRUE(img.configure(cfg));

    uint8_t payload[8] = {0xDE, 0xAD, 0xBE, 0xEF, 1, 2, 3, 4};
    img.publishInputCopy(payload, sizeof(payload));
    const uint8_t* in = img.inputRead();
    ASSERT_NE(in, nullptr);
    EXPECT_EQ(in[0], 0xDE);
    EXPECT_EQ(in[7], 0x04);
    EXPECT_GE(img.inputSeq(), 1u);
}

TEST(ProcessImageCov, PublishInputViewZeroCopy) {
    ProcessImage img;
    ProcessImage::Config cfg;
    cfg.mode = ImageMode::Direct;
    cfg.rx_bytes = 8;
    cfg.tx_bytes = 8;
    ASSERT_TRUE(img.configure(cfg));

    static uint8_t wire[8] = {9, 9, 9, 9, 9, 9, 9, 9};
    img.publishInputView(wire, 8, 0, nullptr);
    EXPECT_EQ(img.inputRead(), wire);        // zero-copy: same pointer
}

TEST(ProcessImageCov, CommitInputPublishesBank) {
    ProcessImage img;
    ProcessImage::Config cfg;
    cfg.mode = ImageMode::Direct;
    cfg.rx_bytes = 8;
    cfg.tx_bytes = 8;
    ASSERT_TRUE(img.configure(cfg));

    uint8_t* bank = img.inputWriteBank();
    ASSERT_NE(bank, nullptr);
    bank[0] = 0x42;
    img.commitInput();
    ASSERT_NE(img.inputRead(), nullptr);
    EXPECT_EQ(img.inputRead()[0], 0x42);
}

TEST(ProcessImageCov, WaitInputWokenByPublish) {
    ProcessImage img;
    ProcessImage::Config cfg;
    cfg.mode = ImageMode::Direct;
    cfg.rx_bytes = 8;
    cfg.tx_bytes = 8;
    ASSERT_TRUE(img.configure(cfg));

    uint8_t p[8] = {};
    img.publishInputCopy(p, 8);
    uint64_t seq = img.inputSeq();

    std::atomic<bool> woke{false};
    std::thread t([&] {
        woke = img.waitInput(seq, 500'000'000);
    });
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    img.publishInputCopy(p, 8);              // bumps seq → wakes waiter
    t.join();
    EXPECT_TRUE(woke);
}

TEST(ProcessImageCov, WaitInputTimeout) {
    ProcessImage img;
    ProcessImage::Config cfg;
    cfg.mode = ImageMode::Direct;
    cfg.rx_bytes = 8;
    cfg.tx_bytes = 8;
    ASSERT_TRUE(img.configure(cfg));
    uint8_t p[8] = {};
    img.publishInputCopy(p, 8);
    uint64_t seq = img.inputSeq();
    // No publish within timeout → false
    EXPECT_FALSE(img.waitInput(seq, 5'000'000));
    // seq already advanced → immediate true
    img.publishInputCopy(p, 8);
    EXPECT_TRUE(img.waitInput(seq, 1'000'000));
}

// ============================================================================
// Producer send triggers
// ============================================================================

TEST(ProcessImageCov, ProducerSlots) {
    ProcessImage img;
    ProcessImage::Config cfg;
    cfg.mode = ImageMode::Direct;
    cfg.rx_bytes = 8;
    cfg.tx_bytes = 8;
    ASSERT_TRUE(img.configure(cfg));

    int a = img.claimProducerSlot();
    int b = img.claimProducerSlot();
    EXPECT_GE(a, 0);
    EXPECT_GT(b, a);
    img.triggerSend(static_cast<uint32_t>(a));
    EXPECT_GE(img.producerTriggerCount(static_cast<uint32_t>(a)), 1u);
}

TEST(ProcessImageCov, WaitSendWokenByTrigger) {
    ProcessImage img;
    ProcessImage::Config cfg;
    cfg.mode = ImageMode::Direct;
    cfg.rx_bytes = 8;
    cfg.tx_bytes = 8;
    ASSERT_TRUE(img.configure(cfg));

    std::atomic<bool> woke{false};
    std::thread t([&] {
        woke = img.waitSend(0, 500'000'000);
    });
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    img.triggerSend(0);
    t.join();
    EXPECT_TRUE(woke);
}

TEST(ProcessImageCov, WaitSendTimeout) {
    ProcessImage img;
    ProcessImage::Config cfg;
    cfg.mode = ImageMode::Direct;
    cfg.rx_bytes = 8;
    cfg.tx_bytes = 8;
    ASSERT_TRUE(img.configure(cfg));
    EXPECT_FALSE(img.waitSend(0, 5'000'000));
}

// ============================================================================
// SHM export — falls back gracefully if segment can't be created
// ============================================================================

TEST(ProcessImageCov, ShmExportOrFallback) {
    ProcessImage img;
    ProcessImage::Config cfg;
    cfg.mode = ImageMode::Direct;
    cfg.rx_bytes = 32;
    cfg.tx_bytes = 16;
    cfg.shm_name = "tether-test-img-cov";
    cfg.shm_seqlock = true;
    cfg.shm_entry_capacity = 8;
    // Either maps the segment (Direct over shm) or falls back — both OK
    img.configure(cfg);
    EXPECT_TRUE(img.configured());
    // Non-compact (image_bytes > rx+tx) skips export
    ProcessImage img2;
    cfg.image_bytes = 128;
    cfg.shm_name = "tether-test-img-cov2";
    img2.configure(cfg);
    EXPECT_TRUE(img2.configured());
}

}  // namespace
