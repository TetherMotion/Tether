/**
 * @file test_vlan_router.cpp
 * @brief Comprehensive unit tests for EtherCAT::VLANRouter.
 *
 * Tests cover:
 *   - TX encapsulation (with and without VLAN)
 *   - RX decapsulation and routing by VLAN ID
 *   - Unencapsulated RX routing
 *   - Multiple masters sharing a VLAN ID
 *   - Frame dropping when no master matches
 *   - Malformed / truncated frames
 *   - Thread-safe add/remove under traffic
 *   - Query APIs (networkInterfaceFor, mastersForVlanId, entries, masterCount)
 *   - Duplicate registration update
 *   - EtherType name lookup
 */

#include <gtest/gtest.h>

#include <atomic>
#include <cstring>
#include <thread>
#include <vector>

#include "tether/ethercat/EtherCATMaster.hpp"
#include "tether/ethercat/EtherCATTypes.hpp"
#include "tether/ethercat/VLANRouter.hpp"

using namespace EtherCAT;

// ============================================================================
// Test helpers
// ============================================================================

static constexpr uint8_t kDstMac[6] = {0x01, 0x01, 0x05, 0x00, 0x00, 0x00};
static constexpr uint8_t kSrcMac[6]  = {0x26, 0x8a, 0x07, 0x6e, 0x63, 0x60};

struct CapturedFrame {
    const EtherCATMaster* master = nullptr;
    std::vector<uint8_t> data;
};

struct SpyBackend {
    std::vector<std::vector<uint8_t>> tx_frames;

    NetworkInterface iface{};

    SpyBackend() {
        iface.send = [this](const uint8_t* data, size_t len) -> bool {
            tx_frames.emplace_back(data, data + len);
            return true;
        };
        iface.receive = [](uint8_t*, size_t, size_t* out_len) -> bool {
            if (out_len) *out_len = 0;
            return false;
        };
    }
};

/**
 * @brief Build a minimal Ethernet frame with the given EtherType and payload.
 */
static std::vector<uint8_t> buildFrame(uint16_t ether_type,
                                        const std::vector<uint8_t>& payload)
{
    std::vector<uint8_t> frame;
    frame.reserve(14 + payload.size());
    frame.insert(frame.end(), kDstMac, kDstMac + 6);
    frame.insert(frame.end(), kSrcMac, kSrcMac + 6);
    frame.push_back(static_cast<uint8_t>(ether_type >> 8));
    frame.push_back(static_cast<uint8_t>(ether_type & 0xFF));
    frame.insert(frame.end(), payload.begin(), payload.end());
    return frame;
}

/**
 * @brief Build a VLAN-tagged Ethernet frame.
 *
 * @param vid       12-bit VLAN ID
 * @param inner_et  EtherType after the VLAN tag
 * @param payload   Bytes following the inner EtherType
 */
static std::vector<uint8_t> buildTaggedFrame(uint16_t vid,
                                              uint16_t inner_et,
                                              const std::vector<uint8_t>& payload)
{
    std::vector<uint8_t> frame;
    frame.reserve(18 + payload.size());
    frame.insert(frame.end(), kDstMac, kDstMac + 6);
    frame.insert(frame.end(), kSrcMac, kSrcMac + 6);
    // TPID 0x8100
    frame.push_back(0x81);
    frame.push_back(0x00);
    // TCI: VID only, PCP=0, DEI=0
    uint16_t tci = vid & 0x0FFFu;
    frame.push_back(static_cast<uint8_t>(tci >> 8));
    frame.push_back(static_cast<uint8_t>(tci & 0xFF));
    // Inner EtherType
    frame.push_back(static_cast<uint8_t>(inner_et >> 8));
    frame.push_back(static_cast<uint8_t>(inner_et & 0xFF));
    frame.insert(frame.end(), payload.begin(), payload.end());
    return frame;
}

// ============================================================================
// Test fixture
// ============================================================================

class VLANRouterTest : public ::testing::Test {
protected:
    void SetUp() override {
        backend_ = std::make_unique<SpyBackend>();
        router_ = std::make_unique<VLANRouter>();
        router_->setBackend(&backend_->iface);

        // Capture every delivered frame instead of calling handleRxFrame
        router_->setDeliverFunction(
            [this](EtherCATMaster* m, const uint8_t* d, size_t l) {
                CapturedFrame cf;
                cf.master = m;
                cf.data.assign(d, d + l);
                std::lock_guard<std::mutex> lock(capture_mutex_);
                captured_.push_back(std::move(cf));
            });
    }

    void TearDown() override {
        router_->clearMasters();
    }

    std::vector<CapturedFrame> captured() const {
        std::lock_guard<std::mutex> lock(capture_mutex_);
        return captured_;
    }

    void clearCaptured() {
        std::lock_guard<std::mutex> lock(capture_mutex_);
        captured_.clear();
    }

    std::unique_ptr<SpyBackend> backend_;
    std::unique_ptr<VLANRouter> router_;

    mutable std::mutex capture_mutex_;
    std::vector<CapturedFrame> captured_;
};

// ============================================================================
// Construction / lifecycle
// ============================================================================

TEST_F(VLANRouterTest, DefaultConstructionHasNoMasters) {
    EXPECT_EQ(router_->masterCount(), 0u);
    EXPECT_TRUE(router_->entries().empty());
}

TEST_F(VLANRouterTest, ClearMastersRemovesAll) {
    auto m = std::make_shared<EtherCATMaster>();
    router_->addMaster(m, 100, 100);
    EXPECT_EQ(router_->masterCount(), 1u);
    router_->clearMasters();
    EXPECT_EQ(router_->masterCount(), 0u);
    EXPECT_EQ(router_->networkInterfaceFor(m.get()), nullptr);
}

// ============================================================================
// TX: encapsulation
// ============================================================================

TEST_F(VLANRouterTest, TxEncapsulatesWithVlanTag) {
    auto master = std::make_shared<EtherCATMaster>();
    router_->addMaster(master, std::nullopt, 100);

    NetworkInterface* iface = router_->networkInterfaceFor(master.get());
    ASSERT_NE(iface, nullptr);
    ASSERT_TRUE(iface->send);

    std::vector<uint8_t> payload(20, 0xAB);
    auto raw = buildFrame(0x88A4, payload);
    ASSERT_TRUE(iface->send(raw.data(), raw.size()));

    ASSERT_EQ(backend_->tx_frames.size(), 1u);
    const auto& tx = backend_->tx_frames[0];
    ASSERT_EQ(tx.size(), raw.size() + 4);  // +4 for VLAN tag

    // MACs unchanged
    EXPECT_EQ(std::memcmp(tx.data(), kDstMac, 6), 0);
    EXPECT_EQ(std::memcmp(tx.data() + 6, kSrcMac, 6), 0);

    // TPID = 0x8100
    EXPECT_EQ(tx[12], 0x81);
    EXPECT_EQ(tx[13], 0x00);

    // TCI = VID 100
    uint16_t tci = static_cast<uint16_t>((tx[14] << 8) | tx[15]);
    EXPECT_EQ(tci & 0x0FFFu, 100u);

    // Original EtherType follows
    EXPECT_EQ(tx[16], 0x88);
    EXPECT_EQ(tx[17], 0xA4);

    // Payload follows
    EXPECT_EQ(std::memcmp(tx.data() + 18, payload.data(), payload.size()), 0);
}

TEST_F(VLANRouterTest, TxWithoutVlanPassesThroughUnchanged) {
    auto master = std::make_shared<EtherCATMaster>();
    router_->addMaster(master, std::nullopt, std::nullopt);

    NetworkInterface* iface = router_->networkInterfaceFor(master.get());
    ASSERT_NE(iface, nullptr);

    std::vector<uint8_t> payload(20, 0xCD);
    auto raw = buildFrame(0x88A4, payload);
    ASSERT_TRUE(iface->send(raw.data(), raw.size()));

    ASSERT_EQ(backend_->tx_frames.size(), 1u);
    EXPECT_EQ(backend_->tx_frames[0].size(), raw.size());
    EXPECT_EQ(std::memcmp(backend_->tx_frames[0].data(), raw.data(), raw.size()), 0);
}

TEST_F(VLANRouterTest, TxMultipleMastersDifferentVlans) {
    auto m1 = std::make_shared<EtherCATMaster>();
    auto m2 = std::make_shared<EtherCATMaster>();
    router_->addMaster(m1, std::nullopt, 100);
    router_->addMaster(m2, std::nullopt, 200);

    NetworkInterface* iface1 = router_->networkInterfaceFor(m1.get());
    NetworkInterface* iface2 = router_->networkInterfaceFor(m2.get());
    ASSERT_NE(iface1, nullptr);
    ASSERT_NE(iface2, nullptr);

    std::vector<uint8_t> payload(8, 0x11);
    auto raw = buildFrame(0x88A4, payload);

    iface1->send(raw.data(), raw.size());
    iface2->send(raw.data(), raw.size());

    ASSERT_EQ(backend_->tx_frames.size(), 2u);

    uint16_t tci1 = static_cast<uint16_t>((backend_->tx_frames[0][14] << 8) |
                                           backend_->tx_frames[0][15]);
    uint16_t tci2 = static_cast<uint16_t>((backend_->tx_frames[1][14] << 8) |
                                           backend_->tx_frames[1][15]);
    EXPECT_EQ(tci1 & 0x0FFFu, 100u);
    EXPECT_EQ(tci2 & 0x0FFFu, 200u);
}

TEST_F(VLANRouterTest, TxShortFrameRejected) {
    auto master = std::make_shared<EtherCATMaster>();
    router_->addMaster(master, std::nullopt, 100);

    NetworkInterface* iface = router_->networkInterfaceFor(master.get());
    ASSERT_NE(iface, nullptr);

    uint8_t short_frame[10] = {0};
    EXPECT_FALSE(iface->send(short_frame, sizeof(short_frame)));
}

// ============================================================================
// RX: decapsulation and routing
// ============================================================================

TEST_F(VLANRouterTest, RxTaggedFrameRoutedToMatchingMaster) {
    auto master = std::make_shared<EtherCATMaster>();
    router_->addMaster(master, 100, std::nullopt);

    std::vector<uint8_t> payload(12, 0x42);
    auto tagged = buildTaggedFrame(100, 0x88A4, payload);
    router_->processRxFrame(tagged.data(), tagged.size());

    auto caps = captured();
    ASSERT_EQ(caps.size(), 1u);
    EXPECT_EQ(caps[0].master, master.get());

    // Delivered frame should be decapsulated (no VLAN tag)
    EXPECT_EQ(caps[0].data.size(), tagged.size() - 4);
    EXPECT_EQ(caps[0].data[12], 0x88);
    EXPECT_EQ(caps[0].data[13], 0xA4);
    EXPECT_EQ(std::memcmp(caps[0].data.data() + 14, payload.data(), payload.size()), 0);
}

TEST_F(VLANRouterTest, RxTaggedFrameDroppedWhenNoMatch) {
    auto master = std::make_shared<EtherCATMaster>();
    router_->addMaster(master, 100, std::nullopt);

    auto tagged = buildTaggedFrame(999, 0x88A4, {0x42});
    router_->processRxFrame(tagged.data(), tagged.size());

    EXPECT_TRUE(captured().empty());
}

TEST_F(VLANRouterTest, UndefinedTargetReceivesUnmatchedTaggedFrames) {
    auto specific = std::make_shared<EtherCATMaster>();
    auto catch_all = std::make_shared<EtherCATMaster>();
    router_->addMaster(specific, 100, std::nullopt);
    ASSERT_TRUE(router_->setUndefinedTarget(catch_all, std::nullopt, false));

    // VID 999 has no exact match -> goes to undefined target
    auto tagged = buildTaggedFrame(999, 0x88A4, {0x42});
    router_->processRxFrame(tagged.data(), tagged.size());

    auto caps = captured();
    ASSERT_EQ(caps.size(), 1u);
    EXPECT_EQ(caps[0].master, catch_all.get());
    EXPECT_EQ(caps[0].data.size(), tagged.size() - 4);
}

TEST_F(VLANRouterTest, UndefinedTargetDoesNotStealExactMatches) {
    auto specific = std::make_shared<EtherCATMaster>();
    auto catch_all = std::make_shared<EtherCATMaster>();
    router_->addMaster(specific, 100, std::nullopt);
    ASSERT_TRUE(router_->setUndefinedTarget(catch_all, std::nullopt, false));

    // VID 100 has an exact match -> only specific receives it
    auto tagged = buildTaggedFrame(100, 0x88A4, {0x42});
    router_->processRxFrame(tagged.data(), tagged.size());

    auto caps = captured();
    ASSERT_EQ(caps.size(), 1u);
    EXPECT_EQ(caps[0].master, specific.get());
}

TEST_F(VLANRouterTest, UndefinedTargetRejectsSecondWithoutReplace) {
    auto m1 = std::make_shared<EtherCATMaster>();
    auto m2 = std::make_shared<EtherCATMaster>();
    EXPECT_TRUE(router_->setUndefinedTarget(m1, std::nullopt, false));
    EXPECT_FALSE(router_->setUndefinedTarget(m2, std::nullopt, false));
}

TEST_F(VLANRouterTest, UndefinedTargetReplaceTrueOverwrites) {
    auto m1 = std::make_shared<EtherCATMaster>();
    auto m2 = std::make_shared<EtherCATMaster>();
    EXPECT_TRUE(router_->setUndefinedTarget(m1, std::nullopt, false));
    EXPECT_TRUE(router_->setUndefinedTarget(m2, std::nullopt, true));
    EXPECT_EQ(router_->undefinedTarget().get(), m2.get());
}

TEST_F(VLANRouterTest, UndefinedTargetClearRemovesIt) {
    auto m = std::make_shared<EtherCATMaster>();
    router_->setUndefinedTarget(m, std::nullopt, false);
    router_->clearUndefinedTarget();
    EXPECT_EQ(router_->undefinedTarget(), nullptr);
    EXPECT_EQ(router_->undefinedNetworkInterface(), nullptr);
}

TEST_F(VLANRouterTest, UndefinedTargetTxEncapsulates) {
    auto m = std::make_shared<EtherCATMaster>();
    router_->setUndefinedTarget(m, 77, false);

    NetworkInterface* iface = router_->undefinedNetworkInterface();
    ASSERT_NE(iface, nullptr);
    ASSERT_TRUE(iface->send);

    auto raw = buildFrame(0x88A4, {0x11});
    ASSERT_TRUE(iface->send(raw.data(), raw.size()));

    ASSERT_EQ(backend_->tx_frames.size(), 1u);
    uint16_t tci = static_cast<uint16_t>((backend_->tx_frames[0][14] << 8) |
                                          backend_->tx_frames[0][15]);
    EXPECT_EQ(tci & 0x0FFFu, 77u);
}

TEST_F(VLANRouterTest, RxUntaggedFrameRoutedToNulloptMaster) {
    auto master = std::make_shared<EtherCATMaster>();
    router_->addMaster(master, std::nullopt, std::nullopt);

    std::vector<uint8_t> payload(12, 0x77);
    auto raw = buildFrame(0x88A4, payload);
    router_->processRxFrame(raw.data(), raw.size());

    auto caps = captured();
    ASSERT_EQ(caps.size(), 1u);
    EXPECT_EQ(caps[0].master, master.get());
    EXPECT_EQ(caps[0].data.size(), raw.size());
    EXPECT_EQ(std::memcmp(caps[0].data.data(), raw.data(), raw.size()), 0);
}

TEST_F(VLANRouterTest, RxUntaggedFrameDroppedForVlanOnlyMaster) {
    auto master = std::make_shared<EtherCATMaster>();
    router_->addMaster(master, 100, std::nullopt);

    auto raw = buildFrame(0x88A4, {0x77});
    router_->processRxFrame(raw.data(), raw.size());

    EXPECT_TRUE(captured().empty());
}

TEST_F(VLANRouterTest, RxMultipleMastersShareVlanId) {
    auto m1 = std::make_shared<EtherCATMaster>();
    auto m2 = std::make_shared<EtherCATMaster>();
    router_->addMaster(m1, 100, std::nullopt);
    router_->addMaster(m2, 100, std::nullopt);

    auto tagged = buildTaggedFrame(100, 0x88A4, {0x55});
    router_->processRxFrame(tagged.data(), tagged.size());

    auto caps = captured();
    ASSERT_EQ(caps.size(), 2u);
    EXPECT_EQ(caps[0].master, m1.get());
    EXPECT_EQ(caps[1].master, m2.get());
}

TEST_F(VLANRouterTest, RxTaggedFrameWithNonEtherCATInnerStillRouted) {
    auto master = std::make_shared<EtherCATMaster>();
    router_->addMaster(master, 50, std::nullopt);

    auto tagged = buildTaggedFrame(50, 0x0800, {0xAA, 0xBB});  // inner = IPv4
    router_->processRxFrame(tagged.data(), tagged.size());

    auto caps = captured();
    ASSERT_EQ(caps.size(), 1u);
    EXPECT_EQ(caps[0].master, master.get());
    // Inner EtherType should be preserved
    EXPECT_EQ(caps[0].data[12], 0x08);
    EXPECT_EQ(caps[0].data[13], 0x00);
}

// ============================================================================
// RX: malformed frames
// ============================================================================

TEST_F(VLANRouterTest, RxTooShortFrameIgnored) {
    auto master = std::make_shared<EtherCATMaster>();
    router_->addMaster(master, std::nullopt, std::nullopt);

    uint8_t shorty[10] = {0};
    router_->processRxFrame(shorty, sizeof(shorty));
    EXPECT_TRUE(captured().empty());
}

TEST_F(VLANRouterTest, RxTruncatedVlanTagIgnored) {
    auto master = std::make_shared<EtherCATMaster>();
    router_->addMaster(master, 100, std::nullopt);

    // Frame has TPID 0x8100 but no TCI bytes
    std::vector<uint8_t> bad;
    bad.insert(bad.end(), kDstMac, kDstMac + 6);
    bad.insert(bad.end(), kSrcMac, kSrcMac + 6);
    bad.push_back(0x81);
    bad.push_back(0x00);
    // Missing TCI
    router_->processRxFrame(bad.data(), bad.size());
    EXPECT_TRUE(captured().empty());
}

TEST_F(VLANRouterTest, RxNullDataIgnored) {
    router_->processRxFrame(nullptr, 64);
    EXPECT_TRUE(captured().empty());
}

// ============================================================================
// Query APIs
// ============================================================================

TEST_F(VLANRouterTest, NetworkInterfaceForUnknownReturnsNull) {
    auto master = std::make_shared<EtherCATMaster>();
    EXPECT_EQ(router_->networkInterfaceFor(master.get()), nullptr);
}

TEST_F(VLANRouterTest, MastersForVlanIdReturnsCorrectMasters) {
    auto m1 = std::make_shared<EtherCATMaster>();
    auto m2 = std::make_shared<EtherCATMaster>();
    auto m3 = std::make_shared<EtherCATMaster>();
    router_->addMaster(m1, 100, std::nullopt);
    router_->addMaster(m2, 100, std::nullopt);
    router_->addMaster(m3, 200, std::nullopt);

    auto r100 = router_->mastersForVlanId(100);
    ASSERT_EQ(r100.size(), 2u);
    EXPECT_EQ(r100[0].get(), m1.get());
    EXPECT_EQ(r100[1].get(), m2.get());

    auto r200 = router_->mastersForVlanId(200);
    ASSERT_EQ(r200.size(), 1u);
    EXPECT_EQ(r200[0].get(), m3.get());

    auto r999 = router_->mastersForVlanId(999);
    EXPECT_TRUE(r999.empty());
}

TEST_F(VLANRouterTest, EntriesSnapshotIsCorrect) {
    auto m1 = std::make_shared<EtherCATMaster>();
    auto m2 = std::make_shared<EtherCATMaster>();
    router_->addMaster(m1, 100, 200);
    router_->addMaster(m2, std::nullopt, std::nullopt);

    auto entries = router_->entries();
    ASSERT_EQ(entries.size(), 2u);

    EXPECT_EQ(entries[0].master.get(), m1.get());
    EXPECT_TRUE(entries[0].rx_vlan_range.has_value());
    EXPECT_EQ(entries[0].rx_vlan_range->start, 100u);
    EXPECT_EQ(entries[0].rx_vlan_range->end, 100u);
    EXPECT_TRUE(entries[0].tx_vlan_id.has_value());
    EXPECT_EQ(entries[0].tx_vlan_id.value(), 200u);

    EXPECT_EQ(entries[1].master.get(), m2.get());
    EXPECT_FALSE(entries[1].rx_vlan_range.has_value());
    EXPECT_FALSE(entries[1].tx_vlan_id.has_value());
}

TEST_F(VLANRouterTest, EntriesExcludesUndefinedTarget) {
    auto m = std::make_shared<EtherCATMaster>();
    router_->setUndefinedTarget(m, std::nullopt, false);
    EXPECT_EQ(router_->masterCount(), 0u);
    EXPECT_TRUE(router_->entries().empty());
}

TEST_F(VLANRouterTest, MasterCountTracksAddsAndRemoves) {
    auto m1 = std::make_shared<EtherCATMaster>();
    auto m2 = std::make_shared<EtherCATMaster>();

    EXPECT_EQ(router_->masterCount(), 0u);
    router_->addMaster(m1, 1, 1);
    EXPECT_EQ(router_->masterCount(), 1u);
    router_->addMaster(m2, 2, 2);
    EXPECT_EQ(router_->masterCount(), 2u);
    router_->removeMaster(m1.get());
    EXPECT_EQ(router_->masterCount(), 1u);
    router_->removeMaster(m2.get());
    EXPECT_EQ(router_->masterCount(), 0u);
}

// ============================================================================
// Duplicate registration
// ============================================================================

TEST_F(VLANRouterTest, AddingSameMasterUpdatesVlans) {
    auto m = std::make_shared<EtherCATMaster>();
    router_->addMaster(m, 100, 100);
    router_->addMaster(m, 200, 200);

    EXPECT_EQ(router_->masterCount(), 1u);
    auto entries = router_->entries();
    ASSERT_EQ(entries.size(), 1u);
    EXPECT_EQ(entries[0].rx_vlan_range->start, 200u);
    EXPECT_EQ(entries[0].rx_vlan_range->end, 200u);
    EXPECT_EQ(entries[0].tx_vlan_id.value(), 200u);
}

// ============================================================================
// Thread safety
// ============================================================================

TEST_F(VLANRouterTest, ConcurrentAddRemoveAndTx) {
    constexpr int kIterations = 500;

    std::atomic<int> tx_ok{0};
    std::atomic<int> tx_fail{0};

    std::thread tx_thread([this, &tx_ok, &tx_fail]() {
        std::vector<uint8_t> raw = buildFrame(0x88A4, {0x01, 0x02, 0x03});
        for (int i = 0; i < kIterations; ++i) {
            // Pick a random master index (0..2)
            int idx = i % 3;
            EtherCATMaster* mp = nullptr;
            {
                auto entries = router_->entries();
                if (idx < static_cast<int>(entries.size())) {
                    mp = entries[idx].master.get();
                }
            }
            if (mp) {
                NetworkInterface* iface = router_->networkInterfaceFor(mp);
                if (iface && iface->send) {
                    if (iface->send(raw.data(), raw.size())) {
                        tx_ok.fetch_add(1, std::memory_order_relaxed);
                    } else {
                        tx_fail.fetch_add(1, std::memory_order_relaxed);
                    }
                }
            }
        }
    });

    std::thread mutate_thread([this]() {
        for (int i = 0; i < kIterations / 10; ++i) {
            auto m = std::make_shared<EtherCATMaster>();
            router_->addMaster(m, static_cast<uint16_t>(i % 10), static_cast<uint16_t>(i % 10));
            std::this_thread::yield();
            if (i % 3 == 0) {
                router_->removeMaster(m.get());
            }
        }
    });

    tx_thread.join();
    mutate_thread.join();

    // The only guarantee we make is that no crash / data race occurred.
    SUCCEED();
}

TEST_F(VLANRouterTest, ConcurrentAddRemoveAndRx) {
    constexpr int kIterations = 500;

    auto m1 = std::make_shared<EtherCATMaster>();
    auto m2 = std::make_shared<EtherCATMaster>();
    router_->addMaster(m1, 100, std::nullopt);
    router_->addMaster(m2, std::nullopt, std::nullopt);

    std::vector<uint8_t> tagged = buildTaggedFrame(100, 0x88A4, {0xAA});
    std::vector<uint8_t> raw = buildFrame(0x88A4, {0xBB});

    std::thread rx_thread([this, &tagged, &raw]() {
        for (int i = 0; i < kIterations; ++i) {
            if (i % 2 == 0) {
                router_->processRxFrame(tagged.data(), tagged.size());
            } else {
                router_->processRxFrame(raw.data(), raw.size());
            }
        }
    });

    std::thread mutate_thread([this]() {
        for (int i = 0; i < kIterations / 10; ++i) {
            auto m = std::make_shared<EtherCATMaster>();
            router_->addMaster(m, static_cast<uint16_t>(i % 10), std::nullopt);
            std::this_thread::yield();
            router_->removeMaster(m.get());
        }
    });

    rx_thread.join();
    mutate_thread.join();

    SUCCEED();
}

// ============================================================================
// EtherType name helper
// ============================================================================

TEST_F(VLANRouterTest, EtherTypeNameKnownTypes) {
    EXPECT_STREQ(VLANRouter::etherTypeName(0x0800), "IPv4");
    EXPECT_STREQ(VLANRouter::etherTypeName(0x0806), "ARP");
    EXPECT_STREQ(VLANRouter::etherTypeName(0x8100), "VLAN (802.1Q)");
    EXPECT_STREQ(VLANRouter::etherTypeName(0x88A4), "EtherCAT");
    EXPECT_STREQ(VLANRouter::etherTypeName(0x86DD), "IPv6");
    EXPECT_STREQ(VLANRouter::etherTypeName(0x88CC), "LLDP");
}

TEST_F(VLANRouterTest, EtherTypeNameUnknownReturnsNull) {
    EXPECT_EQ(VLANRouter::etherTypeName(0x1234), nullptr);
    EXPECT_EQ(VLANRouter::etherTypeName(0xFFFF), nullptr);
}

// ============================================================================
// Mixed RX/TX integration
// ============================================================================

TEST_F(VLANRouterTest, RoundTripThroughVlanPreservesPayload) {
    auto master = std::make_shared<EtherCATMaster>();
    router_->addMaster(master, 42, 42);

    // TX: master -> router -> backend
    NetworkInterface* iface = router_->networkInterfaceFor(master.get());
    ASSERT_NE(iface, nullptr);

    std::vector<uint8_t> payload(32, 0xDE);
    auto raw = buildFrame(0x88A4, payload);
    ASSERT_TRUE(iface->send(raw.data(), raw.size()));

    // Grab the encapsulated frame from the backend
    ASSERT_EQ(backend_->tx_frames.size(), 1u);
    const auto& encap = backend_->tx_frames[0];

    // Feed the encapsulated frame back into RX
    router_->processRxFrame(encap.data(), encap.size());

    auto caps = captured();
    ASSERT_EQ(caps.size(), 1u);
    EXPECT_EQ(caps[0].master, master.get());

    // Decapsulated frame should equal the original
    ASSERT_EQ(caps[0].data.size(), raw.size());
    EXPECT_EQ(std::memcmp(caps[0].data.data(), raw.data(), raw.size()), 0);
}

// ============================================================================
// VLAN Range tests
// ============================================================================

TEST_F(VLANRouterTest, RangeSingleVidEqualsOldBehavior) {
    auto master = std::make_shared<EtherCATMaster>();
    router_->addMaster(master, VLANRouter::VlanRange{100, 100}, std::nullopt);

    auto tagged = buildTaggedFrame(100, 0x88A4, {0x42});
    router_->processRxFrame(tagged.data(), tagged.size());

    auto caps = captured();
    ASSERT_EQ(caps.size(), 1u);
    EXPECT_EQ(caps[0].master, master.get());
}

TEST_F(VLANRouterTest, RangeMultipleVidsMatch) {
    auto master = std::make_shared<EtherCATMaster>();
    router_->addMaster(master, VLANRouter::VlanRange{100, 200}, std::nullopt);

    auto tagged = buildTaggedFrame(150, 0x88A4, {0x42});
    router_->processRxFrame(tagged.data(), tagged.size());

    auto caps = captured();
    ASSERT_EQ(caps.size(), 1u);
    EXPECT_EQ(caps[0].master, master.get());
}

TEST_F(VLANRouterTest, RangeOverlapTwoMastersBothReceive) {
    auto m1 = std::make_shared<EtherCATMaster>();
    auto m2 = std::make_shared<EtherCATMaster>();
    router_->addMaster(m1, VLANRouter::VlanRange{100, 150}, std::nullopt);
    router_->addMaster(m2, VLANRouter::VlanRange{140, 200}, std::nullopt);

    auto tagged = buildTaggedFrame(145, 0x88A4, {0x55});
    router_->processRxFrame(tagged.data(), tagged.size());

    auto caps = captured();
    ASSERT_EQ(caps.size(), 2u);
}

TEST_F(VLANRouterTest, RangeEdgeCases) {
    auto master = std::make_shared<EtherCATMaster>();
    router_->addMaster(master, VLANRouter::VlanRange{100, 200}, std::nullopt);

    clearCaptured();
    router_->processRxFrame(buildTaggedFrame(100, 0x88A4, {0x01}).data(), 18);
    EXPECT_EQ(captured().size(), 1u);

    clearCaptured();
    router_->processRxFrame(buildTaggedFrame(200, 0x88A4, {0x01}).data(), 18);
    EXPECT_EQ(captured().size(), 1u);

    clearCaptured();
    router_->processRxFrame(buildTaggedFrame(99, 0x88A4, {0x01}).data(), 18);
    EXPECT_TRUE(captured().empty());

    clearCaptured();
    router_->processRxFrame(buildTaggedFrame(201, 0x88A4, {0x01}).data(), 18);
    EXPECT_TRUE(captured().empty());
}

TEST_F(VLANRouterTest, MastersForVlanIdWithRanges) {
    auto m1 = std::make_shared<EtherCATMaster>();
    router_->addMaster(m1, VLANRouter::VlanRange{100, 200}, std::nullopt);

    EXPECT_EQ(router_->mastersForVlanId(150).size(), 1u);
    EXPECT_TRUE(router_->mastersForVlanId(99).empty());
    EXPECT_TRUE(router_->mastersForVlanId(201).empty());
}

// ============================================================================
// API rejection tests
// ============================================================================

TEST_F(VLANRouterTest, AddMasterRejectsUndefinedVlanId) {
    auto m = std::make_shared<EtherCATMaster>();
    router_->addMaster(m, VLANRouter::kUndefinedVlanId, std::nullopt);
    EXPECT_EQ(router_->masterCount(), 0u);
}

TEST_F(VLANRouterTest, AddMasterRejectsInvertedRange) {
    auto m = std::make_shared<EtherCATMaster>();
    router_->addMaster(m, VLANRouter::VlanRange{200, 100}, std::nullopt);
    EXPECT_EQ(router_->masterCount(), 0u);
}
