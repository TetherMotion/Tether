/**
 * @file test_integration_paired.cpp
 * @brief Integration tests using LinuxPairedNetworkInterface
 *
 * Tests master ↔ slave communication end-to-end using paired
 * loopback interfaces.  A lightweight inline slave responder
 * handles BRD / APRD / APWR / BWR frames.
 */

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <thread>
#include <vector>

#include "tether/ethercat/EtherCATMaster.hpp"
#include "tether/ethercat/EtherCATTypes.hpp"
#include "tether/ethercat/LinuxDevNullNetworkInterface.hpp"
#include "tether/ethercat/LinuxPairedNetworkInterface.hpp"
#include "tether/ethercat/TransactionRouter.hpp"
#include "tether/platform/EspCompat.hpp"

// We need the internal wire format to build correct responses
#include "ethercat/raw/internal.hpp"

using namespace EtherCAT;
using namespace EtherCAT::Raw;

// ============================================================================
// Lightweight slave responder
// ============================================================================

/**
 * @brief Minimal EtherCAT bus responder that sits on side-B of a
 *        LinuxPairedNetworkInterface and echoes frames back to the
 *        master after incrementing the Working Counter.
 *
 * This responder simulates `num_slaves` devices.  For BRD commands
 * the WKC is incremented by num_slaves.  For APRD / APWR the WKC is
 * set to 1 if the addressed slave exists (adp <= num_slaves).
 *
 * Register reads return sensible defaults:
 *  - AL_STATUS → 0x0001 (INIT) initially; updated by AL_CONTROL writes.
 *  - EEPROM → simple SII stub (vendor=0xDEAD, product=0xBEEF).
 */
class SimpleSlaveResponder {
public:
    explicit SimpleSlaveResponder(LinuxPairedNetworkInterface& pair,
                                  uint16_t num_slaves = 1)
        : pair_(pair), num_slaves_(num_slaves)
    {
        slave_states_.resize(num_slaves_, 0x01);  // all start in INIT
        // Whenever side-A (master) sends a frame, we receive it here
        pair_.setRxCallbackB([this](const uint8_t* data, size_t len) {
            handleFrame(data, len);
        });
    }

    ~SimpleSlaveResponder() {
        pair_.setRxCallbackB(nullptr);
    }

    /// Change the number of emulated slaves (does NOT reset states).
    void setNumSlaves(uint16_t n) {
        num_slaves_ = n;
        slave_states_.resize(n, 0x01);
    }

    /// Get the current AL state of a slave.
    uint8_t slaveState(uint16_t idx) const {
        return idx < slave_states_.size() ? slave_states_[idx] : 0;
    }

private:
    void handleFrame(const uint8_t* data, size_t len) {
        // We need at least Ethernet(14) + EC frame header(2) + datagram header(10) + WKC(2)
        constexpr size_t kMinLen = 14 + 2 + 10 + 2;
        if (len < kMinLen) return;

        // Copy frame so we can modify it
        std::vector<uint8_t> reply(data, data + len);

        // Swap Ethernet src/dst
        std::swap_ranges(reply.begin(), reply.begin() + 6,
                         reply.begin() + 6);

        // Parse datagram
        auto* dg = reinterpret_cast<EtherCATDatagramHeader*>(
            reply.data() + 14 + 2);  // after eth + ec header

        uint16_t datalen_raw = le16_to_host(dg->lenFlags.raw_le) & 0x07FFu;
        size_t wkc_offset = 14 + 2 + 10 + datalen_raw;
        if (wkc_offset + 2 > len) return;

        uint16_t* wkc_ptr = reinterpret_cast<uint16_t*>(reply.data() + wkc_offset);
        uint8_t* payload = reply.data() + 14 + 2 + 10;

        auto cmd = static_cast<Command>(static_cast<uint8_t>(dg->cmd));
        uint16_t adp = le16_to_host(dg->adp_le);
        uint16_t ado = le16_to_host(dg->ado_le);

        uint16_t wkc = 0;

        switch (cmd) {
        case Command::BRD:
            // Broadcast read — every slave increments WKC
            wkc = num_slaves_;
            fillRegisterData(payload, datalen_raw, ado, 0);
            break;

        case Command::BWR:
            // Broadcast write
            wkc = num_slaves_;
            for (uint16_t i = 0; i < num_slaves_; ++i)
                applyWrite(i, ado, payload, datalen_raw);
            break;

        case Command::APRD: {
            // Auto-increment physical read
            // adp is the position *after* traversal: for slave n, adp == (0-n)
            uint16_t slave_idx = static_cast<uint16_t>(0u - adp);
            if (slave_idx < num_slaves_) {
                wkc = 1;
                fillRegisterData(payload, datalen_raw, ado, slave_idx);
                // Decrement adp (slave has been "passed through")
                dg->adp_le = host_to_le16(static_cast<uint16_t>(adp + 1));
            }
            break;
        }

        case Command::APWR: {
            uint16_t slave_idx = static_cast<uint16_t>(0u - adp);
            if (slave_idx < num_slaves_) {
                wkc = 1;
                applyWrite(slave_idx, ado, payload, datalen_raw);
                dg->adp_le = host_to_le16(static_cast<uint16_t>(adp + 1));
            }
            break;
        }

        default:
            // Unknown command — echo with WKC=0
            break;
        }

        *wkc_ptr = host_to_le16(wkc);

        // Send reply back to side-A (master)
        pair_.ifaceB().send(reply.data(), reply.size());
    }

    /// Fill register data for a read command
    void fillRegisterData(uint8_t* buf, uint16_t len, uint16_t ado, uint16_t slave_idx) {
        if (!buf || len == 0) return;
        std::memset(buf, 0, len);

        switch (ado) {
        case EC_REG_AL_STATUS:
            if (len >= 2) {
                uint16_t state = slave_idx < slave_states_.size()
                    ? slave_states_[slave_idx] : 0x01;
                std::memcpy(buf, &state, 2);
            }
            break;
        case EC_REG_AL_STATUS_CODE:
            // No error
            break;
        case EC_REG_EEPCTL:  // also EC_REG_EEPSTAT (same address)
            // EEPROM idle
            break;
        default:
            break;
        }
    }

    /// Apply a write command to a slave
    void applyWrite(uint16_t slave_idx, uint16_t ado,
                    const uint8_t* data, uint16_t len) {
        if (slave_idx >= slave_states_.size()) return;

        switch (ado) {
        case EC_REG_AL_CONTROL:
            if (len >= 2) {
                uint16_t val;
                std::memcpy(&val, data, 2);
                val = le16_to_host(val);
                // Accept any state request (simplified)
                slave_states_[slave_idx] = static_cast<uint8_t>(val & 0x0F);
            }
            break;
        default:
            break;
        }
    }

    LinuxPairedNetworkInterface& pair_;
    uint16_t num_slaves_;
    std::vector<uint8_t> slave_states_;
};

// ============================================================================
// Test fixture
// ============================================================================

class IntegrationPairedTest : public ::testing::Test {
protected:
    void SetUp() override {
        pair_ = std::make_unique<LinuxPairedNetworkInterface>();
    }

    void TearDown() override {
        if (master_) master_->stop();
        pair_.reset();
    }

    /// Create & start a master on side-A with an inline RX callback.
    EtherCATMaster& startMaster() {
        master_ = std::make_unique<EtherCATMaster>();
        pair_->setRxCallbackA([this](const uint8_t* data, size_t len) {
            if (master_) master_->handleRxFrame(data, len);
        });
        master_->start(pair_->ifaceA(), dummy_mac_);
        return *master_;
    }

    std::unique_ptr<LinuxPairedNetworkInterface> pair_;
    std::unique_ptr<EtherCATMaster> master_;
    uint8_t dummy_mac_[6] = {0x02, 0x00, 0x00, 0x00, 0x00, 0x01};
};

// ============================================================================
// 1. Slave count detection
// ============================================================================

TEST_F(IntegrationPairedTest, DiscoverSingleSlave) {
    SimpleSlaveResponder responder(*pair_, 1);
    auto& master = startMaster();

    // Wait for discovery
    for (int i = 0; i < 50 && master.getDiscoveredSlaveCount() == 0; ++i)
        std::this_thread::sleep_for(std::chrono::milliseconds(50));

    EXPECT_EQ(master.getDiscoveredSlaveCount(), 1u);
}

TEST_F(IntegrationPairedTest, DiscoverMultipleSlaves) {
    SimpleSlaveResponder responder(*pair_, 3);
    auto& master = startMaster();

    for (int i = 0; i < 50 && master.getDiscoveredSlaveCount() == 0; ++i)
        std::this_thread::sleep_for(std::chrono::milliseconds(50));

    EXPECT_EQ(master.getDiscoveredSlaveCount(), 3u);
}

TEST_F(IntegrationPairedTest, DiscoverZeroSlaves) {
    // No responder — frames are sent but nobody replies
    auto& master = startMaster();

    // The master will try repeatedly; give it a shorter window.
    // Because discoverSlaves() retries up to 200 times with 100ms+300ms
    // per attempt, we stop the master early to avoid a long wait.
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    EXPECT_EQ(master.getDiscoveredSlaveCount(), 0u);
}

// ============================================================================
// 2. Mailbox configuration (auto-configure from SII)
// ============================================================================

TEST_F(IntegrationPairedTest, AutoConfigureMailboxDoesNotCrash) {
    SimpleSlaveResponder responder(*pair_, 1);
    auto& master = startMaster();

    for (int i = 0; i < 50 && master.getDiscoveredSlaveCount() == 0; ++i)
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    ASSERT_GE(master.getDiscoveredSlaveCount(), 1u);

    // autoConfigureMailbox should not crash even with
    // our minimal responder that doesn't provide real SII data
    // (it will fall back to defaults).
    bool ok = master.autoConfigureMailbox(0);
    // We don't require success (SII stub is minimal), just no crash
    (void)ok;
}

// ============================================================================
// 3. State transitions: INIT → PRE_OP → SAFE_OP → OP
// ============================================================================

TEST_F(IntegrationPairedTest, TransitionToPreOp) {
    SimpleSlaveResponder responder(*pair_, 1);
    auto& master = startMaster();

    for (int i = 0; i < 50 && master.getDiscoveredSlaveCount() == 0; ++i)
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    ASSERT_GE(master.getDiscoveredSlaveCount(), 1u);

    // The master's discoverSlaves already pushes to PRE_OP internally.
    // Verify we can explicitly request PRE_OP:
    bool ok = master.requestSlaveApplicationLayerState(0, 0x02);
    EXPECT_TRUE(ok);

    // Confirm the responder applied the state
    EXPECT_EQ(responder.slaveState(0), 0x02);
}

TEST_F(IntegrationPairedTest, TransitionToSafeOpAndOp) {
    SimpleSlaveResponder responder(*pair_, 1);
    auto& master = startMaster();

    for (int i = 0; i < 50 && master.getDiscoveredSlaveCount() == 0; ++i)
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    ASSERT_GE(master.getDiscoveredSlaveCount(), 1u);

    // PRE_OP
    ASSERT_TRUE(master.requestSlaveApplicationLayerState(0, 0x02));
    EXPECT_EQ(responder.slaveState(0), 0x02);

    // SAFE_OP
    ASSERT_TRUE(master.requestSlaveApplicationLayerState(0, 0x04));
    EXPECT_EQ(responder.slaveState(0), 0x04);

    // OP
    ASSERT_TRUE(master.requestSlaveApplicationLayerState(0, 0x08));
    EXPECT_EQ(responder.slaveState(0), 0x08);
}

TEST_F(IntegrationPairedTest, ReadSlaveState) {
    SimpleSlaveResponder responder(*pair_, 1);
    auto& master = startMaster();

    for (int i = 0; i < 50 && master.getDiscoveredSlaveCount() == 0; ++i)
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    ASSERT_GE(master.getDiscoveredSlaveCount(), 1u);

    // Move to PRE_OP
    ASSERT_TRUE(master.requestSlaveApplicationLayerState(0, 0x02));

    uint8_t state = 0;
    ASSERT_TRUE(master.readSlaveApplicationLayerState(0, state));
    EXPECT_EQ(state, 0x02);
}

// ============================================================================
// 4. DC clock configuration (basic smoke test)
// ============================================================================

TEST_F(IntegrationPairedTest, DCManagerInit) {
    SimpleSlaveResponder responder(*pair_, 1);
    auto& master = startMaster();

    for (int i = 0; i < 50 && master.getDiscoveredSlaveCount() == 0; ++i)
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    ASSERT_GE(master.getDiscoveredSlaveCount(), 1u);

    // Access DC manager — just verify it doesn't crash.
    auto& dc = master.dc();
    (void)dc;
}

// ============================================================================
// DevNull interface tests
// ============================================================================

TEST(LinuxDevNullNetworkInterfaceTest, SendSucceeds) {
    LinuxDevNullNetworkInterface devnull;
    uint8_t data[64] = {};
    EXPECT_TRUE(devnull.iface().send(data, sizeof(data)));
    EXPECT_EQ(devnull.txCount(), 1u);
    EXPECT_EQ(devnull.txBytes(), 64u);
}

TEST(LinuxDevNullNetworkInterfaceTest, ReceiveReturnsFalse) {
    LinuxDevNullNetworkInterface devnull;
    uint8_t buf[64];
    size_t out_len = 999;
    EXPECT_FALSE(devnull.iface().receive(buf, sizeof(buf), &out_len));
    EXPECT_EQ(out_len, 0u);
}

TEST(LinuxDevNullNetworkInterfaceTest, ResetCounters) {
    LinuxDevNullNetworkInterface devnull;
    uint8_t data[10] = {};
    devnull.iface().send(data, sizeof(data));
    ASSERT_EQ(devnull.txCount(), 1u);
    devnull.resetCounters();
    EXPECT_EQ(devnull.txCount(), 0u);
    EXPECT_EQ(devnull.txBytes(), 0u);
}

// ============================================================================
// Paired interface tests
// ============================================================================

TEST(LinuxPairedNetworkInterfaceTest, SendFromAToB) {
    LinuxPairedNetworkInterface pair;
    std::vector<uint8_t> received;
    pair.setRxCallbackB([&](const uint8_t* data, size_t len) {
        received.assign(data, data + len);
    });

    uint8_t msg[] = {0xDE, 0xAD, 0xBE, 0xEF};
    EXPECT_TRUE(pair.ifaceA().send(msg, sizeof(msg)));
    ASSERT_EQ(received.size(), sizeof(msg));
    EXPECT_EQ(received[0], 0xDE);
    EXPECT_EQ(received[3], 0xEF);
}

TEST(LinuxPairedNetworkInterfaceTest, SendFromBToA) {
    LinuxPairedNetworkInterface pair;
    std::vector<uint8_t> received;
    pair.setRxCallbackA([&](const uint8_t* data, size_t len) {
        received.assign(data, data + len);
    });

    uint8_t msg[] = {0xCA, 0xFE};
    EXPECT_TRUE(pair.ifaceB().send(msg, sizeof(msg)));
    ASSERT_EQ(received.size(), sizeof(msg));
    EXPECT_EQ(received[0], 0xCA);
}

TEST(LinuxPairedNetworkInterfaceTest, NoCallbackCountsDropped) {
    LinuxPairedNetworkInterface pair;
    // No callback on B
    uint8_t msg[] = {0x01};
    EXPECT_TRUE(pair.ifaceA().send(msg, sizeof(msg)));
    EXPECT_EQ(pair.statsA().dropped.load(), 1u);
}

TEST(LinuxPairedNetworkInterfaceTest, ResetStats) {
    LinuxPairedNetworkInterface pair;
    pair.setRxCallbackB([](const uint8_t*, size_t) {});
    uint8_t msg[1] = {};
    pair.ifaceA().send(msg, sizeof(msg));
    ASSERT_EQ(pair.statsA().tx_count.load(), 1u);
    pair.resetStats();
    EXPECT_EQ(pair.statsA().tx_count.load(), 0u);
}

// ============================================================================
// TransactionRouter unit tests (basic)
// ============================================================================

TEST(TransactionRouterTest, InitAndShutdown) {
    TransactionRouter router;
    EXPECT_TRUE(router.init());
    EXPECT_FALSE(router.hasWaiters());
    EXPECT_EQ(router.waiterCount(), 0u);
    router.shutdown();
}

TEST(TransactionRouterTest, RouteWithNoWaiters) {
    TransactionRouter router;
    router.init();
    RxDatagram dgram{};
    dgram.idx = 42;
    dgram.wkc = 1;
    EXPECT_EQ(router.routePacket(dgram), 0u);
    EXPECT_EQ(router.getStats().packets_dropped, 1u);
    router.shutdown();
}

TEST(TransactionRouterTest, SendAndWaitTimeout) {
    TransactionRouter router;
    router.init();

    uint8_t buf[64] = {};
    auto result = router.sendAndWait(0, buf, sizeof(buf),
        []{ return true; },
        10);  // 10ms timeout, no one will respond
    EXPECT_FALSE(result.success);
    EXPECT_TRUE(result.timeout);
    router.shutdown();
}

TEST(TransactionRouterTest, SendAndWaitSuccess) {
    TransactionRouter router;
    router.init();

    uint8_t buf[64] = {};

    // Spawn a thread that routes a response after a tiny delay
    std::thread responder([&]() {
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
        RxDatagram dgram{};
        dgram.idx = 7;
        dgram.wkc = 1;
        dgram.cmd = Command::BRD;
        dgram.datalen = 2;
        dgram.data[0] = 0xAB;
        dgram.data[1] = 0xCD;
        router.routePacket(dgram);
    });

    auto result = router.sendAndWait(7, buf, sizeof(buf),
        []{ return true; },
        1000);

    responder.join();

    EXPECT_TRUE(result.success);
    EXPECT_EQ(result.wkc, 1u);
    EXPECT_EQ(result.idx, 7u);
    EXPECT_EQ(buf[0], 0xAB);
    EXPECT_EQ(buf[1], 0xCD);
    router.shutdown();
}

TEST(TransactionRouterTest, PreRegisterAndWait) {
    TransactionRouter router;
    router.init();

    uint8_t buf[64] = {};
    auto filter = PacketFilter::byIndex(99);
    size_t slot = router.preRegisterWaiter(filter, buf, sizeof(buf));
    EXPECT_LT(slot, TransactionRouter::kNumSlots);
    EXPECT_TRUE(router.hasWaiters());

    // Route a matching packet
    std::thread responder([&]() {
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
        RxDatagram dgram{};
        dgram.idx = 99;
        dgram.wkc = 2;
        dgram.datalen = 1;
        dgram.data[0] = 0x42;
        router.routePacket(dgram);
    });

    auto result = router.waitForPreRegistered(slot, 1000);
    responder.join();

    EXPECT_TRUE(result.success);
    EXPECT_EQ(result.wkc, 2u);
    EXPECT_EQ(buf[0], 0x42);
    router.shutdown();
}

TEST(TransactionRouterTest, CancelPreRegistered) {
    TransactionRouter router;
    router.init();

    uint8_t buf[64] = {};
    auto filter = PacketFilter::byIndex(10);
    size_t slot = router.preRegisterWaiter(filter, buf, sizeof(buf));
    EXPECT_TRUE(router.hasWaiters());

    router.cancelPreRegistered(slot);
    EXPECT_FALSE(router.hasWaiters());
    router.shutdown();
}
