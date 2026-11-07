/**
 * @file test_encapsulation_setup.cpp
 * @brief setupEncapsulation() paths reachable without a live wire probe:
 *        untagged/no-reply warn-through, VLAN-active early failure, the
 *        UDP-encapsulation build-gate error, and the rx-callback +
 *        return-backend tail.
 */

#include <gtest/gtest.h>

#include <optional>

#include <arpa/inet.h>
#include <linux/if_ether.h>
#include <sys/socket.h>
#include <unistd.h>

#include "tether/ethercat/Encapsulation.hpp"
#include "tether/ethercat/Master.hpp"
#include "tether/ethercat/VLANRouter.hpp"
#include "tether/hal/IEthernet.hpp"

using namespace EtherCAT;

namespace {

class FakeEth : public HAL::IEthernet {
public:
    int fd = -1;
    const char* name = "lo0-nonexistent";

    HAL::Error init(const HAL::EthernetConfig&) override { return HAL::Error::OK; }
    void shutdown() override {}
    bool isInitialized() const override { return true; }
    HAL::Error getMacAddress(HAL::MacAddress&) const override { return HAL::Error::OK; }
    HAL::Error setMacAddress(const HAL::MacAddress&) override { return HAL::Error::OK; }
    HAL::Error transmit(const uint8_t*, size_t) override { return HAL::Error::OK; }
    HAL::Error transmitVlan(const uint8_t*, size_t, uint16_t, uint8_t) override {
        return HAL::Error::OK;
    }
    HAL::Error transmitGather(const BufferDesc*, size_t) override {
        return HAL::Error::OK;
    }
    void setRxCallback(HAL::RxCallback cb, void* ud) override {
        rx_cb = std::move(cb);
        rx_ud = ud;
    }
    int poll(HAL::Milliseconds) override { return 0; }
    void setEthertypeFilter(uint16_t) override {}
    HAL::Error setPromiscuous(bool) override { return HAL::Error::OK; }
    HAL::Error addMulticastAddress(const HAL::MacAddress&) override {
        return HAL::Error::OK;
    }
    HAL::Error removeMulticastAddress(const HAL::MacAddress&) override {
        return HAL::Error::OK;
    }
    HAL::Error setAllMulticast(bool) override { return HAL::Error::OK; }
    HAL::LinkStatus getLinkStatus() const override { return {}; }
    void setLinkCallback(HAL::LinkCallback, void*) override {}
    HAL::Error waitForLinkUp(HAL::Milliseconds) override { return HAL::Error::OK; }
    HAL::EthernetStats getStats() const override { return {}; }
    void resetStats() override {}
    void* nativeHandle() override {
        return reinterpret_cast<void*>(static_cast<intptr_t>(fd));
    }
    const char* getInterfaceName() const override { return name; }

    HAL::RxCallback rx_cb;
    void* rx_ud = nullptr;
};

struct EncapFixture {
    FakeEth eth;
    NetworkInterface backend;
    Master master;
    std::unique_ptr<VLANRouter> router;
};

} // namespace

// Untagged + no probe reply: warn-and-continue → attach-BPF early-return
// (fd<0) → no VLAN → installs direct rx callback → returns &backend.
TEST(EncapsulationSetupTest, UntaggedNoReplyReturnsBackend) {
    EncapFixture f;
    EncapsulationConfig cfg{};
    NetworkInterface* out = setupEncapsulation(f.eth, f.backend, f.master,
                                             f.router, cfg, "test");
    EXPECT_EQ(out, &f.backend);
    EXPECT_NE(f.eth.rx_cb, nullptr);
}

// The installed direct-receive callback forwards frames into the master.
TEST(EncapsulationSetupTest, DirectRxCallbackFeedsMaster) {
    EncapFixture f;
    EncapsulationConfig cfg{};
    ASSERT_EQ(setupEncapsulation(f.eth, f.backend, f.master, f.router,
                                 cfg, "test"),
              &f.backend);
    ASSERT_NE(f.eth.rx_cb, nullptr);
    const uint8_t frame[64] = {0};
    HAL::RxFrameInfo info{};
    EXPECT_NO_THROW(f.eth.rx_cb(frame, sizeof(frame), info, f.eth.rx_ud));
}

// VLAN requested but the wire probe can't reply → hard failure.
TEST(EncapsulationSetupTest, VlanNoReplyFails) {
    EncapFixture f;
    EncapsulationConfig cfg{};
    cfg.txVlan = 100;
    ASSERT_TRUE(cfg.vlanActive());
    EXPECT_EQ(setupEncapsulation(f.eth, f.backend, f.master, f.router,
                               cfg, "test"),
              nullptr);
}

TEST(EncapsulationSetupTest, VlanRxRangeNoReplyFails) {
    EncapFixture f;
    EncapsulationConfig cfg{};
    cfg.rxRange = VLANRouter::VLANRange{100, 200};
    ASSERT_TRUE(cfg.vlanActive());
    EXPECT_EQ(setupEncapsulation(f.eth, f.backend, f.master, f.router,
                               cfg, "test"),
              nullptr);
}

TEST(EncapsulationSetupTest, VlanRxAnyNoReplyFails) {
    EncapFixture f;
    EncapsulationConfig cfg{};
    cfg.rxAny = true;
    ASSERT_TRUE(cfg.vlanActive());
    EXPECT_EQ(setupEncapsulation(f.eth, f.backend, f.master, f.router,
                               cfg, "test"),
              nullptr);
}

namespace {

// AF_PACKET open is the cheapest capability probe for the loopback tests.
bool haveRawPacketSocket() {
    int fd = ::socket(AF_PACKET, SOCK_RAW, htons(ETH_P_ALL));
    if (fd < 0) return false;
    ::close(fd);
    return true;
}

// Real IEthernet on loopback for answered-probe tests (CAP_NET_RAW).
struct LoFixture {
    std::unique_ptr<HAL::IEthernet> eth;
    Master master;
    NetworkInterface backend{};
    std::unique_ptr<VLANRouter> router;

    bool init() {
        eth = HAL::createDefaultEthernet();
        if (!eth) return false;
        HAL::EthernetConfig cfg{};
        cfg.interfaceName = "lo";
        cfg.promiscuous = false;
        return eth->init(cfg) == HAL::Error::OK;
    }
    ~LoFixture() { if (eth) eth->shutdown(); }
};

} // namespace

// Answered wire probe on loopback: outgoing tagged frames echo back →
// the probe succeeds → VLAN router allocation + direct-encap wiring.
TEST(EncapsulationSetupTest, VlanAnsweredProbeAllocatesRouter) {
    if (!haveRawPacketSocket())
        GTEST_SKIP() << "no CAP_NET_RAW for AF_PACKET on lo";
    LoFixture f;
    ASSERT_TRUE(f.init());

    EncapsulationConfig cfg{};
    cfg.txVlan = 100;
    NetworkInterface* out = setupEncapsulation(*f.eth, f.backend, f.master,
                                             f.router, cfg, "test");
    EXPECT_NE(out, nullptr);
    EXPECT_NE(f.router, nullptr);
}

// Catch-all RX mode: undefined-target branch + undefinedNetworkInterface.
TEST(EncapsulationSetupTest, VlanRxAnyAnsweredProbe) {
    if (!haveRawPacketSocket())
        GTEST_SKIP() << "no CAP_NET_RAW for AF_PACKET on lo";
    LoFixture f;
    ASSERT_TRUE(f.init());

    EncapsulationConfig cfg{};
    cfg.rxAny = true;
    cfg.txVlan = 100;
    EXPECT_NE(setupEncapsulation(*f.eth, f.backend, f.master, f.router,
                                 cfg, "test"),
              nullptr);
    EXPECT_NE(f.router, nullptr);
}

// RX-range mode with the probe VID outside the configured range → the
// "replies carry VID outside the range" warning branch.
TEST(EncapsulationSetupTest, VlanRxRangeProbeVidOutsideRange) {
    if (!haveRawPacketSocket())
        GTEST_SKIP() << "no CAP_NET_RAW for AF_PACKET on lo";
    LoFixture f;
    ASSERT_TRUE(f.init());

    EncapsulationConfig cfg{};
    cfg.txVlan = 100;                            // probe VID = 100
    cfg.rxRange = VLANRouter::VLANRange{200, 300}; // reply VID ∉ range
    EXPECT_NE(setupEncapsulation(*f.eth, f.backend, f.master, f.router,
                                 cfg, "test"),
              nullptr);
}

// UDP encapsulation requested: with TETHER_ENABLE_UDP_ENCAPSULATION=0 the
// setup must fail cleanly; when compiled in it proceeds to the backend.
TEST(EncapsulationSetupTest, UdpGate) {
    EncapFixture f;
    EncapsulationConfig cfg{};
    cfg.udp = true;
    NetworkInterface* out = setupEncapsulation(f.eth, f.backend, f.master,
                                             f.router, cfg, "test");
#if TETHER_ENABLE_UDP_ENCAPSULATION
    EXPECT_EQ(out, &f.backend);
#else
    EXPECT_EQ(out, nullptr);
#endif
}

// UDP + VLAN: the probe fails first → nullptr before the UDP block.
TEST(EncapsulationSetupTest, UdpPlusVlanNoReplyFailsEarly) {
    EncapFixture f;
    EncapsulationConfig cfg{};
    cfg.udp = true;
    cfg.txVlan = 42;
    EXPECT_EQ(setupEncapsulation(f.eth, f.backend, f.master, f.router,
                               cfg, "test"),
              nullptr);
}
