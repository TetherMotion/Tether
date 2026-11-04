/**
 * @file test_host_setup.cpp
 * @brief Coverage for HostSetup helpers — session lifecycle, master start,
 *        encapsulation delegation and the HostSessionGuard RAII path.
 */

#include <gtest/gtest.h>

#include <atomic>
#include <cstring>
#include <memory>

#include "tether/ethercat/HostSetup.hpp"
#include "tether/ethercat/Master.hpp"
#include "tether/hal/IEthernet.hpp"

using namespace EtherCAT;

namespace {

/// Controllable IEthernet stub — records calls, returns scripted results.
class FakeEthernet : public HAL::IEthernet {
public:
    HAL::Error init_error = HAL::Error::OK;
    bool link_up = true;
    int poll_calls = 0;
    int shutdown_calls = 0;
    int transmit_calls = 0;

    HAL::Error init(const HAL::EthernetConfig&) override {
        initialized_ = (init_error == HAL::Error::OK);
        return init_error;
    }
    void shutdown() override { ++shutdown_calls; initialized_ = false; }
    bool isInitialized() const override { return initialized_; }
    HAL::Error getMacAddress(HAL::MacAddress& mac) const override {
        std::memset(mac.bytes, 0x5A, 6);
        return mac_error;
    }
    HAL::Error setMacAddress(const HAL::MacAddress&) override { return HAL::Error::OK; }
    HAL::Error transmit(const uint8_t*, size_t) override {
        ++transmit_calls;
        return transmit_error;
    }
    HAL::Error transmitVlan(const uint8_t*, size_t, uint16_t, uint8_t) override {
        return HAL::Error::OK;
    }
    HAL::Error transmitGather(const BufferDesc*, size_t) override { return HAL::Error::OK; }
    void setRxCallback(HAL::RxCallback cb, void* ud) override {
        rx_cb = std::move(cb);
        rx_ud = ud;
    }
    int poll(HAL::Milliseconds) override { ++poll_calls; return 0; }
    void setEthertypeFilter(uint16_t) override {}
    HAL::Error setPromiscuous(bool) override { return HAL::Error::OK; }
    HAL::Error addMulticastAddress(const HAL::MacAddress&) override { return HAL::Error::OK; }
    HAL::Error removeMulticastAddress(const HAL::MacAddress&) override { return HAL::Error::OK; }
    HAL::Error setAllMulticast(bool) override { return HAL::Error::OK; }
    HAL::LinkStatus getLinkStatus() const override {
        HAL::LinkStatus s{};
        s.up = link_up;
        s.speedMbps = 100;
        s.fullDuplex = true;
        return s;
    }
    void setLinkCallback(HAL::LinkCallback, void*) override {}
    HAL::Error waitForLinkUp(HAL::Milliseconds) override { return HAL::Error::OK; }
    HAL::EthernetStats getStats() const override { return {}; }
    void resetStats() override {}
    void* nativeHandle() override { return nullptr; }
    const char* getInterfaceName() const override { return "fake0"; }

    HAL::Error mac_error = HAL::Error::OK;
    HAL::Error transmit_error = HAL::Error::OK;
    HAL::RxCallback rx_cb;
    void* rx_ud = nullptr;
    bool initialized_ = false;
};

}  // namespace

// ============================================================================
// initHostEthernet failure paths (no real interface on this host)
// ============================================================================

TEST(HostSetupTest, InitFailsForBogusInterface) {
    HostEtherNetSession session;
    EXPECT_FALSE(initHostEthernet(session, "definitely-not-an-iface-0", "test"));
    EXPECT_FALSE(session.eth);
}

TEST(HostSetupTest, BringUpFailsForBogusInterface) {
    HostEtherNetSession session;
    Master master;
    EncapsulationConfig encap;
    EXPECT_FALSE(bringUpHost(session, master, "definitely-not-an-iface-0",
                             encap, "test"));
}

// ============================================================================
// startHostMaster / startHostMasterAndDiscover
// ============================================================================

TEST(HostSetupTest, StartHostMasterFailsWithoutEncapsulation) {
    HostEtherNetSession session;
    Master master;
    EXPECT_FALSE(startHostMaster(session, master, "test"));
}

TEST(HostSetupTest, StartAndDiscoverFailsWithoutEncapsulation) {
    HostEtherNetSession session;
    Master master;
    EXPECT_FALSE(startHostMasterAndDiscover(session, master, "test"));
}

// ============================================================================
// shutdownHostEthernet
// ============================================================================

TEST(HostSetupTest, ShutdownOnEmptySessionIsSafe) {
    HostEtherNetSession session;
    shutdownHostEthernet(session);  // no eth, no thread — must not crash
    EXPECT_FALSE(session.pollRunning.load());
}

TEST(HostSetupTest, ShutdownShutsDownEthernet) {
    HostEtherNetSession session;
    auto eth = std::make_unique<FakeEthernet>();
    auto* raw = eth.get();
    session.eth = std::move(eth);
    shutdownHostEthernet(session);
    EXPECT_EQ(raw->shutdown_calls, 1);
}

TEST(HostSetupTest, ShutdownJoinsPollThread) {
    HostEtherNetSession session;
    auto eth = std::make_unique<FakeEthernet>();
    auto* raw = eth.get();
    session.eth = std::move(eth);
    session.pollRunning.store(true);
    session.pollThread = std::thread([&session] {
        while (session.pollRunning.load()) { /* spin */ }
    });
    shutdownHostEthernet(session);
    EXPECT_FALSE(session.pollRunning.load());
    EXPECT_FALSE(session.pollThread.joinable());
    EXPECT_EQ(raw->shutdown_calls, 1);
}

// ============================================================================
// startHostPollThread
// ============================================================================

TEST(HostSetupTest, PollThreadCallsEthPollUntilStopped) {
    HostEtherNetSession session;
    auto eth = std::make_unique<FakeEthernet>();
    auto* raw = eth.get();
    session.eth = std::move(eth);

    startHostPollThread(session, "test");
    // Let the thread tick a few times.
    for (int i = 0; i < 100 && raw->poll_calls < 3; ++i) {
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    shutdownHostEthernet(session);
    EXPECT_GE(raw->poll_calls, 1);
}

// ============================================================================
// Session destructor
// ============================================================================

TEST(HostSetupTest, SessionDestructorStopsPollAndShutsDown) {
    FakeEthernet* raw = nullptr;
    {
        HostEtherNetSession session;
        auto eth = std::make_unique<FakeEthernet>();
        raw = eth.get();
        session.eth = std::move(eth);
        session.pollRunning.store(true);
        session.pollThread = std::thread([&session] {
            while (session.pollRunning.load()) { /* spin */ }
        });
    }  // destructor must stop the poll loop and call eth->shutdown()
    EXPECT_EQ(raw->shutdown_calls, 1);
}

// ============================================================================
// setupEncapsulation (non-VLAN): backend passthrough interface
// ============================================================================

TEST(HostSetupTest, SetupEncapsulationReturnsBackend) {
    HostEtherNetSession session;
    auto eth = std::make_unique<FakeEthernet>();
    session.eth = std::move(eth);
    session.ni = std::make_unique<NetworkInterface>();
    session.ni->send = [](const uint8_t*, size_t) { return true; };

    Master master;
    EncapsulationConfig encap;   // no VLAN, no UDP → backend passthrough
    EXPECT_TRUE(setupEncapsulation(session, master, encap, "test"));
    EXPECT_EQ(session.masterInterface, session.ni.get());

    master.stop();
}

TEST(HostSetupTest, StartHostMasterUsesBackendInterface) {
    HostEtherNetSession session;
    auto eth = std::make_unique<FakeEthernet>();
    session.eth = std::move(eth);
    session.ni = std::make_unique<NetworkInterface>();
    session.ni->send = [](const uint8_t*, size_t) { return true; };

    Master master;
    EncapsulationConfig encap;
    ASSERT_TRUE(setupEncapsulation(session, master, encap, "test"));
    EXPECT_TRUE(startHostMaster(session, master, "test"));

    master.stop();
}

TEST(HostSetupTest, SetupEncapsulationVlanBuildsRouter) {
    HostEtherNetSession session;
    auto eth = std::make_unique<FakeEthernet>();
    session.eth = std::move(eth);
    session.ni = std::make_unique<NetworkInterface>();
    session.ni->send = [](const uint8_t*, size_t) { return true; };

    Master master;
    EncapsulationConfig encap;
    encap.rxAny = true;
    // VLAN probe gets no reply on the fake → setup fails (documented
    // behaviour: refuse to start when the probe cannot verify delivery).
    EXPECT_FALSE(setupEncapsulation(session, master, encap, "test"));

    master.stop();
}

// ============================================================================
// HostSessionGuard
// ============================================================================

TEST(HostSetupTest, GuardRunsOnExitThenShutsDown) {
    FakeEthernet* raw = nullptr;
    int exit_calls = 0;
    {
        HostEtherNetSession session;
        auto eth = std::make_unique<FakeEthernet>();
        raw = eth.get();
        session.eth = std::move(eth);
        HostSessionGuard guard(session, [&] { ++exit_calls; });
    }
    EXPECT_EQ(exit_calls, 1);
    EXPECT_EQ(raw->shutdown_calls, 1);
}

TEST(HostSetupTest, GuardDismissSkipsOnExitButStillShutsDown) {
    FakeEthernet* raw = nullptr;
    int exit_calls = 0;
    {
        HostEtherNetSession session;
        auto eth = std::make_unique<FakeEthernet>();
        raw = eth.get();
        session.eth = std::move(eth);
        HostSessionGuard guard(session, [&] { ++exit_calls; });
        guard.dismiss();
    }
    EXPECT_EQ(exit_calls, 0);
    EXPECT_EQ(raw->shutdown_calls, 1);
}

TEST(HostSetupTest, GuardSetOnExitReplacesAction) {
    int first = 0, second = 0;
    {
        HostEtherNetSession session;
        HostSessionGuard guard(session, [&] { ++first; });
        guard.setOnExit([&] { ++second; });
    }
    EXPECT_EQ(first, 0);
    EXPECT_EQ(second, 1);
}
