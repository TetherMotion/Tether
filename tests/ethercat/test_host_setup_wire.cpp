/**
 * @file test_host_setup_wire.cpp
 * @brief startHostMasterAndDiscover over the emulated wire — the
 *        slave-enumeration log path and the empty-bus early return.
 */

#include <gtest/gtest.h>

#include <cstring>

#include <arpa/inet.h>
#include <linux/if_ether.h>
#include <sys/socket.h>
#include <unistd.h>

#include "mocks/EmulatedBus.hpp"
#include "tether/ethercat/Encapsulation.hpp"
#include "tether/ethercat/HostSetup.hpp"
#include "tether/ethercat/Master.hpp"
#include "tether/ethercat/SlaveEmulator.hpp"

using namespace EtherCAT;

// Discovery finds slaves → per-slave log lines → true.
TEST(HostSetupWireTest, StartAndDiscoverOverEmulatedBus) {
    EmulatedBus bus;
    bus.net.addSlave(Emulator::createCiA402Drive(0x00000002, 0xBEEF, "hs0"));
    bus.net.addSlave(Emulator::createCiA402Drive(0x00000002, 0xBEEF, "hs1"));

    Master master;
    bus.attach(master);

    HostEtherNetSession session;
    session.masterInterface = &bus.iface;
    std::memcpy(session.srcMac, kEmulatedMac, 6);

    EXPECT_TRUE(startHostMaster(session, master, "test"));
    EXPECT_TRUE(startHostMasterAndDiscover(session, master, "test"));
    EXPECT_EQ(master.getDiscoveredSlaveCount(), 2u);

    bus.shutdown();
    master.packetRouter().shutdown();
}

// Discovery on a slave-less bus → "No slaves found" → stop + false.
TEST(HostSetupWireTest, DiscoverEmptyBusFails) {
    EmulatedBus bus;

    Master master;
    bus.attach(master);

    HostEtherNetSession session;
    session.masterInterface = &bus.iface;
    std::memcpy(session.srcMac, kEmulatedMac, 6);

    EXPECT_FALSE(startHostMasterAndDiscover(session, master, "test"));

    bus.shutdown();
    master.packetRouter().shutdown();
}

// bringUpHost: initHostEthernet fails for a nonexistent interface →
// early false (needs no privileges).
TEST(HostSetupWireTest, BringUpHostBadInterface) {
    HostEtherNetSession session;
    Master master;
    EXPECT_FALSE(bringUpHost(session, master, "tether-nosuch-0",
                             EncapsulationConfig{}, "test"));
}

namespace {

// AF_PACKET open is the cheapest capability probe for the loopback tests.
bool haveRawPacketSocket() {
    int fd = ::socket(AF_PACKET, SOCK_RAW, htons(ETH_P_ALL));
    if (fd < 0) return false;
    ::close(fd);
    return true;
}

} // namespace

// Full bringUpHost success path on the loopback device: init + untagged
// encapsulation + poll thread + master start, then RAII teardown via
// HostSessionGuard. Needs CAP_NET_RAW; skipped otherwise.
TEST(HostSetupWireTest, BringUpHostLoopbackUntagged) {
    if (!haveRawPacketSocket())
        GTEST_SKIP() << "no CAP_NET_RAW for AF_PACKET on lo";

    HostEtherNetSession session;
    Master master;
    ASSERT_TRUE(bringUpHost(session, master, "lo", EncapsulationConfig{},
                            "test"));
    {
        HostSessionGuard guard(session, [&master] { master.stop(); });
    }
    shutdownHostEthernet(session);
    master.packetRouter().shutdown();
}

#if !TETHER_ENABLE_UDP_ENCAPSULATION
// With UDP encapsulation compiled out, a UDP config makes
// setupEncapsulation fail → bringUpHost runs the shutdown cleanup and
// returns false. Deterministic once raw sockets are available.
TEST(HostSetupWireTest, BringUpHostUdpGateCleanup) {
    if (!haveRawPacketSocket())
        GTEST_SKIP() << "no CAP_NET_RAW for AF_PACKET on lo";

    HostEtherNetSession session;
    Master master;
    EncapsulationConfig encap{};
    encap.udp = true;
    EXPECT_FALSE(bringUpHost(session, master, "lo", encap, "test"));
}
#endif
