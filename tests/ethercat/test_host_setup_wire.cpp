/**
 * @file test_host_setup_wire.cpp
 * @brief startHostMasterAndDiscover over the emulated wire — the
 *        slave-enumeration log path and the empty-bus early return.
 */

#include <gtest/gtest.h>

#include <cstring>

#include "mocks/EmulatedBus.hpp"
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
