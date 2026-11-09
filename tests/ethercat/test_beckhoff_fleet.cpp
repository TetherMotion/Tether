// SPDX-License-Identifier: MIT
/**
 * @file test_beckhoff_fleet.cpp
 * @brief End-to-end test: emulated Beckhoff fleet <-> Tether master
 *        over the UDP-carried Ethernet-frame transport.
 *
 * Runs a BeckhoffFleet (2x EK1100 + EL1004/EL2004/EL3104/EL4104 per
 * coupler) and a FleetMaster in one process, connected through two
 * UdpEthercatLink sockets on loopback.  Verifies discovery, SII
 * identities, AL bring-up to OP, and cyclic PDO exchange in both
 * directions:
 *
 *   slave inputs  --(Tether IO params equivalent: profile setters)-->
 *   TxPDO -> master entry storage
 *   master entry storage -> RxPDO -> slave outputs
 */
#include "common/BeckhoffFleet.hpp"
#include "common/EthercatFleetMaster.hpp"
#include "common/UdpEthercatLink.hpp"

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <stdexcept>
#include <cstdint>
#include <cstring>
#include <thread>
#include <vector>

using namespace tether::examples;

namespace {

/// Slave-side UDP server: forwards datagrams through the fleet chain.
class FleetUdpServer {
public:
    FleetUdpServer(BeckhoffFleet& fleet, uint16_t port)
        : fleet_(fleet) {
        if (!link_.open("127.0.0.1", 0, port, "127.0.0.1"))
            throw std::runtime_error("FleetUdpServer: link open failed");
        port_ = link_.localPort();
        running_.store(true);
        thread_ = std::thread([this]() { loop(); });
    }
    ~FleetUdpServer() { stop(); }

    uint16_t port() const { return port_; }

    void stop() {
        running_.store(false);
        if (thread_.joinable()) thread_.join();
    }

private:
    void loop() {
        uint8_t frame[2048];
        sockaddr_in from{};
        while (running_.load()) {
            size_t n = 0;
            if (!link_.recvFrom(frame, sizeof(frame), &n, from)) {
                std::this_thread::sleep_for(std::chrono::microseconds(100));
                continue;
            }
            auto resp = fleet_.processFrame(frame, n);
            if (!resp.empty()) link_.sendTo(resp.data(), resp.size(), from);
        }
    }

    BeckhoffFleet& fleet_;
    UdpEthercatLink link_;
    uint16_t port_ = 0;
    std::atomic<bool> running_{false};
    std::thread thread_;
};

struct FleetFixture {
    BeckhoffFleet fleet;
    FleetUdpServer server{fleet, 0};
    FleetMaster master;
    std::atomic<bool> ticking{true};
    std::thread tickThread;

    FleetFixture() {
        fleet.start();
        tickThread = std::thread([this]() {
            while (ticking.load()) {
                fleet.tick(1000);   // 1 ms per iteration
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }
        });
        if (!master.open("127.0.0.1", server.port()))
            throw std::runtime_error("FleetFixture: master open failed");
    }
    ~FleetFixture() {
        ticking.store(false);
        if (tickThread.joinable()) tickThread.join();
        server.stop();
        fleet.stop();
    }
};

/// Read a TxPDO (slave->master) entry's storage for @p key.
static const uint8_t* txStorage(FleetMaster& fm, const FleetSlaveInfo& inf,
                                size_t& size) {
    int e = fm.findEntry(static_cast<uint16_t>(inf.chainIndex), true);
    EXPECT_GE(e, 0);
    if (e < 0) return nullptr;
    const auto* ent = fm.pdo().mapping().get_entry(e);
    EXPECT_NE(ent, nullptr);
    if (!ent) return nullptr;
    size = ent->data_size;
    return ent->storage;
}

/// Locate + return a mutable RxPDO (master->slave) entry for @p key.
static uint8_t* rxStorage(FleetMaster& fm, const FleetSlaveInfo& inf,
                          size_t& size) {
    int e = fm.findEntry(static_cast<uint16_t>(inf.chainIndex), false);
    EXPECT_GE(e, 0);
    if (e < 0) return nullptr;
    auto* ent = fm.pdo().mapping().get_entry_mut(e);
    EXPECT_NE(ent, nullptr);
    if (!ent) return nullptr;
    size = ent->data_size;
    return ent->storage;
}

static const FleetSlaveInfo* infoByKey(
    const std::vector<FleetSlaveInfo>& infos, const char* key) {
    for (const auto& i : infos)
        if (i.key == key) return &i;
    return nullptr;
}

} // namespace

TEST(BeckhoffFleet, BringUpAndCyclicExchange) {
    FleetFixture fx;
    const auto infos = fleetInfos();
    ASSERT_EQ(infos.size(), 10u);

    std::vector<EtherCAT::DiscoveredSlave> found;
    int n = fx.master.bringUp(infos, &found);
    ASSERT_EQ(n, 10);
    ASSERT_EQ(found.size(), infos.size());

    // SII identities match the Beckhoff parts.
    for (size_t i = 0; i < infos.size(); ++i) {
        ASSERT_TRUE(found[i].vendor_id.has_value());
        ASSERT_TRUE(found[i].product_code.has_value());
        EXPECT_EQ(*found[i].vendor_id, infos[i].vendorId);
        EXPECT_EQ(*found[i].product_code, infos[i].productCode);
        EXPECT_EQ(*found[i].revision_number,
                  beckhoff_ids::kRevision);
    }

    // Cyclic exchange runs on the master's background thread; give it
    // time to settle and confirm it stays healthy (request/reply
    // counters keep advancing, checked via a fresh exchange result).
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    EXPECT_TRUE(fx.master.exchange());
}

TEST(BeckhoffFleet, DigitalAndAnalogPath) {
    FleetFixture fx;
    const auto infos = fleetInfos();
    ASSERT_EQ(fx.master.bringUp(infos), 10);

    const FleetSlaveInfo* el1004 = infoByKey(infos, "el1004.c1");
    const FleetSlaveInfo* el2004 = infoByKey(infos, "el2004.c1");
    const FleetSlaveInfo* el3104 = infoByKey(infos, "el3104.c1");
    const FleetSlaveInfo* el4104 = infoByKey(infos, "el4104.c1");
    ASSERT_TRUE(el1004 && el2004 && el3104 && el4104);

    // --- slave inputs -> master TxPDO storage -------------------------
    fx.fleet.slave(el1004->chainIndex).setDigitalInput8(0, 0xA5);
    fx.fleet.slave(el3104->chainIndex).setAnalogInput(0, 1234);
    fx.fleet.slave(el3104->chainIndex).setAnalogInput(3, -7);
    // Let background cycles run so updateTxPDO + exchange pick the
    // values up (exchangeAll runs on the master's exchange thread).
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    size_t sz = 0;
    const uint8_t* di = txStorage(fx.master, *el1004, sz);
    ASSERT_NE(di, nullptr);
    EXPECT_EQ(sz, 1u);
    EXPECT_EQ(di[0], 0xA5);

    const uint8_t* ai = txStorage(fx.master, *el3104, sz);
    ASSERT_NE(ai, nullptr);
    EXPECT_EQ(sz, 8u);
    int16_t v0, v3;
    std::memcpy(&v0, ai + 0, 2);
    std::memcpy(&v3, ai + 6, 2);
    EXPECT_EQ(v0, 1234);
    EXPECT_EQ(v3, -7);

    // --- master RxPDO storage -> slave outputs ------------------------
    uint8_t* dout = rxStorage(fx.master, *el2004, sz);
    ASSERT_NE(dout, nullptr);
    EXPECT_EQ(sz, 1u);
    dout[0] = 0x3C;
    uint8_t* aout = rxStorage(fx.master, *el4104, sz);
    ASSERT_NE(aout, nullptr);
    EXPECT_EQ(sz, 8u);
    int16_t av = -320;
    std::memcpy(aout + 2, &av, 2);   // analog channel 1

    // Poll until the write has propagated through several cycles.
    for (int i = 0; i < 100; ++i) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        if (fx.fleet.slave(el2004->chainIndex).getDigitalOutput8(0) == 0x3C &&
            fx.fleet.slave(el4104->chainIndex).getAnalogOutput(1) == -320)
            break;
    }

    EXPECT_EQ(fx.fleet.slave(el2004->chainIndex).getDigitalOutput8(0),
              0x3C);
    EXPECT_EQ(fx.fleet.slave(el4104->chainIndex).getAnalogOutput(1),
              -320);
}
