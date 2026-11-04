/**
 * @file test_beckhoff_compact_drive_terminals.cpp
 * @brief Emulator-backed coverage for CompactDriveTerminal (EL70xx/ELM72xx
 *        DRV-interface drives) — SII axis resolution (0x70 spacing, the
 *        alternate 0x10 spacing and two-axis devices), CiA 402 control /
 *        status accessors, the EL7201 fallback shape and the rejection
 *        paths.
 */

#include <gtest/gtest.h>

#include <array>
#include <cstring>
#include <span>
#include <vector>

#include "mocks/EmulatedBus.hpp"
#include "tether/Beckhoff/CompactDriveTerminal.hpp"
#include "tether/ethercat/LogicalAddressManager.hpp"
#include "tether/ethercat/PDOManager.hpp"

using namespace EtherCAT;
using EtherCAT::Emulator::SlaveEmulator;
namespace Dev = EtherCAT::Beckhoff::Devices;
using EtherCAT::Beckhoff::Error;

namespace {

SII::SIISyncManager mkSm(uint16_t addr, uint16_t len, uint8_t type) {
    SII::SIISyncManager sm{};
    sm.phys_start_address = addr;
    sm.length = len;
    sm.sm_type = type;
    sm.enable = std::bit_cast<EtherCAT::SyncManager::SMActivateReg>(
        static_cast<uint8_t>(0x01));
    return sm;
}

SII::SIIPDOEntry mkEntry(uint16_t index, uint8_t sub, uint8_t bits) {
    SII::SIIPDOEntry e{};
    e.index = index;
    e.subindex = sub;
    e.bit_length = bits;
    return e;
}

SII::SIIPDO mkPdo(uint16_t index, uint8_t sm,
                  std::vector<SII::SIIPDOEntry> entries) {
    SII::SIIPDO p{};
    p.pdo_index = index;
    p.sync_manager = sm;
    p.entries.assign(entries.begin(), entries.end());
    p.n_entries = static_cast<uint8_t>(p.entries.size());
    return p;
}

SlaveEmulator* addDriveSlave(EmulatedBus& bus, uint32_t product,
                             uint16_t out_addr, uint16_t out_len,
                             uint16_t in_addr, uint16_t in_len) {
    auto slave = std::make_unique<SlaveEmulator>();
    EtherCAT::Emulator::SIIConfig cfg;
    cfg.vendor_id = 0x00000002;
    cfg.product_code = static_cast<uint16_t>(product & 0xFFFF);
    cfg.mailbox_protocol = 0x04;
    auto addSm = [&cfg](uint16_t addr, uint16_t len, uint8_t ctrl) {
        EtherCAT::Emulator::SIIConfig::SMConfig sm{};
        sm.start_addr = addr;
        sm.length     = len;
        sm.control    = std::bit_cast<EtherCAT::SyncManager::SMControlReg>(ctrl);
        sm.enable     = std::bit_cast<EtherCAT::SyncManager::SMActivateReg>(
            static_cast<uint8_t>(0x01));
        cfg.sync_managers.push_back(sm);
    };
    addSm(0x1000, 128, 0x26);
    addSm(0x1080, 128, 0x22);
    addSm(out_addr, out_len, 0x64);
    addSm(in_addr, in_len, 0x40);
    slave->setSIIConfig(cfg);
    auto* raw = slave.get();
    bus.net.addSlave(std::move(slave));
    return raw;
}

/// Append one DRV axis' objects to the RX/TX entry lists.
void addAxis(std::vector<SII::SIIPDOEntry>& rx,
             std::vector<SII::SIIPDOEntry>& tx,
             uint16_t out_obj, uint16_t in_obj, bool with_fb_object) {
    rx.push_back(mkEntry(out_obj, 0x01, 16));  // controlword
    rx.push_back(mkEntry(out_obj, 0x03, 8));   // modes of operation
    rx.push_back(mkEntry(out_obj, 0x05, 32));  // target position
    rx.push_back(mkEntry(out_obj, 0x06, 32));  // target velocity
    rx.push_back(mkEntry(out_obj, 0x09, 16));  // target torque

    if (with_fb_object) {
        // Feedback position object ahead of the DRV input object.
        tx.push_back(mkEntry(static_cast<uint16_t>(in_obj - 0x10), 0x11, 32));
    }
    tx.push_back(mkEntry(in_obj, 0x01, 16));   // statusword
    tx.push_back(mkEntry(in_obj, 0x03, 8));    // modes display
    tx.push_back(mkEntry(in_obj, 0x06, 32));   // following error
    tx.push_back(mkEntry(in_obj, 0x07, 32));   // actual velocity
    tx.push_back(mkEntry(in_obj, 0x08, 16));   // actual torque
}

DiscoveredSlave makeDrive(uint32_t product, const char* name,
                          uint16_t out_addr, uint16_t out_len,
                          uint16_t in_addr, uint16_t in_len,
                          bool two_axes = false, bool alt_spacing = false) {
    DiscoveredSlave s;
    s.index = 0;
    s.vendor_id = 0x00000002;
    s.product_code = product;
    s.device_name = name;
    s.sync_managers = std::vector<SII::SIISyncManager>{
        mkSm(0x1000, 128, SII::SM_TYPE_MBX_WRITE),
        mkSm(0x1080, 128, SII::SM_TYPE_MBX_READ),
        mkSm(out_addr, out_len, SII::SM_TYPE_PROCESS_OUT),
        mkSm(in_addr, in_len, SII::SM_TYPE_PROCESS_IN),
    };
    std::vector<SII::SIIPDOEntry> rx;
    std::vector<SII::SIIPDOEntry> tx;
    addAxis(rx, tx, 0x7010, 0x6010, true);
    if (two_axes) {
        const uint16_t out1 = alt_spacing ? 0x7020 : 0x7080;
        const uint16_t in1  = alt_spacing ? 0x6020 : 0x6080;
        addAxis(rx, tx, out1, in1, true);
    }
    s.rx_pdos = std::vector<SII::SIIPDO>{mkPdo(0x1600, 2, std::move(rx))};
    s.tx_pdos = std::vector<SII::SIIPDO>{mkPdo(0x1A00, 3, std::move(tx))};
    return s;
}

class BeckhoffCompactDriveTerminalsTest : public ::testing::Test {
protected:
    void TearDown() override {
        bus_.shutdown();
        master_.packetRouter().shutdown();
    }

    void bringUp() {
        bus_.attach(master_);
        master_.start(bus_.iface, kEmulatedMac);
        master_.initSlaves(1);
    }

    EmulatedBus bus_;
    Master master_;
};

}  // namespace

TEST_F(BeckhoffCompactDriveTerminalsTest, ConfigureResolvesSingleAxis) {
    auto* slave = addDriveSlave(bus_, 0x1C213052, 0x1100, 13, 0x1180, 17);
    bringUp();
    auto ds = makeDrive(0x1C213052, "EL7201", 0x1100, 13, 0x1180, 17);
    Beckhoff::CompactDriveTerminal term(master_, ds, Dev::EL7201);

    ASSERT_TRUE(term.configure().has_value());
    EXPECT_TRUE(term.configured());
    EXPECT_EQ(term.axisCount(), 1u);
    EXPECT_EQ(slave->getState(), SlaveState::SAFE_OP);

    EXPECT_TRUE(term.hasControlword(0));
    EXPECT_TRUE(term.hasModesOfOperation(0));
    EXPECT_TRUE(term.hasTargetPosition(0));
    EXPECT_TRUE(term.hasTargetVelocity(0));
    EXPECT_TRUE(term.hasTargetTorque(0));
    EXPECT_TRUE(term.hasStatusword(0));
    EXPECT_TRUE(term.hasModesOfOperationDisplay(0));
    EXPECT_TRUE(term.hasActualPosition(0));
    EXPECT_TRUE(term.hasActualVelocity(0));
    EXPECT_TRUE(term.hasActualTorque(0));
    EXPECT_TRUE(term.hasFollowingError(0));

    // Out-of-range axis has nothing.
    EXPECT_FALSE(term.hasControlword(1));
    EXPECT_FALSE(term.hasStatusword(1));

    // Idempotent.
    EXPECT_TRUE(term.configure().has_value());
}

TEST_F(BeckhoffCompactDriveTerminalsTest, OutputsLandInSm2Image) {
    auto* slave = addDriveSlave(bus_, 0x1C213052, 0x1100, 13, 0x1180, 17);
    bringUp();
    auto ds = makeDrive(0x1C213052, "EL7201", 0x1100, 13, 0x1180, 17);
    Beckhoff::CompactDriveTerminal term(master_, ds, Dev::EL7201);
    ASSERT_TRUE(term.configure().has_value());

    term.setControlword(0, 0x000F);
    EXPECT_EQ(term.controlword(0), 0x000Fu);
    term.setModesOfOperation(0, 8);
    term.setTargetPosition(0, 100000);
    term.setTargetVelocity(0, -5000);
    term.setTargetTorque(0, -250);

    ASSERT_TRUE(master_.pdoForSlave(0).exchangePhysical(1));
    EXPECT_EQ(slave->smBufferByteForTest(2, 0), 0x0F);
    EXPECT_EQ(slave->smBufferByteForTest(2, 1), 0x00);
    EXPECT_EQ(slave->smBufferByteForTest(2, 2), 8);
    int32_t pos = 0;
    uint8_t raw[4] = {slave->smBufferByteForTest(2, 3),
                      slave->smBufferByteForTest(2, 4),
                      slave->smBufferByteForTest(2, 5),
                      slave->smBufferByteForTest(2, 6)};
    std::memcpy(&pos, raw, 4);
    EXPECT_EQ(pos, 100000);
    int32_t vel = 0;
    raw[0] = slave->smBufferByteForTest(2, 7);
    raw[1] = slave->smBufferByteForTest(2, 8);
    raw[2] = slave->smBufferByteForTest(2, 9);
    raw[3] = slave->smBufferByteForTest(2, 10);
    std::memcpy(&vel, raw, 4);
    EXPECT_EQ(vel, -5000);
    int16_t tq = 0;
    raw[0] = slave->smBufferByteForTest(2, 11);
    raw[1] = slave->smBufferByteForTest(2, 12);
    std::memcpy(&tq, raw, 2);
    EXPECT_EQ(tq, -250);

    // Out-of-range axis setters are inert.
    term.setControlword(3, 0xFFFF);
    term.setModesOfOperation(3, 1);
    term.setTargetPosition(3, 1);
    term.setTargetVelocity(3, 1);
    term.setTargetTorque(3, 1);
}

TEST_F(BeckhoffCompactDriveTerminalsTest, InputsReadFromSm3Image) {
    auto* slave = addDriveSlave(bus_, 0x1C213052, 0x1100, 13, 0x1180, 17);
    bringUp();
    auto ds = makeDrive(0x1C213052, "EL7201", 0x1100, 13, 0x1180, 17);
    Beckhoff::CompactDriveTerminal term(master_, ds, Dev::EL7201);
    ASSERT_TRUE(term.configure().has_value());

    // in image: pos(0..3) status(4..5) modes(6) ferr(7..10) vel(11..14)
    // torque(15..16)
    uint8_t img[17] = {};
    const int32_t pos = -123456;
    std::memcpy(img + 0, &pos, 4);
    const uint16_t sw = 0x0027;  // operation enabled
    std::memcpy(img + 4, &sw, 2);
    img[6] = 8;
    const int32_t ferr = 42;
    std::memcpy(img + 7, &ferr, 4);
    const int32_t vel = 3000;
    std::memcpy(img + 11, &vel, 4);
    const int16_t tq = 77;
    std::memcpy(img + 15, &tq, 2);
    slave->setSmBufferForTest(3, 0, img, sizeof(img));
    ASSERT_TRUE(master_.pdoForSlave(0).exchangePhysical(1));

    EXPECT_EQ(term.statusword(0), 0x0027u);
    EXPECT_EQ(term.actualPosition(0), -123456);
    EXPECT_EQ(term.modesOfOperationDisplay(0), 8);
    EXPECT_EQ(term.followingError(0), 42);
    EXPECT_EQ(term.actualVelocity(0), 3000);
    EXPECT_EQ(term.actualTorque(0), 77);

    EXPECT_EQ(term.driveState(0), Beckhoff::DriveState::OperationEnabled);
    EXPECT_TRUE(term.operationEnabled(0));
    EXPECT_FALSE(term.fault(0));
    EXPECT_FALSE(term.warningPresent(0));
    EXPECT_FALSE(term.targetReached(0));

    // Fault / warning / target-reached bits.
    const uint16_t sw2 = 0x0008 | 0x0080 | 0x0400;
    slave->setSmBufferForTest(3, 4, reinterpret_cast<const uint8_t*>(&sw2), 2);
    ASSERT_TRUE(master_.pdoForSlave(0).exchangePhysical(1));
    EXPECT_EQ(term.driveState(0), Beckhoff::DriveState::Fault);
    EXPECT_TRUE(term.fault(0));
    EXPECT_TRUE(term.warningPresent(0));
    EXPECT_TRUE(term.targetReached(0));
}

TEST_F(BeckhoffCompactDriveTerminalsTest, ControlwordHelpers) {
    addDriveSlave(bus_, 0x1C213052, 0x1100, 13, 0x1180, 17);
    bringUp();
    auto ds = makeDrive(0x1C213052, "EL7201", 0x1100, 13, 0x1180, 17);
    Beckhoff::CompactDriveTerminal term(master_, ds, Dev::EL7201);
    ASSERT_TRUE(term.configure().has_value());

    term.requestShutdown(0);
    EXPECT_EQ(term.controlword(0), 0x0006u);
    term.requestSwitchOn(0);
    EXPECT_EQ(term.controlword(0), 0x0007u);
    term.requestEnableOperation(0);
    EXPECT_EQ(term.controlword(0), 0x000Fu);
    term.requestDisableVoltage(0);
    EXPECT_EQ(term.controlword(0), 0x0000u);
    term.requestQuickStop(0);
    EXPECT_EQ(term.controlword(0), 0x0002u);
    term.requestFaultReset(0);
    EXPECT_EQ(term.controlword(0), 0x0082u);
    term.clearFaultReset(0);
    EXPECT_EQ(term.controlword(0), 0x0002u);
}

TEST_F(BeckhoffCompactDriveTerminalsTest, TwoAxesWith0x70Spacing) {
    addDriveSlave(bus_, 0x1B963052, 0x1100, 26, 0x1180, 34);
    bringUp();
    auto ds = makeDrive(0x1B963052, "EL7062", 0x1100, 26, 0x1180, 34,
                        /*two_axes=*/true);
    Beckhoff::CompactDriveTerminal term(master_, ds, Dev::EL7062);
    ASSERT_TRUE(term.configure().has_value());
    EXPECT_EQ(term.axisCount(), 2u);

    term.setControlword(0, 0x1111);
    term.setControlword(1, 0x2222);
    EXPECT_EQ(term.controlword(0), 0x1111u);
    EXPECT_EQ(term.controlword(1), 0x2222u);
    EXPECT_TRUE(term.hasActualPosition(1));
}

TEST_F(BeckhoffCompactDriveTerminalsTest, TwoAxesWithAlternateSpacing) {
    addDriveSlave(bus_, 0x1B963052, 0x1100, 26, 0x1180, 34);
    bringUp();
    auto ds = makeDrive(0x1B963052, "EL7062", 0x1100, 26, 0x1180, 34,
                        /*two_axes=*/true, /*alt_spacing=*/true);
    Beckhoff::CompactDriveTerminal term(master_, ds, Dev::EL7062);
    ASSERT_TRUE(term.configure().has_value());
    EXPECT_EQ(term.axisCount(), 2u);

    term.setControlword(1, 0x00AA);
    EXPECT_EQ(term.controlword(1), 0x00AAu);
    EXPECT_TRUE(term.hasStatusword(1));
}

TEST_F(BeckhoffCompactDriveTerminalsTest, FallbackWithoutSii) {
    auto* slave = addDriveSlave(bus_, 0x1C213052, 0x1100, 6, 0x1180, 6);
    bringUp();
    // No process SM: the EL7201 default 6B/6B shape is assumed.
    DiscoveredSlave ds;
    ds.index = 0;
    ds.vendor_id = 0x00000002;
    ds.product_code = 0x1C213052;
    ds.device_name = "EL7201";
    ds.sync_managers = std::vector<SII::SIISyncManager>{
        mkSm(0x1000, 128, SII::SM_TYPE_MBX_WRITE),
        mkSm(0x1080, 128, SII::SM_TYPE_MBX_READ),
        mkSm(0, 0, SII::SM_TYPE_UNUSED),
        mkSm(0, 0, SII::SM_TYPE_UNUSED),
    };

    Beckhoff::CompactDriveTerminal term(master_, ds, Dev::EL7201);
    ASSERT_TRUE(term.configure().has_value());
    EXPECT_EQ(term.axisCount(), 1u);
    EXPECT_TRUE(term.hasControlword(0));
    EXPECT_TRUE(term.hasTargetVelocity(0));
    EXPECT_TRUE(term.hasActualPosition(0));
    EXPECT_TRUE(term.hasStatusword(0));
    EXPECT_FALSE(term.hasTargetPosition(0));

    term.setControlword(0, 0x000F);
    term.setTargetVelocity(0, -100);
    ASSERT_TRUE(master_.pdoForSlave(0).exchangePhysical(1));
    EXPECT_EQ(slave->smBufferByteForTest(2, 0), 0x0F);
}

TEST_F(BeckhoffCompactDriveTerminalsTest, NoDrvObjectsRejected) {
    addDriveSlave(bus_, 0x1C213052, 0x1100, 4, 0x1180, 4);
    bringUp();
    // Process SMs exist but carry no DRV controlword/statusword objects.
    DiscoveredSlave ds;
    ds.index = 0;
    ds.vendor_id = 0x00000002;
    ds.product_code = 0x1C213052;
    ds.device_name = "EL7201";
    ds.sync_managers = std::vector<SII::SIISyncManager>{
        mkSm(0x1000, 128, SII::SM_TYPE_MBX_WRITE),
        mkSm(0x1080, 128, SII::SM_TYPE_MBX_READ),
        mkSm(0x1100, 4, SII::SM_TYPE_PROCESS_OUT),
        mkSm(0x1180, 4, SII::SM_TYPE_PROCESS_IN),
    };
    ds.rx_pdos = std::vector<SII::SIIPDO>{mkPdo(0x1600, 2, {
        mkEntry(0x3000, 1, 16)})};
    ds.tx_pdos = std::vector<SII::SIIPDO>{mkPdo(0x1A00, 3, {
        mkEntry(0x3100, 1, 16)})};

    Beckhoff::CompactDriveTerminal term(master_, ds, Dev::EL7201);
    auto r = term.configure();
    ASSERT_FALSE(r.has_value());
    EXPECT_EQ(r.error(), Error::SmConfigFailed);
}

TEST_F(BeckhoffCompactDriveTerminalsTest, Rejections) {
    bringUp();
    auto ds = makeDrive(0x1C213052, "EL7201", 0x1100, 13, 0x1180, 17);
    Beckhoff::CompactDriveTerminal wrong(master_, ds, Dev::EL7211);
    auto r = wrong.configure();
    ASSERT_FALSE(r.has_value());
    EXPECT_EQ(r.error(), Error::WrongDevice);

    Beckhoff::CompactDriveTerminal far(master_, PDO::kMaxPDOSlaves,
                                       Dev::EL7201);
    auto r2 = far.configure();
    ASSERT_FALSE(r2.has_value());
    EXPECT_EQ(r2.error(), Error::SlaveIndexOutOfRange);
}

TEST_F(BeckhoffCompactDriveTerminalsTest, Factories) {
    auto ds = makeDrive(0x1C213052, "EL7201", 0x1100, 13, 0x1180, 17);
    std::span<const DiscoveredSlave> scan{&ds, 1};

    auto miss = Beckhoff::CompactDriveTerminal::findFirst(master_, Dev::EL7211,
                                                          scan);
    ASSERT_FALSE(miss.has_value());
    EXPECT_EQ(miss.error(), Error::NoDeviceFound);

    auto hit = Beckhoff::CompactDriveTerminal::findFirst(master_, Dev::EL7201,
                                                         scan);
    ASSERT_TRUE(hit.has_value());

    addDriveSlave(bus_, 0x1C213052, 0x1100, 13, 0x1180, 17);
    bringUp();
    auto live = Beckhoff::CompactDriveTerminal::findFirst(master_, Dev::EL7201);
    EXPECT_FALSE(live.has_value());
    EXPECT_EQ(live.error(), Error::NoDeviceFound);
}

TEST_F(BeckhoffCompactDriveTerminalsTest, LogicalAndStart) {
    auto* slave = addDriveSlave(bus_, 0x1C213052, 0x1100, 13, 0x1180, 17);
    bringUp();
    auto ds = makeDrive(0x1C213052, "EL7201", 0x1100, 13, 0x1180, 17);
    Beckhoff::CompactDriveTerminal term(master_, ds, Dev::EL7201);

    ASSERT_TRUE(term.prepareForLogicalExchange().has_value());
    auto& pdo = master_.pdoForSlave(0);
    auto& lam = master_.logicalAddressManagerForSlave(0);
    ASSERT_TRUE(lam.buildAddressMap(pdo.slaveConfigs(), 1));
    ASSERT_TRUE(term.mapLogicalAndEnterSafeOp().has_value());
    EXPECT_TRUE(term.mapLogicalAndEnterSafeOp().has_value());
    EXPECT_GT(term.logicalAddress(), 0u);
    EXPECT_TRUE(term.requestOp(2000).has_value());
    EXPECT_TRUE(term.operational());
    term.stop();
    (void)slave;
}

TEST_F(BeckhoffCompactDriveTerminalsTest, StartWithoutManagedLoop) {
    auto* slave = addDriveSlave(bus_, 0x1C213052, 0x1100, 13, 0x1180, 17);
    bringUp();
    auto ds = makeDrive(0x1C213052, "EL7201", 0x1100, 13, 0x1180, 17);
    Beckhoff::CompactDriveTerminal term(master_, ds, Dev::EL7201);

    Beckhoff::StartOptions opts;
    opts.manage_realtime_loop = false;
    opts.op_timeout_ms = 2000;
    ASSERT_TRUE(term.start(opts).has_value());
    EXPECT_EQ(slave->getState(), SlaveState::OP);
    EXPECT_TRUE(term.operational());
}
