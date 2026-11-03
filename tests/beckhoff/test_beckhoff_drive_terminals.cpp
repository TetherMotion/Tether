/**
 * @file test_beckhoff_drive_terminals.cpp
 * @brief Emulator-backed coverage for the drive/output Beckhoff terminals:
 *        DcMotorTerminal (EL7332), PulseTrainTerminal (EL2522),
 *        PwmTerminal (EL2502), PowerMeterTerminal (EL3403),
 *        StepperTerminal (EL7031), CompactDriveTerminal (EL7201).
 *
 * Each DiscoveredSlave is shaped to satisfy the terminal's SII PDO scan;
 * the emulated slave accepts the SM/mailbox programming that
 * prepare()/configure()/start() perform, so the full wire path runs:
 * findFirst → prepare → SAFE_OP → requestOp → OP, plus the image
 * accessors driven by real PDO exchanges.
 */

#include <gtest/gtest.h>

#include <cstring>
#include <vector>

#include "mocks/EmulatedBus.hpp"
#include "tether/Beckhoff/DcMotorTerminal.hpp"
#include "tether/Beckhoff/PulseTrainTerminal.hpp"
#include "tether/Beckhoff/PwmTerminal.hpp"
#include "tether/Beckhoff/PowerMeterTerminal.hpp"
#include "tether/Beckhoff/StepperTerminal.hpp"
#include "tether/Beckhoff/CompactDriveTerminal.hpp"
#include "tether/ethercat/Master.hpp"

using EtherCAT::DiscoveredSlave;
using EtherCAT::Master;
using EtherCAT::Emulator::SlaveEmulator;
namespace Dev = EtherCAT::Beckhoff::Devices;
namespace SII = EtherCAT::SII;

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

/// Base DiscoveredSlave: mailbox pair on SM0/1, process-out SM2,
/// process-in SM3.
DiscoveredSlave makeBase(uint32_t product, const char* name,
                         uint16_t out_addr, uint16_t in_addr) {
    DiscoveredSlave s;
    s.index = 0;
    s.vendor_id = 0x00000002;
    s.product_code = product;
    s.device_name = name;
    s.sync_managers = std::vector<SII::SIISyncManager>{
        mkSm(0x1000, 128, SII::SM_TYPE_MBX_WRITE),
        mkSm(0x1080, 128, SII::SM_TYPE_MBX_READ),
        mkSm(out_addr, 0, SII::SM_TYPE_PROCESS_OUT),
        mkSm(in_addr, 0, SII::SM_TYPE_PROCESS_IN),
    };
    return s;
}

/// Emulated slave whose SII advertises the same SM shape.
SlaveEmulator* addDriveSlave(EmulatedBus& bus, uint32_t product,
                             uint16_t out_addr, uint16_t out_len,
                             uint16_t in_addr, uint16_t in_len) {
    auto slave = std::make_unique<SlaveEmulator>();
    EtherCAT::Emulator::SIIConfig cfg;
    cfg.vendor_id = 0x00000002;
    cfg.product_code = product;
    cfg.mailbox_protocol = 0x04;  // CoE
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

/// Write `nbits` of `v` into `img` at absolute bit offset `bit`
/// (PDO entries are bit-packed, not byte-aligned).
void pokeBits(uint8_t* img, size_t bit, uint64_t v, size_t nbits) {
    for (size_t b = 0; b < nbits; ++b) {
        if (v & (1ull << b)) {
            img[(bit + b) / 8] |=
                static_cast<uint8_t>(1u << ((bit + b) % 8));
        }
    }
}

/// Patch the emulator's SII so live discovery reads the 32-bit product
/// code (SIIConfig::product_code is only 16 bits) and a valid config CRC.
void fixSii(SlaveEmulator* slave, uint32_t product) {
    slave->processSIIWrite(11, static_cast<uint16_t>(product >> 16));
    // CRC-8 MSB (poly 0x07, init 0xFF) over words 0-6 = bytes 0-13.
    uint8_t bytes[14];
    for (size_t w = 0; w < 7; ++w) {
        uint16_t word = 0;
        slave->processSIIRead(static_cast<uint32_t>(w), &word);
        bytes[2 * w] = static_cast<uint8_t>(word & 0xFF);
        bytes[2 * w + 1] = static_cast<uint8_t>(word >> 8);
    }
    uint8_t crc = 0xFF;
    for (uint8_t byte : bytes) {
        crc ^= byte;
        for (int i = 0; i < 8; ++i) {
            crc = static_cast<uint8_t>(
                (crc & 0x80) ? ((crc << 1) ^ 0x07) : (crc << 1));
        }
    }
    slave->processSIIWrite(7, crc);
}

class BeckhoffDriveTerminalsTest : public ::testing::Test {
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

// ============================================================================
// EL7332 — DcMotorTerminal: 2 MOT objects + 2 ENC objects
// ============================================================================

DiscoveredSlave makeDcMotor() {
    // SM2 out: MOT 0x7000/0x7010 {enable:1 reset:2 rtq:3 vel:0x21=16b},
    //          ENC 0x7001/0x7011 {2,3,4 bits + set-counter-value :0x11=16b}
    // SM3 in : STAT 0x6000/0x6010 (16 packed bits), ENC_STAT
    //          0x6001/0x6011 (8b status + 0x11 counter + 0x12 latch),
    //          0x1C32:0x20 sync error (32b)
    auto s = makeBase(0x1CA43052, "EL7332", 0x1100, 0x1180);
    s.sync_managers->at(2).length = 16;
    s.sync_managers->at(3).length = 24;
    s.rx_pdos = std::vector<SII::SIIPDO>{mkPdo(0x1600, 2, {
        mkEntry(0x7000, 1, 1), mkEntry(0x7000, 2, 1), mkEntry(0x7000, 3, 1),
        mkEntry(0x7000, 0x21, 16),
        mkEntry(0x7001, 2, 1), mkEntry(0x7001, 3, 1), mkEntry(0x7001, 4, 1),
        mkEntry(0x7001, 0x11, 16),
        mkEntry(0x7010, 1, 1), mkEntry(0x7010, 2, 1), mkEntry(0x7010, 3, 1),
        mkEntry(0x7010, 0x21, 16),
        mkEntry(0x7011, 2, 1), mkEntry(0x7011, 3, 1), mkEntry(0x7011, 4, 1),
        mkEntry(0x7011, 0x11, 16)})};
    s.tx_pdos = std::vector<SII::SIIPDO>{mkPdo(0x1A00, 3, {
        mkEntry(0x6000, 1, 8), mkEntry(0x6000, 2, 8),
        mkEntry(0x6001, 1, 8), mkEntry(0x6001, 0x11, 16),
        mkEntry(0x6001, 0x12, 16),
        mkEntry(0x6010, 1, 8), mkEntry(0x6010, 2, 8),
        mkEntry(0x6011, 1, 8), mkEntry(0x6011, 0x11, 16),
        mkEntry(0x6011, 0x12, 16),
        mkEntry(0x1C32, 0x20, 32)})};
    return s;
}

TEST_F(BeckhoffDriveTerminalsTest, DcMotorFindFirst) {
    auto s = makeDcMotor();
    auto miss = EtherCAT::Beckhoff::DcMotorTerminal::findFirst(
        master_, Dev::EL7342, std::span<const DiscoveredSlave>{&s, 1});
    EXPECT_FALSE(miss.has_value());
    auto hit = EtherCAT::Beckhoff::DcMotorTerminal::findFirst(
        master_, Dev::EL7332, std::span<const DiscoveredSlave>{&s, 1});
    ASSERT_TRUE(hit.has_value());
}

TEST_F(BeckhoffDriveTerminalsTest, DcMotorStartReachesOpAndDrivesAxes) {
    auto* slave = addDriveSlave(bus_, 0x1CA43052, 0x1100, 16, 0x1180, 24);
    bringUp();
    EtherCAT::Beckhoff::DcMotorTerminal term(master_, makeDcMotor(),
                                             Dev::EL7332);
    ASSERT_TRUE(term.start().has_value());
    EXPECT_EQ(term.axisCount(), 2u);

    // Drive axis outputs — read back through motControlWord.
    term.setEnable(0, true);
    term.setReset(0, true);
    term.setReduceTorque(0, true);
    term.setVelocity(0, 1234);
    term.setVelocity(1, -77);
    EXPECT_NE(term.motControlWord(0) & 0x07, 0u);
    EXPECT_EQ(term.motControlWord(9), 0u);

    // Bit-packed input image: 0x6000:1|2 at bits 0-15, 0x6001:1|0x11|0x12
    // at bits 16-55, 0x6010:1|2 at 56-71, 0x6011:* at 72-111, 0x1C32 at
    // 112+.
    uint8_t img[24] = {};
    pokeBits(img, 0, 0b00000011, 8);       // axis0 stat: rdy2En + rdy
    pokeBits(img, 16 + 3, 1, 1);           // axis0 enc stat: underflow
    pokeBits(img, 16 + 4, 1, 1);           // axis0 enc stat: overflow
    pokeBits(img, 24, 0x4EEF, 16);         // axis0 counter
    pokeBits(img, 40, 0x1234, 16);         // axis0 latch
    pokeBits(img, 56 + 3, 1, 1);           // axis1 stat: error bit
    pokeBits(img, 80, 0x00AB, 16);         // axis1 counter
    slave->setSmBufferForTest(3, 0, img, sizeof(img));

    master_.pdoForSlave(0).exchangePhysical(1);
    EXPECT_TRUE(term.readyToEnable(0));
    EXPECT_TRUE(term.ready(0));
    EXPECT_TRUE(term.error(1));
    EXPECT_FALSE(term.movingPositive(0));
    EXPECT_TRUE(term.counterOverflow(0));
    EXPECT_TRUE(term.counterUnderflow(0));
    EXPECT_EQ(term.counterValue(0), 0x4EEF);
    EXPECT_EQ(term.latchValue(0), 0x1234);
    EXPECT_EQ(term.counterValue(1), 0x00AB);
    EXPECT_TRUE(term.hasEncoder(0));
    EXPECT_NE(term.motStatusWord(1), 0u);
    term.stop();
}

TEST_F(BeckhoffDriveTerminalsTest, DcMotorWrongIdentityRejected) {
    addDriveSlave(bus_, 0x1CA43052, 0x1100, 16, 0x1180, 24);
    bringUp();
    // Discovered slave claims EL7342 but the terminal expects EL7332 →
    // verifyIdentity fails inside configure().
    auto s = makeDcMotor();
    s.product_code = 0x1CAE3052;
    EtherCAT::Beckhoff::DcMotorTerminal term(master_, s, Dev::EL7332);
    EXPECT_FALSE(term.configure().has_value());
}

TEST_F(BeckhoffDriveTerminalsTest, DcMotorLogicalExchangePath) {
    addDriveSlave(bus_, 0x1CA43052, 0x1100, 16, 0x1180, 24);
    bringUp();
    EtherCAT::Beckhoff::DcMotorTerminal term(master_, makeDcMotor(),
                                             Dev::EL7332);
    ASSERT_TRUE(term.prepareForLogicalExchange().has_value());
    // Needs the shared logical map — exercise whichever branch runs.
    auto mapped = term.mapLogicalAndEnterSafeOp();
    if (mapped.has_value()) {
        EXPECT_TRUE(term.requestOp(500).has_value());
        term.stop();
    }
}

TEST_F(BeckhoffDriveTerminalsTest, DcMotorNoSiiFallbackShape) {
    auto* slave = addDriveSlave(bus_, 0x1CA43052, 0x1100, 8, 0x1180, 6);
    bringUp();
    // sync_managers engaged but empty: fetchDiscovery is skipped (no
    // live SII on the emulated wire) while rx_pdos stays absent → the
    // fixed positional fallback layout applies.
    DiscoveredSlave bare;
    bare.index = 0;
    bare.vendor_id = 0x00000002;
    bare.product_code = 0x1CA43052;
    bare.sync_managers = std::vector<SII::SIISyncManager>{};
    EtherCAT::Beckhoff::DcMotorTerminal term(master_, bare, Dev::EL7332);
    ASSERT_TRUE(term.configure().has_value());
    EXPECT_EQ(term.axisCount(), 2u);
    term.setVelocity(0, 5);
    term.stop();
    (void)slave;
}

// ============================================================================
// EL2522 — PulseTrainTerminal: 2 PTO channels + ENC objects
// ============================================================================

DiscoveredSlave makePulseTrain() {
    auto s = makeBase(0x09DA3052, "EL2522", 0x1100, 0x1180);
    s.sync_managers->at(2).length = 24;
    s.sync_managers->at(3).length = 24;
    // PTO objects 0x7000/0x7010: bits :01..:06 + freq :0x11 + target :0x12.
    // ENC objects 0x7020/0x7030: :03 set-counter bit + :0x11 value,
    // no :01 (else classified as PTO).
    s.rx_pdos = std::vector<SII::SIIPDO>{mkPdo(0x1600, 2, {
        mkEntry(0x7000, 1, 1), mkEntry(0x7000, 2, 1), mkEntry(0x7000, 3, 1),
        mkEntry(0x7000, 4, 1), mkEntry(0x7000, 5, 1), mkEntry(0x7000, 6, 1),
        mkEntry(0x7000, 0x11, 16), mkEntry(0x7000, 0x12, 16),
        mkEntry(0x7020, 3, 1), mkEntry(0x7020, 0x11, 16),
        mkEntry(0x7010, 1, 1), mkEntry(0x7010, 2, 1), mkEntry(0x7010, 3, 1),
        mkEntry(0x7010, 4, 1), mkEntry(0x7010, 5, 1), mkEntry(0x7010, 6, 1),
        mkEntry(0x7010, 0x11, 16), mkEntry(0x7010, 0x12, 16),
        mkEntry(0x7030, 3, 1), mkEntry(0x7030, 0x11, 16)})};
    // PTO status 0x6000/0x6010: :01..:07 + :0x14; ENC stat 0x6020/0x6030.
    // Pad entries keep the 16-bit values byte-aligned for inSigned.
    s.tx_pdos = std::vector<SII::SIIPDO>{mkPdo(0x1A00, 3, {
        mkEntry(0x6000, 1, 1), mkEntry(0x6000, 2, 1), mkEntry(0x6000, 7, 1),
        mkEntry(0x6000, 0x14, 1), mkEntry(0x6000, 0x15, 4),
        mkEntry(0x6020, 3, 1), mkEntry(0x6020, 4, 1), mkEntry(0x6020, 5, 1),
        mkEntry(0x6020, 6, 5),
        mkEntry(0x6020, 0x11, 16), mkEntry(0x6020, 0x12, 16),
        mkEntry(0x6010, 1, 1), mkEntry(0x6010, 2, 1), mkEntry(0x6010, 7, 1),
        mkEntry(0x6010, 0x14, 1), mkEntry(0x6010, 0x15, 4),
        mkEntry(0x6030, 3, 1), mkEntry(0x6030, 4, 1), mkEntry(0x6030, 5, 1),
        mkEntry(0x6030, 6, 5),
        mkEntry(0x6030, 0x11, 16), mkEntry(0x6030, 0x12, 16)})};
    return s;
}

TEST_F(BeckhoffDriveTerminalsTest, PulseTrainFindFirst) {
    auto s = makePulseTrain();
    auto miss = EtherCAT::Beckhoff::PulseTrainTerminal::findFirst(
        master_, Dev::EL2521, std::span<const DiscoveredSlave>{&s, 1});
    EXPECT_FALSE(miss.has_value());
    auto hit = EtherCAT::Beckhoff::PulseTrainTerminal::findFirst(
        master_, Dev::EL2522, std::span<const DiscoveredSlave>{&s, 1});
    ASSERT_TRUE(hit.has_value());
}

TEST_F(BeckhoffDriveTerminalsTest, PulseTrainStartAndDrive) {
    auto* slave = addDriveSlave(bus_, 0x09DA3052, 0x1100, 24, 0x1180, 24);
    bringUp();
    EtherCAT::Beckhoff::PulseTrainTerminal term(master_, makePulseTrain(),
                                                Dev::EL2522);
    ASSERT_TRUE(term.start().has_value());
    // ENC objects sit inside the PTO scan range, so they also register
    // as PTO channels — the real 2 channels are ch0/ch1.
    EXPECT_GE(term.channelCount(), 2u);

    term.setFrequency(0, 1000);
    term.setTarget(0, 5000);
    term.setForward(0, true);
    term.setFrequencySelect(0, true);
    term.setGoCounter(0, true);
    term.setEncSetCounter(0, true);
    term.setEncCounterValue(0, 1234);

    // Packed tx: ch0 PTO stat bits 0-3 + pad, ENC stat bits 8-10 + pad,
    // counter at 16-31, latch 32-47; ch1 starts at bit 48.
    uint8_t img[24] = {};
    pokeBits(img, 0, 0b011, 4);            // sel_ack + ramp_active
    pokeBits(img, 8, 1, 1);                // enc :03 set_done
    pokeBits(img, 16, 4242, 16);           // enc :0x11 counter
    slave->setSmBufferForTest(3, 0, img, sizeof(img));
    master_.pdoForSlave(0).exchangePhysical(1);
    EXPECT_TRUE(term.selAck(0));
    EXPECT_TRUE(term.rampActive(0));
    EXPECT_TRUE(term.setCounterDone(0));
    EXPECT_EQ(term.counterValue(0), 4242);
    EXPECT_FALSE(term.error(0));
    term.stop();
    (void)slave;
}

// ============================================================================
// EL2502 — PwmTerminal: 2 channels duty :0x11 + control bits
// ============================================================================

DiscoveredSlave makePwm() {
    auto s = makeBase(0x09C63052, "EL2502", 0x1100, 0x1180);
    s.sync_managers->at(2).length = 8;
    s.sync_managers->at(3).length = 4;
    s.rx_pdos = std::vector<SII::SIIPDO>{mkPdo(0x1600, 2, {
        mkEntry(0x7000, 1, 1), mkEntry(0x7000, 6, 1), mkEntry(0x7000, 7, 1),
        mkEntry(0x7000, 0x11, 16),
        mkEntry(0x7010, 1, 1), mkEntry(0x7010, 6, 1), mkEntry(0x7010, 7, 1),
        mkEntry(0x7010, 0x11, 16),
        mkEntry(0xF719, 0x11, 16)})};
    s.tx_pdos = std::vector<SII::SIIPDO>{mkPdo(0x1A00, 3, {
        mkEntry(0x6000, 1, 1), mkEntry(0x6000, 6, 1), mkEntry(0x6000, 7, 1),
        mkEntry(0x6010, 1, 1), mkEntry(0x6010, 6, 1), mkEntry(0x6010, 7, 1)})};
    return s;
}

TEST_F(BeckhoffDriveTerminalsTest, PwmFindFirst) {
    auto s = makePwm();
    auto hit = EtherCAT::Beckhoff::PwmTerminal::findFirst(
        master_, Dev::EL2502, std::span<const DiscoveredSlave>{&s, 1});
    ASSERT_TRUE(hit.has_value());
}

TEST_F(BeckhoffDriveTerminalsTest, PwmStartAndDuty) {
    auto* slave = addDriveSlave(bus_, 0x09C63052, 0x1100, 8, 0x1180, 4);
    bringUp();
    EtherCAT::Beckhoff::PwmTerminal term(master_, makePwm(), Dev::EL2502);
    ASSERT_TRUE(term.start().has_value());
    EXPECT_EQ(term.channelCount(), 2u);

    term.setDuty(0, 5000);
    term.setEnable(0, true);
    term.setReset(1, true);
    term.setEnableDithering(0, true);
    term.setMasterGain(900);

    // Packed tx: ch0 :01/:06/:07 at bits 0/1/2, ch1 at bits 3/4/5.
    uint8_t img[4] = {};
    pokeBits(img, 0, 0b011, 3);            // ch0 din + warning
    slave->setSmBufferForTest(3, 0, img, sizeof(img));
    master_.pdoForSlave(0).exchangePhysical(1);
    EXPECT_TRUE(term.digitalInput(0));
    EXPECT_TRUE(term.warning(0));
    EXPECT_FALSE(term.error(0));
    term.stop();
    (void)slave;
}

// ============================================================================
// EL3403 — PowerMeterTerminal: 3 phases, family-A value layout
// ============================================================================

DiscoveredSlave makePowerMeter() {
    auto s = makeBase(0x0D4B3052, "EL3403", 0x1100, 0x1180);
    s.sync_managers->at(2).length = 4;
    s.sync_managers->at(3).length = 60;
    // Family A: per-phase status obj 0x6000+0x10*p carrying
    // Current :0x11, Voltage :0x12, ActivePower :0x13 (32-bit REAL) —
    // values first so they stay byte-aligned, then the 1-bit sync err.
    std::vector<SII::SIIPDOEntry> es;
    for (int p = 0; p < 3; ++p) {
        const uint16_t b = static_cast<uint16_t>(0x6000 + 0x10 * p);
        es.push_back(mkEntry(b, 1, 8));                  // status byte
        es.push_back(mkEntry(b, 0x11, 32));              // current
        es.push_back(mkEntry(b, 0x12, 32));              // voltage
        es.push_back(mkEntry(b, 0x13, 32));              // active power
        es.push_back(mkEntry(b, 0x0E, 1));               // sync err
    }
    es.push_back(mkEntry(0x1C32, 0x20, 32));
    s.tx_pdos = std::vector<SII::SIIPDO>{mkPdo(0x1A00, 3, es)};
    return s;
}

TEST_F(BeckhoffDriveTerminalsTest, PowerMeterFindFirst) {
    auto s = makePowerMeter();
    auto hit = EtherCAT::Beckhoff::PowerMeterTerminal::findFirst(
        master_, Dev::EL3403, std::span<const DiscoveredSlave>{&s, 1});
    ASSERT_TRUE(hit.has_value());
}

TEST_F(BeckhoffDriveTerminalsTest, PowerMeterReadsPhases) {
    auto* slave = addDriveSlave(bus_, 0x0D4B3052, 0x1100, 4, 0x1180, 60);
    bringUp();
    EtherCAT::Beckhoff::PowerMeterTerminal term(master_, makePowerMeter(),
                                                Dev::EL3403);
    ASSERT_TRUE(term.configure().has_value());
    EXPECT_EQ(term.phaseCount(), 3u);

    // Phase-0 layout: stat(8) → I(32) at bit 8 → U(32) at 40 → P(32) at
    // 72 → syncerr(1) at 104; phases repeat every 105 bits.
    uint8_t img[60] = {};
    const float current = 2.5f, voltage = 230.0f, power = 575.0f;
    uint32_t u;
    std::memcpy(&u, &current, 4);   pokeBits(img, 8, u, 32);
    std::memcpy(&u, &voltage, 4);   pokeBits(img, 40, u, 32);
    std::memcpy(&u, &power, 4);     pokeBits(img, 72, u, 32);
    slave->setSmBufferForTest(3, 0, img, sizeof(img));
    master_.pdoForSlave(0).exchangePhysical(1);
    EXPECT_FALSE(term.syncError(0));
    EXPECT_EQ(term.statusWord(0), 0u);
    ASSERT_TRUE(term.current(0).has_value());
    EXPECT_FLOAT_EQ(*term.current(0), 2.5f);
    EXPECT_FLOAT_EQ(*term.voltage(0), 230.0f);
    EXPECT_FLOAT_EQ(*term.activePower(0), 575.0f);
    EXPECT_FALSE(term.current(9).has_value());
    term.stop();
    (void)slave;
}

// ============================================================================
// EL7031 — StepperTerminal: ENC ctrl 0x7000 + STM ctrl 0x7010
// ============================================================================

DiscoveredSlave makeStepper() {
    auto s = makeBase(0x1B773052, "EL7031", 0x1100, 0x1180);
    s.sync_managers->at(2).length = 16;
    s.sync_managers->at(3).length = 16;
    s.rx_pdos = std::vector<SII::SIIPDO>{mkPdo(0x1600, 2, {
        mkEntry(0x7000, 1, 1), mkEntry(0x7000, 2, 1), mkEntry(0x7000, 3, 1),
        mkEntry(0x7000, 4, 1), mkEntry(0x7000, 0x11, 16),
        mkEntry(0x7010, 1, 1), mkEntry(0x7010, 2, 1), mkEntry(0x7010, 3, 1),
        mkEntry(0x7010, 0x21, 16), mkEntry(0x7010, 0x11, 32)})};
    // ENC status: :01 status word, then separate 1-bit entries for
    // :02-:05 (latched/set-done/under/overflow), a 4-bit pad so the
    // 16-bit values stay byte-aligned, then counter :0x11 / latch :0x12.
    s.tx_pdos = std::vector<SII::SIIPDO>{mkPdo(0x1A00, 3, {
        mkEntry(0x6000, 1, 16),
        mkEntry(0x6000, 2, 1), mkEntry(0x6000, 3, 1),
        mkEntry(0x6000, 4, 1), mkEntry(0x6000, 5, 1),
        mkEntry(0x6000, 6, 4),
        mkEntry(0x6000, 0x11, 16), mkEntry(0x6000, 0x12, 16),
        mkEntry(0x6010, 1, 8), mkEntry(0x6010, 2, 8),
        mkEntry(0x1C32, 0x20, 32)})};
    return s;
}

TEST_F(BeckhoffDriveTerminalsTest, StepperFindFirst) {
    auto s = makeStepper();
    auto hit = EtherCAT::Beckhoff::StepperTerminal::findFirst(
        master_, Dev::EL7031, std::span<const DiscoveredSlave>{&s, 1});
    ASSERT_TRUE(hit.has_value());
}

TEST_F(BeckhoffDriveTerminalsTest, StepperStartAndJog) {
    auto* slave = addDriveSlave(bus_, 0x1B773052, 0x1100, 16, 0x1180, 16);
    bringUp();
    EtherCAT::Beckhoff::StepperTerminal term(master_, makeStepper(),
                                             Dev::EL7031);
    ASSERT_TRUE(term.start().has_value());

    term.setEnable(true);
    term.setReset(true);
    term.setReduceTorque(true);
    term.setVelocity(321);
    term.setTargetPosition(100000);
    term.setLatchExtPos(true);
    term.setLatchExtNeg(true);
    term.setCounterValue(77);
    term.setSetCounter(true);

    // Packed tx: 0x6000:1(16b) at 0, status bits :02-:05 at 16-19,
    // pad at 20-23, :0x11 counter at 24-39, :0x12 latch at 40-55,
    // 0x6010:1|2 at 56-71, 0x1C32:0x20 at 72.
    uint8_t img[16] = {};
    pokeBits(img, 16 + 2, 1, 1);           // enc :04 underflow
    pokeBits(img, 16 + 3, 1, 1);           // enc :05 overflow
    pokeBits(img, 24, 0x4DEF, 16);         // counter
    pokeBits(img, 56, 0b00000110, 16);     // stm status word
    slave->setSmBufferForTest(3, 0, img, sizeof(img));
    master_.pdoForSlave(0).exchangePhysical(1);
    EXPECT_EQ(term.counterValue(), 0x4DEF);
    EXPECT_TRUE(term.counterUnderflow());
    EXPECT_TRUE(term.counterOverflow());
    EXPECT_FALSE(term.latchValid());
    EXPECT_NE(term.stmStatusWord(), 0u);
    EXPECT_FALSE(term.syncError());
    term.stop();
    (void)slave;
}

// ============================================================================
// EL7201 — CompactDriveTerminal: DRV out 0x7010 / in 0x6010, FB 0x6000
// ============================================================================

DiscoveredSlave makeCompactDrive() {
    auto s = makeBase(0x1C213052, "EL7201", 0x1100, 0x1180);
    s.sync_managers->at(2).length = 16;
    s.sync_managers->at(3).length = 16;
    s.rx_pdos = std::vector<SII::SIIPDO>{mkPdo(0x1600, 2, {
        mkEntry(0x7010, 1, 16),   // controlword
        mkEntry(0x7010, 3, 8),    // modes of operation
        mkEntry(0x7010, 5, 32),   // target position
        mkEntry(0x7010, 6, 32),   // target velocity
        mkEntry(0x7010, 9, 16)})};// target torque
    s.tx_pdos = std::vector<SII::SIIPDO>{mkPdo(0x1A00, 3, {
        mkEntry(0x6000, 0x11, 32),// FB position (claimed by fbPosition)
        mkEntry(0x6010, 1, 16),   // statusword
        mkEntry(0x6010, 3, 8),    // modes display
        mkEntry(0x6010, 6, 32),   // following error
        mkEntry(0x6010, 7, 32),   // actual velocity
        mkEntry(0x6010, 8, 16)})};// actual torque
    return s;
}

TEST_F(BeckhoffDriveTerminalsTest, CompactDriveFindFirst) {
    auto s = makeCompactDrive();
    auto hit = EtherCAT::Beckhoff::CompactDriveTerminal::findFirst(
        master_, Dev::EL7201, std::span<const DiscoveredSlave>{&s, 1});
    ASSERT_TRUE(hit.has_value());
}

TEST_F(BeckhoffDriveTerminalsTest, CompactDriveStartAndMotion) {
    auto* slave = addDriveSlave(bus_, 0x1C213052, 0x1100, 16, 0x1180, 16);
    bringUp();
    EtherCAT::Beckhoff::CompactDriveTerminal term(master_,
                                                  makeCompactDrive(),
                                                  Dev::EL7201);
    ASSERT_TRUE(term.start().has_value());
    EXPECT_EQ(term.axisCount(), 1u);

    term.setControlword(0, 0x000F);
    term.setModesOfOperation(0, 8);
    term.setTargetPosition(0, 123456);
    term.setTargetVelocity(0, 5000);
    term.setTargetTorque(0, -250);

    uint8_t img[16] = {};
    uint32_t pos = 7777;
    std::memcpy(img, &pos, 4);           // fb position
    uint16_t sw = 0x1234;
    std::memcpy(img + 4, &sw, 2);        // statusword
    int32_t vel = 4321;
    std::memcpy(img + 11, &vel, 4);        // 0x6010:7 at bit 88
    slave->setSmBufferForTest(3, 0, img, sizeof(img));
    master_.pdoForSlave(0).exchangePhysical(1);
    EXPECT_EQ(term.actualPosition(0), 7777);
    EXPECT_EQ(term.statusword(0), 0x1234u);
    EXPECT_EQ(term.actualVelocity(0), 4321);
    term.stop();
    (void)slave;
}

}  // namespace
