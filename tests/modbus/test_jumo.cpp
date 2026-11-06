/**
 * @file test_jumo.cpp
 * @brief JUMO Imago 500 driver: register map integrity, JUMO wire
 *        encodings (datasheet examples), typed device API over SimBus,
 *        BoundDevice/profile exposure, access-mode enforcement.
 */
#include <cstring>
#include <set>

#include <gtest/gtest.h>

#include "tether/modbus/drivers/JUMO/Imago500.hpp"
#include "tether/modbus/profile/Profile.hpp"
#include "tether/modbus/sim/Sim.hpp"

using namespace tether::modbus;
using namespace tether::modbus::jumo::imago500;

namespace {

struct JumoRig {
    JumoRig() {
        slave.store().declare(TableId::InputRegister,  0x0000, 0x1000);
        slave.store().declare(TableId::HoldingRegister,0x0000, 0x1000);
    }
    SimSlave    slave{7};
    SimInvoker  inv{slave};
    ModbusClient cli{inv, 7};
    JumoImago500 dev{cli, 7};
    void ir(uint16_t a, uint16_t v) {
        (void)slave.store().setReg(TableId::InputRegister, a, v);
    }
    void hr(uint16_t a, uint16_t v) {
        (void)slave.store().setReg(TableId::HoldingRegister, a, v);
    }
    uint16_t hr(uint16_t a) {
        return slave.store().getReg(TableId::HoldingRegister, a)
            .value_or(0xFFFF);
    }
    /// Write an IEEE-754 float into two regs in JUMO order (low word
    /// first) at `a` — matches the datasheet wire layout.
    void irF(uint16_t a, float f) {
        uint32_t u; std::memcpy(&u, &f, 4);
        ir(a, uint16_t(u & 0xFFFF)); ir(a + 1, uint16_t(u >> 16));
    }
    void hrF(uint16_t a, float f) {
        uint32_t u; std::memcpy(&u, &f, 4);
        hr(a, uint16_t(u & 0xFFFF)); hr(a + 1, uint16_t(u >> 16));
    }
};

// ---- §4 wire encodings ------------------------------------------------------
TEST(JumoCodec, FloatLowWordFirst) {
    // Datasheet example: 25.0 arrives as words "0000 41C8".
    const uint16_t regs[2] = {0x0000, 0x41C8};
    EXPECT_DOUBLE_EQ(decodeScalar(regs, kFloat), 25.0);
    // And 3000.0 -> PC bytes 00 80 3B 45 -> MODbus 80 00 45 3B.
    const uint16_t r2[2] = {0x8000, 0x453B};
    EXPECT_FLOAT_EQ(float(decodeScalar(r2, kFloat)), 3000.0f);
    auto enc = encodeScalar(3000.0, kFloat);
    ASSERT_TRUE(enc);
    EXPECT_EQ((*enc)[0], 0x8000u);
    EXPECT_EQ((*enc)[1], 0x453Bu);
}

TEST(JumoCodec, LongHighWordFirst) {
    // Datasheet: 66051 = 0x00010203 -> MODbus bytes "00 01 02 03".
    const uint16_t regs[2] = {0x0001, 0x0203};
    EXPECT_DOUBLE_EQ(decodeScalar(regs, kLong), 66051.0);
    auto enc = encodeScalar(66051.0, kLong);
    ASSERT_TRUE(enc);
    EXPECT_EQ((*enc)[0], 0x0001u);
    EXPECT_EQ((*enc)[1], 0x0203u);
}

// ---- map integrity ------------------------------------------------------------
TEST(JumoMap, CompleteAndGrouped) {
    const auto regs = allRegisters();
    EXPECT_GT(regs.size(), 190u);
    std::set<std::string> names;
    std::set<std::pair<int,int>> addrs;
    for (const auto& r : regs) {
        EXPECT_TRUE(names.insert(r.name).second)
            << "dup name " << r.name;
        EXPECT_TRUE(addrs.insert({int(r.addr.table), r.addr.offset}).second)
            << "dup addr " << r.name;
        EXPECT_FALSE(r.group.empty()) << r.name;
    }
    // Every group used.
    for (const auto g : {grp::status, grp::process, grp::program,
                         grp::setpoints, grp::manual, grp::params,
                         grp::prog_start, grp::prog_tx, grp::commands,
                         grp::ram, grp::ident, grp::clock}) {
        bool seen = false;
        for (const auto& r : regs) if (r.group == g) seen = true;
        EXPECT_TRUE(seen) << "group unused: " << g;
    }
}

TEST(JumoMap, DatasheetAddresses) {
    EXPECT_EQ(addr::genStatus,    0x008B);
    EXPECT_EQ(addr::chBlockBase,  0x00C8);
    EXPECT_EQ(addr::setpointBase, 0x083C);
    EXPECT_EQ(addr::manualOutBase,0x016A);
    EXPECT_EQ(addr::paramsBase,   0x085C);
    EXPECT_EQ(addr::progBuffer,   0x01B4);
    EXPECT_EQ(addr::manBuffer,    0x01C1);
    EXPECT_EQ(addr::ptxActivate,  0x01D1);
    EXPECT_EQ(addr::cmdProgram,   0x0172);
    EXPECT_EQ(addr::ramSetpointBase, 0x015A);
    EXPECT_EQ(addr::ramPvBase,    0x015C);
    EXPECT_EQ(addr::identOutBoard,0x019F);
    EXPECT_EQ(addr::clockBase,    0x007E);
    // Param strides (datasheet §5.4).
    EXPECT_EQ(JumoImago500::paramAddr(1,1,JumoImago500::Param::XP1), 0x085E);
    EXPECT_EQ(JumoImago500::paramAddr(1,2,JumoImago500::Param::XP1), 0x0882);
    EXPECT_EQ(JumoImago500::paramAddr(2,1,JumoImago500::Param::TN1), 0x08AE);
    EXPECT_EQ(JumoImago500::paramAddr(4,2,JumoImago500::Param::TK2), 0x097A);
    // Odd logic-output bit order (datasheet §5.1 0x008D).
    EXPECT_EQ(logicOutputBit(1), 0); EXPECT_EQ(logicOutputBit(7), 1);
    EXPECT_EQ(logicOutputBit(2), 2); EXPECT_EQ(logicOutputBit(12), 11);
}

// ---- device API over SimBus -----------------------------------------------------
TEST(JumoDevice, GeneratorStatusBits) {
    JumoRig rig;
    rig.ir(addr::genStatus, uint16_t((1<<8) | (1<<4) | (1<<2)));
    auto s = rig.dev.generatorStatus();
    ASSERT_TRUE(s);
    EXPECT_TRUE(s->automatic); EXPECT_TRUE(s->programEnd);
    EXPECT_TRUE(s->delay);     EXPECT_FALSE(s->manual);
    EXPECT_FALSE(s->standstill);
}

TEST(JumoDevice, ControllerStatusPerChannel) {
    JumoRig rig;
    // C4 manual (bit0) + C1 self-opt (bit13).
    rig.ir(addr::ctlStatus, uint16_t((1<<0) | (1<<13)));
    EXPECT_TRUE(rig.dev.ctlManualActive(4).value_or(false));
    EXPECT_TRUE(rig.dev.ctlSelfOptActive(1).value_or(false));
    EXPECT_FALSE(rig.dev.ctlManualActive(1).value_or(true));
    EXPECT_FALSE(rig.dev.ctlSelfOptActive(4).value_or(true));
}

TEST(JumoDevice, LogicOutputOddOrdering) {
    JumoRig rig;
    rig.ir(addr::logicOutputs, uint16_t(1 << 1));   // bit1 = output 7
    EXPECT_TRUE(rig.dev.logicOutput(7).value_or(false));
    EXPECT_FALSE(rig.dev.logicOutput(1).value_or(true));
    EXPECT_FALSE(rig.dev.logicOutput(2).value_or(true));
    EXPECT_FALSE(rig.dev.logicOutput(0).has_value());   // range check
    EXPECT_FALSE(rig.dev.logicOutput(13).has_value());
}

TEST(JumoDevice, ChannelPvBlock) {
    JumoRig rig;
    const uint16_t b = addr::chBlockBase;   // C1
    rig.irF(b + 0x02, 123.5f);              // pv_filtered
    rig.irF(b + 0x06, 200.0f);              // setpoint
    rig.irF(b + 0x08, 42.5f);               // output %
    rig.irF(b + 0x0A, 55.0f);               // out heating
    rig.ir(b + 0x12, 1);                    // sw heating ON
    auto pv = rig.dev.channelPv(1);
    ASSERT_TRUE(pv);
    EXPECT_FLOAT_EQ(float(pv->pvFiltered), 123.5f);
    EXPECT_FLOAT_EQ(float(pv->setpoint), 200.0f);
    EXPECT_FLOAT_EQ(float(pv->output), 42.5f);
    EXPECT_FLOAT_EQ(float(pv->outHeating), 55.0f);
    EXPECT_TRUE(pv->heatingOn);
    EXPECT_FALSE(pv->coolingOn);
    // Scalar accessors agree.
    EXPECT_FLOAT_EQ(float(rig.dev.pvFiltered(1).value_or(-1)), 123.5f);
    EXPECT_TRUE(rig.dev.heatingOn(1).value_or(false));
}

TEST(JumoDevice, ChannelBlocksStrided) {
    JumoRig rig;
    // C4 block: 0x00C8 + 3*0x14 = 0x0104.
    rig.irF(addr::chBlockBase + 3*0x14 + 0x02, 77.25f);
    EXPECT_FLOAT_EQ(float(rig.dev.pvFiltered(4).value_or(-1)), 77.25f);
    EXPECT_FALSE(rig.dev.pvFiltered(0).has_value());
    EXPECT_FALSE(rig.dev.pvFiltered(5).has_value());
}

TEST(JumoDevice, ProgramStateLongs) {
    JumoRig rig;
    // LONG = high word first: 3600 s = 0x00000E10.
    rig.ir(addr::progRunTime, 0x0000); rig.ir(addr::progRunTime + 1, 0x0E10);
    EXPECT_EQ(rig.dev.programRunTimeS().value_or(-1), 3600);
    // PCh3 segment time: 0x0146 + 6*2 = 0x0152.
    rig.ir(0x0152, 0x0000); rig.ir(0x0153, 600);
    EXPECT_EQ(rig.dev.segmentTimeS(3).value_or(-1), 600);
}

TEST(JumoDevice, EepromVsRamSetpoints) {
    JumoRig rig;
    ASSERT_TRUE(rig.dev.setSetpoint(2, 3, 88.5));       // EEPROM W3
    // C2 W3 = 0x083C + 0x08 + 4 = 0x0848, low word first.
    const uint16_t w[2] = {rig.hr(0x0848), rig.hr(0x0849)};
    EXPECT_FLOAT_EQ(float(decodeScalar(w, kFloat)), 88.5f);
    // RAM setpoint C3 = 0x0162 (volatile — EEPROM-safe path).
    ASSERT_TRUE(rig.dev.setSetpointRam(3, 55.0));
    const uint16_t rw[2] = {rig.hr(0x0162), rig.hr(0x0163)};
    EXPECT_FLOAT_EQ(float(decodeScalar(rw, kFloat)), 55.0f);
    // Read-back paths.
    rig.hrF(0x015A, 11.0f);
    EXPECT_FLOAT_EQ(float(rig.dev.setpointRam(1).value_or(-1)), 11.0f);
}

TEST(JumoDevice, ManualOutputAndParams) {
    JumoRig rig;
    ASSERT_TRUE(rig.dev.setManualOutput(1, 33.0));
    const uint16_t w[2] = {rig.hr(0x016A), rig.hr(0x016B)};
    EXPECT_FLOAT_EQ(float(decodeScalar(w, kFloat)), 33.0f);
    // Param: C2 set2 TN1 = 0x08A4+0x24+10 = 0x08D2.
    ASSERT_TRUE(rig.dev.setParam(2, 2, JumoImago500::Param::TN1, 120.0));
    const uint16_t pw[2] = {rig.hr(0x08D2), rig.hr(0x08D3)};
    EXPECT_FLOAT_EQ(float(decodeScalar(pw, kFloat)), 120.0f);
    // INT param (struct words) round-trip.
    ASSERT_TRUE(rig.dev.setParam(1, 1, JumoImago500::Param::Struct1, 3));
    EXPECT_EQ(rig.hr(0x085C), 3u);
}

TEST(JumoDevice, CommandBits) {
    JumoRig rig;
    ASSERT_TRUE(rig.dev.programPause());
    EXPECT_EQ(rig.hr(addr::cmdProgram), uint16_t(1 << 13));
    ASSERT_TRUE(rig.dev.ctlAutomatic(1));
    EXPECT_EQ(rig.hr(addr::cmdCtlBase), uint16_t(1 << 8));
    ASSERT_TRUE(rig.dev.ctlStartSelfOpt(4));
    EXPECT_EQ(rig.hr(addr::cmdCtlBase + 3), uint16_t(1 << 7));
    ASSERT_TRUE(rig.dev.keyInhibit());
    EXPECT_EQ(rig.hr(addr::cmdOperation), uint16_t(1 << 15));
    ASSERT_TRUE(rig.dev.selectSetpoint(2, 3));
    EXPECT_EQ(rig.hr(addr::chSpChangeBase + 3), 3u);
    ASSERT_TRUE(rig.dev.selectProgram(-1));
    EXPECT_EQ(rig.hr(addr::programSelect), 0xFFFFu);
    // Relay outputs: mask + activation bit15.
    ASSERT_TRUE(rig.dev.setRelayOutputs((1<<0)|(1<<2), true));
    EXPECT_EQ(rig.hr(addr::relayOutputs),
              uint16_t((1<<0)|(1<<2)|(1<<15)));
}

TEST(JumoDevice, ProgramStartAndManualMode) {
    JumoRig rig;
    JumoImago500::ProgramStart ps;
    ps.programNo = 5; ps.delayS = 30; ps.secs = -1;
    ps.startSegment = 2; ps.startSegRemainS = 90;
    ASSERT_TRUE(rig.dev.startProgram(ps));
    EXPECT_EQ(rig.hr(addr::progStartNo), 5u);
    EXPECT_EQ(rig.hr(addr::progStartDelay), 0u);     // LONG hi
    EXPECT_EQ(rig.hr(addr::progStartDelay+1), 30u);  // LONG lo
    EXPECT_EQ(rig.hr(addr::progTimeBase), 0xFFFFu);  // -1 instant
    EXPECT_EQ(rig.hr(addr::progStartSeg), 2u);
    EXPECT_EQ(rig.hr(addr::progBuffer), uint16_t(1 << 8));
    // Manual mode.
    JumoImago500::ManualModeData m;
    m.setpoint[0] = 150.0; m.contacts = 0x05;
    m.paramSet[1] = 1; m.ctlFuncControllers = 0x03;
    ASSERT_TRUE(rig.dev.setManualMode(m));
    const uint16_t sp[2] = {rig.hr(addr::manSetpointBase),
                            rig.hr(addr::manSetpointBase + 1)};
    EXPECT_FLOAT_EQ(float(decodeScalar(sp, kFloat)), 150.0f);
    EXPECT_EQ(rig.hr(addr::manContacts), 5u);
    EXPECT_EQ(rig.hr(addr::manParamSetBase + 1), 1u);
    EXPECT_EQ(rig.hr(addr::manCtlFuncCtl), 3u);
    EXPECT_EQ(rig.hr(addr::manBuffer), uint16_t(1 << 9));
}

TEST(JumoDevice, ProgramTransmission) {
    JumoRig rig;
    JumoImago500::SegmentData s;
    s.program = 3; s.channel = 2; s.segment = 7;
    s.setpoint = 250.0; s.tolMin = -1.0; s.tolMax = 2.0;
    s.timeS = 300; s.repeats = -1; s.targetSegment = 1; s.paramSet = 1;
    ASSERT_TRUE(rig.dev.transmitSegment(s));
    EXPECT_EQ(rig.hr(addr::ptxFunction),
              uint16_t(ProgTxFn::TransmitSegment));
    EXPECT_EQ(rig.hr(addr::ptxProgram), 3u);
    EXPECT_EQ(rig.hr(addr::ptxSegment), 7u);
    EXPECT_EQ(rig.hr(addr::ptxRepeats), 0xFFFFu);
    EXPECT_EQ(rig.hr(addr::ptxActivate), 1u);
    const uint16_t w[2] = {rig.hr(addr::ptxSetpoint),
                           rig.hr(addr::ptxSetpoint + 1)};
    EXPECT_FLOAT_EQ(float(decodeScalar(w, kFloat)), 250.0f);
    const uint16_t tw[2] = {rig.hr(addr::ptxSegTime),
                            rig.hr(addr::ptxSegTime + 1)};
    EXPECT_DOUBLE_EQ(decodeScalar(tw, kLong), 300.0);
    ASSERT_TRUE(rig.dev.progTxRun(ProgTxFn::DeleteProgram, 3));
    EXPECT_EQ(rig.hr(addr::ptxFunction), uint16_t(ProgTxFn::DeleteProgram));
}

TEST(JumoDevice, RamProcessValueAndClock) {
    JumoRig rig;
    ASSERT_TRUE(rig.dev.setProcessValueRam(2, 66.6));
    const uint16_t w[2] = {rig.hr(0x0160), rig.hr(0x0161)};
    EXPECT_FLOAT_EQ(float(decodeScalar(w, kFloat)), 66.6f);
    ASSERT_TRUE(rig.dev.setClock(24, 6, 15, 10, 30, 0));
    EXPECT_EQ(rig.hr(addr::clockBase + 0), 24u);
    EXPECT_EQ(rig.hr(addr::clockBase + 5), 0u);
    EXPECT_EQ(rig.hr(addr::clockSetCmd), 1u);
}

TEST(JumoDevice, DeviceIdentBlock) {
    JumoRig rig;
    for (int i = 0; i < 6; ++i) rig.ir(addr::identOutBoard + i, 10 + i);
    rig.ir(addr::identInterface, 7);
    rig.ir(addr::identEr8, 1);
    auto id = rig.dev.deviceIdent();
    ASSERT_TRUE(id);
    EXPECT_EQ(id->outBoard[0], 10); EXPECT_EQ(id->outBoard[5], 15);
    EXPECT_EQ(id->interface, 7);    EXPECT_EQ(id->er8, 1);
}

// ---- profile / BoundDevice exposure ------------------------------------------
TEST(JumoProfile, BoundDeviceNameAccess) {
    JumoRig rig;
    const auto prof = imago500Profile(7);
    BoundDevice bd(rig.cli, prof);
    rig.irF(addr::chBlockBase + 0x02, 99.5f);
    auto v = bd.read("c1.pv_filtered");
    ASSERT_TRUE(v);
    EXPECT_FLOAT_EQ(float(*v), 99.5f);
    // Named write lands at the RAM-setpoint address (EEPROM-safe).
    ASSERT_TRUE(bd.write("c3.ram_setpoint", 42.0));
    const uint16_t w[2] = {rig.hr(0x0162), rig.hr(0x0163)};
    EXPECT_FLOAT_EQ(float(decodeScalar(w, kFloat)), 42.0f);
}

TEST(JumoProfile, ReadOnlyEnforced) {
    JumoRig rig;
    const auto prof = imago500Profile(7);
    BoundDevice bd(rig.cli, prof);
    const auto* d = prof.find("c1.pv_filtered");
    ASSERT_NE(d, nullptr);
    EXPECT_FALSE(d->writable);
    // TypedRef write on an InputRegister decl must not go out.
    EXPECT_FALSE(bd.write("c1.pv_filtered", 1.0).has_value());
}

TEST(JumoProfile, PollablesAndGroups) {
    const auto prof = imago500Profile(1);
    int polled = 0, grouped = 0;
    for (const auto& r : prof.registers) {
        if (r.pollMs > 0) ++polled;
        if (!r.group.empty()) ++grouped;
    }
    EXPECT_GT(polled, 30);
    EXPECT_EQ(grouped, int(prof.registers.size()));
}

TEST(JumoProfile, RegisteredInBuiltins) {
    ProfileRegistry reg;
    reg.registerBuiltins();
    const auto* p = reg.find("jumo/imago500");
    ASSERT_NE(p, nullptr);
    EXPECT_EQ(p->vendor, "JUMO");
    EXPECT_EQ(p->quirks.maxReadQty, 127u);
    EXPECT_GT(p->registers.size(), 190u);
}

} // namespace
