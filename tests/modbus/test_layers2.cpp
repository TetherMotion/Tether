// Tests for: AddressNotation, BlockPlan, Profile/BoundDevice, INI
// config, ModbusManager, Gateway.
#include <gtest/gtest.h>

#include <cstdio>
#include <fstream>
#include <iterator>
#include <vector>

#include "tether/modbus/client/WriteCache.hpp"
#include "tether/modbus/diag/Trace.hpp"
#include "tether/modbus/config/Config.hpp"
#include "tether/modbus/server/Gateway.hpp"
#include "tether/modbus/sim/Sim.hpp"
#include "tether/modbus/typed/AddressNotation.hpp"
#include "tether/modbus/typed/TypedRef.hpp"

using namespace tether::modbus;

TEST(AddressNotation, ParseForms) {
    auto a = parseAddress("40001");
    ASSERT_TRUE(a.has_value());
    EXPECT_EQ(a->table, TableId::HoldingRegister);
    EXPECT_EQ(a->offset, 0);
    a = parseAddress("30005.2");
    ASSERT_TRUE(a.has_value());
    EXPECT_EQ(a->table, TableId::InputRegister);
    EXPECT_EQ(a->offset, 4);
    EXPECT_EQ(a->bit, 2);
    a = parseAddress("hr:17");
    EXPECT_EQ(a->offset, 17);
    a = parseAddress("4x9");
    EXPECT_EQ(a->table, TableId::HoldingRegister);
    EXPECT_EQ(a->offset, 9);
    a = parseAddress("coil:3");
    EXPECT_EQ(a->table, TableId::Coil);
    a = parseAddress("0x20");
    EXPECT_EQ(a->offset, 0x20);
    EXPECT_FALSE(parseAddress("garbage").has_value());
    EXPECT_FALSE(parseAddress("40000").has_value());
    EXPECT_EQ(formatPlcAddress(Address{TableId::HoldingRegister, 5}),
              "40006");
}

TEST(BlockPlan, Coalesces) {
    BlockPlan bp(4, 125);
    bp.add(TableId::HoldingRegister, 0, 2);
    bp.add(TableId::HoldingRegister, 2, 2);   // contiguous
    bp.add(TableId::HoldingRegister, 6, 1);   // gap=2 ≤ 4 → merge
    bp.add(TableId::HoldingRegister, 50, 1);  // far → new
    bp.add(TableId::InputRegister, 0, 4);     // other table
    auto bl = bp.blocks();
    ASSERT_EQ(bl.size(), 3u);
    EXPECT_EQ(bl[0].table, TableId::InputRegister);  // 3 < 4 sorts first
    EXPECT_EQ(bl[0].offset, 0);
    EXPECT_EQ(bl[0].qty, 4);
    EXPECT_EQ(bl[1].offset, 0);
    EXPECT_EQ(bl[1].qty, 7);    // merged [0..6]
    EXPECT_EQ(bl[1].table, TableId::HoldingRegister);
    EXPECT_EQ(bl[2].offset, 50);
}

TEST(Profile, BoundDeviceReadWrite) {
    ProfileRegistry reg;
    reg.registerBuiltins();
    const auto* prof = reg.find("generic/vfd");
    ASSERT_NE(prof, nullptr);

    SimSlave s(1);
    (void)s.store().declare(TableId::HoldingRegister, 0, 10);
    (void)s.store().declare(TableId::InputRegister, 0, 10);
    (void)s.store().declare(TableId::Coil, 0, 10);
    (void)s.store().setReg(TableId::InputRegister, 1, 5000); // 50.00 Hz
    SimInvoker inv(s);
    ModbusClient cli(inv, 1);
    BoundDevice dev(cli, *prof);

    auto f = dev.read("freq_act");
    ASSERT_TRUE(f.has_value());
    EXPECT_NEAR(*f, 50.0, 0.01);
    ASSERT_TRUE(dev.write("freq_set", 60.0).has_value());
    EXPECT_EQ(*s.store().getReg(TableId::HoldingRegister, 1), 6000);
    EXPECT_FALSE(dev.write("freq_set", 9999.0).has_value()); // > max
    EXPECT_FALSE(dev.write("status", 1.0).has_value());      // read-only
    EXPECT_FALSE(dev.read("nonexistent").has_value());
}

TEST(Config, IniParse) {
    const char* text = R"(
# comment
[modbus.press]
mode = rtu
device = /dev/ttyUSB0:19200-8E1
unit = 3
timeout_ms = 750
retries = 2
profile = generic/vfd
poll = freq_act:100, current:200

[modbus.plc]
mode = tcp
host = 192.168.1.20
port = 502
)";
    auto ini = parseIni(text);
    ASSERT_TRUE(ini.has_value());
    auto cfg = ModbusConfig::fromIni(*ini);
    ASSERT_TRUE(cfg.has_value());
    ASSERT_EQ(cfg->buses.size(), 2u);
    const auto& b0 = cfg->buses[0];
    EXPECT_EQ(b0.name, "plc");            // map sorted: plc < press
    const BusConfig* press = nullptr;
    for (auto& b : cfg->buses) if (b.name == "press") press = &b;
    ASSERT_NE(press, nullptr);
    EXPECT_EQ(press->serial.baud, 19200u);
    EXPECT_EQ(press->serial.parity, Parity::Even);
    EXPECT_EQ(press->unit, 3);
    EXPECT_EQ(press->tx.timeoutMs, 750);
    EXPECT_EQ(press->tx.retries, 2);
    EXPECT_EQ(press->profileId, "generic/vfd");
    ASSERT_EQ(press->polls.size(), 2u);
    EXPECT_EQ(press->polls[0].second, 100);
}

TEST(Config, BadIni) {
    EXPECT_FALSE(parseIni("no_section_key = x").has_value());
    EXPECT_FALSE(parseIni("[s]\nbadline").has_value());
}

TEST(Gateway, ForwardsAndSerializes) {
    // downstream: sim slave on an invoker
    SimSlave s(5);
    (void)s.store().declare(TableId::HoldingRegister, 0, 10);
    s.preload(0, {111});
    SimInvoker downstream(s, false);    // don't check unit (gateway maps)

    // gateway "forward" path (no sockets needed)
    Gateway gw(nullptr, downstream);
    auto req = build::readRange(FunctionCode::ReadHoldingRegisters, 0, 1);
    auto rsp = gw.forward(5, req->bytes());
    ASSERT_EQ(rsp.size(), 4u);
    EXPECT_EQ(rsp[0], 0x03);
    EXPECT_EQ((rsp[2] << 8) | rsp[3], 111);

    // downstream exception → relayed
    auto req2 = build::readRange(FunctionCode::ReadHoldingRegisters,
                                 99, 1);
    auto rsp2 = gw.forward(5, req2->bytes());
    ASSERT_EQ(rsp2.size(), 2u);
    EXPECT_EQ(rsp2[0], 0x83);
    EXPECT_EQ(rsp2[1], 0x02);
}

TEST(WriteCache, DirtyCoalesceAndSeed) {
    SimSlave s(1);
    (void)s.store().declare(TableId::HoldingRegister, 0, 20);
    (void)s.store().declare(TableId::Coil, 0, 20);
    s.preload(0, {100, 200, 300, 400});
    SimInvoker inv(s);
    ModbusClient cli(inv, 1);
    WriteCache wc(cli, /*maxGap=*/2);

    // seed known state — equal values are not dirty
    wc.seed(0, 100); wc.seed(1, 200); wc.seed(2, 300); wc.seed(3, 400);
    wc.set(0, 100);              // unchanged → not dirty
    wc.set(1, 250);              // dirty
    wc.set(3, 999);              // dirty, gap at 2 (known 300, fillable)
    wc.set(10, 7);               // far, unknown gap → separate run
    EXPECT_EQ(wc.dirtyRegs(), 3u);

    auto runs = wc.dirtyRuns();
    ASSERT_EQ(runs.size(), 2u);
    EXPECT_EQ(runs[0].first, 1);
    ASSERT_EQ(runs[0].second.size(), 3u);   // [1,2,3] w/ known fill @2
    EXPECT_EQ(runs[0].second[1], 300);
    EXPECT_EQ(runs[1].first, 10);

    ASSERT_TRUE(wc.flush().has_value());
    EXPECT_EQ(*s.store().getReg(TableId::HoldingRegister, 1), 250);
    EXPECT_EQ(*s.store().getReg(TableId::HoldingRegister, 2), 300); // fill
    EXPECT_EQ(*s.store().getReg(TableId::HoldingRegister, 3), 999);
    EXPECT_EQ(*s.store().getReg(TableId::HoldingRegister, 10), 7);
    EXPECT_EQ(wc.dirtyRegs(), 0u);

    // coils: dirty bit + known-fill run
    wc.seedCoil(0, false); wc.seedCoil(1, false);
    wc.setCoil(0, true); wc.setCoil(1, false);
    EXPECT_EQ(wc.dirtyCoils(), 1u);
    ASSERT_TRUE(wc.flush().has_value());
    EXPECT_EQ(*s.store().getBit(TableId::Coil, 0), true);
}

TEST(Poller, EpsilonDebounce) {
    SimSlave s(1);
    (void)s.store().declare(TableId::HoldingRegister, 0, 4);
    SimInvoker inv(s);
    ModbusClient cli(inv, 1);
    Poller poller(cli);

    int fires = 0;
    PollItem it;
    it.fc = FunctionCode::ReadHoldingRegisters;
    it.addr = 0; it.qty = 1;
    it.changedOnly = true;
    it.epsilon = 10;             // ignore wiggles ≤ 10
    it.debounceN = 1;            // need 2 consecutive differing polls
    it.onResult = [&](const PollItem&, const std::vector<uint16_t>&,
                      const ModbusError*) { ++fires; };
    poller.add(std::move(it));

    auto poll = [&] { poller.kick(); poller.runOnce(); };
    poll();                      // first poll → fires (baseline)
    EXPECT_EQ(fires, 1);

    (void)s.store().setReg(TableId::HoldingRegister, 0, 5);  // ≤ε
    poll();
    EXPECT_EQ(fires, 1);         // within deadband → silent

    (void)s.store().setReg(TableId::HoldingRegister, 0, 50); // >ε
    poll();                      // differs (streak 1 ≤ debounce 1)
    EXPECT_EQ(fires, 1);
    poll();                      // still differs (streak 2 > 1) → fire
    EXPECT_EQ(fires, 2);

    poll();                      // back to steady → silent
    EXPECT_EQ(fires, 2);
}

#ifdef TETHER_MODBUS_IO_BRIDGE
#include "tether/io/Registry.hpp"
#include "tether/modbus/bridge/IoBridge.hpp"

TEST(IoBridge, ExposesParamsAndSignals) {
    DeviceProfile prof;
    prof.id = "test/dev";
    prof.defaultUnit = 1;
    prof.registers.push_back({"setpoint",
        Address{TableId::HoldingRegister, 0},
        ScalarSpec{ValueType::U16, WordOrder::BigEndian, 0.1, 0.0},
        0, true, 0.0, 100.0, "kW", "cfg", "power setpoint"});
    prof.registers.push_back({"running",
        Address{TableId::Coil, 1},
        ScalarSpec{ValueType::Bit, WordOrder::BigEndian, 1.0, 0.0},
        0, false, 0.0, 0.0, "", "st", "run state"});

    SimSlave s(1);
    (void)s.store().declare(TableId::HoldingRegister, 0, 4);
    (void)s.store().declare(TableId::Coil, 0, 8);
    s.preload(0, {420});
    (void)s.store().setBit(TableId::Coil, 1, true);
    SimInvoker inv(s);
    ModbusClient cli(inv, 1);
    BoundDevice dev(cli, prof);
    IoBridge bridge(dev, "vfd");

    tether::io::Registry reg;
    bridge.expose(reg, tether::io::ModuleId::User);
    EXPECT_EQ(reg.paramCount(), 1u);
    EXPECT_EQ(reg.signalCount(), 1u);

    // Param read: 420 * 0.1 = 42.0
    auto pv = reg.paramPage(0, 10).front();
    EXPECT_EQ(pv.name(), "vfd.setpoint");
    EXPECT_TRUE(pv.writable());
    double d = 0;
    pv.read(&d);
    EXPECT_NEAR(d, 42.0, 0.01);

    // Param write through io → modbus store
    double w = 55.0;
    pv.write(&w);
    EXPECT_EQ(*s.store().getReg(TableId::HoldingRegister, 0), 550);

    // Signal read: Bool marshals as 1 byte
    auto sv = reg.signalPage(0, 10).front();
    EXPECT_EQ(sv.name(), "vfd.running");
    EXPECT_EQ(sv.valueSize(), 1u);
    uint8_t b = 0;
    sv.read(&b);
    EXPECT_EQ(b, 1u);
    EXPECT_FALSE(sv.writable());
}

TEST(IoBridge, PollerCache) {
    DeviceProfile prof;
    prof.id = "test/dev2";
    prof.registers.push_back({"temp",
        Address{TableId::InputRegister, 0},
        ScalarSpec{ValueType::U16, WordOrder::BigEndian, 1.0, 0.0},
        50, false, 0.0, 0.0, "°C", "", "temp"});

    SimSlave s(1);
    (void)s.store().declare(TableId::InputRegister, 0, 4);
    SimInvoker inv(s);
    ModbusClient cli(inv, 1);
    BoundDevice dev(cli, prof);
    Poller poller(cli);
    IoBridge bridge(dev, "dev", &poller);

    // Nothing published → live read hits device (0).
    tether::io::Registry reg;
    bridge.expose(reg, tether::io::ModuleId::User);
    auto sv = reg.signalPage(0, 10).front();
    double d = -1;
    sv.read(&d);
    EXPECT_EQ(d, 0.0);

    // Publish → cached path, no device hit.
    bridge.publish("temp", 77.0);
    EXPECT_TRUE(bridge.fresh("temp"));
    sv.read(&d);
    EXPECT_EQ(d, 77.0);
}

// ---- driver exposers + generic read/write functions ----------------------
#include "tether/modbus/bridge/ModbusIoExposer.hpp"
#include "tether/modbus/drivers/Drivers.hpp"

namespace {
tether::io::FunctionArgument argU8(uint8_t v) {
    tether::io::FunctionArgument a;
    a.value = {v};
    a.provided = true;
    return a;
}
tether::io::FunctionArgument argU16(uint16_t v) {
    tether::io::FunctionArgument a;
    a.value = {uint8_t(v), uint8_t(v >> 8)};
    a.provided = true;
    return a;
}
tether::io::FunctionArgument argBin(std::vector<uint8_t> b) {
    tether::io::FunctionArgument a;
    a.value = std::move(b);
    a.provided = true;
    return a;
}
}

TEST(IoDrivers, BusExposerGenericReadWrite) {
    SimSlave s(1);
    (void)s.store().declare(TableId::HoldingRegister, 0, 8);
    (void)s.store().declare(TableId::Coil, 0, 8);
    s.preload(0, {0x1234, 0xABCD});
    SimInvoker inv(s);
    ModbusClient cli(inv, 1);

    ModbusBusExposer xp(cli, "bus0");
    tether::io::Registry reg;
    xp.expose(reg, tether::io::ModuleId::User);
    ASSERT_EQ(reg.functionCount(), 2u);

    auto read = reg.findFunction(tether::io::makeId(
        tether::io::ModuleId::User, 0x01));
    auto write = reg.findFunction(tether::io::makeId(
        tether::io::ModuleId::User, 0x02));
    ASSERT_TRUE(read);
    ASSERT_TRUE(write);

    // read hr:0 qty2 → BE words
    auto r = read.invoke({argU8(4), argU16(0), argU16(2)});
    ASSERT_TRUE(r.success) << r.errorMessage;
    ASSERT_EQ(r.returnValue.size(), 4u);
    EXPECT_EQ(r.returnValue[0], 0x12);
    EXPECT_EQ(r.returnValue[1], 0x34);
    EXPECT_EQ(r.returnValue[2], 0xAB);
    EXPECT_EQ(r.returnValue[3], 0xCD);

    // write hr:4 = [0x00,0x2A] → 42
    auto w = write.invoke({argU8(4), argU16(4),
                           argBin({0x00, 0x2A})});
    ASSERT_TRUE(w.success) << w.errorMessage;
    EXPECT_EQ(*s.store().getReg(TableId::HoldingRegister, 4), 42);

    // write coils addr0 = packed bits 0b00000101 → coils 0,2 on
    w = write.invoke({argU8(0), argU16(0), argBin({0x05})});
    ASSERT_TRUE(w.success) << w.errorMessage;
    EXPECT_TRUE(*s.store().getBit(TableId::Coil, 0));
    EXPECT_FALSE(*s.store().getBit(TableId::Coil, 1));
    EXPECT_TRUE(*s.store().getBit(TableId::Coil, 2));

    // read coils → packed bits back
    r = read.invoke({argU8(0), argU16(0), argU16(8)});
    ASSERT_TRUE(r.success);
    ASSERT_EQ(r.returnValue.size(), 1u);
    EXPECT_EQ(r.returnValue[0], 0x05);

    // read-only table rejected; bad table rejected.
    w = write.invoke({argU8(3), argU16(0), argBin({0, 1})});
    EXPECT_FALSE(w.success);
    w = write.invoke({argU8(9), argU16(0), argBin({0, 1})});
    EXPECT_FALSE(w.success);
}

TEST(IoDrivers, VfdExposer) {
    ProfileRegistry pr;
    pr.registerBuiltins();
    const auto* prof = pr.find("generic/vfd");
    ASSERT_NE(prof, nullptr);

    SimSlave s(1);
    (void)s.store().declare(TableId::HoldingRegister, 0, 10);
    (void)s.store().declare(TableId::InputRegister, 0, 10);
    (void)s.store().declare(TableId::Coil, 0, 10);
    (void)s.store().setReg(TableId::InputRegister, 1, 5000); // 50 Hz
    SimInvoker inv(s);
    ModbusClient cli(inv, 1);
    BoundDevice dev(cli, *prof);
    ModbusVfd vfd(dev);

    ModbusVfdExposer xp(vfd, "vfd0");
    tether::io::Registry reg;
    xp.expose(reg, tether::io::ModuleId::User + 1);

    // signals
    double hz = 0;
    auto page = reg.signalPage(0, 10);
    ASSERT_GE(page.size(), 5u);
    for (auto& sv : page) {
        if (sv.name() == "vfd0.freq_act") sv.read(&hz);
    }
    EXPECT_NEAR(hz, 50.0, 0.01);

    // params: freq_set write → HR1 = 6000
    bool wrote = false;
    auto pp = reg.paramPage(0, 10);
    for (auto& pe : pp) {
        if (pe.name() == "vfd0.freq_set") {
            double v = 60.0;
            pe.write(&v);
            wrote = true;
        }
    }
    EXPECT_TRUE(wrote);
    EXPECT_EQ(*s.store().getReg(TableId::HoldingRegister, 1), 6000);

    // run param
    for (auto& pe : pp) {
        if (pe.name() == "vfd0.run") {
            uint8_t on = 1;
            pe.write(&on);
        }
    }
}

TEST(IoDrivers, RemoteIoExposer) {
    SimSlave s(1);
    (void)s.store().declare(TableId::DiscreteInput, 0, 4);
    (void)s.store().declare(TableId::Coil, 0, 4);
    (void)s.store().declare(TableId::InputRegister, 0, 4);
    (void)s.store().declare(TableId::HoldingRegister, 0, 4);
    (void)s.store().setBit(TableId::DiscreteInput, 1, true);
    (void)s.store().setReg(TableId::InputRegister, 0, 777);
    SimInvoker inv(s);
    ModbusClient cli(inv, 1);
    ModbusRemoteIo mod(cli, {.nDi = 2, .baseDi = 0,
                            .nDo = 2, .baseDo = 0,
                            .nAi = 2, .baseAi = 0,
                            .nAo = 2, .baseAo = 0});

    ModbusRemoteIoExposer xp(mod, "rio");
    tether::io::Registry reg;
    xp.expose(reg, tether::io::ModuleId::User + 2);

    // di1 signal
    uint8_t di1 = 0, di0 = 1;
    for (auto& sv : reg.signalPage(0, 10)) {
        if (sv.name() == "rio.di1") sv.read(&di1);
        if (sv.name() == "rio.di0") sv.read(&di0);
        if (sv.name() == "rio.ai0") {
            uint16_t a = 0;
            sv.read(&a);
            EXPECT_EQ(a, 777);
        }
    }
    EXPECT_EQ(di1, 1);
    EXPECT_EQ(di0, 0);

    // do1 param write → coil 1 on
    for (auto& pe : reg.paramPage(0, 10)) {
        if (pe.name() == "rio.do1") {
            uint8_t on = 1;
            pe.write(&on);
        }
        if (pe.name() == "rio.ao0") {
            uint16_t v = 4242;
            pe.write(&v);
        }
    }
    EXPECT_TRUE(*s.store().getBit(TableId::Coil, 1));
    EXPECT_EQ(*s.store().getReg(TableId::HoldingRegister, 0), 4242);

    // index-out-of-range guards
    EXPECT_FALSE(mod.digitalInput(2).has_value());
    EXPECT_FALSE(mod.setAnalogOutput(9, 1).has_value());
}
#endif // TETHER_MODBUS_IO_BRIDGE

// ---- CaptureFile: real libpcap output ------------------------------------
TEST(CaptureFile, PcapFormat) {
    CaptureFile cap;
    const std::string path = "/tmp/mb_test.pcap";
    ASSERT_TRUE(cap.openPcap(path).has_value());
    const uint8_t adu[] = {0x01, 0x03, 0x00, 0x6B, 0x00, 0x03, 0x76, 0x87};
    cap.record(true, adu);
    cap.record(false, adu);
    cap.close();

    std::ifstream in(path, std::ios::binary);
    std::vector<uint8_t> b((std::istreambuf_iterator<char>(in)), {});
    // 24B global + 2 × (16B rec + 1B dir + 8B adu) = 74 bytes.
    ASSERT_EQ(b.size(), 74u);
    EXPECT_EQ(b[0], 0xd4); // LE magic
    EXPECT_EQ(b[20], 0x93) << "LINKTYPE_USER0 expected";
    EXPECT_EQ(b[24 + 16], 1u);  // first record dir=tx
    EXPECT_EQ(b[24 + 16 + 1], 0x01); // ADU starts
    std::remove(path.c_str());
}
