// std::future-based async API: client, raw invoker, TypedRef,
// BoundDevice, WriteCache; span snapshotting; TCP pipelined async.
#include <gtest/gtest.h>

#include "tether/modbus/client/BusExecutor.hpp"
#include "tether/modbus/client/ModbusClient.hpp"
#include "tether/modbus/client/TransactionManager.hpp"
#include "tether/modbus/client/WriteCache.hpp"
#include "tether/modbus/link/TcpLink.hpp"
#include "tether/modbus/profile/Profile.hpp"
#include "tether/modbus/server/Server.hpp"
#include "tether/modbus/sim/Sim.hpp"

using namespace tether::modbus;
using namespace std::chrono_literals;

namespace {
SimSlave& makeSlave(SimSlave& s) {
    if (!s.store().covers(TableId::HoldingRegister, 0, 16))
        (void)s.store().declare(TableId::HoldingRegister, 0, 16);
    if (!s.store().covers(TableId::Coil, 0, 16))
        (void)s.store().declare(TableId::Coil, 0, 16);
    if (!s.store().covers(TableId::InputRegister, 0, 8))
        (void)s.store().declare(TableId::InputRegister, 0, 8);
    if (!s.store().covers(TableId::DiscreteInput, 0, 8))
        (void)s.store().declare(TableId::DiscreteInput, 0, 8);
    return s;
}
} // namespace

// ---- ModbusClient async reads/writes over SimInvoker ----------------------

TEST(Async, ClientReads) {
    SimSlave s(1);
    makeSlave(s).preload(0, {11, 22, 33});
    SimInvoker inv(s);
    ModbusClient cli(inv, 1);

    auto fh = cli.readHoldingAsync(0, 3);
    ASSERT_TRUE(fh.get().has_value());
    EXPECT_EQ(*cli.readHoldingAsync(0, 3).get(),
              (std::vector<uint16_t>{11, 22, 33}));
    EXPECT_EQ(*cli.readHoldingOneAsync(1).get(), 22);
    ASSERT_TRUE(cli.readInputAsync(0, 2).get().has_value());
    ASSERT_TRUE(cli.readCoilsAsync(0, 4).get().has_value());
    ASSERT_TRUE(cli.readDiscreteInputsAsync(0, 4).get().has_value());
}

TEST(Async, ClientWrites) {
    SimSlave s(1);
    makeSlave(s);
    SimInvoker inv(s);
    ModbusClient cli(inv, 1);

    ASSERT_TRUE(cli.writeRegAsync(4, 0xBEEF).get().has_value());
    EXPECT_EQ(*s.store().getReg(TableId::HoldingRegister, 4), 0xBEEF);

    ASSERT_TRUE(cli.writeCoilAsync(2, true).get().has_value());
    EXPECT_TRUE(cli.readCoils(2, 1)->front());

    std::vector<uint16_t> regs{0xAA, 0xBB, 0xCC};
    ASSERT_TRUE(cli.writeRegsAsync(6, regs).get().has_value());
    EXPECT_EQ(*cli.readHolding(6, 3), regs);

    const bool bits[] = {true, false, true};
    ASSERT_TRUE(cli.writeCoilsAsync(8, std::span<const bool>(bits, 3))
                    .get().has_value());
    auto rb = cli.readCoils(8, 3);
    ASSERT_TRUE(rb.has_value());
    EXPECT_TRUE((*rb)[0]); EXPECT_FALSE((*rb)[1]); EXPECT_TRUE((*rb)[2]);

    ASSERT_TRUE(cli.writeCoilsAsync(11, std::vector<bool>{true, true})
                    .get().has_value());
    EXPECT_TRUE(cli.readCoils(11, 1)->front());

    ASSERT_TRUE(cli.maskWriteAsync(4, 0x00FF, 0x0F00).get().has_value());
    // (0xBEEF & 0x00FF) | (0x0F00 & ~0x00FF) = 0x00EF | 0x0F00 = 0x0FEF
    EXPECT_EQ(*s.store().getReg(TableId::HoldingRegister, 4), 0x0FEF);
}

TEST(Async, SpanArgsAreSnapshotted) {
    // The caller may mutate/free span-backed storage right after the
    // call returns — the async task must still see the original data.
    SimSlave s(1);
    makeSlave(s);
    SimInvoker inv(s);
    ModbusClient cli(inv, 1);

    std::vector<uint16_t> src{0xDE, 0xAD, 0xBE, 0xEF};
    auto fut = cli.writeRegsAsync(0, std::span<const uint16_t>(src));
    src.assign(4, 0);           // caller mutates immediately
    ASSERT_TRUE(fut.get().has_value());
    EXPECT_EQ(*cli.readHolding(0, 4),
              (std::vector<uint16_t>{0xDE, 0xAD, 0xBE, 0xEF}));
}

TEST(Async, ClientMetaAndRawInvoke) {
    SimSlave s(1);
    makeSlave(s).preload(0, {7});
    SimInvoker inv(s);
    ModbusClient cli(inv, 1);

    // Raw invokeAsync → future<Expected<Pdu>>
    auto rq = build::readRange(FunctionCode::ReadHoldingRegisters, 0, 1);
    auto f = cli.invokeAsync(FunctionCode::ReadHoldingRegisters,
                             rq->body());
    auto pdu = f.get();
    ASSERT_TRUE(pdu.has_value());
    EXPECT_EQ(pdu->body()[0], 2);            // byte count
    EXPECT_EQ((pdu->body()[1] << 8) | pdu->body()[2], 7);

    // readWriteMultiAsync
    const uint16_t w[] = {9, 9};
    auto rw = cli.readWriteMultiAsync(0, 1, 8,
                                      std::span<const uint16_t>(w, 2));
    ASSERT_TRUE(rw.get().has_value());
    EXPECT_EQ(*s.store().getReg(TableId::HoldingRegister, 8), 9);
}

TEST(Async, ErrorsFlowThroughFuture) {
    SimSlave s(1, SimTraits{.dropResponse = 1.0});
    makeSlave(s);
    SimInvoker inv(s);
    ModbusClient cli(inv, 1);
    auto f = cli.readHoldingAsync(0, 1);
    auto r = f.get();
    ASSERT_FALSE(r.has_value());
    EXPECT_TRUE(r.error().isTimeout());
}

TEST(Async, ManyConcurrentFutures) {
    // Serialized invoker + many outstanding async ops must all complete.
    SimSlave s(1);
    makeSlave(s).preload(0, {1, 2, 3, 4, 5, 6, 7, 8});
    SimInvoker inv(s);
    ModbusClient cli(inv, 1);

    std::vector<std::future<Expected<std::vector<uint16_t>>>> fs;
    for (uint16_t i = 0; i < 8; ++i)
        fs.push_back(cli.readHoldingAsync(i, 1));
    for (uint16_t i = 0; i < 8; ++i) {
        auto r = fs[i].get();
        ASSERT_TRUE(r.has_value());
        EXPECT_EQ(r->front(), i + 1);
    }
}

TEST(Async, Broadcast) {
    SimBus bus;
    auto& s = makeSlave(bus.add(1));
    (void)makeSlave(bus.add(2));
    SimInvoker inv(s);
    ModbusClient cli(inv, 1);
    const uint16_t v[] = {0x55};
    ASSERT_TRUE(cli.broadcastWriteRegsAsync(3,
        std::span<const uint16_t>(v, 1)).get().has_value());
    EXPECT_EQ(*s.store().getReg(TableId::HoldingRegister, 3), 0x55);
    ASSERT_TRUE(cli.broadcastWriteRegAsync(4, 0x66).get().has_value());
    EXPECT_EQ(*s.store().getReg(TableId::HoldingRegister, 4), 0x66);
}

// ---- TypedRef / BoundDevice / WriteCache async ----------------------------

TEST(Async, TypedRef) {
    SimSlave s(1);
    makeSlave(s).preload(0, {0x0000, 0x4120});   // f32 10.0, low word first
    SimInvoker inv(s);
    ModbusClient cli(inv, 1);
    TypedRef ref(cli, {TableId::HoldingRegister, 0},
                 {ValueType::F32, WordOrder::WordSwap});
    auto r = ref.readAsync().get();
    ASSERT_TRUE(r.has_value());
    EXPECT_DOUBLE_EQ(*r, 10.0);
    ASSERT_TRUE(ref.writeAsync(25.5).get().has_value());
    auto rr = ref.read();
    ASSERT_TRUE(rr.has_value());
    EXPECT_DOUBLE_EQ(*rr, 25.5);

    TypedRef bit(cli, {TableId::Coil, 3}, {ValueType::Bit});
    ASSERT_TRUE(bit.writeAsync(1.0).get().has_value());
    EXPECT_DOUBLE_EQ(*bit.readAsync().get(), 1.0);
}

TEST(Async, BoundDevice) {
    ProfileRegistry reg;
    reg.registerBuiltins();
    const auto* prof = reg.find("generic/vfd");
    ASSERT_NE(prof, nullptr);

    SimSlave s(prof->defaultUnit);
    auto& st = s.store();
    (void)st.declare(TableId::HoldingRegister, 0, 4);
    (void)st.declare(TableId::InputRegister, 0, 4);
    (void)st.declare(TableId::Coil, 0, 4);
    SimInvoker inv(s);
    ModbusClient cli(inv, prof->defaultUnit);
    BoundDevice dev(cli, *prof);

    ASSERT_TRUE(dev.writeAsync("freq_set", 123.45).get().has_value());
    auto v = dev.readAsync("freq_set").get();
    ASSERT_TRUE(v.has_value());
    EXPECT_DOUBLE_EQ(*v, 123.45);
    ASSERT_TRUE(dev.writeAsync("run", 1.0).get().has_value());
    EXPECT_DOUBLE_EQ(*dev.readAsync("run").get(), 1.0);

    auto bad = dev.readAsync("no_such_reg").get();
    ASSERT_FALSE(bad.has_value());
}

TEST(Async, WriteCacheFlush) {
    SimSlave s(1);
    makeSlave(s);
    SimInvoker inv(s);
    ModbusClient cli(inv, 1);
    WriteCache wc(cli);
    wc.seed(0, 0);
    wc.set(0, 0x1111);
    wc.set(1, 0x2222);
    wc.setCoil(0, true);
    ASSERT_TRUE(wc.flushAsync().get().has_value());
    EXPECT_EQ(*s.store().getReg(TableId::HoldingRegister, 0), 0x1111);
    EXPECT_EQ(*s.store().getReg(TableId::HoldingRegister, 1), 0x2222);
    EXPECT_TRUE(cli.readCoils(0, 1)->front());
    EXPECT_EQ(wc.dirtyRegs(), 0u);
}

// ---- True pipelining: async futures over TransactionManager ---------------

TEST(Async, TcpPipelinedFutures) {
    DataStore store;
    ASSERT_TRUE(store.declare(TableId::HoldingRegister, 0, 32)
                    .has_value());
    ASSERT_TRUE(store.setReg(TableId::HoldingRegister, 9, 0xF00D));
    RequestDispatcher d(store);
    const uint16_t port = 15531;
    TcpServer server(
        std::make_unique<TcpListener>("127.0.0.1", port, 4, true), d);
    ASSERT_TRUE(server.start().has_value());
    std::this_thread::sleep_for(50ms);

    auto link = std::make_unique<TcpLink>("127.0.0.1", port, 2000);
    MbapChannel ch(std::move(link));
    TransactionManager tm(ch, /*maxInflight=*/8);
    ASSERT_TRUE(tm.start().has_value());
    ModbusClient client(tm, 1);

    // Launch all 8 reads without blocking on any single one.
    std::vector<std::future<Expected<uint16_t>>> fs;
    for (int i = 0; i < 8; ++i)
        fs.push_back(client.readHoldingOneAsync(9));
    int ok = 0;
    for (auto& f : fs) {
        auto v = f.get();
        if (v && *v == 0xF00D) ++ok;
    }
    EXPECT_EQ(ok, 8);
    EXPECT_EQ(tm.inflight(), 0);

    tm.stop();
    server.stop();
}
