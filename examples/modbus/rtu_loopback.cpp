// modbus_rtu_loopback — full RTU master <-> slave over a pty pair.
//   Real serial bytes (termios, RTU framing, t3.5 gaps, CRC) with no
//   external hardware: the kernel pty driver moves the bytes.
//   Master = RtuChannel + BusExecutor + ModbusClient;
//   Slave  = ModbusServer + RequestDispatcher + DataStore.
//   Wire traffic is logged via FrameLogger taps on both links.
#include <chrono>
#include <print>
#include <thread>

#include "tether/modbus/client/BusExecutor.hpp"
#include "tether/modbus/client/ModbusClient.hpp"
#include "tether/modbus/diag/Trace.hpp"
#include "tether/modbus/link/TestLinks.hpp"   // PtyPair (Linux)
#include "tether/modbus/server/Server.hpp"

using namespace tether::modbus;

int main() {
    // ---- pty pair = two ends of one serial wire ------------------------------
    auto pty = PtyPair::open(SerialConfig{.baud = 19200});
    if (!pty) { std::print(stderr, "pty: {}\n", pty.error().format());
                return 1; }
    std::print("wire: {}\n", (*pty)->slavePath());

    FrameLogger log([](const std::string& l) { std::print("{}\n", l); });

    // ---- slave side -----------------------------------------------------------
    DataStore store;
    (void)store.declare(TableId::HoldingRegister, 0, 100);
    (void)store.declare(TableId::InputRegister, 0, 100);
    (void)store.declare(TableId::Coil, 0, 64);
    for (uint16_t i = 0; i < 8; ++i)
        (void)store.setReg(TableId::HoldingRegister, i,
                           uint16_t(1000 + i * 11));
    (void)store.setReg(TableId::InputRegister, 0, 42);

    RequestDispatcher disp(store, {.units = {7}});   // serve unit 7 only
    auto slaveLink = (*pty)->slaveLink();
    auto slaveCh = std::make_unique<RtuChannel>(
        std::move(slaveLink),
        RtuTiming::forParams(19200, 11));            // 8E1 = 11 bits
    ModbusServer server(std::move(slaveCh), disp);
    if (auto e = server.start(); !e) {
        std::print(stderr, "slave: {}\n", e.error().format());
        return 1;
    }

    // ---- master side ----------------------------------------------------------
    auto masterLink = (*pty)->masterLink();
    masterLink->setTap([&log](bool tx, std::span<const uint8_t> b) {
        log.tap(tx, b);                          // wire trace
    });
    auto masterCh = std::make_unique<RtuChannel>(
        std::move(masterLink), RtuTiming::forParams(19200, 11));
    if (auto e = masterCh->open(); !e) {
        std::print(stderr, "master: {}\n", e.error().format());
        return 1;
    }
    BusExecutor ex(*masterCh);
    ex.setDefaultOptions(TransactionOptions{.timeoutMs = 300});
    ModbusClient cli(ex, 7);                        // target unit 7

    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    std::print("\n== read holding regs 0..3\n");
    if (auto r = cli.readHolding(0, 4); r) {
        std::print("  ->");
        for (auto v : *r) std::print(" {}", v);
        std::print("\n");
    } else std::print("  err {}\n", r.error().format());

    std::print("\n== write single reg 20 = 0xBEEF\n");
    if (auto e = cli.writeReg(20, 0xBEEF); !e)
        std::print("  err {}\n", e.error().format());
    else {
        auto v = cli.readHoldingOne(20);
        std::print("  readback = 0x{:04x}\n", v.value_or(0));
    }

    std::print("\n== coil write + read\n");
    (void)cli.writeCoil(5, true);
    if (auto c = cli.readCoils(0, 8); c) {
        std::print("  coils 0..7:");
        for (auto b : *c) std::print(" {}", int(b));
        std::print("\n");
    }

    std::print("\n== broadcast write (unit 0, no response)\n");
    (void)cli.broadcastWriteReg(50, 0x1234);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    auto b = cli.readHoldingOne(50);
    std::print("  hr50 = 0x{:04x} (written by broadcast)\n",
               b.value_or(0));

    std::print("\n== wrong unit is ignored (slave serves only 7)\n");
    cli.setUnit(9);
    auto wrong = cli.readHolding(0, 1);
    std::print("  unit 9 read: {}\n",
               wrong ? "unexpected reply" : wrong.error().format());
    cli.setUnit(7);

    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    server.stop();
    std::print("\nserver stats: reqs={} resp={} exc={} bcast={} bad={}\n",
               server.stats().requests.load(),
               server.stats().responses.load(),
               server.stats().exceptions.load(),
               server.stats().broadcasts.load(),
               server.stats().badFrames.load());
    return 0;
}
