// modbus_rtu_slave — Modbus RTU slave (server role) on a serial port.
//   Serves an in-memory data model to any master on the RS-485 bus.
//   usage: modbus_rtu_slave <serial-spec> [unit]   e.g.
//          modbus_rtu_slave /dev/ttyUSB0:19200-8E1 17
//          modbus_rtu_slave /dev/pts/5:9600-8N1 1      (pty for testing)
//
//   Data model (visible to the master):
//     holding regs 0..99   — writable, seeded with hr[i] = i*10
//     input  regs 0..99    — read-only sensor block (ir[i] = 1000+i)
//     coils    0..63       — writable
//     discrete 0..63       — read-only, mirrors coils
#include <print>
#include <thread>

#include "tether/modbus/link/SerialPortLink.hpp"
#include "tether/modbus/server/Server.hpp"

using namespace tether::modbus;

int main(int argc, char** argv) {
    if (argc < 2) {
        std::print(stderr, "usage: {} <serial-spec> [unit]\n", argv[0]);
        return 1;
    }
    auto cfg = SerialConfig::parse(argv[1]);
    if (!cfg) {
        std::print(stderr, "serial: {}\n", cfg.error().format());
        return 1;
    }
    const uint8_t unit = argc > 2 ? uint8_t(std::stoi(argv[2])) : 1;

    DataStore store;
    (void)store.declare(TableId::HoldingRegister, 0, 100);
    (void)store.declare(TableId::InputRegister, 0, 100);
    (void)store.declare(TableId::Coil, 0, 64);
    (void)store.declare(TableId::DiscreteInput, 0, 64);
    for (uint16_t i = 0; i < 100; ++i) {
        (void)store.setReg(TableId::HoldingRegister, i, uint16_t(i * 10));
        (void)store.setReg(TableId::InputRegister, i, uint16_t(1000 + i));
    }

    // Only serve our unit id; broadcast writes stay enabled.
    ServerOpts sopts;
    sopts.units = {unit};
    // Audit every write on stderr.
    store.setWriteHook([](TableId t, uint16_t a, uint16_t n,
                          std::span<const uint16_t> regs,
                          std::span<const uint8_t> bits) {
        if (t == TableId::Coil)
            std::print(stderr, "  write coil[{}..{}] {}\n", a, a + n - 1,
                       int(bits.empty() ? 0 : bits[0]));
        else
            std::print(stderr, "  write hr[{}..{}] {}\n", a, a + n - 1,
                       regs.empty() ? 0 : regs[0]);
        return true;                     // accept
    });

    RequestDispatcher disp(store, sopts);

    auto link = makeSerialLink(*cfg, OpenOpts{});
    if (!link) {
        std::print(stderr, "link: {}\n", link.error().format());
        return 1;
    }
    auto ch = std::make_unique<RtuChannel>(std::move(*link),
                                           RtuTiming::forSerial(*cfg));
    ModbusServer server(std::move(ch), disp);
    if (auto e = server.start(); !e) {
        std::print(stderr, "start {}: {}\n", cfg->describe(),
                   e.error().format());
        return 1;
    }
    std::print("rtu slave unit={} on {} — Ctrl-C to quit\n", unit,
               cfg->describe());
    std::fflush(stdout);
    for (;;) {
        std::this_thread::sleep_for(std::chrono::seconds(5));
        std::print("  reqs={} resp={} exc={} bcast={} bad={}\n",
                   server.stats().requests.load(),
                   server.stats().responses.load(),
                   server.stats().exceptions.load(),
                   server.stats().broadcasts.load(),
                   server.stats().badFrames.load());
    }
}
