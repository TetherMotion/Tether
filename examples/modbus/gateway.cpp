// modbus_gateway — Modbus/TCP front-end → RTU downstream on a serial bus.
//   usage: modbus_gateway <listen-port> <serial-spec>   e.g.
//          modbus_gateway 1502 /dev/ttyUSB0:19200-8E1
#include <print>
#include <thread>

#include "tether/modbus/client/BusExecutor.hpp"
#include "tether/modbus/client/ModbusClient.hpp"
#include "tether/modbus/link/SerialPortLink.hpp"
#include "tether/modbus/server/Gateway.hpp"

using namespace tether::modbus;

int main(int argc, char** argv) {
    if (argc < 3) {
        std::print(stderr, "usage: {} <port> <serial>\n", argv[0]);
        return 1;
    }
    const auto port = uint16_t(std::stoi(argv[1]));
    auto cfg = SerialConfig::parse(argv[2]);
    if (!cfg) { std::print(stderr, "serial: {}\n",
                           cfg.error().format()); return 1; }

    auto link = makeSerialLink(*cfg, OpenOpts{});
    if (!link) { std::print(stderr, "link: {}\n",
                          link.error().format()); return 1; }
    auto ch = std::make_unique<RtuChannel>(
        std::move(*link), RtuTiming::forSerial(*cfg));
    if (auto e = ch->open(); !e) {
        std::print(stderr, "open {}: {}\n", cfg->describe(),
                   e.error().format()); return 1;
    }
    BusExecutor downstream(*ch);

    auto listener =
        std::make_unique<TcpListener>("0.0.0.0", port, 8, true);
    Gateway gw(std::move(listener), downstream);
    if (auto e = gw.start(); !e) {
        std::print(stderr, "listen: {}\n", e.error().format()); return 1;
    }
    std::print("gateway :{} → {} — Ctrl-C to quit\n", port, cfg->describe());
    std::fflush(stdout);
    for (;;) std::this_thread::sleep_for(std::chrono::seconds(60));
}
