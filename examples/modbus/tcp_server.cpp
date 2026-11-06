// modbus_tcp_server — serve an in-memory data model on :1502 (default).
//   usage: modbus_tcp_server [port]
#include <print>
#include <thread>

#include "tether/modbus/link/TcpLink.hpp"
#include "tether/modbus/server/Server.hpp"

using namespace tether::modbus;

int main(int argc, char** argv) {
    const auto port = argc > 1 ? uint16_t(std::stoi(argv[1])) : 1502;

    DataStore store;
    (void)store.declare(TableId::HoldingRegister, 0, 1000);
    (void)store.declare(TableId::InputRegister, 0, 1000);
    (void)store.declare(TableId::Coil, 0, 1000);
    (void)store.declare(TableId::DiscreteInput, 0, 1000);
    for (uint16_t i = 0; i < 1000; ++i)
        (void)store.setReg(TableId::HoldingRegister, i, i);

    RequestDispatcher disp(store);
    auto listener =
        std::make_unique<TcpListener>("0.0.0.0", port, 8, true);
    TcpServer server(std::move(listener), disp);
    if (auto e = server.start(); !e) {
        std::print(stderr, "start: {}\n", e.error().format()); return 1;
    }
    std::print("modbus/tcp on :{} — Ctrl-C to quit\n", port);
    std::fflush(stdout);
    for (;;) std::this_thread::sleep_for(std::chrono::seconds(60));
}
