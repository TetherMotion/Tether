// modbus_tcp_client — poll registers from a Modbus/TCP device.
//   usage: modbus_tcp_client <host> [port] [unit] [addr] [qty] [ms]
#include <print>
#include <thread>

#include "tether/modbus/client/ModbusClient.hpp"
#include "tether/modbus/client/Poller.hpp"

using namespace tether::modbus;

int main(int argc, char** argv) {
    if (argc < 2) {
        std::print(stderr, "usage: {} <host> [port] [unit] [addr] [qty] [ms]\n",
                   argv[0]);
        return 1;
    }
    const auto host = argv[1];
    const auto port = argc > 2 ? uint16_t(std::stoi(argv[2])) : 502;
    const auto unit = argc > 3 ? uint8_t(std::stoi(argv[3])) : 1;
    const auto addr = argc > 4 ? uint16_t(std::stoi(argv[4])) : 0;
    const auto qty  = argc > 5 ? uint16_t(std::stoi(argv[5])) : 4;
    const auto ms   = argc > 6 ? std::stoi(argv[6]) : 500;

    auto m = makeTcpMaster(host, port, unit, 3000,
                           TransactionOptions{1000, 1, 0, {}});
    if (!m) { std::print(stderr, "connect: {}\n",
                         m.error().format()); return 1; }

    Poller poller(*m->client);
    PollItem it;
    it.fc = FunctionCode::ReadHoldingRegisters;
    it.addr = addr; it.qty = qty; it.intervalMs = ms; it.name = "watch";
    it.onResult = [](const PollItem& i,
                     const std::vector<uint16_t>& v, const ModbusError* e) {
        if (e) { std::print(stderr, "poll {}: {}\n", i.name, e->format());
                 return; }
        std::print("hr[{}..{}]:", i.addr, i.addr + i.qty - 1);
        for (auto x : v) std::print(" {}", x);
        std::print("\n");
    };
    poller.add(std::move(it));
    if (auto e = poller.start(); !e) {
        std::print(stderr, "poller: {}\n", e.error().format()); return 1;
    }
    std::print("polling {}:{} unit {} — Ctrl-C to quit\n", host, port, unit);
    for (;;) std::this_thread::sleep_for(std::chrono::seconds(60));
}
