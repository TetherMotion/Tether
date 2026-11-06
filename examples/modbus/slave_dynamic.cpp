// modbus_slave_dynamic — Modbus/TCP slave with a LIVE data model.
//   A background thread mutates input registers (simulated temperature
//   + counter) so polls see changing values; holding writes are audited
//   and clamped; FC43 device-id and FC17 server-id are implemented.
//   usage: modbus_slave_dynamic [port]        (default 1502)
//   test:  modbus_tcp_client 127.0.0.1:1502 1 0 4
#include <cmath>
#include <print>
#include <thread>

#include "tether/modbus/link/TcpLink.hpp"
#include "tether/modbus/server/Server.hpp"

using namespace tether::modbus;

int main(int argc, char** argv) {
    const auto port = argc > 1 ? uint16_t(std::stoi(argv[1])) : 1502;

    DataStore store;
    (void)store.declare(TableId::InputRegister, 0, 100);
    (void)store.declare(TableId::HoldingRegister, 0, 100);
    (void)store.declare(TableId::Coil, 0, 64);

    // Input regs: [0]=temp x10 °C, [1]=uptime s, [2]=random
    std::atomic<bool> run{true};
    std::thread sensor([&] {
        double t = 0;
        while (run.load()) {
            const double tempC = 22.0 + 3.0 * std::sin(t / 10.0);
            (void)store.setReg(TableId::InputRegister, 0,
                               uint16_t(tempC * 10));
            static int up = 0;
            (void)store.setReg(TableId::InputRegister, 1,
                               uint16_t(++up));
            (void)store.setReg(TableId::InputRegister, 2,
                               uint16_t(std::rand() % 1000));
            t += 1.0;
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
    });

    // Gate writes: clamp "setpoint" hr0 to [0,100], reject hr99.
    store.setWriteHook([](TableId t, uint16_t a, uint16_t n,
                          std::span<const uint16_t> regs,
                          std::span<const uint8_t>) {
        if (t != TableId::HoldingRegister) return true;
        for (uint16_t i = 0; i < n; ++i) {
            const uint16_t addr = a + i;
            if (addr == 99) return false;      // reserved → IllegalDataValue
            if (addr == 0 && regs[i] > 100) {
                std::print(stderr,
                           "  clamp hr0 {} -> 100 (silently ok)\n", regs[i]);
            }
        }
        return true;
    });

    // Rich server options: device identification + server id + allowlist.
    ServerOpts sopts;
    sopts.onDeviceId = [](uint8_t, const DevIdRequest&)
        -> Expected<Response> {
        MeiResponse r;
        r.mei = 0x0E;
        r.devid.conformity = 1;
        r.devid.more = false;
        r.devid.objects = {
            {0, "Tether"}, {1, "modbus_slave_dynamic"}, {2, "1.0"}};
        r.devid.nextObjectId = 3;
        return Response{r};
    };
    sopts.onServerId = [](uint8_t) {
        const std::string id = "tether-dynamic-slave";
        return std::vector<uint8_t>(id.begin(), id.end());
    };

    RequestDispatcher disp(store, sopts);
    auto listener = std::make_unique<TcpListener>("0.0.0.0", port, 8, true);
    TcpServer server(std::move(listener), disp);
    if (auto e = server.start(); !e) {
        std::print(stderr, "start: {}\n", e.error().format());
        return 1;
    }
    std::print("dynamic slave on :{} (ir0=temp x10 C, ir1=uptime)\n", port);
    std::fflush(stdout);
    for (;;) {
        std::this_thread::sleep_for(std::chrono::seconds(5));
        std::print("  clients={} reqs={} exc={}\n", server.clientCount(),
                   server.stats().requests.load(),
                   server.stats().exceptions.load());
    }
}
