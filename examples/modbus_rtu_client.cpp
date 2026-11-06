// modbus_rtu_client — read/write holding registers over RS-485.
//   usage: modbus_rtu_client /dev/ttyUSB0:19200-8E1 [unit] [addr] [qty]
#include <print>

#include "tether/modbus/client/ModbusClient.hpp"

using namespace tether::modbus;

int main(int argc, char** argv) {
    const auto dev   = argc > 1 ? argv[1] : "/dev/ttyUSB0:19200-8E1";
    const auto unit  = argc > 2 ? uint8_t(std::stoi(argv[2])) : 1;
    const auto addr  = argc > 3 ? uint16_t(std::stoi(argv[3])) : 0;
    const auto qty   = argc > 4 ? uint16_t(std::stoi(argv[4])) : 4;

    auto cfg = SerialConfig::parse(dev);
    if (!cfg) { std::print(stderr, "bad device spec: {}\n",
                           cfg.error().format()); return 1; }

    auto m = makeRtuMaster(*cfg, unit, {}, {},
                           TransactionOptions{1000, 2, 20, {}});
    if (!m) { std::print(stderr, "open failed: {}\n",
                         m.error().format()); return 1; }

    auto regs = m->client->readHolding(addr, qty);
    if (!regs) { std::print(stderr, "read: {}\n",
                          regs.error().format()); return 2; }
    std::print("hr[{}..{}]:", addr, addr + qty - 1);
    for (auto v : *regs) std::print(" {}", v);
    std::print("\n");
    return 0;
}
