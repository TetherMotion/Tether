// modbus_diag_capture — capture live wire traffic to pcap + console.
//   A SimLink carries real RTU frames between the master and a simulated
//   device; attachByteTap feeds every byte block to a FrameLogger (human
//   decode) and a CaptureFile (Wireshark-readable libpcap).
//   Output: /tmp/modbus_capture.pcap (LINKTYPE_USER0).
#include <print>

#include "tether/modbus/client/BusExecutor.hpp"
#include "tether/modbus/client/ModbusClient.hpp"
#include "tether/modbus/diag/Trace.hpp"
#include "tether/modbus/sim/Sim.hpp"

using namespace tether::modbus;

int main(int argc, char** argv) {
    const std::string out =
        argc > 1 ? argv[1] : "/tmp/modbus_capture.pcap";

    SimBus bus;
    auto& dev = bus.add(1);
    (void)dev.store().declare(TableId::HoldingRegister, 0, 64);
    (void)dev.store().declare(TableId::Coil, 0, 32);
    dev.preload(0, {0x1111, 0x2222, 0x3333, 0x4444});

    auto link = std::make_unique<SimLink>(bus, /*rtu*/true);
    auto* raw = link.get();

    CaptureFile cap;
    if (auto e = cap.openPcap(out); !e) {
        std::print(stderr, "pcap: {}\n", e.error().format());
        return 1;
    }
    FrameLogger log([](const std::string& l) { std::print("  {}\n", l); });
    raw->setTap([&](bool tx, std::span<const uint8_t> b) {
        log.tap(tx, b);
        cap.record(tx, b);
    });

    auto ch = std::make_unique<RtuChannel>(
        std::move(link), RtuTiming::forParams(19200, 11));
    if (auto e = ch->open(); !e) {
        std::print(stderr, "open: {}\n", e.error().format());
        return 1;
    }
    BusExecutor ex(*ch);
    ModbusClient cli(ex, 1);

    std::print("== transactions (also written to {})\n", out);
    (void)cli.readHolding(0, 4);
    (void)cli.writeReg(8, 0xBEEF);
    (void)cli.writeCoil(3, true);
    (void)cli.readHolding(0, 9);          // exercises reassembly
    (void)cli.broadcastWriteReg(20, 0x5A);  // broadcast = TX only

    cap.close();
    std::print("done — open {} in Wireshark "
               "(decode-as USER_DLT0 or read raw RTU bytes)\n", out);
    return 0;
}
