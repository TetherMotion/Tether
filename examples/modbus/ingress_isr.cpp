// modbus_ingress_isr — ISR-style byte ingress (push model).
//   Demonstrates the microcontroller receive path on Linux: a "UART IRQ"
//   thread pushes response bytes one at a time into an RxIngress ring
//   (lock-free SPSC, atomic seq wake); the master's RtuChannel drains it
//   through an IngressLink. TX side forwards request ADUs straight into
//   the slave's dispatcher — the slave reply is what gets IRQ-fed.
#include <chrono>
#include <deque>
#include <mutex>
#include <print>
#include <thread>

#include "tether/modbus/client/BusExecutor.hpp"
#include "tether/modbus/client/ModbusClient.hpp"
#include "tether/modbus/link/RxIngress.hpp"
#include "tether/modbus/protocol/Crc.hpp"
#include "tether/modbus/server/Server.hpp"

using namespace tether::modbus;

static int64_t nowNs() {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
}

int main() {
    // ---- slave model -------------------------------------------------------
    DataStore store;
    (void)store.declare(TableId::HoldingRegister, 0, 32);
    for (uint16_t i = 0; i < 8; ++i)
        (void)store.setReg(TableId::HoldingRegister, i, uint16_t(500 + i));
    RequestDispatcher disp(store, {.units = {3}});

    // ---- "UART": tx hook hands the request ADU to the slave; the IRQ
    //      thread pushes the response ADU into the ring byte-by-byte. ----
    RxIngress<512> rx;
    std::deque<uint8_t> txFifo;                  // bytes awaiting "TX IRQ"
    std::mutex      txM;
    std::atomic<bool> run{true};

    // IRQ thread: feeds rx ring ONE byte at a time — the worst-case
    // pattern a byte-receive interrupt produces (and stamps wire time).
    std::thread irq([&] {
        while (run.load()) {
            uint8_t b = 0;
            {
                std::lock_guard lk(txM);
                if (txFifo.empty()) {
                    std::this_thread::sleep_for(
                        std::chrono::microseconds(50));
                    continue;
                }
                b = txFifo.front(); txFifo.pop_front();
            }
            rx.onByte(b, nowNs());               // ISR: push + wake
        }
    });

    // TX hook: request ADU -> slave dispatcher -> response ADU queued
    // for the IRQ thread to push into the rx ring.
    auto linkPtr = std::make_unique<IngressLink>(rx,
        [&](std::span<const uint8_t> adu, int) -> Expected<void> {
            if (adu.size() < 4) return {};
            const uint8_t unit = adu[0];
            const auto pdu = adu.subspan(1, adu.size() - 3);
            auto res = disp.dispatch(unit, pdu);
            if (!res.respond) return {};
            std::vector<uint8_t> rsp;
            rsp.push_back(unit);
            rsp.insert(rsp.end(), res.pdu.begin(), res.pdu.end());
            const uint16_t c = Crc16::of(std::span(rsp));
            rsp.push_back(uint8_t(c)); rsp.push_back(uint8_t(c >> 8));
            std::lock_guard lk(txM);
            txFifo.insert(txFifo.end(), rsp.begin(), rsp.end());
            return {};
        },
        IngressWait{}, IngressLink::WaitFn{},
        int64_t(RtuTiming::forParams(1000000, 10).t35.count() * 1000));
    auto chn = std::make_unique<RtuChannel>(
        std::move(linkPtr), RtuTiming::forParams(1000000, 10));
    if (auto e = chn->open(); !e) {
        std::print(stderr, "open: {}\n", e.error().format());
        return 1;
    }

    BusExecutor ex(*chn);
    ex.setDefaultOptions(TransactionOptions{.timeoutMs = 200});
    ModbusClient cli(ex, 3);

    std::print("read via IRQ-fed ring (byte-at-a-time push):\n");
    for (int i = 0; i < 3; ++i) {
        if (auto r = cli.readHolding(0, 4); r) {
            std::print("  ->");
            for (auto v : *r) std::print(" {}", v);
            std::print("\n");
        } else {
            std::print("  err {}\n", r.error().format());
        }
    }
    (void)cli.writeReg(10, 0xABCD);
    std::print("write+readback hr10 = 0x{:04x}\n",
               cli.readHoldingOne(10).value_or(0));
    std::print("ring overruns={} (bytes pushed per-byte via 'IRQ')\n",
               rx.overruns());

    run.store(false); irq.join();
    return 0;
}
