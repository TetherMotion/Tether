// RxIngress (ISR-style byte ingress) + IngressLink tests — covers the
// push path: interrupt-fed bytes, atomic wake, dribbled delivery,
// producer timestamps, and pull→push pumping.
#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <thread>
#include <vector>

#include "tether/modbus/client/BusExecutor.hpp"
#include "tether/modbus/link/Channel.hpp"
#include "tether/modbus/link/RxIngress.hpp"
#include "tether/modbus/link/TestLinks.hpp"

using namespace tether::modbus;
using std::chrono::steady_clock;

namespace {
int64_t nowNs() {
    return steady_clock::now().time_since_epoch().count();
}
std::vector<uint8_t> rtuFrame(uint8_t unit, uint8_t fc,
                              std::vector<uint8_t> body) {
    std::vector<uint8_t> f(body.size() + 4);
    f[0] = unit; f[1] = fc;
    std::copy(body.begin(), body.end(), f.begin() + 2);
    const uint16_t c = Crc16::of(std::span(f.data(), f.size() - 2));
    f[f.size() - 2] = uint8_t(c); f[f.size() - 1] = uint8_t(c >> 8);
    return f;
}
const RtuTiming kFast = RtuTiming::forParams(1000000, 10);
} // namespace

// ---------- ring mechanics ----------

TEST(RxIngress, PushDrain) {
    RxIngress<16> rx;
    EXPECT_TRUE(rx.onByte(0x11));
    EXPECT_TRUE(rx.onBytes(std::vector<uint8_t>{0x33, 0x44}));
    EXPECT_EQ(rx.pending(), 3u);
    uint8_t out[4];
    EXPECT_EQ(rx.drain(std::span(out)), 3u);
    EXPECT_EQ(out[0], 0x11); EXPECT_EQ(out[2], 0x44);
    EXPECT_EQ(rx.pending(), 0u);
}

TEST(RxIngress, TimestampsCarried) {
    RxIngress<16> rx;
    rx.onBytes(std::vector<uint8_t>{0x01, 0x02}, 777);
    rx.onByte(0x03);                    // untimed
    uint8_t out[4];
    int64_t firstTs = 0, lastTs = 0;
    EXPECT_EQ(rx.drain(std::span(out), 0, &firstTs, &lastTs), 3u);
    EXPECT_EQ(firstTs, 777);
    EXPECT_EQ(lastTs, 0);               // last entry was untimed
}

TEST(RxIngress, WrapAroundAndOverrun) {
    RxIngress<8> rx;                    // 7 usable slots
    for (int i = 0; i < 7; ++i) EXPECT_TRUE(rx.onByte(uint8_t(i)));
    EXPECT_FALSE(rx.onByte(0xFF));      // full → drop + count
    EXPECT_EQ(rx.overruns(), 1u);
    uint8_t out[4];
    EXPECT_EQ(rx.drain(std::span(out)), 4u);   // free 4 slots
    for (int i = 7; i < 11; ++i) EXPECT_TRUE(rx.onByte(uint8_t(i)));
    uint8_t rest[8];
    EXPECT_EQ(rx.drain(std::span(rest)), 7u);
    EXPECT_EQ(rest[0], 4); EXPECT_EQ(rest[6], 10);   // order preserved
}

TEST(RxIngress, AtomicSeqWakesWaiter) {
    RxIngress<16> rx;
    const uint32_t old = rx.seq().load();
    std::atomic<bool> woke{false};
    std::thread consumer([&] {
        rx.seq().wait(old);             // blocks until seq changes
        woke = true;
    });
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    EXPECT_FALSE(woke.load());          // still waiting — nothing queued
    rx.onByte(0x01);                    // "ISR" — must wake the waiter
    consumer.join();
    EXPECT_TRUE(woke.load());
}

TEST(RxIngress, WaitAvailableDeadline) {
    RxIngress<16> rx;
    const int64_t t0 = nowNs();
    EXPECT_FALSE(rx.waitAvailable(t0 + 20'000'000));  // 20 ms → empty
    EXPECT_GE(nowNs() - t0, 18'000'000);              // waited ~deadline
    rx.onByte(0xAA);
    EXPECT_TRUE(rx.waitAvailable(t0 + 20'000'000));   // instant true
}

// ---------- ILink adapter ----------

TEST(IngressLink, DribbledRtuTransact) {
    RxIngress<256> rx;
    std::vector<uint8_t> txLog;
    auto link = std::make_unique<IngressLink>(rx,
        [&](std::span<const uint8_t> b, int) {
            txLog.insert(txLog.end(), b.begin(), b.end());
            return Expected<void>{};
        });
    RtuChannel ch(std::move(link), kFast);
    ASSERT_TRUE(ch.open().has_value());

    // "ISR": feed the response byte-by-byte with real wire gaps
    // (500 µs per byte — dribble delivery, the interrupt-arrival model).
    const auto rsp = rtuFrame(1, 0x03, {0x02, 0x00, 0x2B});
    std::thread isr([&] {
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
        for (uint8_t b : rsp) {
            rx.onByte(b, nowNs());
            std::this_thread::sleep_for(
                std::chrono::microseconds(500));
        }
    });
    auto r = ch.transact(1, std::vector<uint8_t>{0x03, 0, 0, 0, 1}, 500);
    isr.join();
    ASSERT_TRUE(r.has_value());
    EXPECT_EQ(r->unit, 1);
    ASSERT_EQ(r->pdu.size(), 4u);       // fc + bytecount + 2 data
    EXPECT_EQ(r->pdu[3], 0x2B);
    EXPECT_FALSE(txLog.empty());        // request went through TX hook
}

TEST(IngressLink, SilentTimeout) {
    RxIngress<64> rx;
    auto link = std::make_unique<IngressLink>(rx,
        [](std::span<const uint8_t>, int) { return Expected<void>{}; });
    RtuChannel ch(std::move(link), kFast);
    ASSERT_TRUE(ch.open().has_value());
    const int64_t t0 = nowNs();
    auto r = ch.transact(1, std::vector<uint8_t>{0x03, 0, 0, 0, 1}, 60);
    ASSERT_FALSE(r.has_value());
    EXPECT_TRUE(r.error().isTimeout());
    EXPECT_GE(nowNs() - t0, 55'000'000);
}

TEST(IngressLink, ProducerTimestampSplitsCoalescedBursts) {
    // The discriminating case for wire-arrival timestamps: a garbage
    // burst and a valid frame arrived on the WIRE 10 ms apart, but the
    // ring drained them in one shot. With producer timestamps +
    // intraGapNs the drain stops at the wire gap — the channel sees a
    // real split: garbage dropped, frame delivered. Without them the
    // two bursts fuse into one CRC-failing blob and the frame is lost.
    RxIngress<256> rx;
    const int64_t t35Ns =
        kFast.t35.count() * 1000;                    // ~35 µs at kFast
    auto link = std::make_unique<IngressLink>(rx,
        [](std::span<const uint8_t>, int) { return Expected<void>{}; },
        IngressWait{}, IngressLink::WaitFn{}, t35Ns);
    RtuChannel ch(std::move(link), kFast);
    ASSERT_TRUE(ch.open().has_value());

    const int64_t t0 = nowNs();
    const std::vector<uint8_t> garbage{0x7E, 0x11, 0x55, 0xAA, 0x99, 0x01};
    const auto good = rtuFrame(1, 0x03, {0x02, 0x22, 0x22});
    rx.onBytes(std::span(garbage), t0);              // burst at t0
    rx.onBytes(std::span(good), t0 + 10'000'000);    // wire gap: +10 ms

    auto frame = ch.receiveFrame(150);
    ASSERT_TRUE(frame.has_value());
    EXPECT_EQ(frame->pdu[2], 0x22);
}

TEST(IngressLink, CustomWaitFnUsed) {
    RxIngress<64> rx;
    int waitCalls = 0;
    IngressLink link(rx,
        [](std::span<const uint8_t>, int) { return Expected<void>{}; },
        IngressWait{},
        [&](RxIngressBase& i, int64_t dl) {
            ++waitCalls;
            return i.waitAvailable(dl);
        });
    link.open();
    uint8_t buf[8];
    auto n = link.read(std::span(buf), 30);   // empty → waitFn invoked
    ASSERT_TRUE(n.has_value());
    EXPECT_EQ(*n, 0u);
    EXPECT_EQ(waitCalls, 1);
}

TEST(IngressLink, FlushClearsRing) {
    RxIngress<64> rx;
    rx.onBytes(std::vector<uint8_t>{1, 2, 3});
    IngressLink link(rx,
        [](std::span<const uint8_t>, int) { return Expected<void>{}; });
    link.open();
    ASSERT_TRUE(link.flush(Flush::Rx).has_value());
    EXPECT_EQ(rx.pending(), 0u);
    EXPECT_EQ(rx.overruns(), 0u);
}

// ---------- pull→push pump ----------

TEST(IngressPump, PumpedRtuTransact) {
    RxIngress<256> rx;
    // Source: a device-like scripted link answering each request.
    auto src = std::make_unique<ScriptedLink>();
    src->push(rtuFrame(1, 0x03, {0x02, 0x00, 0x2B}));
    ASSERT_TRUE(src->open().has_value());

    // TX writes go onto the same wire the pump reads — arms the
    // scripted device so it "responds" like real hardware.
    auto link = std::make_unique<IngressLink>(rx,
        [&](std::span<const uint8_t> b, int ms) {
            return src->write(b, ms);
        });
    RtuChannel ch(std::move(link), kFast);
    ASSERT_TRUE(ch.open().has_value());
    IngressPump pump(*src, rx);
    pump.start();
    auto r = ch.transact(1, std::vector<uint8_t>{0x03, 0, 0, 0, 1}, 500);
    pump.stop();
    ASSERT_TRUE(r.has_value());
    EXPECT_EQ(r->pdu[3], 0x2B);
}
