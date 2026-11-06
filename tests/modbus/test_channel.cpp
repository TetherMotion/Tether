// RTU/TCP channel + framer tests incl. byte loss/corruption matrix.
#include <gtest/gtest.h>

#include <termios.h>
#include <thread>
#include <tuple>
#include <vector>

#include "tether/modbus/link/Channel.hpp"
#include "tether/modbus/client/TransactionManager.hpp"
#include "tether/modbus/link/SerialPortLink.hpp"
#include "tether/modbus/link/TestLinks.hpp"
#include "tether/modbus/protocol/Builder.hpp"

using namespace tether::modbus;

namespace {
/// Build a valid RTU response frame for unit/fc + body.
std::vector<uint8_t> rtuFrame(uint8_t unit, uint8_t fc,
                              std::vector<uint8_t> body) {
    std::vector<uint8_t> f(body.size() + 4);   // unit + fc + body + crc(2)
    f[0] = unit; f[1] = fc;
    std::copy(body.begin(), body.end(), f.begin() + 2);
    const uint16_t c = Crc16::of(std::span(f.data(), f.size() - 2));
    f[f.size() - 2] = uint8_t(c); f[f.size() - 1] = uint8_t(c >> 8);
    return f;
}
const RtuTiming kFast = RtuTiming::forParams(1000000, 10); // tiny gaps
} // namespace

TEST(RtuFramer, RoundTrip) {
    RtuFramer f;
    const auto frame = f.encode(0x11, std::vector<uint8_t>{0x03,0x00,0x6B,0x00,0x03}, 0);
    ASSERT_EQ(frame.size(), 8u);
    EXPECT_TRUE(Crc16::check(frame));
    std::vector<Adu> out;
    f.feed(frame, out);
    f.finishFrame(out);
    ASSERT_EQ(out.size(), 1u);
    EXPECT_EQ(out[0].unit, 0x11);
    ASSERT_EQ(out[0].pdu.size(), 5u);
    EXPECT_EQ(out[0].pdu[0], 0x03);
}

TEST(RtuFramer, BadCrcDropped) {
    RtuFramer f;
    auto frame = f.encode(0x11, std::vector<uint8_t>{0x03,0x00,0x6B}, 0);
    frame.back() ^= 0xFF;
    std::vector<Adu> out;
    f.feed(frame, out);
    f.finishFrame(out);
    EXPECT_TRUE(out.empty());
}

TEST(MbapFramer, FragmentedAndPipelined) {
    MbapFramer f;
    const auto a = f.encode(1, std::vector<uint8_t>{0x03,0x02,0x00}, 7);
    const auto b = f.encode(1, std::vector<uint8_t>{0x06,0x00}, 8);
    std::vector<uint8_t> both(a);
    both.insert(both.end(), b.begin(), b.end());
    std::vector<Adu> out;
    // drip-feed one byte at a time
    for (uint8_t x : both) f.feed(std::span(&x, 1), out);
    ASSERT_EQ(out.size(), 2u);
    EXPECT_EQ(out[0].txn, 7); EXPECT_EQ(out[1].txn, 8);
}

TEST(MbapFramer, BadProtoResync) {
    MbapFramer f;
    const auto good = f.encode(1, std::vector<uint8_t>{0x03,0x00}, 1);
    std::vector<uint8_t> junk = {0xFF, 0xFF, 0xFF};
    junk.insert(junk.end(), good.begin(), good.end());
    std::vector<Adu> out;
    f.feed(junk, out);
    ASSERT_EQ(out.size(), 1u);
}

// ---- channel transact against scripted link ---------------------------------

TEST(RtuChannel, TransactHappy) {
    auto rsp = rtuFrame(1, 0x03, {0x02, 0x00, 0x2B});
    auto link = std::make_unique<ScriptedLink>();
    link->push(rsp);
    RtuChannel ch(std::move(link), kFast);
    ASSERT_TRUE(ch.open().has_value());
    auto r = ch.transact(1, std::vector<uint8_t>{0x03,0,0,0,1}, 500);
    ASSERT_TRUE(r.has_value());
    ASSERT_EQ(r->pdu.size(), 4u);
    EXPECT_EQ(r->pdu[2], 0x00); EXPECT_EQ(r->pdu[3], 0x2B);
}

TEST(RtuChannel, TimeoutOnNoResponse) {
    auto link = std::make_unique<ScriptedLink>();
    RtuChannel ch(std::move(link), kFast);
    ASSERT_TRUE(ch.open().has_value());
    auto r = ch.transact(1, std::vector<uint8_t>{0x03,0,0,0,1}, 80);
    ASSERT_FALSE(r.has_value());
    EXPECT_TRUE(r.error().isTimeout());
}

TEST(RtuChannel, CorruptCrcTimeout) {
    auto rsp = rtuFrame(1, 0x03, {0x02, 0x00, 0x2B});
    rsp.back() ^= 0xFF;                     // corrupt CRC
    auto link = std::make_unique<ScriptedLink>();
    link->push(rsp);
    RtuChannel ch(std::move(link), kFast);
    ASSERT_TRUE(ch.open().has_value());
    auto r = ch.transact(1, std::vector<uint8_t>{0x03,0,0,0,1}, 200);
    ASSERT_FALSE(r.has_value());
    // A CRC-failed frame is distinguishable from silence: Corrupt, not
    // TimedOut — error policies retry corruption differently.
    EXPECT_TRUE(r.error().isCorrupt());
    EXPECT_FALSE(r.error().isTimeout());
}

TEST(RtuChannel, LostByteTimeout) {
    auto rsp = rtuFrame(1, 0x03, {0x02, 0x00, 0x2B});
    rsp.erase(rsp.begin() + 3);             // middle byte lost
    auto link = std::make_unique<ScriptedLink>();
    link->push(rsp);
    RtuChannel ch(std::move(link), kFast);
    ASSERT_TRUE(ch.open().has_value());
    auto r = ch.transact(1, std::vector<uint8_t>{0x03,0,0,0,1}, 200);
    ASSERT_FALSE(r.has_value());
}

TEST(RtuChannel, WrongUnitIgnoredThenRight) {
    auto wrong = rtuFrame(7, 0x03, {0x02, 0xAA, 0xAA});
    auto right = rtuFrame(1, 0x03, {0x02, 0x00, 0x2B});
    auto link = std::make_unique<ScriptedLink>();
    link->push(wrong); link->push(right);
    RtuChannel ch(std::move(link), kFast);
    ASSERT_TRUE(ch.open().has_value());
    auto r = ch.transact(1, std::vector<uint8_t>{0x03,0,0,0,1}, 500);
    ASSERT_TRUE(r.has_value());
    EXPECT_EQ(r->unit, 1);
}

TEST(RtuChannel, ExceptionResponsePropagates) {
    auto rsp = rtuFrame(1, 0x83, {0x02});   // fc3 + 0x80, exc 02
    auto link = std::make_unique<ScriptedLink>();
    link->push(rsp);
    RtuChannel ch(std::move(link), kFast);
    ASSERT_TRUE(ch.open().has_value());
    auto r = ch.transact(1, std::vector<uint8_t>{0x03,0,0,0,1}, 500);
    ASSERT_TRUE(r.has_value());
    EXPECT_EQ(r->pdu[0], 0x83);
}

TEST(MbapChannel, TransactMatchesTxn) {
    MbapFramer mk;
    std::vector<uint8_t> rsp;
    {
        const auto f1 = mk.encode(1, std::vector<uint8_t>{0x03,0x02,0x00,0x01}, 5); // wrong txn
        const auto f2 = mk.encode(1, std::vector<uint8_t>{0x03,0x02,0x00,0x2B}, 1); // ours
        rsp.insert(rsp.end(), f1.begin(), f1.end());
        rsp.insert(rsp.end(), f2.begin(), f2.end());
    }
    auto link = std::make_unique<ScriptedLink>();
    link->push(rsp);
    MbapChannel ch(std::move(link));
    ASSERT_TRUE(ch.open().has_value());
    auto r = ch.transact(1, std::vector<uint8_t>{0x03,0,0,0,1}, 500);
    ASSERT_TRUE(r.has_value());
    EXPECT_EQ(r->pdu[3], 0x2B);
}

TEST(MbapChannel, Timeout) {
    auto link = std::make_unique<ScriptedLink>();
    MbapChannel ch(std::move(link));
    ASSERT_TRUE(ch.open().has_value());
    auto r = ch.transact(1, std::vector<uint8_t>{0x03,0,0,0,1}, 60);
    ASSERT_FALSE(r.has_value());
    EXPECT_TRUE(r.error().isTimeout());
}

TEST(ScriptDsl, Parses) {
    auto steps = parseScript("push 01 02 | drop 1 | stall 5ms | close");
    ASSERT_EQ(steps.size(), 4u);
}

// ---- SimLink: wire-level simulated bus --------------------------------------
#include "tether/modbus/sim/Sim.hpp"

namespace {
/// SimBus with one slave (unit 1, HR[0]=0xBEEF) on a wire-level link.
struct SimRtuRig {
    SimBus bus;
    SimRtuRig() {
        auto& s = bus.add(1);
        (void)s.store().declare(TableId::HoldingRegister, 0, 8);
        s.preload(0, {0xBEEF});
    }
};
}

TEST(SimLink, RtuHappyPath) {
    SimRtuRig rig;
    RtuChannel ch(std::make_unique<SimLink>(rig.bus, true), kFast);
    ASSERT_TRUE(ch.open().has_value());
    auto r = ch.transact(1, std::vector<uint8_t>{0x03, 0, 0, 0, 1}, 500);
    ASSERT_TRUE(r.has_value()) << r.error().format();
    EXPECT_EQ(r->unit, 1);
    ASSERT_EQ(r->pdu.size(), 4u);        // fc + byteCnt + 2 data
    EXPECT_EQ((r->pdu[2] << 8) | r->pdu[3], 0xBEEF);
}

/// Every response-byte position lost → CRC/unit mismatch → timeout.
TEST(SimLink, RtuByteLossMatrix) {
    // FC3 response to qty=1 read: unit+fc+byteCnt+data[2]+crc[2] = 7B
    for (int pos = 0; pos < 7; ++pos) {
        SimRtuRig rig;
        SimLinkTraits t;
        t.mutateResponse = [pos](std::vector<uint8_t>& f) {
            if (pos < int(f.size())) f.erase(f.begin() + pos);
        };
        RtuChannel ch(
            std::make_unique<SimLink>(rig.bus, true, t), kFast);
        ASSERT_TRUE(ch.open().has_value()) << "pos " << pos;
        auto r = ch.transact(1,
            std::vector<uint8_t>{0x03, 0, 0, 0, 1}, 200);
        EXPECT_FALSE(r.has_value())
            << "lost byte at pos " << pos << " should not decode";
    }
}

TEST(SimLink, RtuCorruptThenRecover) {
    SimRtuRig rig;
    auto link = std::make_unique<SimLink>(rig.bus, true);
    auto* lp = link.get();
    lp->traits().corruptByte = 1.0;
    RtuChannel ch(std::move(link), kFast);
    ASSERT_TRUE(ch.open().has_value());
    EXPECT_FALSE(
        ch.transact(1, std::vector<uint8_t>{0x03,0,0,0,1}, 200)
            .has_value());
    lp->traits().corruptByte = 0.0;   // heal the wire
    auto r = ch.transact(1, std::vector<uint8_t>{0x03,0,0,0,1}, 300);
    ASSERT_TRUE(r.has_value());
    EXPECT_EQ((r->pdu[2] << 8) | r->pdu[3], 0xBEEF);
}

TEST(SimLink, MbapHappyPath) {
    SimRtuRig rig;
    MbapChannel ch(std::make_unique<SimLink>(rig.bus, false));
    ASSERT_TRUE(ch.open().has_value());
    auto r = ch.transact(1, std::vector<uint8_t>{0x03, 0, 0, 0, 1}, 500);
    ASSERT_TRUE(r.has_value()) << r.error().format();
    EXPECT_EQ((r->pdu[2] << 8) | r->pdu[3], 0xBEEF);
    // MBAP byte loss → MBAP length mismatch → frame never completes.
    rig.bus.find(1);  // keep rig alive
    auto link2 = std::make_unique<SimLink>(rig.bus, false);
    link2->traits().mutateResponse = [](std::vector<uint8_t>& f) {
        f.erase(f.begin() + 8);          // drop a payload byte
    };
    MbapChannel ch2(std::move(link2));
    ASSERT_TRUE(ch2.open().has_value());
    EXPECT_FALSE(
        ch2.transact(1, std::vector<uint8_t>{0x03,0,0,0,1}, 200)
            .has_value());
}

#if defined(TETHER_MODBUS_ASCII)
TEST(AsciiFramer, RoundTripAndCorruption) {
    AsciiFramer f;
    const uint8_t pdu[] = {0x03, 0x00, 0x6B, 0x00, 0x03};
    auto frame = f.encode(0x11, pdu, 0);
    ASSERT_EQ(frame[0], ':');
    ASSERT_GE(frame.size(), 2u);
    EXPECT_EQ(frame[frame.size() - 2], '\r');
    EXPECT_EQ(frame[frame.size() - 1], '\n');

    std::vector<Adu> out;
    f.feed(frame, out);
    ASSERT_EQ(out.size(), 1u);
    EXPECT_EQ(out[0].unit, 0x11);
    EXPECT_EQ(out[0].pdu.size(), 5u);
    EXPECT_EQ(out[0].pdu[0], 0x03);

    // Corrupt a hex digit → LRC fails → dropped, framer recovers.
    AsciiFramer f2;
    std::vector<uint8_t> bad(frame.begin(), frame.end());
    bad[3] = (bad[3] == '0') ? '1' : '0';
    out.clear();
    f2.feed(bad, out);
    EXPECT_TRUE(out.empty());
    out.clear();
    f2.feed(frame, out);
    ASSERT_EQ(out.size(), 1u);

    // Two back-to-back frames + leading noise.
    AsciiFramer f3;
    std::vector<uint8_t> two = {'x', 'y'};
    two.insert(two.end(), frame.begin(), frame.end());
    two.insert(two.end(), frame.begin(), frame.end());
    out.clear();
    f3.feed(two, out);
    EXPECT_EQ(out.size(), 2u);
}
#endif // TETHER_MODBUS_ASCII

/// Probabilistic traits at p=1.0 → deterministic fault paths.
TEST(SimLink, TraitPaths) {
    using T = SimLinkTraits;
    auto run = [](T t, int timeoutMs) {
        SimRtuRig rig;
        RtuChannel ch(std::make_unique<SimLink>(rig.bus, true, t), kFast);
        if (!ch.open().has_value()) return Expected<Adu>{};
        return ch.transact(1, std::vector<uint8_t>{0x03,0,0,0,1},
                           timeoutMs);
    };
    { T t; t.dropRequest = 1.0;
      EXPECT_FALSE(run(t, 200).has_value()) << "dropped request"; }
    { T t; t.dropResponse = 1.0;
      EXPECT_FALSE(run(t, 200).has_value()) << "dropped response"; }
    { T t; t.dupByte = 1.0;
      EXPECT_FALSE(run(t, 200).has_value()) << "dup byte breaks CRC"; }
    { T t; t.dropByte = 1.0;
      EXPECT_FALSE(run(t, 200).has_value()) << "lost byte breaks CRC"; }
    { T t; t.dribbleBytes = 1;             // byte-wise delivery still works
      EXPECT_TRUE(run(t, 500).has_value()) << "dribbled response"; }
    // turnaround applies inside write() — before the channel's response
    // deadline — so a slow device still answers, just later.
    { T t; t.turnaroundMs = 60;
      EXPECT_TRUE(run(t, 500).has_value()) << "slow device → ok"; }
}

/// stop() with a blocked invoke → cancelled, not hung.
TEST(SimLink, TxnManagerCancel) {
    SimRtuRig rig;
    SimLinkTraits t;
    t.dropResponse = 1.0;                 // response never arrives
    MbapChannel ch(std::make_unique<SimLink>(rig.bus, false, t));
    TransactionManager tm(ch, 4);
    ASSERT_TRUE(tm.start().has_value());

    std::atomic<bool> done{false};
    Expected<Pdu> res;
    std::thread th([&] {
        res = tm.invoke(1, FunctionCode::ReadHoldingRegisters,
                        std::vector<uint8_t>{0,0,0,1}, {.timeoutMs = 30000});
        done = true;
    });
    while (tm.inflight() == 0)
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    tm.stop();                            // must wake the blocked invoke
    th.join();
    EXPECT_TRUE(done);
    ASSERT_FALSE(res.has_value())
        << "stopped manager must fail the invoke, not hang";
}

// ---- hardware parity/framing-error handling -------------------------------
TEST(ParityCheck, Unmarker) {
    using V = std::vector<uint8_t>;
    // Literal FF doubling: marked FF FF → clean FF.
    size_t errs = 99;
    auto clean = ParityUnmarker::unmark(V{0x01, 0xFF, 0xFF, 0x02}, &errs);
    EXPECT_EQ(clean, (V{0x01, 0xFF, 0x02}));
    EXPECT_EQ(errs, 0u);

    // FF 00 X → X kept, counted as errored byte.
    clean = ParityUnmarker::unmark(V{0x01, 0xFF, 0x00, 0xAA, 0x02}, &errs);
    EXPECT_EQ(clean, (V{0x01, 0xAA, 0x02}));
    EXPECT_EQ(errs, 1u);

    // Mark straddling feeds: [01 FF][00 AA 02] → same result.
    ParityUnmarker u;
    V out;
    errs = u.feed(V{0x01, 0xFF}, out);
    errs += u.feed(V{0x00, 0xAA, 0x02}, out);
    EXPECT_EQ(out, (V{0x01, 0xAA, 0x02}));
    EXPECT_EQ(errs, 1u);

    // FF at stream end alone → pending, resolved next feed (FF = literal).
    out.clear();
    errs = u.feed(V{0xFF}, out);
    errs += u.feed(V{0xFF, 0x09}, out);
    EXPECT_EQ(out, (V{0xFF, 0x09}));
    EXPECT_EQ(errs, 0u);
}

TEST(ParityCheck, TermiosFlagsApplied) {
    auto pp = PtyPair::open();
    ASSERT_TRUE(pp.has_value());
    SerialConfig cfg;
    cfg.parity = Parity::Even;
    cfg.device = (*pp)->slavePath();
    for (auto [mode, wantInpck, wantIgnpar, wantParmrk] : {
             std::tuple{ParityCheck::Passthrough, false, false, false},
             std::tuple{ParityCheck::Drop,        true,  true,  false},
             std::tuple{ParityCheck::Mark,        true,  false, true}}) {
        OpenOpts o;
        o.parityCheck = mode;
        o.exclusive = false;  // PtyPair holds slaveFd_; TIOCEXCL persists
        SerialPortLink link(cfg, o);
        ASSERT_TRUE(link.open().has_value()) << int(mode);
        termios t{};
        ASSERT_EQ(::tcgetattr(link.fd(), &t), 0);
        EXPECT_EQ(bool(t.c_iflag & INPCK), wantInpck) << int(mode);
        EXPECT_EQ(bool(t.c_iflag & IGNPAR), wantIgnpar) << int(mode);
        EXPECT_EQ(bool(t.c_iflag & PARMRK), wantParmrk) << int(mode);
        link.close();
    }
    // Parity::None → checking never enabled regardless of mode.
    cfg.parity = Parity::None;
    OpenOpts o;
    o.exclusive = false;
    SerialPortLink link(cfg, o);
    ASSERT_TRUE(link.open().has_value());
    termios t{};
    ASSERT_EQ(::tcgetattr(link.fd(), &t), 0);
    EXPECT_FALSE(t.c_iflag & (INPCK | IGNPAR | PARMRK));
}

TEST(ParityCheck, SerialErrorCountersApi) {
    auto pp = PtyPair::open();
    ASSERT_TRUE(pp.has_value());
    SerialConfig cfg;
    cfg.device = (*pp)->slavePath();
    SerialPortLink link(cfg);
    ASSERT_TRUE(link.open().has_value());
    auto c = link.serialErrors();
    if (c.has_value()) {          // ptys report zeros; real UARTs count
        EXPECT_GE(c->frame, 0);
        EXPECT_GE(c->parity, 0);
    }
    EXPECT_EQ(link.parityMarkedErrors(), 0u);
}

// End-to-end through a real SerialPortLink + RtuChannel on a pty: bytes
// arriving as PARMRK sequences (FF 00 <byte>) or fused into the frame are
// corruption — CRC must reject them; a clean retry must succeed. On a pty
// PARENB cannot be applied, so marks pass raw (unmarking correctly stays
// off) and the framer's CRC is the gate — same path as any serial garbage.
TEST(ParityCheck, MarkedStreamRejectedThenRecovers) {
    auto pp = PtyPair::open();
    ASSERT_TRUE(pp.has_value());
    SerialConfig cfg;
    cfg.parity = Parity::Even;
    cfg.device = (*pp)->slavePath();
    OpenOpts o;
    o.exclusive = false;
    o.parityCheck = ParityCheck::Mark;
    auto link = std::make_unique<SerialPortLink>(cfg, o);
    auto* lp = link.get();
    auto master = (*pp)->masterLink();
    RtuChannel ch(std::move(link), kFast);
    ASSERT_TRUE(ch.open().has_value());
    EXPECT_FALSE(lp->parityApplied());      // pty silently drops PARENB

    // Case 1: mark-style garbage fused into the response (no gap) → the
    // blob's CRC fails → timeout, never accepted as data.
    std::thread resp1([&] {
        std::vector<uint8_t> req(16);
        (void)master->read(req, 1000);      // swallow the request frame
        std::vector<uint8_t> marked{0xFF, 0x00, 0x55, 0xFF, 0x00, 0x01};
        ASSERT_TRUE(master->write(marked, 200).has_value());
        auto frame = rtuFrame(1, 0x03, {0x02, 0x00, 0x2B});
        (void)master->write(frame, 200);    // fused after garbage → bad CRC
    });
    auto r1 = ch.transact(1, std::vector<uint8_t>{0x03,0,0,0,1}, 500);
    resp1.join();
    EXPECT_FALSE(r1.has_value());
    EXPECT_EQ(lp->parityMarkedErrors(), 0u); // no marks on a non-marking port

    // Case 2: marked garbage as its own gapped blob (CRC fails, dropped),
    // then a clean frame → accepted.
    std::thread resp2([&] {
        std::vector<uint8_t> req(16);
        (void)master->read(req, 1000);
        std::vector<uint8_t> marked{0xFF, 0x00, 0x55};
        ASSERT_TRUE(master->write(marked, 200).has_value());
        std::this_thread::sleep_for(std::chrono::milliseconds(30)); // > t3.5
        auto good = rtuFrame(1, 0x03, {0x02, 0x00, 0x2B});
        ASSERT_TRUE(master->write(good, 200).has_value());
    });
    auto r2 = ch.transact(1, std::vector<uint8_t>{0x03,0,0,0,1}, 800);
    resp2.join();
    ASSERT_TRUE(r2.has_value());
    ASSERT_EQ(r2->pdu.size(), 4u);
    EXPECT_EQ(r2->pdu[3], 0x2B);
}
