// End-to-end: client↔server over pty RTU and real TCP loopback;
// dispatcher exception paths; broadcast silence.
#include <gtest/gtest.h>

#include "tether/modbus/client/BusExecutor.hpp"
#include "tether/modbus/client/ModbusClient.hpp"
#include "tether/modbus/client/TransactionManager.hpp"
#include "tether/modbus/link/TestLinks.hpp"
#include "tether/modbus/link/TcpLink.hpp"
#include <memory>
#include "tether/modbus/server/Server.hpp"

using namespace tether::modbus;
using namespace std::chrono_literals;

namespace {
std::unique_ptr<DataStore> makeStore() {
    auto sp = std::make_unique<DataStore>();
    DataStore& s = *sp;
    EXPECT_TRUE(s.declare(TableId::HoldingRegister, 0, 100).has_value());
    EXPECT_TRUE(s.declare(TableId::Coil, 0, 64).has_value());
    EXPECT_TRUE(s.declare(TableId::InputRegister, 0, 32).has_value());
    EXPECT_TRUE(s.declare(TableId::DiscreteInput, 0, 32).has_value());
    return sp;
}
const RtuTiming kFast = RtuTiming::forParams(1000000, 10);
}

TEST(Dispatcher, ReadHoldingAndWrite) {
    auto storePtr = makeStore();
    auto& store = *storePtr;
    ASSERT_TRUE(store.setReg(TableId::HoldingRegister, 5, 0x1234));
    RequestDispatcher d(store);
    auto req = build::readRange(FunctionCode::ReadHoldingRegisters, 5, 1);
    auto res = d.dispatch(1, req->bytes());
    ASSERT_TRUE(res.respond);
    EXPECT_EQ(res.pdu[0], 0x03);
    EXPECT_EQ(res.pdu[1], 2);
    EXPECT_EQ((res.pdu[2] << 8) | res.pdu[3], 0x1234);
}

TEST(Dispatcher, IllegalAddressException) {
    auto storePtr = makeStore();
    auto& store = *storePtr;
    RequestDispatcher d(store);
    auto req = build::readRange(FunctionCode::ReadHoldingRegisters, 500, 1);
    auto res = d.dispatch(1, req->bytes());
    ASSERT_TRUE(res.respond);
    EXPECT_EQ(res.pdu[0], 0x83);
    EXPECT_EQ(res.pdu[1], 0x02);
}

TEST(Dispatcher, BroadcastSilent) {
    auto storePtr = makeStore();
    auto& store = *storePtr;
    RequestDispatcher d(store);
    auto req = build::writeSingleReg(3, 0x77);
    auto res = d.dispatch(0, req->bytes());
    EXPECT_FALSE(res.respond);
    auto v = store.getReg(TableId::HoldingRegister, 3);
    ASSERT_TRUE(v.has_value());
    EXPECT_EQ(*v, 0x77);
}

TEST(Dispatcher, WriteThenReadCoil) {
    auto storePtr = makeStore();
    auto& store = *storePtr;
    RequestDispatcher d(store);
    auto w = build::writeSingleCoil(10, true);
    ASSERT_TRUE(d.dispatch(1, w->bytes()).respond);
    auto r = build::readRange(FunctionCode::ReadCoils, 10, 1);
    auto res = d.dispatch(1, r->bytes());
    ASSERT_TRUE(res.respond);
    EXPECT_EQ(res.pdu[2] & 1, 1);
}

TEST(Dispatcher, ReadOnlyTableWriteRejected) {
    auto storePtr = makeStore();
    auto& store = *storePtr;
    RequestDispatcher d(store);
    auto w = build::writeSingleReg(0, 1);      // FC6 → holding (writable)
    ASSERT_TRUE(d.dispatch(1, w->bytes()).respond);
    // FC6 on input register isn't a thing on the wire; illegal-address
    // path is exercised via multi-reg write to DI-adjacent gap instead:
    auto w2 = build::writeMultiRegs(300, std::vector<uint16_t>{1});
    auto res = d.dispatch(1, w2->bytes());
    EXPECT_EQ(res.pdu[1], 0x02);
}

// ---- RTU over pty -------------------------------------------------------------
TEST(EndToEnd, RtuOverPty) {
    auto pty = PtyPair::open(SerialConfig{});
    ASSERT_TRUE(pty.has_value());
    auto storePtr = makeStore();
    auto& store = *storePtr;
    ASSERT_TRUE(store.setReg(TableId::HoldingRegister, 0, 4242));
    RequestDispatcher d(store);
    RtuChannel serverCh((*pty)->slaveLink(), kFast);
    ModbusServer server(std::make_unique<RtuChannel>(std::move(serverCh)),
                        d);
    ASSERT_TRUE(server.start().has_value());

    RtuChannel clientCh((*pty)->masterLink(), kFast);
    ASSERT_TRUE(clientCh.open().has_value());
    BusExecutor ex(clientCh);
    ModbusClient cli(ex, 1, TransactionOptions{500, 2, 0, {}});

    auto v = cli.readHoldingOne(0);
    ASSERT_TRUE(v.has_value()) << v.error().format();
    EXPECT_EQ(*v, 4242);

    ASSERT_TRUE(cli.writeReg(7, 0xBEEF).has_value());
    auto back = store.getReg(TableId::HoldingRegister, 7);
    ASSERT_TRUE(back.has_value());
    EXPECT_EQ(*back, 0xBEEF);

    auto err = cli.readHolding(400, 1);
    ASSERT_FALSE(err.has_value());
    EXPECT_TRUE(err.error().isException());

    server.stop();
}

// ---- TCP loopback ---------------------------------------------------------------
TEST(EndToEnd, TcpLoopback) {
    auto storePtr = makeStore();
    auto& store = *storePtr;
    ASSERT_TRUE(store.setReg(TableId::HoldingRegister, 2, 0x0D15EA5E & 0xFFFF));
    RequestDispatcher d(store);
    const uint16_t port = 15521;
    auto listener =
        std::make_unique<TcpListener>("127.0.0.1", port, 4, true);
    TcpServer server(std::move(listener), d);
    ASSERT_TRUE(server.start().has_value());
    std::this_thread::sleep_for(50ms);

    auto m = makeTcpMaster("127.0.0.1", port, 1, 2000,
                           TransactionOptions{1000, 1, 0, {}});
    ASSERT_TRUE(m.has_value()) << m.error().format();
    auto v = m->client->readHoldingOne(2);
    ASSERT_TRUE(v.has_value()) << v.error().format();
    EXPECT_EQ(*v, 0xEA5E);

    ASSERT_TRUE(m->client->writeReg(3, 0xCAFE).has_value());
    auto back = store.getReg(TableId::HoldingRegister, 3);
    ASSERT_TRUE(back.has_value());
    EXPECT_EQ(*back, 0xCAFE);

    server.stop();
}

// ---- TCP pipelining via TransactionManager --------------------------------------
namespace {
/// IDataModel wrapper: delays every read — makes pipelining observable.
class SlowStore final : public IDataModel {
public:
    SlowStore(DataStore& inner, int delayMs)
        : inner_(inner), delayMs_(delayMs) {}
    bool covers(TableId t, uint16_t a, uint16_t c) const override {
        return inner_.covers(t, a, c);
    }
    Expected<void> read(TableId t, uint16_t a, uint16_t c,
                        std::span<uint16_t> r,
                        std::span<uint8_t> b) override {
        std::this_thread::sleep_for(std::chrono::milliseconds(delayMs_));
        return inner_.read(t, a, c, r, b);
    }
    Expected<void> write(TableId t, uint16_t a, uint16_t c,
                         std::span<const uint16_t> r,
                         std::span<const uint8_t> b) override {
        return inner_.write(t, a, c, r, b);
    }
private:
    DataStore& inner_;
    int        delayMs_;
};
}

TEST(EndToEnd, TcpPipelinedTransactionManager) {
    DataStore store;
    ASSERT_TRUE(store.declare(TableId::HoldingRegister, 0, 100)
                    .has_value());
    ASSERT_TRUE(store.setReg(TableId::HoldingRegister, 7, 0xBEEF));
    SlowStore slow(store, 30);
    RequestDispatcher d(slow);
    const uint16_t port = 15523;
    TcpServer server(
        std::make_unique<TcpListener>("127.0.0.1", port, 4, true), d);
    ASSERT_TRUE(server.start().has_value());
    std::this_thread::sleep_for(50ms);

    auto link = std::make_unique<TcpLink>("127.0.0.1", port, 2000);
    MbapChannel ch(std::move(link));
    TransactionManager tm(ch, /*maxInflight=*/8);
    ASSERT_TRUE(tm.start().has_value());
    ModbusClient client(tm, 1);

    constexpr int kThreads = 8;
    std::atomic<int> ok{0}, failed{0}, maxInflight{0};
    std::atomic<bool> watch{true};
    std::thread watcher([&] {
        while (watch) {
            int v = tm.inflight();
            int old = maxInflight.load();
            while (v > old && !maxInflight.compare_exchange_weak(old, v))
                ;
            std::this_thread::sleep_for(1ms);
        }
    });

    std::vector<std::thread> ts;
    for (int i = 0; i < kThreads; ++i)
        ts.emplace_back([&] {
            auto v = client.readHoldingOne(7);
            (v && *v == 0xBEEF ? ok : failed).fetch_add(1);
        });
    for (auto& t : ts) t.join();
    watch = false;
    watcher.join();

    EXPECT_EQ(ok.load(), kThreads);
    EXPECT_EQ(failed.load(), 0);
    // Pipelining invariant: >1 txn on the wire concurrently — the
    // 30ms server-side delay guarantees a window where queued
    // requests are in flight while earlier ones are processed.
    EXPECT_GT(maxInflight.load(), 1);
    EXPECT_EQ(tm.inflight(), 0);   // all slots released

    tm.stop();
    server.stop();
}

#ifdef TETHER_MODBUS_UDP
#include "tether/modbus/link/UdpLink.hpp"
#include <arpa/inet.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

// In-process Modbus/UDP responder: recv datagram → dispatch → reply.
namespace {
class UdpResponder {
public:
    UdpResponder(RequestDispatcher& d, uint16_t port) : d_(d) {
        fd_ = ::socket(AF_INET, SOCK_DGRAM, 0);
        sockaddr_in a{};
        a.sin_family = AF_INET;
        a.sin_port = htons(port);
        a.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
        if (::bind(fd_, reinterpret_cast<sockaddr*>(&a), sizeof a) < 0)
            throw std::runtime_error("udp bind");
        th_ = std::thread([this] { loop(); });
    }
    ~UdpResponder() {
        stop_ = true;
        if (th_.joinable()) th_.join();
        ::close(fd_);
    }
private:
    void loop() {
        uint8_t buf[512];
        while (!stop_) {
            pollfd p{fd_, POLLIN, 0};
            if (::poll(&p, 1, 50) <= 0) continue;
            sockaddr_in from{};
            socklen_t fl = sizeof from;
            const ssize_t n = ::recvfrom(fd_, buf, sizeof buf, 0,
                reinterpret_cast<sockaddr*>(&from), &fl);
            if (n < 7 || stop_) continue;
            // MBAP: txn(2) proto(2) len(2) unit(1) pdu…
            std::vector<uint8_t> pdu(buf + 7, buf + n);
            auto res = d_.dispatch(buf[6], pdu);
            if (!res.respond || res.pdu.empty()) continue;
            std::vector<uint8_t> out(buf, buf + 7);
            const uint16_t len = uint16_t(res.pdu.size() + 1);
            out[4] = uint8_t(len >> 8); out[5] = uint8_t(len);
            out.insert(out.end(), res.pdu.begin(), res.pdu.end());
            (void)::sendto(fd_, out.data(), out.size(), 0,
                reinterpret_cast<sockaddr*>(&from), fl);
        }
    }
    RequestDispatcher& d_;
    int fd_ = -1;
    std::thread th_;
    std::atomic<bool> stop_{false};
};
}

TEST(EndToEnd, UdpLoopback) {
    auto storePtr = makeStore();
    auto& store = *storePtr;
    ASSERT_TRUE(store.setReg(TableId::HoldingRegister, 5, 0x1234));
    RequestDispatcher d(store);
    const uint16_t port = 15525;
    UdpResponder responder(d, port);

    auto link = makeUdpLink("127.0.0.1", port, 2000);
    ASSERT_TRUE(link.has_value()) << link.error().format();
    MbapChannel ch(std::move(*link));
    BusExecutor ex(ch);
    ModbusClient client(ex, 1);

    auto v = client.readHoldingOne(5);
    ASSERT_TRUE(v.has_value()) << v.error().format();
    EXPECT_EQ(*v, 0x1234);
}
#endif // TETHER_MODBUS_UDP
