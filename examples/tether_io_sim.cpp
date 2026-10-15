/**
 * @file tether_io_sim.cpp
 * @brief Tether IO simulator server — serves the simulated CiA 402 fleet
 *        (machine profile signals + machine.events.read) plus a few
 *        writable demo parameters and a 1 kHz counter signal, over raw
 *        TCP/SLIP on port 4000.
 *
 *   tether_io_sim [--port 4000] [--bind 127.0.0.1]
 *
 * Connect with:
 *   tether_io_client -u tcp://127.0.0.1:4000 list signals
 *   tether_io_client -u tcp://127.0.0.1:4000 stream sim.counter
 *
 * @copyright Copyright (C) 2025-2026 Tether Authors
 */
#include "tether/io/Server.hpp"
#include "tether/io/TcpTransport.hpp"
#include "tether/io/Registry.hpp"
#include "tether/io/SchemaCatalog.hpp"
#include "tether/io/SchemaDigest.hpp"
#include "tether/io/SimulatedCiA402Fleet.hpp"

#include <arpa/inet.h>
#include <netinet/in.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

#include <argparse/argparse.hpp>

#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <cstring>
#include <thread>

using namespace tether::io;
using namespace tether::io::cia402;

namespace {

std::atomic<bool> g_stop{false};
void onSigint(int) { g_stop = true; }

uint64_t nowUs() {
    return static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count());
}

void simLog(const char* level, const char* msg, ...) {
    va_list ap;
    va_start(ap, msg);
    std::fprintf(stderr, "[sim:%s] ", level);
    std::vfprintf(stderr, msg, ap);
    std::fprintf(stderr, "\n");
    va_end(ap);
}

/// A few writable scalar params so `set`/`get`/`stream` have something
/// mutable to work with. Values live in a shared store.
struct ParamStore {
    double   targetSpeed = 0.0;
    double   kp = 1.0;
    uint32_t mode = 0;
    bool     enable = false;
    std::atomic<uint64_t> counter{0};   // ticks in the updater thread
};

/// The machine-profile manifest only covers the composite roots — plain
/// scalars live in the graph without a slot.  Add dedicated demo scalar
/// nodes (custom keys) to the graph and manifest before installing so the
/// params/signals have real slots.
struct DemoSchemas {
    SchemaGraph graph;
    std::vector<SchemaManifestEntry> manifest;
    // Slots assigned at manifest append time; refs carry the digest.
    SchemaRef f64, u32, boolean, u64;
    uint32_t slotF64 = 0, slotU32 = 0, slotBool = 0, slotU64 = 0;
};

DemoSchemas buildDemoSchemas() {
    DemoSchemas d;
    d.graph = machineProfileSchemaGraph();
    d.manifest = machineProfileManifest(d.graph);

    auto addScalar = [&](uint8_t tag, ValueType t, const char* name,
                         SchemaRef& ref, uint32_t& slot) {
        SchemaNode n;
        n.key = {};
        n.key[0] = 0xE0;                 // demo namespace, no collision
        n.key[1] = tag;
        n.kind = SchemaKind::Scalar;
        n.scalarType = t;
        n.name = name;
        ref = {n.key, computeSchemaDigest(n)};
        slot = static_cast<uint32_t>(d.manifest.size());
        d.graph.nodes.push_back(std::move(n));
        d.manifest.push_back({ref, 1});
    };
    addScalar(1, ValueType::F64, "sim.F64", d.f64, d.slotF64);
    addScalar(2, ValueType::U32, "sim.U32", d.u32, d.slotU32);
    addScalar(3, ValueType::Bool, "sim.Bool", d.boolean, d.slotBool);
    addScalar(4, ValueType::U64, "sim.U64", d.u64, d.slotU64);
    return d;
}

bool registerParams(Registry& reg, const DemoSchemas& d,
                    ParamStore& store) {
    ParamEntry p;
    p.id = 100; p.name = "target_speed"; p.group = "sim";
    p.description = "Writable f64 demo param";
    p.valueType = ValueType::F64;
    p.schema = d.f64;
    p.schemaSlot = d.slotF64;
    p.readFn  = [&](void* dst) { std::memcpy(dst, &store.targetSpeed, 8); };
    p.writeFn = [&](const void* s) { std::memcpy(&store.targetSpeed, s, 8); };
    if (!reg.addParam(p)) return false;

    p.id = 101; p.name = "kp";
    p.description = "Writable f64 gain";
    p.readFn  = [&](void* dst) { std::memcpy(dst, &store.kp, 8); };
    p.writeFn = [&](const void* s) { std::memcpy(&store.kp, s, 8); };
    if (!reg.addParam(p)) return false;

    p.id = 102; p.name = "mode"; p.valueType = ValueType::U32;
    p.description = "Writable u32 mode";
    p.schema = d.u32;
    p.schemaSlot = d.slotU32;
    p.readFn  = [&](void* dst) { std::memcpy(dst, &store.mode, 4); };
    p.writeFn = [&](const void* s) { std::memcpy(&store.mode, s, 4); };
    if (!reg.addParam(p)) return false;

    p.id = 103; p.name = "enable"; p.valueType = ValueType::Bool;
    p.description = "Writable bool";
    p.schema = d.boolean;
    p.schemaSlot = d.slotBool;
    p.readFn  = [&](void* dst) { std::memcpy(dst, &store.enable, 1); };
    p.writeFn = [&](const void* s) { std::memcpy(&store.enable, s, 1); };
    if (!reg.addParam(p)) return false;

    return true;
}

bool registerSignals(Registry& reg, const DemoSchemas& d,
                     ParamStore& store) {
    SignalEntry s;
    s.id = 200; s.name = "counter"; s.group = "sim";
    s.description = "1 kHz monotonic counter (streamable)";
    s.valueType = ValueType::U64;
    s.schema = d.u64;
    s.schemaSlot = d.slotU64;
    s.readFn = [&](void* dst) {
        const uint64_t c = store.counter.load();
        std::memcpy(dst, &c, 8);
    };
    return reg.addSignal(s);
}

// ---------------------------------------------------------------------------
// Minimal RFC 6455 server transport for ws:// E2E — enough for binary,
// unfragmented client frames (what WsClientTransport sends), plus ping/pong
// and close. Not production-grade; just enough to exercise the WS client.
// ---------------------------------------------------------------------------
namespace simws {

std::string sha1b64(const std::string& in) {
    // tiny SHA-1 → base64 (only used for Sec-WebSocket-Accept)
    auto rol = [](uint32_t v, int b) { return (v << b) | (v >> (32 - b)); };
    uint32_t h[5] = {0x67452301, 0xEFCDAB89, 0x98BADCFE, 0x10325476,
                     0xC3D2E1F0};
    std::vector<uint8_t> m(in.begin(), in.end());
    const uint64_t bits = uint64_t(m.size()) * 8;
    m.push_back(0x80);
    while (m.size() % 64 != 56) m.push_back(0);
    for (int i = 7; i >= 0; --i) m.push_back(uint8_t(bits >> (i * 8)));
    for (size_t off = 0; off + 64 <= m.size(); off += 64) {
        uint32_t w[80];
        for (int i = 0; i < 16; ++i)
            w[i] = (uint32_t(m[off + i*4]) << 24) | (uint32_t(m[off+i*4+1]) << 16) |
                   (uint32_t(m[off+i*4+2]) << 8) | uint32_t(m[off+i*4+3]);
        for (int i = 16; i < 80; ++i) w[i] = rol(w[i-3]^w[i-8]^w[i-14]^w[i-16], 1);
        uint32_t a=h[0],b=h[1],c=h[2],d=h[3],e=h[4];
        for (int i = 0; i < 80; ++i) {
            uint32_t f, k;
            if (i < 20)      { f = (b&c)|((~b)&d);        k = 0x5A827999; }
            else if (i < 40) { f = b^c^d;                 k = 0x6ED9EBA1; }
            else if (i < 60) { f = (b&c)|(b&d)|(c&d);     k = 0x8F1BBCDC; }
            else             { f = b^c^d;                 k = 0xCA62C1D6; }
            const uint32_t t = rol(a,5)+f+e+k+w[i];
            e=d; d=c; c=rol(b,30); b=a; a=t;
        }
        h[0]+=a; h[1]+=b; h[2]+=c; h[3]+=d; h[4]+=e;
    }
    std::vector<uint8_t> dig;
    for (uint32_t v : h)
        for (int i = 3; i >= 0; --i) dig.push_back(uint8_t(v >> (i*8)));
    static const char* b64 =
        "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    std::string out;
    size_t i = 0;
    for (; i + 2 < dig.size(); i += 3) {
        uint32_t n = (uint32_t(dig[i])<<16)|(uint32_t(dig[i+1])<<8)|dig[i+2];
        out += b64[(n>>18)&63]; out += b64[(n>>12)&63];
        out += b64[(n>>6)&63];  out += b64[n&63];
    }
    const size_t rem = dig.size() - i;        // 20 % 3 = 2
    if (rem == 1) {
        const uint32_t n = uint32_t(dig[i]) << 16;
        out += b64[(n>>18)&63]; out += b64[(n>>12)&63]; out += "==";
    } else if (rem == 2) {
        const uint32_t n = (uint32_t(dig[i])<<16)|(uint32_t(dig[i+1])<<8);
        out += b64[(n>>18)&63]; out += b64[(n>>12)&63];
        out += b64[(n>>6)&63];  out += '=';
    }
    return out;
}

bool readExact(int fd, uint8_t* buf, size_t len, int timeoutMs) {
    size_t off = 0;
    while (off < len) {
        pollfd p{fd, POLLIN, 0};
        if (poll(&p, 1, timeoutMs) <= 0) return false;
        const auto n = ::recv(fd, buf + off, len - off, 0);
        if (n <= 0) return false;
        off += size_t(n);
    }
    return true;
}

bool writeAll(int fd, const uint8_t* buf, size_t len) {
    size_t off = 0;
    while (off < len) {
        const auto n = ::send(fd, buf + off, len - off, MSG_NOSIGNAL);
        if (n <= 0) return false;
        off += size_t(n);
    }
    return true;
}

class WsTransport : public ITransport {
public:
    explicit WsTransport(int fd) : fd_(fd) {}
    ~WsTransport() override { close(); }

    bool handshake() {
        // read HTTP request headers
        std::string req;
        char c;
        while (req.size() < 16384) {
            if (!readExact(fd_, reinterpret_cast<uint8_t*>(&c), 1, 5000))
                return false;
            req += c;
            if (req.size() >= 4 && req.substr(req.size()-4) == "\r\n\r\n")
                break;
        }
        auto pos = req.find("Sec-WebSocket-Key:");
        if (pos == std::string::npos) return false;
        pos += 18;
        while (pos < req.size() && req[pos] == ' ') ++pos;
        auto end = req.find("\r\n", pos);
        const std::string key = req.substr(pos, end - pos);
        const std::string b64 = sha1b64(
            key + "258EAFA5-E914-47DA-95CA-C5AB0DC85B11");
        const std::string resp =
            "HTTP/1.1 101 Switching Protocols\r\n"
            "Upgrade: websocket\r\n"
            "Connection: Upgrade\r\n"
            "Sec-WebSocket-Accept: " + b64 + "\r\n\r\n";
        return writeAll(fd_, reinterpret_cast<const uint8_t*>(resp.data()),
                        resp.size());
    }

    bool send(const uint8_t* data, size_t len) override {
        std::vector<uint8_t> f;
        f.push_back(0x82);                    // FIN + binary
        if (len < 126)      f.push_back(uint8_t(len));
        else if (len <= 0xFFFF) {
            f.push_back(126);
            f.push_back(uint8_t(len >> 8)); f.push_back(uint8_t(len));
        } else {
            f.push_back(127);
            for (int i = 7; i >= 0; --i) f.push_back(uint8_t(len >> (i*8)));
        }
        f.insert(f.end(), data, data + len);
        return writeAll(fd_, f.data(), f.size());
    }

    size_t receive(uint8_t* buf, size_t maxLen, uint32_t timeoutMs) override {
        std::vector<uint8_t> m;
        if (!receiveMessage(m, timeoutMs)) return 0;
        const size_t n = std::min(m.size(), maxLen);
        std::memcpy(buf, m.data(), n);
        return n;
    }

    bool receiveMessage(std::vector<uint8_t>& out,
                        uint32_t timeoutMs) override {
        // poll for the frame header — timeout is normal (Session uses a
        // streaming-driven wait), not a disconnect.
        pollfd p{fd_, POLLIN, 0};
        const int pr = poll(&p, 1, int(timeoutMs));
        if (pr <= 0) return false;      // timeout or signal — still connected
        while (connected_) {
            uint8_t hdr[2];
            if (!readExact(fd_, hdr, 2, int(timeoutMs)))
                { connected_ = false; return false; }
            const uint8_t op = hdr[0] & 0x0F;
            const bool masked = hdr[1] & 0x80;
            uint64_t len = hdr[1] & 0x7F;
            if (len == 126) { uint8_t e[2];
                if (!readExact(fd_, e, 2, 5000)) return false;
                len = (uint64_t(e[0]) << 8) | e[1];
            } else if (len == 127) { uint8_t e[8];
                if (!readExact(fd_, e, 8, 5000)) return false;
                len = 0; for (auto b : e) len = (len << 8) | b;
            }
            uint8_t mask[4] = {0,0,0,0};
            if (masked && !readExact(fd_, mask, 4, 5000)) return false;
            out.assign(len, 0);
            if (len && !readExact(fd_, out.data(), len, 5000)) return false;
            if (masked) for (uint64_t i = 0; i < len; ++i) out[i] ^= mask[i & 3];
            if (op == 0x9) {           // ping → pong
                std::vector<uint8_t> f{0x8A, uint8_t(len)};
                f.insert(f.end(), out.begin(), out.end());
                writeAll(fd_, f.data(), f.size());
                continue;
            }
            if (op == 0x8) {           // close → echo close
                std::vector<uint8_t> f{0x88, 0};
                writeAll(fd_, f.data(), f.size());
                connected_ = false;
                return false;
            }
            if (op == 0x2 || op == 0x1 || op == 0x0) return true; // data
        }
        return false;
    }

    void close() override {
        if (fd_ >= 0) { ::close(fd_); fd_ = -1; }
        connected_ = false;
    }
    bool isConnected() const override { return connected_ && fd_ >= 0; }

private:
    int fd_ = -1;
    std::atomic<bool> connected_{true};
};

class WsListener : public ITransportServer {
public:
    WsListener(uint16_t port, std::string bind)
        : port_(port), bind_(std::move(bind)) {}
    ~WsListener() override { stop(); }

    bool start() override {
        fd_ = ::socket(AF_INET, SOCK_STREAM, 0);
        if (fd_ < 0) return false;
        int one = 1;
        setsockopt(fd_, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));
        sockaddr_in a{};
        a.sin_family = AF_INET;
        a.sin_port = htons(port_);
        a.sin_addr.s_addr = bind_.empty() ? INADDR_ANY
                                        : inet_addr(bind_.c_str());
        if (::bind(fd_, reinterpret_cast<sockaddr*>(&a), sizeof(a)) < 0 ||
            ::listen(fd_, 4) < 0) { ::close(fd_); fd_ = -1; return false; }
        listening_ = true;
        return true;
    }
    void stop() override { if (fd_ >= 0) { ::close(fd_); fd_ = -1; } }
    bool isListening() const override { return listening_; }

    std::unique_ptr<ITransport> accept() override {
        sockaddr_in ca{}; socklen_t cl = sizeof(ca);
        const int cfd = ::accept(fd_, reinterpret_cast<sockaddr*>(&ca), &cl);
        if (cfd < 0) return nullptr;
        auto t = std::make_unique<WsTransport>(cfd);
        if (!t->handshake()) return nullptr;
        return t;
    }
private:
    uint16_t port_; std::string bind_; int fd_ = -1;
    std::atomic<bool> listening_{false};
};

} // namespace simws

} // namespace

int main(int argc, char** argv) {
    argparse::ArgumentParser prog("tether_io_sim");
    prog.add_argument("--port").default_value(std::string("4000"));
    prog.add_argument("--bind").default_value(std::string("127.0.0.1"));
    prog.add_argument("--ws-port")
        .help("also serve RFC 6455 WebSocket on this port (0 = off)")
        .default_value(std::string("0"));
    try { prog.parse_args(argc, argv); }
    catch (const std::exception& e) {
        std::fprintf(stderr, "%s\n", e.what());
        return 2;
    }
    const uint16_t port = static_cast<uint16_t>(
        std::stoul(prog.get<std::string>("--port")));
    const std::string bind = prog.get<std::string>("--bind");

    signal(SIGINT, onSigint);
    signal(SIGTERM, onSigint);

    // --- catalog + registry -------------------------------------------------
    // Build the machine-profile graph + manifest, then append demo scalar
    // nodes so writable params have real schema slots.
    DemoSchemas demo = buildDemoSchemas();
    SchemaCatalog catalog;
    if (!catalog.install(demo.graph, demo.manifest)) {
        std::fprintf(stderr, "schema install failed\n");
        return 1;
    }

    Registry registry;
    SimulatedCiA402Fleet fleet;
    if (!fleet.registerSignals(registry, catalog)) {
        std::fprintf(stderr, "signal registration failed\n");
        return 1;
    }
    ParamStore store;
    if (!registerParams(registry, demo, store) ||
        !registerSignals(registry, demo, store)) {
        std::fprintf(stderr, "demo param/signal registration failed\n");
        return 1;
    }

    // --- server --------------------------------------------------------------
    ServerConfig cfg;
    cfg.framing = Framing::Slip;
    cfg.timestampFn = nowUs;
    cfg.logFn = simLog;
    cfg.schemaCatalog = &catalog;
    cfg.maxClients = 4;

    Server server(registry,
                  std::make_unique<TcpTransportServer>(port, bind), cfg);
    if (!server.start()) {
        std::fprintf(stderr, "listen %s:%u failed\n", bind.c_str(), port);
        return 1;
    }
    std::printf("tether_io_sim: SLIP/TCP on %s:%u — %zu schema slots\n",
                bind.c_str(), port, catalog.size());

    // Optional WS listener — ws:// E2E coverage for the client.
    const uint16_t wsPort = static_cast<uint16_t>(
        std::stoul(prog.get<std::string>("--ws-port")));
    std::unique_ptr<Server> wsServer;
    if (wsPort) {
        ServerConfig wcfg = cfg;
        wcfg.framing = Framing::None;   // WS frame = whole message
        wsServer = std::make_unique<Server>(
            registry,
            std::make_unique<simws::WsListener>(wsPort, bind), wcfg);
        if (!wsServer->start()) {
            std::fprintf(stderr, "ws listen %s:%u failed\n",
                         bind.c_str(), wsPort);
            return 1;
        }
        std::printf("tether_io_sim: WS on %s:%u\n", bind.c_str(), wsPort);
    }

    // counter ticker ~1 kHz
    std::thread ticker([&] {
        while (!g_stop) {
            ++store.counter;
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    });

    while (!g_stop) std::this_thread::sleep_for(std::chrono::milliseconds(100));

    if (wsServer) wsServer->stop();
    server.stop();
    ticker.join();
    return 0;
}
