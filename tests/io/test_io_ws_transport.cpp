/**
 * @file test_io_ws_transport.cpp
 * @brief Tests for WsClientTransport — the minimal RFC 6455 WebSocket
 *        client — against an in-proc test server over PipeTransport.
 */
#include <gtest/gtest.h>
#include "tether/io/WsTransport.hpp"
#include "PipeTransport.hpp"

#include <atomic>
#include <chrono>
#include <cstring>
#include <string>
#include <thread>

using namespace tether::io;
using namespace tether::io::testing;

namespace {

// ---------------------------------------------------------------------------
// Minimal SHA-1 + base64 for the test server (Sec-WebSocket-Accept)
// ---------------------------------------------------------------------------

inline uint32_t rol32(uint32_t v, unsigned n) { return (v << n) | (v >> (32 - n)); }

struct Sha1 {
    uint32_t h[5] = {0x67452301, 0xEFCDAB89, 0x98BADCFE, 0x10325476, 0xC3D2E1F0};
    uint64_t len = 0;
    uint8_t tail[64];
    size_t tailLen = 0;
    void block(const uint8_t* p) {
        uint32_t w[80];
        for (int i = 0; i < 16; ++i)
            w[i] = (uint32_t(p[4*i])<<24)|(uint32_t(p[4*i+1])<<16)|
                   (uint32_t(p[4*i+2])<<8)|uint32_t(p[4*i+3]);
        for (int i = 16; i < 80; ++i)
            w[i] = rol32(w[i-3]^w[i-8]^w[i-14]^w[i-16], 1);
        uint32_t a=h[0],b=h[1],c=h[2],d=h[3],e=h[4];
        for (int i = 0; i < 80; ++i) {
            uint32_t f,k;
            if (i<20){f=(b&c)|(~b&d);k=0x5A827999;}
            else if(i<40){f=b^c^d;k=0x6ED9EBA1;}
            else if(i<60){f=(b&c)|(b&d)|(c&d);k=0x8F1BBCDC;}
            else{f=b^c^d;k=0xCA62C1D6;}
            uint32_t t=rol32(a,5)+f+e+k+w[i];
            e=d;d=c;c=rol32(b,30);b=a;a=t;
        }
        h[0]+=a;h[1]+=b;h[2]+=c;h[3]+=d;h[4]+=e;
    }
    void update(const uint8_t* p, size_t n) {
        len += n;
        while (n) {
            size_t take = std::min(n, 64 - tailLen);
            std::memcpy(tail + tailLen, p, take);
            tailLen += take; p += take; n -= take;
            if (tailLen == 64) { block(tail); tailLen = 0; }
        }
    }
    void finish(uint8_t out[20]) {
        uint64_t bits = len * 8;
        uint8_t one = 0x80, zero = 0;
        update(&one, 1);
        while (tailLen != 56) update(&zero, 1);
        uint8_t l[8];
        for (int i = 0; i < 8; ++i) l[i] = uint8_t(bits >> (56 - 8*i));
        std::memcpy(tail + tailLen, l, 8); tailLen += 8;
        block(tail);
        for (int i = 0; i < 5; ++i) {
            out[4*i]=uint8_t(h[i]>>24); out[4*i+1]=uint8_t(h[i]>>16);
            out[4*i+2]=uint8_t(h[i]>>8); out[4*i+3]=uint8_t(h[i]);
        }
    }
};

std::string b64(const uint8_t* d, size_t n) {
    static const char* A =
        "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    std::string o;
    for (size_t i = 0; i < n; i += 3) {
        uint32_t v = (uint32_t(d[i])<<16) |
            (i+1<n ? uint32_t(d[i+1])<<8 : 0) | (i+2<n ? uint32_t(d[i+2]) : 0);
        o += A[(v>>18)&63]; o += A[(v>>12)&63];
        o += (i+1<n) ? A[(v>>6)&63] : '=';
        o += (i+2<n) ? A[v&63] : '=';
    }
    return o;
}

// ---------------------------------------------------------------------------
// In-proc test WebSocket server over a byte-stream transport
// ---------------------------------------------------------------------------

struct TestWsServer {
    std::unique_ptr<ITransport> t;
    std::atomic<bool> handshakeOk{false};
    std::atomic<bool> sawPong{false};
    std::atomic<bool> sawClose{false};
    std::atomic<bool> rejectHandshake{false};
    std::atomic<int> echoCount{0};
    std::thread thread;

    static std::string getHeader(const std::string& headers,
                               const std::string& name) {
        std::string needle = name + ":";
        auto pos = headers.find(needle);
        if (pos == std::string::npos) return {};
        pos += needle.size();
        while (pos < headers.size() && headers[pos] == ' ') ++pos;
        auto end = headers.find("\r\n", pos);
        return headers.substr(pos, end - pos);
    }

    bool readExact(uint8_t* dst, size_t n) {
        size_t off = 0;
        while (off < n) {
            size_t r = t->receive(dst + off, n - off, 2000);
            if (r == 0) return false;
            off += r;
        }
        return true;
    }

    bool sendFrame(uint8_t opcode, const uint8_t* p, size_t n) {
        std::vector<uint8_t> f;
        f.push_back(0x80 | opcode);
        if (n < 126) f.push_back(uint8_t(n));
        else if (n <= 0xFFFF) {
            f.push_back(126); f.push_back(uint8_t(n>>8)); f.push_back(uint8_t(n));
        } else {
            f.push_back(127);
            for (int i = 7; i >= 0; --i) f.push_back(uint8_t(uint64_t(n)>>(8*i)));
        }
        f.insert(f.end(), p, p + n);
        return t->send(f.data(), f.size());
    }

    void run() {
        // --- handshake ---
        std::string headers;
        while (headers.size() < 16384) {
            uint8_t c;
            if (!readExact(&c, 1)) return;
            headers.push_back(char(c));
            if (headers.size() >= 4 &&
                headers.compare(headers.size()-4, 4, "\r\n\r\n") == 0) break;
        }
        if (rejectHandshake) {
            const char* resp = "HTTP/1.1 403 Forbidden\r\nContent-Length: 0\r\n\r\n";
            t->send(reinterpret_cast<const uint8_t*>(resp), strlen(resp));
            return;
        }
        const std::string key = getHeader(headers, "Sec-WebSocket-Key");
        if (key.empty() ||
            getHeader(headers, "Sec-WebSocket-Version") != "13") {
            return;
        }
        const std::string src = key + "258EAFA5-E914-47DA-95CA-C5AB0DC85B11";
        uint8_t digest[20];
        Sha1 s;
        s.update(reinterpret_cast<const uint8_t*>(src.data()), src.size());
        s.finish(digest);
        const std::string resp =
            "HTTP/1.1 101 Switching Protocols\r\n"
            "Upgrade: websocket\r\n"
            "Connection: Upgrade\r\n"
            "Sec-WebSocket-Accept: " + b64(digest, 20) + "\r\n\r\n";
        if (!t->send(reinterpret_cast<const uint8_t*>(resp.data()),
                     resp.size())) return;
        handshakeOk = true;

        // --- frame loop: echo binary, ping back, track close ---
        bool fragmented = false;
        std::vector<uint8_t> message;
        while (true) {
            uint8_t hdr[2];
            if (!readExact(hdr, 2)) return;
            const bool fin = hdr[0] & 0x80;
            const uint8_t op = hdr[0] & 0x0F;
            const bool masked = hdr[1] & 0x80;
            uint64_t len = hdr[1] & 0x7F;
            if (len == 126) {
                uint8_t e[2]; if (!readExact(e, 2)) return;
                len = (uint64_t(e[0])<<8)|e[1];
            } else if (len == 127) {
                uint8_t e[8]; if (!readExact(e, 8)) return;
                len = 0;
                for (uint8_t b : e) len = (len<<8)|b;
            }
            uint8_t mask[4] = {};
            if (masked && !readExact(mask, 4)) return;
            std::vector<uint8_t> payload(static_cast<size_t>(len));
            if (len && !readExact(payload.data(), size_t(len))) return;
            if (masked)
                for (size_t i = 0; i < payload.size(); ++i)
                    payload[i] ^= mask[i & 3];

            if (op == 0x8) {
                sendFrame(0x8, payload.data(), payload.size());
                sawClose = true;
                return;
            }
            if (op == 0x9) { sendFrame(0xA, payload.data(), payload.size()); continue; }
            if (op == 0xA) { sawPong = true; continue; }
            if (op == 0x0) {
                if (!fragmented) return;
                message.insert(message.end(), payload.begin(), payload.end());
            } else {
                message = std::move(payload);
            }
            if (op != 0x0) fragmented = !fin;
            if (fin) {
                ++echoCount;
                if (!sendFrame(0x2, message.data(), message.size())) return;
                // After the 2nd echo, ping the client to verify pong.
                if (echoCount == 2) {
                    const uint8_t ping[] = {1, 2, 3, 4};
                    sendFrame(0x9, ping, sizeof(ping));
                }
                message.clear();
                fragmented = false;
            }
        }
    }
};

struct ServerFixture {
    std::unique_ptr<TestWsServer> server;
    std::unique_ptr<WsClientTransport> client;
    ServerFixture() {
        auto [clientEnd, serverEnd] = PipeTransport::create();
        server = std::make_unique<TestWsServer>();
        server->t = std::move(serverEnd);
        server->thread = std::thread([this] { server->run(); });
        client = std::make_unique<WsClientTransport>(std::move(clientEnd));
    }
    ~ServerFixture() {
        if (client) client->close();
        if (server && server->thread.joinable()) server->thread.join();
    }
};

TEST(IOWsTransport, HandshakeAndEchoRoundTrip) {
    ServerFixture f;
    WsClientTransport::Options opts;
    opts.host = "127.0.0.1:8080";
    opts.target = "/tether-io";
    opts.headers = {{"Authorization", "Bearer tok123"}};
    auto hs = f.client->handshake(opts);
    ASSERT_TRUE(hs) << hs.error();
    ASSERT_TRUE(f.client->isConnected());
    EXPECT_TRUE(f.server->handshakeOk);

    // Small payload (short form length).
    const std::string m1 = "hello";
    ASSERT_TRUE(f.client->send(reinterpret_cast<const uint8_t*>(m1.data()),
                             m1.size()));
    std::vector<uint8_t> out;
    ASSERT_TRUE(f.client->receiveMessage(out, 2000));
    EXPECT_EQ(std::string(out.begin(), out.end()), m1);

    // 200-byte payload (16-bit length) + 70k payload (64-bit length).
    std::vector<uint8_t> m2(200, 0xAB);
    ASSERT_TRUE(f.client->send(m2.data(), m2.size()));
    ASSERT_TRUE(f.client->receiveMessage(out, 2000));
    EXPECT_EQ(out, m2);

    // The server pinged us after the 2nd echo; a synchronous client only
    // answers the ping while pumping (next receiveMessage) — verify the
    // pong arrived after the following exchange.
    std::vector<uint8_t> m3(70000, 0x5A);
    ASSERT_TRUE(f.client->send(m3.data(), m3.size()));
    ASSERT_TRUE(f.client->receiveMessage(out, 5000));
    EXPECT_EQ(out.size(), m3.size());
    EXPECT_EQ(out, m3);
    for (int i = 0; i < 50 && !f.server->sawPong; ++i)
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    EXPECT_TRUE(f.server->sawPong);
}

TEST(IOWsTransport, RejectsBadHandshakeAndReservedHeaders) {
    {
        ServerFixture f;
        f.server->rejectHandshake = true;
        WsClientTransport::Options opts;
        opts.host = "h";
        EXPECT_FALSE(f.client->handshake(opts));
    }
    {
        ServerFixture f;
        f.server->rejectHandshake = true;   // don't even finish handshake
        WsClientTransport::Options opts;
        opts.host = "h";
        opts.headers = {{"Upgrade", "evil"}};
        auto r = f.client->handshake(opts);
        EXPECT_FALSE(r);
        EXPECT_NE(r.error().find("reserved"), std::string::npos);
    }
}

TEST(IOWsTransport, CloseHandshakeDisconnects) {
    ServerFixture f;
    WsClientTransport::Options opts;
    opts.host = "h";
    ASSERT_TRUE(f.client->handshake(opts));
    f.client->close();
    for (int i = 0; i < 50 && !f.server->sawClose; ++i)
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    EXPECT_TRUE(f.server->sawClose);
    EXPECT_FALSE(f.client->isConnected());
}

} // namespace
