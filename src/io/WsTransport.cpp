/**
 * @file WsTransport.cpp
 * @brief Minimal RFC 6455 WebSocket client transport implementation.
 * @copyright Copyright (C) 2025-2026 Tether Authors
 */
#include "tether/io/WsTransport.hpp"
#include "tether/io/TcpTransport.hpp"

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cstring>
#include <random>
#if defined(__linux__)
#include <sys/random.h>
#endif

namespace tether { namespace io {

namespace {

// ---------------------------------------------------------------------------
// Minimal SHA-1 (needed for Sec-WebSocket-Accept verification)
// ---------------------------------------------------------------------------

inline uint32_t rol32(uint32_t v, unsigned n) {
    return (v << n) | (v >> (32 - n));
}

struct Sha1 {
    uint32_t h[5] = {0x67452301, 0xEFCDAB89, 0x98BADCFE, 0x10325476, 0xC3D2E1F0};
    uint64_t totalLen = 0;
    uint8_t tail[64];
    size_t tailLen = 0;

    void block(const uint8_t* p) {
        uint32_t w[80];
        for (int i = 0; i < 16; ++i) {
            w[i] = (static_cast<uint32_t>(p[4 * i]) << 24) |
                   (static_cast<uint32_t>(p[4 * i + 1]) << 16) |
                   (static_cast<uint32_t>(p[4 * i + 2]) << 8) |
                   static_cast<uint32_t>(p[4 * i + 3]);
        }
        for (int i = 16; i < 80; ++i) {
            w[i] = rol32(w[i - 3] ^ w[i - 8] ^ w[i - 14] ^ w[i - 16], 1);
        }
        uint32_t a = h[0], b = h[1], c = h[2], d = h[3], e = h[4];
        for (int i = 0; i < 80; ++i) {
            uint32_t f, k;
            if (i < 20)      { f = (b & c) | (~b & d);          k = 0x5A827999; }
            else if (i < 40) { f = b ^ c ^ d;                    k = 0x6ED9EBA1; }
            else if (i < 60) { f = (b & c) | (b & d) | (c & d);  k = 0x8F1BBCDC; }
            else             { f = b ^ c ^ d;                    k = 0xCA62C1D6; }
            const uint32_t tmp = rol32(a, 5) + f + e + k + w[i];
            e = d; d = c; c = rol32(b, 30); b = a; a = tmp;
        }
        h[0] += a; h[1] += b; h[2] += c; h[3] += d; h[4] += e;
    }

    void update(const uint8_t* data, size_t len) {
        totalLen += len;
        while (len > 0) {
            const size_t n = std::min(len, 64 - tailLen);
            std::memcpy(tail + tailLen, data, n);
            tailLen += n;
            data += n;
            len -= n;
            if (tailLen == 64) { block(tail); tailLen = 0; }
        }
    }

    void finish(uint8_t out[20]) {
        const uint64_t bitLen = totalLen * 8;
        uint8_t pad = 0x80;
        update(&pad, 1);
        pad = 0;
        while (tailLen != 56) update(&pad, 1);
        uint8_t len8[8];
        for (int i = 0; i < 8; ++i) {
            len8[i] = static_cast<uint8_t>(bitLen >> (56 - 8 * i));
        }
        // Bypass totalLen accounting — write the length block directly.
        std::memcpy(tail + tailLen, len8, 8);
        tailLen += 8;
        block(tail);
        for (int i = 0; i < 5; ++i) {
            out[4 * i]     = static_cast<uint8_t>(h[i] >> 24);
            out[4 * i + 1] = static_cast<uint8_t>(h[i] >> 16);
            out[4 * i + 2] = static_cast<uint8_t>(h[i] >> 8);
            out[4 * i + 3] = static_cast<uint8_t>(h[i]);
        }
    }
};

void sha1(const uint8_t* data, size_t len, uint8_t out[20]) {
    Sha1 s;
    s.update(data, len);
    s.finish(out);
}

// ---------------------------------------------------------------------------
// Base64 (encode only — the key is generated locally)
// ---------------------------------------------------------------------------

std::string base64Encode(const uint8_t* data, size_t len) {
    static constexpr char kAlphabet[] =
        "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    std::string out;
    out.reserve(((len + 2) / 3) * 4);
    for (size_t i = 0; i < len; i += 3) {
        const uint32_t b0 = data[i];
        const uint32_t b1 = (i + 1 < len) ? data[i + 1] : 0;
        const uint32_t b2 = (i + 2 < len) ? data[i + 2] : 0;
        const uint32_t v = (b0 << 16) | (b1 << 8) | b2;
        out.push_back(kAlphabet[(v >> 18) & 63]);
        out.push_back(kAlphabet[(v >> 12) & 63]);
        out.push_back((i + 1 < len) ? kAlphabet[(v >> 6) & 63] : '=');
        out.push_back((i + 2 < len) ? kAlphabet[v & 63] : '=');
    }
    return out;
}

void randomBytes(uint8_t* out, size_t len) {
#if defined(__linux__)
    size_t got = 0;
    while (got < len) {
        const ssize_t r = ::getrandom(out + got, len - got, 0);
        if (r > 0) { got += static_cast<size_t>(r); continue; }
        break;
    }
    if (got == len) return;
#endif
    static std::mt19937_64 rng{std::random_device{}()};
    for (size_t i = 0; i < len; ++i) {
        out[i] = static_cast<uint8_t>(rng());
    }
}

std::string toLower(std::string_view s) {
    std::string out(s);
    for (auto& c : out) c = static_cast<char>(std::tolower(
        static_cast<unsigned char>(c)));
    return out;
}

std::string_view trim(std::string_view s) {
    while (!s.empty() && (s.front() == ' ' || s.front() == '\t')) s.remove_prefix(1);
    while (!s.empty() && (s.back() == ' ' || s.back() == '\t' ||
           s.back() == '\r' || s.back() == '\n')) s.remove_suffix(1);
    return s;
}

} // namespace

// ---------------------------------------------------------------------------
// WsClientTransport
// ---------------------------------------------------------------------------

WsClientTransport::WsClientTransport(std::unique_ptr<ITransport> byteTransport)
    : inner_(std::move(byteTransport)) {}

WsClientTransport::~WsClientTransport() {
    close();
}

std::expected<std::unique_ptr<WsClientTransport>, std::string>
WsClientTransport::connect(const char* host, uint16_t port,
                           const Options& opts) {
    auto tcp = tcpConnect(host, port, opts.handshakeTimeoutMs);
    if (!tcp) {
        return std::unexpected(std::string("TCP connect failed to ") +
                               (host ? host : "") + ":" + std::to_string(port));
    }
    Options merged = opts;
    if (merged.host.empty()) merged.host = host;
    auto ws = std::make_unique<WsClientTransport>(std::move(tcp));
    if (auto r = ws->handshake(merged); !r) {
        return std::unexpected(r.error());
    }
    return ws;
}

bool WsClientTransport::readExact(
    uint8_t* dst, size_t n,
    std::chrono::steady_clock::time_point deadline) {
    size_t off = 0;
    while (off < n) {
        const int64_t remain = std::chrono::duration_cast<std::chrono::milliseconds>(
            deadline - std::chrono::steady_clock::now()).count();
        const uint32_t slice = (remain <= 0) ? 0u
            : static_cast<uint32_t>(std::min<int64_t>(remain, UINT32_MAX));
        const size_t r = inner_->receive(dst + off, n - off, slice);
        if (r == 0) {
            if (!inner_->isConnected() || remain <= 0) return false;
            continue;
        }
        off += r;
    }
    return true;
}

std::expected<void, std::string>
WsClientTransport::handshake(const Options& opts) {
    if (!inner_ || !inner_->isConnected()) {
        return std::unexpected(std::string("byte transport not connected"));
    }

    uint8_t keyBytes[16];
    randomBytes(keyBytes, sizeof(keyBytes));
    const std::string key = base64Encode(keyBytes, sizeof(keyBytes));

    std::string req = "GET " + opts.target + " HTTP/1.1\r\n"
        "Host: " + (opts.host.empty() ? std::string("localhost") : opts.host) + "\r\n"
        "Upgrade: websocket\r\n"
        "Connection: Upgrade\r\n"
        "Sec-WebSocket-Key: " + key + "\r\n"
        "Sec-WebSocket-Version: 13\r\n";
    static const char* kReserved[] = {
        "host", "upgrade", "connection", "sec-websocket-"};
    for (const auto& [name, value] : opts.headers) {
        const std::string lower = toLower(name);
        bool reserved = false;
        for (const char* r : kReserved) {
            if (lower.rfind(r, 0) == 0) { reserved = true; break; }
        }
        if (reserved) {
            return std::unexpected(std::string("reserved handshake header: ") + name);
        }
        if (name.empty() || name.find_first_of(":\r\n") != std::string::npos ||
            value.find_first_of("\r\n") != std::string::npos) {
            return std::unexpected(std::string("invalid handshake header: ") + name);
        }
        req += name + ": " + value + "\r\n";
    }
    req += "\r\n";

    if (!inner_->send(reinterpret_cast<const uint8_t*>(req.data()), req.size())) {
        return std::unexpected(std::string("handshake send failed"));
    }

    // Read response headers byte-by-byte until CRLFCRLF (bounded).
    const auto deadline = std::chrono::steady_clock::now() +
        std::chrono::milliseconds(opts.handshakeTimeoutMs);
    std::string header;
    header.reserve(512);
    while (header.size() < 16384) {
        uint8_t c = 0;
        if (!readExact(&c, 1, deadline)) {
            return std::unexpected(std::string("handshake timed out"));
        }
        header.push_back(static_cast<char>(c));
        if (header.size() >= 4 &&
            std::memcmp(header.data() + header.size() - 4, "\r\n\r\n", 4) == 0) {
            break;
        }
    }
    if (header.size() < 4 ||
        std::memcmp(header.data() + header.size() - 4, "\r\n\r\n", 4) != 0) {
        return std::unexpected(std::string("oversized handshake response"));
    }

    // Status line: "HTTP/1.x 101 ..."
    const auto lineEnd = header.find("\r\n");
    const std::string_view statusLine(header.data(),
        lineEnd == std::string::npos ? header.size() : lineEnd);
    // "HTTP/1.x 101 <reason>" — the status code is the first token after
    // the version.
    const auto sp = statusLine.find(' ');
    const std::string_view code =
        (sp == std::string_view::npos)
            ? std::string_view{}
            : trim(statusLine.substr(sp + 1, 3));
    if (statusLine.rfind("HTTP/", 0) != 0 || code != "101") {
        return std::unexpected(std::string("handshake rejected: ") +
                               std::string(statusLine));
    }

    // Expected accept = base64(SHA1(key + GUID)).
    static constexpr char kGuid[] = "258EAFA5-E914-47DA-95CA-C5AB0DC85B11";
    const std::string acceptSrc = key + kGuid;
    uint8_t digest[20];
    sha1(reinterpret_cast<const uint8_t*>(acceptSrc.data()),
         acceptSrc.size(), digest);
    const std::string expectedAccept = base64Encode(digest, sizeof(digest));

    bool acceptOk = false;
    for (size_t pos = (lineEnd == std::string::npos) ? header.size() : lineEnd + 2;
         pos < header.size();) {
        const auto eol = header.find("\r\n", pos);
        if (eol == std::string::npos || eol == pos) break;
        const std::string_view line(header.data() + pos, eol - pos);
        pos = eol + 2;
        const auto colon = line.find(':');
        if (colon == std::string_view::npos) continue;
        const std::string name = toLower(trim(line.substr(0, colon)));
        const std::string_view value = trim(line.substr(colon + 1));
        if (name == "sec-websocket-accept" && value == expectedAccept) {
            acceptOk = true;
        }
    }
    if (!acceptOk) {
        return std::unexpected(std::string("bad Sec-WebSocket-Accept"));
    }

    maxMessageSize_ = opts.maxMessageSize;
    connected_ = true;
    return {};
}

bool WsClientTransport::sendFrame(uint8_t opcode,
                                  const uint8_t* payload, size_t len) {
    if (!connected_ || !inner_ || !inner_->isConnected()) return false;

    std::vector<uint8_t> frame;
    frame.reserve(len + 14);
    frame.push_back(static_cast<uint8_t>(0x80 | opcode));   // FIN + opcode
    if (len < 126) {
        frame.push_back(static_cast<uint8_t>(0x80 | len));
    } else if (len <= 0xFFFF) {
        frame.push_back(0x80 | 126);
        frame.push_back(static_cast<uint8_t>(len >> 8));
        frame.push_back(static_cast<uint8_t>(len));
    } else {
        frame.push_back(0x80 | 127);
        for (int i = 7; i >= 0; --i) {
            frame.push_back(static_cast<uint8_t>(
                static_cast<uint64_t>(len) >> (8 * i)));
        }
    }
    uint8_t mask[4];
    randomBytes(mask, 4);
    frame.insert(frame.end(), mask, mask + 4);
    const size_t maskBase = frame.size();
    if (len != 0) {
        frame.insert(frame.end(), payload, payload + len);
        for (size_t i = 0; i < len; ++i) {
            frame[maskBase + i] ^= mask[i & 3];
        }
    }
    return inner_->send(frame.data(), frame.size());
}

void WsClientTransport::protocolError() {
    if (connected_ && !sentClose_) {
        const uint8_t code[2] = {0x03, 0xEA};   // 1002 protocol error
        sendFrame(0x8, code, 2);
        sentClose_ = true;
    }
    connected_ = false;
    if (inner_) inner_->close();
}

bool WsClientTransport::receiveMessage(std::vector<uint8_t>& out,
                                       uint32_t timeoutMs) {
    if (!connected_ || !inner_ || !inner_->isConnected()) return false;

    // Deliver a queued complete message first (receive() leftovers are
    // handled in receive(), not here).
    const auto deadline = std::chrono::steady_clock::now() +
                          std::chrono::milliseconds(timeoutMs);
    uint8_t hdr[2];
    while (true) {
        if (timeoutMs != 0 && std::chrono::steady_clock::now() >= deadline) {
            return false;
        }
        if (!readExact(hdr, 2,
                       timeoutMs == 0 ? std::chrono::steady_clock::now()
                                      : deadline)) {
            return false;
        }
        const bool fin = (hdr[0] & 0x80) != 0;
        const uint8_t opcode = hdr[0] & 0x0F;
        const bool validOpcode = opcode <= 0x2 || (opcode >= 0x8 && opcode <= 0xA);
        if (!validOpcode) { protocolError(); return false; }
        const bool masked = (hdr[1] & 0x80) != 0;
        uint64_t len = hdr[1] & 0x7F;
        if (len == 126) {
            uint8_t ext[2];
            if (!readExact(ext, 2, deadline)) return false;
            len = (static_cast<uint64_t>(ext[0]) << 8) | ext[1];
        } else if (len == 127) {
            uint8_t ext[8];
            if (!readExact(ext, 8, deadline)) return false;
            len = 0;
            for (uint8_t b : ext) len = (len << 8) | b;
            if (len > SIZE_MAX) { protocolError(); return false; }
        }
        const bool control = opcode >= 0x8;
        if (control && (!fin || len > 125)) { protocolError(); return false; }
        if (opcode == 0x0 && !fragmented_) { protocolError(); return false; }
        if ((opcode == 0x1 || opcode == 0x2) && fragmented_) {
            protocolError();
            return false;
        }
        if (!control && len > maxMessageSize_ - message_.size()) {
            protocolError();
            return false;
        }

        uint8_t mask[4];
        if (masked && !readExact(mask, 4, deadline)) return false;
        std::vector<uint8_t> payload(static_cast<size_t>(len));
        if (len && !readExact(payload.data(), static_cast<size_t>(len), deadline)) {
            return false;
        }
        if (masked) {
            for (size_t i = 0; i < payload.size(); ++i) payload[i] ^= mask[i & 3];
        }

        switch (opcode) {
            case 0x8:   // close
                if (!sentClose_) {
                    sendFrame(0x8, payload.data(), payload.size());
                    sentClose_ = true;
                }
                connected_ = false;
                if (inner_) inner_->close();
                return false;
            case 0x9:   // ping → pong
                if (!sendFrame(0xA, payload.data(), payload.size())) return false;
                continue;
            case 0xA:   // pong — ignore
                continue;
            case 0x0:   // continuation
                message_.insert(message_.end(), payload.begin(), payload.end());
                break;
            case 0x1:   // text — accepted leniently like binary
            case 0x2:   // binary
                message_ = std::move(payload);
                break;
        }
        if (opcode != 0x0) fragmented_ = !fin;
        if (fin) {
            out = std::move(message_);
            message_.clear();
            return true;
        }
        fragmented_ = true;
    }
}

size_t WsClientTransport::receive(uint8_t* buf, size_t maxLen,
                                  uint32_t timeoutMs) {
    if (msgOffset_ >= msgBytes_.size()) {
        msgBytes_.clear();
        msgOffset_ = 0;
        if (!receiveMessage(msgBytes_, timeoutMs)) return 0;
    }
    const size_t n = std::min(maxLen, msgBytes_.size() - msgOffset_);
    std::memcpy(buf, msgBytes_.data() + msgOffset_, n);
    msgOffset_ += n;
    return n;
}

bool WsClientTransport::send(const uint8_t* data, size_t len) {
    return sendFrame(0x2, data, len);
}

void WsClientTransport::close() {
    if (connected_ && !sentClose_) {
        sendFrame(0x8, nullptr, 0);
        sentClose_ = true;
    }
    connected_ = false;
    if (inner_) inner_->close();
}

bool WsClientTransport::isConnected() const {
    return connected_ && inner_ && inner_->isConnected();
}

}} // namespace tether::io
