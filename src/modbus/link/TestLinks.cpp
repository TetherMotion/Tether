#include "tether/modbus/link/TestLinks.hpp"

#include <charconv>
#include <chrono>
#include <cstring>
#include <thread>

#if !defined(ESP_PLATFORM)
#include <fcntl.h>
#include <poll.h>
#include <pty.h>
#include <stdlib.h>
#include <unistd.h>

#include "tether/modbus/link/SerialPortLink.hpp"
#endif

namespace tether::modbus {

// ---- LoopbackLink -------------------------------------------------------------
Expected<void> LoopbackLink::write(std::span<const uint8_t> b, int) {
    std::lock_guard g(m_);
    for (uint8_t x : b) q_.push_back(x);
    return {};
}
Expected<size_t> LoopbackLink::read(std::span<uint8_t> buf, int timeoutMs) {
    const auto deadline = std::chrono::steady_clock::now() +
                          std::chrono::milliseconds(timeoutMs);
    for (;;) {
        {
            std::lock_guard g(m_);
            if (!q_.empty()) {
                size_t n = 0;
                while (n < buf.size() && !q_.empty()) {
                    buf[n++] = q_.front(); q_.pop_front();
                }
                return n;
            }
        }
        if (std::chrono::steady_clock::now() >= deadline) return size_t{0};
        std::this_thread::sleep_for(std::chrono::microseconds(200));
    }
}
Expected<void> LoopbackLink::flush(Flush) {
    std::lock_guard g(m_);
    q_.clear();
    return {};
}

// ---- ScriptedLink ---------------------------------------------------------------
ScriptedLink::ScriptedLink(std::vector<Step> steps)
    : steps_(steps.begin(), steps.end()) {}

void ScriptedLink::push(std::vector<uint8_t> bytes) {
    std::lock_guard g(m_);
    // Queued as a Push step → released only after a write() (device-like).
    steps_.push_back(Step{Step::Push, std::move(bytes), 0});
}
void ScriptedLink::pushLive(std::vector<uint8_t> bytes) {
    std::lock_guard g(m_);
    for (uint8_t b : bytes) rx_.push_back(b);
}
void ScriptedLink::stall(int ms) {
    std::lock_guard g(m_);
    stallUntil_ = std::chrono::steady_clock::now().time_since_epoch().count() +
                  int64_t(ms) * 1000000;
}
void ScriptedLink::close() {
    std::lock_guard g(m_);
    open_ = false;
}

Expected<void> ScriptedLink::write(std::span<const uint8_t> b, int) {
    std::lock_guard g(m_);
    txLog_.push_back({b.begin(), b.end()});
    armed_ = true;   // device only "responds" after a request, like hardware
    return {};
}

Expected<size_t> ScriptedLink::read(std::span<uint8_t> buf, int timeoutMs) {
    const auto deadline = std::chrono::steady_clock::now() +
                          std::chrono::milliseconds(timeoutMs);
    for (;;) {
        {
            std::lock_guard g(m_);
            if (!open_) return err<size_t>(ModbusError::transport("closed"));
            const int64_t now =
                std::chrono::steady_clock::now().time_since_epoch().count();
            if (now < stallUntil_ || !armed_) { /* wait */ }
            else if (!steps_.empty()) {
                Step s = std::move(steps_.front());
                steps_.pop_front();
                switch (s.kind) {
                    case Step::Push:
                        for (uint8_t x : s.bytes) rx_.push_back(x);
                        break;
                    case Step::Drop:
                        for (int i = 0; i < s.n && !rx_.empty(); ++i)
                            rx_.pop_front();
                        break;
                    case Step::Stall:
                        stallUntil_ = now + int64_t(s.n) * 1000000;
                        break;
                    case Step::Close:
                        open_ = false;
                        return err<size_t>(ModbusError::transport("closed"));
                }
            }
            if (!rx_.empty()) {
                size_t n = 0;
                while (n < buf.size() && !rx_.empty()) {
                    buf[n++] = rx_.front(); rx_.pop_front();
                }
                return n;
            }
        }
        if (std::chrono::steady_clock::now() >= deadline) return size_t{0};
        std::this_thread::sleep_for(std::chrono::microseconds(200));
    }
}
Expected<void> ScriptedLink::flush(Flush) {
    std::lock_guard g(m_);
    rx_.clear();
    return {};
}

std::vector<ScriptedLink::Step> parseScript(std::string_view text) {
    std::vector<ScriptedLink::Step> steps;
    auto hexByte = [](std::string_view s, uint8_t& out) {
        unsigned v = 0;
        auto [p, ec] = std::from_chars(s.data(), s.data() + s.size(), v, 16);
        if (ec != std::errc{} || p != s.data() + s.size() || v > 0xFF)
            return false;
        out = uint8_t(v);
        return true;
    };
    size_t i = 0;
    auto skipWs = [&] { while (i < text.size() &&
        (text[i] == ' ' || text[i] == '\t' || text[i] == '\n')) ++i; };
    auto word = [&]() {
        skipWs();
        const size_t start = i;
        while (i < text.size() && text[i] != ' ' && text[i] != '\t' &&
               text[i] != '\n' && text[i] != '|') ++i;
        return text.substr(start, i - start);
    };
    while (i < text.size()) {
        skipWs();
        if (i >= text.size()) break;
        if (text[i] == '|') { ++i; continue; }
        if (text[i] == '#') { while (i < text.size() && text[i] != '\n') ++i;
                              continue; }
        const auto w = word();
        if (w == "push") {
            ScriptedLink::Step s{ScriptedLink::Step::Push, {}, 0};
            for (;;) {
                const auto t = word();
                if (t.empty() || t == "push" || t == "stall" ||
                    t == "drop" || t == "close") {
                    if (!t.empty()) i -= t.size();
                    break;
                }
                uint8_t b;
                if (!hexByte(t, b)) { i -= t.size(); break; }
                s.bytes.push_back(b);
            }
            steps.push_back(std::move(s));
        } else if (w == "stall") {
            const auto n = word();
            int ms = 0;
            if (n.size() > 2 && n.substr(n.size() - 2) == "ms") {
                std::from_chars(n.data(), n.data() + n.size() - 2, ms);
            } else if (n.size() > 2 && n.substr(n.size() - 2) == "us") {
                int us = 0;
                std::from_chars(n.data(), n.data() + n.size() - 2, us);
                ms = (us + 999) / 1000;
            } else {
                std::from_chars(n.data(), n.data() + n.size(), ms);
            }
            steps.push_back({ScriptedLink::Step::Stall, {}, ms});
        } else if (w == "drop") {
            int n = 0;
            const auto t = word();
            std::from_chars(t.data(), t.data() + t.size(), n);
            steps.push_back({ScriptedLink::Step::Drop, {}, n});
        } else if (w == "close") {
            steps.push_back({ScriptedLink::Step::Close, {}, 0});
        }
        // unknown tokens skipped
    }
    return steps;
}

#if !defined(ESP_PLATFORM)
PtyPair::~PtyPair() {
    if (masterFd_ >= 0) ::close(masterFd_);
    if (slaveFd_ >= 0) ::close(slaveFd_);
}
Expected<std::unique_ptr<PtyPair>>
PtyPair::open(const SerialConfig& cfg) {
    int mfd = -1, sfd = -1;
    char name[128] = {0};
    if (::openpty(&mfd, &sfd, name, nullptr, nullptr) != 0)
        return err<std::unique_ptr<PtyPair>>(
            ModbusError::transport("openpty: " + std::string(strerror(errno)),
                                   errno));
    auto p = std::make_unique<PtyPair>();
    p->masterFd_ = mfd;
    p->slaveFd_ = sfd;
    p->slavePath_ = name;
    if (cfg.baud != 0) {
        if (auto e = SerialPortLink::applyTermios(sfd, cfg); !e)
            return err<std::unique_ptr<PtyPair>>(e.error());
    }
    int f = ::fcntl(mfd, F_GETFL, 0); ::fcntl(mfd, F_SETFL, f | O_NONBLOCK);
    f = ::fcntl(sfd, F_GETFL, 0); ::fcntl(sfd, F_SETFL, f | O_NONBLOCK);
    return p;
}
std::unique_ptr<ILink> PtyPair::masterLink() {
    return std::make_unique<FdLink>(masterFd_, "master");
}
std::unique_ptr<ILink> PtyPair::slaveLink() {
    return std::make_unique<FdLink>(slaveFd_, slavePath_);
}

Expected<void> FdLink::write(std::span<const uint8_t> b, int) {
    size_t off = 0;
    while (off < b.size()) {
        const ssize_t n = ::write(fd_, b.data() + off, b.size() - off);
        if (n > 0) { off += size_t(n); continue; }
        if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
            pollfd pfd{fd_, POLLOUT, 0};
            ::poll(&pfd, 1, 10);
            continue;
        }
        return err(ModbusError::transport("pty write: " +
                   std::string(strerror(errno)), errno));
    }
    return {};
}
Expected<size_t> FdLink::read(std::span<uint8_t> buf, int timeoutMs) {
    pollfd pfd{fd_, POLLIN, 0};
    const int rc = ::poll(&pfd, 1, timeoutMs);
    if (rc < 0)
        return err<size_t>(ModbusError::transport("poll", errno));
    if (rc == 0) return size_t{0};
    const ssize_t n = ::read(fd_, buf.data(), buf.size());
    if (n < 0) {
        if (errno == EAGAIN || errno == EWOULDBLOCK) return size_t{0};
        return err<size_t>(ModbusError::transport("pty read", errno));
    }
    return n > 0 ? size_t(n) : size_t{0};
}
Expected<void> FdLink::flush(Flush) { return {}; }
#endif

Expected<std::unique_ptr<ILink>> makeLoopbackLink() {
    return std::make_unique<LoopbackLink>();
}

} // namespace tether::modbus
