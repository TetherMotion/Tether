/**
 * @file TestLinks.hpp
 * @brief LoopbackLink, ScriptedLink, PtyPair — test/diagnostic links.
 */
#pragma once

#include <deque>
#include <mutex>
#include <vector>

#include "tether/modbus/link/ILink.hpp"
#include "tether/modbus/link/SerialConfig.hpp"

namespace tether::modbus {

/**@brief Echoes TX bytes back on RX verbatim. */
class LoopbackLink final : public ILink {
public:
    Expected<void>   open() override { open_ = true; return {}; }
    void             close() override { open_ = false; }
    bool             isOpen() const override { return open_; }
    Expected<void>   write(std::span<const uint8_t> b, int) override;
    Expected<size_t> read(std::span<uint8_t> buf, int timeoutMs) override;
    Expected<void>   flush(Flush) override;
    LinkInfo info() const override {
        return {"loopback", "loopback", LinkKind::Loopback, false, true};
    }
private:
    std::deque<uint8_t> q_;
    std::mutex          m_;
    bool                open_ = false;
};

/**
 * @brief Deterministically scripted link for tests.
 *
 * Steps: push(rx bytes) | drop(n rx) | stall(ms) | close.
 * TX bytes are recorded into `txLog` for assertions.
 */
class ScriptedLink final : public ILink {
public:
    struct Step {
        enum Kind { Push, Drop, Stall, Close } kind;
        std::vector<uint8_t> bytes;
        int n = 0;
    };
    explicit ScriptedLink(std::vector<Step> steps = {});

    void push(std::vector<uint8_t> bytes);   // queue rx data for next write
    /// Pre-queued responses fire only after the next write() (device-like).
    /// For free-running feeds call pushLive() instead.
    void pushLive(std::vector<uint8_t> bytes);
    void stall(int ms);                      // make reads time out for ms
    const std::vector<std::vector<uint8_t>>& txLog() const { return txLog_; }

    Expected<void>   open() override { open_ = true; return {}; }
    void             close() override;
    bool             isOpen() const override { return open_; }
    Expected<void>   write(std::span<const uint8_t> b, int) override;
    Expected<size_t> read(std::span<uint8_t> buf, int timeoutMs) override;
    Expected<void>   flush(Flush) override;
    LinkInfo info() const override {
        return {"script", "scripted", LinkKind::Script, false, true};
    }
private:
    std::deque<Step>   steps_;
    std::deque<uint8_t> rx_;
    std::vector<std::vector<uint8_t>> txLog_;
    std::mutex          m_;
    bool                open_ = false;
    bool                armed_ = false;
    int64_t             stallUntil_ = 0;   // steady ns
};

/// Parse the mini-DSL: "push 01 02 | stall 5ms | drop 3 | close"
std::vector<ScriptedLink::Step> parseScript(std::string_view text);

#if !defined(ESP_PLATFORM)
/**@brief A real pty pair: termios applies; `master`/`slave` fds. */
class PtyPair {
public:
    ~PtyPair();
    static Expected<std::unique_ptr<PtyPair>> open(const SerialConfig& = {});
    std::unique_ptr<ILink> masterLink();
    std::unique_ptr<ILink> slaveLink();
    std::string slavePath() const { return slavePath_; }
private:
    int masterFd_ = -1, slaveFd_ = -1;
    std::string slavePath_;
};

/**@brief ILink over an already-open fd (pty ends). */
class FdLink final : public ILink {
public:
    explicit FdLink(int fd, std::string name) : fd_(fd), name_(std::move(name)) {}
    Expected<void>   open() override { return {}; }
    void             close() override {}
    bool             isOpen() const override { return fd_ >= 0; }
    Expected<void>   write(std::span<const uint8_t> b, int timeoutMs) override;
    Expected<size_t> read(std::span<uint8_t> buf, int timeoutMs) override;
    Expected<void>   flush(Flush) override;
    LinkInfo info() const override {
        return {"pty:" + name_, name_, LinkKind::Pty, true, true};
    }
    int fd() const override { return fd_; }
    void setTap(std::function<void(bool, std::span<const uint8_t>)> t)
        override { tap_ = std::move(t); }
    /// Internal: fire the tap (called by write()/read()).
    void tap(bool tx, std::span<const uint8_t> b) const {
        if (tap_) tap_(tx, b);
    }
private:
    int fd_;
    std::string name_;
    std::function<void(bool, std::span<const uint8_t>)> tap_;
};
#endif

Expected<std::unique_ptr<ILink>> makeLoopbackLink();

} // namespace tether::modbus
