/**
 * @file SerialPortLink.hpp
 * @brief POSIX termios serial link (RS-485 aware).
 *
 * Not compiled on ESP_PLATFORM — see platform/EspUartPort for the
 * embedded equivalent.
 */
#pragma once

#include "tether/modbus/link/ILink.hpp"
#include "tether/modbus/link/SerialConfig.hpp"

#if !defined(ESP_PLATFORM)

namespace tether::modbus {

/**
 * @brief Stateful PARMRK un-marker (3-state machine):
 *   Normal   — byte != FF → emit; FF → AfterFF
 *   AfterFF  — FF → emit FF (literal); 00 → AfterFF00; else → emit, +err
 *   AfterFF00— byte → emit (errored byte, kept), +err → Normal
 * Marks may straddle read() boundaries — state persists across feeds.
 */
struct ParityUnmarker {
    /// Process `in` (marked stream) into `out` (clean). Returns error count.
    size_t feed(std::span<const uint8_t> in, std::vector<uint8_t>& out) {
        size_t errs = 0;
        for (uint8_t b : in) {
            switch (state_) {
            case Normal:
                if (b == 0xFF) state_ = AfterFF;
                else out.push_back(b);
                break;
            case AfterFF:
                if (b == 0xFF) { out.push_back(0xFF); state_ = Normal; }
                else if (b == 0x00) state_ = AfterFF00;
                else { out.push_back(b); ++errs; state_ = Normal; }
                break;
            case AfterFF00:
                out.push_back(b);              // errored byte (kept)
                ++errs;
                state_ = Normal;
                break;
            }
        }
        return errs;
    }
    /// One-shot helper for tests: marks must be complete.
    static std::vector<uint8_t> unmark(std::span<const uint8_t> in,
                                       size_t* errs = nullptr) {
        ParityUnmarker u;
        std::vector<uint8_t> out;
        const size_t e = u.feed(in, out);
        if (errs) *errs = e;
        return out;
    }
private:
    enum State : uint8_t { Normal, AfterFF, AfterFF00 } state_ = Normal;
};

class SerialPortLink final : public ILink {
public:
    SerialPortLink(SerialConfig cfg, OpenOpts opts = {});
    ~SerialPortLink() override;

    Expected<void>   open() override;
    void             close() override;
    bool             isOpen() const override;
    Expected<void>   write(std::span<const uint8_t>, int timeoutMs) override;
    Expected<size_t> read(std::span<uint8_t>, int timeoutMs) override;
    Expected<void>   flush(Flush) override;
    LinkInfo         info() const override;
    int              fd() const override;
    Expected<size_t> drain() override;
    void setTap(std::function<void(bool, std::span<const uint8_t>)> t) override {
        tap_ = std::move(t);
    }

    /// Apply `cfg` (+ parityCheck mode from `opts`) to an already-open fd.
    static Expected<void> applyTermios(
        int fd, const SerialConfig& cfg, const OpenOpts& opts = {});

    /// Manual-RTS RS-485 helpers (rtsBefore mode).
    Expected<void> setRts(bool on);

    /// Bytes the UART flagged with parity/framing errors (Mark mode).
    uint64_t parityMarkedErrors() const { return parityMarkedErrors_; }

    /// Linux serial_icounter_struct counters (frame/parity/overrun/brk).
    struct SerialErrorCounters { int frame = 0, parity = 0, overrun = 0,
                                     brk = 0; };
    Expected<SerialErrorCounters> serialErrors() const;

    /// False when the driver silently dropped the requested parity bits
    /// (ptys, virtual ports) — no marks/parity checking will ever occur.
    bool parityApplied() const { return parityApplied_; }

private:
    Expected<void> openOnce();

    SerialConfig   cfg_;
    OpenOpts       opts_;
    int            fd_ = -1;
    ParityUnmarker unmarker_;
    uint64_t       parityMarkedErrors_ = 0;
    bool           unmarkActive_ = false;
    bool           parityApplied_ = false;
    std::function<void(bool, std::span<const uint8_t>)> tap_;
};

} // namespace tether::modbus

#endif // !ESP_PLATFORM
