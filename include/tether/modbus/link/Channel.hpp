/**
 * @file Channel.hpp
 * @brief Channels = link + framer + timing: the ADU-level interface the
 *        executor and server use (plan §7.6).
 */
#pragma once

#include <atomic>
#include <chrono>
#include <deque>
#include <memory>
#include <optional>
#include <thread>

#include "tether/modbus/link/Framer.hpp"
#include "tether/modbus/link/ILink.hpp"
#include "tether/modbus/link/SerialConfig.hpp"

namespace tether::modbus {

struct ChannelOpts {
    int interFrameDelayUs = 0;      ///< extra master turnaround delay
    bool expectEcho = false;        ///< RS-485 dongle that echoes TX
    int turnaroundDelayMs = 0;      ///< slave: delay before replying
};

/// What a channel reports per exchange.
enum class RxOutcome { Ok, Timeout, BadFrame, WrongUnit, Exception, Closed };

class IChannel {
public:
    virtual ~IChannel() = default;
    virtual Expected<void> open() = 0;
    virtual void           close() = 0;
    virtual bool           isOpen() const = 0;
    /// Send request ADU; receive the matching response ADU.
    /// For RTU the unit must equal request's; for TCP, txn must match.
    virtual Expected<Adu>  transact(uint8_t unit,
                                    std::span<const uint8_t> reqPdu,
                                    int timeoutMs) = 0;
    /// Send request, do not wait for response (broadcast / fire-forget).
    virtual Expected<void> sendOnly(uint8_t unit,
                                    std::span<const uint8_t> reqPdu) = 0;
    virtual void flush() = 0;
    virtual ILink& link() = 0;
    virtual IFramer& framer() = 0;
};

/**@brief RTU channel: t3.5 frame-end detection + serialization is the
 * caller's job (BusExecutor serializes). */
class RtuChannel final : public IChannel {
public:
    RtuChannel(std::unique_ptr<ILink> link, RtuTiming timing,
               ChannelOpts opts = {});

    Expected<void> open() override;
    void close() override;
    bool isOpen() const override { return link_->isOpen(); }
    Expected<Adu> transact(uint8_t unit, std::span<const uint8_t> reqPdu,
                           int timeoutMs) override;
    Expected<void> sendOnly(uint8_t unit,
                            std::span<const uint8_t> reqPdu) override;
    void flush() override;
    ILink& link() override { return *link_; }
    IFramer& framer() override { return framer_; }
    const RtuTiming& timing() const { return timing_; }

    /// Receive one frame (server path): blocks up to timeoutMs for the
    /// start of a frame, then t1.5-aware accumulate until t3.5 gap.
    Expected<Adu> receiveFrame(int timeoutMs);

private:
    /// Read until t3.5 gap → returns the parsed frame (or empty).
    Expected<std::vector<Adu>> collect(int firstByteTimeoutMs,
                                       int totalTimeoutMs);

    std::unique_ptr<ILink> link_;
    RtuFramer framer_;
    RtuTiming timing_;
    ChannelOpts opts_;
    int64_t lastByteNs_ = 0;
};

/**@brief MBAP/TCP channel: request/response on one connection;
 * concurrency via txn ids handled by TransactionManager above. */
class MbapChannel final : public IChannel {
public:
    explicit MbapChannel(std::unique_ptr<ILink> link,
                         ChannelOpts opts = {});

    Expected<void> open() override;
    void close() override;
    bool isOpen() const override { return link_->isOpen(); }
    Expected<Adu> transact(uint8_t unit, std::span<const uint8_t> reqPdu,
                           int timeoutMs) override;
    Expected<void> sendOnly(uint8_t unit,
                            std::span<const uint8_t> reqPdu) override;
    void flush() override;
    ILink& link() override { return *link_; }
    IFramer& framer() override { return framer_; }

    /// Receive the next complete ADU (server path / pipelined path).
    Expected<Adu> receiveAny(int timeoutMs);
    /// Raw send of a fully-formed ADU (server replies).
    Expected<void> sendAdu(const Adu& a, std::span<const uint8_t> pdu);

    uint16_t nextTxn() { return txn_++; }

private:
    std::unique_ptr<ILink> link_;
    MbapFramer framer_;
    ChannelOpts opts_;
    std::atomic<uint16_t> txn_{1};
    std::deque<Adu> pending_;
};

/// ASCII (optional, TETHER_MODBUS_ASCII).
#if defined(TETHER_MODBUS_ASCII)
class AsciiFramer final : public IFramer {
public:
    Frame encode(uint8_t unit, std::span<const uint8_t> pdu,
                 uint16_t) override;
    void feed(std::span<const uint8_t> bytes,
              std::vector<Adu>& out) override;
    void reset() override { buf_.clear(); }
    size_t overhead() const override { return 7; }  // ':' + 2*LRC + CRLF
private:
    std::vector<uint8_t> buf_;   // raw ascii chars, in-frame
};
#endif

} // namespace tether::modbus
