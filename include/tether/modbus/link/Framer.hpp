/**
 * @file Framer.hpp
 * @brief ADU encode/decode + streaming frame extraction (plan §7.5).
 */
#pragma once

#include <cstdint>
#include <deque>
#include <optional>
#include <span>
#include <vector>

#include "tether/modbus/ModbusError.hpp"
#include "tether/modbus/link/RtuTiming.hpp"
#include "tether/modbus/protocol/Constants.hpp"
#include "tether/modbus/protocol/Crc.hpp"

namespace tether::modbus {

/// A decoded application data unit.
struct Adu {
    uint8_t              unit = 0;
    uint16_t             txn = 0;          ///< MBAP transaction id (TCP)
    std::vector<uint8_t> pdu;              ///< function code + body
};

/// Encode result: full wire frame.
using Frame = std::vector<uint8_t>;

class IFramer {
public:
    virtual ~IFramer() = default;
    /// Encode ADU → wire bytes.
    virtual Frame encode(uint8_t unit, std::span<const uint8_t> pdu,
                         uint16_t txn = 0) = 0;
    /// Feed received bytes; extract any complete ADUs into `out`.
    virtual void feed(std::span<const uint8_t> bytes,
                      std::vector<Adu>& out) = 0;
    /// Called when an inter-frame silence elapses (RTU) or link
    /// activity boundary (TCP: no-op).
    virtual void onGap() {}
    /// Drop partial state (after error/flush).
    virtual void reset() = 0;
    /// Max wire bytes for a request of `pduLen` bytes.
    virtual size_t overhead() const = 0;
    virtual bool isStream() const { return true; }
};

// ---- RTU -----------------------------------------------------------------------
/**@brief Streaming RTU framer; frame boundaries come from RtuChannel
 * (t3.5 gap detection) — `onGap()` finalizes the pending frame. */
class RtuFramer final : public IFramer {
public:
    explicit RtuFramer(uint8_t localUnit = 0 /* for echo strip */)
        : localUnit_(localUnit) {}

    Frame encode(uint8_t unit, std::span<const uint8_t> pdu,
                 uint16_t) override {
        Frame f(pdu.size() + 3);
        f[0] = unit;
        std::copy(pdu.begin(), pdu.end(), f.begin() + 1);
        const uint16_t crc = Crc16::of(std::span(f.data(), f.size() - 2));
        f[f.size() - 2] = uint8_t(crc & 0xFF);
        f[f.size() - 1] = uint8_t(crc >> 8);
        return f;
    }

    void feed(std::span<const uint8_t> bytes,
              std::vector<Adu>& out) override {
        for (uint8_t b : bytes) {
            if (buf_.size() >= kMaxRtuAduSize) buf_.clear();  // oversize → reset
            buf_.push_back(b);
            // Early completion: valid CRC ends a frame even without an
            // observed gap — splits back-to-back concatenated frames.
            if (buf_.size() >= 4 && Crc16::check(buf_))
                finishFrame(out);
        }
    }

    /// t3.5 elapsed → current buffer becomes a complete frame.
    void onGap(std::vector<Adu>& out) {
        finishFrame(out);
    }
    void onGap() override {}

    void reset() override { buf_.clear(); }
    size_t overhead() const override { return 3; }
    size_t pending() const { return buf_.size(); }

    /// Parse the accumulated buffer into an Adu (called by channel on gap).
    void finishFrame(std::vector<Adu>& out) {
        if (buf_.size() < 4) { buf_.clear(); return; }
        const size_t n = buf_.size();
        const uint16_t got = uint16_t(buf_[n - 2]) |
                             uint16_t(buf_[n - 1] << 8);
        const uint16_t want = Crc16::of(std::span(buf_.data(), n - 2));
        if (got == want) {
            Adu a;
            a.unit = buf_[0];
            a.pdu.assign(buf_.begin() + 1, buf_.end() - 2);
            out.push_back(std::move(a));
        }
        buf_.clear();
    }
    const std::vector<uint8_t>& partial() const { return buf_; }

private:
    std::vector<uint8_t> buf_;
    uint8_t localUnit_;
};

// ---- MBAP/TCP --------------------------------------------------------------------
/**@brief MBAP framer; handles pipelined + fragmented TCP delivery. */
class MbapFramer final : public IFramer {
public:
    Frame encode(uint8_t unit, std::span<const uint8_t> pdu,
                 uint16_t txn) override {
        Frame f(7 + pdu.size());
        f[0] = uint8_t(txn >> 8);  f[1] = uint8_t(txn);
        f[2] = 0;                  f[3] = 0;              // protocol id
        const uint16_t len = uint16_t(pdu.size() + 1);    // unit + pdu
        f[4] = uint8_t(len >> 8);  f[5] = uint8_t(len);
        f[6] = unit;
        std::copy(pdu.begin(), pdu.end(), f.begin() + 7);
        return f;
    }

    void feed(std::span<const uint8_t> bytes,
              std::vector<Adu>& out) override {
        for (uint8_t b : bytes) {
            buf_.push_back(b);
            if (buf_.size() > kMaxMbapAduSize) buf_.clear();
        }
        for (;;) {
            if (buf_.size() < 7) return;
            const uint16_t proto = uint16_t(buf_[2] << 8) | buf_[3];
            const uint16_t len   = uint16_t(buf_[4] << 8) | buf_[5];
            if (proto != 0 || len < 2 || len > kMaxPduSize + 1) {
                buf_.pop_front();   // resync: drop one byte
                continue;
            }
            const size_t total = 6 + len;
            if (buf_.size() < total) return;
            Adu a;
            a.txn  = uint16_t(buf_[0] << 8) | buf_[1];
            a.unit = buf_[6];
            a.pdu.assign(buf_.begin() + 7, buf_.begin() + total);
            out.push_back(std::move(a));
            buf_.erase(buf_.begin(), buf_.begin() + total);
        }
    }
    void reset() override { buf_.clear(); }
    size_t overhead() const override { return 7; }

private:
    std::deque<uint8_t> buf_;
};

} // namespace tether::modbus
