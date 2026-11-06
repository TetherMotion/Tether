/**
 * @file Pdu.hpp
 * @brief Protocol Data Unit container + header view (plan §6.3/AB.4).
 */
#pragma once

#include <array>
#include <cstdint>
#include <optional>
#include <span>
#include <vector>

#include "tether/modbus/protocol/Constants.hpp"
#include "tether/modbus/protocol/ExceptionCode.hpp"
#include "tether/modbus/protocol/FunctionCode.hpp"

namespace tether::modbus {

/**
 * @brief Owning PDU: `[fc][body…]` capped at kMaxPduSize.
 *
 * `data[0]` is the function code (possibly fc|0x80 for exceptions).
 */
class Pdu {
public:
    Pdu() = default;
    explicit Pdu(FunctionCode fc) { push(static_cast<uint8_t>(fc)); }
    explicit Pdu(std::span<const uint8_t> bytes) {
        size_ = bytes.size() <= kMaxPduSize ? bytes.size() : kMaxPduSize;
        std::copy_n(bytes.begin(), size_, data_.begin());
    }
    Pdu(FunctionCode fc, std::span<const uint8_t> body) : Pdu(fc) {
        append(body);
    }

    /// Make an exception PDU `[fc|0x80][code]`.
    static Pdu exception(uint8_t fc, uint8_t code) {
        Pdu p;
        p.push(static_cast<uint8_t>(fc | 0x80));
        p.push(code);
        return p;
    }

    bool push(uint8_t b) {
        if (size_ >= kMaxPduSize) return false;
        data_[size_++] = b;
        return true;
    }
    bool append(std::span<const uint8_t> b) {
        if (size_ + b.size() > kMaxPduSize) return false;
        std::copy(b.begin(), b.end(), data_.begin() + size_);
        size_ += static_cast<uint16_t>(b.size());
        return true;
    }
    bool appendU16(uint16_t v) { return push(uint8_t(v >> 8)) && push(uint8_t(v)); }
    bool appendU32(uint32_t v) {
        return appendU16(uint16_t(v >> 16)) && appendU16(uint16_t(v));
    }

    uint8_t fc() const { return size_ ? data_[0] : 0; }
    std::span<const uint8_t> bytes() const { return {data_.data(), size_}; }
    std::span<uint8_t> bytes() { return {data_.data(), size_}; }
    std::span<const uint8_t> body() const {
        return size_ > 1 ? std::span{data_.data() + 1, size_t(size_ - 1)}
                         : std::span<const uint8_t>{};
    }
    size_t size() const { return size_; }
    bool   empty() const { return size_ == 0; }
    void   clear() { size_ = 0; }

    bool operator==(const Pdu&) const = default;

private:
    std::array<uint8_t, kMaxPduSize> data_{};
    uint16_t size_ = 0;
};

/**@brief Parsed PDU header view (non-owning). */
struct PduHead {
    FunctionCode  fc{};
    std::span<const uint8_t> body;
    bool          valid = false;
    bool          exception = false;
    ExceptionCode exc = ExceptionCode::IllegalFunction;

    static PduHead invalid() { return {}; }
};

/**
 * @brief Parse the head of a raw PDU.
 * Exception PDUs must be exactly `[fc|0x80][code]` (2 bytes).
 */
inline PduHead parsePduHead(std::span<const uint8_t> pdu) {
    PduHead h;
    if (pdu.empty()) return h;
    const uint8_t fcByte = pdu[0];
    if (fcByte & 0x80) {
        if (pdu.size() != 2) return PduHead::invalid();
        h.fc = static_cast<FunctionCode>(fcByte & 0x7F);
        h.exception = true;
        h.exc = static_cast<ExceptionCode>(pdu[1]);
        h.body = {};
        h.valid = true;
        return h;
    }
    h.fc = static_cast<FunctionCode>(fcByte);
    h.exception = false;
    h.body = pdu.subspan(1);
    h.valid = true;
    return h;
}

} // namespace tether::modbus
