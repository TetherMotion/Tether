/**
 * @file SlipTransport.cpp
 * @brief SLIP-framed client transport adapter implementation.
 * @copyright Copyright (C) 2025-2026 Tether Authors
 */
#include "tether/io/SlipTransport.hpp"

#include "SLIPStream/Buffer.hpp"

#include <chrono>

namespace tether { namespace io {

SlipFramedTransport::SlipFramedTransport(std::unique_ptr<ITransport> inner,
                                         size_t maxEncodedSize,
                                         size_t maxDecodedSize)
    : inner_(std::move(inner))
    , maxEncoded_(maxEncodedSize)
    , maxDecoded_(maxDecodedSize) {
    encoded_.reserve(4096);
}

bool SlipFramedTransport::send(const uint8_t* data, size_t len) {
    if (!inner_ || !inner_->isConnected()) return false;

    const size_t encLen = SLIPStream::encoded_length(data, len);
    if (encLen == SLIPStream::ENCODE_ERROR) return false;

    std::vector<uint8_t> encoded(encLen);
    const size_t written = SLIPStream::encode_packet(
        data, len, encoded.data(), encoded.size());
    if (written == SLIPStream::ENCODE_ERROR) return false;

    return inner_->send(encoded.data(), written);
}

size_t SlipFramedTransport::receive(uint8_t* buf, size_t maxLen,
                                    uint32_t timeoutMs) {
    if (!inner_) return 0;
    return inner_->receive(buf, maxLen, timeoutMs);
}

bool SlipFramedTransport::feedByte(uint8_t byte, std::vector<uint8_t>& out) {
    if (discardUntilEnd_) {
        if (byte == SLIPStream::END) {
            discardUntilEnd_ = false;
            encoded_.clear();
        }
        return false;
    }

    if (encoded_.size() >= maxEncoded_) {
        // Oversized frame: drop everything through the next END.
        ++droppedFrames_;
        encoded_.clear();
        discardUntilEnd_ = (byte != SLIPStream::END);
        return false;
    }
    encoded_.push_back(byte);

    if (byte != SLIPStream::END) return false;

    const size_t decLen = SLIPStream::decoded_length(encoded_.data(),
                                                   encoded_.size());
    if (decLen == SLIPStream::DECODE_ERROR || decLen == 0 ||
        decLen > maxDecoded_) {
        if (decLen != 0 && decLen != SLIPStream::DECODE_ERROR) {
            ++droppedFrames_;
        }
        encoded_.clear();
        return false;
    }
    decoded_.resize(decLen);
    const size_t wrote = SLIPStream::decode_packet(
        encoded_.data(), encoded_.size(), decoded_.data(), decoded_.size());
    encoded_.clear();
    if (wrote == SLIPStream::DECODE_ERROR || wrote == 0) {
        ++droppedFrames_;
        return false;
    }
    ++decodedFrames_;
    out.assign(decoded_.begin(), decoded_.begin() +
               static_cast<ptrdiff_t>(wrote));
    return true;
}

bool SlipFramedTransport::receiveMessage(std::vector<uint8_t>& out,
                                         uint32_t timeoutMs) {
    if (!inner_ || !inner_->isConnected()) return false;

    const auto deadline = std::chrono::steady_clock::now() +
                          std::chrono::milliseconds(timeoutMs);
    uint8_t buf[1024];
    while (true) {
        // Drain bytes left over from a previous receive() chunk first — a
        // transport read may carry more than one SLIP packet.
        while (pendingOff_ < pending_.size()) {
            if (feedByte(pending_[pendingOff_++], out)) {
                if (pendingOff_ == pending_.size()) {
                    pending_.clear();
                    pendingOff_ = 0;
                }
                return true;
            }
        }
        pending_.clear();
        pendingOff_ = 0;

        uint32_t slice = timeoutMs;
        if (timeoutMs != 0) {
            const auto now = std::chrono::steady_clock::now();
            if (now >= deadline) return false;
            slice = static_cast<uint32_t>(
                std::chrono::duration_cast<std::chrono::milliseconds>(
                    deadline - now).count());
            if (slice == 0) slice = 1;
        }
        const size_t r = inner_->receive(buf, sizeof(buf), slice);
        if (r == 0) {
            if (!inner_->isConnected()) return false;
            if (timeoutMs == 0) return false;   // poll mode: nothing pending
            continue;                          // select timed out — re-check deadline
        }
        pending_.assign(buf, buf + r);
        pendingOff_ = 0;
    }
}

void SlipFramedTransport::close() {
    if (inner_) inner_->close();
}

bool SlipFramedTransport::isConnected() const {
    return inner_ && inner_->isConnected();
}

}} // namespace tether::io
