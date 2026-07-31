/**
 * @file CanTransport.cpp
 * @brief CAN byte-stream transport implementation.
 */

#include "tether/klipper/transport/CanTransport.hpp"

#include <chrono>
#include <thread>

namespace tether::klipper::transport {

CanTransport::~CanTransport() {
    close();
}

bool CanTransport::open() {
    if (open_) return true;
    if (!config_.can) return false;
    // The HAL CAN interface is expected to be already opened by the caller.
    open_ = true;
    return true;
}

bool CanTransport::isOpen() const { return open_; }

void CanTransport::close() {
    open_ = false;
}

size_t CanTransport::write(std::span<const uint8_t> data) {
    if (!open_ || !config_.can) return 0;
    size_t total = 0;
    while (total < data.size()) {
        uint8_t payload[8];
        size_t chunk = std::min<size_t>(8, data.size() - total);
        for (size_t i = 0; i < chunk; ++i) payload[i] = data[total + i];
        tether::hal::CanFrame frame;
        frame.id = config_.txCanId;
        frame.dlc = static_cast<uint8_t>(chunk);
        std::memcpy(frame.data, payload, chunk);
        if (!config_.can->send(frame)) break;
        total += chunk;
    }
    return total;
}

size_t CanTransport::available() const {
    if (!open_ || !config_.can) return 0;
    std::lock_guard<std::mutex> lk(const_cast<std::mutex&>(rxMtx_));
    return rxBuf_.size();
}

size_t CanTransport::read(uint8_t* out, size_t maxLen, bool canBlock) {
    if (!open_ || !config_.can) return 0;
    pumpRx();
    if (canBlock) {
        while (true) {
            {
                std::lock_guard<std::mutex> lk(rxMtx_);
                if (!rxBuf_.empty()) break;
            }
            std::this_thread::sleep_for(std::chrono::microseconds(config_.pollIntervalUs));
            pumpRx();
        }
    }
    std::lock_guard<std::mutex> lk(rxMtx_);
    size_t n = std::min(maxLen, rxBuf_.size());
    for (size_t i = 0; i < n; ++i) out[i] = rxBuf_[i];
    rxBuf_.erase(rxBuf_.begin(), rxBuf_.begin() + n);
    return n;
}

void CanTransport::pumpRx() {
    tether::hal::CanFrame frame;
    while (config_.can->recv(frame, false)) {
        if (config_.rxCanId != 0 && frame.id != config_.rxCanId) continue;
        std::lock_guard<std::mutex> lk(rxMtx_);
        for (uint8_t i = 0; i < frame.dlc; ++i) rxBuf_.push_back(frame.data[i]);
    }
}

} // namespace tether::klipper::transport
