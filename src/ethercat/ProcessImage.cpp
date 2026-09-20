/**
 * @file ProcessImage.cpp
 * @brief Process image configuration + input publish implementation.
 */

#include "tether/ethercat/ProcessImage.hpp"
#include "tether/ethercat/CyclicChannel.hpp"

#include <cstring>

namespace EtherCAT {

bool ProcessImage::configure(const Config& cfg) {
    rx_bytes_ = cfg.rx_bytes;
    tx_bytes_ = cfg.tx_bytes;
    size_     = cfg.rx_bytes + cfg.tx_bytes;
    mode_     = cfg.mode;
    entry_count_ = cfg.entry_count < kMaxEntries ? cfg.entry_count
                                                 : kMaxEntries;
    if (cfg.entry_offsets) {
        std::memcpy(entry_off_, cfg.entry_offsets,
                    entry_count_ * sizeof(int32_t));
    } else {
        for (size_t i = 0; i < entry_count_; ++i) entry_off_[i] = -1;
    }

    clearInputHold();
    img_.reset();
    for (auto& b : dbl_) b.reset();
    for (auto& b : tri_) b.reset();
    in_bank_.reset();
    in_ptr_.store(nullptr, std::memory_order_release);
    in_seq_.store(0, std::memory_order_release);
    rotating_frame_ = nullptr;
    rotating_payload_.store(nullptr, std::memory_order_release);

    if (size_ == 0 || mode_ == ImageMode::Buffered) {
        mode_ = ImageMode::Buffered;
        epoch_.fetch_add(1, std::memory_order_acq_rel);
        return size_ > 0;
    }

    switch (mode_) {
        case ImageMode::Direct:
            img_ = std::make_unique<uint8_t[]>(size_);
            std::memset(img_.get(), 0, size_);
            break;
        case ImageMode::DoubleBuffered:
            for (auto& b : dbl_) {
                b = std::make_unique<uint8_t[]>(size_);
                std::memset(b.get(), 0, size_);
            }
            dblSeq_.store(0, std::memory_order_release);  // clean, idle
            break;
        case ImageMode::TripleBuffered:
            for (auto& b : tri_) {
                b = std::make_unique<uint8_t[]>(size_);
                std::memset(b.get(), 0, size_);
            }
            triState_.store(0b100100, std::memory_order_release); // w0 r1 s2
            break;
        case ImageMode::Rotating:
            break;   // payload lives in the channel frame — nothing owned
        default: break;
    }
    // Owned input bank for the no-channel fallback (publishInputCopy) —
    // allocated here so the RT publish path never allocates.
    in_bank_ = std::make_unique<uint8_t[]>(size_);
    epoch_.fetch_add(1, std::memory_order_acq_rel);
    return true;
}

void ProcessImage::clearInputHold() {
    if (in_channel_ && in_cookie_ >= 0) {
        in_channel_->rxRelease(static_cast<uint32_t>(in_cookie_));
    }
    in_channel_ = nullptr;
    in_cookie_  = -1;
}

void ProcessImage::publishInputView(const uint8_t* payload, uint32_t len,
                                    uint32_t cookie, ICyclicChannel* ch) {
    // Release the previous held view, then hold the new one — the pointer
    // handed to readers stays valid until the next publish.
    clearInputHold();
    if (ch) {
        ch->rxHold(cookie);
        in_channel_ = ch;
        in_cookie_  = static_cast<int64_t>(cookie);
    }
    in_ptr_.store(payload, std::memory_order_release);
    in_len_ = len;
    in_seq_.fetch_add(1, std::memory_order_acq_rel);
}

void ProcessImage::publishInputCopy(const uint8_t* payload, uint32_t len) {
    if (!in_bank_) return;   // not configured
    const uint32_t n = len < size_ ? len : size_;
    std::memcpy(in_bank_.get(), payload, n);
    clearInputHold();
    in_ptr_.store(in_bank_.get(), std::memory_order_release);
    in_len_ = n;
    in_seq_.fetch_add(1, std::memory_order_acq_rel);
}

} // namespace EtherCAT
