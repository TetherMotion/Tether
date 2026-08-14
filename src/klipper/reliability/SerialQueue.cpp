/**
 * @file SerialQueue.cpp
 * @brief SerialQueue implementation.
 */

#include "tether/klipper/reliability/SerialQueue.hpp"
#include "tether/klipper/KlipperLog.hpp"

#include <algorithm>
#include <format>

namespace tether::klipper::reliability {

std::optional<uint8_t> SerialQueue::send(std::span<const uint8_t> content) {
    std::lock_guard<std::mutex> lk(mutex_);
    if (pending_.size() >= maxPending_) {
        KLIPPER_LOG_WARN("SerialQueue::send() rejected - window full");
        return std::nullopt;
    }
    uint8_t seq = sendSeq_.value();
    auto wire = protocol::buildBlockVec(seq, content);
    if (wire.empty()) {
        KLIPPER_LOG_ERROR("SerialQueue::send() failed to build message block");
        return std::nullopt;
    }
    size_t written = transport_.write(wire);
    if (written != wire.size()) {
        KLIPPER_LOG_ERROR(std::format("SerialQueue::send() partial write: {}/{} bytes", written, wire.size()));
        return std::nullopt;
    }
    PendingBlock pb;
    pb.sequence = seq;
    pb.wireBytes = std::move(wire);
    pb.sendTime = Clock::now();
    pending_.push_back(std::move(pb));
    sendSeq_.advance();
    return seq;
}

void SerialQueue::processAck(const protocol::MessageBlock& block) {
    std::lock_guard<std::mutex> lk(mutex_);
    uint8_t ackedSeq = block.sequence;

    // Measure RTT for the acked block
    if (!pending_.empty() && ackCb_) {
        auto now = Clock::now();
        for (const auto& pb : pending_) {
            if (pb.sequence == ackedSeq) {
                double rtt = std::chrono::duration<double>(now - pb.sendTime).count();
                rto_.update(rtt);
                ackCb_(rtt);
                break;
            }
        }
    }

    // Remove all pending blocks with sequence <= ackedSeq (mod 16).
    // Use index-based approach to avoid modifying while iterating.
    while (!pending_.empty()) {
        uint8_t frontSeq = pending_.front().sequence;
        uint8_t distance = (ackedSeq - frontSeq) & 0x0F;
        if (frontSeq != ackedSeq && distance > 0 && distance < 0x08) {
            // frontSeq is ahead of ackedSeq — shouldn't happen for in-order acks.
            break;
        }
        pending_.pop_front();
        if (frontSeq == ackedSeq) break;
    }
    ackSeq_.set(ackedSeq);
}

void SerialQueue::checkTimeouts(TimePoint now) {
    std::lock_guard<std::mutex> lk(mutex_);
    double rto = rto_.rto();
    auto rtoDur = std::chrono::duration_cast<Clock::duration>(
        std::chrono::duration<double>(rto));

    // Retransmit timed-out blocks, drop blocks that exceeded max retransmits
    for (auto it = pending_.begin(); it != pending_.end(); ) {
        if (now - it->sendTime >= rtoDur) {
            if (it->retransmitCount >= kMaxRetransmits) {
                KLIPPER_LOG_WARN(std::format(
                    "SerialQueue: dropping block seq={} after {} retransmits",
                    it->sequence, it->retransmitCount));
                it = pending_.erase(it);
                continue;
            }
            transport_.write(it->wireBytes);
            it->sendTime = now;
            it->retransmitCount++;
        }
        ++it;
    }
}

void SerialQueue::reset() {
    std::lock_guard<std::mutex> lk(mutex_);
    pending_.clear();
    sendSeq_.set(0);
    ackSeq_.set(0);
    rto_.reset();
}

} // namespace tether::klipper::reliability
